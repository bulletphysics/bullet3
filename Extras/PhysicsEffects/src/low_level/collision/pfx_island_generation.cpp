/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/rigidbody/pfx_rigid_state.h"
#include "../../../include/physics_effects/low_level/collision/pfx_island_generation.h"

namespace sce {
namespace PhysicsEffects {

struct PfxIslandNode {
	PfxUInt32 rootId;
	PfxUInt32 rank;
	PfxUInt32 islandId;
	PfxUInt32 isRoot;
};

struct PfxIslandUnit
{
	PfxUInt32 id;
	PfxIslandUnit *next;
};

struct PfxIsland
{
	PfxUInt32 numNodes;
	PfxUInt32 numIslands;
	PfxIslandNode *nodes;
	PfxIslandUnit *islandsUnits;
	PfxIslandUnit **islandsHeads;
};

PfxUInt32 pfxIslandNodeFind(PfxUInt32 i,PfxIsland *island)
{
	if( i != island->nodes[i].rootId ) {
		island->nodes[i].rootId = pfxIslandNodeFind(island->nodes[i].rootId,island);
	}
	return island->nodes[i].rootId;
}

void pfxIslandNodeLink(PfxUInt32 iA,PfxUInt32 iB,PfxIsland *island)
{
	if(island->nodes[iA].rank > island->nodes[iB].rank) {
		island->nodes[iB].rootId = iA;
	}
	else if(island->nodes[iA].rank == island->nodes[iB].rank) {
		island->nodes[iA].rootId = iB;
		island->nodes[iB].rank++;
	}
	else {
		island->nodes[iA].rootId = iB;
	}
}

void pfxIslandNodeUnion(PfxUInt32 iA,PfxUInt32 iB,PfxIsland *island)
{
	SCE_PFX_ALWAYS_ASSERT(iA<island->numNodes);
	SCE_PFX_ALWAYS_ASSERT(iB<island->numNodes);
	
	pfxIslandNodeLink(pfxIslandNodeFind(iA,island),pfxIslandNodeFind(iB,island),island);
}

PfxUInt32 pfxGetIslandBytesOfGenerateIsland(PfxUInt32 numObjects)
{
	return 16 + 
		SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxIsland)) + 
		SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxIslandNode)*numObjects) + 
		SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxIslandUnit*)*numObjects) +
		SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxIslandUnit)*numObjects);
}

SCE_PFX_FORCE_INLINE int pfxCheckParamOfGenerateIsland(const PfxGenerateIslandParam &param)
{
	if(!param.islandBuff || !param.pairs) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.pairs)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.islandBuff,param.islandBytes) < pfxGetIslandBytesOfGenerateIsland(param.numObjects) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

PfxInt32 pfxGenerateIsland(PfxGenerateIslandParam &param,PfxGenerateIslandResult &result)
{
	int ret = pfxCheckParamOfGenerateIsland(param);
	if(ret != SCE_PFX_OK) return ret;
	
	PfxConstraintPair *pairs = param.pairs;
	PfxUInt32 numPairs = param.numPairs;
	PfxUInt32 numUnits = param.numObjects;

	memset(param.islandBuff,0,param.islandBytes);

	PfxHeapManager pool((unsigned char*)param.islandBuff,param.islandBytes);

	PfxIsland *island = (PfxIsland*)pool.allocate(sizeof(PfxIsland));
	island->numIslands = 0;
	island->numNodes = numUnits;
	island->nodes = (PfxIslandNode*)pool.allocate(sizeof(PfxIslandNode)*numUnits);
	island->islandsHeads = (PfxIslandUnit**)pool.allocate(sizeof(PfxIslandUnit*)*numUnits);
	island->islandsUnits = (PfxIslandUnit*)pool.allocate(sizeof(PfxIslandUnit)*numUnits);
	
	result.island = island;
	
	// 初期化
	for(PfxUInt32 i=0;i<island->numNodes;i++) {
		island->nodes[i].rootId = i;
		island->nodes[i].rank = 0;
	}
	
	return pfxAppendPairs(island,pairs,numPairs);
}

PfxUInt32 pfxGetNumIslands(const PfxIsland *islands)
{
	SCE_PFX_ALWAYS_ASSERT(islands);
	return islands->numIslands;
}

PfxIslandUnit *pfxGetFirstUnitInIsland(const PfxIsland *islands,PfxUInt32 islandId)
{
	SCE_PFX_ALWAYS_ASSERT(islands);
	SCE_PFX_ALWAYS_ASSERT(islandId < islands->numIslands);
	return islands->islandsHeads[islandId];
}

PfxIslandUnit *pfxGetNextUnitInIsland(const PfxIslandUnit *islandUnit)
{
	SCE_PFX_ALWAYS_ASSERT(islandUnit);
	return islandUnit->next;
}

PfxUInt32 pfxGetUnitId(const PfxIslandUnit *islandUnit)
{
	SCE_PFX_ALWAYS_ASSERT(islandUnit);
	return islandUnit->id;
}

PfxUInt32 pfxGetIslandId(const PfxIsland *islands,PfxUInt32 unitId)
{
	SCE_PFX_ALWAYS_ASSERT(islands&&unitId<islands->numNodes);
	return islands->nodes[unitId].islandId;
}

PfxInt32 pfxAppendPairs(PfxIsland *island,PfxConstraintPair *pairs,PfxUInt32 numPairs)
{
	if(numPairs == 0) {
		return SCE_PFX_OK;
	}

	if(!island || !pairs) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(island) || !SCE_PFX_PTR_IS_ALIGNED16(pairs)) return SCE_PFX_ERR_INVALID_ALIGN;

	// 統合
	for(PfxUInt32 i=0;i<numPairs;i++) {
		PfxConstraintPair &pair = pairs[i];
		if(pfxGetActive(pair)) {
			PfxUInt32 iA = pfxGetObjectIdA(pair);
			PfxUInt32 iB = pfxGetObjectIdB(pair);
			
			SCE_PFX_ALWAYS_ASSERT(iA<island->numNodes);
			SCE_PFX_ALWAYS_ASSERT(iB<island->numNodes);
			
			if( (SCE_PFX_MOTION_MASK_DYNAMIC(pfxGetMotionMaskA(pair)&SCE_PFX_MOTION_MASK_TYPE)) && 
				(SCE_PFX_MOTION_MASK_DYNAMIC(pfxGetMotionMaskB(pair)&SCE_PFX_MOTION_MASK_TYPE)) ) {
				pfxIslandNodeUnion(iA,iB,island);
			}
		}
	}

	// アイランド生成のための初期化
	for(PfxUInt32 i=0;i<island->numNodes;i++) {
		island->nodes[i].islandId = 0;
		island->nodes[i].isRoot = 0;
		island->islandsHeads[i] = NULL;
	}

	// 親へ直結
	PfxUInt32 id = 0;
	for(PfxUInt32 i=0;i<island->numNodes;i++) {
		PfxUInt32 rootId = pfxIslandNodeFind(i,island);
		if( island->nodes[rootId].isRoot == 0 ) {
			island->nodes[rootId].islandId = id++;
			island->nodes[rootId].isRoot = 1;
		}
		island->nodes[i].islandId = island->nodes[rootId].islandId;
	}

	// アイランド作成
	PfxUInt32 n = 0;
	for(PfxUInt32 i=0;i<island->numNodes;i++) {
		PfxUInt32 islandId = island->nodes[i].islandId;
		PfxIslandUnit *newUnit = &island->islandsUnits[n++];
		newUnit->id = i;

		if(!island->islandsHeads[islandId]) {
			island->islandsHeads[islandId] = newUnit;
			continue;
		}
		
		PfxIslandUnit *unit=island->islandsHeads[islandId];
		island->islandsHeads[islandId] = newUnit;
		newUnit->next = unit;
	}
	
	island->numIslands = id;
	
	return SCE_PFX_OK;
}

void pfxResetIsland(PfxIsland *island)
{
	SCE_PFX_ALWAYS_ASSERT(island);
	island->numIslands = 0;
	for(PfxUInt32 i=0;i<island->numNodes;i++) {
		island->nodes[i].rootId = i;
		island->nodes[i].rank = 0;
	}
}

} //namespace PhysicsEffects
} //namespace sce
