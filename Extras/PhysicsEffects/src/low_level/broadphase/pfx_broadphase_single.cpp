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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/base_level/sort/pfx_sort.h"
#include "../../../include/physics_effects/base_level/broadphase/pfx_update_broadphase_proxy.h"
#include "../../../include/physics_effects/low_level/broadphase/pfx_broadphase.h"
#include "../../base_level/broadphase/pfx_check_collidable.h"

namespace sce {
namespace PhysicsEffects {

PfxUInt32 pfxGetWorkBytesOfUpdateBroadphaseProxies(PfxUInt32 numRigidBodies)
{
	return 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphaseProxy) * numRigidBodies);
}

PfxUInt32 pfxGetWorkBytesOfUpdateBroadphaseProxies(PfxUInt32 numRigidBodies,PfxUInt32 maxTasks)
{
	(void)maxTasks;
	return 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphaseProxy) * numRigidBodies) * 6;
}

PfxUInt32 pfxGetWorkBytesOfFindPairs(PfxUInt32 maxPairs,PfxUInt32 maxTasks)
{
	return 128 + SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair) * maxPairs) * maxTasks;
}

PfxUInt32 pfxGetPairBytesOfFindPairs(PfxUInt32 maxPairs)
{
	return SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair) * maxPairs);
}

PfxUInt32 pfxGetWorkBytesOfDecomposePairs(PfxUInt32 numPreviousPairs,PfxUInt32 numCurrentPairs,int maxTasks)
{
	return 128 +
		SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair)*numPreviousPairs) + 
		SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair)*numPreviousPairs) * maxTasks + 
		SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxBroadphasePair)*numCurrentPairs) * maxTasks;
}

PfxInt32 pfxCheckParamOfUpdateBroadphaseProxies(const PfxUpdateBroadphaseProxiesParam &param)
{
	if(!param.workBuff || 
		!param.proxiesX || !param.proxiesY || !param.proxiesZ ||
		!param.proxiesXb || !param.proxiesYb || !param.proxiesZb ||
		!param.offsetRigidStates || !param.offsetCollidables ) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.proxiesX) || !SCE_PFX_PTR_IS_ALIGNED16(param.proxiesY) || !SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZ) ||
		!SCE_PFX_PTR_IS_ALIGNED16(param.proxiesXb) || !SCE_PFX_PTR_IS_ALIGNED16(param.proxiesYb) || !SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZb)	) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.workBuff,param.workBytes) < pfxGetWorkBytesOfUpdateBroadphaseProxies(param.numRigidBodies) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

PfxInt32 pfxCheckParamOfFindPairs(const PfxFindPairsParam &param,int maxTasks)
{
	if(!param.workBuff || !param.pairBuff || !param.proxies || param.axis > 2 || param.axis < 0) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.proxies) || !SCE_PFX_PTR_IS_ALIGNED16(param.pairBuff)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.workBuff,param.workBytes) < pfxGetWorkBytesOfFindPairs(param.maxPairs,maxTasks) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.pairBuff,param.pairBytes) < pfxGetPairBytesOfFindPairs(param.maxPairs) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

PfxUInt32 pfxGetPairBytesOfDecomposePairs(PfxUInt32 numPreviousPairs,PfxUInt32 numCurrentPairs)
{
	return sizeof(PfxBroadphasePair)*(numPreviousPairs*2+numCurrentPairs);
}

PfxInt32 pfxCheckParamOfDecomposePairs(const PfxDecomposePairsParam &param,int maxTasks)
{
	if(!param.workBuff || !param.pairBuff || !param.previousPairs || !param.currentPairs) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.previousPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.currentPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.pairBuff) ) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.workBuff,param.workBytes) < pfxGetWorkBytesOfDecomposePairs(param.numPreviousPairs,param.numCurrentPairs,maxTasks)) return SCE_PFX_ERR_OUT_OF_BUFFER;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.pairBuff,param.pairBytes) < pfxGetPairBytesOfDecomposePairs(param.numPreviousPairs,param.numCurrentPairs)) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

PfxInt32 pfxUpdateBroadphaseProxies(PfxUpdateBroadphaseProxiesParam &param,PfxUpdateBroadphaseProxiesResult &result)
{
	PfxInt32 ret = pfxCheckParamOfUpdateBroadphaseProxies(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxUpdateBroadphaseProxies")

	result.numOutOfWorldProxies = 0;

	for(int i=0;i<(int)param.numRigidBodies;i++) {
		PfxInt32 chk = pfxUpdateBroadphaseProxy(
			param.proxiesX[i],
			param.proxiesY[i],
			param.proxiesZ[i],
			param.proxiesXb[i],
			param.proxiesYb[i],
			param.proxiesZb[i],
			param.offsetRigidStates[i],
			param.offsetCollidables[i],
			param.worldCenter,
			param.worldExtent);

		if(chk == SCE_PFX_ERR_OUT_OF_WORLD) {
			result.numOutOfWorldProxies++;

			if(param.outOfWorldBehavior & SCE_PFX_OUT_OF_WORLD_BEHAVIOR_FIX_MOTION) {
				PfxRigidState &state = param.offsetRigidStates[i];
				state.setMotionType(kPfxMotionTypeFixed);
				pfxSetMotionMask(param.proxiesX[i],state.getMotionMask());
				pfxSetMotionMask(param.proxiesY[i],state.getMotionMask());
				pfxSetMotionMask(param.proxiesZ[i],state.getMotionMask());
				pfxSetMotionMask(param.proxiesXb[i],state.getMotionMask());
				pfxSetMotionMask(param.proxiesYb[i],state.getMotionMask());
				pfxSetMotionMask(param.proxiesZb[i],state.getMotionMask());
			}
			
			if(param.outOfWorldBehavior & SCE_PFX_OUT_OF_WORLD_BEHAVIOR_REMOVE_PROXY) {
				pfxSetKey(param.proxiesX[i],SCE_PFX_SENTINEL_KEY);
				pfxSetKey(param.proxiesY[i],SCE_PFX_SENTINEL_KEY);
				pfxSetKey(param.proxiesZ[i],SCE_PFX_SENTINEL_KEY);
				pfxSetKey(param.proxiesXb[i],SCE_PFX_SENTINEL_KEY);
				pfxSetKey(param.proxiesYb[i],SCE_PFX_SENTINEL_KEY);
				pfxSetKey(param.proxiesZb[i],SCE_PFX_SENTINEL_KEY);
			}
		}
	}
	
	PfxHeapManager pool((unsigned char*)param.workBuff,param.workBytes);
	PfxBroadphaseProxy *workProxies = (PfxBroadphaseProxy*)pool.allocate(sizeof(PfxBroadphaseProxy)*param.numRigidBodies,PfxHeapManager::ALIGN128);
	
	pfxSort(param.proxiesX,workProxies,param.numRigidBodies);
	pfxSort(param.proxiesY,workProxies,param.numRigidBodies);
	pfxSort(param.proxiesZ,workProxies,param.numRigidBodies);
	pfxSort(param.proxiesXb,workProxies,param.numRigidBodies);
	pfxSort(param.proxiesYb,workProxies,param.numRigidBodies);
	pfxSort(param.proxiesZb,workProxies,param.numRigidBodies);
	
	pool.deallocate(workProxies);

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxFindPairs(PfxFindPairsParam &param,PfxFindPairsResult &result)
{
	PfxInt32 ret = pfxCheckParamOfFindPairs(param,0);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxFindPairs")

	void *workBuff = param.workBuff;
	PfxUInt32 workBytes = param.workBytes;
	PfxBroadphaseProxy *proxies = param.proxies;
	PfxUInt32 numProxies = param.numProxies;
	PfxUInt32 maxPairs = param.maxPairs;
	int axis = param.axis;

	(void) workBytes;

	PfxBroadphasePair *pairs = (PfxBroadphasePair*)SCE_PFX_PTR_ALIGN16(param.pairBuff);
	PfxUInt32 numPairs = 0;
	
	for(PfxUInt32 i=0;i<numProxies;i++) {
		for(PfxUInt32 j=i+1;j<numProxies;j++) {
			PfxBroadphaseProxy proxyA,proxyB;
			if(pfxGetObjectId(proxies[i]) < pfxGetObjectId(proxies[j])) {
				proxyA = proxies[i];
				proxyB = proxies[j];
			}
			else {
				proxyA = proxies[j];
				proxyB = proxies[i];
			}
			
			if(pfxGetXYZMax(proxyA,axis) < pfxGetXYZMin(proxyB,axis)) {
				break;
			}

			if(	pfxCheckCollidableInBroadphase(proxyA,proxyB) ) {
				if(numPairs >= maxPairs) 
					return SCE_PFX_ERR_OUT_OF_MAX_PAIRS;

				PfxBroadphasePair &pair = pairs[numPairs++];
				pfxSetActive(pair,true);
				pfxSetObjectIdA(pair,pfxGetObjectId(proxyA));
				pfxSetObjectIdB(pair,pfxGetObjectId(proxyB));
				pfxSetMotionMaskA(pair,pfxGetMotionMask(proxyA));
				pfxSetMotionMaskB(pair,pfxGetMotionMask(proxyB));
				
				pfxSetKey(pair,pfxCreateUniqueKey(pfxGetObjectId(proxyA),pfxGetObjectId(proxyB)));
			}
		}
	}
	
	pfxSort(pairs,(PfxBroadphasePair*)workBuff,numPairs);
	
	result.pairs = pairs;
	result.numPairs = numPairs;

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxDecomposePairs(PfxDecomposePairsParam &param,PfxDecomposePairsResult &result)
{
	PfxInt32 ret = pfxCheckParamOfDecomposePairs(param,0);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxDecomposePairs")

	PfxBroadphasePair *previousPairs = param.previousPairs;
	PfxUInt32 numPreviousPairs = param.numPreviousPairs;
	PfxBroadphasePair *currentPairs = param.currentPairs;
	PfxUInt32 numCurrentPairs = param.numCurrentPairs;
	
	PfxBroadphasePair *outNewPairs = (PfxBroadphasePair*)SCE_PFX_PTR_ALIGN16(param.pairBuff);
	PfxBroadphasePair *outKeepPairs = outNewPairs + numCurrentPairs;
	PfxBroadphasePair *outRemovePairs = outKeepPairs + numPreviousPairs;
	
	PfxUInt32 nNew = 0;
	PfxUInt32 nKeep = 0;
	PfxUInt32 nRemove = 0;
	
	PfxUInt32 oldId = 0,newId = 0;
	
	while(oldId<numPreviousPairs&&newId<numCurrentPairs) {
		if(pfxGetKey(currentPairs[newId]) > pfxGetKey(previousPairs[oldId])) {
			// remove
			SCE_PFX_ASSERT(nRemove<=numPreviousPairs);
			outRemovePairs[nRemove] = previousPairs[oldId];
			nRemove++;
			oldId++;
		}
		else if(pfxGetKey(currentPairs[newId]) == pfxGetKey(previousPairs[oldId])) {
			// keep
			SCE_PFX_ASSERT(nKeep<=numPreviousPairs);
			outKeepPairs[nKeep] = currentPairs[newId];
			pfxSetContactId(outKeepPairs[nKeep],pfxGetContactId(previousPairs[oldId]));
			nKeep++;
			oldId++;
			newId++;
		}
		else {
			// new
			SCE_PFX_ASSERT(nNew<=numCurrentPairs);
			outNewPairs[nNew] = currentPairs[newId];
			nNew++;
			newId++;
		}
	};
	
	if(newId<numCurrentPairs) {
		// all new
		for(;newId<numCurrentPairs;newId++,nNew++) {
			SCE_PFX_ASSERT(nNew<=numCurrentPairs);
			outNewPairs[nNew] = currentPairs[newId];
		}
	}
	else if(oldId<numPreviousPairs) {
		// all remove
		for(;oldId<numPreviousPairs;oldId++,nRemove++) {
			SCE_PFX_ASSERT(nRemove<=numPreviousPairs);
			outRemovePairs[nRemove] = previousPairs[oldId];
		}
	}
	
	result.outNewPairs = outNewPairs;
	result.outKeepPairs = outKeepPairs;
	result.outRemovePairs = outRemovePairs;
	result.numOutNewPairs = nNew;
	result.numOutKeepPairs = nKeep;
	result.numOutRemovePairs = nRemove;

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
