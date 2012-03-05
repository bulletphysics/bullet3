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

#include "../../include/physics_effects/util/pfx_mesh_creator.h"
#include "pfx_array.h"
#include "../base_level/collision/pfx_intersect_common.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_LARGETRIMESH_MAX_ISLANDS 256

///////////////////////////////////////////////////////////////////////////////
// 凸メッシュ作成時に使用する関数

PfxInt32 pfxCreateConvexMesh(PfxConvexMesh &convex,const PfxCreateConvexMeshParam &param)
{
	// Check input
	if(param.numVerts == 0 || param.numTriangles == 0 || !param.verts || !param.triangles)
		return SCE_PFX_ERR_INVALID_VALUE;
	
	if(param.numVerts >= SCE_PFX_NUMMESHVERTICES || param.numTriangles >= SCE_PFX_NUMMESHFACETS)
		return SCE_PFX_ERR_OUT_OF_RANGE;
	
	PfxArray<PfxVector3> vertList;
	for(PfxUInt32 i=0;i<param.numVerts;i++) {
		PfxFloat *vtx = (PfxFloat*)((uintptr_t)param.verts + param.vertexStrideBytes * i);
		vertList.push(PfxVector3(vtx[0],vtx[1],vtx[2]));
	}

	PfxArray<PfxUInt32> facetList;
	for(PfxUInt32 i=0;i<param.numTriangles;i++) {
		void *ids = (void*)((uintptr_t)param.triangles + param.triangleStrideBytes * i);
		PfxUInt32 idx[3];
		
		if(param.flag & SCE_PFX_MESH_FLAG_32BIT_INDEX) {
			if(param.flag & SCE_PFX_MESH_FLAG_NORMAL_FLIP) {
				idx[0] = ((PfxUInt32*)ids)[2];
				idx[1] = ((PfxUInt32*)ids)[1];
				idx[2] = ((PfxUInt32*)ids)[0];
			}
			else {
				idx[0] = ((PfxUInt32*)ids)[0];
				idx[1] = ((PfxUInt32*)ids)[1];
				idx[2] = ((PfxUInt32*)ids)[2];
			}
		}
		else if(param.flag & SCE_PFX_MESH_FLAG_16BIT_INDEX) {
			if(param.flag & SCE_PFX_MESH_FLAG_NORMAL_FLIP) {
				idx[0] = ((PfxUInt16*)ids)[2];
				idx[1] = ((PfxUInt16*)ids)[1];
				idx[2] = ((PfxUInt16*)ids)[0];
			}
			else {
				idx[0] = ((PfxUInt16*)ids)[0];
				idx[1] = ((PfxUInt16*)ids)[1];
				idx[2] = ((PfxUInt16*)ids)[2];
			}
		}
		else {
			return SCE_PFX_ERR_INVALID_FLAG;
		}

		// 面積が０の面を排除
		PfxFloat area = lengthSqr(cross(vertList[idx[1]]-vertList[idx[0]],vertList[idx[2]]-vertList[idx[0]]));

		if((param.flag & SCE_PFX_MESH_FLAG_AUTO_ELIMINATION) && area < 0.00001f) {
			continue;
		}
		
		facetList.push(idx[0]);
		facetList.push(idx[1]);
		facetList.push(idx[2]);
	}

	convex.m_numVerts = vertList.size();
	for(PfxUInt32 i=0;i<convex.m_numVerts;i++) {
		convex.m_verts[i] = vertList[i];
	}

	convex.m_numIndices = facetList.size();
	for(PfxUInt32 i=0;i<convex.m_numIndices;i++) {
		convex.m_indices[i] = facetList[i];
	}

	convex.updateAABB();

	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュ作成時に使用する構造体

struct PfxMcVert {
	PfxInt16 i;
	PfxInt16 flag;
	SCE_PFX_PADDING(1,12)
	PfxVector3  coord;
};

struct PfxMcEdge {
	PfxUInt32 vertId[2];  // 
	PfxUInt32 edgeId[2];  // 面におけるエッジのインデックス
	PfxUInt32 facetId[2]; // 
	PfxUInt32 numFacets;
	PfxUInt32 angleType;  // 辺の種類
	PfxFloat angle;		 // 辺の角度
	PfxMcEdge *next;
};

struct PfxMcFacet {
	PfxMcVert *v[3];
	PfxMcEdge *e[3];
	PfxInt32 neighbor[3];	// 隣接面のインデックス
	PfxInt32 neighborEdgeId[3];	// 隣接面のエッジインデックス
	PfxFloat thickness;	// 厚み
	PfxFloat area;			// 面積
	SCE_PFX_PADDING(1,8)
	PfxVector3 n;
	PfxVector3 aabbMin;
	PfxVector3 aabbMax;
};

struct PfxMcTriList {
	PfxMcFacet *facet;
	PfxMcTriList *next;

	PfxMcTriList()
	{
		facet = NULL;
		next = NULL;
	}
};

struct PfxMcEdgeEntry {
	PfxUInt8 vertId[2];
	PfxUInt8 facetId[2];
	PfxUInt8 numFacets;
	PfxUInt8 edgeNum[2];
	PfxUInt8 edgeId;
	SCE_PFX_PADDING(1,8)
	PfxVector3 dir;
	PfxMcEdgeEntry *next;
	SCE_PFX_PADDING(2,12)
};

struct PfxMcFacetLink {
	PfxInt32 baseEdgeId;
	PfxInt32 vid1;
	PfxInt32 vid2;
	PfxInt32 ifacetId;
	PfxInt32 iedgeId;
	PfxInt32 ofacetId;
	PfxInt32 oedgeId;
	
	PfxMcFacetLink(PfxInt32 baseEdgeId_,PfxInt32 vid1_,PfxInt32 vid2_,PfxInt32 ifacetId_,PfxInt32 iedgeId_,PfxInt32 ofacetId_,PfxInt32 oedgeId_)
	{
		baseEdgeId = baseEdgeId_;
		vid1 = vid1_;
		vid2 = vid2_;
		ifacetId = ifacetId_;
		iedgeId = iedgeId_;
		ofacetId = ofacetId_;
		oedgeId = oedgeId_;
	};
};

typedef PfxMcFacet* PfxMcFacetPtr;

struct PfxMcIslands {
	PfxArray<PfxMcFacetPtr> facetsInIsland[SCE_PFX_MAX_LARGETRIMESH_ISLANDS];
	PfxUInt32 numIslands;
	SCE_PFX_PADDING(1,12)
	
	PfxMcIslands()
	{
		numIslands = 0;
	}
	
	void add(PfxArray<PfxMcFacetPtr> &facets)
	{
		facetsInIsland[numIslands++] = facets;
	}
};

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュ作成時に使用する補助関数

static
bool intersect(const PfxMcFacet &facetA,const PfxMcFacet &facetB,PfxFloat &closestDistance)
{
	const PfxFloat epsilon = 0.00001f;

	PfxVector3 pA[3] = {
		facetA.v[0]->coord,
		facetA.v[1]->coord,
		facetA.v[2]->coord
	};
	
	PfxVector3 pB[3] = {
		facetB.v[0]->coord,
		facetB.v[1]->coord,
		facetB.v[2]->coord
	};

	// 面Bが面Aの厚みを考慮した範囲内に有るかどうかチェック

	// 上下面
	{
		PfxPlane planeA(facetA.n,pA[0]);
		PfxFloat dmin = SCE_PFX_FLT_MAX;
		PfxFloat dmax = -SCE_PFX_FLT_MAX;
		for(int i=0;i<3;i++) {
			PfxFloat d = planeA.onPlane(pB[i]);
			dmin = SCE_PFX_MIN(dmin,d);
			dmax = SCE_PFX_MAX(dmax,d);
		}
		
		if(dmin > -epsilon || dmax < -facetA.thickness) return false;
		
		// 面Aと面Bの最近接距離
		if(dmax > 0.0f) {
			// 面A,Bは交差
			return false;
		}
		else if(dmax > -epsilon) {
			// 隣接面
			return false;
		}
		else {
			closestDistance = -dmax;
		}
	}
	
	// 側面
	for(int p=0;p<3;p++) {
		PfxVector3 sideVec = normalize(cross((pA[(p+1)%3]-pA[p]),facetA.n));
		PfxPlane planeA(sideVec,pA[p]);

		PfxFloat dmin = SCE_PFX_FLT_MAX;
		for(int i=0;i<3;i++) {
			PfxFloat d = planeA.onPlane(pB[i]);
			dmin = SCE_PFX_MIN(dmin,d);
		}
		
		if(dmin > -epsilon) return false;
	}
	
	return true;
}

static
void divideMeshes(
	PfxUInt32 numFacetsLimit,PfxFloat islandsRatio,
	PfxMcIslands &islands,
	PfxArray<PfxMcFacetPtr> &facets,
	const PfxVector3 &center,const PfxVector3 &half)
{
	PfxFloat halfLimit = length(half) * islandsRatio;
	
	// 含まれる面数が規定値以下であれば、アイランドに登録
	if((facets.size() <= SCE_PFX_NUMMESHFACETS && length(half) < halfLimit) || 
		(facets.size() <= numFacetsLimit ) ) {
		islands.add(facets);
		return;
	}

	// さらに分割
	PfxVector3 newCenter0,newCenter1;
	PfxVector3 newHalf0,newHalf1;
	PfxArray<PfxMcFacetPtr> newFacets0;
	PfxArray<PfxMcFacetPtr> newFacets1;

	// 最も適切と思われる分離軸を探す
	int divAxis;
	{
		if(half[0] > half[1]) {
			if(half[0] > half[2]) {
				divAxis = 0;
			}
			else if(half[1] > half[2]) {
				divAxis = 1;
			}
			else {
				divAxis = 2;
			}
		}
		else {
			if(half[1] > half[2]) {
				divAxis = 1;
			}
			else if(half[0] > half[2]) {
				divAxis = 0;
			}
			else {
				divAxis = 2;
			}
		}
	}

	// 中心で分割して、さらに再帰的に処理を続ける
	{
		PfxVector3 movCenter(0.0f);
		movCenter[divAxis] = 0.5f*half[divAxis];
		
		newCenter0 = center + movCenter;
		newCenter1 = center - movCenter;
		newHalf0 = half;
		newHalf0[divAxis] *= 0.5f;
		newHalf1 = newHalf0;
	}
	
	// 新しいAABBに含まれる面をそれぞれの領域に分配
	for(PfxUInt32 f=0;f<facets.size();f++) {
		// 面のAABB
		PfxVector3 facetCenter = (facets[f]->aabbMin + facets[f]->aabbMax) * 0.5f;
		PfxVector3 facetHalf = (facets[f]->aabbMax - facets[f]->aabbMin) * 0.5f;
		
		// AABB判定
		if(!(fabsf(newCenter0[divAxis]-facetCenter[divAxis]) > (newHalf0[divAxis]+facetHalf[divAxis]))) {
			// この面はAABB0に登録
			newFacets0.push(facets[f]);
		}
		else {
			// この面はAABB1に登録
			newFacets1.push(facets[f]);
		}
	}

	if(newFacets0.size() < newFacets1.size()) {
		if(newFacets0.size() > 0)
			divideMeshes(numFacetsLimit,islandsRatio,islands,newFacets0,newCenter0,newHalf0);
		if(newFacets1.size() > 0)
			divideMeshes(numFacetsLimit,islandsRatio,islands,newFacets1,newCenter1,newHalf1);
	}
	else {
		if(newFacets1.size() > 0)
			divideMeshes(numFacetsLimit,islandsRatio,islands,newFacets1,newCenter1,newHalf1);
		if(newFacets0.size() > 0)
			divideMeshes(numFacetsLimit,islandsRatio,islands,newFacets0,newCenter0,newHalf0);
	}
}

static
int addIslandToLargeTriMesh(PfxLargeTriMesh &lmesh,PfxTriMesh &island)
{
	SCE_PFX_ASSERT(island.m_numFacets <= SCE_PFX_NUMMESHFACETS);

	int newIsland = lmesh.m_numIslands++;
	lmesh.m_islands[newIsland] = island;
	
	// アイランドローカルのAABBを計算
	if(island.m_numFacets > 0) {
		PfxVector3 aabbMin(SCE_PFX_FLT_MAX),aabbMax(-SCE_PFX_FLT_MAX);

		for(PfxUInt32 i=0;i<island.m_numFacets;i++) {
			aabbMin = minPerElem(aabbMin,pfxReadVector3(island.m_facets[i].m_center)-pfxReadVector3(island.m_facets[i].m_half));
			aabbMax = maxPerElem(aabbMax,pfxReadVector3(island.m_facets[i].m_center)+pfxReadVector3(island.m_facets[i].m_half));
		}

		PfxVecInt3 aabbMinL,aabbMaxL;
		lmesh.getLocalPosition(aabbMin,aabbMax,aabbMinL,aabbMaxL);

		pfxSetXMin(lmesh.m_aabbList[newIsland],aabbMinL.getX());
		pfxSetXMax(lmesh.m_aabbList[newIsland],aabbMaxL.getX());
		pfxSetYMin(lmesh.m_aabbList[newIsland],aabbMinL.getY());
		pfxSetYMax(lmesh.m_aabbList[newIsland],aabbMaxL.getY());
		pfxSetZMin(lmesh.m_aabbList[newIsland],aabbMinL.getZ());
		pfxSetZMax(lmesh.m_aabbList[newIsland],aabbMaxL.getZ());
	}
	
	return newIsland;
}

static
void createIsland(PfxTriMesh &island,const PfxArray<PfxMcFacetPtr> &facets)
{
	if(facets.empty()) return;
	
	island.m_numFacets = facets.size();
	
	PfxUInt32 vertsFlag[(0xff*SCE_PFX_NUMMESHFACETS*3+31)/32];
	memset(vertsFlag,0,sizeof(PfxUInt32)*((0xff*SCE_PFX_NUMMESHFACETS*3+31)/32));
	
	PfxArray<PfxMcEdgeEntry*> edgeHead(facets.size()*3);
	PfxArray<PfxMcEdgeEntry> edgeList(facets.size()*3);

	PfxMcEdgeEntry* nl = NULL;
	edgeHead.assign(facets.size()*3,nl);
	edgeList.assign(facets.size()*3,PfxMcEdgeEntry());
	
	int vcnt = 0;
	int ecnt = 0;
	for(PfxUInt32 f=0;f<facets.size();f++) {
		PfxMcFacet &iFacet = *facets[f];
		PfxMcEdge *iEdge[3] = {
			iFacet.e[0],
			iFacet.e[1],
			iFacet.e[2],
		};

		PfxFacet &oFacet = island.m_facets[f];

		oFacet.m_half[0] = oFacet.m_half[1] = oFacet.m_half[2] = 0.0f;
		oFacet.m_center[0] = oFacet.m_center[1] = oFacet.m_center[2] = 0.0f;
		pfxStoreVector3(iFacet.n,oFacet.m_normal);
		oFacet.m_thickness = iFacet.thickness;
		
		// Vertex
		for(int v=0;v<3;v++) {
			PfxMcVert *vert = facets[f]->v[v];
			PfxUInt32 idx = vert->i;
			PfxUInt32 mask = 1 << (idx & 31);
			if((vertsFlag[idx>>5] & mask) == 0) {
				SCE_PFX_ASSERT(vcnt<SCE_PFX_NUMMESHVERTICES);
                vertsFlag[idx>>5] |= mask;
                island.m_verts[vcnt] = vert->coord;
				vert->flag = vcnt;// 新しいインデックス
				vcnt++;
			}
			oFacet.m_vertIds[v] = (PfxUInt8)vert->flag;
		}
		
		// Edge
		for(int v=0;v<3;v++) {
			PfxUInt8 viMin = SCE_PFX_MIN(oFacet.m_vertIds[v],oFacet.m_vertIds[(v+1)%3]);
			PfxUInt8 viMax = SCE_PFX_MAX(oFacet.m_vertIds[v],oFacet.m_vertIds[(v+1)%3]);
			int key = ((0x8da6b343*viMin+0xd8163841*viMax)%(island.m_numFacets*3));
			for(PfxMcEdgeEntry *e=edgeHead[key];;e=e->next) {
				if(!e) {
					edgeList[ecnt].vertId[0] = viMin;
					edgeList[ecnt].vertId[1] = viMax;
					edgeList[ecnt].facetId[0] = f;
					edgeList[ecnt].numFacets = 1;
					edgeList[ecnt].edgeNum[0] = v;
					edgeList[ecnt].edgeId = ecnt;
					edgeList[ecnt].dir = normalize(island.m_verts[viMax]-island.m_verts[viMin]);
					edgeList[ecnt].next = edgeHead[key];
					edgeHead[key] = &edgeList[ecnt];
					
					PfxEdge edge;
					edge.m_angleType = iEdge[v]->angleType;
					// 厚み角の設定 0～πを0～255の整数値に変換して格納
					edge.m_tilt = (PfxUInt8)((iEdge[v]->angle/(0.5f*SCE_PFX_PI))*255.0f);
					edge.m_vertId[0] = viMin;
					edge.m_vertId[1] = viMax;
					
					oFacet.m_edgeIds[v] = ecnt;
					island.m_edges[ecnt] = edge;
					SCE_PFX_ASSERT(ecnt <= SCE_PFX_NUMMESHEDGES);
					ecnt++;
					break;
				}
				
				if(e->vertId[0] == viMin && e->vertId[1] == viMax) {
					SCE_PFX_ASSERT(e->numFacets==1);
                    e->facetId[1] = f;
					e->edgeNum[1] = v;
					e->numFacets = 2;
					oFacet.m_edgeIds[v] = e->edgeId;
					break;
				}
			}
		}
	}

	island.m_numEdges = ecnt;
	island.m_numVerts = vcnt;
	
	island.updateAABB();
}

///////////////////////////////////////////////////////////////////////////////
// ラージメッシュ

PfxInt32 pfxCreateLargeTriMesh(PfxLargeTriMesh &lmesh,const PfxCreateLargeTriMeshParam &param)
{
	// Check input
	if(param.numVerts == 0 || param.numTriangles == 0 || !param.verts || !param.triangles)
		return SCE_PFX_ERR_INVALID_VALUE;
	
	if(param.islandsRatio < 0.0f || param.islandsRatio > 1.0f)
		return SCE_PFX_ERR_OUT_OF_RANGE;
	
	if(param.numFacetsLimit == 0 || param.numFacetsLimit > SCE_PFX_NUMMESHFACETS)
		return SCE_PFX_ERR_OUT_OF_RANGE;
	
	const PfxFloat epsilon = 0.00001f;
	
	PfxArray<PfxMcVert>  vertList(param.numVerts);		// 頂点配列
	PfxArray<PfxMcFacet> facetList(param.numTriangles);	// 面配列
	PfxArray<PfxMcEdge>  edgeList(param.numTriangles*3);	// エッジ配列
	PfxArray<PfxMcEdge*> edgeHead(param.numTriangles*3);
	
	//J 頂点配列作成
	for(PfxUInt32 i=0;i<param.numVerts;i++) {
		PfxFloat *vtx = (PfxFloat*)((uintptr_t)param.verts + param.vertexStrideBytes * i);
		PfxMcVert mcv;
		mcv.flag = 0;
		mcv.i = i;
		mcv.coord = pfxReadVector3(vtx);
		vertList.push(mcv);
	}
	
	// 面配列作成
	for(PfxUInt32 i=0;i<param.numTriangles;i++) {
		void *ids = (void*)((uintptr_t)param.triangles + param.triangleStrideBytes * i);
		
		PfxUInt32 idx[3];
		
		if(param.flag & SCE_PFX_MESH_FLAG_32BIT_INDEX) {
			if(param.flag & SCE_PFX_MESH_FLAG_NORMAL_FLIP) {
				idx[0] = ((PfxUInt32*)ids)[2];
				idx[1] = ((PfxUInt32*)ids)[1];
				idx[2] = ((PfxUInt32*)ids)[0];
			}
			else {
				idx[0] = ((PfxUInt32*)ids)[0];
				idx[1] = ((PfxUInt32*)ids)[1];
				idx[2] = ((PfxUInt32*)ids)[2];
			}
		}
		else if(param.flag & SCE_PFX_MESH_FLAG_16BIT_INDEX) {
			if(param.flag & SCE_PFX_MESH_FLAG_NORMAL_FLIP) {
				idx[0] = ((PfxUInt16*)ids)[2];
				idx[1] = ((PfxUInt16*)ids)[1];
				idx[2] = ((PfxUInt16*)ids)[0];
			}
			else {
				idx[0] = ((PfxUInt16*)ids)[0];
				idx[1] = ((PfxUInt16*)ids)[1];
				idx[2] = ((PfxUInt16*)ids)[2];
			}
		}
		else {
			return SCE_PFX_ERR_INVALID_FLAG;
		}
		
		const PfxVector3 pnts[3] = {
			vertList[idx[0]].coord,
			vertList[idx[1]].coord,
			vertList[idx[2]].coord,
		};

		// 面積が０の面を排除
		PfxFloat area = lengthSqr(cross(pnts[1]-pnts[0],pnts[2]-pnts[0]));

		if((param.flag & SCE_PFX_MESH_FLAG_AUTO_ELIMINATION) && area < 0.00001f) {
			continue;
		}

		PfxMcFacet facet;
		facet.v[0] = &vertList[idx[0]];
		facet.v[1] = &vertList[idx[1]];
		facet.v[2] = &vertList[idx[2]];
		facet.e[0] = facet.e[1] = facet.e[2] = NULL;
		facet.n = normalize(cross(pnts[2]-pnts[1],pnts[0]-pnts[1]));
		facet.area = area;
		facet.thickness = param.defaultThickness;
		facet.neighbor[0] = facet.neighbor[1] = facet.neighbor[2] = -1;
		facet.neighborEdgeId[0] = facet.neighborEdgeId[1] = facet.neighborEdgeId[2] = -1;

		facetList.push(facet);
	}

	const PfxUInt32 numTriangles = facetList.size();

	{
		PfxArray<PfxMcTriList> triEntry(numTriangles*3);
		PfxArray<PfxMcTriList*> triHead(numTriangles*3);	// 頂点から面への参照リスト
		PfxInt32 cnt = 0;
		
		PfxMcTriList* nl = NULL;
		triEntry.assign(numTriangles*3,PfxMcTriList());
		triHead.assign(numTriangles*3,nl);
		
		// 頂点から面への参照リストを作成
		for(PfxUInt32 i=0;i<numTriangles;i++) {
			for(PfxUInt32 v=0;v<3;v++) {
				PfxUInt32 vertId = facetList[i].v[v]->i;
				triEntry[cnt].facet = &facetList[i];
				triEntry[cnt].next = triHead[vertId];
				triHead[vertId] = &triEntry[cnt++];
			}
		}
		
		// 同一頂点をまとめる
		if(param.flag & SCE_PFX_MESH_FLAG_AUTO_ELIMINATION) {
			for(PfxUInt32 i=0;i<param.numVerts;i++) {
				if(vertList[i].flag == 1) continue;
				for(PfxUInt32 j=i+1;j<param.numVerts;j++) {
					if(vertList[j].flag == 1) continue;

					PfxFloat lenSqr = lengthSqr(vertList[i].coord-vertList[j].coord);
					
					if(lenSqr < epsilon) {
						//SCE_PFX_PRINTF("same position %d,%d\n",i,j);
						vertList[j].flag = 1; // 同一点なのでフラグを立てる
						for(PfxMcTriList *f=triHead[j];f!=NULL;f=f->next) {
							for(PfxInt32 k=0;k<3;k++) {
								if(f->facet->v[k] == &vertList[j]) {
									f->facet->v[k] = &vertList[i]; // 頂点を付け替える
									break;
								}
							}
						}
					}
				}
			}
		}
	}

	// 接続面間の角度を算出して面にセット
	PfxMcEdge *nl = NULL;
	edgeHead.assign(numTriangles*3,nl);
	edgeList.assign(numTriangles*3,PfxMcEdge());
	
	// エッジ配列の作成
	PfxUInt32 ecnt = 0;
	for(PfxUInt32 i=0;i<numTriangles;i++) {
		PfxMcFacet &f = facetList[i];
		
		for(PfxUInt32 v=0;v<3;v++) {
			uintptr_t vp1 = ((uintptr_t)f.v[v]-(uintptr_t)&vertList[0])/sizeof(PfxMcVert);
			uintptr_t vp2 = ((uintptr_t)f.v[(v+1)%3]-(uintptr_t)&vertList[0])/sizeof(PfxMcVert);
			PfxUInt32 viMin = SCE_PFX_MIN(vp1,vp2);
			PfxUInt32 viMax = SCE_PFX_MAX(vp1,vp2);
			PfxInt32 key = ((0x8da6b343*viMin+0xd8163841*viMax)%(numTriangles*3));
			for(PfxMcEdge *e = edgeHead[key];;e=e->next) {
				if(!e) {
					edgeList[ecnt].vertId[0] = viMin;
					edgeList[ecnt].vertId[1] = viMax;
					edgeList[ecnt].facetId[0] = i;
					edgeList[ecnt].edgeId[0] = v;
					edgeList[ecnt].numFacets = 1;
					edgeList[ecnt].next = edgeHead[key];
					edgeList[ecnt].angleType = SCE_PFX_EDGE_CONVEX;
					edgeList[ecnt].angle = 0.0f;
					edgeHead[key] = &edgeList[ecnt];
					f.e[v] = &edgeList[ecnt];
					ecnt++;
					break;
				}
				
				if(e->vertId[0] == viMin && e->vertId[1] == viMax) {
					SCE_PFX_ALWAYS_ASSERT_MSG(e->numFacets == 1,"An edge connected with over 2 triangles is invalid");
                    e->facetId[1] = i;
					e->edgeId[1] = v;
					e->numFacets = 2;
					f.e[v] = e;
					f.neighbor[v] = e->facetId[0];
					f.neighborEdgeId[v] = e->edgeId[0];
					facetList[e->facetId[0]].neighbor[e->edgeId[0]] = i;
					facetList[e->facetId[0]].neighborEdgeId[e->edgeId[0]] = e->edgeId[1];
					break;
				}
			}
		}
	}
	
	// 角度を計算
	for(PfxUInt32 i=0;i<numTriangles;i++) {
		PfxMcFacet &facetA = facetList[i];

		PfxQueue<PfxMcFacetLink> cqueue(ecnt);

		for(PfxUInt32 j=0;j<3;j++) {
			if(facetA.neighbor[j] >= 0) {
				cqueue.push(PfxMcFacetLink(
					j,
					facetA.e[j]->vertId[0],facetA.e[j]->vertId[1],
					i,j,
					facetA.neighbor[j],facetA.neighborEdgeId[j]));
			}
		}

		while(!cqueue.empty()) {
			PfxMcFacetLink link = cqueue.front();
			cqueue.pop();
			
			PfxMcFacet &ofacet = facetList[link.ofacetId];
			PfxMcEdge *edge = ofacet.e[link.oedgeId];
			
			// facetAとのなす角を計算
			{
				// 面に含まれるが、このエッジに含まれない点
				PfxUInt32 ids[3] = {2,0,1};
				PfxVector3 v1 = facetA.v[ids[link.baseEdgeId]]->coord;
				PfxVector3 v2 = ofacet.v[ids[link.oedgeId]]->coord;
				
				// エッジの凹凸判定
				PfxVector3 midPnt = (v1 + v2) * 0.5f;
				PfxVector3 pntOnEdge = facetA.v[link.baseEdgeId]->coord;
				
				PfxFloat chk1 = dot(facetA.n,midPnt-pntOnEdge);
				PfxFloat chk2 = dot(ofacet.n,midPnt-pntOnEdge);
				
				if(chk1 < -epsilon && chk2 < -epsilon) {
					if(link.ifacetId == i) edge->angleType = SCE_PFX_EDGE_CONVEX;

					// 厚み角の判定に使う角度をセット
					if(param.flag & SCE_PFX_MESH_FLAG_AUTO_THICKNESS) {
						edge->angle = 0.5f*acosf(dot(facetA.n,ofacet.n));
					}
				}
				else if(chk1 > epsilon && chk2 > epsilon) {
					if(link.ifacetId == i) edge->angleType = SCE_PFX_EDGE_CONCAVE;
				}
				else {
					if(link.ifacetId == i) edge->angleType = SCE_PFX_EDGE_FLAT;
				}
			}
			
			// 次の接続面を登録（コメントアウトすると頂点で接続された面を考慮しない）
			if(param.flag & SCE_PFX_MESH_FLAG_AUTO_THICKNESS) {
				PfxInt32 nextEdgeId = (link.oedgeId+1)%3;
				PfxMcEdge *nextEdge = ofacet.e[nextEdgeId];
				if(ofacet.neighbor[nextEdgeId] >= 0 && ofacet.neighbor[nextEdgeId] != i && 
				  ((PfxInt32)nextEdge->vertId[0] == link.vid1 || (PfxInt32)nextEdge->vertId[0] == link.vid2 || 
				   (PfxInt32)nextEdge->vertId[1] == link.vid1 || (PfxInt32)nextEdge->vertId[1] == link.vid2) ) {
					cqueue.push(PfxMcFacetLink(
						link.baseEdgeId,
						link.vid1,link.vid2,
						link.ofacetId,link.iedgeId,
						ofacet.neighbor[nextEdgeId],ofacet.neighborEdgeId[nextEdgeId]));
				}
				nextEdgeId = (link.oedgeId+2)%3;
				nextEdge = ofacet.e[nextEdgeId];
				if(ofacet.neighbor[nextEdgeId] >= 0 && ofacet.neighbor[nextEdgeId] != i && 
				  ((PfxInt32)nextEdge->vertId[0] == link.vid1 || (PfxInt32)nextEdge->vertId[0] == link.vid2 || 
				   (PfxInt32)nextEdge->vertId[1] == link.vid1 || (PfxInt32)nextEdge->vertId[1] == link.vid2) ) {
					cqueue.push(PfxMcFacetLink(
						link.baseEdgeId,
						link.vid1,link.vid2,
						link.ofacetId,link.iedgeId,
						ofacet.neighbor[nextEdgeId],ofacet.neighborEdgeId[nextEdgeId]));
				}
			}
		}
	}
	
	// 面に厚みを付ける
	if(param.flag & SCE_PFX_MESH_FLAG_AUTO_THICKNESS) {
		for(PfxUInt32 i=0;i<numTriangles;i++) {
			PfxMcFacet &facetA = facetList[i];
			for(PfxUInt32 j=0;j<numTriangles;j++) {
				// 隣接面は比較対象にしない
				if( i==j ||
					j == (PfxInt32)facetA.e[0]->facetId[0] ||
					j == (PfxInt32)facetA.e[0]->facetId[1] ||
					j == (PfxInt32)facetA.e[1]->facetId[0] ||
					j == (PfxInt32)facetA.e[1]->facetId[1] ||
					j == (PfxInt32)facetA.e[2]->facetId[0] ||
					j == (PfxInt32)facetA.e[2]->facetId[1]) {
					continue;
				}
				
				PfxMcFacet &facetB = facetList[j];
				
				// 交差判定
				PfxFloat closestDistance=0;
				if(intersect(facetA,facetB,closestDistance)) {
					// 最近接距離/2を厚みとして採用
					facetA.thickness = SCE_PFX_MAX(param.defaultThickness,SCE_PFX_MIN(facetA.thickness,closestDistance * 0.5f));
				}
			}

		}
	}

	// 面の面積によって３種類に分類する
	PfxFloat areaMin=SCE_PFX_FLT_MAX,areaMax=-SCE_PFX_FLT_MAX;
	for(PfxUInt32 f=0;f<(PfxUInt32)numTriangles;f++) {
		PfxVector3 pnts[3] = {
			facetList[f].v[0]->coord,
			facetList[f].v[1]->coord,
			facetList[f].v[2]->coord,
		};
		areaMin = SCE_PFX_MIN(areaMin,facetList[f].area);
		areaMax = SCE_PFX_MAX(areaMax,facetList[f].area);
		
		// 面のAABBを算出
		facetList[f].aabbMin = minPerElem(pnts[2],minPerElem(pnts[1],pnts[0]));
		facetList[f].aabbMax = maxPerElem(pnts[2],maxPerElem(pnts[1],pnts[0]));
	}

	PfxFloat areaDiff = (areaMax-areaMin)/3.0f;
	PfxFloat areaLevel0,areaLevel1;
	areaLevel0 = areaMin + areaDiff;
	areaLevel1 = areaMin + areaDiff * 2.0f;

	PfxArray<PfxMcFacetPtr> facetsLv0(numTriangles);
	PfxArray<PfxMcFacetPtr> facetsLv1(numTriangles);
	PfxArray<PfxMcFacetPtr> facetsLv2(numTriangles);

	for(PfxUInt32 f=0;f<numTriangles;f++) {
		PfxFloat area = facetList[f].area;

		PfxMcFacet *fct = &facetList[f];
		if(area < areaLevel0) {
			facetsLv0.push(fct);
		}
		else if(area > areaLevel1) {
			facetsLv2.push(fct);
		}
		else {
			facetsLv1.push(fct);
		}
	}

	// アイランドの配列
	PfxMcIslands islands;
	PfxVector3 lmeshSize;

	// レベル毎にPfxTriMeshを作成
	if(!facetsLv0.empty()) {
		// 全体のAABBを求める
		PfxVector3 aabbMin,aabbMax,center,half;
		aabbMin =facetsLv0[0]->aabbMin;
		aabbMax = facetsLv0[0]->aabbMax;
		for(PfxUInt32 f=1;f<facetsLv0.size();f++) {
			aabbMin = minPerElem(facetsLv0[f]->aabbMin,aabbMin);
			aabbMax = maxPerElem(facetsLv0[f]->aabbMax,aabbMax);
		}
		center = ( aabbMin + aabbMax ) * 0.5f;
		half = ( aabbMax - aabbMin ) * 0.5f;

		// 再帰的に処理
		divideMeshes(
			param.numFacetsLimit,param.islandsRatio,
			islands,
			facetsLv0,
			center,half);

		lmeshSize = maxPerElem(lmeshSize,maxPerElem(absPerElem(aabbMin),absPerElem(aabbMax)));
	}

	if(!facetsLv1.empty()) {
		// 全体のAABBを求める
		PfxVector3 aabbMin,aabbMax,center,half;
		aabbMin =facetsLv1[0]->aabbMin;
		aabbMax = facetsLv1[0]->aabbMax;
		for(PfxUInt32 f=1;f<facetsLv1.size();f++) {
			aabbMin = minPerElem(facetsLv1[f]->aabbMin,aabbMin);
			aabbMax = maxPerElem(facetsLv1[f]->aabbMax,aabbMax);
		}
		center = ( aabbMin + aabbMax ) * 0.5f;
		half = ( aabbMax - aabbMin ) * 0.5f;

		// 再帰的に処理
		divideMeshes(
			param.numFacetsLimit,param.islandsRatio,
			islands,
			facetsLv1,
			center,half);

		lmeshSize = maxPerElem(lmeshSize,maxPerElem(absPerElem(aabbMin),absPerElem(aabbMax)));
	}

	if(!facetsLv2.empty()) {
		// 全体のAABBを求める
		PfxVector3 aabbMin,aabbMax,center,half;
		aabbMin =facetsLv2[0]->aabbMin;
		aabbMax = facetsLv2[0]->aabbMax;
		for(PfxUInt32 f=1;f<facetsLv2.size();f++) {
			aabbMin = minPerElem(facetsLv2[f]->aabbMin,aabbMin);
			aabbMax = maxPerElem(facetsLv2[f]->aabbMax,aabbMax);
		}
		center = ( aabbMin + aabbMax ) * 0.5f;
		half = ( aabbMax - aabbMin ) * 0.5f;

		// 再帰的に処理
		divideMeshes(
			param.numFacetsLimit,param.islandsRatio,
			islands,
			facetsLv2,
			center,half);

		lmeshSize = maxPerElem(lmeshSize,maxPerElem(absPerElem(aabbMin),absPerElem(aabbMax)));
	}

	lmesh.m_half = lmeshSize;

	// Check Islands
	//for(PfxInt32 i=0;i<islands.numIslands;i++) {
	//	SCE_PFX_PRINTF("island %d\n",i);
	//	for(PfxInt32 f=0;f<islands.facetsInIsland[i].size();f++) {
	//		PfxMcFacet *facet = islands.facetsInIsland[i][f];
	//		SCE_PFX_PRINTF("   %d %d %d\n",facet->v[0]->i,facet->v[1]->i,facet->v[2]->i);
	//	}
	//}

	// PfxLargeTriMeshの生成
	if(islands.numIslands > 0) {
		lmesh.m_numIslands = 0;
		lmesh.m_aabbList = (PfxAabb16*)SCE_PFX_UTIL_ALLOC(128,sizeof(PfxAabb16)*islands.numIslands);
		lmesh.m_islands = (PfxTriMesh*)SCE_PFX_UTIL_ALLOC(128,sizeof(PfxTriMesh)*islands.numIslands);
		
		PfxInt32 maxFacets=0,maxVerts=0,maxEdges=0;
		for(PfxUInt32 i=0;i<islands.numIslands;i++) {
			PfxTriMesh island;
			createIsland(island,islands.facetsInIsland[i]);
			addIslandToLargeTriMesh(lmesh,island);
			maxFacets = SCE_PFX_MAX(maxFacets,island.m_numFacets);
			maxVerts = SCE_PFX_MAX(maxVerts,island.m_numVerts);
			maxEdges = SCE_PFX_MAX(maxEdges,island.m_numEdges);
			//SCE_PFX_PRINTF("island %d verts %d edges %d facets %d\n",i,island.m_numVerts,island.m_numEdges,island.m_numFacets);
		}

		SCE_PFX_PRINTF("generate completed!\n\tinput mesh verts %d triangles %d\n\tislands %d max triangles %d verts %d edges %d\n",
			param.numVerts,param.numTriangles,
			lmesh.m_numIslands,maxFacets,maxVerts,maxEdges);
		SCE_PFX_PRINTF("\tsizeof(PfxLargeTriMesh) %d sizeof(PfxTriMesh) %d\n",sizeof(PfxLargeTriMesh),sizeof(PfxTriMesh));
	}
	else {
		SCE_PFX_PRINTF("islands overflow! %d/%d\n",islands.numIslands,SCE_PFX_LARGETRIMESH_MAX_ISLANDS);
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}

	return SCE_PFX_OK;
}

void pfxReleaseLargeTriMesh(PfxLargeTriMesh &lmesh)
{
	SCE_PFX_UTIL_FREE(lmesh.m_aabbList);
	SCE_PFX_UTIL_FREE(lmesh.m_islands);
	lmesh.m_numIslands = 0;
}

} //namespace PhysicsEffects
} //namespace sce
