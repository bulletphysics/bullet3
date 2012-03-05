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

#ifndef _SCE_PFX_GJK_SOLVER_H
#define _SCE_PFX_GJK_SOLVER_H

#include "../../../include/physics_effects/base_level/base/pfx_common.h"
#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "pfx_simplex_solver.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_GJK_EPSILON			1e-04f
#define SCE_PFX_GJK_MARGIN			0.025f
#define SCE_PFX_GJK_ITERATION_MAX	10
#define SCE_PFX_EPA_ITERATION_MAX	10

///////////////////////////////////////////////////////////////////////////////
// Support Function

typedef void (*PfxGetSupportVertexFunc)(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);

///////////////////////////////////////////////////////////////////////////////
// Gjk
class PfxSimplexSolver;

class PfxGjkSolver
{
private:
	static const PfxUInt32 MAX_VERTS = 128;
	static const PfxUInt32 MAX_EDGES = 128;
	static const PfxUInt32 MAX_FACETS = 64;
	
	PfxSimplexSolver m_simplex;

	// 面
	struct PfxGjkFacet {
		PfxVector3 normal;		// 面の法線
		PfxVector3 closest;		// 原点からの最短ベクトル
		PfxUInt32 obsolete;		// 廃棄面判定
		PfxFloat distSqr;		// 原点からの距離
		PfxInt32 v[3];			// 頂点
		PfxInt32 j[3];			// 隣接面から見たIndex
		PfxGjkFacet *adj[3];	// 隣接面
		SCE_PFX_PADDING(1,4)
	};
	
	// エッジ
	struct PfxGjkEdge {
		PfxGjkFacet *f;
		PfxInt32 i;
		PfxGjkEdge() {}
		PfxGjkEdge(PfxGjkFacet *f_,PfxInt32 i_)
		{
			f = f_;
			i= i_;
		}
	};
	
	PfxVector3 g_vertsP[MAX_VERTS];
	PfxVector3 g_vertsQ[MAX_VERTS];
	PfxVector3 g_vertsW[MAX_VERTS];
	PfxGjkFacet g_facets[MAX_FACETS];
	PfxGjkFacet *g_facetsHead[MAX_FACETS];
	PfxGjkEdge  g_edges[MAX_EDGES];

	PfxVector3 SCE_PFX_ALIGNED(16) *vertsP;
	SCE_PFX_PADDING(1,12)
	PfxVector3 SCE_PFX_ALIGNED(16) *vertsQ;
	SCE_PFX_PADDING(2,12)
	PfxVector3 SCE_PFX_ALIGNED(16) *vertsW;
	SCE_PFX_PADDING(3,12)
	PfxGjkFacet SCE_PFX_ALIGNED(16) *facets;
	SCE_PFX_PADDING(4,12)
	PfxGjkFacet SCE_PFX_ALIGNED(16) **facetsHead;
	SCE_PFX_PADDING(5,12)
	PfxGjkEdge  SCE_PFX_ALIGNED(16) *edges;
	
	int numVerts;
	int numEdges;
	int numFacets;
	int numFacetsHead;
	SCE_PFX_PADDING(6,12)
	
	inline PfxGjkFacet *addFacet(int v1,int v2,int v3);

	inline void linkFacets(PfxGjkFacet *f1,int e1,PfxGjkFacet *f2,int e2);
	void silhouette(PfxGjkFacet *facet,int i,PfxVector3 &w);

	inline bool originInTetrahedron(const PfxVector3& p0,const PfxVector3& p1,const PfxVector3& p2,const PfxVector3& p3);

	PfxFloat detectPenetrationDepth(
		const PfxTransform3 &transformA,const PfxTransform3 &transformB,
		PfxVector3 &pA,PfxVector3 &pB,PfxVector3 &normal);

	void *shapeA;
	void *shapeB;
	PfxGetSupportVertexFunc getSupportVertexShapeA;
	PfxGetSupportVertexFunc getSupportVertexShapeB;

public:
	PfxGjkSolver();
	~PfxGjkSolver();
	
	void setup(void *sA,void *sB,PfxGetSupportVertexFunc fA,PfxGetSupportVertexFunc fB);
	
	PfxFloat collide( PfxVector3& normal, PfxPoint3 &pointA, PfxPoint3 &pointB,
					const PfxTransform3 & transformA,
					const PfxTransform3 & transformB,
					PfxFloat distanceThreshold = SCE_PFX_FLT_MAX);
};

inline
PfxGjkSolver::PfxGjkFacet *PfxGjkSolver::addFacet(int v1,int v2,int v3)
{
	if(SCE_PFX_UNLIKELY(numFacets == (int)MAX_FACETS))
		return NULL;

	PfxGjkFacet &facet = facets[numFacets];

	PfxVector3 V1 = vertsW[v1];
	PfxVector3 V2 = vertsW[v2];
	PfxVector3 V3 = vertsW[v3];
	facet.obsolete = 0;
	facet.v[0] = v1;
	facet.v[1] = v2;
	facet.v[2] = v3;

	PfxVector3 normal = cross(V3-V1,V2-V1);
	
	PfxFloat l = lengthSqr(normal);

	if(l < SCE_PFX_GJK_EPSILON * SCE_PFX_GJK_EPSILON) {
		return NULL;
	}

	normal /= sqrtf(l);
	facet.closest = dot(V1,normal)*normal;
	facet.normal =normal;

	facet.distSqr = lengthSqr(facet.closest);

	facetsHead[numFacetsHead++] = &facet;
	numFacets++;

	return &facet;
}

inline
void PfxGjkSolver::linkFacets(PfxGjkFacet *f1,int e1,PfxGjkFacet *f2,int e2)
{
	f1->adj[e1] = f2;
	f2->adj[e2] = f1;
	f1->j[e1] = e2;
	f2->j[e2] = e1;
}

inline
bool PfxGjkSolver::originInTetrahedron(const PfxVector3& p0,const PfxVector3& p1,const PfxVector3& p2,const PfxVector3& p3)
{
    PfxVector3 n0 = cross((p1-p0),(p2-p0));
    PfxVector3 n1 = cross((p2-p1),(p3-p1));
    PfxVector3 n2 = cross((p3-p2),(p0-p2));
    PfxVector3 n3 = cross((p0-p3),(p1-p3));

	return 
		dot(n0,p0) * dot(n0,p3-p0) <= 0.0f &&
		dot(n1,p1) * dot(n1,p0-p1) <= 0.0f &&
		dot(n2,p2) * dot(n2,p1-p2) <= 0.0f &&
		dot(n3,p3) * dot(n3,p2-p3) <= 0.0f;
}

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_GJK_SOLVER_H
