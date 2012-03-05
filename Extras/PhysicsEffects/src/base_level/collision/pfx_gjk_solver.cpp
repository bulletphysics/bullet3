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

#include "pfx_gjk_solver.h"
#include "pfx_intersect_common.h"


namespace sce {
namespace PhysicsEffects {


template <class T, int maxData = 20>
struct PfxGjkStack {
	int cur;
	T data[maxData];

	PfxGjkStack() : cur(0) {}

	void push(const T &d)
	{
		SCE_PFX_ASSERT(cur<maxData-1);
		data[cur++] = d;
	}
	
	T pop()
	{
		return data[--cur];
	}

	bool isEmpty()
	{
		return cur == 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Allocate Buffers

PfxGjkSolver::PfxGjkSolver()
{
vertsP = g_vertsP;
vertsQ = g_vertsQ;
vertsW = g_vertsW;
facets = g_facets;
facetsHead = g_facetsHead;
edges = g_edges;
}

PfxGjkSolver::~PfxGjkSolver()
{
}

///////////////////////////////////////////////////////////////////////////////
// Setup Buffers

void PfxGjkSolver::setup(void *sA,void *sB,PfxGetSupportVertexFunc fA,PfxGetSupportVertexFunc fB)
{
	shapeA = sA;
	shapeB = sB;
	getSupportVertexShapeA = fA;
	getSupportVertexShapeB = fB;
}

///////////////////////////////////////////////////////////////////////////////
// Construct Silhouette

void PfxGjkSolver::silhouette(PfxGjkFacet *facet,int i,PfxVector3 &w)
{
	PfxGjkStack<PfxGjkEdge> gs;

	gs.push(PfxGjkEdge(facet,i));

	do {
		PfxGjkEdge stk = gs.pop();
		PfxGjkFacet *ft = stk.f;

		if(ft->obsolete==0) {
			// wから見えるかどうかを判定
			if(dot(ft->normal,w-ft->closest) < 0.0f) {
				// 見えないのでエッジを登録
				SCE_PFX_ASSERT(numEdges<=(int)MAX_FACETS);
				edges[numEdges] = stk;
				numEdges++;
			}
			else {
				ft->obsolete = 1;
				gs.push(PfxGjkEdge(ft->adj[(stk.i+2)%3],ft->j[(stk.i+2)%3]));
				gs.push(PfxGjkEdge(ft->adj[(stk.i+1)%3],ft->j[(stk.i+1)%3]));
			}
		}
	} while(SCE_PFX_UNLIKELY(!gs.isEmpty()));
}

///////////////////////////////////////////////////////////////////////////////
// Detect Penetration Depth (EPA)

PfxFloat PfxGjkSolver::detectPenetrationDepth(
	const PfxTransform3 &transformA,const PfxTransform3 &transformB,
	PfxVector3 &pA,PfxVector3 &pB,PfxVector3 &normal)
{
	PfxMatrix3 invRotA = transpose(transformA.getUpper3x3());
	PfxMatrix3 invRotB = transpose(transformB.getUpper3x3());

	int epaIterationCount = 0;
	PfxFloat distance = SCE_PFX_FLT_MAX;

	numFacets = 0;
	numFacetsHead = 0;

	// 初期状態の判定
	if(m_simplex.numVertices <= 1) {
		return distance;
	}
	else if(m_simplex.numVertices == 2) {
		PfxVector3 v0 = m_simplex.W[0];
		PfxVector3 v1 = m_simplex.W[1];
		PfxVector3 dir = normalize(v1-v0);
		PfxMatrix3 rot = PfxMatrix3::rotation(2.0943951023932f,dir);//120 deg
		int axis;
		if(dir[0] < dir[1]) {
			if(dir[0] < dir[2]) {
				axis = 0;
			}
			else {
				axis = 2;
			}
		}
		else {
			if(dir[1] < dir[2]) {
				axis = 1;
			}
			else {
				axis = 2;
			}
		}
        PfxVector3 vec(0.0f);
		vec[axis] = 1.0f;

		PfxVector3 aux[3];
		aux[0] = cross(dir,vec);
		aux[1] = rot * aux[0];
		aux[2] = rot * aux[1];

		PfxVector3 p[3],q[3],w[3];

		for(int i=0;i<3;i++) {
			PfxVector3 pInA,qInB;
			getSupportVertexShapeA(shapeA,invRotA * aux[i],pInA);
			getSupportVertexShapeB(shapeB,invRotB * (-aux[i]),qInB);
			p[i] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[i] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[i] = p[i] - q[i];
			vertsP[i] = p[i];
			vertsQ[i] = q[i];
			vertsW[i] = w[i];
		}

		if(originInTetrahedron(w[0],w[1],w[2],v0)) {
			vertsP[3] = m_simplex.P[0];
			vertsQ[3] = m_simplex.Q[0];
			vertsW[3] = m_simplex.W[0];
			numVerts = 4;
		}
		else if(originInTetrahedron(w[0],w[1],w[2],v1)){
			vertsP[3] = m_simplex.P[1];
			vertsQ[3] = m_simplex.Q[1];
			vertsW[3] = m_simplex.W[1];
			numVerts = 4;
		}
		else {
			return distance;
		}
	}
	else if(m_simplex.numVertices == 3) {
		numVerts = 3;
		for(int i=0;i<numVerts;i++) {
			vertsP[i] = m_simplex.P[i];
			vertsQ[i] = m_simplex.Q[i];
			vertsW[i] = m_simplex.W[i];
		}

		PfxVector3 p[2],q[2],w[2];

		{
			PfxVector3 v = cross(vertsW[2]-vertsW[0],vertsW[1]-vertsW[0]);
			PfxVector3 pInA,qInB;
			getSupportVertexShapeA(shapeA,invRotA * v,pInA);
			getSupportVertexShapeB(shapeB,invRotB * (-v),qInB);
			p[0] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[0] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[0] = p[0] - q[0];
			getSupportVertexShapeA(shapeA,invRotA * (-v),pInA);
			getSupportVertexShapeB(shapeB,invRotB * v,qInB);
			p[1] = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
			q[1] = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
			w[1] = p[1] - q[1];
		}

		if(originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],w[0])) {
			vertsP[3] = p[0];
			vertsQ[3] = q[0];
			vertsW[3] = w[0];
			numVerts = 4;
		}
		else if(originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],w[1])){
			vertsP[3] = p[1];
			vertsQ[3] = q[1];
			vertsW[3] = w[1];
			numVerts = 4;
		}
		else {
			return distance;
		}
	}
	else {
		numVerts = 4;
		for(int i=0;i<numVerts;i++) {
			vertsP[i] = m_simplex.P[i];
			vertsQ[i] = m_simplex.Q[i];
			vertsW[i] = m_simplex.W[i];
		}
	}

	SCE_PFX_ASSERT(numVerts == 4);

	// 原点が単体の内部にあるかどうかを判定
	if(SCE_PFX_UNLIKELY(!originInTetrahedron(vertsW[0],vertsW[1],vertsW[2],vertsW[3]))) {
		return distance;
	}

	// 面の向きをチェック
	if(dot(-vertsW[0],cross(vertsW[2]-vertsW[0],vertsW[1]-vertsW[0])) > 0.0f) {
		PfxVector3 vertsP1,vertsQ1,vertsW1;
		PfxVector3 vertsP3,vertsQ3,vertsW3;
		vertsQ1=vertsQ[1];vertsW1=vertsW[1];vertsP1=vertsP[1];
		vertsQ3=vertsQ[3];vertsW3=vertsW[3];vertsP3=vertsP[3];
		vertsQ[1]=vertsQ3;vertsW[1]=vertsW3;vertsP[1]=vertsP3;
		vertsQ[3]=vertsQ1;vertsW[3]=vertsW1;vertsP[3]=vertsP1;
	}

	{
		PfxGjkFacet *f0 = addFacet(0,1,2);
		PfxGjkFacet *f1 = addFacet(0,3,1);
		PfxGjkFacet *f2 = addFacet(0,2,3);
		PfxGjkFacet *f3 = addFacet(1,3,2);

		if(SCE_PFX_UNLIKELY(!f0 || !f1 || !f2 || !f3)) return distance;

		linkFacets(f0,0,f1,2);
		linkFacets(f0,1,f3,2);
		linkFacets(f0,2,f2,0);
		linkFacets(f1,0,f2,2);
		linkFacets(f1,1,f3,0);
		linkFacets(f2,1,f3,1);
	}

	// 探索
	PfxGjkFacet *facetMin = NULL;

	do {
		// 原点から一番近い点を算出し、そのベクトルと支点を返す
		int minFacetIdx = 0;
		{
			PfxFloat minDistSqr = SCE_PFX_FLT_MAX;
			for(int i=0;i<numFacetsHead;i++) {
				if(facetsHead[i]->obsolete==0 && facetsHead[i]->distSqr < minDistSqr) {
					minDistSqr = facetsHead[i]->distSqr;
					facetMin = facetsHead[i];
					minFacetIdx = i;
				}
			}
		}

		// リストからはずす
		facetsHead[minFacetIdx] = facetsHead[--numFacetsHead];

		PfxVector3 pInA(0.0f),qInB(0.0f);
		getSupportVertexShapeA(shapeA,invRotA * facetMin->normal,pInA);
		getSupportVertexShapeB(shapeB,invRotB * (-facetMin->normal),qInB);
		PfxVector3 p = transformA.getTranslation() + transformA.getUpper3x3() * pInA;
		PfxVector3 q = transformB.getTranslation() + transformB.getUpper3x3() * qInB;
		PfxVector3 w = p - q;
		PfxVector3 v = facetMin->closest;

		// 最近接点チェック
		PfxFloat l0 = length(v);
		PfxFloat l1 = dot(facetMin->normal,w);

		if((l1 - l0) < SCE_PFX_GJK_EPSILON) {
			break;
		}

		// 求めた点を追加して面を分割
		{
			SCE_PFX_ASSERT(numVerts<(int)MAX_VERTS);
			int vId = numVerts++;
			vertsP[vId] = p;
			vertsQ[vId] = q;
			vertsW[vId] = w;

			facetMin->obsolete = 1;

			numEdges = 0;

			silhouette(facetMin->adj[0],facetMin->j[0],w);
			silhouette(facetMin->adj[1],facetMin->j[1],w);
			silhouette(facetMin->adj[2],facetMin->j[2],w);

			if(SCE_PFX_UNLIKELY(numEdges == 0)) break;

			bool edgeCheck = true;
			PfxGjkFacet *firstFacet,*lastFacet;
			{
				PfxGjkEdge &edge = edges[0];
				int v0 = edge.f->v[(edge.i+1)%3];
				int v1 = edge.f->v[edge.i];
				firstFacet = addFacet(v0,v1,vId);
				if(SCE_PFX_UNLIKELY(!firstFacet)) {
					edgeCheck = false;
					break;
				}
				linkFacets(edge.f,edge.i,firstFacet,0);
				lastFacet = firstFacet;
			}

			if(SCE_PFX_UNLIKELY(!edgeCheck)) break;

			for(int e=1;e<numEdges;e++) {
				PfxGjkEdge &edge = edges[e];
				int v0 = edge.f->v[(edge.i+1)%3];
				int v1 = edge.f->v[edge.i];
				PfxGjkFacet *f = addFacet(v0,v1,vId);
				if(SCE_PFX_UNLIKELY(!f)) {edgeCheck=false;break;}
				linkFacets(edge.f,edge.i,f,0);
				linkFacets(f,2,lastFacet,1);
				lastFacet = f;
			}
			if(SCE_PFX_UNLIKELY(!edgeCheck)) break;

			linkFacets(lastFacet,1,firstFacet,2);
		}

		epaIterationCount++;
		if(SCE_PFX_UNLIKELY(epaIterationCount > SCE_PFX_EPA_ITERATION_MAX || numFacetsHead == 0)) {
			break;
		}
	} while(1);

	// 衝突点計算
	int v1 = facetMin->v[0];
	int v2 = facetMin->v[1];
	int v3 = facetMin->v[2];

	PfxVector3 p0 = vertsW[v2]-vertsW[v1];
	PfxVector3 p1 = vertsW[v3]-vertsW[v1];
	PfxVector3 p2 = facetMin->closest-vertsW[v1];

	PfxVector3 v = cross( p0, p1 );
	PfxVector3 crS = cross( v, p0 );
	PfxVector3 crT = cross( v, p1 );
	PfxFloat d0 = dot( crT, p0 );
	PfxFloat d1 = dot( crS, p1 );

	if(fabsf(d0) < SCE_PFX_GJK_EPSILON || fabsf(d1) < SCE_PFX_GJK_EPSILON) return distance;

	PfxFloat lamda1 = dot( crT, p2 ) / d0;
	PfxFloat lamda2 = dot( crS, p2 ) / d1;

	pA = vertsP[v1] + lamda1*(vertsP[v2]-vertsP[v1]) + lamda2*(vertsP[v3]-vertsP[v1]);
	pB = vertsQ[v1] + lamda1*(vertsQ[v2]-vertsQ[v1]) + lamda2*(vertsQ[v3]-vertsQ[v1]);

	PfxFloat lenSqr = lengthSqr(pB-pA);
	normal = normalize(pB-pA);

	return -sqrtf(lenSqr);
}

///////////////////////////////////////////////////////////////////////////////
// Gjk

PfxFloat PfxGjkSolver::collide( PfxVector3& normal, PfxPoint3 &pointA, PfxPoint3 &pointB,
						const PfxTransform3 & transformA,
						const PfxTransform3 & transformB,
						PfxFloat distanceThreshold)
{
	(void) distanceThreshold;

	int gjkIterationCount = 0;

	m_simplex.reset();

	PfxTransform3 cTransformA = transformA;
	PfxTransform3 cTransformB = transformB;
	PfxMatrix3 invRotA = transpose(cTransformA.getUpper3x3());
	PfxMatrix3 invRotB = transpose(cTransformB.getUpper3x3());

	PfxVector3 offset = (cTransformA.getTranslation() + cTransformB.getTranslation())*0.5f;
	cTransformA.setTranslation(cTransformA.getTranslation()-offset);
	cTransformB.setTranslation(cTransformB.getTranslation()-offset);

	PfxVector3 separatingAxis(-cTransformA.getTranslation());
	if(lengthSqr(separatingAxis) < 0.000001f) separatingAxis = PfxVector3(1.0,0.0,0.0);
	PfxFloat squaredDistance = SCE_PFX_FLT_MAX;
	PfxFloat delta = 0.0f;
	PfxFloat distance = SCE_PFX_FLT_MAX;

	for(;;) {
		// サポート頂点の取得
		PfxVector3 pInA,qInB;

		getSupportVertexShapeA(shapeA,invRotA * (-separatingAxis),pInA);
		getSupportVertexShapeB(shapeB,invRotB * separatingAxis,qInB);

		PfxVector3 p = cTransformA.getTranslation() + cTransformA.getUpper3x3() * pInA;
		PfxVector3 q = cTransformB.getTranslation() + cTransformB.getUpper3x3() * qInB;
		PfxVector3 w = p - q;

		delta = dot(separatingAxis,w);

		// 早期終了チェック
		if(SCE_PFX_UNLIKELY(delta > 0.0f)) {
			normal = separatingAxis;
			return distance;
		}

		if(SCE_PFX_UNLIKELY(m_simplex.inSimplex(w))) {
			break;
		}

		PfxFloat f0 = squaredDistance - delta;
		PfxFloat f1 = squaredDistance * SCE_PFX_GJK_EPSILON;

		if (SCE_PFX_UNLIKELY(f0 <= f1)) {
			break;
		}

		// 頂点を単体に追加
		m_simplex.addVertex(w,p,q);
		
		// 原点と単体の最近接点を求め、分離軸を返す
		if(SCE_PFX_UNLIKELY(!m_simplex.closest(separatingAxis))) {
			normal = separatingAxis;
			return distance;
		}

		squaredDistance = lengthSqr(separatingAxis);

		if(SCE_PFX_UNLIKELY(gjkIterationCount >= SCE_PFX_GJK_ITERATION_MAX || m_simplex.fullSimplex())) {
			break;
		}

		gjkIterationCount++;
	}

	PfxVector3 pA(0.0f),pB(0.0f);

	// ２つのConvexは交差しているので、接触点を探索する
	PfxFloat dist = detectPenetrationDepth(cTransformA,cTransformB,pA,pB,normal);

	//マージン考慮
	if(dist < 0.0f) {
		pA += normal * SCE_PFX_GJK_MARGIN;
		pB -= normal * SCE_PFX_GJK_MARGIN;
		dist = dot(normal,pA-pB);
		pointA = orthoInverse(transformA)*PfxPoint3(pA+offset);
		pointB = orthoInverse(transformB)*PfxPoint3(pB+offset);
	}

	return dist;
}
} //namespace PhysicsEffects
} //namespace sce
