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

#ifndef _SCE_PFX_MESH_COMMON_H
#define _SCE_PFX_MESH_COMMON_H

#include "../../../include/physics_effects/base_level/base/pfx_common.h"
#include "../../../include/physics_effects/base_level/collision/pfx_tri_mesh.h"

namespace sce {
namespace PhysicsEffects {


struct PfxClosestPoints {
	PfxPoint3 pA[4],pB[4];
	PfxFloat distSqr[4];
	PfxFloat closestDistSqr;
	int numPoints;
	SCE_PFX_PADDING(1,8)
	
	PfxClosestPoints()
	{
		numPoints = 0;
		closestDistSqr = SCE_PFX_FLT_MAX;
	}
	
	void set(int i,const PfxPoint3 &pointA,const PfxPoint3 &pointB,PfxFloat d)
	{
		pA[i] = pointA;
		pB[i] = pointB;
		distSqr[i] = d;
	}
	
	void add(const PfxPoint3 &pointA,const PfxPoint3 &pointB,PfxFloat d)
	{
		const PfxFloat epsilon = 0.00001f;
		if(closestDistSqr < d) return;
		
		closestDistSqr = d + epsilon;
		
		int replaceId = -1;
		PfxFloat distMax = -SCE_PFX_FLT_MAX;
		for(int i=0;i<numPoints;i++) {
			if(lengthSqr(pA[i]-pointA) < epsilon) {
				return;
			}
			if(distMax < distSqr[i]) {
				distMax = distSqr[i];
				replaceId = i;
			}
		}
		
		replaceId = (numPoints<4)?(numPoints++):replaceId;
		
		set(replaceId,pointA,pointB,d);
	}
};

static SCE_PFX_FORCE_INLINE
PfxUInt32 pfxGatherFacets(
	const PfxTriMesh *mesh,
	const PfxFloat *aabbHalf,
	const PfxVector3 &offsetPos,
	const PfxMatrix3 &offsetRot,
	PfxUInt8 *selFacets)
{
	PfxMatrix3 absOffsetRot = absPerElem(offsetRot);
	
	PfxUInt32 numSelFacets = 0;
	
for(int f=0;f<(int)mesh->m_numFacets;f++) {
	const PfxFacet &facet = mesh->m_facets[f];

	PfxVector3 facetCenter = absPerElem(offsetPos + offsetRot * pfxReadVector3(facet.m_center));
	PfxVector3 halfBA = absOffsetRot * pfxReadVector3(facet.m_half);

	// ConvexBのAABBとチェック
	if(facetCenter[0] > (halfBA[0]+aabbHalf[0])) continue;
	if(facetCenter[1] > (halfBA[1]+aabbHalf[1])) continue;
	if(facetCenter[2] > (halfBA[2]+aabbHalf[2])) continue;
	
	// この面は判定
	selFacets[numSelFacets++] = (PfxUInt8)f;
}
	
	return numSelFacets;
}

static SCE_PFX_FORCE_INLINE
void pfxGetProjAxisPnts6(
	const PfxVector3 *verts,const PfxVector3 &axis,
	PfxFloat &distMin,PfxFloat &distMax)
{
PfxFloat p0 = dot(axis, verts[0]);
PfxFloat p1 = dot(axis, verts[1]);
PfxFloat p2 = dot(axis, verts[2]);
PfxFloat p3 = dot(axis, verts[3]);
PfxFloat p4 = dot(axis, verts[4]);
PfxFloat p5 = dot(axis, verts[5]);
distMin = SCE_PFX_MIN(p5,SCE_PFX_MIN(p4,SCE_PFX_MIN(p3,SCE_PFX_MIN(p2,SCE_PFX_MIN(p0,p1)))));
distMax = SCE_PFX_MAX(p5,SCE_PFX_MAX(p4,SCE_PFX_MAX(p3,SCE_PFX_MAX(p2,SCE_PFX_MAX(p0,p1)))));
}

static SCE_PFX_FORCE_INLINE
void pfxGetProjAxisPnts3(
	const PfxVector3 *verts,const PfxVector3 &axis,
	PfxFloat &distMin,PfxFloat &distMax)
{
PfxFloat p0 = dot(axis, verts[0]);
PfxFloat p1 = dot(axis, verts[1]);
PfxFloat p2 = dot(axis, verts[2]);
distMin = SCE_PFX_MIN(p2,SCE_PFX_MIN(p0,p1));
distMax = SCE_PFX_MAX(p2,SCE_PFX_MAX(p0,p1));
}

static SCE_PFX_FORCE_INLINE
void pfxGetProjAxisPnts2(
	const PfxVector3 *verts,const PfxVector3 &axis,
	PfxFloat &distMin,PfxFloat &distMax)
{
PfxFloat p0 = dot(axis, verts[0]);
PfxFloat p1 = dot(axis, verts[1]);
distMin = SCE_PFX_MIN(p0,p1);
distMax = SCE_PFX_MAX(p0,p1);
}

///////////////////////////////////////////////////////////////////////////////
// ２つのベクトルの向きをチェック

static SCE_PFX_FORCE_INLINE
bool pfxIsSameDirection(const PfxVector3 &vecA,const PfxVector3 &vecB)
{
return fabsf(dot(vecA,vecB)) > 0.9999f;
}

///////////////////////////////////////////////////////////////////////////////
// 面ローカルの座標を算出

static SCE_PFX_FORCE_INLINE
void pfxGetLocalCoords(
	const PfxVector3 &pointOnTriangle,
	const PfxTriangle &triangle,
	PfxFloat &s,PfxFloat &t)
{
	PfxVector3 v0 = triangle.points[1] - triangle.points[0];
	PfxVector3 v1 = triangle.points[2] - triangle.points[0];
	PfxVector3 dir = pointOnTriangle - triangle.points[0];
	PfxVector3 v = cross( v0, v1 );
	PfxVector3 crS = cross( v, v0 );
	PfxVector3 crT = cross( v, v1 );
	s = dot( crT, dir ) / dot( crT, v0 );
	t = dot( crS, dir ) / dot( crS, v1 );
}

// a,bからなる直線上に点pがあるかどうかを判定
static SCE_PFX_FORCE_INLINE
bool pfxPointOnLine(const PfxVector3 &p,const PfxVector3 &a,const PfxVector3 &b)
{
	PfxVector3 ab = normalize(b-a);
	PfxVector3 q = a + ab * dot(p-a,ab);
	return lengthSqr(p-q) < 0.00001f;
}

// 線分a,b上に点pがあるかどうかを判定
static SCE_PFX_FORCE_INLINE
bool pfxPointOnSegment(const PfxVector3 &p,const PfxVector3 &a,const PfxVector3 &b)
{
	PfxVector3 ab = b-a;
	PfxVector3 ap = p-a;
	PfxFloat denom = dot(ab,ab);
	PfxFloat num = dot(ap,ab);
	PfxFloat t = num/denom;
	if(t < 0.0f || t > 1.0f) return false;
	return (dot(ap,ap)-num*t) < 0.00001f;
}


} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_MESH_COMMON_H
