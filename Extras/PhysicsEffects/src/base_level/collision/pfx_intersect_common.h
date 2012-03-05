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

#ifndef _SCE_PFX_INTERSECT_COMMON_H
#define _SCE_PFX_INTERSECT_COMMON_H

#include "../../../include/physics_effects/base_level/collision/pfx_ray.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_INTERSECT_COMMON_EPSILON 0.00001f
#define SCE_PFX_RAY_TRIANGLE_EPSILON 0.00001f

// Internally used intersect functions

struct PfxTriangle
{
	PfxVector3 points[3];
	
	PfxTriangle(const PfxVector3 &p0,const PfxVector3 &p1,const PfxVector3 &p2)
	{
		points[0] = p0;
		points[1] = p1;
		points[2] = p2;
	}
};

struct PfxPlane
{
	PfxVector3 normal; // normal
	PfxVector3 point; // point on the plane
	
	PfxPlane(const PfxVector3 &n,const PfxVector3 &q)
	{
		normal = n;
		point = q;
	}
	
	PfxPlane(const PfxTriangle &triangle)
	{
		normal = normalize(cross(triangle.points[1]-triangle.points[0],triangle.points[2]-triangle.points[0]));
		point = triangle.points[0];
	}
	
	PfxFloat onPlane(const PfxVector3 &p) const
	{
		return dot((p-point),normal);
	}
};

static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayAABBFast(
	const PfxVector3 &rayStartPosition,
	const PfxVector3 &rayDirection,
	const PfxVector3 &AABBcenter,
	const PfxVector3 &AABBhalf,
	PfxFloat &variable)
{
	PfxVector3 AABBmin = AABBcenter - AABBhalf;
	PfxVector3 AABBmax = AABBcenter + AABBhalf;
	
	PfxVector3 dir = rayDirection;
	PfxVector3 absDir = absPerElem(dir);
	PfxVector3 sign = copySignPerElem(PfxVector3(1.0),dir);
	
if(absDir[0] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[0] < AABBmin[0] || rayStartPosition[0] > AABBmax[0]) {
		return false;
	}
	dir[0] = sign[0] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

if(absDir[1] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[1] < AABBmin[1] || rayStartPosition[1] > AABBmax[1]) {
		return false;
	}
	dir[1] = sign[1] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

if(absDir[2] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[2] < AABBmin[2] || rayStartPosition[2] > AABBmax[2]) {
		return false;
	}
	dir[2] = sign[2] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

	PfxVector3 t1 = divPerElem(AABBmin - rayStartPosition, dir);
	PfxVector3 t2 = divPerElem(AABBmax - rayStartPosition, dir);
	
	PfxVector3 tmin = minPerElem(t1,t2);
	PfxVector3 tmax = maxPerElem(t1,t2);
	
if(maxElem(tmin) > minElem(tmax)) return false;

if(tmin[0] > tmin[1]) {
	if(tmin[0] > tmin[2]) {
		variable = tmin[0];
	}
	else {
		variable = tmin[2];
	}
}
else {
	if(tmin[1] > tmin[2]) {
		variable = tmin[1];
	}
	else {
		variable = tmin[2];
	}
}

	return true;
}

static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayAABB(
	const PfxVector3 &rayStartPosition,
	const PfxVector3 &rayDirection,
	const PfxVector3 &AABBcenter,
	const PfxVector3 &AABBhalf,
	PfxFloat &variable,
	PfxVector3 &normal)
{
	PfxVector3 AABBmin = AABBcenter - AABBhalf;
	PfxVector3 AABBmax = AABBcenter + AABBhalf;
	
	PfxVector3 dir = rayDirection;
	PfxVector3 absDir = absPerElem(dir);
	PfxVector3 sign = copySignPerElem(PfxVector3(1.0),dir);

	// 始点がBoxの内側にあるか判定
	if( AABBmin[0] < rayStartPosition[0] && rayStartPosition[0] < AABBmax[0] &&
		AABBmin[1] < rayStartPosition[1] && rayStartPosition[1] < AABBmax[1] &&
		AABBmin[2] < rayStartPosition[2] && rayStartPosition[2] < AABBmax[2]) {
		return false;
	}

if(absDir[0] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[0] < AABBmin[0] || rayStartPosition[0] > AABBmax[0]) {
		return false;
	}
	dir[0] = sign[0] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

if(absDir[1] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[1] < AABBmin[1] || rayStartPosition[1] > AABBmax[1]) {
		return false;
	}
	dir[1] = sign[1] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

if(absDir[2] < SCE_PFX_INTERSECT_COMMON_EPSILON) {
	if(rayStartPosition[2] < AABBmin[2] || rayStartPosition[2] > AABBmax[2]) {
		return false;
	}
	dir[2] = sign[2] * SCE_PFX_INTERSECT_COMMON_EPSILON;
}

	PfxVector3 t1 = divPerElem(AABBmin - rayStartPosition, dir);
	PfxVector3 t2 = divPerElem(AABBmax - rayStartPosition, dir);
	
	PfxVector3 tmin = minPerElem(t1,t2);
	PfxVector3 tmax = maxPerElem(t1,t2);
	
	normal = PfxVector3(0);

if(maxElem(tmin) > minElem(tmax)) return false;

if(tmin[0] > tmin[1]) {
	if(tmin[0] > tmin[2]) {
		variable = tmin[0];
		normal[0] = -sign[0];
	}
	else {
		variable = tmin[2];
		normal[2] = -sign[2];
	}
}
else {
	if(tmin[1] > tmin[2]) {
		variable = tmin[1];
		normal[1] = -sign[1];
	}
	else {
		variable = tmin[2];
		normal[2] = -sign[2];
	}
}

	return true;
}

static SCE_PFX_FORCE_INLINE
void pfxClosestTwoLines(
	const PfxVector3 &p1,const PfxVector3 &q1, // line1
	const PfxVector3 &p2,const PfxVector3 &q2, // line2
	PfxVector3 &s1,PfxVector3 &s2)
{
	PfxVector3 v1 = q1-p1;
	PfxVector3 v2 = q2-p2;
	PfxVector3 r = p1 - p2;

	PfxFloat a = dot(v1,v1);
	PfxFloat e = dot(v2,v2);
	PfxFloat f = dot(v2,r);
	PfxFloat b = dot(v1,v2);
	PfxFloat c = dot(v1,r);
	PfxFloat den = a*e-b*b;
	
	PfxFloat s,t;
	
	if(den != 0.0f) {
		s = SCE_PFX_CLAMP((b*f-c*e)/den,0.0f,1.0f);
	}
	else {
		s = 0.0f;
	}
	
	t = (b*s+f)/e;
	
	if(t < 0.0f) {
		t = 0.0f;
		s = SCE_PFX_CLAMP(-c/a,0.0f,1.0f);
	}
	else if(t > 1.0f) {
		t = 1.0f;
		s = SCE_PFX_CLAMP((b-c)/a,0.0f,1.0f);
	}
	
	s1 = p1 + s * v1;
	s2 = p2 + t * v2;
}

static SCE_PFX_FORCE_INLINE
void pfxClosestPointAABB(
	const PfxVector3 &point,
	const PfxVector3 &AABBhalf,
	PfxVector3 &s)
{
	s = point;
	s = maxPerElem(s,-AABBhalf);
	s = minPerElem(s,AABBhalf);
}

static SCE_PFX_FORCE_INLINE
void pfxClosestPointTriangle(
	const PfxVector3 &point,
	const PfxTriangle &triangle,
	PfxVector3 &s)
{
	PfxVector3 a = triangle.points[0];
	PfxVector3 b = triangle.points[1];
	PfxVector3 c = triangle.points[2];
    PfxVector3 ab = b - a;
    PfxVector3 ac = c - a;
    PfxVector3 ap = point - a;
    PfxFloat d1 = dot(ab, ap);
    PfxFloat d2 = dot(ac, ap);
	if(d1 <= 0.0f && d2 <= 0.0f) {
		s = a;
		return;
	}

    PfxVector3 bp = point - b;
    PfxFloat d3 = dot(ab, bp);
    PfxFloat d4 = dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) {
		s = b;
		return;
	}

    PfxFloat vc = d1*d4 - d3*d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        PfxFloat v = d1 / (d1 - d3);
        s = a + v * ab;
		return;
    }

    PfxVector3 cp = point - c;
    PfxFloat d5 = dot(ab, cp);
    PfxFloat d6 = dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) {
		s = c;
		return;
	}

    PfxFloat vb = d5*d2 - d1*d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        PfxFloat w = d2 / (d2 - d6);
        s = a + w * ac;
		return;
    }

    PfxFloat va = d3*d6 - d5*d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        PfxFloat w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        s = b + w * (c - b);
		return;
    }

    PfxFloat den = 1.0f / (va + vb + vc);
    PfxFloat v = vb * den;
    PfxFloat w = vc * den;
    s = a + ab * v + ac * w;
}

static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayTriangle(
	const PfxVector3 &rayStartPosition,
	const PfxVector3 &rayDirection,
	const PfxTriangle &triangle,
	PfxFloat &variable)
{
	PfxFloat v,w;
	PfxVector3 ab = triangle.points[1] - triangle.points[0];
	PfxVector3 ac = triangle.points[2] - triangle.points[0];

	PfxVector3 n = cross(ab,ac);

	PfxFloat d = dot(-rayDirection,n);
	
	if(fabsf(d) < 0.00001f) return false;

	PfxVector3 ap = rayStartPosition - triangle.points[0];
	PfxFloat t = dot(ap,n) / d;

	if(t <= 0.0f || t >= 1.0f) return false;

	variable = t;

	PfxVector3 e = cross(-rayDirection,ap);
	v = dot(ac,e) / d;
	if(v < -SCE_PFX_RAY_TRIANGLE_EPSILON || v > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab,e) / d;
	if(w < -SCE_PFX_RAY_TRIANGLE_EPSILON || v+w > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	return true;
}

static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayTriangleWithoutFrontFace(
	const PfxVector3 &rayStartPosition,
	const PfxVector3 &rayDirection,
	const PfxTriangle &triangle,
	PfxFloat &variable)
{
	PfxFloat v,w;
	PfxVector3 ab = triangle.points[1] - triangle.points[0];
	PfxVector3 ac = triangle.points[2] - triangle.points[0];

	PfxVector3 n = cross(ab,ac);

	PfxFloat d = dot(-rayDirection,n);
	
	if(d >= 0.0f) return false;

	PfxVector3 ap = rayStartPosition - triangle.points[0];
	PfxFloat t = dot(ap,n) / d;

	if(t <= 0.0f || t >= 1.0f) return false;

	variable = t;

	PfxVector3 e = cross(-rayDirection,ap);
	v = dot(ac,e) / d;
	if(v < -SCE_PFX_RAY_TRIANGLE_EPSILON || v > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab,e) / d;
	if(w < -SCE_PFX_RAY_TRIANGLE_EPSILON || v+w > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	return true;
}

static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayTriangleWithoutBackFace(
	const PfxVector3 &rayStartPosition,
	const PfxVector3 &rayDirection,
	const PfxTriangle &triangle,
	PfxFloat &variable)
{
	PfxFloat v,w;
	PfxVector3 ab = triangle.points[1] - triangle.points[0];
	PfxVector3 ac = triangle.points[2] - triangle.points[0];

	PfxVector3 n = cross(ab,ac);

	PfxFloat d = dot(-rayDirection,n);
	
	if(d <= 0.0f) return false;

	PfxVector3 ap = rayStartPosition - triangle.points[0];
	PfxFloat t = dot(ap,n) / d;

	if(t <= 0.0f || t >= 1.0f) return false;

	variable = t;

	PfxVector3 e = cross(-rayDirection,ap);
	v = dot(ac,e) / d;
	if(v < -SCE_PFX_RAY_TRIANGLE_EPSILON || v > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab,e) / d;
	if(w < -SCE_PFX_RAY_TRIANGLE_EPSILON || v+w > 1.0f+SCE_PFX_RAY_TRIANGLE_EPSILON) return false;

	return true;
}

} //namespace PhysicsEffects
} //namespace sce


#endif // _SCE_PFX_INTERSECT_COMMON_H
