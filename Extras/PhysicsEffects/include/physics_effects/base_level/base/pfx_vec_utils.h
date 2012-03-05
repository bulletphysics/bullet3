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

#ifndef _SCE_PFX_VEC_UTILS_H
#define _SCE_PFX_VEC_UTILS_H

#include "pfx_common.h"
#include "pfx_vec_int3.h"

namespace sce {
namespace PhysicsEffects {
static SCE_PFX_FORCE_INLINE PfxVector3 pfxReadVector3(const PfxFloat* fptr)
{
	PfxVector3 v;
	
loadXYZ(v, fptr);

	return v;
}

static SCE_PFX_FORCE_INLINE PfxPoint3 pfxReadPoint3(const PfxFloat* fptr)
{
	PfxPoint3 v;

loadXYZ(v, fptr);

	return v;
}

static SCE_PFX_FORCE_INLINE PfxQuat pfxReadQuat(const PfxFloat* fptr)
{
	PfxQuat vq;

loadXYZW(vq, fptr);

	return vq;
}

static SCE_PFX_FORCE_INLINE void pfxStoreVector3(const PfxVector3 &src, PfxFloat* fptr)
{
storeXYZ(src, fptr);
}

static SCE_PFX_FORCE_INLINE void pfxStorePoint3(const PfxPoint3 &src, PfxFloat* fptr)
{
storeXYZ(src, fptr);
}

static SCE_PFX_FORCE_INLINE void pfxStoreQuat(const PfxQuat &src, PfxFloat* fptr)
{
storeXYZW(src, fptr);
}

} // namespace PhysicsEffects
} // namespace sce

namespace sce {
namespace PhysicsEffects {
static SCE_PFX_FORCE_INLINE
void pfxGetPlaneSpace(const PfxVector3& n, PfxVector3& fptr, PfxVector3& q)
{
	if(fabsf(n[2]) > 0.707f) {
		// choose fptr in y-z plane
		PfxFloat a = n[1]*n[1] + n[2]*n[2];
		PfxFloat k = 1.0f/sqrtf(a);
		fptr[0] = 0;
		fptr[1] = -n[2]*k;
		fptr[2] = n[1]*k;
		// set q = n x fptr
		q[0] = a*k;
		q[1] = -n[0]*fptr[2];
		q[2] = n[0]*fptr[1];
	}
	else {
		// choose fptr in x-y plane
		PfxFloat a = n[0]*n[0] + n[1]*n[1];
		PfxFloat k = 1.0f/sqrtf(a);
		fptr[0] = -n[1]*k;
		fptr[1] = n[0]*k;
		fptr[2] = 0;
		// set q = n x fptr
		q[0] = -n[2]*fptr[1];
		q[1] = n[2]*fptr[0];
		q[2] = a*k;
	}
}

static SCE_PFX_FORCE_INLINE
void pfxGetRotationAngleAndAxis(const PfxQuat &unitQuat,PfxFloat &angle,PfxVector3 &axis)
{
	const PfxFloat epsilon=0.00001f;

	if(fabsf(unitQuat.getW()) < 1.0f-epsilon && lengthSqr(unitQuat.getXYZ()) > epsilon) {
		PfxFloat angleHalf = acosf(unitQuat.getW());
		PfxFloat sinAngleHalf = sinf(angleHalf);

		if(fabsf(sinAngleHalf) > 1.0e-10f) {
			axis = unitQuat.getXYZ()/sinAngleHalf;
		} else {
			axis = unitQuat.getXYZ();
		}
		angle = 2.0f*angleHalf;
	} else {
		angle = 0.0f;
		axis = PfxVector3(1.0f, 0.0f, 0.0f);
	}
}

static SCE_PFX_FORCE_INLINE
PfxFloat pfxSafeAtan2(PfxFloat y,PfxFloat x)
{
	if(SCE_PFX_SQR(x) < 0.000001f || SCE_PFX_SQR(y) < 0.000001f) {
		return 0.0f;
	}
	return atan2f(y,x);
}

static SCE_PFX_FORCE_INLINE
PfxVector3 pfxSafeNormalize(const PfxVector3 &vec)
{
	float lenSqr = lengthSqr( vec );

	if( lenSqr > 0.000001f ) {
		return normalize(vec);
	}else {
		return PfxVector3( 1.0f, 0.0f, 0.0f );
	}
}

static SCE_PFX_FORCE_INLINE
PfxVecInt3 pfxConvertCoordWorldToLocal(const PfxVector3 &coord,const PfxVector3 &center,const PfxVector3 &half)
{
	const PfxVector3 sz(65535.0f);
	PfxVector3 q = divPerElem(coord - center + half,2.0f*half);
	q = minPerElem(maxPerElem(q,PfxVector3(0.0f)),PfxVector3(1.0f)); // clamp 0.0 - 1.0
	q = mulPerElem(q,sz);
	return PfxVecInt3(q);
}

static SCE_PFX_FORCE_INLINE
void pfxConvertCoordWorldToLocal(
	const PfxVector3 &center,const PfxVector3 &half,
	const PfxVector3 &coordMin,const PfxVector3 &coordMax,
	PfxVecInt3 &localMin,PfxVecInt3 &localMax)
{
	const PfxVector3 sz(65535.0f);
	PfxVector3 qmin = divPerElem(coordMin - center + half,2.0f*half);
	qmin = minPerElem(maxPerElem(qmin,PfxVector3(0.0f)),PfxVector3(1.0f)); // clamp 0.0 - 1.0
	qmin = mulPerElem(qmin,sz);
	
	PfxVector3 qmax = divPerElem(coordMax - center + half,2.0f*half);
	qmax = minPerElem(maxPerElem(qmax,PfxVector3(0.0f)),PfxVector3(1.0f)); // clamp 0.0 - 1.0
	qmax = mulPerElem(qmax,sz);
localMin = PfxVecInt3(floorf(qmin[0]),floorf(qmin[1]),floorf(qmin[2]));
localMax = PfxVecInt3(ceilf(qmax[0]),ceilf(qmax[1]),ceilf(qmax[2]));
}

static SCE_PFX_FORCE_INLINE
PfxVector3 pfxConvertCoordLocalToWorld(const PfxVecInt3 &coord,const PfxVector3 &center,const PfxVector3 &half)
{
	PfxVector3 sz(65535.0f),vcoord(coord);
	PfxVector3 q = divPerElem(vcoord,sz);
	return mulPerElem(q,2.0f*half) + center - half;
}
} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_VEC_UTILS_H
