//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxVec3.h"
#include "foundation/PxIntrinsics.h"
#include "PsFPU.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionRayBoxSIMD.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
*	Computes a ray-AABB intersection.
*	Original code by Andrew Woo, from "Graphics Gems", Academic Press, 1990
*	Optimized code by Pierre Terdiman, 2000 (~20-30% faster on my Celeron 500)
*	Epsilon value added by Klaus Hartmann. (discarding it saves a few cycles only)
*
*	Hence this version is faster as well as more robust than the original one.
*
*	Should work provided:
*	1) the integer representation of 0.0f is 0x00000000
*	2) the sign bit of the float is the most significant one
*
*	Report bugs: p.terdiman@codercorner.com
*
*	\param		aabb		[in] the axis-aligned bounding box
*	\param		origin		[in] ray origin
*	\param		dir			[in] ray direction
*	\param		coord		[out] impact coordinates
*	\return		true if ray intersects AABB
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RAYAABB_EPSILON 0.00001f
bool Gu::rayAABBIntersect(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& origin, const PxVec3& _dir, PxVec3& coord)
{
	Ps::IntBool Inside = Ps::IntTrue;
	PxVec3 MaxT(-1.0f, -1.0f, -1.0f);
	const PxReal* dir = &_dir.x;
	const PxU32* idir = reinterpret_cast<const PxU32*>(dir);
	// Find candidate planes.
	for(PxU32 i=0;i<3;i++)
	{
		if(origin[i] < minimum[i])
		{
			coord[i]	= minimum[i];
			Inside		= Ps::IntFalse;

			// Calculate T distances to candidate planes
			if(idir[i])
//			if(PX_IR(dir[i]))
				MaxT[i] = (minimum[i] - origin[i]) / dir[i];
		}
		else if(origin[i] > maximum[i])
		{
			coord[i]	= maximum[i];
			Inside		= Ps::IntFalse;

			// Calculate T distances to candidate planes
			if(idir[i])
//			if(PX_IR(dir[i]))
				MaxT[i] = (maximum[i] - origin[i]) / dir[i];
		}
	}

	// Ray origin inside bounding box
	if(Inside)
	{
		coord = origin;
		return true;
	}

	// Get largest of the maxT's for final choice of intersection
	PxU32 WhichPlane = 0;
	if(MaxT[1] > MaxT[WhichPlane])	WhichPlane = 1;
	if(MaxT[2] > MaxT[WhichPlane])	WhichPlane = 2;

	// Check final candidate actually inside box
	const PxU32* tmp = reinterpret_cast<const PxU32*>(&MaxT[WhichPlane]);
	if((*tmp)&PX_SIGN_BITMASK)
//	if(PX_IR(MaxT[WhichPlane])&PX_SIGN_BITMASK)
		return false;

	for(PxU32 i=0;i<3;i++)
	{
		if(i!=WhichPlane)
		{
			coord[i] = origin[i] + MaxT[WhichPlane] * dir[i];
#ifdef RAYAABB_EPSILON
			if(coord[i] < minimum[i] - RAYAABB_EPSILON || coord[i] > maximum[i] + RAYAABB_EPSILON)
#else
			if(coord[i] < minimum[i] || coord[i] > maximum[i])
#endif
				return false;
		}
	}
	return true;	// ray hits box
}



/**
*	Computes a ray-AABB intersection.
*	Original code by Andrew Woo, from "Graphics Gems", Academic Press, 1990
*	Optimized code by Pierre Terdiman, 2000 (~20-30% faster on my Celeron 500)
*	Epsilon value added by Klaus Hartmann. (discarding it saves a few cycles only)
*  Return of intersected face code and parameter by Adam!  Also modified behavior for ray starts inside AABB. 2004 :-p
*
*	Hence this version is faster as well as more robust than the original one.
*
*	Should work provided:
*	1) the integer representation of 0.0f is 0x00000000
*	2) the sign bit of the float is the most significant one
*
*	Report bugs: p.terdiman@codercorner.com
*
*	\param		minimum		[in] the smaller corner of the bounding box
*	\param		maximum		[in] the larger corner of the bounding box
*	\param		origin		[in] ray origin
*	\param		_dir		[in] ray direction
*	\param		coord		[out] impact coordinates
*	\param		t			[out] t such that coord = origin + dir * t
*	\return					false if ray does not intersect AABB, or ray origin is inside AABB. Else:
							1 + coordinate index of box axis that was hit 

	Note: sign bit that determines if the minimum (0) or maximum (1) of the axis was hit is equal to sign(coord[returnVal-1]).
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 Gu::rayAABBIntersect2(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& origin, const PxVec3& _dir, PxVec3& coord, PxReal & t)
{
	Ps::IntBool Inside = Ps::IntTrue;
	PxVec3 MaxT(-1.0f, -1.0f, -1.0f);
	const PxReal* dir = &_dir.x;
	const PxU32* idir = reinterpret_cast<const PxU32*>(dir);
	// Find candidate planes.
	for(PxU32 i=0;i<3;i++)
	{
		if(origin[i] < minimum[i])
		{
			coord[i]	= minimum[i];
			Inside		= Ps::IntFalse;

			// Calculate T distances to candidate planes
			if(idir[i])
//			if(PX_IR(dir[i]))
				MaxT[i] = (minimum[i] - origin[i]) / dir[i];
		}
		else if(origin[i] > maximum[i])
		{
			coord[i]	= maximum[i];
			Inside		= Ps::IntFalse;

			// Calculate T distances to candidate planes
			if(idir[i])
//			if(PX_IR(dir[i]))
				MaxT[i] = (maximum[i] - origin[i]) / dir[i];
		}
	}

	// Ray origin inside bounding box
	if(Inside)
	{
		coord = origin;
		t = 0;
		return 1;
	}

	// Get largest of the maxT's for final choice of intersection
	PxU32 WhichPlane = 0;
	if(MaxT[1] > MaxT[WhichPlane])	WhichPlane = 1;
	if(MaxT[2] > MaxT[WhichPlane])	WhichPlane = 2;

	// Check final candidate actually inside box
	const PxU32* tmp = reinterpret_cast<const PxU32*>(&MaxT[WhichPlane]);
	if((*tmp)&PX_SIGN_BITMASK)
//	if(PX_IR(MaxT[WhichPlane])&PX_SIGN_BITMASK)
		return 0;

	for(PxU32 i=0;i<3;i++)
	{
		if(i!=WhichPlane)
		{
			coord[i] = origin[i] + MaxT[WhichPlane] * dir[i];
#ifdef RAYAABB_EPSILON
			if(coord[i] < minimum[i] - RAYAABB_EPSILON || coord[i] > maximum[i] + RAYAABB_EPSILON)	return 0;
#else
			if(coord[i] < minimum[i] || coord[i] > maximum[i])	return 0;
#endif
		}
	}
	t = MaxT[WhichPlane];
	return 1 + WhichPlane;	// ray hits box
}

// Collide ray defined by ray origin (ro) and ray direction (rd)
// with the bounding box. Returns -1 on no collision and the face index
// for first intersection if a collision is found together with
// the distance to the collision points (tnear and tfar)

// ptchernev: 
// Should we use an enum, or should we keep the anonymous ints? 
// Should we increment the return code by one (return 0 for non intersection)?

int Gu::intersectRayAABB(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& ro, const PxVec3& rd, float& tnear, float& tfar)
{
	// Refactor
	int ret=-1;

	tnear = -PX_MAX_F32;
	tfar = PX_MAX_F32;
	// PT: why did we change the initial epsilon value?
	#define LOCAL_EPSILON PX_EPS_F32
	//#define LOCAL_EPSILON 0.0001f

	for(unsigned int a=0;a<3;a++)
	{
		if(rd[a]>-LOCAL_EPSILON && rd[a]<LOCAL_EPSILON)
		{
			if(ro[a]<minimum[a] || ro[a]>maximum[a])
				return -1;
		}	
		else
		{
			const PxReal OneOverDir = 1.0f / rd[a];
			PxReal t1 = (minimum[a]-ro[a]) * OneOverDir;
			PxReal t2 = (maximum[a]-ro[a]) * OneOverDir;

			unsigned int b = a;
			if(t1>t2)
			{
				PxReal t=t1;
				t1=t2;
				t2=t;
				b += 3;
			}

			if(t1>tnear)
			{
				tnear = t1;
				ret = int(b);
			}
			if(t2<tfar)
				tfar=t2;
			if(tnear>tfar || tfar<LOCAL_EPSILON)
				return -1;
		}
	}

	if(tnear>tfar || tfar<LOCAL_EPSILON)
		return -1;

	return ret;
}

// PT: specialized version where oneOverDir is available
int Gu::intersectRayAABB(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& ro, const PxVec3& rd, const PxVec3& oneOverDir, float& tnear, float& tfar)
{
	// PT: why did we change the initial epsilon value?
	#define LOCAL_EPSILON PX_EPS_F32
	//#define LOCAL_EPSILON 0.0001f

	if(physx::intrinsics::abs(rd.x)<LOCAL_EPSILON)
//	if(rd.x>-LOCAL_EPSILON && rd.x<LOCAL_EPSILON)
		if(ro.x<minimum.x || ro.x>maximum.x)
				return -1;
	if(physx::intrinsics::abs(rd.y)<LOCAL_EPSILON)
//	if(rd.y>-LOCAL_EPSILON && rd.y<LOCAL_EPSILON)
		if(ro.y<minimum.y || ro.y>maximum.y)
				return -1;
	if(physx::intrinsics::abs(rd.z)<LOCAL_EPSILON)
//	if(rd.z>-LOCAL_EPSILON && rd.z<LOCAL_EPSILON)
		if(ro.z<minimum.z || ro.z>maximum.z)
				return -1;

	PxReal t1x = (minimum.x - ro.x) * oneOverDir.x;
	PxReal t2x = (maximum.x - ro.x) * oneOverDir.x;
	PxReal t1y = (minimum.y - ro.y) * oneOverDir.y;
	PxReal t2y = (maximum.y - ro.y) * oneOverDir.y;
	PxReal t1z = (minimum.z - ro.z) * oneOverDir.z;
	PxReal t2z = (maximum.z - ro.z) * oneOverDir.z;

	int bx;
	int by;
	int bz;

	if(t1x>t2x)
	{
		PxReal t=t1x; t1x=t2x; t2x=t;
		bx = 3;
	}
	else
	{
		bx = 0;
	}

	if(t1y>t2y)
	{
		PxReal t=t1y; t1y=t2y; t2y=t;
		by = 4;
	}
	else
	{
		by = 1;
	}

	if(t1z>t2z)
	{
		PxReal t=t1z; t1z=t2z; t2z=t;
		bz = 5;
	}
	else
	{
		bz = 2;
	}

	int ret;
//	if(t1x>tnear)	// PT: no need to test for the first value
	{
		tnear = t1x;
		ret = bx;
	}
//	tfar = PxMin(tfar, t2x);
	tfar = t2x;		// PT: no need to test for the first value

	if(t1y>tnear)
	{
		tnear = t1y;
		ret = by;
	}
	tfar = PxMin(tfar, t2y);

	if(t1z>tnear)
	{
		tnear = t1z;
		ret = bz;
	}
	tfar = PxMin(tfar, t2z);

	if(tnear>tfar || tfar<LOCAL_EPSILON)
		return -1;

	return ret;
}

bool Gu::intersectRayAABB2(
	const PxVec3& minimum, const PxVec3& maximum, const PxVec3& ro, const PxVec3& rd, float maxDist, float& tnear, float& tfar)
{
	PX_ASSERT(maximum.x-minimum.x >= GU_MIN_AABB_EXTENT*0.5f);
	PX_ASSERT(maximum.y-minimum.y >= GU_MIN_AABB_EXTENT*0.5f);
	PX_ASSERT(maximum.z-minimum.z >= GU_MIN_AABB_EXTENT*0.5f);
	// not using vector math due to vector to integer pipeline penalties. TODO: verify that it's indeed faster
	namespace i = physx::intrinsics;

	// P+tD=a; t=(a-P)/D
	// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
	const PxF32 dEpsilon = 1e-9f;
	// using recipFast fails height field unit tests case where a ray cast from y=10000 to 0 gets clipped to 0.27 in y
	PxF32 invDx = i::recip(i::selectMax(i::abs(rd.x), dEpsilon) * i::sign(rd.x));
#ifdef RAYAABB_EPSILON
	PxF32 tx0 = (minimum.x - RAYAABB_EPSILON - ro.x) * invDx;
	PxF32 tx1 = (maximum.x + RAYAABB_EPSILON - ro.x) * invDx;
#else
	PxF32 tx0 = (minimum.x - ro.x) * invDx;
	PxF32 tx1 = (maximum.x - ro.x) * invDx;
#endif	
	PxF32 txMin = i::selectMin(tx0, tx1);
	PxF32 txMax = i::selectMax(tx0, tx1);

	PxF32 invDy = i::recip(i::selectMax(i::abs(rd.y), dEpsilon) * i::sign(rd.y));
#ifdef RAYAABB_EPSILON
	PxF32 ty0 = (minimum.y - RAYAABB_EPSILON - ro.y) * invDy;
	PxF32 ty1 = (maximum.y + RAYAABB_EPSILON - ro.y) * invDy;
#else
	PxF32 ty0 = (minimum.y - ro.y) * invDy;
	PxF32 ty1 = (maximum.y - ro.y) * invDy;
#endif
	PxF32 tyMin = i::selectMin(ty0, ty1);
	PxF32 tyMax = i::selectMax(ty0, ty1);

	PxF32 invDz = i::recip(i::selectMax(i::abs(rd.z), dEpsilon) * i::sign(rd.z));
#ifdef RAYAABB_EPSILON
	PxF32 tz0 = (minimum.z - RAYAABB_EPSILON - ro.z) * invDz;
	PxF32 tz1 = (maximum.z + RAYAABB_EPSILON - ro.z) * invDz;
#else
	PxF32 tz0 = (minimum.z - ro.z) * invDz;
	PxF32 tz1 = (maximum.z - ro.z) * invDz;
#endif
	PxF32 tzMin = i::selectMin(tz0, tz1);
	PxF32 tzMax = i::selectMax(tz0, tz1);

	PxF32 maxOfNears = i::selectMax(i::selectMax(txMin, tyMin), tzMin);
	PxF32 minOfFars  = i::selectMin(i::selectMin(txMax, tyMax), tzMax);

	tnear = i::selectMax(maxOfNears, 0.0f);
	tfar  = i::selectMin(minOfFars, maxDist);

	return (tnear<tfar);
}

bool Gu::intersectRayAABB2(const Ps::aos::Vec3VArg minimum, const Ps::aos::Vec3VArg maximum, 
					   const Ps::aos::Vec3VArg ro, const Ps::aos::Vec3VArg rd, const Ps::aos::FloatVArg maxDist, 
					   Ps::aos::FloatV& tnear, Ps::aos::FloatV& tfar)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const Vec3V eps = V3Load(1e-9f);
	const Vec3V absRD = V3Max(V3Abs(rd), eps);
	const Vec3V signRD = V3Sign(rd);
	const Vec3V rdV = V3Mul(absRD, signRD);
	const Vec3V rdVRecip = V3Recip(rdV);
	const Vec3V _min = V3Mul(V3Sub(minimum, ro), rdVRecip);
	const Vec3V _max = V3Mul(V3Sub(maximum, ro), rdVRecip);
	const Vec3V min = V3Min(_max, _min);
	const Vec3V max = V3Max(_max, _min);
	const FloatV maxOfNears = FMax(V3GetX(min), FMax(V3GetY(min), V3GetZ(min)));
	const FloatV minOfFars = FMin(V3GetX(max), FMin(V3GetY(max), V3GetZ(max)));

	tnear = FMax(maxOfNears, zero);
	tfar  = FMin(minOfFars, maxDist);
	//tfar  = FAdd(FMin(minOfFars, maxDist), V3GetX(eps)); // AP: + epsilon because a test vs empty box should return true

	return FAllGrtr(tfar, tnear) != 0;
}
