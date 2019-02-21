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

#ifndef GU_BV4_CAPSULE_SWEEP_INTERNAL_H
#define GU_BV4_CAPSULE_SWEEP_INTERNAL_H

// PT: for capsule-sweeps please refer to %SDKRoot%\InternalDocumentation\GU\Sweep strategies.ppt.
// We use:
// - method 3 if the capsule is axis-aligned (SWEEP_AABB_IMPL is defined)
// - method 2 otherwise (SWEEP_AABB_IMPL is undefined)

// PT: TODO: get rid of that one
static PX_FORCE_INLINE bool sweepSphereVSTriangle(	const PxVec3& center, const float radius,
													const PxVec3* PX_RESTRICT triVerts, const PxVec3& triUnitNormal,
													const PxVec3& unitDir,
													float& curT, bool& directHit)
{
	float currentDistance;
	if(!sweepSphereVSTri(triVerts, triUnitNormal, center, radius, unitDir, currentDistance, directHit, true))
		return false;

	// PT: using ">" or ">=" is enough to block the CCT or not in the DE5967 visual test. Change to ">=" if a repro is needed.
	if(currentDistance > curT)
		return false;
	curT = currentDistance;
	return true;
}

static PX_FORCE_INLINE bool sweepSphereVSQuad(	const PxVec3& center, const float radius,
												const PxVec3* PX_RESTRICT quadVerts, const PxVec3& quadUnitNormal,
												const PxVec3& unitDir,
												float& curT)
{
	float currentDistance;
	if(!sweepSphereVSQuad(quadVerts, quadUnitNormal, center, radius, unitDir, currentDistance))
		return false;

	// PT: using ">" or ">=" is enough to block the CCT or not in the DE5967 visual test. Change to ">=" if a repro is needed.
	if(currentDistance > curT)
		return false;
	curT = currentDistance;
	return true;
}

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: __fastcall removed to make it compile everywhere. Revisit.
static bool /*__fastcall*/ testTri(	const CapsuleSweepParams* PX_RESTRICT params, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const PxVec3& N,
								const PxVec3& unitDir, const float capsuleRadius, const float dpc0, float& curT, bool& status)
{
	// PT: TODO: check the assembly here (TA34704)
	PxVec3 currentTri[3];
	// PT: TODO: optimize this copy (TA34704)
	currentTri[0] = p0;
	currentTri[1] = p1;
	currentTri[2] = p2;

	// PT: beware, culling is only ok on the sphere I think
	if(rejectTriangle(params->mCapsuleCenter, unitDir, curT, capsuleRadius, currentTri, dpc0))
		return false;

	float magnitude = N.magnitude();
	if(magnitude==0.0f)
		return false;

	PxVec3 triNormal = N / magnitude;

	bool DirectHit;
	if(sweepSphereVSTriangle(params->mCapsuleCenter, capsuleRadius, currentTri, triNormal, unitDir, curT, DirectHit))
	{
		status = true;
	}
	return DirectHit;
}

// PT: TODO: __fastcall removed to make it compile everywhere. Revisit.
static void /*__fastcall*/ testQuad(const CapsuleSweepParams* PX_RESTRICT params, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const PxVec3& p3, const PxVec3& N,
								const PxVec3& unitDir, const float capsuleRadius, const float dpc0, float& curT, bool& status)
{
	// PT: TODO: optimize this copy (TA34704)
	PxVec3 currentQuad[4];
	currentQuad[0] = p0;
	currentQuad[1] = p1;
	currentQuad[2] = p2;
	currentQuad[3] = p3;

	// PT: beware, culling is only ok on the sphere I think
	if(rejectQuad(params->mCapsuleCenter, unitDir, curT, capsuleRadius, currentQuad, dpc0))
		return;

	float magnitude = N.magnitude();
	if(magnitude==0.0f)
		return;

	PxVec3 triNormal = N / magnitude;

	if(sweepSphereVSQuad(params->mCapsuleCenter, capsuleRadius, currentQuad, triNormal, unitDir, curT))
	{
		status = true;
	}
}

static PX_FORCE_INLINE float Set2(const PxVec3& p0, const PxVec3& n, const PxVec3& p)
{
	return (p-p0).dot(n);
}

static PX_FORCE_INLINE bool sweepCapsuleVsTriangle(const CapsuleSweepParams* PX_RESTRICT params, const PxTriangle& triangle, float& t, bool isDoubleSided, PxVec3& normal)
{
	const PxVec3& unitDir = params->mLocalDir_Padded;

	// Create triangle normal
	PxVec3 denormalizedNormal = (triangle.verts[0] - triangle.verts[1]).cross(triangle.verts[0] - triangle.verts[2]);

	normal = denormalizedNormal;

	// Backface culling
	const bool culled = denormalizedNormal.dot(unitDir) > 0.0f;
	if(culled)
	{
		if(!isDoubleSided)
			return false;

		denormalizedNormal = -denormalizedNormal;
	}

	const float capsuleRadius = params->mLocalCapsule.radius;
	float curT = params->mStabbedFace.mDistance;
	const float dpc0 = params->mCapsuleCenter.dot(unitDir);

	bool status = false;

	// Extrude mesh on the fly
	const PxVec3 p0 = triangle.verts[0] - params->mExtrusionDir;
	const PxVec3 p1 = triangle.verts[1+culled] - params->mExtrusionDir;
	const PxVec3 p2 = triangle.verts[2-culled] - params->mExtrusionDir;

	const PxVec3 p0b = triangle.verts[0] + params->mExtrusionDir;
	const PxVec3 p1b = triangle.verts[1+culled] + params->mExtrusionDir;
	const PxVec3 p2b = triangle.verts[2-culled] + params->mExtrusionDir;

	const float extrusionSign = denormalizedNormal.dot(params->mExtrusionDir);

	const PxVec3 p2b_p1b = p2b - p1b;
	const PxVec3 p0b_p1b = p0b - p1b;
	const PxVec3 p2b_p2 = 2.0f * params->mExtrusionDir;
	const PxVec3 p1_p1b = -p2b_p2;

	const PxVec3 N1 = p2b_p1b.cross(p0b_p1b);
	const float dp0 = Set2(p0b, N1, params->mCapsuleCenter);

	const PxVec3 N2 = (p2 - p1).cross(p0 - p1);
	const float dp1 = -Set2(p0, N2, params->mCapsuleCenter);

	bool directHit;
	if(extrusionSign >= 0.0f)
		directHit = testTri(params, p0b, p1b, p2b, N1, unitDir, capsuleRadius, dpc0, curT, status);
	else
		directHit = testTri(params, p0, p1, p2, N2, unitDir, capsuleRadius, dpc0, curT, status);

	const PxVec3 N3 = p2b_p1b.cross(p1_p1b);
	const float dp2 = -Set2(p1, N3, params->mCapsuleCenter);
	if(!directHit)
	{
		const float dp = N3.dot(unitDir);
		if(dp*extrusionSign>=0.0f)
			testQuad(params, p1, p1b, p2, p2b, N3, unitDir, capsuleRadius, dpc0, curT, status);
	}

	const PxVec3 N5 = p2b_p2.cross(p0 - p2);
	const float dp3 = -Set2(p0, N5, params->mCapsuleCenter);
	if(!directHit)
	{
		const float dp = N5.dot(unitDir);
		if(dp*extrusionSign>=0.0f)
			testQuad(params, p2, p2b, p0, p0b, N5, unitDir, capsuleRadius, dpc0, curT, status);
	}

	const PxVec3 N7 = p1_p1b.cross(p0b_p1b);
	const float dp4 = -Set2(p0b, N7, params->mCapsuleCenter);
	if(!directHit)
	{
		const float dp = N7.dot(unitDir);
		if(dp*extrusionSign>=0.0f)
			testQuad(params, p0, p0b, p1, p1b, N7, unitDir, capsuleRadius, dpc0, curT, status);
	}

	if(1)
	{
		bool originInside = true;
		if(extrusionSign<0.0f)
		{
			if(dp0<0.0f || dp1<0.0f || dp2<0.0f || dp3<0.0f || dp4<0.0f)
				originInside = false;
		}
		else
		{
			if(dp0>0.0f || dp1>0.0f || dp2>0.0f || dp3>0.0f || dp4>0.0f)
				originInside = false;
		}
		if(originInside)
		{
			t = 0.0f;
			return true;
		}
	}

	if(!status)
		return false;	// We didn't touch any triangle

	t = curT;

	return true;
}

// PT: TODO: __fastcall removed to make it compile everywhere. Revisit.
static bool /*__fastcall*/ triCapsuleSweep(CapsuleSweepParams* PX_RESTRICT params, PxU32 primIndex, bool nodeSorting=true)
{
	PxU32 VRef0, VRef1, VRef2;
	getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

	const PxVec3& p0 = params->mVerts[VRef0];
	const PxVec3& p1 = params->mVerts[VRef1];
	const PxVec3& p2 = params->mVerts[VRef2];

	const PxTriangle Tri(p0, p1, p2);	// PT: TODO: check calls to empty ctor/dtor here (TA34704)

	const bool isDoubleSided = params->mBackfaceCulling==0;

	float Dist;
	PxVec3 denormalizedNormal;
	if(sweepCapsuleVsTriangle(params, Tri, Dist, isDoubleSided, denormalizedNormal))
	{
		const PxReal distEpsilon = GU_EPSILON_SAME_DISTANCE; // pick a farther hit within distEpsilon that is more opposing than the previous closest hit
		const PxReal alignmentValue = computeAlignmentValue(denormalizedNormal, params->mLocalDir_Padded);

		if(keepTriangle(Dist, alignmentValue, params->mBestDistance, params->mBestAlignmentValue, params->mMaxDist, distEpsilon))		
		{
			params->mStabbedFace.mDistance = Dist;
			params->mStabbedFace.mTriangleID = primIndex;

			params->mP0 = p0;
			params->mP1 = p1;
			params->mP2 = p2;

			params->mBestDistance = PxMin(params->mBestDistance, Dist); // exact lower bound
			params->mBestAlignmentValue = alignmentValue;
			params->mBestTriNormal = denormalizedNormal;

			if(nodeSorting)
			{
#ifdef SWEEP_AABB_IMPL
	#ifndef GU_BV4_USE_SLABS
				setupRayData(params, Dist, params->mOrigin_Padded, params->mLocalDir_PaddedAligned);
	#endif
#else
				params->ShrinkOBB(Dist);
#endif
			}
			return true;
		}
	}
	return false;
}

#include "GuDistanceSegmentTriangleSIMD.h"

namespace
{
class LeafFunction_CapsuleSweepClosest
{
public:
	static PX_FORCE_INLINE void doLeafTest(CapsuleSweepParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			triCapsuleSweep(params, primIndex);
			primIndex++;
		}while(nbToGo--);
	}
};

class LeafFunction_CapsuleSweepAny
{
public:

	static PX_FORCE_INLINE Ps::IntBool doLeafTest(CapsuleSweepParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(triCapsuleSweep(params, primIndex))
				return 1;
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};

class ImpactFunctionCapsule
{
public:
	static PX_FORCE_INLINE void computeImpact(PxVec3& impactPos, PxVec3& impactNormal, const Capsule& capsule, const PxVec3& dir, const PxReal t, const TrianglePadded& triangle)
	{
		const PxVec3 delta = dir * t;
		const Vec3p P0 = capsule.p0 + delta;
		const Vec3p P1 = capsule.p1 + delta;
		Vec3V pointOnSeg, pointOnTri;
		distanceSegmentTriangleSquared(
			// PT: we use Vec3p so it is safe to V4LoadU P0 and P1
			V3LoadU_SafeReadW(P0), V3LoadU_SafeReadW(P1),
			// PT: we use TrianglePadded so it is safe to V4LoadU the triangle vertices
			V3LoadU_SafeReadW(triangle.verts[0]), V3LoadU_SafeReadW(triangle.verts[1]), V3LoadU_SafeReadW(triangle.verts[2]),
			pointOnSeg, pointOnTri);

		PxVec3 localImpactPos, tmp;
		V3StoreU(pointOnTri, localImpactPos);
		V3StoreU(pointOnSeg, tmp);

		// PT: TODO: refactor with computeSphereTriImpactData (TA34704)
		PxVec3 localImpactNormal = tmp - localImpactPos;
		const float M = localImpactNormal.magnitude();
		if(M<1e-3f)
		{
			localImpactNormal = (triangle.verts[0] - triangle.verts[1]).cross(triangle.verts[0] - triangle.verts[2]);
			localImpactNormal.normalize();
		}
		else
			localImpactNormal /= M;

		impactPos = localImpactPos;
		impactNormal = localImpactNormal;
	}
};
}

static void computeBoxAroundCapsule(const Capsule& capsule, Box& box, PxVec3& extrusionDir)
{
	// Box center = center of the two capsule's endpoints
	box.center = capsule.computeCenter();

	extrusionDir = (capsule.p0 - capsule.p1)*0.5f;
	const PxF32 d = extrusionDir.magnitude();

	// Box extents
	box.extents.x = capsule.radius + d;
	box.extents.y = capsule.radius;
	box.extents.z = capsule.radius;

	// Box orientation
	if(d==0.0f)
	{
		box.rot = PxMat33(PxIdentity);
	}
	else
	{
		PxVec3 dir, right, up;
		Ps::computeBasis(capsule.p0, capsule.p1, dir, right, up);
		box.setAxes(dir, right, up);
	}
}

template<class ParamsT>
static PX_FORCE_INLINE void setupCapsuleParams(ParamsT* PX_RESTRICT params, const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree* PX_RESTRICT tree, const SourceMesh* PX_RESTRICT mesh, PxU32 flags)
{
	params->mStabbedFace.mTriangleID = PX_INVALID_U32;
	params->mBestAlignmentValue = 2.0f;
	params->mBestDistance = maxDist + GU_EPSILON_SAME_DISTANCE;
	params->mMaxDist = maxDist;

	setupParamsFlags(params, flags);

	setupMeshPointersAndQuantizedCoeffs(params, mesh, tree);

	params->mLocalCapsule = capsule;

	Box localBox;
	computeBoxAroundCapsule(capsule, localBox, params->mExtrusionDir);

	params->mCapsuleCenter = localBox.center;

	const PxVec3& localDir = dir;

#ifdef SWEEP_AABB_IMPL
	const PxVec3& localP0 = params->mLocalCapsule.p0;
	const PxVec3& localP1 = params->mLocalCapsule.p1;
	const PxVec3 sweepOrigin = (localP0+localP1)*0.5f;
	const PxVec3 sweepExtents = PxVec3(params->mLocalCapsule.radius) + (localP0-localP1).abs()*0.5f;

	#ifndef GU_BV4_USE_SLABS
	params->mLocalDir_PaddedAligned = localDir;
	#endif
	params->mOrigin_Padded = sweepOrigin;

	const Box aabb(sweepOrigin, sweepExtents, PxMat33(PxIdentity));
	prepareSweepData(aabb, localDir, maxDist, params);	// PT: TODO: optimize this call for idt rotation (TA34704)

	#ifndef GU_BV4_USE_SLABS
	setupRayData(params, maxDist, sweepOrigin, localDir);
	#endif
#else
	prepareSweepData(localBox, localDir, maxDist, params);
#endif
}

#endif // GU_BV4_CAPSULE_SWEEP_INTERNAL_H
