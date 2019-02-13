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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuSweepTests.h"
#include "GuSweepMesh.h"
#include "GuInternal.h"
#include "GuConvexUtilsInternal.h"
#include "CmScaling.h"
#include "GuSweepMTD.h"
#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuSweepBoxTriangle_SAT.h"
#include "GuSweepCapsuleTriangle.h"
#include "GuSweepSphereTriangle.h"
#include "GuDistancePointTriangle.h"
#include "GuCapsule.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace physx::shdfnd::aos;

#include "GuSweepConvexTri.h"

///////////////////////////////////////////////////////////////////////////////

static bool sweepSphereTriangle(const PxTriangle& tri,
								const PxVec3& center, PxReal radius,
								const PxVec3& unitDir, const PxReal distance,
								PxSweepHit& hit, PxVec3& triNormalOut,
								PxHitFlags hitFlags, bool isDoubleSided)
{
	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
	{
		const bool doBackfaceCulling = !isDoubleSided && !meshBothSides;

		// PT: test if shapes initially overlap
		// PT: add culling here for now, but could be made more efficiently...

		// Create triangle normal
		PxVec3 denormalizedNormal;
		tri.denormalizedNormal(denormalizedNormal);

		// Backface culling
		if(doBackfaceCulling && (denormalizedNormal.dot(unitDir) > 0.0f))
			return false;

		float s_unused, t_unused;
		const PxVec3 cp = closestPtPointTriangle(center, tri.verts[0], tri.verts[1], tri.verts[2], s_unused, t_unused);
		const PxReal dist2 = (cp - center).magnitudeSquared();
		if(dist2<=radius*radius)
		{
			triNormalOut = denormalizedNormal.getNormalized();
			return setInitialOverlapResults(hit, unitDir, 0);
		}
	}
	
	return sweepSphereTriangles(1, &tri,
								center, radius,
								unitDir, distance,
								NULL,
								hit, triNormalOut,
								isDoubleSided, meshBothSides, false, false);
}

///////////////////////////////////////////////////////////////////////////////

SweepShapeMeshHitCallback::SweepShapeMeshHitCallback(CallbackMode::Enum mode, const PxHitFlags& hitFlags, bool flipNormal, float distCoef) :
	MeshHitCallback<PxRaycastHit>	(mode),
	mHitFlags						(hitFlags),
	mStatus							(false),
	mInitialOverlap					(false),
	mFlipNormal						(flipNormal),
	mDistCoeff						(distCoef)
{		
}

///////////////////////////////////////////////////////////////////////////////

SweepCapsuleMeshHitCallback::SweepCapsuleMeshHitCallback(
	PxSweepHit& sweepHit, const Matrix34& worldMatrix, PxReal distance, bool meshDoubleSided,
	const Capsule& capsule, const PxVec3& unitDir, const PxHitFlags& hitFlags, bool flipNormal, float distCoef) :
	SweepShapeMeshHitCallback	(CallbackMode::eMULTIPLE, hitFlags, flipNormal, distCoef),
	mSweepHit					(sweepHit),
	mVertexToWorldSkew			(worldMatrix),	
	mTrueSweepDistance			(distance),
	mBestAlignmentValue			(2.0f),
	mBestDist					(distance + GU_EPSILON_SAME_DISTANCE),
	mCapsule					(capsule),
	mUnitDir					(unitDir),
	mMeshDoubleSided			(meshDoubleSided),
	mIsSphere					(capsule.p0 == capsule.p1)
{
	mSweepHit.distance = mTrueSweepDistance;
}

PxAgain SweepCapsuleMeshHitCallback::processHit( // all reported coords are in mesh local space including hit.position
												const PxRaycastHit& aHit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal& shrunkMaxT, const PxU32*)
{
	const PxTriangle tmpt(	mVertexToWorldSkew.transform(v0),
							mVertexToWorldSkew.transform(mFlipNormal ? v2 : v1),
							mVertexToWorldSkew.transform(mFlipNormal ? v1 : v2));

	PxSweepHit localHit;	// PT: TODO: ctor!
	PxVec3 triNormal;
	// pick a farther hit within distEpsilon that is more opposing than the previous closest hit
	// make it a relative epsilon to make sure it still works with large distances
	const PxReal distEpsilon = GU_EPSILON_SAME_DISTANCE * PxMax(1.0f, mSweepHit.distance); 
	const float minD = mSweepHit.distance + distEpsilon;
	if(mIsSphere)
	{
		if(!::sweepSphereTriangle(	tmpt,
									mCapsule.p0, mCapsule.radius,
									mUnitDir, minD,
									localHit, triNormal,
									mHitFlags, mMeshDoubleSided))
			return true;
	}
	else
	{
		// PT: this one is safe because cullbox is NULL (no need to allocate one more triangle)		
		if(!sweepCapsuleTriangles_Precise(	1, &tmpt,
											mCapsule,
											mUnitDir, minD,
											NULL,
											localHit, triNormal,
											mHitFlags, mMeshDoubleSided,
											NULL))
			return true;
	}

	const PxReal alignmentValue = computeAlignmentValue(triNormal, mUnitDir);
//	if(keepTriangle(localHit.distance, alignmentValue, mBestDist, mBestAlignmentValue, mTrueSweepDistance, distEpsilon))	
	if(keepTriangle(localHit.distance, alignmentValue, mBestDist, mBestAlignmentValue, mTrueSweepDistance, GU_EPSILON_SAME_DISTANCE))	
	{
		mBestAlignmentValue = alignmentValue;

		// AP: need to shrink the sweep distance passed into sweepCapsuleTriangles for correctness so that next sweep is closer		
		shrunkMaxT = localHit.distance * mDistCoeff; // shrunkMaxT is scaled

		mBestDist = PxMin(mBestDist, localHit.distance); // exact lower bound
		mSweepHit.flags		= localHit.flags;
		mSweepHit.distance	= localHit.distance;
		mSweepHit.normal	= localHit.normal;
		mSweepHit.position	= localHit.position;
		mSweepHit.faceIndex	= aHit.faceIndex;

		mStatus = true;
		//ML:this is the initial overlap condition
		if(localHit.distance == 0.0f)
		{
			mInitialOverlap = true;
			return false;
		}
		if(mHitFlags & PxHitFlag::eMESH_ANY)
			return false; // abort traversal
	}
	return true;
}

bool SweepCapsuleMeshHitCallback::finalizeHit(	PxSweepHit& sweepHit, const Capsule& lss, const PxTriangleMeshGeometry& triMeshGeom,
												const PxTransform& pose, bool isDoubleSided) const
{
	if(!mStatus)
		return false;

	if(mInitialOverlap)
	{
		// PT: TODO: consider using 'setInitialOverlapResults' here
		bool hasContacts = false;
		if(mHitFlags & PxHitFlag::eMTD)
		{
			const Vec3V p0 = V3LoadU(mCapsule.p0);
			const Vec3V p1 = V3LoadU(mCapsule.p1);
			const FloatV radius = FLoad(lss.radius);
			CapsuleV capsuleV;
			capsuleV.initialize(p0, p1, radius);

			//we need to calculate the MTD
			hasContacts = computeCapsule_TriangleMeshMTD(triMeshGeom, pose, capsuleV, mCapsule.radius, isDoubleSided, sweepHit);
		}
		setupSweepHitForMTD(sweepHit, hasContacts, mUnitDir);
	}
	else
	{
		sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::ePOSITION | PxHitFlag::eFACE_INDEX;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool sweepCapsule_MeshGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);

	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	return Midphase::sweepCapsuleVsMesh(meshData, meshGeom, pose, lss, unitDir, distance, sweepHit, hitFlags, inflation);
}

///////////////////////////////////////////////////////////////////////////////

	// same as 'mat.transform(p)' but using SIMD
	static PX_FORCE_INLINE Vec4V transformV(const Vec4V p, const Matrix34Padded& mat)
	{
		Vec4V ResV = V4Scale(V4LoadU(&mat.m.column0.x), V4GetX(p));
		ResV = V4ScaleAdd(V4LoadU(&mat.m.column1.x), V4GetY(p), ResV);
		ResV = V4ScaleAdd(V4LoadU(&mat.m.column2.x), V4GetZ(p), ResV);
		ResV = V4Add(ResV, V4LoadU(&mat.p.x));	// PT: this load is safe thanks to padding
		return ResV;
	}

///////////////////////////////////////////////////////////////////////////////

SweepBoxMeshHitCallback::SweepBoxMeshHitCallback(	CallbackMode::Enum mode_, const Matrix34Padded& meshToBox, PxReal distance, bool bothTriangleSidesCollide,
													const Box& box, const PxVec3& localMotion, const PxVec3& localDir, const PxVec3& unitDir,
													const PxHitFlags& hitFlags, const PxReal inflation, bool flipNormal, float distCoef) :
	SweepShapeMeshHitCallback	(mode_, hitFlags, flipNormal,distCoef),
	mMeshToBox					(meshToBox),
	mDist						(distance),
	mBox						(box),
	mLocalDir					(localDir),
	mWorldUnitDir				(unitDir),
	mInflation					(inflation),
	mBothTriangleSidesCollide	(bothTriangleSidesCollide)
{
	mLocalMotionV = V3LoadU(localMotion);
	mDistV = FLoad(distance);
	mDist0 = distance;
	mOneOverDir = PxVec3(
		mLocalDir.x!=0.0f ? 1.0f/mLocalDir.x : 0.0f,
		mLocalDir.y!=0.0f ? 1.0f/mLocalDir.y : 0.0f,
		mLocalDir.z!=0.0f ? 1.0f/mLocalDir.z : 0.0f);
}

PxAgain SweepBoxMeshHitCallback::processHit( // all reported coords are in mesh local space including hit.position
											const PxRaycastHit& meshHit, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2, PxReal& shrinkMaxT, const PxU32*)
{
	if(mHitFlags & PxHitFlag::ePRECISE_SWEEP)
	{
		const PxTriangle currentTriangle(
				mMeshToBox.transform(lp0),
				mMeshToBox.transform(mFlipNormal ? lp2 : lp1),
				mMeshToBox.transform(mFlipNormal ? lp1 : lp2));

		PxF32 t = PX_MAX_REAL; // PT: could be better!
		if(!triBoxSweepTestBoxSpace(currentTriangle, mBox.extents, mLocalDir, mOneOverDir, mDist, t, !mBothTriangleSidesCollide))
			return true;

		if(t <= mDist)
		{
			// PT: test if shapes initially overlap
			mDist				= t;
			shrinkMaxT			= t * mDistCoeff; // shrunkMaxT is scaled
			mMinClosestA		= V3LoadU(currentTriangle.verts[0]); // PT: this is arbitrary
			mMinNormal			= V3LoadU(-mWorldUnitDir);
			mStatus				= true;
			mMinTriangleIndex	= meshHit.faceIndex;
			mHitTriangle		= currentTriangle;
			if(t == 0.0f)
			{
				mInitialOverlap = true;
				return false; // abort traversal
			}
		}
	}
	else
	{
		const FloatV zero = FZero();

		// PT: SIMD code similar to:
		//	const Vec3V triV0 = V3LoadU(mMeshToBox.transform(lp0));
		//	const Vec3V triV1 = V3LoadU(mMeshToBox.transform(lp1));
		//	const Vec3V triV2 = V3LoadU(mMeshToBox.transform(lp2));
		//
		// SIMD version works but we need to ensure all loads are safe.
		// For incoming vertices they should either come from the vertex array or from a binary deserialized file.
		// For the vertex array we can just allocate one more vertex. For the binary file it should be ok as soon
		// as vertices aren't the last thing serialized in the file.
		// For the matrix only the last column is a problem, and we can easily solve that with some padding in the local class.
		const Vec3V triV0 = Vec3V_From_Vec4V(transformV(V4LoadU(&lp0.x), mMeshToBox));
		const Vec3V triV1 = Vec3V_From_Vec4V(transformV(V4LoadU(mFlipNormal ? &lp2.x : &lp1.x), mMeshToBox));
		const Vec3V triV2 = Vec3V_From_Vec4V(transformV(V4LoadU(mFlipNormal ? &lp1.x : &lp2.x), mMeshToBox));

		if(!mBothTriangleSidesCollide)
		{
			const Vec3V triNormal = V3Cross(V3Sub(triV2, triV1),V3Sub(triV0, triV1)); 
			if(FAllGrtrOrEq(V3Dot(triNormal, mLocalMotionV), zero))
				return true;
		}

		const Vec3V zeroV = V3Zero();
		const Vec3V boxExtents = V3LoadU(mBox.extents);
		const BoxV boxV(zeroV, boxExtents);

		const TriangleV triangleV(triV0, triV1, triV2);

		FloatV lambda;   
		Vec3V closestA, normal;//closestA and normal is in the local space of convex hull
		LocalConvex<TriangleV> convexA(triangleV);
		LocalConvex<BoxV> convexB(boxV);
		const Vec3V initialSearchDir = V3Sub(triangleV.getCenter(), boxV.getCenter());
		if(!gjkRaycastPenetration< LocalConvex<TriangleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, mLocalMotionV, lambda, normal, closestA, mInflation, false))
			return true;

		mStatus = true;
		mMinClosestA = closestA;
		mMinTriangleIndex = meshHit.faceIndex;
		if(FAllGrtrOrEq(zero, lambda)) // lambda < 0? => initial overlap
		{
			mInitialOverlap = true;
			shrinkMaxT = 0.0f;
			mDistV = zero;
			mDist = 0.0f;
			mMinNormal = V3LoadU(-mWorldUnitDir);
			return false;
		}

		PxF32 f;
		FStore(lambda, &f);
		mDist = f*mDist; // shrink dist
		mLocalMotionV = V3Scale(mLocalMotionV, lambda); // shrink localMotion
		mDistV = FMul(mDistV, lambda); // shrink distV
		mMinNormal = normal;
		if(mDist * mDistCoeff < shrinkMaxT) // shrink shrinkMaxT
			shrinkMaxT = mDist * mDistCoeff; // shrunkMaxT is scaled

		//mHitTriangle = currentTriangle;
		V3StoreU(triV0, mHitTriangle.verts[0]);
		V3StoreU(triV1, mHitTriangle.verts[1]);
		V3StoreU(triV2, mHitTriangle.verts[2]);
	}
	return true;
}

bool SweepBoxMeshHitCallback::finalizeHit(	PxSweepHit& sweepHit, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
											const PxTransform& boxTransform, const PxVec3& localDir,
											bool meshBothSides, bool isDoubleSided) const
{
	if(!mStatus)
		return false;

	Vec3V minClosestA = mMinClosestA;
	Vec3V minNormal = mMinNormal;
	sweepHit.faceIndex = mMinTriangleIndex;

	if(mInitialOverlap)
	{
		bool hasContacts = false;
		if(mHitFlags & PxHitFlag::eMTD)
			hasContacts = computeBox_TriangleMeshMTD(triMeshGeom, pose, mBox, boxTransform, mInflation, mBothTriangleSidesCollide, sweepHit);

		setupSweepHitForMTD(sweepHit, hasContacts, mWorldUnitDir);
	}
	else
	{
		sweepHit.distance = mDist;
		sweepHit.flags = PxHitFlag::eFACE_INDEX;

		// PT: we need the "best triangle" normal in order to call 'shouldFlipNormal'. We stored the best
		// triangle in both GJK & precise codepaths (in box space). We use a dedicated 'shouldFlipNormal'
		// function that delays computing the triangle normal.
		// TODO: would still be more efficient to store the best normal directly, it's already computed at least
		// in the GJK codepath.

		const Vec3V p0 = V3LoadU(&boxTransform.p.x);
		const QuatV q0 = QuatVLoadU(&boxTransform.q.x);
		const PsTransformV boxPos(p0, q0);

		if(mHitFlags & PxHitFlag::ePRECISE_SWEEP)
		{
			computeBoxLocalImpact(sweepHit.position, sweepHit.normal, sweepHit.flags, mBox, localDir, mHitTriangle, mHitFlags, isDoubleSided, meshBothSides, mDist);
		}
		else
		{
			sweepHit.flags |= PxHitFlag::eNORMAL|PxHitFlag::ePOSITION;

			// PT: now for the GJK path, we must first always negate the returned normal. Similar to what happens in the precise path,
			// we can't delay this anymore: our normal must be properly oriented in order to call 'shouldFlipNormal'.
			minNormal = V3Neg(minNormal);

			// PT: this one is to ensure the normal respects the mesh-both-sides/double-sided convention
			PxVec3 tmp;
			V3StoreU(minNormal, tmp);

			if(shouldFlipNormal(tmp, meshBothSides, isDoubleSided, mHitTriangle, localDir, NULL))
				minNormal = V3Neg(minNormal);

			// PT: finally, this moves everything back to world space
			V3StoreU(boxPos.rotate(minNormal), sweepHit.normal);
			V3StoreU(boxPos.transform(minClosestA), sweepHit.position);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool sweepBox_MeshGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);

	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	return Midphase::sweepBoxVsMesh(meshData, meshGeom, pose, box, unitDir, distance, sweepHit, hitFlags, inflation);
}

///////////////////////////////////////////////////////////////////////////////

SweepConvexMeshHitCallback::SweepConvexMeshHitCallback(	const ConvexHullData& hull, const PxMeshScale& convexScale, const FastVertex2ShapeScaling& meshScale,
														const PxTransform& convexPose, const PxTransform& meshPose,
														const PxVec3& unitDir, const PxReal distance, PxHitFlags hitFlags, const bool bothTriangleSidesCollide, const PxReal inflation,
														const bool anyHit, float distCoef) :
	SweepShapeMeshHitCallback	(CallbackMode::eMULTIPLE, hitFlags, meshScale.flipsNormal(), distCoef),
	mMeshScale					(meshScale),
	mUnitDir					(unitDir),
	mInflation					(inflation),
	mAnyHit						(anyHit),
	mBothTriangleSidesCollide	(bothTriangleSidesCollide)
{
	mSweepHit.distance = distance; // this will be shrinking progressively as we sweep and clip the sweep length
	mSweepHit.faceIndex = 0xFFFFFFFF;

	mMeshSpaceUnitDir = meshPose.rotateInv(unitDir);
	
	const Vec3V worldDir = V3LoadU(unitDir);
	const FloatV dist = FLoad(distance);
	const QuatV q0 = QuatVLoadU(&meshPose.q.x);
	const Vec3V p0 = V3LoadU(&meshPose.p.x);

	const QuatV q1 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p1 = V3LoadU(&convexPose.p.x);

	const PsTransformV meshPoseV(p0, q0);
	const PsTransformV convexPoseV(p1, q1);

	mMeshToConvex = convexPoseV.transformInv(meshPoseV);
	mConvexPoseV = convexPoseV;
	mConvexSpaceDir = convexPoseV.rotateInv(V3Neg(V3Scale(worldDir, dist)));
	mInitialDistance = dist;

	const Vec3V vScale = V3LoadU_SafeReadW(convexScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexScale.rotation.x);
	mConvexHull.initialize(&hull, V3Zero(), vScale, vQuat, convexScale.isIdentity());
}

PxAgain SweepConvexMeshHitCallback::processHit( // all reported coords are in mesh local space including hit.position
												const PxRaycastHit& hit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal& shrunkMaxT, const PxU32*)
{
	const PxVec3 v0 = mMeshScale * av0;
	const PxVec3 v1 = mMeshScale * (mFlipNormal ?  av2 : av1);
	const PxVec3 v2 = mMeshScale * (mFlipNormal ?  av1 : av2);

	// mSweepHit will be updated if sweep distance is < input mSweepHit.distance
	const PxReal oldDist = mSweepHit.distance;
	if(sweepConvexVsTriangle(
		v0, v1, v2, mConvexHull, mMeshToConvex, mConvexPoseV, mConvexSpaceDir,
		mUnitDir, mMeshSpaceUnitDir, mInitialDistance, oldDist, mSweepHit, mBothTriangleSidesCollide,
		mInflation, mInitialOverlap, hit.faceIndex))
	{
		mStatus = true;
		shrunkMaxT = mSweepHit.distance * mDistCoeff; // shrunkMaxT is scaled

		// PT: added for 'shouldFlipNormal'
		mHitTriangle.verts[0] = v0;
		mHitTriangle.verts[1] = v1;
		mHitTriangle.verts[2] = v2;

		if(mAnyHit)
			return false; // abort traversal
			
		if(mSweepHit.distance == 0.0f)
			return false;
	}
	return true; // continue traversal
}

bool SweepConvexMeshHitCallback::finalizeHit(	PxSweepHit& sweepHit, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
												const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
												const PxVec3& unitDir, PxReal inflation,
												bool isMtd, bool meshBothSides, bool isDoubleSided, bool bothTriangleSidesCollide)
{
	if(!mStatus)
		return false;

	if(mInitialOverlap)
	{
		bool hasContacts = false;
		if(isMtd)
			hasContacts = computeConvex_TriangleMeshMTD(meshGeom,  pose, convexGeom, convexPose, inflation, bothTriangleSidesCollide, sweepHit);

		setupSweepHitForMTD(sweepHit, hasContacts, unitDir);

		sweepHit.faceIndex = mSweepHit.faceIndex;
	}
	else
	{
		sweepHit = mSweepHit;
		//sweepHit.position += unitDir * sweepHit.distance;
		sweepHit.normal = -sweepHit.normal;
		sweepHit.normal.normalize();

		// PT: this one is to ensure the normal respects the mesh-both-sides/double-sided convention
		// PT: beware, the best triangle is in mesh-space, but the impact data is in world-space already
		if(shouldFlipNormal(sweepHit.normal, meshBothSides, isDoubleSided, mHitTriangle, unitDir, &pose))
			sweepHit.normal = -sweepHit.normal;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

bool sweepConvex_MeshGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	const bool idtScaleMesh = meshGeom.scale.isIdentity();

	FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(meshGeom.scale);

	PX_ASSERT(!convexMesh->getLocalBoundsFast().isEmpty());
	const PxBounds3 hullAABB = convexMesh->getLocalBoundsFast().transformFast(convexScaling.getVertex2ShapeSkew());

	Box hullOBB;
	computeHullOBB(hullOBB, hullAABB, 0.0f, Matrix34(convexPose), Matrix34(pose), meshScaling, idtScaleMesh);

	hullOBB.extents.x += inflation;
	hullOBB.extents.y += inflation;
	hullOBB.extents.z += inflation;

	const PxVec3 localDir = pose.rotateInv(unitDir);

	// inverse transform the sweep direction and distance to mesh space	
	PxVec3 meshSpaceSweepVector = meshScaling.getShape2VertexSkew().transform(localDir*distance);
	const PxReal meshSpaceSweepDist = meshSpaceSweepVector.normalize();

	PxReal distCoeff = 1.0f;
	if (!idtScaleMesh)
		distCoeff = meshSpaceSweepDist / distance;

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool isDoubleSided = meshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED;
	const bool bothTriangleSidesCollide = isDoubleSided || meshBothSides;
	const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;
	SweepConvexMeshHitCallback callback(
		convexMesh->getHull(), convexGeom.scale, meshScaling, convexPose, pose, -unitDir, distance, hitFlags,
		bothTriangleSidesCollide, inflation, anyHit, distCoeff);
	
	Midphase::sweepConvexVsMesh(meshData, hullOBB, meshSpaceSweepVector, meshSpaceSweepDist, callback, anyHit);

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	return callback.finalizeHit(sweepHit, meshGeom, pose, convexGeom, convexPose, unitDir, inflation, isMtd, meshBothSides, isDoubleSided, bothTriangleSidesCollide);
}

///////////////////////////////////////////////////////////////////////////////

