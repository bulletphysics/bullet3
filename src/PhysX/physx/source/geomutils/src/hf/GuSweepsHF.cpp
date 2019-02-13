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
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "GuVecCapsule.h"
#include "GuSweepMTD.h"
#include "GuSweepTriangleUtils.h"
#include "GuVecBox.h"
#include "CmScaling.h"
#include "GuSweepCapsuleTriangle.h"
#include "GuInternal.h"
#include "GuGJKRaycast.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace physx::shdfnd::aos;

#include "GuSweepConvexTri.h"

#define AbortTraversal		false
#define ContinueTraversal	true

///////////////////////////////////////////////////////////////////////////////

class HeightFieldTraceSegmentSweepHelper
{
	PX_NOCOPY(HeightFieldTraceSegmentSweepHelper)
public:
	HeightFieldTraceSegmentSweepHelper(const HeightFieldTraceUtil& hfUtil, const PxVec3& aabbExtentHfLocalSpace)
		: mHfUtil(hfUtil), mOverlapObjectExtent(aabbExtentHfLocalSpace)
	{
		mHfUtil.computeLocalBounds(mLocalBounds);
		// extend the bounds
		mLocalBounds.minimum = mLocalBounds.minimum - aabbExtentHfLocalSpace;
		mLocalBounds.maximum = mLocalBounds.maximum + aabbExtentHfLocalSpace;
	}

	template<class T>
	PX_INLINE void traceSegment(const PxVec3& aP0, const PxVec3& rayDirNorm, const float rayLength, T* aCallback) const
	{
		mHfUtil.traceSegment<T, false, true>(aP0, rayDirNorm, rayLength, aCallback, mLocalBounds, false, &mOverlapObjectExtent);
	}

private:
	const HeightFieldTraceUtil&	mHfUtil;
	const PxVec3&				mOverlapObjectExtent;
	PxBounds3					mLocalBounds;
};

///////////////////////////////////////////////////////////////////////////////

class HeightFieldTraceSegmentReport : public EntityReport<PxU32>
{
	PX_NOCOPY(HeightFieldTraceSegmentReport)
public:

	HeightFieldTraceSegmentReport(const HeightFieldUtil& hfUtil, const PxHitFlags hitFlags) :
		mHfUtil			(hfUtil),
		mHitFlags		(hitFlags),
		mStatus			(false),
		mInitialOverlap	(false),
		mIsDoubleSided	((hfUtil.getHeightFieldGeometry().heightFieldFlags & PxMeshGeometryFlag::eDOUBLE_SIDED) || (hitFlags & PxHitFlag::eMESH_BOTH_SIDES)),
		mIsAnyHit		(hitFlags & PxHitFlag::eMESH_ANY)
	{
	}

	bool underFaceHit(const Gu::HeightFieldUtil&, const PxVec3&, const PxVec3&, PxF32, PxF32, PxF32, PxU32)
	{
		return true;
	}

	bool faceHit(const Gu::HeightFieldUtil&, const PxVec3&, PxU32, PxReal, PxReal)
	{
		return true;
	}

	protected:
	const HeightFieldUtil&	mHfUtil;
	const PxHitFlags		mHitFlags;
	bool					mStatus;
	bool					mInitialOverlap;
	const bool				mIsDoubleSided;
	const bool				mIsAnyHit;
};

///////////////////////////////////////////////////////////////////////////////

class CapsuleTraceSegmentReport : public HeightFieldTraceSegmentReport
{
	PX_NOCOPY(CapsuleTraceSegmentReport)
public:
	CapsuleTraceSegmentReport(	const HeightFieldUtil& hfUtil, const PxHitFlags hitFlags,
								const Capsule& inflatedCapsule,
								const PxVec3& unitDir, PxSweepHit& sweepHit, const PxTransform& pose, PxReal distance) :
		HeightFieldTraceSegmentReport	(hfUtil, hitFlags),
		mInflatedCapsule				(inflatedCapsule),
		mUnitDir						(unitDir),
		mSweepHit						(sweepHit),
		mPose							(pose),
		mDistance						(distance)
	{
		mSweepHit.faceIndex = 0xFFFFffff;
	}

	virtual PxAgain onEvent(PxU32 nb, PxU32* indices)
	{
		PX_ALIGN_PREFIX(16) PxU8 tribuf[HF_SWEEP_REPORT_BUFFER_SIZE*sizeof(PxTriangle)] PX_ALIGN_SUFFIX(16);
		PxTriangle* tmpT = reinterpret_cast<PxTriangle*>(tribuf);
		PX_ASSERT(nb <= HF_SWEEP_REPORT_BUFFER_SIZE);
		for(PxU32 i=0; i<nb; i++)
		{
			const PxU32 triangleIndex = indices[i];
			mHfUtil.getTriangle(mPose, tmpT[i], NULL, NULL, triangleIndex, true);
		}

		PxSweepHit h;	// PT: TODO: ctor!
		// PT: this one is safe because cullbox is NULL (no need to allocate one more triangle)
		// PT: TODO: is it ok to pass the initial distance here?
		PxVec3 bestNormal;
		const bool status = sweepCapsuleTriangles_Precise(nb, tmpT, mInflatedCapsule, mUnitDir, mDistance, NULL, h, bestNormal, mHitFlags, mIsDoubleSided);
		if(status && (h.distance <= mSweepHit.distance))
		{
			mSweepHit.faceIndex	= indices[h.faceIndex];
			mSweepHit.normal	= h.normal;
			mSweepHit.position	= h.position;
			mSweepHit.distance	= h.distance;

			mStatus = true;
			if(h.distance == 0.0f)
			{
				mInitialOverlap = true;
				return AbortTraversal;
			}

			if(mIsAnyHit)
				return AbortTraversal;
		}
		return ContinueTraversal;
	}

	bool finalizeHit(PxSweepHit& sweepHit, const PxHeightFieldGeometry& hfGeom, const PxTransform& pose, const Capsule& lss, const Capsule& inflatedCapsule, const PxVec3& unitDir)
	{
		if(!mStatus)
			return false;

		if(mInitialOverlap)
		{
			// PT: TODO: consider using 'setInitialOverlapResults' here
			sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;

			if(mHitFlags & PxHitFlag::eMTD)
			{
				const Vec3V p0 = V3LoadU(lss.p0);
				const Vec3V p1 = V3LoadU(lss.p1);
				const FloatV radius = FLoad(lss.radius);
				CapsuleV capsuleV;
				capsuleV.initialize(p0, p1, radius);

				//calculate MTD
				const bool hasContacts = computeCapsule_HeightFieldMTD(hfGeom, pose, capsuleV, inflatedCapsule.radius, mIsDoubleSided, GuHfQueryFlags::eWORLD_SPACE, sweepHit);

				//ML: the center of mass is below the surface, we won't have MTD contact generate
				if(!hasContacts)
				{
					sweepHit.distance	= 0.0f;
					sweepHit.normal		= -unitDir;
				}
				else
				{
					sweepHit.flags |= PxHitFlag::ePOSITION;
				}
			}
			else
			{
				sweepHit.distance = 0.0f;
				sweepHit.normal = -unitDir;
			}
		}
		else
		{
			sweepHit.flags = PxHitFlag::eNORMAL| PxHitFlag::ePOSITION | PxHitFlag::eFACE_INDEX;
		}
		return true;
	}

	private:
	const Capsule&		mInflatedCapsule;
	const PxVec3&		mUnitDir;
	PxSweepHit&			mSweepHit;
	const PxTransform&	mPose;
	const PxReal		mDistance;
};

bool sweepCapsule_HeightFieldGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);

	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const Capsule inflatedCapsule(lss.p0, lss.p1, lss.radius + inflation);

	// Compute swept box
	Box capsuleBox;
	computeBoxAroundCapsule(inflatedCapsule, capsuleBox);

	const PxVec3 capsuleAABBExtents = capsuleBox.computeAABBExtent();

	const HeightFieldTraceUtil hfUtil(hfGeom);
    CapsuleTraceSegmentReport myReport(hfUtil, hitFlags, inflatedCapsule, unitDir, sweepHit, pose, distance);

	sweepHit.distance = PX_MAX_F32;

	// need hf local space stuff	
	const PxTransform inversePose = pose.getInverse();
	const PxVec3 centerLocalSpace = inversePose.transform(capsuleBox.center);
	const PxVec3 sweepDirLocalSpace = inversePose.rotate(unitDir);
	const PxVec3 capsuleAABBBExtentHfLocalSpace = PxBounds3::basisExtent(centerLocalSpace, PxMat33Padded(inversePose.q), capsuleAABBExtents).getExtents();

	HeightFieldTraceSegmentSweepHelper traceSegmentHelper(hfUtil, capsuleAABBBExtentHfLocalSpace);
	traceSegmentHelper.traceSegment<CapsuleTraceSegmentReport>(centerLocalSpace, sweepDirLocalSpace, distance, &myReport);

	return myReport.finalizeHit(sweepHit, hfGeom, pose, lss, inflatedCapsule, unitDir);
}

///////////////////////////////////////////////////////////////////////////////

class ConvexTraceSegmentReport : public HeightFieldTraceSegmentReport
{
	PX_NOCOPY(ConvexTraceSegmentReport)
public:
	ConvexTraceSegmentReport(	const HeightFieldUtil& hfUtil, const ConvexHullData& hull, const PxMeshScale& convexScale,
								const PxTransform& convexPose, const PxTransform& heightFieldPose,
								const PxVec3& unitDir, PxReal distance, PxHitFlags hitFlags, PxReal inflation) :
			HeightFieldTraceSegmentReport	(hfUtil, hitFlags),
			mUnitDir						(unitDir),
			mInflation						(inflation)
	{
		using namespace Ps::aos;
		mSweepHit.faceIndex = 0xFFFFffff;
		mSweepHit.distance = distance;
		const Vec3V worldDir = V3LoadU(unitDir);
		const FloatV dist = FLoad(distance);
		const QuatV q0 = QuatVLoadU(&heightFieldPose.q.x);
		const Vec3V p0 = V3LoadU(&heightFieldPose.p.x);

		const QuatV q1 = QuatVLoadU(&convexPose.q.x);
		const Vec3V p1 = V3LoadU(&convexPose.p.x);

		const PsTransformV meshTransf(p0, q0);
		const PsTransformV convexTransf(p1, q1);

		mMeshToConvex = convexTransf.transformInv(meshTransf);
		mConvexPoseV = convexTransf;
		mConvexSpaceDir = convexTransf.rotateInv(V3Neg(V3Scale(worldDir, dist)));
		mDistance = dist;

		const Vec3V vScale = V3LoadU_SafeReadW(convexScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat = QuatVLoadU(&convexScale.rotation.x);

		mMeshSpaceUnitDir = heightFieldPose.rotateInv(unitDir);
		mConvexHull.initialize(&hull, V3Zero(), vScale, vQuat, convexScale.isIdentity());
	}

	virtual PxAgain onEvent(PxU32 nbEntities, PxU32* entities)
	{
		const PxTransform idt(PxIdentity);
		for(PxU32 i=0; i<nbEntities; i++)
		{
			PxTriangle tri;
			mHfUtil.getTriangle(idt, tri, NULL, NULL, entities[i], false, false);  // First parameter not needed if local space triangle is enough

			// use mSweepHit.distance as max sweep distance so far, mSweepHit.distance will be clipped by this function
			if(sweepConvexVsTriangle(	tri.verts[0], tri.verts[1], tri.verts[2], mConvexHull, mMeshToConvex, mConvexPoseV,
										mConvexSpaceDir, mUnitDir, mMeshSpaceUnitDir, mDistance, mSweepHit.distance, mSweepHit, mIsDoubleSided,
										mInflation, mInitialOverlap, entities[i]))
			{
				mStatus = true;
				if(mIsAnyHit || mSweepHit.distance == 0.0f)
					return AbortTraversal;
			}
		}
		return ContinueTraversal;
	}

	bool finalizeHit(PxSweepHit& sweepHit,
		const PxHeightFieldGeometry& hfGeom, const PxTransform& pose,
		const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
		const PxVec3& unitDir, PxReal inflation)
	{
		if(!mStatus)
			return false;

		if(mInitialOverlap)
		{
			if(mHitFlags & PxHitFlag::eMTD)
			{
				const bool hasContacts = computeConvex_HeightFieldMTD(hfGeom,  pose, convexGeom, convexPose, inflation, mIsDoubleSided, GuHfQueryFlags::eWORLD_SPACE, sweepHit);

				sweepHit.faceIndex = mSweepHit.faceIndex;
				sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
				if(!hasContacts)
				{
					sweepHit.distance	= 0.0f;
					sweepHit.normal		= -unitDir;
				}
				else
				{
					sweepHit.flags |= PxHitFlag::ePOSITION;
				}
			}
			else
			{
				setInitialOverlapResults(sweepHit, unitDir, mSweepHit.faceIndex);	// hit index must be set to closest for IO
			}
		}
		else
		{
			sweepHit = mSweepHit;
			sweepHit.normal = -sweepHit.normal;
			sweepHit.normal.normalize();
		}
		return true;
	}

	private:
	PsMatTransformV		mMeshToConvex;
	PsTransformV		mConvexPoseV;
	ConvexHullV			mConvexHull;
	PxSweepHit			mSweepHit;
	Vec3V				mConvexSpaceDir;
	FloatV				mDistance;
	const PxVec3		mUnitDir;
	PxVec3				mMeshSpaceUnitDir;
	const PxReal		mInflation;
};

bool sweepConvex_HeightFieldGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const Matrix34 convexTM(convexPose);
	const Matrix34 meshTM(pose);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	const bool idtScaleConvex = convexGeom.scale.isIdentity();

	FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
		convexScaling.init(convexGeom.scale);

	PX_ASSERT(!convexMesh->getLocalBoundsFast().isEmpty());
	const PxBounds3 hullAABBLocalSpace = convexMesh->getLocalBoundsFast().transformFast(convexScaling.getVertex2ShapeSkew());

	const HeightFieldTraceUtil hfUtil(hfGeom);
	ConvexTraceSegmentReport entityReport(
		hfUtil, convexMesh->getHull(), convexGeom.scale, convexPose, pose, -unitDir, distance, hitFlags, inflation);

	// need hf local space stuff	
	const PxBounds3 hullAABB = PxBounds3::transformFast(convexPose, hullAABBLocalSpace);
	const PxVec3 aabbExtents = hullAABB.getExtents() + PxVec3(inflation);
	const PxTransform inversePose = pose.getInverse();
	const PxVec3 centerLocalSpace = inversePose.transform(hullAABB.getCenter());
	const PxVec3 sweepDirLocalSpace = inversePose.rotate(unitDir);
	const PxVec3 convexAABBExtentHfLocalSpace = PxBounds3::basisExtent(centerLocalSpace, PxMat33Padded(inversePose.q), aabbExtents).getExtents();	

	HeightFieldTraceSegmentSweepHelper traceSegmentHelper(hfUtil, convexAABBExtentHfLocalSpace);
	traceSegmentHelper.traceSegment<ConvexTraceSegmentReport>(centerLocalSpace, sweepDirLocalSpace, distance, &entityReport);

	return entityReport.finalizeHit(sweepHit, hfGeom, pose, convexGeom, convexPose, unitDir, inflation);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class BoxTraceSegmentReport : public HeightFieldTraceSegmentReport
{
	PX_NOCOPY(BoxTraceSegmentReport)
public:
	BoxTraceSegmentReport(	const HeightFieldUtil& hfUtil, const PxHitFlags hitFlags,
							const PsTransformV& worldToBoxV, const PxTransform& pose, const BoxV& box, const PxVec3& localMotion,
							PxSweepHit& sweepHit, PxReal inflation) :
		HeightFieldTraceSegmentReport	(hfUtil, hitFlags),
		mWorldToBoxV					(worldToBoxV),
		mPose							(pose),
		mBox							(box),
		mLocalMotion					(localMotion),
		mSweepHit						(sweepHit),
		mInflation						(inflation)
	{
		mMinToi = FMax();
		mSweepHit.faceIndex = 0xFFFFffff;
	}

	virtual PxAgain onEvent(PxU32 nb, PxU32* indices)
	{
		const FloatV zero = FZero();
		const Vec3V zeroV = V3Zero();
		const Vec3V dir = V3LoadU(mLocalMotion);
		//FloatV minToi = FMax();
		FloatV toi;
		Vec3V closestA, normal;//closestA and normal is in the local space of box

		for(PxU32 i=0; i<nb; i++)
		{
			const PxU32 triangleIndex = indices[i];

			PxTriangle currentTriangle;	// in world space
			mHfUtil.getTriangle(mPose, currentTriangle, NULL, NULL, triangleIndex, true, true);

			const Vec3V localV0 = V3LoadU(currentTriangle.verts[0]);
			const Vec3V localV1 = V3LoadU(currentTriangle.verts[1]);
			const Vec3V localV2 = V3LoadU(currentTriangle.verts[2]);

			const Vec3V triV0 = mWorldToBoxV.transform(localV0);
			const Vec3V triV1 = mWorldToBoxV.transform(localV1);
			const Vec3V triV2 = mWorldToBoxV.transform(localV2);

			if(!mIsDoubleSided)
			{
				const Vec3V triNormal = V3Cross(V3Sub(triV2, triV1),V3Sub(triV0, triV1)); 
				if(FAllGrtrOrEq(V3Dot(triNormal, dir), zero))
					continue;
			}

			const TriangleV triangle(triV0, triV1, triV2);

			////move triangle to box space
			//const Vec3V localV0 = Vec3V_From_PxVec3(WorldToBox.transform(currentTriangle.verts[0]));
			//const Vec3V localV1 = Vec3V_From_PxVec3(WorldToBox.transform(currentTriangle.verts[1]));
			//const Vec3V localV2 = Vec3V_From_PxVec3(WorldToBox.transform(currentTriangle.verts[2]));

			//TriangleV triangle(localV0, localV1, localV2);
			const LocalConvex<TriangleV> convexA(triangle);
			const LocalConvex<BoxV> convexB(mBox);
			const Vec3V initialSearchDir = V3Sub(triangle.getCenter(), mBox.getCenter());

			if(gjkRaycastPenetration< LocalConvex<TriangleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, mInflation, false))
			{
				mStatus	= true;
				if(FAllGrtr(toi, zero))
				{
					if(FAllGrtr(mMinToi, toi))
					{
						mMinToi = toi;
						FStore(toi, &mSweepHit.distance);
						V3StoreU(normal, mSweepHit.normal);
						V3StoreU(closestA, mSweepHit.position);
						mSweepHit.faceIndex = triangleIndex;

						if(mIsAnyHit)
							return AbortTraversal;
					}
				}
				else
				{
					mSweepHit.distance = 0.0f;
					mSweepHit.faceIndex	= triangleIndex;
					mInitialOverlap = true;
					return AbortTraversal;
				}
			}
		}
		return ContinueTraversal;
	}

	bool finalizeHit(PxSweepHit& sweepHit,
			const PxHeightFieldGeometry& hfGeom, const PxTransform& pose,
			const PxTransform& boxPose_, const Box& box,
			const PxVec3& unitDir, PxReal distance, PxReal inflation)
	{
		if(!mStatus)
			return false;

		if(mInitialOverlap)
		{
			// PT: TODO: consider using 'setInitialOverlapResults' here

			sweepHit.flags = PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;

			if(mHitFlags & PxHitFlag::eMTD)
			{
				const bool hasContacts = computeBox_HeightFieldMTD(hfGeom, pose, box, boxPose_, inflation, mIsDoubleSided, GuHfQueryFlags::eWORLD_SPACE, sweepHit);
				
				//ML: the center of mass is below the surface, we won't have MTD contact generate
				if(!hasContacts)
				{
					sweepHit.distance	= 0.0f;
					sweepHit.normal		= -unitDir;	
				}
				else
				{
					sweepHit.flags |= PxHitFlag::ePOSITION;
				}
			}
			else
			{
				sweepHit.distance	= 0.0f;
				sweepHit.normal		= -unitDir;
			}
		}
		else
		{
			PxVec3 n = sweepHit.normal.getNormalized();
			if((n.dot(mLocalMotion))>0.0f)
				n = -n;

			sweepHit.distance *= distance;  // stored as toi [0,1] during computation -> scale
			sweepHit.normal = boxPose_.rotate(n);
			sweepHit.position = boxPose_.transform(sweepHit.position);
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		}
		return true;
	}

	private:
	const PsTransformV&	mWorldToBoxV;
	const PxTransform&	mPose;
	const BoxV&			mBox;
	FloatV				mMinToi;
	const PxVec3		mLocalMotion;
	PxSweepHit&			mSweepHit;
	const PxReal		mInflation;
};

#if PX_VC 
     #pragma warning(pop) 
#endif

bool sweepBox_HeightFieldGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(boxGeom_);
	PX_UNUSED(hitFlags);

	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const PxVec3 boxAABBExtent = box.computeAABBExtent() + PxVec3(inflation);

	// Move to AABB space
	PX_ALIGN_PREFIX(16) PxTransform WorldToBox PX_ALIGN_SUFFIX(16);
	WorldToBox = boxPose_.getInverse();

	const QuatV q1 = QuatVLoadA(&WorldToBox.q.x);
	const Vec3V p1 = V3LoadA(&WorldToBox.p.x);
	const PsTransformV WorldToBoxV(p1, q1);

	const PxVec3 motion = unitDir * distance;
	const PxVec3 localMotion = WorldToBox.rotate(motion);

	const BoxV boxV(V3Zero(), V3LoadU(box.extents));

	sweepHit.distance = PX_MAX_F32;

	const HeightFieldTraceUtil hfUtil(hfGeom);
	BoxTraceSegmentReport myReport(hfUtil, hitFlags, WorldToBoxV, pose, boxV, localMotion, sweepHit, inflation);

	// need hf local space stuff	
	const PxTransform inversePose = pose.getInverse();
	const PxVec3 centerLocalSpace = inversePose.transform(box.center);
	const PxVec3 sweepDirLocalSpace = inversePose.rotate(unitDir);
	const PxVec3 boxAABBExtentInHfLocalSpace = PxBounds3::basisExtent(centerLocalSpace, PxMat33Padded(inversePose.q), boxAABBExtent).getExtents();

	HeightFieldTraceSegmentSweepHelper traceSegmentHelper(hfUtil, boxAABBExtentInHfLocalSpace);
	traceSegmentHelper.traceSegment<BoxTraceSegmentReport>(centerLocalSpace, sweepDirLocalSpace, distance, &myReport);

	return myReport.finalizeHit(sweepHit, hfGeom, pose, boxPose_, box, unitDir, distance, inflation);
}

///////////////////////////////////////////////////////////////////////////////
