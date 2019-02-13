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

#ifndef GU_SWEEP_MESH_H
#define GU_SWEEP_MESH_H

#include "GuMidphaseInterface.h"
#include "GuVecConvexHull.h"

namespace physx
{

namespace Gu
{
	// PT: class to make sure we can safely V4Load Matrix34's last column
	class Matrix34Padded : public Cm::Matrix34
	{
		public:
			PX_FORCE_INLINE	Matrix34Padded(const Matrix34& src) : Matrix34(src)	{}
			PX_FORCE_INLINE	Matrix34Padded()									{}
			PX_FORCE_INLINE	~Matrix34Padded()									{}
			PxU32	padding;
	};
	PX_COMPILE_TIME_ASSERT(0==(sizeof(Matrix34Padded)==16));

	// PT: intermediate class containing shared bits of code & members
	struct SweepShapeMeshHitCallback : MeshHitCallback<PxRaycastHit>
	{
							SweepShapeMeshHitCallback(CallbackMode::Enum mode, const PxHitFlags& hitFlags, bool flipNormal, float distCoef);

		const PxHitFlags	mHitFlags;
		bool				mStatus;			// Default is false, set to true if a valid hit is found. Stays true once true.
		bool				mInitialOverlap;	// Default is false, set to true if an initial overlap hit is found. Reset for each hit.
		bool				mFlipNormal;		// If negative scale is used we need to flip normal
		PxReal				mDistCoeff;			// dist coeff from unscaled to scaled distance

		void operator=(const SweepShapeMeshHitCallback&) {}
	};

	struct SweepCapsuleMeshHitCallback : SweepShapeMeshHitCallback
	{		
		PxSweepHit&			mSweepHit;
		const Cm::Matrix34&	mVertexToWorldSkew;		
		const PxReal		mTrueSweepDistance;		// max sweep distance that can be used
		PxReal				mBestAlignmentValue;	// best alignment value for triangle normal
		PxReal				mBestDist;				// best distance, not the same as sweepHit.distance, can be shorter by epsilon
		const Capsule&		mCapsule;
		const PxVec3&		mUnitDir;
		const bool			mMeshDoubleSided;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED
		const bool			mIsSphere;

		SweepCapsuleMeshHitCallback(PxSweepHit& sweepHit, const Cm::Matrix34& worldMatrix, PxReal distance, bool meshDoubleSided,
									const Capsule& capsule, const PxVec3& unitDir, const PxHitFlags& hitFlags, bool flipNormal, float distCoef);

		virtual PxAgain processHit(const PxRaycastHit& aHit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal& shrunkMaxT, const PxU32*);

		// PT: TODO: unify these operators
		void operator=(const SweepCapsuleMeshHitCallback&) {}

		bool finalizeHit(	PxSweepHit& sweepHit, const Capsule& lss, const PxTriangleMeshGeometry& triMeshGeom,
							const PxTransform& pose, bool isDoubleSided) const;
	};

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	struct SweepBoxMeshHitCallback : SweepShapeMeshHitCallback
	{		
		const Matrix34Padded&		mMeshToBox;
		PxReal						mDist, mDist0;
		physx::shdfnd::aos::FloatV	mDistV;
		const Box&					mBox;
		const PxVec3&				mLocalDir;
		const PxVec3&				mWorldUnitDir;
		PxReal						mInflation;
		PxTriangle					mHitTriangle;
		physx::shdfnd::aos::Vec3V	mMinClosestA;
		physx::shdfnd::aos::Vec3V	mMinNormal;
		physx::shdfnd::aos::Vec3V	mLocalMotionV;
		PxU32						mMinTriangleIndex;
		PxVec3						mOneOverDir;
		const bool					mBothTriangleSidesCollide;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED || PxHitFlag::eMESH_BOTH_SIDES

		SweepBoxMeshHitCallback(CallbackMode::Enum mode_, const Matrix34Padded& meshToBox, PxReal distance, bool bothTriangleSidesCollide, 
								const Box& box, const PxVec3& localMotion, const PxVec3& localDir, const PxVec3& unitDir,
								const PxHitFlags& hitFlags, const PxReal inflation, bool flipNormal, float distCoef);

		virtual ~SweepBoxMeshHitCallback() {}

		virtual PxAgain processHit(const PxRaycastHit& meshHit, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2, PxReal& shrinkMaxT, const PxU32*);

		bool	finalizeHit(	PxSweepHit& sweepHit, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
								const PxTransform& boxTransform, const PxVec3& localDir,
								bool meshBothSides, bool isDoubleSided)	const;

	private:
		SweepBoxMeshHitCallback& operator=(const SweepBoxMeshHitCallback&);
	};

	struct SweepConvexMeshHitCallback : SweepShapeMeshHitCallback
	{
		PxTriangle							mHitTriangle;
		ConvexHullV							mConvexHull;
		physx::shdfnd::aos::PsMatTransformV	mMeshToConvex;
		physx::shdfnd::aos::PsTransformV	mConvexPoseV;
		const Cm::FastVertex2ShapeScaling&	mMeshScale;
		PxSweepHit							mSweepHit; // stores either the closest or any hit depending on value of mAnyHit
		physx::shdfnd::aos::FloatV			mInitialDistance;
		physx::shdfnd::aos::Vec3V			mConvexSpaceDir; // convexPose.rotateInv(-unit*distance)
		PxVec3								mUnitDir;
		PxVec3								mMeshSpaceUnitDir;
		PxReal								mInflation;
		const bool							mAnyHit;
		const bool							mBothTriangleSidesCollide;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED || PxHitFlag::eMESH_BOTH_SIDES

		SweepConvexMeshHitCallback(	const ConvexHullData& hull, const PxMeshScale& convexScale, const Cm::FastVertex2ShapeScaling& meshScale,
									const PxTransform& convexPose, const PxTransform& meshPose,
									const PxVec3& unitDir, const PxReal distance, PxHitFlags hitFlags, const bool bothTriangleSidesCollide, const PxReal inflation,
									const bool anyHit, float distCoef);

		virtual ~SweepConvexMeshHitCallback()	{}

		virtual PxAgain processHit(const PxRaycastHit& hit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal& shrunkMaxT, const PxU32*);

		bool	finalizeHit(PxSweepHit& sweepHit, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
							const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							const PxVec3& unitDir, PxReal inflation,
							bool isMtd, bool meshBothSides, bool isDoubleSided, bool bothTriangleSidesCollide);

	private:
		SweepConvexMeshHitCallback& operator=(const SweepConvexMeshHitCallback&);
	};

#if PX_VC 
     #pragma warning(pop) 
#endif

}
}

#endif
