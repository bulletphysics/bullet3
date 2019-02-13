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


#ifndef PXD_ARTICULATION_H
#define PXD_ARTICULATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsVecMath.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "PxArticulationJoint.h"
#include "DyVArticulation.h"

namespace physx
{
	class PxConstraintAllocator;


#define DY_DEBUG_ARTICULATION 0

namespace Dy
{
	struct FsInertia;
	struct FsData;

#if PX_VC 
    #pragma warning(push)   
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

PX_ALIGN_PREFIX(64)

class Articulation : public ArticulationV
{
public:
	// public interface

	Articulation(Sc::ArticulationSim*);
	~Articulation();

	virtual bool resize(const PxU32 linkCount);

	virtual	void onUpdateSolverDesc()
	{
		PxMemZero(mExternalLoads.begin(), sizeof(Ps::aos::Mat33V) * mExternalLoads.size());
		PxMemZero(mInternalLoads.begin(), sizeof(Ps::aos::Mat33V) * mExternalLoads.size());
	}

	FsData*					getFsDataPtr()									const { return reinterpret_cast<FsData *>(mFsDataBytes.begin()); }
	//void					setFsDataPtr(FsData* data) { mFsData = data; }

	// get data sizes for allocation at higher levels
	virtual void		getDataSizes(PxU32 linkCount, 
								 PxU32 &solverDataSize, 
								 PxU32& totalSize, 
								 PxU32& scratchSize);

	virtual	void	getImpulseResponse(
		PxU32 linkID,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse,
		Cm::SpatialVector& deltaV) const;

	virtual	void	getImpulseSelfResponse(
		PxU32 linkID0,
		PxU32 linkID1,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse0,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV0,
		Cm::SpatialVector& deltaV1) const;

	virtual Cm::SpatialVectorV getLinkVelocity(const PxU32 linkID) const;

	virtual Cm::SpatialVectorV getLinkMotionVector(const PxU32 linkID) const;

	//this is called by island gen to determine whether the articulation should be awake or sleep
	virtual Cm::SpatialVector getMotionVelocity(const PxU32 linkID) const;

	virtual PxReal getLinkMaxPenBias(const PxU32 linkID) const;


	static PxU32 computeUnconstrainedVelocities(
		const ArticulationSolverDesc& desc,
		PxReal dt,
		PxConstraintAllocator& allocator,
		PxSolverConstraintDesc* constraintDesc,
		PxU32& acCount,
		const PxVec3& gravity, PxU64 contextID,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

	static void computeUnconstrainedVelocitiesTGS(
		const ArticulationSolverDesc& desc,
		PxReal dt, const PxVec3& gravity,
		PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

	static PxU32 setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
		PxcConstraintBlockStream& stream,
		PxTGSSolverConstraintDesc* constraintDesc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		PxU32& acCount,
		PxsConstraintBlockManager& constraintBlockManager,
		Cm::SpatialVectorF* /*Z*/);

	static void saveVelocity(const ArticulationSolverDesc& d, Cm::SpatialVectorF* deltaV);

	static void saveVelocityTGS(const ArticulationSolverDesc& d, PxReal invDtF32);

	static void updateBodies(const ArticulationSolverDesc& desc, PxReal dt);

	static void recordDeltaMotion(const ArticulationSolverDesc &desc, const PxReal dt, Cm::SpatialVectorF* deltaV);

	static void deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt);

	virtual void pxcFsApplyImpulse(PxU32 linkID, Ps::aos::Vec3V linear, 
		Ps::aos::Vec3V angular, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

	virtual void pxcFsApplyImpulses(PxU32 linkID, const Ps::aos::Vec3V& linear,
		const Ps::aos::Vec3V& angular, PxU32 linkID2, const Ps::aos::Vec3V& linear2,
		const Ps::aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

	virtual void solveInternalConstraints(const PxReal dt, const PxReal invDt, Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV,
		bool velIteration);
	
	virtual Cm::SpatialVectorV pxcFsGetVelocity(PxU32 linkID);
	virtual void pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1);

	virtual Cm::SpatialVectorV pxcFsGetVelocityTGS(PxU32 linkID) { return Articulation::pxcFsGetVelocity(linkID); }

	virtual const PxTransform& getCurrentTransform(PxU32 linkID)const
	{
		return mPose[linkID];
	}

	virtual const PxQuat& getDeltaQ(PxU32 linkID) const
	{
		return mDeltaQ[linkID];
	}

	static void prepareDataBlock(FsData& fsData,
		const ArticulationLink* links,
		PxU16 linkCount,
		PxTransform* poses,
		PxQuat* deltaQ,
		FsInertia* baseInertia,
		ArticulationJointTransforms* jointTransforms,
		PxU32 expectedSize);

	static void prepareFsData(FsData& fsData,
		const ArticulationLink* links);

	static PxReal getResistance(PxReal compliance);

	static PxU32 getFsDataSize(PxU32 linkCount);

	static PxU32 getLtbDataSize(PxU32 linkCount);

	static void setInertia(FsInertia& inertia,
		const PxsBodyCore& body,
		const PxTransform& pose);

	static void setJointTransforms(ArticulationJointTransforms& transforms,
		const PxTransform& parentPose,
		const PxTransform& childPose,
		const ArticulationJointCore& joint);

	static void applyImpulses(const FsData& matrix,
		Cm::SpatialVectorV* Z,
		Cm::SpatialVectorV* V);

private:
	
#if DY_DEBUG_ARTICULATION
	// debug quantities

	Cm::SpatialVector		computeMomentum(const FsInertia *inertia) const;
	void					computeResiduals(const Cm::SpatialVector *, 
											 const ArticulationJointTransforms* jointTransforms,
											 bool dump = false) const;
	void					checkLimits() const;
#endif

	//PX_FORCE_INLINE Cm::SpatialVectorV* getVelocity(FsData& matrix);

	void	computeUnconstrainedVelocitiesInternal(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID,
		FsInertia* PX_RESTRICT baseInertia,
		ArticulationJointTransforms* PX_RESTRICT jointTransforms,
		PxcFsScratchAllocator& allocator);

	void prepareLtbMatrix(FsData& fsData,
		const FsInertia* baseInertia,
		const PxTransform* poses,
		const ArticulationJointTransforms* jointTransforms,
		PxReal recipDt);

	void computeJointDrives(FsData& fsData,
		Ps::aos::Vec3V* drives,
		const ArticulationLink* links,
		const PxTransform* poses,
		const ArticulationJointTransforms* transforms,
		const Ps::aos::Mat33V* loads,
		PxReal dt);

	mutable Ps::Array<char>				mFsDataBytes;			// drive cache creation (which is const) can force a resize

																// persistent state of the articulation for warm-starting joint load computation
	Ps::Array<Ps::aos::Mat33V>			mInternalLoads;
	Ps::Array<Ps::aos::Mat33V>			mExternalLoads;
	Ps::Array<char>						mScratchMemory;			// drive cache creation (which is const) can force a resize
	Ps::Array<PxTransform>				mPose;
	Ps::Array<PxQuat>					mDeltaQ;
	Ps::Array<Cm::SpatialVectorV>		mMotionVelocity;		// saved here in solver to communicate back to island management/sleeping

} PX_ALIGN_SUFFIX(64);

#if PX_VC 
     #pragma warning(pop) 
#endif

class PxvArticulationDriveCache
{
public:
	// drive cache stuff
	static void		initialize(
							   FsData &cache,
							   PxU16 linkCount,
							   const ArticulationLink* links,
							   PxReal compliance,
							   PxU32 iterations,
							   char* scratchMemory,
							   PxU32 scratchMemorySize);

	static PxU32	getLinkCount(const FsData& cache);

	static void		applyImpulses(const FsData& cache,
								  Cm::SpatialVectorV* Z,
								  Cm::SpatialVectorV* V);

	static void		getImpulseResponse(const FsData& cache, 
									   PxU32 linkID, 
									   const Cm::SpatialVectorV& impulse,
									   Cm::SpatialVectorV& deltaV);
};

void PxvRegisterArticulations();

}

}

#endif
