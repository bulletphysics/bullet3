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


#ifndef DY_THREADCONTEXT_H
#define DY_THREADCONTEXT_H

#include "foundation/PxTransform.h"
#include "PxvConfig.h"
#include "CmBitMap.h"
#include "CmMatrix34.h"
#include "PxcThreadCoherentCache.h"
#include "DyThresholdTable.h"
#include "PsAllocator.h"
#include "PsAllocator.h"
#include "GuContactBuffer.h"
#include "DySolverConstraintDesc.h"
#include "PxvDynamics.h"
#include "DyArticulation.h"
#include "DyFrictionPatchStreamPair.h"
#include "PxcConstraintBlockStream.h"
#include "DyCorrelationBuffer.h"

namespace physx
{
struct PxsIndexedContactManager;

namespace Dy
{

	//Needed by all constraints
	struct PxTGSSolverBodyVel
	{
		PX_ALIGN(16, PxVec3) linearVelocity;			//12
		PxU16			nbStaticInteractions;	//14 Used to accumulate the number of static interactions
		PxU16			maxDynamicPartition;	//16 Used to accumualte the max partition of dynamic interactions
		PxVec3			angularVelocity;		//28
		PxU32			partitionMask;			//32 Used in partitioning as a bit-field
		PxVec3			deltaAngDt;				//44
		PxReal			maxAngVel;				//48
		PxVec3			deltaLinDt;				//60
		PxU16			lockFlags;				//62
		bool			isKinematic;			//63
		PxU8			pad;					//64

		PX_FORCE_INLINE PxReal projectVelocity(const PxVec3& lin, const PxVec3& ang)	const
		{
			return linearVelocity.dot(lin) + angularVelocity.dot(ang);
		}

	};
	
	//Needed only by prep, integration and 1D constraints
	struct PxTGSSolverBodyTxInertia
	{
		PxTransform deltaBody2World;
		PxMat33 sqrtInvInertia;
	};

	struct PxStepSolverBodyData
	{
		PX_ALIGN(16, PxVec3) originalLinearVelocity;
		PxReal maxContactImpulse;
		PxVec3 originalAngularVelocity;
		PxReal penBiasClamp;
		
		PxReal invMass;
		PxU32 nodeIndex;
		PxReal reportThreshold;
		PxU32 pad;

		PxReal projectVelocity(const PxVec3& linear, const PxVec3& angular) const
		{
			return originalLinearVelocity.dot(linear) + originalAngularVelocity.dot(angular);
		}
	};

/*!
Cache information specific to the software implementation(non common).

See PxcgetThreadContext.

Not thread-safe, so remember to have one object per thread!

TODO! refactor this and rename(it is a general per thread cache). Move transform cache into its own class.
*/
class ThreadContext : 
	public PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool>::EntryBase
{
	PX_NOCOPY(ThreadContext)
public:

#if PX_ENABLE_SIM_STATS
	struct ThreadSimStats
	{
		void clear()
		{

			numActiveConstraints = 0;
			numActiveDynamicBodies = 0;
			numActiveKinematicBodies = 0;
			numAxisSolverConstraints = 0;

		}

		PxU32 numActiveConstraints;
		PxU32 numActiveDynamicBodies;
		PxU32 numActiveKinematicBodies;
		PxU32 numAxisSolverConstraints;

	};
#endif

	//TODO: tune cache size based on number of active objects.
	ThreadContext(PxcNpMemBlockPool* memBlockPool);
	void reset();
	void resizeArrays(PxU32 frictionConstraintDescCount, PxU32 articulationCount);

	PX_FORCE_INLINE	Ps::Array<ArticulationSolverDesc>&		getArticulations()								{ return mArticulations;					}


#if PX_ENABLE_SIM_STATS
	PX_FORCE_INLINE ThreadSimStats& getSimStats()
	{
		return mThreadSimStats;
	}
#endif

	Gu::ContactBuffer mContactBuffer;

		// temporary buffer for correlation
	PX_ALIGN(16, CorrelationBuffer			mCorrelationBuffer); 

	FrictionPatchStreamPair		mFrictionPatchStreamPair;	// patch streams

	PxsConstraintBlockManager		mConstraintBlockManager;	// for when this thread context is "lead" on an island
	PxcConstraintBlockStream 		mConstraintBlockStream;		// constraint block pool


	// this stuff is just used for reformatting the solver data. Hopefully we should have a more
	// sane format for this when the dust settles - so it's just temporary. If we keep this around
	// here we should move these from public to private

	PxU32 mNumDifferentBodyConstraints;
	PxU32 mNumDifferentBodyFrictionConstraints;
	PxU32 mNumSelfConstraints;
	PxU32 mNumSelfFrictionConstraints;
	PxU32 mNumSelfConstraintBlocks;
	PxU32 mNumSelfConstraintFrictionBlocks;

	Ps::Array<PxU32>					mConstraintsPerPartition;
	Ps::Array<PxU32>					mFrictionConstraintsPerPartition;
	Ps::Array<PxU32>					mPartitionNormalizationBitmap;
	PxsBodyCore**						mBodyCoreArray;
	PxsRigidBody**						mRigidBodyArray;
	ArticulationV**						mArticulationArray;
	Cm::SpatialVector*					motionVelocityArray;
	PxU32*								bodyRemapTable;
	PxU32*								mNodeIndexArray;

	//Constraint info for normal constraint sovler
	PxSolverConstraintDesc*				contactConstraintDescArray;
	PxU32								contactDescArraySize;
	PxSolverConstraintDesc*				orderedContactConstraints;
	PxConstraintBatchHeader*			contactConstraintBatchHeaders;
	PxU32								numContactConstraintBatches;

	//Constraint info for partitioning
	PxSolverConstraintDesc*				tempConstraintDescArray;

	//Additional constraint info for 1d/2d friction model
	Ps::Array<PxSolverConstraintDesc>	frictionConstraintDescArray;
	Ps::Array<PxConstraintBatchHeader>	frictionConstraintBatchHeaders;

	//Info for tracking compound contact managers (temporary data - could use scratch memory!)
	Ps::Array<CompoundContactManager>	compoundConstraints;

	//Used for sorting constraints. Temporary, could use scratch memory
	Ps::Array<const PxsIndexedContactManager*>	orderedContactList;
	Ps::Array<const PxsIndexedContactManager*>	tempContactList;
	Ps::Array<PxU32>							sortIndexArray;
	
	Ps::Array<Cm::SpatialVectorF>				mZVector;
	Ps::Array<Cm::SpatialVectorF>				mDeltaV;


	PxU32								numDifferentBodyBatchHeaders;
	PxU32								numSelfConstraintBatchHeaders;

	
	PxU32								mOrderedContactDescCount;
	PxU32								mOrderedFrictionDescCount;

	PxU32								mConstraintSize;

	PxU32 mAxisConstraintCount;
	SelfConstraintBlock* mSelfConstraintBlocks;
	
	SelfConstraintBlock* mSelfConstraintFrictionBlocks;

	PxU32 mMaxPartitions;
	PxU32 mMaxFrictionPartitions;
	PxU32 mMaxSolverPositionIterations;
	PxU32 mMaxSolverVelocityIterations;
	PxU32 mMaxArticulationLength;
	PxU32 mMaxArticulationSolverLength;
	PxU32 mMaxArticulationLinks;
	
	PxSolverConstraintDesc* mContactDescPtr;
	PxSolverConstraintDesc* mStartContactDescPtr;
	PxSolverConstraintDesc* mFrictionDescPtr;

private:

	Ps::Array<ArticulationSolverDesc>	mArticulations;

#if PX_ENABLE_SIM_STATS
	ThreadSimStats				mThreadSimStats;
#endif

	public:

};

}

}

#endif //DY_THREADCONTEXT_H
