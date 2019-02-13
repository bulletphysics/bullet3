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

#include "PsTime.h"
#include "PsAtomic.h"
#include "PxvDynamics.h"

#include "common/PxProfileZone.h"
#include "PxsRigidBody.h"
#include "PxsContactManager.h"
#include "DyTGSDynamics.h"
#include "DyBodyCoreIntegrator.h"
#include "DySolverCore.h"
#include "DySolverControl.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "DyArticulationContactPrep.h"
#include "DySolverBody.h"

#include "DyConstraintPrep.h"
#include "DyConstraintPartition.h"
#include "DyArticulation.h"

#include "CmFlushPool.h"
#include "DyArticulationPImpl.h"
#include "PxsMaterialManager.h"
#include "DySolverContactPF4.h"
#include "DyContactReduction.h"
#include "PxcNpContactPrepShared.h"
#include "DyContactPrep.h"
#include "DySolverControlPF.h"
#include "PxSceneDesc.h"
#include "PxsSimpleIslandManager.h"
#include "PxvNphaseImplementationContext.h"
#include "PxsContactManagerState.h"
#include "PxsDefaultMemoryManager.h"
#include "DyContactPrepShared.h"
#include "DySolverContext.h"
#include "DyDynamics.h"
#include "DySolverConstraint1D.h"
#include "DyTGSPartition.h"
#include "PxvSimStats.h"

#define PX_USE_BLOCK_SOLVER 1
#define PX_USE_BLOCK_1D 1


namespace physx
{
namespace Dy
{

Context* createTGSDynamicsContext(PxcNpMemBlockPool* memBlockPool,
	PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
	PxvSimStats& simStats, PxTaskManager* taskManager, Ps::VirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
	IG::IslandSim* accurateIslandSim, PxU64 contextID,
	const bool enableStabilization, const bool useEnhancedDeterminism, const bool useAdaptiveForce,
	const PxReal lengthScale
	)
{
	return DynamicsTGSContext::create(memBlockPool, scratchAllocator, taskPool, simStats, taskManager, allocatorCallback, materialManager, accurateIslandSim,
		contextID, enableStabilization, useEnhancedDeterminism, useAdaptiveForce, lengthScale);
}

// PT: TODO: consider removing this function. We already have "createDynamicsContext".
DynamicsTGSContext* DynamicsTGSContext::create(PxcNpMemBlockPool* memBlockPool,
	PxcScratchAllocator& scratchAllocator,
	Cm::FlushPool& taskPool,
	PxvSimStats& simStats,
	PxTaskManager* taskManager,
	Ps::VirtualAllocatorCallback* allocatorCallback,
	PxsMaterialManager* materialManager,
	IG::IslandSim* accurateIslandSim,
	PxU64 contextID,
	const bool enableStabilization,
	const bool useEnhancedDeterminism,
	const bool useAdaptiveForce,
	const PxReal lengthScale
	)
{
	// PT: TODO: inherit from UserAllocated, remove placement new
	DynamicsTGSContext* dc = reinterpret_cast<DynamicsTGSContext*>(PX_ALLOC(sizeof(DynamicsTGSContext), "DynamicsTGSContext"));
	if (dc)
	{
		new(dc)DynamicsTGSContext(memBlockPool, scratchAllocator, taskPool, simStats, taskManager, allocatorCallback, materialManager, accurateIslandSim, contextID, enableStabilization, useEnhancedDeterminism, useAdaptiveForce, lengthScale);
	}
	return dc;
}

void DynamicsTGSContext::destroy()
{
	this->~DynamicsTGSContext();
	PX_FREE(this);
}

void DynamicsTGSContext::resetThreadContexts()
{
	PxcThreadCoherentCacheIterator<ThreadContext, PxcNpMemBlockPool> threadContextIt(mThreadContextPool);
	ThreadContext* threadContext = threadContextIt.getNext();

	while (threadContext != NULL)
	{
		threadContext->reset();
		threadContext = threadContextIt.getNext();
	}
}

PX_FORCE_INLINE PxVec3 safeRecip(const PxVec3& v)
{
	return PxVec3(v.x == 0.f ? 0.f : 1.f/v.x, v.y == 0.f ? 0.f : 1.f/v.y, v.z == 0.f ? 0.f : 1.f/v.z);
}

void copyToSolverBodyDataStep(const PxVec3& linearVelocity, const PxVec3& angularVelocity, const PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
	const PxReal maxDepenetrationVelocity, const PxReal maxContactImpulse, const PxU32 nodeIndex, const PxReal reportThreshold, 
	const PxReal maxAngVelSq, PxU32 lockFlags, bool isKinematic,
	PxTGSSolverBodyVel& solverVel, PxTGSSolverBodyTxInertia& solverBodyTxInertia, PxStepSolverBodyData& solverBodyData)
{
	const PxMat33 rotation(globalPose.q);

	const PxVec3 sqrtInvInertia = computeSafeSqrtInertia(invInertia);

	const PxVec3 sqrtBodySpaceInertia = safeRecip(sqrtInvInertia);

	Cm::transformInertiaTensor(sqrtInvInertia, rotation, solverBodyTxInertia.sqrtInvInertia);	

	solverBodyTxInertia.deltaBody2World.p = globalPose.p;
	solverBodyTxInertia.deltaBody2World.q = PxQuat(PxIdentity);

	PxMat33 sqrtInertia;
	Cm::transformInertiaTensor(sqrtBodySpaceInertia, rotation, sqrtInertia);

	PxVec3 lv = linearVelocity;
	PxVec3 av = angularVelocity;

	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			lv.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			lv.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			lv.z = 0.f;

		//KS - technically, we can zero the inertia columns and produce stiffer constraints. However, this can cause numerical issues with the 
		//joint solver, which is fixed by disabling joint preprocessing and setting minResponseThreshold to some reasonable value > 0. However, until
		//this is handled automatically, it's probably better not to zero these inertia rows
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
		{
			av.x = 0.f;
			//data.sqrtInvInertia.column0 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
		{
			av.y = 0.f;
			//data.sqrtInvInertia.column1 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
		{
			av.z = 0.f;
			//data.sqrtInvInertia.column2 = PxVec3(0.f);
		}
	}

	solverVel.linearVelocity = lv;
	solverVel.angularVelocity = sqrtInertia * av;
	solverVel.deltaLinDt = PxVec3(0.f);
	solverVel.deltaAngDt = PxVec3(0.f);
	solverVel.lockFlags = PxU16(lockFlags);
	solverVel.isKinematic = isKinematic;
	solverVel.maxAngVel = PxSqrt(maxAngVelSq);
	solverVel.partitionMask = 0;

	

	solverBodyData.nodeIndex = nodeIndex;
	solverBodyData.invMass = invMass;
	solverBodyData.penBiasClamp = maxDepenetrationVelocity;
	solverBodyData.maxContactImpulse = maxContactImpulse;	
	solverBodyData.reportThreshold = reportThreshold;
	solverBodyData.originalLinearVelocity = lv;
	solverBodyData.originalAngularVelocity = av;

	PX_ASSERT(lv.isFinite());
	PX_ASSERT(av.isFinite());
}



void copyToSolverBodyDataStepKinematic(const PxVec3& linearVelocity, const PxVec3& angularVelocity, const PxTransform& globalPose,
	const PxReal maxDepenetrationVelocity, const PxReal maxContactImpulse, const PxU32 nodeIndex, const PxReal reportThreshold,
	const PxReal maxAngVelSq,
	PxTGSSolverBodyVel& solverVel, PxTGSSolverBodyTxInertia& solverBodyTxInertia, PxStepSolverBodyData& solverBodyData)
{
	const PxMat33 rotation(globalPose.q);

	solverBodyTxInertia.deltaBody2World.p = globalPose.p;
	solverBodyTxInertia.deltaBody2World.q = PxQuat(PxIdentity);

	solverBodyTxInertia.sqrtInvInertia = PxMat33(PxVec3(0.f), PxVec3(0.f), PxVec3(0.f));

	solverVel.linearVelocity = PxVec3(0.f);
	solverVel.angularVelocity = PxVec3(0.f);
	solverVel.deltaLinDt = PxVec3(0.f);
	solverVel.deltaAngDt = PxVec3(0.f);
	solverVel.lockFlags = 0;
	solverVel.isKinematic = true;
	solverVel.maxAngVel = PxSqrt(maxAngVelSq);
	solverVel.partitionMask = 0;

	solverBodyData.nodeIndex = nodeIndex;
	solverBodyData.invMass = 0.f;
	solverBodyData.penBiasClamp = maxDepenetrationVelocity;
	solverBodyData.maxContactImpulse = maxContactImpulse;
	solverBodyData.reportThreshold = reportThreshold;
	solverBodyData.originalLinearVelocity = linearVelocity;
	solverBodyData.originalAngularVelocity = angularVelocity;
}


// =========================== Basic methods


DynamicsTGSContext::DynamicsTGSContext(PxcNpMemBlockPool* memBlockPool,
	PxcScratchAllocator& scratchAllocator,
	Cm::FlushPool& taskPool,
	PxvSimStats& simStats,
	PxTaskManager* taskManager,
	Ps::VirtualAllocatorCallback* allocatorCallback,
	PxsMaterialManager* materialManager,
	IG::IslandSim* accurateIslandSim,
	PxU64 contextID,
	const bool enableStabilization,
	const bool useEnhancedDeterminism,
	const bool useAdaptiveForce,
	const PxReal lengthScale
	) :
	Dy::Context(accurateIslandSim, allocatorCallback, simStats, enableStabilization, useEnhancedDeterminism, useAdaptiveForce, PX_MAX_F32),
	mThreadContextPool(memBlockPool),
	mMaterialManager(materialManager),
	mLengthScale(lengthScale),
	mScratchAllocator(scratchAllocator),
	mTaskPool(taskPool),
	mTaskManager(taskManager),
	mContextID(contextID)
{
	createThresholdStream(*allocatorCallback);
	createForceChangeThresholdStream(*allocatorCallback);
	mExceededForceThresholdStream[0] = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ThresholdStream), PX_DEBUG_EXP("ExceededForceThresholdStream[0]")), ThresholdStream(*allocatorCallback));
	mExceededForceThresholdStream[1] = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ThresholdStream), PX_DEBUG_EXP("ExceededForceThresholdStream[1]")), ThresholdStream(*allocatorCallback));
	mThresholdStreamOut = 0;
	mCurrentIndex = 0;

	PxMemZero(&mWorldSolverBodyVel, sizeof(mWorldSolverBodyVel));
	
	mWorldSolverBodyVel.lockFlags = 0;
	mWorldSolverBodyVel.isKinematic = false;
	mWorldSolverBodyTxInertia.sqrtInvInertia = PxMat33(PxZero);
	mWorldSolverBodyTxInertia.deltaBody2World = PxTransform(PxIdentity);
	
	mWorldSolverBodyData2.penBiasClamp = -PX_MAX_REAL;
	mWorldSolverBodyData2.maxContactImpulse = PX_MAX_REAL;
	mWorldSolverBodyData2.nodeIndex = IG_INVALID_NODE;
	mWorldSolverBodyData2.invMass = 0;
	mWorldSolverBodyData2.reportThreshold = PX_MAX_REAL;
	mWorldSolverBodyData2.originalLinearVelocity = PxVec3(0.f);
	mWorldSolverBodyData2.originalAngularVelocity = PxVec3(0.f);
	

}

DynamicsTGSContext::~DynamicsTGSContext()
{
	if (mExceededForceThresholdStream[0])
	{
		mExceededForceThresholdStream[0]->~ThresholdStream();
		PX_FREE(mExceededForceThresholdStream[0]);
	}
	mExceededForceThresholdStream[0] = NULL;

	if (mExceededForceThresholdStream[1])
	{
		mExceededForceThresholdStream[1]->~ThresholdStream();
		PX_FREE(mExceededForceThresholdStream[1]);
	}
	mExceededForceThresholdStream[1] = NULL;

}



void DynamicsTGSContext::setDescFromIndices(PxTGSSolverConstraintDesc& desc,
	const PxsIndexedInteraction& constraint, const PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies)
{
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eBODY == 0);
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eKINEMATIC == 1);
	const PxU32 offsetMap[] = { solverBodyOffset, 0 };
	//const PxU32 offsetMap[] = {mKinematicCount, 0};

	if (constraint.indexType0 == PxsIndexedInteraction::eARTICULATION)
	{
		ArticulationV* a = getArticulation(constraint.articulation0);
		desc.articulationA = a;
		desc.articulationALength = Ps::to16(a->getSolverDataSize());
		PX_ASSERT(0 == (desc.articulationALength & 0x0f));
		desc.linkIndexA = Ps::to16(a->getLinkIndex(constraint.articulation0));
		desc.bodyAIdx = 0;
	}
	else
	{
		desc.bodyA = constraint.indexType0 == PxsIndexedInteraction::eWORLD ? &mWorldSolverBodyVel
			: &solverBodies[PxU32(constraint.solverBody0) + offsetMap[constraint.indexType0] + 1];

		desc.bodyAIdx = constraint.indexType0 == PxsIndexedInteraction::eWORLD ? 0
			: PxU32(constraint.solverBody0) + offsetMap[constraint.indexType0] + 1;

		desc.linkIndexA = PxTGSSolverConstraintDesc::NO_LINK;
	}

	if (constraint.indexType1 == PxsIndexedInteraction::eARTICULATION)
	{
		ArticulationV* b = getArticulation(constraint.articulation1);
		desc.articulationB = b;
		desc.articulationBLength = Ps::to16(b->getSolverDataSize());
		PX_ASSERT(0 == (desc.articulationBLength & 0x0f));
		desc.linkIndexB = Ps::to16(b->getLinkIndex(constraint.articulation1));
		desc.bodyBIdx = 0;
	}
	else
	{
		desc.bodyB = constraint.indexType1 == PxsIndexedInteraction::eWORLD ? &mWorldSolverBodyVel
			: &solverBodies[PxU32(constraint.solverBody1) + offsetMap[constraint.indexType1] + 1];

		desc.bodyBIdx = constraint.indexType1 == PxsIndexedInteraction::eWORLD ? 0
			: PxU32(constraint.solverBody1) + offsetMap[constraint.indexType1] + 1;

		desc.linkIndexB = PxTGSSolverConstraintDesc::NO_LINK;
	}
}

void DynamicsTGSContext::setDescFromIndices(PxTGSSolverConstraintDesc& desc, IG::EdgeIndex edgeIndex, const IG::SimpleIslandManager& islandManager,
	PxU32* bodyRemap, const PxU32 solverBodyOffset, PxTGSSolverBodyVel* solverBodies)
{
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eBODY == 0);
	PX_COMPILE_TIME_ASSERT(PxsIndexedInteraction::eKINEMATIC == 1);

	const IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

	IG::NodeIndex node1 = islandSim.getNodeIndex1(edgeIndex);
	if (node1.isStaticBody())
	{
		desc.bodyA = &mWorldSolverBodyVel;
		desc.bodyAIdx = 0;
		desc.linkIndexA = PxTGSSolverConstraintDesc::NO_LINK;
	}
	else
	{
		const IG::Node& node = islandSim.getNode(node1);

		if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
		{
			PX_ASSERT(node1.isArticulation());
			Dy::ArticulationV* a = islandSim.getLLArticulation(node1);
			desc.articulationA = a;
			desc.articulationALength = Ps::to16(a->getSolverDataSize());
			PX_ASSERT(0 == (desc.articulationALength & 0x0f));
			desc.linkIndexA = Ps::to16(node1.articulationLinkId());
			desc.bodyAIdx = 0;
		}
		else
		{
			PX_ASSERT(!node1.isArticulation());
			PxU32 activeIndex = islandSim.getActiveNodeIndex(node1);
			PxU32 index = node.isKinematic() ? activeIndex : bodyRemap[activeIndex] + solverBodyOffset;
			desc.bodyA = &solverBodies[index + 1];
			desc.bodyAIdx = index + 1;
			desc.linkIndexA = PxTGSSolverConstraintDesc::NO_LINK;
		}
	}

	IG::NodeIndex node2 = islandSim.getNodeIndex2(edgeIndex);
	if (node2.isStaticBody())
	{
		desc.bodyB = &mWorldSolverBodyVel;
		desc.bodyBIdx = 0;
		desc.linkIndexB = PxTGSSolverConstraintDesc::NO_LINK;
	}
	else
	{
		const IG::Node& node = islandSim.getNode(node2);
		if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
		{
			PX_ASSERT(node2.isArticulation());
			Dy::ArticulationV* b = islandSim.getLLArticulation(node2);
			desc.articulationB = b;
			desc.articulationBLength = Ps::to16(b->getSolverDataSize());
			PX_ASSERT(0 == (desc.articulationBLength & 0x0f));
			desc.linkIndexB = Ps::to16(node2.articulationLinkId());
			desc.bodyBIdx = 0;
		}
		else
		{
			PX_ASSERT(!node2.isArticulation());
			PxU32 activeIndex = islandSim.getActiveNodeIndex(node2);
			PxU32 index = node.isKinematic() ? activeIndex : bodyRemap[activeIndex] + solverBodyOffset;
			desc.bodyB = &solverBodies[index + 1];
			desc.bodyBIdx = index + 1;
			desc.linkIndexB = PxTGSSolverConstraintDesc::NO_LINK;
		}
	}
}

class DynamicsMergeTask : public Cm::Task 
{
	PxBaseTask* mSecondContinuation;

public:
	DynamicsMergeTask(PxU64 contextId) : Cm::Task(contextId), mSecondContinuation(NULL)
	{
	}

	void setSecondContinuation(PxBaseTask* task) { task->addReference();  mSecondContinuation = task; }

	virtual const char* getName() const { return "MergeTask"; }

	virtual void runInternal()
	{
	}

	virtual void release()
	{
		mSecondContinuation->removeReference();
		Cm::Task::release();
	}

};

class KinematicCopyTGSTask : public Cm::Task 
{
	const IG::NodeIndex* const mKinematicIndices;
	const PxU32 mNbKinematics;
	const IG::IslandSim& mIslandSim;
	PxTGSSolverBodyVel* mVels;
	PxTGSSolverBodyTxInertia* mInertia;
	PxStepSolverBodyData* mBodyData;

	PX_NOCOPY(KinematicCopyTGSTask)

public:

	static const PxU32 NbKinematicsPerTask = 1024;

	KinematicCopyTGSTask(const IG::NodeIndex* const kinematicIndices,
		const PxU32 nbKinematics, const IG::IslandSim& islandSim, PxTGSSolverBodyVel* vels,
		PxTGSSolverBodyTxInertia* inertias, PxStepSolverBodyData* datas, PxU64 contextID) : Cm::Task(contextID), 
		mKinematicIndices(kinematicIndices), mNbKinematics(nbKinematics),
		mIslandSim(islandSim), mVels(vels), mInertia(inertias), mBodyData(datas)
	{
	}

	virtual const char* getName() const { return "KinematicCopyTask"; }

	virtual void runInternal()
	{
		for (PxU32 i = 0; i<mNbKinematics; i++)
		{
			PxsRigidBody* rigidBody = mIslandSim.getRigidBody(mKinematicIndices[i]);
			const PxsBodyCore& core = rigidBody->getCore();
			copyToSolverBodyDataStepKinematic(core.linearVelocity, core.angularVelocity, core.body2World, core.maxPenBias,
				core.maxContactImpulse, mKinematicIndices[i].index(), core.contactReportThreshold, core.maxAngularVelocitySq,
				mVels[i], mInertia[i], mBodyData[i]);

			rigidBody->saveLastCCDTransform();
		}
	}
};

class UpdateContinuationTGSTask : public Cm::Task
{
	DynamicsTGSContext& mContext;
	IG::SimpleIslandManager& mSimpleIslandManager;
	PxBaseTask* mLostTouchTask;

	PX_NOCOPY(UpdateContinuationTGSTask)
public:

	UpdateContinuationTGSTask(DynamicsTGSContext& context,
	IG::SimpleIslandManager& simpleIslandManager,
	PxBaseTask* lostTouchTask,
		PxU64 contextID) : Cm::Task(contextID), mContext(context), mSimpleIslandManager(simpleIslandManager),
		mLostTouchTask(lostTouchTask)
	{
	}

	virtual const char* getName() const { return "UpdateContinuationTask";}

	virtual void runInternal()
	{
		mContext.updatePostKinematic(mSimpleIslandManager, mCont, mLostTouchTask);
		//Allow lost touch task to run once all tasks have be scheduled
		mLostTouchTask->removeReference();
	}

};


void DynamicsTGSContext::update(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask,
	PxsContactManager** /*foundPatchManagers*/, PxU32 /*nbFoundPatchManagers*/,
	PxsContactManager** /*lostPatchManagers*/, PxU32 /*nbLostPatchManagers*/,
	PxU32 /*maxPatchesPerCM*/,
	PxsContactManagerOutputIterator& iterator,
	PxsContactManagerOutput*,
	const PxReal dt, const PxVec3& gravity, const PxU32 /*bitMapWordCounts*/)
{
	PX_PROFILE_ZONE("Dynamics.solverQueueTasks", mContextID);

	PX_UNUSED(simpleIslandManager);

	mOutputIterator = iterator;

	//const PxU32 nbSubsteps = 16;

	mDt = dt;
	mInvDt = 1.f / dt;

	mGravity = gravity;

	const IG::IslandSim& islandSim = simpleIslandManager.getAccurateIslandSim();

	const PxU32 islandCount = islandSim.getNbActiveIslands();

	const PxU32 activatedContactCount = islandSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
	const IG::EdgeIndex* const activatingEdges = islandSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);

	for (PxU32 a = 0; a < activatedContactCount; ++a)
	{
		PxsContactManager* cm = simpleIslandManager.getContactManager(activatingEdges[a]);
		if (cm)
		{
			cm->getWorkUnit().frictionPatchCount = 0; //KS - zero the friction patch count on any activating edges
		}
	}

#if PX_ENABLE_SIM_STATS
	if (islandCount > 0)
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
		mSimStats.mNbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);
	}
	else
	{
		mSimStats.mNbActiveKinematicBodies = islandSim.getNbActiveKinematics();
		mSimStats.mNbActiveDynamicBodies = 0;
		mSimStats.mNbActiveConstraints = 0;
	}
#endif

	mThresholdStreamOut = 0;

	resetThreadContexts();

	//If there is no work to do then we can do nothing at all.
	if (0 == islandCount)
	{
		return;
	}

	//Block to make sure it doesn't run before stage2 of update!
	lostTouchTask->addReference();

	UpdateContinuationTGSTask* task = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(UpdateContinuationTGSTask)), UpdateContinuationTGSTask)
		(*this, simpleIslandManager, lostTouchTask, mContextID);

	task->setContinuation(continuation);

	//KS - test that world solver body's velocities are finite and 0, then set it to 0.
	//Technically, the velocity should always be 0 but can be stomped if a NAN creeps into the simulation.
	PX_ASSERT(mWorldSolverBodyVel.linearVelocity == PxVec3(0.f));
	PX_ASSERT(mWorldSolverBodyVel.angularVelocity == PxVec3(0.f));
	PX_ASSERT(mWorldSolverBodyVel.linearVelocity.isFinite());
	PX_ASSERT(mWorldSolverBodyVel.angularVelocity.isFinite());

	mWorldSolverBodyVel.linearVelocity = mWorldSolverBodyVel.angularVelocity = PxVec3(0.f);

	const PxU32 kinematicCount = islandSim.getNbActiveKinematics();
	const IG::NodeIndex* const kinematicIndices = islandSim.getActiveKinematics();
	mKinematicCount = kinematicCount;

	const PxU32 bodyCount = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	PxU32 numArtics = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);

	{
		if (kinematicCount + bodyCount > mSolverBodyVelPool.capacity())
		{
			mSolverBodyRemapTable.reserve((kinematicCount + bodyCount + 31 + 1) & ~31);
			mSolverBodyVelPool.reserve((kinematicCount + bodyCount +31 + 1) & ~31);
			mSolverBodyTxInertiaPool.reserve((kinematicCount + bodyCount +31 + 1) & ~31);
			mSolverBodyDataPool2.reserve((kinematicCount + bodyCount +31 + 1) & ~31);
		}

		{
			mSolverBodyVelPool.resize(kinematicCount + bodyCount +1);
			mSolverBodyTxInertiaPool.resize(kinematicCount + bodyCount +1);
			mSolverBodyDataPool2.resize(kinematicCount + bodyCount +1);
			mSolverBodyRemapTable.resize(kinematicCount + bodyCount + 1);
		}

		// integrate and copy all the kinematics - overkill, since not all kinematics
		// need solver bodies

		mSolverBodyVelPool[0] = mWorldSolverBodyVel;
		mSolverBodyTxInertiaPool[0] = mWorldSolverBodyTxInertia;
		mSolverBodyDataPool2[0] = mWorldSolverBodyData2;


		{
			PX_PROFILE_ZONE("Dynamics.updateKinematics", mContextID);
			for (PxU32 i = 0; i < kinematicCount; i+= KinematicCopyTGSTask::NbKinematicsPerTask)
			{
				const PxU32 nbToProcess = PxMin(kinematicCount - i, KinematicCopyTGSTask::NbKinematicsPerTask);

				KinematicCopyTGSTask* kinematicTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(KinematicCopyTGSTask)), KinematicCopyTGSTask)
					(kinematicIndices + i, nbToProcess, islandSim, &mSolverBodyVelPool[i+1],
						&mSolverBodyTxInertiaPool[i + 1], &mSolverBodyDataPool2[i + 1], mContextID);

				kinematicTask->setContinuation(task);
				kinematicTask->removeReference();
			}
		}
	}

	PxU32 numArticulationConstraints = numArtics*Dy::DY_ARTICULATION_MAX_SIZE; //Just allocate enough memory to fit worst-case maximum size articulations...

	const PxU32 nbActiveContactManagers = islandSim.getNbActiveEdges(IG::Edge::eCONTACT_MANAGER);
	const PxU32 nbActiveConstraints = islandSim.getNbActiveEdges(IG::Edge::eCONSTRAINT);

	PxU32 totalConstraintCount = nbActiveConstraints + nbActiveContactManagers + numArticulationConstraints;

	mSolverConstraintDescPool.forceSize_Unsafe(0);
	mSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mOrderedSolverConstraintDescPool.forceSize_Unsafe(0);
	mOrderedSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mOrderedSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mContactConstraintBatchHeaders.forceSize_Unsafe(0);
	mContactConstraintBatchHeaders.reserve((totalConstraintCount + 63) & (~63));
	mContactConstraintBatchHeaders.forceSize_Unsafe(totalConstraintCount);

	mTempSolverConstraintDescPool.forceSize_Unsafe(0);
	mTempSolverConstraintDescPool.reserve((totalConstraintCount + 63) & (~63));
	mTempSolverConstraintDescPool.forceSize_Unsafe(totalConstraintCount);

	mContactList.forceSize_Unsafe(0);
	mContactList.reserve((nbActiveContactManagers + 63u) & (~63u));
	mContactList.forceSize_Unsafe(nbActiveContactManagers);

	mMotionVelocityArray.forceSize_Unsafe(0);
	mMotionVelocityArray.reserve((bodyCount + 63u) & (~63u));
	mMotionVelocityArray.forceSize_Unsafe(bodyCount);

	mBodyCoreArray.forceSize_Unsafe(0);
	mBodyCoreArray.reserve((bodyCount + 63u) & (~63u));
	mBodyCoreArray.forceSize_Unsafe(bodyCount);

	mRigidBodyArray.forceSize_Unsafe(0);
	mRigidBodyArray.reserve((bodyCount + 63u) & (~63u));
	mRigidBodyArray.forceSize_Unsafe(bodyCount);

	mArticulationArray.forceSize_Unsafe(0);
	mArticulationArray.reserve((numArtics + 63u) & (~63u));
	mArticulationArray.forceSize_Unsafe(numArtics);

	mNodeIndexArray.forceSize_Unsafe(0);
	mNodeIndexArray.reserve((bodyCount + 63u) & (~63u));
	mNodeIndexArray.forceSize_Unsafe(bodyCount);




	ThresholdStream& stream = getThresholdStream();
	stream.forceSize_Unsafe(0);
	stream.reserve(Ps::nextPowerOfTwo(nbActiveContactManagers != 0 ? nbActiveContactManagers - 1 : nbActiveContactManagers));

	

	//flip exceeded force threshold buffer
	mCurrentIndex = 1 - mCurrentIndex;

	task->removeReference();

}

void DynamicsTGSContext::updatePostKinematic(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* lostTouchTask)
{
	const IG::IslandSim& islandSim = simpleIslandManager.getAccurateIslandSim();

	const IG::IslandId*const islandIds = islandSim.getActiveIslands();

	DynamicsMergeTask* mergeTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(DynamicsMergeTask)), DynamicsMergeTask)(mContextID);

	mergeTask->setContinuation(continuation);
	mergeTask->setSecondContinuation(lostTouchTask);

	//PxReal dt = mDt;

	PxU32 currentIsland = 0;
	PxU32 currentBodyIndex = 0;
	PxU32 currentArticulation = 0;
	PxU32 currentContact = 0;

	PxU32 constraintIndex = 0;

	const PxU32 minIslandSize = mSolverBatchSize;

	const PxU32 islandCount = islandSim.getNbActiveIslands();

	//while(start<sentinel)
	while (currentIsland < islandCount)
	{
		SolverIslandObjectsStep objectStarts;
		objectStarts.articulations = mArticulationArray.begin() + currentArticulation;
		objectStarts.bodies = mRigidBodyArray.begin() + currentBodyIndex;
		objectStarts.contactManagers = mContactList.begin() + currentContact;
		objectStarts.constraintDescs = mSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.orderedConstraintDescs = mOrderedSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.constraintBatchHeaders = mContactConstraintBatchHeaders.begin() + constraintIndex;
		objectStarts.tempConstraintDescs = mTempSolverConstraintDescPool.begin() + constraintIndex;
		objectStarts.motionVelocities = mMotionVelocityArray.begin() + currentBodyIndex;
		objectStarts.bodyCoreArray = mBodyCoreArray.begin() + currentBodyIndex;
		objectStarts.islandIds = islandIds + currentIsland;
		objectStarts.bodyRemapTable = mSolverBodyRemapTable.begin();
		objectStarts.nodeIndexArray = mNodeIndexArray.begin() + currentBodyIndex;

		PxU32 startIsland = currentIsland;
		PxU32 constraintCount = 0;

		PxU32 nbArticulations = 0;
		PxU32 nbBodies = 0;
		PxU32 nbConstraints = 0;
		PxU32 nbContactManagers = 0;
		

		while (nbBodies < minIslandSize && currentIsland < islandCount && nbArticulations < 1)
		{
			const IG::Island& island = islandSim.getIsland(islandIds[currentIsland]);

			nbBodies += island.mSize[IG::Node::eRIGID_BODY_TYPE];
			nbArticulations += island.mSize[IG::Node::eARTICULATION_TYPE];
			nbConstraints += island.mEdgeCount[IG::Edge::eCONSTRAINT];
			nbContactManagers += island.mEdgeCount[IG::Edge::eCONTACT_MANAGER];
			constraintCount = nbConstraints + nbContactManagers;
			currentIsland++;
		}

		objectStarts.numIslands = currentIsland - startIsland;

		constraintIndex += nbArticulations*Dy::DY_ARTICULATION_MAX_SIZE;

		PxsIslandIndices counts;

		counts.articulations = nbArticulations;
		counts.bodies = nbBodies;

		counts.constraints = nbConstraints;
		counts.contactManagers = nbContactManagers;
		
		solveIsland(objectStarts, counts,
			mKinematicCount + currentBodyIndex, simpleIslandManager, mSolverBodyRemapTable.begin(), mMaterialManager, mOutputIterator,
			mergeTask);

		currentBodyIndex += nbBodies;
		currentArticulation += nbArticulations;
		currentContact += nbContactManagers;

		constraintIndex += constraintCount;
	}

	mergeTask->removeReference();
}

void DynamicsTGSContext::prepareBodiesAndConstraints(const SolverIslandObjectsStep& objects,
	IG::SimpleIslandManager& islandManager,
	IslandContextStep& islandContext)
{
	Dy::ThreadContext& mThreadContext = *islandContext.mThreadContext;

	mThreadContext.mMaxSolverPositionIterations = 0;
	mThreadContext.mMaxSolverVelocityIterations = 0;
	mThreadContext.mAxisConstraintCount = 0;
	mThreadContext.mContactDescPtr = mThreadContext.contactConstraintDescArray;
	mThreadContext.mFrictionDescPtr = mThreadContext.frictionConstraintDescArray.begin();
	mThreadContext.mNumDifferentBodyConstraints = 0;
	mThreadContext.mNumSelfConstraintBlocks = 0;
	mThreadContext.mNumSelfConstraints = 0;
	mThreadContext.mNumDifferentBodyFrictionConstraints = 0;
	mThreadContext.mNumSelfConstraintFrictionBlocks = 0;
	mThreadContext.mNumSelfFrictionConstraints = 0;
	mThreadContext.numContactConstraintBatches = 0;
	mThreadContext.contactDescArraySize = 0;

	mThreadContext.motionVelocityArray = objects.motionVelocities;
	mThreadContext.mBodyCoreArray = objects.bodyCoreArray;
	mThreadContext.mRigidBodyArray = objects.bodies;
	mThreadContext.mArticulationArray = objects.articulations;
	mThreadContext.bodyRemapTable = objects.bodyRemapTable;
	mThreadContext.mNodeIndexArray = objects.nodeIndexArray;

	const PxU32 frictionConstraintCount = 0;
	mThreadContext.resizeArrays(frictionConstraintCount, islandContext.mCounts.articulations);

	PxsBodyCore** PX_RESTRICT bodyArrayPtr = mThreadContext.mBodyCoreArray;
	PxsRigidBody** PX_RESTRICT rigidBodyPtr = mThreadContext.mRigidBodyArray;
	ArticulationV** PX_RESTRICT articulationPtr = mThreadContext.mArticulationArray;
	PxU32* PX_RESTRICT bodyRemapTable = mThreadContext.bodyRemapTable;
	PxU32* PX_RESTRICT nodeIndexArray = mThreadContext.mNodeIndexArray;

	PxU32 nbIslands = objects.numIslands;
	const IG::IslandId* const islandIds = objects.islandIds;

	const IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

	PxU32 bodyIndex = 0, articIndex = 0;
	for (PxU32 i = 0; i < nbIslands; ++i)
	{
		const IG::Island& island = islandSim.getIsland(islandIds[i]);

		IG::NodeIndex currentIndex = island.mRootNode;

		while (currentIndex.isValid())
		{
			const IG::Node& node = islandSim.getNode(currentIndex);

			if (node.getNodeType() == IG::Node::eARTICULATION_TYPE)
			{
				articulationPtr[articIndex++] = node.getArticulation();
			}
			else
			{
				PxsRigidBody* rigid = node.getRigidBody();
				PX_ASSERT(bodyIndex < (islandContext.mCounts.bodies + mKinematicCount + 1));
				rigidBodyPtr[bodyIndex] = rigid;
				bodyArrayPtr[bodyIndex] = &rigid->getCore();
				nodeIndexArray[bodyIndex] = currentIndex.index();
				bodyRemapTable[islandSim.getActiveNodeIndex(currentIndex)] = bodyIndex++;
			}

			currentIndex = node.mNextNode;
		}
	}


	PxsIndexedContactManager* indexedManagers = objects.contactManagers;

	PxU32 currentContactIndex = 0;
	for (PxU32 i = 0; i < nbIslands; ++i)
	{
		const IG::Island& island = islandSim.getIsland(islandIds[i]);

		IG::EdgeIndex contactEdgeIndex = island.mFirstEdge[IG::Edge::eCONTACT_MANAGER];

		while (contactEdgeIndex != IG_INVALID_EDGE)
		{
			const IG::Edge& edge = islandSim.getEdge(contactEdgeIndex);

			PxsContactManager* contactManager = islandManager.getContactManager(contactEdgeIndex);

			if (contactManager)
			{
				const IG::NodeIndex nodeIndex1 = islandSim.getNodeIndex1(contactEdgeIndex);
				const IG::NodeIndex nodeIndex2 = islandSim.getNodeIndex2(contactEdgeIndex);

				PxsIndexedContactManager& indexedManager = indexedManagers[currentContactIndex++];
				indexedManager.contactManager = contactManager;

				PX_ASSERT(!nodeIndex1.isStaticBody());
				{
					const IG::Node& node1 = islandSim.getNode(nodeIndex1);

					//Is it an articulation or not???
					if (node1.getNodeType() == IG::Node::eARTICULATION_TYPE)
					{
						indexedManager.indexType0 = PxsIndexedInteraction::eARTICULATION;
						indexedManager.solverBody0 = size_t(node1.getArticulation()) | nodeIndex1.articulationLinkId();
					}
					else
					{
						if (node1.isKinematic())
						{
							indexedManager.indexType0 = PxsIndexedInteraction::eKINEMATIC;
							indexedManager.solverBody0 = islandSim.getActiveNodeIndex(nodeIndex1);
						}
						else
						{
							indexedManager.indexType0 = PxsIndexedInteraction::eBODY;
							indexedManager.solverBody0 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex1)];
						}
						PX_ASSERT(indexedManager.solverBody0 < (islandContext.mCounts.bodies + mKinematicCount + 1));
					}

				}

				if (nodeIndex2.isStaticBody())
				{
					indexedManager.indexType1 = PxsIndexedInteraction::eWORLD;
				}
				else
				{
					const IG::Node& node2 = islandSim.getNode(nodeIndex2);

					//Is it an articulation or not???
					if (node2.getNodeType() == IG::Node::eARTICULATION_TYPE)
					{
						indexedManager.indexType1 = PxsIndexedInteraction::eARTICULATION;
						indexedManager.solverBody1 = size_t(node2.getArticulation()) | nodeIndex2.articulationLinkId();
					}
					else
					{
						if (node2.isKinematic())
						{
							indexedManager.indexType1 = PxsIndexedInteraction::eKINEMATIC;
							indexedManager.solverBody1 = islandSim.getActiveNodeIndex(nodeIndex2);
						}
						else
						{
							indexedManager.indexType1 = PxsIndexedInteraction::eBODY;
							indexedManager.solverBody1 = bodyRemapTable[islandSim.getActiveNodeIndex(nodeIndex2)];
						}
						PX_ASSERT(indexedManager.solverBody1 < (islandContext.mCounts.bodies + mKinematicCount + 1));
					}
				}

			}
			contactEdgeIndex = edge.mNextIslandEdge;
		}
	}

	islandContext.mCounts.contactManagers = currentContactIndex;
}

struct ConstraintLess
{
	bool operator()(const PxTGSSolverConstraintDesc& left, const PxTGSSolverConstraintDesc& right) const
	{
		return reinterpret_cast<Constraint*>(left.constraint)->index > reinterpret_cast<Constraint*>(right.constraint)->index;
	}
};

void DynamicsTGSContext::setupDescs(IslandContextStep& mIslandContext, const SolverIslandObjectsStep& mObjects, IG::SimpleIslandManager& mIslandManager,
	PxU32* mBodyRemapTable, PxU32 mSolverBodyOffset, PxsContactManagerOutputIterator& outputs)
{
	PX_UNUSED(outputs);
	ThreadContext& mThreadContext = *mIslandContext.mThreadContext;
	PxTGSSolverConstraintDesc* contactDescPtr = mObjects.constraintDescs;

	//PxU32 constraintCount = mCounts.constraints + mCounts.contactManagers;

	PxU32 nbIslands = mObjects.numIslands;
	const IG::IslandId* const islandIds = mObjects.islandIds;

	const IG::IslandSim& islandSim = mIslandManager.getAccurateIslandSim();

	for (PxU32 i = 0; i < nbIslands; ++i)
	{
		const IG::Island& island = islandSim.getIsland(islandIds[i]);

		IG::EdgeIndex edgeId = island.mFirstEdge[IG::Edge::eCONSTRAINT];

		while (edgeId != IG_INVALID_EDGE)
		{
			PxTGSSolverConstraintDesc& desc = *contactDescPtr;

			const IG::Edge& edge = islandSim.getEdge(edgeId);
			Dy::Constraint* constraint = mIslandManager.getConstraint(edgeId);
			setDescFromIndices(desc, edgeId, mIslandManager, mBodyRemapTable, mSolverBodyOffset, 
				mSolverBodyVelPool.begin());
			desc.constraint = reinterpret_cast<PxU8*>(constraint);
			desc.constraintLengthOver16 = DY_SC_TYPE_RB_1D;
			contactDescPtr++;
			edgeId = edge.mNextIslandEdge;
		}

	}

	Ps::sort(mObjects.constraintDescs, PxU32(contactDescPtr - mObjects.constraintDescs), ConstraintLess());

	if (mIslandContext.mCounts.contactManagers)
	{
		{
			for (PxU32 a = 0; a < mIslandContext.mCounts.contactManagers; ++a)
			{
				//PxsContactManagerOutput& output = outputs.getContactManager(mObjects.contactManagers[a].contactManager->getWorkUnit().mNpIndex);
				//if (output.nbContacts > 0)
				{
					PxTGSSolverConstraintDesc& desc = *contactDescPtr;
					setDescFromIndices(desc, mObjects.contactManagers[a], mSolverBodyOffset, mSolverBodyVelPool.begin());
					desc.constraint = reinterpret_cast<PxU8*>(mObjects.contactManagers[a].contactManager);
					desc.constraintLengthOver16 = DY_SC_TYPE_RB_CONTACT;
					contactDescPtr++;
				}
				//else
				//{
				//	//Clear friction state!
				//	mObjects.contactManagers[a].contactManager->getWorkUnit().frictionDataPtr = NULL;
				//	mObjects.contactManagers[a].contactManager->getWorkUnit().frictionPatchCount = 0;
				//}
			}

		}
	}
	mThreadContext.contactDescArraySize = PxU32(contactDescPtr - mObjects.constraintDescs);
}

void DynamicsTGSContext::preIntegrateBodies(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
	PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxStepSolverBodyData* solverBodyDataPool2, PxU32* nodeIndexArray, const PxU32 bodyCount, const PxVec3& gravity, const PxReal dt, PxU32& posIters, PxU32& velIters, PxU32 /*iteration*/)
{
	PX_PROFILE_ZONE("PreIntegrate", 0);
	PxU32 localMaxPosIter = 0;
	PxU32 localMaxVelIter = 0;
	for (PxU32 i = 0; i < bodyCount; ++i)
	{
		PxsBodyCore& core = *bodyArray[i];
		const PxsRigidBody& rBody = *originalBodyArray[i];

		PxU16 iterWord = core.solverIterationCounts;
		localMaxPosIter = PxMax<PxU32>(PxU32(iterWord & 0xff), localMaxPosIter);
		localMaxVelIter = PxMax<PxU32>(PxU32(iterWord >> 8), localMaxVelIter);

		//const Cm::SpatialVector& accel = originalBodyArray[i]->getAccelerationV();
		bodyCoreComputeUnconstrainedVelocity(gravity, dt, core.linearDamping, core.angularDamping, rBody.accelScale, core.maxLinearVelocitySq, core.maxAngularVelocitySq,
			core.linearVelocity, core.angularVelocity, !!(rBody.mInternalFlags & PxcRigidBody::eDISABLE_GRAVITY));

		copyToSolverBodyDataStep(core.linearVelocity, core.angularVelocity, core.inverseMass, core.inverseInertia, core.body2World, core.maxPenBias, core.maxContactImpulse, nodeIndexArray[i],
			core.contactReportThreshold, core.maxAngularVelocitySq, core.lockFlags, false,
			solverBodyVelPool[i+1], solverBodyTxInertia[i+1], solverBodyDataPool2[i+1]);
	}

	posIters = localMaxPosIter;
	velIters = localMaxVelIter;
}

class BlockAllocator : public PxConstraintAllocator
{
	PxsConstraintBlockManager& mConstraintBlockManager;
	PxcConstraintBlockStream& mConstraintBlockStream;
	FrictionPatchStreamPair& mFrictionPatchStreamPair;
	PxU32& mTotalConstraintByteSize;
public:

	BlockAllocator(PxsConstraintBlockManager& constraintBlockManager, PxcConstraintBlockStream& constraintBlockStream, FrictionPatchStreamPair& frictionPatchStreamPair,
		PxU32& totalConstraintByteSize) :
		mConstraintBlockManager(constraintBlockManager), mConstraintBlockStream(constraintBlockStream), mFrictionPatchStreamPair(frictionPatchStreamPair),
		mTotalConstraintByteSize(totalConstraintByteSize)
	{
	}

	virtual PxU8* reserveConstraintData(const PxU32 size)
	{
		mTotalConstraintByteSize += size;
		return mConstraintBlockStream.reserve(size, mConstraintBlockManager);
	}

	virtual PxU8* reserveFrictionData(const PxU32 size)
	{
		return mFrictionPatchStreamPair.reserve<PxU8>(size);
	}

	virtual PxU8* findInputPatches(PxU8* frictionCookie)
	{
		return frictionCookie;
	}

	PX_NOCOPY(BlockAllocator)

};

PxU32 getContactManagerConstraintDesc(const PxsContactManagerOutput& cmOutput, const PxsContactManager& /*cm*/, PxTGSSolverConstraintDesc& desc)
{
	desc.writeBackLengthOver4 = cmOutput.nbContacts;
	desc.writeBack = cmOutput.contactForces;
	return cmOutput.nbContacts;// cm.getWorkUnit().axisConstraintCount;
}

bool createFinalizeSolverContactsStep(PxTGSSolverContactDesc& contactDesc,
	PxsContactManagerOutput& output,
	ThreadContext& threadContext,
	const PxReal invDtF32,
	const PxReal invTotalDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxConstraintAllocator& constraintAllocator);

PxU32 SetupSolverConstraintStep(SolverConstraintShaderPrepDesc& shaderDesc,
	PxTGSSolverConstraintPrepDesc& prepDesc,
	PxConstraintAllocator& allocator,
	const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt, const PxU32 nbSubsteps,
	const PxReal lengthScale);

SolverConstraintPrepState::Enum setupSolverConstraintStep4
(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
	PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
	PxConstraintAllocator& allocator, const PxReal lengthScale);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
	PxsContactManagerOutput** cmOutputs,
	ThreadContext& threadContext,
	PxTGSSolverContactDesc* blockDescs,
	const PxReal invDtF32,
	const PxReal invTotalDtF32,
	PxReal bounceThresholdF32,
	PxReal	frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator);


void DynamicsTGSContext::createSolverConstraints(PxTGSSolverConstraintDesc* contactDescPtr, PxConstraintBatchHeader* headers, const PxU32 nbHeaders,
	PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& islandThreadContext, Dy::ThreadContext& threadContext, PxReal stepDt, PxReal totalDt, PxReal invStepDt,
	PxU32 nbSubsteps)
{
	PX_UNUSED(totalDt);
	PX_PROFILE_ZONE("CreateConstraints", 0);
	PxTransform idt(PxIdentity);

	BlockAllocator blockAllocator(islandThreadContext.mConstraintBlockManager, threadContext.mConstraintBlockStream, threadContext.mFrictionPatchStreamPair, threadContext.mConstraintSize);

	PxTGSSolverBodyTxInertia* txInertias = mSolverBodyTxInertiaPool.begin();
	PxStepSolverBodyData* solverBodyDatas = mSolverBodyDataPool2.begin();

	const PxReal invTotalDt = 1.f / totalDt;

	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		PxConstraintBatchHeader& hdr = headers[h];
		PxU32 startIdx = hdr.mStartIndex;
		PxU32 endIdx = startIdx + hdr.mStride;
		
		if (contactDescPtr[startIdx].constraintLengthOver16 == DY_SC_TYPE_RB_CONTACT)
		{
			PxTGSSolverContactDesc blockDescs[4];
			PxsContactManagerOutput* cmOutputs[4];
			PxsContactManager* cms[4];

			for (PxU32 a = startIdx, i = 0; a < endIdx; ++a, i++)
			{
				PxTGSSolverConstraintDesc& desc = contactDescPtr[a];
				PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);

				PxTGSSolverContactDesc& blockDesc = blockDescs[i];

				cms[i] = cm;

				PxcNpWorkUnit& unit = cm->getWorkUnit();

				PxsContactManagerOutput* cmOutput = &outputs.getContactManager(unit.mNpIndex);

				cmOutputs[i] = cmOutput;

				PxTGSSolverBodyVel& b0 = *desc.bodyA;
				PxTGSSolverBodyVel& b1 = *desc.bodyB;

				PxTGSSolverBodyTxInertia& txI0 = txInertias[desc.bodyAIdx];
				PxTGSSolverBodyTxInertia& txI1 = txInertias[desc.bodyBIdx];

				PxStepSolverBodyData& data0 = solverBodyDatas[desc.bodyAIdx];
				PxStepSolverBodyData& data1 = solverBodyDatas[desc.bodyBIdx];

				blockDesc.body0 = &b0;
				blockDesc.body1 = &b1;

				PxU8 flags = unit.rigidCore0->mFlags;
				if (unit.rigidCore1)
					flags |= PxU8(unit.rigidCore1->mFlags);

				blockDesc.bodyFrame0 = unit.rigidCore0->body2World;
				blockDesc.bodyFrame1 = unit.rigidCore1->body2World;
				blockDesc.shapeInteraction = cm->getShapeInteraction();
				blockDesc.contactForces = cmOutput->contactForces;
				blockDesc.desc = &desc;
				blockDesc.body0 = &b0;
				blockDesc.body1 = &b1;
				blockDesc.body0TxI = &txI0;
				blockDesc.body1TxI = &txI1;
				blockDesc.bodyData0 = &data0;
				blockDesc.bodyData1 = &data1;
				blockDesc.hasForceThresholds = !!(unit.flags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
				blockDesc.disableStrongFriction = !!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
				blockDesc.bodyState0 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
				blockDesc.bodyState1 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? PxSolverContactDesc::eARTICULATION : (unit.flags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
					((unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
				//blockDesc.flags = unit.flags;

				PxReal maxImpulse0 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? static_cast<const PxsBodyCore*>(unit.rigidCore0)->maxContactImpulse : data0.maxContactImpulse;
				PxReal maxImpulse1 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? static_cast<const PxsBodyCore*>(unit.rigidCore1)->maxContactImpulse : data1.maxContactImpulse;

				PxReal dominance0 = unit.dominance0 ? 1.f : 0.f;
				PxReal dominance1 = unit.dominance1 ? 1.f : 0.f;

				blockDesc.mInvMassScales.linear0 = blockDesc.mInvMassScales.angular0 = dominance0;
				blockDesc.mInvMassScales.linear1 = blockDesc.mInvMassScales.angular1 = dominance1;
				blockDesc.restDistance = unit.restDistance;
				blockDesc.frictionPtr = unit.frictionDataPtr;
				blockDesc.frictionCount = unit.frictionPatchCount;
				blockDesc.maxCCDSeparation = PX_MAX_F32;
				blockDesc.maxImpulse = PxMin(maxImpulse0, maxImpulse1);
				blockDesc.torsionalPatchRadius = unit.mTorsionalPatchRadius;
				blockDesc.minTorsionalPatchRadius = unit.mMinTorsionalPatchRadius;
			}

			SolverConstraintPrepState::Enum buildState = SolverConstraintPrepState::eUNBATCHABLE;

#if PX_USE_BLOCK_SOLVER
			if (hdr.mStride == 4)
			{
				buildState = createFinalizeSolverContacts4Step(
					cmOutputs,
					threadContext,
					blockDescs,
					invStepDt,
					invTotalDt,
					mBounceThreshold,
					mFrictionOffsetThreshold,
					mCorrelationDistance,
					mSolverOffsetSlop,
					blockAllocator);
			}
#endif

			if (buildState != SolverConstraintPrepState::eSUCCESS)
			{

				for (PxU32 a = startIdx, i = 0; a < endIdx; ++a, i++)
				{
					PxTGSSolverConstraintDesc& desc = contactDescPtr[a];
					PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
					//PxcNpWorkUnit& n = cm->getWorkUnit();

					PxsContactManagerOutput& output = *cmOutputs[i];
					//PX_ASSERT(output.nbContacts != 0);

					createFinalizeSolverContactsStep(blockDescs[i], output, threadContext,
						invStepDt, invTotalDt, mBounceThreshold, mFrictionOffsetThreshold, mCorrelationDistance, blockAllocator);

					getContactManagerConstraintDesc(output, *cm, desc);

					//PX_ASSERT(desc.constraint != NULL);
				}
			}

			for (PxU32 i = 0; i < hdr.mStride; ++i)
			{
				PxsContactManager* cm = cms[i];

				PxcNpWorkUnit& unit = cm->getWorkUnit();
				unit.frictionDataPtr = blockDescs[i].frictionPtr;
				unit.frictionPatchCount = blockDescs[i].frictionCount;

			}

		}
		else if (contactDescPtr[startIdx].constraintLengthOver16 == DY_SC_TYPE_RB_1D)
		{
			SolverConstraintShaderPrepDesc shaderPrepDescs[4];
			PxTGSSolverConstraintPrepDesc prepDescs[4];

			for (PxU32 a = startIdx, i = 0; a < endIdx; ++a, i++)
			{
				SolverConstraintShaderPrepDesc& shaderPrepDesc = shaderPrepDescs[i];
				PxTGSSolverConstraintPrepDesc& prepDesc = prepDescs[i];

				PxTransform id(PxIdentity);

				{
					PxTGSSolverConstraintDesc& desc = contactDescPtr[a];
					const Constraint* constraint = reinterpret_cast<const Constraint*>(desc.constraint);

					const PxConstraintSolverPrep solverPrep = constraint->solverPrep;
					const void* constantBlock = constraint->constantBlock;
					const PxU32 constantBlockByteSize = constraint->constantBlockSize;
					const PxTransform& pose0 = (constraint->body0 ? constraint->body0->getPose() : id);
					const PxTransform& pose1 = (constraint->body1 ? constraint->body1->getPose() : id);
					const PxTGSSolverBodyVel* sbody0 = desc.bodyA;
					const PxTGSSolverBodyVel* sbody1 = desc.bodyB;

					PxTGSSolverBodyTxInertia& txI0 = txInertias[desc.bodyAIdx];
					PxTGSSolverBodyTxInertia& txI1 = txInertias[desc.bodyBIdx];

					PxStepSolverBodyData& data0 = solverBodyDatas[desc.bodyAIdx];
					PxStepSolverBodyData& data1 = solverBodyDatas[desc.bodyBIdx];

					shaderPrepDesc.constantBlock = constantBlock;
					shaderPrepDesc.constantBlockByteSize = constantBlockByteSize;
					shaderPrepDesc.constraint = constraint;
					shaderPrepDesc.solverPrep = solverPrep;

					prepDesc.desc = &desc;
					prepDesc.bodyFrame0 = pose0;
					prepDesc.bodyFrame1 = pose1;
					prepDesc.body0 = sbody0;
					prepDesc.body1 = sbody1;
					prepDesc.body0TxI = &txI0;
					prepDesc.body1TxI = &txI1;
					prepDesc.bodyData0 = &data0;
					prepDesc.bodyData1 = &data1;
					prepDesc.linBreakForce = constraint->linBreakForce;
					prepDesc.angBreakForce = constraint->angBreakForce;
					prepDesc.writeback = &getConstraintWriteBackPool()[constraint->index];
					prepDesc.disablePreprocessing = !!(constraint->flags & PxConstraintFlag::eDISABLE_PREPROCESSING);
					prepDesc.improvedSlerp = !!(constraint->flags & PxConstraintFlag::eIMPROVED_SLERP);
					prepDesc.driveLimitsAreForces = !!(constraint->flags & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES);
					prepDesc.extendedLimits = !!(constraint->flags & PxConstraintFlag::eENABLE_EXTENDED_LIMITS);
					prepDesc.minResponseThreshold = constraint->minResponseThreshold;

					prepDesc.bodyState0 = desc.linkIndexA == PxSolverConstraintDesc::NO_LINK ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eARTICULATION;
					prepDesc.bodyState1 = desc.linkIndexB == PxSolverConstraintDesc::NO_LINK ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eARTICULATION;
				}
			}

			SolverConstraintPrepState::Enum buildState = SolverConstraintPrepState::eUNBATCHABLE;

#if PX_USE_BLOCK_SOLVER
#if PX_USE_BLOCK_1D
			if (hdr.mStride == 4)
			{
				PxU32 totalRows;
				buildState = setupSolverConstraintStep4
				(shaderPrepDescs, prepDescs, stepDt, totalDt, invStepDt, invTotalDt, totalRows, blockAllocator, mLengthScale);
			}
#endif
#endif

			if (buildState != SolverConstraintPrepState::eSUCCESS)
			{
				for (PxU32 a = startIdx, i = 0; a < endIdx; ++a, i++)
				{
					PxReal clampedInvDt = invStepDt;
					SetupSolverConstraintStep(shaderPrepDescs[i], prepDescs[i], blockAllocator, stepDt, totalDt, clampedInvDt, invTotalDt, nbSubsteps, mLengthScale);
				}
			}
		}
	}
}

void solveContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime);

void solve1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime);

void solveExtContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime, SolverContext& cache);

void solveExt1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime, SolverContext& cache,
	const PxTGSSolverBodyTxInertia* const txInertias);

void solveContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const bool doFriction,
	const PxReal minPenetration, const PxReal elapsedTime);

void solve1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias,
	const PxReal elapsedTime);


void DynamicsTGSContext::solveConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders, bool doFriction,
	PxReal invStepDt, const PxTGSSolverBodyTxInertia* const solverTxInertia, const PxReal elapsedTime, const PxReal minPenetration, SolverContext& cache)
{
	PX_UNUSED(invStepDt);
	PX_UNUSED(solverTxInertia);

	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		const PxConstraintBatchHeader& hdr = batchHeaders[h];
		
		switch (hdr.mConstraintType)
		{
		case DY_SC_TYPE_RB_CONTACT:
		case DY_SC_TYPE_STATIC_CONTACT:
			solveContactBlock(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_CONTACT:
			solveExtContactBlock(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime, cache);
			break;
		case DY_SC_TYPE_RB_1D:
			solve1DBlock(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_1D:
			solveExt1DBlock(hdr, contactDescPtr, elapsedTime, cache, solverTxInertia);
			break;
		case DY_SC_TYPE_BLOCK_RB_CONTACT:
			solveContact4(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime);
			break;
		case DY_SC_TYPE_BLOCK_1D:
			solve1D4(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		default:
			break;
		}
	}

}

void DynamicsTGSContext::parallelSolveConstraints(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders, bool doFriction,
	PxTGSSolverBodyTxInertia* solverTxInertia, const PxReal elapsedTime, const PxReal minPenetration,
	SolverContext& cache)
{
	PX_UNUSED(solverTxInertia);
	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		const PxConstraintBatchHeader& hdr = batchHeaders[h];


		switch (hdr.mConstraintType)
		{
		case DY_SC_TYPE_RB_CONTACT:
		case DY_SC_TYPE_STATIC_CONTACT:
			solveContactBlock(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_CONTACT:
			solveExtContactBlock(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime, cache);
			break;
		case DY_SC_TYPE_RB_1D:
			solve1DBlock(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_1D:
			solveExt1DBlock(hdr, contactDescPtr, elapsedTime, cache, solverTxInertia);
			break;
		case DY_SC_TYPE_BLOCK_RB_CONTACT:
			solveContact4(hdr, contactDescPtr, doFriction, minPenetration, elapsedTime);
			break;
		case DY_SC_TYPE_BLOCK_1D:
			solve1D4(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		default:
			break;
		}
	}

}


void writeBackContact(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, SolverContext* cache);


void writeBack1D(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc);

void writeBackContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, SolverContext* cache);

void writeBack1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc);


void DynamicsTGSContext::writebackConstraintsIteration(const PxConstraintBatchHeader* const hdrs, const PxTGSSolverConstraintDesc* const contactDescPtr, const PxU32 nbHeaders)
{
	PX_PROFILE_ZONE("Writeback", 0);

	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		const PxConstraintBatchHeader& hdr = hdrs[h];

		switch (hdr.mConstraintType)
		{
		case DY_SC_TYPE_RB_CONTACT:
		case DY_SC_TYPE_STATIC_CONTACT:
		case DY_SC_TYPE_EXT_CONTACT:
			writeBackContact(hdr, contactDescPtr, NULL);
			break;
		case DY_SC_TYPE_RB_1D:
		case DY_SC_TYPE_EXT_1D:
			writeBack1D(hdr, contactDescPtr);
			break;
		case DY_SC_TYPE_BLOCK_RB_CONTACT:
			writeBackContact4(hdr, contactDescPtr, NULL);
			break;
		case DY_SC_TYPE_BLOCK_1D:
			writeBack1D4(hdr, contactDescPtr);
			break;
		default:
			break;
		};		
	}
}

void DynamicsTGSContext::parallelWritebackConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr, const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders)
{
	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		const PxConstraintBatchHeader& hdr = batchHeaders[h];

		switch (hdr.mConstraintType)
		{
		case DY_SC_TYPE_RB_CONTACT:
		case DY_SC_TYPE_STATIC_CONTACT:
		case DY_SC_TYPE_EXT_CONTACT:
			writeBackContact(hdr, contactDescPtr, NULL);
			break;
		case DY_SC_TYPE_RB_1D:
		case DY_SC_TYPE_EXT_1D:
			writeBack1D(hdr, contactDescPtr);
			break;
		case DY_SC_TYPE_BLOCK_RB_CONTACT:
			writeBackContact4(hdr, contactDescPtr, NULL);
			break;
		default:
			break;
		};
	}
}

void solveConclude1DBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc,
	const PxTGSSolverBodyTxInertia* const txInertias, const PxReal elapsedTime);

void solveConcludeContactBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime);

void solveConcludeContact4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxReal elapsedTime);

void solveConclude1D4(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* PX_RESTRICT desc, const PxTGSSolverBodyTxInertia* const txInertias,
	const PxReal elapsedTime);

void solveConcludeContactExtBlock(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc, const PxReal elapsedTime, SolverContext& cache);

void solveConclude1DBlockExt(const PxConstraintBatchHeader& hdr, const PxTGSSolverConstraintDesc* desc,
	const PxReal elapsedTime, SolverContext& cache, const PxTGSSolverBodyTxInertia* const txInertias);

void DynamicsTGSContext::solveConcludeConstraintsIteration(const PxTGSSolverConstraintDesc* const contactDescPtr,
	const PxConstraintBatchHeader* const batchHeaders, const PxU32 nbHeaders, PxTGSSolverBodyTxInertia* solverTxInertia, 
	const PxReal elapsedTime, SolverContext& cache)
{
	for (PxU32 h = 0; h < nbHeaders; ++h)
	{
		const PxConstraintBatchHeader& hdr = batchHeaders[h];

		switch (hdr.mConstraintType)
		{
		case DY_SC_TYPE_RB_CONTACT:
		case DY_SC_TYPE_STATIC_CONTACT:
			solveConcludeContactBlock(hdr, contactDescPtr, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_CONTACT:
			solveConcludeContactExtBlock(hdr, contactDescPtr, elapsedTime, cache);
			break;
		case DY_SC_TYPE_RB_1D:
			solveConclude1DBlock(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		case DY_SC_TYPE_EXT_1D:
			solveConclude1DBlockExt(hdr, contactDescPtr, elapsedTime, cache, solverTxInertia);
			break;
		case DY_SC_TYPE_BLOCK_RB_CONTACT:
			solveConcludeContact4(hdr, contactDescPtr, elapsedTime);
			break;
		case DY_SC_TYPE_BLOCK_1D:
			solveConclude1D4(hdr, contactDescPtr, solverTxInertia, elapsedTime);
			break;
		default:
			break;
		}
	}
}


static void integrateCoreStep(PxTGSSolverBodyVel& vel, PxTGSSolverBodyTxInertia& txInertia, const PxStepSolverBodyData& /*bodyData*/, const PxF32 dt)
{
	PxU32 lockFlags = vel.lockFlags;
	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			vel.linearVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			vel.linearVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			vel.linearVelocity.z = 0.f;

		//The angular velocity should be 0 because it is now impossible to make it rotate around that axis!
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
			vel.angularVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
			vel.angularVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
			vel.angularVelocity.z = 0.f;
	}

	PxVec3 linearMotionVel = vel.linearVelocity;
	const PxVec3 delta = linearMotionVel * dt;


	PxVec3 unmolestedAngVel = vel.angularVelocity;
	PxVec3 angularMotionVel = txInertia.sqrtInvInertia * vel.angularVelocity;
	PxReal w2 = angularMotionVel.magnitudeSquared();
	txInertia.deltaBody2World.p += delta;
	PX_ASSERT(txInertia.deltaBody2World.p.isFinite());


	// Integrate the rotation using closed form quaternion integrator
	if (w2 != 0.0f)
	{
		PxReal w = PxSqrt(w2);
		
		//KS - we allow a little bit more angular velocity than the default
		//maxAngVel member otherwise the simulation feels a little flimsy
		/*const PxReal maxW = PxMax(50.f, vel.maxAngVel);

		if (w > maxW)
		{
			PxReal ratio = maxW / w;

			vel.angularVelocity *= ratio;
		}*/

		const PxReal v = dt * w * 0.5f;
		PxReal s, q;
		Ps::sincos(v, s, q);
		s /= w;

		const PxVec3 pqr = angularMotionVel * s;
		const PxQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
		PxQuat result = quatVel * txInertia.deltaBody2World.q;

		result += txInertia.deltaBody2World.q * q;

		txInertia.deltaBody2World.q = result.getNormalized();
		PX_ASSERT(txInertia.deltaBody2World.q.isSane());
		PX_ASSERT(txInertia.deltaBody2World.q.isFinite());

		//Accumulate the angular rotations in a space we can project the angular constraints to	
	}

	vel.deltaAngDt += unmolestedAngVel * dt;
	vel.deltaLinDt += delta;

	/*solverBodyData.body2World = txInertia.body2World;
	solverBodyData.deltaLinDt = vel.deltaLinDt;
	solverBodyData.deltaAngDt = vel.deltaAngDt;*/
}


void DynamicsTGSContext::integrateBodies(const SolverIslandObjectsStep& /*objects*/,
	const PxU32 count, PxTGSSolverBodyVel* PX_RESTRICT vels, PxTGSSolverBodyTxInertia* PX_RESTRICT txInertias, const PxStepSolverBodyData*const PX_RESTRICT bodyDatas, PxReal dt)
{
	for (PxU32 k = 0; k < count; k++)
	{
		integrateCoreStep(vels[k+1], txInertias[k+1], bodyDatas[k+1], dt);
	}
}

void DynamicsTGSContext::parallelIntegrateBodies(PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
	const PxStepSolverBodyData* const bodyDatas, const PxU32 count, PxReal dt)
{
	for (PxU32 k = 0; k < count; k++)
	{
		integrateCoreStep(vels[k+1], txInertias[k+1], bodyDatas[k+1], dt);
	}
}

void DynamicsTGSContext::copyBackBodies(const SolverIslandObjectsStep& objects,
	PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
	PxStepSolverBodyData* solverBodyDatas, PxReal invDt, IG::IslandSim& islandSim,
	PxU32 startIdx, PxU32 endIdx)
{
	for (PxU32 k = startIdx; k < endIdx; k++)
	{
		//PxStepSolverBody& solverBodyData = solverBodyData2[k + 1];
		PxTGSSolverBodyVel& solverBodyVel = vels[k + 1];
		PxTGSSolverBodyTxInertia& solverBodyTxI = txInertias[k + 1];
		PxStepSolverBodyData& solverBodyData = solverBodyDatas[k + 1];

		Cm::SpatialVector motionVel(solverBodyVel.deltaLinDt*invDt, solverBodyTxI.sqrtInvInertia*(solverBodyVel.deltaAngDt*invDt));

		PxsRigidBody& rBody = *objects.bodies[k];
		PxsBodyCore& core = rBody.getCore();
		rBody.mLastTransform = core.body2World;
		core.body2World.q = (solverBodyTxI.deltaBody2World.q * core.body2World.q).getNormalized();
		core.body2World.p = solverBodyTxI.deltaBody2World.p;
		core.linearVelocity = (solverBodyVel.linearVelocity);
		core.angularVelocity = solverBodyTxI.sqrtInvInertia*(solverBodyVel.angularVelocity);


		bool hasStaticTouch = islandSim.getIslandStaticTouchCount(IG::NodeIndex(solverBodyData.nodeIndex)) != 0;
		sleepCheck(&rBody, mDt, invDt, mEnableStabilization, mUseAdaptiveForce, motionVel,
			hasStaticTouch);
	}
}


void DynamicsTGSContext::stepArticulations(Dy::ThreadContext& threadContext, const PxsIslandIndices& counts, PxReal dt)
{
	for (PxU32 a = 0; a < counts.articulations; ++a)
	{
		ArticulationSolverDesc& d = threadContext.getArticulations()[a];
		//if(d.articulation->numTotalConstraints > 0)
		//d.articulation->solveInternalConstraints(dt, 1.f / dt, threadContext.mZVector.begin(), threadContext.mDeltaV.begin(), false);
		ArticulationPImpl::updateDeltaMotion(d, dt, threadContext.mDeltaV.begin());
			
	}
}

void DynamicsTGSContext::updateArticulations(Dy::ThreadContext& threadContext, const PxU32 startIdx, const PxU32 endIdx, PxReal dt)
{
	for (PxU32 a = startIdx; a < endIdx; ++a)
	{
		ArticulationSolverDesc& d = threadContext.getArticulations()[a];
		ArticulationPImpl::updateBodiesTGS(d, dt);
	}
}

class ArticulationTask : public Cm::Task
{
	Dy::DynamicsTGSContext& mContext;
	ArticulationSolverDesc* mDescs;
	PxU32 mNbDescs;
	PxVec3 mGravity;
	PxReal mDt;

	PX_NOCOPY(ArticulationTask)

public:
	static const PxU32 MaxNbPerTask = 4;

	ArticulationTask(Dy::DynamicsTGSContext& context, ArticulationSolverDesc* descs, const PxU32 nbDescs, const PxVec3& gravity,
		PxReal dt, PxU64 contextId) : Cm::Task(contextId), mContext(context),
		mDescs(descs), mNbDescs(nbDescs), mGravity(gravity), mDt(dt)
	{
	}

	virtual const char* getName() const { return "ArticulationTask";  }

	virtual void runInternal()
	{
		PxU32 maxLinks = 0;
		for (PxU32 i = 0; i < mNbDescs; i++)
		{
			maxLinks = PxMax(maxLinks, PxU32(mDescs[i].linkCount));
		}

		Dy::ThreadContext& threadContext = *mContext.getThreadContext();

		threadContext.mZVector.forceSize_Unsafe(0);
		threadContext.mZVector.reserve(maxLinks);
		threadContext.mZVector.forceSize_Unsafe(maxLinks);

		threadContext.mDeltaV.forceSize_Unsafe(0);
		threadContext.mDeltaV.reserve(maxLinks);
		threadContext.mDeltaV.forceSize_Unsafe(maxLinks);

		for (PxU32 a = 0; a < mNbDescs; ++a)
		{
			
			ArticulationPImpl::computeUnconstrainedVelocitiesTGS(mDescs[a], mDt, 
				mGravity, getContextId(), threadContext.mZVector.begin(), threadContext.mDeltaV.begin());
		}

		mContext.putThreadContext(&threadContext);
	}

};


void DynamicsTGSContext::setupArticulations(IslandContextStep& islandContext, const PxVec3& gravity, const PxReal dt, PxU32& posIters,
	PxU32& velIters, PxBaseTask* continuation)
{
	Dy::ArticulationV** articulations = islandContext.mThreadContext->mArticulationArray;
	PxU32 nbArticulations = islandContext.mCounts.articulations;

	PxU32 maxVelIters = 0;
	PxU32 maxPosIters = 0;
	PxU32 maxArticulationLength = 0;
	PxU32 maxSolverArticLength = 0;

	//PxU32 startIdx = 0;
	for (PxU32 a = 0; a < nbArticulations; a+= ArticulationTask::MaxNbPerTask)
	{
		const PxU32 endIdx = PxMin(nbArticulations, a + ArticulationTask::MaxNbPerTask);
		for (PxU32 b = a; b < endIdx; ++b)
		{
			ArticulationSolverDesc& desc = islandContext.mThreadContext->getArticulations()[b];
			articulations[b]->getSolverDesc(desc);
			maxArticulationLength = PxMax(maxArticulationLength, PxU32(desc.totalDataSize));
			maxSolverArticLength = PxMax(maxSolverArticLength, PxU32(desc.solverDataSize));

			const PxU16 iterWord = articulations[b]->getIterationCounts();
			maxVelIters = PxMax<PxU32>(PxU32(iterWord >> 8), maxVelIters);
			maxPosIters = PxMax<PxU32>(PxU32(iterWord & 0xff), maxPosIters);
		}

		Dy::ThreadContext* threadContext = islandContext.mThreadContext;
		ArticulationTask* task = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(ArticulationTask)), ArticulationTask)(*this, threadContext->getArticulations().begin() + a,
			endIdx - a, gravity, dt, getContextId());

		task->setContinuation(continuation);
		task->removeReference();

		

		//startIdx += descCount;

	}

	velIters = PxMax(maxVelIters, velIters);
	posIters = PxMax(maxPosIters, posIters);
	
}

PxU32 DynamicsTGSContext::setupArticulationInternalConstraints(IslandContextStep& islandContext, PxReal dt, PxReal invStepDt, PxTGSSolverConstraintDesc* constraintDescs)
{
	Dy::ArticulationV** articulations = islandContext.mThreadContext->mArticulationArray;
	PxU32 nbArticulations = islandContext.mCounts.articulations;

	ThreadContext* threadContext = getThreadContext();
	threadContext->mConstraintBlockStream.reset();

	PxU32 startIdx = 0;
	PxU32 totalDescCount = 0;
	Cm::SpatialVectorF Z[64];
	for (PxU32 a = 0; a < nbArticulations; ++a)
	{
		ArticulationSolverDesc& desc = islandContext.mThreadContext->getArticulations()[a];
		articulations[a]->getSolverDesc(desc);

		PxU32 acCount;
		PxU32 descCount = ArticulationPImpl::setupSolverInternalConstraintsTGS(desc, threadContext->mConstraintBlockStream, constraintDescs+startIdx,
			islandContext.mStepDt, invStepDt, dt, acCount, islandContext.mThreadContext->mConstraintBlockManager, Z);

		desc.numInternalConstraints = Ps::to8(descCount);

		totalDescCount += descCount;

		startIdx += DY_ARTICULATION_MAX_SIZE;
	}

	putThreadContext(threadContext);

	islandContext.mThreadContext->contactDescArraySize += totalDescCount;

	return totalDescCount;
}

class SetupDescsTask : public Cm::Task
{
	Dy::IslandContextStep& mIslandContext;
	const SolverIslandObjectsStep& mObjects;
	IG::SimpleIslandManager& mIslandManager;
	PxU32* mBodyRemapTable; 
	PxU32 mSolverBodyOffset; 
	PxsContactManagerOutputIterator& mOutputs;
	DynamicsTGSContext& mContext;

	PX_NOCOPY(SetupDescsTask)

public:

	SetupDescsTask(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, IG::SimpleIslandManager& islandManager,
		PxU32* bodyRemapTable, PxU32 solverBodyOffset, PxsContactManagerOutputIterator& outputs, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mObjects(objects), mIslandManager(islandManager), mBodyRemapTable(bodyRemapTable), mSolverBodyOffset(solverBodyOffset),
		mOutputs(outputs), mContext(context)
	{
	}

	virtual const char* getName() const { return "SetupDescsTask"; }

	virtual void runInternal()
	{
		mContext.setupDescs(mIslandContext, mObjects, mIslandManager, mBodyRemapTable, mSolverBodyOffset, mOutputs);
		mIslandContext.mArticulationOffset = mIslandContext.mThreadContext->contactDescArraySize;
	}
};


class PreIntegrateParallelTask : public Cm::Task
{
	PxsBodyCore** mBodyArray;
	PxsRigidBody** mOriginalBodyArray;
	PxTGSSolverBodyVel* mSolverBodyVelPool;
	PxTGSSolverBodyTxInertia* mSolverBodyTxInertia;
	PxStepSolverBodyData* mSolverBodyDataPool2;
	PxU32* mNodeIndexArray;
	const PxU32 mBodyCount;
	const PxVec3& mGravity;
	const PxReal mDt;
	PxU32& mPosIters;
	PxU32& mVelIters;
	DynamicsTGSContext& mContext;

	PX_NOCOPY(PreIntegrateParallelTask)

public:

	PreIntegrateParallelTask(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
		PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxStepSolverBodyData* solverBodyDataPool2,
		PxU32* nodeIndexArray, const PxU32 bodyCount, const PxVec3& gravity, const PxReal dt, PxU32& posIters, PxU32& velIters,
		DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mBodyArray(bodyArray), mOriginalBodyArray(originalBodyArray), mSolverBodyVelPool(solverBodyVelPool),
		mSolverBodyTxInertia(solverBodyTxInertia), mSolverBodyDataPool2(solverBodyDataPool2), mNodeIndexArray(nodeIndexArray),
		mBodyCount(bodyCount), mGravity(gravity), mDt(dt), mPosIters(posIters), mVelIters(velIters), mContext(context)
	{
	}

	virtual const char* getName() const { return "PreIntegrateParallelTask"; }

	virtual void runInternal()
	{
		PxU32 posIters = 0;
		PxU32 velIters = 0;
		mContext.preIntegrateBodies(mBodyArray, mOriginalBodyArray, mSolverBodyVelPool, mSolverBodyTxInertia, mSolverBodyDataPool2, mNodeIndexArray, mBodyCount, mGravity, mDt, posIters, velIters, 0);

		Ps::atomicMax(reinterpret_cast<PxI32*>(&mPosIters), PxI32(posIters));
		Ps::atomicMax(reinterpret_cast<PxI32*>(&mVelIters), PxI32(velIters));

	}
};


class PreIntegrateTask : public Cm::Task
{
	PxsBodyCore** mBodyArray;
	PxsRigidBody** mOriginalBodyArray;
	PxTGSSolverBodyVel* mSolverBodyVelPool;
	PxTGSSolverBodyTxInertia* mSolverBodyTxInertia;
	PxStepSolverBodyData* mSolverBodyDataPool2;
	PxU32* mNodeIndexArray;
	const PxU32 mBodyCount; 
	const PxVec3& mGravity;
	const PxReal mDt;
	PxU32& mPosIters;
	PxU32& mVelIters; 
	DynamicsTGSContext& mContext;

	PX_NOCOPY(PreIntegrateTask)

public:

	PreIntegrateTask(PxsBodyCore** bodyArray, PxsRigidBody** originalBodyArray,
		PxTGSSolverBodyVel* solverBodyVelPool, PxTGSSolverBodyTxInertia* solverBodyTxInertia, PxStepSolverBodyData* solverBodyDataPool2,
		PxU32* nodeIndexArray, const PxU32 bodyCount, const PxVec3& gravity, const PxReal dt, PxU32& posIters, PxU32& velIters,
		DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mBodyArray(bodyArray), mOriginalBodyArray(originalBodyArray), mSolverBodyVelPool(solverBodyVelPool),
		mSolverBodyTxInertia(solverBodyTxInertia), mSolverBodyDataPool2(solverBodyDataPool2), mNodeIndexArray(nodeIndexArray), 
		mBodyCount(bodyCount), mGravity(gravity), mDt(dt), mPosIters(posIters), mVelIters(velIters), mContext(context)
	{
	}

	virtual const char* getName() const { return "PreIntegrateTask"; }

	virtual void runInternal()
	{
		const PxU32 BodiesPerTask = 512;

		if (mBodyCount <= BodiesPerTask)
		{
			PxU32 posIters = 0;
			PxU32 velIters = 0;
			mContext.preIntegrateBodies(mBodyArray, mOriginalBodyArray, mSolverBodyVelPool, mSolverBodyTxInertia, mSolverBodyDataPool2, mNodeIndexArray, mBodyCount, mGravity, mDt, posIters, velIters, 0);

			Ps::atomicMax(reinterpret_cast<PxI32*>(&mPosIters), PxI32(posIters));
			Ps::atomicMax(reinterpret_cast<PxI32*>(&mVelIters), PxI32(velIters));
		}
		else
		{
			for (PxU32 i = 0; i < mBodyCount; i += BodiesPerTask)
			{
				const PxU32 nbToProcess = PxMin(mBodyCount - i, BodiesPerTask);
				PreIntegrateParallelTask* task = PX_PLACEMENT_NEW(mContext.getTaskPool().allocate(sizeof(PreIntegrateParallelTask)), PreIntegrateParallelTask)
					(mBodyArray+i, mOriginalBodyArray+i, mSolverBodyVelPool+i, mSolverBodyTxInertia+i, mSolverBodyDataPool2+i,mNodeIndexArray+i,
						nbToProcess, mGravity, mDt, mPosIters, mVelIters, mContext);

				task->setContinuation(mCont);
				task->removeReference();
			}
		}


	}
};

class SetStepperTask : public Cm::Task
{
	Dy::IslandContextStep& mIslandContext;
	Dy::DynamicsTGSContext& mContext;

	PxBaseTask* mAdditionalContinuation;

	PX_NOCOPY(SetStepperTask)

public:

	SetStepperTask(Dy::IslandContextStep& islandContext,
		DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mContext(context),
		mAdditionalContinuation(NULL)
	{
	}

	virtual const char* getName() const { return "SetStepperTask"; }

	void setAdditionalContinuation(PxBaseTask* cont)
	{
		mAdditionalContinuation = cont;
		cont->addReference();
	}

	virtual void runInternal()
	{
		PxReal dt = mContext.getDt();

		//ML: TGS can't work well with high velocity iteration counts, so we should limit the velocity iteration counts to be DY_MAX_ITERATION_COUNT. However,
		//we should put the extra iterations to the position iteration count so the users will see some behaviour improvements

		const PxU32 newVelIters = PxMin(mIslandContext.mVelIters, PxU32(DY_MAX_VELOCITY_COUNT));
		const PxU32 remainVelIters = mIslandContext.mVelIters - newVelIters;

		mIslandContext.mVelIters = newVelIters;
		mIslandContext.mPosIters = mIslandContext.mPosIters + remainVelIters;

		mIslandContext.mStepDt = dt / PxReal(mIslandContext.mPosIters);
		mIslandContext.mInvStepDt = 1.f/mIslandContext.mStepDt;//PxMin(1000.f, 1.f / mIslandContext.mStepDt);
	}

	virtual void release()
	{
		Cm::Task::release();
		mAdditionalContinuation->removeReference();
	}
};

class SetupArticulationTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	const PxVec3& mGravity;
	const PxReal mDt;
	PxU32& mPosIters;
	PxU32& mVelIters;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(SetupArticulationTask)

public:

	SetupArticulationTask(IslandContextStep& islandContext, const PxVec3& gravity, const PxReal dt, PxU32& posIters,
		PxU32& velIters, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mGravity(gravity), mDt(dt), mPosIters(posIters), mVelIters(velIters), mContext(context)
	{
	}

	virtual const char* getName() const { return "SetupArticulationTask"; }

	virtual void runInternal()
	{
		PxU32 posIters = 0, velIters = 0;
		mContext.setupArticulations(mIslandContext, mGravity, mDt, posIters, velIters, mCont);

		Ps::atomicMax(reinterpret_cast<PxI32*>(&mPosIters), PxI32(posIters));
		Ps::atomicMax(reinterpret_cast<PxI32*>(&mVelIters), PxI32(velIters));
	}
};

class SetupArticulationInternalConstraintsTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	const PxReal mDt;
	const PxReal mInvDt;
	PxTGSSolverConstraintDesc* mConstraintDescs;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(SetupArticulationInternalConstraintsTask)

public:

	SetupArticulationInternalConstraintsTask(IslandContextStep& islandContext, PxReal dt, PxReal invDt, PxTGSSolverConstraintDesc* constraintDescs, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mDt(dt), mInvDt(invDt), mConstraintDescs(constraintDescs), mContext(context)
	{
	}

	virtual const char* getName() const { return "SetupArticulationInternalConstraintsTask"; }

	virtual void runInternal()
	{
		mContext.setupArticulationInternalConstraints(mIslandContext, mDt, mIslandContext.mInvStepDt, mConstraintDescs + mIslandContext.mArticulationOffset);
	}
};

class SetupSolverConstraintsSubTask : public Cm::Task
{
	PxTGSSolverConstraintDesc* mContactDescPtr;
	PxConstraintBatchHeader* mHeaders;
	const PxU32 mNbHeaders;
	PxsContactManagerOutputIterator& mOutputs;
	PxReal mStepDt;
	PxReal mTotalDt;
	PxReal mInvStepDt; 
	PxReal mInvDtTotal;
	PxU32 mNbSubsteps;
	DynamicsTGSContext& mContext;
	ThreadContext& mIslandThreadContext;

	PX_NOCOPY(SetupSolverConstraintsSubTask)

public:

	static const PxU32 MaxPerTask = 64;

	SetupSolverConstraintsSubTask(PxTGSSolverConstraintDesc* contactDescPtr, PxConstraintBatchHeader* headers, const PxU32 nbHeaders,
		PxsContactManagerOutputIterator& outputs, PxReal stepDt, PxReal totalDt, PxReal invStepDt, PxReal invDtTotal, PxU32 nbSubsteps, ThreadContext& islandThreadContext, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mContactDescPtr(contactDescPtr), mHeaders(headers), mNbHeaders(nbHeaders), mOutputs(outputs), mStepDt(stepDt), mTotalDt(totalDt), mInvStepDt(invStepDt),
		mInvDtTotal(invDtTotal), mNbSubsteps(nbSubsteps), mContext(context), mIslandThreadContext(islandThreadContext)
	{
	}

	virtual const char* getName() const { return "SetupSolverConstraintsSubTask"; }

	virtual void runInternal()
	{
		ThreadContext* tempContext = mContext.getThreadContext();
		tempContext->mConstraintBlockStream.reset();
		mContext.createSolverConstraints(mContactDescPtr, mHeaders, mNbHeaders, mOutputs, mIslandThreadContext, *tempContext, mStepDt, mTotalDt, mInvStepDt,
			mNbSubsteps);
		mContext.putThreadContext(tempContext);
	}

};

class SetupSolverConstraintsTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	PxTGSSolverConstraintDesc* mContactDescPtr;
	PxsContactManagerOutputIterator& mOutputs;
	Dy::ThreadContext& mThreadContext;
	PxReal mTotalDt;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(SetupSolverConstraintsTask)

public:

	

	SetupSolverConstraintsTask(IslandContextStep& islandContext, PxTGSSolverConstraintDesc* contactDescPtr,
		PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& threadContext, PxReal totalDt, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mContactDescPtr(contactDescPtr), mOutputs(outputs), mThreadContext(threadContext), 
		mTotalDt(totalDt), mContext(context)
	{
	}

	virtual const char* getName() const { return "SetupSolverConstraintsTask"; }

	virtual void runInternal()
	{
		Dy::ThreadContext& threadContext = *mIslandContext.mThreadContext;
		const PxU32 nbBatches = threadContext.numContactConstraintBatches;
		PxConstraintBatchHeader* hdr = mIslandContext.mObjects.constraintBatchHeaders;
		//for (PxU32 a = 0; a < mIslandContext.mArticulationOffset; a += SetupSolverConstraintsSubTask::MaxPerTask)
		for (PxU32 a = 0; a < nbBatches; a += SetupSolverConstraintsSubTask::MaxPerTask)
		{
			const PxU32 nbConstraints = PxMin(nbBatches - a, SetupSolverConstraintsSubTask::MaxPerTask);
			SetupSolverConstraintsSubTask* task = PX_PLACEMENT_NEW(mContext.mTaskPool.allocate(sizeof(SetupSolverConstraintsSubTask)), SetupSolverConstraintsSubTask)
				(mContactDescPtr, hdr + a, nbConstraints, mOutputs, mIslandContext.mStepDt, mTotalDt, mIslandContext.mInvStepDt, mContext.mInvDt, mIslandContext.mPosIters, mThreadContext, mContext);

			task->setContinuation(mCont);
			task->removeReference();
		}
	}
};

static bool isArticulationConstraint(PxTGSSolverConstraintDesc& desc)
{
	return desc.linkIndexA != PxTGSSolverConstraintDesc::NO_LINK ||
		desc.linkIndexB != PxTGSSolverConstraintDesc::NO_LINK;
}


class PartitionTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	PxTGSSolverConstraintDesc* mContactDescPtr;
	PxTGSSolverBodyVel* mSolverBodyData;
	Dy::ThreadContext& mThreadContext;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(PartitionTask)

public:



	PartitionTask(IslandContextStep& islandContext, PxTGSSolverConstraintDesc* contactDescPtr, PxTGSSolverBodyVel* solverBodyData,
		Dy::ThreadContext& threadContext, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mContactDescPtr(contactDescPtr), mSolverBodyData(solverBodyData), mThreadContext(threadContext),
		mContext(context)
	{
	}

	virtual const char* getName() const { return "PartitionTask"; }

	virtual void runInternal()
	{
	
		ArticulationSolverDesc* artics = mThreadContext.getArticulations().begin();
		if (mIslandContext.mCounts.articulations)
		{
			PxU32 nbArticConstraints = artics[0].numInternalConstraints + mIslandContext.mArticulationOffset;

			PxTGSSolverConstraintDesc* currDesc = mContactDescPtr;
			for (PxU32 a = 1, startIdx = mIslandContext.mArticulationOffset+DY_ARTICULATION_MAX_SIZE; a < mIslandContext.mCounts.articulations; ++a, startIdx += DY_ARTICULATION_MAX_SIZE)
			{
				//Compact pairs...
				const PxU32 nbInternalConstraints = artics[a].numInternalConstraints;
				const PxU32 endIdx = startIdx + nbInternalConstraints;

				for (PxU32 b = startIdx; b < endIdx; ++b)
				{
					currDesc[nbArticConstraints++] = currDesc[b];
				}
			}
		}

		PxU32 totalDescCount = mThreadContext.contactDescArraySize;

		mThreadContext.mConstraintsPerPartition.forceSize_Unsafe(0);
		mThreadContext.mConstraintsPerPartition.resize(1);
		mThreadContext.mConstraintsPerPartition[0] = 0;

		TGSConstraintPartitionArgs args;
		args.enhancedDeterminism = false;
		args.mBodies = mSolverBodyData;
		args.mArticulationPtrs = artics;
		args.mContactConstraintDescriptors = mContactDescPtr;
		args.mNumArticulationPtrs = mThreadContext.getArticulations().size();
		args.mNumBodies = mIslandContext.mCounts.bodies;
		args.mNumContactConstraintDescriptors = totalDescCount;
		args.mOrderedContactConstraintDescriptors = mIslandContext.mObjects.orderedConstraintDescs;
		args.mTempContactConstraintDescriptors = mIslandContext.mObjects.tempConstraintDescs;
		args.mNumDifferentBodyConstraints = args.mNumSelfConstraints = args.mNumSelfConstraintBlocks = 0;
		args.mConstraintsPerPartition = &mThreadContext.mConstraintsPerPartition;
		args.mBitField = &mThreadContext.mPartitionNormalizationBitmap;

		//Partition constraints
		mThreadContext.mMaxPartitions = partitionContactConstraintsStep(args);
		mThreadContext.mNumDifferentBodyConstraints = args.mNumDifferentBodyConstraints;
		mThreadContext.mNumSelfConstraints = args.mNumSelfConstraints;
		mThreadContext.mNumSelfConstraintBlocks = args.mNumSelfConstraintBlocks;


		{
			PxU32 descCount = mThreadContext.mNumDifferentBodyConstraints;
			PxU32 selfConstraintDescCount = mThreadContext.contactDescArraySize - mThreadContext.mNumDifferentBodyConstraints;

			Ps::Array<PxU32>& accumulatedConstraintsPerPartition = mThreadContext.mConstraintsPerPartition;

			PxU32 numHeaders = 0;
			PxU32 currentPartition = 0;
			PxU32 maxJ = descCount == 0 ? 0 : accumulatedConstraintsPerPartition[0];

			const PxU32 maxBatchPartition = 0xFFFFFFFF;

			const PxU32 maxBatchSize = args.enhancedDeterminism ? 1u : 4u;

			PxConstraintBatchHeader* batchHeaders = mIslandContext.mObjects.constraintBatchHeaders;

			PxTGSSolverConstraintDesc* orderedConstraints = mIslandContext.mObjects.orderedConstraintDescs;

			PxU32 headersPerPartition = 0;
			for (PxU32 a = 0; a < descCount;)
			{


				PxU32 loopMax = PxMin(maxJ - a, maxBatchSize);
				PxU16 j = 0;
				if (loopMax > 0)
				{
					PxConstraintBatchHeader& header = batchHeaders[numHeaders++];

					j = 1;
					PxTGSSolverConstraintDesc& desc = orderedConstraints[a];
					if (!isArticulationConstraint(desc) && (desc.constraintLengthOver16 == DY_SC_TYPE_RB_CONTACT ||
						desc.constraintLengthOver16 == DY_SC_TYPE_RB_1D) && currentPartition < maxBatchPartition)
					{
						for (; j < loopMax && desc.constraintLengthOver16 == orderedConstraints[a + j].constraintLengthOver16 &&
							!isArticulationConstraint(orderedConstraints[a + j]); ++j);
					}
					header.mStartIndex = a;
					header.mStride = j;
					header.mConstraintType = desc.constraintLengthOver16;
					headersPerPartition++;
				}
				if (maxJ == (a + j) && maxJ != descCount)
				{
					//Go to next partition!
					accumulatedConstraintsPerPartition[currentPartition] = headersPerPartition;
					headersPerPartition = 0;
					currentPartition++;
					maxJ = accumulatedConstraintsPerPartition[currentPartition];
				}
				a += j;
			}
			if (descCount)
				accumulatedConstraintsPerPartition[currentPartition] = headersPerPartition;



			accumulatedConstraintsPerPartition.forceSize_Unsafe(mThreadContext.mMaxPartitions);

			PxU32 numDifferentBodyBatchHeaders = numHeaders;

			for (PxU32 a = 0; a < selfConstraintDescCount; ++a)
			{
				PxConstraintBatchHeader& header = batchHeaders[numHeaders++];
				header.mStartIndex = a + descCount;
				header.mStride = 1;
				header.mConstraintType = DY_SC_TYPE_EXT_1D;
			}

			PxU32 numSelfConstraintBatchHeaders = numHeaders - numDifferentBodyBatchHeaders;

			mThreadContext.numDifferentBodyBatchHeaders = numDifferentBodyBatchHeaders;
			mThreadContext.numSelfConstraintBatchHeaders = numSelfConstraintBatchHeaders;
			mThreadContext.numContactConstraintBatches = numHeaders;
		}

	}
};

PX_FORCE_INLINE PxU32 getConstraintLength(const PxTGSSolverConstraintDesc& desc)
{
	return PxU32(desc.constraintLengthOver16 << 4);
}

class ParallelSolveTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	const SolverIslandObjectsStep& mObjects;
	const PxsIslandIndices& mCounts;
	ThreadContext& mThreadContext;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(ParallelSolveTask)

public:

	ParallelSolveTask(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& threadContext,
		DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mObjects(objects), mCounts(counts), mThreadContext(threadContext), mContext(context)
	{
	}

	virtual const char* getName() const { return "ParallelSolveTask"; }

	virtual void runInternal()
	{
		mContext.iterativeSolveIslandParallel(mObjects, mCounts, mThreadContext, mIslandContext.mStepDt, mIslandContext.mPosIters, mIslandContext.mVelIters,
			&mIslandContext.mSharedSolverIndex, &mIslandContext.mSharedRigidBodyIndex, &mIslandContext.mSharedArticulationIndex,
			&mIslandContext.mSolvedCount, &mIslandContext.mRigidBodyIntegratedCount, &mIslandContext.mArticulationIntegratedCount,
			4, 128);
	}
};


class SolveIslandTask : public Cm::Task
{
	IslandContextStep& mIslandContext;
	const SolverIslandObjectsStep& mObjects;
	const PxsIslandIndices& mCounts;
	ThreadContext& mThreadContext;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(SolveIslandTask)

public:

	SolveIslandTask(IslandContextStep& islandContext, const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& threadContext,
		DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mIslandContext(islandContext), mObjects(objects), mCounts(counts), mThreadContext(threadContext), mContext(context)
	{
	}

	virtual const char* getName() const { return "SolveIslandTask"; }

	virtual void runInternal()
	{

		PxU32 j = 0, i = 0;

		PxU32 numBatches = 0;

		PxU32 currIndex = 0;
		PxU32 totalCount = 0;

		PxTGSSolverConstraintDesc* contactDescBegin = mObjects.orderedConstraintDescs;
		PxTGSSolverConstraintDesc* contactDescPtr = contactDescBegin;
		PxConstraintBatchHeader* headers = mObjects.constraintBatchHeaders;

		PxU32 totalPartitions = 0;
		for (PxU32 a = 0; a < mThreadContext.mConstraintsPerPartition.size(); ++a)
		{
			PxU32 endIndex = currIndex + mThreadContext.mConstraintsPerPartition[a];

			PxU32 numBatchesInPartition = 0;
			for (PxU32 b = currIndex; b < endIndex; ++b)
			{
				PxConstraintBatchHeader& _header = headers[b];
				PxU16 stride = _header.mStride, newStride = _header.mStride;
				PxU32 startIndex = j;
				for (PxU16 c = 0; c < stride; ++c)
				{
					if (getConstraintLength(contactDescBegin[i]) == 0)
					{
						newStride--;
						i++;
					}
					else
					{
						if (i != j)
							contactDescBegin[j] = contactDescBegin[i];
						i++;
						j++;
						contactDescPtr++;
					}
				}

				if (newStride != 0)
				{
					headers[numBatches].mStartIndex = startIndex;
					headers[numBatches].mStride = newStride;
					PxU8 type = *contactDescBegin[startIndex].constraint;
					if (type == DY_SC_TYPE_STATIC_CONTACT)
					{
						//Check if any block of constraints is classified as type static (single) contact constraint.
						//If they are, iterate over all constraints grouped with it and switch to "dynamic" contact constraint
						//type if there's a dynamic contact constraint in the group.
						for (PxU32 c = 1; c < newStride; ++c)
						{
							if (*contactDescBegin[startIndex + c].constraint == DY_SC_TYPE_RB_CONTACT)
							{
								type = DY_SC_TYPE_RB_CONTACT;
							}
						}
					}

					headers[numBatches].mConstraintType = type;
					numBatches++;
					numBatchesInPartition++;
				}
			}
			currIndex += mThreadContext.mConstraintsPerPartition[a];
			mThreadContext.mConstraintsPerPartition[totalPartitions] = numBatchesInPartition;
			if (numBatchesInPartition)
				totalPartitions++;
			totalCount += numBatchesInPartition;
		}

		currIndex = totalCount;
		//Decision whether to spawn multi-threaded solver or single-threaded solver...

		mThreadContext.mConstraintsPerPartition.forceSize_Unsafe(totalPartitions);
		mThreadContext.numContactConstraintBatches = totalCount;

		PxU32 maxLinks = 0;
		for (PxU32 a = 0; a < mCounts.articulations; ++a)
		{
			ArticulationSolverDesc& desc = mThreadContext.getArticulations()[a];
			maxLinks = PxMax(maxLinks, PxU32(desc.linkCount));
		}

		mThreadContext.mZVector.forceSize_Unsafe(0);
		mThreadContext.mZVector.reserve(maxLinks);
		mThreadContext.mZVector.forceSize_Unsafe(maxLinks);

		mThreadContext.mDeltaV.forceSize_Unsafe(0);
		mThreadContext.mDeltaV.reserve(maxLinks);
		mThreadContext.mDeltaV.forceSize_Unsafe(maxLinks);

		SolverContext cache;
		cache.Z = mThreadContext.mZVector.begin();
		cache.deltaV = mThreadContext.mDeltaV.begin();

		if (mThreadContext.mConstraintsPerPartition.size())
		{
			const PxU32 threadCount = this->getTaskManager()->getCpuDispatcher()->getWorkerCount();

			const PxU32 nbHeadersPerPartition = ((currIndex + mThreadContext.mConstraintsPerPartition.size() - 1) / mThreadContext.mConstraintsPerPartition.size());

			const PxU32 NbBatchesPerThread = 8;

			const PxU32 nbIdealThreads = (nbHeadersPerPartition + NbBatchesPerThread-1) / NbBatchesPerThread;

			if (threadCount < 2 || nbIdealThreads < 2)
				mContext.iterativeSolveIsland(mObjects, mCounts, mThreadContext, mIslandContext.mStepDt, mIslandContext.mInvStepDt, 
					mIslandContext.mPosIters, mIslandContext.mVelIters, cache);
			else
			{

				mIslandContext.mSharedSolverIndex = 0;
				mIslandContext.mSolvedCount = 0;
				mIslandContext.mSharedRigidBodyIndex = 0;
				mIslandContext.mRigidBodyIntegratedCount = 0;
				mIslandContext.mSharedArticulationIndex = 0;
				mIslandContext.mArticulationIntegratedCount = 0;

				PxU32 nbThreads = PxMin(threadCount, nbIdealThreads);

				ParallelSolveTask* tasks = reinterpret_cast<ParallelSolveTask*>(mContext.getTaskPool().allocate(sizeof(ParallelSolveTask)*nbThreads));

				for (PxU32 a = 0; a < nbThreads; ++a)
				{
					PX_PLACEMENT_NEW(&tasks[a], ParallelSolveTask)(mIslandContext, mObjects, mCounts, mThreadContext, mContext);
					tasks[a].setContinuation(mCont);
					tasks[a].removeReference();
				}
			}
		}
		else
		{
			mContext.iterativeSolveIsland(mObjects, mCounts, mThreadContext, mIslandContext.mStepDt, 
				mIslandContext.mInvStepDt, mIslandContext.mPosIters, mIslandContext.mVelIters, cache);
		}
	}
};

class EndIslandTask : public Cm::Task
{
	ThreadContext& mThreadContext;
	DynamicsTGSContext& mContext;

	PX_NOCOPY(EndIslandTask)

public:

	EndIslandTask(ThreadContext& threadContext, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mThreadContext(threadContext), mContext(context)
	{
	}

	virtual const char* getName() const { return "EndIslandTask"; }

	virtual void runInternal()
	{
		mContext.endIsland(mThreadContext);
	}
};

class FinishSolveIslandTask : public Cm::Task
{
	ThreadContext& mThreadContext;
	const SolverIslandObjectsStep& mObjects;
	const PxsIslandIndices& mCounts;
	IG::SimpleIslandManager& mIslandManager;

	DynamicsTGSContext& mContext;

	PX_NOCOPY(FinishSolveIslandTask)

public:

	FinishSolveIslandTask(ThreadContext& threadContext, const SolverIslandObjectsStep& objects,
		const PxsIslandIndices& counts, IG::SimpleIslandManager& islandManager, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mThreadContext(threadContext), mObjects(objects), mCounts(counts), mIslandManager(islandManager), mContext(context)
	{
	}

	virtual const char* getName() const { return "FinishSolveIslandTask"; }

	virtual void runInternal()
	{
		mContext.finishSolveIsland(mThreadContext, mObjects, mCounts, mIslandManager, mCont);
	}
};



void DynamicsTGSContext::iterativeSolveIsland(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
	const PxReal stepDt, const PxReal invStepDt, const PxU32 posIters, const PxU32 velIters, SolverContext& cache)
{
	PX_PROFILE_ZONE("Dynamics:solveIsland", 0);
	PxReal elapsedTime = 0.f;
	PxReal recipStepDt = 1.f/stepDt;

	const PxU32 bodyOffset = objects.solverBodyOffset;

	if (mThreadContext.numContactConstraintBatches == 0)
	{
		for (PxU32 i = 0; i < counts.articulations; ++i)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[i];
			for (PxU32 a = 1; a < posIters; a++)
			{
				d.articulation->solveInternalConstraints(stepDt, recipStepDt, mThreadContext.mZVector.begin(), mThreadContext.mDeltaV.begin(), false);
				stepArticulations(mThreadContext, counts, stepDt);
			}

			ArticulationPImpl::saveVelocityTGS(d, mInvDt);

			for (PxU32 a = 0; a < velIters; ++a)
			{
				d.articulation->solveInternalConstraints(stepDt, recipStepDt, mThreadContext.mZVector.begin(), mThreadContext.mDeltaV.begin(), true);
			}
		}

		integrateBodies(objects, counts.bodies, mSolverBodyVelPool.begin() + bodyOffset, mSolverBodyTxInertiaPool.begin() + bodyOffset, mSolverBodyDataPool2.begin() + bodyOffset, mDt);
		return;
	}


	for (PxU32 a = 1; a < posIters; a++)
	{
		bool doFriction = true;

		for (PxU32 i = 0; i < counts.articulations; ++i)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[i];
			d.articulation->solveInternalConstraints(stepDt, recipStepDt, mThreadContext.mZVector.begin(), mThreadContext.mDeltaV.begin(), false);
		}

		solveConstraintsIteration(objects.orderedConstraintDescs, objects.constraintBatchHeaders, mThreadContext.numContactConstraintBatches, doFriction, invStepDt,
			mSolverBodyTxInertiaPool.begin(), elapsedTime, -PX_MAX_F32, cache);
		integrateBodies(objects, counts.bodies, mSolverBodyVelPool.begin() + bodyOffset, mSolverBodyTxInertiaPool.begin() + bodyOffset, mSolverBodyDataPool2.begin() + bodyOffset, stepDt);

		stepArticulations(mThreadContext, counts, stepDt);
		elapsedTime += stepDt;
	}

	for (PxU32 i = 0; i < counts.articulations; ++i)
	{
		ArticulationSolverDesc& d = mThreadContext.getArticulations()[i];
		d.articulation->solveInternalConstraints(stepDt, recipStepDt, mThreadContext.mZVector.begin(), mThreadContext.mDeltaV.begin(), false);
	}

	solveConcludeConstraintsIteration(objects.orderedConstraintDescs, objects.constraintBatchHeaders, mThreadContext.numContactConstraintBatches,
		mSolverBodyTxInertiaPool.begin(), elapsedTime, cache);
	elapsedTime += stepDt;

	const PxReal invDt = mInvDt;

	integrateBodies(objects, counts.bodies, mSolverBodyVelPool.begin() + bodyOffset, mSolverBodyTxInertiaPool.begin() + bodyOffset, mSolverBodyDataPool2.begin() + bodyOffset, stepDt);

	stepArticulations(mThreadContext, counts, stepDt);

	for(PxU32 a = 0; a < counts.articulations; ++a)
	{
		Dy::ArticulationSolverDesc& desc = mThreadContext.getArticulations()[a];

		//ArticulationPImpl::updateDeltaMotion(desc, stepDt, mThreadContext.mDeltaV.begin());
		ArticulationPImpl::saveVelocityTGS(desc, invDt);
	}

	for (PxU32 a = 0; a < velIters; ++a)
	{
		for (PxU32 i = 0; i < counts.articulations; ++i)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[i];
			d.articulation->solveInternalConstraints(stepDt, recipStepDt, mThreadContext.mZVector.begin(), mThreadContext.mDeltaV.begin(), false);
		}
		bool doFriction = true;
		solveConstraintsIteration(objects.orderedConstraintDescs, objects.constraintBatchHeaders, mThreadContext.numContactConstraintBatches, doFriction, invStepDt,
			mSolverBodyTxInertiaPool.begin(), elapsedTime, 0.f, cache);
	}

	writebackConstraintsIteration(objects.constraintBatchHeaders, objects.orderedConstraintDescs, mThreadContext.numContactConstraintBatches);
}


void DynamicsTGSContext::iterativeSolveIslandParallel(const SolverIslandObjectsStep& objects, const PxsIslandIndices& counts, ThreadContext& mThreadContext,
	const PxReal stepDt, const PxU32 posIters, const PxU32 velIters, PxI32* solverCounts, PxI32* integrationCounts, PxI32* articulationIntegrationCounts,
	PxI32* solverProgressCount, PxI32* integrationProgressCount, PxI32* articulationProgressCount, PxU32 solverUnrollSize, PxU32 integrationUnrollSize)
{
	PX_PROFILE_ZONE("Dynamics:solveIslandParallel", 0);
	Dy::ThreadContext& threadContext = *getThreadContext();
	PxU32 startSolveIdx = PxU32(Ps::atomicAdd(solverCounts, PxI32(solverUnrollSize))) - solverUnrollSize;
	PxU32 nbSolveRemaining = solverUnrollSize;

	PxU32 startIntegrateIdx = PxU32(Ps::atomicAdd(integrationCounts, PxI32(integrationUnrollSize))) - integrationUnrollSize;
	PxU32 nbIntegrateRemaining = integrationUnrollSize;

	//For now, just do articulations 1 at a time. Might need to tweak this later depending on performance
	PxU32 startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;

	PxU32 targetSolverProgressCount = 0, targetIntegrationProgressCount = 0, targetArticulationProgressCount = 0;

	const PxU32 nbSolverBatches = mThreadContext.numContactConstraintBatches;
	const PxU32 nbBodies = counts.bodies;// + mKinematicCount;
	const PxU32 nbArticulations = counts.articulations;

	PxTGSSolverConstraintDesc* contactDescs = objects.orderedConstraintDescs;
	PxConstraintBatchHeader* batchHeaders = objects.constraintBatchHeaders;
	
	PxTGSSolverBodyVel* solverVels = mSolverBodyVelPool.begin();
	PxTGSSolverBodyTxInertia* solverTxInertias = mSolverBodyTxInertiaPool.begin();
	const PxStepSolverBodyData*const solverBodyData = mSolverBodyDataPool2.begin();

	PxU32* constraintsPerPartitions = mThreadContext.mConstraintsPerPartition.begin();
	const PxU32 nbPartitions = mThreadContext.mConstraintsPerPartition.size();

	const PxU32 bodyOffset = objects.solverBodyOffset;
	
	SolverContext cache;
	cache.Z = threadContext.mZVector.begin();
	cache.deltaV = threadContext.mDeltaV.begin();

	PxReal elapsedTime = 0.f;

	PxReal invStepDt = 1.f/ stepDt;

	for (PxU32 a = 1; a < posIters; ++a, targetIntegrationProgressCount += nbBodies, targetArticulationProgressCount += nbArticulations)
	{
		WAIT_FOR_PROGRESS(integrationProgressCount, PxI32(targetIntegrationProgressCount));
		WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));

		PxU32 artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;

		PxU32 nbArticsProcessed = 0;
		while (artIcStartIdx < nbArticulations)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[artIcStartIdx];

			d.articulation->solveInternalConstraints(stepDt, invStepDt, threadContext.mZVector.begin(),
				threadContext.mDeltaV.begin(), false);
			ArticulationPImpl::updateDeltaMotion(d, stepDt, cache.deltaV);

			nbArticsProcessed++;

			startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;
			artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;
		}

		if (nbArticsProcessed)
			Ps::atomicAdd(articulationProgressCount, PxI32(nbArticsProcessed));

		targetArticulationProgressCount += nbArticulations;

		WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));


		PxU32 offset = 0;
		for (PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));
			//Find the startIdx in the partition to process
			PxU32 startIdx = startSolveIdx - targetSolverProgressCount;

			const PxU32 nbBatches = constraintsPerPartitions[b];

			PxU32 nbSolved = 0;
			while (startIdx < nbBatches)
			{
				PxU32 nbToSolve = PxMin(nbBatches - startIdx, nbSolveRemaining);
				parallelSolveConstraints(contactDescs, batchHeaders + startIdx + offset, nbToSolve, true,
					solverTxInertias, elapsedTime, -PX_MAX_F32, cache);
				nbSolveRemaining -= nbToSolve;
				startSolveIdx += nbToSolve;
				startIdx += nbToSolve;

				nbSolved += nbToSolve;

				if (nbSolveRemaining == 0)
				{
					startSolveIdx = PxU32(Ps::atomicAdd(solverCounts, PxI32(solverUnrollSize))) - solverUnrollSize;
					nbSolveRemaining = solverUnrollSize;
					startIdx = startSolveIdx - targetSolverProgressCount;
				}
			}

			if (nbSolved)
				Ps::atomicAdd(solverProgressCount, PxI32(nbSolved));

			targetSolverProgressCount += nbBatches;
			offset += nbBatches;
		}

		WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));


		PxU32 integStartIdx = startIntegrateIdx - targetIntegrationProgressCount;

		PxU32 nbIntegrated = 0;
		while (integStartIdx < nbBodies)
		{
			PxU32 nbToIntegrate = PxMin(nbBodies - integStartIdx, nbIntegrateRemaining);

			parallelIntegrateBodies(solverVels + integStartIdx + bodyOffset, solverTxInertias + integStartIdx + bodyOffset,
				solverBodyData + integStartIdx + bodyOffset, nbToIntegrate, stepDt);

			nbIntegrateRemaining -= nbToIntegrate;
			startIntegrateIdx += nbToIntegrate;
			integStartIdx += nbToIntegrate;

			nbIntegrated += nbToIntegrate;

			if (nbIntegrateRemaining == 0)
			{
				startIntegrateIdx = PxU32(Ps::atomicAdd(integrationCounts, PxI32(integrationUnrollSize))) - integrationUnrollSize;
				nbIntegrateRemaining = integrationUnrollSize;
				integStartIdx = startIntegrateIdx - targetIntegrationProgressCount;
			}
		}

		if (nbIntegrated)
			Ps::atomicAdd(integrationProgressCount, PxI32(nbIntegrated));

		artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;

		nbArticsProcessed = 0;
		while (artIcStartIdx < nbArticulations)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[artIcStartIdx];
			ArticulationPImpl::updateDeltaMotion(d, stepDt, cache.deltaV);

			nbArticsProcessed++;

			startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;
			artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;
		}

		if (nbArticsProcessed)
			Ps::atomicAdd(articulationProgressCount, PxI32(nbArticsProcessed));

		elapsedTime += stepDt;

	}

	{
		WAIT_FOR_PROGRESS(integrationProgressCount, PxI32(targetIntegrationProgressCount));
		WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));

		PxU32 artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;

		PxU32 nbArticsProcessed = 0;
		while (artIcStartIdx < nbArticulations)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[artIcStartIdx];

			d.articulation->solveInternalConstraints(stepDt, invStepDt, threadContext.mZVector.begin(),
				threadContext.mDeltaV.begin(), false);
			ArticulationPImpl::updateDeltaMotion(d, stepDt, cache.deltaV);

			nbArticsProcessed++;

			startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;
			artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;
		}

		if (nbArticsProcessed)
			Ps::atomicAdd(articulationProgressCount, PxI32(nbArticsProcessed));

		targetArticulationProgressCount += nbArticulations;

		WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));


		PxU32 offset = 0;
		for (PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));
			//Find the startIdx in the partition to process
			PxU32 startIdx = startSolveIdx - targetSolverProgressCount;

			const PxU32 nbBatches = constraintsPerPartitions[b];

			PxU32 nbSolved = 0;
			while (startIdx < nbBatches)
			{
				PxU32 nbToSolve = PxMin(nbBatches - startIdx, nbSolveRemaining);
				solveConcludeConstraintsIteration(contactDescs, batchHeaders + startIdx + offset, nbToSolve,
					solverTxInertias, elapsedTime, cache);
				nbSolveRemaining -= nbToSolve;
				startSolveIdx += nbToSolve;
				startIdx += nbToSolve;

				nbSolved += nbToSolve;

				if (nbSolveRemaining == 0)
				{
					startSolveIdx = PxU32(Ps::atomicAdd(solverCounts, PxI32(solverUnrollSize))) - solverUnrollSize;
					nbSolveRemaining = solverUnrollSize;
					startIdx = startSolveIdx - targetSolverProgressCount;
				}
			}

			if (nbSolved)
				Ps::atomicAdd(solverProgressCount, PxI32(nbSolved));

			targetSolverProgressCount += nbBatches;
			offset += nbBatches;

			
		}

		WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));


		const PxReal invDt = mInvDt;
		//const PxReal invDtPt25 = mInvDt * 0.25f;
		PxU32 integStartIdx = startIntegrateIdx - targetIntegrationProgressCount;

		PxU32 nbIntegrated = 0;
		while (integStartIdx < nbBodies)
		{
			PxU32 nbToIntegrate = PxMin(nbBodies - integStartIdx, nbIntegrateRemaining);

			parallelIntegrateBodies(solverVels + integStartIdx + bodyOffset, solverTxInertias + integStartIdx + bodyOffset,
				solverBodyData + integStartIdx + bodyOffset, nbToIntegrate, stepDt);

			nbIntegrateRemaining -= nbToIntegrate;
			startIntegrateIdx += nbToIntegrate;
			integStartIdx += nbToIntegrate;

			nbIntegrated += nbToIntegrate;

			if (nbIntegrateRemaining == 0)
			{
				startIntegrateIdx = PxU32(Ps::atomicAdd(integrationCounts, PxI32(integrationUnrollSize))) - integrationUnrollSize;
				nbIntegrateRemaining = integrationUnrollSize;
				integStartIdx = startIntegrateIdx - targetIntegrationProgressCount;
			}
		}

		if (nbIntegrated)
			Ps::atomicAdd(integrationProgressCount, PxI32(nbIntegrated));

		artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;

		nbArticsProcessed = 0;
		while (artIcStartIdx < nbArticulations)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[artIcStartIdx];
			ArticulationPImpl::updateDeltaMotion(d, stepDt, cache.deltaV);
			ArticulationPImpl::saveVelocityTGS(d, invDt);

			nbArticsProcessed++;

			startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;
			artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;
		}

		if (nbArticsProcessed)
			Ps::atomicAdd(articulationProgressCount, PxI32(nbArticsProcessed));

		elapsedTime += stepDt;

		targetIntegrationProgressCount += nbBodies;
		targetArticulationProgressCount += nbArticulations;

		putThreadContext(&threadContext);
	}

	//Write back constraints...

	WAIT_FOR_PROGRESS(integrationProgressCount, PxI32(targetIntegrationProgressCount));
	WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));


	for (PxU32 a = 0; a < velIters; ++a)
	{
		WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));

		PxU32 artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;

		PxU32 nbArticsProcessed = 0;
		while (artIcStartIdx < nbArticulations)
		{
			ArticulationSolverDesc& d = mThreadContext.getArticulations()[artIcStartIdx];

			d.articulation->solveInternalConstraints(stepDt, invStepDt, threadContext.mZVector.begin(),
				threadContext.mDeltaV.begin(), false);
			ArticulationPImpl::updateDeltaMotion(d, stepDt, cache.deltaV);

			nbArticsProcessed++;

			startArticulationIdx = PxU32(Ps::atomicAdd(articulationIntegrationCounts, PxI32(1))) - 1;
			artIcStartIdx = startArticulationIdx - targetArticulationProgressCount;
		}

		if (nbArticsProcessed)
			Ps::atomicAdd(articulationProgressCount, PxI32(nbArticsProcessed));

		targetArticulationProgressCount += nbArticulations;

		WAIT_FOR_PROGRESS(articulationProgressCount, PxI32(targetArticulationProgressCount));

		PxU32 offset = 0;
		for (PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));

			//Find the startIdx in the partition to process
			PxU32 startIdx = startSolveIdx - targetSolverProgressCount;

			const PxU32 nbBatches = constraintsPerPartitions[b];

			PxU32 nbSolved = 0;
			while (startIdx < nbBatches)
			{
				PxU32 nbToSolve = PxMin(nbBatches - startIdx, nbSolveRemaining);
				parallelSolveConstraints(contactDescs, batchHeaders + startIdx + offset, nbToSolve, true,
					solverTxInertias, elapsedTime, 0.f, cache);
				nbSolveRemaining -= nbToSolve;
				startSolveIdx += nbToSolve;
				startIdx += nbToSolve;

				nbSolved += nbToSolve;


				if (nbSolveRemaining == 0)
				{
					startSolveIdx = PxU32(Ps::atomicAdd(solverCounts, PxI32(solverUnrollSize))) - solverUnrollSize;
					nbSolveRemaining = solverUnrollSize;

					startIdx = startSolveIdx - targetSolverProgressCount;
				}
			}

			if (nbSolved)
				Ps::atomicAdd(solverProgressCount, PxI32(nbSolved));

			targetSolverProgressCount += nbBatches;
			offset += nbBatches;
		}
	}

	{
		WAIT_FOR_PROGRESS(solverProgressCount, PxI32(targetSolverProgressCount));
		{
			//Find the startIdx in the partition to process
			PxU32 startIdx = startSolveIdx - targetSolverProgressCount;

			const PxU32 nbBatches = nbSolverBatches;

			PxU32 nbSolved = 0;
			while (startIdx < nbBatches)
			{
				PxU32 nbToSolve = PxMin(nbBatches - startIdx, nbSolveRemaining);
				parallelWritebackConstraintsIteration(contactDescs, batchHeaders + startIdx, nbToSolve);
				nbSolveRemaining -= nbToSolve;
				startSolveIdx += nbToSolve;
				startIdx += nbToSolve;

				nbSolved += nbToSolve;

				if (nbSolveRemaining == 0)
				{
					startSolveIdx = PxU32(Ps::atomicAdd(solverCounts, PxI32(solverUnrollSize))) - solverUnrollSize;
					nbSolveRemaining = solverUnrollSize;
					startIdx = startSolveIdx - targetSolverProgressCount;
				}
			}

			if (nbSolved)
				Ps::atomicAdd(solverProgressCount, PxI32(nbSolved));

			targetSolverProgressCount += nbBatches;
		}

	}

}

class CopyBackTask : public Cm::Task
{
	const SolverIslandObjectsStep& mObjects;
	PxTGSSolverBodyVel* mVels;
	PxTGSSolverBodyTxInertia* mTxInertias;
	PxStepSolverBodyData* mSolverBodyDatas;
	const PxReal mInvDt;
	IG::IslandSim& mIslandSim;
	const PxU32 mStartIdx;
	const PxU32 mEndIdx;
	DynamicsTGSContext& mContext;

	PX_NOCOPY(CopyBackTask)

public:

	CopyBackTask(const SolverIslandObjectsStep& objects,
		PxTGSSolverBodyVel* vels, PxTGSSolverBodyTxInertia* txInertias,
		PxStepSolverBodyData* solverBodyDatas, PxReal invDt, IG::IslandSim& islandSim,
		PxU32 startIdx, PxU32 endIdx, DynamicsTGSContext& context) : Cm::Task(context.getContextId()),
		mObjects(objects), mVels(vels), mTxInertias(txInertias), mSolverBodyDatas(solverBodyDatas), mInvDt(invDt),
		mIslandSim(islandSim), mStartIdx(startIdx), mEndIdx(endIdx), mContext(context)
	{
	}

	virtual const char* getName() const { return "CopyBackTask"; }

	virtual void runInternal()
	{
		mContext.copyBackBodies(mObjects, mVels, mTxInertias, mSolverBodyDatas, mInvDt, mIslandSim, mStartIdx, mEndIdx);
	}
};

class UpdateArticTask : public Cm::Task
{
	Dy::ThreadContext& mThreadContext;
	PxU32 mStartIdx;
	PxU32 mEndIdx;
	PxReal mDt;
	DynamicsTGSContext& mContext;

	PX_NOCOPY(UpdateArticTask)

public:

	UpdateArticTask(Dy::ThreadContext& threadContext, PxU32 startIdx, PxU32 endIdx,	PxReal dt, DynamicsTGSContext& context) :
		Cm::Task(context.getContextId()),
		mThreadContext(threadContext), mStartIdx(startIdx), mEndIdx(endIdx), mDt(dt), mContext(context)
	{
	}

	virtual const char* getName() const { return "UpdateArticTask"; }

	virtual void runInternal()
	{
		mContext.updateArticulations(mThreadContext, mStartIdx, mEndIdx, mDt);
	}
};


void DynamicsTGSContext::finishSolveIsland(ThreadContext& mThreadContext, const SolverIslandObjectsStep& objects,
	const PxsIslandIndices& counts, IG::SimpleIslandManager& islandManager, PxBaseTask* continuation)
{
	mThreadContext.mConstraintBlockManager.reset();
	mThreadContext.mConstraintBlockStream.reset();

	const PxU32 NbBodiesPerTask = 512;

	for (PxU32 a = 0; a < counts.bodies; a += NbBodiesPerTask)
	{
		CopyBackTask* task = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(CopyBackTask)), CopyBackTask)
			(objects, mSolverBodyVelPool.begin() + objects.solverBodyOffset,
				mSolverBodyTxInertiaPool.begin() + objects.solverBodyOffset, mSolverBodyDataPool2.begin() + objects.solverBodyOffset,
				mInvDt, islandManager.getAccurateIslandSim(), a, PxMin(a + NbBodiesPerTask, counts.bodies),*this);

		task->setContinuation(continuation);
		task->removeReference();
	}

	const PxU32 NbArticsPerTask = 2;

	for (PxU32 a = 0; a < counts.articulations; a += NbArticsPerTask)
	{
		UpdateArticTask* task = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(UpdateArticTask)), UpdateArticTask)
			(mThreadContext, a, PxMin(counts.articulations, a+NbArticsPerTask), mDt, *this);

		task->setContinuation(continuation);
		task->removeReference();
	}
}


void DynamicsTGSContext::endIsland(ThreadContext& mThreadContext)
{
	putThreadContext(&mThreadContext);
}


void DynamicsTGSContext::solveIsland(const SolverIslandObjectsStep& objects,
	const PxsIslandIndices& counts,
	const PxU32 solverBodyOffset,
	IG::SimpleIslandManager& islandManager,
	PxU32* bodyRemapTable, PxsMaterialManager* materialManager,
	PxsContactManagerOutputIterator& iterator,
	PxBaseTask* continuation)
{
	PX_UNUSED(continuation);

	PX_UNUSED(iterator);
	PX_UNUSED(materialManager);
	PX_UNUSED(bodyRemapTable);
	PX_UNUSED(islandManager);

	ThreadContext& mThreadContext = *getThreadContext();

	IslandContextStep& islandContext = *reinterpret_cast<IslandContextStep*>(mTaskPool.allocate(sizeof(IslandContextStep)));
	islandContext.mThreadContext = &mThreadContext;
	islandContext.mCounts = counts;
	islandContext.mObjects = objects;
	islandContext.mPosIters = 0;
	islandContext.mVelIters = 0;
	islandContext.mObjects.solverBodyOffset = solverBodyOffset;


	prepareBodiesAndConstraints(islandContext.mObjects, islandManager, islandContext);

	////Create task chain...
	SetupDescsTask* descTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SetupDescsTask)), SetupDescsTask)(islandContext, islandContext.mObjects, islandManager,
		bodyRemapTable, solverBodyOffset, iterator, *this);
	PreIntegrateTask* intTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(PreIntegrateTask)), PreIntegrateTask)(islandContext.mObjects.bodyCoreArray, islandContext.mObjects.bodies,
		mSolverBodyVelPool.begin() + solverBodyOffset,
		mSolverBodyTxInertiaPool.begin() + solverBodyOffset, mSolverBodyDataPool2.begin() + solverBodyOffset,
		mThreadContext.mNodeIndexArray, islandContext.mCounts.bodies, mGravity, mDt, islandContext.mPosIters,
		islandContext.mVelIters, *this);
	SetupArticulationTask* articTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SetupArticulationTask)), SetupArticulationTask)(islandContext, mGravity, 
		mDt, islandContext.mPosIters, islandContext.mVelIters, *this);
	SetStepperTask* stepperTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SetStepperTask)), SetStepperTask)(islandContext, *this);
	SetupArticulationInternalConstraintsTask* articConTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SetupArticulationInternalConstraintsTask)), SetupArticulationInternalConstraintsTask)
		(islandContext, mDt, mInvDt, islandContext.mObjects.constraintDescs, *this);
	PartitionTask* partitionTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(PartitionTask)), PartitionTask)
		(islandContext, islandContext.mObjects.constraintDescs, mSolverBodyVelPool.begin()+solverBodyOffset+1, mThreadContext,
			*this);
	SetupSolverConstraintsTask* constraintTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SetupSolverConstraintsTask)), SetupSolverConstraintsTask)
		(islandContext, islandContext.mObjects.orderedConstraintDescs, iterator, mThreadContext,
			mDt, *this);

	SolveIslandTask* solveTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(SolveIslandTask)), SolveIslandTask)(islandContext, islandContext.mObjects, islandContext.mCounts, mThreadContext, *this);

	FinishSolveIslandTask* finishTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(FinishSolveIslandTask)), FinishSolveIslandTask)(mThreadContext, islandContext.mObjects, islandContext.mCounts, islandManager, *this);

	EndIslandTask* endTask = PX_PLACEMENT_NEW(mTaskPool.allocate(sizeof(EndIslandTask)), EndIslandTask)(mThreadContext, *this);

	endTask->setContinuation(continuation);
	finishTask->setContinuation(endTask);
	solveTask->setContinuation(finishTask);
	constraintTask->setContinuation(solveTask);
	partitionTask->setContinuation(constraintTask);
	articConTask->setContinuation(partitionTask);
	//Stepper triggers both articCon and constraintTask
	stepperTask->setContinuation(articConTask);
	stepperTask->setAdditionalContinuation(constraintTask);

	articTask->setContinuation(stepperTask);
	intTask->setContinuation(stepperTask);
	descTask->setContinuation(stepperTask);

	endTask->removeReference();
	finishTask->removeReference();
	solveTask->removeReference();
	constraintTask->removeReference();
	partitionTask->removeReference();
	articConTask->removeReference();
	stepperTask->removeReference();
	articTask->removeReference();
	intTask->removeReference();
	descTask->removeReference();


	//Launch a setupDescs task
	//setupDescs(islandContext, objects, islandManager, bodyRemapTable, mKinematicCount, iterator);
	//
	////Launch a preIntegrateBodies task
	//preIntegrateBodies(objects.bodyCoreArray, objects.bodies, mThreadContext.solverBodyData.begin() + mKinematicCount,
	//	mThreadContext.mNodeIndexArray, counts.bodies, mGravity, mDt, islandContext.mPosIters, islandContext.mVelIters, 0);

	//const PxU32 articConstraintOffset = mThreadContext.contactDescArraySize;
	//islandContext.mArticulationOffset = mThreadContext.contactDescArraySize;

	////Launch an articulation setup task
	//setupArticulations(islandContext, mGravity, mDt, islandContext.mPosIters, islandContext.mVelIters);

	//PxU32 substeps = islandContext.mPosIters;

	//PxReal stepDt = mDt / PxReal(substeps);
	//PxReal invStepDt = PxMin(1000.f, mInvDt*PxReal(substeps));

	//islandContext.mStepDt = stepDt;
	//islandContext.mInvStepDt = invStepDt;

	////Setup articulation constraints - needs to be refactored for compaction etc.
	//setupArticulationInternalConstraints(islandContext, mDt, invStepDt, objects.constraintDescs + articConstraintOffset);

	//createSolverConstraints(objects.constraintDescs, articConstraintOffset, mThreadContext.solverBodyData.begin(), iterator, mThreadContext,
	//	mDt, invStepDt, substeps);

	//iterativeSolveIsland(objects, counts, mThreadContext, stepDt, invStepDt, islandContext.mPosIters, islandContext.mVelIters);

	//endIsland(mThreadContext, objects, counts, islandManager);
	
}

void DynamicsTGSContext::updateBodyCore(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
}

void DynamicsTGSContext::mergeResults()
{
}



}
}

