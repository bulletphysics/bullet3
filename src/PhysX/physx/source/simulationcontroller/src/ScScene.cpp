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


#define NOMINMAX

#include "common/PxProfileZone.h"
#include "ScPhysics.h"
#include "ScScene.h"
#include "ScClient.h"
#include "BpAABBManager.h"
#include "BpBroadPhase.h"
#include "ScStaticSim.h"
#include "ScConstraintSim.h"
#include "ScConstraintProjectionManager.h"
#include "ScConstraintCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointCore.h"
#include "ScMaterialCore.h"
#include "ScArticulationSim.h"
#include "ScArticulationJointSim.h"
#include "PsTime.h"
#include "ScConstraintInteraction.h"
#include "ScTriggerInteraction.h"
#include "ScSimStats.h"
#include "ScTriggerPairs.h"
#include "ScObjectIDTracker.h"
#include "DyArticulation.h"
#include "PxvManager.h"
#include "PxvGlobals.h"
#include "DyContext.h"
#include "PxsCCD.h"
#include "PxsSimpleIslandManager.h"
#include "PxsSimulationController.h"
#include "PxsContext.h"
#include "ScSqBoundsManager.h"
#include "ScElementSim.h"


#if defined(__APPLE__) && defined(__POWERPC__)
#include <ppc_intrinsics.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#include "PxsKernelWrangler.h"
#include "PxsHeapMemoryAllocator.h"
#include "cudamanager/PxCudaContextManager.h"
#endif

#include "PxsMemoryManager.h"

////////////

#include "PxvNphaseImplementationContext.h"
#include "ScShapeInteraction.h"
#include "ScElementInteractionMarker.h"
#include "PxsContext.h"

#include "PxRigidDynamic.h"

#include "PxvDynamics.h"
#include "DyArticulation.h"
#include "DyFeatherstoneArticulation.h"

using namespace physx;
using namespace physx::shdfnd;
using namespace physx::Cm;
using namespace physx::Dy;

// slightly ugly, but we don't want a compile-time dependency on DY_ARTICULATION_MAX_SIZE in the ScScene.h header
namespace physx { 
#if PX_SUPPORT_GPU_PHYSX

PX_PHYSX_GPU_API Bp::BPMemoryAllocator* createGpuMemoryAllocator();

#endif

namespace Sc {

class LLArticulationPool: public Ps::Pool<Articulation, Ps::AlignedAllocator<DY_ARTICULATION_MAX_SIZE> > 
{
public:
	LLArticulationPool() {}
};

class LLArticulationRCPool : public Ps::Pool<FeatherstoneArticulation, Ps::AlignedAllocator<DY_ARTICULATION_MAX_SIZE> >
{
public:
	LLArticulationRCPool() {}
};


static const char* sFilterShaderDataMemAllocId = "SceneDesc filterShaderData";

}}

void PxcClearContactCacheStats();
void PxcDisplayContactCacheStats();

class ScAfterIntegrationTask :  public Cm::Task
{
public:
	static const PxU32 MaxTasks = 256;
private:
	const IG::NodeIndex* const	mIndices;
	const PxU32					mNumBodies;
	PxsContext*					mContext;
	Context*					mDynamicsContext;
	PxsTransformCache&			mCache;
	Sc::Scene&					mScene;
	
public:

	ScAfterIntegrationTask(const IG::NodeIndex* const indices, PxU32 numBodies, PxsContext* context, Context* dynamicsContext, PxsTransformCache& cache, Sc::Scene& scene) :
		Cm::Task		(scene.getContextId()),
		mIndices		(indices),
		mNumBodies		(numBodies),
		mContext		(context),
		mDynamicsContext(dynamicsContext),
		mCache			(cache),
		mScene			(scene)
	{
	}

	virtual void runInternal()
	{		
		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		Sc::BodySim* bpUpdates[MaxTasks];
		Sc::BodySim* ccdBodies[MaxTasks];
		Sc::BodySim* activateBodies[MaxTasks];
		Sc::BodySim* deactivateBodies[MaxTasks];
		PxU32 nbBpUpdates = 0, nbCcdBodies = 0;

		IG::SimpleIslandManager& manager = *mScene.getSimpleIslandManager();
		const IG::IslandSim& islandSim = manager.getAccurateIslandSim();
		Bp::BoundsArray& boundsArray = mScene.getBoundsArray();

		Sc::BodySim* frozen[MaxTasks], * unfrozen[MaxTasks];
		PxU32 nbFrozen = 0, nbUnfrozen = 0;
		PxU32 nbActivated = 0, nbDeactivated = 0;

		for(PxU32 i = 0; i < mNumBodies; i++)
		{
			PxsRigidBody* rigid = islandSim.getRigidBody(mIndices[i]);
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
			//This move to PxgPostSolveWorkerTask for the gpu dynamic
			//bodySim->sleepCheck(mDt, mOneOverDt, mEnableStabilization);
		
			PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();
			//If we got in this code, then this is an active object this frame. The solver computed the new wakeCounter and we 
			//commit it at this stage. We need to do it this way to avoid a race condition between the solver and the island gen, where
			//the island gen may have deactivated a body while the solver decided to change its wake counter.
			bodyCore.wakeCounter = bodyCore.solverWakeCounter;
			PxsRigidBody& llBody = bodySim->getLowLevelBody();

			const Ps::IntBool isFrozen = bodySim->isFrozen();
			if(!isFrozen)
			{
				bpUpdates[nbBpUpdates++] = bodySim;

				// PT: TODO: remove duplicate "isFrozen" test inside updateCached
//				bodySim->updateCached(NULL);
				bodySim->updateCached(mCache, boundsArray);
			}

			if(llBody.isFreezeThisFrame() && isFrozen)
			{
				frozen[nbFrozen++] = bodySim;
			}
			else if(llBody.isUnfreezeThisFrame())
			{
				unfrozen[nbUnfrozen++] = bodySim;
			}

			if(bodyCore.mFlags & PxRigidBodyFlag::eENABLE_CCD)
				ccdBodies[nbCcdBodies++] = bodySim;

			if(llBody.isActivateThisFrame())
			{
				PX_ASSERT(!llBody.isDeactivateThisFrame());
				activateBodies[nbActivated++] = bodySim;
			}
			else if(llBody.isDeactivateThisFrame())
			{
				deactivateBodies[nbDeactivated++] = bodySim;
			}
			llBody.clearAllFrameFlags();
		}
		if(nbBpUpdates)
		{
			mCache.setChangedState();
			boundsArray.setChangedState();
		}

		if(nbBpUpdates>0 || nbFrozen > 0 || nbCcdBodies>0 || nbActivated>0 || nbDeactivated>0)
		{
			//Write active bodies to changed actor map
			mContext->getLock().lock();
			Cm::BitMapPinned& changedAABBMgrHandles = mScene.getAABBManager()->getChangedAABBMgActorHandleMap();
			
			for(PxU32 i = 0; i < nbBpUpdates; i++)
			{
				Sc::ElementSim* current = bpUpdates[i]->getElements_();
				while(current)
				{
					Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
					// PT: TODO: what's the difference between this test and "isInBroadphase" as used in bodySim->updateCached ?
					// PT: Also, shouldn't it be "isInAABBManager" rather than BP ?
					if(sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
						changedAABBMgrHandles.growAndSet(sim->getElementID());
					current = current->mNextInActor;
				}
			}

			Ps::Array<Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
			for (PxU32 i = 0; i < nbCcdBodies; i++)
				sceneCcdBodies.pushBack(ccdBodies[i]);

			for(PxU32 i=0;i<nbFrozen;i++)
			{
				PX_ASSERT(frozen[i]->isFrozen());
				frozen[i]->freezeTransforms(&changedAABBMgrHandles);
			}

			for(PxU32 i=0;i<nbUnfrozen;i++)
			{
				PX_ASSERT(!unfrozen[i]->isFrozen());
				unfrozen[i]->createSqBounds();
			}
			
			for(PxU32 i = 0; i < nbActivated; ++i)
			{
				activateBodies[i]->notifyNotReadyForSleeping();
			}

			for(PxU32 i = 0; i < nbDeactivated; ++i)
			{
				deactivateBodies[i]->notifyReadyForSleeping();
			}

			mContext->getLock().unlock();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.afterIntegrationTask";
	}

private:
	PX_NOCOPY(ScAfterIntegrationTask)
};

class ScSimulationControllerCallback : public PxsSimulationControllerCallback
{
	Sc::Scene* mScene; 
public:

	ScSimulationControllerCallback(Sc::Scene* scene) : mScene(scene)
	{
	}
	
	virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)
	{
		PxsContext* contextLL =  mScene->getLowLevelContext();
		IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
		Dy::Context* dynamicContext = mScene->getDynamicsContext();

		Cm::FlushPool& flushPool = contextLL->getTaskPool();

		const PxU32 MaxBodiesPerTask = ScAfterIntegrationTask::MaxTasks;

		PxsTransformCache& cache = contextLL->getTransformCache();

		const IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();

		/*const*/ PxU32 numBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		const IG::NodeIndex*const nodeIndices = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		if(1)
		{
			PxU32 nbShapes = 0;
			PxU32 startIdx = 0;
			for (PxU32 i = 0; i < numBodies; i++)
			{
				if (nbShapes >= MaxBodiesPerTask)
				{
					ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, i - startIdx,
						contextLL, dynamicContext, cache, *mScene));
					task->setContinuation(continuation);
					task->removeReference();
					startIdx = i;
					nbShapes = 0;
				}

				PxsRigidBody* rigid = islandSim.getRigidBody(nodeIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				nbShapes += PxMax(1u, bodySim->getNbShapes()); //Always add at least 1 shape in, even if the body has zero shapes because there is still some per-body overhead
			}

			if (nbShapes)
			{
				ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices + startIdx, numBodies - startIdx,
					contextLL, dynamicContext, cache, *mScene));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}
		else
		{
			// PT:
			const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();

			PxU32 nbPerTask;
			if(numCpuTasks)
			{
				nbPerTask = numBodies > numCpuTasks ? numBodies / numCpuTasks : numBodies;
			}
			else
			{
				nbPerTask = numBodies;
			}

			// PT: we need to respect that limit even with a single thread, because of hardcoded buffer limits in ScAfterIntegrationTask.
			if(nbPerTask>MaxBodiesPerTask)
				nbPerTask = MaxBodiesPerTask;

			PxU32 start = 0;
			while(numBodies)
			{
				const PxU32 nb = numBodies < nbPerTask ? numBodies : nbPerTask;

				ScAfterIntegrationTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScAfterIntegrationTask)), ScAfterIntegrationTask(nodeIndices+start, nb, 
					contextLL, dynamicContext, cache, *mScene));

				start += nb;
				numBodies -= nb;

				task->setContinuation(continuation);
				task->removeReference();
			}
		}
	}

	virtual PxU32 getNbCcdBodies()	
	{ 
		return mScene->getCcdBodies().size(); 
	}
};

class PxgUpdateBodyAndShapeStatusTask :  public Cm::Task
{
public:
	static const PxU32 MaxTasks = 256;
private:
	const IG::NodeIndex* const mNodeIndices;
	const PxU32 mNumBodies;
	Sc::Scene& mScene;
	void**	mRigidBodyLL;
	PxU32*	mActivatedBodies;
	PxU32*	mDeactivatedBodies;
	PxI32&	mCCDBodyWriteIndex;
	
public:

	PxgUpdateBodyAndShapeStatusTask(const IG::NodeIndex* const indices, PxU32 numBodies, void** rigidBodyLL, PxU32* activatedBodies, PxU32* deactivatedBodies, Sc::Scene& scene, PxI32& ccdBodyWriteIndex) : 
		Cm::Task			(scene.getContextId()),
		mNodeIndices		(indices),
		mNumBodies			(numBodies),
		mScene				(scene),
		mRigidBodyLL		(rigidBodyLL),
		mActivatedBodies	(activatedBodies),  
		mDeactivatedBodies	(deactivatedBodies),
		mCCDBodyWriteIndex	(ccdBodyWriteIndex)
	{
	}

	virtual void runInternal()
	{		
		IG::SimpleIslandManager& islandManager = *mScene.getSimpleIslandManager();
		const IG::IslandSim& islandSim = islandManager.getAccurateIslandSim();

		PxU32 nbCcdBodies = 0;

		Ps::Array<Sc::BodySim*>& sceneCcdBodies = mScene.getCcdBodies();
		Sc::BodySim* ccdBodies[MaxTasks];

		const size_t bodyOffset =  PX_OFFSET_OF_RT(Sc::BodySim, getLowLevelBody());

		for(PxU32 i=0; i<mNumBodies; ++i)
		{
			const PxU32 nodeIndex = mNodeIndices[i].index();
			PxsRigidBody* rigidLL = reinterpret_cast<PxsRigidBody*>(mRigidBodyLL[nodeIndex]);

			PxsBodyCore* bodyCore = &rigidLL->getCore();
			bodyCore->wakeCounter = bodyCore->solverWakeCounter;
			//we can set the frozen/unfrozen flag in GPU, but we have copied the internalflags
			//from the solverbodysleepdata to pxsbodycore, so we just need to clear the frozen flag in here
			rigidLL->clearAllFrameFlags();

			PX_ASSERT(mActivatedBodies[nodeIndex] <= 1);
			PX_ASSERT(mDeactivatedBodies[nodeIndex] <= 1);
			if(mActivatedBodies[nodeIndex])
			{
				PX_ASSERT(bodyCore->wakeCounter > 0.0f);
				islandManager.activateNode(mNodeIndices[i]);
			}
			else if(mDeactivatedBodies[nodeIndex])
			{
				//KS - the CPU code can reset the wake counter due to lost touches in parallel with the solver, so we need to verify
				//that the wakeCounter is still 0 before deactivating the node
				if(bodyCore->wakeCounter == 0.0f) 
					islandManager.deactivateNode(mNodeIndices[i]);
			}

			if (bodyCore->mFlags & PxRigidBodyFlag::eENABLE_CCD)
			{
				PxsRigidBody* rigidBody = islandSim.getRigidBody(mNodeIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - bodyOffset);
				ccdBodies[nbCcdBodies++] = bodySim;
			}
		}
		if(nbCcdBodies > 0)
		{
			PxI32 startIndex = Ps::atomicAdd(&mCCDBodyWriteIndex, PxI32(nbCcdBodies)) - PxI32(nbCcdBodies);
			for(PxU32 a = 0; a < nbCcdBodies; ++a)
			{
				sceneCcdBodies[startIndex + a] = ccdBodies[a];
			}
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.PxgUpdateBodyAndShapeStatusTask";
	}

private:
	PX_NOCOPY(PxgUpdateBodyAndShapeStatusTask)
};

class PxgSimulationControllerCallback : public PxsSimulationControllerCallback
{
	Sc::Scene* mScene; 
	PxI32 mCcdBodyWriteIndex;

public:
	PxgSimulationControllerCallback(Sc::Scene* scene) : mScene(scene), mCcdBodyWriteIndex(0)
	{
	}

	virtual void updateScBodyAndShapeSim(PxBaseTask* continuation)
	{
		IG::SimpleIslandManager* islandManager = mScene->getSimpleIslandManager();
		PxsSimulationController* simulationController = mScene->getSimulationController();
		PxsContext*	contextLL = mScene->getLowLevelContext();
		IG::IslandSim& islandSim = islandManager->getAccurateIslandSim();
		const PxU32 numBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
		const IG::NodeIndex*const nodeIndices = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

		PxU32* activatedBodies = simulationController->getActiveBodies();
		PxU32* deactivatedBodies = simulationController->getDeactiveBodies();

		//PxsRigidBody** rigidBodyLL = simulationController->getRigidBodies();
		void** rigidBodyLL = simulationController->getRigidBodies();

		Cm::FlushPool& flushPool = contextLL->getTaskPool();

		Ps::Array<Sc::BodySim*>& ccdBodies = mScene->getCcdBodies();
		ccdBodies.forceSize_Unsafe(0);
		ccdBodies.reserve(numBodies);
		ccdBodies.forceSize_Unsafe(numBodies);

		mCcdBodyWriteIndex = 0;

		for(PxU32 i = 0; i < numBodies; i+=PxgUpdateBodyAndShapeStatusTask::MaxTasks)
		{
			PxgUpdateBodyAndShapeStatusTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgUpdateBodyAndShapeStatusTask)), PxgUpdateBodyAndShapeStatusTask(nodeIndices + i, 
				PxMin(PxgUpdateBodyAndShapeStatusTask::MaxTasks, numBodies - i), rigidBodyLL, activatedBodies, deactivatedBodies, *mScene, mCcdBodyWriteIndex));
			task->setContinuation(continuation);
			task->removeReference();
		}
		
		PxU32* unfrozenShapeIndices = simulationController->getUnfrozenShapes();
		PxU32* frozenShapeIndices = simulationController->getFrozenShapes();
		const PxU32 nbFrozenShapes = simulationController->getNbFrozenShapes();
		const PxU32 nbUnfrozenShapes = simulationController->getNbUnfrozenShapes();

		PxsShapeSim** shapeSimsLL = simulationController->getShapeSims();
	
		const size_t shapeOffset = PX_OFFSET_OF_RT(Sc::ShapeSim, getLLShapeSim());

		for(PxU32 i=0; i<nbFrozenShapes; ++i)
		{
			const PxU32 shapeIndex = frozenShapeIndices[i];
			PxsShapeSim* shapeLL = shapeSimsLL[shapeIndex];
			Sc::ShapeSim* shape = reinterpret_cast<Sc::ShapeSim*>(reinterpret_cast<PxU8*>(shapeLL) - shapeOffset);
			shape->destroySqBounds();
		}

		for(PxU32 i=0; i<nbUnfrozenShapes; ++i)
		{
			const PxU32 shapeIndex = unfrozenShapeIndices[i];
			PxsShapeSim* shapeLL = shapeSimsLL[shapeIndex];
			Sc::ShapeSim* shape = reinterpret_cast<Sc::ShapeSim*>(reinterpret_cast<PxU8*>(shapeLL) - shapeOffset);
			shape->createSqBounds();
		}
	}

	virtual PxU32	getNbCcdBodies()
	{
		return PxU32(mCcdBodyWriteIndex);
	}
};

Sc::Scene::Scene(const PxSceneDesc& desc, PxU64 contextID) :
	mContextId						(contextID),
	mActiveBodies					(PX_DEBUG_EXP("sceneActiveBodies")),
	mActiveKinematicBodyCount		(0),
	mPointerBlock8Pool				(PX_DEBUG_EXP("scenePointerBlock8Pool")),
	mPointerBlock16Pool				(PX_DEBUG_EXP("scenePointerBlock16Pool")),
	mPointerBlock32Pool				(PX_DEBUG_EXP("scenePointerBlock32Pool")),
	mLLContext						(0),
	mBodyGravityDirty				(true),	
	mDt								(0),
	mOneOverDt						(0),
	mTimeStamp						(1),		// PT: has to start to 1 to fix determinism bug. I don't know why yet but it works.
	mReportShapePairTimeStamp		(0),
	mTriggerBufferAPI				(PX_DEBUG_EXP("sceneTriggerBufferAPI")),
	mRemovedShapeCountAtSimStart	(0),
	mArticulations					(PX_DEBUG_EXP("sceneArticulations")),
	mBrokenConstraints				(PX_DEBUG_EXP("sceneBrokenConstraints")),
	mActiveBreakableConstraints		(PX_DEBUG_EXP("sceneActiveBreakableConstraints")),
	mMemBlock128Pool				(PX_DEBUG_EXP("PxsContext ConstraintBlock128Pool")),
	mMemBlock256Pool				(PX_DEBUG_EXP("PxsContext ConstraintBlock256Pool")),
	mMemBlock384Pool				(PX_DEBUG_EXP("PxsContext ConstraintBlock384Pool")),
	mNPhaseCore						(NULL),
	mKineKineFilteringMode			(desc.kineKineFilteringMode),
	mStaticKineFilteringMode		(desc.staticKineFilteringMode),
	mSleepBodies					(PX_DEBUG_EXP("sceneSleepBodies")),
	mWokeBodies						(PX_DEBUG_EXP("sceneWokeBodies")),
	mEnableStabilization			(desc.flags & PxSceneFlag::eENABLE_STABILIZATION),
	mClients						(PX_DEBUG_EXP("sceneClients")),
	mActiveActors					(PX_DEBUG_EXP("clientActiveActors")),
	mFrozenActors					(PX_DEBUG_EXP("clientFrozenActors")),
	mClientPosePreviewBodies		(PX_DEBUG_EXP("clientPosePreviewBodies")),
	mClientPosePreviewBuffer		(PX_DEBUG_EXP("clientPosePreviewBuffer")),
	mSimulationEventCallback		(NULL),
	mBroadPhaseCallback				(NULL),
	mInternalFlags					(SceneInternalFlag::eSCENE_DEFAULT),
	mPublicFlags					(desc.flags),
	mStaticAnchor					(NULL),
	mBatchRemoveState				(NULL),
	mLostTouchPairs					(PX_DEBUG_EXP("sceneLostTouchPairs")),
	mOutOfBoundsIDs					(PX_DEBUG_EXP("sceneOutOfBoundsIds")),
	mVisualizationScale				(0.0f),
	mVisualizationParameterChanged	(false),
	mNbRigidStatics					(0),
	mNbRigidDynamics				(0),
	mSecondPassNarrowPhase			(contextID, this, "ScScene.secondPassNarrowPhase"),
	mPostNarrowPhase				(contextID, this, "ScScene.postNarrowPhase"),
	mFinalizationPhase				(contextID, this, "ScScene.finalizationPhase"),
	mUpdateCCDMultiPass				(contextID, this, "ScScene.updateCCDMultiPass"),
	mAfterIntegration				(contextID, this, "ScScene.afterIntegration"),
	mConstraintProjection			(contextID, this, "ScScene.constraintProjection"),
	mPostSolver						(contextID, this, "ScScene.postSolver"),
	mSolver							(contextID, this, "ScScene.rigidBodySolver"),
	mUpdateBodiesAndShapes			(contextID, this, "ScScene.updateBodiesAndShapes"),
	mUpdateSimulationController		(contextID, this, "ScScene.updateSimulationController"),
	mUpdateDynamics					(contextID, this, "ScScene.updateDynamics"),
	mProcessLostContactsTask		(contextID, this, "ScScene.processLostContact"),
	mProcessLostContactsTask2		(contextID, this, "ScScene.processLostContact2"),
	mProcessLostContactsTask3		(contextID, this, "ScScene.processLostContact3"),
	mDestroyManagersTask			(contextID, this, "ScScene.destroyManagers"),
	mLostTouchReportsTask			(contextID, this, "ScScene.lostTouchReports"),
	mUnregisterInteractionsTask		(contextID, this, "ScScene.unregisterInteractions"),
	mProcessNarrowPhaseLostTouchTasks(contextID, this, "ScScene.processNpLostTouchTask"),
	mProcessNPLostTouchEvents		(contextID, this, "ScScene.processNPLostTouchEvents"),
	mPostThirdPassIslandGenTask		(contextID, this, "ScScene.postThirdPassIslandGenTask"),
	mPostIslandGen					(contextID, this, "ScScene.postIslandGen"),
	mIslandGen						(contextID, this, "ScScene.islandGen"),
	mPreRigidBodyNarrowPhase		(contextID, this, "ScScene.preRigidBodyNarrowPhase"),
	mSetEdgesConnectedTask			(contextID, this, "ScScene.setEdgesConnectedTask"),
	mFetchPatchEventsTask			(contextID, this, "ScScene.fetchPatchEventsTask"),
	mProcessLostPatchesTask			(contextID, this, "ScScene.processLostSolverPatchesTask"),
	mRigidBodyNarrowPhase			(contextID, this, "ScScene.rigidBodyNarrowPhase"),
	mRigidBodyNPhaseUnlock			(contextID, this, "ScScene.unblockNarrowPhase"),
	mPostBroadPhase					(contextID, this, "ScScene.postBroadPhase"),
	mPostBroadPhaseCont				(contextID, this, "ScScene.postBroadPhaseCont"),
	mPostBroadPhase2				(contextID, this, "ScScene.postBroadPhase2"),
	mPostBroadPhase3				(contextID, this, "ScScene.postBroadPhase3"),
	mPreallocateContactManagers		(contextID, this, "ScScene.preallocateContactManagers"),
	mIslandInsertion				(contextID, this, "ScScene.islandInsertion"),
	mRegisterContactManagers		(contextID, this, "ScScene.registerContactManagers"),
	mRegisterInteractions			(contextID, this, "ScScene.registerInteractions"),
	mRegisterSceneInteractions		(contextID, this, "ScScene.registerSceneInteractions"),
	mBroadPhase						(contextID, this, "ScScene.broadPhase"),
	mAdvanceStep					(contextID, this, "ScScene.advanceStep"),
	mCollideStep					(contextID, this, "ScScene.collideStep"),	
	mTaskPool						(16384),
	mContactReportsNeedPostSolverVelocity(false),
	mUseGpuRigidBodies				(false),
	mSimulationStage				(SimulationStage::eCOMPLETE),
	mTmpConstraintGroupRootBuffer	(NULL),
	mPosePreviewBodies				(PX_DEBUG_EXP("scenePosePreviewBodies")),
	mOverlapFilterTaskHead			(NULL)
{

	mCCDPass = 0;
	for (int i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; ++i)
		mActiveInteractionCount[i] = 0;

	mStats						= PX_NEW(SimStats);
	mConstraintIDTracker = PX_NEW(ObjectIDTracker);
	mShapeIDTracker				= PX_NEW(ObjectIDTracker);
	mRigidIDTracker				= PX_NEW(ObjectIDTracker);
	mElementIDPool				= PX_NEW(ObjectIDTracker);

	mTriggerBufferExtraData		= reinterpret_cast<TriggerBufferExtraData*>(PX_ALLOC(sizeof(TriggerBufferExtraData), "ScScene::TriggerBufferExtraData"));
	new(mTriggerBufferExtraData) TriggerBufferExtraData(PX_DEBUG_EXP("ScScene::TriggerPairExtraData"));

	mStaticSimPool				= PX_NEW(PreallocatingPool<StaticSim>)(64, "StaticSim");
	mBodySimPool				= PX_NEW(PreallocatingPool<BodySim>)(64, "BodySim");
	mShapeSimPool				= PX_NEW(PreallocatingPool<ShapeSim>)(64, "ShapeSim");
	mConstraintSimPool			= PX_NEW(Ps::Pool<ConstraintSim>)(PX_DEBUG_EXP("ScScene::ConstraintSim"));
	mConstraintInteractionPool	= PX_NEW(Ps::Pool<ConstraintInteraction>)(PX_DEBUG_EXP("ScScene::ConstraintInteraction"));
	mLLArticulationPool			= PX_NEW(LLArticulationPool);
	mLLArticulationRCPool		= PX_NEW(LLArticulationRCPool);

	mSimStateDataPool			= PX_NEW(Ps::Pool<SimStateData>)(PX_DEBUG_EXP("ScScene::SimStateData"));

	mClients.pushBack(PX_NEW(Client)());
	mProjectionManager = PX_NEW(ConstraintProjectionManager)();

	mSqBoundsManager = PX_NEW(SqBoundsManager);

	mTaskManager = physx::PxTaskManager::createTaskManager(Ps::getFoundation().getErrorCallback(), desc.cpuDispatcher, desc.gpuDispatcher);

	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		mNbGeometries[i] = 0;

	bool useGpuDynamics = false;
	bool useGpuBroadphase = false;

#if PX_SUPPORT_GPU_PHYSX
	if (desc.flags & PxSceneFlag::eENABLE_GPU_DYNAMICS)
	{
		useGpuDynamics = true;
		if (mTaskManager->getGpuDispatcher() == NULL)
		{
			shdfnd::getFoundation().error(PxErrorCode::eDEBUG_WARNING,
				__FILE__, __LINE__, "GPU solver pipeline failed, switching to software");

			useGpuDynamics = false;
		}
		else if (!mTaskManager->getGpuDispatcher()->getCudaContextManager()->supportsArchSM30())
			useGpuDynamics = false;
	}

	if (desc.broadPhaseType == PxBroadPhaseType::eGPU)
	{
		useGpuBroadphase = true;
		if (mTaskManager->getGpuDispatcher() == NULL)
		{
			shdfnd::getFoundation().error(PxErrorCode::eDEBUG_WARNING,
				__FILE__, __LINE__, "GPU Bp pipeline failed, switching to software");

			useGpuBroadphase = false;
		}
		else if (!mTaskManager->getGpuDispatcher()->getCudaContextManager()->supportsArchSM30())
			useGpuBroadphase = false;
	}
#endif

	mUseGpuRigidBodies = useGpuBroadphase || useGpuDynamics;

	mLLContext = PX_NEW(PxsContext)(desc, mTaskManager, mTaskPool, contextID);
	
	if (mLLContext == 0)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Failed to create context!");
		return;
	}
	mLLContext->setMaterialManager(&getMaterialManager());

	mMemoryManager = NULL;

#if PX_SUPPORT_GPU_PHYSX
	mHeapMemoryAllocationManager = NULL;
	mGpuWranglerManagers = NULL;

	if (useGpuBroadphase || useGpuDynamics)
	{
		mMemoryManager = PxvGetPhysXGpu(true)->createGpuMemoryManager(mLLContext->getTaskManager().getGpuDispatcher(), NULL);
		mGpuWranglerManagers = PxvGetPhysXGpu(true)->createGpuKernelWranglerManager(mLLContext->getTaskManager().getGpuDispatcher(), getFoundation().getErrorCallback(),desc.gpuComputeVersion);
		mHeapMemoryAllocationManager = PxvGetPhysXGpu(true)->createGpuHeapMemoryAllocatorManager(desc.gpuDynamicsConfig.heapCapacity, mMemoryManager, desc.gpuComputeVersion);
	}
	else
#endif
	{
		mMemoryManager = createMemoryManager();
	}

	//Note: broadphase should be independent of AABBManager.  MBP uses it to call getBPBounds but it has 
	//already been passed all bounds in BroadPhase::update() so should use that instead.
	if(!useGpuBroadphase)
	{
		PxBroadPhaseType::Enum broadPhaseType = desc.broadPhaseType;

		if (broadPhaseType == PxBroadPhaseType::eGPU)
			broadPhaseType = PxBroadPhaseType::eABP;

		mBP = Bp::BroadPhase::create(
			broadPhaseType, 
			desc.limits.maxNbRegions, 
			desc.limits.maxNbBroadPhaseOverlaps, 
			desc.limits.maxNbStaticShapes, 
			desc.limits.maxNbDynamicShapes,
			contextID);
	}
	else
	{
#if PX_SUPPORT_GPU_PHYSX
		mBP = PxvGetPhysXGpu(true)->createGpuBroadPhase
			(
			mGpuWranglerManagers,
			mLLContext->getTaskManager().getGpuDispatcher(),
			NULL,
			desc.gpuComputeVersion,
			desc.gpuDynamicsConfig,
			mHeapMemoryAllocationManager);
#endif
	}

	//create allocator
	Ps::VirtualAllocatorCallback* allocatorCallback = mMemoryManager->createHostMemoryAllocator(desc.gpuComputeVersion);
	Ps::VirtualAllocator allocator(allocatorCallback);

	typedef Ps::Array<PxReal, Ps::VirtualAllocator> ContactDistArray;

	mBoundsArray = PX_NEW(Bp::BoundsArray)(allocator);
	//mBoundsArray = PX_NEW(Bp::BoundsArray);
	mContactDistance = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ContactDistArray), PX_DEBUG_EXP("ContactDistance")), ContactDistArray)(allocator);
	mHasContactDistanceChanged = false;

	const bool useEnhancedDeterminism = getPublicFlags() & PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
	const bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

	mSimpleIslandManager = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(IG::SimpleIslandManager), PX_DEBUG_EXP("SimpleIslandManager")), IG::SimpleIslandManager)(useEnhancedDeterminism, contextID);

	if (!useGpuDynamics)
	{
		if (desc.solverType == PxSolverType::ePGS)
		{
			mDynamicsContext = createDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				&mSimpleIslandManager->getAccurateIslandSim(), contextID, mEnableStabilization, useEnhancedDeterminism, useAdaptiveForce, desc.maxBiasCoefficient,
				!!(desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION));
		}
		else
		{
			mDynamicsContext = createTGSDynamicsContext
			(&mLLContext->getNpMemBlockPool(), mLLContext->getScratchAllocator(),
				mLLContext->getTaskPool(), mLLContext->getSimStats(), &mLLContext->getTaskManager(), allocatorCallback, &getMaterialManager(),
				&mSimpleIslandManager->getAccurateIslandSim(), contextID, mEnableStabilization, useEnhancedDeterminism, useAdaptiveForce,
				desc.getTolerancesScale().length);
		}

		mLLContext->setNphaseImplementationContext(createNphaseImplementationContext(*mLLContext, &mSimpleIslandManager->getAccurateIslandSim()));

		mSimulationControllerCallback = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ScSimulationControllerCallback), PX_DEBUG_EXP("ScSimulationControllerCallback")), ScSimulationControllerCallback(this));
		mSimulationController = createSimulationController(mSimulationControllerCallback);

		mAABBManager = PX_NEW(Bp::AABBManager)(*mBP, *mBoundsArray, *mContactDistance,
												desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, allocator, contextID,
												desc.kineKineFilteringMode, desc.staticKineFilteringMode);
	}
	else
	{
#if PX_SUPPORT_GPU_PHYSX
		mDynamicsContext = PxvGetPhysXGpu(true)->createGpuDynamicsContext(mLLContext->getTaskPool(), mGpuWranglerManagers, mLLContext->getTaskManager().getGpuDispatcher(), NULL,
			desc.gpuDynamicsConfig, &mSimpleIslandManager->getAccurateIslandSim(), desc.gpuMaxNumPartitions, mEnableStabilization, useEnhancedDeterminism, useAdaptiveForce, desc.maxBiasCoefficient, desc.gpuComputeVersion, mLLContext->getSimStats(),
			mHeapMemoryAllocationManager, !!(desc.flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION), desc.solverType);

		void* contactStreamBase = NULL;
		void* patchStreamBase = NULL;
		void* forceAndIndiceStreamBase = NULL;

		mDynamicsContext->getDataStreamBase(contactStreamBase, patchStreamBase, forceAndIndiceStreamBase);

		PxvNphaseImplementationContextUsableAsFallback* cpuNphaseImplementation = createNphaseImplementationContext(*mLLContext, &mSimpleIslandManager->getAccurateIslandSim());
		mLLContext->setNphaseFallbackImplementationContext(cpuNphaseImplementation);

		PxvNphaseImplementationContext* gpuNphaseImplementation = PxvGetPhysXGpu(true)->createGpuNphaseImplementationContext(*mLLContext, mGpuWranglerManagers, cpuNphaseImplementation, desc.gpuDynamicsConfig, contactStreamBase, patchStreamBase,
			forceAndIndiceStreamBase, getBoundsArray().getBounds(), &mSimpleIslandManager->getAccurateIslandSim(), mDynamicsContext, desc.gpuComputeVersion, mHeapMemoryAllocationManager);

		mSimulationControllerCallback = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxgSimulationControllerCallback), PX_DEBUG_EXP("PxgSimulationControllerCallback")), PxgSimulationControllerCallback(this));

		mSimulationController = PxvGetPhysXGpu(true)->createGpuSimulationController(mGpuWranglerManagers, mLLContext->getTaskManager().getGpuDispatcher(), NULL,
			mDynamicsContext, gpuNphaseImplementation, mBP, useGpuBroadphase, mSimpleIslandManager, mSimulationControllerCallback, desc.gpuComputeVersion, mHeapMemoryAllocationManager);

		mSimulationController->setBounds(mBoundsArray);
		mDynamicsContext->setSimulationController(mSimulationController);

		mLLContext->setNphaseImplementationContext(gpuNphaseImplementation);

		mLLContext->mContactStreamPool = &mDynamicsContext->getContactStreamPool();
		mLLContext->mPatchStreamPool = &mDynamicsContext->getPatchStreamPool();
		mLLContext->mForceAndIndiceStreamPool = &mDynamicsContext->getForceStreamPool();

		Ps::VirtualAllocator tAllocator(mHeapMemoryAllocationManager->mMappedMemoryAllocators);
	
		mAABBManager = PX_NEW(Bp::AABBManager)(*mBP, *mBoundsArray, *mContactDistance,
												desc.limits.maxNbAggregates, desc.limits.maxNbStaticShapes + desc.limits.maxNbDynamicShapes, tAllocator, contextID,
												desc.kineKineFilteringMode, desc.staticKineFilteringMode);
#endif
	}

	//Construct the bitmap of updated actors required as input to the broadphase update
	if(desc.limits.maxNbBodies)
	{
		// PT: TODO: revisit this. Why do we handle the added/removed and updated bitmaps entirely differently, in different places? And what is this weird formula here?
		mAABBManager->getChangedAABBMgActorHandleMap().resize((2*desc.limits.maxNbBodies + 256) & ~255);
	}

	//mLLContext->createTransformCache(mDynamicsContext->getAllocatorCallback());

	mLLContext->createTransformCache(*allocatorCallback);
	mLLContext->setContactDistance(mContactDistance);

	mCCDContext = physx::PxsCCDContext::create(mLLContext, mDynamicsContext->getThresholdStream(), *mLLContext->getNphaseImplementationContext(),
		desc.ccdThreshold);
	
	setSolverBatchSize(desc.solverBatchSize);
	mDynamicsContext->setFrictionOffsetThreshold(desc.frictionOffsetThreshold);
	mDynamicsContext->setCCDSeparationThreshold(desc.ccdMaxSeparation);
	mDynamicsContext->setSolverOffsetSlop(desc.solverOffsetSlop);

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mDynamicsContext->setCorrelationDistance(0.025f * scale.length);
	mLLContext->setMeshContactMargin(0.01f * scale.length);
	mLLContext->setToleranceLength(scale.length);

	// the original descriptor uses 
	//    bounce iff impact velocity  > threshold
	// but LL use 
	//    bounce iff separation velocity < -threshold 
	// hence we negate here.

	mDynamicsContext->setBounceThreshold(-desc.bounceThresholdVelocity);

	StaticCore* anchorCore = PX_NEW(StaticCore)(PxTransform(PxIdentity));

	mStaticAnchor = mStaticSimPool->construct(*this, *anchorCore);

	mNPhaseCore = PX_NEW(NPhaseCore)(*this, desc);

	initDominanceMatrix();
		
//	DeterminismDebugger::begin();

	mWokeBodyListValid = true;
	mSleepBodyListValid = true;

	//load from desc:
	setLimits(desc.limits);

	// Create broad phase
	setBroadPhaseCallback(desc.broadPhaseCallback);

	setGravity(desc.gravity);

	setFrictionType(desc.frictionType);

	setPCM(desc.flags & PxSceneFlag::eENABLE_PCM);

	setContactCache(!(desc.flags & PxSceneFlag::eDISABLE_CONTACT_CACHE));
	setSimulationEventCallback(desc.simulationEventCallback);
	setContactModifyCallback(desc.contactModifyCallback);
	setCCDContactModifyCallback(desc.ccdContactModifyCallback);
	setCCDMaxPasses(desc.ccdMaxPasses);
	PX_ASSERT(mNPhaseCore); // refactor paranoia
	
	PX_ASSERT(	((desc.filterShaderData) && (desc.filterShaderDataSize > 0)) ||
				(!(desc.filterShaderData) && (desc.filterShaderDataSize == 0))	);
	if (desc.filterShaderData)
	{
		mFilterShaderData = PX_ALLOC(desc.filterShaderDataSize, sFilterShaderDataMemAllocId);
		PxMemCopy(mFilterShaderData, desc.filterShaderData, desc.filterShaderDataSize);
		mFilterShaderDataSize = desc.filterShaderDataSize;
		mFilterShaderDataCapacity = desc.filterShaderDataSize;
	}
	else
	{
		mFilterShaderData = NULL;
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
	mFilterShader = desc.filterShader;
	mFilterCallback = desc.filterCallback;
}

void Sc::Scene::release()
{
	// TODO: PT: check virtual stuff

	mTimeStamp++;

	//collisionSpace.purgeAllPairs();

	//purgePairs();
	//releaseTagData();

	// We know release all the shapes before the collision space
	//collisionSpace.deleteAllShapes();

	//collisionSpace.release();

	//DeterminismDebugger::end();

	///clear broken constraint list:
	clearBrokenConstraintBuffer();

	PX_DELETE_AND_RESET(mNPhaseCore);

	PX_FREE_AND_RESET(mFilterShaderData);

	if (mStaticAnchor)
	{
		StaticCore& core = mStaticAnchor->getStaticCore();
		mStaticSimPool->destroy(mStaticAnchor);
		delete &core;
	}

	// Free object IDs and the deleted object id map
	postReportsCleanup();

	//before the task manager
	if (mLLContext)
	{
		if(mLLContext->getNphaseFallbackImplementationContext())
		{
			mLLContext->getNphaseFallbackImplementationContext()->destroy();
			mLLContext->setNphaseFallbackImplementationContext(NULL);
		}

		if(mLLContext->getNphaseImplementationContext())
		{
			mLLContext->getNphaseImplementationContext()->destroy();
			mLLContext->setNphaseImplementationContext(NULL);
		}
	}

	PX_DELETE_AND_RESET(mProjectionManager);
	PX_DELETE_AND_RESET(mSqBoundsManager);
	PX_DELETE_AND_RESET(mBoundsArray);

	for(PxU32 i=0;i<mClients.size(); i++)
		PX_DELETE_AND_RESET(mClients[i]);

	PX_DELETE(mConstraintInteractionPool);
	PX_DELETE(mConstraintSimPool);
	PX_DELETE(mSimStateDataPool);
	PX_DELETE(mStaticSimPool);
	PX_DELETE(mShapeSimPool);
	PX_DELETE(mBodySimPool);
	PX_DELETE(mLLArticulationPool);
	PX_DELETE(mLLArticulationRCPool);

	mTriggerBufferExtraData->~TriggerBufferExtraData();
	PX_FREE(mTriggerBufferExtraData);

	PX_DELETE(mElementIDPool);
	PX_DELETE(mRigidIDTracker);
	PX_DELETE(mShapeIDTracker);
	PX_DELETE(mConstraintIDTracker);
	PX_DELETE(mStats);

	mAABBManager->destroy();

	mBP->destroy();

	mSimulationControllerCallback->~PxsSimulationControllerCallback();
	PX_FREE(mSimulationControllerCallback);
	mSimulationController->~PxsSimulationController();
	PX_FREE(mSimulationController);

	mDynamicsContext->destroy();

	mCCDContext->destroy();

	mSimpleIslandManager->~SimpleIslandManager();
	PX_FREE(mSimpleIslandManager);

#if PX_SUPPORT_GPU_PHYSX
	if (mGpuWranglerManagers)
	{
		mGpuWranglerManagers->~PxsKernelWranglerManager();
		PX_FREE(mGpuWranglerManagers);
		mGpuWranglerManagers = NULL;
	}

	if (mHeapMemoryAllocationManager)
	{
		mHeapMemoryAllocationManager->~PxsHeapMemoryAllocatorManager();
		PX_FREE(mHeapMemoryAllocationManager);
		mHeapMemoryAllocationManager = NULL;
	}
#endif

	if (mTaskManager)
		mTaskManager->release();

	if (mLLContext)
	{
		PX_DELETE(mLLContext);
		mLLContext = NULL;
	}

	mContactDistance->~Array();
	PX_FREE(mContactDistance);


	if (mMemoryManager)
	{
		mMemoryManager->~PxsMemoryManager();
		PX_FREE(mMemoryManager);
		mMemoryManager = NULL;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::preAllocate(PxU32 nbStatics, PxU32 nbBodies, PxU32 nbStaticShapes, PxU32 nbDynamicShapes)
{
	// PT: TODO: this is only used for my addActors benchmark for now. Pre-allocate more arrays here.

	mActiveBodies.reserve(PxMax<PxU32>(64,nbBodies));

	mStaticSimPool->preAllocate(nbStatics);

	mBodySimPool->preAllocate(nbBodies);

	mShapeSimPool->preAllocate(nbStaticShapes + nbDynamicShapes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::addToActiveBodyList(BodySim& body)
{
	PX_ASSERT(body.getActiveListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);

	// Sort: kinematic before dynamic
	const PxU32 size = mActiveBodies.size();
	BodyCore* appendedBodyCore = &body.getBodyCore();					// PT: by default we append the incoming body...
	PxU32 incomingBodyActiveListIndex = size;							// PT: ...at the end of the current array.

	if(body.isKinematic())												// PT: Except if incoming body is kinematic, in which case:
	{
		const PxU32 nbKinematics = mActiveKinematicBodyCount++;			// PT: - we increase their number
		if(nbKinematics != size)										// PT: - if there's at least one dynamic in the array...
		{
			PX_ASSERT(appendedBodyCore != mActiveBodies[nbKinematics]);
			appendedBodyCore = mActiveBodies[nbKinematics];				// PT: ...then we grab the first dynamic after the kinematics...
			appendedBodyCore->getSim()->setActiveListIndex(size);		// PT: ...and we move that one back to the end of the array...

			mActiveBodies[nbKinematics] = &body.getBodyCore();			// PT: ...while the incoming kine replaces the dynamic we moved out.
			incomingBodyActiveListIndex = nbKinematics;					// PT: ...thus the incoming kine's index is the prev #kines.
		}
	}
	// for active compound rigids add to separate array, so we dont have to traverse all active actors
	if(body.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
	{
		PX_ASSERT(body.getActiveCompoundListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);
		const PxU32 compoundIndex = mActiveCompoundBodies.size();
		mActiveCompoundBodies.pushBack(appendedBodyCore);
		body.setActiveCompoundListIndex(compoundIndex);
	}
	body.setActiveListIndex(incomingBodyActiveListIndex);				// PT: will be 'size' or 'nbKinematics'
	mActiveBodies.pushBack(appendedBodyCore);							// PT: will be the incoming object or the first dynamic we moved out.
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::addToActiveCompoundBodyList(BodySim& body)
{
	PX_ASSERT(body.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID));

	BodyCore* appendedBodyCore = &body.getBodyCore();

	PX_ASSERT(body.getActiveCompoundListIndex() >= SC_NOT_IN_ACTIVE_LIST_INDEX);
	const PxU32 compoundIndex = mActiveCompoundBodies.size();
	mActiveCompoundBodies.pushBack(appendedBodyCore);
	body.setActiveCompoundListIndex(compoundIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::removeFromActiveCompoundBodyList(BodySim& body)
{
	PxU32 removedCompoundIndex = body.getActiveCompoundListIndex();
	PX_ASSERT(removedCompoundIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	body.setActiveCompoundListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	const PxU32 newCompoundSize = mActiveCompoundBodies.size() - 1;

	if(removedCompoundIndex!=newCompoundSize)
	{
		Sc::BodyCore* lastBody = mActiveCompoundBodies[newCompoundSize];
		mActiveCompoundBodies[removedCompoundIndex] = lastBody;
		lastBody->getSim()->setActiveListIndex(removedCompoundIndex);
	}
	mActiveCompoundBodies.forceSize_Unsafe(newCompoundSize);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::removeFromActiveBodyList(BodySim& body)
{
	PxU32 removedIndex = body.getActiveListIndex();
	PX_ASSERT(removedIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
	PX_ASSERT(mActiveBodies[removedIndex]==&body.getBodyCore());
	body.setActiveListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

	const PxU32 newSize = mActiveBodies.size() - 1;

	// Sort: kinematic before dynamic
	if(removedIndex < mActiveKinematicBodyCount)	// PT: same as 'body.isKinematic()' but without accessing the Core data
	{
		PX_ASSERT(mActiveKinematicBodyCount);
		PX_ASSERT(body.isKinematic());
		const PxU32 swapIndex = --mActiveKinematicBodyCount;
		if(newSize != swapIndex				// PT: equal if the array only contains kinematics
			&& removedIndex < swapIndex)	// PT: i.e. "if we don't remove the last kinematic"
		{
			BodyCore* swapBody = mActiveBodies[swapIndex];
			swapBody->getSim()->setActiveListIndex(removedIndex);
			mActiveBodies[removedIndex] = swapBody;
			removedIndex = swapIndex;
		}
	}

	// for active compound rigids add to separate array, so we dont have to traverse all active actors
	// A.B. TODO we should handle kinematic switch, no need to hold kinematic rigids in compound list
	if(body.readInternalFlag(BodySim::BF_IS_COMPOUND_RIGID))
	{
		PxU32 removedCompoundIndex = body.getActiveCompoundListIndex();
		PX_ASSERT(removedCompoundIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);
		body.setActiveCompoundListIndex(SC_NOT_IN_ACTIVE_LIST_INDEX);

		const PxU32 newCompoundSize = mActiveCompoundBodies.size() - 1;

		if(removedCompoundIndex!=newCompoundSize)
		{
			Sc::BodyCore* lastBody = mActiveCompoundBodies[newCompoundSize];
			mActiveCompoundBodies[removedCompoundIndex] = lastBody;
			lastBody->getSim()->setActiveCompoundListIndex(removedCompoundIndex);
		}
		mActiveCompoundBodies.forceSize_Unsafe(newCompoundSize);
	}

	if(removedIndex!=newSize)
	{
		Sc::BodyCore* lastBody = mActiveBodies[newSize];
		mActiveBodies[removedIndex] = lastBody;
		lastBody->getSim()->setActiveListIndex(removedIndex);
	}
	mActiveBodies.forceSize_Unsafe(newSize);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::swapInActiveBodyList(BodySim& body)
{
	const PxU32 activeListIndex = body.getActiveListIndex();
	PX_ASSERT(activeListIndex < SC_NOT_IN_ACTIVE_LIST_INDEX);

	PxU32 swapIndex;
	PxU32 newActiveKinematicBodyCount;
	if(activeListIndex < mActiveKinematicBodyCount)
	{
		// kinematic -> dynamic
		PX_ASSERT(!body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount > 0);  // there has to be at least one kinematic body

		swapIndex = mActiveKinematicBodyCount - 1;
		newActiveKinematicBodyCount = swapIndex;
	}
	else
	{
		// dynamic -> kinematic
		PX_ASSERT(body.isKinematic());  // the corresponding flag gets switched before this call
		PX_ASSERT(mActiveKinematicBodyCount < mActiveBodies.size());  // there has to be at least one dynamic body

		swapIndex = mActiveKinematicBodyCount;
		newActiveKinematicBodyCount = swapIndex + 1;
	}

	BodyCore*& swapBodyRef = mActiveBodies[swapIndex];
	body.setActiveListIndex(swapIndex);
	BodyCore* swapBody = swapBodyRef;
	swapBodyRef = &body.getBodyCore();

	swapBody->getSim()->setActiveListIndex(activeListIndex);
	mActiveBodies[activeListIndex] = swapBody;

	mActiveKinematicBodyCount = newActiveKinematicBodyCount;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::registerInteraction(Interaction* interaction, bool active)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = mInteractions[type].size();
	interaction->setInteractionId(sceneArrayIndex);
	if(mInteractions[type].capacity()==0)
		mInteractions[type].reserve(64);

	mInteractions[type].pushBack(interaction);
	if (active)
	{
		if (sceneArrayIndex > mActiveInteractionCount[type])
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
		mActiveInteractionCount[type]++;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::unregisterInteraction(Interaction* interaction)
{
	const InteractionType::Enum type = interaction->getType();
	const PxU32 sceneArrayIndex = interaction->getInteractionId();
	mInteractions[type].replaceWithLast(sceneArrayIndex);
	interaction->setInteractionId(PX_INVALID_INTERACTION_SCENE_ID);
	if (sceneArrayIndex<mInteractions[type].size()) // The removed interaction was the last one, do not reset its sceneArrayIndex
		mInteractions[type][sceneArrayIndex]->setInteractionId(sceneArrayIndex);
	if (sceneArrayIndex<mActiveInteractionCount[type])
	{
		mActiveInteractionCount[type]--;
		if (mActiveInteractionCount[type]<mInteractions[type].size())
			swapInteractionArrayIndices(sceneArrayIndex, mActiveInteractionCount[type], type);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::swapInteractionArrayIndices(PxU32 id1, PxU32 id2, InteractionType::Enum type)
{
	Ps::Array<Interaction*>& interArray = mInteractions[type];
	Interaction* interaction1 = interArray[id1];
	Interaction* interaction2 = interArray[id2];
	interArray[id1] = interaction2;
	interArray[id2] = interaction1;
	interaction1->setInteractionId(id2);
	interaction2->setInteractionId(id1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::notifyInteractionActivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();

	PX_ASSERT(interaction->getInteractionId() >= mActiveInteractionCount[type]);
	
	if (mActiveInteractionCount[type] < mInteractions[type].size())
		swapInteractionArrayIndices(mActiveInteractionCount[type], interaction->getInteractionId(), type);
	mActiveInteractionCount[type]++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::notifyInteractionDeactivated(Interaction* interaction)
{
	PX_ASSERT((interaction->getType() == InteractionType::eOVERLAP) || (interaction->getType() == InteractionType::eTRIGGER));
	PX_ASSERT(!interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_ASSERT(interaction->getInteractionId() != PX_INVALID_INTERACTION_SCENE_ID);

	const InteractionType::Enum type = interaction->getType();
	PX_ASSERT(interaction->getInteractionId() < mActiveInteractionCount[type]);

	if (mActiveInteractionCount[type] > 1)
		swapInteractionArrayIndices(mActiveInteractionCount[type]-1, interaction->getInteractionId(), type);
	mActiveInteractionCount[type]--;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void** Sc::Scene::allocatePointerBlock(PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	void* ptr;
	if(size==8)
		ptr = mPointerBlock8Pool.construct();
	else if(size == 16)
		ptr = mPointerBlock16Pool.construct();
	else if(size == 32)
		ptr = mPointerBlock32Pool.construct();
	else
		ptr = PX_ALLOC(size * sizeof(void*), "void*");

	return reinterpret_cast<void**>(ptr);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::deallocatePointerBlock(void** block, PxU32 size)
{
	PX_ASSERT(size>32 || size == 32 || size == 16 || size == 8);
	if(size==8)
		mPointerBlock8Pool.destroy(reinterpret_cast<PointerBlock8*>(block));
	else if(size == 16)
		mPointerBlock16Pool.destroy(reinterpret_cast<PointerBlock16*>(block));
	else if(size == 32)
		mPointerBlock32Pool.destroy(reinterpret_cast<PointerBlock32*>(block));
	else
		return PX_FREE(block);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxBroadPhaseType::Enum Sc::Scene::getBroadPhaseType() const
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->getType();
}

bool Sc::Scene::getBroadPhaseCaps(PxBroadPhaseCaps& caps) const
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->getCaps(caps);
}

PxU32 Sc::Scene::getNbBroadPhaseRegions() const
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->getNbRegions();
}

PxU32 Sc::Scene::getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->getRegions(userBuffer, bufferSize, startIndex);
}

PxU32 Sc::Scene::addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->addRegion(region, populateRegion);
}

bool Sc::Scene::removeBroadPhaseRegion(PxU32 handle)
{
	Bp::BroadPhase* bp = mAABBManager->getBroadPhase();
	return bp->removeRegion(handle);
}

void** Sc::Scene::getOutOfBoundsAggregates()
{
	PxU32 dummy;
	return mAABBManager->getOutOfBoundsAggregates(dummy);
}

PxU32 Sc::Scene::getNbOutOfBoundsAggregates()
{
	PxU32 val;
	mAABBManager->getOutOfBoundsAggregates(val);
	return val;
}

void Sc::Scene::clearOutOfBoundsAggregates()
{
	mAABBManager->clearOutOfBoundsAggregates();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::Scene::setFilterShaderData(const void* data, PxU32 dataSize)
{
	PX_UNUSED(sFilterShaderDataMemAllocId);

	if (data)
	{
		PX_ASSERT(dataSize > 0);

		void* buffer;

		if (dataSize <= mFilterShaderDataCapacity)
			buffer = mFilterShaderData;
		else
		{
			buffer = PX_ALLOC(dataSize, sFilterShaderDataMemAllocId);
			if (buffer)
			{
				mFilterShaderDataCapacity = dataSize;
				if (mFilterShaderData)
					PX_FREE(mFilterShaderData);
			}
			else
			{
				Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Failed to allocate memory for filter shader data!");
				return;
			}
		}

		PxMemCopy(buffer, data, dataSize);
		mFilterShaderData = buffer;
		mFilterShaderDataSize = dataSize;
	}
	else
	{
		PX_ASSERT(dataSize == 0);

		if (mFilterShaderData)
			PX_FREE_AND_RESET(mFilterShaderData);
		mFilterShaderDataSize = 0;
		mFilterShaderDataCapacity = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Cm::RenderBuffer& Sc::Scene::getRenderBuffer()
{
	return mLLContext->getRenderBuffer();
}

void Sc::Scene::prepareCollide()
{
	mReportShapePairTimeStamp++;	// deleted actors/shapes should get separate pair entries in contact reports
	mContactReportsNeedPostSolverVelocity = false;

	mRemovedShapeCountAtSimStart = mShapeIDTracker->getDeletedIDCount();

	getRenderBuffer().clear();

	// Clear broken constraint list:
	clearBrokenConstraintBuffer();

	// Update from visualization parameters
	if(mVisualizationParameterChanged)
	{
		mVisualizationParameterChanged = false;

		// Update SIPs if visualization is enabled
		if(	getVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL) || 
			getVisualizationParameter(PxVisualizationParameter::eCONTACT_ERROR) || getVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE))
			mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION;
	}

	visualizeStartStep();
	
	PxcClearContactCacheStats();
}

void Sc::Scene::simulate(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		mDt = timeStep;
		mOneOverDt = 0.0f < mDt ? 1.0f/mDt : 0.0f;

		mAdvanceStep.setContinuation(continuation);

		prepareCollide();
		stepSetupCollide(&mAdvanceStep); 
		
		mCollideStep.setContinuation(&mAdvanceStep);

		mAdvanceStep.removeReference();
		mCollideStep.removeReference();
	}
}

void Sc::Scene::advance(PxReal timeStep, PxBaseTask* continuation)
{
	if(timeStep != 0.0f)
	{
		mDt = timeStep;
		mOneOverDt = 0.0f < mDt ? 1.0f/mDt : 0.0f;

		mAdvanceStep.setContinuation(continuation);

		stepSetupSolve(&mAdvanceStep);		
		
		mAdvanceStep.removeReference();
	}
}

void Sc::Scene::setBounceThresholdVelocity(const PxReal t)
{
	mDynamicsContext->setBounceThreshold(-t);
}

PxReal Sc::Scene::getBounceThresholdVelocity() const
{
	return -mDynamicsContext->getBounceThreshold();
}

void Sc::Scene::collide(PxReal timeStep, PxBaseTask* continuation)
{
	mDt = timeStep;

	prepareCollide();
	stepSetupCollide(continuation);

	mLLContext->beginUpdate();

	mCollideStep.setContinuation(continuation);
	mCollideStep.removeReference();
}

void Sc::Scene::setFrictionType(PxFrictionType::Enum model)
{
	mDynamicsContext->setFrictionType(model);
}

PxFrictionType::Enum Sc::Scene::getFrictionType() const
{
	return mDynamicsContext->getFrictionType();
}

void Sc::Scene::setPCM(bool enabled)
{
	mLLContext->setPCM(enabled);
}

void Sc::Scene::setContactCache(bool enabled)
{
	mLLContext->setContactCache(enabled);
}

void Sc::Scene::endSimulation()
{
	// Handle user contact filtering
	// Note: Do this before the contact callbacks get fired since the filter callback might
	//       trigger contact reports (touch lost due to re-filtering)

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->fireCustomFilteringCallbacks(outputs, mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE);

	mNPhaseCore->preparePersistentContactEventListForNextFrame();

	mSimulationController->releaseDeferredArticulationIds();

	endStep();	// - Update time stamps

	PxcDisplayContactCacheStats();
}

void Sc::Scene::flush(bool sendPendingReports)
{
	if (sendPendingReports)
	{
		fireQueuedContactCallbacks(true);
		mNPhaseCore->clearContactReportStream();
		mNPhaseCore->clearContactReportActorPairs(true);

		fireTriggerCallbacks();
	}
	else
	{
		mNPhaseCore->clearContactReportActorPairs(true);  // To clear the actor pair set
	}
	postReportsCleanup();
	mNPhaseCore->freeContactReportStreamMemory();

	mTriggerBufferAPI.reset();
	mTriggerBufferExtraData->reset();

	clearBrokenConstraintBuffer();
	mBrokenConstraints.reset();

	clearSleepWakeBodies();  //!!! If we send out these reports on flush then this would not be necessary

	mClients.shrink();

	mShapeIDTracker->reset();
	mRigidIDTracker->reset();

	processLostTouchPairs();  // Processes the lost touch bodies
	PX_ASSERT(mLostTouchPairs.size() == 0);
	mLostTouchPairs.reset();
	// Does not seem worth deleting the bitmap for the lost touch pair list

	mActiveBodies.shrink();

	for(PxU32 i=0; i < InteractionType::eTRACKED_IN_SCENE_COUNT; i++)
	{
		mInteractions[i].shrink();
	}

	//!!! TODO: look into retrieving memory from the NPhaseCore & Broadphase class (all the pools in there etc.)

	mLLContext->getNpMemBlockPool().releaseUnusedBlocks();
}

// User callbacks

void Sc::Scene::setSimulationEventCallback(PxSimulationEventCallback* callback)
{
	if(!mSimulationEventCallback && callback)
	{
		// if there was no callback before, the sleeping bodies have to be prepared for potential notification events (no shortcut possible anymore)
		BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
		for(PxU32 i=0; i < mSleepBodies.size(); i++)
			sleepingBodies[i]->getSim()->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
	}

	mSimulationEventCallback = callback;
}

PxSimulationEventCallback* Sc::Scene::getSimulationEventCallback() const
{
	return mSimulationEventCallback;
}

void Sc::Scene::setContactModifyCallback(PxContactModifyCallback* callback)
{
	mLLContext->setContactModifyCallback(callback);
}

PxContactModifyCallback* Sc::Scene::getContactModifyCallback() const
{
	return mLLContext->getContactModifyCallback();
}

void Sc::Scene::setCCDContactModifyCallback(PxCCDContactModifyCallback* callback)
{
	mCCDContext->setCCDContactModifyCallback(callback);
}

PxCCDContactModifyCallback* Sc::Scene::getCCDContactModifyCallback() const
{
	return mCCDContext->getCCDContactModifyCallback();
}

void Sc::Scene::setCCDMaxPasses(PxU32 ccdMaxPasses)
{
	mCCDContext->setCCDMaxPasses(ccdMaxPasses);
}

PxU32 Sc::Scene::getCCDMaxPasses() const
{
	return mCCDContext->getCCDMaxPasses();
}

void Sc::Scene::removeBody(BodySim& body)	//this also notifies any connected joints!
{
	ConstraintGroupNode* node = body.getConstraintGroup();
	if (node)
	{
		//invalidate the constraint group:
		//this adds all constraints of the group to the dirty list such that groups get re-generated next frame
		getProjectionManager().invalidateGroup(*node, NULL);
	}

	BodyCore& core = body.getBodyCore();

	// Remove from sleepBodies array
	mSleepBodies.erase(&core);
	PX_ASSERT(!mSleepBodies.contains(&core));

	// Remove from wokeBodies array
	mWokeBodies.erase(&core);
	PX_ASSERT(!mWokeBodies.contains(&core));

	if (body.isActive() && (core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW))
		removeFromPosePreviewList(body);
	else
		PX_ASSERT(!isInPosePreviewList(body));

	markReleasedBodyIDForLostTouch(body.getRigidID());
}

void Sc::Scene::addConstraint(ConstraintCore& constraint, RigidCore* body0, RigidCore* body1)
{
	ConstraintSim* sim = mConstraintSimPool->construct(constraint, body0, body1, *this);
	PX_UNUSED(sim);

	mConstraints.insert(&constraint);
}

void Sc::Scene::removeConstraint(ConstraintCore& constraint)
{
	ConstraintSim* cSim = constraint.getSim();

	if (cSim)
	{
		BodySim* b = cSim->getAnyBody();
		ConstraintGroupNode* n = b->getConstraintGroup();
		
		if (n)
			getProjectionManager().invalidateGroup(*n, cSim);
		mConstraintSimPool->destroy(cSim);
	}

	mConstraints.erase(&constraint);
}

void Sc::Scene::addArticulation(ArticulationCore& articulation, BodyCore& root)
{
	ArticulationSim* sim = PX_NEW(ArticulationSim)(articulation, *this, root);

	if (sim && (sim->getLowLevelArticulation() == NULL))
	{
		PX_DELETE(sim);
		return;
	}
	mArticulations.insert(&articulation);
}

void Sc::Scene::removeArticulation(ArticulationCore& articulation)
{
	ArticulationSim* a = articulation.getSim();
	if (a)
		PX_DELETE(a);
	mArticulations.erase(&articulation);
}

void Sc::Scene::addArticulationJoint(ArticulationJointCore& joint, BodyCore& parent, BodyCore& child)
{
	ArticulationJointSim* sim = PX_NEW(ArticulationJointSim)(joint, *parent.getSim(), *child.getSim());
	PX_UNUSED(sim);
}

void Sc::Scene::removeArticulationJoint(ArticulationJointCore& joint)
{
	if (joint.getSim())
		PX_DELETE(joint.getSim());
}

void Sc::Scene::addArticulationSimControl(Sc::ArticulationCore& core)
{
	Sc::ArticulationSim* sim = core.getSim();
	if (sim)
		mSimulationController->addArticulation(sim->getLowLevelArticulation(), sim->getIslandNodeIndex());
}

void Sc::Scene::addBrokenConstraint(Sc::ConstraintCore* c)
{
	PX_ASSERT(mBrokenConstraints.find(c) == mBrokenConstraints.end());
	mBrokenConstraints.pushBack(c);
}

void Sc::Scene::addActiveBreakableConstraint(Sc::ConstraintSim* c, Sc::ConstraintInteraction* ci)
{
	PX_ASSERT(ci && ci->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_UNUSED(ci);
	PX_ASSERT(!mActiveBreakableConstraints.contains(c));
	PX_ASSERT(!c->isBroken());
	mActiveBreakableConstraints.insert(c);
	c->setFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

void Sc::Scene::removeActiveBreakableConstraint(Sc::ConstraintSim* c)
{
	const bool exists = mActiveBreakableConstraints.erase(c);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
	c->clearFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

void* Sc::Scene::allocateConstraintBlock(PxU32 size)
{
	if(size<=128)
		return mMemBlock128Pool.construct();
	else if(size<=256)
		return mMemBlock256Pool.construct();
	else  if(size<=384)
		return mMemBlock384Pool.construct();
	else
		return PX_ALLOC(size, "ConstraintBlock");
}

void Sc::Scene::deallocateConstraintBlock(void* ptr, PxU32 size)
{
	if(size<=128)
		mMemBlock128Pool.destroy(reinterpret_cast<MemBlock128*>(ptr));
	else if(size<=256)
		mMemBlock256Pool.destroy(reinterpret_cast<MemBlock256*>(ptr));
	else  if(size<=384)
		mMemBlock384Pool.destroy(reinterpret_cast<MemBlock384*>(ptr));
	else
		PX_FREE(ptr);
}

/*-------------------------------*\
| Adam's explanation of the RB solver:
| This is a novel idea of mine, 
| a combination of ideas on
| Milenkovic's Optimization
| Based Animation, and Trinkle's 
| time stepping schemes.
|
| A time step goes like this:
|
| Taking no substeps:
| 0) Compute contact points.
| 1) Update external forces. This may include friction.
| 2) Integrate external forces to current velocities.
| 3) Solve for impulses at contacts which will prevent 
|	interpenetration at next timestep given some 
|	velocity integration scheme.
| 4) Use the integration scheme on velocity to
|	reach the next state. Here we should not have any
|   interpenetration at the old contacts, but perhaps
|	at new contacts. If interpenetrating at new contacts,
|	just add these to the contact list; no need to repeat
|	the time step, because the scheme will get rid of the
|	penetration by the next step.
|
|
| Advantages:
| + Large steps, LOD realism.
| + very simple.
|
\*-------------------------------*/

void Sc::Scene::advanceStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.solveQueueTasks", getContextId());

	if (mDt != 0.0f)
	{
		mFinalizationPhase.addDependent(*continuation);
		mFinalizationPhase.removeReference();

		if (mPublicFlags & PxSceneFlag::eENABLE_CCD)
		{
			mUpdateCCDMultiPass.setContinuation(&mFinalizationPhase);
			mAfterIntegration.setContinuation(&mUpdateCCDMultiPass);
			mUpdateCCDMultiPass.removeReference();
		}
		else
		{
			mAfterIntegration.setContinuation(&mFinalizationPhase);
		}

		mPostSolver.setContinuation(&mAfterIntegration);
		mUpdateSimulationController.setContinuation(&mPostSolver);
		mUpdateDynamics.setContinuation(&mUpdateSimulationController);
		mUpdateBodiesAndShapes.setContinuation(&mUpdateDynamics);
		mSolver.setContinuation(&mUpdateBodiesAndShapes);
		mPostIslandGen.setContinuation(&mSolver);
		mIslandGen.setContinuation(&mPostIslandGen);
		mPostNarrowPhase.addDependent(mIslandGen);
		mPostNarrowPhase.removeReference();

		mSecondPassNarrowPhase.setContinuation(&mPostNarrowPhase);

		mFinalizationPhase.removeReference();
		mAfterIntegration.removeReference();
		mPostSolver.removeReference();
		mUpdateSimulationController.removeReference();
		mUpdateDynamics.removeReference();
		mUpdateBodiesAndShapes.removeReference();
		mSolver.removeReference();
		mPostIslandGen.removeReference();
		mIslandGen.removeReference();
		mPostNarrowPhase.removeReference();
		mSecondPassNarrowPhase.removeReference();
	}
}

//void Sc::Scene::advanceStep(PxBaseTask* continuation)
//{
//	PX_PROFILE_ZONE("Sim.solveQueueTasks", getContextId());
//
//	if(mDt!=0.0f)
//	{
//		mFinalizationPhase.addDependent(*continuation);
//		mFinalizationPhase.removeReference();
//
//		if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
//		{
//			mUpdateCCDMultiPass.setContinuation(&mFinalizationPhase);
//			mAfterIntegration.setContinuation(&mUpdateCCDMultiPass);
//			mUpdateCCDMultiPass.removeReference();
//		}
//		else
//		{
//			mAfterIntegration.setContinuation(&mFinalizationPhase);
//		}
//
//		mPostSolver.setContinuation(&mAfterIntegration);
//		mUpdateSimulationController.setContinuation(&mPostSolver);
//		mUpdateDynamics.setContinuation(&mUpdateSimulationController);
//		mSolver.setContinuation(&mUpdateDynamics);
//		mProcessTriggerInteractions.setContinuation(&mSolver);
//		mPostIslandGen.setContinuation(&mProcessTriggerInteractions);
//		mIslandGen.setContinuation(&mPostIslandGen);
//
//		mFinalizationPhase.removeReference();
//		mAfterIntegration.removeReference();
//		mPostSolver.removeReference();
//		mUpdateSimulationController.removeReference();
//		mUpdateDynamics.removeReference();
//		mSolver.removeReference();
//		mProcessTriggerInteractions.removeReference();
//		mPostIslandGen.removeReference();
//		mIslandGen.removeReference();
//	}
//}

void Sc::Scene::collideStep(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.collideQueueTasks", getContextId());
	PX_PROFILE_START_CROSSTHREAD("Basic.collision", getContextId());

	mStats->simStart();
	mLLContext->beginUpdate();

	mPostNarrowPhase.setTaskManager(*continuation->getTaskManager());
	mPostNarrowPhase.addReference();

	mFinalizationPhase.setTaskManager(*continuation->getTaskManager());
	mFinalizationPhase.addReference();

	mRigidBodyNarrowPhase.setContinuation(continuation);
	mPreRigidBodyNarrowPhase.setContinuation(&mRigidBodyNarrowPhase);

	mRigidBodyNarrowPhase.removeReference();
	mPreRigidBodyNarrowPhase.removeReference();
}

void Sc::Scene::broadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.broadPhase", getContextId());

	const PxU32 numCpuTasks = continuation->getTaskManager()->getCpuDispatcher()->getWorkerCount();
	mAABBManager->updateAABBsAndBP(numCpuTasks, mLLContext->getTaskPool(), &mLLContext->getScratchAllocator(), mHasContactDistanceChanged, continuation, &mRigidBodyNPhaseUnlock);
}

void Sc::Scene::postBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.postBroadPhase", getContextId());

	//Notify narrow phase that broad phase has completed
	mLLContext->getNphaseImplementationContext()->postBroadPhaseUpdateContactManager();
	mAABBManager->postBroadPhase(continuation, &mRigidBodyNPhaseUnlock, *getFlushPool());
}

void Sc::Scene::postBroadPhaseContinuation(PxBaseTask* continuation)
{
	mAABBManager->getChangedAABBMgActorHandleMap().clear();

	// - Finishes broadphase update
	// - Adds new interactions (and thereby contact managers if needed)
	finishBroadPhase(continuation);
}

void Sc::Scene::postBroadPhaseStage2(PxBaseTask* continuation)
{
	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();
	//Release unused Cms back to the pool (later, this needs to be done in a thread-safe way from multiple worker threads
	mIslandInsertion.setContinuation(continuation);
	mRegisterContactManagers.setContinuation(continuation);
	mRegisterInteractions.setContinuation(continuation);
	mRegisterSceneInteractions.setContinuation(continuation);
	mIslandInsertion.removeReference();
	mRegisterContactManagers.removeReference();
	mRegisterInteractions.removeReference();
	mRegisterSceneInteractions.removeReference();

	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.release", getContextId());
		for (PxU32 a = 0; a < mPreallocatedContactManagers.size(); ++a)
		{
			if ((reinterpret_cast<size_t>(mPreallocatedContactManagers[a]) & 1) == 0)
				mLLContext->getContactManagerPool().put(mPreallocatedContactManagers[a]);
		}

		for (PxU32 a = 0; a < mPreallocatedShapeInteractions.size(); ++a)
		{
			if ((reinterpret_cast<size_t>(mPreallocatedShapeInteractions[a]) & 1) == 0)
				mNPhaseCore->mShapeInteractionPool.deallocate(mPreallocatedShapeInteractions[a]);
		}

		for (PxU32 a = 0; a < mPreallocatedInteractionMarkers.size(); ++a)
		{
			if ((reinterpret_cast<size_t>(mPreallocatedInteractionMarkers[a]) & 1) == 0)
				mNPhaseCore->mInteractionMarkerPool.deallocate(mPreallocatedInteractionMarkers[a]);
		}
	}
}

void Sc::Scene::postBroadPhaseStage3(PxBaseTask* /*continuation*/)
{
	finishBroadPhaseStage2(0);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.postBroadPhase", getContextId());
	PX_PROFILE_STOP_CROSSTHREAD("Basic.broadPhase", getContextId());
}

class DirtyShapeUpdatesTask : public Cm::Task
{
public:
	static const PxU32 MaxShapes = 256;

	PxsTransformCache& mCache;
	Bp::BoundsArray& mBoundsArray;
	Sc::ShapeSim* mShapes[MaxShapes];
	PxU32 mNbShapes;

	DirtyShapeUpdatesTask(PxU64 contextID, PxsTransformCache& cache, Bp::BoundsArray& boundsArray) : 
		Cm::Task	(contextID),
		mCache		(cache),
		mBoundsArray(boundsArray),
		mNbShapes	(0)
	{
	}

	virtual void runInternal() 
	{
		for (PxU32 a = 0; a < mNbShapes; ++a)
		{
			mShapes[a]->updateCached(mCache, mBoundsArray);
		}
	}

	virtual const char* getName() const { return "DirtyShapeUpdatesTask";  }

private:
	PX_NOCOPY(DirtyShapeUpdatesTask)
};

class SpeculativeCCDContactDistanceUpdateTask : public Cm::Task
{
public:
	static const PxU32 MaxBodies = 128;
	PxReal* mContactDistances;
	const PxReal mDt;
	Sc::BodySim* mBodySims[MaxBodies];
	PxU32 mNbBodies;

	Bp::BoundsArray& mBoundsArray;

	SpeculativeCCDContactDistanceUpdateTask(PxU64 contextID, PxReal* contactDistances, const PxReal dt, Bp::BoundsArray& boundsArray) :
		Cm::Task			(contextID),
		mContactDistances	(contactDistances),
		mDt					(dt),
		mNbBodies			(0),
		mBoundsArray		(boundsArray)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbBodies; ++a)
		{
			mBodySims[a]->updateContactDistance(mContactDistances, mDt, mBoundsArray);
		}
	}

	virtual const char* getName() const { return "SpeculativeCCDContactDistanceUpdateTask"; }

private:
	PX_NOCOPY(SpeculativeCCDContactDistanceUpdateTask)
};

class SpeculativeCCDContactDistanceArticulationUpdateTask : public Cm::Task
{
public:
	PxReal* mContactDistances;
	const PxReal mDt;
	Sc::ArticulationSim* mArticulation;
	Bp::BoundsArray& mBoundsArray;

	SpeculativeCCDContactDistanceArticulationUpdateTask(PxU64 contextID, PxReal* contactDistances, const PxReal dt, Bp::BoundsArray& boundsArray) :
		Cm::Task			(contextID),
		mContactDistances	(contactDistances),
		mDt					(dt),
		mBoundsArray		(boundsArray)
	{
	}

	virtual void runInternal()
	{
		mArticulation->updateContactDistance(mContactDistances, mDt, mBoundsArray);
	}

	virtual const char* getName() const { return "SpeculativeCCDContactDistanceArticulationUpdateTask"; }

private:
	PX_NOCOPY(SpeculativeCCDContactDistanceArticulationUpdateTask)
};


void Sc::Scene::preRigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Scene.preNarrowPhase", getContextId());
	
	PxU32 index;

	Cm::FlushPool& pool = mLLContext->getTaskPool();

	//calculate contact distance for speculative CCD shapes
	Cm::BitMap::Iterator speculativeCCDIter(mSpeculativeCCDRigidBodyBitMap);

	SpeculativeCCDContactDistanceUpdateTask* ccdTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceUpdateTask)), SpeculativeCCDContactDistanceUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray);

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	Cm::BitMapPinned& changedMap = mAABBManager->getChangedAABBMgActorHandleMap();

	const size_t bodyOffset = PX_OFFSET_OF_RT(Sc::BodySim, getLowLevelBody());

	bool hasContactDistanceChanged = mHasContactDistanceChanged;
	while ((index = speculativeCCDIter.getNext()) != Cm::BitMap::Iterator::DONE)
	{
		PxsRigidBody* rigidBody = islandSim.getRigidBody(IG::NodeIndex(index));
		Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody)-bodyOffset);
		if (bodySim)
		{
			hasContactDistanceChanged = true;
			ccdTask->mBodySims[ccdTask->mNbBodies++] = bodySim;

			ElementSim* current = bodySim->getElements_();
			while (current)
			{
				if (static_cast<Sc::ShapeSim*>(current)->getFlags() & PxShapeFlag::eSIMULATION_SHAPE)
					changedMap.growAndSet(current->getElementID());
				current = current->mNextInActor;
			}

			if (ccdTask->mNbBodies == SpeculativeCCDContactDistanceUpdateTask::MaxBodies)
			{
				ccdTask->setContinuation(continuation);
				ccdTask->removeReference();
				ccdTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceUpdateTask)), SpeculativeCCDContactDistanceUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray);
			}
		}
	}

	if (ccdTask->mNbBodies != 0)
	{
		ccdTask->setContinuation(continuation);
		ccdTask->removeReference();
	}

	//calculate contact distance for articulation links
	SpeculativeCCDContactDistanceArticulationUpdateTask* articulationUpdateTask = NULL;

	Cm::BitMap::Iterator articulateCCDIter(mSpeculativeCDDArticulationBitMap);
	while ((index = articulateCCDIter.getNext()) != Cm::BitMap::Iterator::DONE)
	{
		Sc::ArticulationSim* articulationSim = islandSim.getLLArticulation(IG::NodeIndex(index))->getArticulationSim();
		if (articulationSim)
		{
			hasContactDistanceChanged = true;
			articulationUpdateTask = PX_PLACEMENT_NEW(pool.allocate(sizeof(SpeculativeCCDContactDistanceArticulationUpdateTask)), SpeculativeCCDContactDistanceArticulationUpdateTask)(getContextId(), mContactDistance->begin(), mDt, *mBoundsArray);
			articulationUpdateTask->mArticulation = articulationSim;
			articulationUpdateTask->setContinuation(continuation);
			articulationUpdateTask->removeReference();
		}
	}

	mHasContactDistanceChanged = hasContactDistanceChanged;
	
	//Process dirty shapeSims...
	Cm::BitMap::Iterator dirtyShapeIter(mDirtyShapeSimMap);

	PxsTransformCache& cache = mLLContext->getTransformCache();
	Bp::BoundsArray& boundsArray = mAABBManager->getBoundsArray();

	DirtyShapeUpdatesTask* task = PX_PLACEMENT_NEW(pool.allocate(sizeof(DirtyShapeUpdatesTask)), DirtyShapeUpdatesTask)(getContextId(), cache, boundsArray);

	bool hasDirtyShapes = false;
	while ((index = dirtyShapeIter.getNext()) != Cm::BitMap::Iterator::DONE)
	{
		Sc::ShapeSim* shapeSim = reinterpret_cast<Sc::ShapeSim*>(mAABBManager->getUserData(index));
		if (shapeSim)
		{
			hasDirtyShapes = true;
			changedMap.growAndSet(index);
			task->mShapes[task->mNbShapes++] = shapeSim;
			if (task->mNbShapes == DirtyShapeUpdatesTask::MaxShapes)
			{
				task->setContinuation(continuation);
				task->removeReference();
				task = PX_PLACEMENT_NEW(pool.allocate(sizeof(DirtyShapeUpdatesTask)), DirtyShapeUpdatesTask)(getContextId(), cache, boundsArray);
			}
		}
	}

	if (hasDirtyShapes)
	{
		//Setting the boundsArray and transform cache as dirty so that they get DMAd to GPU if GPU dynamics and BP are being used respectively.
		//These bits are no longer set when we update the cached state for actors due to an optimization avoiding setting these dirty bits multiple times.
		getBoundsArray().setChangedState();
		getLowLevelContext()->getTransformCache().setChangedState();
	}

	if (task->mNbShapes != 0)
	{
		task->setContinuation(continuation);
		task->removeReference();
	}

	mDirtyShapeSimMap.clear();
}

void Sc::Scene::rigidBodyNarrowPhase(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.narrowPhase", getContextId());

	mCCDPass = 0;

	mPostBroadPhase3.addDependent(*continuation);
	mPostBroadPhase2.setContinuation(&mPostBroadPhase3);
	mPostBroadPhaseCont.setContinuation(&mPostBroadPhase2);
	mPostBroadPhase.setContinuation(&mPostBroadPhaseCont);
	mBroadPhase.setContinuation(&mPostBroadPhase);
	mRigidBodyNPhaseUnlock.setContinuation(continuation);
	mRigidBodyNPhaseUnlock.addReference();
	
	mLLContext->resetThreadContexts();

	mLLContext->updateContactManager(mDt, mBoundsArray->hasChanged(), mHasContactDistanceChanged, continuation, &mRigidBodyNPhaseUnlock); // Starts update of contact managers

	mPostBroadPhase3.removeReference();
	mPostBroadPhase2.removeReference();
	mPostBroadPhaseCont.removeReference();
	mPostBroadPhase.removeReference();
	mBroadPhase.removeReference();
}

void Sc::Scene::unblockNarrowPhase(PxBaseTask*)
{
	this->mLLContext->getNphaseImplementationContext()->startNarrowPhaseTasks();
}

void Sc::Scene::postNarrowPhase(PxBaseTask* /*continuation*/)
{
	mHasContactDistanceChanged = false;
	mLLContext->fetchUpdateContactManager(); //Sync on contact gen results!

	releaseConstraints(false);

	PX_PROFILE_STOP_CROSSTHREAD("Basic.narrowPhase", getContextId());
	PX_PROFILE_STOP_CROSSTHREAD("Basic.collision", getContextId());
}

void Sc::Scene::fetchPatchEvents(PxBaseTask*)
{
	PxU32 foundPatchCount, lostPatchCount;

	{
		PX_PROFILE_ZONE("Sim.preIslandGen.managerPatchEvents", getContextId());
		mLLContext->getManagerPatchEventCount(foundPatchCount, lostPatchCount);

		mFoundPatchManagers.forceSize_Unsafe(0);
		mFoundPatchManagers.resizeUninitialized(foundPatchCount);

		mLostPatchManagers.forceSize_Unsafe(0);
		mLostPatchManagers.resizeUninitialized(lostPatchCount);

		mLLContext->fillManagerPatchChangedEvents(mFoundPatchManagers.begin(), foundPatchCount, mLostPatchManagers.begin(), lostPatchCount);

		mFoundPatchManagers.forceSize_Unsafe(foundPatchCount);
		mLostPatchManagers.forceSize_Unsafe(lostPatchCount);
	}
}

void Sc::Scene::processNarrowPhaseTouchEvents()
{
	PxsContext* context = mLLContext;

	{
		PX_PROFILE_ZONE("Sim.preIslandGen", getContextId());

		// Update touch states from LL
		PxU32 newTouchCount, lostTouchCount;
		PxU32 ccdTouchCount = 0;
		{
			PX_PROFILE_ZONE("Sim.preIslandGen.managerTouchEvents", getContextId());
			context->getManagerTouchEventCount(reinterpret_cast<PxI32*>(&newTouchCount), reinterpret_cast<PxI32*>(&lostTouchCount), NULL);
			//PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
			//PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);

			mTouchFoundEvents.forceSize_Unsafe(0);
			mTouchFoundEvents.reserve(newTouchCount);
			mTouchFoundEvents.forceSize_Unsafe(newTouchCount);

			mTouchLostEvents.forceSize_Unsafe(0);
			mTouchLostEvents.reserve(lostTouchCount);
			mTouchLostEvents.forceSize_Unsafe(lostTouchCount);

			{
				context->fillManagerTouchEvents(mTouchFoundEvents.begin(), reinterpret_cast<PxI32&>(newTouchCount), mTouchLostEvents.begin(),
					reinterpret_cast<PxI32&>(lostTouchCount), NULL, reinterpret_cast<PxI32&>(ccdTouchCount));

				mTouchFoundEvents.forceSize_Unsafe(newTouchCount);
				mTouchLostEvents.forceSize_Unsafe(lostTouchCount);
			}
		}

		context->getSimStats().mNbNewTouches = newTouchCount;
		context->getSimStats().mNbLostTouches = lostTouchCount;
	}
}

class InteractionNewTouchTask : public Cm::Task
{
	PxvContactManagerTouchEvent* mEvents;
	const PxU32 mNbEvents;
	PxsContactManagerOutputIterator mOutputs;
	const bool mUseAdaptiveForce;

public:
	InteractionNewTouchTask(PxU64 contextID, PxvContactManagerTouchEvent* events, PxU32 nbEvents, PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce) :
		Cm::Task			(contextID),
		mEvents				(events),
		mNbEvents			(nbEvents),
		mOutputs			(outputs),
		mUseAdaptiveForce	(useAdaptiveForce)
	{
	}

	virtual const char* getName() const
	{
		return "InteractionNewTouchTask";
	}
	
	void hackInContinuation(PxBaseTask* cont)
	{
		PX_ASSERT(mCont == NULL);
		mCont = cont;
		if (mCont)
			mCont->addReference();
	}

	virtual void runInternal()
	{
		for (PxU32 i = 0; i < mNbEvents; ++i)
		{
			Sc::ShapeInteraction* si = reinterpret_cast<Sc::ShapeInteraction*>(mEvents[i].userData);
			PX_ASSERT(si);
			si->managerNewTouch(0, true, mOutputs, mUseAdaptiveForce);
		}
	}
private:
	PX_NOCOPY(InteractionNewTouchTask)
};

void Sc::Scene::processNarrowPhaseTouchEventsStage2(PxBaseTask* continuation)
{
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

	//Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	PxU32 newTouchCount = mTouchFoundEvents.size();

	{
		const PxU32 nbPerTask = 256;
		PX_PROFILE_ZONE("Sim.preIslandGen.newTouches", getContextId());

		InteractionNewTouchTask* prevTask = NULL;

		//for (PxU32 i = 0; i < newTouchCount; ++i)
		for (PxU32 i = 0; i < newTouchCount; i+= nbPerTask)
		{
			const PxU32 nbToProcess = PxMin(newTouchCount - i, nbPerTask);

			for (PxU32 a = 0; a < nbToProcess; ++a)
			{
				ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchFoundEvents[i + a].userData);
				PX_ASSERT(si);
				mNPhaseCore->managerNewTouch(*si);
				si->managerNewTouch(0, true, outputs, useAdaptiveForce);
			}

			//InteractionNewTouchTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(InteractionNewTouchTask)), InteractionNewTouchTask)(mTouchFoundEvents.begin() + i, nbToProcess, outputs);
			////task->setContinuation(continuation);
			//task->setContinuation(*continuation->getTaskManager(), NULL);
			//if (prevTask)
			//{
			//	prevTask->hackInContinuation(task);
			//	prevTask->removeReference();
			//}

			//prevTask = task;

			//task->removeReference();
		}

		if (prevTask)
		{
			prevTask->hackInContinuation(continuation);
			prevTask->removeReference();
		}
	}

	/*{
		PX_PROFILE_ZONE("Sim.preIslandGen.newTouchesInteraction", getContextId());
		for (PxU32 i = 0; i < newTouchCount; ++i)
		{
			ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchFoundEvents[i].userData);
			PX_ASSERT(si);
			si->managerNewTouch(0, true, outputs);
		}
	}*/
	
}

void Sc::Scene::setEdgesConnected(PxBaseTask*)
{
	{
		PxU32 newTouchCount = mTouchFoundEvents.size();
		PX_PROFILE_ZONE("Sim.preIslandGen.islandTouches", getContextId());
		{
			PX_PROFILE_ZONE("Sim.preIslandGen.setEdgesConnected", getContextId());
			for (PxU32 i = 0; i < newTouchCount; ++i)
			{
				ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchFoundEvents[i].userData);
				if (!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				{
					mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex());
				}
			}
		}

		mSimpleIslandManager->secondPassIslandGen();

		wakeObjectsUp(ActorSim::AS_PART_OF_ISLAND_GEN);
	}
}

void Sc::Scene::processNarrowPhaseLostTouchEventsIslands(PxBaseTask*)
{
	{
		PX_PROFILE_ZONE("Sc::Scene.islandLostTouches", getContextId());
		for (PxU32 i = 0; i < mTouchLostEvents.size(); ++i)
		{
			ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchLostEvents[i].userData);
			mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
		}
	}
}

void Sc::Scene::processNarrowPhaseLostTouchEvents(PxBaseTask*)
{
	{
		PX_PROFILE_ZONE("Sc::Scene.processNarrowPhaseLostTouchEvents", getContextId());
		PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
		bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;
		for (PxU32 i = 0; i < mTouchLostEvents.size(); ++i)
		{
			ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(mTouchLostEvents[i].userData);
			PX_ASSERT(si);
			if (si->managerLostTouch(0, true, outputs, useAdaptiveForce) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
				addToLostTouchList(si->getShape0().getBodySim(), si->getShape1().getBodySim());
		}
	}
}

void Sc::Scene::processLostSolverPatches(PxBaseTask* /*continuation*/)
{
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	mDynamicsContext->processLostPatches(*mSimpleIslandManager, mLostPatchManagers.begin(), mLostPatchManagers.size(), outputs);
}

void Sc::Scene::islandGen(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", getContextId());

	//mLLContext->runModifiableContactManagers(); //KS - moved here so that we can get up-to-date touch found/lost events in IG

	mProcessLostPatchesTask.setContinuation(&mUpdateDynamics);
	mFetchPatchEventsTask.setContinuation(&mProcessLostPatchesTask);
	mProcessLostPatchesTask.removeReference();
	mFetchPatchEventsTask.removeReference();
	processNarrowPhaseTouchEvents();

	mSetEdgesConnectedTask.setContinuation(continuation);
	mSetEdgesConnectedTask.removeReference();

	processNarrowPhaseTouchEventsStage2(continuation);	
}

PX_FORCE_INLINE void Sc::Scene::putObjectsToSleep(PxU32 infoFlag)
{
	PX_PROFILE_ZONE("Sc::Scene::putObjectsToSleep", getContextId());
	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	//Set to sleep all bodies that were in awake islands that have just been put to sleep.
	const PxU32 nbBodiesToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	const IG::NodeIndex*const bodyIndices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

	for(PxU32 i=0;i<nbBodiesToSleep;i++)
	{
		PxsRigidBody* rigidBody = islandSim.getRigidBody(bodyIndices[i]);
		if (rigidBody && !islandSim.getNode(bodyIndices[i]).isActive())
		{
			Sc::BodySim* bodySim = reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - Sc::BodySim::getRigidBodyOffset());
			bodySim->setActive(false, infoFlag);
		}
	}

	const PxU32 nbArticulationsToSleep = islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);
	const IG::NodeIndex*const articIndices = islandSim.getNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	for(PxU32 i=0;i<nbArticulationsToSleep;i++)
	{
		Sc::ArticulationSim* articSim = islandSim.getLLArticulation(articIndices[i])->getArticulationSim();
		if (articSim && !islandSim.getNode(articIndices[i]).isActive())
			articSim->setActive(false, infoFlag);
	}
}

PX_FORCE_INLINE void Sc::Scene::putInteractionsToSleep()
{
	PX_PROFILE_ZONE("Sc::Scene::putInteractionsToSleep", getContextId());
	const IG::IslandSim& islandSim = mSimpleIslandManager->getSpeculativeIslandSim();

	//KS - only deactivate contact managers based on speculative state to trigger contact gen. When the actors were deactivated based on accurate state
	//joints should have been deactivated.

	{
		const PxU32 nbDeactivatingEdges = islandSim.getNbDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);
		const IG::EdgeIndex* deactivatingEdgeIds = islandSim.getDeactivatingEdges(IG::Edge::eCONTACT_MANAGER);

		for (PxU32 i = 0; i < nbDeactivatingEdges; ++i)
		{
			Sc::Interaction* interaction = mSimpleIslandManager->getInteraction(deactivatingEdgeIds[i]);

			if (interaction && interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
			{
				if (!islandSim.getEdge(deactivatingEdgeIds[i]).isActive())
				{
					const bool proceed = deactivateInteraction(interaction);
					if (proceed && (interaction->getType() < InteractionType::eTRACKED_IN_SCENE_COUNT))
						notifyInteractionDeactivated(interaction);
				}
			}
		}
	}
}

PX_FORCE_INLINE void Sc::Scene::wakeObjectsUp(PxU32 infoFlag)
{
	//Wake up all bodies that were in sleeping islands that have just been hit by a moving object.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbBodiesToWake = islandSim.getNbNodesToActivate(IG::Node::eRIGID_BODY_TYPE);
	const IG::NodeIndex*const bodyIndices = islandSim.getNodesToActivate(IG::Node::eRIGID_BODY_TYPE);

	for(PxU32 i=0;i<nbBodiesToWake;i++)
	{
		PxsRigidBody* rigidBody = islandSim.getRigidBody(bodyIndices[i]);
		if (rigidBody && islandSim.getNode(bodyIndices[i]).isActive())
		{
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigidBody) - Sc::BodySim::getRigidBodyOffset());
			bodySim->setActive(true, infoFlag);
		}
	}

	const PxU32 nbArticulationsToWake = islandSim.getNbNodesToActivate(IG::Node::eARTICULATION_TYPE);
	const IG::NodeIndex*const articIndices = islandSim.getNodesToActivate(IG::Node::eARTICULATION_TYPE);

	for(PxU32 i=0;i<nbArticulationsToWake;i++)
	{
		Sc::ArticulationSim* articSim = islandSim.getLLArticulation(articIndices[i])->getArticulationSim();
		if (articSim && islandSim.getNode(articIndices[i]).isActive())
			articSim->setActive(true, infoFlag);
	}
}

void Sc::Scene::postIslandGen(PxBaseTask* continuationTask)
{
	PX_PROFILE_ZONE("Sim.postIslandGen", getContextId());

	// - Performs collision detection for trigger interactions
	mNPhaseCore->processTriggerInteractions(continuationTask);
}

void Sc::Scene::solver(PxBaseTask* continuation)
{
	PX_PROFILE_STOP_CROSSTHREAD("Basic.narrowPhase", getContextId());
	PX_PROFILE_START_CROSSTHREAD("Basic.rigidBodySolver", getContextId());
	//Update forces per body in parallel. This can overlap with the other work in this phase.
	beforeSolver(continuation);

	PX_PROFILE_ZONE("Sim.postNarrowPhaseSecondPass", getContextId());
	//Narrowphase is completely finished so the streams can be swapped.
	mLLContext->swapStreams();

	//PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

void Sc::Scene::updateBodiesAndShapes(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	//dma bodies and shapes data to gpu
	mSimulationController->updateBodiesAndShapes(continuation);
}

Cm::FlushPool* Sc::Scene::getFlushPool()
{
	return &mLLContext->getTaskPool();
}

void Sc::Scene::postThirdPassIslandGen(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::postThirdPassIslandGen", getContextId());
	putObjectsToSleep(ActorSim::AS_PART_OF_ISLAND_GEN);
	putInteractionsToSleep();

	PxsContactManagerOutputIterator outputs = this->mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	mNPhaseCore->processPersistentContactEvents(outputs, continuation);
}

void Sc::Scene::processLostContacts(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::processLostContacts", getContextId());
	mProcessNarrowPhaseLostTouchTasks.setContinuation(continuation);
	mProcessNarrowPhaseLostTouchTasks.removeReference();

	//mLostTouchReportsTask.setContinuation(&mProcessLostContactsTask3);
	mProcessNPLostTouchEvents.setContinuation(continuation);
	mProcessNPLostTouchEvents.removeReference();

	{
		PX_PROFILE_ZONE("Sim.findInteractionsPtrs", getContextId());

		Bp::AABBManager* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
			ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);
			p->mPairUserData = mNPhaseCore->onOverlapRemovedStage1(volume0, volume1);
			p++;
		}
	}
}

void Sc::Scene::lostTouchReports(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.lostTouchReports", getContextId());
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

	Bp::AABBManager* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;

	{
		const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			if(p->mPairUserData)
			{
				Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP)
					mNPhaseCore->lostTouchReports(static_cast<Sc::ShapeInteraction*>(elemInteraction), PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), 0, outputs, useAdaptiveForce);
			}
			p++;
		}
	}
}

void Sc::Scene::unregisterInteractions(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.unregisterInteractions", getContextId());

	Bp::AABBManager* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;

	{
		const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
		while(destroyedOverlapCount--)
		{
			if(p->mPairUserData)
			{
				Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP || elemInteraction->getType() == Sc::InteractionType::eMARKER)
				{
					unregisterInteraction(elemInteraction);
					mNPhaseCore->unregisterInteraction(elemInteraction);
				}
			}
			p++;
		}
	}
}

void Sc::Scene::destroyManagers(PxBaseTask*)
{
	PX_PROFILE_ZONE("Sim.destroyManagers", getContextId());

	mPostThirdPassIslandGenTask.setContinuation(mProcessLostContactsTask3.getContinuation());

	mSimpleIslandManager->thirdPassIslandGen(&mPostThirdPassIslandGenTask);

	Bp::AABBManager* aabbMgr = mAABBManager;
	PxU32 destroyedOverlapCount;
	const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
	while(destroyedOverlapCount--)
	{
		if(p->mPairUserData)
		{
			Sc::ElementSimInteraction* elemInteraction = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
			if(elemInteraction->getType() == Sc::InteractionType::eOVERLAP)
			{
				Sc::ShapeInteraction* si = static_cast<Sc::ShapeInteraction*>(elemInteraction);
				if(si->getContactManager())
					si->destroyManager();
			}
		}
		p++;
	}
}

void Sc::Scene::processLostContacts2(PxBaseTask* continuation)
{
	mDestroyManagersTask.setContinuation(continuation);
	mLostTouchReportsTask.setContinuation(&mDestroyManagersTask);
	mLostTouchReportsTask.removeReference();
	

	mUnregisterInteractionsTask.setContinuation(continuation);
	mUnregisterInteractionsTask.removeReference();
	
	{
		PX_PROFILE_ZONE("Sim.clearIslandData", getContextId());
//		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		Bp::AABBManager* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;
		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				Sc::ElementSimInteraction* pair = reinterpret_cast<Sc::ElementSimInteraction*>(p->mPairUserData);
				if(pair)
				{
					if(pair->getType() == InteractionType::eOVERLAP)
					{
						ShapeInteraction* si = static_cast<ShapeInteraction*>(pair);
						si->clearIslandGenData();
					}
				}
				p++;
			}
		}
	}

	mDestroyManagersTask.removeReference();
}

void Sc::Scene::processLostContacts3(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processLostOverlapsStage2", getContextId());

		bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;
		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		Bp::AABBManager* aabbMgr = mAABBManager;
		PxU32 destroyedOverlapCount;

		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, p->mPairUserData, outputs, useAdaptiveForce);
				p++;
			}
		}

		{
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);
			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				mNPhaseCore->onOverlapRemoved(volume0, volume1, false, NULL, outputs, useAdaptiveForce);
				p++;
			}
		}

		aabbMgr->getBroadPhase()->deletePairs();
		aabbMgr->freeBuffers();
	}

	mPostThirdPassIslandGenTask.removeReference();
}

//This is called after solver finish
void Sc::Scene::updateSimulationController(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateSimulationController", getContextId());
	//for pxgdynamicscontext: copy solver body data to body core 
	mDynamicsContext->updateBodyCore(continuation);

	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();
	
	Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
	//changedAABBMgrActorHandles.resizeAndClear(getElementIDPool().getMaxID());

	mSimulationController->gpuDmabackData(cache, boundArray, changedAABBMgrActorHandles);
	//mSimulationController->update(cache, boundArray, changedAABBMgrActorHandles);
}

void Sc::Scene::updateDynamics(PxBaseTask* continuation)
{
	//Allow processLostContactsTask to run until after 2nd pass of solver completes (update bodies, run sleeping logic etc.)
	mProcessLostContactsTask3.setContinuation(static_cast<PxLightCpuTask*>(continuation)->getContinuation());
	mProcessLostContactsTask2.setContinuation(&mProcessLostContactsTask3);
	mProcessLostContactsTask.setContinuation(&mProcessLostContactsTask2);

	////dma bodies and shapes data to gpu
	//mSimulationController->updateBodiesAndShapes();

	mLLContext->getNpMemBlockPool().acquireConstraintMemory();

	PX_PROFILE_START_CROSSTHREAD("Basic.dynamics", getContextId());
	PxU32 maxPatchCount = mLLContext->getMaxPatchCount();

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	PxsContactManagerOutput* cmOutputBase = mLLContext->getNphaseImplementationContext()->getGPUContactManagerOutputBase();

	Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
	changedAABBMgrActorHandles.resizeAndClear(getElementIDPool().getMaxID());

	//mNPhaseCore->processPersistentContactEvents(outputs, continuation);

	mDynamicsContext->update(*mSimpleIslandManager, continuation, &mProcessLostContactsTask,
		mFoundPatchManagers.begin(), mFoundPatchManagers.size(), mLostPatchManagers.begin(), mLostPatchManagers.size(),
		maxPatchCount, outputs, cmOutputBase, mDt, mGravity, changedAABBMgrActorHandles.getWordCount());

	mSimpleIslandManager->clearDestroyedEdges();

	mProcessLostContactsTask3.removeReference();
	mProcessLostContactsTask2.removeReference();
	mProcessLostContactsTask.removeReference();
}

void Sc::Scene::updateCCDMultiPass(PxBaseTask* parentContinuation)
{
	getCcdBodies().forceSize_Unsafe(mSimulationControllerCallback->getNbCcdBodies());
	
	// second run of the broadphase for making sure objects we have integrated did not tunnel.
	if(mPublicFlags & PxSceneFlag::eENABLE_CCD)
	{
		if (mContactReportsNeedPostSolverVelocity)
		{
			// the CCD code will overwrite the post solver body velocities, hence, we need to extract the info
			// first if any CCD enabled pair requested it.
			collectPostSolverVelocitiesBeforeCCD();
		}

		//We use 2 CCD task chains to be able to chain together an arbitrary number of ccd passes
		if(mPostCCDPass.size() != 2)
		{
			mPostCCDPass.clear();
			mUpdateCCDSinglePass.clear();
			mCCDBroadPhase.clear();
			mCCDBroadPhaseAABB.clear();
			mPostCCDPass.reserve(2);
			mUpdateCCDSinglePass.reserve(2);
			mUpdateCCDSinglePass2.reserve(2);
			mUpdateCCDSinglePass3.reserve(2);
			mCCDBroadPhase.reserve(2);
			mCCDBroadPhaseAABB.reserve(2);
			for (int j = 0; j < 2; j++)
			{
				mPostCCDPass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::postCCDPass>(getContextId(), this, "ScScene.postCCDPass"));
				mUpdateCCDSinglePass.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePass>(getContextId(), this, "ScScene.updateCCDSinglePass"));
				mUpdateCCDSinglePass2.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage2>(getContextId(), this, "ScScene.updateCCDSinglePassStage2"));
				mUpdateCCDSinglePass3.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::updateCCDSinglePassStage3>(getContextId(), this, "ScScene.updateCCDSinglePassStage3"));
				mCCDBroadPhase.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhase>(getContextId(), this, "ScScene.ccdBroadPhase"));
				mCCDBroadPhaseAABB.pushBack(Cm::DelegateTask<Sc::Scene, &Sc::Scene::ccdBroadPhaseAABB>(getContextId(), this, "ScScene.ccdBroadPhaseAABB"));
			}
		}

		//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
		mLLContext->resetThreadContexts();

		mCCDContext->updateCCDBegin();

		mCCDBroadPhase[0].setContinuation(parentContinuation);
		mCCDBroadPhaseAABB[0].setContinuation(&mCCDBroadPhase[0]);
		mCCDBroadPhase[0].removeReference();
		mCCDBroadPhaseAABB[0].removeReference();
	}
}

class UpdateCCDBoundsTask : public Cm::Task
{

	Sc::BodySim** mBodySims;
	PxU32 mNbToProcess;
	PxI32* mNumFastMovingShapes;

public:

	static const PxU32 MaxPerTask = 256;

	UpdateCCDBoundsTask(PxU64 contextID, Sc::BodySim** bodySims, PxU32 nbToProcess, PxI32* numFastMovingShapes) :
		Cm::Task			(contextID),
		mBodySims			(bodySims), 
		mNbToProcess		(nbToProcess),
		mNumFastMovingShapes(numFastMovingShapes)
	{
	}

	virtual const char* getName() const { return "UpdateCCDBoundsTask";}

	virtual void runInternal()
	{
		PxU32 activeShapes = 0;
		for (PxU32 i = 0; i < mNbToProcess; i++)
		{
			PxU32 isFastMoving = 0;
			Sc::BodySim& bodySim = *mBodySims[i];

			Sc::ElementSim* current = bodySim.getElements_();
			while(current)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
				if(sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
				{
					Ps::IntBool fastMovingShape = sim->updateSweptBounds();
					activeShapes += fastMovingShape;

					isFastMoving = isFastMoving | fastMovingShape;
				}
				current = current->mNextInActor;
			}

			bodySim.getLowLevelBody().getCore().isFastMoving = PxU16(isFastMoving);
		}

		Ps::atomicAdd(mNumFastMovingShapes, PxI32(activeShapes));
	}
};

void Sc::Scene::ccdBroadPhaseAABB(PxBaseTask* continuation)
{
	PX_PROFILE_START_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());
	PX_PROFILE_ZONE("Sim.ccdBroadPhaseAABB", getContextId());
	PX_UNUSED(continuation);

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mNumFastMovingShapes = 0;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if( currentPass == 0 || mCCDContext->getNumSweepHits())
	{
		for (PxU32 i = 0; i < mCcdBodies.size(); i+= UpdateCCDBoundsTask::MaxPerTask)
		{
			const PxU32 nbToProcess = PxMin(UpdateCCDBoundsTask::MaxPerTask, mCcdBodies.size() - i);
			UpdateCCDBoundsTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateCCDBoundsTask)), UpdateCCDBoundsTask)(getContextId(), &mCcdBodies[i], nbToProcess, &mNumFastMovingShapes);
			task->setContinuation(continuation);
			task->removeReference();
		}
	}
}

void Sc::Scene::ccdBroadPhase(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.ccdBroadPhase", getContextId());

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	const PxU32 ccdMaxPasses = mCCDContext->getCCDMaxPasses();
	mCCDPass = currentPass+1;

	//If we are on the 1st pass or we had some sweep hits previous CCD pass, we need to run CCD again
	if( (currentPass == 0 || mCCDContext->getNumSweepHits()) && mNumFastMovingShapes != 0)
	{
		const PxU32 currIndex = currentPass & 1;
		const PxU32 nextIndex = 1 - currIndex;
		//Initialize the CCD task chain unless this is the final pass
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].setContinuation(continuation);
			mCCDBroadPhaseAABB[nextIndex].setContinuation(&mCCDBroadPhase[nextIndex]);
		}
		mPostCCDPass[currIndex].setContinuation(currentPass == ccdMaxPasses-1 ? continuation : &mCCDBroadPhaseAABB[nextIndex]);
		mUpdateCCDSinglePass3[currIndex].setContinuation(&mPostCCDPass[currIndex]);
		mUpdateCCDSinglePass2[currIndex].setContinuation(&mUpdateCCDSinglePass3[currIndex]);
		mUpdateCCDSinglePass[currIndex].setContinuation(&mUpdateCCDSinglePass2[currIndex]);

		//Do the actual broad phase
		PxBaseTask* continuationTask = &mUpdateCCDSinglePass[currIndex];
		const PxU32 numCpuTasks = continuationTask->getTaskManager()->getCpuDispatcher()->getWorkerCount();
		
		mAABBManager->updateAABBsAndBP(numCpuTasks, mLLContext->getTaskPool(), &mLLContext->getScratchAllocator(), false, continuationTask, NULL);

		//Allow the CCD task chain to continue
		mPostCCDPass[currIndex].removeReference();
		mUpdateCCDSinglePass3[currIndex].removeReference();
		mUpdateCCDSinglePass2[currIndex].removeReference();
		mUpdateCCDSinglePass[currIndex].removeReference();
		if(currentPass != (ccdMaxPasses - 1))
		{
			mCCDBroadPhase[nextIndex].removeReference();
			mCCDBroadPhaseAABB[nextIndex].removeReference();
		}
	}
	else if (currentPass == 0)
	{
		PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());
		mCCDContext->resetContactManagers();
	}
}

void Sc::Scene::updateCCDSinglePass(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePass", getContextId());
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	mAABBManager->postBroadPhase(NULL, NULL, *getFlushPool());
	finishBroadPhase(continuation);
	
	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	if(currentPass == 1)		// reset the handle map so we only update CCD objects from here on
	{
		Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
		//changedAABBMgrActorHandles.clear();
		for(PxU32 i = 0; i < mCcdBodies.size();i++)
		{
			Sc::ElementSim* current = mCcdBodies[i]->getElements_();
			while(current)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
				if(sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))	// TODO: need trigger shape here?
					changedAABBMgrActorHandles.growAndSet(sim->getElementID());
				current = current->mNextInActor;
			}
		}
	}
}

void Sc::Scene::updateCCDSinglePassStage2(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage2", getContextId());
	postBroadPhaseStage2(continuation);
}

void Sc::Scene::updateCCDSinglePassStage3(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateCCDSinglePassStage3", getContextId());
	mReportShapePairTimeStamp++;  // This will makes sure that new report pairs will get created instead of re-using the existing ones.

	const PxU32 currentPass = mCCDContext->getCurrentCCDPass() + 1;  // 0 is reserved for discrete collision phase
	finishBroadPhaseStage2(currentPass);
	PX_PROFILE_STOP_CROSSTHREAD("Sim.ccdBroadPhaseComplete", getContextId());

	//reset thread context in a place we know all tasks possibly accessing it, are in sync with. (see US6664)
	mLLContext->resetThreadContexts();

	mCCDContext->updateCCD(mDt, continuation, mSimpleIslandManager->getAccurateIslandSim(), (mPublicFlags & PxSceneFlag::eDISABLE_CCD_RESWEEP), mNumFastMovingShapes);
}

class ScKinematicPoseUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	PxU32					mNbKinematics;

public:
	static const PxU32 NbKinematicsPerTask = 1024;

	ScKinematicPoseUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbKinematics; ++a)
		{
			if ((a + 16) < mNbKinematics)
			{
				Ps::prefetchLine(mKinematics[a + 16]);

				if ((a + 4) < mNbKinematics)
				{
					Ps::prefetchLine(mKinematics[a + 4]->getSim());
					Ps::prefetchLine(mKinematics[a + 4]->getSimStateData_Unchecked());
				}
			}
			Sc::BodyCore* b = mKinematics[a];
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());
			b->getSim()->updateKinematicPose();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScKinematicPoseUpdateTask";
	}
};

void Sc::Scene::integrateKinematicPose()
{
	PX_PROFILE_ZONE("Sim.integrateKinematicPose", getContextId());

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	for(PxU32 i=0; i<nbKinematics; i+= ScKinematicPoseUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicPoseUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicPoseUpdateTask)), ScKinematicPoseUpdateTask)
			(kinematics + i, PxMin(nbKinematics - i, ScKinematicPoseUpdateTask::NbKinematicsPerTask), mContextId);
		task->setContinuation(&mConstraintProjection);
		task->removeReference();
	}
}

class ScKinematicShapeUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	PxU32					mNbKinematics;
	PxsTransformCache&		mCache;
	Bp::BoundsArray&		mBoundsArray;

	PX_NOCOPY(ScKinematicShapeUpdateTask)

public:
	static const PxU32 NbKinematicsShapesPerTask = 1024;

	ScKinematicShapeUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxsTransformCache& cache, Bp::BoundsArray& boundsArray, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mCache(cache), mBoundsArray(boundsArray)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbKinematics; ++a)
		{
			Sc::BodyCore* b = mKinematics[a];
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->updateCached(mCache, mBoundsArray);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicShapeUpdateTask";
	}
};

void Sc::Scene::updateKinematicCached(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateKinematicCached", getContextId());

	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();
	
	PxU32 startIndex = 0;
	PxU32 nbShapes = 0;

	{
		PX_PROFILE_ZONE("ShapeUpdate", getContextId());
		for(PxU32 i=0; i<nbKinematics; i++)
		{
			BodyCore* b = kinematics[i];
			BodySim* sim = b->getSim();
			PX_ASSERT(sim->isKinematic());
			PX_ASSERT(sim->isActive());

			nbShapes += sim->getNbShapes();

			if (nbShapes >= ScKinematicShapeUpdateTask::NbKinematicsShapesPerTask)
			{
				ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
					(kinematics + startIndex, (i + 1) - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

				task->setContinuation(continuation);
				task->removeReference();
				startIndex = i + 1;
				nbShapes = 0;
			}
		}

		if(nbShapes)
		{
			ScKinematicShapeUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicShapeUpdateTask)), ScKinematicShapeUpdateTask)
				(kinematics + startIndex, nbKinematics - startIndex, mLLContext->getTransformCache(), *mBoundsArray, mContextId);

			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	if(nbKinematics)
	{
		Cm::BitMapPinned& changedAABBMap = mAABBManager->getChangedAABBMgActorHandleMap();
		mLLContext->getTransformCache().setChangedState();
		mBoundsArray->setChangedState();
		for (PxU32 i = 0; i < nbKinematics; ++i)
		{
			Sc::BodySim* bodySim = kinematics[i]->getSim();

			if ((i+16) < nbKinematics)
			{
				Ps::prefetchLine(kinematics[i + 16]);
				if ((i + 8) < nbKinematics)
				{
					Ps::prefetchLine(kinematics[i + 8]->getSim());
				}
				if ((i + 4) < nbKinematics)
				{
					Ps::prefetchLine(kinematics[i + 4]->getSim()->getElements_());
				}
			}

			Sc::ElementSim* current = bodySim->getElements_();
			while(current)
			{
				Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
				//KS - TODO - can we parallelize this? The problem with parallelizing is that it's a bit operation,
				//so we would either need to use atomic operations or have some high-level concept that guarantees 
				//that threads don't write to the same word in the map simultaneously
				if (sim->getFlags()&PxU32(PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
				{
					changedAABBMap.set(sim->getElementID());
				}
				current = current->mNextInActor;
			}

			mSimulationController->updateDynamic(false, bodySim->getNodeIndex());
		}
	}
}

class ConstraintProjectionTask : public Cm::Task
{
private:
	PX_NOCOPY(ConstraintProjectionTask)

public:
	ConstraintProjectionTask(Sc::ConstraintGroupNode* const* projectionRoots, PxU32 projectionRootCount, Ps::Array<Sc::BodySim*>& projectedBodies, PxsContext* llContext) :
		Cm::Task			(llContext->getContextId()),
		mProjectionRoots	(projectionRoots),
		mProjectionRootCount(projectionRootCount),
		mProjectedBodies	(projectedBodies),
		mLLContext			(llContext)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("ConstraintProjection", mContextID);
		PxcNpThreadContext* context = mLLContext->getNpThreadContext();
		Ps::Array<Sc::BodySim*>& tempArray = context->mBodySimPool;
		tempArray.forceSize_Unsafe(0);
		for(PxU32 i=0; i < mProjectionRootCount; i++)
		{
			PX_ASSERT(mProjectionRoots[i]->hasProjectionTreeRoot());  // else, it must not be in the projection root list
			Sc::ConstraintGroupNode::projectPose(*mProjectionRoots[i], tempArray);
			mProjectionRoots[i]->clearFlag(Sc::ConstraintGroupNode::eIN_PROJECTION_PASS_LIST);
		}

		if (tempArray.size() > 0)
		{
			mLLContext->getLock().lock();
			for (PxU32 a = 0; a < tempArray.size(); ++a)
				mProjectedBodies.pushBack(tempArray[a]);
			mLLContext->getLock().unlock();
		}

		mLLContext->putNpThreadContext(context);
	}

	virtual const char* getName() const
	{
		return "ScScene.constraintProjectionWork";
	}

public:
	static const PxU32 sProjectingConstraintsPerTask = 256;  // just a guideline, will not match exactly most of the time

private:
	Sc::ConstraintGroupNode* const* mProjectionRoots;
	const PxU32 mProjectionRootCount;
	Ps::Array<Sc::BodySim*>& mProjectedBodies;
	PxsContext* mLLContext;
};

void Sc::Scene::constraintProjection(PxBaseTask* continuation)
{
	if (mConstraints.size() == 0)
		return;
	PxU32 constraintGroupRootCount = 0;
	//BodyCore*const* activeBodies = getActiveBodiesArray();
	//PxU32 activeBodyCount = getNumActiveBodies();
	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();
	PxU32 activeBodyCount = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);
	const IG::NodeIndex* activeNodeIds = islandSim.getActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	PX_ASSERT(!mTmpConstraintGroupRootBuffer);
	PxU32 index = 0;

	const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

	if(activeBodyCount)
	{
		mTmpConstraintGroupRootBuffer = reinterpret_cast<ConstraintGroupNode**>(mLLContext->getScratchAllocator().alloc(sizeof(ConstraintGroupNode*) * activeBodyCount, true));
		if(mTmpConstraintGroupRootBuffer)
		{
			while(activeBodyCount--)
			{
				PxsRigidBody* rBody = islandSim.getRigidBody(activeNodeIds[index++]);

				Sc::BodySim* sim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rBody) - rigidBodyOffset);
				//This move to PxgPostSolveWorkerTask for the gpu dynamic
				//bodySim->sleepCheck(mDt, mOneOverDt, mEnableStabilization);

				if(sim->getConstraintGroup())
				{
					ConstraintGroupNode& root = sim->getConstraintGroup()->getRoot();
					if(!root.readFlag(ConstraintGroupNode::eIN_PROJECTION_PASS_LIST) && root.hasProjectionTreeRoot())
					{
						mTmpConstraintGroupRootBuffer[constraintGroupRootCount++] = &root;
						root.raiseFlag(ConstraintGroupNode::eIN_PROJECTION_PASS_LIST);
					}
				}
			}

			Cm::FlushPool& flushPool = mLLContext->getTaskPool();

			PxU32 constraintsToProjectCount = 0;
			PxU32 startIndex = 0;
			for(PxU32 i=0; i < constraintGroupRootCount; i++)
			{
				ConstraintGroupNode* root = mTmpConstraintGroupRootBuffer[i];

				constraintsToProjectCount += root->getProjectionCountHint();  // for load balancing
				if (constraintsToProjectCount >= ConstraintProjectionTask::sProjectingConstraintsPerTask)
				{
					ConstraintProjectionTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ConstraintProjectionTask)), 
																		ConstraintProjectionTask(mTmpConstraintGroupRootBuffer + startIndex, i - startIndex + 1, mProjectedBodies, mLLContext));
					task->setContinuation(continuation);
					task->removeReference();

					constraintsToProjectCount = 0;
					startIndex = i + 1;
				}
			}

			if (constraintsToProjectCount)
			{
				PX_ASSERT(startIndex < constraintGroupRootCount);

				ConstraintProjectionTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ConstraintProjectionTask)), 
																	ConstraintProjectionTask(mTmpConstraintGroupRootBuffer + startIndex, constraintGroupRootCount - startIndex, mProjectedBodies, mLLContext));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}
		else
		{
			getFoundation().getErrorCallback().reportError(PxErrorCode::eOUT_OF_MEMORY, "List for collecting constraint projection roots could not be allocated. No projection will take place.", __FILE__, __LINE__);
		}
	}
}

void Sc::Scene::postSolver(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sc::Scene::postSolver", getContextId());
	PxcNpMemBlockPool& blockPool = mLLContext->getNpMemBlockPool();

	//Merge...
	mDynamicsContext->mergeResults();
	blockPool.releaseConstraintMemory();
	//Swap friction!
	blockPool.swapFrictionStreams();

	mCcdBodies.clear();
	mProjectedBodies.clear();

#if PX_ENABLE_SIM_STATS
	mLLContext->getSimStats().mPeakConstraintBlockAllocations = blockPool.getPeakConstraintBlockCount();
#endif

	mConstraintProjection.setContinuation(continuation);

	integrateKinematicPose();

	mConstraintProjection.removeReference();

	//afterIntegration(continuation);
}

void Sc::Scene::postCCDPass(PxBaseTask* /*continuation*/)
{
	// - Performs sleep check
	// - Updates touch flags

	PxU32 currentPass = mCCDContext->getCurrentCCDPass();
	PX_ASSERT(currentPass > 0); // to make sure changes to the CCD pass counting get noticed. For contact reports, 0 means discrete collision phase.

	int newTouchCount, lostTouchCount, ccdTouchCount;
	mLLContext->getManagerTouchEventCount(&newTouchCount, &lostTouchCount, &ccdTouchCount);
	PX_ALLOCA(newTouches, PxvContactManagerTouchEvent, newTouchCount);
	PX_ALLOCA(lostTouches, PxvContactManagerTouchEvent, lostTouchCount);
	PX_ALLOCA(ccdTouches, PxvContactManagerTouchEvent, ccdTouchCount);

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

	// Note: For contact notifications it is important that the new touch pairs get processed before the lost touch pairs.
	//       This allows to know for sure if a pair of actors lost all touch (see eACTOR_PAIR_LOST_TOUCH).
	mLLContext->fillManagerTouchEvents(newTouches, newTouchCount, lostTouches, lostTouchCount, ccdTouches, ccdTouchCount);
	for(PxI32 i=0; i<newTouchCount; ++i)
	{
		ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(newTouches[i].userData);
		PX_ASSERT(si);
		mNPhaseCore->managerNewTouch(*si);
		si->managerNewTouch(currentPass, true, outputs, useAdaptiveForce);
		if (!si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
		{
			mSimpleIslandManager->setEdgeConnected(si->getEdgeIndex());
		}
	}
	for(PxI32 i=0; i<lostTouchCount; ++i)
	{
		ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(lostTouches[i].userData);
		PX_ASSERT(si);
		if (si->managerLostTouch(currentPass, true, outputs, useAdaptiveForce) && !si->readFlag(ShapeInteraction::CONTACTS_RESPONSE_DISABLED))
			addToLostTouchList(si->getShape0().getBodySim(), si->getShape1().getBodySim());

		mSimpleIslandManager->setEdgeDisconnected(si->getEdgeIndex());
	}
	for(PxI32 i=0; i<ccdTouchCount; ++i)
	{
		ShapeInteraction* si = reinterpret_cast<ShapeInteraction*>(ccdTouches[i].userData);
		PX_ASSERT(si);
		si->sendCCDRetouch(currentPass, outputs);
	}
	checkForceThresholdContactEvents(currentPass);
	{
		Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();

		for (PxU32 i = 0, s = mCcdBodies.size(); i < s; i++)
		{
			BodySim*const body = mCcdBodies[i];
			if(i+8 < s)
				Ps::prefetch(mCcdBodies[i+8], 512);

			PX_ASSERT(body->getBody2World().p.isFinite());
			PX_ASSERT(body->getBody2World().q.isFinite());

			body->updateCached(&changedAABBMgrActorHandles);
		}

		ArticulationCore* const* articList = mArticulations.getEntries();
		for(PxU32 i=0;i<mArticulations.size();i++)
		{
			articList[i]->getSim()->updateCached(&changedAABBMgrActorHandles);
		}
	}
}

void Sc::Scene::finalizationPhase(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.sceneFinalization", getContextId());

	if (mCCDContext)
	{
		//KS - force simulation controller to update any bodies updated by the CCD. When running GPU simulation, this would be required
		//to ensure that cached body states are updated
		const PxU32 nbUpdatedBodies = mCCDContext->getNumUpdatedBodies();
		PxsRigidBody*const* updatedBodies = mCCDContext->getUpdatedBodies();

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();			

		for (PxU32 a = 0; a < nbUpdatedBodies; ++a)
		{
			Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(updatedBodies[a]) - rigidBodyOffset);
			mSimulationController->updateDynamic(bodySim->isArticulationLink(), bodySim->getNodeIndex());
		}

		mCCDContext->clearUpdatedBodies();
	}

	if (mTmpConstraintGroupRootBuffer)
	{
		mLLContext->getScratchAllocator().free(mTmpConstraintGroupRootBuffer);
		mTmpConstraintGroupRootBuffer = NULL;
	}

	fireOnAdvanceCallback();  // placed here because it needs to be done after sleep check and afrer potential CCD passes

	checkConstraintBreakage(); // Performs breakage tests on breakable constraints

	PX_PROFILE_STOP_CROSSTHREAD("Basic.rigidBodySolver", getContextId());

	//KS - process deleted elementIDs now - before GPU particles releases elements, causing issues - sschirm: particles are gone...
	mElementIDPool->processPendingReleases();
	mElementIDPool->clearDeletedIDMap();

	visualizeEndStep();

	mTaskPool.clear();

	mReportShapePairTimeStamp++;	// important to do this before fetchResults() is called to make sure that delayed deleted actors/shapes get
									// separate pair entries in contact reports
}

void Sc::Scene::postReportsCleanup()
{
	mShapeIDTracker->processPendingReleases();
	mShapeIDTracker->clearDeletedIDMap();

	mRigidIDTracker->processPendingReleases();
	mRigidIDTracker->clearDeletedIDMap();

	mConstraintIDTracker->processPendingReleases();
	mConstraintIDTracker->clearDeletedIDMap();
}

void Sc::Scene::syncSceneQueryBounds(SqBoundsSync& sync, SqRefFinder& finder)
{
	mSqBoundsManager->syncBounds(sync, finder, mBoundsArray->begin(), getContextId(), mDirtyShapeSimMap);
}

class ScKinematicUpdateTask : public Cm::Task
{
	Sc::BodyCore*const*	mKinematics;
	const PxU32			mNbKinematics;
	const PxReal		mOneOverDt;

	PX_NOCOPY(ScKinematicUpdateTask)
public:

	static const PxU32 NbKinematicsPerTask = 1024;
	
	ScKinematicUpdateTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxReal oneOverDt, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mOneOverDt(oneOverDt)
	{
	}

	virtual void runInternal()
	{
		Sc::BodyCore*const*	kinematics = mKinematics;
		PxU32 nb = mNbKinematics;
		const float oneOverDt = mOneOverDt;

		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			PX_ASSERT(b->getSim()->isKinematic());
			PX_ASSERT(b->getSim()->isActive());

			b->getSim()->calculateKinematicVelocity(oneOverDt);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicUpdateTask";
	}
};

class ScKinematicAddDynamicTask : public Cm::Task
{
	Sc::BodyCore*const*		mKinematics;
	const PxU32				mNbKinematics;
	PxsSimulationController& mSimulationController;

	PX_NOCOPY(ScKinematicAddDynamicTask)

public:

	ScKinematicAddDynamicTask(Sc::BodyCore*const* kinematics, PxU32 nbKinematics, PxsSimulationController& simulationController, PxU64 contextID) :
		Cm::Task(contextID), mKinematics(kinematics), mNbKinematics(nbKinematics), mSimulationController(simulationController)
	{
	}

	virtual void runInternal()
	{
		Sc::BodyCore*const*	kinematics = mKinematics;
		PxU32 nb = mNbKinematics;

		while(nb--)
		{
			Sc::BodyCore* b = *kinematics++;
			Sc::BodySim* bodySim = b->getSim();
			mSimulationController.updateDynamic(false, bodySim->getNodeIndex());
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.KinematicAddDynamicTask";
	}
};

void Sc::Scene::kinematicsSetup(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	const PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	for(PxU32 i = 0; i < nbKinematics; i += ScKinematicUpdateTask::NbKinematicsPerTask)
	{
		ScKinematicUpdateTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicUpdateTask)), ScKinematicUpdateTask)
			(kinematics + i, PxMin(ScKinematicUpdateTask::NbKinematicsPerTask, nbKinematics - i), mOneOverDt, mContextId);

		task->setContinuation(continuation);
		task->removeReference();
	}

	if((mPublicFlags & PxSceneFlag::eENABLE_GPU_DYNAMICS))
	{
		ScKinematicAddDynamicTask* addTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScKinematicAddDynamicTask)), ScKinematicAddDynamicTask)
			(kinematics, nbKinematics, *mSimulationController, mContextId);

		addTask->setContinuation(continuation);
		addTask->removeReference();
	}
}

//stepSetup is called in solve, but not collide
void Sc::Scene::stepSetupSolve(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupSolve", getContextId());

	kinematicsSetup(continuation);
}

void Sc::Scene::stepSetupCollide(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.stepSetupCollide", getContextId());

	{
		PX_PROFILE_ZONE("Sim.projectionTreeUpdates", getContextId());
		mProjectionManager->processPendingUpdates(mLLContext->getScratchAllocator());
	}

	kinematicsSetup(continuation);
	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	// Update all dirty interactions
	mNPhaseCore->updateDirtyInteractions(outputs, mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE);
	mInternalFlags &= ~(SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE | SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_VISUALIZATION);
}

void Sc::Scene::processLostTouchPairs()
{
	PX_PROFILE_ZONE("Sc::Scene::processLostTouchPairs", getContextId());
	for (PxU32 i=0; i<mLostTouchPairs.size(); ++i)
	{
		// If one has been deleted, we wake the other one
		const Ps::IntBool deletedBody1 = mLostTouchPairsDeletedBodyIDs.boundedTest(mLostTouchPairs[i].body1ID);
		const Ps::IntBool deletedBody2 = mLostTouchPairsDeletedBodyIDs.boundedTest(mLostTouchPairs[i].body2ID);
		if (deletedBody1 || deletedBody2)
		{
			if (!deletedBody1) 
				mLostTouchPairs[i].body1->internalWakeUp();
			if (!deletedBody2) 
				mLostTouchPairs[i].body2->internalWakeUp();
			continue;
		}

		// If both are sleeping, we let them sleep
		// (for example, two sleeping objects touch and the user teleports one (without waking it up))
		if (!mLostTouchPairs[i].body1->isActive() &&
			!mLostTouchPairs[i].body2->isActive())
		{
			continue;
		}

		// If only one has fallen asleep, we wake them both
		if (!mLostTouchPairs[i].body1->isActive() ||
			!mLostTouchPairs[i].body2->isActive())
		{
			mLostTouchPairs[i].body1->internalWakeUp();
			mLostTouchPairs[i].body2->internalWakeUp();
		}
	}

	mLostTouchPairs.clear();
	mLostTouchPairsDeletedBodyIDs.clear();
}

class ScBeforeSolverTask :  public Cm::Task
{
public:
	static const PxU32 MaxBodiesPerTask = 256;
	IG::NodeIndex				mBodies[MaxBodiesPerTask];
	PxU32						mNumBodies;
	const PxReal				mDt;
	IG::SimpleIslandManager*	mIslandManager;
	PxsSimulationController*	mSimulationController;
	const bool					mSimUsesAdaptiveForce;

public:

	ScBeforeSolverTask(PxReal dt, IG::SimpleIslandManager* islandManager, PxsSimulationController* simulationController, PxU64 contextID, bool simUsesAdaptiveForce) : 
		Cm::Task				(contextID),
		mDt						(dt),
		mIslandManager			(islandManager),
		mSimulationController	(simulationController),
		mSimUsesAdaptiveForce	(simUsesAdaptiveForce)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.ScBeforeSolverTask", mContextID);
		const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();
		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		PxsRigidBody* updatedBodySims[MaxBodiesPerTask];
		PxU32 updatedBodyNodeIndices[MaxBodiesPerTask];
		PxU32 nbUpdatedBodySims = 0;

		PxU32 nb = mNumBodies;
		const IG::NodeIndex* bodies = mBodies;;
		while(nb--)
		{
			const IG::NodeIndex index = *bodies++;
			if(islandSim.getActiveNodeIndex(index) != IG_INVALID_NODE)
			{
				if (islandSim.getNode(index).mType == IG::Node::eRIGID_BODY_TYPE)
				{
					PxsRigidBody* body = islandSim.getRigidBody(index);
					Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(body) - rigidBodyOffset);
					bodySim->updateForces(mDt, updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims, NULL, false, mSimUsesAdaptiveForce);
				}
			}
		}

		if(nbUpdatedBodySims)
			mSimulationController->updateBodies(updatedBodySims, updatedBodyNodeIndices, nbUpdatedBodySims);
	}

	virtual const char* getName() const
	{
		return "ScScene.beforeSolver";
	}

private:
	PX_NOCOPY(ScBeforeSolverTask)
};


class ScArticBeforeSolverTask : public Cm::Task
{
public:
	const IG::NodeIndex* const	mArticIndices;
	const PxU32					mNumArticulations;
	const PxReal				mDt;
	IG::SimpleIslandManager*	mIslandManager;
	const bool					mSimUsesAdaptiveForce;

public:

	ScArticBeforeSolverTask(const IG::NodeIndex* const	articIndices, PxU32 nbArtics, PxReal dt, IG::SimpleIslandManager* islandManager, PxU64 contextID, bool simUsesAdaptiveForce) :
		Cm::Task(contextID),
		mArticIndices(articIndices),
		mNumArticulations(nbArtics),
		mDt(dt),
		mIslandManager(islandManager),
		mSimUsesAdaptiveForce(simUsesAdaptiveForce)
	{
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.ScArticBeforeSolverTask", mContextID);
		const IG::IslandSim& islandSim = mIslandManager->getAccurateIslandSim();

		for (PxU32 a = 0; a < mNumArticulations; ++a)
		{
			Sc::ArticulationSim* PX_RESTRICT articSim = islandSim.getLLArticulation(mArticIndices[a])->getArticulationSim();
			articSim->checkResize();
			articSim->updateForces(mDt, mSimUsesAdaptiveForce);
			articSim->saveLastCCDTransform();
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.ScArticBeforeSolverTask";
	}

private:
	PX_NOCOPY(ScArticBeforeSolverTask)
};


void Sc::Scene::beforeSolver(PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.updateForces", getContextId());

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Atoms that passed contact force threshold
	ThresholdStream& thresholdStream = mDynamicsContext->getThresholdStream();
	thresholdStream.clear();

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	const PxU32 nbActiveBodies = islandSim.getNbActiveNodes(IG::Node::eRIGID_BODY_TYPE);

	mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);
	mNumDeactivatingNodes[IG::Node::eARTICULATION_TYPE] = 0;//islandSim.getNbNodesToDeactivate(IG::Node::eARTICULATION_TYPE);

	const PxU32 MaxBodiesPerTask = ScBeforeSolverTask::MaxBodiesPerTask;

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	mSimulationController->reserve(nbActiveBodies);

	bool simUsesAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

	{
		Cm::BitMap::Iterator iter(mVelocityModifyMap);

		for (PxU32 i = iter.getNext(); i != Cm::BitMap::Iterator::DONE; /*i = iter.getNext()*/)
		{
			ScBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScBeforeSolverTask)), ScBeforeSolverTask(mDt, mSimpleIslandManager, mSimulationController, getContextId(), simUsesAdaptiveForce));
			PxU32 count = 0;
			for (; count < MaxBodiesPerTask && i != Cm::BitMap::Iterator::DONE; i = iter.getNext())
			{
				PxsRigidBody* body = islandSim.getRigidBody(IG::NodeIndex(i));
				bool retainsAccelerations = false;
				if (body)
				{
					task->mBodies[count++] = IG::NodeIndex(i);

					retainsAccelerations = (body->mCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS);
					
				}

				if(!retainsAccelerations)
				{
					mVelocityModifyMap.reset(i);
				}
			}
			task->mNumBodies = count;
			task->setContinuation(continuation);
			task->removeReference();
		}
	}

	const PxU32 nbActiveArticulations = islandSim.getNbActiveNodes(IG::Node::eARTICULATION_TYPE);
	const IG::NodeIndex* const articIndices = islandSim.getActiveNodes(IG::Node::eARTICULATION_TYPE);

	const PxU32 nbArticsPerTask = 8;
	
	for(PxU32 a = 0; a < nbActiveArticulations; a+= nbArticsPerTask)
	{
		const PxU32 nbToProcess = PxMin(PxU32(nbActiveArticulations - a), nbArticsPerTask);
		ScArticBeforeSolverTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ScArticBeforeSolverTask)), ScArticBeforeSolverTask(articIndices+a, nbToProcess, 
			mDt, mSimpleIslandManager, getContextId(), simUsesAdaptiveForce));

		task->setContinuation(continuation);
		task->removeReference();
	}

	mBodyGravityDirty = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class UpdatProjectedPoseTask : public Cm::Task
{
	Sc::BodySim** mProjectedBodies;
	const PxU32 mNbBodiesToProcess;

	PX_NOCOPY(UpdatProjectedPoseTask)
public:

	UpdatProjectedPoseTask(PxU64 contextID, Sc::BodySim** projectedBodies, PxU32 nbBodiesToProcess) :
		Cm::Task			(contextID),
		mProjectedBodies	(projectedBodies),
		mNbBodiesToProcess	(nbBodiesToProcess)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 a = 0; a < mNbBodiesToProcess; ++a)
		{
			mProjectedBodies[a]->updateCached(NULL);
		}
	}

	virtual const char* getName() const
	{
		return "ScScene.UpdatProjectedPoseTask";
	}
};

class UpdateArticulationTask : public Cm::Task
{
	Sc::ArticulationCore* const* mArticList;
	const PxU32 mNbArticulations;
	const PxReal mDt;

	PX_NOCOPY(UpdateArticulationTask)
public:

	UpdateArticulationTask(PxU64 contextId, Sc::ArticulationCore* const* articList, PxU32 nbArticulations, PxReal dt) :
		Cm::Task			(contextId),
		mArticList			(articList),
		mNbArticulations	(nbArticulations),
		mDt					(dt)
	{
	}

	virtual const char* getName() const { return "UpdateArticulationTask"; }

	virtual void runInternal()
	{
		for (PxU32 i = 0; i < mNbArticulations; ++i)
		{
			Sc::ArticulationSim* articSim = mArticList[i]->getSim();
			articSim->sleepCheck(mDt);
			articSim->updateCached(NULL);
		}
	}
};

void Sc::Scene::afterIntegration(PxBaseTask* continuation)
{		
	PX_PROFILE_ZONE("Sc::Scene::afterIntegration", getContextId());
	mLLContext->getTransformCache().resetChangedState(); //Reset the changed state. If anything outside of the GPU kernels updates any shape's transforms, this will be raised again
	getBoundsArray().resetChangedState();

	PxsTransformCache& cache = getLowLevelContext()->getTransformCache();
	Bp::BoundsArray& boundArray = getBoundsArray();

	{
		PX_PROFILE_ZONE("AfterIntegration::lockStage", getContextId());
		mLLContext->getLock().lock();
		
		mSimulationController->udpateScBodyAndShapeSim(cache, boundArray, continuation);

		const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

		const PxU32 rigidBodyOffset = Sc::BodySim::getRigidBodyOffset();

		const PxU32 numBodiesToDeactivate = islandSim.getNbNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		const IG::NodeIndex*const deactivatingIndices = islandSim.getNodesToDeactivate(IG::Node::eRIGID_BODY_TYPE);

		PxU32 previousNumBodiesToDeactivate = mNumDeactivatingNodes[IG::Node::eRIGID_BODY_TYPE];

		{
			Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
			PX_PROFILE_ZONE("AfterIntegration::deactivateStage", getContextId());
			for (PxU32 i = previousNumBodiesToDeactivate; i < numBodiesToDeactivate; i++)
			{
				PxsRigidBody* rigid = islandSim.getRigidBody(deactivatingIndices[i]);
				Sc::BodySim* bodySim = reinterpret_cast<Sc::BodySim*>(reinterpret_cast<PxU8*>(rigid) - rigidBodyOffset);
				//we need to set the rigid body back to the previous pose for the deactivated objects. This emulates the previous behavior where island gen ran before the solver, ensuring
				//that bodies that should be deactivated this frame never reach the solver. We now run the solver in parallel with island gen, so objects that should be deactivated this frame
				//still reach the solver and are integrated. However, on the frame when they should be deactivated, we roll back to their state at the beginning of the frame to ensure that the
				//user perceives the same behavior as before.

				PxsBodyCore& bodyCore = bodySim->getBodyCore().getCore();

				//if(!islandSim.getNode(bodySim->getNodeIndex()).isActive())
				rigid->setPose(rigid->getLastCCDTransform());

				bodySim->updateCached(&changedAABBMgrActorHandles);
				mSimulationController->updateDynamic(bodySim->isArticulationLink(), bodySim->getNodeIndex());

				//solver is running in parallel with IG(so solver might solving the body which IG identify as deactivatedNodes). After we moved sleepCheck into the solver after integration, sleepChecks
				//might have processed bodies that are now considered deactivated. This could have resulted in either freezing or unfreezing one of these bodies this frame, so we need to process those
				//events to ensure that the SqManager's bounds arrays are consistently maintained. Also, we need to clear the frame flags for these bodies.

				if (rigid->isFreezeThisFrame())
					bodySim->freezeTransforms(&mAABBManager->getChangedAABBMgActorHandleMap());
				/*else if(bodyCore.isUnfreezeThisFrame())
					bodySim->createSqBounds();*/

				//PX_ASSERT(bodyCore.wakeCounter == 0.0f);
				//KS - the IG deactivates bodies in parallel with the solver. It appears that under certain circumstances, the solver's integration (which performs
				//sleep checks) could decide that the body is no longer a candidate for sleeping on the same frame that the island gen decides to deactivate the island
				//that the body is contained in. This is a rare occurrence but the behavior we want to emulate is that of IG running before solver so we should therefore
				//permit the IG to make the authoritative decision over whether the body should be active or inactive.
				bodyCore.wakeCounter = 0.0f;
				bodyCore.linearVelocity = PxVec3(0.0f);
				bodyCore.angularVelocity = PxVec3(0.0f);

				rigid->clearAllFrameFlags();

				/*Sc::ShapeSim* sim;
				for (Sc::ShapeIterator iterator(*bodySim); (sim = iterator.getNext()) != NULL;)
				{
					if (sim->isInBroadPhase())
						changedAABBMgrActorHandles.growAndSet(sim->getElementID());
				}*/
			}
		}

		const PxU32 maxBodiesPerTask = 256;

		Cm::FlushPool& flushPool = mLLContext->getTaskPool();

		{
			PX_PROFILE_ZONE("AfterIntegration::dispatchTasks", getContextId());
			for (PxU32 a = 0; a < mProjectedBodies.size(); a += maxBodiesPerTask)
			{
				UpdatProjectedPoseTask* task =
					PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdatProjectedPoseTask)), UpdatProjectedPoseTask)(getContextId(), &mProjectedBodies[a], PxMin(maxBodiesPerTask, mProjectedBodies.size() - a));
				task->setContinuation(continuation);
				task->removeReference();
			}
		}

		{
			Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
			PX_PROFILE_ZONE("AfterIntegration::growAndSet", getContextId());
			for (PxU32 a = 0; a < mProjectedBodies.size(); ++a)
			{
				if (!mProjectedBodies[a]->isFrozen())
				{
					Sc::ElementSim* current = mProjectedBodies[a]->getElements_();
					while(current)
					{
						Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
						if(sim->isInBroadPhase())
							changedAABBMgrActorHandles.growAndSet(sim->getElementID());
						current = current->mNextInActor;
					}
				}
			}
		}
		{
			PX_PROFILE_ZONE("AfterIntegration::managerAndDynamic", getContextId());
			const PxU32 unrollSize = 256;
			for (PxU32 a = 0; a < mProjectedBodies.size(); a += unrollSize)
			{
				PxsRigidBody* tempBodies[unrollSize];
				PxU32 nodeIds[unrollSize];
				const PxU32 nbToProcess = PxMin(unrollSize, mProjectedBodies.size() - a);
				for (PxU32 i = 0; i < nbToProcess; ++i)
				{
					tempBodies[i] = &mProjectedBodies[a + i]->getLowLevelBody();
					nodeIds[i] = mProjectedBodies[a + i]->getNodeIndex().index();
				}
				//KS - it seems that grabbing the CUDA context/releasing it is expensive so we should minimize how
				//frequently we do that. Batch processing like this helps
				mSimulationController->addDynamics(tempBodies, nodeIds, nbToProcess);
			}
		}

		updateKinematicCached(continuation);

		mLLContext->getLock().unlock();
	}

	//KS - TODO - parallelize this bit!!!!!
	if(mArticulations.size())
	{
		Cm::FlushPool& flushPool = mLLContext->getTaskPool();
		ArticulationCore* const* articList = mArticulations.getEntries();

		const PxU32 nbArticulationsPerTask = 8;
		const PxU32 articCount = mArticulations.size();
		for (PxU32 i = 0; i < articCount; i += nbArticulationsPerTask)
		{
			UpdateArticulationTask* task =
				PX_PLACEMENT_NEW(flushPool.allocate(sizeof(UpdateArticulationTask)), UpdateArticulationTask)(getContextId(), articList + i, PxMin(nbArticulationsPerTask, PxU32(articCount - i)), mDt);
			task->setContinuation(continuation);
			task->removeReference();
		}

		mLLContext->getLock().lock();
		Cm::BitMapPinned& changedAABBMgrActorHandles = mAABBManager->getChangedAABBMgActorHandleMap();
		
		Sc::BodySim* ccdBodySims[DY_ARTICULATION_MAX_SIZE];

		for(PxU32 i=0;i<mArticulations.size();i++)
		{
			Sc::ArticulationSim* articSim = articList[i]->getSim();
			
			//KS - check links for CCD flags and add to mCcdBodies list if required....
			PxU32 nbIdx = articSim->getCCDLinks(ccdBodySims);

			for (PxU32 a = 0; a < nbIdx; ++a)
			{
				mCcdBodies.pushBack(ccdBodySims[a]);
			}

			articSim->markShapesUpdated(&changedAABBMgrActorHandles);
		}
		mLLContext->getLock().unlock();
	}

	PX_PROFILE_STOP_CROSSTHREAD("Basic.dynamics", getContextId());

	checkForceThresholdContactEvents(0); 		
}

void Sc::Scene::checkForceThresholdContactEvents(const PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sim.checkForceThresholdContactEvents", getContextId());

	// Note: For contact notifications it is important that force threshold checks are done after new/lost touches have been processed
	//       because pairs might get added to the list processed below

	// Bodies that passed contact force threshold

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	ThresholdStream& thresholdStream = mDynamicsContext->getForceChangedThresholdStream();

	const PxU32 nbThresholdElements = thresholdStream.size();

	for (PxU32 i = 0; i< nbThresholdElements; ++i)
	{
		ThresholdStreamElement& elem = thresholdStream[i];
		ShapeInteraction* si = elem.shapeInteraction;

		//If there is a shapeInteraction and the shapeInteraction points to a contactManager (i.e. the CM was not destroyed in parallel with the solver)
		if (si != NULL)
		{
			PxU32 pairFlags = si->getPairFlags();
			if (pairFlags & ShapeInteraction::CONTACT_FORCE_THRESHOLD_PAIRS)
			{
				si->swapAndClearForceThresholdExceeded();

				if (elem.accumulatedForce > elem.threshold * mDt)
				{
					si->raiseFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_NOW);

					PX_ASSERT(si->hasTouch());

					//If the accumulatedForce is large than the threshold in the current frame and the accumulatedForce is less than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_FOUND
					if ((!si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE)) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND, 0, false, ccdPass, false, outputs);
					}
					else if (si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_PERSISTS, 0, false, ccdPass, false, outputs);
					}
				}
				else
				{
					//If the accumulatedForce is less than the threshold in the current frame and the accumulatedForce is large than the threshold in the previous frame, 
					//and the user request notify for found event, we will raise eNOTIFY_THRESHOLD_FORCE_LOST
					if (si->readFlag(ShapeInteraction::FORCE_THRESHOLD_EXCEEDED_BEFORE) && (pairFlags & PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST))
					{
						si->processUserNotification(PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST, 0, false, ccdPass, false, outputs);
					}
				}
			}
		}
	}
}

void Sc::Scene::endStep()
{
	mTimeStamp++;
//  INVALID_SLEEP_COUNTER is 0xffffffff. Therefore the last bit is masked. Look at Body::isForcedToSleep() for example.
//	if(timeStamp==PX_INVALID_U32)	timeStamp = 0;	// Reserve INVALID_ID for something else
	mTimeStamp &= 0x7fffffff;

	mReportShapePairTimeStamp++;  // to make sure that deleted shapes/actors after fetchResults() create new report pairs
}

void Sc::Scene::resizeReleasedBodyIDMaps(PxU32 maxActors, PxU32 numActors)
{ 
	mLostTouchPairsDeletedBodyIDs.resize(maxActors);
	mRigidIDTracker->resizeDeletedIDMap(maxActors,numActors); 
	mShapeIDTracker->resizeDeletedIDMap(maxActors,numActors);
}

/**
Render objects before simulation starts
*/
void Sc::Scene::visualizeStartStep()
{
	PX_PROFILE_ZONE("Sim.visualizeStartStep", getContextId());

#if PX_ENABLE_DEBUG_VISUALIZATION
	if(!getVisualizationScale())
	{
		// make sure visualization inside simulate was skipped
		PX_ASSERT(getRenderBuffer().empty()); 
		return; // early out if visualization scale is 0
	}

	Cm::RenderOutput out(getRenderBuffer());

	if(getVisualizationParameter(PxVisualizationParameter::eCOLLISION_COMPOUNDS))
		mAABBManager->visualize(out);

	// Visualize joints
	Sc::ConstraintCore*const * constraints = mConstraints.getEntries();
	for(PxU32 i=0, size = mConstraints.size();i<size; i++)
		constraints[i]->getSim()->visualize(getRenderBuffer());

	PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

	mNPhaseCore->visualize(out, outputs);
#endif
}

/**
Render objects after simulation finished. Use this for data that is only available after simulation.
*/
void Sc::Scene::visualizeEndStep()
{
	PX_PROFILE_ZONE("Sim.visualizeEndStep", getContextId());

#if PX_ENABLE_DEBUG_VISUALIZATION
	if(!getVisualizationScale())
	{
		// make sure any visualization before was skipped
		PX_ASSERT(getRenderBuffer().empty()); 
		return; // early out if visualization scale is 0
	}

	Cm::RenderOutput out(getRenderBuffer());
#endif
}

void Sc::Scene::collectPostSolverVelocitiesBeforeCCD()
{
	if (mContactReportsNeedPostSolverVelocity)
	{
		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if (i < (nbActorPairs - 1))
				Ps::prefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];

			ContactStreamManager& cs = aPair->getContactStreamManager();

			PxU32 streamManagerFlag = cs.getFlags();
			if(streamManagerFlag & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;

			PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
			
			if(i + 1 < nbActorPairs)
				Ps::prefetch(&(actorPairs[i+1]->getContactStreamManager()));

			if (!cs.extraDataSize)
				continue;
			else if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
				cs.setContactReportPostSolverVelocity(stream, aPair->getActorA(), aPair->getActorB());
		}
	}
}

void Sc::Scene::finalizeContactStreamAndCreateHeader(PxContactPairHeader& header, const ActorPairReport& aPair, ContactStreamManager& cs, PxU32 removedShapeTestMask)
{
	PxU8* stream = mNPhaseCore->getContactReportPairData(cs.bufferIndex);
	PxU32 streamManagerFlag = cs.getFlags();
	ContactShapePair* contactPairs = cs.getShapePairs(stream);
	const PxU16 nbShapePairs = cs.currentPairCount;
	PX_ASSERT(nbShapePairs > 0);

	if (streamManagerFlag & removedShapeTestMask)
	{
		// At least one shape of this actor pair has been deleted. Need to traverse the contact buffer,
		// find the pairs which contain deleted shapes and set the flags accordingly.

		ContactStreamManager::convertDeletedShapesInContactStream(contactPairs, nbShapePairs, getShapeIDTracker());
	}
	PX_ASSERT(contactPairs);

	ObjectIDTracker& RigidIDTracker = getRigidIDTracker();
	header.actors[0] = static_cast<PxRigidActor*>(aPair.getPxActorA());
	header.actors[1] = static_cast<PxRigidActor*>(aPair.getPxActorB());
	PxU16 headerFlags = 0;
	if (RigidIDTracker.isDeletedID(aPair.getActorAID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_0;
	if (RigidIDTracker.isDeletedID(aPair.getActorBID()))
		headerFlags |= PxContactPairHeaderFlag::eREMOVED_ACTOR_1;
	header.flags = PxContactPairHeaderFlags(headerFlags);
	header.pairs = reinterpret_cast<PxContactPair*>(contactPairs);
	header.nbPairs = nbShapePairs;

	PxU16 extraDataSize = cs.extraDataSize;
	if (!extraDataSize)
		header.extraDataStream = NULL;
	else
	{
		PX_ASSERT(extraDataSize >= sizeof(ContactStreamHeader));
		extraDataSize -= sizeof(ContactStreamHeader);
		header.extraDataStream = stream + sizeof(ContactStreamHeader);

		if (streamManagerFlag & ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY)
		{
			PX_ASSERT(!(headerFlags & Ps::to16(PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | PxContactPairHeaderFlag::eREMOVED_ACTOR_1)));
			cs.setContactReportPostSolverVelocity(stream, aPair.getActorA(), aPair.getActorB());
		}
	}
	header.extraDataStreamSize = extraDataSize;
}

const Ps::Array<PxContactPairHeader>& Sc::Scene::getQueuedContactPairHeaders()
{
	// if buffered shape removals occured, then the criteria for testing the contact stream for events with removed shape pointers needs to be more strict.
	PX_ASSERT(mRemovedShapeCountAtSimStart <= mShapeIDTracker->getDeletedIDCount());
	bool reducedTestForRemovedShapes = mRemovedShapeCountAtSimStart == mShapeIDTracker->getDeletedIDCount();
	const PxU32 removedShapeTestMask = PxU32((reducedTestForRemovedShapes ? ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES : (ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES | ContactStreamManagerFlag::eHAS_PAIRS_THAT_LOST_TOUCH)));

	ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
	PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
	mQueuedContactPairHeaders.reserve(nbActorPairs);
	mQueuedContactPairHeaders.clear();

	for (PxU32 i = 0; i < nbActorPairs; i++)
	{
		if (i < (nbActorPairs - 1))
			Ps::prefetchLine(actorPairs[i + 1]);

		ActorPairReport* aPair = actorPairs[i];
		ContactStreamManager& cs = aPair->getContactStreamManager();
		if (cs.getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
			continue;

		if (i + 1 < nbActorPairs)
			Ps::prefetch(&(actorPairs[i + 1]->getContactStreamManager()));

		PxContactPairHeader &pairHeader = mQueuedContactPairHeaders.insert();
		finalizeContactStreamAndCreateHeader(pairHeader, *aPair, cs, removedShapeTestMask);

		cs.maxPairCount = cs.currentPairCount;
		cs.setMaxExtraDataSize(cs.extraDataSize);
	}

	return mQueuedContactPairHeaders;
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void Sc::Scene::fireQueuedContactCallbacks(bool asPartOfFlush)
{
	if(mSimulationEventCallback)
	{
		// if buffered shape removals occured, then the criteria for testing the contact stream for events with removed shape pointers needs to be more strict.
		PX_ASSERT(asPartOfFlush || (mRemovedShapeCountAtSimStart <= mShapeIDTracker->getDeletedIDCount()));
		bool reducedTestForRemovedShapes = asPartOfFlush || (mRemovedShapeCountAtSimStart == mShapeIDTracker->getDeletedIDCount());
		const PxU32 removedShapeTestMask = PxU32(reducedTestForRemovedShapes ? ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES : (ContactStreamManagerFlag::eTEST_FOR_REMOVED_SHAPES | ContactStreamManagerFlag::eHAS_PAIRS_THAT_LOST_TOUCH));

		ActorPairReport*const* actorPairs = mNPhaseCore->getContactReportActorPairs();
		PxU32 nbActorPairs = mNPhaseCore->getNbContactReportActorPairs();
		for(PxU32 i=0; i < nbActorPairs; i++)
		{
			if (i < (nbActorPairs - 1))
				Ps::prefetchLine(actorPairs[i+1]);

			ActorPairReport* aPair = actorPairs[i];
			ContactStreamManager* cs = &aPair->getContactStreamManager();
			if (cs == NULL || cs->getFlags() & ContactStreamManagerFlag::eINVALID_STREAM)
				continue;
			
			if (i + 1 < nbActorPairs)
				Ps::prefetch(&(actorPairs[i+1]->getContactStreamManager()));

			PxContactPairHeader pairHeader;
			finalizeContactStreamAndCreateHeader(pairHeader, *aPair, *cs, removedShapeTestMask);

			mSimulationEventCallback->onContact(pairHeader, pairHeader.pairs, pairHeader.nbPairs);

			// estimates for next frame
			cs->maxPairCount = cs->currentPairCount;
			cs->setMaxExtraDataSize(cs->extraDataSize);
		}
	}
}

PX_FORCE_INLINE void markDeletedShapes(Sc::ObjectIDTracker& idTracker, Sc::TriggerPairExtraData& tped, PxTriggerPair& pair)
{
	PxTriggerPairFlags::InternalType flags = 0;
	if (idTracker.isDeletedID(tped.shape0ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER;
	if (idTracker.isDeletedID(tped.shape1ID))
		flags |= PxTriggerPairFlag::eREMOVED_SHAPE_OTHER;

	pair.flags = PxTriggerPairFlags(flags);
}

void Sc::Scene::fireTriggerCallbacks()
{
	// triggers
	const PxU32 nbTriggerPairs = mTriggerBufferAPI.size();
	PX_ASSERT(nbTriggerPairs == mTriggerBufferExtraData->size());
	if(nbTriggerPairs) 
	{
		// cases to take into account:
		// - no simulation/trigger shape has been removed -> no need to test shape references for removed shapes
		// - simulation/trigger shapes have been removed but only while the simulation was not running -> only test the events that have 
		//   a marker for removed shapes set
		// - simulation/trigger shapes have been removed while the simulation was running -> need to test all events (see explanation
		//   below)
		//
		// If buffered shape removals occured, then all trigger events need to be tested for removed shape pointers.
		// An optimization like in the contact report case is not applicable here because trigger interactions do not
		// have a reference to their reported events. It can happen that a trigger overlap found event is created but
		// a shape of that pair gets removed while the simulation is running. When processing the lost touch from the 
		// shape removal, no link to the overlap found event is possible and thus it can not be marked as dirty.
		const bool hasRemovedShapes = mShapeIDTracker->getDeletedIDCount() > 0;
		const bool forceTestsForRemovedShapes = (mRemovedShapeCountAtSimStart < mShapeIDTracker->getDeletedIDCount());

		if(mSimulationEventCallback)
		{
			if (!hasRemovedShapes)
				mSimulationEventCallback->onTrigger(mTriggerBufferAPI.begin(), nbTriggerPairs);
			else
			{
				for(PxU32 i = 0; i < nbTriggerPairs; i++)
				{
					PxTriggerPair& triggerPair = mTriggerBufferAPI[i];

					if (forceTestsForRemovedShapes || (PxTriggerPairFlags::InternalType(triggerPair.flags) & TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES))
						markDeletedShapes(*mShapeIDTracker, (*mTriggerBufferExtraData)[i], triggerPair);
				}

				mSimulationEventCallback->onTrigger(mTriggerBufferAPI.begin(), nbTriggerPairs);
			}
		}
	}

	// PT: clear the buffer **even when there's no simulationEventCallback**.
	mTriggerBufferAPI.clear();
	mTriggerBufferExtraData->clear();
}

void Sc::Scene::fireBrokenConstraintCallbacks()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 count = mBrokenConstraints.size();
	for(PxU32 i=0;i<count;i++)
	{
		Sc::ConstraintCore* c = mBrokenConstraints[i];
		if(c->getSim())  // the constraint might have been removed while the simulation was running
		{
			PxU32 typeID = 0xffffffff;
			void* externalRef = c->getPxConnector()->getExternalReference(typeID);
			PX_CHECK_MSG(typeID != 0xffffffff, "onConstraintBreak: Invalid constraint type ID.");

			PxConstraintInfo constraintInfo(c->getPxConstraint(), externalRef, typeID);
			mSimulationEventCallback->onConstraintBreak(&constraintInfo, 1);
		}
	}
}

/*
Threading: called in the context of the user thread, but only after the physics thread has finished its run
*/
void Sc::Scene::fireCallbacksPostSync()
{
	//
	// Fire sleep & woken callbacks
	//

	// A body should be either in the sleep or the woken list. If it is in both, remove it from the list it was
	// least recently added to.

	if(!mSleepBodyListValid)
		cleanUpSleepBodies();

	if(!mWokeBodyListValid)
		cleanUpWokenBodies();

	if(mSimulationEventCallback)
	{
		// allocate temporary data
		const PxU32 nbSleep = mSleepBodies.size();
		const PxU32 nbWoken = mWokeBodies.size();
		const PxU32 arrSize = PxMax(nbSleep, nbWoken);
		PxActor** actors = arrSize ? reinterpret_cast<PxActor**>(PX_ALLOC_TEMP(arrSize*sizeof(PxActor*), "PxActor*")) : NULL;
		if(actors)
		{
			if(nbSleep)
			{
				PxU32 destSlot = 0;
				BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
				for(PxU32 i=0; i<nbSleep; i++)
				{
					BodyCore* body = sleepingBodies[i];
					if(body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
				}

				if(destSlot)
					mSimulationEventCallback->onSleep(actors, destSlot);

				//if (PX_DBG_IS_CONNECTED())
				//{
				//	for (PxU32 i = 0; i < nbSleep; ++i)
				//	{
				//		BodyCore* body = mSleepBodies[i];
				//		PX_ASSERT(body->getActorType() == PxActorType::eRIGID_DYNAMIC);
				//	}
				//}
			}

			// do the same thing for bodies that have just woken up

			if(nbWoken)
			{
				PxU32 destSlot = 0;
				BodyCore* const* wokenBodies = mWokeBodies.getEntries();
				for(PxU32 i=0; i<nbWoken; i++)
				{
					BodyCore* body = wokenBodies[i];
					if(body->getActorFlags() & PxActorFlag::eSEND_SLEEP_NOTIFIES)
						actors[destSlot++] = body->getPxActor();
				}

				if(destSlot)
					mSimulationEventCallback->onWake(actors, destSlot);

				//if (PX_DBG_IS_CONNECTED())
				//{
				//	for (PxU32 i = 0; i < nbWoken; ++i)
				//	{
				//		BodyCore* body = mWokeBodies[i];
				//		PX_ASSERT(actors[i]->getType() == PxActorType::eRIGID_DYNAMIC);
				//	}
				//}
			}

			PX_FREE_AND_RESET(actors);
		}
	}

	clearSleepWakeBodies();
}

void Sc::Scene::prepareOutOfBoundsCallbacks()
{
	PxU32 nbOut0;
	void** outObjects = mAABBManager->getOutOfBoundsObjects(nbOut0);

	mOutOfBoundsIDs.clear();
	for(PxU32 i=0;i<nbOut0;i++)
	{
		ElementSim* volume = reinterpret_cast<ElementSim*>(outObjects[i]);

		Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(volume);
		PxU32 id = sim->getID();
		mOutOfBoundsIDs.pushBack(id);
	}
}

bool Sc::Scene::fireOutOfBoundsCallbacks()
{
	bool outputWarning = false;

	// Actors
	{
		PxU32 nbOut0;
		void** outObjects = mAABBManager->getOutOfBoundsObjects(nbOut0);

		const ObjectIDTracker& tracker = getShapeIDTracker();

		PxBroadPhaseCallback* cb = mBroadPhaseCallback;
		for(PxU32 i=0;i<nbOut0;i++)
		{
			ElementSim* volume = reinterpret_cast<ElementSim*>(outObjects[i]);

			Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(volume);
			if(tracker.isDeletedID(mOutOfBoundsIDs[i]))
				continue;

			if(cb)
			{
				ActorSim& actor = volume->getActor();
				RigidSim& rigidSim = static_cast<RigidSim&>(actor);
				PxActor* pxActor = rigidSim.getPxActor();
				PxShape* px = sim->getPxShape();
				cb->onObjectOutOfBounds(*px, *pxActor);
			}
			else
			{
				outputWarning = true;
			}
		}
		mAABBManager->clearOutOfBoundsObjects();
	}

	return outputWarning;
}

void Sc::Scene::fireOnAdvanceCallback()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 nbPosePreviews = mPosePreviewBodies.size();
	if(!nbPosePreviews)
		return;

	mClientPosePreviewBodies.clear();
	mClientPosePreviewBodies.reserve(nbPosePreviews);

	mClientPosePreviewBuffer.clear();
	mClientPosePreviewBuffer.reserve(nbPosePreviews);
		
	const BodySim*const* PX_RESTRICT posePreviewBodies = mPosePreviewBodies.getEntries();
	for(PxU32 i=0; i<nbPosePreviews; i++)
	{
		const BodySim& b = *posePreviewBodies[i];
		if(!b.isFrozen())
		{
			PxsBodyCore& c = b.getBodyCore().getCore();
			mClientPosePreviewBodies.pushBack(static_cast<const PxRigidBody*>(b.getPxActor()));
			mClientPosePreviewBuffer.pushBack(c.body2World * c.getBody2Actor().getInverse());
		}
	}

	const PxU32 bodyCount = mClientPosePreviewBodies.size();
	if(bodyCount)
		mSimulationEventCallback->onAdvance(mClientPosePreviewBodies.begin(), mClientPosePreviewBuffer.begin(), bodyCount);
}

void Sc::Scene::postCallbacksPreSync()
{
	PX_PROFILE_ZONE("Sim.postCallbackPreSync", mContextId);
	// clear contact stream data
	mNPhaseCore->clearContactReportStream();
	mNPhaseCore->clearContactReportActorPairs(false);

	// Put/prepare kinematics to/for sleep and invalidate target pose
	// note: this needs to get done after the contact callbacks because
	//       the target might get read there.
	//
	PxU32 nbKinematics = getActiveKinematicBodiesCount();
	BodyCore*const* kinematics = getActiveKinematicBodies();

	//KS - this method must run over the kinematic actors in reverse.
	while(nbKinematics--)
	{
		if(nbKinematics > 16)
		{
			Ps::prefetchLine((kinematics[nbKinematics-16]));
		}
		if (nbKinematics > 4)
		{
			Ps::prefetchLine(((kinematics[nbKinematics - 4]))->getSimStateData_Unchecked());
		}

		BodyCore* b = kinematics[nbKinematics];
		//kinematics++;
		PX_ASSERT(b->getSim()->isKinematic());
		PX_ASSERT(b->getSim()->isActive());

		b->invalidateKinematicTarget();
		b->getSim()->deactivateKinematic();
	}

	releaseConstraints(true); //release constraint blocks at the end of the frame, so user can retrieve the blocks
}

void Sc::Scene::setNbContactDataBlocks(PxU32 numBlocks)
{
	mLLContext->getNpMemBlockPool().setBlockCount(numBlocks);
}

PxU32 Sc::Scene::getNbContactDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getUsedBlockCount();
}

PxU32 Sc::Scene::getMaxNbContactDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getMaxUsedBlockCount();
}

PxU32 Sc::Scene::getMaxNbConstraintDataBlocksUsed() const
{
	return mLLContext->getNpMemBlockPool().getPeakConstraintBlockCount();
}

void Sc::Scene::setScratchBlock(void* addr, PxU32 size)
{
	return mLLContext->setScratchBlock(addr, size);
}

void Sc::Scene::checkConstraintBreakage()
{
	PX_PROFILE_ZONE("Sim.checkConstraintBreakage", getContextId());

	PxU32 count = mActiveBreakableConstraints.size();
	ConstraintSim* const* constraints = mActiveBreakableConstraints.getEntries(); 
	while(count)
	{
		count--;
		constraints[count]->checkMaxForceExceeded();  // start from the back because broken constraints get removed from the list
	}
}

void Sc::Scene::getStats(PxSimulationStatistics& s) const
{
	mStats->readOut(s, mLLContext->getSimStats());
	s.nbStaticBodies = mNbRigidStatics;
	s.nbDynamicBodies = mNbRigidDynamics;
	s.nbArticulations = mArticulations.size(); 

	s.nbAggregates = mAABBManager->getNbActiveAggregates();
	for(PxU32 i=0; i<PxGeometryType::eGEOMETRY_COUNT; i++)
		s.nbShapes[i] = mNbGeometries[i];
}

void Sc::Scene::addShapes(void *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& bodySim, PxBounds3* outBounds)
{
	for(PxU32 i=0;i<nbShapes;i++)
	{
		ShapeCore& sc = *reinterpret_cast<ShapeCore*>(reinterpret_cast<size_t>(shapes[i])+ptrOffset);
	
		//PxBounds3* target = uninflatedBounds ? uninflatedBounds + i : uninflatedBounds;
		//mShapeSimPool->construct(sim, sc, llBody, target);

		ShapeSim* shapeSim = mShapeSimPool->construct(bodySim, sc);
		mNbGeometries[sc.getGeometryType()]++;

		mSimulationController->addShape(&shapeSim->getLLShapeSim(), shapeSim->getID());

		if (outBounds)
			outBounds[i] = mBoundsArray->getBounds(shapeSim->getElementID());

		mLLContext->getNphaseImplementationContext()->registerShape(sc.getCore());
	}
}

void Sc::Scene::removeShapes(Sc::RigidSim& sim, Ps::InlineArray<Sc::ShapeSim*, 64>& shapesBuffer , Ps::InlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	Sc::ElementSim* current = sim.getElements_();
	while(current)
	{
		Sc::ShapeSim* s = static_cast<Sc::ShapeSim*>(current);
		// can do two 2x the allocs in the worst case, but actors with >64 shapes are not common
		shapesBuffer.pushBack(s);
		removedShapes.pushBack(&s->getCore());
		current = current->mNextInActor;
	}

	for(PxU32 i=0;i<shapesBuffer.size();i++)
		removeShape(*shapesBuffer[i], wakeOnLostTouch);
}

void Sc::Scene::addStatic(StaticCore& ro, void*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* uninflatedBounds)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	StaticSim* sim = mStaticSimPool->construct(*this, ro);
	
	mNbRigidStatics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, uninflatedBounds);
}

void Sc::Scene::prefetchForRemove(const StaticCore& core) const
{
	StaticSim* sim = core.getSim();
	if(sim)
	{
		Ps::prefetch(sim,sizeof(Sc::StaticSim));
		Ps::prefetch(sim->getElements_(),sizeof(Sc::ElementSim));
	}
}

void Sc::Scene::prefetchForRemove(const BodyCore& core) const
{
	BodySim *sim = core.getSim();	
	if(sim)
	{
		Ps::prefetch(sim,sizeof(Sc::BodySim));
		Ps::prefetch(sim->getElements_(),sizeof(Sc::ElementSim));
	}
}

void Sc::Scene::removeStatic(StaticCore& ro, Ps::InlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	PX_ASSERT(ro.getActorCoreType() == PxActorType::eRIGID_STATIC);

	StaticSim* sim = ro.getSim();
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes ,removedShapes, wakeOnLostTouch);
		}
		else
		{
			Ps::InlineArray<Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim, shapesBuffer ,removedShapes, wakeOnLostTouch);
		}		
		mStaticSimPool->destroy(static_cast<Sc::StaticSim*>(ro.getSim()));
		mNbRigidStatics--;
	}
}

void Sc::Scene::addBody(BodyCore& body, void*const *shapes, PxU32 nbShapes, size_t shapePtrOffset, PxBounds3* outBounds, bool compound)
{
	// sim objects do all the necessary work of adding themselves to broad phase,
	// activation, registering with the interaction system, etc

	BodySim* sim = mBodySimPool->construct(*this, body, compound);

	const bool isArticulationLink = sim->isArticulationLink();

	if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		if (isArticulationLink)
		{
			if (sim->getNodeIndex().isValid())
				mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
		}
		else
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}
	//if rigid body is articulation link, the node index will be invalid. We should add the link to the scene after we add the
	//articulation for gpu
	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());
	mNbRigidDynamics++;
	addShapes(shapes, nbShapes, shapePtrOffset, *sim, outBounds);
}

void Sc::Scene::removeBody(BodyCore& body, Ps::InlineArray<const Sc::ShapeCore*,64>& removedShapes, bool wakeOnLostTouch)
{
	BodySim *sim = body.getSim();	
	if(sim)
	{
		if(mBatchRemoveState)
		{
			removeShapes(*sim, mBatchRemoveState->bufferedShapes ,removedShapes, wakeOnLostTouch);
		}
		else
		{
			Ps::InlineArray<Sc::ShapeSim*, 64>  shapesBuffer;
			removeShapes(*sim,shapesBuffer, removedShapes, wakeOnLostTouch);
		}

		if (!sim->isArticulationLink())
		{
			//clear bit map
			if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
				mSpeculativeCCDRigidBodyBitMap.reset(sim->getNodeIndex().index());
		}
		mBodySimPool->destroy(sim);
		mNbRigidDynamics--;
	}
}

void Sc::Scene::addShape(RigidSim& owner, ShapeCore& shapeCore, PxBounds3* uninflatedBounds)
{
	ShapeSim* sim = mShapeSimPool->construct(owner, shapeCore);
	mNbGeometries[shapeCore.getGeometryType()]++;

	//register shape
	mSimulationController->addShape(&sim->getLLShapeSim(), sim->getID());
	if (uninflatedBounds)
		*uninflatedBounds = mBoundsArray->getBounds(sim->getElementID());
	registerShapeInNphase(shapeCore);
}

void Sc::Scene::removeShape(ShapeSim& shape, bool wakeOnLostTouch)
{
	//BodySim* body = shape.getBodySim();
	//if(body)
	//	body->postShapeDetach();
	
	unregisterShapeFromNphase(shape.getCore());

	mSimulationController->removeShape(shape.getID());

	mNbGeometries[shape.getCore().getGeometryType()]--;
	shape.removeFromBroadPhase(wakeOnLostTouch);
	mShapeSimPool->destroy(&shape);
}

void Sc::Scene::registerShapeInNphase(const ShapeCore& shape)
{
	mLLContext->getNphaseImplementationContext()->registerShape(shape.getCore());
}

void Sc::Scene::unregisterShapeFromNphase(const ShapeCore& shape)
{
	mLLContext->getNphaseImplementationContext()->unregisterShape(shape.getCore());
}

void Sc::Scene::notifyNphaseOnUpdateShapeMaterial(const ShapeCore& shapeCore)
{
	mLLContext->getNphaseImplementationContext()->updateShapeMaterial(shapeCore.getCore());
}

void Sc::Scene::startBatchInsertion(BatchInsertionState&state)
{
	state.shapeSim = mShapeSimPool->allocateAndPrefetch();
	state.staticSim = mStaticSimPool->allocateAndPrefetch();
	state.bodySim = mBodySimPool->allocateAndPrefetch();														   
}

void Sc::Scene::addShapes(void *const* shapes, PxU32 nbShapes, size_t ptrOffset, RigidSim& rigidSim, ShapeSim*& prefetchedShapeSim, PxBounds3* outBounds)
{
	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(i+1<nbShapes)
			Ps::prefetch(shapes[i+1], PxU32(ptrOffset+sizeof(Sc::ShapeCore)));
		ShapeSim* nextShapeSim = mShapeSimPool->allocateAndPrefetch();
		const ShapeCore& sc = *Ps::pointerOffset<const ShapeCore*>(shapes[i], ptrdiff_t(ptrOffset));
		new(prefetchedShapeSim) ShapeSim(rigidSim, sc);
		outBounds[i] = mBoundsArray->getBounds(prefetchedShapeSim->getElementID());

		mSimulationController->addShape(&prefetchedShapeSim->getLLShapeSim(), prefetchedShapeSim->getID());
		prefetchedShapeSim = nextShapeSim;
		mNbGeometries[sc.getGeometryType()]++;

		
		mLLContext->getNphaseImplementationContext()->registerShape(sc.getCore());
	}	
}

void Sc::Scene::addStatic(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds)
{
	// static core has been prefetched by caller
	Sc::StaticSim* sim = s.staticSim;		// static core has been prefetched by the caller

	const Cm::PtrTable* shapeTable = Ps::pointerOffset<const Cm::PtrTable*>(actor, s.staticShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();
	if(shapeTable->getCount())
		Ps::prefetch(shapes[0],PxU32(s.shapeOffset+sizeof(Sc::ShapeCore)));

	mStaticSimPool->construct(sim, *this, *Ps::pointerOffset<Sc::StaticCore*>(actor, s.staticActorOffset));
	s.staticSim = mStaticSimPool->allocateAndPrefetch();

	addShapes(shapes, shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);
	mNbRigidStatics++;
}

void Sc::Scene::addBody(PxActor* actor, BatchInsertionState& s, PxBounds3* outBounds, bool compound)
{
	Sc::BodySim* sim = s.bodySim;		// body core has been prefetched by the caller

	const Cm::PtrTable* shapeTable = Ps::pointerOffset<const Cm::PtrTable*>(actor, s.dynamicShapeTableOffset);
	void*const* shapes = shapeTable->getPtrs();
	if(shapeTable->getCount())
		Ps::prefetch(shapes[0], PxU32(s.shapeOffset+sizeof(Sc::ShapeCore)));

	mBodySimPool->construct(sim, *this, *Ps::pointerOffset<Sc::BodyCore*>(actor, s.dynamicActorOffset), compound);
	s.bodySim = mBodySimPool->allocateAndPrefetch();

	const bool isArticulationLink = sim->isArticulationLink();

	if (isArticulationLink)
	{
		if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
			mSpeculativeCDDArticulationBitMap.growAndSet(sim->getNodeIndex().index());
	}
	else
	{
		if (sim->getLowLevelBody().mCore->mFlags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
			mSpeculativeCCDRigidBodyBitMap.growAndSet(sim->getNodeIndex().index());
	}

	if(sim->getNodeIndex().isValid())
		mSimulationController->addDynamic(&sim->getLowLevelBody(), sim->getNodeIndex());

	addShapes(shapes, shapeTable->getCount(), size_t(s.shapeOffset), *sim, s.shapeSim, outBounds);
	mNbRigidDynamics++;
}

void Sc::Scene::finishBatchInsertion(BatchInsertionState& state)
{
	// a little bit lazy - we could deal with the last one in the batch specially to avoid overallocating by one.
	
	mStaticSimPool->releasePreallocated(static_cast<Sc::StaticSim*>(state.staticSim));	
	mBodySimPool->releasePreallocated(static_cast<Sc::BodySim*>(state.bodySim));
	mShapeSimPool->releasePreallocated(state.shapeSim);
}

void Sc::Scene::initContactsIterator(ContactIterator& contactIterator, PxsContactManagerOutputIterator& outputs)
{
	outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();
	Interaction** first = mInteractions[Sc::InteractionType::eOVERLAP].begin();
	contactIterator = ContactIterator(first, first + mActiveInteractionCount[Sc::InteractionType::eOVERLAP], outputs);
}

void Sc::Scene::setDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance)
{
	struct {
		void operator()(PxU32& bits, PxDominanceGroup shift, PxReal weight)
		{
			if(weight != PxReal(0))
				bits |=  (PxU32(1) << shift);
			else 
				bits &= ~(PxU32(1) << shift);
		}
	} bitsetter;

	bitsetter(mDominanceBitMatrix[group1], group2, dominance.dominance0);
	bitsetter(mDominanceBitMatrix[group2], group1, dominance.dominance1);

	mInternalFlags |= SceneInternalFlag::eSCENE_SIP_STATES_DIRTY_DOMINANCE;		//force an update on all interactions on matrix change -- very expensive but we have no choice!!
}

PxDominanceGroupPair Sc::Scene::getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const
{
	PxU8 dom0 = PxU8((mDominanceBitMatrix[group1]>>group2) & 0x1 ? 1u : 0u);
	PxU8 dom1 = PxU8((mDominanceBitMatrix[group2]>>group1) & 0x1 ? 1u : 0u);
	return PxDominanceGroupPair(dom0, dom1);
}

void Sc::Scene::setCreateContactReports(bool s)		
{ 
	if(mLLContext)
		mLLContext->setCreateContactStream(s); 
}

void Sc::Scene::setSolverBatchSize(PxU32 solverBatchSize)
{
	mDynamicsContext->setSolverBatchSize(solverBatchSize);
}

PxU32 Sc::Scene::getSolverBatchSize() const
{
	return mDynamicsContext->getSolverBatchSize();
}

void Sc::Scene::setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value)
{
	mVisualizationParameterChanged = true;

	PX_ASSERT(mLLContext->getVisualizationParameter(PxVisualizationParameter::eSCALE) == mVisualizationScale); // Safety check because the scale is duplicated for performance reasons

	mLLContext->setVisualizationParameter(param, value);

	if (param == PxVisualizationParameter::eSCALE)
		mVisualizationScale = value;
}

PxReal Sc::Scene::getVisualizationParameter(PxVisualizationParameter::Enum param) const
{
	PX_ASSERT(mLLContext->getVisualizationParameter(PxVisualizationParameter::eSCALE) == mVisualizationScale); // Safety check because the scale is duplicated for performance reasons

	return mLLContext->getVisualizationParameter(param);
}

void Sc::Scene::setVisualizationCullingBox(const PxBounds3& box)
{
	mLLContext->setVisualizationCullingBox(box);
}

const PxBounds3& Sc::Scene::getVisualizationCullingBox() const
{
	return mLLContext->getVisualizationCullingBox();
}

PxReal Sc::Scene::getFrictionOffsetThreshold() const
{
	return mDynamicsContext->getFrictionOffsetThreshold();
}

PxU32 Sc::Scene::getDefaultContactReportStreamBufferSize() const
{
	return mNPhaseCore->getDefaultContactReportStreamBufferSize();
}

void Sc::Scene::buildActiveActors()
{
	PxU32 numActiveBodies = 0;
	BodyCore*const* PX_RESTRICT activeBodies;
	if(!(getPublicFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
	{
		numActiveBodies = getNumActiveBodies();
		activeBodies = getActiveBodiesArray();
	}
	else
	{
		numActiveBodies = getActiveDynamicBodiesCount();
		activeBodies = getActiveDynamicBodies();
	}

	mActiveActors.clear();

	for(PxU32 i=0; i<numActiveBodies; i++)
	{
		if(!activeBodies[i]->isFrozen())
		{
			PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
			PX_ASSERT(ra);
			mActiveActors.pushBack(ra);
		}
	}
}

// PT: TODO: unify buildActiveActors & buildActiveAndFrozenActors
void Sc::Scene::buildActiveAndFrozenActors()
{
	PxU32 numActiveBodies = 0;
	BodyCore*const* PX_RESTRICT activeBodies;
	if(!(getPublicFlags() & PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS))
	{
		numActiveBodies = getNumActiveBodies();
		activeBodies = getActiveBodiesArray();
	}
	else
	{
		numActiveBodies = getActiveDynamicBodiesCount();
		activeBodies = getActiveDynamicBodies();
	}

	mActiveActors.clear();
	mFrozenActors.clear();

	for(PxU32 i=0; i<numActiveBodies; i++)
	{
		PxRigidActor* ra = static_cast<PxRigidActor*>(activeBodies[i]->getPxActor());
		PX_ASSERT(ra);

		if(!activeBodies[i]->isFrozen())
			mActiveActors.pushBack(ra);
		else
			mFrozenActors.pushBack(ra);
	}
}

PxActor** Sc::Scene::getActiveActors(PxU32& nbActorsOut)
{
	nbActorsOut = mActiveActors.size();
	
	if(!nbActorsOut)
		return NULL;

	return mActiveActors.begin();
}

void Sc::Scene::setActiveActors(PxActor** actors, PxU32 nbActors)
{
	mActiveActors.forceSize_Unsafe(0);
	mActiveActors.resize(nbActors);
	PxMemCopy(mActiveActors.begin(), actors, sizeof(PxActor*) * nbActors);
}

PxActor** Sc::Scene::getFrozenActors(PxU32& nbActorsOut)
{
	nbActorsOut = mFrozenActors.size();

	if(!nbActorsOut)
		return NULL;

	return mFrozenActors.begin();
}

void Sc::Scene::reserveTriggerReportBufferSpace(const PxU32 pairCount, PxTriggerPair*& triggerPairBuffer, TriggerPairExtraData*& triggerPairExtraBuffer)
{
	const PxU32 oldSize = mTriggerBufferAPI.size();
	const PxU32 newSize = oldSize + pairCount;
	const PxU32 newCapacity = PxU32(newSize * 1.5f);
	mTriggerBufferAPI.reserve(newCapacity);
	mTriggerBufferAPI.forceSize_Unsafe(newSize);
	triggerPairBuffer = mTriggerBufferAPI.begin() + oldSize;

	PX_ASSERT(oldSize == mTriggerBufferExtraData->size());
	mTriggerBufferExtraData->reserve(newCapacity);
	mTriggerBufferExtraData->forceSize_Unsafe(newSize);
	triggerPairExtraBuffer = mTriggerBufferExtraData->begin() + oldSize;
}

PxClientID Sc::Scene::createClient()
{
	mClients.pushBack(PX_NEW(Client)());
	return PxClientID(mClients.size()-1);
}

void Sc::Scene::clearSleepWakeBodies(void)
{
	// Clear sleep/woken marker flags
	BodyCore* const* sleepingBodies = mSleepBodies.getEntries();
	for(PxU32 i=0; i < mSleepBodies.size(); i++)
	{
		BodySim* body = sleepingBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(BodySim::BF_WAKEUP_NOTIFY));
		body->clearInternalFlag(BodySim::BF_SLEEP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
        body->clearInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}

	BodyCore* const* wokenBodies = mWokeBodies.getEntries();
	for(PxU32 i=0; i < mWokeBodies.size(); i++)
	{
		BodySim* body = wokenBodies[i]->getSim();

		PX_ASSERT(!body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY));
		body->clearInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

		// A body can be in both lists depending on the sequence of events
		body->clearInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
        body->clearInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}

	mSleepBodies.clear();
	mWokeBodies.clear();
	mWokeBodyListValid = true;
	mSleepBodyListValid = true;
}

void Sc::Scene::onBodySleep(BodySim* body)
{
	if(mSimulationEventCallback)
	{
		if (body->readInternalFlag(BodySim::BF_WAKEUP_NOTIFY))
		{
			PX_ASSERT(!body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY));

			// Body is in the list of woken bodies, hence, mark this list as dirty such that it gets cleaned up before
			// being sent to the user
			body->clearInternalFlag(BodySim::BF_WAKEUP_NOTIFY);
			mWokeBodyListValid = false;
		}

		body->raiseInternalFlag(BodySim::BF_SLEEP_NOTIFY);
	}

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	//
	// even if no sleep events are requested, we still need to track the objects which were put to sleep because
	// for those we need to sync the buffered state (strictly speaking this only applies to sleep changes that are 
	// triggered by the simulation and not the user but we do not distinguish here)
	if (!body->readInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST))
	{
		if(mSimulationEventCallback)
		{
			PX_ASSERT(!mSleepBodies.contains(&body->getBodyCore()));
		}
		mSleepBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(BodySim::BF_IS_IN_SLEEP_LIST);
	}
}

void Sc::Scene::onBodyWakeUp(BodySim* body)
{
	if(!mSimulationEventCallback)
		return;

	if (body->readInternalFlag(BodySim::BF_SLEEP_NOTIFY))
	{
		PX_ASSERT(!body->readInternalFlag(BodySim::BF_WAKEUP_NOTIFY));

		// Body is in the list of sleeping bodies, hence, mark this list as dirty such it gets cleaned up before
		// being sent to the user
		body->clearInternalFlag(BodySim::BF_SLEEP_NOTIFY);
		mSleepBodyListValid = false;
	}

	body->raiseInternalFlag(BodySim::BF_WAKEUP_NOTIFY);

	// Avoid multiple insertion (the user can do multiple transitions between asleep and awake)
	if (!body->readInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST))
	{
		PX_ASSERT(!mWokeBodies.contains(&body->getBodyCore()));
		mWokeBodies.insert(&body->getBodyCore());
		body->raiseInternalFlag(BodySim::BF_IS_IN_WAKEUP_LIST);
	}
}

PX_INLINE void Sc::Scene::cleanUpSleepBodies()
{
	BodyCore* const* bodyArray = mSleepBodies.getEntries();
	PxU32 bodyCount = mSleepBodies.size();

	IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	while (bodyCount--)
	{
		BodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(BodySim::BF_WAKEUP_NOTIFY)))
		{
			body->clearInternalFlag(static_cast<BodySim::InternalFlags>(BodySim::BF_IS_IN_WAKEUP_LIST));
			mSleepBodies.erase(bodyArray[bodyCount]);
		}
		else if (islandSim.getNode(body->getNodeIndex()).isActive())
		{
			//This body is still active in the island simulation, so the request to deactivate the actor by the application must have failed. Recover by undoing this
			mSleepBodies.erase(bodyArray[bodyCount]);
			body->internalWakeUp();
		}
	}

	mSleepBodyListValid = true;
}

PX_INLINE void Sc::Scene::cleanUpWokenBodies()
{
	cleanUpSleepOrWokenBodies(mWokeBodies, BodySim::BF_SLEEP_NOTIFY, mWokeBodyListValid);
}

PX_INLINE void Sc::Scene::cleanUpSleepOrWokenBodies(Ps::CoalescedHashSet<BodyCore*>& bodyList, PxU32 removeFlag, bool& validMarker)
{
	// With our current logic it can happen that a body is added to the sleep as well as the woken body list in the
	// same frame.
	//
	// Examples:
	// - Kinematic is created (added to woken list) but has not target (-> deactivation -> added to sleep list)
	// - Dynamic is created (added to woken list) but is forced to sleep by user (-> deactivation -> added to sleep list)
	//
	// This code traverses the sleep/woken body list and removes bodies which have been initially added to the given
	// list but do not belong to it anymore.

	BodyCore* const* bodyArray = bodyList.getEntries();
	PxU32 bodyCount = bodyList.size();
	while (bodyCount--)
	{
		BodySim* body = bodyArray[bodyCount]->getSim();

		if (body->readInternalFlag(static_cast<BodySim::InternalFlags>(removeFlag)))
			bodyList.erase(bodyArray[bodyCount]);
	}

	validMarker = true;
}

void Sc::Scene::releaseConstraints(bool endOfScene)
{
	PX_ASSERT(mLLContext);

	if(getStabilizationEnabled())
	{
		//If stabilization is enabled, we're caching contacts for next frame
		if(!endOfScene)
		{
			//So we only clear memory (flip buffers) when not at the end-of-scene.
			//This means we clear after narrow phase completed so we can 
			//release the previous frame's contact buffers before we enter the solve phase.
			mLLContext->getNpMemBlockPool().releaseContacts();
		}
	}
	else if(endOfScene)
	{
		//We now have a double-buffered pool of mem blocks so we must
		//release both pools (which actually triggers the memory used this 
		//frame to be released 
		mLLContext->getNpMemBlockPool().releaseContacts();
		mLLContext->getNpMemBlockPool().releaseContacts();
	}
}

PX_INLINE void Sc::Scene::clearBrokenConstraintBuffer()
{
	mBrokenConstraints.clear();
}

void Sc::Scene::addToLostTouchList(BodySim* body1, BodySim* body2)
{
	PX_ASSERT(body1 != 0);
	PX_ASSERT(body2 != 0);
	SimpleBodyPair p = { body1, body2, body1->getRigidID(), body2->getRigidID() };
	mLostTouchPairs.pushBack(p);
}

void Sc::Scene::initDominanceMatrix()
{
	//init all dominance pairs such that:
	//if g1 == g2, then (1.0f, 1.0f) is returned
	//if g1 <  g2, then (0.0f, 1.0f) is returned
	//if g1 >  g2, then (1.0f, 0.0f) is returned

	PxU32 mask = ~PxU32(1);
	for (unsigned i = 0; i < PX_MAX_DOMINANCE_GROUP; ++i, mask <<= 1)
		mDominanceBitMatrix[i] = ~mask;
}

ArticulationV* Sc::Scene::createLLArticulation(Sc::ArticulationSim* sim)
{
	ArticulationV* articulation = NULL;
	
	if (sim->getCore().getArticulationType() == PxArticulationBase::eMaximumCoordinate)
	{
		articulation =  mLLArticulationPool->construct(sim);
	}
	else 
	{
		PX_ASSERT(sim->getCore().getArticulationType() == PxArticulationBase::eReducedCoordinate);
		articulation = mLLArticulationRCPool->construct(sim);
	}

	return articulation;
}

void Sc::Scene::destroyLLArticulation(ArticulationV& articulation)
{
	if (articulation.getType() == PxArticulationBase::eMaximumCoordinate)
		mLLArticulationPool->destroy(static_cast<Dy::Articulation*>(&articulation));
	else
	{
		PX_ASSERT(articulation.getType() == PxArticulationBase::eReducedCoordinate);
		mLLArticulationRCPool->destroy(static_cast<Dy::FeatherstoneArticulation*>(&articulation));
	}
}

PxU32 Sc::Scene::getNbArticulations() const
{
	return mArticulations.size();
}

Sc::ArticulationCore* const* Sc::Scene::getArticulations() 
{
	return mArticulations.getEntries();
}

PxU32 Sc::Scene::getNbConstraints() const
{
	return mConstraints.size();
}

Sc::ConstraintCore*const * Sc::Scene::getConstraints() 
{
	return mConstraints.getEntries();
}

PxU32 Sc::Scene::createAggregate(void* userData, bool selfCollisions)
{
	const physx::Bp::BoundsIndex index = getElementIDPool().createID();
	mBoundsArray->initEntry(index);
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	return mAABBManager->createAggregate(index, Bp::FilterGroup::eINVALID, userData, selfCollisions);
#else
	// PT: TODO: ideally a static compound would have a static group
	const PxU32 rigidId	= getRigidIDTracker().createID();
	const Bp::FilterGroup::Enum bpGroup = Bp::FilterGroup::Enum(rigidId + Bp::FilterGroup::eDYNAMICS_BASE);
	return mAABBManager->createAggregate(index, bpGroup, userData, selfCollisions);
#endif
}

void Sc::Scene::deleteAggregate(PxU32 id)
{
	Bp::BoundsIndex index;
	Bp::FilterGroup::Enum bpGroup;
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);
	}
#else
	if(mAABBManager->destroyAggregate(index, bpGroup, id))
	{
		getElementIDPool().releaseID(index);

		// PT: this is clumsy....
		const PxU32 rigidId	= PxU32(bpGroup) - Bp::FilterGroup::eDYNAMICS_BASE;
		getRigidIDTracker().releaseID(rigidId);
	}
#endif
}

void Sc::Scene::shiftOrigin(const PxVec3& shift)
{
	//
	// adjust low level context
	//
	mLLContext->shiftOrigin(shift);

	// adjust bounds array
	//
	mBoundsArray->shiftOrigin(shift);

	//
	// adjust broadphase
	//
	mAABBManager->shiftOrigin(shift);

	//
	// adjust constraints
	//
	ConstraintCore*const * constraints = mConstraints.getEntries();
	for(PxU32 i=0, size = mConstraints.size(); i < size; i++)
	{
		constraints[i]->getPxConnector()->onOriginShift(shift);
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sc::Scene::islandInsertion(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.islandInsertion", getContextId());

		const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
		for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
		{
			size_t address = reinterpret_cast<size_t>(mPreallocatedShapeInteractions[a]);
			if (address & 1)
			{
				ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));

				PxsContactManager* contactManager = const_cast<PxsContactManager*>(interaction->getContactManager());

				Sc::BodySim* bs0 = interaction->getShape0().getBodySim();
				Sc::BodySim* bs1 = interaction->getShape1().getBodySim();

				IG::NodeIndex nodeIndexB;
				if (bs1)
					nodeIndexB = bs1->getNodeIndex();

				IG::EdgeIndex edgeIdx = mSimpleIslandManager->addContactManager(contactManager, bs0->getNodeIndex(), nodeIndexB, interaction);
				interaction->mEdgeIndex = edgeIdx;

				if (contactManager)
					contactManager->getWorkUnit().mEdgeIndex = edgeIdx;
			}
		}

		// - Wakes actors that lost touch if appropriate
		//processLostTouchPairs();

		if(mCCDPass == 0)
		{
			mSimpleIslandManager->firstPassIslandGen();
		}
	}
}

void Sc::Scene::registerContactManagers(PxBaseTask* /*continuation*/)
{
	{
		PxvNphaseImplementationContext* nphaseContext = mLLContext->getNphaseImplementationContext();
		PX_PROFILE_ZONE("Sim.processNewOverlaps.registerCms", getContextId());
		//nphaseContext->registerContactManagers(mPreallocatedContactManagers.begin(), mPreallocatedContactManagers.size(), mLLContext->getContactManagerPool().getMaxUsedIndex());
		const PxU32 nbCmsCreated = mPreallocatedContactManagers.size();
		for (PxU32 a = 0; a < nbCmsCreated; ++a)
		{
			size_t address = reinterpret_cast<size_t>(mPreallocatedContactManagers[a]);
			if (address & 1)
			{
				PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(address&size_t(~1));
				nphaseContext->registerContactManager(cm, 0, 0);
			}
		}
	}
}

void Sc::Scene::registerInteractions(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractions", getContextId());
		const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
		for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
		{
			size_t address = reinterpret_cast<size_t>(mPreallocatedShapeInteractions[a]);
			if (address & 1)
			{
				ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));

				ActorSim& actorSim0 = interaction->getActorSim0();
				ActorSim& actorSim1 = interaction->getActorSim1();
				actorSim0.registerInteractionInActor(interaction);
				actorSim1.registerInteractionInActor(interaction);

				Sc::BodySim* bs0 = actorSim0.isDynamicRigid() ? static_cast<BodySim*>(&actorSim0) : NULL;
				Sc::BodySim* bs1 = actorSim1.isDynamicRigid() ? static_cast<BodySim*>(&actorSim1) : NULL;

				bs0->registerCountedInteraction();
				if(bs1)
					bs1->registerCountedInteraction();
			}
		}

		const PxU32 nbMarkersCreated = mPreallocatedInteractionMarkers.size();
		for (PxU32 a = 0; a < nbMarkersCreated; ++a)
		{
			size_t address = reinterpret_cast<size_t>(mPreallocatedInteractionMarkers[a]);
			if (address & 1)
			{
				ElementInteractionMarker* interaction = reinterpret_cast<ElementInteractionMarker*>(address&size_t(~1));
				interaction->registerInActors(NULL);
			}
		}
	}
}

void Sc::Scene::registerSceneInteractions(PxBaseTask* /*continuation*/)
{
	PX_PROFILE_ZONE("Sim.processNewOverlaps.registerInteractionsScene", getContextId());
	const PxU32 nbShapeIdxCreated = mPreallocatedShapeInteractions.size();
	for (PxU32 a = 0; a < nbShapeIdxCreated; ++a)
	{
		size_t address = reinterpret_cast<size_t>(mPreallocatedShapeInteractions[a]);
		if (address & 1)
		{
			ShapeInteraction* interaction = reinterpret_cast<ShapeInteraction*>(address&size_t(~1));
			registerInteraction(interaction, interaction->getContactManager() != NULL);
			mNPhaseCore->registerInteraction(interaction);

			const PxsContactManager* cm = interaction->getContactManager();
			if (cm != NULL)
			{
				mLLContext->setActiveContactManager(cm);
			}
		}
	}

	const PxU32 nbInteractionMarkers = mPreallocatedInteractionMarkers.size();
	for (PxU32 a = 0; a < nbInteractionMarkers; ++a)
	{
		size_t address = reinterpret_cast<size_t>(mPreallocatedInteractionMarkers[a]);
		if (address & 1)
		{
			ElementInteractionMarker* interaction = reinterpret_cast<ElementInteractionMarker*>(address&size_t(~1));
			registerInteraction(interaction, false);
			mNPhaseCore->registerInteraction(interaction);
		}
	}
}

class OverlapFilterTask : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 512;
	Sc::NPhaseCore*			mNPhaseCore;
	const Bp::AABBOverlap*	mPairs;

	PxU32					mNbToProcess;

	PxU32					mKeepMap[MaxPairs/32];
	PxU32					mCallbackMap[MaxPairs/32];

	PxFilterInfo*			mFinfo;

	PxU32					mNbToKeep;
	PxU32					mNbToSuppress;
	PxU32					mNbToCallback;

	OverlapFilterTask*		mNext;

	OverlapFilterTask(PxU64 contextID, Sc::NPhaseCore* nPhaseCore, PxFilterInfo* fInfo, const Bp::AABBOverlap* pairs, const PxU32 nbToProcess) :
		Cm::Task		(contextID),
		mNPhaseCore		(nPhaseCore),
		mPairs			(pairs),
		mNbToProcess	(nbToProcess),
		mFinfo			(fInfo),
		mNbToKeep		(0),
		mNbToSuppress	(0),
		mNbToCallback	(0),
		mNext			(NULL)
	{
		PxMemZero(mKeepMap, sizeof(mKeepMap));
		PxMemZero(mCallbackMap, sizeof(mCallbackMap));
	}

	virtual void runInternal()
	{
		mNPhaseCore->runOverlapFilters(	mNbToProcess, mPairs, mFinfo, mNbToKeep, mNbToSuppress, mNbToCallback, mKeepMap, mCallbackMap);
	}

	virtual const char* getName() const { return "OverlapFilterTask"; }
};

class OnOverlapCreatedTask : public Cm::Task
{
public:
	Sc::NPhaseCore*					mNPhaseCore;
	const Bp::AABBOverlap*			mPairs;
	const PxFilterInfo*				mFinfo;
	PxsContactManager**				mContactManagers;
	Sc::ShapeInteraction**			mShapeInteractions;
	Sc::ElementInteractionMarker**	mInteractionMarkers;
	PxU32							mNbToProcess;

	OnOverlapCreatedTask(PxU64 contextID, Sc::NPhaseCore* nPhaseCore, const Bp::AABBOverlap* pairs, const PxFilterInfo* fInfo, PxsContactManager** contactManagers, Sc::ShapeInteraction** shapeInteractions, Sc::ElementInteractionMarker** interactionMarkers,
		PxU32 nbToProcess) :
		Cm::Task			(contextID),
		mNPhaseCore			(nPhaseCore),
		mPairs				(pairs),
		mFinfo				(fInfo),
		mContactManagers	(contactManagers),
		mShapeInteractions	(shapeInteractions),
		mInteractionMarkers	(interactionMarkers),
		mNbToProcess		(nbToProcess)
	{
	}

	virtual void runInternal()
	{
		PxsContactManager** currentCm = mContactManagers;
		Sc::ShapeInteraction** currentSI = mShapeInteractions;
		Sc::ElementInteractionMarker** currentEI = mInteractionMarkers;

		for(PxU32 i=0; i<mNbToProcess; i++)
		{
			const Bp::AABBOverlap& pair = mPairs[i];
			Sc::ShapeSim* s0 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData1);
			Sc::ShapeSim* s1 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData0);

			Sc::ElementSimInteraction* interaction = mNPhaseCore->createRbElementInteraction(mFinfo[i], *s0, *s1, *currentCm, *currentSI, *currentEI, 0);
			if(interaction)
			{
				if(interaction->getType() == Sc::InteractionType::eOVERLAP)
				{
					*currentSI = reinterpret_cast<Sc::ShapeInteraction*>(reinterpret_cast<size_t>(*currentSI) | 1);
					currentSI++;

					if(static_cast<Sc::ShapeInteraction*>(interaction)->getContactManager())
					{
						*currentCm = reinterpret_cast<PxsContactManager*>(reinterpret_cast<size_t>(*currentCm) | 1);
						currentCm++;
					}
				}
				else if(interaction->getType() == Sc::InteractionType::eMARKER)
				{
					*currentEI = reinterpret_cast<Sc::ElementInteractionMarker*>(reinterpret_cast<size_t>(*currentEI) | 1);
					currentEI++;
				}
			}
		}
	}

	virtual const char* getName() const { return "OnOverlapCreatedTask"; }
};

void Sc::Scene::finishBroadPhase(PxBaseTask* continuation)
{
	PX_UNUSED(continuation);
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase", getContextId());

	Bp::AABBManager* aabbMgr = mAABBManager;

	{
		PX_PROFILE_ZONE("Sim.processNewOverlaps", getContextId());

		{
			//KS - these functions call "registerInActors", while OverlapFilterTask reads the list of interactions
			//in an actor. This could lead to a race condition and a crash if they occur at the same time, so we 
			//serialize these operations
			PX_PROFILE_ZONE("Sim.processNewOverlaps.createOverlapsNoShapeInteractions", getContextId());
			{
				PxU32 createdOverlapCount;
				const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getCreatedOverlaps(Bp::ElementType::eTRIGGER, createdOverlapCount);

				mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;
				mNPhaseCore->onOverlapCreated(p, createdOverlapCount);
			}
		}

		{
			PxU32 createdOverlapCount;
			const Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getCreatedOverlaps(Bp::ElementType::eSHAPE, createdOverlapCount);
			{
				//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
				mPreallocatedContactManagers.reserve(1);
				mPreallocatedShapeInteractions.reserve(1);
				mPreallocatedInteractionMarkers.reserve(1);

				mPreallocatedContactManagers.forceSize_Unsafe(1);
				mPreallocatedShapeInteractions.forceSize_Unsafe(1);
				mPreallocatedInteractionMarkers.forceSize_Unsafe(1);
			}

			mLLContext->getSimStats().mNbNewPairs += createdOverlapCount;

			mPreallocateContactManagers.setContinuation(continuation);
			Cm::FlushPool& flushPool = mLLContext->getTaskPool();

			mFilterInfo.forceSize_Unsafe(0);
			mFilterInfo.reserve(createdOverlapCount);
			mFilterInfo.forceSize_Unsafe(createdOverlapCount);

			const PxU32 nbPairsPerTask = OverlapFilterTask::MaxPairs;
			mOverlapFilterTaskHead = NULL;
			OverlapFilterTask* previousTask = NULL;
			for(PxU32 a=0; a<createdOverlapCount; a+=nbPairsPerTask)
			{
				PxU32 nbToProcess = PxMin(createdOverlapCount - a, nbPairsPerTask);
				OverlapFilterTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OverlapFilterTask)), OverlapFilterTask)(getContextId(), mNPhaseCore, mFilterInfo.begin() + a, p + a, nbToProcess);

				task->setContinuation(&mPreallocateContactManagers);
				task->removeReference();

				if(previousTask)
					previousTask->mNext = task;
				else
					mOverlapFilterTaskHead = task;

				previousTask = task;
			}
		}

		mPreallocateContactManagers.removeReference();
	}	
}

void Sc::Scene::preallocateContactManagers(PxBaseTask* continuation)
{
	//Iterate over all filter tasks and work out how many pairs we need...

	PxU32 createdOverlapCount = 0;

	PxU32 totalCreatedPairs = 0;
	PxU32 totalSuppressPairs = 0;

	PxU32 overlapCount;
	Bp::AABBOverlap* PX_RESTRICT p = mAABBManager->getCreatedOverlaps(Bp::ElementType::eSHAPE, overlapCount);
	PxFilterInfo* fInfo = mFilterInfo.begin();

	OverlapFilterTask* task = mOverlapFilterTaskHead;
	while(task)
	{
		if(task->mNbToCallback)
		{
			//Iterate and process callbacks. Refilter then increment the results, setting the appropriate settings
			const FilteringContext context(*this, mNPhaseCore->mFilterPairManager);

			for(PxU32 w = 0; w < (OverlapFilterTask::MaxPairs / 32); ++w)
			{
				for(PxU32 b = task->mCallbackMap[w]; b; b &= b - 1)
				{
					const PxU32 index = (w << 5) + Ps::lowestSetBit(b);
					const Bp::AABBOverlap& pair = task->mPairs[index];
					Sc::ShapeSim* s0 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData0);
					Sc::ShapeSim* s1 = reinterpret_cast<Sc::ShapeSim*>(pair.mUserData1);

					const PxFilterInfo finfo = filterRbCollisionPairSecondStage(context, *s0, *s1, s0->getBodySim(), s1->getBodySim(), INVALID_FILTER_PAIR_INDEX, true);

					task->mFinfo[index] = finfo;

					if(!(finfo.filterFlags & PxFilterFlag::eKILL))
					{
						if((finfo.filterFlags & PxFilterFlag::eSUPPRESS) == false)
							task->mNbToKeep++;
						else
							task->mNbToSuppress++;
						task->mKeepMap[index / 32] |= (1 << (index & 31));
					}
				}
			}
		}

		totalCreatedPairs += task->mNbToKeep;
		totalSuppressPairs += task->mNbToSuppress;
		task = task->mNext;
	}

	{
		//We allocate at least 1 element in this array to ensure that the onOverlapCreated functions don't go bang!
		mPreallocatedContactManagers.reserve(totalCreatedPairs+1);
		mPreallocatedShapeInteractions.reserve(totalCreatedPairs+1);
		mPreallocatedInteractionMarkers.reserve(totalSuppressPairs+1);

		mPreallocatedContactManagers.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedShapeInteractions.forceSize_Unsafe(totalCreatedPairs);
		mPreallocatedInteractionMarkers.forceSize_Unsafe(totalSuppressPairs);
	}

	const PxU32 nbPairsPerTask = 256;
	PxsContactManager** cms = mPreallocatedContactManagers.begin();
	Sc::ShapeInteraction** shapeInter = mPreallocatedShapeInteractions.begin();
	Sc::ElementInteractionMarker** markerIter = mPreallocatedInteractionMarkers.begin();

	Cm::FlushPool& flushPool = mLLContext->getTaskPool();

	struct Local
	{
		static void processBatch(const PxU32 createdCurrIdx, PxU32& createdStartIdx, const PxU32 suppressedCurrIdx, PxU32& suppressedStartIdx, const PxU32 batchSize,
			PxsContext* const context, NPhaseCore* const core, OnOverlapCreatedTask* const createTask, PxBaseTask* const continuation_,
			PxsContactManager** const cms_, Sc::ShapeInteraction** const shapeInter_, Sc::ElementInteractionMarker** const markerIter_)
		{
			const PxU32 nbToCreate = createdCurrIdx - createdStartIdx;
			const PxU32 nbToSuppress = suppressedCurrIdx - suppressedStartIdx;

			context->getContactManagerPool().preallocate(nbToCreate, cms_ + createdStartIdx);
			for(PxU32 i=0; i<nbToCreate; ++i)
				shapeInter_[createdStartIdx + i] = core->mShapeInteractionPool.allocate();
			for(PxU32 i=0; i<nbToSuppress; ++i)
				markerIter_[suppressedStartIdx + i] = core->mInteractionMarkerPool.allocate();
				
			createdStartIdx = createdCurrIdx;
			suppressedStartIdx = suppressedCurrIdx;

			createTask->mNbToProcess = batchSize;
			createTask->setContinuation(continuation_);
			createTask->removeReference();
		}
	};

	// PT: TODO: why do we create the task immediately? Why not create it only when a batch is full?
	OnOverlapCreatedTask* createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(getContextId(), mNPhaseCore, p, fInfo, cms, shapeInter, markerIter, 0);

	PxU32 batchSize = 0;
	PxU32 suppressedStartIdx = 0;
	PxU32 createdStartIdx = 0;
	PxU32 suppressedCurrIdx = 0;
	PxU32 createdCurrIdx = 0;
	PxU32 currentReadIdx = 0;

	task = mOverlapFilterTaskHead;
	while(task)
	{
		if(task->mNbToKeep || task->mNbToSuppress)
		{
			for(PxU32 w = 0; w < (OverlapFilterTask::MaxPairs/32); ++w)
			{
				for(PxU32 b = task->mKeepMap[w]; b; b &= b-1)
				{
					const PxU32 index = (w<<5) + Ps::lowestSetBit(b);

					if(createdOverlapCount < (index + currentReadIdx))
					{
						p[createdOverlapCount] = task->mPairs[index];
						fInfo[createdOverlapCount] = task->mFinfo[index];
					}
					createdOverlapCount++;
					batchSize++;
				}
			}

			suppressedCurrIdx += task->mNbToSuppress;
			createdCurrIdx += task->mNbToKeep;

			if(batchSize >= nbPairsPerTask)
			{
				Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);

				createTask = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(OnOverlapCreatedTask)), OnOverlapCreatedTask)(getContextId(), mNPhaseCore, p + createdOverlapCount,
					fInfo + createdOverlapCount, cms + createdStartIdx, shapeInter + createdStartIdx, markerIter + suppressedStartIdx, 0);

				batchSize = 0;
			}
		}
		currentReadIdx += OverlapFilterTask::MaxPairs;
		task = task->mNext;
	}

	if(batchSize)
		Local::processBatch(createdCurrIdx, createdStartIdx, suppressedCurrIdx, suppressedStartIdx, batchSize, mLLContext, mNPhaseCore, createTask, continuation, cms, shapeInter, markerIter);
}

void Sc::Scene::finishBroadPhaseStage2(const PxU32 ccdPass)
{
	PX_PROFILE_ZONE("Sc::Scene::finishBroadPhase2", getContextId());

	Bp::AABBManager* aabbMgr = mAABBManager;

	for(PxU32 i=0; i<Bp::ElementType::eCOUNT; i++)
	{
		PxU32 destroyedOverlapCount;
		aabbMgr->getDestroyedOverlaps(Bp::ElementType::Enum(i), destroyedOverlapCount);
		mLLContext->getSimStats().mNbLostPairs += destroyedOverlapCount;
	}

	//KS - we need to defer processing lost overlaps until later!
	if (ccdPass)
	{
		PX_PROFILE_ZONE("Sim.processLostOverlaps", getContextId());
		PxsContactManagerOutputIterator outputs = mLLContext->getNphaseImplementationContext()->getContactManagerOutputs();

		const bool useAdaptiveForce = mPublicFlags & PxSceneFlag::eADAPTIVE_FORCE;

		PxU32 destroyedOverlapCount;

		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eSHAPE, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				//KS - this is a bit ugly. We split the "onOverlapRemoved" for shape interactions to parallelize it and that means
				//that we have to call each of the individual stages of the remove here.

				//First, we have to get the interaction pointer...
				Sc::ElementSimInteraction* interaction = mNPhaseCore->onOverlapRemovedStage1(volume0, volume1);
				p->mPairUserData = interaction;
				if(interaction)
				{
					if(interaction->getType() == Sc::InteractionType::eOVERLAP || interaction->getType() == Sc::InteractionType::eMARKER)
					{
						//If it's a standard "overlap" interaction, we have to send a lost touch report, unregister it, and destroy its manager and island gen data.
						if(interaction->getType() == Sc::InteractionType::eOVERLAP)
						{
							Sc::ShapeInteraction* si = static_cast<Sc::ShapeInteraction*>(interaction);
							mNPhaseCore->lostTouchReports(si, PxU32(PairReleaseFlag::eWAKE_ON_LOST_TOUCH), 0, outputs, useAdaptiveForce);
							si->destroyManager();
							si->clearIslandGenData();
						}

						unregisterInteraction(interaction);
						mNPhaseCore->unregisterInteraction(interaction);
					}

					//Then call "onOverlapRemoved" to actually free the interaction
					mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, interaction, outputs, useAdaptiveForce);
				}
				p++;
			}
		}

		{
			Bp::AABBOverlap* PX_RESTRICT p = aabbMgr->getDestroyedOverlaps(Bp::ElementType::eTRIGGER, destroyedOverlapCount);

			while(destroyedOverlapCount--)
			{
				ElementSim* volume0 = reinterpret_cast<ElementSim*>(p->mUserData0);
				ElementSim* volume1 = reinterpret_cast<ElementSim*>(p->mUserData1);

				p->mPairUserData = NULL;

				//KS - this is a bit ugly. 
				mNPhaseCore->onOverlapRemoved(volume0, volume1, ccdPass, NULL, outputs, useAdaptiveForce);
				p++;
			}
		}
	}

	// - Wakes actors that lost touch if appropriate
	processLostTouchPairs();

	if (ccdPass)
	{
		aabbMgr->getBroadPhase()->deletePairs();

		aabbMgr->freeBuffers();
	}
}

void Sc::Scene::secondPassNarrowPhase(PxBaseTask* /*continuation*/)
{
	{
		PX_PROFILE_ZONE("Sim.postIslandGen", getContextId());
		mSimpleIslandManager->additionalSpeculativeActivation();
		// wake interactions
		{
			PX_PROFILE_ZONE("ScScene.wakeInteractions", getContextId());
			const IG::IslandSim& speculativeSim = mSimpleIslandManager->getSpeculativeIslandSim();

			//KS - only wake contact managers based on speculative state to trigger contact gen. Waking actors based on accurate state
			//should activate and joints.
			{
				PxU32 nbActivatingEdges = speculativeSim.getNbActivatedEdges(IG::Edge::eCONTACT_MANAGER);
				const IG::EdgeIndex* activatingEdges = speculativeSim.getActivatedEdges(IG::Edge::eCONTACT_MANAGER);

				for(PxU32 i = 0; i < nbActivatingEdges; ++i)
				{
					Sc::Interaction* interaction = mSimpleIslandManager->getInteraction(activatingEdges[i]);

					if(interaction && !interaction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
					{
						if(speculativeSim.getEdge(activatingEdges[i]).isActive())
						{
							const bool proceed = activateInteraction(interaction, NULL);

							if(proceed && (interaction->getType() < InteractionType::eTRACKED_IN_SCENE_COUNT))
								notifyInteractionActivated(interaction);
						}
					}
				}
			}
		}
	}
	mLLContext->secondPassUpdateContactManager(mDt, &mPostNarrowPhase); // Starts update of contact managers
}

//~BROADPHASE

void Sc::Scene::registerMaterialInNP(const PxsMaterialCore& materialCore)
{
	mLLContext->getNphaseImplementationContext()->registerMaterial(materialCore);
}

void Sc::Scene::updateMaterialInNP(const PxsMaterialCore& materialCore)
{
	mLLContext->getNphaseImplementationContext()->updateMaterial(materialCore);
}

void Sc::Scene::unregisterMaterialInNP(const PxsMaterialCore& materialCore)
{
	mLLContext->getNphaseImplementationContext()->unregisterMaterial(materialCore);
}

bool Sc::activateInteraction(Sc::Interaction* interaction, void* data)
{
	switch(interaction->getType())
	{
		case InteractionType::eOVERLAP:
			return static_cast<Sc::ShapeInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eTRIGGER:
			return static_cast<Sc::TriggerInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eMARKER:
			return static_cast<Sc::ElementInteractionMarker*>(interaction)->onActivate_(data);

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<Sc::ConstraintInteraction*>(interaction)->onActivate_(data);

		case InteractionType::eARTICULATION:
			return static_cast<Sc::ArticulationJointSim*>(interaction)->onActivate_(data);

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}

bool Sc::deactivateInteraction(Sc::Interaction* interaction)
{
	switch(interaction->getType())
	{
		case InteractionType::eOVERLAP:
			return static_cast<Sc::ShapeInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eTRIGGER:
			return static_cast<Sc::TriggerInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eMARKER:
			return static_cast<Sc::ElementInteractionMarker*>(interaction)->onDeactivate_();

		case InteractionType::eCONSTRAINTSHADER:
			return static_cast<Sc::ConstraintInteraction*>(interaction)->onDeactivate_();

		case InteractionType::eARTICULATION:
			return static_cast<Sc::ArticulationJointSim*>(interaction)->onDeactivate_();

		case InteractionType::eTRACKED_IN_SCENE_COUNT:
		case InteractionType::eINVALID:
		PX_ASSERT(0);
		break;
	}
	return false;
}
