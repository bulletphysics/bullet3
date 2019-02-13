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

       
#include "PxsContext.h"
#include "CmFlushPool.h"
#include "PxsSimpleIslandManager.h"
#include "common/PxProfileZone.h"

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#include "task/PxGpuDispatcher.h"
#endif

#include "PxsContactManagerState.h"

#include "PxsNphaseImplementationContext.h"
#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "PxvGlobals.h"

#include "PxcNpContactPrepShared.h"

using namespace physx;
using namespace physx::shdfnd;


class PxsCMUpdateTask : public Cm::Task
{
public:

	static const PxU32 BATCH_SIZE = 128;

	PxsCMUpdateTask(PxsContext* context, PxReal dt, PxsContactManager** cmArray, PxsContactManagerOutput* cmOutputs, Gu::Cache* caches, PxU32 cmCount, PxContactModifyCallback* callback) :
			Cm::Task	(context->getContextId()),
			mCmArray	(cmArray),
			mCmOutputs	(cmOutputs),
			mCaches		(caches),
			mCmCount	(cmCount),
			mDt			(dt),
			mContext	(context),
			mCallback	(callback)
	{
	}

	virtual void release();

	/*PX_FORCE_INLINE void insert(PxsContactManager* cm)
	{
		PX_ASSERT(mCmCount < BATCH_SIZE);
		mCmArray[mCmCount++]=cm;
	}*/

protected:	
	//PxsContactManager*	mCmArray[BATCH_SIZE];
	PxsContactManager**			mCmArray;
	PxsContactManagerOutput*	mCmOutputs;
	Gu::Cache*					mCaches;
	PxU32						mCmCount;
	PxReal						mDt;		//we could probably retrieve from context to save space?
	PxsContext*					mContext;
	PxContactModifyCallback*	mCallback;
};

void PxsCMUpdateTask::release()
{
	// We used to do Task::release(); here before fixing DE1106 (xbox pure virtual crash)
	// Release in turn causes the dependent tasks to start running
	// The problem was that between the time release was called and by the time we got to the destructor
	// The task chain would get all the way to scene finalization code which would reset the allocation pool
	// And a new task would get allocated at the same address, then we would invoke the destructor on that freshly created task
	// This could potentially cause any number of other problems, it is suprising that it only manifested itself
	// as a pure virtual crash
	PxBaseTask* saveContinuation = mCont;
	this->~PxsCMUpdateTask();
	if (saveContinuation)
		saveContinuation->removeReference();
}

class PxsCMDiscreteUpdateTask : public PxsCMUpdateTask
{
public:
	PxsCMDiscreteUpdateTask(PxsContext* context, PxReal dt, PxsContactManager** cms, PxsContactManagerOutput* cmOutputs, Gu::Cache* caches, PxU32 nbCms,
		PxContactModifyCallback* callback):
	  PxsCMUpdateTask(context, dt, cms, cmOutputs, caches, nbCms, callback) 
	{}

	virtual ~PxsCMDiscreteUpdateTask()
	{}

	void runModifiableContactManagers(PxU32* modifiableIndices, PxU32 nbModifiableManagers, PxcNpThreadContext& threadContext, PxU32& foundPatchCount_, PxU32& lostPatchCount_,
		PxU32& maxPatches_)
	{
		PX_ASSERT(nbModifiableManagers != 0);

		PxU32 foundPatchCount = foundPatchCount_;
		PxU32 lostPatchCount = lostPatchCount_;
		PxU32 maxPatches = maxPatches_;

		Cm::BitMap& localPatchChangedMap = threadContext.getLocalPatchChangeMap();

		class PxcContactSet: public PxContactSet
		{
		public:
			PxcContactSet(PxU32 count, PxModifiableContact *contacts)
			{
				mContacts = contacts;
				mCount = count;
			}
			PxModifiableContact*	getContacts()	{ return mContacts; }
			PxU32					getCount()		{ return mCount; }	

		};



		if(mCallback)
		{
			PX_ALLOCA(mModifiablePairArray, PxContactModifyPair, nbModifiableManagers);


			PxsTransformCache& transformCache = mContext->getTransformCache();
		
			for(PxU32 i = 0; i < nbModifiableManagers; ++i)
			{
				PxU32 index = modifiableIndices[i];
				PxsContactManager& cm = *mCmArray[index];
	
				PxsContactManagerOutput& output = mCmOutputs[index];
	
				PxU32 count = output.nbContacts;
	
				if(count)
				{
					PxContactModifyPair& p = mModifiablePairArray[i];
					PxcNpWorkUnit &unit = cm.getWorkUnit();
	
					p.shape[0] = gPxvOffsetTable.convertPxsShape2Px(unit.shapeCore0);
					p.shape[1] = gPxvOffsetTable.convertPxsShape2Px(unit.shapeCore1);
	
					p.actor[0] = unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY0 ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(unit.rigidCore0)
						: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(unit.rigidCore0);

					p.actor[1] = unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1 ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(unit.rigidCore1)
						: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(unit.rigidCore1);
	
					p.transform[0] = transformCache.getTransformCache(unit.mTransformCache0).transform;
					p.transform[1] = transformCache.getTransformCache(unit.mTransformCache1).transform;
	
					PxModifiableContact* contacts = reinterpret_cast<PxModifiableContact*>(output.contactPoints);
					static_cast<PxcContactSet&>(p.contacts) = PxcContactSet(count, contacts);
	
					PxReal mi0 = unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY0 ? static_cast<const PxsBodyCore*>(unit.rigidCore0)->maxContactImpulse : PX_MAX_F32;
					PxReal mi1 = unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1 ? static_cast<const PxsBodyCore*>(unit.rigidCore1)->maxContactImpulse : PX_MAX_F32;
					PxReal maxImpulse = PxMin(mi0, mi1);
					for (PxU32 j = 0; j < count; j++)
						contacts[j].maxImpulse = maxImpulse;
	
		#if PX_ENABLE_SIM_STATS
					PxU8 gt0 = Ps::to8(unit.geomType0), gt1 = Ps::to8(unit.geomType1);
					threadContext.mModifiedContactPairs[PxMin(gt0, gt1)][PxMax(gt0, gt1)]++;
		#endif
				}
			}
	
			mCallback->onContactModify(mModifiablePairArray, nbModifiableManagers);
		}
	
		for(PxU32 i = 0; i < nbModifiableManagers; ++i)
		{
			PxU32 index = modifiableIndices[i];
			PxsContactManager& cm = *mCmArray[index];
	
			//Loop through the contacts in the contact stream and update contact count!
	
			PxU32 numContacts = 0;
			PxcNpWorkUnit& unit = cm.getWorkUnit();
			PxsContactManagerOutput& output = mCmOutputs[index];
	
			
			PxU32 numPatches = output.nbPatches;
		
			if (output.nbContacts)
			{
				//PxU8* compressedContacts = cm.getWorkUnit().compressedContacts;
				//PxModifyContactHeader* header = reinterpret_cast<PxModifyContactHeader*>(compressedContacts);
				PxContactPatch* patches = reinterpret_cast<PxContactPatch*>(output.contactPatches);
				PxModifiableContact* points = reinterpret_cast<PxModifiableContact*>(output.contactPoints);

				if (patches->internalFlags & PxContactPatch::eREGENERATE_PATCHES)
				{
					//Some data was modified that must trigger patch re-generation...
					for (PxU8 k = 0; k < numPatches; ++k)
					{
						PxU32 startIndex = patches[k].startContactIndex;

						patches[k].normal = points[startIndex].normal;
						patches[k].dynamicFriction = points[startIndex].dynamicFriction;
						patches[k].staticFriction = points[startIndex].staticFriction;
						patches[k].restitution = points[startIndex].restitution;

						for (PxU32 j = 1; j < patches[k].nbContacts; ++j)
						{
							if (points[startIndex].normal.dot(points[j + startIndex].normal) < PXC_SAME_NORMAL
								&& points[startIndex].maxImpulse > 0.f) //TODO - this needs extending for material indices but we don't support modifying those yet
							{
								//The points are now in a separate friction patch...
								for (PxU32 c = numPatches - 1; c > k; --c)
								{
									patches[c + 1] = patches[c];
								}
								numPatches++;
								patches[k + 1].materialFlags = patches[k].materialFlags;
								patches[k + 1].internalFlags = patches[k].internalFlags;
								patches[k + 1].startContactIndex = Ps::to8(j + startIndex);
								patches[k + 1].nbContacts = Ps::to8(patches[k].nbContacts - j);
								//Fill in patch information now that final patches are available
								patches[k].nbContacts = PxU8(j);
								break;
							}
						}
					}
				}

				if (output.prevPatches < numPatches)
				{
					foundPatchCount++;
					localPatchChangedMap.growAndSet(unit.index);
				}

				maxPatches = PxMax(maxPatches, PxU32(numPatches));

				output.nbPatches = PxU8(numPatches);

				for (PxU32 a = 0; a < output.nbContacts; ++a)
				{
					numContacts += points[a].maxImpulse != 0.f;
				}
			}

			if(output.nbPatches < output.prevPatches)
			{
				lostPatchCount++;
				//Trigger a lost patch event...required to let the solver 
				localPatchChangedMap.growAndSet(unit.index);
			}
	
			if(!numContacts)
			{	
				//KS - we still need to retain the patch count from the previous frame to detect found/lost events...
				PxcNpWorkUnitClearFrictionCachedState(unit);
				output.nbPatches = 0;
				output.nbContacts = 0;

				if(output.prevPatches)
				{
					lostPatchCount++;
					//Trigger a lost patch event...required to let the solver 
					localPatchChangedMap.growAndSet(unit.index);
				}

				continue;
			}
	
			if(threadContext.mContactStreamPool)
			{
				//We need to allocate a new structure inside the contact stream pool
	
				PxU32 patchSize = output.nbPatches * sizeof(PxContactPatch);
				PxU32 contactSize = output.nbContacts * sizeof(PxExtendedContact);
	
				/*PxI32 increment = (PxI32)(patchSize + contactSize);
				PxI32 index = Ps::atomicAdd(&mContactStreamPool->mSharedContactIndex, increment) - increment;
				PxU8* address = mContactStreamPool->mContactStream + index;*/
				bool isOverflown = false;
	
				PxI32 contactIncrement = PxI32(contactSize);
				PxI32 contactIndex = Ps::atomicAdd(&threadContext.mContactStreamPool->mSharedDataIndex, contactIncrement);
				
				if (threadContext.mContactStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Contact buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
							
				PxU8* contactAddress = threadContext.mContactStreamPool->mDataStream  + threadContext.mContactStreamPool->mDataStreamSize - contactIndex;
	
				PxI32 patchIncrement = PxI32(patchSize);
				PxI32 patchIndex = Ps::atomicAdd(&threadContext.mPatchStreamPool->mSharedDataIndex, patchIncrement);
				
				if (threadContext.mPatchStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Patch buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
				
				PxU8* patchAddress = threadContext.mPatchStreamPool->mDataStream + threadContext.mPatchStreamPool->mDataStreamSize - patchIndex;
	
	
				PxU32 internalFlags = reinterpret_cast<PxContactPatch*>(output.contactPatches)->internalFlags;
	
				PxI32 increment2 = PxI32(output.nbContacts * sizeof(PxReal));
				PxI32 index2 = Ps::atomicAdd(&threadContext.mForceAndIndiceStreamPool->mSharedDataIndex, increment2);
				
				if (threadContext.mForceAndIndiceStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Force buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
	
				if (isOverflown)
				{
					output.contactPoints = NULL;
					output.contactPatches = NULL;
					output.contactForces = NULL;
				
					output.nbContacts = output.nbPatches = 0;
				}
				else
				{
					output.contactForces = reinterpret_cast<PxReal*>(threadContext.mForceAndIndiceStreamPool->mDataStream + threadContext.mForceAndIndiceStreamPool->mDataStreamSize - index2);
				
					PxMemZero(output.contactForces, sizeof(PxReal) * output.nbContacts);
					
					PxExtendedContact* contacts = reinterpret_cast<PxExtendedContact*>(contactAddress);
					PxMemCopy(patchAddress, output.contactPatches, sizeof(PxContactPatch) * output.nbPatches);
	
					PxContactPatch* newPatches = reinterpret_cast<PxContactPatch*>(patchAddress);
	
					internalFlags |= PxContactPatch::eCOMPRESSED_MODIFIED_CONTACT;
	
					for(PxU32 a = 0; a < output.nbPatches; ++a)
					{
						newPatches[a].internalFlags = PxU8(internalFlags);
					}
	
					//KS - only the first patch will have mass modification properties set. For the GPU solver, this must be propagated to the remaining patches
					for(PxU32 a = 1; a < output.nbPatches; ++a)
					{
						newPatches[a].mMassModification = newPatches->mMassModification;
					}
	
					PxModifiableContact* sourceContacts = reinterpret_cast<PxModifiableContact*>(output.contactPoints);
	
					for(PxU32 a = 0; a < output.nbContacts; ++a)
					{
						PxExtendedContact& contact = contacts[a];
						PxModifiableContact& srcContact = sourceContacts[a];
						contact.contact = srcContact.contact;
						contact.separation = srcContact.separation;
						contact.targetVelocity = srcContact.targetVelocity;
						contact.maxImpulse = srcContact.maxImpulse;
					}
	
					output.contactPatches = patchAddress;
					output.contactPoints = reinterpret_cast<PxU8*>(contacts);
				}
			}
		}

		foundPatchCount_ = foundPatchCount;
		lostPatchCount_ = lostPatchCount;
		maxPatches_ = maxPatches;
	}


	template < void (*NarrowPhase)(PxcNpThreadContext&, const PxcNpWorkUnit&, Gu::Cache&, PxsContactManagerOutput&)>
	void processCms(PxcNpThreadContext* threadContext)
	{
		// PT: use local variables to avoid reading class members N times, if possible
		const PxU32 nb = mCmCount;
		PxsContactManager** PX_RESTRICT cmArray = mCmArray;

		PxU32 lostPatchCount = 0, foundPatchCount = 0;

		PxU32 maxPatches = threadContext->mMaxPatches;

		PxU32 newTouchCMCount = 0, lostTouchCMCount = 0;
		Cm::BitMap& localChangeTouchCM = threadContext->getLocalChangeTouch();
		Cm::BitMap& localPatchChangedMap = threadContext->getLocalPatchChangeMap();

		PX_ALLOCA(modifiableIndices, PxU32, nb);
		PxU32 modifiableCount = 0;

		for(PxU32 i=0;i<nb;i++)
		{
			const PxU32 prefetch1 = PxMin(i + 1, nb - 1);
			const PxU32 prefetch2 = PxMin(i + 2, nb - 1);

			Ps::prefetchLine(cmArray[prefetch2]);
			Ps::prefetchLine(&mCmOutputs[prefetch2]);
			Ps::prefetchLine(cmArray[prefetch1]->getWorkUnit().shapeCore0);
			Ps::prefetchLine(cmArray[prefetch1]->getWorkUnit().shapeCore1);
			Ps::prefetchLine(&threadContext->mTransformCache->getTransformCache(cmArray[prefetch1]->getWorkUnit().mTransformCache0));
			Ps::prefetchLine(&threadContext->mTransformCache->getTransformCache(cmArray[prefetch1]->getWorkUnit().mTransformCache1));

			PxsContactManager* cm = cmArray[i];			

			if(cm)
			{
				PxsContactManagerOutput& output = mCmOutputs[i];
				PxcNpWorkUnit& unit = cm->getWorkUnit();

				output.prevPatches = output.nbPatches;


				PxU8 oldStatusFlag = output.statusFlag;

				PxU8 oldTouch = Ps::to8(oldStatusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH);

				Gu::Cache& cache = mCaches[i];

				NarrowPhase(*threadContext, unit, cache, output);
				
				PxU16 newTouch = Ps::to8(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH);
				

				bool modifiable = output.nbPatches != 0 && unit.flags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT;

				if(modifiable)
				{
					modifiableIndices[modifiableCount++] = i;
				}
				else
				{
					maxPatches = PxMax(maxPatches, Ps::to32(output.nbPatches));

					if(output.prevPatches != output.nbPatches)
					{
						localPatchChangedMap.growAndSet(cmArray[i]->getIndex());	
						if(output.prevPatches < output.nbPatches)
							foundPatchCount++;
						else
							lostPatchCount++;
					}
				}

				if (newTouch ^ oldTouch)
				{
					cm->getWorkUnit().statusFlags = PxU8(output.statusFlag | (unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH));  //KS - todo - remove the need to access the work unit at all!
					localChangeTouchCM.growAndSet(cmArray[i]->getIndex());
					if(newTouch)
						newTouchCMCount++;
					else
						lostTouchCMCount++;
				}
				else if (!(oldStatusFlag&PxsContactManagerStatusFlag::eTOUCH_KNOWN))
				{
					cm->getWorkUnit().statusFlags = PxU8(output.statusFlag | (unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH));  //KS - todo - remove the need to access the work unit at all!
				}
			}
		}

		if(modifiableCount)
		{
			runModifiableContactManagers(modifiableIndices, modifiableCount, *threadContext, foundPatchCount, lostPatchCount, maxPatches);
		}


		threadContext->addLocalNewTouchCount(newTouchCMCount);
		threadContext->addLocalLostTouchCount(lostTouchCMCount);

		threadContext->addLocalFoundPatchCount(foundPatchCount);
		threadContext->addLocalLostPatchCount(lostPatchCount);

		threadContext->mMaxPatches = maxPatches;
	}

	virtual void runInternal()
	{
		PX_PROFILE_ZONE("Sim.narrowPhase", mContext->getContextId());

		PxcNpThreadContext* PX_RESTRICT threadContext = mContext->getNpThreadContext(); 
	
		threadContext->mDt = mDt;

		const bool pcm = mContext->getPCM();
		threadContext->mPCM = pcm;
		threadContext->mCreateAveragePoint = mContext->getCreateAveragePoint();
		threadContext->mContactCache = mContext->getContactCacheFlag();
		threadContext->mTransformCache = &mContext->getTransformCache();
		threadContext->mContactDistance = mContext->getContactDistance();

		if(pcm)
		{
			processCms<PxcDiscreteNarrowPhasePCM>(threadContext);
		}
		else
		{
			processCms<PxcDiscreteNarrowPhase>(threadContext);
		}

		mContext->putNpThreadContext(threadContext);
	}

	virtual const char* getName() const
	{
		return "PxsContext.contactManagerDiscreteUpdate";
	}
};

void PxsNphaseImplementationContext::processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation)
{
		//Iterate all active contact managers
	mContext.mTaskPool.lock();
	const PxU32 nbCmsToProcess = mNarrowPhasePairs.mContactManagerMapping.size();

	for(PxU32 a = 0; a < nbCmsToProcess;)
	{
		void* ptr = mContext.mTaskPool.allocateNotThreadSafe(sizeof(PxsCMDiscreteUpdateTask));
		PxU32 nbToProcess = PxMin(nbCmsToProcess - a, PxsCMUpdateTask::BATCH_SIZE);
		PxsCMDiscreteUpdateTask* task = PX_PLACEMENT_NEW(ptr, PxsCMDiscreteUpdateTask)(&mContext, dt, mNarrowPhasePairs.mContactManagerMapping.begin() + a, 
			cmOutputs + a, mNarrowPhasePairs.mCaches.begin() + a, nbToProcess, mModifyCallback);

		a += nbToProcess;

		task->setContinuation(continuation);
		task->removeReference();
	}
	mContext.mTaskPool.unlock();
}

void PxsNphaseImplementationContext::processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation)
{
		//Iterate all active contact managers
	mContext.mTaskPool.lock();

	const PxU32 nbCmsToProcess = mNewNarrowPhasePairs.mContactManagerMapping.size();

	for(PxU32 a = 0; a < nbCmsToProcess;)
	{
		void* ptr = mContext.mTaskPool.allocateNotThreadSafe(sizeof(PxsCMDiscreteUpdateTask));
		PxU32 nbToProcess = PxMin(nbCmsToProcess - a, PxsCMUpdateTask::BATCH_SIZE);
		PxsCMDiscreteUpdateTask* task = PX_PLACEMENT_NEW(ptr, PxsCMDiscreteUpdateTask)(&mContext, dt, mNewNarrowPhasePairs.mContactManagerMapping.begin() + a, 
			mNewNarrowPhasePairs.mOutputContactManagers.begin() + a, mNewNarrowPhasePairs.mCaches.begin() + a, nbToProcess,
			mModifyCallback);

		a += nbToProcess;

		task->setContinuation(continuation);
		task->removeReference();
	}
	mContext.mTaskPool.unlock();
}

void PxsNphaseImplementationContext::updateContactManager(PxReal dt, bool /*hasBoundsArrayChanged*/, bool /*hasContactDistanceChanged*/, PxBaseTask* continuation, PxBaseTask* firstPassNpContinuation)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", mContext.mContextID);

	firstPassNpContinuation->removeReference();

	mContext.clearManagerTouchEvents();

#if PX_ENABLE_SIM_STATS
	mContext.mSimStats.mNbDiscreteContactPairsTotal = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithCacheHits = 0;
	mContext.mSimStats.mNbDiscreteContactPairsWithContacts = 0;
#endif

	
	//KS - temporarily put this here. TODO - move somewhere better
	mContext.mTotalCompressedCacheSize = 0;
	mContext.mMaxPatches = 0;
	
	processContactManager(dt, mNarrowPhasePairs.mOutputContactManagers.begin(), continuation);

}

void PxsNphaseImplementationContext::secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation)
{
	PX_PROFILE_ZONE("Sim.queueNarrowPhase", mContext.mContextID);

	processContactManagerSecondPass(dt, continuation);		
}

PxsNphaseImplementationContext* PxsNphaseImplementationContext::create(PxsContext& context, IG::IslandSim* islandSim)
{
	PxsNphaseImplementationContext* npImplContext = reinterpret_cast<PxsNphaseImplementationContext*>(
		PX_ALLOC(sizeof(PxsNphaseImplementationContext), "PxsNphaseImplementationContext"));

	if (npImplContext)
	{
		new(npImplContext) PxsNphaseImplementationContext(context, islandSim);
	}

	return npImplContext;
}

void PxsNphaseImplementationContext::destroy()
{
	this->~PxsNphaseImplementationContext();
	PX_FREE(this);
}

void PxsNphaseImplementationContext::registerContactManagers(PxsContactManager** cms, PxU32 nbContactManagers, PxU32 maxContactManagerId)
{
	PX_UNUSED(maxContactManagerId);
	for (PxU32 a = 0; a < nbContactManagers; ++a)
	{
		registerContactManager(cms[a], 0, 0);
	}
}

void PxsNphaseImplementationContext::registerContactManager(PxsContactManager* cm, PxI32 touching, PxU32 patchCount)
{
	PxcNpWorkUnit& workUnit = cm->getWorkUnit();
	PxsContactManagerOutput output;

	PxU8 geomType0 = PxU8(workUnit.geomType0);
	PxU8 geomType1 = PxU8(workUnit.geomType1);

	Gu::Cache cache;

	mContext.createCache(cache, cm, geomType0, geomType1);

	PxMemZero(&output, sizeof(output));
	output.nbPatches = Ps::to8(patchCount);

	if(workUnit.flags & PxcNpWorkUnitFlag::eOUTPUT_CONSTRAINTS)
		output.statusFlag |= PxsContactManagerStatusFlag::eREQUEST_CONSTRAINTS;

	if (touching > 0)
	{
		output.statusFlag |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	}
	else if (touching < 0)
	{
		output.statusFlag |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;
	}

	output.statusFlag |= PxsContactManagerStatusFlag::eDIRTY_MANAGER;

	if (cm->getWorkUnit().statusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
		cm->getWorkUnit().statusFlags |= PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH;

	mNewNarrowPhasePairs.mOutputContactManagers.pushBack(output);
	mNewNarrowPhasePairs.mCaches.pushBack(cache);
	mNewNarrowPhasePairs.mContactManagerMapping.pushBack(cm);
	PxU32 newSz = mNewNarrowPhasePairs.mOutputContactManagers.size();
	cm->getWorkUnit().mNpIndex = mNewNarrowPhasePairs.computeId(newSz - 1) | PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK;
}

void PxsNphaseImplementationContext::removeContactManagersFallback(PxsContactManagerOutput* cmOutputs)
{
	if (mRemovedContactManagers.size())
	{
		Ps::sort(mRemovedContactManagers.begin(), mRemovedContactManagers.size(), Ps::Greater<PxU32>());

		for (PxU32 a = 0; a < mRemovedContactManagers.size(); ++a)
		{
#if PX_DEBUG
			if (a > 0)
				PX_ASSERT(mRemovedContactManagers[a] < mRemovedContactManagers[a - 1]);
#endif
			unregisterContactManagerInternal(mRemovedContactManagers[a], mNarrowPhasePairs, cmOutputs);
		}

		mRemovedContactManagers.forceSize_Unsafe(0);
	}
}

void PxsNphaseImplementationContext::unregisterContactManager(PxsContactManager* cm)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		unregisterContactManagerInternal(index, mNarrowPhasePairs, mNarrowPhasePairs.mOutputContactManagers.begin());
		mNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNarrowPhasePairs.mOutputContactManagers.size()-1);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		unregisterContactManagerInternal(index, mNewNarrowPhasePairs, mNewNarrowPhasePairs.mOutputContactManagers.begin());
		mNewNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNewNarrowPhasePairs.mOutputContactManagers.size()-1);
	}
}

void PxsNphaseImplementationContext::refreshContactManager(PxsContactManager* cm)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);
	PxsContactManagerOutput output;
	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		output = mNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index)];
		unregisterContactManagerInternal(index, mNarrowPhasePairs, mNarrowPhasePairs.mOutputContactManagers.begin());
		mNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNarrowPhasePairs.mOutputContactManagers.size()-1);
	}
	else
	{
		output = mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
		//KS - the index in the "new" list will be the index 
		unregisterContactManagerInternal(index, mNewNarrowPhasePairs, mNewNarrowPhasePairs.mOutputContactManagers.begin());
		mNewNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNewNarrowPhasePairs.mOutputContactManagers.size()-1);
	}
	PxI32 touching = 0;
	if(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH)
		touching = 1;
	else if (output.statusFlag & PxsContactManagerStatusFlag::eHAS_NO_TOUCH)
		touching = -1;
	registerContactManager(cm, touching, output.nbPatches);
}

void PxsNphaseImplementationContext::unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* /*cmOutputs*/)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		mRemovedContactManagers.pushBack(index);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		unregisterContactManagerInternal(index, mNewNarrowPhasePairs, mNewNarrowPhasePairs.mOutputContactManagers.begin());
		mNewNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNewNarrowPhasePairs.mOutputContactManagers.size()-1);
	}
}

void PxsNphaseImplementationContext::refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs)
{
	PxcNpWorkUnit& unit = cm->getWorkUnit();
	PxU32 index = unit.mNpIndex;
	PX_ASSERT(index != 0xFFffFFff);

	PxsContactManagerOutput output;
	if (!(index & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))
	{
		output = cmOutputs[PxsContactManagerBase::computeIndexFromId(index)];
		//unregisterContactManagerInternal(index, mNarrowPhasePairs, cmOutputs);
		unregisterContactManagerFallback(cm, cmOutputs);
	}
	else
	{
		//KS - the index in the "new" list will be the index 
		output = mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(index & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
		unregisterContactManagerInternal(index, mNewNarrowPhasePairs, mNewNarrowPhasePairs.mOutputContactManagers.begin());
		mNewNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(mNewNarrowPhasePairs.mOutputContactManagers.size()-1);
	}

	PxI32 touching = 0;
	if(output.statusFlag & PxsContactManagerStatusFlag::eHAS_TOUCH)
	{
		touching = 1;
		cm->getWorkUnit().statusFlags |= PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH;
	}
	else if (output.statusFlag & PxsContactManagerStatusFlag::eHAS_NO_TOUCH)
		touching = -1;
	registerContactManager(cm, touching, output.nbPatches);
		

}

void PxsNphaseImplementationContext::appendContactManagers()
{
	//Copy new pairs to end of old pairs. Clear new flag, update npIndex on CM and clear the new pair buffer
	const PxU32 existingSize = mNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 nbToAdd = mNewNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 newSize =existingSize + nbToAdd;
	
	if(newSize > mNarrowPhasePairs.mContactManagerMapping.capacity())
	{
		PxU32 newSz = PxMax(256u, PxMax(mNarrowPhasePairs.mContactManagerMapping.capacity()*2, newSize));

		mNarrowPhasePairs.mContactManagerMapping.reserve(newSz);
		mNarrowPhasePairs.mOutputContactManagers.reserve(newSz);
		mNarrowPhasePairs.mCaches.reserve(newSz);
	}

	mNarrowPhasePairs.mContactManagerMapping.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mOutputContactManagers.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mCaches.forceSize_Unsafe(newSize);

	PxMemCopy(mNarrowPhasePairs.mContactManagerMapping.begin() + existingSize, mNewNarrowPhasePairs.mContactManagerMapping.begin(), sizeof(PxsContactManager*)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mOutputContactManagers.begin() + existingSize, mNewNarrowPhasePairs.mOutputContactManagers.begin(), sizeof(PxsContactManagerOutput)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mCaches.begin() + existingSize, mNewNarrowPhasePairs.mCaches.begin(), sizeof(Gu::Cache)*nbToAdd);

	PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

	for(PxU32 a = 0; a < mNewNarrowPhasePairs.mContactManagerMapping.size(); ++a)
	{
		PxsContactManager* cm = mNewNarrowPhasePairs.mContactManagerMapping[a];
		PxcNpWorkUnit& unit = cm->getWorkUnit();
		unit.mNpIndex = mNarrowPhasePairs.computeId(existingSize + a);

		if(unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH)
		{
			unit.statusFlags &= (~PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH);
			if(!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
			{
				PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(unit.mEdgeIndex);

				while(partitionEdge)
				{
					edgeNodeIndices[partitionEdge->mUniqueIndex] = unit.mNpIndex;
					partitionEdge = partitionEdge->mNextPatch;
				}
			}
		}
	}

	mNewNarrowPhasePairs.clear();
}

void PxsNphaseImplementationContext::appendContactManagersFallback(PxsContactManagerOutput* cmOutputs)
{
	PX_PROFILE_ZONE("PxsNphaseImplementationContext.appendContactManagersFallback", mContext.mContextID);

	//Copy new pairs to end of old pairs. Clear new flag, update npIndex on CM and clear the new pair buffer
	const PxU32 existingSize = mNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 nbToAdd = mNewNarrowPhasePairs.mContactManagerMapping.size();
	const PxU32 newSize =existingSize + nbToAdd;
	
	if(newSize > mNarrowPhasePairs.mContactManagerMapping.capacity())
	{
		PxU32 newSz = PxMax(mNarrowPhasePairs.mContactManagerMapping.capacity()*2, newSize);

		mNarrowPhasePairs.mContactManagerMapping.reserve(newSz);
		mNarrowPhasePairs.mCaches.reserve(newSz);
		/*mNarrowPhasePairs.mLostFoundPairsCms.reserve(2 * newSz);
		mNarrowPhasePairs.mLostFoundPairsOutputData.reserve(2*newSz);*/
	}

	mNarrowPhasePairs.mContactManagerMapping.forceSize_Unsafe(newSize);
	mNarrowPhasePairs.mCaches.forceSize_Unsafe(newSize);

	PxMemCopy(mNarrowPhasePairs.mContactManagerMapping.begin() + existingSize, mNewNarrowPhasePairs.mContactManagerMapping.begin(), sizeof(PxsContactManager*)*nbToAdd);
	PxMemCopy(cmOutputs + existingSize, mNewNarrowPhasePairs.mOutputContactManagers.begin(), sizeof(PxsContactManagerOutput)*nbToAdd);
	PxMemCopy(mNarrowPhasePairs.mCaches.begin() + existingSize, mNewNarrowPhasePairs.mCaches.begin(), sizeof(Gu::Cache)*nbToAdd);

	PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

	for(PxU32 a = 0; a < mNewNarrowPhasePairs.mContactManagerMapping.size(); ++a)
	{
		PxsContactManager* cm = mNewNarrowPhasePairs.mContactManagerMapping[a];
		PxcNpWorkUnit& unit = cm->getWorkUnit();
		unit.mNpIndex = mNarrowPhasePairs.computeId(existingSize + a);

		if(unit.statusFlags & PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH)
		{
			unit.statusFlags &= (~PxcNpWorkUnitStatusFlag::eREFRESHED_WITH_TOUCH);
			if(!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
			{
				PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(unit.mEdgeIndex);

				while(partitionEdge)
				{
					edgeNodeIndices[partitionEdge->mUniqueIndex] = unit.mNpIndex;
					partitionEdge = partitionEdge->mNextPatch;
				}
			}
		}
	}

	//const PxU32 existingLostFoundPairs = mNarrowPhasePairs.mLostFoundPairsCms.size();
	//const PxU32 newLostFoundPairs = mNewNarrowPhasePairs.mLostFoundPairsCms.size();
	//const PxU32 newLostFoundSize = existingLostFoundPairs + newLostFoundPairs;

	//if (mNarrowPhasePairs.mLostFoundPairsCms.capacity() < newLostFoundSize)
	//{
	//	const PxU32 newSz = PxMax(newLostFoundSize, 2 * mNarrowPhasePairs.mLostFoundPairsCms.capacity());
	//	mNarrowPhasePairs.mLostFoundPairsCms.reserve(newSz);
	//	mNarrowPhasePairs.mLostFoundPairsOutputData.reserve(newSz);
	//}

	//mNarrowPhasePairs.mLostFoundPairsCms.forceSize_Unsafe(newLostFoundSize);
	//mNarrowPhasePairs.mLostFoundPairsOutputData.forceSize_Unsafe(newLostFoundSize);

	//PxMemCopy(mNarrowPhasePairs.mLostFoundPairsCms.begin() + existingLostFoundPairs, mNewNarrowPhasePairs.mLostFoundPairsCms.begin(), sizeof(PxsContactManager*)* newLostFoundPairs);
	//PxMemCopy(mNarrowPhasePairs.mLostFoundPairsOutputData.begin() + existingLostFoundPairs, mNewNarrowPhasePairs.mLostFoundPairsOutputData.begin(), sizeof(PxsContactManagerOutput) * newLostFoundPairs);

	mNewNarrowPhasePairs.clear();
}



void PxsNphaseImplementationContext::unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs)
{
	//TODO - remove this element from the list.
	PxU32 index = PxsContactManagerBase::computeIndexFromId((npIndex & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK)));

	//Now we replace-with-last and remove the elements...

	PxU32 replaceIndex = managers.mContactManagerMapping.size()-1;

	PxsContactManager* replaceManager = managers.mContactManagerMapping[replaceIndex];

	mContext.destroyCache(managers.mCaches[index]);

	managers.mContactManagerMapping[index] = replaceManager;
	managers.mCaches[index] = managers.mCaches[replaceIndex];
	cmOutputs[index] = cmOutputs[replaceIndex];

	PxU32* edgeNodeIndices = mIslandSim->getEdgeNodeIndexPtr();

	PxcNpWorkUnit& replaceUnit = replaceManager->getWorkUnit();
	replaceUnit.mNpIndex = npIndex;
	if(replaceUnit.statusFlags & PxcNpWorkUnitStatusFlag::eHAS_TOUCH)
	{
		if(!(replaceUnit.flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE))
		{
			PartitionEdge* partitionEdge = mIslandSim->getFirstPartitionEdge(replaceUnit.mEdgeIndex);
			while(partitionEdge)
			{
				edgeNodeIndices[partitionEdge->mUniqueIndex] = replaceUnit.mNpIndex;
				partitionEdge = partitionEdge->mNextPatch;
			}
		}
	}

	managers.mContactManagerMapping.forceSize_Unsafe(replaceIndex);
	managers.mCaches.forceSize_Unsafe(replaceIndex);
}

PxsContactManagerOutput& PxsNphaseImplementationContext::getNewContactManagerOutput(PxU32 npId)
{
	PX_ASSERT(npId & PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK);
	return this->mNewNarrowPhasePairs.mOutputContactManagers[PxsContactManagerBase::computeIndexFromId(npId & (~PxsContactManagerBase::NEW_CONTACT_MANAGER_MASK))];
}

void PxsNphaseImplementationContext::registerShape(const PxsShapeCore& shapeCore)
{
	PX_UNUSED(shapeCore);
}

void PxsNphaseImplementationContext::updateShapeMaterial(const PxsShapeCore& shapeCore)
{
	PX_UNUSED(shapeCore);
}

void PxsNphaseImplementationContext::updateShapeContactOffset(const PxsShapeCore& shapeCore)
{
	PX_UNUSED(shapeCore);
}

void PxsNphaseImplementationContext::unregisterShape(const PxsShapeCore& shapeCore)
{
	PX_UNUSED(shapeCore);
}

void PxsNphaseImplementationContext::registerMaterial(const PxsMaterialCore& materialCore)
{
	PX_UNUSED(materialCore);
}

void PxsNphaseImplementationContext::updateMaterial(const PxsMaterialCore& materialCore)
{
	PX_UNUSED(materialCore);
}

void PxsNphaseImplementationContext::unregisterMaterial(const PxsMaterialCore& materialCore)
{
	PX_UNUSED(materialCore);
}

PxsContactManagerOutputIterator PxsNphaseImplementationContext::getContactManagerOutputs()
{
	PxU32 offsets[1] = {0};
	return PxsContactManagerOutputIterator(offsets, 1, this->mNarrowPhasePairs.mOutputContactManagers.begin());
}


PxvNphaseImplementationContextUsableAsFallback* physx::createNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim)
{
	return PxsNphaseImplementationContext::create(context, islandSim);
}

