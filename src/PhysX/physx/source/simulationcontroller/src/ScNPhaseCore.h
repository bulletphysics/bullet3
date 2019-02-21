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

#ifndef PX_PHYSICS_SCP_NPHASE_CORE
#define PX_PHYSICS_SCP_NPHASE_CORE

#include "CmPhysXCommon.h"
#include "CmRenderOutput.h"
#include "PxPhysXConfig.h"
#include "PsUserAllocated.h"
#include "PsMutex.h"
#include "PsAtomic.h"
#include "PsPool.h"
#include "PsHashSet.h"
#include "PsHashMap.h"
#include "PxSimulationEventCallback.h"
#include "ScTriggerPairs.h"
#include "ScScene.h"
#include "ScContactReportBuffer.h"
#include "PsHash.h"

namespace physx
{
namespace Bp
{
	struct AABBOverlap;
	struct BroadPhasePair;
}

namespace Sc
{
	class ActorSim;
	class ElementSim;
	class ShapeSim;

	class Interaction;
	class ElementSimInteraction;
	class ElementInteractionMarker;
	class TriggerInteraction;

	class ShapeInteraction;
	class ActorPair;
	class ActorPairReport;

	class ActorPairContactReportData;
	struct ContactShapePair;

	class NPhaseContext;
	class ContactStreamManager;

	struct FilterPair;
	class FilterPairManager;

	class RigidSim;

	struct PairReleaseFlag
	{
		enum Enum
		{
			eBP_VOLUME_REMOVED			=	(1 << 0),	// the broadphase volume of one pair object has been removed.
			eSHAPE_BP_VOLUME_REMOVED	=	(1 << 1) | eBP_VOLUME_REMOVED,  // the removed broadphase volume was from a rigid body shape.
			eWAKE_ON_LOST_TOUCH			=	(1 << 2)
		};
	};

	/*
	Description: NPhaseCore encapsulates the near phase processing to allow multiple implementations(eg threading and non
	threaded).

	The broadphase inserts shape pairs into the NPhaseCore, which are then processed into contact point streams.
	Pairs can then be processed into AxisConstraints by the GroupSolveCore.
	*/

	struct BodyPairKey
	{
		PxU32 mSim0;
		PxU32 mSim1;

		PX_FORCE_INLINE	bool operator == (const BodyPairKey& pair) const { return mSim0 == pair.mSim0 && mSim1 == pair.mSim1; }
	};

	PX_INLINE PxU32 hash(const BodyPairKey& key)
	{
		const PxU32 add0 = key.mSim0;
		const PxU32 add1 = key.mSim1;

		const PxU32 base = PxU32((add0 & 0xFFFF) | (add1 << 16));

		return physx::shdfnd::hash(base);
	}

	struct ElementSimKey
	{
		ElementSim* mSim0, *mSim1;

		ElementSimKey() : mSim0(NULL), mSim1(NULL)
		{}

		ElementSimKey(ElementSim* sim0, ElementSim* sim1)
		{
			if(sim0 > sim1)
				Ps::swap(sim0, sim1);
			 mSim0 = sim0;
			 mSim1 = sim1;
		}

		PX_FORCE_INLINE	bool operator == (const ElementSimKey& pair) const { return mSim0 == pair.mSim0 && mSim1 == pair.mSim1; }
	};

	PX_INLINE PxU32 hash(const ElementSimKey& key)
	{
		PxU32 add0 = (size_t(key.mSim0)) & 0xFFFFFFFF;
		PxU32 add1 = (size_t(key.mSim1)) & 0xFFFFFFFF;

		//Clear the lower 2 bits, they will be 0s anyway
		add0 = add0 >> 2;
		add1 = add1 >> 2;

		const PxU32 base = PxU32((add0 & 0xFFFF) | (add1 << 16));

		return physx::shdfnd::hash(base);
	}

	class ContactReportAllocationManager
	{
		PxU8* mBuffer;
		PxU32 mBufferSize;
		PxU32 mCurrentBufferIndex;
		PxU32 mCurrentOffset;
		ContactReportBuffer& mReportBuffer;
		Ps::Mutex& mMutex;
		const PxU32 mBuferBlockSize;
		PX_NOCOPY(ContactReportAllocationManager)
	public:

		ContactReportAllocationManager(ContactReportBuffer& buffer, Ps::Mutex& mutex, const PxU32 bufferBlockSize = 16384) : mBuffer(NULL), mBufferSize(0), mCurrentBufferIndex(0), 
			mCurrentOffset(0), mReportBuffer(buffer), mMutex(mutex), mBuferBlockSize(bufferBlockSize)
		{
		}

		PxU8* allocate(const PxU32 size, PxU32& index, PxU32 alignment = 16u)
		{
			//(1) fix up offsets...
			PxU32 pad = ((mCurrentBufferIndex + alignment - 1)&~(alignment - 1)) - mCurrentBufferIndex;
			PxU32 currOffset = mCurrentOffset + pad;

			if ((currOffset + size) > mBufferSize)
			{
				const PxU32 allocSize = PxMax(size, mBuferBlockSize);

				mMutex.lock();
				mBuffer = mReportBuffer.allocateNotThreadSafe(allocSize, mCurrentBufferIndex, alignment);
				mCurrentOffset = currOffset = 0;
				mBufferSize = allocSize;
				mMutex.unlock();
			}

			PxU8* ret = mBuffer + currOffset;
			index = mCurrentBufferIndex + currOffset;
			mCurrentOffset = currOffset + size;
			return ret;
		}
	};

	class NPhaseCore : public Ps::UserAllocated
	{
		PX_NOCOPY(NPhaseCore)

	public:
		NPhaseCore(Scene& scene, const PxSceneDesc& desc);
		~NPhaseCore();

		void onOverlapCreated(const Bp::AABBOverlap* PX_RESTRICT pairs, PxU32 pairCount);

		void	runOverlapFilters(	PxU32 nbToProcess, const Bp::AABBOverlap* PX_RESTRICT pairs, PxFilterInfo* PX_RESTRICT filterInfo,
										PxU32& nbToKeep, PxU32& nbToSuppress, PxU32& nbToCallback, PxU32* PX_RESTRICT keepMap, PxU32* PX_RESTRICT callbackMap);

		ElementSimInteraction* onOverlapRemovedStage1(ElementSim* volume0, ElementSim* volume1);
		void onOverlapRemoved(ElementSim* volume0, ElementSim* volume1, const PxU32 ccdPass, void* elemSim, PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);
		void onVolumeRemoved(ElementSim* volume, PxU32 flags, PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);

		void managerNewTouch(Sc::ShapeInteraction& interaction);

		PxU32 getDefaultContactReportStreamBufferSize() const;

		void fireCustomFilteringCallbacks(PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);

		void addToDirtyInteractionList(Interaction* interaction);
		void removeFromDirtyInteractionList(Interaction* interaction);
		void updateDirtyInteractions(PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);

		/*
		Description: Perform trigger overlap tests.
		*/
		void processTriggerInteractions(PxBaseTask* continuation);

		/*
		Description: Gather results from trigger overlap tests and clean up.
		*/
		void mergeProcessedTriggerInteractions(PxBaseTask* continuation);

		/*
		Description: Check candidates for persistent touch contact events and create those events if necessary.
		*/
		void processPersistentContactEvents(PxsContactManagerOutputIterator& outputs, PxBaseTask* continuation);

		/*
		Description: Displays visualizations associated with the near phase.
		*/
		void visualize(Cm::RenderOutput& out, PxsContactManagerOutputIterator& outputs);

		PX_FORCE_INLINE Scene& getScene() const	{ return mOwnerScene;	}

		PX_FORCE_INLINE void addToContactReportActorPairSet(ActorPairReport* pair) { mContactReportActorPairSet.pushBack(pair); }
		void clearContactReportActorPairs(bool shrinkToZero);
		PX_FORCE_INLINE PxU32 getNbContactReportActorPairs() const { return mContactReportActorPairSet.size(); }
		PX_FORCE_INLINE ActorPairReport* const* getContactReportActorPairs() const { return mContactReportActorPairSet.begin(); }

		void addToPersistentContactEventPairs(ShapeInteraction*);
		void addToPersistentContactEventPairsDelayed(ShapeInteraction*);
		void removeFromPersistentContactEventPairs(ShapeInteraction*);
		PX_FORCE_INLINE PxU32 getCurrentPersistentContactEventPairCount() const { return mNextFramePersistentContactEventPairIndex; }
		PX_FORCE_INLINE ShapeInteraction* const* getCurrentPersistentContactEventPairs() const { return mPersistentContactEventPairList.begin(); }
		PX_FORCE_INLINE PxU32 getAllPersistentContactEventPairCount() const { return mPersistentContactEventPairList.size(); }
		PX_FORCE_INLINE ShapeInteraction* const* getAllPersistentContactEventPairs() const { return mPersistentContactEventPairList.begin(); }
		PX_FORCE_INLINE void preparePersistentContactEventListForNextFrame();

		void addToForceThresholdContactEventPairs(ShapeInteraction*);
		void removeFromForceThresholdContactEventPairs(ShapeInteraction*);
		PX_FORCE_INLINE PxU32 getForceThresholdContactEventPairCount() const { return mForceThresholdContactEventPairList.size(); }
		PX_FORCE_INLINE ShapeInteraction* const* getForceThresholdContactEventPairs() const { return mForceThresholdContactEventPairList.begin(); }

		PX_FORCE_INLINE PxU8* getContactReportPairData(const PxU32& bufferIndex) const { return mContactReportBuffer.getData(bufferIndex); }
		PxU8* reserveContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, PxU32& bufferIndex, ContactReportAllocationManager* alloc = NULL);
		PxU8* resizeContactReportPairData(PxU32 pairCount, PxU32 extraDataSize, Sc::ContactStreamManager& csm);
		PX_FORCE_INLINE void clearContactReportStream() { mContactReportBuffer.reset(); }  // Do not free memory at all
		PX_FORCE_INLINE void freeContactReportStreamMemory() { mContactReportBuffer.flush(); }

		ActorPairContactReportData* createActorPairContactReportData();
		void releaseActorPairContactReportData(ActorPairContactReportData* data);

		void registerInteraction(ElementSimInteraction* interaction);
		void unregisterInteraction(ElementSimInteraction* interaction);
		
		ElementSimInteraction* createRbElementInteraction(const PxFilterInfo& fInfo, ShapeSim& s0, ShapeSim& s1, PxsContactManager* contactManager, Sc::ShapeInteraction* shapeInteraction, 
			Sc::ElementInteractionMarker* interactionMarker, bool isTriggerPair);

	private:
		ElementSimInteraction* createRbElementInteraction(ShapeSim& s0, ShapeSim& s1, PxsContactManager* contactManager, Sc::ShapeInteraction* shapeInteraction, 
			Sc::ElementInteractionMarker* interactionMarker);

		void releaseElementPair(ElementSimInteraction* pair, PxU32 flags, const PxU32 ccdPass, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);
		void lostTouchReports(ShapeInteraction* pair, PxU32 flags, const PxU32 ccdPass, PxsContactManagerOutputIterator& outputs, bool useAdaptiveForce);

		ShapeInteraction* createShapeInteraction(ShapeSim& s0, ShapeSim& s1, PxPairFlags pairFlags, PxsContactManager* contactManager, Sc::ShapeInteraction* shapeInteraction);
		TriggerInteraction* createTriggerInteraction(ShapeSim& s0, ShapeSim& s1, PxPairFlags triggerFlags);
		ElementInteractionMarker* createElementInteractionMarker(ElementSim& e0, ElementSim& e1, ElementInteractionMarker* marker);

		//------------- Filtering -------------

		ElementSimInteraction* refilterInteraction(ElementSimInteraction* pair, const PxFilterInfo* filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs,
			bool useAdaptiveForce);
		//-------------------------------------

		ElementSimInteraction* convert(ElementSimInteraction* pair, InteractionType::Enum type, PxFilterInfo& filterInfo, bool removeFromDirtyList, PxsContactManagerOutputIterator& outputs,
			bool useAdaptiveForce);

		ActorPair* findActorPair(ShapeSim* s0, ShapeSim* s1, Ps::IntBool isReportPair);
		PX_FORCE_INLINE void destroyActorPairReport(ActorPairReport&);

		Sc::ElementSimInteraction* findInteraction(ElementSim* _element0, ElementSim* _element1);

		// Pooling
		Scene&											mOwnerScene;

		Ps::Array<ActorPairReport*>						mContactReportActorPairSet;
		Ps::Array<ShapeInteraction*>					mPersistentContactEventPairList;	// Pairs which request events which do not get triggered by the sdk and thus need to be tested actively every frame.
																							// May also contain force threshold event pairs (see mForceThresholdContactEventPairList)
																							// This list is split in two, the elements in front are for the current frame, the elements at the
																							// back will get added next frame.
		PxU32											mNextFramePersistentContactEventPairIndex;  // start index of the pairs which need to get added to the persistent list for next frame

		Ps::Array<ShapeInteraction*>					mForceThresholdContactEventPairList;	// Pairs which request force threshold contact events. A pair is only in this list if it does have contact.
																								// Note: If a pair additionally requests PxPairFlag::eNOTIFY_TOUCH_PERSISTS events, then it
																								// goes into mPersistentContactEventPairList instead. This allows to share the list index.
		//
		//  data layout:
		//  ContactActorPair0_ExtraData, ContactShapePair0_0, ContactShapePair0_1, ... ContactShapePair0_N, 
		//  ContactActorPair1_ExtraData, ContactShapePair1_0, ...
		//
		ContactReportBuffer								mContactReportBuffer;				// Shape pair information for contact reports

		Ps::CoalescedHashSet<Interaction*>				mDirtyInteractions;
		FilterPairManager*								mFilterPairManager;

		// Pools
		Ps::Pool<ActorPair>								mActorPairPool;
		Ps::Pool<ActorPairReport>						mActorPairReportPool;
		Ps::Pool<ShapeInteraction>						mShapeInteractionPool;
		Ps::Pool<TriggerInteraction>					mTriggerInteractionPool;
		Ps::Pool<ActorPairContactReportData>			mActorPairContactReportDataPool;
		Ps::Pool<ElementInteractionMarker>				mInteractionMarkerPool;

		Cm::DelegateTask<Sc::NPhaseCore, &Sc::NPhaseCore::mergeProcessedTriggerInteractions> mMergeProcessedTriggerInteractions;
		void*											mTmpTriggerProcessingBlock;  // temporary memory block to process trigger pairs in parallel
		Ps::Mutex										mTriggerWriteBackLock;
		volatile PxI32									mTriggerPairsToDeactivateCount;
		Ps::HashMap<BodyPairKey, ActorPair*>			mActorPairMap; 

		Ps::HashMap<ElementSimKey, ElementSimInteraction*> mElementSimMap;

		Ps::Mutex										mBufferAllocLock;
		Ps::Mutex										mReportAllocLock;

		friend class Sc::Scene;
		friend class Sc::ShapeInteraction;
	};

	struct FilteringContext
	{
		FilteringContext(const Sc::Scene& scene, FilterPairManager* filterPairManager) :
			mFilterShader			(scene.getFilterShaderFast()),
			mFilterShaderData		(scene.getFilterShaderDataFast()),
			mFilterShaderDataSize	(scene.getFilterShaderDataSizeFast()),
			mFilterCallback			(scene.getFilterCallbackFast()),
			mFilterPairManager		(filterPairManager),
			mSceneFlags				(scene.getPublicFlags())
		{
			const PxPairFilteringMode::Enum kineKineFilteringMode = scene.getKineKineFilteringMode();
			if(kineKineFilteringMode==PxPairFilteringMode::eKEEP)
				mSceneFlags |= PxSceneFlag::eENABLE_KINEMATIC_PAIRS;
			else if(kineKineFilteringMode==PxPairFilteringMode::eSUPPRESS)
				mSceneFlags &= ~PxSceneFlag::eENABLE_KINEMATIC_PAIRS;

			const PxPairFilteringMode::Enum staticKineFilteringMode = scene.getStaticKineFilteringMode();
			if(staticKineFilteringMode==PxPairFilteringMode::eKEEP)
				mSceneFlags |= PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS;
			else if(staticKineFilteringMode==PxPairFilteringMode::eSUPPRESS)
				mSceneFlags &= ~PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS;
		}

		PxSimulationFilterShader	mFilterShader;
		const void*					mFilterShaderData;
		PxU32						mFilterShaderDataSize;
		PxSimulationFilterCallback*	mFilterCallback;
		FilterPairManager*			mFilterPairManager;
		PxSceneFlags				mSceneFlags;
	};

	// helper function to run the filter logic after some hardwired filter criteria have been passed successfully
	PxFilterInfo filterRbCollisionPairSecondStage(const FilteringContext& context, const ShapeSim& s0, const ShapeSim& s1, const Sc::BodySim* b0, const Sc::BodySim* b1, PxU32 filterPairIndex, bool runCallbacks);

} // namespace Sc


PX_FORCE_INLINE void Sc::NPhaseCore::preparePersistentContactEventListForNextFrame()
{
	// reports have been processed -> "activate" next frame candidates for persistent contact events
	mNextFramePersistentContactEventPairIndex = mPersistentContactEventPairList.size();
}

}

#endif
