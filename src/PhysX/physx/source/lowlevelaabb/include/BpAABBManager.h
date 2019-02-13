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

#ifndef BP_AABBMANAGER_H
#define BP_AABBMANAGER_H

	#include "PxvConfig.h"
	#include "CmPhysXCommon.h"
	#include "BpBroadPhaseUpdate.h"
	#include "GuGeometryUnion.h"
	#include "CmBitMap.h"
	#include "CmTask.h"
	#include "PsAllocator.h"
	#include "GuBounds.h"
	#include "PsHashMap.h"
	#include "CmRadixSortBuffered.h"
	#include "PsFoundation.h"
	#include "BpAABBManagerTasks.h"
	#include "PsHashSet.h"
	#include "PxFiltering.h"
	#include "PsSList.h"

	namespace physx
	{
	class PxcScratchAllocator;
	struct PxBroadPhaseType;

	namespace Cm
	{
		class RenderOutput;
		class EventProfiler;
		class FlushPool;
	}

	namespace Bp
	{
		typedef PxU32 BoundsIndex;
		typedef PxU32 AggregateHandle;	// PT: currently an index in mAggregates array
		typedef PxU32 ActorHandle;

		struct BroadPhasePair;

		struct ElementType
		{
			enum Enum
			{
				eSHAPE = 0,
				eTRIGGER,

				eCOUNT
			};
		};
		PX_COMPILE_TIME_ASSERT(ElementType::eCOUNT <= 4);			// 2 bits reserved for type

		/**
		\brief Changes to the configuration of overlap pairs are reported as void* pairs.
		\note Each void* in the pair corresponds to the void* passed to AABBManager::createVolume.
		@see AABBManager::createVolume, AABBManager::getCreatedOverlaps, AABBManager::getDestroyedOverlaps
		*/
		struct AABBOverlap
		{
			PX_FORCE_INLINE AABBOverlap()	{}
			PX_FORCE_INLINE AABBOverlap(void* userData0, void* userData1/*, ActorHandle pairHandle*/) : mUserData0(userData0), mUserData1(userData1)/*, mPairHandle(pairHandle)*/	{}

			void*	mUserData0;
			void*	mUserData1;
	/*		union
			{
				ActorHandle	mPairHandle;		//For created pairs, this is the index into the pair in the pair manager
				void*		mUserData;			//For deleted pairs, this is the user data written by the application to the pair
			};*/
			void*		mPairUserData;			//For deleted pairs, this is the user data written by the application to the pair
		};

		struct BpCacheData : public Ps::SListEntry
		{
			Ps::Array<AABBOverlap> mCreatedPairs[2];
			Ps::Array<AABBOverlap> mDeletedPairs[2];

			void reset()
			{
				mCreatedPairs[0].resizeUninitialized(0);
				mCreatedPairs[1].resizeUninitialized(0);
				mDeletedPairs[0].resizeUninitialized(0);
				mDeletedPairs[1].resizeUninitialized(0);
			}
		};

		class BoundsArray : public Ps::UserAllocated
		{
			PX_NOCOPY(BoundsArray)

		public:
			BoundsArray(Ps::VirtualAllocator& allocator) : mBounds(allocator)
			{
			}

			PX_FORCE_INLINE void initEntry(PxU32 index)
			{
				index++;	// PT: always pretend we need one more entry, to make sure reading the last used entry will be SIMD-safe.
				const PxU32 oldCapacity = mBounds.capacity();
				if(index>=oldCapacity)
				{
					const PxU32 newCapacity = Ps::nextPowerOfTwo(index);
					mBounds.reserve(newCapacity);
					mBounds.forceSize_Unsafe(newCapacity);
				}
			}

			PX_FORCE_INLINE void updateBounds(const PxTransform& transform, const Gu::GeometryUnion& geom, PxU32 index)
			{
				Gu::computeBounds(mBounds[index], geom.getGeometry(), transform, 0.0f, NULL, 1.0f);
				mHasAnythingChanged = true;
			}

			PX_FORCE_INLINE const PxBounds3& getBounds(PxU32 index) const
			{
				return mBounds[index];
			}

			PX_FORCE_INLINE void setBounds(const PxBounds3& bounds, PxU32 index)
			{
	//			PX_CHECK_AND_RETURN(bounds.isValid() && !bounds.isEmpty(), "BoundsArray::setBounds - illegal bounds\n");
				mBounds[index] = bounds;
				mHasAnythingChanged = true;
			}

			PX_FORCE_INLINE const PxBounds3* begin()	const
			{
				return mBounds.begin();
			}

			PX_FORCE_INLINE PxBounds3* begin()
			{
				return mBounds.begin();
			}

			PX_FORCE_INLINE Ps::Array<PxBounds3, Ps::VirtualAllocator>& getBounds()
			{
				return mBounds;
			}

			PX_FORCE_INLINE PxU32 getCapacity()	const
			{
				return mBounds.size();
			}

			void shiftOrigin(const PxVec3& shift)
			{
				// we shift some potential NaNs here because we don't know what's active, but should be harmless
				for(PxU32 i=0;i<mBounds.size();i++)
				{
					mBounds[i].minimum -= shift;
					mBounds[i].maximum -= shift;
				}
				mHasAnythingChanged = true;
			}

			PX_FORCE_INLINE	bool hasChanged()	const	{ return mHasAnythingChanged;	}
			PX_FORCE_INLINE	void resetChangedState()	{ mHasAnythingChanged = false;	}
			PX_FORCE_INLINE	void setChangedState()		{ mHasAnythingChanged = true;	}

		private:
			Ps::Array<PxBounds3, Ps::VirtualAllocator>	mBounds;
			bool										mHasAnythingChanged;
		};

		struct VolumeData
		{ 
			PX_FORCE_INLINE	void			reset()
											{
												mAggregate = PX_INVALID_U32;
												mUserData = NULL;
											}

			PX_FORCE_INLINE	void			setSingleActor()						{ mAggregate = PX_INVALID_U32;				}
			PX_FORCE_INLINE	bool			isSingleActor()	const					{ return mAggregate == PX_INVALID_U32;		}

			PX_FORCE_INLINE void			setUserData(void* userData)
											{
	//											PX_ASSERT(!(reinterpret_cast<size_t>(userData) & 3));
												mUserData = userData;
											}

			PX_FORCE_INLINE void*			getUserData() const
											{
												return reinterpret_cast<void*>(reinterpret_cast<size_t>(mUserData)& (~size_t(3)));
											}

			PX_FORCE_INLINE void			setVolumeType(ElementType::Enum volumeType)
											{
												PX_ASSERT(volumeType < 2);
												mUserData = reinterpret_cast<void*>(reinterpret_cast<size_t>(getUserData()) | static_cast<size_t>(volumeType));
											}

			PX_FORCE_INLINE ElementType::Enum	getVolumeType() const
											{
												return ElementType::Enum(reinterpret_cast<size_t>(mUserData) & 3);
											}

			PX_FORCE_INLINE	void			setAggregate(AggregateHandle handle)
			{
				PX_ASSERT(handle!=PX_INVALID_U32);
				mAggregate = (handle<<1)|1;
			}
			PX_FORCE_INLINE	bool			isAggregate()	const					{ return !isSingleActor() && ((mAggregate&1)!=0);	}

			PX_FORCE_INLINE	void			setAggregated(AggregateHandle handle)
			{
				PX_ASSERT(handle!=PX_INVALID_U32);
				mAggregate = (handle<<1)|0;
			}

			PX_FORCE_INLINE	bool			isAggregated()	const
			{
				return !isSingleActor() && ((mAggregate&1)==0);
			}

			PX_FORCE_INLINE	AggregateHandle	getAggregateOwner()	const		{ return mAggregate>>1;					}
			PX_FORCE_INLINE	AggregateHandle	getAggregate()		const		{ return mAggregate>>1;					}

			private:
			void*			mUserData;
			// PT: TODO: consider moving this to a separate array, which wouldn't be allocated at all for people not using aggregates.
			// PT: current encoding:
			// aggregate == PX_INVALID_U32 => single actor
			// aggregate != PX_INVALID_U32 => aggregate index<<1|LSB. LSB==1 for aggregates, LSB==0 for aggregated actors.
			AggregateHandle mAggregate;
		};

		// PT: TODO: revisit this.....
		class Aggregate;
		class PersistentPairs;
		class PersistentActorAggregatePair;
		class PersistentAggregateAggregatePair;
		class PersistentSelfCollisionPairs;
		struct AggPair
		{
			PX_FORCE_INLINE AggPair() {}
			PX_FORCE_INLINE	AggPair(ShapeHandle index0, ShapeHandle index1) : mIndex0(index0), mIndex1(index1)	{}
			ShapeHandle	mIndex0;
			ShapeHandle	mIndex1;

			PX_FORCE_INLINE bool operator==(const AggPair& p) const
			{
				return (p.mIndex0 == mIndex0) && (p.mIndex1 == mIndex1);
			}
		};
		typedef Ps::CoalescedHashMap<AggPair, PersistentPairs*> AggPairMap;

		// PT: TODO: isn't there a generic pair structure somewhere? refactor with AggPair anyway
		struct Pair
		{
			PX_FORCE_INLINE	Pair(PxU32 id0, PxU32 id1) : mID0(id0), mID1(id1)	{}
			PX_FORCE_INLINE Pair(){}

				PX_FORCE_INLINE bool operator<(const Pair& p) const
				{
					const PxU64 value0 = *reinterpret_cast<const PxU64*>(this);
					const PxU64 value1 = *reinterpret_cast<const PxU64*>(&p);
					return value0 < value1;
				}

				PX_FORCE_INLINE bool operator==(const Pair& p) const
				{
					return (p.mID0 == mID0) && (p.mID1 == mID1);
				}

				PX_FORCE_INLINE bool operator!=(const Pair& p) const
				{
					return (p.mID0 != mID0) || (p.mID1 != mID1);
				}

			PxU32	mID0;
			PxU32	mID1;
		};

		class AABBManager;

		class PostBroadPhaseStage2Task : public Cm::Task
		{
			Cm::FlushPool* mFlushPool;
			AABBManager& mManager;

			PX_NOCOPY(PostBroadPhaseStage2Task)
		public:

			PostBroadPhaseStage2Task(PxU64 contextID, AABBManager& manager) : Cm::Task(contextID), mFlushPool(NULL), mManager(manager)
			{
			}

			virtual const char* getName() const { return "PostBroadPhaseStage2Task"; }

			void setFlushPool(Cm::FlushPool* pool) { mFlushPool = pool; }

			virtual void runInternal();

		};

		class ProcessAggPairsBase;

		/**
		\brief A structure responsible for:
		* storing an aabb representation for each active shape in the related scene
		* managing the creation/removal of aabb representations when their related shapes are created/removed
		* updating all aabbs that require an update due to modification of shape geometry or transform
		* updating the aabb of all aggregates from the union of the aabbs of all shapes that make up each aggregate
		* computing and reporting the incremental changes to the set of overlapping aabb pairs 
		*/
		class AABBManager : public Ps::UserAllocated
		{
			PX_NOCOPY(AABBManager)
		public:

			AABBManager(BroadPhase& bp, BoundsArray& boundsArray, Ps::Array<PxReal, Ps::VirtualAllocator>& contactDistance,
						PxU32 maxNbAggregates, PxU32 maxNbShapes, Ps::VirtualAllocator& allocator, PxU64 contextID,
						PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode);

			void	destroy();

			AggregateHandle	createAggregate(BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, const bool selfCollisions);
			bool			destroyAggregate(BoundsIndex& index, Bp::FilterGroup::Enum& group, AggregateHandle aggregateHandle);

			bool			addBounds(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userdata, AggregateHandle aggregateHandle, ElementType::Enum volumeType);
			void			reserveSpaceForBounds(BoundsIndex index);
			void			removeBounds(BoundsIndex index);
			PX_FORCE_INLINE Ps::IntBool isMarkedForRemove(BoundsIndex index) const { return mRemovedHandleMap.boundedTest(index); }

			void			setContactOffset(BoundsIndex handle, PxReal offset)
			{
				// PT: this works even for aggregated shapes, since the corresponding bit will also be set in the 'updated' map.
				mContactDistance.begin()[handle] = offset;
				mPersistentStateChanged = true;
				mChangedHandleMap.growAndSet(handle);
			}

			void setVolumeType(BoundsIndex handle, ElementType::Enum volumeType)
			{
				mVolumeData[handle].setVolumeType(volumeType);
			}

			void setBPGroup(BoundsIndex index, Bp::FilterGroup::Enum group)
			{
				PX_ASSERT((index + 1) < mVolumeData.size());
				PX_ASSERT(group != Bp::FilterGroup::eINVALID);	// PT: we use group == Bp::FilterGroup::eINVALID to mark removed/invalid entries
				mGroups[index] = group;
			}

			// PT: TODO: revisit name: we don't "update AABBs" here anymore
			void			updateAABBsAndBP(	PxU32 numCpuTasks,
												Cm::FlushPool& flushPool,
												PxcScratchAllocator* scratchAllocator,
												bool hasContactDistanceUpdated,
												PxBaseTask* continuation,
												PxBaseTask* narrowPhaseUnlockTask);

			void			finalizeUpdate(		PxU32 numCpuTasks,
												PxcScratchAllocator* scratchAllocator,
												PxBaseTask* continuation,
												PxBaseTask* narrowPhaseUnlockTask);

			AABBOverlap*				getCreatedOverlaps(ElementType::Enum type, PxU32& count)
			{
				PX_ASSERT(type < ElementType::eCOUNT);
				count = mCreatedOverlaps[type].size();
				return mCreatedOverlaps[type].begin();
			}

			AABBOverlap*				getDestroyedOverlaps(ElementType::Enum type, PxU32& count)
			{
				PX_ASSERT(type < ElementType::eCOUNT);
				count = mDestroyedOverlaps[type].size();
				return mDestroyedOverlaps[type].begin();
			}

			void						freeBuffers();

			void**						getOutOfBoundsObjects(PxU32& nbOutOfBoundsObjects)
			{
				nbOutOfBoundsObjects = mOutOfBoundsObjects.size();
				return mOutOfBoundsObjects.begin();
			}

			void						clearOutOfBoundsObjects()
			{
				mOutOfBoundsObjects.clear();
			}

			void**						getOutOfBoundsAggregates(PxU32& nbOutOfBoundsAggregates)
			{
				nbOutOfBoundsAggregates = mOutOfBoundsAggregates.size();
				return mOutOfBoundsAggregates.begin();
			}

			void						clearOutOfBoundsAggregates()
			{
				mOutOfBoundsAggregates.clear();
			}

			void						shiftOrigin(const PxVec3& shift);

			void						visualize(Cm::RenderOutput& out);

			PX_FORCE_INLINE	BroadPhase*					getBroadPhase()				const	{ return &mBroadPhase;				}
			PX_FORCE_INLINE	BoundsArray&				getBoundsArray()					{ return mBoundsArray;				}
			PX_FORCE_INLINE	PxU32						getNbActiveAggregates()		const	{ return mNbAggregates;				}
			PX_FORCE_INLINE	const float*				getContactDistances()		const	{ return mContactDistance.begin();	}
			PX_FORCE_INLINE	Cm::BitMapPinned&			getChangedAABBMgActorHandleMap()	{ return mChangedHandleMap;			}

			PX_FORCE_INLINE void*						getUserData(const BoundsIndex index) const { if (index < mVolumeData.size()) return mVolumeData[index].getUserData(); return NULL; }
			PX_FORCE_INLINE	PxU64						getContextId()				const	{ return mContextID;				}

			void postBroadPhase(PxBaseTask*, PxBaseTask* narrowPhaseUnlockTask, Cm::FlushPool& flushPool);



			BpCacheData* getBpCacheData();
			void putBpCacheData(BpCacheData*);
			void resetBpCacheData();

			Ps::Mutex					mMapLock;

		private:
			void reserveShapeSpace(PxU32 nbShapes);

			void postBpStage2(PxBaseTask*, Cm::FlushPool&);

			void postBpStage3(PxBaseTask*);

			PostBroadPhaseStage2Task														mPostBroadPhase2;
			Cm::DelegateTask<AABBManager, &AABBManager::postBpStage3>						mPostBroadPhase3;
		
			//Cm::DelegateTask<SimpleAABBManager, &AABBManager::postBroadPhase>			mPostBroadPhase;

			FinalizeUpdateTask				mFinalizeUpdateTask;

			// PT: we have bitmaps here probably to quickly handle added/removed objects during same frame.
			// PT: TODO: consider replacing with plain arrays (easier to parse, already existing below, etc)
			Cm::BitMap					mAddedHandleMap;		// PT: indexed by BoundsIndex
			Cm::BitMap					mRemovedHandleMap;		// PT: indexed by BoundsIndex
			Cm::BitMapPinned			mChangedHandleMap;

			PX_FORCE_INLINE void removeBPEntry(BoundsIndex index)	// PT: only for objects passed to the BP
			{
				if(mAddedHandleMap.test(index))		// PT: if object had been added this frame...
					mAddedHandleMap.reset(index);	// PT: ...then simply revert the previous operation locally (it hasn't been passed to the BP yet).
				else
					mRemovedHandleMap.set(index);	// PT: else we need to remove it from the BP
			}

			PX_FORCE_INLINE void addBPEntry(BoundsIndex index)
			{
				if(mRemovedHandleMap.test(index))
					mRemovedHandleMap.reset(index);
				else
					mAddedHandleMap.set(index);
			}

			// PT: TODO: when do we need 'Ps::VirtualAllocator' and when don't we? When memory is passed to GPU BP?
			//ML: we create mGroups and mContactDistance in the AABBManager constructor. Ps::Array will take Ps::VirtualAllocator as a parameter. Therefore, if GPU BP is using,
			//we will passed a pinned host memory allocator, otherwise, we will just pass a normal allocator.
			Ps::Array<Bp::FilterGroup::Enum, Ps::VirtualAllocator>	mGroups;				// NOTE: we stick Bp::FilterGroup::eINVALID in this slot to indicate that the entry is invalid (removed or never inserted.)
			Ps::Array<PxReal, Ps::VirtualAllocator>& 				mContactDistance;
			Ps::Array<VolumeData>									mVolumeData;
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
			bool													mLUT[Bp::FilterType::COUNT][Bp::FilterType::COUNT];
	#endif
			PX_FORCE_INLINE void initEntry(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData)
			{
				if ((index + 1) >= mVolumeData.size())
					reserveShapeSpace(index + 1);

				// PT: TODO: why is this needed at all? Why aren't size() and capacity() enough?
				mUsedSize = PxMax(index+1, mUsedSize);

				PX_ASSERT(group != Bp::FilterGroup::eINVALID);	// PT: we use group == Bp::FilterGroup::eINVALID to mark removed/invalid entries
				mGroups[index] = group;
				mContactDistance.begin()[index] = contactDistance;
				mVolumeData[index].setUserData(userData);
			}

			PX_FORCE_INLINE void resetEntry(BoundsIndex index)
			{
				mGroups[index] = Bp::FilterGroup::eINVALID;
				mContactDistance.begin()[index] = 0.0f;
				mVolumeData[index].reset();
			}

			// PT: TODO: remove confusion between BoundsIndex and ShapeHandle here!
			Ps::Array<ShapeHandle, Ps::VirtualAllocator>			mAddedHandles;
			Ps::Array<ShapeHandle, Ps::VirtualAllocator>			mUpdatedHandles;
			Ps::Array<ShapeHandle, Ps::VirtualAllocator>			mRemovedHandles;

			BroadPhase&					mBroadPhase;
			BoundsArray&				mBoundsArray;

			Ps::Array<void*>			mOutOfBoundsObjects;
			Ps::Array<void*>			mOutOfBoundsAggregates;
			Ps::Array<AABBOverlap>		mCreatedOverlaps[ElementType::eCOUNT];
			Ps::Array<AABBOverlap>		mDestroyedOverlaps[ElementType::eCOUNT];

			PxcScratchAllocator*		mScratchAllocator;

			PxBaseTask*					mNarrowPhaseUnblockTask;
			PxU32						mUsedSize;				// highest used value + 1
			bool						mOriginShifted;
			bool						mPersistentStateChanged;

			PxU32						mNbAggregates;
			PxU32						mFirstFreeAggregate;
			Ps::Array<Aggregate*>		mAggregates;		// PT: indexed by AggregateHandle
			Ps::Array<Aggregate*>		mDirtyAggregates;

			PxU32						mTimestamp;

			AggPairMap					mActorAggregatePairs;
			AggPairMap					mAggregateAggregatePairs;

			Ps::Array<ProcessAggPairsBase*> mAggPairTasks;

	#ifdef BP_USE_AGGREGATE_GROUP_TAIL
			// PT: TODO: even in the 3.4 trunk this stuff is a clumsy mess: groups are "BpHandle" suddenly passed
			// to BroadPhaseUpdateData as "ShapeHandle".
			//Free aggregate group ids.
			PxU32								mAggregateGroupTide;
			Ps::Array<Bp::FilterGroup::Enum>	mFreeAggregateGroups;	// PT: TODO: remove this useless array
	#endif
			Ps::HashSet<Pair>			mCreatedPairs;

			PxU64						mContextID;

			Ps::SList					mBpThreadContextPool;

			PX_FORCE_INLINE Aggregate* getAggregateFromHandle(AggregateHandle handle)
			{
				PX_ASSERT(handle<mAggregates.size());
				return mAggregates[handle];
			}

	#ifdef BP_USE_AGGREGATE_GROUP_TAIL
			PX_FORCE_INLINE void		releaseAggregateGroup(const Bp::FilterGroup::Enum group) 
			{
				PX_ASSERT(group != Bp::FilterGroup::eINVALID);
				mFreeAggregateGroups.pushBack(group);
			}

			PX_FORCE_INLINE Bp::FilterGroup::Enum	getAggregateGroup()
			{
				PxU32 id;
				if(mFreeAggregateGroups.size())
					id = mFreeAggregateGroups.popBack();
				else
				{
					id = mAggregateGroupTide--;
		#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
					id<<=2;
					id|=FilterType::AGGREGATE;
		#endif
				}
				const Bp::FilterGroup::Enum group = Bp::FilterGroup::Enum(id);
				PX_ASSERT(group != Bp::FilterGroup::eINVALID);
				return group;
			}
	#endif
			void startAggregateBoundsComputationTasks(PxU32 nbToGo, PxU32 numCpuTasks, Cm::FlushPool& flushPool);
			PersistentActorAggregatePair* createPersistentActorAggregatePair(ShapeHandle volA, ShapeHandle volB);
			PersistentAggregateAggregatePair* createPersistentAggregateAggregatePair(ShapeHandle volA, ShapeHandle volB);
			void updatePairs(PersistentPairs& p, BpCacheData* data = NULL);
			void handleOriginShift();
			public:
			void processBPCreatedPair(const BroadPhasePair& pair);
			void processBPDeletedPair(const BroadPhasePair& pair);
	//		bool checkID(ShapeHandle id);
			friend class PersistentActorAggregatePair;
			friend class PersistentAggregateAggregatePair;
			friend class ProcessSelfCollisionPairsParallel;
			friend class PostBroadPhaseStage2Task;
		};

	} //namespace Bp

	} //namespace physx

#endif //BP_AABBMANAGER_H
