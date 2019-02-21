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

#ifndef SQ_AABB_PRUNER_H
#define SQ_AABB_PRUNER_H

#include "SqPruningPool.h"
#include "SqExtendedBucketPruner.h"
#include "SqAABBTreeUpdateMap.h"
#include "SqAABBTree.h"

namespace physx
{

namespace Sq
{
	// PT: we build the new tree over a number of frames/states, in order to limit perf spikes in 'updatePruningTrees'.
	// The states are as follows:
	//
	// BUILD_NOT_STARTED (1 frame, AABBPruner):
	//
	// This is the initial state, before the new (AABBTree) build even starts. In this frame/state, we perform the AABBPruner-related
	// memory allocations:
	// - the new AABB tree is allocated
	// - the array of cached bounding boxes is allocated and filled
	//
	// BUILD_INIT (1 frame, AABBTree):
	//
	// This is the first frame in which the new tree gets built. It deserves its own special state since various things happen in the
	// first frame, that do no happen in subsequent frames. Basically most initial AABBTree-related allocations happen here (but no
	// build step per se).
	//
	// BUILD_IN_PROGRESS (N frames, AABBTree):
	//
	// This is the core build function, actually building the tree. This should be mostly allocation-free, except here and there when
	// building non-complete trees, and during the last call when the tree is finally built.
	//
	// BUILD_NEW_MAPPING (1 frame, AABBPruner):
	//
	// After the new AABBTree is built, we recreate an AABBTreeUpdateMap for the new tree, and use it to invalidate nodes whose objects
	// have been removed during the build.
	//
	// We need to do that before doing a full refit in the next stage/frame. If we don't do that, the refit code will fetch a wrong box,
	// that may very well belong to an entirely new object.
	//
	// Note that this mapping/update map (mNewTreeMap) is temporary, and only needed for the next stage.
	//
	// BUILD_FULL_REFIT (1 frame, AABBPruner):
	//
	// Once the new update map is available, we fully refit the new tree. AABBs of moved objects get updated. AABBs of removed objects
	// become empty.
	//
	// BUILD_LAST_FRAME (1 frame, AABBPruner):
	//
	// This is an artificial frame used to delay the tree switching code. The switch happens as soon as we reach the BUILD_FINISHED
	// state, but we don't want to execute BUILD_FULL_REFIT and the switch in the same frame. This extra BUILD_LAST_FRAME stage buys
	// us one frame, i.e. we have one frame in which we do BUILD_FULL_REFIT, and in the next frame we'll do both BUILD_LAST_FRAME /
	// BUILD_FINISHED / the switch.
	//
	// BUILD_FINISHED (1 frame, AABBPruner):
	//
	// Several things happen in this 'finalization' frame/stage:
	// - We switch the trees (old one is deleted, cached boxes are deleted, new tree pointer is setup)
	// - A new (final) update map is created (mTreeMap). The map is used to invalidate objects that may have been removed during
	//   the BUILD_NEW_MAPPING and BUILD_FULL_REFIT frames. The nodes containing these removed objects are marked for refit.
	// - Nodes containing objects that have moved during the BUILD_NEW_MAPPING and BUILD_FULL_REFIT frames are marked for refit.
	// - We do a partial refit on the new tree, to take these final changes into account. This small partial refit is usually much
	//   cheaper than the full refit we previously performed here.
	// - We remove old objects from the bucket pruner
	//
	enum BuildStatus
	{
		BUILD_NOT_STARTED,
		BUILD_INIT,
		BUILD_IN_PROGRESS,
		BUILD_NEW_MAPPING,
		BUILD_FULL_REFIT,
		BUILD_LAST_FRAME,
		BUILD_FINISHED,

		BUILD_FORCE_DWORD	= 0xffffffff
	};

	// This class implements the Pruner interface for internal SQ use with some additional specialized functions
	// The underlying data structure is a binary AABB tree
	// AABBPruner supports insertions, removals and updates for dynamic objects
	// The tree is either entirely rebuilt in a single frame (static pruner) or progressively rebuilt over multiple frames (dynamic pruner)
	// The rebuild happens on a copy of the tree
	// the copy is then swapped with current tree at the time commit() is called (only if mBuildState is BUILD_FINISHED),
	// otherwise commit() will perform a refit operation applying any pending changes to the current tree
	// While the tree is being rebuilt a temporary data structure (BucketPruner) is also kept in sync and used to speed up
	// queries on updated objects that are not yet in either old or new tree.
	// The requirements on the order of calls:
	// commit() is required to be called before any queries to apply modifications
	// queries can be issued on multiple threads after commit is called
	// commit, buildStep, add/remove/update have to be called from the same thread or otherwise strictly serialized by external code
	// and cannot be issued while a query is running
	class AABBPruner : public IncrementalPruner
	{
		public:
												AABBPruner(bool incrementalRebuild, PxU64 contextID); // true is equivalent to former dynamic pruner
		virtual									~AABBPruner();

		// Pruner
		virtual			bool					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* userData, PxU32 count, bool hasPruningStructure);
		virtual			void					removeObjects(const PrunerHandle* handles, PxU32 count);
		virtual			void					updateObjectsAfterManualBoundsUpdates(const PrunerHandle* handles, PxU32 count);
		virtual			void					updateObjectsAndInflateBounds(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* newBounds, PxU32 count);
		virtual			void					commit();
		virtual			PxAgain					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&)	const;
		virtual			PxAgain					overlap(const Gu::ShapeData& queryVolume, PrunerCallback&)	const;
		virtual			PxAgain					sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&)	const;
		virtual			const PrunerPayload&	getPayload(PrunerHandle handle)						const	{ return mPool.getPayload(handle);			}
		virtual			const PrunerPayload&	getPayload(PrunerHandle handle, PxBounds3*& bounds)	const	{ return mPool.getPayload(handle, bounds);	}
		virtual			void					preallocate(PxU32 entries)									{ mPool.preallocate(entries);				}
		virtual			void					shiftOrigin(const PxVec3& shift);
		virtual			void					visualize(Cm::RenderOutput& out, PxU32 color) const;		
		virtual			void					merge(const void* mergeParams);		
		//~Pruner
		
		// IncrementalPruner
		virtual			void					purge();		// gets rid of internal accel struct
		virtual			void					setRebuildRateHint(PxU32 nbStepsForRebuild);	// Besides the actual rebuild steps, 3 additional steps are needed.
		virtual			bool					buildStep(bool synchronousCall = true);	// returns true if finished
		virtual			bool					prepareBuild();	// returns true if new tree is needed
		//~IncrementalPruner

		// direct access for test code

		PX_FORCE_INLINE	PxU32					getNbAddedObjects()	const		{ return mBucketPruner.getNbObjects();					}
		PX_FORCE_INLINE	const Sq::AABBTree*		getAABBTree()		const		{ PX_ASSERT(!mUncommittedChanges); return mAABBTree;	}
		PX_FORCE_INLINE	Sq::AABBTree*			getAABBTree()					{ PX_ASSERT(!mUncommittedChanges); return mAABBTree;	}
		PX_FORCE_INLINE	void					setAABBTree(Sq::AABBTree* tree)	{ mAABBTree = tree; }
		PX_FORCE_INLINE	const Sq::AABBTree*		hasAABBTree()		const		{ return mAABBTree;	}
		PX_FORCE_INLINE	BuildStatus				getBuildStatus()	const		{ return mProgress;	}
				
		// local functions
//		private:
						Sq::AABBTree*			mAABBTree; // current active tree
						Gu::AABBTreeBuildParams	mBuilder; // this class deals with the details of the actual tree building
						Gu::BuildStats			mBuildStats;

		// tree with build in progress, assigned to mAABBTree in commit, when mProgress is BUILD_FINISHED
		// created in buildStep(), BUILD_NOT_STARTED
		// This is non-null when there is a tree rebuild going on in progress
		// and thus also indicates that we have to start saving the fixups
						Sq::AABBTree*			mNewTree;

		// during rebuild the pool might change so we need a copy of boxes for the tree build
						PxBounds3*				mCachedBoxes;
						PxU32					mNbCachedBoxes;

		// incremented in commit(), serves as a progress counter for rebuild
						PxU32					mNbCalls;

		// PT: incremented each time we start building a new tree (i.e. effectively identifies a given tree)
		// Timestamp is passed to bucket pruner to mark objects added there, linking them to a specific tree.
		// When switching to the new tree, timestamp is used to remove old objects (now in the new tree) from
		// the bucket pruner.
						PxU32					mTimeStamp;

		// this pruner is used for queries on objects that are not in the current tree yet
		// includes both the objects in the tree being rebuilt and all the objects added later
						ExtendedBucketPruner	mBucketPruner;

						BuildStatus				mProgress;		// current state of second tree build progress

		// Fraction (as in 1/Nth) of the total number of primitives
		// that should be processed per step by the AABB builder
		// so if this value is 1, all primitives will be rebuilt, 2 => 1/2 of primitives per step etc.
		// see also mNbCalls, mNbCalls varies from 0 to mRebuildRateHint-1
						PxU32					mRebuildRateHint;

		// Estimate for how much work has to be done to rebuild the tree.
						PxU32					mTotalWorkUnits;

		// Term to correct the work unit estimate if the rebuild rate is not matched
						PxI32					mAdaptiveRebuildTerm;

						PruningPool				mPool; // Pool of AABBs

		// maps pruning pool indices to aabb tree indices
		// maps to INVALID_NODE_ID if the pool entry was removed or "pool index is outside input domain"
		// The map is the inverse of the tree mapping: (node[map[poolID]].primitive == poolID)
		// So:
		// treeNodeIndex = mTreeMap.operator[](poolIndex)
		// aabbTree->treeNodes[treeNodeIndex].primitives[0] == poolIndex
						AABBTreeUpdateMap		mTreeMap;
		// Temporary update map, see BuildStatus notes above for details
						AABBTreeUpdateMap		mNewTreeMap;

		// This is only set once in the constructor and is equivalent to isDynamicTree
		// if it set to false then a 1-shot rebuild is performed in commit()
		// bucket pruner is only used with incremental rebuild
						bool					mIncrementalRebuild;

		// A rebuild can be triggered even when the Pruner is not dirty
		// mUncommittedChanges is set to true in add, remove, update and buildStep
		// mUncommittedChanges is set to false in commit
		// mUncommittedChanges has to be false (commit() has to be called) in order to run a query as defined by the
		// mUncommittedChanges is not set to true in add, when pruning structure is provided. Scene query shapes
		// are merged to current AABB tree directly
		// Pruner higher level API
						bool					mUncommittedChanges;

		// A new AABB tree is built if an object was added, removed or updated
		// Changing objects during a build will trigger another rebuild right afterwards
		// this is set to true if a new tree has to be created again after the current rebuild is done
						bool					mNeedsNewTree;

		// This struct is used to record modifications made to the pruner state
		// while a tree is building in the background
		// this is so we can apply the modifications to the tree at the time of completion
		// the recorded fixup information is: removedIndex (in ::remove()) and 
		// lastIndexMoved which is the last index in the pruner array
		// (since the way we remove from PruningPool is by swapping last into removed slot,
		// we need to apply a fixup so that it syncs up that operation in the new tree)
						struct NewTreeFixup
						{
							PX_FORCE_INLINE NewTreeFixup(PxU32 removedIndex_, PxU32 relocatedLastIndex_)
								: removedIndex(removedIndex_), relocatedLastIndex(relocatedLastIndex_) {}
							PxU32 removedIndex;
							PxU32 relocatedLastIndex;
						};
						Ps::Array<NewTreeFixup>	mNewTreeFixups;

						Ps::Array<PoolIndex>	mToRefit;

						PxU64					mContextID;

		// Internal methods
						bool					fullRebuildAABBTree(); // full rebuild function, used with static pruner mode
						void					release();
						void					refitUpdatedAndRemoved();
						void					updateBucketPruner();
						PxBounds3				getAABB(PrunerHandle h);
	};

} // namespace Sq

}

#endif // SQ_AABB_PRUNER_H
