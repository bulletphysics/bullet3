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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "common/PxProfileZone.h"
#include "PsIntrinsics.h"
#include "PsUserAllocated.h"
#include "PsBitUtils.h"
#include "PsFoundation.h"
#include "SqAABBPruner.h"
#include "SqAABBTree.h"
#include "SqPrunerMergeData.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuAABBTreeQuery.h"
#include "GuBounds.h"

using namespace physx;
using namespace Gu;
using namespace Sq;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IncrementalPruner* physx::Sq::createAABBPruner(bool incrementalRebuild)
{
	return PX_NEW(Sq::AABBPruner)(incrementalRebuild, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: currently limited to 15 max
#define NB_OBJECTS_PER_NODE	4

AABBPruner::AABBPruner(bool incrementalRebuild, PxU64 contextID) :
	mAABBTree			(NULL),
	mNewTree			(NULL),
	mCachedBoxes		(NULL),
	mNbCachedBoxes		(0),
	mNbCalls			(0),
	mTimeStamp			(0),
	mBucketPruner		(&mPool),
	mProgress			(BUILD_NOT_STARTED),
	mRebuildRateHint	(100),
	mAdaptiveRebuildTerm(0),
	mIncrementalRebuild	(incrementalRebuild),
	mUncommittedChanges	(false),
	mNeedsNewTree		(false),
	mNewTreeFixups		(PX_DEBUG_EXP("AABBPruner::mNewTreeFixups")),
	mContextID			(contextID)
{
}

AABBPruner::~AABBPruner()
{
	release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Add, Remove, Update methods
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool AABBPruner::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* payload, PxU32 count, bool hasPruningStructure)
{
	PX_PROFILE_ZONE("SceneQuery.prunerAddObjects", mContextID);

	if(!count)
		return true;

	// no need to do refitMarked for added objects since they are not in the tree

	// if we have provided pruning structure, we will merge it, the changes will be applied after the objects has been addded
	if(!hasPruningStructure || !mAABBTree)
		mUncommittedChanges = true;

	// PT: TODO: 'addObjects' for bucket pruner too. Not urgent since we always call the function with count=1 at the moment
	const PxU32 valid = mPool.addObjects(results, bounds, payload, count);

	// Bucket pruner is only used while the dynamic pruner is rebuilding
	// For the static pruner a full rebuild will happen in commit() every time we modify something, this is not true if
	// pruning structure was provided. The objects tree will be merged directly into the static tree. No rebuild will be triggered.
	if(mIncrementalRebuild && mAABBTree)
	{
		mNeedsNewTree = true; // each add forces a tree rebuild

		// if a pruner structure is provided, we dont move the new objects into bucket pruner
		// the pruning structure will be merged into the bucket pruner
		if(!hasPruningStructure)
		{
			for(PxU32 i=0;i<valid;i++)
			{
#if USE_INCREMENTAL_PRUNER
				const PrunerHandle& handle = results[i];
				const PoolIndex poolIndex = mPool.getIndex(handle);
				mBucketPruner.addObject(payload[i], bounds[i], mTimeStamp, poolIndex);
#else
				mBucketPruner.addObject(payload[i], bounds[i], mTimeStamp, INVALID_NODE_ID);
#endif
			}
		}
	}
	return valid==count;
}

void AABBPruner::updateObjectsAfterManualBoundsUpdates(const PrunerHandle* handles, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mContextID);

	if(!count)
		return;

	mUncommittedChanges = true;

	if(mIncrementalRebuild && mAABBTree) 
	{
		mNeedsNewTree = true; // each update forces a tree rebuild
		const PxBounds3* newBounds = mPool.getCurrentWorldBoxes();
		PrunerPayload* payloads = mPool.getObjects();
		for(PxU32 i=0; i<count; i++)
		{
			const PoolIndex poolIndex = mPool.getIndex(handles[i]);
			const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
			if(treeNodeIndex!=INVALID_NODE_ID) // this means it's in the current tree still and hasn't been removed
				mAABBTree->markNodeForRefit(treeNodeIndex);
			else // otherwise it means it should be in the bucket pruner
			{
				bool found = mBucketPruner.updateObject(newBounds[poolIndex], payloads[poolIndex], poolIndex);
				PX_UNUSED(found); PX_ASSERT(found);
			}

			if(mProgress==BUILD_NEW_MAPPING || mProgress==BUILD_FULL_REFIT)
				mToRefit.pushBack(poolIndex);
		}
	}
}

void AABBPruner::updateObjectsAndInflateBounds(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* newBounds, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mContextID);

	if(!count)
		return;

	mUncommittedChanges = true;

	mPool.updateObjectsAndInflateBounds(handles, indices, newBounds, count);

	if(mIncrementalRebuild && mAABBTree)
	{
		mNeedsNewTree = true; // each update forces a tree rebuild
		PrunerPayload* payloads = mPool.getObjects();
		for(PxU32 i=0; i<count; i++)
		{
			const PoolIndex poolIndex = mPool.getIndex(handles[i]);
			const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
			if(treeNodeIndex != INVALID_NODE_ID) // this means it's in the current tree still and hasn't been removed
				mAABBTree->markNodeForRefit(treeNodeIndex);
			else // otherwise it means it should be in the bucket pruner
			{
				// PT: TODO: is this line correct?
//				bool found = mBucketPruner.updateObject(newBounds[indices[i]], mPool.getPayload(handles[i]));
				PX_ASSERT(&payloads[poolIndex]==&mPool.getPayload(handles[i]));
				// PT: TODO: don't we need to read the pool's array here, to pass the inflated bounds?
				bool found = mBucketPruner.updateObject(newBounds[indices[i]], payloads[poolIndex], poolIndex);
				PX_UNUSED(found); PX_ASSERT(found);
			}

			if(mProgress == BUILD_NEW_MAPPING || mProgress == BUILD_FULL_REFIT)
				mToRefit.pushBack(poolIndex);
		}
	}
}

void AABBPruner::removeObjects(const PrunerHandle* handles, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerRemoveObjects", mContextID);

	if(!count)
		return;

	mUncommittedChanges = true;

	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];
		// copy the payload before removing it since we need to know the payload to remove it from the bucket pruner
		const PrunerPayload removedPayload = mPool.getPayload(h);
		const PoolIndex poolIndex = mPool.getIndex(h); // save the pool index for removed object
		const PoolIndex poolRelocatedLastIndex = mPool.removeObject(h); // save the lastIndex returned by removeObject
		if(mIncrementalRebuild && mAABBTree)
		{
			mNeedsNewTree = true;

			const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex]; // already removed from pool but still in tree map
			const PrunerPayload swappedPayload = mPool.getObjects()[poolIndex];
			if(treeNodeIndex!=INVALID_NODE_ID) // can be invalid if removed
			{
				mAABBTree->markNodeForRefit(treeNodeIndex); // mark the spot as blank
				mBucketPruner.swapIndex(poolIndex, swappedPayload, poolRelocatedLastIndex);	// if swapped index is in bucket pruner
			}
			else
			{
				PX_ASSERT(treeNodeIndex==INVALID_PRUNERHANDLE);
				PxU32 timeStamp;				
				bool status = mBucketPruner.removeObject(removedPayload, poolIndex, swappedPayload, poolRelocatedLastIndex, timeStamp);
				PX_ASSERT(status);
				PX_UNUSED(status);
			}

			mTreeMap.invalidate(poolIndex, poolRelocatedLastIndex, *mAABBTree);
			if(mNewTree)
				mNewTreeFixups.pushBack(NewTreeFixup(poolIndex, poolRelocatedLastIndex));
		}
	}

	if (mPool.getNbActiveObjects()==0)
	{
		// this is just to make sure we release all the internal data once all the objects are out of the pruner
		// since this is the only place we know that and we don't want to keep memory reserved
		release();

		// Pruner API requires a commit before the next query, even if we ended up removing the entire tree here. This
		// forces that to happen.
		mUncommittedChanges = true;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Query Implementation
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxAgain AABBPruner::overlap(const ShapeData& queryVolume, PrunerCallback& pcb) const
{
	PX_ASSERT(!mUncommittedChanges);

	PxAgain again = true;

	if(mAABBTree)
	{
		switch(queryVolume.getType())
		{
		case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
					again = AABBTreeOverlap<Gu::OBBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
				}
				else
				{
					const Gu::AABBAABBTest test(queryVolume.getPrunerInflatedWorldAABB());
					again = AABBTreeOverlap<Gu::AABBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
				}
			}
			break;
		case PxGeometryType::eCAPSULE:
			{
				const Gu::Capsule& capsule = queryVolume.getGuCapsule();
				const Gu::CapsuleAABBTest test(	capsule.p1, queryVolume.getPrunerWorldRot33().column0,
												queryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));
				again = AABBTreeOverlap<Gu::CapsuleAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
			}
			break;
		case PxGeometryType::eSPHERE:
			{
				const Gu::Sphere& sphere = queryVolume.getGuSphere();
				Gu::SphereAABBTest test(sphere.center, sphere.radius);
				again = AABBTreeOverlap<Gu::SphereAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
			}
			break;
		case PxGeometryType::eCONVEXMESH:
			{
				const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
				again = AABBTreeOverlap<Gu::OBBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);			
			}
			break;
		case PxGeometryType::ePLANE:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
		}
	}

	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.overlap(queryVolume, pcb);

	return again;
}

PxAgain AABBPruner::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PX_ASSERT(!mUncommittedChanges);

	PxAgain again = true;

	if(mAABBTree)
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		const PxVec3 extents = aabb.getExtents();
		again = AABBTreeRaycast<true, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, aabb.getCenter(), unitDir, inOutDistance, extents, pcb);
	}

	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.sweep(queryVolume, unitDir, inOutDistance, pcb);

	return again;
}

PxAgain AABBPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PX_ASSERT(!mUncommittedChanges);

	PxAgain again = true;

	if(mAABBTree)
		again = AABBTreeRaycast<false, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
		
	if(again && mIncrementalRebuild && mBucketPruner.getNbObjects())
		again = mBucketPruner.raycast(origin, unitDir, inOutDistance, pcb);

	return again;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Other methods of Pruner Interface
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This isn't part of the pruner virtual interface, but it is part of the public interface
// of AABBPruner - it gets called by SqManager to force a rebuild, and requires a commit() before 
// queries can take place

void AABBPruner::purge()
{
	release();
	mUncommittedChanges = true; // this ensures a commit() must happen before any query
} 

void AABBPruner::setRebuildRateHint(PxU32 nbStepsForRebuild) 
{ 
	PX_ASSERT(nbStepsForRebuild > 3);
	mRebuildRateHint = (nbStepsForRebuild-3); // looks like a magic number to account for the rebuild pipeline latency
	mAdaptiveRebuildTerm = 0; 
}

// Commit either performs a refit if background rebuild is not yet finished
// or swaps the current tree for the second tree rebuilt in the background
void AABBPruner::commit()
{
	PX_PROFILE_ZONE("SceneQuery.prunerCommit", mContextID);

	if(!mUncommittedChanges && (mProgress != BUILD_FINISHED))
		// Q: seems like this is both for refit and finalization so is this is correct?
		// i.e. in a situation when we started rebuilding a tree and didn't add anything since
		// who is going to set mUncommittedChanges to true?
		// A: it's set in buildStep at final stage, so that finalization is forced.
		// Seems a bit difficult to follow and verify correctness.
		return;

	mUncommittedChanges = false;

	if(!mAABBTree || !mIncrementalRebuild)
	{
#if PX_CHECKED
		if(!mIncrementalRebuild && mAABBTree)
			Ps::getFoundation().error(PxErrorCode::ePERF_WARNING, __FILE__, __LINE__, "SceneQuery static AABB Tree rebuilt, because a shape attached to a static actor was added, removed or moved, and PxSceneDesc::staticStructure is set to eSTATIC_AABB_TREE.");
#endif
		fullRebuildAABBTree();
		return;
	}

	// Note: it is not safe to call AABBPruner::build() here
	// because the first thread will perform one step of the incremental update,
	// continue raycasting, while the second thread performs the next step in
	// the incremental update

	// Calling Refit() below is safe. It will call 
	// StaticPruner::build() when necessary. Both will early
	// exit if the tree is already up to date, if it is not already, then we 
	// must be the first thread performing raycasts on a dirty tree and other 
	// scene query threads will be locked out by the write lock in 
	// SceneQueryManager::flushUpdates()


	if (mProgress != BUILD_FINISHED)	
	{
		// Calling refit because the second tree is not ready to be swapped in (mProgress != BUILD_FINISHED)
		// Generally speaking as long as things keep moving the second build will never catch up with true state
		refitUpdatedAndRemoved();
	}
	else
	{
		PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFinalize", mContextID);

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeSwitch", mContextID);

			PX_DELETE(mAABBTree); // delete the old tree
			PX_FREE_AND_RESET(mCachedBoxes);
			mProgress = BUILD_NOT_STARTED; // reset the build state to initial

			// Adjust adaptive term to get closer to specified rebuild rate.
			// perform an even division correction to make sure the rebuild rate adds up
			if (mNbCalls > mRebuildRateHint)
				mAdaptiveRebuildTerm++;
			else if (mNbCalls < mRebuildRateHint)
				mAdaptiveRebuildTerm--;

			// Switch trees
#if PX_DEBUG
			mNewTree->validate();
#endif
			mAABBTree = mNewTree; // set current tree to progressively rebuilt tree
			mNewTree = NULL; // clear out the progressively rebuild tree pointer
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeMapping", mContextID);

			// rebuild the tree map to match the current (newly built) tree
			mTreeMap.initMap(PxMax(mPool.getNbActiveObjects(), mNbCachedBoxes), *mAABBTree);

			// The new mapping has been computed using only indices stored in the new tree. Those indices map the pruning pool
			// we had when starting to build the tree. We need to re-apply recorded moves to fix the tree that finished rebuilding.
			// AP: the problem here is while we are rebuilding the tree there are ongoing modifications to the current tree
			// but the background build has a cached copy of all the AABBs at the time it was started
			// (and will produce indices referencing those)
			// Things that can happen in the meantime: update, remove, add, commit
			for(NewTreeFixup* r = mNewTreeFixups.begin(); r < mNewTreeFixups.end(); r++)
			{
				// PT: we're not doing a full refit after this point anymore, so the remaining deleted objects must be manually marked for
				// refit (otherwise their AABB in the tree would remain valid, leading to crashes when the corresponding index is 0xffffffff).
				// We must do this before invalidating the corresponding tree nodes in the map, obviously (otherwise we'd be reading node
				// indices that we already invalidated).
				const PoolIndex poolIndex = r->removedIndex;
				const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
				if(treeNodeIndex!=INVALID_NODE_ID)
					mAABBTree->markNodeForRefit(treeNodeIndex);

				mTreeMap.invalidate(r->removedIndex, r->relocatedLastIndex, *mAABBTree);
			}
			mNewTreeFixups.clear(); // clear out the fixups since we just applied them all
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFinalRefit", mContextID);

			const PxU32 size = mToRefit.size();
			for(PxU32 i=0;i<size;i++)
			{
				const PoolIndex poolIndex = mToRefit[i];
				const TreeNodeIndex treeNodeIndex = mTreeMap[poolIndex];
				if(treeNodeIndex!=INVALID_NODE_ID)
					mAABBTree->markNodeForRefit(treeNodeIndex);
			}
			mToRefit.clear();
			refitUpdatedAndRemoved();
		}

		{
			PX_PROFILE_ZONE("SceneQuery.prunerNewTreeRemoveObjects", mContextID);

			PxU32 nbRemovedPairs = mBucketPruner.removeMarkedObjects(mTimeStamp-1);
			PX_UNUSED(nbRemovedPairs);

			mNeedsNewTree = mBucketPruner.getNbObjects()>0;
		}
	}

	updateBucketPruner();
}


void AABBPruner::shiftOrigin(const PxVec3& shift)
{
	mPool.shiftOrigin(shift);

	if(mAABBTree)
		mAABBTree->shiftOrigin(shift);

	if(mIncrementalRebuild)
		mBucketPruner.shiftOrigin(shift);

	if(mNewTree)
		mNewTree->shiftOrigin(shift);
}

#include "CmRenderOutput.h"
void AABBPruner::visualize(Cm::RenderOutput& out, PxU32 color) const
{
	// getAABBTree() asserts when pruner is dirty. NpScene::visualization() does not enforce flushUpdate. see DE7834
	const AABBTree* tree = mAABBTree;

	if(tree && tree->getNodes())
	{
		struct Local
		{
			static void _Draw(const AABBTreeRuntimeNode* root, const AABBTreeRuntimeNode* node, Cm::RenderOutput& out_)
			{
				out_ << Cm::DebugBox(node->mBV, true);
				if (node->isLeaf())
					return;
				_Draw(root, node->getPos(root), out_);
				_Draw(root, node->getNeg(root), out_);
			}
		};
		out << PxTransform(PxIdentity);
		out << color;
		Local::_Draw(tree->getNodes(), tree->getNodes(), out);
	}

	// Render added objects not yet in the tree
	out << PxTransform(PxIdentity);
	out << PxU32(PxDebugColor::eARGB_WHITE);

	if(mIncrementalRebuild && mBucketPruner.getNbObjects())
		mBucketPruner.visualize(out, color);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Internal methods
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool AABBPruner::buildStep(bool synchronousCall)
{
	PX_PROFILE_ZONE("SceneQuery.prunerBuildStep", mContextID);

	PX_ASSERT(mIncrementalRebuild);
	if(mNeedsNewTree)
	{
		if(mProgress==BUILD_NOT_STARTED)
		{
			if(!synchronousCall || !prepareBuild())
				return false;
		}
		else if(mProgress==BUILD_INIT)
		{
			mNewTree->progressiveBuild(mBuilder, mBuildStats, 0, 0);
			mProgress = BUILD_IN_PROGRESS;
			mNbCalls = 0;

			// Use a heuristic to estimate the number of work units needed for rebuilding the tree.
			// The general idea is to use the number of work units of the previous tree to build the new tree.
			// This works fine as long as the number of leaves remains more or less the same for the old and the
			// new tree. If that is not the case, this estimate can be way off and the work units per step will
			// be either much too small or too large. Hence, in that case we will try to estimate the number of work
			// units based on the number of leaves of the new tree as follows:
 			//
			// - Assume new tree with n leaves is perfectly-balanced
			// - Compute the depth of perfectly-balanced tree with n leaves
			// - Estimate number of working units for the new tree

			const PxU32 depth = Ps::ilog2(mBuilder.mNbPrimitives);	// Note: This is the depth without counting the leaf layer
			const PxU32 estimatedNbWorkUnits = depth * mBuilder.mNbPrimitives;	// Estimated number of work units for new tree
			const PxU32 estimatedNbWorkUnitsOld = mAABBTree ? mAABBTree->getTotalPrims() : 0;
			if ((estimatedNbWorkUnits <= (estimatedNbWorkUnitsOld << 1)) && (estimatedNbWorkUnits >= (estimatedNbWorkUnitsOld >> 1)))
				// The two estimates do not differ by more than a factor 2
				mTotalWorkUnits = estimatedNbWorkUnitsOld;
 			else
			{
 				mAdaptiveRebuildTerm = 0;
				mTotalWorkUnits = estimatedNbWorkUnits;
 			}
 
 			const PxI32 totalWorkUnits = PxI32(mTotalWorkUnits + (mAdaptiveRebuildTerm * mBuilder.mNbPrimitives));
 			mTotalWorkUnits = PxU32(PxMax(totalWorkUnits, 0));
		}
		else if(mProgress==BUILD_IN_PROGRESS)
		{
			mNbCalls++;
			const PxU32 Limit = 1 + (mTotalWorkUnits / mRebuildRateHint);
			// looks like progressiveRebuild returns 0 when finished
			if (!mNewTree->progressiveBuild(mBuilder, mBuildStats, 1, Limit))
			{
				// Done
				mProgress = BUILD_NEW_MAPPING;
#if PX_DEBUG
				mNewTree->validate();
#endif
			}
		}
		else if(mProgress==BUILD_NEW_MAPPING)
		{
			mNbCalls++;
			mProgress = BUILD_FULL_REFIT;

			// PT: we can't call fullRefit without creating the new mapping first: the refit function will fetch boxes from
			// the pool using "primitive indices" captured in the tree. But some of these indices may have been invalidated
			// if objects got removed while the tree was built. So we need to invalidate the corresponding nodes before refit,
			// that way the #prims will be zero and the code won't fetch a wrong box (which may now below to a different object).
			{
				PX_PROFILE_ZONE("SceneQuery.prunerNewTreeMapping", mContextID);

				if(mNewTreeFixups.size())
				{
					mNewTreeMap.initMap(PxMax(mPool.getNbActiveObjects(), mNbCachedBoxes), *mNewTree);

					// The new mapping has been computed using only indices stored in the new tree. Those indices map the pruning pool
					// we had when starting to build the tree. We need to re-apply recorded moves to fix the tree.
					for(NewTreeFixup* r = mNewTreeFixups.begin(); r < mNewTreeFixups.end(); r++)
						mNewTreeMap.invalidate(r->removedIndex, r->relocatedLastIndex, *mNewTree);

					mNewTreeFixups.clear();
#if PX_DEBUG
					mNewTree->validate();
#endif
				}
			}
		}
		else if(mProgress==BUILD_FULL_REFIT)
		{
			mNbCalls++;
			mProgress = BUILD_LAST_FRAME;

			{
				PX_PROFILE_ZONE("SceneQuery.prunerNewTreeFullRefit", mContextID);

				// We need to refit the new tree because objects may have moved while we were building it.
				mNewTree->fullRefit(mPool.getCurrentWorldBoxes());
			}
		}
		else if(mProgress==BUILD_LAST_FRAME)
		{
			mProgress = BUILD_FINISHED;
		}

		// This is required to be set because commit handles both refit and a portion of build finalization (why?)
		// This is overly conservative also only necessary in case there were no updates at all to the tree since the last tree swap
		// It also overly conservative in a sense that it could be set only if mProgress was just set to BUILD_FINISHED
		// If run asynchronously from a different thread, we touched just the new AABB build phase, we should not mark the main tree as dirty
		if(synchronousCall)
			mUncommittedChanges = true;

		return mProgress==BUILD_FINISHED;
	}

	return false;
}

bool AABBPruner::prepareBuild()
{
	PX_PROFILE_ZONE("SceneQuery.prepareBuild", mContextID);

	PX_ASSERT(mIncrementalRebuild);
	if(mNeedsNewTree)
	{
		if(mProgress==BUILD_NOT_STARTED)
		{
			const PxU32 nbObjects = mPool.getNbActiveObjects();
			if(!nbObjects)
				return false;

			PX_DELETE(mNewTree);
			mNewTree = PX_NEW(AABBTree);

			mNbCachedBoxes = nbObjects;
			// PT: we always allocate one extra box, to make sure we can safely use V4 loads on the array
			mCachedBoxes = reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(nbObjects+1), "PxBound3"));

			PxMemCopy(mCachedBoxes, mPool.getCurrentWorldBoxes(), nbObjects*sizeof(PxBounds3));

			// PT: objects currently in the bucket pruner will be in the new tree. They are marked with the
			// current timestamp (mTimeStamp). However more objects can get added while we compute the new tree,
			// and those ones will not be part of it. These new objects will be marked with the new timestamp
			// value (mTimeStamp+1), and we can use these different values to remove the proper objects from
			// the bucket pruner (when switching to the new tree).
			mTimeStamp++;
#if USE_INCREMENTAL_PRUNER
			// notify the incremental pruner to swap trees
			mBucketPruner.timeStampChange();
#endif
			mBuilder.reset();
			mBuilder.mNbPrimitives	= mNbCachedBoxes;
			mBuilder.mAABBArray		= mCachedBoxes;
			mBuilder.mLimit			= NB_OBJECTS_PER_NODE;

			mBuildStats.reset();

			// start recording modifications to the tree made during rebuild to reapply (fix the new tree) eventually
			PX_ASSERT(mNewTreeFixups.size()==0);

			mProgress = BUILD_INIT;
		}
	}
	else
		return false;

	return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds an AABB-tree for objects in the pruning pool.
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBPruner::fullRebuildAABBTree()
{
	PX_PROFILE_ZONE("SceneQuery.prunerFullRebuildAABBTree", mContextID);

	// Release possibly already existing tree
	PX_DELETE_AND_RESET(mAABBTree);

	// Don't bother building an AABB-tree if there isn't a single static object
	const PxU32 nbObjects = mPool.getNbActiveObjects();
	if(!nbObjects)
		return true;

	bool Status;
	{
		// Create a new tree
		mAABBTree = PX_NEW(AABBTree);

		AABBTreeBuildParams TB;
		TB.mNbPrimitives	= nbObjects;
		TB.mAABBArray		= mPool.getCurrentWorldBoxes();
		TB.mLimit			= NB_OBJECTS_PER_NODE;
		Status = mAABBTree->build(TB);
	}

	// No need for the tree map for static pruner
	if(mIncrementalRebuild)
		mTreeMap.initMap(PxMax(nbObjects,mNbCachedBoxes),*mAABBTree);

	return Status;
}

// called in the end of commit(), but only if mIncrementalRebuild is true
void AABBPruner::updateBucketPruner()
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateBucketPruner", mContextID);

	PX_ASSERT(mIncrementalRebuild);
	mBucketPruner.build();
}

PxBounds3 AABBPruner::getAABB(PrunerHandle handle)
{
	return mPool.getWorldAABB(handle);
}

void AABBPruner::release() // this can be called from purge()
{
	mBucketPruner.release();

	mTimeStamp = 0;

	mTreeMap.release();
	mNewTreeMap.release();

	PX_FREE_AND_RESET(mCachedBoxes);
	mBuilder.reset();
	PX_DELETE_AND_RESET(mNewTree);
	PX_DELETE_AND_RESET(mAABBTree);

	mNbCachedBoxes = 0;
	mProgress = BUILD_NOT_STARTED;
	mNewTreeFixups.clear();
	mUncommittedChanges = false;
}

// Refit current tree
void AABBPruner::refitUpdatedAndRemoved()
{
	PX_PROFILE_ZONE("SceneQuery.prunerRefitUpdatedAndRemoved", mContextID);

	PX_ASSERT(mIncrementalRebuild);
	AABBTree* tree = getAABBTree();
	if(!tree)
		return;

#if PX_DEBUG
	tree->validate();
#endif

	//### missing a way to skip work if not needed

	const PxU32 nbObjects = mPool.getNbActiveObjects();
	// At this point there still can be objects in the tree that are blanked out so it's an optimization shortcut (not required)
	if(!nbObjects)
		return;

	mBucketPruner.refitMarkedNodes(mPool.getCurrentWorldBoxes());
	tree->refitMarkedNodes(mPool.getCurrentWorldBoxes());
}

void AABBPruner::merge(const void* mergeParams)
{
	const AABBPrunerMergeData& pruningStructure = *reinterpret_cast<const AABBPrunerMergeData*> (mergeParams);

	if(mAABBTree)
	{
		// index in pruning pool, where new objects were added
		const PxU32 pruningPoolIndex = mPool.getNbActiveObjects() - pruningStructure.mNbObjects;

		// create tree from given nodes and indices
		AABBTreeMergeData aabbTreeMergeParams(pruningStructure.mNbNodes, pruningStructure.mAABBTreeNodes,
			pruningStructure.mNbObjects, pruningStructure.mAABBTreeIndices, pruningPoolIndex);

		if (!mIncrementalRebuild)
		{
			// merge tree directly
			mAABBTree->mergeTree(aabbTreeMergeParams);		
		}
		else
		{
			mBucketPruner.addTree(aabbTreeMergeParams, mTimeStamp);
		}
	}
}
