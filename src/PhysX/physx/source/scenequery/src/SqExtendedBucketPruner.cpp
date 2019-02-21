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


#include "SqExtendedBucketPruner.h"
#include "SqAABBTree.h"
#include "SqPrunerMergeData.h"
#include "GuAABBTreeQuery.h"
#include "GuBounds.h"
#include "CmBitMap.h"

using namespace physx;
using namespace Sq;
using namespace Gu;
using namespace Ps;

#define NB_OBJECTS_PER_NODE	4

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor, preallocate trees, bounds
ExtendedBucketPruner::ExtendedBucketPruner(const PruningPool* pool)
	: 
#if USE_INCREMENTAL_PRUNER
	mPrunerCore(pool),
#else
	mPrunerCore(false), 
#endif
	mPruningPool(pool), mMainTree(NULL), mBounds(NULL), mMergedTrees(NULL), 
	mCurrentTreeIndex(0), mTreesDirty(false)
{
	// preallocated size for bounds, trees
	mCurrentTreeCapacity = 32;

	mBounds = reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(mCurrentTreeCapacity + 1), "Bounds"));			
	mMergedTrees = reinterpret_cast<MergedTree*>(PX_ALLOC(sizeof(MergedTree)*mCurrentTreeCapacity, "AABB trees"));			
	mExtendedBucketPrunerMap.reserve(mCurrentTreeCapacity);

	// create empty main tree
	mMainTree = PX_NEW(AABBTree);

	// create empty merge trees
	for (PxU32 i = 0; i < mCurrentTreeCapacity; i++)
	{
		mMergedTrees[i].mTimeStamp = 0;
		mMergedTrees[i].mTree = PX_NEW(AABBTree);
	}
}

//////////////////////////////////////////////////////////////////////////

ExtendedBucketPruner::~ExtendedBucketPruner()
{
	// release main tree
	if (mMainTree)
	{		
		PX_DELETE_AND_RESET(mMainTree);
	}

	// release merged trees
	for (PxU32 i = 0; i < mCurrentTreeCapacity; i++)
	{
		AABBTree* aabbTree = mMergedTrees[i].mTree;		
		PX_DELETE(aabbTree);
	}

	PX_FREE(mBounds);
	PX_FREE(mMergedTrees);
}

//////////////////////////////////////////////////////////////////////////
// release all objects in bucket pruner
void ExtendedBucketPruner::release()
{	
	// release core bucket pruner
	mPrunerCore.release();

	mMainTreeUpdateMap.release();
	mMergeTreeUpdateMap.release();
	
	// release all objecs from the map
	mExtendedBucketPrunerMap.clear();

	// release all merged trees
	for (PxU32 i = 0; i < mCurrentTreeCapacity; i++)
	{
		mMergedTrees[i].mTimeStamp = 0;
		mMergedTrees[i].mTree->release();
	}

	// reset current tree index
	mCurrentTreeIndex = 0;
}

//////////////////////////////////////////////////////////////////////////
// Add a tree from a pruning structure 
// 1. get new tree index
// 2. initialize merged tree, bounds
// 3. create update map for the merged tree
// 4. build new tree of trees from given trees bounds
// 5. add new objects into extended bucket pruner map
// 6. shift indices in the merged tree
void ExtendedBucketPruner::addTree(const AABBTreeMergeData& mergeData, PxU32 timeStamp)
{
	// check if we have to resize
	if(mCurrentTreeIndex == mCurrentTreeCapacity)
	{
		resize(mCurrentTreeCapacity*2);
	}

	// get current merge tree index
	const PxU32 mergeTreeIndex = mCurrentTreeIndex++;	

	// get payloads pointers - the pointers start at mIndicesOffset, thats where all 
	// objects were added before merge was called
	const PrunerPayload* payloads = &mPruningPool->getObjects()[mergeData.mIndicesOffset];

	// setup merged tree with the merge data and timestamp
	mMergedTrees[mergeTreeIndex].mTimeStamp = timeStamp;
	AABBTree& mergedTree = *mMergedTrees[mergeTreeIndex].mTree;	
	mergedTree.initTree(mergeData);
	// set bounds
	mBounds[mergeTreeIndex] = mergeData.getRootNode().mBV;
	
	// update temporally update map for the current merge tree, map is used to setup the base extended bucket pruner map 
	mMergeTreeUpdateMap.initMap(mergeData.mNbIndices, mergedTree);

	// create new base tree of trees
	buildMainAABBTree();

	// Add each object into extended bucket pruner hash map
	for (PxU32 i = 0; i < mergeData.mNbIndices; i++)
	{
		ExtendedBucketPrunerData mapData;
		mapData.mMergeIndex = mergeTreeIndex;		
		mapData.mTimeStamp = timeStamp;		
		PX_ASSERT(mMergeTreeUpdateMap[i] < mergedTree.getNbNodes());
		// get node information from the merge tree update map
		mapData.mSubTreeNode = mMergeTreeUpdateMap[i];
		mExtendedBucketPrunerMap.insert(payloads[i], mapData);		
	}
	// merged tree indices needs to be shifted now, we cannot shift it in init - the update map 
	// could not be constructed otherwise, as the indices wont start from 0. The indices 
	// needs to be shifted by offset from the pruning pool, where the new objects were added into the pruning pool.
	mergedTree.shiftIndices(mergeData.mIndicesOffset);

#if PX_DEBUG
	checkValidity();
#endif // PX_DEBUG
}

//////////////////////////////////////////////////////////////////////////
// Builds the new main AABB tree with given current active merged trees and its bounds
void ExtendedBucketPruner::buildMainAABBTree()
{
	// create the AABB tree from given merged trees bounds
	AABBTreeBuildParams sTB;
	sTB.mNbPrimitives = mCurrentTreeIndex;
	sTB.mAABBArray = mBounds;
	sTB.mLimit = NB_OBJECTS_PER_NODE;
	bool status = mMainTree->build(sTB);

	PX_UNUSED(status);
	PX_ASSERT(status);

	// Init main tree update map for the new main tree
	mMainTreeUpdateMap.initMap(mCurrentTreeIndex, *mMainTree);
}

//////////////////////////////////////////////////////////////////////////
// resize internal memory, buffers
void ExtendedBucketPruner::resize(PxU32 size)
{
	PX_ASSERT(size > mCurrentTreeCapacity);
	// allocate new bounds
	PxBounds3* newBounds = reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(size + 1), "Bounds"));
	// copy previous bounds
	PxMemCopy(newBounds, mBounds, sizeof(PxBounds3)*mCurrentTreeCapacity);
	PX_FREE(mBounds);
	mBounds = newBounds;

	// allocate new merged trees
	MergedTree* newMergeTrees = reinterpret_cast<MergedTree*>(PX_ALLOC(sizeof(MergedTree)*size, "AABB trees"));
	// copy previous merged trees
	PxMemCopy(newMergeTrees, mMergedTrees, sizeof(MergedTree)*mCurrentTreeCapacity);
	PX_FREE(mMergedTrees);
	mMergedTrees = newMergeTrees;
	// allocate new trees for merged trees
	for (PxU32 i = mCurrentTreeCapacity; i < size; i++)
	{
		mMergedTrees[i].mTimeStamp = 0;
		mMergedTrees[i].mTree = PX_NEW(AABBTree);
	}

	mCurrentTreeCapacity = size;
}

//////////////////////////////////////////////////////////////////////////
// Update object
bool ExtendedBucketPruner::updateObject(const PxBounds3& worldAABB, const PrunerPayload& object, const PoolIndex poolIndex)
{	
	const ExtendedBucketPrunerMap::Entry* extendedPrunerEntry = mExtendedBucketPrunerMap.find(object);

	// if object is not in tree of trees, it is in bucket pruner core
	if(!extendedPrunerEntry)
	{		
#if USE_INCREMENTAL_PRUNER
		PX_UNUSED(worldAABB);
		return mPrunerCore.updateObject(poolIndex);
#else
		PX_UNUSED(poolIndex);
		return mPrunerCore.updateObject(worldAABB, object);
#endif 
	}
	else
	{
		const ExtendedBucketPrunerData& data = extendedPrunerEntry->second;

		PX_ASSERT(data.mMergeIndex < mCurrentTreeIndex);

		// update tree where objects belongs to
		AABBTree& tree = *mMergedTrees[data.mMergeIndex].mTree;
		PX_ASSERT(data.mSubTreeNode < tree.getNbNodes());
		// mark for refit node in merged tree
		tree.markNodeForRefit(data.mSubTreeNode);
		PX_ASSERT(mMainTreeUpdateMap[data.mMergeIndex] < mMainTree->getNbNodes());
		// mark for refit node in main aabb tree
		mMainTree->markNodeForRefit(mMainTreeUpdateMap[data.mMergeIndex]);
		mTreesDirty = true;
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////
// refit merged nodes 
// 1. refit nodes in merged trees
// 2. check if after refit root node is valid - might happen edge case
//		where all objects were released - the root node is then invalid
//		in this edge case we need to compact the merged trees array 
//		and create new main AABB tree
// 3. If all merged trees bounds are valid - refit main tree
// 4. If bounds are invalid create new main AABB tree
void ExtendedBucketPruner::refitMarkedNodes(const PxBounds3* boxes)
{
	// if no tree needs update early exit
	if(!mTreesDirty)
		return;

	// refit trees and update bounds for main tree	
	PxU32 nbValidTrees = 0;
	for (PxU32 i = mCurrentTreeIndex; i--; )
	{
		AABBTree& tree = *mMergedTrees[i].mTree;
		tree.refitMarkedNodes(boxes);
		const PxBounds3& bounds = tree.getNodes()[0].mBV;
		// check if bounds are valid, if all objects of the tree were released, the bounds 
		// will be invalid, in that case we cannot use this tree anymore.
		if(bounds.isValid())
		{			
			nbValidTrees++;
		}
		mBounds[i] = bounds;
	}
	
	if(nbValidTrees == mCurrentTreeIndex)
	{
		// no tree has been removed refit main tree
		mMainTree->refitMarkedNodes(mBounds);
	}
	else
	{	
		// edge case path, tree does not have a valid root node bounds - all objects from the tree were released
		// we might even fire perf warning
		// compact the tree array - no holes in the array, remember the swap position
		PxU32* swapMap = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mCurrentTreeIndex + 1, "Swap Map"));
		PxU32 writeIndex = 0;
		for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
		{
			AABBTree& tree = *mMergedTrees[i].mTree;
			if(tree.getNodes()[0].mBV.isValid())
			{
				// we have to store the tree into an empty location
				if(i != writeIndex)
				{
					PX_ASSERT(writeIndex < i);
					AABBTree* ptr = mMergedTrees[writeIndex].mTree;
					mMergedTrees[writeIndex] = mMergedTrees[i];
					mMergedTrees[i].mTree = ptr;
					mBounds[writeIndex] = mBounds[i];
				}
				// remember the swap location
				swapMap[i] = writeIndex;
				writeIndex++;				
			}
			else
			{
				// tree is not valid, release it
				tree.release();
				mMergedTrees[i].mTimeStamp = 0;
			}

			// remember the swap
			swapMap[mCurrentTreeIndex] = i;
		}		

		PX_ASSERT(writeIndex == nbValidTrees);

		// new merged trees size
		mCurrentTreeIndex = nbValidTrees;

		if(mCurrentTreeIndex)
		{
			// trees have changed, we need to rebuild the main tree
			buildMainAABBTree();

			// fixup the object entries, the merge index has changed	
			for (ExtendedBucketPrunerMap::Iterator iter = mExtendedBucketPrunerMap.getIterator(); !iter.done(); ++iter)
			{			
				ExtendedBucketPrunerData& data = iter->second;
				PX_ASSERT(swapMap[data.mMergeIndex] < nbValidTrees);
				data.mMergeIndex = swapMap[data.mMergeIndex];
			}
		}
		else
		{
			// if there is no tree release the main tree
			mMainTree->release();
		}
		PX_FREE(swapMap);
	}
#if PX_DEBUG
	checkValidity();
#endif
	mTreesDirty = false;
}

//////////////////////////////////////////////////////////////////////////
// remove object
bool ExtendedBucketPruner::removeObject(const PrunerPayload& object, PxU32 objectIndex, const PrunerPayload& swapObject,
	PxU32 swapObjectIndex, PxU32& timeStamp)
{
	ExtendedBucketPrunerMap::Entry dataEntry;
	
	// if object is not in tree of trees, it is in bucket pruner core
	if (!mExtendedBucketPrunerMap.erase(object, dataEntry))
	{
		// we need to call invalidateObjects, it might happen that the swapped object
		// does belong to the extended bucket pruner, in that case the objects index
		// needs to be swapped.
		// do not call additional bucket pruner swap, that does happen during remove
		swapIndex(objectIndex, swapObject, swapObjectIndex, false);
#if USE_INCREMENTAL_PRUNER
		return mPrunerCore.removeObject(objectIndex, swapObjectIndex, timeStamp);
#else
		return mPrunerCore.removeObject(object, timeStamp);
#endif
	}
	else
	{	
		const ExtendedBucketPrunerData& data = dataEntry.second;

		// mark tree nodes where objects belongs to
		AABBTree& tree = *mMergedTrees[data.mMergeIndex].mTree;
		PX_ASSERT(data.mSubTreeNode < tree.getNbNodes());
		// mark the merged tree for refit
		tree.markNodeForRefit(data.mSubTreeNode);
		PX_ASSERT(mMainTreeUpdateMap[data.mMergeIndex] < mMainTree->getNbNodes());
		// mark the main tree for refit
		mMainTree->markNodeForRefit(mMainTreeUpdateMap[data.mMergeIndex]);

		// call invalidate object to swap the object indices in the merged trees
		invalidateObject(data, objectIndex, swapObject, swapObjectIndex);		

		mTreesDirty = true;
	}
#if PX_DEBUG
	checkValidity();
#endif // PX_DEBUG
	return true;
}

//////////////////////////////////////////////////////////////////////////
// invalidate object
// remove the objectIndex from the merged tree
void ExtendedBucketPruner::invalidateObject(const ExtendedBucketPrunerData& data, PxU32 objectIndex, const PrunerPayload& swapObject, 
	PxU32 swapObjectIndex)
{
	// get the merged tree
	AABBTree& tree = *mMergedTrees[data.mMergeIndex].mTree;
	PX_ASSERT(data.mSubTreeNode < tree.getNbNodes());
	PX_ASSERT(tree.getNodes()[data.mSubTreeNode].isLeaf());
	// get merged tree node
	AABBTreeRuntimeNode& node0 = tree.getNodes()[data.mSubTreeNode];
	const PxU32 nbPrims = node0.getNbRuntimePrimitives();
	PX_ASSERT(nbPrims <= NB_OBJECTS_PER_NODE);

	// retrieve the primitives pointer
	PxU32* primitives = node0.getPrimitives(tree.getIndices());
	PX_ASSERT(primitives);

	// Look for desired pool index in the leaf
	bool foundIt = false;
	for (PxU32 i = 0; i < nbPrims; i++)
	{
		if (objectIndex == primitives[i])
		{
			foundIt = true;
			const PxU32 last = nbPrims - 1;
			node0.setNbRunTimePrimitives(last);
			primitives[i] = INVALID_POOL_ID;			// Mark primitive index as invalid in the node				

			// Swap within the leaf node. No need to update the mapping since they should all point
			// to the same tree node anyway.
			if (last != i)
				Ps::swap(primitives[i], primitives[last]);
			break;
		}
	}
	PX_ASSERT(foundIt);
	PX_UNUSED(foundIt);

	swapIndex(objectIndex, swapObject, swapObjectIndex);
}

// Swap object index
// if swapObject is in a merged tree its index needs to be swapped with objectIndex
void ExtendedBucketPruner::swapIndex(PxU32 objectIndex, const PrunerPayload& swapObject, PxU32 swapObjectIndex, bool corePrunerIncluded)
{
	PX_UNUSED(corePrunerIncluded);
	if (objectIndex == swapObjectIndex)
		return;

	const ExtendedBucketPrunerMap::Entry* extendedPrunerSwapEntry = mExtendedBucketPrunerMap.find(swapObject);

	// if swapped object index is in extended pruner, we have to fix the primitives index
	if (extendedPrunerSwapEntry)
	{
		const ExtendedBucketPrunerData& swapData = extendedPrunerSwapEntry->second;
		AABBTree& swapTree = *mMergedTrees[swapData.mMergeIndex].mTree;
		// With multiple primitives per leaf, tree nodes may very well be the same for different pool indices.
		// However the pool indices may be the same when a swap has been skipped in the pruning pool, in which
		// case there is nothing to do.
		PX_ASSERT(swapData.mSubTreeNode < swapTree.getNbNodes());
		PX_ASSERT(swapTree.getNodes()[swapData.mSubTreeNode].isLeaf());
		AABBTreeRuntimeNode* node1 = swapTree.getNodes() + swapData.mSubTreeNode;
		const PxU32 nbPrims = node1->getNbRuntimePrimitives();
		PX_ASSERT(nbPrims <= NB_OBJECTS_PER_NODE);

		// retrieve the primitives pointer
		PxU32* primitives = node1->getPrimitives(swapTree.getIndices());
		PX_ASSERT(primitives);

		// look for desired pool index in the leaf
		bool foundIt = false;
		for (PxU32 i = 0; i < nbPrims; i++)
		{
			if (swapObjectIndex == primitives[i])
			{
				foundIt = true;
				primitives[i] = objectIndex;	// point node to the pool object moved to 
				break;
			}
		}
		PX_ASSERT(foundIt);
		PX_UNUSED(foundIt);
	}
#if USE_INCREMENTAL_PRUNER
	else
	{
		if(corePrunerIncluded)
			mPrunerCore.swapIndex(objectIndex, swapObjectIndex);
	}
#endif
}

//////////////////////////////////////////////////////////////////////////
// Optimized removal of timestamped objects from the extended bucket pruner
PxU32 ExtendedBucketPruner::removeMarkedObjects(PxU32 timeStamp)
{
	// remove objects from the core bucket pruner
	PxU32 retVal = mPrunerCore.removeMarkedObjects(timeStamp);

	// nothing to be removed
	if(!mCurrentTreeIndex)
		return retVal;

	// if last merged tree is the timeStamp to remove, we can clear all
	// this is safe as the merged trees array is time ordered, never shifted
	if(mMergedTrees[mCurrentTreeIndex - 1].mTimeStamp == timeStamp)
	{
		retVal += mExtendedBucketPrunerMap.size();
		cleanTrees();
		return retVal;
	}

	// get the highest index in the merged trees array, where timeStamp match
	// we release than all trees till the index
	PxU32 highestTreeIndex = 0xFFFFFFFF;
	for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		if(mMergedTrees[i].mTimeStamp == timeStamp)
			highestTreeIndex = i;
		else
			break;
	}

	// if no timestamp found early exit
	if(highestTreeIndex == 0xFFFFFFFF)
	{
		return retVal;
	}

	PX_ASSERT(highestTreeIndex < mCurrentTreeIndex);
	// get offset, where valid trees start
	const PxU32 mergeTreeOffset = highestTreeIndex + 1;

	// shrink the array to merged trees with a valid timeStamp
	mCurrentTreeIndex = mCurrentTreeIndex - mergeTreeOffset;
	// go over trees and swap released trees with valid trees from the back (valid trees are at the back) 
	for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		// store bounds, timestamp
		mBounds[i] = mMergedTrees[mergeTreeOffset + i].mTree->getNodes()[0].mBV;		
		mMergedTrees[i].mTimeStamp = mMergedTrees[mergeTreeOffset + i].mTimeStamp;

		// release the tree with timestamp
		AABBTree* ptr = mMergedTrees[i].mTree;
		ptr->release();

		// store the valid tree
		mMergedTrees[i].mTree = mMergedTrees[mergeTreeOffset + i].mTree;
		// store the release tree at the offset
		mMergedTrees[mergeTreeOffset + i].mTree = ptr;
		mMergedTrees[mergeTreeOffset + i].mTimeStamp = 0;
	}
	// release the rest of the trees with not valid timestamp
	for (PxU32 i = mCurrentTreeIndex; i <= highestTreeIndex; i++)
	{
		mMergedTrees[i].mTree->release();
		mMergedTrees[i].mTimeStamp = 0;
	}

	// build new main AABB tree with only trees with valid valid timeStamp
	buildMainAABBTree();

	// remove all unnecessary trees and map entries
	bool removeEntry = false;
	PxU32 numRemovedEntries = 0;
	ExtendedBucketPrunerMap::EraseIterator eraseIterator = mExtendedBucketPrunerMap.getEraseIterator();
	ExtendedBucketPrunerMap::Entry* entry = eraseIterator.eraseCurrentGetNext(removeEntry);
	while (entry)
	{
		ExtendedBucketPrunerData& data = entry->second;
		// data to be removed
		if (data.mTimeStamp == timeStamp)
		{
			removeEntry = true;
			numRemovedEntries++;
		}
		else
		{
			// update the merge index and main tree node index
			PX_ASSERT(highestTreeIndex < data.mMergeIndex);
			data.mMergeIndex -= mergeTreeOffset;
			removeEntry = false;
		}
		entry = eraseIterator.eraseCurrentGetNext(removeEntry);
	}

#if PX_DEBUG
	checkValidity();
#endif // PX_DEBUG
	// return the number of removed objects
	return retVal + numRemovedEntries;
}

//////////////////////////////////////////////////////////////////////////
// clean all trees, all objects have been released
void ExtendedBucketPruner::cleanTrees()
{
	for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		mMergedTrees[i].mTree->release();
		mMergedTrees[i].mTimeStamp = 0;
	}
	mExtendedBucketPrunerMap.clear();
	mCurrentTreeIndex = 0;
	mMainTree->release();
}

//////////////////////////////////////////////////////////////////////////
// shift origin
void ExtendedBucketPruner::shiftOrigin(const PxVec3& shift)
{
	mMainTree->shiftOrigin(shift);

	for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		mMergedTrees[i].mTree->shiftOrigin(shift);
	}

	mPrunerCore.shiftOrigin(shift);
}

//////////////////////////////////////////////////////////////////////////
// Queries implementation
//////////////////////////////////////////////////////////////////////////
// Raycast/sweeps callback for main AABB tree
template<bool tInflate>
struct MainTreeRaycastPrunerCallback: public PrunerCallback
{
	MainTreeRaycastPrunerCallback(const PxVec3& origin, const PxVec3& unitDir, const PxVec3& extent, PrunerCallback& prunerCallback, const PruningPool* pool)
		: mOrigin(origin), mUnitDir(unitDir), mExtent(extent), mPrunerCallback(prunerCallback), mPruningPool(pool)
	{
	}

	virtual PxAgain invoke(PxReal& distance, const PrunerPayload& payload)
	{
		// payload data match merged tree data MergedTree, we can cast it
		const AABBTree* aabbTree = reinterpret_cast<const AABBTree*> (payload.data[0]);
		// raycast the merged tree
		return AABBTreeRaycast<tInflate, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPruningPool->getObjects(), mPruningPool->getCurrentWorldBoxes(), *aabbTree, mOrigin, mUnitDir, distance, mExtent, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeRaycastPrunerCallback)

private:
	const PxVec3&		mOrigin;
	const PxVec3&		mUnitDir;	
	const PxVec3&		mExtent;
	PrunerCallback&		mPrunerCallback;
	const PruningPool*	mPruningPool;
};

//////////////////////////////////////////////////////////////////////////
// raycast against the extended bucket pruner
PxAgain ExtendedBucketPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& prunerCallback) const
{
	PxAgain again = true;	

	// searc the bucket pruner first
	if (mPrunerCore.getNbObjects())
		again = mPrunerCore.raycast(origin, unitDir, inOutDistance, prunerCallback);

	if (again && mExtendedBucketPrunerMap.size())
	{
		const PxVec3 extent(0.0f);
		// main tree callback
		MainTreeRaycastPrunerCallback<false> pcb(origin, unitDir, extent, prunerCallback, mPruningPool);
		// traverse the main tree
		again = AABBTreeRaycast<false, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, origin, unitDir, inOutDistance, extent, pcb);
	}

	return again;
}

//////////////////////////////////////////////////////////////////////////
// overlap main tree callback
template<typename Test>
struct MainTreeOverlapPrunerCallback : public PrunerCallback
{
	MainTreeOverlapPrunerCallback(const Test& test, PrunerCallback& prunerCallback, const PruningPool* pool)
		: mTest(test), mPrunerCallback(prunerCallback), mPruningPool(pool)
	{
	}

	virtual PxAgain invoke(PxReal& , const PrunerPayload& payload)
	{
		// payload data match merged tree data MergedTree, we can cast it
		const AABBTree* aabbTree = reinterpret_cast<const AABBTree*> (payload.data[0]);
		// overlap the merged tree
		return AABBTreeOverlap<Test, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(mPruningPool->getObjects(), mPruningPool->getCurrentWorldBoxes(), *aabbTree, mTest, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeOverlapPrunerCallback)

private:
	const Test&			mTest;	
	PrunerCallback&		mPrunerCallback;
	const PruningPool*	mPruningPool;
};

//////////////////////////////////////////////////////////////////////////
// overlap implementation
PxAgain ExtendedBucketPruner::overlap(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback) const
{
	PxAgain again = true;

	// core bucket pruner overlap
	if (mPrunerCore.getNbObjects())
		again = mPrunerCore.overlap(queryVolume, prunerCallback);

	if(again && mExtendedBucketPrunerMap.size())
	{
		switch (queryVolume.getType())
		{
		case PxGeometryType::eBOX:
		{
			if (queryVolume.isOBB())
			{
				const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
				MainTreeOverlapPrunerCallback<Gu::OBBAABBTest> pcb(test, prunerCallback, mPruningPool);
				again = AABBTreeOverlap<Gu::OBBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, test, pcb);
			}
			else
			{
				const Gu::AABBAABBTest test(queryVolume.getPrunerInflatedWorldAABB());
				MainTreeOverlapPrunerCallback<Gu::AABBAABBTest> pcb(test, prunerCallback, mPruningPool);
				again = AABBTreeOverlap<Gu::AABBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, test, pcb);				
			}
		}
		break;
		case PxGeometryType::eCAPSULE:
		{
			const Gu::Capsule& capsule = queryVolume.getGuCapsule();
			const Gu::CapsuleAABBTest test(capsule.p1, queryVolume.getPrunerWorldRot33().column0,
				queryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));
			MainTreeOverlapPrunerCallback<Gu::CapsuleAABBTest> pcb(test, prunerCallback, mPruningPool);			
			again = AABBTreeOverlap<Gu::CapsuleAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, test, pcb);				
		}
		break;
		case PxGeometryType::eSPHERE:
		{
			const Gu::Sphere& sphere = queryVolume.getGuSphere();
			Gu::SphereAABBTest test(sphere.center, sphere.radius);
			MainTreeOverlapPrunerCallback<Gu::SphereAABBTest> pcb(test, prunerCallback, mPruningPool);
			again = AABBTreeOverlap<Gu::SphereAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, test, pcb);				
		}
		break;
		case PxGeometryType::eCONVEXMESH:
		{
			const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
			MainTreeOverlapPrunerCallback<Gu::OBBAABBTest> pcb(test, prunerCallback, mPruningPool);			
			again = AABBTreeOverlap<Gu::OBBAABBTest, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, test, pcb);				
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

	return again;
}

//////////////////////////////////////////////////////////////////////////
// sweep implementation 
PxAgain ExtendedBucketPruner::sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& prunerCallback) const
{
	PxAgain again = true;

	// core bucket pruner sweep
	if (mPrunerCore.getNbObjects())
		again = mPrunerCore.sweep(queryVolume, unitDir, inOutDistance, prunerCallback);

	if(again && mExtendedBucketPrunerMap.size())
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		const PxVec3 extents = aabb.getExtents();
		const PxVec3 center = aabb.getCenter();
		MainTreeRaycastPrunerCallback<true> pcb(center, unitDir, extents, prunerCallback, mPruningPool);
		again = AABBTreeRaycast<true, AABBTree, AABBTreeRuntimeNode, PrunerPayload, PrunerCallback>()(reinterpret_cast<const PrunerPayload*>(mMergedTrees), mBounds, *mMainTree, center, unitDir, inOutDistance, extents, pcb);
	}
	return again;
}


//////////////////////////////////////////////////////////////////////////
#include "CmRenderOutput.h"

// visualization
static void visualizeTree(Cm::RenderOutput& out, PxU32 color, AABBTree* tree)
{
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
}

void ExtendedBucketPruner::visualize(Cm::RenderOutput& out, PxU32 color) const
{	
	visualizeTree(out, color, mMainTree);

	for(PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		visualizeTree(out, color, mMergedTrees[i].mTree);
	}

	mPrunerCore.visualize(out, color);
}

//////////////////////////////////////////////////////////////////////////

#if PX_DEBUG
// extended bucket pruner validity check
bool ExtendedBucketPruner::checkValidity()
{
	Cm::BitMap testBitmap;
	testBitmap.resizeAndClear(mCurrentTreeIndex);
	for (PxU32 i = 0; i < mMainTree->getNbNodes(); i++)
	{
		const AABBTreeRuntimeNode& node = mMainTree->getNodes()[i];
		if(node.isLeaf())
		{
			const PxU32 nbPrims = node.getNbRuntimePrimitives();
			PX_ASSERT(nbPrims <= NB_OBJECTS_PER_NODE);
			
			const PxU32* primitives = node.getPrimitives(mMainTree->getIndices());
			for (PxU32 j = 0; j < nbPrims; j++)
			{				
				const PxU32 index = primitives[j];
				// check if index is correct
				PX_ASSERT(index < mCurrentTreeIndex);
				// mark the index in the test bitmap, must be once set only, all merged trees must be in the main tree
				PX_ASSERT(testBitmap.test(index) == IntFalse);
				testBitmap.set(index);
			}
		}
	}
	
	Cm::BitMap mergeTreeTestBitmap;
	mergeTreeTestBitmap.resizeAndClear(mPruningPool->getNbActiveObjects());
	for (PxU32 i = 0; i < mCurrentTreeIndex; i++)
	{
		// check if bounds are the same as the merged tree root bounds
		PX_ASSERT(mBounds[i].maximum.x == mMergedTrees[i].mTree->getNodes()[0].mBV.maximum.x);
		PX_ASSERT(mBounds[i].maximum.y == mMergedTrees[i].mTree->getNodes()[0].mBV.maximum.y);
		PX_ASSERT(mBounds[i].maximum.z == mMergedTrees[i].mTree->getNodes()[0].mBV.maximum.z);
		PX_ASSERT(mBounds[i].minimum.x == mMergedTrees[i].mTree->getNodes()[0].mBV.minimum.x);
		PX_ASSERT(mBounds[i].minimum.y == mMergedTrees[i].mTree->getNodes()[0].mBV.minimum.y);
		PX_ASSERT(mBounds[i].minimum.z == mMergedTrees[i].mTree->getNodes()[0].mBV.minimum.z);

		// check each tree
		const AABBTree& mergedTree = *mMergedTrees[i].mTree;
		for (PxU32 j = 0; j < mergedTree.getNbNodes(); j++)
		{
			const AABBTreeRuntimeNode& node = mergedTree.getNodes()[j];
			if (node.isLeaf())
			{
				const PxU32 nbPrims = node.getNbRuntimePrimitives();
				PX_ASSERT(nbPrims <= NB_OBJECTS_PER_NODE);

				const PxU32* primitives = node.getPrimitives(mergedTree.getIndices());
				for (PxU32 k = 0; k < nbPrims; k++)
				{
					const PxU32 index = primitives[k];
					// check if index is correct
					PX_ASSERT(index < mPruningPool->getNbActiveObjects());
					// mark the index in the test bitmap, must be once set only, all merged trees must be in the main tree
					PX_ASSERT(mergeTreeTestBitmap.test(index) == IntFalse);
					mergeTreeTestBitmap.set(index);

					const PrunerPayload& payload = mPruningPool->getObjects()[index];
					const ExtendedBucketPrunerMap::Entry* extendedPrunerSwapEntry = mExtendedBucketPrunerMap.find(payload);
					PX_ASSERT(extendedPrunerSwapEntry);

					const ExtendedBucketPrunerData& data = extendedPrunerSwapEntry->second;
					PX_ASSERT(data.mMergeIndex == i);
					PX_ASSERT(data.mSubTreeNode == j);
				}
			}
		}
	}
	for (PxU32 i = mCurrentTreeIndex; i < mCurrentTreeCapacity; i++)
	{
		PX_ASSERT(mMergedTrees[i].mTree->getIndices() == NULL);
		PX_ASSERT(mMergedTrees[i].mTree->getNodes() == NULL);
	}
	for (ExtendedBucketPrunerMap::Iterator iter = mExtendedBucketPrunerMap.getIterator(); !iter.done(); ++iter)
	{		
		const ExtendedBucketPrunerData& data = iter->second;
		PX_ASSERT(mMainTreeUpdateMap[data.mMergeIndex] < mMainTree->getNbNodes());
		PX_ASSERT(data.mMergeIndex < mCurrentTreeIndex);
		PX_ASSERT(data.mSubTreeNode < mMergedTrees[data.mMergeIndex].mTree->getNbNodes());
	}
	return true;
}
#endif

