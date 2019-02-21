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


#include "PsFoundation.h"
#include "PsAllocator.h"
#include "SqCompoundPruningPool.h"
#include "SqAABBTree.h"
#include "SqPruningPool.h"
#include "GuBVHStructure.h"

using namespace physx;
using namespace Gu;
using namespace Sq;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::updateObjectAfterManualBoundsUpdates(PrunerHandle handle)
{
	const PxBounds3* newBounds = mPruningPool->getCurrentWorldBoxes();	
	const PoolIndex poolIndex = mPruningPool->getIndex(handle);
	NodeList changedLeaves;
	changedLeaves.reserve(8);
	IncrementalAABBTreeNode* node = mTree->update((*mUpdateMap)[poolIndex], poolIndex, newBounds, changedLeaves);
	// we removed node during update, need to update the mapping	
	updateMapping(poolIndex, node, changedLeaves);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::removeObject(PrunerHandle handle)
{
	const PoolIndex poolIndex = mPruningPool->getIndex(handle); // save the pool index for removed object
	const PoolIndex poolRelocatedLastIndex = mPruningPool->removeObject(handle); // save the lastIndex returned by removeObject

	IncrementalAABBTreeNode* node = mTree->remove((*mUpdateMap)[poolIndex], poolIndex, mPruningPool->getCurrentWorldBoxes());
	// if node moved to its parent
	if (node && node->isLeaf())
	{
		for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
		{
			const PoolIndex index = node->getPrimitives(NULL)[j];
			(*mUpdateMap)[index] = node;
		}
	}

	(*mUpdateMap)[poolIndex] = (*mUpdateMap)[poolRelocatedLastIndex];
	// fix indices if we made a swap
	if(poolRelocatedLastIndex != poolIndex)
		mTree->fixupTreeIndices((*mUpdateMap)[poolIndex], poolRelocatedLastIndex, poolIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool CompoundTree::addObject(PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload payload)
{
	mPruningPool->addObjects(&result, &bounds, &payload, 1);	
	
	const PoolIndex poolIndex = mPruningPool->getIndex(result);
	NodeList changedLeaves;
	changedLeaves.reserve(8);
	IncrementalAABBTreeNode* node = mTree->insert(poolIndex, mPruningPool->getCurrentWorldBoxes(), changedLeaves);
	updateMapping(poolIndex, node, changedLeaves);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTree::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node, const NodeList& changedLeaves)
{
	// if a node was split we need to update the node indices and also the sibling indices
	if(!changedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				(*mUpdateMap)[index] = node;
			}
		}

		for(PxU32 i = 0; i < changedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = changedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				const PoolIndex index = changedNode->getPrimitives(NULL)[j];
				(*mUpdateMap)[index] = changedNode;
			}
		}
	}
	else
	{
		(*mUpdateMap)[poolIndex] = node;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

CompoundTreePool::CompoundTreePool(): 
	mNbObjects(0),
	mMaxNbObjects(0),
	mCompoundBounds(NULL),
	mCompoundTrees(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////

CompoundTreePool::~CompoundTreePool()
{
	PX_FREE_AND_RESET(mCompoundBounds);
	PX_FREE_AND_RESET(mCompoundTrees);
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool CompoundTreePool::resize(PxU32 newCapacity)
{
	// PT: we always allocate one extra box, to make sure we can safely use V4 loads on the array
	PxBounds3*				newBoxes			= reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(newCapacity+1), "PxBounds3"));
	CompoundTree*			newTrees			= reinterpret_cast<CompoundTree*>(PX_ALLOC(sizeof(CompoundTree)*newCapacity, "IncrementalTrees*"));

	// memzero, we need to set the pointers in the compound tree to NULL
	PxMemZero(newTrees, sizeof(CompoundTree)*newCapacity);

	if((NULL==newBoxes) || (NULL==newTrees))
	{
		PX_FREE_AND_RESET(newBoxes);
		PX_FREE_AND_RESET(newTrees);	
		return false;
	}

	if(mCompoundBounds)		PxMemCopy(newBoxes, mCompoundBounds, mNbObjects*sizeof(PxBounds3));
	if(mCompoundTrees)		PxMemCopy(newTrees, mCompoundTrees, mNbObjects*sizeof(CompoundTree));
	mMaxNbObjects = newCapacity;

	PX_FREE_AND_RESET(mCompoundBounds);
	PX_FREE_AND_RESET(mCompoundTrees);
	mCompoundBounds		= newBoxes;
	mCompoundTrees		= newTrees;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTreePool::preallocate(PxU32 newCapacity)
{
	if(newCapacity>mMaxNbObjects)
		resize(newCapacity);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void CompoundTreePool::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0; i < mNbObjects; i++)
	{
		mCompoundBounds[i].minimum -= shift;
		mCompoundBounds[i].maximum -= shift;

		mCompoundTrees[i].mGlobalPose.p -= shift;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

PoolIndex CompoundTreePool::addCompound(PrunerHandle* results, const BVHStructure& bvhStructure, const PxBounds3& compoundBounds, const PxTransform& transform, 
	CompoundFlag::Enum flags, const PrunerPayload* userData)
{
	if(mNbObjects==mMaxNbObjects) // increase the capacity on overflow
	{
		if(!resize(PxMax<PxU32>(mMaxNbObjects*2, 32)))
		{
			// pool can return an invalid handle if memory alloc fails			
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "CompoundTreePool::addCompound memory allocation in resize failed.");
			return INVALID_PRUNERHANDLE;
		}
	}
	PX_ASSERT(mNbObjects!=mMaxNbObjects);

	const PoolIndex index = mNbObjects++;
		
	mCompoundBounds[index] = compoundBounds;

	const PxU32 nbObjects = bvhStructure.getNbBounds();

	CompoundTree& tree = mCompoundTrees[index];
	PX_ASSERT(tree.mPruningPool == NULL);
	PX_ASSERT(tree.mTree == NULL);
	PX_ASSERT(tree.mUpdateMap == NULL);

	tree.mGlobalPose = transform;
	tree.mFlags = flags;

	// prepare the pruning pool
	PruningPool* pool = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PruningPool), PX_DEBUG_EXP("Pruning pool")), PruningPool);
	pool->preallocate(nbObjects);
	pool->addObjects(results, bvhStructure.getBounds(), userData, nbObjects);
	tree.mPruningPool = pool;

	// prepare update map
    UpdateMap* map = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(UpdateMap), PX_DEBUG_EXP("Update map")), UpdateMap);
	map->resizeUninitialized(nbObjects);
	tree.mUpdateMap = map;

	IncrementalAABBTree* iTree = PX_NEW(IncrementalAABBTree)();
	iTree->copy(bvhStructure, *map);
	tree.mTree = iTree;

	return index;
}

///////////////////////////////////////////////////////////////////////////////////////////////

PoolIndex CompoundTreePool::removeCompound(PoolIndex indexOfRemovedObject)
{
	PX_ASSERT(mNbObjects);

	// release the tree
	mCompoundTrees[indexOfRemovedObject].mTree->release();
	mCompoundTrees[indexOfRemovedObject].mTree->~IncrementalAABBTree();
	PX_FREE_AND_RESET(mCompoundTrees[indexOfRemovedObject].mTree);

	mCompoundTrees[indexOfRemovedObject].mUpdateMap->clear();
	mCompoundTrees[indexOfRemovedObject].mUpdateMap->~Array();
	PX_FREE_AND_RESET(mCompoundTrees[indexOfRemovedObject].mUpdateMap);

	mCompoundTrees[indexOfRemovedObject].mPruningPool->~PruningPool();
	PX_FREE_AND_RESET(mCompoundTrees[indexOfRemovedObject].mPruningPool);

	const PoolIndex indexOfLastObject = --mNbObjects; // swap the object at last index with index
	if(indexOfLastObject!=indexOfRemovedObject)
	{
		// PT: move last object's data to recycled spot (from removed object)

		// PT: the last object has moved so we need to handle the mappings for this object
		mCompoundBounds		[indexOfRemovedObject]	= mCompoundBounds	[indexOfLastObject];
		mCompoundTrees		[indexOfRemovedObject]	= mCompoundTrees	[indexOfLastObject];

		mCompoundTrees		[indexOfLastObject].mPruningPool = NULL;
		mCompoundTrees		[indexOfLastObject].mUpdateMap = NULL;
		mCompoundTrees		[indexOfLastObject].mTree = NULL;
	}

	return indexOfLastObject;
}

