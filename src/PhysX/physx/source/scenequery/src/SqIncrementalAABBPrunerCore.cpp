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

#include "SqIncrementalAABBPrunerCore.h"
#include "SqIncrementalAABBTree.h"
#include "SqPruningPool.h"
#include "SqAABBTree.h"
#include "GuAABBTreeQuery.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuBounds.h"

using namespace physx;
using namespace Gu;
using namespace Sq;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PARANOIA_CHECKS 0

IncrementalAABBPrunerCore::IncrementalAABBPrunerCore(const PruningPool* pool) :
	mCurrentTree		(1),
	mLastTree			(0),
	mPool				(pool)
{
	mAABBTree[0].mapping.reserve(256);
	mAABBTree[1].mapping.reserve(256);
	mChangedLeaves.reserve(32);
}

IncrementalAABBPrunerCore::~IncrementalAABBPrunerCore()
{
	release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IncrementalAABBPrunerCore::release() // this can be called from purge()
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		if(mAABBTree[i].tree)
		{
			PX_DELETE(mAABBTree[i].tree);
			mAABBTree[i].tree = NULL;
		}
		mAABBTree[i].mapping.clear();
		mAABBTree[i].timeStamp = 0;
	}
	mCurrentTree = 1;
	mLastTree = 0;
}

bool IncrementalAABBPrunerCore::addObject(const PoolIndex poolIndex, PxU32 timeStamp)
{
	CoreTree& tree = mAABBTree[mCurrentTree];
	if(!tree.tree || !tree.tree->getNodes())
	{
		if(!tree.tree)
			tree.tree = PX_NEW(IncrementalAABBTree)();
		tree.timeStamp = timeStamp;
	}
	PX_ASSERT(tree.timeStamp == timeStamp);

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = tree.tree->insert(poolIndex, mPool->getCurrentWorldBoxes(), mChangedLeaves);
	updateMapping(tree.mapping, poolIndex, node);

#if PARANOIA_CHECKS
	test();
#endif

	return true;
}

void IncrementalAABBPrunerCore::updateMapping(IncrementalPrunerMap& mapping, const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// if some node leaves changed, we need to update mapping
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				mapping[index] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				const PoolIndex index = changedNode->getPrimitives(NULL)[j];
				mapping[index] = changedNode;
			}
		}
	}
	else
	{
		PX_ASSERT(node->isLeaf());
		mapping[poolIndex] = node;
	}
}

bool IncrementalAABBPrunerCore::removeObject(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex, PxU32& timeStamp)
{
	// erase the entry and get the data
	IncrementalPrunerMap::Entry entry;
	bool foundEntry = true;
	const PxU32 treeIndex = mAABBTree[mLastTree].mapping.erase(poolIndex, entry) ? mLastTree : mCurrentTree;
	// if it was not found in the last tree look at the current tree
	if(treeIndex == mCurrentTree)
		foundEntry = mAABBTree[mCurrentTree].mapping.erase(poolIndex, entry);

	// exit somethings is wrong here, entry was not found here
	PX_ASSERT(foundEntry);
	if(!foundEntry)
		return false;

	// tree must exist
	PX_ASSERT(mAABBTree[treeIndex].tree);
	CoreTree& tree = mAABBTree[treeIndex];
	timeStamp = tree.timeStamp;

	// remove the poolIndex from the tree, update the tree bounds immediatelly
	IncrementalAABBTreeNode* node = tree.tree->remove(entry.second, poolIndex, mPool->getCurrentWorldBoxes());
	if(node && node->isLeaf())
	{
		for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
		{
			const PoolIndex index = node->getPrimitives(NULL)[j];
			tree.mapping[index] = node;
		}
	}

	// nothing to swap, last object, early exit
	if(poolIndex == poolRelocatedLastIndex)
	{
#if PARANOIA_CHECKS
	test();
#endif
		return true;
	}

	// fix the indices, we need to swap the index with last index
	// erase the relocated index from the tree it is
	IncrementalPrunerMap::Entry relocatedEntry;
	const PxU32 treeRelocatedIndex = mAABBTree[mCurrentTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry) ? mCurrentTree : mLastTree;
	foundEntry = true;
	if(treeRelocatedIndex == mLastTree)
		foundEntry = mAABBTree[mLastTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry);

	if(foundEntry)
	{
		CoreTree& relocatedTree = mAABBTree[treeRelocatedIndex];

		// set the new mapping
		relocatedTree.mapping[poolIndex] = relocatedEntry.second;
		// update the tree indices - swap 
		relocatedTree.tree->fixupTreeIndices(relocatedEntry.second, poolRelocatedLastIndex, poolIndex);
	}

#if PARANOIA_CHECKS
	test();
#endif
	return true;
}

void IncrementalAABBPrunerCore::swapIndex(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex)
{
	// fix the indices, we need to swap the index with last index
	// erase the relocated index from the tre it is
	IncrementalPrunerMap::Entry relocatedEntry;
	const PxU32 treeRelocatedIndex = mAABBTree[mCurrentTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry) ? mCurrentTree : mLastTree;
	bool foundEntry = true;
	if(treeRelocatedIndex == mLastTree)
		foundEntry = mAABBTree[mLastTree].mapping.erase(poolRelocatedLastIndex, relocatedEntry);

	// relocated index is not here
	if(!foundEntry)
		return;

	CoreTree& relocatedTree = mAABBTree[treeRelocatedIndex];

	// set the new mapping
	relocatedTree.mapping[poolIndex] = relocatedEntry.second;
	// update the tree indices - swap 
	relocatedTree.tree->fixupTreeIndices(relocatedEntry.second, poolRelocatedLastIndex, poolIndex);
}

bool IncrementalAABBPrunerCore::updateObject(const PoolIndex poolIndex)
{
	const IncrementalPrunerMap::Entry* entry = mAABBTree[mLastTree].mapping.find(poolIndex);
	const PxU32 treeIndex = entry ? mLastTree : mCurrentTree;
	if(!entry)
		entry = mAABBTree[mCurrentTree].mapping.find(poolIndex);

	// we have not found it
	PX_ASSERT(entry);
	if(!entry)
		return false;

	CoreTree& tree = mAABBTree[treeIndex];
	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = tree.tree->updateFast(entry->second, poolIndex, mPool->getCurrentWorldBoxes(), mChangedLeaves);
	if(!mChangedLeaves.empty() || node != entry->second)
		updateMapping(tree.mapping, poolIndex, node);

#if PARANOIA_CHECKS
	test(false);
#endif

	return true;
}

PxU32 IncrementalAABBPrunerCore::removeMarkedObjects(PxU32 timeStamp)
{
	// early exit is no tree exists
	if(!mAABBTree[mLastTree].tree || !mAABBTree[mLastTree].tree->getNodes())
	{
		PX_ASSERT(mAABBTree[mLastTree].mapping.size() == 0);
		PX_ASSERT(!mAABBTree[mCurrentTree].tree || mAABBTree[mCurrentTree].timeStamp != timeStamp);
		return 0;
	}

	PX_UNUSED(timeStamp);
	PX_ASSERT(timeStamp == mAABBTree[mLastTree].timeStamp);

	// release the last tree
	CoreTree& tree = mAABBTree[mLastTree];
	PxU32 nbObjects = tree.mapping.size();
	tree.mapping.clear();
	tree.timeStamp = 0;

	tree.tree->release();

	return nbObjects;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Query Implementation
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxAgain IncrementalAABBPrunerCore::overlap(const ShapeData& queryVolume, PrunerCallback& pcb) const
{
	PxAgain again = true;

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			switch(queryVolume.getType())
			{
			case PxGeometryType::eBOX:
				{
					if(queryVolume.isOBB())
					{	
						const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
						again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, test, pcb);
					}
					else
					{
						const Gu::AABBAABBTest test(queryVolume.getPrunerInflatedWorldAABB());
						again = AABBTreeOverlap<Gu::AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, test, pcb);
					}
				}
				break;
			case PxGeometryType::eCAPSULE:
				{
					const Gu::Capsule& capsule = queryVolume.getGuCapsule();
					const Gu::CapsuleAABBTest test(	capsule.p1, queryVolume.getPrunerWorldRot33().column0,
													queryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));
					again = AABBTreeOverlap<Gu::CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, test, pcb);
				}
				break;
			case PxGeometryType::eSPHERE:
				{
					const Gu::Sphere& sphere = queryVolume.getGuSphere();
					Gu::SphereAABBTest test(sphere.center, sphere.radius);
					again = AABBTreeOverlap<Gu::SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, test, pcb);
				}
				break;
			case PxGeometryType::eCONVEXMESH:
				{
					const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
					again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, test, pcb);			
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
	}

	return again;
}

PxAgain IncrementalAABBPrunerCore::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PxAgain again = true;

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
			const PxVec3 extents = aabb.getExtents();
			again = AABBTreeRaycast<true, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, aabb.getCenter(), unitDir, inOutDistance, extents, pcb);
		}
	}

	return again;
}

PxAgain IncrementalAABBPrunerCore::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PxAgain again = true;

	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		const CoreTree& tree = mAABBTree[i];
		if(tree.tree && tree.tree->getNodes() && again)
		{
			again = AABBTreeRaycast<false, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool->getObjects(), mPool->getCurrentWorldBoxes(), *tree.tree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
		}
	}
	return again;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IncrementalAABBPrunerCore::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		if(mAABBTree[i].tree)
		{
			mAABBTree[i].tree->shiftOrigin(shift);
		}
	}
}


#include "CmRenderOutput.h"
void IncrementalAABBPrunerCore::visualize(Cm::RenderOutput& out, PxU32 color) const
{
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{
		if(mAABBTree[i].tree && mAABBTree[i].tree->getNodes())
		{
			struct Local
			{
				static void _Draw(const IncrementalAABBTreeNode* root, const IncrementalAABBTreeNode* node, Cm::RenderOutput& out_)
				{
					PxBounds3 bounds;
					V4StoreU(node->mBVMin, &bounds.minimum.x);
					V4StoreU(node->mBVMax, &bounds.maximum.x);
					out_ << Cm::DebugBox(bounds, true);
					if (node->isLeaf())
						return;
					_Draw(root, node->getPos(root), out_);
					_Draw(root, node->getNeg(root), out_);
				}
			};
			out << PxTransform(PxIdentity);
			out << color;
			Local::_Draw(mAABBTree[i].tree->getNodes(), mAABBTree[i].tree->getNodes(), out);
			

			// Render added objects not yet in the tree
			out << PxTransform(PxIdentity);
			out << PxU32(PxDebugColor::eARGB_WHITE);
		}
	}
}

void IncrementalAABBPrunerCore::test(bool chierarcyCheck)
{
	PxU32 maxDepth[NUM_TREES] = { 0, 0 };
	for(PxU32 i = 0; i < NUM_TREES; i++)
	{		
		if(mAABBTree[i].tree)
		{
			if(chierarcyCheck)
				mAABBTree[i].tree->hierarchyCheck(mPool->getCurrentWorldBoxes());
			for (IncrementalPrunerMap::Iterator iter = mAABBTree[i].mapping.getIterator(); !iter.done(); ++iter)
			{
				mAABBTree[i].tree->checkTreeLeaf(iter->second, iter->first);
				PxU32 depth = mAABBTree[i].tree->getTreeLeafDepth(iter->second);
				if(depth > maxDepth[i])
					maxDepth[i] = depth;
			}
		}
	}
}
