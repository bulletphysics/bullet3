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

#include "common/PxProfileZone.h"
#include "SqIncrementalAABBPruner.h"
#include "SqIncrementalAABBTree.h"
#include "SqAABBTree.h"
#include "GuAABBTreeQuery.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuBounds.h"
#include "PsBitUtils.h"

using namespace physx;
using namespace Gu;
using namespace Sq;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: currently limited to 15 max
#define NB_OBJECTS_PER_NODE	4

#define PARANOIA_CHECKS 0

IncrementalAABBPruner::IncrementalAABBPruner(PxU32 sceneLimit, PxU64 contextID) :
	mAABBTree			(NULL),
	mContextID			(contextID)
{
	PX_UNUSED(mContextID);
	mMapping.resizeUninitialized(sceneLimit);
	mPool.preallocate(sceneLimit);

	mChangedLeaves.reserve(32);
}

IncrementalAABBPruner::~IncrementalAABBPruner()
{
	release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Add, Remove, Update methods
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IncrementalAABBPruner::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* payload, PxU32 count, bool )
{
	PX_PROFILE_ZONE("SceneQuery.prunerAddObjects", mContextID);

	if(!count)
		return true;
	
	const PxU32 valid = mPool.addObjects(results, bounds, payload, count);

	if(mAABBTree)
	{
		for(PxU32 i=0;i<valid;i++)
		{
			const PrunerHandle& handle = results[i];
			const PoolIndex poolIndex = mPool.getIndex(handle);
			mChangedLeaves.clear();
			IncrementalAABBTreeNode* node = mAABBTree->insert(poolIndex, mPool.getCurrentWorldBoxes(), mChangedLeaves);
			updateMapping(poolIndex, node);
		}

	#if PARANOIA_CHECKS
		test();
	#endif
	}

	return valid==count;
}

void IncrementalAABBPruner::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// resize mapping if needed
	if(mMapping.size() <= poolIndex)
	{
		mMapping.resize(mMapping.size() * 2);
	}

	// if a node was split we need to update the node indices and also the sibling indices
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				mMapping[node->getPrimitives(NULL)[j]] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				mMapping[changedNode->getPrimitives(NULL)[j]] = changedNode;
			}
		}
	}
	else
	{
		mMapping[poolIndex] = node;
	}
}

void IncrementalAABBPruner::updateObjectsAfterManualBoundsUpdates(const PrunerHandle* handles, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mContextID);

	if(!count || !mAABBTree)
		return;

	const PxBounds3* newBounds = mPool.getCurrentWorldBoxes();	
	for(PxU32 i=0; i<count; i++)
	{		
		const PrunerHandle h = handles[i];
		const PoolIndex poolIndex = mPool.getIndex(h);
		mChangedLeaves.clear();
		IncrementalAABBTreeNode* node = mAABBTree->update(mMapping[poolIndex], poolIndex, newBounds, mChangedLeaves);
		// we removed node during update, need to update the mapping		
		updateMapping(poolIndex, node);
	}	

#if PARANOIA_CHECKS
	test();
#endif

}

void IncrementalAABBPruner::updateObjectsAndInflateBounds(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* newBounds, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerUpdateObjects", mContextID);

	if(!count)
		return;
	
	mPool.updateObjectsAndInflateBounds(handles, indices, newBounds, count);

	if(!mAABBTree)
		return;

	const PxBounds3* poolBounds = mPool.getCurrentWorldBoxes();	
	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];
		const PoolIndex poolIndex = mPool.getIndex(h);
		mChangedLeaves.clear();
		IncrementalAABBTreeNode* node = mAABBTree->update(mMapping[poolIndex], poolIndex, poolBounds, mChangedLeaves);
		// we removed node during update, need to update the mapping
		updateMapping(poolIndex, node);
	}

#if PARANOIA_CHECKS
	test();
#endif

}

void IncrementalAABBPruner::removeObjects(const PrunerHandle* handles, PxU32 count)
{
	PX_PROFILE_ZONE("SceneQuery.prunerRemoveObjects", mContextID);

	if(!count)
		return;

	for(PxU32 i=0; i<count; i++)
	{
		const PrunerHandle h = handles[i];		
		const PoolIndex poolIndex = mPool.getIndex(h); // save the pool index for removed object
		const PoolIndex poolRelocatedLastIndex = mPool.removeObject(h); // save the lastIndex returned by removeObject

		if(mAABBTree)
		{
			IncrementalAABBTreeNode* node = mAABBTree->remove(mMapping[poolIndex], poolIndex, mPool.getCurrentWorldBoxes());
			// if node moved to its parent
			if (node && node->isLeaf())
			{
				for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
				{
					const PoolIndex index = node->getPrimitives(NULL)[j];
					mMapping[index] = node;
				}
			}
			mMapping[poolIndex] = mMapping[poolRelocatedLastIndex];
			// fix indices if we made a swap
			if(poolRelocatedLastIndex != poolIndex)
				mAABBTree->fixupTreeIndices(mMapping[poolIndex], poolRelocatedLastIndex, poolIndex);

			if(!mAABBTree->getNodes())
			{
				release();
			}
		}
	}

#if PARANOIA_CHECKS
	test();
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Query Implementation
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxAgain IncrementalAABBPruner::overlap(const ShapeData& queryVolume, PrunerCallback& pcb) const
{
	PxAgain again = true;

	if(mAABBTree && mAABBTree->getNodes())
	{
		switch(queryVolume.getType())
		{
		case PxGeometryType::eBOX:
			{
				if(queryVolume.isOBB())
				{	
					const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
					again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
				}
				else
				{
					const Gu::AABBAABBTest test(queryVolume.getPrunerInflatedWorldAABB());
					again = AABBTreeOverlap<Gu::AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
				}
			}
			break;
		case PxGeometryType::eCAPSULE:
			{
				const Gu::Capsule& capsule = queryVolume.getGuCapsule();
				const Gu::CapsuleAABBTest test(	capsule.p1, queryVolume.getPrunerWorldRot33().column0,
												queryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));
				again = AABBTreeOverlap<Gu::CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
			}
			break;
		case PxGeometryType::eSPHERE:
			{
				const Gu::Sphere& sphere = queryVolume.getGuSphere();
				Gu::SphereAABBTest test(sphere.center, sphere.radius);
				again = AABBTreeOverlap<Gu::SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);
			}
			break;
		case PxGeometryType::eCONVEXMESH:
			{
				const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
				again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, test, pcb);			
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

PxAgain IncrementalAABBPruner::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PxAgain again = true;

	if(mAABBTree && mAABBTree->getNodes())
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		const PxVec3 extents = aabb.getExtents();
		again = AABBTreeRaycast<true, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, aabb.getCenter(), unitDir, inOutDistance, extents, pcb);
	}

	return again;
}

PxAgain IncrementalAABBPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& pcb) const
{
	PxAgain again = true;

	if(mAABBTree && mAABBTree->getNodes())
		again = AABBTreeRaycast<false, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()(mPool.getObjects(), mPool.getCurrentWorldBoxes(), *mAABBTree, origin, unitDir, inOutDistance, PxVec3(0.0f), pcb);
		
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

void IncrementalAABBPruner::purge()
{
	release();	
} 

void IncrementalAABBPruner::setRebuildRateHint(PxU32 ) 
{ 
}

bool IncrementalAABBPruner::buildStep(bool )
{
	return true;
}

bool IncrementalAABBPruner::prepareBuild()
{
	return false;
}

// Commit either performs a refit if background rebuild is not yet finished
// or swaps the current tree for the second tree rebuilt in the background
void IncrementalAABBPruner::commit()
{
	PX_PROFILE_ZONE("SceneQuery.prunerCommit", mContextID);

	if (!mAABBTree)
	{
		fullRebuildAABBTree();
		return;
	}
}

void IncrementalAABBPruner::fullRebuildAABBTree()
{
	// Don't bother building an AABB-tree if there isn't a single static object
	const PxU32 nbObjects = mPool.getNbActiveObjects();
	if (!nbObjects)
		return;

	const PxU32 indicesSize = Ps::nextPowerOfTwo(nbObjects);
	if(indicesSize > mMapping.size())
	{
		mMapping.resizeUninitialized(indicesSize);
	}
	
	// copy the temp optimized tree into the new incremental tree
	mAABBTree = PX_NEW(IncrementalAABBTree)();

	AABBTreeBuildParams TB;
	TB.mNbPrimitives = nbObjects;
	TB.mAABBArray = mPool.getCurrentWorldBoxes();
	TB.mLimit = NB_OBJECTS_PER_NODE;
	mAABBTree->build(TB, mMapping);

#if PARANOIA_CHECKS
	test();
#endif
}

void IncrementalAABBPruner::shiftOrigin(const PxVec3& shift)
{
	mPool.shiftOrigin(shift);

	if(mAABBTree)
		mAABBTree->shiftOrigin(shift);
}

#include "CmRenderOutput.h"
void IncrementalAABBPruner::visualize(Cm::RenderOutput& out, PxU32 color) const
{
	// getAABBTree() asserts when pruner is dirty. NpScene::visualization() does not enforce flushUpdate. see DE7834
	const IncrementalAABBTree* tree = mAABBTree;

	if(tree && tree->getNodes())
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
		Local::_Draw(tree->getNodes(), tree->getNodes(), out);
	}

	// Render added objects not yet in the tree
	out << PxTransform(PxIdentity);
	out << PxU32(PxDebugColor::eARGB_WHITE);
}

void IncrementalAABBPruner::release() // this can be called from purge()
{
	if (mAABBTree)
	{
		PX_DELETE(mAABBTree);
		mAABBTree = NULL;
	}
}

void IncrementalAABBPruner::test()
{
	if(mAABBTree)
	{
		mAABBTree->hierarchyCheck(mPool.getNbActiveObjects(), mPool.getCurrentWorldBoxes());
		for(PxU32 i = 0; i < mPool.getNbActiveObjects(); i++)
		{
			mAABBTree->checkTreeLeaf(mMapping[i], i);
		}
	}
}

void IncrementalAABBPruner::merge(const void* )
{
	//const AABBPrunerMergeData& pruningStructure = *reinterpret_cast<const AABBPrunerMergeData*> (mergeParams);

	//if(mAABBTree)
	//{
	//	// index in pruning pool, where new objects were added
	//	const PxU32 pruningPoolIndex = mPool.getNbActiveObjects() - pruningStructure.mNbObjects;

	//	// create tree from given nodes and indices
	//	AABBTreeMergeData aabbTreeMergeParams(pruningStructure.mNbNodes, pruningStructure.mAABBTreeNodes,
	//		pruningStructure.mNbObjects, pruningStructure.mAABBTreeIndices, pruningPoolIndex);

	//	if (!mIncrementalRebuild)
	//	{
	//		// merge tree directly
	//		mAABBTree->mergeTree(aabbTreeMergeParams);		
	//	}
	//	else
	//	{
	//		mBucketPruner.addTree(aabbTreeMergeParams, mTimeStamp);
	//	}
	//}
}
