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


#include "PsFoundation.h"
#include "SqCompoundPruner.h"
#include "SqIncrementalAABBTree.h"
#include "SqPruningPool.h"
#include "GuAABBTreeQuery.h"
#include "GuSphere.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "GuBounds.h"
#include "GuBVHStructure.h"


using namespace physx;
using namespace Gu;
using namespace Sq;
using namespace Cm;

#define PARANOIA_CHECKS 0

///////////////////////////////////////////////////////////////////////////////////////////////

BVHCompoundPruner::BVHCompoundPruner()
{
	mCompoundTreePool.preallocate(32);
	mMainTreeUpdateMap.resizeUninitialized(32);
	mPoolActorMap.resizeUninitialized(32);
	mChangedLeaves.reserve(32);
}

///////////////////////////////////////////////////////////////////////////////////////////////

BVHCompoundPruner::~BVHCompoundPruner()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::addCompound(PrunerHandle* results, const Gu::BVHStructure& bvhStructure, PrunerCompoundId compoundId, const PxTransform& transform, CompoundFlag::Enum flags, const PrunerPayload* userData)
{
	PX_ASSERT(bvhStructure.getNbBounds());
	
	const PxBounds3 compoundBounds = PxBounds3::transformFast(transform, bvhStructure.getNodes()->mBV);
	const PoolIndex poolIndex = mCompoundTreePool.addCompound(results, bvhStructure, compoundBounds, transform, flags, userData);

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* node = mMainTree.insert(poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
	updateMapping(poolIndex, node);

	mActorPoolMap[compoundId] = poolIndex;
	mPoolActorMap[poolIndex] = compoundId;

#if PARANOIA_CHECKS
	test();
#endif
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node)
{
	// resize mapping if needed
	if(mMainTreeUpdateMap.size() <= poolIndex)
	{
		const PxU32 resizeSize = mMainTreeUpdateMap.size() * 2;
		mMainTreeUpdateMap.resize(resizeSize);
		mPoolActorMap.resize(resizeSize);
	}

	// if a node was split we need to update the node indices and also the sibling indices
	if(!mChangedLeaves.empty())
	{
		if(node && node->isLeaf())
		{
			for(PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				mMainTreeUpdateMap[node->getPrimitives(NULL)[j]] = node;
			}
		}

		for(PxU32 i = 0; i < mChangedLeaves.size(); i++)
		{
			IncrementalAABBTreeNode* changedNode = mChangedLeaves[i];
			PX_ASSERT(changedNode->isLeaf());

			for(PxU32 j = 0; j < changedNode->getNbPrimitives(); j++)
			{
				mMainTreeUpdateMap[changedNode->getPrimitives(NULL)[j]] = changedNode;
			}
		}
	}
	else
	{
		mMainTreeUpdateMap[poolIndex] = node;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::removeCompound(PrunerCompoundId compoundId)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	if(poolIndexEntry)
	{
		const PoolIndex poolIndex = poolIndexEntry->second;
		const PoolIndex poolRelocatedLastIndex = mCompoundTreePool.removeCompound(poolIndex);

		IncrementalAABBTreeNode* node = mMainTree.remove(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds());
		// if node moved to its parent
		if(node && node->isLeaf())
		{
			for (PxU32 j = 0; j < node->getNbPrimitives(); j++)
			{
				const PoolIndex index = node->getPrimitives(NULL)[j];
				mMainTreeUpdateMap[index] = node;
			}
		}	

		// fix indices if we made a swap
		if(poolRelocatedLastIndex != poolIndex)
		{
			mMainTreeUpdateMap[poolIndex] = mMainTreeUpdateMap[poolRelocatedLastIndex];
			mMainTree.fixupTreeIndices(mMainTreeUpdateMap[poolIndex], poolRelocatedLastIndex, poolIndex);

			mActorPoolMap[mPoolActorMap[poolRelocatedLastIndex]] = poolIndex;
			mPoolActorMap[poolIndex] = mPoolActorMap[poolRelocatedLastIndex];
		}

		mActorPoolMap.erase(compoundId);
	}

#if PARANOIA_CHECKS
	test();
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateCompound(PrunerCompoundId compoundId, const PxTransform& transform)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	if(poolIndexEntry)
	{
		const PxU32 poolIndex = poolIndexEntry->second;
		PxBounds3 localBounds;
		const IncrementalAABBTreeNode* node = mCompoundTreePool.getCompoundTrees()[poolIndex].mTree->getNodes();
		mCompoundTreePool.getCompoundTrees()[poolIndex].mGlobalPose = transform;
		V4StoreU(node->mBVMin, &localBounds.minimum.x);
		PX_ALIGN(16, PxVec4) max4;
		V4StoreA(node->mBVMax, &max4.x);
		localBounds.maximum = PxVec3(max4.x, max4.y, max4.z);
		const PxBounds3 compoundBounds = PxBounds3::transformFast(transform, localBounds);
		mCompoundTreePool.getCurrentCompoundBounds()[poolIndex] = compoundBounds;
		mChangedLeaves.clear();
		IncrementalAABBTreeNode* mainTreeNode = mMainTree.update(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
		// we removed node during update, need to update the mapping
		updateMapping(poolIndex, mainTreeNode);
	}

#if PARANOIA_CHECKS
	test();
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::test()
{

	if(mMainTree.getNodes())
	{
		for(PxU32 i = 0; i < mCompoundTreePool.getNbObjects(); i++)
		{
			mMainTree.checkTreeLeaf(mMainTreeUpdateMap[i], i);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::release()
{
}

//////////////////////////////////////////////////////////////////////////
// Queries implementation
//////////////////////////////////////////////////////////////////////////
// Raycast/sweeps callback for main AABB tree
template<bool tInflate>
struct MainTreeRaycastCompoundPrunerCallback
{
	MainTreeRaycastCompoundPrunerCallback(const PxVec3& origin, const PxVec3& unitDir, const PxVec3& extent, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: mOrigin(origin), mUnitDir(unitDir), mExtent(extent), mPrunerCallback(prunerCallback), mQueryFlags(flags)
	{
	}

	virtual ~MainTreeRaycastCompoundPrunerCallback() {}

	virtual PxAgain invoke(PxReal& distance, const CompoundTree& compoundTree)
	{
		if(!(compoundTree.mFlags &  PxU32(mQueryFlags)) || !compoundTree.mTree->getNodes())
			return true;

		// transfer to actor local space
		const PxVec3 localOrigin = compoundTree.mGlobalPose.transformInv(mOrigin);
		const PxVec3 localDir = compoundTree.mGlobalPose.q.rotateInv(mUnitDir);
		PxVec3 localExtent = mExtent;

		if(tInflate)
		{
			PxBounds3 wBounds = PxBounds3::centerExtents(mOrigin, mExtent);
			PxBounds3 localBounds = PxBounds3::transformSafe(compoundTree.mGlobalPose.getInverse(), wBounds);
			localExtent = localBounds.getExtents();
		}

		// raycast the merged tree
		return AABBTreeRaycast<tInflate, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()
			(compoundTree.mPruningPool->getObjects(), compoundTree.mPruningPool->getCurrentWorldBoxes(), *compoundTree.mTree, localOrigin, localDir, distance, localExtent, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeRaycastCompoundPrunerCallback)

private:
	const PxVec3&		mOrigin;
	const PxVec3&		mUnitDir;	
	const PxVec3&		mExtent;
	PrunerCallback&		mPrunerCallback;
	PxQueryFlags		mQueryFlags;
};

//////////////////////////////////////////////////////////////////////////
// raycast against the compound pruner
PxAgain BVHCompoundPruner::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& prunerCallback, PxQueryFlags flags) const
{
	PxAgain again = true;	

	// search the main tree if there are nodes
	if(mMainTree.getNodes())
	{
		const PxVec3 extent(0.0f);
		// main tree callback
		MainTreeRaycastCompoundPrunerCallback<false> pcb(origin, unitDir, extent, prunerCallback, flags);
		// traverse the main tree
		again = AABBTreeRaycast<false, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeRaycastCompoundPrunerCallback<false> >()
			(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, origin, unitDir, inOutDistance, extent, pcb);
	}

	return again;
}

//////////////////////////////////////////////////////////////////////////
// overlap main tree callback
// A.B. templated version is complicated due to test transformations, will do a callback per primitive
struct MainTreeOverlapCompoundPrunerCallback
{
	MainTreeOverlapCompoundPrunerCallback(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: mQueryVolume(queryVolume), mPrunerCallback(prunerCallback), mQueryFlags(flags)
	{
	}

	virtual ~MainTreeOverlapCompoundPrunerCallback() {}

	PX_NOCOPY(MainTreeOverlapCompoundPrunerCallback)

protected:
	const Gu::ShapeData&	mQueryVolume;	
	PrunerCallback&			mPrunerCallback;
	PxQueryFlags			mQueryFlags;
};

// OBB
struct MainTreeOBBOverlapCompoundPrunerCallback: public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeOBBOverlapCompoundPrunerCallback(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags) {}
	virtual PxAgain invoke(PxReal& , const CompoundTree& compoundTree)
	{
		if(!(compoundTree.mFlags & PxU32(mQueryFlags)) || !compoundTree.mTree->getNodes())
			return true;

		const PxVec3 localPos = compoundTree.mGlobalPose.transformInv(mQueryVolume.getPrunerWorldPos());
		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const PxMat33 localRot = transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33();

		const Gu::OBBAABBTest localTest(localPos, localRot, mQueryVolume.getPrunerBoxGeomExtentsInflated());		
		// overlap the compound local tree
		return AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()
			(compoundTree.mPruningPool->getObjects(), compoundTree.mPruningPool->getCurrentWorldBoxes(), *compoundTree.mTree, localTest, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeOBBOverlapCompoundPrunerCallback)
};

// AABB
struct MainTreeAABBOverlapCompoundPrunerCallback: public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeAABBOverlapCompoundPrunerCallback(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags) {}

	virtual PxAgain invoke(PxReal& , const CompoundTree& compoundTree)
	{
		if(!(compoundTree.mFlags & PxU32(mQueryFlags)) || !compoundTree.mTree->getNodes())
			return true;

		const PxVec3 localPos = compoundTree.mGlobalPose.transformInv(mQueryVolume.getPrunerWorldPos());
		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const PxMat33 localRot = transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33();

		// A.B. we dont have the AABB in local space, either we test OBB local space or
		// we retest the AABB with the worldSpace AABB of the local tree???
		const Gu::OBBAABBTest localTest(localPos, localRot, mQueryVolume.getPrunerBoxGeomExtentsInflated());		
		// overlap the compound local tree
		return AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()
			(compoundTree.mPruningPool->getObjects(), compoundTree.mPruningPool->getCurrentWorldBoxes(), *compoundTree.mTree, localTest, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeAABBOverlapCompoundPrunerCallback)
};

// Capsule
struct MainTreeCapsuleOverlapCompoundPrunerCallback: public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeCapsuleOverlapCompoundPrunerCallback(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags) {}

	virtual PxAgain invoke(PxReal& , const CompoundTree& compoundTree)
	{
		if(!(compoundTree.mFlags & PxU32(mQueryFlags)) || !compoundTree.mTree->getNodes())
			return true;

		const PxMat33 transfMat(compoundTree.mGlobalPose.q);
		const Gu::Capsule& capsule = mQueryVolume.getGuCapsule();
		const Gu::CapsuleAABBTest localTest(
			compoundTree.mGlobalPose.transformInv(capsule.p1), 
			transfMat.getTranspose()*mQueryVolume.getPrunerWorldRot33().column0,
			mQueryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));

		// overlap the compound local tree
		return AABBTreeOverlap<Gu::CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()
			(compoundTree.mPruningPool->getObjects(), compoundTree.mPruningPool->getCurrentWorldBoxes(), *compoundTree.mTree, localTest, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeCapsuleOverlapCompoundPrunerCallback)
};

// Sphere
struct MainTreeSphereOverlapCompoundPrunerCallback: public MainTreeOverlapCompoundPrunerCallback
{
	MainTreeSphereOverlapCompoundPrunerCallback(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags)
		: MainTreeOverlapCompoundPrunerCallback(queryVolume, prunerCallback, flags) {}

	virtual PxAgain invoke(PxReal& , const CompoundTree& compoundTree)
	{
		if(!(compoundTree.mFlags & PxU32(mQueryFlags)) || !compoundTree.mTree->getNodes())
			return true;

		const Gu::Sphere& sphere = mQueryVolume.getGuSphere();
		Gu::SphereAABBTest localTest(compoundTree.mGlobalPose.transformInv(sphere.center), sphere.radius);

		// overlap the compound local tree
		return AABBTreeOverlap<Gu::SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, PrunerPayload, PrunerCallback>()
			(compoundTree.mPruningPool->getObjects(), compoundTree.mPruningPool->getCurrentWorldBoxes(), *compoundTree.mTree, localTest, mPrunerCallback);
	}

	PX_NOCOPY(MainTreeSphereOverlapCompoundPrunerCallback)
};


//////////////////////////////////////////////////////////////////////////
// overlap implementation
PxAgain BVHCompoundPruner::overlap(const Gu::ShapeData& queryVolume, PrunerCallback& prunerCallback, PxQueryFlags flags) const
{
	PxAgain again = true;

	if(mMainTree.getNodes())
	{
		switch (queryVolume.getType())
		{
		case PxGeometryType::eBOX:
		{
			if(queryVolume.isOBB())
			{
				const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
				MainTreeOBBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags);
				again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeOBBOverlapCompoundPrunerCallback>()
					(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, test, pcb);
			}
			else
			{
				const Gu::AABBAABBTest test(queryVolume.getPrunerInflatedWorldAABB());
				MainTreeAABBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags);
				again = AABBTreeOverlap<Gu::AABBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeAABBOverlapCompoundPrunerCallback>()
					(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, test, pcb);				
			}
		}
		break;
		case PxGeometryType::eCAPSULE:
		{
			const Gu::Capsule& capsule = queryVolume.getGuCapsule();
			const Gu::CapsuleAABBTest test(capsule.p1, queryVolume.getPrunerWorldRot33().column0,
				queryVolume.getCapsuleHalfHeight()*2.0f, PxVec3(capsule.radius*SQ_PRUNER_INFLATION));
			MainTreeCapsuleOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags);			
			again = AABBTreeOverlap<Gu::CapsuleAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeCapsuleOverlapCompoundPrunerCallback >()
				(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, test, pcb);				
		}
		break;
		case PxGeometryType::eSPHERE:
		{
			const Gu::Sphere& sphere = queryVolume.getGuSphere();
			Gu::SphereAABBTest test(sphere.center, sphere.radius);
			MainTreeSphereOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags);
			again = AABBTreeOverlap<Gu::SphereAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeSphereOverlapCompoundPrunerCallback>()
				(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, test, pcb);				
		}
		break;
		case PxGeometryType::eCONVEXMESH:
		{
			const Gu::OBBAABBTest test(queryVolume.getPrunerWorldPos(), queryVolume.getPrunerWorldRot33(), queryVolume.getPrunerBoxGeomExtentsInflated());
			MainTreeOBBOverlapCompoundPrunerCallback pcb(queryVolume, prunerCallback, flags);			
			again = AABBTreeOverlap<Gu::OBBAABBTest, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeOBBOverlapCompoundPrunerCallback>()
				(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, test, pcb);				
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

///////////////////////////////////////////////////////////////////////////////////////////////

PxAgain BVHCompoundPruner::sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback& prunerCallback, PxQueryFlags flags) const
{
	PxAgain again = true;

	if(mMainTree.getNodes())
	{
		const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
		const PxVec3 extents = aabb.getExtents();
		const PxVec3 center = aabb.getCenter();
		MainTreeRaycastCompoundPrunerCallback<true> pcb(center, unitDir, extents, prunerCallback, flags);
		again = AABBTreeRaycast<true, IncrementalAABBTree, IncrementalAABBTreeNode, CompoundTree, MainTreeRaycastCompoundPrunerCallback<true> >()
			(mCompoundTreePool.getCompoundTrees(), mCompoundTreePool.getCurrentCompoundBounds(), mMainTree, center, unitDir, inOutDistance, extents, pcb);
	}
	return again;
}

///////////////////////////////////////////////////////////////////////////////////////////////

const PrunerPayload& BVHCompoundPruner::getPayload(PrunerHandle handle, PrunerCompoundId compoundId) const
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	return mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].mPruningPool->getPayload(handle);
}

///////////////////////////////////////////////////////////////////////////////////////////////

const PrunerPayload& BVHCompoundPruner::getPayload(PrunerHandle handle, PrunerCompoundId compoundId, PxBounds3*& bounds) const
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);

	return mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].mPruningPool->getPayload(handle, bounds);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const PrunerHandle handle)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return;

	mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].updateObjectAfterManualBoundsUpdates(handle);

	const PxU32 poolIndex = poolIndexEntry->second;
	updateMainTreeNode(poolIndex);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::removeObject(PrunerCompoundId compoundId, const PrunerHandle handle)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return;

	const PxU32 poolIndex = poolIndexEntry->second;

	mCompoundTreePool.getCompoundTrees()[poolIndex].removeObject(handle);

	// edge case, we removed all objects for the compound tree, we need to remove it now completely
	if(!mCompoundTreePool.getCompoundTrees()[poolIndex].mTree->getNodes())
	{
		removeCompound(compoundId);
	}
	else
	{
		updateMainTreeNode(poolIndex);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

bool BVHCompoundPruner::addObject(PrunerCompoundId compoundId, PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload userData)
{
	const ActorIdPoolIndexMap::Entry* poolIndexEntry = mActorPoolMap.find(compoundId);
	PX_ASSERT(poolIndexEntry);
	if(!poolIndexEntry)
		return false;

	mCompoundTreePool.getCompoundTrees()[poolIndexEntry->second].addObject(result, bounds, userData);

	const PxU32 poolIndex = poolIndexEntry->second;
	updateMainTreeNode(poolIndex);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::updateMainTreeNode(PoolIndex poolIndex)
{
	PxBounds3 localBounds;
	const IncrementalAABBTreeNode* node = mCompoundTreePool.getCompoundTrees()[poolIndex].mTree->getNodes();
	V4StoreU(node->mBVMin, &localBounds.minimum.x);
	PX_ALIGN(16, PxVec4) max4;
	V4StoreA(node->mBVMax, &max4.x);
	localBounds.maximum = PxVec3(max4.x, max4.y, max4.z);
	const PxBounds3 compoundBounds = PxBounds3::transformFast(mCompoundTreePool.getCompoundTrees()[poolIndex].mGlobalPose, localBounds);
	mCompoundTreePool.getCurrentCompoundBounds()[poolIndex] = compoundBounds;

	mChangedLeaves.clear();
	IncrementalAABBTreeNode* mainTreeNode = mMainTree.update(mMainTreeUpdateMap[poolIndex], poolIndex, mCompoundTreePool.getCurrentCompoundBounds(), mChangedLeaves);
	// we removed node during update, need to update the mapping
	updateMapping(poolIndex, mainTreeNode);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::shiftOrigin(const PxVec3& shift)
{
	mCompoundTreePool.shiftOrigin(shift);

	mMainTree.shiftOrigin(shift);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void BVHCompoundPruner::visualize(Cm::RenderOutput&, PxU32) const
{
}

///////////////////////////////////////////////////////////////////////////////////////////////

