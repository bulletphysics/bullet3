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

#include "SqSceneQueryManager.h"
#include "SqAABBPruner.h"
#include "SqIncrementalAABBPruner.h"
#include "SqBucketPruner.h"
#include "SqPrunerMergeData.h"
#include "SqBounds.h"
#include "NpBatchQuery.h"
#include "PxFiltering.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulationLink.h"
#include "CmTransformUtils.h"
#include "PsAllocator.h"
#include "PxSceneDesc.h"
#include "ScBodyCore.h"
#include "SqPruner.h"
#include "SqCompoundPruner.h"
#include "GuBounds.h"
#include "NpShape.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Sq;
using namespace Sc;

PrunerExt::PrunerExt() :
	mPruner		(NULL),
	mDirtyList	(PX_DEBUG_EXP("SQmDirtyList")),
	mPrunerType	(PxPruningStructureType::eLAST),
	mTimestamp	(0xffffffff)
{
}

PrunerExt::~PrunerExt()
{
	PX_DELETE_AND_RESET(mPruner);
}

void PrunerExt::init(PxPruningStructureType::Enum type, PxU64 contextID, PxU32 )
{
	if(0)	// PT: to force testing the bucket pruner
	{
		mPrunerType = PxPruningStructureType::eNONE;
		mTimestamp	= 0;
		mPruner = PX_NEW(BucketPruner);
		return;
	}

	mPrunerType = type;
	mTimestamp	= 0;
	Pruner* pruner = NULL;
	switch(type)
	{
		case PxPruningStructureType::eNONE:					{ pruner = PX_NEW(BucketPruner);					break;	}
		case PxPruningStructureType::eDYNAMIC_AABB_TREE:	{ pruner = PX_NEW(AABBPruner)(true, contextID);		break;	}
		case PxPruningStructureType::eSTATIC_AABB_TREE:		{ pruner = PX_NEW(AABBPruner)(false, contextID);	break;	}
		case PxPruningStructureType::eLAST:					break;
	}
	mPruner = pruner;
}

void PrunerExt::preallocate(PxU32 nbShapes)
{
	if(nbShapes > mDirtyMap.size())
		mDirtyMap.resize(nbShapes);

	if(mPruner)
		mPruner->preallocate(nbShapes);
}

void PrunerExt::flushMemory()
{
	if(!mDirtyList.size())
		mDirtyList.reset();

	// PT: TODO: flush bitmap here

	// PT: TODO: flush pruner here?
}

void PrunerExt::flushShapes(PxU32 index)
{
	const PxU32 numDirtyList = mDirtyList.size();
	if(!numDirtyList)
		return;
	const PrunerHandle* const prunerHandles = mDirtyList.begin();

	const ComputeBoundsFunc func = gComputeBoundsTable[index];

	for(PxU32 i=0; i<numDirtyList; i++)
	{
		const PrunerHandle handle = prunerHandles[i];
		mDirtyMap.reset(handle);

		// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
		// to take advantage of batching.
		PxBounds3* bounds;
		const PrunerPayload& pp = mPruner->getPayload(handle, bounds);
		(func)(*bounds, *(reinterpret_cast<Scb::Shape*>(pp.data[0])), *(reinterpret_cast<Scb::Actor*>(pp.data[1])));	//PAYLOAD
	}
	// PT: batch update happens after the loop instead of once per loop iteration
	mPruner->updateObjectsAfterManualBoundsUpdates(prunerHandles, numDirtyList);
	mTimestamp += numDirtyList;
	mDirtyList.clear();
}

// PT: TODO: re-inline this
void PrunerExt::addToDirtyList(PrunerHandle handle)
{
	Cm::BitMap& dirtyMap = mDirtyMap;
	if(!dirtyMap.test(handle))
	{
		dirtyMap.set(handle);
		mDirtyList.pushBack(handle);
		mTimestamp++;
	}
}

// PT: TODO: re-inline this
Ps::IntBool PrunerExt::isDirty(PrunerHandle handle) const
{
	return mDirtyMap.test(handle);
}

// PT: TODO: re-inline this
void PrunerExt::removeFromDirtyList(PrunerHandle handle)
{
	Cm::BitMap& dirtyMap = mDirtyMap;
	if(dirtyMap.test(handle))
	{
		dirtyMap.reset(handle);
		mDirtyList.findAndReplaceWithLast(handle);
	}
}

// PT: TODO: re-inline this
void PrunerExt::growDirtyList(PrunerHandle handle)
{
	// pruners must either provide indices in order or reuse existing indices, so this 'if' is enough to ensure we have space for the new handle
	// PT: TODO: fix this. There is just no need for any of it. The pruning pool itself could support the feature for free, similar to what we do
	// in MBP. There would be no need for the bitmap or the dirty list array. However doing this through the virtual interface would be clumsy,
	// adding the cost of virtual calls for very cheap & simple operations. It would be a lot easier to drop it and go back to what we had before.

	Cm::BitMap& dirtyMap = mDirtyMap;
	if(dirtyMap.size() <= handle)
		dirtyMap.resize(PxMax<PxU32>(dirtyMap.size() * 2, 1024));
	PX_ASSERT(handle<dirtyMap.size());
	dirtyMap.reset(handle);
}

///////////////////////////////////////////////////////////////////////////////

CompoundPrunerExt::CompoundPrunerExt() :
	mPruner		(NULL)	
{
}

CompoundPrunerExt::~CompoundPrunerExt()
{
	PX_DELETE_AND_RESET(mPruner);
}

void CompoundPrunerExt::preallocate(PxU32 nbShapes)
{
	if(nbShapes > mDirtyList.size())
		mDirtyList.reserve(nbShapes);
}

void CompoundPrunerExt::flushMemory()
{
	if(!mDirtyList.size())
		mDirtyList.clear();
}

void CompoundPrunerExt::flushShapes()
{
	const PxU32 numDirtyList = mDirtyList.size();
	if(!numDirtyList)
		return;

	const CompoundPair* const compoundPairs = mDirtyList.getEntries();

	for(PxU32 i=0; i<numDirtyList; i++)
	{
		const PrunerHandle handle = compoundPairs[i].second;
		const PrunerCompoundId compoundId = compoundPairs[i].first;		

		// PT: we compute the new bounds and store them directly in the pruner structure to avoid copies. We delay the updateObjects() call
		// to take advantage of batching.
		PxBounds3* bounds;
		const PrunerPayload& pp = mPruner->getPayload(handle, compoundId, bounds);
		const Scb::Shape& scbShape = *reinterpret_cast<Scb::Shape*>(pp.data[0]);

		const PxTransform& shape2Actor = scbShape.getShape2Actor();		
		Gu::computeBounds(*bounds, scbShape.getGeometry(), shape2Actor, 0.0f, NULL, SQ_PRUNER_INFLATION);

		// A.B. not very effective, we might do better here
		mPruner->updateObjectAfterManualBoundsUpdates(compoundId, handle);
	}
		
	mDirtyList.clear();
}

// PT: TODO: re-inline this
void CompoundPrunerExt::addToDirtyList(PrunerCompoundId compoundId, PrunerHandle handle)
{
	mDirtyList.insert(CompoundPair(compoundId, handle));
}

// PT: TODO: re-inline this
Ps::IntBool CompoundPrunerExt::isDirty(PrunerCompoundId compoundId, PrunerHandle handle) const
{
	return mDirtyList.contains(CompoundPair(compoundId, handle));
}

// PT: TODO: re-inline this
void CompoundPrunerExt::removeFromDirtyList(PrunerCompoundId compoundId, PrunerHandle handle)
{
	mDirtyList.erase(CompoundPair(compoundId, handle));
}

///////////////////////////////////////////////////////////////////////////////

SceneQueryManager::SceneQueryManager(	Scb::Scene& scene, PxPruningStructureType::Enum staticStructure, 
										PxPruningStructureType::Enum dynamicStructure, PxU32 dynamicTreeRebuildRateHint,
										const PxSceneLimits& limits) :
	mScene			(scene)	
{
	mPrunerExt[PruningIndex::eSTATIC].init(staticStructure, scene.getContextId(), limits.maxNbStaticShapes ? limits.maxNbStaticShapes : 1024);
	mPrunerExt[PruningIndex::eDYNAMIC].init(dynamicStructure, scene.getContextId(), limits.maxNbDynamicShapes ? limits.maxNbDynamicShapes : 1024);

	setDynamicTreeRebuildRateHint(dynamicTreeRebuildRateHint);

	preallocate(limits.maxNbStaticShapes, limits.maxNbDynamicShapes);

	mDynamicBoundsSync.mPruner = mPrunerExt[PruningIndex::eDYNAMIC].pruner();
	mDynamicBoundsSync.mTimestamp = &mPrunerExt[PruningIndex::eDYNAMIC].mTimestamp;

	mCompoundPrunerExt.mPruner = PX_NEW(BVHCompoundPruner);
	mCompoundPrunerExt.preallocate(32);

	mPrunerNeedsUpdating = false;
}

SceneQueryManager::~SceneQueryManager()
{
}

void SceneQueryManager::flushMemory()
{
	for(PxU32 i=0;i<PruningIndex::eCOUNT;i++)
		mPrunerExt[i].flushMemory();

	mCompoundPrunerExt.flushMemory();
}

void SceneQueryManager::markForUpdate(PrunerCompoundId compoundId, PrunerData data)
{ 
	mPrunerNeedsUpdating = true;
	const PxU32 index = getPrunerIndex(data);
	const PrunerHandle handle = getPrunerHandle(data);

	if(compoundId == INVALID_PRUNERHANDLE)
		mPrunerExt[index].addToDirtyList(handle);
	else
	{
		// A.B. As there can be static actors in the compounds and they could have moved, 
		// then for exmplae CCT does need to know that something might have changed (timeStamp check), so therefore the invalidateTimestamp() here
		mPrunerExt[index].invalidateTimestamp();
		mCompoundPrunerExt.addToDirtyList(compoundId, handle);
	}
}

void SceneQueryManager::preallocate(PxU32 staticShapes, PxU32 dynamicShapes)
{
	mPrunerExt[PruningIndex::eSTATIC].preallocate(staticShapes);
	mPrunerExt[PruningIndex::eDYNAMIC].preallocate(dynamicShapes);
}

// PT: TODO: consider passing the payload directly to this function, for a cleaner interface that makes more sense.
// But the shape & actor pointers are used directly in the function, so this whole thing is fishy.
PrunerData SceneQueryManager::addPrunerShape(const Scb::Shape& scbShape, const Scb::Actor& scbActor, bool dynamic, PrunerCompoundId compoundId, const PxBounds3* bounds, bool hasPrunerStructure)
{
	mPrunerNeedsUpdating = true;

	PrunerPayload pp;
	pp.data[0] = size_t(&scbShape);	//PAYLOAD
	pp.data[1] = size_t(&scbActor);	//PAYLOAD

	const PxU32 index = PxU32(dynamic);
	PrunerHandle handle;

	mPrunerExt[index].invalidateTimestamp();
	if(compoundId == INVALID_PRUNERHANDLE)
	{
		PxBounds3 b;
		if(bounds)
			inflateBounds(b, *bounds);
		else
			(gComputeBoundsTable[dynamic])(b, scbShape, scbActor);

		PX_ASSERT(mPrunerExt[index].pruner());
		mPrunerExt[index].pruner()->addObjects(&handle, &b, &pp, 1, hasPrunerStructure);		

		mPrunerExt[index].growDirtyList(handle);
	}
	else
	{
		PxBounds3 b;				
		const PxTransform& shape2Actor = scbShape.getShape2Actor();		
		Gu::computeBounds(b, scbShape.getGeometry(), shape2Actor, 0.0f, NULL, SQ_PRUNER_INFLATION);

		PX_ASSERT(mCompoundPrunerExt.pruner());
		mCompoundPrunerExt.pruner()->addObject(compoundId, handle, b, pp);
	}

	return createPrunerData(index, handle);
}

const PrunerPayload& SceneQueryManager::getPayload(PrunerCompoundId compoundId, PrunerData data) const
{	
	const PxU32 index = getPrunerIndex(data);
	const PrunerHandle handle = getPrunerHandle(data);

	if(compoundId == INVALID_PRUNERHANDLE)
		return mPrunerExt[index].pruner()->getPayload(handle);
	else
		return mCompoundPrunerExt.pruner()->getPayload(handle, compoundId);
}

void SceneQueryManager::removePrunerShape(PrunerCompoundId compoundId, PrunerData data)
{
	mPrunerNeedsUpdating = true;
	const PxU32 index = getPrunerIndex(data);
	const PrunerHandle handle = getPrunerHandle(data);

	mPrunerExt[index].invalidateTimestamp();
	if(compoundId == INVALID_PRUNERHANDLE)
	{
		PX_ASSERT(mPrunerExt[index].pruner());

		mPrunerExt[index].removeFromDirtyList(handle);
		
		mPrunerExt[index].pruner()->removeObjects(&handle, 1);
	}
	else
	{
		mCompoundPrunerExt.removeFromDirtyList(compoundId, handle);
		mCompoundPrunerExt.pruner()->removeObject(compoundId, handle);
	}
}

void SceneQueryManager::setDynamicTreeRebuildRateHint(PxU32 rebuildRateHint)
{
	mRebuildRateHint = rebuildRateHint;

	for(PxU32 i=0;i<PruningIndex::eCOUNT;i++)
	{
		if(mPrunerExt[i].pruner() && mPrunerExt[i].type() == PxPruningStructureType::eDYNAMIC_AABB_TREE)
			static_cast<AABBPruner*>(mPrunerExt[i].pruner())->setRebuildRateHint(rebuildRateHint);
	}
}

void SceneQueryManager::afterSync(PxSceneQueryUpdateMode::Enum updateMode)
{
	PX_PROFILE_ZONE("Sim.sceneQueryBuildStep", mScene.getContextId());

	if(updateMode == PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED)
	{
		mPrunerNeedsUpdating = true;
		return;
	}

	// flush user modified objects
	flushShapes();

	bool commit = updateMode == PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED;

	for(PxU32 i = 0; i<2; i++)
	{
		if(mPrunerExt[i].pruner() && mPrunerExt[i].type() == PxPruningStructureType::eDYNAMIC_AABB_TREE)
			static_cast<AABBPruner*>(mPrunerExt[i].pruner())->buildStep(true);

		if(commit)
			mPrunerExt[i].pruner()->commit();
	}

	mPrunerNeedsUpdating = !commit;
}

void SceneQueryManager::flushShapes()
{
	PX_PROFILE_ZONE("SceneQuery.flushShapes", mScene.getContextId());

	// must already have acquired writer lock here

	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
		mPrunerExt[i].flushShapes(i);

	mCompoundPrunerExt.flushShapes();
}

void SceneQueryManager::flushUpdates()
{
	PX_PROFILE_ZONE("SceneQuery.flushUpdates", mScene.getContextId());

	if (mPrunerNeedsUpdating)
	{
		// no need to take lock if manual sq update is enabled
		// as flushUpdates will only be called from NpScene::flushQueryUpdates()
		mSceneQueryLock.lock();

		if (mPrunerNeedsUpdating)
		{

			flushShapes();

			for (PxU32 i = 0; i < PruningIndex::eCOUNT; i++)
				if (mPrunerExt[i].pruner())
					mPrunerExt[i].pruner()->commit();

			Ps::memoryBarrier();
			mPrunerNeedsUpdating = false;
		}
		mSceneQueryLock.unlock();
	}
}

void SceneQueryManager::forceDynamicTreeRebuild(bool rebuildStaticStructure, bool rebuildDynamicStructure)
{
	PX_PROFILE_ZONE("SceneQuery.forceDynamicTreeRebuild", mScene.getContextId());

	const bool rebuild[PruningIndex::eCOUNT] = { rebuildStaticStructure, rebuildDynamicStructure };

	Ps::Mutex::ScopedLock lock(mSceneQueryLock);
	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
	{
		if(rebuild[i] && mPrunerExt[i].pruner() && mPrunerExt[i].type() == PxPruningStructureType::eDYNAMIC_AABB_TREE)
		{
			static_cast<AABBPruner*>(mPrunerExt[i].pruner())->purge();
			static_cast<AABBPruner*>(mPrunerExt[i].pruner())->commit();
		}
	}
}

void SceneQueryManager::sceneQueryBuildStep(PruningIndex::Enum index)
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueryBuildStep", mScene.getContextId());

	if (mPrunerExt[index].pruner() && mPrunerExt[index].type() == PxPruningStructureType::eDYNAMIC_AABB_TREE)
	{
		const bool buildFinished = static_cast<AABBPruner*>(mPrunerExt[index].pruner())->buildStep(false);
		if(buildFinished)
		{
			mPrunerNeedsUpdating = true;
		}
	}
}

bool SceneQueryManager::prepareSceneQueriesUpdate(PruningIndex::Enum index)
{
	bool retVal = false;
	if (mPrunerExt[index].pruner() && mPrunerExt[index].type() == PxPruningStructureType::eDYNAMIC_AABB_TREE)
	{
		retVal = static_cast<AABBPruner*>(mPrunerExt[index].pruner())->prepareBuild();
	}
	return retVal;
}

void SceneQueryManager::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0; i<PruningIndex::eCOUNT; i++)
		mPrunerExt[i].pruner()->shiftOrigin(shift);

	mCompoundPrunerExt.pruner()->shiftOrigin(shift);
}

void DynamicBoundsSync::sync(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds, PxU32 count, const Cm::BitMap& dirtyShapeSimMap)
{
	if(!count)
		return;

	PxU32 startIndex = 0;
	PxU32 numIndices = count;

	// if shape sim map is not empty, parse the indices and skip update for the dirty one
	if(dirtyShapeSimMap.count())
	{
		numIndices = 0;

		for(PxU32 i=0; i<count; i++)
		{
			if(dirtyShapeSimMap.test(indices[i]))
			{
				mPruner->updateObjectsAndInflateBounds(handles + startIndex, indices + startIndex, bounds, numIndices);
				numIndices = 0;
				startIndex = i + 1;
			}
			else
				numIndices++;
		}
		// PT: we fallback to the next line on purpose - no "else"
	}

	mPruner->updateObjectsAndInflateBounds(handles + startIndex, indices + startIndex, bounds, numIndices);

	(*mTimestamp)++;
}

void SceneQueryManager::addPruningStructure(const Sq::PruningStructure& pS)
{
	if(pS.getTreeNodes(PruningIndex::eSTATIC))
	{
		AABBPrunerMergeData params(pS.getTreeNbNodes(PruningIndex::eSTATIC), pS.getTreeNodes(PruningIndex::eSTATIC),
			pS.getNbObjects(PruningIndex::eSTATIC), pS.getTreeIndices(PruningIndex::eSTATIC));
		mPrunerExt[PruningIndex::eSTATIC].pruner()->merge(&params);
	}
	if(pS.getTreeNodes(PruningIndex::eDYNAMIC))
	{
		AABBPrunerMergeData params(pS.getTreeNbNodes(PruningIndex::eDYNAMIC), pS.getTreeNodes(PruningIndex::eDYNAMIC),
			pS.getNbObjects(PruningIndex::eDYNAMIC), pS.getTreeIndices(PruningIndex::eDYNAMIC));
		mPrunerExt[PruningIndex::eDYNAMIC].pruner()->merge(&params);
	}
}

void SceneQueryManager::addCompoundShape(const Gu::BVHStructure& bvhStructure, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerData* prunerData, const Scb::Shape** scbShapes, const Scb::Actor& scbActor)
{
	PX_ASSERT(mCompoundPrunerExt.mPruner);

	const PxU32 nbShapes = bvhStructure.getNbBounds();

	PX_ALLOCA(res, PrunerHandle, nbShapes);
	PX_ALLOCA(payloads, PrunerPayload, nbShapes);

	for(PxU32 i = 0; i < nbShapes; i++)
	{
		payloads[i].data[0] = size_t(scbShapes[i]);	//PAYLOAD
		payloads[i].data[1] = size_t(&scbActor);	//PAYLOAD
	}

	CompoundFlag::Enum flags = (scbActor.getActorType() == PxActorType::eRIGID_DYNAMIC) ? CompoundFlag::DYNAMIC_COMPOUND : CompoundFlag::STATIC_COMPOUND;
	mCompoundPrunerExt.mPruner->addCompound(res, bvhStructure, compoundId, compoundTransform, flags, payloads);
	const PxU32 index = (flags & CompoundFlag::STATIC_COMPOUND) ? PxU32(0) : PxU32(1);
	mPrunerExt[index].invalidateTimestamp();

	for(PxU32 i = 0; i < nbShapes; i++)
	{
		prunerData[i] = createPrunerData(index, res[i]);
	}
}

void SceneQueryManager::updateCompoundActors(Sc::BodyCore*const* bodies, PxU32 numBodies)
{
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	for(PxU32 i = 0; i < numBodies; i++)
	{
		mCompoundPrunerExt.mPruner->updateCompound(bodies[i]->getRigidID(), bodies[i]->getBody2World());
	}
	mPrunerExt[1].invalidateTimestamp();
}

void SceneQueryManager::updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform, bool dynamic)
{	
	mCompoundPrunerExt.mPruner->updateCompound(compoundId, compoundTransform);
	mPrunerExt[dynamic].invalidateTimestamp();
}

void SceneQueryManager::removeCompoundActor(PrunerCompoundId compoundId, bool dynamic)
{
	PX_ASSERT(mCompoundPrunerExt.mPruner);
	mCompoundPrunerExt.mPruner->removeCompound(compoundId);
	mPrunerExt[dynamic].invalidateTimestamp();
}


