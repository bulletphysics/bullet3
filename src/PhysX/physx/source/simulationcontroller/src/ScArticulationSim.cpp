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


#include "ScArticulationSim.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointSim.h"
#include "ScArticulationJointCore.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"
#include "ScScene.h"

#include "DyArticulation.h"
#include "DyConstraint.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsContext.h"
#include "CmSpatialVector.h"
#include "PsVecMath.h"
#include "PxsSimpleIslandManager.h"
#include "ScShapeSim.h"

using namespace physx;
using namespace physx::Dy;


Sc::ArticulationSim::ArticulationSim(ArticulationCore& core, Scene& scene, BodyCore& root) : 
	mLLArticulation(NULL),
	mScene(scene),
	mCore(core),
	mLinks				(PX_DEBUG_EXP("ScArticulationSim::links")),
	mBodies				(PX_DEBUG_EXP("ScArticulationSim::bodies")),
	mJoints				(PX_DEBUG_EXP("ScArticulationSim::joints")),
	mMaxDepth(0)
{
	mLinks.reserve(16);
	mJoints.reserve(16);
	mBodies.reserve(16);

	mLLArticulation = mScene.createLLArticulation(this);
	

	mIslandNodeIndex = scene.getSimpleIslandManager()->addArticulation(this, mLLArticulation, false);

	if(!mLLArticulation)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Articulation: could not allocate low-level resources.");
		return;
	}

	mLLArticulation->setDirty(true);

	PX_ASSERT(root.getSim());

	addBody(*root.getSim(), NULL, NULL);

	

	mCore.setSim(this);

	mLLArticulation->setDyContext(mScene.getDynamicsContext());
	mLLArticulation->getSolverDesc().core			= &core.getCore();
	mLLArticulation->getSolverDesc().internalLoads	= NULL;
	mLLArticulation->getSolverDesc().externalLoads	= NULL;
	mLLArticulation->getSolverDesc().articulation	= NULL;
	mLLArticulation->getSolverDesc().poses			= NULL;
	mLLArticulation->getSolverDesc().motionVelocity	= NULL;
	mLLArticulation->getSolverDesc().acceleration = NULL;
	mLLArticulation->getSolverDesc().totalDataSize	= 0;
	mLLArticulation->getSolverDesc().solverDataSize	= 0;
	mLLArticulation->getSolverDesc().linkCount		= 0;
	mLLArticulation->getSolverDesc().scratchMemory	= NULL;
	mLLArticulation->getSolverDesc().scratchMemorySize = 0;

	//mLLArticulation->onUpdateSolverDesc();
}


Sc::ArticulationSim::~ArticulationSim()
{
	if (!mLLArticulation)
		return;

	mScene.destroyLLArticulation(*mLLArticulation);

	mScene.getSimpleIslandManager()->removeNode(mIslandNodeIndex);

	mCore.setSim(NULL);
}

PxU32 Sc::ArticulationSim::findBodyIndex(BodySim& body) const
{
	for(PxU32 i=0; i<mBodies.size(); i++)
	{
		if(mBodies[i]==&body)
			return i;
	}
	PX_ASSERT(0);
	return 0x80000000;
}

void Sc::ArticulationSim::addLoopConstraint(ConstraintSim* constraintSim)
{
	const PxU32 size = mLoopConstraints.size();
	if (size < mLoopConstraints.size())
		mLoopConstraints.reserve(size*2 + 1);
	
	BodySim* bodySim0 = constraintSim->getBody(0);
	BodySim* bodySim1 = constraintSim->getBody(1);

	ArticulationLoopConstraint lConstraint;
	if (bodySim0)
		lConstraint.linkIndex0 = findBodyIndex(*bodySim0);
	else
		lConstraint.linkIndex0 = 0x80000000;

	if(bodySim1)
		lConstraint.linkIndex1 = findBodyIndex(*bodySim1);
	else
		lConstraint.linkIndex1 = 0x80000000;

	lConstraint.constraint = &constraintSim->getLowLevelConstraint();

	mLoopConstraints.pushBack(lConstraint);
}

void Sc::ArticulationSim::removeLoopConstraint(ConstraintSim* constraintSim)
{
	Dy::Constraint* constraint = &constraintSim->getLowLevelConstraint();

	const PxU32 size = mLoopConstraints.size();
	PxU32 index = 0;
	while (index < size && mLoopConstraints[index].constraint != constraint)
		++index;

	if (index != size)
		mLoopConstraints.replaceWithLast(index);
}

void Sc::ArticulationSim::updateCached(Cm::BitMapPinned* shapeChangedMap)
{
	for(PxU32 i=0; i<mBodies.size(); i++)
		mBodies[i]->updateCached(shapeChangedMap);
}

void Sc::ArticulationSim::markShapesUpdated(Cm::BitMapPinned* shapeChangedMap)
{
	for (PxU32 a = 0; a < mBodies.size(); ++a)
	{
		Sc::ElementSim* current = mBodies[a]->getElements_();
		while (current)
		{
			Sc::ShapeSim* sim = static_cast<Sc::ShapeSim*>(current);
			if (sim->isInBroadPhase())
				shapeChangedMap->growAndSet(sim->getElementID());
			current = current->mNextInActor;
		}
	}
}

void Sc::ArticulationSim::updateContactDistance(PxReal* contactDistance, const PxReal dt, Bp::BoundsArray& boundsArray)
{
	for (PxU32 i = 0; i<mBodies.size(); i++)
		mBodies[i]->updateContactDistance(contactDistance, dt, boundsArray);
}

ArticulationLinkHandle Sc::ArticulationSim::getLinkHandle(BodySim &body) const
{
	return reinterpret_cast<size_t>(mLLArticulation) | findBodyIndex(body);
}

void Sc::ArticulationSim::addBody(BodySim& body, 
								  BodySim* parent, 
								  ArticulationJointSim* joint)
{
	mBodies.pushBack(&body);
	mJoints.pushBack(joint);
	mLLArticulation->addBody();

	PxU32 index = mLinks.size();

	PX_ASSERT((((index==0) && (joint == 0)) && (parent == 0)) ||
			  (((index!=0) && joint) && (parent && (parent->getArticulation() == this))));

	ArticulationLink &link = mLinks.insert();
	link.body = &body.getLowLevelBody();
	link.bodyCore = &body.getBodyCore().getCore();
	link.children = 0;
	bool shouldSleep;
	bool currentlyAsleep;
	bool bodyReadyForSleep = body.checkSleepReadinessBesidesWakeCounter();
	PxReal wakeCounter = getCore().getWakeCounter();

	if(parent)
	{
		currentlyAsleep = !mBodies[0]->isActive();
		shouldSleep = currentlyAsleep && bodyReadyForSleep;

		PxU32 parentIndex = findBodyIndex(*parent);
		link.parent = parentIndex;
		link.pathToRoot = mLinks[parentIndex].pathToRoot | ArticulationBitField(1)<<index;
		link.inboundJoint = &joint->getCore().getCore();
		mLinks[parentIndex].children |= ArticulationBitField(1)<<index;
	}
	else
	{
		currentlyAsleep = (wakeCounter == 0.0f);
		shouldSleep = currentlyAsleep && bodyReadyForSleep;

		link.parent = DY_ARTICULATION_LINK_NONE;
		link.pathToRoot = 1;
		link.inboundJoint = NULL;
	}
	
	const PxU32 low = PxU32(link.pathToRoot & 0xffffffff);
	const PxU32 high = PxU32(link.pathToRoot >> 32);
	const PxU32 depth = Ps::bitCount(low) + Ps::bitCount(high);
	mMaxDepth = PxMax(depth, mMaxDepth);

	mLLArticulation->setMaxDepth(mMaxDepth);

	if (currentlyAsleep && (!shouldSleep))
	{
		for(PxU32 i=0; i < (mBodies.size() - 1); i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}

	body.setArticulation(this, wakeCounter, shouldSleep, index);
}


void Sc::ArticulationSim::removeBody(BodySim &body)
{
	PX_ASSERT(body.getArticulation() == this);
	PxU32 index = findBodyIndex(body);
	body.setArticulation(NULL, 0.0f, true, 0);

	ArticulationLink &link0 = mLinks[index];

	PX_ASSERT(link0.children == 0);
	PX_UNUSED(link0);

	// copy all the later links down by one
	for(PxU32 i=index+1;i<mLinks.size();i++)
	{
		mLinks[i-1] = mLinks[i];
		mBodies[i-1] = mBodies[i];
		mJoints[i-1] = mJoints[i];
		//setIslandHandle(*mBodies[i-1], i-1);
	}

	// adjust parent/child indices
	ArticulationBitField fixedIndices = (ArticulationBitField(1)<<index)-1;
	ArticulationBitField shiftIndices = ~(fixedIndices|(ArticulationBitField(1)<<index));

	mMaxDepth = 0;
	for(PxU32 i=0;i<mLinks.size();i++)
	{
		ArticulationLink &link = mLinks[i];

		if(link.parent != DY_ARTICULATION_LINK_NONE && link.parent>index)
			link.pathToRoot = (link.pathToRoot&fixedIndices) | (link.pathToRoot&shiftIndices)>>1;
		link.children = (link.children&fixedIndices) | (link.children&shiftIndices)>>1;

		const PxU32 low = PxU32(link.pathToRoot & 0xffffffff);
		const PxU32 high = PxU32(link.pathToRoot >> 32);
		const PxU32 depth = Ps::bitCount(low) + Ps::bitCount(high);
		mMaxDepth = PxMax(depth, mMaxDepth);
	}

	mLinks.popBack();

	mLLArticulation->setMaxDepth(mMaxDepth);
	mLLArticulation->removeBody();
}


void Sc::ArticulationSim::checkResize() const
{
	if(!mBodies.size())
		return;

	//if this is needed, we need to re-allocated the link data
	mLLArticulation->resize(mLinks.size());
	
	mLLArticulation->getSolverDesc().links			= const_cast<Dy::ArticulationLink*>(mLinks.begin());
	mLLArticulation->getSolverDesc().linkCount		= Ps::to8(mLinks.size());

	//if this is needed, we need to re-allocated the joint data
	mLLArticulation->onUpdateSolverDesc();
}

PxU32 Sc::ArticulationSim::getCCDLinks(BodySim** sims)
{
	PxU32 nbCCDBodies = 0;
	for (PxU32 a = 0; a < mBodies.size(); ++a)
	{
		if (mBodies[a]->getLowLevelBody().getCore().mFlags & PxRigidBodyFlag::eENABLE_CCD)
		{
			sims[nbCCDBodies++] = mBodies[a];
		}
	}
	return nbCCDBodies;
}

void Sc::ArticulationSim::sleepCheck(PxReal dt)
{
	if(!mBodies.size())
		return;

#if PX_CHECKED
	{
		PxReal maxTimer = 0.0f, minTimer = PX_MAX_F32;
		bool allActive = true, noneActive = true;
		for(PxU32 i=0;i<mLinks.size();i++)
		{
			PxReal timer = mBodies[i]->getBodyCore().getWakeCounter();
			maxTimer = PxMax(maxTimer, timer);
			minTimer = PxMin(minTimer, timer);
			bool active = mBodies[i]->isActive();
			allActive &= active;
			noneActive &= !active;
		}
		// either all links are asleep, or no links are asleep
		PX_ASSERT(maxTimer==0 || minTimer!=0);
		PX_ASSERT(allActive || noneActive);
	}

#endif

	if(!mBodies[0]->isActive())
		return;

	PxReal sleepThreshold = getCore().getCore().sleepThreshold;

	PxReal maxTimer = 0.0f, minTimer = PX_MAX_F32;

	for(PxU32 i=0;i<mLinks.size();i++)
	{
		const Cm::SpatialVector motionVelocity = mLLArticulation->getMotionVelocity(i);
		PxReal timer = mBodies[i]->updateWakeCounter(dt, sleepThreshold, motionVelocity);
		maxTimer = PxMax(maxTimer, timer);
		minTimer = PxMin(minTimer, timer);
	}

	mCore.setWakeCounterInternal(maxTimer);

	if(maxTimer != 0.0f)
	{
		if(minTimer == 0.0f)
		{
			// make sure nothing goes to sleep unless everything does
			for(PxU32 i=0;i<mLinks.size();i++)
				mBodies[i]->getBodyCore().setWakeCounterFromSim(PxMax(1e-6f, mBodies[i]->getBodyCore().getWakeCounter()));
		}
		return;
	}

	for(PxU32 i=0;i<mLinks.size();i++)
	{
		mBodies[i]->notifyReadyForSleeping();
		mBodies[i]->resetSleepFilter();
	}

	mScene.getSimpleIslandManager()->deactivateNode(mIslandNodeIndex);
}

bool Sc::ArticulationSim::isSleeping() const
{
	bool isActive = mBodies[0]->isActive();
	PX_UNUSED(isActive);
	return (mBodies.size() > 0) ? (!mBodies[0]->isActive()) : true;
}

void Sc::ArticulationSim::internalWakeUp(PxReal wakeCounter)
{
	if(mCore.getWakeCounter() < wakeCounter)
	{
		mCore.setWakeCounterInternal(wakeCounter);
		for(PxU32 i=0;i<mLinks.size();i++)
			mBodies[i]->internalWakeUpArticulationLink(wakeCounter);
	}
}

void Sc::ArticulationSim::setActive(const bool b, const PxU32 infoFlag)
{
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			Ps::prefetchLine(mBodies[i+1],0);
			Ps::prefetchLine(mBodies[i+1],128);
		}
		mBodies[i]->setActive(b, infoFlag);
	}
}

void Sc::ArticulationSim::updateForces(PxReal dt, bool simUsesAdaptiveForce)
{
	PxU32 count = 0;

	
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			Ps::prefetchLine(mBodies[i+1],128);
			Ps::prefetchLine(mBodies[i+1],256);
		}

		PxU32 type = mLLArticulation->getType();
		const bool useAccelerations = (type == PxArticulationBase::Enum::eReducedCoordinate);

		mBodies[i]->updateForces(dt, NULL, NULL, count, &mLLArticulation->getSolverDesc().acceleration[i], 
			useAccelerations, simUsesAdaptiveForce);
	}
}

void Sc::ArticulationSim::saveLastCCDTransform()
{
	for(PxU32 i=0;i<mBodies.size();i++)
	{
		if (i+1 < mBodies.size())
		{
			Ps::prefetchLine(mBodies[i+1],128);
			Ps::prefetchLine(mBodies[i+1],256);
		}
		mBodies[i]->getLowLevelBody().saveLastCCDTransform();
	}
}


Sc::ArticulationDriveCache* Sc::ArticulationSim::createDriveCache(PxReal compliance,
																  PxU32 driveIterations) const
{
	
	checkResize();
	PxU32 solverDataSize, totalSize, scratchSize;
	getLowLevelArticulation()->getDataSizes(mLinks.size(), solverDataSize, totalSize, scratchSize);

	// In principle we should only need solverDataSize here. But right now prepareFsData generates the auxiliary data 
	// for use in potential debugging, which takes up extra space. 
	FsData* data = reinterpret_cast<FsData*>(PX_ALLOC(totalSize,"Articulation Drive Cache"));
	PxvArticulationDriveCache::initialize(*data, Ps::to16(mLinks.size()), mLinks.begin(), compliance, driveIterations, mLLArticulation->getSolverDesc().scratchMemory, mLLArticulation->getSolverDesc().scratchMemorySize);
	
	return data;
}


void Sc::ArticulationSim::updateDriveCache(ArticulationDriveCache& cache,
										   PxReal compliance,
										   PxU32 driveIterations) const
{
	checkResize();
	PxvArticulationDriveCache::initialize(cache,  Ps::to16(mLinks.size()), mLinks.begin(), compliance, driveIterations, 
		mLLArticulation->getSolverDesc().scratchMemory, mLLArticulation->getSolverDesc().scratchMemorySize);
}


void Sc::ArticulationSim::releaseDriveCache(Sc::ArticulationDriveCache& driveCache) const
{
	PX_FREE(&driveCache);
}


void Sc::ArticulationSim::applyImpulse(Sc::BodyCore& link,
									   const Sc::ArticulationDriveCache& driveCache,
									   const PxVec3& force,
									   const PxVec3& torque)
{
	Cm::SpatialVectorV v[DY_ARTICULATION_MAX_SIZE], z[DY_ARTICULATION_MAX_SIZE];
	PxMemZero(z, mLinks.size()*sizeof(Cm::SpatialVector));
	PxMemZero(v, mLinks.size()*sizeof(Cm::SpatialVector));

	PxU32 bodyIndex = findBodyIndex(*link.getSim());
	z[bodyIndex].linear = Ps::aos::V3LoadU(-force);
	z[bodyIndex].angular = Ps::aos::V3LoadU(-torque);

	PxvArticulationDriveCache::applyImpulses(driveCache, z, v);
	for(PxU32 i=0;i<mLinks.size();i++)
	{
		Sc::BodyCore& body = mBodies[i]->getBodyCore();
		PxVec3 lv, av;
		Ps::aos::V3StoreU(v[i].linear, lv);
		Ps::aos::V3StoreU(v[i].angular, av);

		body.setLinearVelocity(body.getLinearVelocity()+lv);
		body.setAngularVelocity(body.getAngularVelocity()+av);
	}
}

void Sc::ArticulationSim::computeImpulseResponse(Sc::BodyCore& link,
												  PxVec3& linearResponse, 
												  PxVec3& angularResponse,
												  const Sc::ArticulationDriveCache& driveCache,
												  const PxVec3& force,
												  const PxVec3& torque) const
{
	Cm::SpatialVectorV v;
	PxvArticulationDriveCache::getImpulseResponse(driveCache, findBodyIndex(*link.getSim()), Cm::SpatialVectorV(Ps::aos::V3LoadU(force), Ps::aos::V3LoadU(torque)), v);
	Ps::aos::V3StoreU(v.linear, linearResponse);
	Ps::aos::V3StoreU(v.angular, angularResponse);
}

PxU32 Sc::ArticulationSim::getDofs() const
{
	return mLLArticulation->getDofs();
}

PxU32 Sc::ArticulationSim::getDof(const PxU32 linkID) const
{
	return mLLArticulation->getDof(linkID);
}

PxArticulationCache* Sc::ArticulationSim::createCache() const
{

	checkResize();

	PxU32 totalSize = getCacheDataSize() + sizeof(PxArticulationCache);

	PxU8* tCache = reinterpret_cast<PxU8*>(PX_ALLOC(totalSize, "Articulation cache"));

	PxMemZero(tCache, totalSize);

	const PxU32 totalDofs = mLLArticulation->getDofs();

	PxArticulationCache* cache = reinterpret_cast<PxArticulationCache*>(tCache);

	PxU32 offset = sizeof(PxArticulationCache);
	cache->externalForces = reinterpret_cast<Cm::SpatialVector*>(tCache + offset);

	offset += sizeof(Cm::SpatialVector) * mLinks.size();

	cache->jacobian = reinterpret_cast<PxKinematicJacobian*>(tCache + offset);

	offset += sizeof(PxReal) * totalDofs * 6;//maximum 6 columns for coefficent matrix
	cache->massMatrix = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(PxReal) *totalDofs * totalDofs;
	cache->jointVelocity = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(PxReal) * totalDofs;
	cache->jointAcceleration = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(PxReal) * totalDofs;
	cache->jointPosition = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(PxReal) * totalDofs;
	cache->jointForce = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(void*);
	cache->coefficentMatrix = reinterpret_cast<PxReal*>(tCache + offset);

	offset += sizeof(void*);
	cache->lambda = reinterpret_cast<PxReal*>(tCache + offset);

	const PxU32 scratchMemorySize = getScratchMemorySize();
	void* scratchMemory = PX_ALLOC(scratchMemorySize, "Cache scratch memory");
	cache->scratchMemory = scratchMemory;

	cache->scratchAllocator = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(PxcScratchAllocator), "PxScrachAllocator"), PxcScratchAllocator)();

	reinterpret_cast<PxcScratchAllocator*>(cache->scratchAllocator)->setBlock(scratchMemory, scratchMemorySize);

	return cache;
}

PxU32 Sc::ArticulationSim::getCacheDataSize() const
{
	const PxU32 totalDofs = mLLArticulation->getDofs();

	const PxU32 jointCount = mLinks.size() - 1;
	PxU32 totalSize =
		sizeof(Cm::SpatialVector) * mLinks.size()				//external force
		+ sizeof(PxKinematicJacobian) * jointCount				//jacobian matrix
		+ sizeof(PxReal) * totalDofs * totalDofs				//mass matrix
		+ sizeof(PxReal) * totalDofs * 4						//jointVelocity, jointAcceleration, jointPosition, joint force
		+ sizeof(void*) * 2										//pointer to coefficent matrix and pointer to lambda vector
		+ sizeof(PxArticulationRootLinkData)					//root link data
		+ sizeof(void*) * 2										//pointer to scratch memory/scratch memory allocator
		+ sizeof(PxU32);										//version
		
	return totalSize;
}

PxU32 Sc::ArticulationSim::getCacheConstantDataSize() const
{
	PxU32 totalSize = sizeof(void*) * 2 + sizeof(PxU32);			//pointer to scratch memory & scrach memory allocator, version

	return totalSize;
}

PxU32 Sc::ArticulationSim::getScratchMemorySize() const
{
	const PxU32 totalDofs = mLLArticulation->getDofs();
	const PxU32 linkCount = mLinks.size();

	PxU32 totalSize =
		sizeof(Cm::SpatialVectorF) * linkCount * 5				//motionVelocity, motionAccelerations, coriolisVectors, spatialZAVectors, externalAccels;
		+ sizeof(Dy::SpatialMatrix) * linkCount					//compositeSpatialInertias;
		+ sizeof(PxReal) * totalDofs * 5;						//jointVelocity, jointAcceleration, jointForces, jointPositions, jointFrictionForces

	return totalSize;
}


void Sc::ArticulationSim::zeroCache(PxArticulationCache& cache) const
{
	const PxU32 cacheDataSize = getCacheDataSize();
	const PxU32 constantSize = getCacheConstantDataSize();

	//don't zero gravity, dt and version  
	PxMemZero(cache.externalForces, (cacheDataSize - constantSize));
}

//copy external data to internal data
void  Sc::ArticulationSim::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	//checkResize();
	mLLArticulation->applyCache(cache, flag);
}

//copy internal data to external data
void Sc::ArticulationSim::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	mLLArticulation->copyInternalStateToCache(cache, flag);
}

//release cache
void Sc::ArticulationSim::releaseCache(PxArticulationCache& cache) const
{
	if (cache.scratchAllocator)
	{
		PxcScratchAllocator* scratchAlloc = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);
		scratchAlloc->~PxcScratchAllocator();
		PX_FREE_AND_RESET(cache.scratchAllocator);
	}

	if (cache.scratchMemory)
		PX_FREE_AND_RESET(cache.scratchMemory);

	PX_FREE(&cache);
}

void Sc::ArticulationSim::packJointData(const PxReal* maximum, PxReal* reduced) const
{
	mLLArticulation->packJointData(maximum, reduced);
}

void Sc::ArticulationSim::unpackJointData(const PxReal* reduced, PxReal* maximum) const
{
	mLLArticulation->unpackJointData(reduced, maximum);
}

void Sc::ArticulationSim::commonInit()
{
	mLLArticulation->initializeCommonData();
}

void Sc::ArticulationSim::computeGeneralizedGravityForce(PxArticulationCache& cache)
{
	mLLArticulation->getGeneralizedGravityForce(mScene.getGravityFast(), cache);
}

void Sc::ArticulationSim::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache)
{
	mLLArticulation->getCoriolisAndCentrifugalForce(cache);
}

void Sc::ArticulationSim::computeGeneralizedExternalForce(PxArticulationCache& cache)
{
	mLLArticulation->getGeneralizedExternalForce(cache);
}

void Sc::ArticulationSim::computeJointAcceleration(PxArticulationCache& cache)
{
	mLLArticulation->getJointAcceleration(mScene.getGravityFast(), cache);
}

void Sc::ArticulationSim::computeJointForce(PxArticulationCache& cache)
{
	mLLArticulation->getJointForce(cache);
}

void Sc::ArticulationSim::computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache)
{
	mLLArticulation->getKinematicJacobian(linkID, cache);
}

void Sc::ArticulationSim::computeCoefficentMatrix(PxArticulationCache& cache)
{
	mLLArticulation->getCoefficentMatrixWithLoopJoints(mLoopConstraints.begin(), mLoopConstraints.size(), cache);
}

bool Sc::ArticulationSim::computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState,
	const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter)
{
	return mLLArticulation->getLambda(mLoopConstraints.begin(), mLoopConstraints.size(), cache, initialState, jointTorque, gravity, maxIter);
	
}

void Sc::ArticulationSim::computeGeneralizedMassMatrix(PxArticulationCache& cache)
{

	mLLArticulation->getGeneralizedMassMatrixCRB(cache);


	/*const PxU32 totalDofs = mLLArticulation->getDofs();

	PxReal* massMatrix = reinterpret_cast<PxReal*>(PX_ALLOC(sizeof(PxReal) * totalDofs * totalDofs, "MassMatrix"));
	PxMemCopy(massMatrix, cache.massMatrix, sizeof(PxReal)*totalDofs * totalDofs);

	mLLArticulation->getGeneralizedMassMatrix(cache);

	PxReal* massMatrix1 = cache.massMatrix;
	for (PxU32 i = 0; i < totalDofs; ++i)
	{
		PxReal* row = &massMatrix1[i * totalDofs];

		for (PxU32 j = 0; j < totalDofs; ++j)
		{
			const PxReal dif = row[j] - massMatrix[j*totalDofs + i];
			if (PxAbs(dif) > 2e-4f)
			{
				int bob = 0;
				PX_UNUSED(bob);
			}
		}

	}

	PX_FREE(massMatrix);*/

}

PxU32 Sc::ArticulationSim::getCoefficentMatrixSize() const
{
	const PxU32 size = mLoopConstraints.size();
	const PxU32 totalDofs = mLLArticulation->getDofs();
	return sizeof(PxReal) * size * totalDofs;
}

// This method allows user teleport the root links and the articulation
//system update all other links pose
void Sc::ArticulationSim::setGlobalPose()
{
	checkResize();
	mLLArticulation->teleportRootLink();
}

void Sc::ArticulationSim::setDirty(const bool dirty)
{
	mLLArticulation->setDirty(dirty);
}

void Sc::ArticulationSim::debugCheckWakeCounterOfLinks(PxReal wakeCounter) const
{
	PX_UNUSED(wakeCounter);

#ifdef _DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == wakeCounter);
	}
#endif
}

void Sc::ArticulationSim::debugCheckSleepStateOfLinks(bool isSleeping) const
{
	PX_UNUSED(isSleeping);

#ifdef _DEBUG
	// make sure the links are in sync with the articulation
	for(PxU32 i=0; i < mBodies.size(); i++)
	{
		if (isSleeping)
		{
			PX_ASSERT(!mBodies[i]->isActive());
			PX_ASSERT(mBodies[i]->getBodyCore().getWakeCounter() == 0.0f);
			PX_ASSERT(mBodies[i]->checkSleepReadinessBesidesWakeCounter());
		}
		else
			PX_ASSERT(mBodies[i]->isActive());
	}
#endif
}
