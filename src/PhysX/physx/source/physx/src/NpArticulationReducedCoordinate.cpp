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

#include "NpArticulationReducedCoordinate.h"

#include "CmPhysXCommon.h"
#include "DyFeatherstoneArticulation.h"
#include "ScArticulationSim.h"
#include "ScConstraintSim.h"

#include "PsAlignedMalloc.h"
#include "PsFoundation.h"
#include "PsPool.h"

#include "PxPvdDataStream.h"
#include "extensions/PxJoint.h"

using namespace physx;

class LLReducedArticulationPool : public Ps::Pool<Dy::FeatherstoneArticulation, Ps::AlignedAllocator<Dy::DY_ARTICULATION_MAX_SIZE> >
{
public:
	LLReducedArticulationPool() {}
};

NpArticulationReducedCoordinate::NpArticulationReducedCoordinate()
	: NpArticulationTemplate(PxConcreteType::eARTICULATION, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
{
	mType = PxArticulationBase::eReducedCoordinate;
	mImpl.mArticulation.setArticulationType(PxArticulationBase::eReducedCoordinate);
}

void NpArticulationReducedCoordinate::setArticulationFlags(PxArticulationFlags flags)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	mImpl.getArticulation().setArticulationFlags(flags);
}

void NpArticulationReducedCoordinate::setArticulationFlag(PxArticulationFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	PxArticulationFlags flags = mImpl.getArticulation().getArticulationFlags();

	if(value)
		flags |= flag;
	else
		flags &= (~flag);

	mImpl.getArticulation().setArticulationFlags(flags);
}

PxArticulationFlags	NpArticulationReducedCoordinate::getArticulationFlags() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.getArticulation().getArticulationFlags();
}

PxU32 NpArticulationReducedCoordinate::getDofs() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.mArticulation.getScArticulation().getDofs();
}

PxArticulationCache* NpArticulationReducedCoordinate::createCache() const
{
	PX_CHECK_AND_RETURN_NULL(mImpl.getAPIScene(), "PxArticulation::createCache: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads

	PxArticulationCache* cache = mImpl.mArticulation.getScArticulation().createCache();
	cache->version = mImpl.mCacheVersion;

	return cache;
}

PxU32 NpArticulationReducedCoordinate::getCacheDataSize() const
{
	PX_CHECK_AND_RETURN_NULL(mImpl.getAPIScene(), "PxArticulation::getCacheDataSize: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads

	return mImpl.mArticulation.getScArticulation().getCacheDataSize();
}

void NpArticulationReducedCoordinate::zeroCache(PxArticulationCache& cache)
{
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads
	return mImpl.mArticulation.getScArticulation().zeroCache(cache);
}

void NpArticulationReducedCoordinate::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool autowake)
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::applyCache: object must be in a scene");

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::applyCache : cache is invalid, articulation configuration has changed! ")

		//if we try to do a bulk op when sim is running, return with error
		if (static_cast<NpScene*>(getScene())->getSimulationStage() != Sc::SimulationStage::eCOMPLETE)
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
				"NpArticulation::applyCache() not allowed while simulation is running.");
			return;
		}

	mImpl.mArticulation.getScArticulation().applyCache(cache, flag);

	if (flag & PxArticulationCache::ePOSITION)
	{
		const PxU32 linkCount = mImpl.mArticulationLinks.size();

		for (PxU32 i = 0; i < linkCount; ++i)
		{
			NpArticulationLink* link = mImpl.mArticulationLinks[i];
			//in the lowlevel articulation, we have already updated bodyCore's body2World
			const PxTransform internalPose = link->getScbBodyFast().getScBody().getBody2World();
			link->getScbBodyFast().setBody2World(internalPose, false);
		}
	}

	mImpl.wakeUpInternal(false, autowake);
}

void NpArticulationReducedCoordinate::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::copyInternalStateToCache: object must be in a scene");

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::applyCache : cache is invalid, articulation configuration has changed! ")

	mImpl.mArticulation.getScArticulation().copyInternalStateToCache(cache, flag);
}

void NpArticulationReducedCoordinate::releaseCache(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::releaseCache: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads

	mImpl.mArticulation.getScArticulation().releaseCache(cache);
}

void NpArticulationReducedCoordinate::packJointData(const PxReal* maximum, PxReal* reduced) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::packJointData: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	mImpl.mArticulation.getScArticulation().packJointData(maximum, reduced);
}

void NpArticulationReducedCoordinate::unpackJointData(const PxReal* reduced, PxReal* maximum) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::unpackJointData: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	mImpl.mArticulation.getScArticulation().unpackJointData(reduced, maximum);
}

void NpArticulationReducedCoordinate::commonInit() const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::commonInit: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	mImpl.mArticulation.getScArticulation().commonInit();
}

void NpArticulationReducedCoordinate::computeGeneralizedGravityForce(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeGeneralisedGravityForce: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeGeneralisedGravityForce : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeGeneralizedGravityForce(cache);
}

void NpArticulationReducedCoordinate::computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeCoriolisAndCentrifugalForce: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeCoriolisAndCentrifugalForce : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeCoriolisAndCentrifugalForce(cache);
}

void NpArticulationReducedCoordinate::computeGeneralizedExternalForce(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeGeneralizedExternalForce: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeGeneralizedExternalForce : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeGeneralizedExternalForce(cache);
}

void NpArticulationReducedCoordinate::computeJointAcceleration(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeJointAcceleration: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeJointAcceleration : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeJointAcceleration(cache);
}

void NpArticulationReducedCoordinate::computeJointForce(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeJointForce: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeJointForce : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeJointForce(cache);
}

void NpArticulationReducedCoordinate::computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeKenematicJacobian: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeKenematicJacobian : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeKinematicJacobian(linkID, cache);
}

void NpArticulationReducedCoordinate::computeCoefficentMatrix(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeCoefficentMatrix: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeCoefficentMatrix : cache is invalid, articulation configuration has changed! ");

	for (PxU32 i = 0; i < mLoopJoints.size(); ++i)
	{
		static_cast<NpConstraint*>(mLoopJoints[i]->getConstraint())->updateConstants();
	}

	mImpl.mArticulation.getScArticulation().computeCoefficentMatrix(cache);
}

bool NpArticulationReducedCoordinate::computeLambda(PxArticulationCache& cache, PxArticulationCache& initialState, const PxReal* const jointTorque, const PxU32 maxIter) const
{
	if (!mImpl.getAPIScene())
	{
		physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxArticulation::computeLambda : object must be in a scened!");
		return false;
	}
		
	NP_READ_CHECK(mImpl.getOwnerScene());

	if (cache.version != mImpl.mCacheVersion)
	{
		physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxArticulation::computeLambda : cache is invalid, articulation configuration has changed!");
		return false;
	}

	return mImpl.mArticulation.getScArticulation().computeLambda(cache, initialState, jointTorque, getScene()->getGravity(), maxIter);
}

void NpArticulationReducedCoordinate::computeGeneralizedMassMatrix(PxArticulationCache& cache) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeGeneralizedMassMatrix: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());

	PX_CHECK_AND_RETURN(cache.version == mImpl.mCacheVersion, "PxArticulation::computeGeneralizedMassMatrix : cache is invalid, articulation configuration has changed! ");

	mImpl.mArticulation.getScArticulation().computeGeneralizedMassMatrix(cache);
}

void NpArticulationReducedCoordinate::addLoopJoint(PxJoint* joint)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());

#if PX_CHECKED

	PxRigidActor* actor0;
	PxRigidActor* actor1;

	joint->getActors(actor0, actor1);

	PxArticulationLink* link0 = NULL;
	PxArticulationLink* link1 = NULL;

	if(actor0)
		link0 = actor0->is<PxArticulationLink>();

	if(actor1)
		link1 = actor1->is<PxArticulationLink>();

	PX_CHECK_AND_RETURN((link0 || link1), "PxArticulation::addLoopJoint : at least one of the PxRigidActors need to be PxArticulationLink! ");

	
	PxArticulationBase* base0 = NULL;
	PxArticulationBase* base1 = NULL;
	if (link0)
		base0 = &link0->getArticulation();

	if (link1)
		base1 = &link1->getArticulation();

	PX_CHECK_AND_RETURN((base0 == this || base1 == this), "PxArticulation::addLoopJoint : at least one of the PxArticulationLink belongs to this articulation! ");
#endif
		
	const PxU32 size = mLoopJoints.size();
	if (size >= mLoopJoints.capacity())
	{
		mLoopJoints.reserve(size * 2 + 1);
	}

	mLoopJoints.pushBack(joint);

	Scb::Articulation& scbArt = mImpl.getArticulation();
	Sc::ArticulationSim* scArtSim = scbArt.getScArticulation().getSim();

	NpConstraint* constraint = static_cast<NpConstraint*>(joint->getConstraint());
	Sc::ConstraintSim* cSim = constraint->getScbConstraint().getScConstraint().getSim();
	if(scArtSim)
		scArtSim->addLoopConstraint(cSim);
}

void NpArticulationReducedCoordinate::removeLoopJoint(PxJoint* joint)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());

	mLoopJoints.findAndReplaceWithLast(joint);

	Scb::Articulation& scbArt = mImpl.getArticulation();
	Sc::ArticulationSim* scArtSim = scbArt.getScArticulation().getSim();

	NpConstraint* constraint = static_cast<NpConstraint*>(joint->getConstraint());
	Sc::ConstraintSim* cSim = constraint->getScbConstraint().getScConstraint().getSim();
	scArtSim->removeLoopConstraint(cSim);
}

PxU32 NpArticulationReducedCoordinate::getNbLoopJoints() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());

	return mLoopJoints.size();
}

PxU32 NpArticulationReducedCoordinate::getLoopJoints(PxJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(mImpl.getOwnerScene());

	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mLoopJoints.begin(), mLoopJoints.size());
}

PxU32 NpArticulationReducedCoordinate::getCoefficentMatrixSize() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	
	return mImpl.mArticulation.getScArticulation().getCoefficentMatrixSize();
}

void NpArticulationReducedCoordinate::teleportRootLink(const PxTransform& pose, bool autowake)
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulationReducedCoordinate::teleportRootLink: object must be in a scene");

	PX_CHECK_AND_RETURN(pose.isValid(), "PxArticulationReducedCoordinate::teleportRootLink pose is not valid.");

	NP_WRITE_CHECK(mImpl.getOwnerScene());

	NpArticulationLink* root = mImpl.mArticulationLinks[0];

	root->setGlobalPoseInternal(pose, autowake);
}

NpArticulationReducedCoordinate::~NpArticulationReducedCoordinate()
{
	NpFactory::getInstance().onArticulationRelease(this);
}

PxArticulationJointBase* NpArticulationReducedCoordinate::createArticulationJoint(PxArticulationLink& parent,
	const PxTransform& parentFrame,
	PxArticulationLink& child,
	const PxTransform& childFrame)
{	
	return NpFactory::getInstance().createNpArticulationJointRC(static_cast<NpArticulationLink&>(parent), parentFrame, static_cast<NpArticulationLink&>(child), childFrame);
}

void NpArticulationReducedCoordinate::releaseArticulationJoint(PxArticulationJointBase* joint)
{
	NpFactory::getInstance().releaseArticulationJointRCToPool(*static_cast<NpArticulationJointReducedCoordinate*>(joint));
}


