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

#include "foundation/PxTransform.h"
#include "NpArticulation.h"
#include "NpArticulationLink.h"
#include "NpWriteCheck.h"
#include "NpReadCheck.h"
#include "NpFactory.h"
#include "ScbArticulation.h"
#include "NpAggregate.h"
#include "CmUtils.h"
#include "NpArticulationJoint.h"

namespace physx
{

// PX_SERIALIZATION
void NpArticulation::requiresObjects(PxProcessPxBaseCallback& c)
{
	// Collect articulation links
	const PxU32 nbLinks = mImpl.mArticulationLinks.size();
	for(PxU32 i=0; i<nbLinks; i++)
		c.process(*mImpl.mArticulationLinks[i]);
}

void NpArticulation::exportExtraData(PxSerializationContext& stream)
{
	Cm::exportInlineArray(mImpl.mArticulationLinks, stream);
	stream.writeName(mImpl.mName);
}

void NpArticulation::importExtraData(PxDeserializationContext& context)
{	
	Cm::importInlineArray(mImpl.mArticulationLinks, context);
	context.readName(mImpl.mName);
}

void NpArticulation::resolveReferences(PxDeserializationContext& context)
{
	const PxU32 nbLinks = mImpl.mArticulationLinks.size();
	for(PxU32 i=0;i<nbLinks;i++)
	{
		context.translatePxBase(mImpl.mArticulationLinks[i]);
	}
	
	mImpl.mAggregate = NULL;
}

NpArticulation* NpArticulation::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulation* obj = new (address) NpArticulation(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpArticulation);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulation::NpArticulation()
	: NpArticulationTemplate(PxConcreteType::eARTICULATION, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
{
	mType = PxArticulationBase::eMaximumCoordinate;
	mImpl.mArticulation.setArticulationType(PxArticulationBase::eMaximumCoordinate);
}


NpArticulation::~NpArticulation()
{
	NpFactory::getInstance().onArticulationRelease(this);
}


PxU32 NpArticulation::getInternalDriveIterations() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.getArticulation().getInternalDriveIterations();
}

void NpArticulation::setInternalDriveIterations(PxU32 iterations)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	mImpl.getArticulation().setInternalDriveIterations(iterations);
}

PxU32 NpArticulation::getExternalDriveIterations() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.getArticulation().getExternalDriveIterations();
}

void NpArticulation::setExternalDriveIterations(PxU32 iterations)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	mImpl.getArticulation().setExternalDriveIterations(iterations);
}

PxU32 NpArticulation::getMaxProjectionIterations() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.getArticulation().getMaxProjectionIterations();
}

void NpArticulation::setMaxProjectionIterations(PxU32 iterations)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	mImpl.getArticulation().setMaxProjectionIterations(iterations);
}

PxReal NpArticulation::getSeparationTolerance() const
{
	NP_READ_CHECK(mImpl.getOwnerScene());
	return mImpl.getArticulation().getSeparationTolerance();
}

void NpArticulation::setSeparationTolerance(PxReal tolerance)
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());
	mImpl.getArticulation().setSeparationTolerance(tolerance);
}


PxArticulationDriveCache* NpArticulation::createDriveCache(PxReal compliance, PxU32 driveIterations) const
{
	PX_CHECK_AND_RETURN_NULL(mImpl.getAPIScene(), "PxArticulation::createDriveCache: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads

	return reinterpret_cast<PxArticulationDriveCache*>(mImpl.mArticulation.getScArticulation().createDriveCache(compliance, driveIterations));
}


void NpArticulation::updateDriveCache(PxArticulationDriveCache& cache, PxReal compliance, PxU32 driveIterations) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::updateDriveCache: object must be in a scene");

	Sc::ArticulationDriveCache& c = reinterpret_cast<Sc::ArticulationDriveCache&>(cache);
	PX_CHECK_AND_RETURN(mImpl.mArticulation.getScArticulation().getCacheLinkCount(c) == mImpl.mArticulationLinks.size(), "PxArticulation::updateDriveCache: Articulation size has changed; drive cache is invalid");

	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads
	mImpl.mArticulation.getScArticulation().updateDriveCache(c, compliance, driveIterations);
}

void NpArticulation::releaseDriveCache(PxArticulationDriveCache&cache ) const
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::releaseDriveCache: object must be in a scene");
	NP_READ_CHECK(mImpl.getOwnerScene());	// doesn't modify the scene, only reads

	mImpl.mArticulation.getScArticulation().releaseDriveCache(reinterpret_cast<Sc::ArticulationDriveCache&>(cache));
}

void NpArticulation::applyImpulse(PxArticulationLink* link,
								  const PxArticulationDriveCache& driveCache,
								  const PxVec3& force,
								  const PxVec3& torque)
{
	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::applyImpulse: object must be in a scene");
	PX_CHECK_AND_RETURN(force.isFinite() && torque.isFinite(), "PxArticulation::applyImpulse: invalid force/torque");
	const Sc::ArticulationDriveCache& c = reinterpret_cast<const Sc::ArticulationDriveCache&>(driveCache);
	PX_CHECK_AND_RETURN(mImpl.mArticulation.getScArticulation().getCacheLinkCount(c) == mImpl.mArticulationLinks.size(), "PxArticulation::applyImpulse: Articulation size has changed; drive cache is invalid");

	NP_WRITE_CHECK(mImpl.getOwnerScene());

	if(mImpl.isSleeping())
		mImpl.wakeUp();

	mImpl.mArticulation.getScArticulation().applyImpulse(static_cast<NpArticulationLink*>(link)->getScbBodyFast().getScBody(), c,force, torque);
	for(PxU32 i=0;i<mImpl.mArticulationLinks.size();i++)
	{
		PxVec3 lv = mImpl.mArticulationLinks[i]->getScbBodyFast().getScBody().getLinearVelocity(),
			   av = mImpl.mArticulationLinks[i]->getScbBodyFast().getScBody().getAngularVelocity();
		mImpl.mArticulationLinks[i]->setLinearVelocity(lv);
		mImpl.mArticulationLinks[i]->setAngularVelocity(av);
	}
}

void NpArticulation::computeImpulseResponse(PxArticulationLink* link,
											PxVec3& linearResponse, 
											PxVec3& angularResponse,
											const PxArticulationDriveCache& driveCache,
											const PxVec3& force,
											const PxVec3& torque) const
{

	PX_CHECK_AND_RETURN(mImpl.getAPIScene(), "PxArticulation::computeImpulseResponse: object must be in a scene");
	PX_CHECK_AND_RETURN(force.isFinite() && torque.isFinite(), "PxArticulation::computeImpulseResponse: invalid force/torque");
	NP_READ_CHECK(mImpl.getOwnerScene());

	const Sc::ArticulationDriveCache& c = reinterpret_cast<const  Sc::ArticulationDriveCache&>(driveCache);
	PX_CHECK_AND_RETURN(mImpl.mArticulation.getScArticulation().getCacheLinkCount(c) == mImpl.mArticulationLinks.size(), "PxArticulation::computeImpulseResponse: Articulation size has changed; drive cache is invalid");
	PX_UNUSED(&c);

	mImpl.mArticulation.getScArticulation().computeImpulseResponse(static_cast<NpArticulationLink*>(link)->getScbBodyFast().getScBody(),
															 linearResponse, angularResponse,
															 reinterpret_cast<const Sc::ArticulationDriveCache&>(driveCache),
															 force, torque);
}

Scb::Body* NpArticulationGetRootFromScb(Scb::Articulation&c)
{
	const size_t offset = size_t(&(reinterpret_cast<NpArticulation*>(0)->mImpl.getScbArticulation()));
	NpArticulation* np = reinterpret_cast<NpArticulation*>(reinterpret_cast<char*>(&c) - offset);

	NpArticulationLink* a = np->mImpl.getRoot();
	
	return a ? &a->getScbBodyFast() : NULL;
}

PxArticulationJointBase* NpArticulation::createArticulationJoint(PxArticulationLink& parent,
	const PxTransform& parentFrame,
	PxArticulationLink& child,
	const PxTransform& childFrame)
{
	return NpFactory::getInstance().createNpArticulationJoint(static_cast<NpArticulationLink&>(parent), parentFrame, static_cast<NpArticulationLink&>(child), childFrame);
}
void NpArticulation::releaseArticulationJoint(PxArticulationJointBase* joint)
{
	NpFactory::getInstance().releaseArticulationJointToPool(*static_cast<NpArticulationJoint*>(joint));
}

}
