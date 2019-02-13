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


#include "NpAggregate.h"

///////////////////////////////////////////////////////////////////////////////

#include "PxActor.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulation.h"
#include "NpActor.h"
#include "GuBVHStructure.h"
#include "CmUtils.h"
#include "NpArticulationTemplate.h"

using namespace physx;

PX_FORCE_INLINE void setAggregate(NpAggregate* aggregate, PxActor& actor)
{
	NpActor& np = NpActor::getFromPxActor(actor);
	np.setAggregate(aggregate, actor);
}

///////////////////////////////////////////////////////////////////////////////


NpAggregate::NpAggregate(PxU32 maxActors, bool selfCollisions)
: PxAggregate(PxConcreteType::eAGGREGATE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mAggregate(this, maxActors, selfCollisions)
, mNbActors(0)
{
	mActors = reinterpret_cast<PxActor**>(PX_ALLOC(sizeof(PxActor*)*maxActors, "PxActor*"));
}

NpAggregate::~NpAggregate()
{
	NpFactory::getInstance().onAggregateRelease(this);
	if(getBaseFlags()&PxBaseFlag::eOWNS_MEMORY)
		PX_FREE(mActors);
}

void NpAggregate::removeAndReinsert(PxActor& actor, bool reinsert)
{
	NpActor& np = NpActor::getFromPxActor(actor);
	Scb::Actor& scb = NpActor::getScbFromPxActor(actor);

	np.setAggregate(NULL, actor);
	
	mAggregate.removeActor(scb, reinsert);
}

void NpAggregate::release()
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_SIMD_GUARD;

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, NULL);

	/*
	"An aggregate should be empty when it gets released. If it isn't, the behavior should be: remove the actors from
	the aggregate, then remove the aggregate from the scene (if any) then delete it. I guess that implies the actors
	get individually reinserted into the broad phase if the aggregate is in a scene."
	*/
	for(PxU32 i=0;i<mNbActors;i++)
	{
		if (mActors[i]->getType() == PxActorType::eARTICULATION_LINK)
		{
			NpArticulationLink* link = static_cast<NpArticulationLink*>(mActors[i]);
			PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(link->getRoot().getImpl());
			impl->setAggregate(NULL);
		}

		removeAndReinsert(*mActors[i], true);
	}

	NpScene* s = getAPIScene();
	if(s)
	{
		s->getScene().removeAggregate(getScbAggregate());
		s->removeFromAggregateList(*this);
	}

	mAggregate.destroy();
}

void NpAggregate::addActorInternal(PxActor& actor, NpScene& s, const PxBVHStructure* bvhStructure)
{
	if (actor.getType() != PxActorType::eARTICULATION_LINK)
	{
		Scb::Actor& scb = NpActor::getScbFromPxActor(actor);

		mAggregate.addActor(scb);

		s.addActorInternal(actor, bvhStructure);
	}
	else if (!actor.getScene())  // This check makes sure that a link of an articulation gets only added once.
	{
		NpArticulationLink& al = static_cast<NpArticulationLink&>(actor);
		PxArticulationBase& npArt = al.getRoot();
		for(PxU32 i=0; i < npArt.getNbLinks(); i++)
		{
			PxArticulationLink* link;
			npArt.getLinks(&link, 1, i);
			mAggregate.addActor(static_cast<NpArticulationLink*>(link)->getScbActorFast());
		}

		s.addArticulationInternal(npArt);
	}
}

bool NpAggregate::addActor(PxActor& actor, const PxBVHStructure* bvhStructure)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_SIMD_GUARD;

	if(mNbActors==mAggregate.getMaxActorCount())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add actor to aggregate, max number of actors reached");
		return false;
	}

	if(actor.getAggregate())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add actor to aggregate, actor already belongs to an aggregate");
		return false;
	}

	if(actor.getScene())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add actor to aggregate, actor already belongs to a scene");
		return false;
	}

	if(actor.getType() == PxActorType::eARTICULATION_LINK)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add articulation link to aggregate, only whole articulations can be added");
		return false;
	}

	setAggregate(this, actor);

	mActors[mNbActors++] = &actor;

	// PT: when an object is added to a aggregate at runtime, i.e. when the aggregate has already been added to the scene,
	// we need to immediately add the newcomer to the scene as well.
	NpScene* s = getAPIScene();
	if(s)
	{
		addActorInternal(actor, *s, bvhStructure);
	}
	else
	{
		// A.B. if BVH structure is provided we need to keep it stored till the aggregate is inseted into a scene
		if(bvhStructure)
		{
			PxBVHStructure* bvhStructureMutable = const_cast<PxBVHStructure*>(bvhStructure);
			static_cast<Gu::BVHStructure*>(bvhStructureMutable)->incRefCount();
			NpActor::getFromPxActor(actor).addConnector(NpConnectorType::eBvhStructure, bvhStructureMutable, "PxBVHStructure already added to the PxActor!");
		}
	}

	return true;
}

bool NpAggregate::removeActorAndReinsert(PxActor& actor, bool reinsert)
{
	for(PxU32 i=0;i<mNbActors;i++)
	{
		if(mActors[i]==&actor)
		{
			mActors[i] = mActors[--mNbActors];
			removeAndReinsert(actor, reinsert);
			return true;
		}
	}
	Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't remove actor, actor doesn't belong to aggregate");
	return false;
}

bool NpAggregate::removeActor(PxActor& actor)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_SIMD_GUARD;

	if(actor.getType() == PxActorType::eARTICULATION_LINK)
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't remove articulation link, only whole articulations can be removed");
		return false;
	}

	// A.B. remove the BVH structure reference if there is and the aggregate was not added to a scene
	NpScene* s = getAPIScene();	
	if(!s)
	{
		NpActor& np = NpActor::getFromPxActor(actor);
		Gu::BVHStructure* bvhStructure = NULL;
		if(np.getConnectors<Gu::BVHStructure>(NpConnectorType::eBvhStructure, &bvhStructure, 1))
		{
			np.removeConnector(actor, NpConnectorType::eBvhStructure, bvhStructure, "PxBVHStructure connector could not have been removed!");
			bvhStructure->decRefCount();
		}
	}

	// PT: there are really 2 cases here:
	// a) the user just wants to remove the actor from the aggregate, but the actor is still alive so if the aggregate has been added to a scene,
	//    we must reinsert the removed actor to that same scene
	// b) this is called by the framework when releasing an actor, in which case we don't want to reinsert it anywhere.
	//
	// We assume that when called by the user, we always want to reinsert. The framework however will call the internal function
	// without reinsertion.
	return removeActorAndReinsert(actor, true);
}

bool NpAggregate::addArticulation(PxArticulationBase& art)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_SIMD_GUARD;

	if((mNbActors+art.getNbLinks()) > mAggregate.getMaxActorCount())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add articulation links, max number of actors reached");
		return false;
	}

	if(art.getAggregate())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add articulation to aggregate, articulation already belongs to an aggregate");
		return false;
	}

	if(art.getScene())
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't add articulation to aggregate, articulation already belongs to a scene");
		return false;
	}

	PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(art.getImpl());
	impl->setAggregate(this);
	NpArticulationLink* const* links = impl->getLinks();
	for(PxU32 i=0; i < impl->getNbLinks(); i++)
	{
		NpArticulationLink& l = *links[i];

		setAggregate(this, l);

		mActors[mNbActors++] = &l;

		mAggregate.addActor(l.getScbActorFast());
	}

	// PT: when an object is added to a aggregate at runtime, i.e. when the aggregate has already been added to the scene,
	// we need to immediately add the newcomer to the scene as well.
	NpScene* s = getAPIScene();
	if(s)
	{
		s->addArticulationInternal(art);
	}

	return true;
}

bool NpAggregate::removeArticulationAndReinsert(PxArticulationBase& art, bool reinsert)
{
	bool found = false;
	PxU32 idx = 0;
	while(idx < mNbActors)
	{
		if ((mActors[idx]->getType() == PxActorType::eARTICULATION_LINK) && (&static_cast<NpArticulationLink*>(mActors[idx])->getRoot() == &art))
		{
			PxActor* a = mActors[idx];
			mActors[idx] = mActors[--mNbActors];
			removeAndReinsert(*a, reinsert);
			found = true;
		}
		else
			idx++;
	}

	reinterpret_cast<PxArticulationImpl*>(art.getImpl())->setAggregate(NULL);

	if (!found)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxAggregate: can't remove articulation, articulation doesn't belong to aggregate");
	return found;
}

bool NpAggregate::removeArticulation(PxArticulationBase& art)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_SIMD_GUARD;

	// see comments in removeActor()
	return removeArticulationAndReinsert(art, true);
}

PxU32 NpAggregate::getNbActors() const
{
	NP_READ_CHECK(getOwnerScene());
	return mNbActors;
}

PxU32 NpAggregate::getMaxNbActors() const
{
	NP_READ_CHECK(getOwnerScene());
	return mAggregate.getMaxActorCount();
}

PxU32 NpAggregate::getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getOwnerScene());
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mActors, getCurrentSizeFast());
}

PxScene* NpAggregate::getScene()
{
	return getAPIScene();
}

NpScene* NpAggregate::getAPIScene() const
{
	return mAggregate.getScbSceneForAPI() ? static_cast<NpScene*>(mAggregate.getScbSceneForAPI()->getPxScene()) : NULL;
}

NpScene* NpAggregate::getOwnerScene() const
{
	return mAggregate.getScbScene() ? static_cast<NpScene*>(mAggregate.getScbScene()->getPxScene()) : NULL;
}

bool NpAggregate::getSelfCollision() const
{
	NP_READ_CHECK(getOwnerScene());
	return mAggregate.getSelfCollide();
}

// PX_SERIALIZATION
void NpAggregate::exportExtraData(PxSerializationContext& stream)
{
	if(mActors)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mActors, mNbActors * sizeof(PxActor*));
	}
}

void NpAggregate::importExtraData(PxDeserializationContext& context)
{
	if(mActors)
		mActors = context.readExtraData<PxActor*, PX_SERIAL_ALIGN>(mNbActors);
}

void NpAggregate::resolveReferences(PxDeserializationContext& context)
{
	// Resolve actor pointers if needed
	for(PxU32 i=0; i < mNbActors; i++)
	{
		context.translatePxBase(mActors[i]);
		{
			//update aggregate if mActors is in external reference
			NpActor& np = NpActor::getFromPxActor(*mActors[i]);
			if(np.getAggregate() == NULL)
			{
				np.setAggregate(this, *mActors[i]);
			}
			if(mActors[i]->getType() == PxActorType::eARTICULATION_LINK)
			{
				PxArticulationBase& articulation = static_cast<NpArticulationLink*>(mActors[i])->getRoot();
				if(!articulation.getAggregate())
					reinterpret_cast<PxArticulationImpl*>(articulation.getImpl())->setAggregate(this);
			}
		}
	}	
}

NpAggregate* NpAggregate::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpAggregate* obj = new (address) NpAggregate(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpAggregate);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void NpAggregate::requiresObjects(PxProcessPxBaseCallback& c)
{
	for(PxU32 i=0; i < mNbActors; i++)
	{
		PxArticulationLink* link = mActors[i]->is<PxArticulationLink>();
		if(link)
			c.process(link->getArticulation());
		else
			c.process(*mActors[i]);
	}
}
// ~PX_SERIALIZATION
