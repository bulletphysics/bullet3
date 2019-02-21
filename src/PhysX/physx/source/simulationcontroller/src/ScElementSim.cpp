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
#include "PxsContext.h"
#include "ScElementSim.h"
#include "ScElementSimInteraction.h"
#include "ScSqBoundsManager.h"
#include "ScSimStats.h"

using namespace physx;
using namespace Sc;

static PX_FORCE_INLINE bool interactionHasElement(const Interaction* it, const ElementSim* elem)
{
	if(it->readInteractionFlag(InteractionFlag::eRB_ELEMENT))
	{
		PX_ASSERT(	(it->getType() == InteractionType::eMARKER) ||
					(it->getType() == InteractionType::eOVERLAP) ||
					(it->getType() == InteractionType::eTRIGGER) );

		const ElementSimInteraction* ei = static_cast<const ElementSimInteraction*>(it);
		if((&ei->getElement0() == elem) || (&ei->getElement1() == elem))
			return true;
	}
	return false;
}

Sc::ElementSimInteraction* Sc::ElementSim::ElementInteractionIterator::getNext()
{
	while(mInteractions!=mInteractionsLast)
	{
		Interaction* it = *mInteractions++;
		if(interactionHasElement(it, mElement))
			return static_cast<ElementSimInteraction*>(it);
	}
	return NULL;
}

Sc::ElementSimInteraction* Sc::ElementSim::ElementInteractionReverseIterator::getNext()
{
	while(mInteractions!=mInteractionsLast)
	{
		Interaction* it = *--mInteractionsLast;
		if(interactionHasElement(it, mElement))
			return static_cast<ElementSimInteraction*>(it);
	}
	return NULL;
}

Sc::ElementSim::ElementSim(ActorSim& actor) :
	mNextInActor	(NULL),
	mActor			(actor),
	mInBroadPhase	(false)
{
	initID();

	actor.onElementAttach(*this);
}

Sc::ElementSim::~ElementSim()
{
	PX_ASSERT(!mInBroadPhase);
	releaseID();
	mActor.onElementDetach(*this);
}

void Sc::ElementSim::setElementInteractionsDirty(InteractionDirtyFlag::Enum flag, PxU8 interactionFlag)
{
	ElementSim::ElementInteractionIterator iter = getElemInteractions();
	ElementSimInteraction* interaction = iter.getNext();
	while(interaction)
	{
		if(interaction->readInteractionFlag(interactionFlag))
			interaction->setDirty(flag);

		interaction = iter.getNext();
	}
}

void Sc::ElementSim::addToAABBMgr(PxReal contactDistance, Bp::FilterGroup::Enum group, Ps::IntBool isTrigger)
{
	Sc::Scene& scene = getScene();
	if(!scene.getAABBManager()->addBounds(mElementID, contactDistance, group, this, mActor.getActorCore().getAggregateID(), isTrigger ? Bp::ElementType::eTRIGGER : Bp::ElementType::eSHAPE))
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Unable to create broadphase entity because only 32768 shapes are supported");
		return;
	}
	mInBroadPhase = true;
#if PX_ENABLE_SIM_STATS
	scene.getStatsInternal().incBroadphaseAdds();
#endif
}

void Sc::ElementSim::removeFromAABBMgr()
{
	PX_ASSERT(mInBroadPhase);
	Sc::Scene& scene = getScene();
	scene.getAABBManager()->removeBounds(mElementID);
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndReset(mElementID);

	mInBroadPhase = false;
#if PX_ENABLE_SIM_STATS
	scene.getStatsInternal().incBroadphaseRemoves();
#endif
}
