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

#include "CmPhysXCommon.h"
#include "ScActorSim.h"
#include "ScActorCore.h"
#include "ScElementSim.h"
#include "ScScene.h"
#include "ScInteraction.h"

using namespace physx;

Sc::ActorSim::ActorSim(Scene& scene, ActorCore& core) :
	mFirstElement	(NULL),
	mElementCount	(0),
	mScene			(scene),
	mCore			(core)
{
	core.setSim(this);
}

Sc::ActorSim::~ActorSim()
{
	mInteractions.releaseMem(*this);
}

void Sc::ActorSim::registerInteractionInActor(Interaction* interaction)
{
	const PxU32 id = mInteractions.size();
	mInteractions.pushBack(interaction, *this);
	interaction->setActorId(this, id);
}

void Sc::ActorSim::unregisterInteractionFromActor(Interaction* interaction)
{
	const PxU32 i = interaction->getActorId(this);
	PX_ASSERT(i < mInteractions.size());
	mInteractions.replaceWithLast(i); 
	if (i<mInteractions.size())
		mInteractions[i]->setActorId(this, i);
}

void Sc::ActorSim::onElementAttach(ElementSim& element)
{
	element.mNextInActor = mFirstElement;
	mFirstElement = &element;
	mElementCount++;
}

void Sc::ActorSim::onElementDetach(ElementSim& element)
{
	PX_ASSERT(mFirstElement);	// PT: else we shouldn't be called
	ElementSim* currentElem = mFirstElement;
	ElementSim* previousElem = NULL;
	while(currentElem)
	{
		if(currentElem==&element)
		{
			if(previousElem)
				previousElem->mNextInActor = currentElem->mNextInActor;
			else
				mFirstElement = currentElem->mNextInActor;
			mElementCount--;
			return;
		}
		previousElem = currentElem;
		currentElem = currentElem->mNextInActor;
	}
	PX_ASSERT(0);
}

// PT: TODO: refactor with Sc::ParticlePacketShape::reallocInteractions - sschirm: particles are gone
void Sc::ActorSim::reallocInteractions(Sc::Interaction**& mem, PxU32& capacity, PxU32 size, PxU32 requiredMinCapacity)
{
	Interaction** newMem;
	PxU32 newCapacity;

	if(requiredMinCapacity==0)
	{
		newCapacity = 0;
		newMem = 0;
	}
	else if(requiredMinCapacity<=INLINE_INTERACTION_CAPACITY)
	{
		newCapacity = INLINE_INTERACTION_CAPACITY;
		newMem = mInlineInteractionMem;
	}
	else
	{
		newCapacity = Ps::nextPowerOfTwo(requiredMinCapacity-1);
		newMem = reinterpret_cast<Interaction**>(mScene.allocatePointerBlock(newCapacity));
	}

	PX_ASSERT(newCapacity >= requiredMinCapacity && requiredMinCapacity>=size);

	if(mem)
	{
		PxMemCopy(newMem, mem, size*sizeof(Interaction*));

		if(mem!=mInlineInteractionMem)
			mScene.deallocatePointerBlock(reinterpret_cast<void**>(mem), capacity);
	}
	
	capacity = newCapacity;
	mem = newMem;
}

void Sc::ActorSim::setActorsInteractionsDirty(InteractionDirtyFlag::Enum flag, const ActorSim* other, PxU8 interactionFlag)
{
	PxU32 size = getActorInteractionCount();
	Interaction** interactions = getActorInteractions();
	while(size--)
	{
		Interaction* interaction = *interactions++;
		if((!other || other == &interaction->getActorSim0() || other == &interaction->getActorSim1()) && (interaction->readInteractionFlag(interactionFlag)))
			interaction->setDirty(flag);
	}
}
