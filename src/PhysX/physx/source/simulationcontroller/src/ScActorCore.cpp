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

#include "ScActorCore.h"
#include "ScActorSim.h"
#include "ScShapeCore.h"
#include "ScShapeSim.h"
#include "ScBodySim.h"

using namespace physx;

Sc::ActorCore::ActorCore(PxActorType::Enum actorType, PxU8 actorFlags, PxClientID owner, PxDominanceGroup dominanceGroup) :
	mSim					(NULL),
	mAggregateIDOwnerClient	((PxU32(owner)<<24)|0x00ffffff),
	mActorFlags				(actorFlags),
	mActorType				(PxU8(actorType)),
	mDominanceGroup			(dominanceGroup)
{
	PX_ASSERT((actorType & 0xff) == actorType);
}

Sc::ActorCore::~ActorCore()
{
}

void Sc::ActorCore::setActorFlags(PxActorFlags af)	
{ 
	const PxActorFlags old = mActorFlags;
	if(af!=old)
	{
		mActorFlags = af;

		if(mSim)
			mSim->postActorFlagChange(old, af);
	}
}	

void Sc::ActorCore::setDominanceGroup(PxDominanceGroup g)
{
	PX_ASSERT(g<128);
	mDominanceGroup = g;
	if(mSim)
	{
		//force all related interactions to refresh, so they fetch new dominance values.
		mSim->setActorsInteractionsDirty(InteractionDirtyFlag::eDOMINANCE, NULL, InteractionFlag::eRB_ELEMENT);
	}
}

void Sc::ActorCore::reinsertShapes()
{
	PX_ASSERT(mSim);
	if(!mSim)
		return;

	ElementSim* current = mSim->getElements_();
	while(current)
	{
		static_cast<ShapeSim*>(current)->reinsertBroadPhase();
		current = current->mNextInActor;
	}
}
