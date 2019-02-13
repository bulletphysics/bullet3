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

#include "foundation/Px.h"

#include "ScInteraction.h"
#include "ScNPhaseCore.h"

using namespace physx;

Sc::Interaction::Interaction(ActorSim& actor0, ActorSim& actor1, InteractionType::Enum type, PxU8 flags) :
	mActor0				(actor0),
	mActor1				(actor1), 
	mSceneId			(PX_INVALID_INTERACTION_SCENE_ID), 
	mActorId0			(PX_INVALID_INTERACTION_ACTOR_ID),
	mActorId1			(PX_INVALID_INTERACTION_ACTOR_ID), 
	mInteractionType	(Ps::to8(type)),
	mInteractionFlags	(flags),
	mDirtyFlags			(0)
{
	PX_ASSERT_WITH_MESSAGE(&actor0.getScene() == &actor1.getScene(),"Cannot create an interaction between actors belonging to different scenes.");
	PX_ASSERT(PxU32(type)<256);	// PT: type is now stored on a byte
}

void Sc::Interaction::addToDirtyList()
{
	getActorSim0().getScene().getNPhaseCore()->addToDirtyInteractionList(this);		
}

void Sc::Interaction::removeFromDirtyList()
{
	getActorSim0().getScene().getNPhaseCore()->removeFromDirtyInteractionList(this);
}

void Sc::Interaction::setClean(bool removeFromList)
{
	if (readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
	{
		if (removeFromList)  // if we process all dirty interactions anyway, then we can just clear the list at the end and save the work here.
			removeFromDirtyList();
		clearInteractionFlag(InteractionFlag::eIN_DIRTY_LIST);
	}

	mDirtyFlags = 0;
}
