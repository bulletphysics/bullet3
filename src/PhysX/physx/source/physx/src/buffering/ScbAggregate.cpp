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

#include "ScbAggregate.h"
#include "ScbActor.h"

using namespace physx;

void Scb::Aggregate::addActor(Scb::Actor& actor)
{
	const ControlState::Enum state = getControlState();

	if(!isBufferingSpecial(state))
	{
		actor.getActorCore().setAggregateID(mAggregateID);
		PvdAttachActorToAggregate( this, &actor );
		PvdUpdateProperties( this );
	}
	else if((state != ControlState::eREMOVE_PENDING))  // If the aggregate is pending for deletion, adding/removing an actor should not be double buffered because the aggregateID must not be set for the actors
	{
		// if available, search in list of removed actors to cover the remove-add case
		Scb::AggregateBuffer* PX_RESTRICT bufferedData = getBufferedData();
		if(bufferedData->removeBufferIdx != 0xffffffff)
		{
			Scb::Actor** removeBuffer = getScbScene()->getActorBuffer(bufferedData->removeBufferIdx);
			for(PxU32 i=0; i<bufferedData->removeCount; i++)
			{
				if(removeBuffer[i] == &actor)
				{
					removeBuffer[i] = removeBuffer[bufferedData->removeCount - 1];
					PX_ASSERT(bufferedData->removeCount > 0);
					bufferedData->removeCount--;
					break;
				}
			}
		}

		Scb::Actor** actorBuffer;
		if(bufferedData->addBufferIdx == 0xffffffff)
			actorBuffer = getScbScene()->allocActorBuffer(mMaxNbActors, bufferedData->addBufferIdx);
		else
			actorBuffer = getScbScene()->getActorBuffer(bufferedData->addBufferIdx);

		PX_ASSERT(bufferedData->addCount < mMaxNbActors);
		actorBuffer[bufferedData->addCount] = &actor;
		bufferedData->addCount++;

		if(state != ControlState::eINSERT_PENDING)
			markUpdated(BF_ADD_ACTOR);
		else
		{
			// Not a great solution but aggregates are special in the sense that even in the pending insert case, data needs to be double buffered
			// (see isBufferingSpecial() for details)
			setBufferFlag(BF_ADD_ACTOR);
		}
	}
}

void Scb::Aggregate::removeActor(Scb::Actor& actor, bool reinsert)
{
	const ControlState::Enum state = getControlState();
	const ControlState::Enum actorState = actor.getControlState();

	if(!isBufferingSpecial(state))
	{
		Sc::ActorCore& ac = actor.getActorCore();
		ac.setAggregateID(PX_INVALID_U32);
		
		if(getScbSceneForAPI() && reinsert)
			ac.reinsertShapes();
	}
	else if((state != ControlState::eREMOVE_PENDING))
	{
		// if available, search in list of added actors to cover the add-remove case
		Scb::AggregateBuffer* PX_RESTRICT bufferedData = getBufferedData();
		if(bufferedData->addBufferIdx != 0xffffffff)
		{
			Scb::Actor** addBuffer = getScbScene()->getActorBuffer(bufferedData->addBufferIdx);
			for(PxU32 i=0; i<bufferedData->addCount; i++)
			{
				if(addBuffer[i] == &actor)
				{
					addBuffer[i] = addBuffer[bufferedData->addCount - 1];
					PX_ASSERT(bufferedData->addCount > 0);
					bufferedData->addCount--;
					return;  // It's fine to abort here because the aggregateID has not been set yet
				}
			}
		}

		Scb::Actor** actorBuffer;
		if(bufferedData->removeBufferIdx == 0xffffffff)
			actorBuffer = getScbScene()->allocActorBuffer(mMaxNbActors, bufferedData->removeBufferIdx);
		else
			actorBuffer = getScbScene()->getActorBuffer(bufferedData->removeBufferIdx);

		PX_ASSERT(bufferedData->removeCount < mMaxNbActors);
		actorBuffer[bufferedData->removeCount] = &actor;
		bufferedData->removeCount++;

		markUpdated(BF_REMOVE_ACTOR);
	}

	//Update pvd status if not buffer OR the actor release while aggregate is already in scene
	if(!isBufferingSpecial(state) 
		|| ((actorState == ControlState::eIN_SCENE || actorState == ControlState::eREMOVE_PENDING ) 
			&& state != ControlState::eINSERT_PENDING
			&& !reinsert))
	{
		PvdDetachActorFromAggregate( this, &actor );
		PvdUpdateProperties( this );
	}
}
