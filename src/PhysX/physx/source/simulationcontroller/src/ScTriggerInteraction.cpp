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

#include "ScTriggerInteraction.h"
#include "ScBodySim.h"
#include "ScNPhaseCore.h"

using namespace physx;
using namespace Sc;

TriggerInteraction::TriggerInteraction(	ShapeSim& tShape, ShapeSim& oShape) :
										ElementSimInteraction(tShape, oShape, InteractionType::eTRIGGER, InteractionFlag::eRB_ELEMENT | InteractionFlag::eFILTERABLE),
										mFlags(PROCESS_THIS_FRAME),
										mLastFrameHadContacts(false)
{
	// The PxPairFlags eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST get stored and mixed up with internal flags. Make sure any breaking change gets noticed.
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_FOUND < PxPairFlag::eNOTIFY_TOUCH_LOST);
	PX_COMPILE_TIME_ASSERT((PAIR_FLAGS_MASK & PxPairFlag::eNOTIFY_TOUCH_FOUND) == PxPairFlag::eNOTIFY_TOUCH_FOUND);
	PX_COMPILE_TIME_ASSERT((PAIR_FLAGS_MASK & PxPairFlag::eNOTIFY_TOUCH_LOST) == PxPairFlag::eNOTIFY_TOUCH_LOST);
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_FOUND < 0xffff);
	PX_COMPILE_TIME_ASSERT(PxPairFlag::eNOTIFY_TOUCH_LOST < 0xffff);
	PX_COMPILE_TIME_ASSERT(LAST < 0xffff);

	bool active = registerInActors();
	Scene& scene = getScene();
	scene.registerInteraction(this, active);
	scene.getNPhaseCore()->registerInteraction(this);

	PX_ASSERT(getTriggerShape().getFlags() & PxShapeFlag::eTRIGGER_SHAPE);
	mTriggerCache.state = Gu::TRIGGER_DISJOINT;
}

TriggerInteraction::~TriggerInteraction()
{
	Scene& scene = getScene();
	scene.unregisterInteraction(this);
	scene.getNPhaseCore()->unregisterInteraction(this);
	unregisterFromActors();
}

static bool isOneActorActive(TriggerInteraction* trigger)
{
	const BodySim* bodySim0 = trigger->getTriggerShape().getBodySim();
	if(bodySim0 && bodySim0->isActive())
	{
		PX_ASSERT(!bodySim0->isKinematic() || bodySim0->readInternalFlag(BodySim::BF_KINEMATIC_MOVED) || 
					bodySim0->readInternalFlag(BodySim::InternalFlags(BodySim::BF_KINEMATIC_SETTLING | BodySim::BF_KINEMATIC_SETTLING_2)));
		return true;
	}

	const BodySim* bodySim1 = trigger->getOtherShape().getBodySim();
	if(bodySim1 && bodySim1->isActive())
	{
		PX_ASSERT(!bodySim1->isKinematic() || bodySim1->readInternalFlag(BodySim::BF_KINEMATIC_MOVED) || 
			bodySim1->readInternalFlag(BodySim::InternalFlags(BodySim::BF_KINEMATIC_SETTLING | BodySim::BF_KINEMATIC_SETTLING_2)));
		return true;
	}

	return false;
}

//
// Some general information about triggers and sleeping
//
// The goal is to avoid running overlap tests if both objects are sleeping.
// This is an optimization for eNOTIFY_TOUCH_LOST events since the overlap state 
// can not change if both objects are sleeping. eNOTIFY_TOUCH_FOUND should be sent nonetheless.
// For this to work the following assumptions are made:
// - On creation or if the pose of an actor is set, the pair will always be checked.
// - If the scenario above does not apply, then a trigger pair can only be deactivated, if both actors are sleeping.
// - If an overlapping actor is activated/deactivated, the trigger interaction gets notified
//
bool TriggerInteraction::onActivate_(void*)
{
	// IMPORTANT: this method can get called concurrently from multiple threads -> make sure shared resources
	//            are protected (note: there are none at the moment but it might change)

	if(!(readFlag(PROCESS_THIS_FRAME)))
	{
		if(isOneActorActive(this))
		{
			raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
			return true;
		}
		else
			return false;
	}
	else
	{
		raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
		return true;  // newly created trigger pairs should always test for overlap, no matter the sleep state
	}
}

bool TriggerInteraction::onDeactivate_()
{
	if(!readFlag(PROCESS_THIS_FRAME))
	{
		if(!isOneActorActive(this))
		{
			clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
			return true;
		}
		else
			return false;
	}
	else
		return false;
}
