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

#include "ScArticulationJointSim.h"
#include "ScArticulationJointCore.h"
#include "ScBodySim.h"
#include "ScScene.h"
#include "PxsRigidBody.h"
#include "DyArticulation.h"
#include "ScArticulationSim.h"
#include "PxsSimpleIslandManager.h"

using namespace physx;

Sc::ArticulationJointSim::ArticulationJointSim(ArticulationJointCore& joint, ActorSim& parent, ActorSim& child) :
	Interaction	(parent, child, InteractionType::eARTICULATION, 0),
	mCore		(joint)
{
	registerInActors();

	BodySim& childBody = static_cast<BodySim&>(child),
		   & parentBody = static_cast<BodySim&>(parent);

	parentBody.getArticulation()->addBody(childBody, &parentBody, this);

	mCore.setSim(this);
}

Sc::ArticulationJointSim::~ArticulationJointSim()
{
	// articulation interactions do not make use of the dirty flags yet. If they did, a setClean(true) has to be introduced here.
	PX_ASSERT(!readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST));
	PX_ASSERT(!getDirtyFlags());

	unregisterFromActors();

	BodySim& child = getChild();
	child.getArticulation()->removeBody(child);

	mCore.setSim(NULL);
}

Sc::BodySim& Sc::ArticulationJointSim::getParent() const
{
	return static_cast<BodySim&>(getActorSim0());
}

Sc::BodySim& Sc::ArticulationJointSim::getChild() const
{
	return static_cast<BodySim&>(getActorSim1());
}

bool Sc::ArticulationJointSim::onActivate_(void*)
{
	if(!(getParent().isActive() && getChild().isActive()))
		return false;

	raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
	return true; 
}

bool Sc::ArticulationJointSim::onDeactivate_()
{
	clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
	return true;
}
