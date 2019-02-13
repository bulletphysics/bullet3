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

#include "ScConstraintInteraction.h"
#include "ScConstraintSim.h"
#include "ScBodySim.h"
#include "ScScene.h"
#include "PxsRigidBody.h"
#include "PxsSimpleIslandManager.h"

using namespace physx;
using namespace Sc;

ConstraintInteraction::ConstraintInteraction(ConstraintSim* constraint, RigidSim& r0, RigidSim& r1) :
	Interaction	(r0, r1, InteractionType::eCONSTRAINTSHADER, InteractionFlag::eCONSTRAINT),
	mConstraint	(constraint)
{
	registerInActors();

	BodySim* b0 = mConstraint->getBody(0);
	BodySim* b1 = mConstraint->getBody(1);

	if(b0)
		b0->onConstraintAttach();
	if(b1)
		b1->onConstraintAttach();

	IG::SimpleIslandManager* simpleIslandManager = getScene().getSimpleIslandManager();
	mEdgeIndex = simpleIslandManager->addConstraint(&mConstraint->getLowLevelConstraint(), b0 ? b0->getNodeIndex() : IG::NodeIndex(), b1 ? b1->getNodeIndex() : IG::NodeIndex(), this);
}

ConstraintInteraction::~ConstraintInteraction()
{
	PX_ASSERT(!readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST));
	PX_ASSERT(!getDirtyFlags());
	PX_ASSERT(!mConstraint->readFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED));
}

static PX_FORCE_INLINE void	removeFromActiveBreakableList(ConstraintSim* constraint, Scene& s)
{
	if(constraint->readFlag(ConstraintSim::eBREAKABLE | ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED) == (ConstraintSim::eBREAKABLE | ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED))
		s.removeActiveBreakableConstraint(constraint);
}

void ConstraintInteraction::destroy()
{
	setClean(true);  // removes the pair from the dirty interaction list etc.

	Scene& scene = getScene();

	removeFromActiveBreakableList(mConstraint, scene);

	if(mEdgeIndex != IG_INVALID_EDGE)
		scene.getSimpleIslandManager()->removeConnection(mEdgeIndex);
	mEdgeIndex = IG_INVALID_EDGE;

	unregisterFromActors();

	BodySim* b0 = mConstraint->getBody(0);
	BodySim* b1 = mConstraint->getBody(1);

	if(b0)
		b0->onConstraintDetach();  // Note: Has to be done AFTER the interaction has unregistered from the actors
	if(b1)
		b1->onConstraintDetach();  // Note: Has to be done AFTER the interaction has unregistered from the actors

	clearInteractionFlag(InteractionFlag::eIS_ACTIVE);  // ensures that broken constraints do not go into the list of active breakable constraints anymore
}

void ConstraintInteraction::updateState()
{
	PX_ASSERT(!mConstraint->isBroken());
	PX_ASSERT(getDirtyFlags() & InteractionDirtyFlag::eBODY_KINEMATIC);  // at the moment this should be the only reason for this method being called

	// at least one of the bodies got switched from kinematic to dynamic. This will not have changed the sleep state of the interactions, so the
	// constraint interactions are just marked dirty and processed as part of the dirty interaction update system.
	// 
	// -> need to check whether to activate the constraint and whether constraint break testing
	//    is now necessary
	//
	// the transition from dynamic to kinematic will always trigger an onDeactivate_() (because the body gets deactivated)
	// and thus there is no need to consider that case here.
	//

	onActivate_(NULL);	// note: this will not activate if the necessary conditions are not met, so it can be called even if the pair has been deactivated again before the
					//       simulation step started
}

bool ConstraintInteraction::onActivate_(void*)
{
	PX_ASSERT(!mConstraint->isBroken());

	BodySim* b0 = mConstraint->getBody(0);
	BodySim* b1 = mConstraint->getBody(1);

	const bool b0Vote = !b0 || b0->isActive();
	const bool b1Vote = !b1 || b1->isActive();

	const bool b0Dynamic = b0 && (!b0->isKinematic());
	const bool b1Dynamic = b1 && (!b1->isKinematic());

	//
	// note: constraints between kinematics and kinematics/statics are always inactive and must not be activated
	//
	if((b0Vote || b1Vote) && (b0Dynamic || b1Dynamic))
	{
		raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);

		if(mConstraint->readFlag(ConstraintSim::eBREAKABLE | ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED) == ConstraintSim::eBREAKABLE)
			getScene().addActiveBreakableConstraint(mConstraint, this);

		return true;
	}
	else
		return false;
}

bool ConstraintInteraction::onDeactivate_()
{
	const BodySim* b0 = mConstraint->getBody(0);
	const BodySim* b1 = mConstraint->getBody(1);

	const bool b0Dynamic = b0 && (!b0->isKinematic());
	const bool b1Dynamic = b1 && (!b1->isKinematic());

	PX_ASSERT(	(!b0 && b1 && !b1->isActive()) || 
				(!b1 && b0 && !b0->isActive()) ||
				((b0 && b1 && (!b0->isActive() || !b1->isActive()))) );

	//
	// note: constraints between kinematics and kinematics/statics should always get deactivated
	//
	if(((!b0 || !b0->isActive()) && (!b1 || !b1->isActive())) || (!b0Dynamic && !b1Dynamic))
	{
		removeFromActiveBreakableList(mConstraint, getScene());

		clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
		
		return true;
	}
	else
		return false;
}
