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

#ifndef PX_PHYSICS_SCP_INTERACTION
#define PX_PHYSICS_SCP_INTERACTION

#include "foundation/Px.h"
#include "ScInteractionFlags.h"
#include "ScScene.h"
#include "ScActorSim.h"
#include "PsUserAllocated.h"
#include "PsUtilities.h"
#include "PsFoundation.h"

namespace physx
{
#define PX_INVALID_INTERACTION_ACTOR_ID 0xffffffff
#define PX_INVALID_INTERACTION_SCENE_ID 0xffffffff

namespace Sc
{
	// Interactions are used for connecting actors into activation groups. An interaction always connects exactly two actors. 
	// An interaction is implicitly active if at least one of the two actors it connects is active.

	class Interaction : public Ps::UserAllocated
	{
		PX_NOCOPY(Interaction)

	protected:
										Interaction(ActorSim& actor0, ActorSim& actor1, InteractionType::Enum interactionType, PxU8 flags);
										~Interaction() { PX_ASSERT(!readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST)); }

	public:
		// Interactions automatically register themselves in the actors here
		PX_FORCE_INLINE bool			registerInActors(void* data = NULL);

		// Interactions automatically unregister themselves from the actors here
		PX_FORCE_INLINE void			unregisterFromActors();

		PX_FORCE_INLINE	ActorSim&		getActorSim0()	const	{ return mActor0; }
		PX_FORCE_INLINE	ActorSim&		getActorSim1()	const	{ return mActor1; }

		PX_FORCE_INLINE Scene&			getScene() const { return mActor0.getScene(); }

		PX_FORCE_INLINE	InteractionType::Enum getType() const { return InteractionType::Enum(mInteractionType); }

		PX_FORCE_INLINE PxU8			readInteractionFlag(PxU8 flag) const { return PxU8(mInteractionFlags & flag); }
		PX_FORCE_INLINE void			raiseInteractionFlag(InteractionFlag::Enum flag) { mInteractionFlags |= flag; }
		PX_FORCE_INLINE void			clearInteractionFlag(InteractionFlag::Enum flag) { mInteractionFlags &= ~flag; }

		/**
		\brief Mark the interaction as dirty. This will put the interaction into a list that is processed once per simulation step.

		@see InteractionDirtyFlag
		*/
		PX_FORCE_INLINE void			setDirty(PxU32 dirtyFlags);

		/**
		\brief Clear all flags that mark the interaction as dirty and optionally remove the interaction from the list of dirty interactions.

		@see InteractionDirtyFlag
		*/
		/*PX_FORCE_INLINE*/ void		setClean(bool removeFromList);

		PX_FORCE_INLINE Ps::IntBool		needsRefiltering() const { return (getDirtyFlags() & InteractionDirtyFlag::eFILTER_STATE); }

		PX_FORCE_INLINE Ps::IntBool		isElementInteraction() const;

		// Called when an interaction is activated or created.
		// Return true if activation should proceed else return false (for example: joint interaction between two kinematics should not get activated)
//		virtual			bool			onActivate_(void* data) = 0;

		// Called when an interaction is deactivated.
		// Return true if deactivation should proceed else return false (for example: joint interaction between two kinematics can ignore deactivation because it always is deactivated)
//		virtual			bool			onDeactivate_() = 0;

		PX_FORCE_INLINE	void			setInteractionId(PxU32 id)	{ mSceneId = id;										}
		PX_FORCE_INLINE	PxU32			getInteractionId()	const	{ return mSceneId;										}
		PX_FORCE_INLINE	bool			isRegistered()		const	{ return mSceneId != PX_INVALID_INTERACTION_SCENE_ID;	}

		PX_FORCE_INLINE	void			setActorId(ActorSim* actor, PxU32 id);
		PX_FORCE_INLINE	PxU32			getActorId(const ActorSim* actor) const;

		PX_FORCE_INLINE PxU8			getDirtyFlags() const { return mDirtyFlags; }

	private:
						void			addToDirtyList();
						void			removeFromDirtyList();

						ActorSim&		mActor0;
						ActorSim&		mActor1;

		// PT: TODO: merge the 6bits of the 3 PxU8s in the top bits of the 3 PxU32s
						PxU32			mSceneId;	// PT: TODO: merge this with mInteractionType

		// PT: TODO: are those IDs even worth caching? Since the number of interactions per actor is (or should be) small,
		// we could just do a linear search and save memory here...
						PxU32			mActorId0;	// PT: id of this interaction within mActor0's mInteractions array
						PxU32			mActorId1;	// PT: id of this interaction within mActor1's mInteractions array
	protected:
						PxU8			mInteractionType;	// PT: stored on a byte to save space, should be InteractionType enum, 5/6 bits needed here
						PxU8			mInteractionFlags;	// PT: 6 bits needed here, see InteractionFlag enum
						PxU8			mDirtyFlags;		// PT: 6 bits needed here, see InteractionDirtyFlag enum
						PxU8			mPadding;
	};

} // namespace Sc

//////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE bool Sc::Interaction::registerInActors(void* data)
{
	bool active = activateInteraction(this, data);

	mActor0.registerInteractionInActor(this);
	mActor1.registerInteractionInActor(this);

	return active;
}

PX_FORCE_INLINE void Sc::Interaction::unregisterFromActors()
{
	mActor0.unregisterInteractionFromActor(this);
	mActor1.unregisterInteractionFromActor(this);
}

PX_FORCE_INLINE	void Sc::Interaction::setActorId(ActorSim* actor, PxU32 id)
{
	PX_ASSERT(id != PX_INVALID_INTERACTION_ACTOR_ID);
	PX_ASSERT(&mActor0 == actor || &mActor1 == actor);
	if(&mActor0 == actor)
		mActorId0 = id;
	else
		mActorId1 = id;
}

PX_FORCE_INLINE	PxU32 Sc::Interaction::getActorId(const ActorSim* actor) const
{
	PX_ASSERT(&mActor0 == actor || &mActor1 == actor);
	return &mActor0 == actor ? mActorId0 : mActorId1;
}

PX_FORCE_INLINE Ps::IntBool Sc::Interaction::isElementInteraction() const
{
	const Ps::IntBool res = readInteractionFlag(InteractionFlag::eRB_ELEMENT);
	PX_ASSERT(	(res && 
				((getType() == InteractionType::eOVERLAP)	|| 
				(getType() == InteractionType::eTRIGGER)	|| 
				(getType() == InteractionType::eMARKER))) ||
				(!res && 
				((getType() == InteractionType::eCONSTRAINTSHADER)	|| 
				(getType() == InteractionType::eARTICULATION))));

	return res;
}

PX_FORCE_INLINE void Sc::Interaction::setDirty(PxU32 dirtyFlags)
{
	PX_ASSERT(getType() != InteractionType::eARTICULATION);

	mDirtyFlags |= Ps::to8(dirtyFlags);
	if(!readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
	{
		addToDirtyList();
		raiseInteractionFlag(InteractionFlag::eIN_DIRTY_LIST);
	}
}

//PX_FORCE_INLINE void Sc::Interaction::setClean(bool removeFromList)
//{
//	if (readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST))
//	{
//		if (removeFromList)  // if we process all dirty interactions anyway, then we can just clear the list at the end and save the work here.
//			removeFromDirtyList();
//		clearInteractionFlag(InteractionFlag::eIN_DIRTY_LIST);
//	}
//
//	mDirtyFlags = 0;
//}


}

#endif // PX_PHYSICS_SCP_INTERACTION
