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

#ifndef PX_PHYSICS_SCP_ACTOR_SIM
#define PX_PHYSICS_SCP_ACTOR_SIM

#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "CmUtils.h"
#include "PxActor.h"
#include "ScInteractionFlags.h"
#include "ScActorCore.h"

namespace physx
{

class PxActor;

namespace Sc
{
	class Interaction;
	class ElementSim;
	
	class ActorSim : public Ps::UserAllocated
	{
		friend class Scene;  // the scene is allowed to set the scene array index
		friend class Interaction;
		PX_NOCOPY(ActorSim)

	public:
		enum ActivityChangeInfoFlag
		{
			AS_PART_OF_CREATION				= (1 << 0),
			AS_PART_OF_ISLAND_GEN			= (1 << 1)
		};

											ActorSim(Scene&, ActorCore&);
		virtual								~ActorSim();

		// Get the scene the actor resides in
		PX_FORCE_INLINE	Scene&				getScene()					const	{ return mScene; }

		// Get the number of interactions connected to the actor
		PX_FORCE_INLINE	PxU32				getActorInteractionCount()	const	{ return mInteractions.size(); }

		// Get an iterator to the interactions connected to the actor
		PX_FORCE_INLINE	Interaction**		getActorInteractions()		const	{ return mInteractions.begin();	}

		// Get first element in the actor (linked list)
		PX_FORCE_INLINE	ElementSim*			getElements_()						{ return mFirstElement;		}
		PX_FORCE_INLINE	const ElementSim*	getElements_()				const	{ return mFirstElement;		}

		PX_FORCE_INLINE PxU32				getElementCount()			const	{ return mElementCount; }

		// Get the type ID of the actor
		PX_FORCE_INLINE	PxActorType::Enum	getActorType()				const	{ return mCore.getActorCoreType();	}

		// Returns true if the actor is a dynamic rigid body (including articulation links)
		PX_FORCE_INLINE	bool				isDynamicRigid()			const	{ const PxActorType::Enum type = getActorType(); return type == PxActorType::eRIGID_DYNAMIC || type == PxActorType::eARTICULATION_LINK; }

						void				onElementAttach(ElementSim& element);
						void				onElementDetach(ElementSim& element);

		virtual			void				postActorFlagChange(PxU32, PxU32) {}

						void				setActorsInteractionsDirty(InteractionDirtyFlag::Enum flag, const ActorSim* other, PxU8 interactionFlag);

		PX_FORCE_INLINE	ActorCore&			getActorCore() const { return mCore; }

	private:
		//These are called from interaction creation/destruction
						void				registerInteractionInActor(Interaction* interaction);
						void				unregisterInteractionFromActor(Interaction* interaction);

						void				reallocInteractions(Sc::Interaction**& mem, PxU32& capacity, PxU32 size, PxU32 requiredMinCapacity);
	protected:
		// dsequeira: interaction arrays are a major cause of small allocations, so we don't want to delegate them to the heap allocator
		// it's not clear this inline array is really needed, we should take it out and see whether the cache perf is worse

		static const PxU32 INLINE_INTERACTION_CAPACITY = 4;
						Interaction*		mInlineInteractionMem[INLINE_INTERACTION_CAPACITY];

		Cm::OwnedArray<Sc::Interaction*, Sc::ActorSim, PxU32, &Sc::ActorSim::reallocInteractions>
											mInteractions;

						ElementSim*			mFirstElement;
						PxU32				mElementCount;

						Scene&				mScene;

						ActorCore&			mCore;
	};

} // namespace Sc

}

#endif
