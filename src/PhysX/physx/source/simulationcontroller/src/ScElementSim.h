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

#ifndef PX_PHYSICS_SCP_ELEMENT_SIM
#define PX_PHYSICS_SCP_ELEMENT_SIM

#include "PsUserAllocated.h"
#include "PxFiltering.h"
#include "PxvConfig.h"
#include "ScActorSim.h"
#include "ScInteraction.h"
#include "BpAABBManager.h"
#include "ScObjectIDTracker.h"

namespace physx
{
namespace Sc
{
	class ElementSimInteraction;

	/*
	A ElementSim is a part of a ActorSim. It contributes to the activation framework by adding its 
	interactions to the actor. */
	class ElementSim : public Ps::UserAllocated
	{
		PX_NOCOPY(ElementSim)

	public:
		class ElementInteractionIterator
		{
			public:
				PX_FORCE_INLINE			ElementInteractionIterator(const ElementSim& e, PxU32 nbInteractions, Interaction** interactions) :
					mInteractions(interactions), mInteractionsLast(interactions + nbInteractions), mElement(&e) {}
				ElementSimInteraction*	getNext();

			private:
				Interaction**					mInteractions;
				Interaction**					mInteractionsLast;
				const ElementSim*				mElement;
		};

		class ElementInteractionReverseIterator
		{
			public:
				PX_FORCE_INLINE			ElementInteractionReverseIterator(const ElementSim& e, PxU32 nbInteractions, Interaction** interactions) :
					mInteractions(interactions), mInteractionsLast(interactions + nbInteractions), mElement(&e) {}
				ElementSimInteraction*	getNext();

			private:
				Interaction**					mInteractions;
				Interaction**					mInteractionsLast;
				const ElementSim*				mElement;
		};

												ElementSim(ActorSim& actor);
		protected:
												~ElementSim();
		public:

		// Get an iterator to the interactions connected to the element
		PX_FORCE_INLINE	ElementInteractionIterator getElemInteractions()	const	{ return ElementInteractionIterator(*this, mActor.getActorInteractionCount(), mActor.getActorInteractions()); }
		PX_FORCE_INLINE	ElementInteractionReverseIterator getElemInteractionsReverse()	const	{ return ElementInteractionReverseIterator(*this, mActor.getActorInteractionCount(), mActor.getActorInteractions()); }

		PX_FORCE_INLINE	ActorSim&				getActor()					const	{ return mActor; }

		PX_FORCE_INLINE	Scene&					getScene()					const	{ return mActor.getScene();	}

		PX_FORCE_INLINE PxU32					getElementID()				const	{ return mElementID;	}
		PX_FORCE_INLINE bool					isInBroadPhase()			const	{ return mInBroadPhase;	}
		
						void					addToAABBMgr(PxReal contactDistance, Bp::FilterGroup::Enum group, Ps::IntBool isTrigger);
						void					removeFromAABBMgr();

						void					setElementInteractionsDirty(InteractionDirtyFlag::Enum flag, PxU8 interactionFlag);

		PX_FORCE_INLINE	void					initID()
												{
													Scene& scene = getScene();
													mElementID = scene.getElementIDPool().createID();
													scene.getBoundsArray().initEntry(mElementID);
												}

		PX_FORCE_INLINE	void					releaseID()
												{
													getScene().getElementIDPool().releaseID(mElementID);
												}
	public:
						ElementSim*				mNextInActor;
	private:
						ActorSim&				mActor;

						PxU32					mElementID : 31;
						PxU32					mInBroadPhase : 1;
	};

	PX_FORCE_INLINE void setFilterObjectAttributeType(PxFilterObjectAttributes& attr, PxFilterObjectType::Enum type)
	{
		PX_ASSERT((attr & (PxFilterObjectType::eMAX_TYPE_COUNT-1)) == 0);
		attr |= type;
	}
} // namespace Sc
}

#endif
