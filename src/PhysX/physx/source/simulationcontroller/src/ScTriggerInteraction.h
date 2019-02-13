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


#ifndef PX_COLLISION_TRIGGERINTERACTION
#define PX_COLLISION_TRIGGERINTERACTION

#include "ScElementSimInteraction.h"
#include "ScShapeSim.h"
#include "GuOverlapTests.h"

namespace physx
{
namespace Sc
{
	class TriggerInteraction : public ElementSimInteraction
	{
	public:
		enum TriggerFlag
		{
			PAIR_FLAGS_MASK					= ((PxPairFlag::eNOTIFY_TOUCH_LOST << 1) - 1),	// Bits where the PxPairFlags eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST get stored
			NEXT_FREE						= ((PAIR_FLAGS_MASK << 1) & ~PAIR_FLAGS_MASK),

			PROCESS_THIS_FRAME				= (NEXT_FREE << 0), // the trigger pair is new or the pose of an actor was set -> initial processing required.
																// This is important to cover cases where a static or kinematic
																// (non-moving) trigger is created and overlaps with a sleeping
																// object. Or for the case where a static/kinematic is teleported to a new
																// location. TOUCH_FOUND should still get sent in that case.
			LAST							= (NEXT_FREE << 1)
		};

											TriggerInteraction(ShapeSim& triggerShape, ShapeSim& otherShape);
											~TriggerInteraction();

		PX_FORCE_INLINE	Gu::TriggerCache&	getTriggerCache()									{ return mTriggerCache;							}
		PX_FORCE_INLINE	ShapeSim&			getTriggerShape()							const	{ return static_cast<ShapeSim&>(getElement0());	}
		PX_FORCE_INLINE	ShapeSim&			getOtherShape()								const	{ return static_cast<ShapeSim&>(getElement1());	}

		PX_FORCE_INLINE bool				lastFrameHadContacts()						const	{ return mLastFrameHadContacts;			}
		PX_FORCE_INLINE void				updateLastFrameHadContacts(bool hasContact)			{ mLastFrameHadContacts = hasContact;	}

		PX_FORCE_INLINE PxPairFlags			getTriggerFlags()							const	{ return PxPairFlags(PxU32(mFlags) & PAIR_FLAGS_MASK);		}
		PX_FORCE_INLINE void				setTriggerFlags(PxPairFlags triggerFlags);

		PX_FORCE_INLINE void				raiseFlag(TriggerFlag flag)				{ mFlags |= flag; }
		PX_FORCE_INLINE void				clearFlag(TriggerFlag flag)				{ mFlags &= ~flag; }
		PX_FORCE_INLINE	Ps::IntBool			readFlag(TriggerFlag flag)		const	{ return Ps::IntBool(mFlags & flag); }

		PX_FORCE_INLINE void				forceProcessingThisFrame(Sc::Scene& scene);

						bool				onActivate_(void*);
						bool				onDeactivate_();

	protected:
						Gu::TriggerCache	mTriggerCache;
						PxU16				mFlags;
						bool				mLastFrameHadContacts;
	};

} // namespace Sc


PX_FORCE_INLINE void Sc::TriggerInteraction::setTriggerFlags(PxPairFlags triggerFlags)
{
	PX_ASSERT(PxU32(triggerFlags) < (PxPairFlag::eDETECT_CCD_CONTACT << 1));  // to find out if a new PxPairFlag has been added in which case PAIR_FLAGS_MASK needs to get adjusted

#if PX_CHECKED
	if (triggerFlags & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
	{
		PX_WARN_ONCE("Trigger pairs do not support PxPairFlag::eNOTIFY_TOUCH_PERSISTS events any longer.");
	}
#endif

	PxU32 newFlags = mFlags;
	PxU32 fl = PxU32(triggerFlags) & PxU32(PxPairFlag::eNOTIFY_TOUCH_FOUND|PxPairFlag::eNOTIFY_TOUCH_LOST);
	newFlags &= (~PAIR_FLAGS_MASK);  // clear old flags
	newFlags |= fl;

	mFlags = PxU16(newFlags);
}


PX_FORCE_INLINE void Sc::TriggerInteraction::forceProcessingThisFrame(Sc::Scene& scene)
{
	raiseFlag(PROCESS_THIS_FRAME);

	if (!readInteractionFlag(InteractionFlag::eIS_ACTIVE))
	{
		raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
		scene.notifyInteractionActivated(this);
	}
}

}

#endif
