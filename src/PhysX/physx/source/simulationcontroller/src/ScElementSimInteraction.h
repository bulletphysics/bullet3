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


#ifndef PX_PHYSICS_SCP_ELEMENT_SIM_INTERACTION
#define PX_PHYSICS_SCP_ELEMENT_SIM_INTERACTION

#include "ScInteraction.h"
#include "ScElementSim.h"

namespace physx
{
namespace Sc
{
	class ElementSimInteraction : public Interaction
	{
	public:
		PX_FORCE_INLINE	ElementSim&	getElement0()						const	{ return mElement0;						}
		PX_FORCE_INLINE	ElementSim&	getElement1()						const	{ return mElement1;						}

		PX_FORCE_INLINE	void		setFilterPairIndex(PxU32 filterPairIndex)	{ mFilterPairIndex = filterPairIndex;	}
		PX_FORCE_INLINE	PxU32		getFilterPairIndex()				const	{ return mFilterPairIndex;				}

	protected:
		PX_INLINE					ElementSimInteraction(ElementSim& element0, ElementSim& element1, InteractionType::Enum type, PxU8 flags);
		virtual						~ElementSimInteraction() {}

		ElementSimInteraction& operator=(const ElementSimInteraction&);

	private:
						ElementSim&	mElement0;
						ElementSim&	mElement1;
						PxU32		mFilterPairIndex;
	};

} // namespace Sc

//////////////////////////////////////////////////////////////////////////

PX_INLINE Sc::ElementSimInteraction::ElementSimInteraction(ElementSim& element0, ElementSim& element1, InteractionType::Enum type, PxU8 flags) :
	Interaction	(element0.getActor(), element1.getActor(), type, flags),
	mElement0	(element0),
	mElement1	(element1),
	mFilterPairIndex(INVALID_FILTER_PAIR_INDEX)
{
}


}

#endif
