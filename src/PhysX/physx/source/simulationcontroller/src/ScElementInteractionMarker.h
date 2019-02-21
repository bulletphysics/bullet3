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

#ifndef PX_COLLISION_ELEMENT_INTERACTION_MARKER
#define PX_COLLISION_ELEMENT_INTERACTION_MARKER

#include "ScElementSimInteraction.h"
#include "ScNPhaseCore.h"

namespace physx
{
namespace Sc
{
	class ElementInteractionMarker : public ElementSimInteraction
	{
	public:
		PX_INLINE		ElementInteractionMarker(ElementSim& element0, ElementSim& element1, bool createParallel/* = false*/);
						~ElementInteractionMarker();

				bool	onActivate_(void*)	{ return false;	}
				bool	onDeactivate_()		{ return true;	}
	};

} // namespace Sc


PX_INLINE Sc::ElementInteractionMarker::ElementInteractionMarker(ElementSim& element0, ElementSim& element1, bool createParallel) :
	ElementSimInteraction(element0, element1, InteractionType::eMARKER, InteractionFlag::eRB_ELEMENT|InteractionFlag::eFILTERABLE)
{
	if(!createParallel)
	{
		bool active = registerInActors();
		PX_UNUSED(active);
		PX_ASSERT(!active);
		getScene().registerInteraction(this, false);
		getScene().getNPhaseCore()->registerInteraction(this);
	}
}

}

#endif //PX_COLLISION_SHAPEINTERACTIONMARKER
