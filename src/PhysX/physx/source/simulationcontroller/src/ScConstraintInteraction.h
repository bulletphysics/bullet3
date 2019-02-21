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

#ifndef PX_PHYSICS_SCP_CONSTRAINTSHADERINTERACTION
#define PX_PHYSICS_SCP_CONSTRAINTSHADERINTERACTION

#include "ScInteraction.h"

namespace physx
{
namespace Sc
{
	class ConstraintSim;
	class RigidSim;

	class ConstraintInteraction : public Interaction
	{
	public:
										ConstraintInteraction(ConstraintSim* shader, RigidSim& r0, RigidSim& r1);
										~ConstraintInteraction();

						bool			onActivate_(void* data);
						bool			onDeactivate_();

						void			updateState();
						void			destroy();  // disables the interaction and unregisters from the system. Does NOT delete the object. This is used on destruction but also when a constraint breaks.

		PX_FORCE_INLINE	ConstraintSim*	getConstraint()			{ return mConstraint;	}
		PX_FORCE_INLINE	PxU32			getEdgeIndex()	const	{ return mEdgeIndex;	}

	private:
						ConstraintSim*	mConstraint;
						PxU32			mEdgeIndex;
	};

} // namespace Sc

}

#endif
