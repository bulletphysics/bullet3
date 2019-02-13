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


#ifndef PX_PHYSICS_NX_RIGIDSTATIC
#define PX_PHYSICS_NX_RIGIDSTATIC
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "PxRigidActor.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief PxRigidStatic represents a static rigid body simulation object in the physics SDK.

PxRigidStatic objects are static rigid physics entities. They shall be used to define solid objects which are fixed in the world.

<h3>Creation</h3>
Instances of this class are created by calling #PxPhysics::createRigidStatic() and deleted with #release().

<h3>Visualizations</h3>
\li #PxVisualizationParameter::eACTOR_AXES

@see PxRigidActor  PxPhysics.createRigidStatic()  release()
*/

class PxRigidStatic : public PxRigidActor
{
public:
	virtual		const char*		getConcreteTypeName() const { return "PxRigidStatic"; }

protected:
	PX_INLINE					PxRigidStatic(PxType concreteType, PxBaseFlags baseFlags) : PxRigidActor(concreteType, baseFlags) {}
	PX_INLINE					PxRigidStatic(PxBaseFlags baseFlags) : PxRigidActor(baseFlags) {}
	virtual						~PxRigidStatic() {}
	virtual		bool			isKindOf(const char* name)	const { return !::strcmp("PxRigidStatic", name) || PxRigidActor::isKindOf(name); }

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
