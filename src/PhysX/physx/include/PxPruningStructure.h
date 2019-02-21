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


#ifndef PX_PHYSICS_NX_PRUNING_STRUCTURE
#define PX_PHYSICS_NX_PRUNING_STRUCTURE
/** \addtogroup physics
@{ */

#include "PxPhysXConfig.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif


/**
\brief A precomputed pruning structure to accelerate scene queries against newly added actors.

The pruning structure can be provided to #PxScene:: addActors() in which case it will get merged
directly into the scene query optimization AABB tree, thus leading to improved performance when
doing queries against the newly added actors. This applies to both static and dynamic actors.

\note PxPruningStructure objects can be added to a collection and get serialized.
\note Adding a PxPruningStructure object to a collection will also add the actors that were used to build the pruning structure.

\note PxPruningStructure must be released before its rigid actors.
\note PxRigidBody objects can be in one PxPruningStructure only.
\note Changing the bounds of PxRigidBody objects assigned to a pruning structure that has not been added to a scene yet will 
invalidate the pruning structure. Same happens if shape scene query flags change or shape gets removed from an actor.

@see PxScene::addActors PxCollection
*/	
class PxPruningStructure : public PxBase
{
public:
	/**
	\brief Release this object.
	*/
	virtual void				release() = 0;

	/**
	\brief Retrieve rigid actors in the pruning structure.

	You can retrieve the number of rigid actor pointers by calling #getNbRigidActors()

	\param[out] userBuffer The buffer to store the actor pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first actor pointer to be retrieved
	\return Number of rigid actor pointers written to the buffer.

	@see PxRigidActor
	*/
	virtual PxU32				getRigidActors(PxRigidActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Returns the number of rigid actors in the pruning structure.

	You can use #getRigidActors() to retrieve the rigid actor pointers.

	\return Number of rigid actors in the pruning structure.

	@see PxRigidActor
	*/
	virtual PxU32				getNbRigidActors() const = 0;

	virtual	const char*			getConcreteTypeName() const	{ return "PxPruningStructure";	}
protected:
	PX_INLINE					PxPruningStructure(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
	PX_INLINE					PxPruningStructure(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	virtual						~PxPruningStructure()	{}
	virtual		bool			isKindOf(const char* name)	const		{ return !::strcmp("PxPruningStructure", name) || PxBase::isKindOf(name); }
};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */ 
#endif // PX_PHYSICS_NX_PRUNING_STRUCTURE
