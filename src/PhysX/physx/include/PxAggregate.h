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


#ifndef PX_PHYSICS_NX_AGGREGATE
#define PX_PHYSICS_NX_AGGREGATE

/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

class PxActor;
class PxBVHStructure;

/**
\brief Class to aggregate actors into a single broad-phase entry.

A PxAggregate object is a collection of PxActors, which will exist as a single entry in the
broad-phase structures. This has 3 main benefits:

1) it reduces "broad phase pollution" by allowing a collection of spatially coherent broad-phase 
entries to be replaced by a single aggregated entry (e.g. a ragdoll or a single actor with a 
large number of attached shapes).

2) it reduces broad-phase memory usage

3) filtering can be optimized a lot if self-collisions within an aggregate are not needed. For
   example if you don't need collisions between ragdoll bones, it's faster to simply disable
   filtering once and for all, for the aggregate containing the ragdoll, rather than filtering
   out each bone-bone collision in the filter shader.

@see PxActor, PxPhysics.createAggregate
*/

class PxAggregate : public PxBase
{
public:

	/**
	\brief Deletes the aggregate object.

	Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
	belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
	to delete both the PxAggregate and its actors, it is best to release the actors first, then release
	the PxAggregate when it is empty.
	*/
	virtual	void		release()				= 0;

	/**
	\brief Adds an actor to the aggregate object.

	A warning is output if the total number of actors is reached, or if the incoming actor already belongs
	to an aggregate.

	If the aggregate belongs to a scene, adding an actor to the aggregate also adds the actor to that scene.

	If the actor already belongs to a scene, a warning is output and the call is ignored. You need to remove
	the actor from the scene first, before adding it to the aggregate.

	\note When BVHStructure is provided the actor shapes are grouped together. 
	The scene query pruning structure inside PhysX SDK will store/update one
	bound per actor. The scene queries against such an actor will query actor
	bounds and then make a local space query against the provided BVH structure, which is in
	actor's local space.

	\param	[in] actor The actor that should be added to the aggregate
	\param	[in] bvhStructure BVHStructure for actor shapes.
	return	true if success
	*/
	virtual	bool		addActor(PxActor& actor, const PxBVHStructure* bvhStructure = NULL)		= 0;

	/**
	\brief Removes an actor from the aggregate object.

	A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
	removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
	scene. If you intend to delete the actor, it is best to call #PxActor::release() directly. That way
	the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.

	\param	[in] actor The actor that should be removed from the aggregate
	return	true if success
	*/
	virtual	bool		removeActor(PxActor& actor)		= 0;

	/**
	\brief Adds an articulation to the aggregate object.

	A warning is output if the total number of actors is reached (every articulation link counts as an actor), 
	or if the incoming articulation already belongs	to an aggregate.

	If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.

	If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
	the articulation from the scene first, before adding it to the aggregate.

	\param	[in] articulation The articulation that should be added to the aggregate
	return	true if success
	*/
	virtual	bool		addArticulation(PxArticulationBase& articulation) = 0;

	/**
	\brief Removes an articulation from the aggregate object.

	A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
	removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
	scene. If you intend to delete the articulation, it is best to call #PxArticulation::release() directly. That way
	the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.

	\param	[in] articulation The articulation that should be removed from the aggregate
	return	true if success
	*/
	virtual	bool		removeArticulation(PxArticulationBase& articulation) = 0;

	/**
	\brief Returns the number of actors contained in the aggregate.

	You can use #getActors() to retrieve the actor pointers.

	\return Number of actors contained in the aggregate.

	@see PxActor getActors()
	*/
	virtual PxU32		getNbActors() const = 0;

	/**
	\brief Retrieves max amount of actors that can be contained in the aggregate.

	\return Max aggregate size. 

	@see PxPhysics::createAggregate()
	*/
	virtual	PxU32		getMaxNbActors() const = 0;

	/**
	\brief Retrieve all actors contained in the aggregate.

	You can retrieve the number of actor pointers by calling #getNbActors()

	\param[out] userBuffer The buffer to store the actor pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first actor pointer to be retrieved
	\return Number of actor pointers written to the buffer.

	@see PxShape getNbShapes()
	*/
	virtual PxU32		getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Retrieves the scene which this aggregate belongs to.

	\return Owner Scene. NULL if not part of a scene.

	@see PxScene
	*/
	virtual	PxScene*	getScene()	= 0;

	/**
	\brief Retrieves aggregate's self-collision flag.

	\return self-collision flag
	*/
	virtual	bool		getSelfCollision()	const	= 0;

	virtual	const char*	getConcreteTypeName() const	{ return "PxAggregate"; }

protected:
	PX_INLINE			PxAggregate(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
	PX_INLINE			PxAggregate(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	virtual				~PxAggregate() {}
	virtual	bool		isKindOf(const char* name) const { return !::strcmp("PxAggregate", name) || PxBase::isKindOf(name); }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
