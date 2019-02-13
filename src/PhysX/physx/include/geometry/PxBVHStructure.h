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


#ifndef PX_PHYSICS_BVH_STRUCTURE
#define PX_PHYSICS_BVH_STRUCTURE
/** \addtogroup geomutils
@{
*/

#include "common/PxBase.h"
#include "foundation/PxTransform.h"
#include "foundation/PxBounds3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Class representing the bounding volume hierarchy structure.

PxBVHStructure can be  provided to PxScene::addActor. In this case the scene query 
pruning structure inside PhysX SDK will store/update one bound per actor. 
The scene queries against such an actor will query actor bounds and then 
make a local space query against the provided BVH structure, which is in
actor's local space.

@see PxScene::addActor
*/
class PxBVHStructure: public PxBase
{
public:

	/**
	\brief Raycast test against a BVH structure.

	\param[in] origin		The origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] maxDist		Maximum ray length, has to be in the [0, inf) range
	\param[in] maxHits		Max number of returned hits = size of 'rayHits' buffer
	\param[out] rayHits		Raycast hits information, bounds indices 
	\return Number of hits  
	*/
	virtual PxU32					raycast(const PxVec3& origin,
										const PxVec3& unitDir,
										PxReal maxDist,
										PxU32 maxHits,
										PxU32* PX_RESTRICT rayHits) const = 0;

	/**
	\brief Sweep test against a BVH structure.

	\param[in] aabb			The axis aligned bounding box to sweep
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] maxDist		Maximum sweep length, has to be in the [0, inf) range
	\param[in] maxHits		Max number of returned hits = size of 'sweepHits' buffer
	\param[out] sweepHits	Sweep hits information, bounds indices 
	\return Number of hits 
	*/
	virtual PxU32					sweep(const PxBounds3& aabb, 
										const PxVec3& unitDir,
										PxReal maxDist,
										PxU32 maxHits,
										PxU32* PX_RESTRICT sweepHits) const = 0;

	/**
	\brief AABB overlap test against a BVH structure.

	\param[in] aabb			The axis aligned bounding box		
	\param[in] maxHits		Max number of returned hits = size of 'overlapHits' buffer
	\param[out] overlapHits	Overlap hits information, bounds indices 
	\return Number of hits 
	*/
	virtual PxU32					overlap(const PxBounds3& aabb, 
										PxU32 maxHits,
										PxU32* PX_RESTRICT overlapHits) const = 0;

	/**
	\brief Retrieve the bounds in the BVH.

	@see PxBounds3
	*/
	virtual const PxBounds3*		getBounds() const = 0;

	/**
	\brief Returns the number of bounds in the BVH.

	You can use #getBounds() to retrieve the bounds.

	\return Number of bounds in the BVH.

	*/
	virtual PxU32					getNbBounds() const = 0;
	
	virtual	const char*				getConcreteTypeName() const	{ return "PxBVHStructure";	}
protected:
	PX_INLINE						PxBVHStructure(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags)	{}
	PX_INLINE						PxBVHStructure(PxBaseFlags baseFlags) : PxBase(baseFlags)										{}
	virtual							~PxBVHStructure() {}

	virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxBVHStructure", name) || PxBase::isKindOf(name); }

};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
