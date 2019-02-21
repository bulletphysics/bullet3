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


#ifndef PX_COLLECTION_EXT_H
#define PX_COLLECTION_EXT_H
/** \addtogroup extensions
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxCollection.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxCollectionExt
	{
	public:
		/**
		\brief Removes and releases all object from a collection.
		
		The Collection itself is not released.

		If the releaseExclusiveShapes flag is not set to true, release() will not be called on exclusive shapes.

		It is assumed that the application holds a reference to each of the objects in the collection, with the exception of objects that are not releasable
		(PxBase::isReleasable()). In general, objects that violate this assumption need to be removed from the collection prior to calling releaseObjects.
		
		\note when a shape is created with PxRigidActor::createShape() or PxRigidActorExt::createExclusiveShape(), the only counted reference is held by the actor. 
		If such a shape and its actor are present in the collection, the reference count will be decremented once when the actor is released, and once when the 
		shape is released, resulting in undefined behavior. Shape reference counts can be incremented with PxShape::acquireReference().

		\param[in] collection to remove and release all object from.
		\param[in] releaseExclusiveShapes if this parameter is set to false, release() will not be called on exclusive shapes.
		*/
		static void	releaseObjects(PxCollection& collection, bool releaseExclusiveShapes = true);

		/**
		\brief Removes objects of a given type from a collection, potentially adding them to another collection.

		\param[in,out] collection Collection from which objects are removed
		\param[in] concreteType PxConcreteType of sdk objects that should be removed	
		\param[in,out] to Optional collection to which the removed objects are added

		@see PxCollection, PxConcreteType
		*/	
		static void remove(PxCollection& collection, PxType concreteType, PxCollection* to = NULL);


		/**
		\brief Collects all objects in PxPhysics that are shareable across multiple scenes.

		This function creates a new collection from all objects that are shareable across multiple 
		scenes. Instances of the following types are included: PxConvexMesh, PxTriangleMesh, 
		PxHeightField, PxShape and PxMaterial.

		This is a helper function to ease the creation of collections for serialization. 

		\param[in] physics The physics SDK instance from which objects are collected. See #PxPhysics
		\return Collection to which objects are added. See #PxCollection

		@see PxCollection, PxPhysics
		*/
		static  PxCollection*	createCollection(PxPhysics& physics);
	
		/**
		\brief Collects all objects from a PxScene.

		This function creates a new collection from all objects that where added to the specified 
		PxScene. Instances of the following types are included: PxActor, PxAggregate, 
		PxArticulation and PxJoint (other PxConstraint types are not included).
	
		This is a helper function to ease the creation of collections for serialization. 
		The function PxSerialization.complete() can be used to complete the collection with required objects prior to 
		serialization.

		\param[in] scene The PxScene instance from which objects are collected. See #PxScene
		\return Collection to which objects are added. See #PxCollection

		@see PxCollection, PxScene, PxSerialization.complete()
		*/
		static	PxCollection*	createCollection(PxScene& scene);
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
