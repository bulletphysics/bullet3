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


#ifndef PX_PHYSICS_PX_COLLECTION
#define PX_PHYSICS_PX_COLLECTION

#include "PxSerialFramework.h"

/** \addtogroup common
@{
*/

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxBase;

/**
\brief Collection class for serialization.

A collection is a set of PxBase objects. PxBase objects can be added to the collection
regardless of other objects they depend on. Objects may be named using PxSerialObjectId values in order 
to resolve dependencies between objects of different collections.

Serialization and deserialization only work through collections.

A scene is typically serialized using the following steps:

 -# create a serialization registry
 -# create a collection for scene objects
 -# complete the scene objects (adds all dependent objects, e.g. meshes)
 -# serialize collection
 -# release collection
 -# release serialization registry

For example the code may look like this:

\code
	PxPhysics* physics; // The physics
	PxScene* scene;		// The physics scene
	SerialStream s;		// The user-defined stream doing the actual write to disk
	
	PxSerializationRegistry* registry = PxSerialization::createSerializationRegistry(*physics);	// step 1)
	PxCollection* collection = PxSerialization::createCollection(*scene);						// step 2)
	PxSerialization::complete(*collection, *registry);											// step 3)
	PxSerialization::serializeCollectionToBinary(s, *collection, *registry);					// step 4)
	collection->release();																		// step 5)
	registry->release();																		// step 6)
\endcode

A scene is typically deserialized using the following steps:

 -# load a serialized collection into memory
 -# create a serialization registry
 -# create a collection by passing the serialized memory block
 -# add collected objects to scene
 -# release collection
 -# release serialization registry

For example the code may look like this:

\code
	PxPhysics* physics; // The physics
	PxScene* scene;		// The physics scene
	void* memory128;	// a 128-byte aligned buffer previously loaded from disk by the user	- step 1)
	
	PxSerializationRegistry* registry = PxSerialization::createSerializationRegistry(*physics);		// step 2)
	PxCollection* collection = PxSerialization::createCollectionFromBinary(memory128, *registry);	// step 3)
	scene->addCollection(*collection);																// step 4)
	collection->release();																			// step 5)
	registry->release();																			// step 6)
\endcode

@see PxBase, PxCreateCollection()
*/
class PxCollection
{
public:	

	/**
	\brief Adds a PxBase object to the collection.

	Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
	in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID 
	means the object remains without id. Objects can be added regardless of other objects they require. If the object
	is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
	operation fails.


	\param[in] object Object to be added to the collection
	\param[in] id Optional PxSerialObjectId id
	*/
	virtual void						add(PxBase& object, PxSerialObjectId id = PX_SERIAL_OBJECT_ID_INVALID) = 0;

	/**
	\brief Removes a PxBase member object from the collection.

	Object needs to be contained by the collection.

	\param[in] object PxBase object to be removed
	*/
	virtual	void						remove(PxBase& object) = 0;
			
	/**
	\brief Returns whether the collection contains a certain PxBase object.

	\param[in] object PxBase object
	\return Whether object is contained.
	*/
	virtual bool						contains(PxBase& object) const = 0;

	/**
	\brief Adds an id to a member PxBase object.

	If the object is already associated with an id within the collection, the id is replaced.
	May only be called for objects that are members of the collection. The id needs to be unique 
	within the collection.
	
	\param[in] object Member PxBase object
	\param[in] id PxSerialObjectId id to be given to the object
	*/
	virtual void						addId(PxBase& object, PxSerialObjectId id) = 0;

	/**
	\brief Removes id from a contained PxBase object.

	May only be called for ids that are associated with an object in the collection.
	
	\param[in] id PxSerialObjectId value
	*/
	virtual void						removeId(PxSerialObjectId id) = 0;
	
	/**
	\brief Adds all PxBase objects and their ids of collection to this collection.

	PxBase objects already in this collection are ignored. Object ids need to be conflict 
	free, i.e. the same object may not have two different ids within the two collections.
	
	\param[in] collection Collection to be added
	*/
	virtual void						add(PxCollection& collection) = 0;

	/**
	\brief Removes all PxBase objects of collection from this collection.

	PxBase objects not present in this collection are ignored. Ids of objects 
	which are removed are also removed.

	\param[in] collection Collection to be removed
	*/
	virtual void						remove(PxCollection& collection) = 0;

	/**
	\brief Gets number of PxBase objects in this collection.
	
	\return Number of objects in this collection
	*/
	virtual	PxU32						getNbObjects() const = 0;

	/**
	\brief Gets the PxBase object of this collection given its index.

	\param[in] index PxBase index in [0, getNbObjects())
	\return PxBase object at index index
	*/
	virtual PxBase&						getObject(PxU32 index) const = 0;

	/**
	\brief Copies member PxBase pointers to a user specified buffer.

	\param[out] userBuffer Array of PxBase pointers
	\param[in] bufferSize Capacity of userBuffer
	\param[in] startIndex Offset into list of member PxBase objects
	\return number of members PxBase objects that have been written to the userBuffer 
	*/
	virtual	PxU32						getObjects(PxBase** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Looks for a PxBase object given a PxSerialObjectId value.

	If there is no PxBase object in the collection with the given id, NULL is returned.

	\param[in] id PxSerialObjectId value to look for
	\return PxBase object with the given id value or NULL
	*/
	virtual PxBase*						find(PxSerialObjectId id) const = 0;
	
	/**
	\brief Gets number of PxSerialObjectId names in this collection.
	
	\return Number of PxSerialObjectId names in this collection
	*/
	virtual PxU32						getNbIds() const = 0;

	/**
	\brief Copies member PxSerialObjectId values to a user specified buffer.

	\param[out] userBuffer Array of PxSerialObjectId values
	\param[in] bufferSize Capacity of userBuffer
	\param[in] startIndex Offset into list of member PxSerialObjectId values
	\return number of members PxSerialObjectId values that have been written to the userBuffer 
	*/
	virtual	PxU32						getIds(PxSerialObjectId* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Gets the PxSerialObjectId name of a PxBase object within the collection.

	The PxBase object needs to be a member of the collection.

	\param[in] object PxBase object to get id for
	\return PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
	*/
	virtual PxSerialObjectId			getId(const PxBase& object) const = 0;	

	/**
	\brief Deletes a collection object.

	This function only deletes the collection object, i.e. the container class. It doesn't delete objects
	that are part of the collection.

	@see PxCreateCollection() 
	*/

	virtual void						release() = 0;

protected:
										PxCollection()	{}
	virtual								~PxCollection()	{}
};
    
#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Creates a collection object.

Objects can only be serialized or deserialized through a collection.
For serialization, users must add objects to the collection and serialize the collection as a whole.
For deserialization, the system gives back a collection of deserialized objects to users.

\return The new collection object.

@see PxCollection, PxCollection::release()
*/
PX_PHYSX_COMMON_API physx::PxCollection* PX_CALL_CONV PxCreateCollection();


/** @} */
#endif
