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


#ifndef PX_PHYSICS_PX_PHYSICS_SERIALIZATION
#define PX_PHYSICS_PX_PHYSICS_SERIALIZATION

#include "PxPhysXConfig.h"
#include "common/PxSerialFramework.h"

#if !PX_DOXYGEN
/**
\brief Retrieves the PhysX SDK metadata.
This function is used to implement PxSerialization.dumpBinaryMetaData() and is not intended to be needed otherwise.
@see PxSerialization.dumpBinaryMetaData()
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxGetPhysicsBinaryMetaData(physx::PxOutputStream& stream);

/**
\brief Registers physics classes for serialization.
This function is used to implement PxSerialization.createSerializationRegistry() and is not intended to be needed otherwise.
@see PxSerializationRegistry
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterPhysicsSerializers(physx::PxSerializationRegistry& sr);

/**
\brief Unregisters physics classes for serialization.
This function is used in the release implementation of PxSerializationRegistry and in not intended to be used otherwise.
@see PxSerializationRegistry
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxUnregisterPhysicsSerializers(physx::PxSerializationRegistry& sr);


/**
\brief Adds collected objects to PxPhysics.

This function adds all objects contained in the input collection to the PxPhysics instance. This is used after deserializing
the collection, to populate the physics with inplace deserialized objects. This function is used in the implementation of 
PxSerialization.createCollectionFromBinary and is not intended to be needed otherwise.
\param[in] collection Objects to add to the PxPhysics instance.

@see PxCollection, PxSerialization.createCollectionFromBinary
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxAddCollectionToPhysics(const physx::PxCollection& collection);

#endif // !PX_DOXYGEN

#endif // PX_PHYSICS_PX_PHYSICS_SERIALIZATION
