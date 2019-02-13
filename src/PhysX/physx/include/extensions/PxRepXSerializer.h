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
#ifndef PX_REPX_SERIALIZER_H
#define PX_REPX_SERIALIZER_H
/** \addtogroup Serializers
  @{
*/

#include "common/PxBase.h"
#include "extensions/PxRepXSimpleType.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	
	class XmlMemoryAllocator;
	class XmlWriter;
	class XmlReader;
	class MemoryBuffer;

	/**
	\brief Serializer interface for RepX (Xml) serialization.

	In order to serialize a class to RepX both a PxSerializer and
	a PxRepXSerializer implementation are needed. 

	A repx Serializer provides the ability to capture a live
	object to a descriptor or static state and the ability to
	write that state out to a file.  Objects allocated
	by the Serializer using the allocator are freed when the
	collection itself is freed.
	SnRepXCoreSerializers.cpp implements a set of Serializers
	for the core PhysX types.

	\note Implementing a PxRepXSerializer is currently not practical without including the internal PhysXExtension header "SnRepXSerializerImpl.h". 

	@see PxSerializer, PX_NEW_REPX_SERIALIZER, PxSerializationRegistry::registerRepXSerializer
	*/
	class PxRepXSerializer
	{
	protected:
		virtual ~PxRepXSerializer(){}
	public:
		
		/**
		\brief The type this Serializer is meant to operate on.
		@see PxRepXObject::typeName
		*/
		virtual const char* getTypeName() = 0;

		/**
		\brief Convert from a RepX object to a key-value pair hierarchy
		
		\param[in] inLiveObject The object to convert to the passed in descriptor.
		\param[in] inCollection The collection to use to find ids of references of this object.
		\param[in] inWriter Interface to write data to.
		\param[in] inTempBuffer used to for temporary allocations.
		\param[in] inArgs The arguments used in create resources and objects.
		*/
		virtual void objectToFile( const PxRepXObject& inLiveObject, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs ) = 0;

		/**
		\brief Convert from a descriptor to a live object.  Must be an object of this Serializer type.
		
		\param[in] inReader The inverse of the writer, a key-value pair database.
		\param[in] inAllocator An allocator to use for temporary allocations.  These will be freed after instantiation completes.
		\param[in] inArgs The arguments used in create resources and objects.
		\param[in] inCollection The collection used to find references.
		
		\return The new live object.  It can be an invalid object if the instantiation cannot take place.
		*/
		virtual PxRepXObject fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection ) = 0;

	};
	
#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Inline helper template function to create PxRepXObject from TDataType type supporting PxTypeInfo<TDataType>::name.
*/
template<typename TDataType>
PX_INLINE physx::PxRepXObject PxCreateRepXObject(const TDataType* inType, const physx::PxSerialObjectId inId)
{
	return physx::PxRepXObject(physx::PxTypeInfo<TDataType>::name(), inType, inId);
}

/**
\brief Inline helper function to create PxRepXObject from a PxBase instance.
*/
PX_INLINE physx::PxRepXObject PxCreateRepXObject(const physx::PxBase* inType, const physx::PxSerialObjectId inId)
{
	PX_ASSERT(inType);
	return physx::PxRepXObject(inType->getConcreteTypeName(), inType, inId);
}

/**
\brief Inline helper template function to create PxRepXObject form TDataType type using inType pointer as a PxSerialObjectId id.
*/
template<typename TDataType>
PX_INLINE physx::PxRepXObject PxCreateRepXObject(const TDataType* inType)
{
	return PxCreateRepXObject(inType, static_cast<physx::PxSerialObjectId>(reinterpret_cast<size_t>(inType)));
}

/**
\brief Preprocessor macro for RepX serializer creation.
*/
#define PX_NEW_REPX_SERIALIZER(T) \
		*PX_PLACEMENT_NEW(PxGetFoundation().getAllocatorCallback().allocate(sizeof(T), "PxRepXSerializer",  __FILE__, __LINE__ ), T)(PxGetFoundation().getAllocatorCallback())

/**
\brief Preprocessor Macro to simplify RepX serializer delete.
*/
#define PX_DELETE_REPX_SERIALIZER(x) \
		{ PxRepXSerializer* s = x; if (s) { PxGetFoundation().getAllocatorCallback().deallocate(s); } }


/** @} */
#endif // PX_REPX_SERIALIZER_H
