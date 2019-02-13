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


#ifndef PX_PHYSICS_COMMON_NX_SERIAL_FRAMEWORK
#define PX_PHYSICS_COMMON_NX_SERIAL_FRAMEWORK

/** \addtogroup common
@{
*/

#include "common/PxPhysXCommonConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

typedef PxU16 PxType;
class PxBase;
class PxSerializationContext;
class PxRepXSerializer;
class PxSerializer;
class PxPhysics;

//! Default serialization alignment
#define PX_SERIAL_ALIGN	16

//! Serialized input data must be aligned to this value
#define PX_SERIAL_FILE_ALIGN 128

//! PxSerialObjectId value for objects that do not have an ID
#define PX_SERIAL_OBJECT_ID_INVALID 0

//! ID type for PxBase objects in a PxCollection
typedef PxU64 PxSerialObjectId;

//! Bit to mark pointer type references, @see PxDeserializationContext
#define PX_SERIAL_REF_KIND_PTR_TYPE_BIT (1u<<31)

//! Reference kind value for PxBase objects
#define PX_SERIAL_REF_KIND_PXBASE		(0 | PX_SERIAL_REF_KIND_PTR_TYPE_BIT)

//! Reference kind value for material indices
#define PX_SERIAL_REF_KIND_MATERIAL_IDX (1)

//! Used to fix multi-byte characters warning from gcc for situations like: PxU32 foo = 'CCTS';
#define PX_MAKE_FOURCC(a, b, c, d) ( (a) | ((b)<<8) | ((c)<<16) | ((d)<<24) )

/**
\brief Callback class used to process PxBase objects.

@see PxSerializer::requires
*/
class PxProcessPxBaseCallback
{
public:
	virtual ~PxProcessPxBaseCallback()  {}
	virtual void process(PxBase&) = 0;	
};


/**
\brief Binary serialization context class.

This class is used to register reference values and write object
and object extra data during serialization.
It is mainly used by the serialization framework. Except for custom 
serializable types, users should not have to worry about it.

@see PxDeserializationContext 
*/
class PxSerializationContext
{
public:

    /**
    \brief Registers a reference value corresponding to a PxBase object.

	This method is assumed to be called in the implementation of PxSerializer::registerReferences for serialized
	references that need to be resolved on deserialization.

	A reference needs to be associated with exactly one PxBase object in either the collection or the 
	external references collection.

	Different kinds of references are supported and need to be specified. In the most common case
	(PX_SERIAL_REF_KIND_PXBASE) the PxBase object matches the reference value (which is the pointer
	to the PxBase object). Integer references maybe registered as well (used for internal material 
	indices with PX_SERIAL_REF_KIND_MATERIAL_IDX). Other kinds could be added with the restriction that
	for pointer types the kind value needs to be marked with the PX_SERIAL_REF_KIND_PTR_TYPE_BIT.

	\param[in]  base		PxBase object associated with the reference
    \param[in]  kind		What kind of reference this is (PX_SERIAL_REF_KIND_PXBASE, PX_SERIAL_REF_KIND_MATERIAL_IDX or custom kind)
    \param[in]  reference	Value of reference

	@see PxDeserializationContext::resolveReference, PX_SERIAL_REF_KIND_PXBASE, PX_SERIAL_REF_KIND_MATERIAL_IDX, PxSerializer::registerReferences
    */
	virtual	void				registerReference(PxBase& base, PxU32 kind, size_t reference)		= 0;

	/**
	\brief Returns the collection that is being serialized.
	*/
	virtual	const PxCollection&	getCollection()	const												= 0;

	/**
	\brief Serializes object data and object extra data.
	
	This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.

	@see PxSerializer::exportData, PxSerializer::exportExtraData, PxSerializer::createObject, PxDeserializationContext::readExtraData
	*/
	virtual	void				writeData(const void* data, PxU32 size)								= 0;

	/**
	\brief Aligns the serialized data.
	
	This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.

	@see PxSerializer::exportData, PxSerializer::exportExtraData, PxDeserializationContext::alignExtraData
	*/
	virtual	void				alignData(PxU32 alignment = PX_SERIAL_ALIGN)						= 0;

	/**
	\brief Helper function to write a name to the extraData if serialization is configured to save names.

	This function is assumed to be called within the implementation of PxSerializer::exportExtraData.

	@see PxSerialization::serializeCollectionToBinary, PxDeserializationContext::readName
	*/
	virtual	void				writeName(const char* name)											= 0;

protected:

							PxSerializationContext() {}
	virtual					~PxSerializationContext() {}
};


/**
\brief Binary deserialization context class.

This class is used to resolve references and access extra data during deserialization.
It is mainly used by the serialization framework. Except for custom 
serializable types, users should not have to worry about it.

@see PxSerializationContext 
*/
class PxDeserializationContext
{
public:

 	/**
    \brief Retrieves a pointer to a deserialized PxBase object given a corresponding deserialized reference value

	This method is assumed to be called in the implementation of PxSerializer::createObject in order
	to update reference values on deserialization.
	
	To update a PxBase reference the corresponding deserialized pointer value needs to be provided in order to retrieve 
	the location of the corresponding deserialized PxBase object. (PxDeserializationContext::translatePxBase simplifies 
	this common case).

	For other kinds of references the reverence values need to be updated by deduction given the corresponding PxBase instance. 

    \param[in] kind			What kind of reference this is (PX_SERIAL_REF_KIND_PXBASE, PX_SERIAL_REF_KIND_MATERIAL_IDX or custom kind)
    \param[in] reference	Deserialized reference value
    \return					PxBase object associated with the reference value

	@see PxSerializationContext::registerReference, PX_SERIAL_REF_KIND_PXBASE, PX_SERIAL_REF_KIND_MATERIAL_IDX, translatePxBase
    */
	virtual		PxBase*			resolveReference(PxU32 kind, size_t reference) const = 0;

	/**
	\brief Helper function to update PxBase pointer on deserialization

	@see resolveReference, PX_SERIAL_REF_KIND_PXBASE
	*/
	template<typename T>
				void			translatePxBase(T*& base) { if (base) { base = static_cast<T*>(resolveReference(PX_SERIAL_REF_KIND_PXBASE, size_t(base))); } }

	/**
	\brief Helper function to read a name from the extra data during deserialization.

	This function is assumed to be called within the implementation of PxSerializer::createObject.

	@see PxSerializationContext::writeName
	*/
	PX_INLINE	void			readName(const char*& name)
	{
		PxU32 len = *reinterpret_cast<PxU32*>(mExtraDataAddress);
		mExtraDataAddress += sizeof(len);
		name = len ? reinterpret_cast<const char*>(mExtraDataAddress) : NULL;
		mExtraDataAddress += len; 
	}

	/**
	\brief Function to read extra data during deserialization.
	
	This function is assumed to be called within the implementation of PxSerializer::createObject.

	@see PxSerializationContext::writeData, PxSerializer::createObject
	*/
	template<typename T>
	PX_INLINE	T*				readExtraData(PxU32 count=1)
	{
		T* data = reinterpret_cast<T*>(mExtraDataAddress);
		mExtraDataAddress += sizeof(T)*count;
		return data;
	}

	/**
	\brief Function to read extra data during deserialization optionally aligning the extra data stream before reading.
	
	This function is assumed to be called within the implementation of PxSerializer::createObject.

	@see PxSerializationContext::writeData, PxDeserializationContext::alignExtraData, PxSerializer::createObject
	*/
	template<typename T, PxU32 alignment>
	PX_INLINE	T*				readExtraData(PxU32 count=1)
	{
		alignExtraData(alignment);
		return readExtraData<T>(count);
	}

	/**
	\brief Function to align the extra data stream to a power of 2 alignment

	This function is assumed to be called within the implementation of PxSerializer::createObject.

	@see PxSerializationContext::alignData, PxSerializer::createObject
	*/
	PX_INLINE	void			alignExtraData(PxU32 alignment = PX_SERIAL_ALIGN)
	{
		size_t addr = reinterpret_cast<size_t>(mExtraDataAddress);
		addr = (addr+alignment-1)&~size_t(alignment-1);
		mExtraDataAddress = reinterpret_cast<PxU8*>(addr);
	}


	/**
	\brief Function to return the PX_PHYSX_VERSION value with which the data was originally serialized
	*/

	virtual		PxU32			getPhysXVersion() const = 0;

protected:

								PxDeserializationContext() {}
	virtual						~PxDeserializationContext() {}

	PxU8*						mExtraDataAddress;	
};

/**
\brief Callback type for exporting binary meta data for a serializable type.
@see PxSerializationRegistry::registerBinaryMetaDataCallback

\param stream	Stream to store binary meta data. 
*/
typedef void (*PxBinaryMetaDataCallback)(PxOutputStream& stream);

/**
\brief Class serving as a registry for XML (RepX) and binary serializable types.

In order to serialize and deserialize objects the application needs
to maintain an instance of this class. It can be created with
PxSerialization::createSerializationRegistry() and released with
PxSerializationRegistry::release().

@see PxSerialization::createSerializationRegistry
*/
class PxSerializationRegistry
{
public:
	/************************************************************************************************/

	/** @name Binary Serialization Functionality
	*/
	//@{

	/**
	\brief Register a serializer for a concrete type 

	\param	type PxConcreteType corresponding to the serializer
	\param	serializer The PxSerializer to be registered

	@see PxConcreteType, PxSerializer, PxSerializationRegistry::unregisterSerializer
	*/
	virtual void						registerSerializer(PxType type, PxSerializer& serializer) = 0;

	/**
	\brief Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.

	\param	type PxConcreteType for which the serializer should be unregistered
	\return	Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.

	@see PxConcreteType, PxSerializationRegistry::registerSerializer, PxSerializationRegistry::release
	*/
	virtual PxSerializer*               unregisterSerializer(PxType type) = 0;

	/**
	\brief Register binary meta data callback

	The callback is executed when calling PxSerialization::dumpBinaryMetaData.

	\param	callback PxBinaryMetaDataCallback to be registered.

	@see PxBinaryMetaDataCallback, PxSerialization::dumpBinaryMetaData
	*/
	virtual void						registerBinaryMetaDataCallback(PxBinaryMetaDataCallback callback) = 0;
	
	/**
	\brief Returns PxSerializer corresponding to type

	\param	type PxConcreteType of the serializer requested.
	\return	Registered PxSerializer object corresponding to type

	@see PxConcreteType
	*/
	virtual const PxSerializer*			getSerializer(PxType type) const = 0;  

	//@}
	/************************************************************************************************/

	/** @name RepX (XML) Serialization Functionality
	*/
	//@{

	/**
	\brief Register a RepX serializer for a concrete type

	\param	type PxConcreteType corresponding to the RepX serializer
	\param	serializer The PxRepXSerializer to be registered
	
	@see PxConcreteType, PxRepXSerializer
	*/
	virtual void						registerRepXSerializer(PxType type, PxRepXSerializer& serializer) = 0;

	/**
	\brief Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.

	\param	type PxConcreteType for which the RepX serializer should be unregistered
	\return	Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
	
	@see PxConcreteType, PxSerializationRegistry::registerRepXSerializer, PxSerializationRegistry::release
	*/
	virtual PxRepXSerializer*			unregisterRepXSerializer(PxType type) = 0;

	/**
	\brief Returns RepX serializer given the corresponding type name

	\param	typeName Name of the type
	\return	Registered PxRepXSerializer object corresponding to type name

	@see PxRepXSerializer, PxTypeInfo, PX_DEFINE_TYPEINFO
	*/
	virtual PxRepXSerializer*			getRepXSerializer(const char* typeName) const = 0;  
	
	//@}
	/************************************************************************************************/

	/**
	\brief Releases PxSerializationRegistry instance.

	This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
	serializers before releasing the PxSerializationRegistry.

	@see PxSerializationRegistry::unregisterSerializer, PxSerializationRegistry::unregisterRepXSerializer
	*/
	virtual void release() = 0;

protected:
	virtual ~PxSerializationRegistry(){}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
