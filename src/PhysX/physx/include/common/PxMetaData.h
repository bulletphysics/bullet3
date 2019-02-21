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


#ifndef PX_PHYSICS_METADATA_H
#define PX_PHYSICS_METADATA_H
/** \addtogroup physics
@{
*/

#include "foundation/Px.h"
#include "foundation/PxIO.h"
#include "PxMetaDataFlags.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Struct to store meta data definitions.

	Note: The individual fields have different meaning depending on the meta data entry configuration.
	*/
	struct PxMetaDataEntry
	{
		const char*		type;			//!< Field type (bool, byte, quaternion, etc)
		const char*		name;			//!< Field name (appears exactly as in the source file)
		PxU32			offset;			//!< Offset from the start of the class (ie from "this", field is located at "this"+Offset)
		PxU32			size;			//!< sizeof(Type)
		PxU32			count;			//!< Number of items of type Type (0 for dynamic sizes)
		PxU32			offsetSize;		//!< Offset of dynamic size param, for dynamic arrays
		PxU32			flags;			//!< Field parameters
		PxU32			alignment;		//!< Explicit alignment 
	};

	#define PX_STORE_METADATA(stream, metaData) stream.write(&metaData, sizeof(PxMetaDataEntry))

	#define PX_SIZE_OF(Class, Member)	sizeof((reinterpret_cast<Class*>(0))->Member)

	/**
	\brief specifies a binary metadata entry for a member variable of a class
	*/
	#define PX_DEF_BIN_METADATA_ITEM(stream, Class, type, name, flags) \
	{ \
		PxMetaDataEntry tmp = {	#type, #name, PxU32(PX_OFFSET_OF_RT(Class, name)), PX_SIZE_OF(Class, name), \
								1, 0, flags, 0}; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a member array variable of a class
	\details similar to PX_DEF_BIN_METADATA_ITEMS_AUTO but for cases with mismatch between specified type and array type
	*/
	#define PX_DEF_BIN_METADATA_ITEMS(stream, Class, type, name, flags, count) \
	{ \
		PxMetaDataEntry tmp = {	#type, #name, PxU32(PX_OFFSET_OF_RT(Class, name)), PX_SIZE_OF(Class, name), \
							count, 0, flags, 0}; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a member array variable of a class
	\details similar to PX_DEF_BIN_METADATA_ITEMS but automatically detects the array length, which only works when the specified 
	type matches the type of the array - does not support PxMetaDataFlag::ePTR
	*/
	#define PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Class, type, name, flags) \
	{ \
		PxMetaDataEntry tmp = {	#type, #name, PxU32(PX_OFFSET_OF_RT(Class, name)), PX_SIZE_OF(Class, name), \
							sizeof((reinterpret_cast<Class*>(0))->name)/sizeof(type), 0, flags, 0}; \
							PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a class
	*/
	#define PX_DEF_BIN_METADATA_CLASS(stream, Class) \
	{ \
		PxMetaDataEntry tmp = {	#Class, 0, 0, sizeof(Class), 0, 0, PxMetaDataFlag::eCLASS, 0 }; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a virtual class
	*/
	#define PX_DEF_BIN_METADATA_VCLASS(stream, Class) \
	{ \
		PxMetaDataEntry tmp = {	#Class, 0, 0, sizeof(Class), 0, 0, PxMetaDataFlag::eCLASS|PxMetaDataFlag::eVIRTUAL, 0}; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a typedef
	*/
	#define PX_DEF_BIN_METADATA_TYPEDEF(stream, newType, oldType) \
	{ \
		PxMetaDataEntry tmp = {	#newType, #oldType, 0, 0, 0, 0, PxMetaDataFlag::eTYPEDEF, 0 }; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for declaring a base class
	*/
	#define PX_DEF_BIN_METADATA_BASE_CLASS(stream, Class, BaseClass) \
	{ \
		Class* myClass = reinterpret_cast<Class*>(42);															\
		BaseClass* s = static_cast<BaseClass*>(myClass);														\
		const PxU32 offset = PxU32(size_t(s) - size_t(myClass));												\
		PxMetaDataEntry tmp = { #Class, #BaseClass, offset, sizeof(Class), 0, 0, PxMetaDataFlag::eCLASS, 0 };	\
		PX_STORE_METADATA(stream, tmp);																			\
	}

	/**
	\brief specifies a binary metadata entry for a union 
	*/
	#define PX_DEF_BIN_METADATA_UNION(stream, Class, name) \
	{ \
		PxMetaDataEntry tmp = {	#Class, 0, PxU32(PX_OFFSET_OF_RT(Class, name)), PX_SIZE_OF(Class, name), \
							1, 0, PxMetaDataFlag::eUNION, 0 }; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for a particular member type of a union
	*/
	#define PX_DEF_BIN_METADATA_UNION_TYPE(stream, Class, type, enumValue)	\
	{ \
		PxMetaDataEntry tmp = {	#Class, #type, enumValue, 0, 0, 0, PxMetaDataFlag::eUNION, 0 }; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for extra data
	*/
	#define PX_DEF_BIN_METADATA_EXTRA_ITEM(stream, Class, type, control, align)	\
	{ \
		PxMetaDataEntry tmp = {	#type, 0, PxU32(PX_OFFSET_OF_RT(Class, control)), sizeof(type), 0, PxU32(PX_SIZE_OF(Class, control)), \
							PxMetaDataFlag::eEXTRA_DATA|PxMetaDataFlag::eEXTRA_ITEM, align }; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for an array of extra data
	*/
	#define PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, Class, type, control, count, flags, align)	\
	{ \
		PxMetaDataEntry tmp = {	#type, 0, PxU32(PX_OFFSET_OF_RT(Class, control)), PxU32(PX_SIZE_OF(Class, control)), \
							PxU32(PX_OFFSET_OF_RT(Class, count)), PxU32(PX_SIZE_OF(Class, count)), \
							PxMetaDataFlag::eEXTRA_DATA|PxMetaDataFlag::eEXTRA_ITEMS|flags, align }; \
		PX_STORE_METADATA(stream, tmp); \
	}
	
	/**
	\brief specifies a binary metadata entry for an array of extra data
	additional to PX_DEF_BIN_METADATA_EXTRA_ITEMS a mask can be specified to interpret the control value
	@see PxMetaDataFlag::eCONTROL_MASK
	*/
	#define PX_DEF_BIN_METADATA_EXTRA_ITEMS_MASKED_CONTROL(stream, Class, type, control, controlMask ,count, flags, align) \
	{ \
		PxMetaDataEntry tmp = {	#type, 0, PxU32(PX_OFFSET_OF_RT(Class, control)), PxU32(PX_SIZE_OF(Class, control)), \
							PxU32(PX_OFFSET_OF_RT(Class, count)), PxU32(PX_SIZE_OF(Class, count)), \
							PxMetaDataFlag::eCONTROL_MASK|PxMetaDataFlag::eEXTRA_DATA|PxMetaDataFlag::eEXTRA_ITEMS|flags|(controlMask & PxMetaDataFlag::eCONTROL_MASK_RANGE) << 16, \
							align}; \
		PX_STORE_METADATA(stream, tmp); \
	}

	/**
	\brief specifies a binary metadata entry for an array of extra data
	\details similar to PX_DEF_BIN_METADATA_EXTRA_ITEMS, but supporting no control - PxMetaDataFlag::ePTR is also not supported
	*/
	#define PX_DEF_BIN_METADATA_EXTRA_ARRAY(stream, Class, type, dyn_count, align, flags) \
	{ \
		PxMetaDataEntry tmp = {	#type, 0, PxU32(PX_OFFSET_OF_RT(Class, dyn_count)), PX_SIZE_OF(Class, dyn_count), align, 0, \
							PxMetaDataFlag::eEXTRA_DATA|flags, align }; \
		PX_STORE_METADATA(stream, tmp); \
	}

    /**
	\brief specifies a binary metadata entry for an string of extra data
	*/
    #define PX_DEF_BIN_METADATA_EXTRA_NAME(stream, Class, control, align) \
	{ \
		PxMetaDataEntry tmp = {	"char", "string", 0, 0, 0, 0, PxMetaDataFlag::eEXTRA_DATA|PxMetaDataFlag::eEXTRA_NAME, align }; \
		PX_STORE_METADATA(stream, tmp); \
	}
	
	/**
	\brief specifies a binary metadata entry declaring an extra data alignment for a class
	*/
	#define PX_DEF_BIN_METADATA_EXTRA_ALIGN(stream, Class, align)	\
	{ \
		PxMetaDataEntry tmp = {	"PxU8", "Alignment", 0, 0, 0, 0, PxMetaDataFlag::eEXTRA_DATA|PxMetaDataFlag::eALIGNMENT, align}; \
		PX_STORE_METADATA(stream, tmp); \
	}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
