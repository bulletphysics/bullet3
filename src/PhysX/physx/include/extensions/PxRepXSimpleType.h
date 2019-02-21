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



#ifndef PX_REPX_SIMPLE_TYPE_H
#define PX_REPX_SIMPLE_TYPE_H

/** \addtogroup extensions
  @{
*/

#include "foundation/PxSimpleTypes.h"
#include "cooking/PxCooking.h"
#include "common/PxStringTable.h"
#include "common/PxSerialFramework.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	
	/**
	\brief Helper class containing the mapping of id to object, and type name.
	*/
	struct PxRepXObject
	{
		/**
		\brief Identifies the extension meant to handle this object.
		@see PxTypeInfo, PX_DEFINE_TYPEINFO, PxRepXSerializer
		*/
		const char*			typeName;

		/**
		\brief Pointer to the serializable this was created from
		*/
		const void*			serializable;

		/**
		\brief Id given to this object at some point
		*/
		PxSerialObjectId 	id;
		PxRepXObject( const char* inTypeName = "", const void* inSerializable = NULL, const PxSerialObjectId inId = 0 )
			: typeName( inTypeName )
			, serializable( inSerializable )
			, id( inId )
		{
		}
		bool isValid() const { return serializable != NULL; }
	};

	/**
	\brief Arguments required to instantiate a serializable object from RepX.

	Extra arguments can be added to the object map under special ids.

	@see PxRepXSerializer::objectToFile, PxRepXSerializer::fileToObject
	*/
	struct PxRepXInstantiationArgs
	{
		PxPhysics&			physics;
		PxCooking*			cooker;
		PxStringTable*		stringTable;
		PxRepXInstantiationArgs( PxPhysics& inPhysics, PxCooking* inCooking = NULL , PxStringTable* inStringTable = NULL ) 
			: physics( inPhysics )
			, cooker( inCooking )
			, stringTable( inStringTable )
		{
		}

		PxRepXInstantiationArgs& operator=(const PxRepXInstantiationArgs&);
	};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
