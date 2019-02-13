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

#ifndef PX_VEHICLE_SERIALIZATION_H
#define PX_VEHICLE_SERIALIZATION_H

#include "extensions/PxRepXSimpleType.h"
#include "SnRepXSerializerImpl.h"

namespace physx
{
	class PxRepXSerializer;
	class PxSerializationRegistry;
	class XmlReader;
	class XmlMemoryAllocator;
	class XmlWriter;
	class MemoryBuffer;

	PX_DEFINE_TYPEINFO(PxVehicleNoDrive,		PxVehicleConcreteType::eVehicleNoDrive)
	PX_DEFINE_TYPEINFO(PxVehicleDrive4W,		PxVehicleConcreteType::eVehicleDrive4W)
	PX_DEFINE_TYPEINFO(PxVehicleDriveNW,		PxVehicleConcreteType::eVehicleDriveNW)
	PX_DEFINE_TYPEINFO(PxVehicleDriveTank,		PxVehicleConcreteType::eVehicleDriveTank)
	
	template<typename TVehicleType>
	struct PxVehicleRepXSerializer : public RepXSerializerImpl<TVehicleType>
	{
		PxVehicleRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<TVehicleType>( inCallback ) {}
		virtual PxRepXObject fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection );
		virtual void objectToFileImpl( const TVehicleType* , PxCollection* , XmlWriter& , MemoryBuffer& , PxRepXInstantiationArgs& );
		virtual TVehicleType* allocateObject( PxRepXInstantiationArgs& ) { return NULL; }
	};

#if PX_SUPPORT_EXTERN_TEMPLATE
	// explicit template instantiation declarations
	extern template struct PxVehicleRepXSerializer<PxVehicleDrive4W>;
	extern template struct PxVehicleRepXSerializer<PxVehicleDriveTank>;
	extern template struct PxVehicleRepXSerializer<PxVehicleDriveNW>;
	extern template struct PxVehicleRepXSerializer<PxVehicleNoDrive>;
#endif

}


#endif//PX_VEHICLE_REPX_SERIALIZER_H
