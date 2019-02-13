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
#ifndef SN_JOINT_REPX_SERIALIZER_H
#define SN_JOINT_REPX_SERIALIZER_H
/** \addtogroup RepXSerializers
  @{
*/

#include "extensions/PxRepXSimpleType.h"
#include "SnRepXSerializerImpl.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	
	class XmlReader;
	class XmlMemoryAllocator;
	class XmlWriter;
	class MemoryBuffer;
	
	template<typename TJointType>
	struct PxJointRepXSerializer : public RepXSerializerImpl<TJointType>
	{
		PxJointRepXSerializer(PxAllocatorCallback& inAllocator) : RepXSerializerImpl<TJointType>(inAllocator) {}
		virtual PxRepXObject fileToObject(XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection);
		virtual void objectToFileImpl(const TJointType* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs&);
		virtual TJointType* allocateObject(PxRepXInstantiationArgs&) { return NULL; }
	};

#if PX_SUPPORT_EXTERN_TEMPLATE
	// explicit template instantiations declarations
	extern template struct PxJointRepXSerializer<PxD6Joint>;
	extern template struct PxJointRepXSerializer<PxDistanceJoint>;
	extern template struct PxJointRepXSerializer<PxContactJoint>;
	extern template struct PxJointRepXSerializer<PxFixedJoint>;
	extern template struct PxJointRepXSerializer<PxPrismaticJoint>;
	extern template struct PxJointRepXSerializer<PxRevoluteJoint>;
	extern template struct PxJointRepXSerializer<PxSphericalJoint>;
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
