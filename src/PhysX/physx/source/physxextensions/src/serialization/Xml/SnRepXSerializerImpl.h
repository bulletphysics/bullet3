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
#ifndef PX_REPX_SERIALIZER_IMPL_H
#define PX_REPX_SERIALIZER_IMPL_H

#include "PsUserAllocated.h"
#include "SnXmlVisitorWriter.h"
#include "SnXmlVisitorReader.h"

namespace physx { 
	using namespace Sn;

	/**
	 *	The repx serializer impl takes the raw, untyped repx extension interface
	 *	and implements the simpler functions plus does the reinterpret-casts required 
	 *	for any object to implement the serializer safely.
	 */
	template<typename TLiveType>
	struct RepXSerializerImpl : public PxRepXSerializer, shdfnd::UserAllocated
	{
	protected:
		RepXSerializerImpl( const RepXSerializerImpl& inOther );
		RepXSerializerImpl& operator=( const RepXSerializerImpl& inOther );

	public:
		PxAllocatorCallback& mAllocator;

		RepXSerializerImpl( PxAllocatorCallback& inAllocator )
			: mAllocator( inAllocator )
		{
		}
				
		virtual const char* getTypeName() { return PxTypeInfo<TLiveType>::name(); }
		
		virtual void objectToFile( const PxRepXObject& inLiveObject, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs )
		{
			const TLiveType* theObj = reinterpret_cast<const TLiveType*>( inLiveObject.serializable );
			objectToFileImpl( theObj, inCollection, inWriter, inTempBuffer, inArgs );
		}

		virtual PxRepXObject fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
		{
			TLiveType* theObj( allocateObject( inArgs ) );
			if ( theObj )
				if(fileToObjectImpl( theObj, inReader, inAllocator, inArgs, inCollection ))
					return PxCreateRepXObject(theObj);
			return PxRepXObject();
		}
		
		virtual void objectToFileImpl( const TLiveType* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/)
		{
			writeAllProperties( inObj, inWriter, inTempBuffer, *inCollection );
		}

		virtual bool fileToObjectImpl( TLiveType* inObj, XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
		{
			return readAllProperties( inArgs, inReader, inObj, inAllocator, *inCollection );
		}

		virtual TLiveType* allocateObject( PxRepXInstantiationArgs& inArgs ) = 0;
	};
}

#endif
