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
#ifndef PX_XML_DESERIALIZER_H
#define PX_XML_DESERIALIZER_H

#include "SnXmlVisitorReader.h"

namespace physx { namespace Sn {
	
	//Definitions needed internally in the Serializer headers.
	template<typename TTriIndexElem>
	struct Triangle
	{
		TTriIndexElem mIdx0;
		TTriIndexElem mIdx1;
		TTriIndexElem mIdx2;
		Triangle( TTriIndexElem inIdx0 = 0, TTriIndexElem inIdx1 = 0, TTriIndexElem inIdx2 = 0)
			: mIdx0( inIdx0 )
			, mIdx1( inIdx1 )
			, mIdx2( inIdx2 )
		{
		}
	};

	struct XmlMemoryAllocateMemoryPoolAllocator
	{
		XmlMemoryAllocator* mAllocator;
		XmlMemoryAllocateMemoryPoolAllocator( XmlMemoryAllocator* inAlloc ) : mAllocator( inAlloc ) {}

		PxU8* allocate( PxU32 inSize ) { return mAllocator->allocate( inSize ); }
		void deallocate( PxU8* inMem ) { mAllocator->deallocate( inMem ); }
	};

	inline bool isEmpty(const char *s)
	{
		while (*s != '\0')
		{
			if (!isspace(*s))
				return false;
			s++;
		}
		return true;
	}

	inline void strtoLong( Triangle<PxU32>& ioDatatype,const char*& ioData )
	{
		strto( ioDatatype.mIdx0, ioData );
		strto( ioDatatype.mIdx1, ioData );
		strto( ioDatatype.mIdx2, ioData );
	}

	inline void strtoLong( PxHeightFieldSample& ioDatatype,const char*& ioData )
	{
		PxU32 tempData;
		strto( tempData, ioData );
		if ( isBigEndian() )
		{
			PxU32& theItem(tempData);
			PxU32 theDest = 0;
			PxU8* theReadPtr( reinterpret_cast< PxU8* >( &theItem ) );
			PxU8* theWritePtr( reinterpret_cast< PxU8* >( &theDest ) );
			//A height field sample is a 16 bit number
			//followed by two bytes.

			//We write this out as a 32 bit integer, LE.
			//Thus, on a big endian, we need to move the bytes
			//around a bit.
			//LE - 1 2 3 4
			//BE - 4 3 2 1 - after convert from xml number
			//Correct BE - 2 1 3 4, just like LE but with the 16 number swapped
			theWritePtr[0] = theReadPtr[2];
			theWritePtr[1] = theReadPtr[3];
			theWritePtr[2] = theReadPtr[1];
			theWritePtr[3] = theReadPtr[0];
			theItem = theDest;
		}
		ioDatatype = *reinterpret_cast<PxHeightFieldSample*>( &tempData );
	}

	template<typename TDataType>
	inline void readStridedFlagsProperty( XmlReader& ioReader, const char* inPropName, TDataType*& outData, PxU32& outStride, PxU32& outCount, XmlMemoryAllocator& inAllocator,
		  const PxU32ToName* inConversions)
	{
		const char* theSrcData;
		outStride = sizeof( TDataType );
		outData = NULL;
		outCount = 0;
		if ( ioReader.read( inPropName, theSrcData ) )
		{
			XmlMemoryAllocateMemoryPoolAllocator tempAllocator( &inAllocator );
			MemoryBufferBase<XmlMemoryAllocateMemoryPoolAllocator> tempBuffer( &tempAllocator );

			if ( theSrcData )
			{
				static PxU32 theCount = 0;
				++theCount;
				char* theStartData = const_cast< char*>( copyStr( &tempAllocator, theSrcData ) );
				char* aData = strtok(theStartData, " \n");
				while( aData )
				{
					TDataType tempValue;
					stringToFlagsType( aData, inAllocator, tempValue, inConversions );						
					aData = strtok(NULL," \n");
					tempBuffer.write( &tempValue, sizeof(TDataType) );
				}
				outData = reinterpret_cast< TDataType* >( tempBuffer.mBuffer );
				outCount = tempBuffer.mWriteOffset / sizeof( TDataType );
				tempAllocator.deallocate( reinterpret_cast<PxU8*>(theStartData) );
			}
			tempBuffer.releaseBuffer();
		}
	}

	template<typename TDataType>
	inline void readStridedBufferProperty( XmlReader& ioReader, const char* inPropName, TDataType*& outData, PxU32& outStride, PxU32& outCount, XmlMemoryAllocator& inAllocator)
	{
		const char* theSrcData;
		outStride = sizeof( TDataType );
		outData = NULL;
		outCount = 0;
		if ( ioReader.read( inPropName, theSrcData ) )
		{
			XmlMemoryAllocateMemoryPoolAllocator tempAllocator( &inAllocator );
			MemoryBufferBase<XmlMemoryAllocateMemoryPoolAllocator> tempBuffer( &tempAllocator );

			if ( theSrcData )
			{
				static PxU32 theCount = 0;
				++theCount;
				char* theStartData = const_cast< char*>( copyStr( &tempAllocator, theSrcData ) );
				const char* theData = theStartData;
				while( !isEmpty(theData) )
				{
					//These buffers are whitespace delimited.
					TDataType theType;
					strtoLong( theType, theData );
					tempBuffer.write( &theType, sizeof(theType) );
				}
				outData = reinterpret_cast< TDataType* >( tempBuffer.mBuffer );
				outCount = tempBuffer.mWriteOffset / sizeof( TDataType );
				tempAllocator.deallocate( reinterpret_cast<PxU8*>(theStartData) );
			}
			tempBuffer.releaseBuffer();
		}
	}
	
	template<typename TDataType>
	inline void readStridedBufferProperty( XmlReader& ioReader, const char* inPropName, PxStridedData& ioData, PxU32& outCount, XmlMemoryAllocator& inAllocator)
	{
		TDataType* tempData = NULL;
		readStridedBufferProperty<TDataType>( ioReader, inPropName, tempData, ioData.stride, outCount, inAllocator ); 
		ioData.data = tempData;
	}
	
	template<typename TDataType>
	inline void readStridedBufferProperty( XmlReader& ioReader, const char* inPropName, PxTypedStridedData<TDataType>& ioData, PxU32& outCount, XmlMemoryAllocator& inAllocator)
	{
		TDataType* tempData = NULL;
		readStridedBufferProperty<TDataType>( ioReader, inPropName, tempData, ioData.stride, outCount, inAllocator );
		ioData.data = reinterpret_cast<PxMaterialTableIndex*>( tempData );
	}

	template<typename TDataType>
	inline void readStridedBufferProperty( XmlReader& ioReader, const char* inPropName, PxBoundedData& ioData, XmlMemoryAllocator& inAllocator)
	{
		return readStridedBufferProperty<TDataType>( ioReader, inPropName, ioData, ioData.count, inAllocator );
	}

} }

#endif
