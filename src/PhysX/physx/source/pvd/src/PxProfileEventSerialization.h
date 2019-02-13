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


#ifndef PXPVDSDK_PXPROFILEEVENTSERIALIZATION_H
#define PXPVDSDK_PXPROFILEEVENTSERIALIZATION_H

#include "PxProfileDataParsing.h"
#include "PxProfileEvents.h"

namespace physx { namespace profile {

	/**
	 *	Array type must be a pxu8 container.  Templated so that this object can write
	 *	to different collections.
	 */
	
	template<typename TArrayType>
	struct EventSerializer
	{
		TArrayType* mArray;
		EventSerializer( TArrayType* inA ) : mArray( inA ) {}

		template<typename TDataType>
		uint32_t streamify( const char*, const TDataType& inType )
		{
			return mArray->write( inType );
		}

		uint32_t streamify( const char*, const char*& inType )
		{
			PX_ASSERT( inType != NULL );
			uint32_t len( static_cast<uint32_t>( strlen( inType ) ) );
			++len; //include the null terminator
			uint32_t writtenSize = 0;
			writtenSize = mArray->write(len);
			writtenSize += mArray->write(inType, len);
			return writtenSize;
		}
		
		uint32_t streamify( const char*, const uint8_t* inData, uint32_t len )
		{
			uint32_t writtenSize = mArray->write(len);
			if ( len )
				writtenSize += mArray->write(inData, len);
			return writtenSize;
		}

		uint32_t streamify( const char* nm, const uint64_t& inType, EventStreamCompressionFlags::Enum inFlags )
		{
			uint32_t writtenSize = 0;
			switch( inFlags )
			{
			case EventStreamCompressionFlags::U8:
					writtenSize = streamify(nm, static_cast<uint8_t>(inType));
					break;
			case EventStreamCompressionFlags::U16:
					writtenSize = streamify(nm, static_cast<uint16_t>(inType));
					break;
			case EventStreamCompressionFlags::U32:
					writtenSize = streamify(nm, static_cast<uint32_t>(inType));
					break;
			case EventStreamCompressionFlags::U64:
				writtenSize = streamify(nm, inType);
				break;
			}
			return writtenSize;
		}
		
		uint32_t streamify( const char* nm, const uint32_t& inType, EventStreamCompressionFlags::Enum inFlags )
		{
			uint32_t writtenSize = 0;
			switch( inFlags )
			{
			case EventStreamCompressionFlags::U8:
					writtenSize = streamify(nm, static_cast<uint8_t>(inType));
					break;
			case EventStreamCompressionFlags::U16:
					writtenSize = streamify(nm, static_cast<uint16_t>(inType));
					break;
			case EventStreamCompressionFlags::U32:
			case EventStreamCompressionFlags::U64:
				writtenSize = streamify(nm, inType);
				break;
			}
			return writtenSize;
		}
	};

	/**
	 *	The event deserializes takes a buffer implements the streamify functions
	 *	by setting the passed in data to the data in the buffer.
	 */	
	template<bool TSwapBytes>
	struct EventDeserializer
	{
		const uint8_t* mData;
		uint32_t		mLength;
		bool		mFail;

		EventDeserializer( const uint8_t* inData,  uint32_t inLength )
			: mData( inData )
			, mLength( inLength )
			, mFail( false )
		{
			if ( mData == NULL )
				mLength = 0;
		}

		bool val() { return TSwapBytes; }

		uint32_t streamify( const char* , uint8_t& inType )
		{
			uint8_t* theData = reinterpret_cast<uint8_t*>( &inType ); //type punned pointer...
			if ( mFail || sizeof( inType ) > mLength )
			{
				PX_ASSERT( false );
				mFail = true;
			}
			else
			{
				for( uint32_t idx = 0; idx < sizeof( uint8_t ); ++idx, ++mData, --mLength )
					theData[idx] = *mData;
			}
			return 0;
		}

		//default streamify reads things natively as bytes.
		template<typename TDataType>
		uint32_t streamify( const char* , TDataType& inType )
		{
			uint8_t* theData = reinterpret_cast<uint8_t*>( &inType ); //type punned pointer...
			if ( mFail || sizeof( inType ) > mLength )
			{
				PX_ASSERT( false );
				mFail = true;
			}
			else
			{
				for( uint32_t idx = 0; idx < sizeof( TDataType ); ++idx, ++mData, --mLength )
					theData[idx] = *mData;
				bool temp = val();
				if ( temp ) 
					BlockParseFunctions::swapBytes<sizeof(TDataType)>( theData );
			}
			return 0;
		}

		uint32_t streamify( const char*, const char*& inType )
		{
			uint32_t theLen;
			streamify( "", theLen );
			theLen = PxMin( theLen, mLength );
			inType = reinterpret_cast<const char*>( mData );
			mData += theLen;
			mLength -= theLen;
			return 0;
		}
		
		uint32_t streamify( const char*, const uint8_t*& inData, uint32_t& len )
		{
			uint32_t theLen;
			streamify( "", theLen );
			theLen = PxMin( theLen, mLength );
			len = theLen;
			inData = reinterpret_cast<const uint8_t*>( mData );
			mData += theLen;
			mLength -= theLen;
			return 0;
		}

		uint32_t streamify( const char* nm, uint64_t& inType, EventStreamCompressionFlags::Enum inFlags )
		{
			switch( inFlags )
			{
			case EventStreamCompressionFlags::U8:
				{
					uint8_t val=0;
					streamify( nm, val );
					inType = val;
				}
					break;
			case EventStreamCompressionFlags::U16:
				{
					uint16_t val;
					streamify( nm, val );
					inType = val;
				}
					break;
			case EventStreamCompressionFlags::U32:
				{
					uint32_t val;
					streamify( nm, val );
					inType = val;
				}
					break;
			case EventStreamCompressionFlags::U64:
				streamify( nm, inType );
				break;
			}
			return 0;
		}
		
		uint32_t streamify( const char* nm, uint32_t& inType, EventStreamCompressionFlags::Enum inFlags )
		{
			switch( inFlags )
			{
			case EventStreamCompressionFlags::U8:
				{
					uint8_t val=0;
					streamify( nm, val );
					inType = val;
				}
					break;
			case EventStreamCompressionFlags::U16:
				{
					uint16_t val=0;
					streamify( nm, val );
					inType = val;
				}
					break;
			case EventStreamCompressionFlags::U32:
			case EventStreamCompressionFlags::U64:
				streamify( nm, inType );
				break;
			}
			return 0;
		}
	};
}}
#endif // PXPVDSDK_PXPROFILEEVENTSERIALIZATION_H
