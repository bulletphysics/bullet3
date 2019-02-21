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


#ifndef PXPVDSDK_PXPROFILEDATAPARSING_H
#define PXPVDSDK_PXPROFILEDATAPARSING_H

#include "foundation/Px.h"

namespace physx { namespace profile {

	//Converts datatypes without using type punning.
	struct BlockParserDataConverter
	{
		union
		{
			uint8_t	mU8[8];
			uint16_t	mU16[4];
			uint32_t	mU32[2];
			uint64_t	mU64[1];
			
			int8_t	mI8[8];
			int16_t	mI16[4];
			int32_t	mI32[2];
			int64_t	mI64[1];


			float	mF32[2];
			double	mF64[1];
		};

		template<typename TDataType> inline TDataType convert() { PX_ASSERT( false ); return TDataType(); }

		template<typename TDataType>
		inline void convert( const TDataType& ) {}
	};
	
	template<> inline uint8_t BlockParserDataConverter::convert<uint8_t>() { return mU8[0]; }
	template<> inline uint16_t BlockParserDataConverter::convert<uint16_t>() { return mU16[0]; }
	template<> inline uint32_t BlockParserDataConverter::convert<uint32_t>() { return mU32[0]; }
	template<> inline uint64_t BlockParserDataConverter::convert<uint64_t>() { return mU64[0]; }
	template<> inline int8_t BlockParserDataConverter::convert<int8_t>() { return mI8[0]; }
	template<> inline int16_t BlockParserDataConverter::convert<int16_t>() { return mI16[0]; }
	template<> inline int32_t BlockParserDataConverter::convert<int32_t>() { return mI32[0]; }
	template<> inline int64_t BlockParserDataConverter::convert<int64_t>() { return mI64[0]; }
	template<> inline float BlockParserDataConverter::convert<float>() { return mF32[0]; }
	template<> inline double BlockParserDataConverter::convert<double>() { return mF64[0]; }
	
	template<> inline void BlockParserDataConverter::convert<uint8_t>( const uint8_t& inData ) { mU8[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<uint16_t>( const uint16_t& inData ) { mU16[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<uint32_t>( const uint32_t& inData ) { mU32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<uint64_t>( const uint64_t& inData ) { mU64[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<int8_t>( const int8_t& inData ) { mI8[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<int16_t>( const int16_t& inData ) { mI16[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<int32_t>( const int32_t& inData ) { mI32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<int64_t>( const int64_t& inData ) { mI64[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<float>( const float& inData ) { mF32[0] = inData; }
	template<> inline void BlockParserDataConverter::convert<double>( const double& inData ) { mF64[0] = inData; }


	//Handles various details around parsing blocks of uint8_t data.
	struct BlockParseFunctions
	{
		template<uint8_t ByteCount>
		static inline void swapBytes( uint8_t* inData )
		{
			for ( uint32_t idx = 0; idx < ByteCount/2; ++idx )
			{
				uint32_t endIdx = ByteCount-idx-1;
				uint8_t theTemp = inData[idx];
				inData[idx] = inData[endIdx];
				inData[endIdx] = theTemp;
			}
		}

		static inline bool checkLength( const uint8_t* inStart, const uint8_t* inStop, uint32_t inLength )
		{
			return static_cast<uint32_t>(inStop - inStart) >= inLength;
		}
		//warning work-around
		template<typename T>
		static inline T val(T v) {return v;}

		template<bool DoSwapBytes, typename TDataType>
		static inline bool parse( const uint8_t*& inStart, const uint8_t* inStop, TDataType& outData )
		{
			if ( checkLength( inStart, inStop, sizeof( TDataType ) ) )
			{
				BlockParserDataConverter theConverter;
				for ( uint32_t idx =0; idx < sizeof( TDataType ); ++idx )
					theConverter.mU8[idx] = inStart[idx];
				if ( val(DoSwapBytes))
					swapBytes<sizeof(TDataType)>( theConverter.mU8 );
				outData = theConverter.convert<TDataType>();
				inStart += sizeof( TDataType );
				return true;
			}
			return false;
		}

		template<bool DoSwapBytes, typename TDataType>
		static inline bool parseBlock( const uint8_t*& inStart, const uint8_t* inStop, TDataType* outData, uint32_t inNumItems )
		{
			uint32_t desired = sizeof(TDataType)*inNumItems;
			if ( checkLength( inStart, inStop, desired ) )
			{
				if ( val(DoSwapBytes) )
				{
					for ( uint32_t item = 0; item < inNumItems; ++item )
					{
						BlockParserDataConverter theConverter;
						for ( uint32_t idx =0; idx < sizeof( TDataType ); ++idx )
							theConverter.mU8[idx] = inStart[idx];
						swapBytes<sizeof(TDataType)>( theConverter.mU8 );
						outData[item] = theConverter.convert<TDataType>();
						inStart += sizeof(TDataType);
					}
				}
				else
				{
					uint8_t* target = reinterpret_cast<uint8_t*>(outData);
					memmove( target, inStart, desired );
					inStart += desired;
				}
				return true;
			}
			return false;
		}
		
		//In-place byte swapping block
		template<bool DoSwapBytes, typename TDataType>
		static inline bool parseBlock( uint8_t*& inStart, const uint8_t* inStop, uint32_t inNumItems )
		{
			uint32_t desired = sizeof(TDataType)*inNumItems;
			if ( checkLength( inStart, inStop, desired ) )
			{
				if ( val(DoSwapBytes) )
				{
					for ( uint32_t item = 0; item < inNumItems; ++item, inStart += sizeof( TDataType ) )
						swapBytes<sizeof(TDataType)>( inStart ); //In-place swap.
				}
				else
					inStart += sizeof( TDataType ) * inNumItems;
				return true;
			}
			return false;
		}
	};

	//Wraps the begin/end keeping track of them.
	template<bool DoSwapBytes>
	struct BlockParser
	{
		const uint8_t* mBegin;
		const uint8_t* mEnd;
		BlockParser( const uint8_t* inBegin=NULL, const uint8_t* inEnd=NULL )
			: mBegin( inBegin )
			, mEnd( inEnd )
		{
		}
		inline bool hasMoreData() const { return mBegin != mEnd; }
		inline bool checkLength( uint32_t inLength ) { return BlockParseFunctions::checkLength( mBegin, mEnd, inLength ); }
		
		template<typename TDataType>
		inline bool read( TDataType& outDatatype ) { return BlockParseFunctions::parse<DoSwapBytes>( mBegin, mEnd, outDatatype ); }

		template<typename TDataType>
		inline bool readBlock( TDataType* outDataPtr, uint32_t inNumItems ) { return BlockParseFunctions::parseBlock<DoSwapBytes>( mBegin, mEnd, outDataPtr, inNumItems ); }

		template<typename TDataType>
		inline bool readBlock( uint32_t inNumItems ) 
		{ 
			uint8_t* theTempPtr = const_cast<uint8_t*>(mBegin);
			bool retval = BlockParseFunctions::parseBlock<DoSwapBytes, TDataType>( theTempPtr, mEnd, inNumItems ); 
			mBegin = theTempPtr;
			return retval;
		}

		uint32_t amountLeft() const { return static_cast<uint32_t>( mEnd - mBegin ); }
	};

	//Reads the data without checking for error conditions
	template<typename TDataType, typename TBlockParserType>
	inline TDataType blockParserRead( TBlockParserType& inType )
	{
		TDataType retval;
		inType.read( retval );
		return retval;
	}
}}

#endif // PXPVDSDK_PXPROFILEDATAPARSING_H
