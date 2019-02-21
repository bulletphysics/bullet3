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


#ifndef PXPVDSDK_PXPROFILEMEMORYEVENTS_H
#define PXPVDSDK_PXPROFILEMEMORYEVENTS_H

#include "PxProfileEvents.h"

//Memory events define their own event stream

namespace physx { namespace profile {
	struct MemoryEventTypes
	{
		enum Enum
		{
			Unknown = 0,
			StringTableEvent, //introduce a new mapping of const char* -> integer
			AllocationEvent,
			DeallocationEvent,
			FullAllocationEvent
		};
	};

	template<unsigned numBits, typename TDataType>
	inline unsigned char convertToNBits( TDataType inType )
	{
		uint8_t conversion = static_cast<uint8_t>( inType );
		PX_ASSERT( conversion < (1 << numBits) );
		return conversion;
	}

	template<typename TDataType>
	inline unsigned char convertToTwoBits( TDataType inType )
	{
		return convertToNBits<2>( inType );
	}

	template<typename TDataType>
	inline unsigned char convertToFourBits( TDataType inType )
	{
		return convertToNBits<4>( inType );
	}

	inline EventStreamCompressionFlags::Enum fromNumber( uint8_t inNum ) { return static_cast<EventStreamCompressionFlags::Enum>( inNum ); } 
	
	template<unsigned lhs, unsigned rhs>
	inline void compileCheckSize()
	{
		PX_COMPILE_TIME_ASSERT( lhs <= rhs );
	}

	//Used for predictable bit fields.
	template<typename TDataType
			, uint8_t TNumBits
			, uint8_t TOffset
			, typename TInputType>
	struct BitMaskSetter
	{
		//Create a mask that masks out the orginal value shift into place
		static TDataType createOffsetMask() { return TDataType(createMask() << TOffset); }
		//Create a mask of TNumBits number of tis
		static TDataType createMask() { return static_cast<TDataType>((1 << TNumBits) - 1); }
		void setValue( TDataType& inCurrent, TInputType inData )
		{
			PX_ASSERT( inData < ( 1 << TNumBits ) );
			
			//Create a mask to remove the current value.
			TDataType theMask = TDataType(~(createOffsetMask()));
			//Clear out current value.
			inCurrent = TDataType(inCurrent & theMask);
			//Create the new value.
			TDataType theAddition = static_cast<TDataType>( inData << TOffset );
			//or it into the existing value.
			inCurrent = TDataType(inCurrent | theAddition);
		}

		TInputType getValue( TDataType inCurrent )
		{
			return static_cast<TInputType>( ( inCurrent >> TOffset ) & createMask() );
		}
	};


	struct MemoryEventHeader
	{
		uint16_t mValue;

		typedef BitMaskSetter<uint16_t, 4, 0, uint8_t> TTypeBitmask;
		typedef BitMaskSetter<uint16_t, 2, 4, uint8_t> TAddrCompressBitmask;
		typedef BitMaskSetter<uint16_t, 2, 6, uint8_t> TTypeCompressBitmask;
		typedef BitMaskSetter<uint16_t, 2, 8, uint8_t> TFnameCompressBitmask;
		typedef BitMaskSetter<uint16_t, 2, 10, uint8_t> TSizeCompressBitmask;
		typedef BitMaskSetter<uint16_t, 2, 12, uint8_t> TLineCompressBitmask;

		//That leaves size as the only thing not compressed usually.

		MemoryEventHeader( MemoryEventTypes::Enum inType = MemoryEventTypes::Unknown ) 
			: mValue( 0 )
		{
			uint8_t defaultCompression( convertToTwoBits( EventStreamCompressionFlags::U64 ) );
			TTypeBitmask().setValue( mValue, convertToFourBits( inType ) );
			TAddrCompressBitmask().setValue( mValue, defaultCompression );
			TTypeCompressBitmask().setValue( mValue, defaultCompression );
			TFnameCompressBitmask().setValue( mValue, defaultCompression );
			TSizeCompressBitmask().setValue( mValue, defaultCompression );
			TLineCompressBitmask().setValue( mValue, defaultCompression );
		}

		MemoryEventTypes::Enum getType() const { return static_cast<MemoryEventTypes::Enum>( TTypeBitmask().getValue( mValue ) ); }

#define DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( name )																			\
	void set##name( EventStreamCompressionFlags::Enum inEnum ) { T##name##Bitmask().setValue( mValue, convertToTwoBits( inEnum ) ); }	\
		EventStreamCompressionFlags::Enum get##name() const { return fromNumber( T##name##Bitmask().getValue( mValue ) ); }

		DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( AddrCompress )
		DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( TypeCompress )
		DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( FnameCompress )
		DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( SizeCompress )
		DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR( LineCompress )

#undef DEFINE_MEMORY_HEADER_COMPRESSION_ACCESSOR

		bool operator==( const MemoryEventHeader& inOther ) const 
		{ 
			return mValue == inOther.mValue; 
		}
		template<typename TStreamType>
		void streamify( TStreamType& inStream ) 
		{ 
			inStream.streamify( "Header", mValue );
		}
	};
	
	//Declaration of type level getMemoryEventType function that maps enumeration event types to datatypes
	template<typename TDataType>
	inline MemoryEventTypes::Enum getMemoryEventType() { PX_ASSERT( false ); return MemoryEventTypes::Unknown; }

	inline bool safeStrEq( const char* lhs, const char* rhs )
	{
		if ( lhs == rhs )
			return true;
		//If they aren't equal, and one of them is null,
		//then they can't be equal.
		//This is assuming that the null char* is not equal to
		//the empty "" char*.
		if ( !lhs || !rhs )
			return false;

		return ::strcmp( lhs, rhs ) == 0;
	}

	struct StringTableEvent
	{
		const char* mString;
		uint32_t		mHandle;

		void init( const char* inStr = "", uint32_t inHdl = 0 )
		{
			mString = inStr;
			mHandle = inHdl;
		}

		void init( const StringTableEvent& inData )
		{
			mString = inData.mString;
			mHandle = inData.mHandle;
		}

		bool operator==( const StringTableEvent& inOther ) const
		{
			return mHandle == inOther.mHandle
				&& safeStrEq( mString, inOther.mString );
		}
		
		void setup( MemoryEventHeader& ) const {}

		template<typename TStreamType>
		void streamify( TStreamType& inStream, const MemoryEventHeader& )
		{
			inStream.streamify( "String", mString );
			inStream.streamify( "Handle", mHandle );
		}
	};
	template<> inline MemoryEventTypes::Enum getMemoryEventType<StringTableEvent>() { return MemoryEventTypes::StringTableEvent; }

	struct MemoryEventData
	{
		uint64_t mAddress;
		void init( uint64_t addr )
		{
			mAddress = addr;
		}

		void init( const MemoryEventData& inData)
		{
			mAddress = inData.mAddress;
		}

		bool operator==( const MemoryEventData& inOther ) const
		{
			return mAddress == inOther.mAddress;
		}
		
		void setup( MemoryEventHeader& inHeader ) const
		{
			inHeader.setAddrCompress( findCompressionValue( mAddress ) );
		}

		template<typename TStreamType>
		void streamify( TStreamType& inStream, const MemoryEventHeader& inHeader )
		{
			inStream.streamify( "Address", mAddress, inHeader.getAddrCompress() );
		}
	};

	struct AllocationEvent : public MemoryEventData
	{
		uint32_t mSize;
		uint32_t mType;
		uint32_t mFile;
		uint32_t mLine;
		void init( size_t size = 0, uint32_t type = 0, uint32_t file = 0, uint32_t line = 0, uint64_t addr = 0 )
		{
			MemoryEventData::init( addr );
			mSize = static_cast<uint32_t>( size );
			mType = type;
			mFile = file;
			mLine = line;
		}

		void init( const AllocationEvent& inData )
		{
			MemoryEventData::init( inData );
			mSize = inData.mSize;
			mType = inData.mType;
			mFile = inData.mFile;
			mLine = inData.mLine;
		}

		bool operator==( const AllocationEvent& inOther ) const
		{
			return MemoryEventData::operator==( inOther )
				&& mSize == inOther.mSize
				&& mType == inOther.mType
				&& mFile == inOther.mFile
				&& mLine == inOther.mLine;
		}

		void setup( MemoryEventHeader& inHeader ) const
		{
			inHeader.setTypeCompress( findCompressionValue( mType ) );
			inHeader.setFnameCompress( findCompressionValue( mFile ) );
			inHeader.setSizeCompress( findCompressionValue( mSize ) );
			inHeader.setLineCompress( findCompressionValue( mLine ) );
			MemoryEventData::setup( inHeader );
		}

		template<typename TStreamType>
		void streamify( TStreamType& inStream, const MemoryEventHeader& inHeader )
		{
			inStream.streamify( "Size", mSize, inHeader.getSizeCompress() );
			inStream.streamify( "Type", mType, inHeader.getTypeCompress() );
			inStream.streamify( "File", mFile, inHeader.getFnameCompress() );
			inStream.streamify( "Line", mLine, inHeader.getLineCompress() );
			MemoryEventData::streamify( inStream, inHeader );
		}
	};
	template<> inline MemoryEventTypes::Enum getMemoryEventType<AllocationEvent>() { return MemoryEventTypes::AllocationEvent; }
	

	struct FullAllocationEvent : public MemoryEventData
	{
		size_t mSize;
		const char* mType;
		const char* mFile;
		uint32_t mLine;
		void init( size_t size, const char* type, const char* file, uint32_t line, uint64_t addr )
		{
			MemoryEventData::init( addr );
			mSize = size;
			mType = type;
			mFile = file;
			mLine = line;
		}

		void init( const FullAllocationEvent& inData )
		{
			MemoryEventData::init( inData );
			mSize = inData.mSize;
			mType = inData.mType;
			mFile = inData.mFile;
			mLine = inData.mLine;
		}

		bool operator==( const FullAllocationEvent& inOther ) const
		{
			return MemoryEventData::operator==( inOther )
				&& mSize == inOther.mSize
				&& safeStrEq( mType, inOther.mType )
				&& safeStrEq( mFile, inOther.mFile )
				&& mLine == inOther.mLine;
		}
			
		void setup( MemoryEventHeader& ) const {}
	};

	template<> inline MemoryEventTypes::Enum getMemoryEventType<FullAllocationEvent>() { return MemoryEventTypes::FullAllocationEvent; }

	struct DeallocationEvent : public MemoryEventData
	{
		void init( uint64_t addr = 0 ) { MemoryEventData::init( addr ); }
		void init( const DeallocationEvent& inData ) { MemoryEventData::init( inData ); }
	};
	
	template<> inline MemoryEventTypes::Enum getMemoryEventType<DeallocationEvent>() { return MemoryEventTypes::DeallocationEvent; }

	class MemoryEvent
	{
	public:
		typedef PX_PROFILE_UNION_5(StringTableEvent, AllocationEvent, DeallocationEvent, FullAllocationEvent, uint8_t) EventData;

	private:
		MemoryEventHeader mHeader;
		EventData mData;
	public:
		
		MemoryEvent() {}
		MemoryEvent( MemoryEventHeader inHeader, const EventData& inData = EventData() )
			: mHeader( inHeader )
			, mData( inData )
		{
		}

		template<typename TDataType>
		MemoryEvent( const TDataType& inType )
			: mHeader( getMemoryEventType<TDataType>() )
			, mData( inType )
		{
			//set the appropriate compression bits.
			inType.setup( mHeader );
		}
		const MemoryEventHeader& getHeader() const { return mHeader; }
		const EventData& getData() const { return mData; }

		template<typename TDataType>
		const TDataType& getValue() const { PX_ASSERT( mHeader.getType() == getMemoryEventType<TDataType>() ); return mData.toType<TDataType>(); }

		template<typename TDataType>
		TDataType& getValue() { PX_ASSERT( mHeader.getType() == getMemoryEventType<TDataType>() ); return mData.toType<TDataType>(); }

		template<typename TRetVal, typename TOperator>
		inline TRetVal visit( TOperator inOp ) const;

		bool operator==( const MemoryEvent& inOther ) const
		{
			if ( !(mHeader == inOther.mHeader ) ) return false;
			if ( mHeader.getType() )
				return inOther.visit<bool>( EventDataEqualOperator<EventData>( mData ) );
			return true;
		}
	};

	template<typename TRetVal, typename TOperator>
	inline TRetVal visit( MemoryEventTypes::Enum inEventType, const MemoryEvent::EventData& inData, TOperator inOperator )
	{
		switch( inEventType )
		{
		case MemoryEventTypes::StringTableEvent:		return inOperator( inData.toType( Type2Type<StringTableEvent>() ) );
		case MemoryEventTypes::AllocationEvent:			return inOperator( inData.toType( Type2Type<AllocationEvent>() ) );
		case MemoryEventTypes::DeallocationEvent:		return inOperator( inData.toType( Type2Type<DeallocationEvent>() ) );
		case MemoryEventTypes::FullAllocationEvent:		return inOperator( inData.toType( Type2Type<FullAllocationEvent>() ) );
		case MemoryEventTypes::Unknown:					return inOperator( static_cast<uint8_t>( inEventType ) );
		}
		return TRetVal();
	}

	template<typename TRetVal, typename TOperator>
	inline TRetVal MemoryEvent::visit( TOperator inOp ) const
	{ 
		return physx::profile::visit<TRetVal>( mHeader.getType(), mData, inOp ); 
	}
}}

#endif // PXPVDSDK_PXPROFILEMEMORYEVENTS_H
