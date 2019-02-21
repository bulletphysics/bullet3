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

#ifndef PXPVDSDK_PXPROFILEEVENTS_H
#define PXPVDSDK_PXPROFILEEVENTS_H

#include "foundation/PxMath.h"
#include "foundation/PxAssert.h"

#include "PxProfileEventId.h"


#define	PX_PROFILE_UNION_1(a)					physx::profile::TUnion<a, physx::profile::Empty>
#define	PX_PROFILE_UNION_2(a,b)					physx::profile::TUnion<a, PX_PROFILE_UNION_1(b)>
#define	PX_PROFILE_UNION_3(a,b,c)				physx::profile::TUnion<a, PX_PROFILE_UNION_2(b,c)>
#define	PX_PROFILE_UNION_4(a,b,c,d)				physx::profile::TUnion<a, PX_PROFILE_UNION_3(b,c,d)>
#define	PX_PROFILE_UNION_5(a,b,c,d,e)			physx::profile::TUnion<a, PX_PROFILE_UNION_4(b,c,d,e)>
#define	PX_PROFILE_UNION_6(a,b,c,d,e,f)			physx::profile::TUnion<a, PX_PROFILE_UNION_5(b,c,d,e,f)>
#define	PX_PROFILE_UNION_7(a,b,c,d,e,f,g)		physx::profile::TUnion<a, PX_PROFILE_UNION_6(b,c,d,e,f,g)>
#define	PX_PROFILE_UNION_8(a,b,c,d,e,f,g,h)		physx::profile::TUnion<a, PX_PROFILE_UNION_7(b,c,d,e,f,g,h)>
#define	PX_PROFILE_UNION_9(a,b,c,d,e,f,g,h,i)	physx::profile::TUnion<a, PX_PROFILE_UNION_8(b,c,d,e,f,g,h,i)>

namespace physx { namespace profile {

	struct Empty {};

	template <typename T> struct Type2Type {};

	template <typename U, typename V>
	union TUnion
	{
		typedef U Head;
		typedef V Tail;

		Head	head;
		Tail	tail;

		template <typename TDataType>
		void init(const TDataType& inData)
		{
			toType(Type2Type<TDataType>()).init(inData);
		}

		template <typename TDataType>
		PX_FORCE_INLINE TDataType& toType(const Type2Type<TDataType>& outData) { return tail.toType(outData); }

		PX_FORCE_INLINE Head& toType(const Type2Type<Head>&) { return head; }

		template <typename TDataType>
		PX_FORCE_INLINE const TDataType& toType(const Type2Type<TDataType>& outData) const { return tail.toType(outData); }

		PX_FORCE_INLINE const Head& toType(const Type2Type<Head>&) const { return head; }
	};

	struct EventTypes
	{
		enum Enum
		{
			Unknown = 0,
			StartEvent,
			StopEvent,
			RelativeStartEvent, //reuses context,id from the earlier event.
			RelativeStopEvent, //reuses context,id from the earlier event.
			EventValue,
			CUDAProfileBuffer //obsolete, placeholder to skip data from PhysX SDKs < 3.4
		};
	};

	struct EventStreamCompressionFlags
	{
		enum Enum
		{
			U8 = 0,
			U16 = 1,
			U32 = 2,
			U64 = 3,
			CompressionMask = 3
		};
	};

#if (PX_PS4) || (PX_APPLE_FAMILY)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-fallthrough"
#endif

	//Find the smallest value that will represent the incoming value without loss.
	//We can enlarge the current compression value, but we can't make is smaller.
	//In this way, we can use this function to find the smallest compression setting
	//that will work for a set of values.
	inline EventStreamCompressionFlags::Enum findCompressionValue( uint64_t inValue, EventStreamCompressionFlags::Enum inCurrentCompressionValue = EventStreamCompressionFlags::U8 )
	{
		PX_ASSERT_WITH_MESSAGE( (inCurrentCompressionValue >= EventStreamCompressionFlags::U8) &&
								(inCurrentCompressionValue <= EventStreamCompressionFlags::U64),
								"Invalid inCurrentCompressionValue in profile::findCompressionValue");

		//Fallthrough is intentional
		switch( inCurrentCompressionValue ) 
		{
		case EventStreamCompressionFlags::U8:
			if ( inValue <= UINT8_MAX )
				return EventStreamCompressionFlags::U8;
		case EventStreamCompressionFlags::U16:
			if ( inValue <= UINT16_MAX )
				return EventStreamCompressionFlags::U16;
		case EventStreamCompressionFlags::U32:
			if ( inValue <= UINT32_MAX )
				return EventStreamCompressionFlags::U32;
		case EventStreamCompressionFlags::U64:
			break;
		}
		return EventStreamCompressionFlags::U64;
	}

	//Find the smallest value that will represent the incoming value without loss.
	//We can enlarge the current compression value, but we can't make is smaller.
	//In this way, we can use this function to find the smallest compression setting
	//that will work for a set of values.
	inline EventStreamCompressionFlags::Enum findCompressionValue( uint32_t inValue, EventStreamCompressionFlags::Enum inCurrentCompressionValue = EventStreamCompressionFlags::U8 )
	{
		PX_ASSERT_WITH_MESSAGE( (inCurrentCompressionValue >= EventStreamCompressionFlags::U8) &&
								(inCurrentCompressionValue <= EventStreamCompressionFlags::U64),
								"Invalid inCurrentCompressionValue in profile::findCompressionValue");

		//Fallthrough is intentional
		switch( inCurrentCompressionValue ) 
		{
		case EventStreamCompressionFlags::U8:
			if ( inValue <= UINT8_MAX )
				return EventStreamCompressionFlags::U8;
		case EventStreamCompressionFlags::U16:
			if ( inValue <= UINT16_MAX )
				return EventStreamCompressionFlags::U16;
		case EventStreamCompressionFlags::U32:
		case EventStreamCompressionFlags::U64:
			break;
		}
		return EventStreamCompressionFlags::U32;
	}

#if (PX_PS4) || (PX_APPLE_FAMILY)
#pragma clang diagnostic pop
#endif

	//Event header is 32 bytes and precedes all events.
	struct EventHeader
	{
		uint8_t	mEventType; //Used to parse the correct event out of the stream
		uint8_t	mStreamOptions; //Timestamp compression, etc.
		uint16_t	mEventId;	//16 bit per-event-system event id
		EventHeader( uint8_t type = 0, uint16_t id = 0 )
			: mEventType( type )
			, mStreamOptions( uint8_t(-1) )
			, mEventId( id )
		{
		}

		EventHeader( EventTypes::Enum type, uint16_t id )
			: mEventType( static_cast<uint8_t>( type ) )
			, mStreamOptions( uint8_t(-1) )
			, mEventId( id )
		{
		}

		EventStreamCompressionFlags::Enum getTimestampCompressionFlags() const 
		{ 
			return static_cast<EventStreamCompressionFlags::Enum> ( mStreamOptions & EventStreamCompressionFlags::CompressionMask );
		}

		uint64_t compressTimestamp( uint64_t inLastTimestamp, uint64_t inCurrentTimestamp )
		{
			mStreamOptions = EventStreamCompressionFlags::U64;
			uint64_t retval = inCurrentTimestamp;
			if ( inLastTimestamp )
			{
				retval = inCurrentTimestamp - inLastTimestamp;
				EventStreamCompressionFlags::Enum compressionValue = findCompressionValue( retval );
				mStreamOptions = static_cast<uint8_t>( compressionValue );
				if ( compressionValue == EventStreamCompressionFlags::U64 )
					retval = inCurrentTimestamp; //just send the timestamp as is.
			}
			return retval;
		}

		uint64_t uncompressTimestamp( uint64_t inLastTimestamp, uint64_t inCurrentTimestamp ) const
		{
			if ( getTimestampCompressionFlags() != EventStreamCompressionFlags::U64 )
				return inLastTimestamp + inCurrentTimestamp;
			return inCurrentTimestamp;
		}

		void setContextIdCompressionFlags( uint64_t inContextId )
		{
			uint8_t options = static_cast<uint8_t>( findCompressionValue( inContextId ) );
			mStreamOptions = uint8_t(mStreamOptions | options << 2);
		}

		EventStreamCompressionFlags::Enum getContextIdCompressionFlags() const 
		{
			return static_cast< EventStreamCompressionFlags::Enum >( ( mStreamOptions >> 2 ) & EventStreamCompressionFlags::CompressionMask );
		}

		bool operator==( const EventHeader& inOther ) const
		{
			return mEventType == inOther.mEventType
				&& mStreamOptions == inOther.mStreamOptions
				&& mEventId == inOther.mEventId;
		}

		template<typename TStreamType>
		inline uint32_t streamify( TStreamType& inStream )
		{
			uint32_t writtenSize = inStream.streamify( "EventType", mEventType ); 
			writtenSize += inStream.streamify("StreamOptions", mStreamOptions); //Timestamp compression, etc.
			writtenSize += inStream.streamify("EventId", mEventId);	//16 bit per-event-system event id
			return writtenSize;
		}


	};

	//Declaration of type level getEventType function that maps enumeration event types to datatypes
	template<typename TDataType>
	inline EventTypes::Enum getEventType() { PX_ASSERT( false ); return EventTypes::Unknown; }

	//Relative profile event means this event is sharing the context and thread id
	//with the event before it.
	struct RelativeProfileEvent
	{
		uint64_t	mTensOfNanoSeconds; //timestamp is in tensOfNanonseconds
		void init( uint64_t inTs ) { mTensOfNanoSeconds = inTs; }
		void init( const RelativeProfileEvent& inData ) { mTensOfNanoSeconds = inData.mTensOfNanoSeconds; }
		bool operator==( const RelativeProfileEvent& other ) const 
		{ 
			return mTensOfNanoSeconds == other.mTensOfNanoSeconds;
		}
		template<typename TStreamType> 
		uint32_t streamify( TStreamType& inStream, const EventHeader& inHeader )
		{
			return inStream.streamify( "TensOfNanoSeconds", mTensOfNanoSeconds, inHeader.getTimestampCompressionFlags() );
		}
		uint64_t getTimestamp() const { return mTensOfNanoSeconds; }
		void setTimestamp( uint64_t inTs ) { mTensOfNanoSeconds = inTs; }
		void setupHeader( EventHeader& inHeader, uint64_t inLastTimestamp )
		{
			mTensOfNanoSeconds = inHeader.compressTimestamp( inLastTimestamp, mTensOfNanoSeconds );
		}

		uint32_t getEventSize(const EventHeader& inHeader)
		{	
			uint32_t size = 0;
			switch (inHeader.getTimestampCompressionFlags())
			{
			case EventStreamCompressionFlags::U8:
				size = 1;
				break;
			case EventStreamCompressionFlags::U16:
				size = 2;
				break;
			case EventStreamCompressionFlags::U32:
				size = 4;
				break;
			case EventStreamCompressionFlags::U64:
				size = 8;
				break;
			}
			return size;
		}
	};

	//Start version of the relative event.
	struct RelativeStartEvent : public RelativeProfileEvent
	{
		void init( uint64_t inTs = 0 ) { RelativeProfileEvent::init( inTs ); }
		void init( const RelativeStartEvent& inData ) { RelativeProfileEvent::init( inData ); }
		template<typename THandlerType>
		void handle( THandlerType* inHdlr, uint16_t eventId, uint32_t thread, uint64_t context, uint8_t inCpuId, uint8_t threadPriority ) const
		{
			inHdlr->onStartEvent( PxProfileEventId( eventId ), thread, context, inCpuId, threadPriority, mTensOfNanoSeconds );
		}
	};
	
	template<> inline EventTypes::Enum getEventType<RelativeStartEvent>() { return EventTypes::RelativeStartEvent; }
	
	//Stop version of relative event.
	struct RelativeStopEvent : public RelativeProfileEvent
	{
		void init( uint64_t inTs = 0 ) { RelativeProfileEvent::init( inTs ); }
		void init( const RelativeStopEvent& inData ) { RelativeProfileEvent::init( inData ); }
		template<typename THandlerType>
		void handle( THandlerType* inHdlr, uint16_t eventId, uint32_t thread, uint64_t context, uint8_t inCpuId, uint8_t threadPriority ) const
		{
			inHdlr->onStopEvent( PxProfileEventId( eventId ), thread, context, inCpuId, threadPriority, mTensOfNanoSeconds );
		}
	};

	template<> inline EventTypes::Enum getEventType<RelativeStopEvent>() { return EventTypes::RelativeStopEvent; }

	struct EventContextInformation
	{
		uint64_t mContextId;
		uint32_t mThreadId; //Thread this event was taken from
		uint8_t  mThreadPriority;
		uint8_t  mCpuId;

		void init( uint32_t inThreadId = UINT32_MAX
								, uint64_t inContextId = (uint64_t(-1))
								, uint8_t inPriority = UINT8_MAX
								, uint8_t inCpuId = UINT8_MAX )
		{
			mContextId = inContextId;
			mThreadId = inThreadId;
			mThreadPriority = inPriority;
			mCpuId = inCpuId;
		}

		void init( const EventContextInformation& inData )
		{
			mContextId = inData.mContextId;
			mThreadId = inData.mThreadId;
			mThreadPriority = inData.mThreadPriority;
			mCpuId = inData.mCpuId;
		}

		template<typename TStreamType> 
		uint32_t streamify( TStreamType& inStream, EventStreamCompressionFlags::Enum inContextIdFlags )
		{
			uint32_t writtenSize = inStream.streamify( "ThreadId", mThreadId );
			writtenSize += inStream.streamify("ContextId", mContextId, inContextIdFlags);
			writtenSize += inStream.streamify("ThreadPriority", mThreadPriority);
			writtenSize += inStream.streamify("CpuId", mCpuId);
			return writtenSize;
		}
		
		bool operator==( const EventContextInformation& other ) const 
		{ 
			return mThreadId == other.mThreadId
				&& mContextId == other.mContextId
				&& mThreadPriority == other.mThreadPriority
				&& mCpuId == other.mCpuId;
		}

		void setToDefault()
		{
			*this = EventContextInformation();
		}
	};
	
	//Profile event contains all the data required to tell the profile what is going
	//on.
	struct ProfileEvent
	{
		EventContextInformation mContextInformation;
		RelativeProfileEvent	mTimeData; //timestamp in seconds.
		void init( uint32_t inThreadId, uint64_t inContextId, uint8_t inCpuId, uint8_t inPriority, uint64_t inTs )
		{
			mContextInformation.init( inThreadId, inContextId, inPriority, inCpuId );
			mTimeData.init( inTs );
		}

		void init( const ProfileEvent& inData )
		{
			mContextInformation.init( inData.mContextInformation );
			mTimeData.init( inData.mTimeData );
		}

		bool operator==( const ProfileEvent& other ) const 
		{ 
			return mContextInformation == other.mContextInformation
					&& mTimeData == other.mTimeData; 
		}

		template<typename TStreamType> 
		uint32_t streamify( TStreamType& inStream, const EventHeader& inHeader )
		{
			uint32_t writtenSize = mContextInformation.streamify(inStream, inHeader.getContextIdCompressionFlags());
			writtenSize += mTimeData.streamify(inStream, inHeader);
			return writtenSize;
		}

		uint32_t getEventSize(const EventHeader& inHeader)
		{
			uint32_t eventSize = 0;
			// time is stored depending on the conpress flag mTimeData.streamify(inStream, inHeader);
			switch (inHeader.getTimestampCompressionFlags())
			{
			case EventStreamCompressionFlags::U8:
				eventSize++;
				break;
			case EventStreamCompressionFlags::U16:
				eventSize += 2;
				break;
			case EventStreamCompressionFlags::U32:
				eventSize += 4;
				break;
			case EventStreamCompressionFlags::U64:
				eventSize += 8;
				break;
			}

			// context information
			// mContextInformation.streamify( inStream, inHeader.getContextIdCompressionFlags() );
			eventSize += 6;  // 		uint32_t mThreadId; uint8_t  mThreadPriority; uint8_t  mCpuId;
			switch (inHeader.getContextIdCompressionFlags())
			{
			case EventStreamCompressionFlags::U8:
				eventSize++;
				break;
			case EventStreamCompressionFlags::U16:
				eventSize += 2;
				break;
			case EventStreamCompressionFlags::U32:
				eventSize += 4;
				break;
			case EventStreamCompressionFlags::U64:
				eventSize += 8;
				break;
			}

			return eventSize;
		}

		uint64_t getTimestamp() const { return mTimeData.getTimestamp(); }
		void setTimestamp( uint64_t inTs ) { mTimeData.setTimestamp( inTs ); }
		
		void setupHeader( EventHeader& inHeader, uint64_t inLastTimestamp )
		{
			mTimeData.setupHeader( inHeader, inLastTimestamp );
			inHeader.setContextIdCompressionFlags( mContextInformation.mContextId );
		}
	};

	//profile start event starts the profile session.
	struct StartEvent : public ProfileEvent
	{
		void init( uint32_t inThreadId = 0, uint64_t inContextId = 0, uint8_t inCpuId = 0, uint8_t inPriority = 0, uint64_t inTensOfNanoSeconds = 0 ) 
		{
			ProfileEvent::init( inThreadId, inContextId, inCpuId, inPriority, inTensOfNanoSeconds );
		}
		void init( const StartEvent& inData )
		{
			ProfileEvent::init( inData );
		}

		RelativeStartEvent getRelativeEvent() const { RelativeStartEvent theEvent; theEvent.init( mTimeData.mTensOfNanoSeconds ); return theEvent; }
		EventTypes::Enum getRelativeEventType() const { return getEventType<RelativeStartEvent>(); }
	};
	
	template<> inline EventTypes::Enum getEventType<StartEvent>() { return EventTypes::StartEvent; }

	//Profile stop event stops the profile session.
	struct StopEvent : public ProfileEvent
	{
		void init( uint32_t inThreadId = 0, uint64_t inContextId = 0, uint8_t inCpuId = 0, uint8_t inPriority = 0, uint64_t inTensOfNanoSeconds = 0 )
		{
			ProfileEvent::init( inThreadId, inContextId, inCpuId, inPriority, inTensOfNanoSeconds );
		}
		void init( const StopEvent& inData )
		{
			ProfileEvent::init( inData );
		}
		RelativeStopEvent getRelativeEvent() const { RelativeStopEvent theEvent; theEvent.init( mTimeData.mTensOfNanoSeconds ); return theEvent; }
		EventTypes::Enum getRelativeEventType() const { return getEventType<RelativeStopEvent>(); }
	};
	
	template<> inline EventTypes::Enum getEventType<StopEvent>() { return EventTypes::StopEvent; }

	struct EventValue
	{
		uint64_t	mValue;
		uint64_t	mContextId;
		uint32_t	mThreadId;
		void init( int64_t inValue = 0, uint64_t inContextId = 0, uint32_t inThreadId = 0 )
		{
			mValue = static_cast<uint64_t>( inValue );
			mContextId = inContextId;
			mThreadId = inThreadId;
		}

		void init( const EventValue& inData )
		{
			mValue = inData.mValue;
			mContextId = inData.mContextId;
			mThreadId = inData.mThreadId;
		}

		int64_t getValue() const { return static_cast<int16_t>( mValue ); }

		void setupHeader( EventHeader& inHeader )
		{
			mValue = inHeader.compressTimestamp( 0, mValue );
			inHeader.setContextIdCompressionFlags( mContextId );
		}

		template<typename TStreamType> 
		uint32_t streamify( TStreamType& inStream, const EventHeader& inHeader )
		{
			uint32_t writtenSize = inStream.streamify("Value", mValue, inHeader.getTimestampCompressionFlags());
			writtenSize += inStream.streamify("ContextId", mContextId, inHeader.getContextIdCompressionFlags());
			writtenSize += inStream.streamify("ThreadId", mThreadId);
			return writtenSize;
		}

		uint32_t getEventSize(const EventHeader& inHeader)
		{
			uint32_t eventSize = 0;
			// value
			switch (inHeader.getTimestampCompressionFlags())
			{
			case EventStreamCompressionFlags::U8:
				eventSize++;
				break;
			case EventStreamCompressionFlags::U16:
				eventSize += 2;
				break;
			case EventStreamCompressionFlags::U32:
				eventSize += 4;
				break;
			case EventStreamCompressionFlags::U64:
				eventSize += 8;
				break;
			}

			// context information						
			switch (inHeader.getContextIdCompressionFlags())
			{
			case EventStreamCompressionFlags::U8:
				eventSize++;
				break;
			case EventStreamCompressionFlags::U16:
				eventSize += 2;
				break;
			case EventStreamCompressionFlags::U32:
				eventSize += 4;
				break;
			case EventStreamCompressionFlags::U64:
				eventSize += 8;
				break;
			}

			eventSize += 4;  // 		uint32_t mThreadId;

			return eventSize;
		}

		bool operator==( const EventValue& other ) const 
		{ 
			return mValue == other.mValue
				&& mContextId == other.mContextId
				&& mThreadId == other.mThreadId;
		}

		template<typename THandlerType>
		void handle( THandlerType* inHdlr, uint16_t eventId ) const
		{
			inHdlr->onEventValue( PxProfileEventId( eventId ), mThreadId, mContextId, getValue() );
		}

	};
	template<> inline EventTypes::Enum getEventType<EventValue>() { return EventTypes::EventValue; }

	//obsolete, placeholder to skip data from PhysX SDKs < 3.4
	struct CUDAProfileBuffer
	{
		uint64_t mTimestamp;
		float mTimespan;
		const uint8_t* mCudaData;
		uint32_t mBufLen;
		uint32_t mVersion;

		template<typename TStreamType> 
		uint32_t streamify( TStreamType& inStream, const EventHeader& )
		{
			uint32_t writtenSize = inStream.streamify("Timestamp", mTimestamp);
			writtenSize += inStream.streamify("Timespan", mTimespan);
			writtenSize += inStream.streamify("CudaData", mCudaData, mBufLen);
			writtenSize += inStream.streamify("BufLen", mBufLen);
			writtenSize += inStream.streamify("Version", mVersion);
			return writtenSize;
		}

		bool operator==( const CUDAProfileBuffer& other ) const 
		{ 
			return mTimestamp == other.mTimestamp
				&& mTimespan == other.mTimespan
				&& mBufLen == other.mBufLen
				&& memcmp( mCudaData, other.mCudaData, mBufLen ) == 0
				&& mVersion == other.mVersion;
		}
	};

	template<> inline EventTypes::Enum getEventType<CUDAProfileBuffer>() { return EventTypes::CUDAProfileBuffer; }

	//Provides a generic equal operation for event data objects.
	template <typename TEventData>
	struct EventDataEqualOperator
	{
		TEventData mData;
		EventDataEqualOperator( const TEventData& inD ) : mData( inD ) {}
		template<typename TDataType> bool operator()( const TDataType& inRhs ) const { return mData.toType( Type2Type<TDataType>() ) == inRhs; }
		bool operator()() const { return false; }
	};

	/**
	 *	Generic event container that combines and even header with the generic event data type.
	 *	Provides unsafe and typesafe access to the event data.
	 */
	class Event
	{
	public:
		typedef PX_PROFILE_UNION_7(StartEvent, StopEvent, RelativeStartEvent, RelativeStopEvent, EventValue, CUDAProfileBuffer, uint8_t) EventData;

	private:
		EventHeader mHeader;
		EventData	mData;
	public:
		Event() {}

		template <typename TDataType>
		Event( EventHeader inHeader, const TDataType& inData )
			: mHeader( inHeader )
		{
			mData.init<TDataType>(inData);
		}

		template<typename TDataType>
		Event( uint16_t eventId, const TDataType& inData )
			: mHeader( getEventType<TDataType>(), eventId )
		{
			mData.init<TDataType>(inData);
		}
		const EventHeader& getHeader() const { return mHeader; }
		const EventData& getData() const { return mData; }

		template<typename TDataType>
		const TDataType& getValue() const { PX_ASSERT( mHeader.mEventType == getEventType<TDataType>() ); return mData.toType<TDataType>(); }

		template<typename TDataType>
		TDataType& getValue() { PX_ASSERT( mHeader.mEventType == getEventType<TDataType>() ); return mData.toType<TDataType>(); }

		template<typename TRetVal, typename TOperator>
		inline TRetVal visit( TOperator inOp ) const;

		bool operator==( const Event& inOther ) const
		{
			if ( !(mHeader == inOther.mHeader ) ) return false;
			if ( mHeader.mEventType )
				return inOther.visit<bool>( EventDataEqualOperator<EventData>( mData ) );
			return true;
		}
	};

	//Combining the above union type with an event type means that an object can get the exact
	//data out of the union.  Using this function means that all callsites will be forced to
	//deal with the newer datatypes and that the switch statement only exists in once place.
	//Implements conversion from enum -> datatype
	template<typename TRetVal, typename TOperator>
	TRetVal visit( EventTypes::Enum inEventType, const Event::EventData& inData, TOperator inOperator )
	{
		switch( inEventType )
		{
		case EventTypes::StartEvent:			return inOperator( inData.toType( Type2Type<StartEvent>() ) );
		case EventTypes::StopEvent:				return inOperator( inData.toType( Type2Type<StopEvent>() ) );
		case EventTypes::RelativeStartEvent:	return inOperator( inData.toType( Type2Type<RelativeStartEvent>() ) );
		case EventTypes::RelativeStopEvent:		return inOperator( inData.toType( Type2Type<RelativeStopEvent>() ) );
		case EventTypes::EventValue:			return inOperator( inData.toType( Type2Type<EventValue>() ) );
		//obsolete, placeholder to skip data from PhysX SDKs < 3.4
		case EventTypes::CUDAProfileBuffer:		return inOperator( inData.toType( Type2Type<CUDAProfileBuffer>() ) );
		case EventTypes::Unknown:				break;
		}
		uint8_t type = static_cast<uint8_t>( inEventType );
		return inOperator( type );
	}

	template<typename TRetVal, typename TOperator>
	inline TRetVal Event::visit( TOperator inOp ) const
	{ 
		return physx::profile::visit<TRetVal>( static_cast<EventTypes::Enum>(mHeader.mEventType), mData, inOp ); 
	}
} }

#endif // PXPVDSDK_PXPROFILEEVENTS_H
