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


#ifndef PXPVDSDK_PXPROFILEEVENTBUFFER_H
#define PXPVDSDK_PXPROFILEEVENTBUFFER_H

#include "PxProfileEvents.h"
#include "PxProfileEventSerialization.h"
#include "PxProfileDataBuffer.h"
#include "PxProfileContextProvider.h"

#include "PsTime.h"

namespace physx { namespace profile {

	/**
	 *	An event buffer maintains an in-memory buffer of events.  When this buffer is full
	 *	it sends to buffer to all handlers registered and resets the buffer.
	 *
	 *	It is parameterized in four ways.  The first is a context provider that provides
	 *	both thread id and context id.
	 *	
	 *	The second is the mutex (which may be null) and a scoped locking mechanism.  Thus the buffer
	 *	may be used in a multithreaded context but clients of the buffer don't pay for this if they
	 *	don't intend to use it this way.
	 *
	 *	Finally the buffer may use an event filtering mechanism.  This mechanism needs one function,
	 *	namely isEventEnabled( uint8_t subsystem, uint8_t eventId ).
	 *
	 *	All of these systems can be parameterized at compile time leading to an event buffer
	 *	that should be as fast as possible given the constraints.
	 *
	 *	Buffers may be chained together as this buffer has a handleBufferFlush method that
	 *	will grab the mutex and add the data to this event buffer.
	 *
	 *	Overall, lets look at the PhysX SDK an how all the pieces fit together.
	 *	The SDK should have a mutex-protected event buffer where actual devs or users of PhysX
	 *	can register handlers.  This buffer has slow but correct implementations of the
	 *	context provider interface.
	 *
	 *	The SDK object should also have a concrete event filter which was used in the
	 *	construction of the event buffer and which it exposes through opaque interfaces.
	 *
	 *	The SDK should protect its event buffer and its event filter from multithreaded
	 *	access and thus this provides the safest and slowest way to log events and to
	 *	enable/disable events.
	 *
	 *	Each scene should also have a concrete event filter.  This filter is updated from
	 *	the SDK event filter (in a mutex protected way) every frame.  Thus scenes can change
	 *	their event filtering on a frame-by-frame basis.  It means that tasks running
	 *	under the scene don't need a mutex when accessing the filter.
	 *
	 *	Furthermore the scene should have an event buffer that always sets the context id
	 *	on each event to the scene.  This allows PVD and other systems to correlate events
	 *	to scenes.  Scenes should provide access only to a relative event sending system
	 *	that looks up thread id upon each event but uses the scene id.
	 *
	 *	The SDK's event buffer should be setup as an EventBufferClient for each scene's
	 *	event buffer. Thus the SDK should expose an EventBufferClient interface that
	 *	any client can use.
	 *
	 *	For extremely *extremely* performance sensitive areas we should create a specialized
	 *	per-scene, per-thread event buffer that is set on the task for these occasions.  This buffer
	 *	uses a trivial event context setup with the scene's context id and the thread id.  It should
	 *	share the scene's concrete event filter and it should have absolutely no locking.  It should
	 *	empty into the scene's event buffer which in some cases should empty into the SDK's event buffer
	 *	which when full will push events all the way out of the system.  The task should *always* flush
	 *	the event buffer (if it has one) when it is finished; nothing else will work reliably.
	 *
	 *	If the per-scene,per-thread event buffer is correctly parameterized and fully defined adding
	 *	a new event should be an inline operation requiring no mutex grabs in the common case.  I don't
	 *	believe you can get faster event production than this; the events are as small as possible (all
	 *	relative events) and they are all produced inline resulting in one 4 byte header and one
	 *	8 byte timestamp per event.  Reducing the memory pressure in this way reduces the communication
	 *	overhead, the mutex grabs, basically everything that makes profiling expensive at the cost
	 *	of a per-scene,per-thread event buffer (which could easily be reduced to a per-thread event
	 *	buffer.
	 */
	template<typename TContextProvider, 
			typename TMutex, 
			typename TScopedLock,
			typename TEventFilter>
	class EventBuffer  : public DataBuffer<TMutex, TScopedLock>
	{
	public:
		typedef DataBuffer<TMutex, TScopedLock> TBaseType;
		typedef TContextProvider	TContextProviderType;
		typedef TEventFilter		TEventFilterType;
		typedef typename TBaseType::TMutexType TMutexType;
		typedef typename TBaseType::TScopedLockType TScopedLockType;
		typedef typename TBaseType::TU8AllocatorType TU8AllocatorType;
		typedef typename TBaseType::TMemoryBufferType TMemoryBufferType;
		typedef typename TBaseType::TBufferClientArray TBufferClientArray;

	private:
		EventContextInformation				mEventContextInformation;
		uint64_t								mLastTimestamp;
		TContextProvider					mContextProvider;
		TEventFilterType					mEventFilter;

	public:
		EventBuffer(PxAllocatorCallback* inFoundation
					, uint32_t inBufferFullAmount
					, const TContextProvider& inProvider
					, TMutexType* inBufferMutex
					, const TEventFilterType& inEventFilter )
					: TBaseType( inFoundation, inBufferFullAmount, inBufferMutex, "struct physx::profile::ProfileEvent" )
			, mLastTimestamp( 0 )
			, mContextProvider( inProvider )
			, mEventFilter( inEventFilter )
		{
			memset(&mEventContextInformation,0,sizeof(EventContextInformation));
		}

		TContextProvider& getContextProvider() { return mContextProvider; }

		PX_FORCE_INLINE void startEvent(uint16_t inId, uint32_t threadId, uint64_t contextId, uint8_t cpuId, uint8_t threadPriority, uint64_t inTimestamp)
		{			
			TScopedLockType lock(TBaseType::mBufferMutex);
			if ( mEventFilter.isEventEnabled( inId ) )
			{
				StartEvent theEvent;
				theEvent.init( threadId, contextId, cpuId, threadPriority, inTimestamp );
				doAddProfileEvent( inId, theEvent );
			}
		}

		PX_FORCE_INLINE void startEvent(uint16_t inId, uint64_t contextId)
		{
			PxProfileEventExecutionContext ctx( mContextProvider.getExecutionContext() );
			startEvent( inId, ctx.mThreadId, contextId, ctx.mCpuId, static_cast<uint8_t>(ctx.mThreadPriority), shdfnd::Time::getCurrentCounterValue() );
		}

		PX_FORCE_INLINE void startEvent(uint16_t inId, uint64_t contextId, uint32_t threadId)
		{
			startEvent( inId, threadId, contextId, 0, 0, shdfnd::Time::getCurrentCounterValue() );
		}

		PX_FORCE_INLINE void stopEvent(uint16_t inId, uint32_t threadId, uint64_t contextId, uint8_t cpuId, uint8_t threadPriority, uint64_t inTimestamp)
		{			
			TScopedLockType lock(TBaseType::mBufferMutex);
			if ( mEventFilter.isEventEnabled( inId ) )
			{
				StopEvent theEvent;
				theEvent.init( threadId, contextId, cpuId, threadPriority, inTimestamp );
				doAddProfileEvent( inId, theEvent );
			}
		}

		PX_FORCE_INLINE void stopEvent(uint16_t inId, uint64_t contextId)
		{
			PxProfileEventExecutionContext ctx( mContextProvider.getExecutionContext() );
			stopEvent( inId, ctx.mThreadId, contextId, ctx.mCpuId, static_cast<uint8_t>(ctx.mThreadPriority), shdfnd::Time::getCurrentCounterValue() );
		}

		PX_FORCE_INLINE void stopEvent(uint16_t inId, uint64_t contextId, uint32_t threadId)
		{
			stopEvent( inId, threadId, contextId, 0, 0, shdfnd::Time::getCurrentCounterValue() );
		}

		inline void eventValue( uint16_t inId, uint64_t contextId, int64_t inValue )
		{
			eventValue( inId, mContextProvider.getThreadId(), contextId, inValue );
		}

		inline void eventValue( uint16_t inId, uint32_t threadId, uint64_t contextId, int64_t inValue )
		{
			TScopedLockType lock( TBaseType::mBufferMutex );
			EventValue theEvent;
			theEvent.init( inValue, contextId, threadId );
			EventHeader theHeader( static_cast<uint8_t>( getEventType<EventValue>() ), inId );
			//set the header relative timestamp;
			EventValue& theType( theEvent );
			theType.setupHeader( theHeader );
			sendEvent( theHeader, theType );
		}

		void flushProfileEvents()
		{				
			TBaseType::flushEvents();
		}

		void release()
		{
			PX_PROFILE_DELETE( TBaseType::mWrapper.mUserFoundation, this );
		}
	protected:
		//Clears the cache meaning event compression
		//starts over again.
		//only called when the buffer mutex is held
		void clearCachedData()
		{
			mEventContextInformation.setToDefault();
			mLastTimestamp = 0;
		}

		template<typename TProfileEventType>
		PX_FORCE_INLINE void doAddProfileEvent(uint16_t eventId, const TProfileEventType& inType)
		{
			TScopedLockType lock(TBaseType::mBufferMutex);
			if (mEventContextInformation == inType.mContextInformation)
				doAddEvent(static_cast<uint8_t>(inType.getRelativeEventType()), eventId, inType.getRelativeEvent());
			else
			{
				mEventContextInformation = inType.mContextInformation;
				doAddEvent( static_cast<uint8_t>( getEventType<TProfileEventType>() ), eventId, inType );
			}
		}

		template<typename TDataType>
		PX_FORCE_INLINE void doAddEvent(uint8_t inEventType, uint16_t eventId, const TDataType& inType)
		{
			EventHeader theHeader( inEventType, eventId );
			//set the header relative timestamp;
			TDataType& theType( const_cast<TDataType&>( inType ) );
			uint64_t currentTs =  inType.getTimestamp();
			theType.setupHeader(theHeader, mLastTimestamp);
			mLastTimestamp = currentTs;
			sendEvent( theHeader, theType );
		}

		template<typename TDataType>
		PX_FORCE_INLINE void sendEvent( EventHeader& inHeader, TDataType& inType )
		{			
			uint32_t sizeToWrite = sizeof(inHeader) + inType.getEventSize(inHeader);
			PX_UNUSED(sizeToWrite);

			uint32_t writtenSize = inHeader.streamify( TBaseType::mSerializer );
			writtenSize += inType.streamify(TBaseType::mSerializer, inHeader);

			PX_ASSERT(writtenSize == sizeToWrite);

			if ( TBaseType::mDataArray.size() >= TBaseType::mBufferFullAmount )
				flushProfileEvents();

		}

	};
}}
#endif // PXPVDSDK_PXPROFILEEVENTBUFFER_H
