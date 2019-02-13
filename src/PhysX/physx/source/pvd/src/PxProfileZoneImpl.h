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


#ifndef PXPVDSDK_PXPROFILEZONEIMPL_H
#define PXPVDSDK_PXPROFILEZONEIMPL_H

#include "PxPvdProfileZone.h"
#include "PxProfileZoneManager.h"
#include "PxProfileContextProviderImpl.h"
#include "PxProfileScopedMutexLock.h"
#include "PxProfileEventBufferAtomic.h"
#include "PsMutex.h"

namespace physx { namespace profile {

	/**
	\brief Simple event filter that enables all events.
	*/
	struct PxProfileNullEventFilter
	{
		void setEventEnabled( const PxProfileEventId&, bool) { PX_ASSERT(false); }
		bool isEventEnabled( const PxProfileEventId&) const { return true; }
	};

	typedef shdfnd::MutexT<PxProfileWrapperReflectionAllocator<uint8_t> >	TZoneMutexType;
	typedef ScopedLockImpl<TZoneMutexType>				TZoneLockType;
	typedef EventBuffer< PxDefaultContextProvider, TZoneMutexType, TZoneLockType, PxProfileNullEventFilter > TZoneEventBufferType;
	//typedef EventBufferAtomic< PxDefaultContextProvider, TZoneMutexType, TZoneLockType, PxProfileNullEventFilter > TZoneEventBufferType;

	template<typename TNameProvider>
	class ZoneImpl : TZoneEventBufferType //private inheritance intended
					, public PxProfileZone
					, public PxProfileEventBufferClient
	{
		typedef shdfnd::MutexT<PxProfileWrapperReflectionAllocator<uint8_t> >	TMutexType;
		typedef PxProfileHashMap<const char*, uint32_t>			TNameToEvtIndexMap;
		//ensure we don't reuse event ids.
		typedef PxProfileHashMap<uint16_t, const char*>			TEvtIdToNameMap;
		typedef TMutexType::ScopedLock						TLockType;


		const char*										mName;
		mutable TMutexType								mMutex;
		PxProfileArray<PxProfileEventName>				mEventNames;
		// to avoid locking, read-only and read-write map exist
		TNameToEvtIndexMap								mNameToEvtIndexMapR;
		TNameToEvtIndexMap								mNameToEvtIndexMapRW;
		//ensure we don't reuse event ids.
		TEvtIdToNameMap									mEvtIdToNameMap;

		PxProfileZoneManager*							mProfileZoneManager;

		PxProfileArray<PxProfileZoneClient*>			mZoneClients;
		volatile bool									mEventsActive;

		PX_NOCOPY(ZoneImpl<TNameProvider>)
	public:
		ZoneImpl( PxAllocatorCallback* inAllocator, const char* inName, uint32_t bufferSize = 0x10000 /*64k*/, const TNameProvider& inProvider = TNameProvider() )
			: TZoneEventBufferType( inAllocator, bufferSize, PxDefaultContextProvider(), NULL, PxProfileNullEventFilter() )
			, mName( inName )
			, mMutex( PxProfileWrapperReflectionAllocator<uint8_t>( mWrapper ) )
			, mEventNames( mWrapper )
			, mNameToEvtIndexMapR( mWrapper )
			, mNameToEvtIndexMapRW( mWrapper )
			, mEvtIdToNameMap( mWrapper )
			, mProfileZoneManager( NULL )
			, mZoneClients( mWrapper )
			, mEventsActive( false )
		{
			TZoneEventBufferType::setBufferMutex( &mMutex );
			//Initialize the event name structure with existing names from the name provider.
			PxProfileNames theNames( inProvider.getProfileNames() );
			for ( uint32_t idx = 0; idx < theNames.eventCount; ++idx )
			{
				const PxProfileEventName& theName (theNames.events[idx]);
				doAddName( theName.name, theName.eventId.eventId, theName.eventId.compileTimeEnabled );
			}
			TZoneEventBufferType::addClient( *this );
		}

		virtual ~ZoneImpl() {
			if ( mProfileZoneManager != NULL )
				mProfileZoneManager->removeProfileZone( *this );
			mProfileZoneManager = NULL;
			TZoneEventBufferType::removeClient( *this );
		}

		void doAddName( const char* inName, uint16_t inEventId, bool inCompileTimeEnabled )
		{
			TLockType theLocker( mMutex );
			mEvtIdToNameMap.insert( inEventId, inName );
			uint32_t idx = static_cast<uint32_t>( mEventNames.size() );
			mNameToEvtIndexMapRW.insert( inName, idx );
			mEventNames.pushBack( PxProfileEventName( inName, PxProfileEventId( inEventId, inCompileTimeEnabled ) ) );
		}

		virtual void flushEventIdNameMap()
		{
			// copy the RW map into R map
			if (mNameToEvtIndexMapRW.size())
			{
				for (TNameToEvtIndexMap::Iterator iter = mNameToEvtIndexMapRW.getIterator(); !iter.done(); ++iter)
				{
					mNameToEvtIndexMapR.insert(iter->first, iter->second);
				}
				mNameToEvtIndexMapRW.clear();
			}
		}

		virtual uint16_t getEventIdForName( const char* inName )
		{
			return getEventIdsForNames( &inName, 1 );
		}

		virtual uint16_t getEventIdsForNames( const char** inNames, uint32_t inLen )
		{
			if ( inLen == 0 )
				return 0;

			// search the read-only map first
			const TNameToEvtIndexMap::Entry* theEntry( mNameToEvtIndexMapR.find( inNames[0] ) );
			if ( theEntry )
				return mEventNames[theEntry->second].eventId;

			TLockType theLocker(mMutex);

			const TNameToEvtIndexMap::Entry* theReEntry(mNameToEvtIndexMapRW.find(inNames[0]));
			if (theReEntry)
				return mEventNames[theReEntry->second].eventId;

			//Else git R dun.
			uint16_t nameSize = static_cast<uint16_t>( mEventNames.size() );
			//We don't allow 0 as an event id.
			uint16_t eventId = nameSize;
			//Find a contiguous set of unique event ids
			bool foundAnEventId = false;
			do
			{
				foundAnEventId = false;
				++eventId;
				for ( uint16_t idx = 0; idx < inLen && foundAnEventId == false; ++idx )
					foundAnEventId = mEvtIdToNameMap.find( uint16_t(eventId + idx) ) != NULL;
			}
			while( foundAnEventId );

			uint32_t clientCount = mZoneClients.size();
			for ( uint16_t nameIdx = 0; nameIdx < inLen; ++nameIdx )
			{
				uint16_t newId = uint16_t(eventId + nameIdx);
				doAddName( inNames[nameIdx], newId, true );
				for( uint32_t clientIdx =0; clientIdx < clientCount; ++clientIdx )
					mZoneClients[clientIdx]->handleEventAdded( PxProfileEventName( inNames[nameIdx], PxProfileEventId( newId ) ) );
			}

			return eventId;
		}

		virtual void setProfileZoneManager(PxProfileZoneManager* inMgr)
		{
			mProfileZoneManager = inMgr;
		}

		virtual PxProfileZoneManager* getProfileZoneManager()
		{
			return mProfileZoneManager;
		}



		const char* getName() { return mName; }

		PxProfileEventBufferClient* getEventBufferClient() { return this; }

		//SDK implementation

		void addClient( PxProfileZoneClient& inClient )
		{
			TLockType lock( mMutex );
			mZoneClients.pushBack( &inClient );
			mEventsActive = true;
		}

		void removeClient( PxProfileZoneClient& inClient )
		{
			TLockType lock( mMutex );
			for ( uint32_t idx =0; idx < mZoneClients.size(); ++idx )
			{
				if (mZoneClients[idx] == &inClient )
				{
					inClient.handleClientRemoved();
					mZoneClients.replaceWithLast( idx );
					break;
				}
			}
			mEventsActive = mZoneClients.size() != 0;
		}

		virtual bool hasClients() const
		{
			return mEventsActive;
		}

		virtual PxProfileNames getProfileNames() const
		{
			TLockType theLocker( mMutex );
			const PxProfileEventName* theNames = mEventNames.begin();
			uint32_t theEventCount = uint32_t(mEventNames.size());
			return PxProfileNames( theEventCount, theNames );
		}

		virtual void release()
		{
			PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}

		//Implementation chaining the buffer flush to our clients
		virtual void handleBufferFlush( const uint8_t* inData, uint32_t inLength )
		{
			TLockType theLocker( mMutex );

			uint32_t clientCount = mZoneClients.size();
			for( uint32_t idx =0; idx < clientCount; ++idx )
				mZoneClients[idx]->handleBufferFlush( inData, inLength );
		}
		//Happens if something removes all the clients from the manager.
		virtual void handleClientRemoved() {}

		//Send a profile event, optionally with a context.  Events are sorted by thread
		//and context in the client side.
		virtual void startEvent( uint16_t inId, uint64_t contextId)
		{
			if( mEventsActive )
			{
				TZoneEventBufferType::startEvent( inId, contextId );
			}
		}
		virtual void stopEvent( uint16_t inId, uint64_t contextId)
		{
			if( mEventsActive )
			{
				TZoneEventBufferType::stopEvent( inId, contextId );
			}
		}

		virtual void startEvent( uint16_t inId, uint64_t contextId, uint32_t threadId)
		{
			if( mEventsActive )
			{
				TZoneEventBufferType::startEvent( inId, contextId, threadId );
			}
		}
		virtual void stopEvent( uint16_t inId, uint64_t contextId, uint32_t threadId )
		{
			if( mEventsActive )
			{
				TZoneEventBufferType::stopEvent( inId, contextId, threadId );
			}
		}

		virtual void atEvent(uint16_t inId, uint64_t contextId, uint32_t threadId, uint64_t start, uint64_t stop)
		{
			if (mEventsActive)
			{
				TZoneEventBufferType::startEvent(inId, threadId, contextId, 0, 0, start);
				TZoneEventBufferType::stopEvent(inId, threadId, contextId, 0, 0, stop);
			}
		}

		/**
		 *	Set an specific events value.  This is different than the profiling value
		 *	for the event; it is a value recorded and kept around without a timestamp associated
		 *	with it.  This value is displayed when the event itself is processed.
		 */
		virtual void eventValue( uint16_t inId, uint64_t contextId, int64_t inValue )
		{
			if( mEventsActive )
			{
				TZoneEventBufferType::eventValue( inId, contextId, inValue );
			}
		}
		virtual void flushProfileEvents()
		{
			TZoneEventBufferType::flushProfileEvents();
		}
	};

}}
#endif // PXPVDSDK_PXPROFILEZONEIMPL_H
