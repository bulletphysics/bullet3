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


#ifndef PXPVDSDK_PXPROFILEZONEMANAGERIMPL_H
#define PXPVDSDK_PXPROFILEZONEMANAGERIMPL_H

#include "PxProfileZoneManager.h"
#include "PxProfileScopedMutexLock.h"
#include "PxPvdProfileZone.h"
#include "PxProfileAllocatorWrapper.h"

#include "PsArray.h"
#include "PsMutex.h"

namespace physx { namespace profile {

	struct NullEventNameProvider : public PxProfileNameProvider
	{
		virtual PxProfileNames getProfileNames() const { return PxProfileNames( 0, 0 ); }
	};

	class ZoneManagerImpl : public PxProfileZoneManager
	{
		typedef ScopedLockImpl<shdfnd::Mutex> TScopedLockType;
		PxProfileAllocatorWrapper					mWrapper;
		PxProfileArray<PxProfileZone*>		mZones;
		PxProfileArray<PxProfileZoneHandler*>	mHandlers;
		shdfnd::Mutex mMutex;

		ZoneManagerImpl( const ZoneManagerImpl& inOther );
		ZoneManagerImpl& operator=( const ZoneManagerImpl& inOther );

	public:

		ZoneManagerImpl(PxAllocatorCallback* inFoundation) 
			: mWrapper( inFoundation )
			, mZones( mWrapper )
			, mHandlers( mWrapper ) 
		{}

		virtual ~ZoneManagerImpl()
		{
			//This assert would mean that a profile zone is outliving us.
			//This will cause a crash when the profile zone is released.
			PX_ASSERT( mZones.size() == 0 );
			while( mZones.size() )
				removeProfileZone( *mZones.back() );
		}

		virtual void addProfileZone( PxProfileZone& inSDK )
		{
			TScopedLockType lock( &mMutex );
			
			if ( inSDK.getProfileZoneManager() != NULL )
			{
				if ( inSDK.getProfileZoneManager() == this )
					return;
				else //there must be two managers in the system somehow.
				{
					PX_ASSERT( false );
					inSDK.getProfileZoneManager()->removeProfileZone( inSDK );
				}
			}
			mZones.pushBack( &inSDK );
			inSDK.setProfileZoneManager( this );
			for ( uint32_t idx =0; idx < mHandlers.size(); ++idx )
				mHandlers[idx]->onZoneAdded( inSDK );
		}

		virtual void removeProfileZone( PxProfileZone& inSDK )
		{
			TScopedLockType lock( &mMutex );
			if ( inSDK.getProfileZoneManager() == NULL )
				return;

			else if ( inSDK.getProfileZoneManager() != this )
			{
				PX_ASSERT( false );
				inSDK.getProfileZoneManager()->removeProfileZone( inSDK );
				return;
			}

			inSDK.setProfileZoneManager( NULL );
			for ( uint32_t idx = 0; idx < mZones.size(); ++idx )
			{
				if ( mZones[idx] == &inSDK )
				{
					for ( uint32_t handler =0; handler < mHandlers.size(); ++handler )
						mHandlers[handler]->onZoneRemoved( inSDK );
					mZones.replaceWithLast( idx );
				}
			}
		}

		virtual void flushProfileEvents()
		{
			uint32_t sdkCount = mZones.size();
			for ( uint32_t idx = 0; idx < sdkCount; ++idx )
				mZones[idx]->flushProfileEvents();
		}

		virtual void addProfileZoneHandler( PxProfileZoneHandler& inHandler )
		{
			TScopedLockType lock( &mMutex );
			mHandlers.pushBack( &inHandler );
			for ( uint32_t idx = 0; idx < mZones.size(); ++idx )
				inHandler.onZoneAdded( *mZones[idx] );
		}

		virtual void removeProfileZoneHandler( PxProfileZoneHandler& inHandler )
		{
			TScopedLockType lock( &mMutex );
			for( uint32_t idx = 0; idx < mZones.size(); ++idx )
				inHandler.onZoneRemoved( *mZones[idx] );
			for( uint32_t idx = 0; idx < mHandlers.size(); ++idx )
			{
				if ( mHandlers[idx] == &inHandler )
					mHandlers.replaceWithLast( idx );
			}
		}
		
		virtual PxProfileZone& createProfileZone( const char* inSDKName, PxProfileNameProvider* inProvider, uint32_t inEventBufferByteSize )
		{
			NullEventNameProvider nullProvider;
			if ( inProvider == NULL )
				inProvider = &nullProvider;
			return createProfileZone( inSDKName, inProvider->getProfileNames(), inEventBufferByteSize );
		}
		
		
		virtual PxProfileZone& createProfileZone( const char* inSDKName, PxProfileNames inNames, uint32_t inEventBufferByteSize )
		{
			PxProfileZone& retval( PxProfileZone::createProfileZone( &mWrapper.getAllocator(), inSDKName, inNames, inEventBufferByteSize ) );
			addProfileZone( retval );
			return retval;
		}

		virtual void release() 
		{  
			PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}
	};
} }


#endif // PXPVDSDK_PXPROFILEZONEMANAGERIMPL_H
