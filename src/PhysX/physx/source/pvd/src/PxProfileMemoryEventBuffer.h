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


#ifndef PXPVDSDK_PXPROFILEMEMORYEVENTBUFFER_H
#define PXPVDSDK_PXPROFILEMEMORYEVENTBUFFER_H

#include "PxProfileDataBuffer.h"
#include "PxProfileMemoryEvents.h"
#include "PxProfileMemory.h"
#include "PxProfileScopedMutexLock.h"
#include "PxProfileAllocatorWrapper.h"
#include "PxProfileEventMutex.h"

#include "PsHash.h"
#include "PsHashMap.h"
#include "PsUserAllocated.h"

namespace physx { namespace profile {

	template<typename TMutex,
			 typename TScopedLock>
	class MemoryEventBuffer : public DataBuffer<TMutex, TScopedLock>
	{
	public:
		typedef DataBuffer<TMutex, TScopedLock> TBaseType;
		typedef typename TBaseType::TMutexType TMutexType;
		typedef typename TBaseType::TScopedLockType TScopedLockType;
		typedef typename TBaseType::TU8AllocatorType TU8AllocatorType;
		typedef typename TBaseType::TMemoryBufferType TMemoryBufferType;
		typedef typename TBaseType::TBufferClientArray TBufferClientArray;
		typedef shdfnd::HashMap<const char*, uint32_t, shdfnd::Hash<const char*>, TU8AllocatorType> TCharPtrToHandleMap;

	protected:
		TCharPtrToHandleMap mStringTable;

	public:

		MemoryEventBuffer( PxAllocatorCallback& cback
					, uint32_t inBufferFullAmount
					, TMutexType* inBufferMutex )
			: TBaseType( &cback, inBufferFullAmount, inBufferMutex, "struct physx::profile::MemoryEvent" )
			, mStringTable( TU8AllocatorType( TBaseType::getWrapper(), "MemoryEventStringBuffer" ) )
		{
		}

		uint32_t getHandle( const char* inData )
		{
			if ( inData == NULL ) inData = "";
			const typename TCharPtrToHandleMap::Entry* result( mStringTable.find( inData ) );
			if ( result )
				return result->second;
			uint32_t hdl = mStringTable.size() + 1;
			mStringTable.insert( inData, hdl );
			StringTableEvent theEvent;
			theEvent.init( inData, hdl );
			sendEvent( theEvent );
			return hdl;
		}

		void onAllocation( size_t inSize, const char* inType, const char* inFile, uint32_t inLine, uint64_t addr )
		{
			if ( addr == 0 )
				return;
			uint32_t typeHdl( getHandle( inType ) );
			uint32_t fileHdl( getHandle( inFile ) );
			AllocationEvent theEvent;
			theEvent.init( inSize, typeHdl, fileHdl, inLine, addr );
			sendEvent( theEvent );
		}

		void onDeallocation( uint64_t addr )
		{
			if ( addr == 0 )
				return;
			DeallocationEvent theEvent;
			theEvent.init( addr );
			sendEvent( theEvent );
		}

		void flushProfileEvents()
		{
			TBaseType::flushEvents();
		}

	protected:
		
		template<typename TDataType>
		void sendEvent( TDataType inType )
		{
			MemoryEventHeader theHeader( getMemoryEventType<TDataType>() );
			inType.setup( theHeader );
			theHeader.streamify( TBaseType::mSerializer );
			inType.streamify( TBaseType::mSerializer, theHeader );
			if ( TBaseType::mDataArray.size() >= TBaseType::mBufferFullAmount )
				flushProfileEvents();
		}
	};

	class PxProfileMemoryEventBufferImpl : public shdfnd::UserAllocated
		, public PxProfileMemoryEventBuffer
	{
		typedef MemoryEventBuffer<PxProfileEventMutex, NullLock> TMemoryBufferType;
		TMemoryBufferType mBuffer;

	public:
		PxProfileMemoryEventBufferImpl( PxAllocatorCallback& alloc, uint32_t inBufferFullAmount )
			: mBuffer( alloc, inBufferFullAmount, NULL )
		{
		}

		virtual void onAllocation( size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory )
		{
			mBuffer.onAllocation( size, typeName, filename, uint32_t(line), static_cast<uint64_t>(reinterpret_cast<size_t>(allocatedMemory)) );
		}
		virtual void onDeallocation( void* allocatedMemory )
		{
			mBuffer.onDeallocation(static_cast<uint64_t>(reinterpret_cast<size_t>(allocatedMemory)) );
		}
		
		virtual void addClient( PxProfileEventBufferClient& inClient ) { mBuffer.addClient( inClient ); }
		virtual void removeClient( PxProfileEventBufferClient& inClient ) { mBuffer.removeClient( inClient ); }
		virtual bool hasClients() const { return mBuffer.hasClients(); }

		virtual void flushProfileEvents() { mBuffer.flushProfileEvents(); }

		virtual void release(){ PX_PROFILE_DELETE( mBuffer.getWrapper().getAllocator(), this ); }
	};
}}

#endif // PXPVDSDK_PXPROFILEMEMORYEVENTBUFFER_H
