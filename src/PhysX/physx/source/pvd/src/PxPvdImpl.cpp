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

#include "PxPvdImpl.h"
#include "PxPvdMemClient.h"
#include "PxPvdProfileZoneClient.h"
#include "PxPvdProfileZone.h"

#include "PsFoundation.h"

#if PX_NVTX
#include "nvToolsExt.h"
#endif

namespace
{
	const char* gSdkName = "PhysXSDK";
}

namespace physx
{
namespace pvdsdk
{

class CmEventNameProvider : public physx::profile::PxProfileNameProvider
{
public:
	physx::profile::PxProfileNames getProfileNames() const
	{
		physx::profile::PxProfileNames  ret;
		ret.eventCount = 0;
		return ret;
	}
};

CmEventNameProvider gProfileNameProvider;

void initializeModelTypes(PvdDataStream& stream)
{
	stream.createClass<profile::PxProfileZone>();
	stream.createProperty<profile::PxProfileZone, uint8_t>(
	    "events", PvdCommStreamEmbeddedTypes::getProfileEventStreamSemantic(), PropertyType::Array);

	stream.createClass<profile::PxProfileMemoryEventBuffer>();
	stream.createProperty<profile::PxProfileMemoryEventBuffer, uint8_t>(
	    "events", PvdCommStreamEmbeddedTypes::getMemoryEventStreamSemantic(), PropertyType::Array);

	stream.createClass<PvdUserRenderer>();
	stream.createProperty<PvdUserRenderer, uint8_t>(
	    "events", PvdCommStreamEmbeddedTypes::getRendererEventStreamSemantic(), PropertyType::Array);
}

PvdImpl* PvdImpl::sInstance = NULL;
uint32_t PvdImpl::sRefCount = 0;

PvdImpl::PvdImpl()
: mPvdTransport(NULL)
, mSharedMetaProvider(NULL)
, mMemClient(NULL)
, mIsConnected(false)
, mIsNVTXSupportEnabled(true)
, mNVTXContext(0)
, mNextStreamId(1)
, mProfileClient(NULL)
, mProfileZone(NULL)
{
	mProfileZoneManager = &physx::profile::PxProfileZoneManager::createProfileZoneManager(&physx::shdfnd::getAllocator());
	mProfileClient = PVD_NEW(PvdProfileZoneClient)(*this);
}

PvdImpl::~PvdImpl()
{
	if((mFlags & PxPvdInstrumentationFlag::ePROFILE) )
	{
		PxSetProfilerCallback(NULL);
	}

	disconnect();

	if ( mProfileZoneManager )
	{
		mProfileZoneManager->release();
		mProfileZoneManager = NULL;
	}

	PVD_DELETE(mProfileClient);
	mProfileClient = NULL;
}

bool PvdImpl::connect(PxPvdTransport& transport, PxPvdInstrumentationFlags flags)
{
	if(mIsConnected)
	{
		physx::shdfnd::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxPvd::connect - recall connect! Should call disconnect before re-connect.");
	    return false;
	}

	mFlags = flags;	
	mPvdTransport = &transport;

	mIsConnected = mPvdTransport->connect();

	if(mIsConnected)
	{
		mSharedMetaProvider = PVD_NEW(MetaDataProvider);
		sendTransportInitialization();

		PvdDataStream* stream = PvdDataStream::create(this);
		initializeModelTypes(*stream);
		stream->release();

		if(mFlags & PxPvdInstrumentationFlag::eMEMORY)
		{
			mMemClient = PVD_NEW(PvdMemClient)(*this);
			mPvdClients.pushBack(mMemClient);
		}

		if((mFlags & PxPvdInstrumentationFlag::ePROFILE) && mProfileZoneManager)
		{			
			mPvdClients.pushBack(mProfileClient);
			mProfileZone = &physx::profile::PxProfileZone::createProfileZone(&physx::shdfnd::getAllocator(),gSdkName,gProfileNameProvider.getProfileNames());
		}

		for(uint32_t i = 0; i < mPvdClients.size(); i++)
			mPvdClients[i]->onPvdConnected();

		if (mProfileZone)
		{
			mProfileZoneManager->addProfileZoneHandler(*mProfileClient);
			mProfileZoneManager->addProfileZone( *mProfileZone );
		}

		if ((mFlags & PxPvdInstrumentationFlag::ePROFILE))
		{
			PxSetProfilerCallback(this);
		}
	}
	return mIsConnected;
}

void PvdImpl::disconnect()
{
	if(mProfileZone)
	{
		mProfileZoneManager->removeProfileZoneHandler(*mProfileClient);		
		mProfileZoneManager->removeProfileZone( *mProfileZone );				
		mProfileZone->release();
		mProfileZone=NULL;	
		removeClient(mProfileClient);
	}

	if(mIsConnected)
	{
		for(uint32_t i = 0; i < mPvdClients.size(); i++)
			mPvdClients[i]->onPvdDisconnected();		

		if(mMemClient)
		{
			removeClient(mMemClient);
			PvdMemClient* tmp = mMemClient;  //avoid tracking deallocation itsself
			mMemClient = NULL;
			PVD_DELETE(tmp);	        
		}
		 
		mSharedMetaProvider->release();
		mPvdTransport->disconnect();
		mObjectRegistrar.clear();
		mIsConnected = false;
	}
}

void PvdImpl::flush()
{
	for(uint32_t i = 0; i < mPvdClients.size(); i++)
		mPvdClients[i]->flush();
	if ( mProfileZone )
	{
		mProfileZone->flushEventIdNameMap();
		mProfileZone->flushProfileEvents();
	}
}

bool PvdImpl::isConnected(bool useCachedStatus)
{
	if(mPvdTransport)
	    return useCachedStatus ? mIsConnected : mPvdTransport->isConnected();
	else
		return false;
}

PxPvdTransport* PvdImpl::getTransport()
{
	return mPvdTransport;
}

PxPvdInstrumentationFlags PvdImpl::getInstrumentationFlags()
{
	return mFlags;
}

void PvdImpl::sendTransportInitialization()
{
	StreamInitialization init;
	EventStreamifier<PxPvdTransport> stream(mPvdTransport->lock());
	init.serialize(stream);
	mPvdTransport->unlock();
}

void PvdImpl::addClient(PvdClient* client)
{
	PX_ASSERT(client);
	for(uint32_t i = 0; i < mPvdClients.size(); i++)
	{
		if(client == mPvdClients[i])
		    return;
	}
	mPvdClients.pushBack(client);
	if(mIsConnected)
	{
		client->onPvdConnected();
	}
}

void PvdImpl::removeClient(PvdClient* client)
{
	for(uint32_t i = 0; i < mPvdClients.size(); i++)
	{
		if(client == mPvdClients[i])
		{
			client->onPvdDisconnected();
			mPvdClients.remove(i);
		}
	}
}

void PvdImpl::onAllocation(size_t inSize, const char* inType, const char* inFile, int inLine, void* inAddr)
{
	if(mMemClient)
       mMemClient->onAllocation(inSize, inType, inFile, inLine, inAddr);
}

void PvdImpl::onDeallocation(void* inAddr)
{
	if(mMemClient)
       mMemClient->onDeallocation(inAddr);
}

PvdOMMetaDataProvider& PvdImpl::getMetaDataProvider()
{
	return *mSharedMetaProvider;
}

bool PvdImpl::registerObject(const void* inItem)
{
	return mObjectRegistrar.addItem(inItem);
}


bool PvdImpl::unRegisterObject(const void* inItem)
{
	return mObjectRegistrar.decItem(inItem);
}

uint64_t PvdImpl::getNextStreamId()
{
	uint64_t retval = ++mNextStreamId;
	return retval;
}

bool PvdImpl::initialize()
{
	if(0 == sRefCount)
	{
		sInstance = PVD_NEW(PvdImpl)();
	}
	++sRefCount;
	return !!sInstance;
}

void PvdImpl::release()
{
	if(sRefCount > 0)
	{
		if(--sRefCount)
			return;

		PVD_DELETE(sInstance);
		sInstance = NULL;
	}
}

PvdImpl* PvdImpl::getInstance()
{
	return sInstance;
}


/**************************************************************************************************************************
Instrumented profiling events
***************************************************************************************************************************/

static const uint32_t CrossThreadId = 99999789;

void* PvdImpl::zoneStart(const char* eventName, bool detached, uint64_t contextId)
{
	if(mProfileZone)
	{
		const uint16_t id = mProfileZone->getEventIdForName(eventName);
		if(detached)
			mProfileZone->startEvent(id, contextId, CrossThreadId);
		else
			mProfileZone->startEvent(id, contextId);
	}
#if PX_NVTX
	if(mIsNVTXSupportEnabled)
	{ 
		if(detached)
		{
			// TODO : Need to use the nvtxRangeStart API for cross thread events
			nvtxEventAttributes_t eventAttrib;
			memset(&eventAttrib, 0, sizeof(eventAttrib));
			eventAttrib.version = NVTX_VERSION;
			eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
			eventAttrib.colorType = NVTX_COLOR_ARGB;
			eventAttrib.color = 0xFF00FF00;
			eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
			eventAttrib.message.ascii = eventName;
			nvtxMarkEx(&eventAttrib);
		}
		else
		{
			nvtxRangePush(eventName);
		}
	}
#endif
	return NULL;
}

void PvdImpl::zoneEnd(void* /*profilerData*/, const char* eventName, bool detached, uint64_t contextId)
{
	if(mProfileZone)
	{
		const uint16_t id = mProfileZone->getEventIdForName(eventName);
		if(detached)
			mProfileZone->stopEvent(id, contextId, CrossThreadId);
		else
			mProfileZone->stopEvent(id, contextId);
	}
#if PX_NVTX
	if(mIsNVTXSupportEnabled)
	{
		if(detached)
		{
			nvtxEventAttributes_t eventAttrib;
			memset(&eventAttrib, 0, sizeof(eventAttrib));
			eventAttrib.version = NVTX_VERSION;
			eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
			eventAttrib.colorType = NVTX_COLOR_ARGB;
			eventAttrib.color = 0xFFFF0000;
			eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
			eventAttrib.message.ascii = eventName;
			nvtxMarkEx(&eventAttrib);
		}
		else
		{
			nvtxRangePop();
		}
	}
#endif
}
} // pvd

} // physx
