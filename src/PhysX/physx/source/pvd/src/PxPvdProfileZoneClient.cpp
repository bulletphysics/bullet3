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
#include "PxPvdProfileZoneClient.h"
#include "PxPvdProfileZone.h"

namespace physx
{
namespace pvdsdk
{
struct ProfileZoneClient : public profile::PxProfileZoneClient, public shdfnd::UserAllocated
{
	profile::PxProfileZone& mZone;
	PvdDataStream& mStream;

	ProfileZoneClient(profile::PxProfileZone& zone, PvdDataStream& stream) : mZone(zone), mStream(stream)
	{
	}

	~ProfileZoneClient()
	{
		mZone.removeClient(*this);
	}

	virtual void createInstance()
	{
		mStream.addProfileZone(&mZone, mZone.getName());
		mStream.createInstance(&mZone);
		mZone.addClient(*this);
		profile::PxProfileNames names(mZone.getProfileNames());
		PVD_FOREACH(idx, names.eventCount)
		{
			handleEventAdded(names.events[idx]);
		}
	}

	virtual void handleEventAdded(const profile::PxProfileEventName& inName)
	{
		mStream.addProfileZoneEvent(&mZone, inName.name, inName.eventId.eventId, inName.eventId.compileTimeEnabled);
	}

	virtual void handleBufferFlush(const uint8_t* inData, uint32_t inLength)
	{
		mStream.setPropertyValue(&mZone, "events", inData, inLength);
	}

	virtual void handleClientRemoved()
	{
		mStream.destroyInstance(&mZone);
	}

  private:
	ProfileZoneClient& operator=(const ProfileZoneClient&);
};
}
}

using namespace physx;
using namespace pvdsdk;

PvdProfileZoneClient::PvdProfileZoneClient(PvdImpl& pvd) : mSDKPvd(pvd), mPvdDataStream(NULL), mIsConnected(false)
{
}

PvdProfileZoneClient::~PvdProfileZoneClient()
{
	mSDKPvd.removeClient(this);
	// all zones should removed
	PX_ASSERT(mProfileZoneClients.size() == 0);
}

PvdDataStream* PvdProfileZoneClient::getDataStream()
{
	return mPvdDataStream;
}

PvdUserRenderer* PvdProfileZoneClient::getUserRender()
{
	PX_ASSERT(0);
	return NULL;
}

void PvdProfileZoneClient::setObjectRegistrar(ObjectRegistrar*)
{
}

bool PvdProfileZoneClient::isConnected() const
{
	return mIsConnected;
}

void PvdProfileZoneClient::onPvdConnected()
{
	if(mIsConnected)
		return;
	mIsConnected = true;

	mPvdDataStream = PvdDataStream::create(&mSDKPvd);

}

void PvdProfileZoneClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;

	mIsConnected = false;
	flush();

	mPvdDataStream->release();
	mPvdDataStream = NULL;
}

void PvdProfileZoneClient::flush()
{
	PVD_FOREACH(idx, mProfileZoneClients.size())
	mProfileZoneClients[idx]->mZone.flushProfileEvents();
}

void PvdProfileZoneClient::onZoneAdded(profile::PxProfileZone& zone)
{
	PX_ASSERT(mIsConnected);
	ProfileZoneClient* client = PVD_NEW(ProfileZoneClient)(zone, *mPvdDataStream);
	mMutex.lock();
	client->createInstance();
	mProfileZoneClients.pushBack(client);
	mMutex.unlock();
}

void PvdProfileZoneClient::onZoneRemoved(profile::PxProfileZone& zone)
{
	for(uint32_t i = 0; i < mProfileZoneClients.size(); i++)
	{
		if(&zone == &mProfileZoneClients[i]->mZone)
		{
			mMutex.lock();
			ProfileZoneClient* client = mProfileZoneClients[i];
			mProfileZoneClients.replaceWithLast(i);
			PVD_DELETE(client);
			mMutex.unlock();
			return;
		}
	}
}
