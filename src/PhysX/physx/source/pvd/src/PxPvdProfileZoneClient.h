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

#ifndef PXPVDSDK_PXPVDPROFILEZONECLIENT_H
#define PXPVDSDK_PXPVDPROFILEZONECLIENT_H
#include "PxPvdClient.h"
#include "PsHashMap.h"
#include "PsMutex.h"
#include "PxProfileZoneManager.h"

namespace physx
{
namespace pvdsdk
{
class PvdImpl;
class PvdDataStream;

struct ProfileZoneClient;

class PvdProfileZoneClient : public PvdClient, public profile::PxProfileZoneHandler, public shdfnd::UserAllocated
{
	PX_NOCOPY(PvdProfileZoneClient)
  public:
	PvdProfileZoneClient(PvdImpl& pvd);
	virtual ~PvdProfileZoneClient();

	bool isConnected() const;
	void onPvdConnected();
	void onPvdDisconnected();
	void flush();

	PvdDataStream* getDataStream();
	PvdUserRenderer* getUserRender();
	void setObjectRegistrar(ObjectRegistrar*);

	// PxProfileZoneHandler
	void onZoneAdded(profile::PxProfileZone& inSDK);
	void onZoneRemoved(profile::PxProfileZone& inSDK);

  private:
	shdfnd::Mutex mMutex; // zoneAdded can called from different threads
	PvdImpl& mSDKPvd;
	PvdDataStream* mPvdDataStream;	
	physx::shdfnd::Array<ProfileZoneClient*> mProfileZoneClients;
	bool mIsConnected;
};

} // namespace pvdsdk
} // namespace physx

#endif // PXPVDSDK_PXPVDPROFILEZONECLIENT_H
