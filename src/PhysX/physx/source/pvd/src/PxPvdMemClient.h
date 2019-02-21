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

#ifndef PXPVDSDK_PXPVDMEMCLIENT_H
#define PXPVDSDK_PXPVDMEMCLIENT_H

#include "PxPvdClient.h"
#include "PsHashMap.h"
#include "PsMutex.h"
#include "PsBroadcast.h"
#include "PxProfileEventBufferClient.h"
#include "PxProfileMemory.h"

namespace physx
{
class PvdDataStream;

namespace pvdsdk
{
class PvdImpl;
class PvdMemClient : public PvdClient,                   
                     public profile::PxProfileEventBufferClient,
                     public shdfnd::UserAllocated
{
	PX_NOCOPY(PvdMemClient)
  public:
	PvdMemClient(PvdImpl& pvd);
	virtual ~PvdMemClient();

	bool isConnected() const;
	void onPvdConnected();
	void onPvdDisconnected();
	void flush();

	PvdDataStream* getDataStream();
	PvdUserRenderer* getUserRender();
	void setObjectRegistrar(ObjectRegistrar*);
	void sendMemEvents();

	// memory event
	void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory);
	void onDeallocation(void* addr);

  private:
	PvdImpl& mSDKPvd;
	PvdDataStream* mPvdDataStream;
	bool mIsConnected;

	// mem profile
	shdfnd::Mutex mMutex; // mem onallocation can called from different threads
	profile::PxProfileMemoryEventBuffer& mMemEventBuffer;
	void handleBufferFlush(const uint8_t* inData, uint32_t inLength);
	void handleClientRemoved();
};

} // namespace pvdsdk
} // namespace physx

#endif // PXPVDSDK_PXPVDMEMCLIENT_H
