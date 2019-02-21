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

namespace physx
{
namespace pvdsdk
{

PvdMemClient::PvdMemClient(PvdImpl& pvd)
: mSDKPvd(pvd)
, mPvdDataStream(NULL)
, mIsConnected(false)
, mMemEventBuffer(profile::PxProfileMemoryEventBuffer::createMemoryEventBuffer(*gPvdAllocatorCallback))
{
}

PvdMemClient::~PvdMemClient()
{
	mSDKPvd.removeClient(this);
	if(mMemEventBuffer.hasClients())
		mPvdDataStream->destroyInstance(&mMemEventBuffer);
	mMemEventBuffer.release();
}

PvdDataStream* PvdMemClient::getDataStream()
{
	return mPvdDataStream;
}

PvdUserRenderer* PvdMemClient::getUserRender()
{
	PX_ASSERT(0);
	return NULL;
}

void PvdMemClient::setObjectRegistrar(ObjectRegistrar*)
{
}

bool PvdMemClient::isConnected() const
{
	return mIsConnected;
}

void PvdMemClient::onPvdConnected()
{
	if(mIsConnected)
		return;
	mIsConnected = true;

	mPvdDataStream = PvdDataStream::create(&mSDKPvd);
	mPvdDataStream->createInstance(&mMemEventBuffer);
	mMemEventBuffer.addClient(*this);
}

void PvdMemClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;
	mIsConnected = false;

	flush();

	mMemEventBuffer.removeClient(*this);
	mPvdDataStream->release();
	mPvdDataStream = NULL;
}

void PvdMemClient::onAllocation(size_t inSize, const char* inType, const char* inFile, int inLine, void* inAddr)
{
	mMutex.lock();
	mMemEventBuffer.onAllocation(inSize, inType, inFile, inLine, inAddr);
	mMutex.unlock();
}

void PvdMemClient::onDeallocation(void* inAddr)
{
	mMutex.lock();
	mMemEventBuffer.onDeallocation(inAddr);
	mMutex.unlock();
}

void PvdMemClient::flush()
{
	mMutex.lock();
	mMemEventBuffer.flushProfileEvents();
	mMutex.unlock();
}

void PvdMemClient::handleBufferFlush(const uint8_t* inData, uint32_t inLength)
{
	if(mPvdDataStream)
	    mPvdDataStream->setPropertyValue(&mMemEventBuffer, "events", inData, inLength);
}

void PvdMemClient::handleClientRemoved()
{
}

} // pvd
} // physx
