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

#include "PxPvdDefaultSocketTransport.h"

namespace physx
{
namespace pvdsdk
{
PvdDefaultSocketTransport::PvdDefaultSocketTransport(const char* host, int port, unsigned int timeoutInMilliseconds)
: mHost(host), mPort(uint16_t(port)), mTimeout(timeoutInMilliseconds), mConnected(false), mWrittenData(0)
{
}

PvdDefaultSocketTransport::~PvdDefaultSocketTransport()
{
}

bool PvdDefaultSocketTransport::connect()
{
	if(mConnected)
		return true;

	if(mSocket.connect(mHost, mPort, mTimeout))
	{
		mSocket.setBlocking(true);
		mConnected = true;
	}
	return mConnected;
}

void PvdDefaultSocketTransport::disconnect()
{
	mSocket.flush();
	mSocket.disconnect();
	mConnected = false;
}

bool PvdDefaultSocketTransport::isConnected()
{
	return mSocket.isConnected();
}

bool PvdDefaultSocketTransport::write(const uint8_t* inBytes, uint32_t inLength)
{
	if(mConnected)
	{
		if(inLength == 0)
			return true;

		uint32_t amountWritten = 0;
		uint32_t totalWritten = 0;
		do
		{
			// Sockets don't have to write as much as requested, so we need
			// to wrap this call in a do/while loop.
			// If they don't write any bytes then we consider them disconnected.
			amountWritten = mSocket.write(inBytes, inLength);
			inLength -= amountWritten;
			inBytes += amountWritten;
			totalWritten += amountWritten;
		} while(inLength && amountWritten);

		if(amountWritten == 0)
			return false;

		mWrittenData += totalWritten;

		return true;
	}
	else
		return false;
}

PxPvdTransport& PvdDefaultSocketTransport::lock()
{
	mMutex.lock();
	return *this;
}

void PvdDefaultSocketTransport::unlock()
{
	mMutex.unlock();
}

void PvdDefaultSocketTransport::flush()
{
	mSocket.flush();
}

uint64_t PvdDefaultSocketTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void PvdDefaultSocketTransport::release()
{
	PX_DELETE(this);
}

} // namespace pvdsdk

PxPvdTransport* PxDefaultPvdSocketTransportCreate(const char* host, int port, unsigned int timeoutInMilliseconds)
{
	return PX_NEW(pvdsdk::PvdDefaultSocketTransport)(host, port, timeoutInMilliseconds);
}

} // namespace physx
