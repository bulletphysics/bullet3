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

#include "PxPvdDefaultFileTransport.h"

namespace physx
{
namespace pvdsdk
{

PvdDefaultFileTransport::PvdDefaultFileTransport(const char* name) : mConnected(false), mWrittenData(0), mLocked(false)
{
	mFileBuffer = PX_NEW(PsFileBuffer)(name, PxFileBuf::OPEN_WRITE_ONLY);
}

PvdDefaultFileTransport::~PvdDefaultFileTransport()
{
}

bool PvdDefaultFileTransport::connect()
{
	PX_ASSERT(mFileBuffer);
	mConnected = mFileBuffer->isOpen();
	return mConnected;
}

void PvdDefaultFileTransport::disconnect()
{
	mConnected = false;
}

bool PvdDefaultFileTransport::isConnected()
{
	return mConnected;
}

bool PvdDefaultFileTransport::write(const uint8_t* inBytes, uint32_t inLength)
{
	PX_ASSERT(mLocked);
	PX_ASSERT(mFileBuffer);
	if (mConnected)
	{
		uint32_t len = mFileBuffer->write(inBytes, inLength);
		mWrittenData += len;
		return len == inLength;
	}
	else
		return false;
}

PxPvdTransport& PvdDefaultFileTransport::lock()
{
	mMutex.lock();
	PX_ASSERT(!mLocked);
	mLocked = true;
	return *this;
}

void PvdDefaultFileTransport::unlock()
{
	PX_ASSERT(mLocked);
	mLocked = false;
	mMutex.unlock();
}

void PvdDefaultFileTransport::flush()
{
}

uint64_t PvdDefaultFileTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void PvdDefaultFileTransport::release()
{
	if (mFileBuffer)
	{
		mFileBuffer->close();
		delete mFileBuffer;
	}
	mFileBuffer = NULL;
	PX_DELETE(this);
}

class NullFileTransport : public physx::PxPvdTransport, public physx::shdfnd::UserAllocated
{
	PX_NOCOPY(NullFileTransport)
  public:
	NullFileTransport();
	virtual ~NullFileTransport();

	virtual bool connect();
	virtual void disconnect();
	virtual bool isConnected();

	virtual bool write(const uint8_t* inBytes, uint32_t inLength);

	virtual PxPvdTransport& lock();
	virtual void unlock();

	virtual void flush();

	virtual uint64_t getWrittenDataSize();

	virtual void release();

  private:
	bool mConnected;
	uint64_t mWrittenData;
	physx::shdfnd::Mutex mMutex;
	bool mLocked; // for debug, remove it when finished
};

NullFileTransport::NullFileTransport() : mConnected(false), mWrittenData(0), mLocked(false)
{
}

NullFileTransport::~NullFileTransport()
{
}

bool NullFileTransport::connect()
{
	mConnected = true;
	return true;
}

void NullFileTransport::disconnect()
{
	mConnected = false;
}

bool NullFileTransport::isConnected()
{
	return mConnected;
}

bool NullFileTransport::write(const uint8_t* /*inBytes*/, uint32_t inLength)
{
	PX_ASSERT(mLocked);
	if(mConnected)
	{
		uint32_t len = inLength;
		mWrittenData += len;
		return len == inLength;
	}
	else
		return false;
}

PxPvdTransport& NullFileTransport::lock()
{
	mMutex.lock();
	PX_ASSERT(!mLocked);
	mLocked = true;
	return *this;
}

void NullFileTransport::unlock()
{
	PX_ASSERT(mLocked);
	mLocked = false;
	mMutex.unlock();
}

void NullFileTransport::flush()
{
}

uint64_t NullFileTransport::getWrittenDataSize()
{
	return mWrittenData;
}

void NullFileTransport::release()
{
	PX_DELETE(this);
}

} // namespace pvdsdk

PxPvdTransport* PxDefaultPvdFileTransportCreate(const char* name)
{
	if(name)
		return PX_NEW(pvdsdk::PvdDefaultFileTransport)(name);
	else
		return PX_NEW(pvdsdk::NullFileTransport)();
}

} // namespace physx

