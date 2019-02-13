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

#ifndef PXPVDSDK_PXPVDDEFAULTFILETRANSPORT_H
#define PXPVDSDK_PXPVDDEFAULTFILETRANSPORT_H

#include "pvd/PxPvdTransport.h"

#include "PsUserAllocated.h"
#include "PsFileBuffer.h"
#include "PsMutex.h"

namespace physx
{
namespace pvdsdk
{

class PvdDefaultFileTransport : public physx::PxPvdTransport, public physx::shdfnd::UserAllocated
{
	PX_NOCOPY(PvdDefaultFileTransport)
  public:
	PvdDefaultFileTransport(const char* name);
	virtual ~PvdDefaultFileTransport();

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
	physx::PsFileBuffer* mFileBuffer;
	bool mConnected;
	uint64_t mWrittenData;
	physx::shdfnd::Mutex mMutex;
	bool mLocked; // for debug, remove it when finished
};

} // pvdsdk
} // physx

#endif // PXPVDSDK_PXPVDDEFAULTFILETRANSPORT_H
