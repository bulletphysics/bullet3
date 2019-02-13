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

#ifndef PXPVDSDK_PSPVD_H
#define PXPVDSDK_PSPVD_H

/** \addtogroup pvd
@{
*/
#include "pvd/PxPvd.h"
#include "PsBroadcast.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvdTransport;

#if !PX_DOXYGEN
namespace pvdsdk
{
#endif

class PvdDataStream;
class PvdClient;
class PvdOMMetaDataProvider;

// PsPvd is used for advanced user, it support custom pvd client API
class PsPvd : public physx::PxPvd, public shdfnd::AllocationListener
{
  public:
	virtual void addClient(PvdClient* client) = 0;
	virtual void removeClient(PvdClient* client) = 0;
	
	virtual bool registerObject(const void* inItem) = 0;
	virtual bool unRegisterObject(const void* inItem) = 0;

	//AllocationListener
	void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory) = 0;
	void onDeallocation(void* addr) = 0;

	virtual PvdOMMetaDataProvider& getMetaDataProvider() = 0;
	
	virtual uint64_t getNextStreamId() = 0;
	// Call to flush events to PVD
	virtual void flush() = 0;

};

#if !PX_DOXYGEN
} // namespace pvdsdk
} // namespace physx
#endif

/** @} */
#endif // PXPVDSDK_PSPVD_H
