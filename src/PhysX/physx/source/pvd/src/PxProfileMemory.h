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


#ifndef PXPVDSDK_PXPROFILEMEMORY_H
#define PXPVDSDK_PXPROFILEMEMORY_H

#include "PxProfileEventBufferClientManager.h"
#include "PxProfileEventSender.h"
#include "PsBroadcast.h"

namespace physx { namespace profile {

	/**
	\brief Record events so a late-connecting client knows about
	all outstanding allocations
	*/
	class PxProfileMemoryEventRecorder : public shdfnd::AllocationListener
	{
	protected:
		virtual ~PxProfileMemoryEventRecorder(){}
	public:
		/**
		\brief Set the allocation listener
		\param inListener Allocation listener.
		*/
		virtual void setListener(AllocationListener* inListener) = 0;
		/**
		\brief Release the instance.
		*/
		virtual void release() = 0;
	};

	/**
	\brief Stores memory events into the memory buffer. 
	*/
	class PxProfileMemoryEventBuffer
		: public shdfnd::AllocationListener //add a new event to the buffer
		, public PxProfileEventBufferClientManager //add clients to handle the serialized memory events
		, public PxProfileEventFlusher //flush the buffer
	{
	protected:
		virtual ~PxProfileMemoryEventBuffer(){}
	public:

		/**
		\brief Release the instance.
		*/
		virtual void release() = 0;
		
		/**
		\brief Create a non-mutex-protected event buffer.		
		\param inAllocator Allocation callback.
		\param inBufferSize Internal buffer size.
		*/
		static PxProfileMemoryEventBuffer& createMemoryEventBuffer(PxAllocatorCallback& inAllocator, uint32_t inBufferSize = 0x1000);
	};



} } // namespace physx


#endif // PXPVDSDK_PXPROFILEMEMORY_H


