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

#ifndef PXPVDSDK_PXPROFILEEVENTSENDER_H
#define PXPVDSDK_PXPROFILEEVENTSENDER_H

#include "foundation/Px.h"

namespace physx { namespace profile {

	/**
	\brief Tagging interface to indicate an object that is capable of flushing a profile
	event stream at a certain point.
	 */
	class PxProfileEventFlusher
	{
	protected:
		virtual ~PxProfileEventFlusher(){}
	public:
		/**
		\brief Flush profile events. Sends the profile event buffer to hooked clients.
		*/
		virtual void flushProfileEvents() = 0;
	};

	/**
	\brief Sends the full events where the caller must provide the context and thread id.
	 */
	class PxProfileEventSender
	{
	protected:
		virtual ~PxProfileEventSender(){}
	public:
	
		/**
		\brief Use this as a thread id for events that start on one thread and end on another
		*/
		static const uint32_t CrossThreadId = 99999789;

		/**
		\brief Send a start profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		*/
		virtual void startEvent( uint16_t inId, uint64_t contextId) = 0;
		/**
		\brief Send a stop profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		*/
		virtual void stopEvent( uint16_t inId, uint64_t contextId) = 0;

		/**
		\brief Send a start profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		\param threadId Thread id.
		*/
		virtual void startEvent( uint16_t inId, uint64_t contextId, uint32_t threadId) = 0;
		/**
		\brief Send a stop profile event, optionally with a context. Events are sorted by thread
		and context in the client side.
		\param inId Profile event id.
		\param contextId Context id.
		\param threadId Thread id.
		*/
		virtual void stopEvent( uint16_t inId, uint64_t contextId, uint32_t threadId ) = 0;

		virtual void atEvent(uint16_t inId, uint64_t contextId, uint32_t threadId, uint64_t start, uint64_t stop) = 0;

		/**
		\brief Set an specific events value. This is different than the profiling value
		for the event; it is a value recorded and kept around without a timestamp associated
		with it. This value is displayed when the event itself is processed.
		\param inId Profile event id.
		\param contextId Context id.
		\param inValue Value to set for the event.
		 */
		virtual void eventValue( uint16_t inId, uint64_t contextId, int64_t inValue ) = 0;
	};

} }

#endif // PXPVDSDK_PXPROFILEEVENTSENDER_H
