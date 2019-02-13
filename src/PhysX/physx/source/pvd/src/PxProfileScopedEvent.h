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

#ifndef PXPVDSDK_PXPROFILESCOPEDEVENT_H
#define PXPVDSDK_PXPROFILESCOPEDEVENT_H

#include "PxProfileEventId.h"
#include "PxProfileCompileTimeEventFilter.h"

namespace physx { namespace profile {

	/**
	\brief Template version of startEvent, called directly on provided profile buffer.

	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<bool TEnabled, typename TBufferType>
	inline void startEvent( TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( TEnabled && inBuffer ) inBuffer->startEvent( inId, inContext );
	}

	/**
	\brief Template version of stopEvent, called directly on provided profile buffer.

	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<bool TEnabled, typename TBufferType>
	inline void stopEvent( TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( TEnabled && inBuffer ) inBuffer->stopEvent( inId, inContext );
	}
	
	/**
	\brief Template version of startEvent, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<typename TBufferType>
	inline void startEvent( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( inEnabled && inBuffer ) inBuffer->startEvent( inId, inContext );
	}

	/**
	\brief Template version of stopEvent, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	*/
	template<typename TBufferType>
	inline void stopEvent( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext )
	{
		if ( inEnabled && inBuffer ) inBuffer->stopEvent( inId, inContext );
	}
	
	/**
	\brief Template version of eventValue, called directly on provided profile buffer.

	\param inEnabled If profile event is enabled.
	\param inBuffer Profile event buffer.
	\param inId Profile event id.
	\param inContext Profile event context.
	\param inValue Event value.
	*/
	template<typename TBufferType>
	inline void eventValue( bool inEnabled, TBufferType* inBuffer, const PxProfileEventId& inId, uint64_t inContext, int64_t inValue )
	{
		if ( inEnabled && inBuffer ) inBuffer->eventValue( inId, inContext, inValue );
	}

}}

#endif // PXPVDSDK_PXPROFILESCOPEDEVENT_H
