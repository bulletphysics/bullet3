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

#ifndef PXPVDSDK_PXPROFILEEVENTNAMES_H
#define PXPVDSDK_PXPROFILEEVENTNAMES_H

#include "PxProfileEventId.h"

namespace physx { namespace profile {

	/**
	\brief Mapping from event id to name.
	*/
	struct PxProfileEventName
	{
		const char*					name;
		PxProfileEventId			eventId;

		/**
		\brief Default constructor.
		\param inName Profile event name.
		\param inId Profile event id.
		*/
		PxProfileEventName( const char* inName, PxProfileEventId inId ) : name( inName ), eventId( inId ) {}
	};

	/**
	\brief Aggregator of event id -> name mappings
	*/
	struct PxProfileNames
	{
		/**
		\brief Default constructor that doesn't point to any names.
		\param inEventCount Number of provided events.
		\param inSubsystems Event names array.
		*/
		PxProfileNames( uint32_t inEventCount = 0, const PxProfileEventName* inSubsystems = NULL )
			: eventCount( inEventCount )
			, events( inSubsystems )
		{
		}

		uint32_t							eventCount;
		const PxProfileEventName*			events;
	};

	/**
	\brief Provides a mapping from event ID -> name.
	*/
	class PxProfileNameProvider
	{
	public:
		/**
		\brief Returns profile event names.
		\return Profile event names.
		*/
		virtual PxProfileNames getProfileNames() const = 0;

	protected:
		virtual ~PxProfileNameProvider(){}
		PxProfileNameProvider& operator=(const PxProfileNameProvider&) { return *this; }
	};
} }

#endif // PXPVDSDK_PXPROFILEEVENTNAMES_H
