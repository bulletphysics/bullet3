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

#ifndef PXPVDSDK_PXPROFILEEVENTID_H
#define PXPVDSDK_PXPROFILEEVENTID_H

#include "foundation/Px.h"

namespace physx { namespace profile {
	/**
	\brief A event id structure. Optionally includes information about
	if the event was enabled at compile time.
	*/
	struct PxProfileEventId
	{
		uint16_t		eventId;
		mutable bool	compileTimeEnabled; 

		/**
		\brief Profile event id constructor.
		\param inId Profile event id.
		\param inCompileTimeEnabled Compile time enabled.
		*/
		PxProfileEventId( uint16_t inId = 0, bool inCompileTimeEnabled = true )
			: eventId( inId )
			, compileTimeEnabled( inCompileTimeEnabled )
		{
		}

		operator uint16_t () const { return eventId; }

		bool operator==( const PxProfileEventId& inOther ) const 
		{ 
			return eventId == inOther.eventId;
		}
	};

} }

#endif // PXPVDSDK_PXPROFILEEVENTID_H
