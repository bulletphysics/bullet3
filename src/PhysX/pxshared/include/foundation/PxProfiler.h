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

#ifndef PXFOUNDATION_PXPROFILER_H
#define PXFOUNDATION_PXPROFILER_H

#include "foundation/Px.h"

namespace physx
{

/**
\brief The pure virtual callback interface for general purpose instrumentation and profiling of GameWorks modules as
well as applications
*/
class PxProfilerCallback
{
protected:
	virtual ~PxProfilerCallback()	{}

public:
	/**************************************************************************************************************************
	Instrumented profiling events
	***************************************************************************************************************************/

	/**
	\brief Mark the beginning of a nested profile block
	\param[in] eventName	Event name. Must be a persistent const char *
	\param[in] detached		True for cross thread events
	\param[in] contextId	the context id of this zone. Zones with the same id belong to the same group. 0 is used for no specific group.
	\return Returns implementation-specific profiler data for this event
	*/
	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) = 0;

	/**
	\brief Mark the end of a nested profile block
	\param[in] profilerData	The data returned by the corresponding zoneStart call (or NULL if not available)
	\param[in] eventName	The name of the zone ending, must match the corresponding name passed with 'zoneStart'. Must be a persistent const char *.
	\param[in] detached		True for cross thread events. Should match the value passed to zoneStart.
	\param[in] contextId	The context of this zone. Should match the value passed to zoneStart.

	\note eventName plus contextId can be used to uniquely match up start and end of a zone.
	*/
	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) = 0;
};

class PxProfileScoped
{
  public:
	PX_FORCE_INLINE PxProfileScoped(PxProfilerCallback* callback, const char* eventName, bool detached, uint64_t contextId) : mCallback(callback), mProfilerData(NULL)
	{
		if(mCallback)
		{
			mEventName		= eventName;
			mContextId		= contextId;
			mDetached		= detached;
			mProfilerData	= mCallback->zoneStart(eventName, detached, contextId);
		}
	}

	PX_FORCE_INLINE ~PxProfileScoped()
	{
		if(mCallback)
			mCallback->zoneEnd(mProfilerData, mEventName, mDetached, mContextId);
	}
	PxProfilerCallback*			mCallback;
	const char*					mEventName;
	void*						mProfilerData;
	uint64_t					mContextId;
	bool						mDetached;
};

} // end of physx namespace

#endif // PXFOUNDATION_PXPROFILER_H
