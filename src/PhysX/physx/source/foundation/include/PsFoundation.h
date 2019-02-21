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

#ifndef PX_FOUNDATION_PSFOUNDATION_H
#define PX_FOUNDATION_PSFOUNDATION_H

#include "foundation/PxErrors.h"
#include "foundation/PxProfiler.h"

#include "PxFoundation.h"

#include "PsBroadcast.h"
#include "PsAllocator.h"
#include "PsTempAllocator.h"
#include "PsMutex.h"
#include "PsHashMap.h"
#include "PsUserAllocated.h"

#include <stdarg.h>

namespace physx
{
namespace shdfnd
{

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4251) // class needs to have dll-interface to be used by clients of class
#endif

class PX_FOUNDATION_API Foundation : public PxFoundation, public UserAllocated
{
	PX_NOCOPY(Foundation)

  public:
	typedef MutexT<Allocator> Mutex;

	typedef HashMap<const NamedAllocator*, const char*, Hash<const NamedAllocator*>, NonTrackingAllocator> AllocNameMap;
	typedef Array<TempAllocatorChunk*, Allocator> AllocFreeTable;

  public:
	// factory
	// note, you MUST eventually call release if createInstance returned true!
	static Foundation* createInstance(PxU32 version, PxErrorCallback& errc, PxAllocatorCallback& alloc);
	static Foundation& getInstance();
	static void setInstance(Foundation& foundation);
	void release();
	static void incRefCount(); // this call requires a foundation object to exist already
	static void decRefCount(); // this call requires a foundation object to exist already

	// Begin Errors
	virtual PxErrorCallback& getErrorCallback()
	{
		return mErrorCallback;
	} // Return the user's error callback
	PxErrorCallback& getInternalErrorCallback()
	{
		return mBroadcastingError;
	} // Return the broadcasting error callback

	void registerErrorCallback(PxErrorCallback& listener);
	void deregisterErrorCallback(PxErrorCallback& listener);

	virtual void setErrorLevel(PxErrorCode::Enum mask)
	{
		mErrorMask = mask;
	}
	virtual PxErrorCode::Enum getErrorLevel() const
	{
		return mErrorMask;
	}

	void error(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, ...); // Report errors with the
	                                                                                        // broadcasting
	void errorImpl(PxErrorCode::Enum, const char* file, int line, const char* messageFmt, va_list); // error callback
	static PxU32 getWarnOnceTimestamp();

	// End errors

	// Begin Allocations
	virtual PxAllocatorCallback& getAllocatorCallback()
	{
		return mAllocatorCallback;
	} // Return the user's allocator callback
	PxAllocatorCallback& getAllocator()
	{
		return mBroadcastingAllocator;
	} // Return the broadcasting allocator

	void registerAllocationListener(physx::shdfnd::AllocationListener& listener);
	void deregisterAllocationListener(physx::shdfnd::AllocationListener& listener);

	virtual bool getReportAllocationNames() const
	{
		return mReportAllocationNames;
	}
	virtual void setReportAllocationNames(bool value)
	{
		mReportAllocationNames = value;
	}

	PX_INLINE AllocNameMap& getNamedAllocMap()
	{
		return mNamedAllocMap;
	}
	PX_INLINE Mutex& getNamedAllocMutex()
	{
		return mNamedAllocMutex;
	}

	PX_INLINE AllocFreeTable& getTempAllocFreeTable()
	{
		return mTempAllocFreeTable;
	}
	PX_INLINE Mutex& getTempAllocMutex()
	{
		return mTempAllocMutex;
	}
	// End allocations

  private:
	static void destroyInstance();

	Foundation(PxErrorCallback& errc, PxAllocatorCallback& alloc);
	~Foundation();

	// init order is tricky here: the mutexes require the allocator, the allocator may require the error stream
	PxAllocatorCallback& mAllocatorCallback;
	PxErrorCallback& mErrorCallback;

	BroadcastingAllocator mBroadcastingAllocator;
	BroadcastingErrorCallback mBroadcastingError;

	bool mReportAllocationNames;

	PxErrorCode::Enum mErrorMask;
	Mutex mErrorMutex;

	AllocNameMap mNamedAllocMap;
	Mutex mNamedAllocMutex;

	AllocFreeTable mTempAllocFreeTable;
	Mutex mTempAllocMutex;

	Mutex mListenerMutex;

	static Foundation* mInstance;
	static PxU32 mRefCount;
	static PxU32 mWarnOnceTimestap;
};
#if PX_VC
#pragma warning(pop)
#endif

PX_INLINE Foundation& getFoundation()
{
	return Foundation::getInstance();
}

PX_INLINE void setFoundationInstance(Foundation& foundation)
{
	Foundation::setInstance(foundation);
}

} // namespace shdfnd
} // namespace physx

// shortcut macros:
// usage: Foundation::error(PX_WARN, "static friction %f is is lower than dynamic friction %d", sfr, dfr);
#define PX_WARN ::physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__
#define PX_INFO ::physx::PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__

#if PX_DEBUG || PX_CHECKED
#define PX_WARN_ONCE(string)                                                                                           \
	{                                                                                                                  \
		static PxU32 timestamp = 0;                                                                                    \
		if(timestamp != Ps::getFoundation().getWarnOnceTimestamp())                                                    \
		{                                                                                                              \
			timestamp = Ps::getFoundation().getWarnOnceTimestamp();                                                    \
			Ps::getFoundation().error(PX_WARN, string);                                                                \
		}                                                                                                              \
	\
}
#define PX_WARN_ONCE_IF(condition, string)                                                                             \
	{                                                                                                                  \
		if(condition)                                                                                                  \
		{                                                                                                              \
			PX_WARN_ONCE(string)                                                                                       \
		}                                                                                                              \
	\
}
#else
#define PX_WARN_ONCE(string) ((void)0)
#define PX_WARN_ONCE_IF(condition, string) ((void)0)
#endif

#endif
