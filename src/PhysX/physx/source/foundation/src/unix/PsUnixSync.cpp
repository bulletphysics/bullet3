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

#include "foundation/PxAssert.h"

#include "Ps.h"
#include "PsUserAllocated.h"
#include "PsSync.h"

#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>

namespace physx
{
namespace shdfnd
{

namespace
{
class _SyncImpl
{
  public:
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	volatile int setCounter;
	volatile bool is_set;
};

_SyncImpl* getSync(SyncImpl* impl)
{
	return reinterpret_cast<_SyncImpl*>(impl);
}
}

uint32_t SyncImpl::getSize()
{
	return sizeof(_SyncImpl);
}

struct PxUnixScopeLock
{
	PxUnixScopeLock(pthread_mutex_t& m) : mMutex(m)
	{
		pthread_mutex_lock(&mMutex);
	}

	~PxUnixScopeLock()
	{
		pthread_mutex_unlock(&mMutex);
	}

  private:
	pthread_mutex_t& mMutex;
};

SyncImpl::SyncImpl()
{
	int status = pthread_mutex_init(&getSync(this)->mutex, 0);
	PX_ASSERT(!status);
	status = pthread_cond_init(&getSync(this)->cond, 0);
	PX_ASSERT(!status);
	PX_UNUSED(status);
	getSync(this)->is_set = false;
	getSync(this)->setCounter = 0;
}

SyncImpl::~SyncImpl()
{
	pthread_cond_destroy(&getSync(this)->cond);
	pthread_mutex_destroy(&getSync(this)->mutex);
}

void SyncImpl::reset()
{
	PxUnixScopeLock lock(getSync(this)->mutex);
	getSync(this)->is_set = false;
}

void SyncImpl::set()
{
	PxUnixScopeLock lock(getSync(this)->mutex);
	if(!getSync(this)->is_set)
	{
		getSync(this)->is_set = true;
		getSync(this)->setCounter++;
		pthread_cond_broadcast(&getSync(this)->cond);
	}
}

bool SyncImpl::wait(uint32_t ms)
{
	PxUnixScopeLock lock(getSync(this)->mutex);
	int lastSetCounter = getSync(this)->setCounter;
	if(!getSync(this)->is_set)
	{
		if(ms == uint32_t(-1))
		{
			// have to loop here and check is_set since pthread_cond_wait can return successfully
			// even if it was not signaled by pthread_cond_broadcast (OS efficiency design decision)
			int status = 0;
			while(!status && !getSync(this)->is_set && (lastSetCounter == getSync(this)->setCounter))
				status = pthread_cond_wait(&getSync(this)->cond, &getSync(this)->mutex);
			PX_ASSERT((!status && getSync(this)->is_set) || (lastSetCounter != getSync(this)->setCounter));
		}
		else
		{
			timespec ts;
			timeval tp;
			gettimeofday(&tp, NULL);
			uint32_t sec = ms / 1000;
			uint32_t usec = (ms - 1000 * sec) * 1000;

			// sschirm: taking into account that us might accumulate to a second
			// otherwise the pthread_cond_timedwait complains on osx.
			usec = tp.tv_usec + usec;
			uint32_t div_sec = usec / 1000000;
			uint32_t rem_usec = usec - div_sec * 1000000;

			ts.tv_sec = tp.tv_sec + sec + div_sec;
			ts.tv_nsec = rem_usec * 1000;

			// have to loop here and check is_set since pthread_cond_timedwait can return successfully
			// even if it was not signaled by pthread_cond_broadcast (OS efficiency design decision)
			int status = 0;
			while(!status && !getSync(this)->is_set && (lastSetCounter == getSync(this)->setCounter))
				status = pthread_cond_timedwait(&getSync(this)->cond, &getSync(this)->mutex, &ts);
			PX_ASSERT((!status && getSync(this)->is_set) || (status == ETIMEDOUT) ||
			          (lastSetCounter != getSync(this)->setCounter));
		}
	}
	return getSync(this)->is_set || (lastSetCounter != getSync(this)->setCounter);
}

} // namespace shdfnd
} // namespace physx
