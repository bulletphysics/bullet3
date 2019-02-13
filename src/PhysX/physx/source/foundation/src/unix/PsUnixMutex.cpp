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
#include "foundation/PxErrorCallback.h"

#include "Ps.h"
#include "PsFoundation.h"
#include "PsUserAllocated.h"
#include "PsMutex.h"
#include "PsAtomic.h"
#include "PsThread.h"

#include <pthread.h>

namespace physx
{
namespace shdfnd
{

namespace
{
struct MutexUnixImpl
{
	pthread_mutex_t lock;
	Thread::Id owner;
};

MutexUnixImpl* getMutex(MutexImpl* impl)
{
	return reinterpret_cast<MutexUnixImpl*>(impl);
}
}

MutexImpl::MutexImpl()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
#if !PX_ANDROID
	// mimic default windows behavior where applicable
	pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
#endif
	pthread_mutex_init(&getMutex(this)->lock, &attr);
	pthread_mutexattr_destroy(&attr);
}

MutexImpl::~MutexImpl()
{
	pthread_mutex_destroy(&getMutex(this)->lock);
}

void MutexImpl::lock()
{
	int err = pthread_mutex_lock(&getMutex(this)->lock);
	PX_ASSERT(!err);
	PX_UNUSED(err);

#if PX_DEBUG
	getMutex(this)->owner = Thread::getId();
#endif
}

bool MutexImpl::trylock()
{
	bool success = !pthread_mutex_trylock(&getMutex(this)->lock);
#if PX_DEBUG
	if(success)
		getMutex(this)->owner = Thread::getId();
#endif
	return success;
}

void MutexImpl::unlock()
{
#if PX_DEBUG
	if(getMutex(this)->owner != Thread::getId())
	{
		shdfnd::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
		                              "Mutex must be unlocked only by thread that has already acquired lock");
		return;
	}
#endif

	int err = pthread_mutex_unlock(&getMutex(this)->lock);
	PX_ASSERT(!err);
	PX_UNUSED(err);
}

uint32_t MutexImpl::getSize()
{
	return sizeof(MutexUnixImpl);
}

class ReadWriteLockImpl
{
  public:
	Mutex mutex;
	volatile int readerCounter;
};

ReadWriteLock::ReadWriteLock()
{
	mImpl = reinterpret_cast<ReadWriteLockImpl*>(PX_ALLOC(sizeof(ReadWriteLockImpl), "ReadWriteLockImpl"));
	PX_PLACEMENT_NEW(mImpl, ReadWriteLockImpl);

	mImpl->readerCounter = 0;
}

ReadWriteLock::~ReadWriteLock()
{
	mImpl->~ReadWriteLockImpl();
	PX_FREE(mImpl);
}

void ReadWriteLock::lockReader(bool takeLock)
{
	if(takeLock)
		mImpl->mutex.lock();

	atomicIncrement(&mImpl->readerCounter);

	if(takeLock)
		mImpl->mutex.unlock();
}

void ReadWriteLock::lockWriter()
{
	mImpl->mutex.lock();

	// spin lock until no readers
	while(mImpl->readerCounter);
}

void ReadWriteLock::unlockReader()
{
	atomicDecrement(&mImpl->readerCounter);
}

void ReadWriteLock::unlockWriter()
{
	mImpl->mutex.unlock();
}

} // namespace shdfnd
} // namespace physx
