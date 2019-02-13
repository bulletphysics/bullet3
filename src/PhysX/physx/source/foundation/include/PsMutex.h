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

#ifndef PSFOUNDATION_PSMUTEX_H
#define PSFOUNDATION_PSMUTEX_H

#include "PsAllocator.h"

/*
 * This <new> inclusion is a best known fix for gcc 4.4.1 error:
 * Creating object file for apex/src/PsAllocator.cpp ...
 * In file included from apex/include/PsFoundation.h:30,
 *                from apex/src/PsAllocator.cpp:26:
 * apex/include/PsMutex.h: In constructor  'physx::shdfnd::MutexT<Alloc>::MutexT(const Alloc&)':
 * apex/include/PsMutex.h:92: error: no matching function for call to 'operator new(unsigned int,
 * physx::shdfnd::MutexImpl*&)'
 * <built-in>:0: note: candidates are: void* operator new(unsigned int)
 */
#include <new>

namespace physx
{
namespace shdfnd
{
class PX_FOUNDATION_API MutexImpl
{
  public:
	/**
	The constructor for Mutex creates a mutex. It is initially unlocked.
	*/
	MutexImpl();

	/**
	The destructor for Mutex deletes the mutex.
	*/
	~MutexImpl();

	/**
	Acquire (lock) the mutex. If the mutex is already locked
	by another thread, this method blocks until the mutex is
	unlocked.
	*/
	void lock();

	/**
	Acquire (lock) the mutex. If the mutex is already locked
	by another thread, this method returns false without blocking.
	*/
	bool trylock();

	/**
	Release (unlock) the mutex.
	*/
	void unlock();

	/**
	Size of this class.
	*/
	static uint32_t getSize();
};

template <typename Alloc = ReflectionAllocator<MutexImpl> >
class MutexT : protected Alloc
{
	PX_NOCOPY(MutexT)
  public:
	class ScopedLock
	{
		MutexT<Alloc>& mMutex;
		PX_NOCOPY(ScopedLock)
	  public:
		PX_INLINE ScopedLock(MutexT<Alloc>& mutex) : mMutex(mutex)
		{
			mMutex.lock();
		}
		PX_INLINE ~ScopedLock()
		{
			mMutex.unlock();
		}
	};

	/**
	The constructor for Mutex creates a mutex. It is initially unlocked.
	*/
	MutexT(const Alloc& alloc = Alloc()) : Alloc(alloc)
	{
		mImpl = reinterpret_cast<MutexImpl*>(Alloc::allocate(MutexImpl::getSize(), __FILE__, __LINE__));
		PX_PLACEMENT_NEW(mImpl, MutexImpl)();
	}

	/**
	The destructor for Mutex deletes the mutex.
	*/
	~MutexT()
	{
		mImpl->~MutexImpl();
		Alloc::deallocate(mImpl);
	}

	/**
	Acquire (lock) the mutex. If the mutex is already locked
	by another thread, this method blocks until the mutex is
	unlocked.
	*/
	void lock() const
	{
		mImpl->lock();
	}

	/**
	Acquire (lock) the mutex. If the mutex is already locked
	by another thread, this method returns false without blocking,
	returns true if lock is successfully acquired
	*/
	bool trylock() const
	{
		return mImpl->trylock();
	}

	/**
	Release (unlock) the mutex, the calling thread must have
	previously called lock() or method will error
	*/
	void unlock() const
	{
		mImpl->unlock();
	}

  private:
	MutexImpl* mImpl;
};

class PX_FOUNDATION_API ReadWriteLock
{
	PX_NOCOPY(ReadWriteLock)
  public:
	ReadWriteLock();
	~ReadWriteLock();

	// "takeLock" can only be false if the thread already holds the mutex, e.g. if it already acquired the write lock
	void lockReader(bool takeLock);
	void lockWriter();

	void unlockReader();
	void unlockWriter();

  private:
	class ReadWriteLockImpl* mImpl;
};

typedef MutexT<> Mutex;

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSMUTEX_H
