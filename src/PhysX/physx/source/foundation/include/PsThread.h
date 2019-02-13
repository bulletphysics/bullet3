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

#ifndef PSFOUNDATION_PSTHREAD_H
#define PSFOUNDATION_PSTHREAD_H

#include "PsUserAllocated.h"

// dsequeira: according to existing comment here (David Black would be my guess)
// "This is useful to reduce bus contention on tight spin locks. And it needs
// to be a macro as the xenon compiler often ignores even __forceinline." What's not
// clear is why a pause function needs inlining...? (TODO: check with XBox team)

// todo: these need to go somewhere else

#if PX_WINDOWS_FAMILY || PX_XBOXONE
#define PxSpinLockPause() __asm pause
#elif PX_LINUX || PX_ANDROID || PX_PS4 || PX_APPLE_FAMILY || PX_SWITCH
#define PxSpinLockPause() asm("nop")
#else
#error "Platform not supported!"
#endif

namespace physx
{
namespace shdfnd
{
struct ThreadPriority // todo: put in some other header file
{
	enum Enum
	{
		/**
	    \brief High priority
	    */
		eHIGH         = 0,

		/**
	    \brief Above Normal priority
	    */
		eABOVE_NORMAL = 1,

		/**
	    \brief Normal/default priority
	    */
		eNORMAL       = 2,

		/**
	    \brief Below Normal priority
	    */
		eBELOW_NORMAL = 3,

		/**
	    \brief Low priority.
	    */
		eLOW          = 4,
		eFORCE_DWORD  = 0xffFFffFF
	};
};

class Runnable
{
  public:
	Runnable()
	{
	}
	virtual ~Runnable()
	{
	}
	virtual void execute(void)
	{
	}
};

class PX_FOUNDATION_API ThreadImpl
{
  public:
	typedef size_t Id; // space for a pointer or an integer
	typedef void* (*ExecuteFn)(void*);

	static uint32_t getDefaultStackSize();
	static Id getId();

	/**
	Construct (but do not start) the thread object. The OS thread object will not be created
	until start() is called. Executes in the context
	of the spawning thread.
	*/

	ThreadImpl();

	/**
	Construct and start the the thread, passing the given arg to the given fn. (pthread style)
	*/

	ThreadImpl(ExecuteFn fn, void* arg, const char* name);

	/**
	Deallocate all resources associated with the thread. Should be called in the
	context of the spawning thread.
	*/

	~ThreadImpl();

	/**
	Create the OS thread and start it running. Called in the context of the spawning thread.
	If an affinity mask has previously been set then it will be applied after the
	thread has been created.
	*/

	void start(uint32_t stackSize, Runnable* r);

	/**
	Violently kill the current thread. Blunt instrument, not recommended since
	it can leave all kinds of things unreleased (stack, memory, mutexes...) Should
	be called in the context of the spawning thread.
	*/

	void kill();

	/**
	Stop the thread. Signals the spawned thread that it should stop, so the
	thread should check regularly
	*/

	void signalQuit();

	/**
	Wait for a thread to stop. Should be called in the context of the spawning
	thread. Returns false if the thread has not been started.
	*/

	bool waitForQuit();

	/**
	check whether the thread is signalled to quit. Called in the context of the
	spawned thread.
	*/

	bool quitIsSignalled();

	/**
	Cleanly shut down this thread. Called in the context of the spawned thread.
	*/
	void quit();

	/**
	Change the affinity mask for this thread. The mask is a platform
	specific value.

	On Windows, Linux, PS4, XboxOne and Switch platforms, each set mask bit represents
	the index of a logical processor that the OS may schedule thread execution on.
	Bits outside the range of valid logical processors may be ignored or cause
	the function to return an error.

	On Apple platforms, this function has no effect.

	If the thread has not yet been started then the mask is stored
	and applied when the thread is started.

	If the thread has already been started then this method	returns the
	previous affinity mask on success, otherwise it returns zero.
	*/
	uint32_t setAffinityMask(uint32_t mask);

	static ThreadPriority::Enum getPriority(Id threadId);

	/** Set thread priority. */
	void setPriority(ThreadPriority::Enum prio);

	/** set the thread's name */
	void setName(const char* name);

	/** Put the current thread to sleep for the given number of milliseconds */
	static void sleep(uint32_t ms);

	/** Yield the current thread's slot on the CPU */
	static void yield();

	/** Return the number of physical cores (does not include hyper-threaded cores), returns 0 on failure */
	static uint32_t getNbPhysicalCores();

	/**
	Size of this class.
	*/
	static uint32_t getSize();
};

/**
Thread abstraction API
*/
template <typename Alloc = ReflectionAllocator<ThreadImpl> >
class ThreadT : protected Alloc, public UserAllocated, public Runnable
{
  public:
	typedef ThreadImpl::Id Id; // space for a pointer or an integer

	/**
	Construct (but do not start) the thread object. Executes in the context
	of the spawning thread
	*/
	ThreadT(const Alloc& alloc = Alloc()) : Alloc(alloc)
	{
		mImpl = reinterpret_cast<ThreadImpl*>(Alloc::allocate(ThreadImpl::getSize(), __FILE__, __LINE__));
		PX_PLACEMENT_NEW(mImpl, ThreadImpl)();
	}

	/**
	Construct and start the the thread, passing the given arg to the given fn. (pthread style)
	*/
	ThreadT(ThreadImpl::ExecuteFn fn, void* arg, const char* name, const Alloc& alloc = Alloc()) : Alloc(alloc)
	{
		mImpl = reinterpret_cast<ThreadImpl*>(Alloc::allocate(ThreadImpl::getSize(), __FILE__, __LINE__));
		PX_PLACEMENT_NEW(mImpl, ThreadImpl)(fn, arg, name);
	}

	/**
	Deallocate all resources associated with the thread. Should be called in the
	context of the spawning thread.
	*/
	virtual ~ThreadT()
	{
		mImpl->~ThreadImpl();
		Alloc::deallocate(mImpl);
	}

	/**
	start the thread running. Called in the context of the spawning thread.
	*/

	void start(uint32_t stackSize = ThreadImpl::getDefaultStackSize())
	{
		mImpl->start(stackSize, this);
	}

	/**
	Violently kill the current thread. Blunt instrument, not recommended since
	it can leave all kinds of things unreleased (stack, memory, mutexes...) Should
	be called in the context of the spawning thread.
	*/

	void kill()
	{
		mImpl->kill();
	}

	/**
	The virtual execute() method is the user defined function that will
	run in the new thread. Called in the context of the spawned thread.
	*/

	virtual void execute(void)
	{
	}

	/**
	stop the thread. Signals the spawned thread that it should stop, so the
	thread should check regularly
	*/

	void signalQuit()
	{
		mImpl->signalQuit();
	}

	/**
	Wait for a thread to stop. Should be called in the context of the spawning
	thread. Returns false if the thread has not been started.
	*/

	bool waitForQuit()
	{
		return mImpl->waitForQuit();
	}

	/**
	check whether the thread is signalled to quit. Called in the context of the
	spawned thread.
	*/

	bool quitIsSignalled()
	{
		return mImpl->quitIsSignalled();
	}

	/**
	Cleanly shut down this thread. Called in the context of the spawned thread.
	*/
	void quit()
	{
		mImpl->quit();
	}

	uint32_t setAffinityMask(uint32_t mask)
	{
		return mImpl->setAffinityMask(mask);
	}

	static ThreadPriority::Enum getPriority(ThreadImpl::Id threadId)
	{
		return ThreadImpl::getPriority(threadId);
	}

	/** Set thread priority. */
	void setPriority(ThreadPriority::Enum prio)
	{
		mImpl->setPriority(prio);
	}

	/** set the thread's name */
	void setName(const char* name)
	{
		mImpl->setName(name);
	}

	/** Put the current thread to sleep for the given number of milliseconds */
	static void sleep(uint32_t ms)
	{
		ThreadImpl::sleep(ms);
	}

	/** Yield the current thread's slot on the CPU */
	static void yield()
	{
		ThreadImpl::yield();
	}

	static uint32_t getDefaultStackSize()
	{
		return ThreadImpl::getDefaultStackSize();
	}

	static ThreadImpl::Id getId()
	{
		return ThreadImpl::getId();
	}

	static uint32_t getNbPhysicalCores()
	{
		return ThreadImpl::getNbPhysicalCores();
	}

  private:
	class ThreadImpl* mImpl;
};

typedef ThreadT<> Thread;

PX_FOUNDATION_API uint32_t TlsAlloc();
PX_FOUNDATION_API void TlsFree(uint32_t index);
PX_FOUNDATION_API void* TlsGet(uint32_t index);
PX_FOUNDATION_API size_t TlsGetValue(uint32_t index);
PX_FOUNDATION_API uint32_t TlsSet(uint32_t index, void* value);
PX_FOUNDATION_API uint32_t TlsSetValue(uint32_t index, size_t value);

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSTHREAD_H
