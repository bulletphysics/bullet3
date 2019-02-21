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

#include "foundation/PxAssert.h"
#include "foundation/PxErrorCallback.h"

#include "Ps.h"
#include "PsFoundation.h"
#include "PsAtomic.h"
#include "PsThread.h"

#include <math.h>
#if !PX_APPLE_FAMILY && !defined(ANDROID) && !defined(__CYGWIN__) && !PX_PS4 && !PX_EMSCRIPTEN
#include <bits/local_lim.h> // PTHREAD_STACK_MIN
#endif
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#if !PX_PS4
#include <sys/syscall.h>
#if !PX_APPLE_FAMILY && !PX_EMSCRIPTEN
#include <asm/unistd.h>
#include <sys/resource.h>
#endif
#endif

#if PX_APPLE_FAMILY
#include <sys/types.h>
#include <sys/sysctl.h>
#include <TargetConditionals.h>
#include <pthread.h>
#endif

// fwd
#if defined(ANDROID)
extern "C" {
int android_getCpuCount(void);
}
#endif

#define PxSpinLockPause() asm("nop")

namespace physx
{
namespace shdfnd
{

#if PX_PS4
	uint64_t setAffinityMaskPS4(pthread_t, uint32_t);
	int32_t setNamePS4(pthread_t, const char*);
	int32_t pthreadCreatePS4(pthread_t *, const pthread_attr_t *, void *(*) (void *), void *, const char*);
#endif

namespace
{

typedef enum
{
	_PxThreadNotStarted,
	_PxThreadStarted,
	_PxThreadStopped
} PxThreadState;

class _ThreadImpl
{
  public:
	ThreadImpl::ExecuteFn fn;
	void* arg;
	volatile int32_t quitNow;
	volatile int32_t threadStarted;
	volatile int32_t state;

	pthread_t thread;
	pid_t tid;

	uint32_t affinityMask;
	const char* name;
};

_ThreadImpl* getThread(ThreadImpl* impl)
{
	return reinterpret_cast<_ThreadImpl*>(impl);
}

static void setTid(_ThreadImpl& threadImpl)
{
// query TID
#if PX_PS4 || (defined (TARGET_OS_TV) && TARGET_OS_TV)
// AM: TODO: neither of the below are implemented
#elif PX_APPLE_FAMILY
	threadImpl.tid = syscall(SYS_gettid);
#elif PX_EMSCRIPTEN
	threadImpl.tid = pthread_self();
#else
	threadImpl.tid = syscall(__NR_gettid);
#endif

	// notify/unblock parent thread
	atomicCompareExchange(&(threadImpl.threadStarted), 1, 0);
}

void* PxThreadStart(void* arg)
{
	_ThreadImpl* impl = getThread(reinterpret_cast<ThreadImpl*>(arg));
	impl->state = _PxThreadStarted;

	// run setTid in thread's context
	setTid(*impl);

	// then run either the passed in function or execute from the derived class (Runnable).
	if(impl->fn)
		(*impl->fn)(impl->arg);
	else if(impl->arg)
		(reinterpret_cast<Runnable*>(impl->arg))->execute();
	return 0;
}
}

uint32_t ThreadImpl::getSize()
{
	return sizeof(_ThreadImpl);
}

ThreadImpl::Id ThreadImpl::getId()
{
	return Id(pthread_self());
}

ThreadImpl::ThreadImpl()
{
	getThread(this)->thread = 0;
	getThread(this)->tid = 0;
	getThread(this)->state = _PxThreadNotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->threadStarted = 0;
	getThread(this)->fn = NULL;
	getThread(this)->arg = NULL;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = "set my name before starting me";
}

ThreadImpl::ThreadImpl(ThreadImpl::ExecuteFn fn, void* arg, const char* name)
{
	getThread(this)->thread = 0;
	getThread(this)->tid = 0;
	getThread(this)->state = _PxThreadNotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->threadStarted = 0;
	getThread(this)->fn = fn;
	getThread(this)->arg = arg;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = name;

	start(0, NULL);
}

ThreadImpl::~ThreadImpl()
{
	if(getThread(this)->state == _PxThreadStarted)
		kill();
}

void ThreadImpl::start(uint32_t stackSize, Runnable* runnable)
{
	if(getThread(this)->state != _PxThreadNotStarted)
		return;

	if(stackSize == 0)
		stackSize = getDefaultStackSize();

#if defined(PTHREAD_STACK_MIN) && !defined(ANDROID)
	if(stackSize < PTHREAD_STACK_MIN)
	{
		shdfnd::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
		                              "ThreadImpl::start(): stack size was set below PTHREAD_STACK_MIN");
		stackSize = PTHREAD_STACK_MIN;
	}
#endif

	if(runnable && !getThread(this)->arg && !getThread(this)->fn)
		getThread(this)->arg = runnable;

	pthread_attr_t attr;
	int status = pthread_attr_init(&attr);
	PX_ASSERT(!status);
	PX_UNUSED(status);

	status = pthread_attr_setstacksize(&attr, stackSize);
	PX_ASSERT(!status);
#if PX_PS4
	status = pthreadCreatePS4(&getThread(this)->thread, &attr, PxThreadStart, this, getThread(this)->name);
#else
	status = pthread_create(&getThread(this)->thread, &attr, PxThreadStart, this);
#endif
	PX_ASSERT(!status);

	// wait for thread to startup and write out TID
	// otherwise TID dependent calls like setAffinity will fail.
	while(atomicCompareExchange(&(getThread(this)->threadStarted), 1, 1) == 0)
		yield();

	// here we are sure that getThread(this)->state >= _PxThreadStarted

	status = pthread_attr_destroy(&attr);
	PX_ASSERT(!status);

	// apply stored affinity mask
	if(getThread(this)->affinityMask)
		setAffinityMask(getThread(this)->affinityMask);

#if !PX_PS4
	if (getThread(this)->name)
		setName(getThread(this)->name);
#endif
}

void ThreadImpl::signalQuit()
{
	atomicIncrement(&(getThread(this)->quitNow));
}

bool ThreadImpl::waitForQuit()
{
	if(getThread(this)->state == _PxThreadNotStarted)
		return false;

	// works also with a stopped/exited thread if the handle is still valid
	pthread_join(getThread(this)->thread, NULL);
	return true;
}

bool ThreadImpl::quitIsSignalled()
{
	return atomicCompareExchange(&(getThread(this)->quitNow), 0, 0) != 0;
}

#if defined(PX_GCC_FAMILY)
__attribute__((noreturn))
#endif
    void ThreadImpl::quit()
{
	getThread(this)->state = _PxThreadStopped;
	pthread_exit(0);
}

void ThreadImpl::kill()
{
#ifndef ANDROID
	if(getThread(this)->state == _PxThreadStarted)
		pthread_cancel(getThread(this)->thread);
	getThread(this)->state = _PxThreadStopped;
#else
	shdfnd::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
	                              "ThreadImpl::kill() called, but is not implemented");
#endif
}

void ThreadImpl::sleep(uint32_t ms)
{
	timespec sleepTime;
	uint32_t remainder = ms % 1000;
	sleepTime.tv_sec = ms - remainder;
	sleepTime.tv_nsec = remainder * 1000000L;

	while(nanosleep(&sleepTime, &sleepTime) == -1)
		continue;
}

void ThreadImpl::yield()
{
	sched_yield();
}

uint32_t ThreadImpl::setAffinityMask(uint32_t mask)
{
	// Same as windows impl if mask is zero
	if(!mask)
		return 0;

	getThread(this)->affinityMask = mask;

	uint64_t prevMask = 0;

	if(getThread(this)->state == _PxThreadStarted)
	{
#if PX_PS4
		prevMask = setAffinityMaskPS4(getThread(this)->thread, mask);
#elif PX_EMSCRIPTEN
		// not supported
#elif !PX_APPLE_FAMILY // Apple doesn't support syscall with getaffinity and setaffinity
		int32_t errGet = syscall(__NR_sched_getaffinity, getThread(this)->tid, sizeof(prevMask), &prevMask);
		if(errGet < 0)
			return 0;

		int32_t errSet = syscall(__NR_sched_setaffinity, getThread(this)->tid, sizeof(mask), &mask);
		if(errSet != 0)
			return 0;
#endif
	}

	return uint32_t(prevMask);
}

void ThreadImpl::setName(const char* name)
{
	getThread(this)->name = name;

	if (getThread(this)->state == _PxThreadStarted)
	{
#if(defined(ANDROID) && (__ANDROID_API__ > 8))
		pthread_setname_np(getThread(this)->thread, name);
#elif PX_PS4
		setNamePS4(getThread(this)->thread, name);
#else
		// not implemented because most unix APIs expect setName()
		// to be called from the thread's context. Example see next comment:

		// this works only with the current thread and can rename
		// the main process if used in the wrong context:
		// prctl(PR_SET_NAME, reinterpret_cast<unsigned long>(name) ,0,0,0);
		PX_UNUSED(name);
#endif
	}
}

#if !PX_APPLE_FAMILY
static ThreadPriority::Enum convertPriorityFromLinux(uint32_t inPrio, int policy)
{
	PX_COMPILE_TIME_ASSERT(ThreadPriority::eLOW > ThreadPriority::eHIGH);
	PX_COMPILE_TIME_ASSERT(ThreadPriority::eHIGH == 0);

	int maxL = sched_get_priority_max(policy);
	int minL = sched_get_priority_min(policy);
	int rangeL = maxL - minL;
	int rangeNv = ThreadPriority::eLOW - ThreadPriority::eHIGH;

	// case for default scheduler policy
	if(rangeL == 0)
		return ThreadPriority::eNORMAL;

	float floatPrio = (float(maxL - inPrio) * float(rangeNv)) / float(rangeL);

	return ThreadPriority::Enum(int(roundf(floatPrio)));
}

static int convertPriorityToLinux(ThreadPriority::Enum inPrio, int policy)
{
	int maxL = sched_get_priority_max(policy);
	int minL = sched_get_priority_min(policy);
	int rangeL = maxL - minL;
	int rangeNv = ThreadPriority::eLOW - ThreadPriority::eHIGH;

	// case for default scheduler policy
	if(rangeL == 0)
		return 0;

	float floatPrio = (float(ThreadPriority::eLOW - inPrio) * float(rangeL)) / float(rangeNv);

	return minL + int(roundf(floatPrio));
}
#endif

void ThreadImpl::setPriority(ThreadPriority::Enum val)
{
	PX_UNUSED(val);
#if !PX_APPLE_FAMILY
	int policy;
	sched_param s_param;
	pthread_getschedparam(getThread(this)->thread, &policy, &s_param);
	s_param.sched_priority = convertPriorityToLinux(val, policy);
	pthread_setschedparam(getThread(this)->thread, policy, &s_param);
#endif
}

ThreadPriority::Enum ThreadImpl::getPriority(Id pthread)
{
	PX_UNUSED(pthread);
#if !PX_APPLE_FAMILY
	int policy;
	sched_param s_param;
	int ret = pthread_getschedparam(pthread_t(pthread), &policy, &s_param);
	if(ret == 0)
		return convertPriorityFromLinux(s_param.sched_priority, policy);
	else
		return ThreadPriority::eNORMAL;
#else
	return ThreadPriority::eNORMAL;
#endif
}

uint32_t ThreadImpl::getNbPhysicalCores()
{
#if PX_APPLE_FAMILY
	int count;
	size_t size = sizeof(count);
	return sysctlbyname("hw.physicalcpu", &count, &size, NULL, 0) ? 0 : count;
#elif defined(ANDROID)
	return android_getCpuCount();
#else
	// Linux exposes CPU topology using /sys/devices/system/cpu
	// https://www.kernel.org/doc/Documentation/cputopology.txt
	if(FILE* f = fopen("/sys/devices/system/cpu/possible", "r"))
	{
		int minIndex, maxIndex;
		int n = fscanf(f, "%d-%d", &minIndex, &maxIndex);
		fclose(f);

		if(n == 2)
			return (maxIndex - minIndex) + 1;
		else if(n == 1)
			return minIndex + 1;
	}

#if PX_PS4
	// Reducing to 6 to take into account that the OS appears to use 2 cores at peak currently.
	return 6;
#else
	// For non-Linux kernels this fallback is possibly the best we can do
	// but will report logical (hyper-threaded) counts
	int n = sysconf(_SC_NPROCESSORS_CONF);
	if(n < 0)
		return 0;
	else
		return n;
#endif
#endif
}

uint32_t TlsAlloc()
{
	pthread_key_t key;
	int status = pthread_key_create(&key, NULL);
	PX_ASSERT(!status);
	PX_UNUSED(status);
	return uint32_t(key);
}

void TlsFree(uint32_t index)
{
	int status = pthread_key_delete(pthread_key_t(index));
	PX_ASSERT(!status);
	PX_UNUSED(status);
}

void* TlsGet(uint32_t index)
{
	return reinterpret_cast<void*>(pthread_getspecific(pthread_key_t(index)));
}

size_t TlsGetValue(uint32_t index)
{
	return reinterpret_cast<size_t>(pthread_getspecific(pthread_key_t(index)));
}

uint32_t TlsSet(uint32_t index, void* value)
{
	int status = pthread_setspecific(pthread_key_t(index), value);
	PX_ASSERT(!status);
	return !status;
}

uint32_t TlsSetValue(uint32_t index, size_t value)
{
	int status = pthread_setspecific(pthread_key_t(index), reinterpret_cast<void*>(value));
	PX_ASSERT(!status);
	return !status;
}

// DM: On Linux x86-32, without implementation-specific restrictions
// the default stack size for a new thread should be 2 megabytes (kernel.org).
// NOTE: take care of this value on other architectures!
uint32_t ThreadImpl::getDefaultStackSize()
{
	return 1 << 21;
}

} // namespace shdfnd
} // namespace physx
