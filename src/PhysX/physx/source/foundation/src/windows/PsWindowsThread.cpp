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

#include "windows/PsWindowsInclude.h"
#include "PsFoundation.h"
#include "PsThread.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxAssert.h"

// an exception for setting the thread name in Microsoft debuggers
#define NS_MS_VC_EXCEPTION 0x406D1388

namespace physx
{
namespace shdfnd
{

namespace
{

#if PX_VC
#pragma warning(disable : 4061) // enumerator 'identifier' in switch of enum 'enumeration' is not handled
#pragma warning(disable : 4191) //'operator/operation' : unsafe conversion from 'type of expression' to 'type required'
#endif

// struct for naming a thread in the debugger
#pragma pack(push, 8)

typedef struct tagTHREADNAME_INFO
{
	DWORD dwType;     // Must be 0x1000.
	LPCSTR szName;    // Pointer to name (in user addr space).
	DWORD dwThreadID; // Thread ID (-1=caller thread).
	DWORD dwFlags;    // Reserved for future use, must be zero.
} THREADNAME_INFO;

#pragma pack(pop)

class _ThreadImpl
{
  public:
	enum State
	{
		NotStarted,
		Started,
		Stopped
	};

	HANDLE thread;
	LONG quitNow; // Should be 32bit aligned on SMP systems.
	State state;
	DWORD threadID;

	ThreadImpl::ExecuteFn fn;
	void* arg;

	uint32_t affinityMask;
	const char* name;
};

_ThreadImpl* getThread(ThreadImpl* impl)
{
	return reinterpret_cast<_ThreadImpl*>(impl);
}

DWORD WINAPI PxThreadStart(LPVOID arg)
{
	_ThreadImpl* impl = getThread((ThreadImpl*)arg);

	// run either the passed in function or execute from the derived class (Runnable).
	if(impl->fn)
		(*impl->fn)(impl->arg);
	else if(impl->arg)
		((Runnable*)impl->arg)->execute();
	return 0;
}

// cache physical thread count
uint32_t gPhysicalCoreCount = 0;
}

uint32_t ThreadImpl::getSize()
{
	return sizeof(_ThreadImpl);
}

ThreadImpl::Id ThreadImpl::getId()
{
	return static_cast<Id>(GetCurrentThreadId());
}

// fwd GetLogicalProcessorInformation()
typedef BOOL(WINAPI* LPFN_GLPI)(PSYSTEM_LOGICAL_PROCESSOR_INFORMATION, PDWORD);

uint32_t ThreadImpl::getNbPhysicalCores()
{
	if(!gPhysicalCoreCount)
	{
		// modified example code from: http://msdn.microsoft.com/en-us/library/ms683194
		LPFN_GLPI glpi;
		PSYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer = NULL;
		PSYSTEM_LOGICAL_PROCESSOR_INFORMATION ptr = NULL;
		DWORD returnLength = 0;
		DWORD processorCoreCount = 0;
		DWORD byteOffset = 0;

		glpi = (LPFN_GLPI)GetProcAddress(GetModuleHandle(TEXT("kernel32")), "GetLogicalProcessorInformation");

		if(NULL == glpi)
		{
			// GetLogicalProcessorInformation not supported on OS < XP Service Pack 3
			return 0;
		}

		DWORD rc = (DWORD)glpi(NULL, &returnLength);
		PX_ASSERT(rc == FALSE);
		PX_UNUSED(rc);

		// first query reports required buffer space
		if(GetLastError() == ERROR_INSUFFICIENT_BUFFER)
		{
			buffer = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION)PxAlloca(returnLength);
		}
		else
		{
			physx::shdfnd::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			                                     "Error querying buffer size for number of physical processors");
			return 0;
		}

		// retrieve data
		rc = (DWORD)glpi(buffer, &returnLength);
		if(rc != TRUE)
		{
			physx::shdfnd::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			                                     "Error querying number of physical processors");
			return 0;
		}

		ptr = buffer;

		while(byteOffset + sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) <= returnLength)
		{
			switch(ptr->Relationship)
			{
			case RelationProcessorCore:
				processorCoreCount++;
				break;
			default:
				break;
			}

			byteOffset += sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
			ptr++;
		}

		gPhysicalCoreCount = processorCoreCount;
	}

	return gPhysicalCoreCount;
}

ThreadImpl::ThreadImpl()
{
	getThread(this)->thread = NULL;
	getThread(this)->state = _ThreadImpl::NotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->fn = NULL;
	getThread(this)->arg = NULL;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = NULL;
}

ThreadImpl::ThreadImpl(ExecuteFn fn, void* arg, const char* name)
{
	getThread(this)->thread = NULL;
	getThread(this)->state = _ThreadImpl::NotStarted;
	getThread(this)->quitNow = 0;
	getThread(this)->fn = fn;
	getThread(this)->arg = arg;
	getThread(this)->affinityMask = 0;
	getThread(this)->name = name;

	start(0, NULL);
}

ThreadImpl::~ThreadImpl()
{
	if(getThread(this)->state == _ThreadImpl::Started)
		kill();
	CloseHandle(getThread(this)->thread);
}

void ThreadImpl::start(uint32_t stackSize, Runnable* runnable)
{
	if(getThread(this)->state != _ThreadImpl::NotStarted)
		return;
	getThread(this)->state = _ThreadImpl::Started;

	if(runnable && !getThread(this)->arg && !getThread(this)->fn)
		getThread(this)->arg = runnable;

	getThread(this)->thread =
	    CreateThread(NULL, stackSize, PxThreadStart, (LPVOID) this, CREATE_SUSPENDED, &getThread(this)->threadID);
	if(!getThread(this)->thread)
	{
		physx::shdfnd::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			                                    "PsWindowsThread::start: Failed to create thread.");
		getThread(this)->state = _ThreadImpl::NotStarted;
		return;
	}

	// set affinity, set name and resume
	if(getThread(this)->affinityMask)
		setAffinityMask(getThread(this)->affinityMask);

	if (getThread(this)->name)
		setName(getThread(this)->name);

	DWORD rc = ResumeThread(getThread(this)->thread);
	if(rc == DWORD(-1))
	{
		physx::shdfnd::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			                                    "PsWindowsThread::start: Failed to resume thread.");
		getThread(this)->state = _ThreadImpl::NotStarted;
		return;
	}	
}

void ThreadImpl::signalQuit()
{
	InterlockedIncrement(&(getThread(this)->quitNow));
}

bool ThreadImpl::waitForQuit()
{
	if(getThread(this)->state == _ThreadImpl::NotStarted)
		return false;

	WaitForSingleObject(getThread(this)->thread, INFINITE);
	return true;
}

bool ThreadImpl::quitIsSignalled()
{
	return InterlockedCompareExchange(&(getThread(this)->quitNow), 0, 0) != 0;
}

void ThreadImpl::quit()
{
	getThread(this)->state = _ThreadImpl::Stopped;
	ExitThread(0);
}

void ThreadImpl::kill()
{
	if(getThread(this)->state == _ThreadImpl::Started)
		TerminateThread(getThread(this)->thread, 0);
	getThread(this)->state = _ThreadImpl::Stopped;
}

void ThreadImpl::sleep(uint32_t ms)
{
	Sleep(ms);
}

void ThreadImpl::yield()
{
	SwitchToThread();
}

uint32_t ThreadImpl::setAffinityMask(uint32_t mask)
{
	if(mask)
	{
		// store affinity
		getThread(this)->affinityMask = mask;

		// if thread already started apply immediately
		if(getThread(this)->state == _ThreadImpl::Started)
		{
			uint32_t err = uint32_t(SetThreadAffinityMask(getThread(this)->thread, mask));
			return err;
		}
	}

	return 0;
}

void ThreadImpl::setName(const char* name)
{
	getThread(this)->name = name;

	if (getThread(this)->state == _ThreadImpl::Started)
	{
		THREADNAME_INFO info;
		info.dwType = 0x1000;
		info.szName = name;
		info.dwThreadID = getThread(this)->threadID;
		info.dwFlags = 0;

		// C++ Exceptions are disabled for this project, but SEH is not (and cannot be)
		// http://stackoverflow.com/questions/943087/what-exactly-will-happen-if-i-disable-c-exceptions-in-a-project
		__try
		{
			RaiseException(NS_MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
		}
		__except (EXCEPTION_EXECUTE_HANDLER)
		{
			// this runs if not attached to a debugger (thus not really naming the thread)
		}
	}
}

void ThreadImpl::setPriority(ThreadPriority::Enum prio)
{
	BOOL rc = false;
	switch(prio)
	{
	case ThreadPriority::eHIGH:
		rc = SetThreadPriority(getThread(this)->thread, THREAD_PRIORITY_HIGHEST);
		break;
	case ThreadPriority::eABOVE_NORMAL:
		rc = SetThreadPriority(getThread(this)->thread, THREAD_PRIORITY_ABOVE_NORMAL);
		break;
	case ThreadPriority::eNORMAL:
		rc = SetThreadPriority(getThread(this)->thread, THREAD_PRIORITY_NORMAL);
		break;
	case ThreadPriority::eBELOW_NORMAL:
		rc = SetThreadPriority(getThread(this)->thread, THREAD_PRIORITY_BELOW_NORMAL);
		break;
	case ThreadPriority::eLOW:
		rc = SetThreadPriority(getThread(this)->thread, THREAD_PRIORITY_LOWEST);
		break;
	default:
		break;
	}
	if(!rc)
	{
		physx::shdfnd::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			"PsWindowsThread::setPriority: Failed to set thread priority.");
	}
}

ThreadPriority::Enum ThreadImpl::getPriority(Id threadId)
{
	ThreadPriority::Enum retval = ThreadPriority::eLOW;
	int priority = GetThreadPriority((HANDLE)threadId);
	PX_COMPILE_TIME_ASSERT(THREAD_PRIORITY_HIGHEST > THREAD_PRIORITY_ABOVE_NORMAL);
	if(priority >= THREAD_PRIORITY_HIGHEST)
		retval = ThreadPriority::eHIGH;
	else if(priority >= THREAD_PRIORITY_ABOVE_NORMAL)
		retval = ThreadPriority::eABOVE_NORMAL;
	else if(priority >= THREAD_PRIORITY_NORMAL)
		retval = ThreadPriority::eNORMAL;
	else if(priority >= THREAD_PRIORITY_BELOW_NORMAL)
		retval = ThreadPriority::eBELOW_NORMAL;
	return retval;
}

uint32_t TlsAlloc()
{
	DWORD rv = ::TlsAlloc();
	PX_ASSERT(rv != TLS_OUT_OF_INDEXES);
	return (uint32_t)rv;
}

void TlsFree(uint32_t index)
{
	::TlsFree(index);
}

void* TlsGet(uint32_t index)
{
	return ::TlsGetValue(index);
}

size_t TlsGetValue(uint32_t index)
{
	return reinterpret_cast<size_t>(::TlsGetValue(index));
}

uint32_t TlsSet(uint32_t index, void* value)
{
	return (uint32_t)::TlsSetValue(index, value);
}

uint32_t TlsSetValue(uint32_t index, size_t value)
{
	return (uint32_t)::TlsSetValue(index, reinterpret_cast<void*>(value));
}

uint32_t ThreadImpl::getDefaultStackSize()
{
	return 1048576;
};

} // namespace shdfnd
} // namespace physx
