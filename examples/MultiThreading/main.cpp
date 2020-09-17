/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// ThreadingDemo shows how to use the cross platform thread support interface.
/// You can start threads and perform a blocking wait for completion
/// Under Windows it uses Win32 Threads. On Mac and Linux it uses pthreads. On PlayStation 3 Cell SPU it uses SPURS.

/// June 2010
/// New: critical section/barriers and non-blocking polling for completion

void SampleThreadFunc(void* userPtr, void* lsMemory);
void* SamplelsMemoryFunc();
void SamplelsMemoryReleaseFunc(void* ptr);
#include <stdio.h>
//#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifndef _WIN32
#include "b3PosixThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("testThreads",
																  SampleThreadFunc,
																  SamplelsMemoryFunc,
SamplelsMemoryReleaseFunc,
																  numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;
}

#elif defined(_WIN32)
#include "b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("testThreads", SampleThreadFunc, SamplelsMemoryFunc, SamplelsMemoryReleaseFunc, numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;
}
#endif

struct SampleArgs
{
	SampleArgs()
		: m_fakeWork(1)
	{
	}
	b3CriticalSection* m_cs;
	float m_fakeWork;
};

struct SampleThreadLocalStorage
{
	int threadId;
};

void SampleThreadFunc(void* userPtr, void* lsMemory)
{
	printf("thread started\n");

	SampleThreadLocalStorage* localStorage = (SampleThreadLocalStorage*)lsMemory;

	SampleArgs* args = (SampleArgs*)userPtr;
	int workLeft = true;
	while (workLeft)
	{
		args->m_cs->lock();
		int count = args->m_cs->getSharedParam(0);
		args->m_cs->setSharedParam(0, count - 1);
		args->m_cs->unlock();
		if (count > 0)
		{
			printf("thread %d processed number %d\n", localStorage->threadId, count);
		}
		//do some fake work
		for (int i = 0; i < 1000000; i++)
			args->m_fakeWork = b3Scalar(1.21) * args->m_fakeWork;
		workLeft = count > 0;
	}
	printf("finished\n");
	//do nothing
}

void* SamplelsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new SampleThreadLocalStorage;
}

void SamplelsMemoryReleaseFunc(void* ptr)
{
        SampleThreadLocalStorage* p = (SampleThreadLocalStorage*)ptr;
        delete p;
}


int main(int argc, char** argv)
{
	int numThreads = 8;

	b3ThreadSupportInterface* threadSupport = createThreadSupport(numThreads);

	for (int i = 0; i < threadSupport->getNumTasks(); i++)
	{
		SampleThreadLocalStorage* storage = (SampleThreadLocalStorage*)threadSupport->getThreadLocalMemory(i);
		b3Assert(storage);
		storage->threadId = i;
	}

	SampleArgs args;
	args.m_cs = threadSupport->createCriticalSection();
	args.m_cs->setSharedParam(0, 100);

	int arg0, arg1;
	int i;
	for (i = 0; i < numThreads; i++)
	{
		threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)&args, i);
	}

	bool blockingWait = false;
	if (blockingWait)
	{
		for (i = 0; i < numThreads; i++)
		{
			threadSupport->waitForResponse(&arg0, &arg1);
			printf("finished waiting for response: %d %d\n", arg0, arg1);
		}
	}
	else
	{
		int numActiveThreads = numThreads;
		while (numActiveThreads)
		{
			if (threadSupport->isTaskCompleted(&arg0, &arg1, 0))
			{
				numActiveThreads--;
				printf("numActiveThreads = %d\n", numActiveThreads);
			}
			else
			{
				//				printf("polling..");
			}
		};
	}

	printf("stopping threads\n");

	delete threadSupport;
	printf("Press ENTER to quit\n");
	//getchar();
	return 0;
}
