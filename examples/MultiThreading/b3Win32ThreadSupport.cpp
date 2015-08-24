#ifdef _WIN32
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "b3Win32ThreadSupport.h"


#include <windows.h>




///The number of threads should be equal to the number of available cores
///@todo: each worker should be linked to a single core, using SetThreadIdealProcessor.

///b3Win32ThreadSupport helps to initialize/shutdown libspe2, start/stop SPU tasks and communication
///Setup and initialize SPU/CELL/Libspe2
b3Win32ThreadSupport::b3Win32ThreadSupport(const Win32ThreadConstructionInfo & threadConstructionInfo)
{
	m_maxNumTasks = threadConstructionInfo.m_numThreads;
	startThreads(threadConstructionInfo);
}

///cleanup/shutdown Libspe2
b3Win32ThreadSupport::~b3Win32ThreadSupport()
{
	stopThreads();
}




#include <stdio.h>

DWORD WINAPI Thread_no_1( LPVOID lpParam ) 
{

	b3Win32ThreadSupport::b3ThreadStatus* status = (b3Win32ThreadSupport::b3ThreadStatus*)lpParam;

	
	while (1)
	{
		WaitForSingleObject(status->m_eventStartHandle,INFINITE);
		
		void* userPtr = status->m_userPtr;

		if (userPtr)
		{
			b3Assert(status->m_status);
			status->m_userThreadFunc(userPtr,status->m_lsMemory);
			status->m_status = 2;
			SetEvent(status->m_eventCompletetHandle);
		} else
		{
			//exit Thread
			status->m_status = 3;
			printf("Thread with taskId %i with handle %p exiting\n",status->m_taskId, status->m_threadHandle);
			SetEvent(status->m_eventCompletetHandle);
			break;
		}
		
	}

	printf("Thread TERMINATED\n");
	return 0;

}

///send messages to SPUs
void b3Win32ThreadSupport::runTask(int uiCommand, void* uiArgument0, int taskId)
{
	///	gMidphaseSPU.sendRequest(CMD_GATHER_AND_PROCESS_PAIRLIST, (void*) &taskDesc);
	
	///we should spawn an SPU task here, and in 'waitForResponse' it should wait for response of the (one of) the first tasks that finished
	


	switch (uiCommand)
	{
	case 	B3_THREAD_SCHEDULE_TASK:
		{


//#define SINGLE_THREADED 1
#ifdef SINGLE_THREADED

			b3ThreadStatus&	threadStatus = m_activeThreadStatus[0];
			threadStatus.m_userPtr=(void*)uiArgument0;
			threadStatus.m_userThreadFunc(threadStatus.m_userPtr,threadStatus.m_lsMemory);
			HANDLE handle =0;
#else


			b3ThreadStatus&	threadStatus = m_activeThreadStatus[taskId];
			b3Assert(taskId>=0);
			b3Assert(int(taskId)<m_activeThreadStatus.size());

			threadStatus.m_commandId = uiCommand;
			threadStatus.m_status = 1;
			threadStatus.m_userPtr = (void*)uiArgument0;

			///fire event to start new task
			SetEvent(threadStatus.m_eventStartHandle);

#endif //CollisionTask_LocalStoreMemory

			

			break;
		}
	default:
		{
			///not implemented
			b3Assert(0);
		}

	};


}


///check for messages from SPUs
void b3Win32ThreadSupport::waitForResponse(int *puiArgument0, int *puiArgument1)
{
	///We should wait for (one of) the first tasks to finish (or other SPU messages), and report its response
	
	///A possible response can be 'yes, SPU handled it', or 'no, please do a PPU fallback'


	b3Assert(m_activeThreadStatus.size());

	int last = -1;
#ifndef SINGLE_THREADED
	DWORD res = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], FALSE, INFINITE);
	b3Assert(res != WAIT_FAILED);
	last = res - WAIT_OBJECT_0;

	b3ThreadStatus& threadStatus = m_activeThreadStatus[last];
	b3Assert(threadStatus.m_threadHandle);
	b3Assert(threadStatus.m_eventCompletetHandle);

	//WaitForSingleObject(threadStatus.m_eventCompletetHandle, INFINITE);
	b3Assert(threadStatus.m_status > 1);
	threadStatus.m_status = 0;

	///need to find an active spu
	b3Assert(last>=0);

#else
	last=0;
	b3ThreadStatus& threadStatus = m_activeThreadStatus[last];
#endif //SINGLE_THREADED

	

	*puiArgument0 = threadStatus.m_taskId;
	*puiArgument1 = threadStatus.m_status;


}


///check for messages from SPUs
bool b3Win32ThreadSupport::isTaskCompleted(int *puiArgument0, int *puiArgument1, int timeOutInMilliseconds)
{
	///We should wait for (one of) the first tasks to finish (or other SPU messages), and report its response
	
	///A possible response can be 'yes, SPU handled it', or 'no, please do a PPU fallback'


	b3Assert(m_activeThreadStatus.size());

	int last = -1;
#ifndef SINGLE_THREADED
	DWORD res = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], FALSE, timeOutInMilliseconds);
	
	if ((res != STATUS_TIMEOUT) && (res != WAIT_FAILED))
	{
		
		b3Assert(res != WAIT_FAILED);
		last = res - WAIT_OBJECT_0;

		b3ThreadStatus& threadStatus = m_activeThreadStatus[last];
		b3Assert(threadStatus.m_threadHandle);
		b3Assert(threadStatus.m_eventCompletetHandle);

		//WaitForSingleObject(threadStatus.m_eventCompletetHandle, INFINITE);
		b3Assert(threadStatus.m_status > 1);
		threadStatus.m_status = 0;

		///need to find an active spu
		b3Assert(last>=0);

	#else
		last=0;
		b3ThreadStatus& threadStatus = m_activeThreadStatus[last];
	#endif //SINGLE_THREADED

		

		*puiArgument0 = threadStatus.m_taskId;
		*puiArgument1 = threadStatus.m_status;

		return true;
	} 

	return false;
}


void b3Win32ThreadSupport::startThreads(const Win32ThreadConstructionInfo& threadConstructionInfo)
{

	m_activeThreadStatus.resize(threadConstructionInfo.m_numThreads);
	m_completeHandles.resize(threadConstructionInfo.m_numThreads);

	m_maxNumTasks = threadConstructionInfo.m_numThreads;

	for (int i=0;i<threadConstructionInfo.m_numThreads;i++)
	{
		printf("starting thread %d\n",i);

		b3ThreadStatus&	threadStatus = m_activeThreadStatus[i];

		LPSECURITY_ATTRIBUTES lpThreadAttributes=NULL;
		SIZE_T dwStackSize=threadConstructionInfo.m_threadStackSize;
		LPTHREAD_START_ROUTINE lpStartAddress=&Thread_no_1;
		LPVOID lpParameter=&threadStatus;
		DWORD dwCreationFlags=0;
		LPDWORD lpThreadId=0;

		threadStatus.m_userPtr=0;

		sprintf(threadStatus.m_eventStartHandleName,"eventStart%s%d",threadConstructionInfo.m_uniqueName,i);
		threadStatus.m_eventStartHandle = CreateEventA (0,false,false,threadStatus.m_eventStartHandleName);

		sprintf(threadStatus.m_eventCompletetHandleName,"eventComplete%s%d",threadConstructionInfo.m_uniqueName,i);
		threadStatus.m_eventCompletetHandle = CreateEventA (0,false,false,threadStatus.m_eventCompletetHandleName);

		m_completeHandles[i] = threadStatus.m_eventCompletetHandle;

		HANDLE handle = CreateThread(lpThreadAttributes,dwStackSize,lpStartAddress,lpParameter,	dwCreationFlags,lpThreadId);
		//SetThreadPriority(handle,THREAD_PRIORITY_HIGHEST);
		SetThreadPriority(handle,THREAD_PRIORITY_TIME_CRITICAL);

		SetThreadAffinityMask(handle, 1<<i);

		threadStatus.m_taskId = i;
		threadStatus.m_commandId = 0;
		threadStatus.m_status = 0;
		threadStatus.m_threadHandle = handle;
		threadStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
		threadStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;

		printf("started thread %d with threadHandle %p\n",i,handle);
		
	}

}

void b3Win32ThreadSupport::startThreads()
{
}


///tell the task scheduler we are done with the SPU tasks
void b3Win32ThreadSupport::stopThreads()
{
	int i;
	for (i=0;i<m_activeThreadStatus.size();i++)
	{
		b3ThreadStatus& threadStatus = m_activeThreadStatus[i];
		if (threadStatus.m_status>0)
		{
			WaitForSingleObject(threadStatus.m_eventCompletetHandle, INFINITE);
		}
		

		threadStatus.m_userPtr = 0;
		SetEvent(threadStatus.m_eventStartHandle);
		WaitForSingleObject(threadStatus.m_eventCompletetHandle, INFINITE);

		CloseHandle(threadStatus.m_eventCompletetHandle);
		CloseHandle(threadStatus.m_eventStartHandle);
		CloseHandle(threadStatus.m_threadHandle);

	}

	m_activeThreadStatus.clear();
	m_completeHandles.clear();

}



class b3Win32Barrier : public b3Barrier
{
private:
	CRITICAL_SECTION mExternalCriticalSection;
	CRITICAL_SECTION mLocalCriticalSection;
	HANDLE mRunEvent,mNotifyEvent;
	int mCounter,mEnableCounter;
	int mMaxCount;

public:
	b3Win32Barrier()
	{
		mCounter = 0;
		mMaxCount = 1;
		mEnableCounter = 0;
		InitializeCriticalSection(&mExternalCriticalSection);
		InitializeCriticalSection(&mLocalCriticalSection);
		mRunEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
		mNotifyEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
	}

	virtual ~b3Win32Barrier()
	{
		DeleteCriticalSection(&mExternalCriticalSection);
		DeleteCriticalSection(&mLocalCriticalSection);
		CloseHandle(mRunEvent);
		CloseHandle(mNotifyEvent);
	}

	void sync()
	{
		int eventId;

		EnterCriticalSection(&mExternalCriticalSection);

		//PFX_PRINTF("enter taskId %d count %d stage %d phase %d mEnableCounter %d\n",taskId,mCounter,debug&0xff,debug>>16,mEnableCounter);

		if(mEnableCounter > 0) {
			ResetEvent(mNotifyEvent);
			LeaveCriticalSection(&mExternalCriticalSection);
			WaitForSingleObject(mNotifyEvent,INFINITE); 
			EnterCriticalSection(&mExternalCriticalSection);
		}

		eventId = mCounter;
		mCounter++;

		if(eventId == mMaxCount-1) {
			SetEvent(mRunEvent);

			mEnableCounter = mCounter-1;
			mCounter = 0;
		}
		else {
			ResetEvent(mRunEvent);
			LeaveCriticalSection(&mExternalCriticalSection);
			WaitForSingleObject(mRunEvent,INFINITE); 
			EnterCriticalSection(&mExternalCriticalSection);
			mEnableCounter--;
		}

		if(mEnableCounter == 0) {
			SetEvent(mNotifyEvent);
		}

		//PFX_PRINTF("leave taskId %d count %d stage %d phase %d mEnableCounter %d\n",taskId,mCounter,debug&0xff,debug>>16,mEnableCounter);

		LeaveCriticalSection(&mExternalCriticalSection);
	}

	virtual void setMaxCount(int n) {mMaxCount = n;}
	virtual int  getMaxCount() {return mMaxCount;}
};

class b3Win32CriticalSection : public b3CriticalSection
{
private:
	CRITICAL_SECTION mCriticalSection;

public:
	b3Win32CriticalSection()
	{
		InitializeCriticalSection(&mCriticalSection);
	}

	~b3Win32CriticalSection()
	{
		DeleteCriticalSection(&mCriticalSection);
	}

	unsigned int getSharedParam(int i)
	{
		b3Assert(i>=0&&i<31);
		return mCommonBuff[i+1];
	}

	void setSharedParam(int i,unsigned int p)
	{
		b3Assert(i>=0&&i<31);
		mCommonBuff[i+1] = p;
	}

	void lock()
	{
		EnterCriticalSection(&mCriticalSection);
		mCommonBuff[0] = 1;
	}

	void unlock()
	{
		mCommonBuff[0] = 0;
		LeaveCriticalSection(&mCriticalSection);
	}
};


b3Barrier*	b3Win32ThreadSupport::createBarrier()
{
	unsigned char* mem = (unsigned char*)b3AlignedAlloc(sizeof(b3Win32Barrier),16);
	b3Win32Barrier* barrier = new(mem) b3Win32Barrier();
	barrier->setMaxCount(getNumTasks());
	return barrier;
}

b3CriticalSection* b3Win32ThreadSupport::createCriticalSection()
{
	unsigned char* mem = (unsigned char*) b3AlignedAlloc(sizeof(b3Win32CriticalSection),16);
	b3Win32CriticalSection* cs = new(mem) b3Win32CriticalSection();
	return cs;
}

void b3Win32ThreadSupport::deleteBarrier(b3Barrier* barrier)
{
	barrier->~b3Barrier();
	b3AlignedFree(barrier);
}

void b3Win32ThreadSupport::deleteCriticalSection(b3CriticalSection* criticalSection)
{
	criticalSection->~b3CriticalSection();
	b3AlignedFree(criticalSection);
}





#endif //_WIN32

