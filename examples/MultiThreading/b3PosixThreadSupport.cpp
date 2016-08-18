#ifndef _WIN32
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

#include <stdio.h>
#include "b3PosixThreadSupport.h"
#include <errno.h>
#include <unistd.h>


#define checkPThreadFunction(returnValue) \
    if(0 != returnValue) { \
        printf("PThread problem at line %i in file %s: %i %d\n", __LINE__, __FILE__, returnValue, errno); \
    }

// The number of threads should be equal to the number of available cores
// Todo: each worker should be linked to a single core, using SetThreadIdealProcessor.


b3PosixThreadSupport::b3PosixThreadSupport(ThreadConstructionInfo& threadConstructionInfo)
{
	startThreads(threadConstructionInfo);
}

// cleanup/shutdown Libspe2
b3PosixThreadSupport::~b3PosixThreadSupport()
{
	stopThreads();
}

#if (defined (__APPLE__))
#define NAMED_SEMAPHORES
#endif


static sem_t* createSem(const char* baseName)
{
	static int semCount = 0;
#ifdef NAMED_SEMAPHORES
        /// Named semaphore begin
        char name[32];
        snprintf(name, 32, "/%8.s-%4.d-%4.4d", baseName, getpid(), semCount++);
        sem_t* tempSem = sem_open(name, O_CREAT, 0600, 0);

        if (tempSem != reinterpret_cast<sem_t *>(SEM_FAILED))
        {
//        printf("Created \"%s\" Semaphore %p\n", name, tempSem);
        }
        else
	{
		//printf("Error creating Semaphore %d\n", errno);
		exit(-1);
	}
        /// Named semaphore end
#else
	sem_t* tempSem = new sem_t;
	checkPThreadFunction(sem_init(tempSem, 0, 0));
#endif
	return tempSem;
}

static void destroySem(sem_t* semaphore)
{
#ifdef NAMED_SEMAPHORES
	checkPThreadFunction(sem_close(semaphore));
#else
	checkPThreadFunction(sem_destroy(semaphore));
	delete semaphore;
#endif
}

static void *threadFunction(void *argument)
{

	b3PosixThreadSupport::b3ThreadStatus* status = (b3PosixThreadSupport::b3ThreadStatus*)argument;


	while (1)
	{
            checkPThreadFunction(sem_wait(status->startSemaphore));

		void* userPtr = status->m_userPtr;

		if (userPtr)
		{
			b3Assert(status->m_status);
			status->m_userThreadFunc(userPtr,status->m_lsMemory);
			status->m_status = 2;
			checkPThreadFunction(sem_post(status->m_mainSemaphore));
	                status->threadUsed++;
		} else {
			//exit Thread
			status->m_status = 3;
			checkPThreadFunction(sem_post(status->m_mainSemaphore));
			printf("Thread with taskId %i exiting\n",status->m_taskId);
			break;
		}

	}

	printf("Thread TERMINATED\n");
	return 0;

}

///send messages to SPUs
void b3PosixThreadSupport::runTask(int uiCommand, void* uiArgument0, int taskId)
{
	///	gMidphaseSPU.sendRequest(CMD_GATHER_AND_PROCESS_PAIRLIST, (int) &taskDesc);

	///we should spawn an SPU task here, and in 'waitForResponse' it should wait for response of the (one of) the first tasks that finished



	switch (uiCommand)
	{
	case 	B3_THREAD_SCHEDULE_TASK:
		{
			b3ThreadStatus&	spuStatus = m_activeThreadStatus[taskId];
			b3Assert(taskId >= 0);
			b3Assert(taskId < m_activeThreadStatus.size());

			spuStatus.m_commandId = uiCommand;
			spuStatus.m_status = 1;
			spuStatus.m_userPtr = (void*)uiArgument0;

			// fire event to start new task
			checkPThreadFunction(sem_post(spuStatus.startSemaphore));
			break;
		}
	default:
		{
			///not implemented
			b3Assert(0);
		}

	};


}

///non-blocking test if a task is completed. First implement all versions, and then enable this API
bool b3PosixThreadSupport::isTaskCompleted(int *puiArgument0, int *puiArgument1, int timeOutInMilliseconds)
{

	b3Assert(m_activeThreadStatus.size());

        // wait for any of the threads to finish
	int result = sem_trywait(m_mainSemaphore);
	if (result==0)
    {
        // get at least one thread which has finished
        int last = -1;
        int status = -1;
        for(int t=0; t < int (m_activeThreadStatus.size()); ++t) {
            status = m_activeThreadStatus[t].m_status;
            if(2 == m_activeThreadStatus[t].m_status) {
                last = t;
                break;
            }
        }

        b3ThreadStatus& spuStatus = m_activeThreadStatus[last];

        b3Assert(spuStatus.m_status > 1);
        spuStatus.m_status = 0;

        // need to find an active spu
        b3Assert(last >= 0);

        *puiArgument0 = spuStatus.m_taskId;
        *puiArgument1 = spuStatus.m_status;
        return true;
    }
    return false;
}


///check for messages from SPUs
void b3PosixThreadSupport::waitForResponse( int *puiArgument0,  int *puiArgument1)
{
	///We should wait for (one of) the first tasks to finish (or other SPU messages), and report its response

	///A possible response can be 'yes, SPU handled it', or 'no, please do a PPU fallback'


	b3Assert(m_activeThreadStatus.size());

        // wait for any of the threads to finish
	checkPThreadFunction(sem_wait(m_mainSemaphore));

	// get at least one thread which has finished
        size_t last = -1;

        for(size_t t=0; t < size_t(m_activeThreadStatus.size()); ++t) {
            if(2 == m_activeThreadStatus[t].m_status) {
                last = t;
                break;
            }
        }

	b3ThreadStatus& spuStatus = m_activeThreadStatus[last];

	b3Assert(spuStatus.m_status > 1);
	spuStatus.m_status = 0;

	// need to find an active spu
	b3Assert(last >= 0);

	*puiArgument0 = spuStatus.m_taskId;
	*puiArgument1 = spuStatus.m_status;
}



void b3PosixThreadSupport::startThreads(ThreadConstructionInfo& threadConstructionInfo)
{
        printf("%s creating %i threads.\n", __FUNCTION__, threadConstructionInfo.m_numThreads);
	m_activeThreadStatus.resize(threadConstructionInfo.m_numThreads);

	m_mainSemaphore = createSem("main");
	//checkPThreadFunction(sem_wait(mainSemaphore));

	for (int i=0;i < threadConstructionInfo.m_numThreads;i++)
	{
		printf("starting thread %d\n",i);

		b3ThreadStatus&	spuStatus = m_activeThreadStatus[i];

		spuStatus.startSemaphore = createSem("threadLocal");

                checkPThreadFunction(pthread_create(&spuStatus.thread, NULL, &threadFunction, (void*)&spuStatus));

		spuStatus.m_userPtr=0;

		spuStatus.m_taskId = i;
		spuStatus.m_commandId = 0;
		spuStatus.m_status = 0;
		spuStatus.m_mainSemaphore = m_mainSemaphore;
		spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
		spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;
        spuStatus.threadUsed = 0;

		printf("started thread %d \n",i);

	}

}



///tell the task scheduler we are done with the SPU tasks
void b3PosixThreadSupport::stopThreads()
{
	for(size_t t=0; t < size_t(m_activeThreadStatus.size()); ++t)
	{
            b3ThreadStatus&	spuStatus = m_activeThreadStatus[t];
            printf("%s: Thread %i used: %ld\n", __FUNCTION__, int(t), spuStatus.threadUsed);

	spuStatus.m_userPtr = 0;
 	checkPThreadFunction(sem_post(spuStatus.startSemaphore));
	checkPThreadFunction(sem_wait(m_mainSemaphore));

	printf("destroy semaphore\n");
            destroySem(spuStatus.startSemaphore);
            printf("semaphore destroyed\n");
		checkPThreadFunction(pthread_join(spuStatus.thread,0));

        }
	printf("destroy main semaphore\n");
    destroySem(m_mainSemaphore);
	printf("main semaphore destroyed\n");
	m_activeThreadStatus.clear();
}

class b3PosixCriticalSection : public b3CriticalSection
{
	pthread_mutex_t m_mutex;

public:
	b3PosixCriticalSection()
	{
		pthread_mutex_init(&m_mutex, NULL);
	}
	virtual ~b3PosixCriticalSection()
	{
		pthread_mutex_destroy(&m_mutex);
	}

	B3_ATTRIBUTE_ALIGNED16(unsigned int mCommonBuff[32]);

	virtual unsigned int getSharedParam(int i)
	{
	    if (i<32)
        {
            return mCommonBuff[i];
        } else
        {
            b3Assert(0);
        }
        return 0;
	}
	virtual void setSharedParam(int i,unsigned int p)
	{
	    if (i<32)
        {
            mCommonBuff[i] = p;    
        } else
        {
            b3Assert(0);
        }
	}

	virtual void lock()
	{
		pthread_mutex_lock(&m_mutex);
	}
	virtual void unlock()
	{
		pthread_mutex_unlock(&m_mutex);
	}
};


#if defined(_POSIX_BARRIERS) && (_POSIX_BARRIERS - 20012L) >= 0
/* OK to use barriers on this platform */
class b3PosixBarrier : public b3Barrier
{
	pthread_barrier_t m_barr;
	int m_numThreads;
public:
	b3PosixBarrier()
	:m_numThreads(0)	{	}
	virtual ~b3PosixBarrier()	{
		pthread_barrier_destroy(&m_barr);
	}

	virtual void sync()
	{
		int rc = pthread_barrier_wait(&m_barr);
		if(rc != 0 && rc != PTHREAD_BARRIER_SERIAL_THREAD)
		{
			printf("Could not wait on barrier\n");
			exit(-1);
		}
	}
	virtual void setMaxCount(int numThreads)
	{
		int result = pthread_barrier_init(&m_barr, NULL, numThreads);
		m_numThreads = numThreads;
		b3Assert(result==0);
	}
	virtual int  getMaxCount()
	{
		return m_numThreads;
	}
};
#else
/* Not OK to use barriers on this platform - insert alternate code here */
class b3PosixBarrier : public b3Barrier
{
	pthread_mutex_t m_mutex;
	pthread_cond_t m_cond;

	int m_numThreads;
	int	m_called;

public:
	b3PosixBarrier()
	:m_numThreads(0)
	{
	}
	virtual ~b3PosixBarrier()
	{
		if (m_numThreads>0)
		{
			pthread_mutex_destroy(&m_mutex);
			pthread_cond_destroy(&m_cond);
		}
	}

	virtual void sync()
	{
		pthread_mutex_lock(&m_mutex);
		m_called++;
		if (m_called == m_numThreads) {
			m_called = 0;
			pthread_cond_broadcast(&m_cond);
		} else {
			pthread_cond_wait(&m_cond,&m_mutex);
		}
		pthread_mutex_unlock(&m_mutex);

	}
	virtual void setMaxCount(int numThreads)
	{
		if (m_numThreads>0)
		{
			pthread_mutex_destroy(&m_mutex);
			pthread_cond_destroy(&m_cond);
		}
		m_called = 0;
		pthread_mutex_init(&m_mutex,NULL);
		pthread_cond_init(&m_cond,NULL);
		m_numThreads = numThreads;
	}
	virtual int  getMaxCount()
	{
		return m_numThreads;
	}
};

#endif//_POSIX_BARRIERS



b3Barrier* b3PosixThreadSupport::createBarrier()
{
	b3PosixBarrier* barrier = new b3PosixBarrier();
	barrier->setMaxCount(getNumTasks());
	return barrier;
}

b3CriticalSection* b3PosixThreadSupport::createCriticalSection()
{
	return new b3PosixCriticalSection();
}

void	b3PosixThreadSupport::deleteBarrier(b3Barrier* barrier)
{
	delete barrier;
}

void b3PosixThreadSupport::deleteCriticalSection(b3CriticalSection* cs)
{
	delete cs;
}
#endif //_WIN32
