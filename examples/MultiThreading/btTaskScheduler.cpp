
#include "LinearMath/btTransform.h"
#include "../Utils/b3Clock.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btThreads.h"
#include "LinearMath/btQuickprof.h"
#include <stdio.h>
#include <algorithm>


typedef void( *btThreadFunc )( void* userPtr, void* lsMemory );
typedef void* ( *btThreadLocalStorageFunc )();

#if BT_THREADSAFE

#if defined( _WIN32 )

#include "b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport( int numThreads, btThreadFunc threadFunc, btThreadLocalStorageFunc localStoreFunc, const char* uniqueName )
{
    b3Win32ThreadSupport::Win32ThreadConstructionInfo constructionInfo( uniqueName, threadFunc, localStoreFunc, numThreads );
    //constructionInfo.m_priority = 0;  // highest priority (the default) -- can cause erratic performance when numThreads > numCores
    //                                     we don't want worker threads to be higher priority than the main thread or the main thread could get
    //                                     totally shut out and unable to tell the workers to stop
    constructionInfo.m_priority = -1;  // normal priority
    b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport( constructionInfo );
    return threadSupport;
}

#else // #if defined( _WIN32 )

#include "b3PosixThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport( int numThreads, btThreadFunc threadFunc, btThreadLocalStorageFunc localStoreFunc, const char* uniqueName)
{
    b3PosixThreadSupport::ThreadConstructionInfo constructionInfo( uniqueName, threadFunc, localStoreFunc, numThreads );
    b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport( constructionInfo );
    return threadSupport;
}

#endif // #else // #if defined( _WIN32 )


///
/// getNumHardwareThreads()
///
///
/// https://stackoverflow.com/questions/150355/programmatically-find-the-number-of-cores-on-a-machine
///
#if __cplusplus >= 201103L

#include <thread>

int getNumHardwareThreads()
{
    return std::thread::hardware_concurrency();
}

#elif defined( _WIN32 )

#define WIN32_LEAN_AND_MEAN

#include <windows.h>

int getNumHardwareThreads()
{
    // caps out at 32
    SYSTEM_INFO info;
    GetSystemInfo( &info );
    return info.dwNumberOfProcessors;
}

#else

int getNumHardwareThreads()
{
    return 0;  // don't know
}

#endif


struct WorkerThreadStatus
{
    enum Type
    {
        kInvalid,
        kWaitingForWork,
        kWorking,
        kSleeping,
    };
};


struct IJob
{
    virtual void executeJob() = 0;
};

class ParallelForJob : public IJob
{
    const btIParallelForBody* mBody;
    int mBegin;
    int mEnd;

public:
    ParallelForJob()
    {
        mBody = NULL;
        mBegin = 0;
        mEnd = 0;
    }
    void init( int iBegin, int iEnd, const btIParallelForBody& body )
    {
        mBody = &body;
        mBegin = iBegin;
        mEnd = iEnd;
    }
    virtual void executeJob() BT_OVERRIDE
    {
        BT_PROFILE( "executeJob" );

        // call the functor body to do the work
        mBody->forLoop( mBegin, mEnd );
    }
};


struct JobContext
{
    JobContext()
    {
        m_queueLock = NULL;
        m_headIndex = 0;
        m_tailIndex = 0;
        m_workersShouldCheckQueue = false;
        m_useSpinMutex = false;
    }
    b3CriticalSection* m_queueLock;
    btSpinMutex m_mutex;
    volatile bool m_workersShouldCheckQueue;

    btAlignedObjectArray<IJob*> m_jobQueue;
    bool m_queueIsEmpty;
    int m_tailIndex;
    int m_headIndex;
    bool m_useSpinMutex;

    void lockQueue()
    {
        if ( m_useSpinMutex )
        {
            m_mutex.lock();
        }
        else
        {
            m_queueLock->lock();
        }
    }
    void unlockQueue()
    {
        if ( m_useSpinMutex )
        {
            m_mutex.unlock();
        }
        else
        {
            m_queueLock->unlock();
        }
    }
    void clearQueue()
    {
        lockQueue();
        m_headIndex = 0;
        m_tailIndex = 0;
        m_queueIsEmpty = true;
        unlockQueue();
        m_jobQueue.resizeNoInitialize( 0 );
    }
    void submitJob( IJob* job )
    {
        m_jobQueue.push_back( job );
        lockQueue();
        m_tailIndex++;
        m_queueIsEmpty = false;
        unlockQueue();
    }
    IJob* consumeJob()
    {
        if ( m_queueIsEmpty )
        {
            // lock free path. even if this is taken erroneously it isn't harmful
            return NULL;
        }
        IJob* job = NULL;
        lockQueue();
        if ( !m_queueIsEmpty )
        {
            job = m_jobQueue[ m_headIndex++ ];
            if ( m_headIndex == m_tailIndex )
            {
                m_queueIsEmpty = true;
            }
        }
        unlockQueue();
        return job;
    }
};


struct WorkerThreadLocalStorage
{
    int threadId;
    WorkerThreadStatus::Type status;
};


static void WorkerThreadFunc( void* userPtr, void* lsMemory )
{
    BT_PROFILE( "WorkerThreadFunc" );
    WorkerThreadLocalStorage* localStorage = (WorkerThreadLocalStorage*) lsMemory;
    localStorage->status = WorkerThreadStatus::kWaitingForWork;
    //printf( "WorkerThreadFunc: worker %d start working\n", localStorage->threadId );

    JobContext* jobContext = (JobContext*) userPtr;

    while ( jobContext->m_workersShouldCheckQueue )
    {
        if ( IJob* job = jobContext->consumeJob() )
        {
            localStorage->status = WorkerThreadStatus::kWorking;
            job->executeJob();
            localStorage->status = WorkerThreadStatus::kWaitingForWork;
        }
        else
        {
            // todo: spin wait a bit to avoid hammering the empty queue
        }
    }

    //printf( "WorkerThreadFunc stop working\n" );
    localStorage->status = WorkerThreadStatus::kSleeping;
    // go idle
}


static void* WorkerThreadAllocFunc()
{
    return new WorkerThreadLocalStorage;
}



class btTaskSchedulerDefault : public btITaskScheduler
{
    JobContext m_jobContext;
    b3ThreadSupportInterface* m_threadSupport;
    btAlignedObjectArray<ParallelForJob> m_jobs;
    btSpinMutex m_antiNestingLock;  // prevent nested parallel-for
    int m_numThreads;
    int m_numWorkerThreads;
    int m_numWorkersRunning;
public:

    btTaskSchedulerDefault() : btITaskScheduler("ThreadSupport")
    {
        m_threadSupport = NULL;
        m_numThreads = getNumHardwareThreads();
        // if can't detect number of cores,
        if ( m_numThreads == 0 )
        {
            // take a guess
            m_numThreads = 4;
        }
        m_numWorkerThreads = m_numThreads - 1;
        m_numWorkersRunning = 0;
    }

    virtual ~btTaskSchedulerDefault()
    {
        shutdown();
    }

    void init()
    {
        int maxNumWorkerThreads = BT_MAX_THREAD_COUNT - 1;
        m_threadSupport = createThreadSupport( maxNumWorkerThreads, WorkerThreadFunc, WorkerThreadAllocFunc, "TaskScheduler" );
        m_jobContext.m_queueLock = m_threadSupport->createCriticalSection();
        for ( int i = 0; i < maxNumWorkerThreads; i++ )
        {
            WorkerThreadLocalStorage* storage = (WorkerThreadLocalStorage*) m_threadSupport->getThreadLocalMemory( i );
            btAssert( storage );
            storage->threadId = i;
            storage->status = WorkerThreadStatus::kSleeping;
        }
        setWorkersActive( false ); // no work for them yet
    }

    virtual void shutdown()
    {
        setWorkersActive( false );
        waitForWorkersToSleep();
        m_threadSupport->deleteCriticalSection( m_jobContext.m_queueLock );
        m_jobContext.m_queueLock = NULL;

        delete m_threadSupport;
        m_threadSupport = NULL;
    }

    void setWorkersActive( bool active )
    {
        m_jobContext.m_workersShouldCheckQueue = active;
    }

    virtual int getMaxNumThreads() const BT_OVERRIDE
    {
        return BT_MAX_THREAD_COUNT;
    }

    virtual int getNumThreads() const BT_OVERRIDE
    {
        return m_numThreads;
    }

    virtual void setNumThreads( int numThreads ) BT_OVERRIDE
    {
        m_numThreads = btMax( btMin(numThreads, int(BT_MAX_THREAD_COUNT)), 1 );
        m_numWorkerThreads = m_numThreads - 1;
    }

    void waitJobs()
    {
        BT_PROFILE( "waitJobs" );
        // have the main thread work until the job queue is empty
        for ( ;; )
        {
            if ( IJob* job = m_jobContext.consumeJob() )
            {
                job->executeJob();
            }
            else
            {
                break;
            }
        }
        // done with jobs for now, tell workers to rest
        setWorkersActive( false );
        waitForWorkersToSleep();
    }

    void wakeWorkers()
    {
        BT_PROFILE( "wakeWorkers" );
        btAssert( m_jobContext.m_workersShouldCheckQueue );
        // tell each worker thread to start working
        for ( int i = 0; i < m_numWorkerThreads; i++ )
        {
            m_threadSupport->runTask( B3_THREAD_SCHEDULE_TASK, &m_jobContext, i );
            m_numWorkersRunning++;
        }
    }

    void waitForWorkersToSleep()
    {
        BT_PROFILE( "waitForWorkersToSleep" );
        while ( m_numWorkersRunning > 0 )
        {
            int iThread;
            int threadStatus;
            m_threadSupport->waitForResponse( &iThread, &threadStatus );  // wait for worker threads to finish working
            m_numWorkersRunning--;
        }
        //m_threadSupport->waitForAllTasksToComplete();
        for ( int i = 0; i < m_numWorkerThreads; i++ )
        {
            //m_threadSupport->waitForTaskCompleted( i );
            WorkerThreadLocalStorage* storage = (WorkerThreadLocalStorage*) m_threadSupport->getThreadLocalMemory( i );
            btAssert( storage );
            btAssert( storage->status == WorkerThreadStatus::kSleeping );
        }
    }

    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_ThreadSupport" );
        btAssert( iEnd >= iBegin );
        btAssert( grainSize >= 1 );
        int iterationCount = iEnd - iBegin;
        if ( iterationCount > grainSize && m_numWorkerThreads > 0 && m_antiNestingLock.tryLock() )
        {
            int jobCount = ( iterationCount + grainSize - 1 ) / grainSize;
            btAssert( jobCount >= 2 );  // need more than one job for multithreading
            if ( jobCount > m_jobs.size() )
            {
                m_jobs.resize( jobCount );
            }
            if ( jobCount > m_jobContext.m_jobQueue.capacity() )
            {
                m_jobContext.m_jobQueue.reserve( jobCount );
            }

            m_jobContext.clearQueue();
            // prepare worker threads for incoming work
            setWorkersActive( true );
            wakeWorkers();
            // submit all of the jobs
            int iJob = 0;
            for ( int i = iBegin; i < iEnd; i += grainSize )
            {
                btAssert( iJob < jobCount );
                int iE = btMin( i + grainSize, iEnd );
                ParallelForJob& job = m_jobs[ iJob ];
                job.init( i, iE, body );
                m_jobContext.submitJob( &job );
                iJob++;
            }

            // put the main thread to work on emptying the job queue and then wait for all workers to finish
            waitJobs();
            m_antiNestingLock.unlock();
        }
        else
        {
            BT_PROFILE( "parallelFor_mainThread" );
            // just run on main thread
            body.forLoop( iBegin, iEnd );
        }
    }
};



btITaskScheduler* createDefaultTaskScheduler()
{
    btTaskSchedulerDefault* ts = new btTaskSchedulerDefault();
    ts->init();
    return ts;
}

#else // #if BT_THREADSAFE

btITaskScheduler* createDefaultTaskScheduler()
{
    return NULL;
}

#endif // #else // #if BT_THREADSAFE