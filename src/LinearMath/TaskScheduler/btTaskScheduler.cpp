
#include "LinearMath/btMinMax.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btThreads.h"
#include "LinearMath/btQuickprof.h"
#include <stdio.h>
#include <algorithm>


typedef void( *btThreadFunc )( void* userPtr, void* lsMemory );
typedef void* ( *btThreadLocalStorageFunc )();

#if BT_THREADSAFE

#include "btThreadSupportInterface.h"




#if defined( _WIN32 )

#define WIN32_LEAN_AND_MEAN

#include <windows.h>

#endif


void btSpinPause()
{
#if defined( _WIN32 )
    YieldProcessor();
#endif
}


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
    virtual void executeJob(int threadId) = 0;
};

class ParallelForJob : public IJob
{
    const btIParallelForBody* mBody;
    int mBegin;
    int mEnd;

public:
    ParallelForJob( int iBegin, int iEnd, const btIParallelForBody& body )
    {
        mBody = &body;
        mBegin = iBegin;
        mEnd = iEnd;
    }
    virtual void executeJob(int threadId) BT_OVERRIDE
    {
        BT_PROFILE( "executeJob" );

        // call the functor body to do the work
        mBody->forLoop( mBegin, mEnd );
    }
};

static const int kCacheLineSize = 64;

struct ThreadLocalSum
{
    btScalar mSum;
    char mCachePadding[ kCacheLineSize - sizeof( btScalar ) ];
};

class ParallelSumJob : public IJob
{
    const btIParallelSumBody* mBody;
    ThreadLocalSum* mSumArray;
    int mBegin;
    int mEnd;

public:
    ParallelSumJob( int iBegin, int iEnd, const btIParallelSumBody& body, ThreadLocalSum* sums )
    {
        mBody = &body;
        mSumArray = sums;
        mBegin = iBegin;
        mEnd = iEnd;
    }
    virtual void executeJob( int threadId ) BT_OVERRIDE
    {
        BT_PROFILE( "executeJob" );

        // call the functor body to do the work
        btScalar val = mBody->sumLoop( mBegin, mEnd );
        // by truncating bits of the result, we can make the parallelSum deterministic (at the expense of precision)
        const float TRUNC_SCALE = float(1<<19);
        val = floor(val*TRUNC_SCALE+0.5f)/TRUNC_SCALE;  // truncate some bits
        mSumArray[threadId].mSum += val;
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
        m_workersShouldSleep = false;
        m_useSpinMutex = false;
        m_coolDownTime = 1000; // 1000 microseconds
    }
    btCriticalSection* m_queueLock;
    btSpinMutex m_mutex;
    volatile bool m_workersShouldCheckQueue;
    volatile bool m_workersShouldSleep;

    btAlignedObjectArray<IJob*> m_jobQueue;
    bool m_queueIsEmpty;
    int m_tailIndex;
    int m_headIndex;
    bool m_useSpinMutex;
    unsigned int m_coolDownTime;
    btClock m_clock;

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
    int numJobsFinished;
    btSpinMutex m_mutex;
};


static void WorkerThreadFunc( void* userPtr, void* lsMemory )
{
    BT_PROFILE( "WorkerThreadFunc" );
    WorkerThreadLocalStorage* localStorage = (WorkerThreadLocalStorage*) lsMemory;
    JobContext* jobContext = (JobContext*) userPtr;

    bool shouldSleep = false;
    while (! shouldSleep)
    {
        // do work
        localStorage->m_mutex.lock();
        while ( IJob* job = jobContext->consumeJob() )
        {
            localStorage->status = WorkerThreadStatus::kWorking;
            job->executeJob( localStorage->threadId );
            localStorage->numJobsFinished++;
        }
        localStorage->status = WorkerThreadStatus::kWaitingForWork;
        localStorage->m_mutex.unlock();
        unsigned long long int clockStart = jobContext->m_clock.getTimeMicroseconds();
        // while queue is empty,
        while (jobContext->m_queueIsEmpty)
        {
            // todo: spin wait a bit to avoid hammering the empty queue
            btSpinPause();
            if ( jobContext->m_workersShouldSleep )
            {
                shouldSleep = true;
                break;
            }
            // if jobs are incoming,
            if (jobContext->m_workersShouldCheckQueue)
            {
                clockStart = jobContext->m_clock.getTimeMicroseconds(); // reset clock
            }
            else
            {
                // if no jobs incoming and queue has been empty for the cooldown time, sleep
                unsigned long long int timeElapsed = jobContext->m_clock.getTimeMicroseconds() - clockStart;
                if (timeElapsed > jobContext->m_coolDownTime)
                {
                    shouldSleep = true;
                    break;
                }
            }
        }
    }

    // go idle
    localStorage->m_mutex.lock();
    localStorage->status = WorkerThreadStatus::kSleeping;
    localStorage->m_mutex.unlock();
}


static void* WorkerThreadAllocFunc()
{
    return new WorkerThreadLocalStorage;
}



class btTaskSchedulerDefault : public btITaskScheduler
{
    JobContext m_jobContext;
    btThreadSupportInterface* m_threadSupport;
    btAlignedObjectArray<char> m_jobMem;
    btAlignedObjectArray<char> m_threadLocalMem;
    btSpinMutex m_antiNestingLock;  // prevent nested parallel-for
    int m_numThreads;
    int m_numWorkerThreads;
    int m_maxNumThreads;
    int m_numJobs;
public:

    btTaskSchedulerDefault() : btITaskScheduler("ThreadSupport")
    {
        m_threadSupport = NULL;
    }

    virtual ~btTaskSchedulerDefault()
    {
        shutdown();
    }

    void init()
    {
        btThreadSupportInterface::ConstructionInfo constructionInfo( "TaskScheduler", WorkerThreadFunc, WorkerThreadAllocFunc );
        m_threadSupport = btThreadSupportInterface::create( constructionInfo );

        m_numWorkerThreads = m_threadSupport->getNumWorkerThreads();
        m_maxNumThreads = m_threadSupport->getNumWorkerThreads() + 1;
        m_numThreads = m_maxNumThreads;
        m_jobContext.m_queueLock = m_threadSupport->createCriticalSection();
        for ( int i = 0; i < m_numWorkerThreads; i++ )
        {
            WorkerThreadLocalStorage* storage = (WorkerThreadLocalStorage*) m_threadSupport->getThreadLocalMemory( i );
            btAssert( storage );
            storage->threadId = i + 1;  // workers start at 1
            storage->status = WorkerThreadStatus::kSleeping;
        }
        setWorkersActive( false ); // no work for them yet
        setNumThreads( m_threadSupport->getCacheFriendlyNumThreads() );
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
        return m_maxNumThreads;
    }

    virtual int getNumThreads() const BT_OVERRIDE
    {
        return m_numThreads;
    }

    virtual void setNumThreads( int numThreads ) BT_OVERRIDE
    {
        m_numThreads = btMax( btMin(numThreads, int(m_maxNumThreads)), 1 );
        m_numWorkerThreads = m_numThreads - 1;
    }

    void waitJobs()
    {
        BT_PROFILE( "waitJobs" );
        // have the main thread work until the job queue is empty
        int numMainThreadJobsFinished = 0;
        while ( IJob* job = m_jobContext.consumeJob() )
        {
            job->executeJob( 0 );
            numMainThreadJobsFinished++;
        }
        // done with jobs for now, tell workers to rest
        setWorkersActive( false );

        unsigned long long int clockStart = m_jobContext.m_clock.getTimeMicroseconds();
        // wait for workers to finish any jobs in progress
        while ( true )
        {
            int numWorkerJobsFinished = 0;
            for ( int iWorker = 0; iWorker < m_numWorkerThreads; ++iWorker )
            {
                WorkerThreadLocalStorage* storage = static_cast<WorkerThreadLocalStorage*>( m_threadSupport->getThreadLocalMemory( iWorker ) );
                storage->m_mutex.lock();
                numWorkerJobsFinished += storage->numJobsFinished;
                storage->m_mutex.unlock();
            }
            if (numWorkerJobsFinished + numMainThreadJobsFinished == m_numJobs)
            {
                break;
            }
            unsigned long long int timeElapsed = m_jobContext.m_clock.getTimeMicroseconds() - clockStart;
            btAssert(timeElapsed < 1000);
            if (timeElapsed > 100000)
            {
                break;
            }
            btSpinPause();
        }
    }

    void wakeWorkers(int numWorkersToWake)
    {
        BT_PROFILE( "wakeWorkers" );
        btAssert( m_jobContext.m_workersShouldCheckQueue );
        int numDesiredWorkers = btMin(numWorkersToWake, m_numWorkerThreads);
        int numActiveWorkers = 0;
        for ( int iWorker = 0; iWorker < m_numWorkerThreads; ++iWorker )
        {
            // note this count of active workers is not necessarily totally reliable, because a worker thread could be
            // just about to put itself to sleep. So we may on occasion fail to wake up all the workers. It should be rare.
            WorkerThreadLocalStorage* storage = static_cast<WorkerThreadLocalStorage*>( m_threadSupport->getThreadLocalMemory( iWorker ) );
            if (storage->status != WorkerThreadStatus::kSleeping)
            {
                numActiveWorkers++;
            }
        }
        for ( int iWorker = 0; iWorker < m_numWorkerThreads && numActiveWorkers < numDesiredWorkers; ++iWorker )
        {
            WorkerThreadLocalStorage* storage = static_cast<WorkerThreadLocalStorage*>( m_threadSupport->getThreadLocalMemory( iWorker ) );
            if (storage->status == WorkerThreadStatus::kSleeping)
            {
                m_threadSupport->runTask( iWorker, &m_jobContext );
                numActiveWorkers++;
            }
        }
    }

    void waitForWorkersToSleep()
    {
        BT_PROFILE( "waitForWorkersToSleep" );
        m_jobContext.m_workersShouldSleep = true;
        m_threadSupport->waitForAllTasks();
        for ( int i = 0; i < m_numWorkerThreads; i++ )
        {
            WorkerThreadLocalStorage* storage = static_cast<WorkerThreadLocalStorage*>( m_threadSupport->getThreadLocalMemory(i) );
            btAssert( storage );
            btAssert( storage->status == WorkerThreadStatus::kSleeping );
        }
    }

    virtual void sleepWorkerThreadsHint() BT_OVERRIDE
    {
        BT_PROFILE( "sleepWorkerThreadsHint" );
        // hint the task scheduler that we may not be using these threads for a little while
        m_jobContext.m_workersShouldSleep = true;
    }

    void prepareWorkerThreads()
    {
        for ( int iWorker = 0; iWorker < m_numWorkerThreads; ++iWorker )
        {
            WorkerThreadLocalStorage* storage = static_cast<WorkerThreadLocalStorage*>( m_threadSupport->getThreadLocalMemory( iWorker ) );
            storage->m_mutex.lock();
            storage->numJobsFinished = 0;
            storage->m_mutex.unlock();
        }
        m_jobContext.m_workersShouldSleep = false;
        setWorkersActive( true );
    }

    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_ThreadSupport" );
        btAssert( iEnd >= iBegin );
        btAssert( grainSize >= 1 );
        int iterationCount = iEnd - iBegin;
        if ( iterationCount > grainSize && m_numWorkerThreads > 0 && m_antiNestingLock.tryLock() )
        {
            typedef ParallelForJob JobType;
            int jobCount = ( iterationCount + grainSize - 1 ) / grainSize;
            m_numJobs = jobCount;
            btAssert( jobCount >= 2 );  // need more than one job for multithreading
            int jobSize = sizeof( JobType );
            int jobBufSize = jobSize * jobCount;
            // make sure we have enough memory allocated to store jobs
            if ( jobBufSize > m_jobMem.size() )
            {
                m_jobMem.resize( jobBufSize );
            }
            // make sure job queue is big enough
            if ( jobCount > m_jobContext.m_jobQueue.capacity() )
            {
                m_jobContext.m_jobQueue.reserve( jobCount );
            }

            m_jobContext.clearQueue();
            // prepare worker threads for incoming work
            prepareWorkerThreads();
            // submit all of the jobs
            int iJob = 0;
            JobType* jobs = reinterpret_cast<JobType*>( &m_jobMem[ 0 ] );
            for ( int i = iBegin; i < iEnd; i += grainSize )
            {
                btAssert( iJob < jobCount );
                int iE = btMin( i + grainSize, iEnd );
                JobType& job = jobs[ iJob ];
                new ( (void*) &job ) ParallelForJob( i, iE, body );  // placement new
                m_jobContext.submitJob( &job );
                iJob++;
            }
            wakeWorkers( jobCount - 1 );

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
    virtual btScalar parallelSum( int iBegin, int iEnd, int grainSize, const btIParallelSumBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelSum_ThreadSupport" );
        btAssert( iEnd >= iBegin );
        btAssert( grainSize >= 1 );
        int iterationCount = iEnd - iBegin;
        if ( iterationCount > grainSize && m_numWorkerThreads > 0 && m_antiNestingLock.tryLock() )
        {
            typedef ParallelSumJob JobType;
            int jobCount = ( iterationCount + grainSize - 1 ) / grainSize;
            m_numJobs = jobCount;
            btAssert( jobCount >= 2 );  // need more than one job for multithreading
            int jobSize = sizeof( JobType );
            int jobBufSize = jobSize * jobCount;
            // make sure we have enough memory allocated to store jobs
            if ( jobBufSize > m_jobMem.size() )
            {
                m_jobMem.resize( jobBufSize );
            }
            // make sure job queue is big enough
            if ( jobCount > m_jobContext.m_jobQueue.capacity() )
            {
                m_jobContext.m_jobQueue.reserve( jobCount );
            }
            // make sure thread local area is big enough
            int threadLocalSize = m_numThreads * sizeof( ThreadLocalSum );
            if ( threadLocalSize > m_threadLocalMem.size() )
            {
                m_threadLocalMem.resize( threadLocalSize );
            }
            // initialize summation
            ThreadLocalSum* threadLocalSum = reinterpret_cast<ThreadLocalSum*>( &m_threadLocalMem[ 0 ] );
            for ( int iThread = 0; iThread < m_numThreads; ++iThread )
            {
                threadLocalSum[ iThread ].mSum = btScalar( 0 );
            }

            m_jobContext.clearQueue();
            // prepare worker threads for incoming work
            prepareWorkerThreads();
            // submit all of the jobs
            int iJob = 0;
            JobType* jobs = reinterpret_cast<JobType*>( &m_jobMem[ 0 ] );
            for ( int i = iBegin; i < iEnd; i += grainSize )
            {
                btAssert( iJob < jobCount );
                int iE = btMin( i + grainSize, iEnd );
                JobType& job = jobs[ iJob ];
                new ( (void*) &job ) ParallelSumJob( i, iE, body, threadLocalSum );  // placement new
                m_jobContext.submitJob( &job );
                iJob++;
            }
            wakeWorkers( jobCount - 1 );

            // put the main thread to work on emptying the job queue and then wait for all workers to finish
            waitJobs();

            // add up all the thread sums
            btScalar sum = btScalar(0);
            for ( int iThread = 0; iThread < m_numThreads; ++iThread )
            {
                sum += threadLocalSum[ iThread ].mSum;
            }
            m_antiNestingLock.unlock();
            return sum;
        }
        else
        {
            BT_PROFILE( "parallelSum_mainThread" );
            // just run on main thread
            return body.sumLoop( iBegin, iEnd );
        }
    }
};



btITaskScheduler* btCreateDefaultTaskScheduler()
{
    btTaskSchedulerDefault* ts = new btTaskSchedulerDefault();
    ts->init();
    return ts;
}

#else // #if BT_THREADSAFE

btITaskScheduler* btCreateDefaultTaskScheduler()
{
    return NULL;
}

#endif // #else // #if BT_THREADSAFE