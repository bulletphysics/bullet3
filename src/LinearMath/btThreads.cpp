/*
Copyright (c) 2003-2014 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btThreads.h"
#include "btQuickprof.h"
#include <algorithm>  // for min and max

#if BT_THREADSAFE

#if BT_USE_OPENMP

#include <omp.h>

#endif // #if BT_USE_OPENMP


#if BT_USE_PPL

// use Microsoft Parallel Patterns Library (installed with Visual Studio 2010 and later)
#include <ppl.h>  // if you get a compile error here, check whether your version of Visual Studio includes PPL
// Visual Studio 2010 and later should come with it
#include <concrtrm.h>  // for GetProcessorCount()

#endif // #if BT_USE_PPL


#if BT_USE_TBB

// use Intel Threading Building Blocks for thread management
#define __TBB_NO_IMPLICIT_LINKAGE 1
#include <tbb/tbb.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#endif // #if BT_USE_TBB


static btITaskScheduler* gBtTaskScheduler;
static int gThreadsRunningCounter = 0;  // useful for detecting if we are trying to do nested parallel-for calls
static btSpinMutex gThreadsRunningCounterMutex;

void btPushThreadsAreRunning()
{
    gThreadsRunningCounterMutex.lock();
    gThreadsRunningCounter++;
    gThreadsRunningCounterMutex.unlock();
}

void btPopThreadsAreRunning()
{
    gThreadsRunningCounterMutex.lock();
    gThreadsRunningCounter--;
    gThreadsRunningCounterMutex.unlock();
}

bool btThreadsAreRunning()
{
    return gThreadsRunningCounter != 0;
}


void btSetTaskScheduler( btITaskScheduler* ts )
{
    gBtTaskScheduler = ts;
}

btITaskScheduler* btGetTaskScheduler()
{
    return gBtTaskScheduler;
}

void btParallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body )
{
    gBtTaskScheduler->parallelFor( iBegin, iEnd, grainSize, body );
}


#if BT_USE_OPENMP
///
/// btTaskSchedulerOpenMP -- OpenMP task scheduler implementation
///
class btTaskSchedulerOpenMP : public btITaskScheduler
{
    int m_numThreads;
public:
    btTaskSchedulerOpenMP() : btITaskScheduler( "OpenMP" )
    {
        m_numThreads = 0;
    }
    virtual int getMaxNumThreads() const BT_OVERRIDE
    {
        return omp_get_max_threads();
    }
    virtual int getNumThreads() const BT_OVERRIDE
    {
        return m_numThreads;
    }
    virtual void setNumThreads( int numThreads ) BT_OVERRIDE
    {
        m_numThreads = ( std::max )( 1, numThreads );
        omp_set_num_threads( m_numThreads );
    }
    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_OpenMP" );
        btPushThreadsAreRunning();
#pragma omp parallel for schedule( static, 1 )
        for ( int i = iBegin; i < iEnd; i += grainSize )
        {
            BT_PROFILE( "OpenMP_job" );
            body.forLoop( i, ( std::min )( i + grainSize, iEnd ) );
        }
        btPopThreadsAreRunning();
    }
};
#endif // #if BT_USE_OPENMP


#if BT_USE_TBB
///
/// btTaskSchedulerTBB -- task scheduler implemented via Intel Threaded Building Blocks
///
class btTaskSchedulerTBB : public btITaskScheduler
{
    int m_numThreads;
    tbb::task_scheduler_init* m_tbbSchedulerInit;

public:
    btTaskSchedulerTBB() : btITaskScheduler( "IntelTBB" )
    {
        m_numThreads = 0;
        m_tbbSchedulerInit = NULL;
    }
    ~btTaskSchedulerTBB()
    {
        if ( m_tbbSchedulerInit )
        {
            delete m_tbbSchedulerInit;
            m_tbbSchedulerInit = NULL;
        }
    }

    virtual int getMaxNumThreads() const BT_OVERRIDE
    {
        return tbb::task_scheduler_init::default_num_threads();
    }
    virtual int getNumThreads() const BT_OVERRIDE
    {
        return m_numThreads;
    }
    virtual void setNumThreads( int numThreads ) BT_OVERRIDE
    {
        m_numThreads = ( std::max )( 1, numThreads );
        if ( m_tbbSchedulerInit )
        {
            delete m_tbbSchedulerInit;
            m_tbbSchedulerInit = NULL;
        }
        m_tbbSchedulerInit = new tbb::task_scheduler_init( m_numThreads );
    }
    struct BodyAdapter
    {
        const btIParallelForBody* mBody;

        void operator()( const tbb::blocked_range<int>& range ) const
        {
            BT_PROFILE( "TBB_job" );
            mBody->forLoop( range.begin(), range.end() );
        }
    };
    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_TBB" );
        // TBB dispatch
        BodyAdapter tbbBody;
        tbbBody.mBody = &body;
        btPushThreadsAreRunning();
        tbb::parallel_for( tbb::blocked_range<int>( iBegin, iEnd, grainSize ),
            tbbBody,
            tbb::simple_partitioner()
        );
        btPopThreadsAreRunning();
    }
};
#endif // #if BT_USE_TBB

#if BT_USE_PPL
///
/// btTaskSchedulerPPL -- task scheduler implemented via Microsoft Parallel Patterns Lib
///
class btTaskSchedulerPPL : public btITaskScheduler
{
    int m_numThreads;
public:
    btTaskSchedulerPPL() : btITaskScheduler( "PPL" )
    {
        m_numThreads = 0;
    }
    virtual int getMaxNumThreads() const BT_OVERRIDE
    {
        return concurrency::GetProcessorCount();
    }
    virtual int getNumThreads() const BT_OVERRIDE
    {
        return m_numThreads;
    }
    virtual void setNumThreads( int numThreads ) BT_OVERRIDE
    {
        m_numThreads = ( std::max )( 1, numThreads );
        using namespace concurrency;
        if ( CurrentScheduler::Id() != -1 )
        {
            CurrentScheduler::Detach();
        }
        SchedulerPolicy policy;
        policy.SetConcurrencyLimits( m_numThreads, m_numThreads );
        CurrentScheduler::Create( policy );
    }
    struct BodyAdapter
    {
        const btIParallelForBody* mBody;
        int mGrainSize;
        int mIndexEnd;

        void operator()( int i ) const
        {
            BT_PROFILE( "PPL_job" );
            mBody->forLoop( i, ( std::min )( i + mGrainSize, mIndexEnd ) );
        }
    };
    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_PPL" );
        // PPL dispatch
        BodyAdapter pplBody;
        pplBody.mBody = &body;
        pplBody.mGrainSize = grainSize;
        pplBody.mIndexEnd = iEnd;
        btPushThreadsAreRunning();
        // note: MSVC 2010 doesn't support partitioner args, so avoid them
        concurrency::parallel_for( iBegin,
            iEnd,
            grainSize,
            pplBody
        );
        btPopThreadsAreRunning();
    }
};
#endif // #if BT_USE_PPL



//
// Lightweight spin-mutex based on atomics
// Using ordinary system-provided mutexes like Windows critical sections was noticeably slower
// presumably because when it fails to lock at first it would sleep the thread and trigger costly
// context switching.
// 

#if __cplusplus >= 201103L

// for anything claiming full C++11 compliance, use C++11 atomics
// on GCC or Clang you need to compile with -std=c++11
#define USE_CPP11_ATOMICS 1

#elif defined( _MSC_VER )

// on MSVC, use intrinsics instead
#define USE_MSVC_INTRINSICS 1

#elif defined( __GNUC__ ) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7))

// available since GCC 4.7 and some versions of clang
// todo: check for clang
#define USE_GCC_BUILTIN_ATOMICS 1

#elif defined( __GNUC__ ) && (__GNUC__ == 4 && __GNUC_MINOR__ >= 1)

// available since GCC 4.1
#define USE_GCC_BUILTIN_ATOMICS_OLD 1

#endif


#if USE_CPP11_ATOMICS

#include <atomic>
#include <thread>

#define THREAD_LOCAL_STATIC thread_local static

bool btSpinMutex::tryLock()
{
    std::atomic<int>* aDest = reinterpret_cast<std::atomic<int>*>(&mLock);
    int expected = 0;
    return std::atomic_compare_exchange_weak_explicit( aDest, &expected, int(1), std::memory_order_acq_rel, std::memory_order_acquire );
}

void btSpinMutex::lock()
{
    // note: this lock does not sleep the thread.
    while (! tryLock())
    {
        // spin
    }
}

void btSpinMutex::unlock()
{
    std::atomic<int>* aDest = reinterpret_cast<std::atomic<int>*>(&mLock);
    std::atomic_store_explicit( aDest, int(0), std::memory_order_release );
}


#elif USE_MSVC_INTRINSICS

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <intrin.h>

#define THREAD_LOCAL_STATIC __declspec( thread ) static


bool btSpinMutex::tryLock()
{
    volatile long* aDest = reinterpret_cast<long*>(&mLock);
    return ( 0 == _InterlockedCompareExchange( aDest, 1, 0) );
}

void btSpinMutex::lock()
{
    // note: this lock does not sleep the thread
    while (! tryLock())
    {
        // spin
    }
}

void btSpinMutex::unlock()
{
    volatile long* aDest = reinterpret_cast<long*>( &mLock );
    _InterlockedExchange( aDest, 0 );
}

#elif USE_GCC_BUILTIN_ATOMICS

#define THREAD_LOCAL_STATIC static __thread


bool btSpinMutex::tryLock()
{
    int expected = 0;
    bool weak = false;
    const int memOrderSuccess = __ATOMIC_ACQ_REL;
    const int memOrderFail = __ATOMIC_ACQUIRE;
    return __atomic_compare_exchange_n(&mLock, &expected, int(1), weak, memOrderSuccess, memOrderFail);
}

void btSpinMutex::lock()
{
    // note: this lock does not sleep the thread
    while (! tryLock())
    {
        // spin
    }
}

void btSpinMutex::unlock()
{
    __atomic_store_n(&mLock, int(0), __ATOMIC_RELEASE);
}

#elif USE_GCC_BUILTIN_ATOMICS_OLD


#define THREAD_LOCAL_STATIC static __thread

bool btSpinMutex::tryLock()
{
    return __sync_bool_compare_and_swap(&mLock, int(0), int(1));
}

void btSpinMutex::lock()
{
    // note: this lock does not sleep the thread
    while (! tryLock())
    {
        // spin
    }
}

void btSpinMutex::unlock()
{
    // write 0
    __sync_fetch_and_and(&mLock, int(0));
}

#else //#elif USE_MSVC_INTRINSICS

#error "no threading primitives defined -- unknown platform"

#endif  //#else //#elif USE_MSVC_INTRINSICS


struct ThreadsafeCounter
{
    unsigned int mCounter;
    btSpinMutex mMutex;

    ThreadsafeCounter() {mCounter=0;}

    unsigned int getNext()
    {
        // no need to optimize this with atomics, it is only called ONCE per thread!
        mMutex.lock();
        unsigned int val = mCounter++;
        mMutex.unlock();
        return val;
    }
};

static ThreadsafeCounter gThreadCounter;


// return a unique index per thread, starting with 0 and counting up
unsigned int btGetCurrentThreadIndex()
{
    const unsigned int kNullIndex = ~0U;
    THREAD_LOCAL_STATIC unsigned int sThreadIndex = kNullIndex;
    if ( sThreadIndex == kNullIndex )
    {
        sThreadIndex = gThreadCounter.getNext();
    }
    return sThreadIndex;
}

bool btIsMainThread()
{
    return btGetCurrentThreadIndex() == 0;
}

#else // #if BT_THREADSAFE

// These should not be called ever
void btSpinMutex::lock()
{
    btAssert(!"unimplemented btSpinMutex::lock() called");
}

void btSpinMutex::unlock()
{
    btAssert(!"unimplemented btSpinMutex::unlock() called");
}

bool btSpinMutex::tryLock()
{
    btAssert(!"unimplemented btSpinMutex::tryLock() called");
    return true;
}

// non-parallel version of btParallelFor
void btParallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body )
{
    btAssert(!"called btParallelFor in non-threadsafe build. enable BT_THREADSAFE");
    body.forLoop( iBegin, iEnd );
}

#endif // #if BT_THREADSAFE


///
/// btTaskSchedulerSequential -- non-threaded implementation of task scheduler
///                              (fallback in case no multi-threaded schedulers are available)
///
class btTaskSchedulerSequential : public btITaskScheduler
{
public:
    btTaskSchedulerSequential() : btITaskScheduler( "Sequential" ) {}
    virtual int getMaxNumThreads() const BT_OVERRIDE { return 1; }
    virtual int getNumThreads() const BT_OVERRIDE { return 1; }
    virtual void setNumThreads( int numThreads ) BT_OVERRIDE {}
    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) BT_OVERRIDE
    {
        BT_PROFILE( "parallelFor_sequential" );
        body.forLoop( iBegin, iEnd );
    }
};

// create a non-threaded task scheduler (always available)
btITaskScheduler* btGetSequentialTaskScheduler()
{
    static btTaskSchedulerSequential sTaskScheduler;
    return &sTaskScheduler;
}


// create an OpenMP task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetOpenMPTaskScheduler()
{
#if BT_USE_OPENMP && BT_THREADSAFE
    static btTaskSchedulerOpenMP sTaskScheduler;
    return &sTaskScheduler;
#else
    return NULL;
#endif
}


// create an Intel TBB task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetTBBTaskScheduler()
{
#if BT_USE_TBB && BT_THREADSAFE
    static btTaskSchedulerTBB sTaskScheduler;
    return &sTaskScheduler;
#else
    return NULL;
#endif
}


// create a PPL task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetPPLTaskScheduler()
{
#if BT_USE_PPL && BT_THREADSAFE
    static btTaskSchedulerPPL sTaskScheduler;
    return &sTaskScheduler;
#else
    return NULL;
#endif
}

