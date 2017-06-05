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



#ifndef BT_THREADS_H
#define BT_THREADS_H

#include "btScalar.h" // has definitions like SIMD_FORCE_INLINE

#if defined (_MSC_VER) && _MSC_VER >= 1600
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#endif

#ifndef BT_OVERRIDE
#define BT_OVERRIDE
#endif

///
/// btSpinMutex -- lightweight spin-mutex implemented with atomic ops, never puts
///               a thread to sleep because it is designed to be used with a task scheduler
///               which has one thread per core and the threads don't sleep until they
///               run out of tasks. Not good for general purpose use.
///
class btSpinMutex
{
    int mLock;

public:
    btSpinMutex()
    {
        mLock = 0;
    }
    void lock();
    void unlock();
    bool tryLock();
};

#if BT_THREADSAFE

// for internal Bullet use only
SIMD_FORCE_INLINE void btMutexLock( btSpinMutex* mutex )
{
    mutex->lock();
}

SIMD_FORCE_INLINE void btMutexUnlock( btSpinMutex* mutex )
{
    mutex->unlock();
}

SIMD_FORCE_INLINE bool btMutexTryLock( btSpinMutex* mutex )
{
    return mutex->tryLock();
}

// for internal use only
bool btIsMainThread();
bool btThreadsAreRunning();
unsigned int btGetCurrentThreadIndex();
const unsigned int BT_MAX_THREAD_COUNT = 64;

#else

// for internal Bullet use only
// if BT_THREADSAFE is undefined or 0, should optimize away to nothing
SIMD_FORCE_INLINE void btMutexLock( btSpinMutex* ) {}
SIMD_FORCE_INLINE void btMutexUnlock( btSpinMutex* ) {}
SIMD_FORCE_INLINE bool btMutexTryLock( btSpinMutex* ) {return true;}
SIMD_FORCE_INLINE bool btThreadsAreRunning() { return false;}
#endif

//
// btIParallelForBody -- subclass this to express work that can be done in parallel
//
class btIParallelForBody
{
public:
    virtual void forLoop( int iBegin, int iEnd ) const = 0;
};

//
// btITaskScheduler -- subclass this to implement a task scheduler that can dispatch work to
//                     worker threads
//
class btITaskScheduler
{
    const char* m_name;
public:
    btITaskScheduler( const char* name ) : m_name( name ) {}
    const char* getName() const { return m_name; }

    virtual ~btITaskScheduler() {}
    virtual int getMaxNumThreads() const = 0;
    virtual int getNumThreads() const = 0;
    virtual void setNumThreads( int numThreads ) = 0;
    virtual void parallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body ) = 0;
};

// set the task scheduler to use for all calls to btParallelFor()
// NOTE: you must set this prior to using any of the multi-threaded "Mt" classes
void btSetTaskScheduler( btITaskScheduler* ts );

// get the current task scheduler
btITaskScheduler* btGetTaskScheduler();

// get non-threaded task scheduler (always available)
btITaskScheduler* btGetSequentialTaskScheduler();

// get OpenMP task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetOpenMPTaskScheduler();

// get Intel TBB task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetTBBTaskScheduler();

// get PPL task scheduler (if available, otherwise returns null)
btITaskScheduler* btGetPPLTaskScheduler();

// btParallelFor -- call this to dispatch work like a for-loop
//                 (iterations may be done out of order, so no dependencies are allowed)
void btParallelFor( int iBegin, int iEnd, int grainSize, const btIParallelForBody& body );


#endif //BT_THREADS_H
