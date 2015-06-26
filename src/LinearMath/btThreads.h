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

//#undef BT_THREADSAFE

#include "btScalar.h" // has definitions like SIMD_FORCE_INLINE

class btMutex
{
    unsigned char mLock;  // assumption: bytes are atomic to read and write on all architectures

public:
    btMutex()
    {
        mLock = 0;
    }
    void lock();
    void unlock();
    bool tryLock();
};

#if BT_THREADSAFE

// for internal Bullet use only
void btMutexLock( btMutex* );
void btMutexUnlock( btMutex* );
bool btMutexTryLock( btMutex* );

bool btIsMainThread();

#else

// for internal Bullet use only
// if BT_THREADSAFE is 0, should optimize away to nothing
SIMD_FORCE_INLINE void btMutexLock( btMutex* ) {}
SIMD_FORCE_INLINE void btMutexUnlock( btMutex* ) {}
SIMD_FORCE_INLINE bool btMutexTryLock( btMutex* ) {return true;}

#endif

//
// btPushThreadsAreRunning()/btPopThreadsAreRunning()
//
// [Optional] User may push this just before tasks are started and pop it after tasks have finished
// to help identify threading bugs in debug builds.  It needs to be push/pop rather than set/clear
// because of the potential for nested task launching (launching tasks from within a task).
//
void btPushThreadsAreRunning();
void btPopThreadsAreRunning();

bool btThreadsAreRunning();  // useful for debugging and asserts

typedef void( *btMutexLockFunc ) ( btMutex* );

// override the mutex lock function (useful for profiling)
void btSetMutexLockFunc( btMutexLockFunc lockFunc );

// don't call directly except within an external wrapper function that has hooked the lock function
void btMutexLockInternal( btMutex* );

#endif //BT_THREADS_H
