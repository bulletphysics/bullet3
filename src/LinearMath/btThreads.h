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
class btVector3;

#if BT_THREADSAFE

// for internal Bullet use only
class btThreadLocalPtr
{
    int mIndex;

public:
    btThreadLocalPtr();
    ~btThreadLocalPtr();
    void setPtr( void* ptr );
    void* getPtr();
};

// Threadsafe Vector3 addition -- not atomic, but multiple threads may add into a common accumulator without corruption
void btThreadsafeVector3Add( btVector3* dest, const btVector3& delta );

#endif // #if BT_THREADSAFE


class btMutex;

#if BT_THREADSAFE

// for internal Bullet use only
void btMutexLock( btMutex* );
void btMutexUnlock( btMutex* );

//bool btMutexTryLock( btMutex* );
btMutex* btMutexCreate();
void btMutexDestroy( btMutex* mutex );

unsigned int btGetCurrentThreadId();
bool btThreadsAreRunning();  // useful for debugging and asserts

#else

// for internal Bullet use only
// if BT_THREADSAFE is 0, should optimize away to nothing
SIMD_FORCE_INLINE void btMutexLock( btMutex* ) {}
SIMD_FORCE_INLINE void btMutexUnlock( btMutex* ) {}
//SIMD_FORCE_INLINE bool btMutexTryLock( btMutex* ) { return true; }
SIMD_FORCE_INLINE btMutex* btMutexCreate() { return NULL; }
SIMD_FORCE_INLINE void btMutexDestroy( btMutex* mutex ) {}

SIMD_FORCE_INLINE unsigned int btGetCurrentThreadId() { return 0; }
SIMD_FORCE_INLINE bool btThreadsAreRunning() {return false;}

#endif

//
// btSetThreadsAreRunning( bool )
//
// [Optional] User may set this just before tasks are started and clear it after tasks have finished
// to help identify threading bugs in debug builds.
//
void btSetThreadsAreRunning( bool f );

typedef void (*btMutexLockFunc) ( btMutex* );

// override the mutex lock/unlock functions
void btSetMutexLockFunc( btMutexLockFunc lockFunc );
void btSetMutexUnlockFunc( btMutexLockFunc unlockFunc );

// don't call these directly except within external wrapper functions that have hooked the lock/unlock functions
void btInternalMutexLock( btMutex* );
void btInternalMutexUnlock( btMutex* );

#endif //BT_THREADS_H
