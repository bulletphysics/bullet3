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

#if BT_THREADSAFE

#if defined(WIN32) || defined(_WIN32)

#define WIN32_LEAN_AND_MEAN

#include <windows.h>


class btMutex : public CRITICAL_SECTION
{
};

void btMutexLock( btMutex* mutex )
{
    EnterCriticalSection( mutex );
}

void btMutexUnlock( btMutex* mutex )
{
    LeaveCriticalSection( mutex );
}

bool btMutexTryLock( btMutex* mutex )
{
    BOOL ret = TryEnterCriticalSection( mutex );
    return ret != 0;
}

btMutex* btMutexCreate()
{
    btMutex* mutex = new btMutex;
    InitializeCriticalSection( mutex );
    return mutex;
}

void btMutexDestroy( btMutex* mutex )
{
    DeleteCriticalSection( mutex );
    delete mutex;
}



btThreadLocalPtr::btThreadLocalPtr()
{
    mIndex = TlsAlloc();
}

btThreadLocalPtr::~btThreadLocalPtr()
{
    TlsFree( mIndex );
}

void btThreadLocalPtr::setPtr( void* ptr )
{
    TlsSetValue( mIndex, ptr );
}

void* btThreadLocalPtr::getPtr()
{
    return TlsGetValue( mIndex );
}

unsigned int btGetCurrentThreadId()
{
    return GetCurrentThreadId();
}

#elif defined(__APPLE__) || defined(LINUX) // #if defined(WIN32) || defined(_WIN32)

#include <pthread.h>

class btMutex : public pthread_mutex_t
{
};

void btMutexLock( btMutex* mutex )
{
    pthread_mutex_lock( mutex );
}

void btMutexUnlock( btMutex* mutex )
{
    pthread_mutex_unlock( mutex );
}

bool btMutexTryLock( btMutex* mutex )
{
    bool ret = pthread_mutex_trylock( mutex );
    return ret != 0;
}

btMutex* btMutexCreate()
{
    btMutex* mutex = new btMutex;
    pthread_mutex_init( mutex );
    return mutex;
}

void btMutexDestroy( btMutex* mutex )
{
    pthread_mutex_destroy( mutex );
    delete mutex;
}

unsigned int btGetCurrentThreadId()
{
    return pthread_self();
}

#else

#error "no threading primitives defined -- unknown platform"

#endif  // #else // #if defined(WIN32) || defined(_WIN32)

#endif //#if BT_THREADSAFE


