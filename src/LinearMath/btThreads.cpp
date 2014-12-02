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
#include "btVector3.h"

bool gThreadsRunning = false;

void btSetThreadsAreRunning( bool f )
{
    gThreadsRunning = f;
}

#if BT_THREADSAFE

bool btThreadsAreRunning()
{
    return gThreadsRunning;
}

#if defined(WIN32) || defined(_WIN32)

#define WIN32_LEAN_AND_MEAN

#include <windows.h>


class btMutex : public CRITICAL_SECTION
{
};

void btInternalMutexLock( btMutex* mutex )
{
    EnterCriticalSection( mutex );
}

void btInternalMutexUnlock( btMutex* mutex )
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

#if defined( _MSC_VER )

#include <intrin.h>

#if defined( _M_IX86 )

// x86 32-bit
struct FloatIntUnion
{
    union
    {
        float mFloat;
        unsigned int mInt;
    };
};

inline void btThreadsafeFloatAdd( volatile float* dest, float delta )
{
    if ( delta != 0.0f )
    {
        FloatIntUnion xchg;
        FloatIntUnion cmp;
        for ( ;; )
        {
            float orig = *dest;
            cmp.mFloat = orig;
            xchg.mFloat = orig + delta;
            unsigned int origValue = _InterlockedCompareExchange( reinterpret_cast<volatile unsigned long*>( dest ), xchg.mInt, cmp.mInt );
            if ( origValue == cmp.mInt )
            {
                break;
            }
        }
    }
}

struct Float2Int64Union
{
    union
    {
        float mFloat[2];
        __int64 mInt;
    };
};

inline void btThreadsafeFloat2Add( volatile float* dest, float delta0, float delta1 )
{
    if ( delta0 != 0.0f || delta1 != 0.0f )
    {
        Float2Int64Union xchg;
        Float2Int64Union cmp;
        for ( ;; )
        {
            float orig0 = dest[ 0 ];
            float orig1 = dest[ 1 ];
            cmp.mFloat[ 0 ] = orig0;
            cmp.mFloat[ 1 ] = orig1;
            xchg.mFloat[ 0 ] = orig0 + delta0;
            xchg.mFloat[ 1 ] = orig1 + delta1;
            __int64 origValue = _InterlockedCompareExchange64( reinterpret_cast<volatile __int64*>( dest ), xchg.mInt, cmp.mInt );
            if ( origValue == cmp.mInt )
            {
                break;
            }
        }
    }
}

void btThreadsafeVector3Add( btVector3* dest, const btVector3& delta )
{
#if 1
    // use 2 64-bit compare-exchange ops
    btThreadsafeFloat2Add( reinterpret_cast<float*>( dest ), delta.x(), delta.y() );
    btThreadsafeFloat2Add( reinterpret_cast<float*>( dest )+2, delta.z(), 0.0f );
#else
    // use 3 32-bit compare-exchange ops
    btThreadsafeFloatAdd( reinterpret_cast<float*>( dest ), delta.x() );
    btThreadsafeFloatAdd( reinterpret_cast<float*>( dest )+1, delta.y() );
    btThreadsafeFloatAdd( reinterpret_cast<float*>( dest )+2, delta.z() );
#endif
}

#elif defined( _M_AMD64 )
// x86 64-bit
struct Float4Int128Union
{
    union
    {
        btSimdFloat4 mSimdF4;
        float mFloat[4];
        __int64 mInt[2];
    };
};

void btThreadsafeVector3Add( btVector3* dest, const btVector3& delta )
{
    // use a single _InterlockedCompareExchange128
    //volatile
    btSimdFloat4* vdest = reinterpret_cast<btSimdFloat4*>(dest);
    Float4Int128Union xchg;
    Float4Int128Union cmp;
    for ( ;; )
    {
        cmp.mSimdF4 = *vdest;
        btVector3 newVal = btVector3(cmp.mSimdF4) + delta;
        xchg.mSimdF4 = newVal.get128();
        unsigned char ret = _InterlockedCompareExchange128( reinterpret_cast<volatile __int64*>( vdest ), xchg.mInt[1], xchg.mInt[0], &cmp.mInt[0] );
        if ( ret )
        {
            break;
        }
    }
}

#else
#endif

#endif //#if defined( _MSC_VER )

#elif defined(__APPLE__) || defined(LINUX) // #if defined(WIN32) || defined(_WIN32)

#include <pthread.h>

class btMutex : public pthread_mutex_t
{
};

void btInternalMutexLock( btMutex* mutex )
{
    pthread_mutex_lock( mutex );
}

void btInternalMutexUnlock( btMutex* mutex )
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

btMutexLockFunc gBtLockFunc = btInternalMutexLock;
btMutexLockFunc gBtUnlockFunc = btInternalMutexUnlock;

void btMutexLock( btMutex* mutex )
{
    gBtLockFunc( mutex );
}

void btMutexUnlock( btMutex* mutex )
{
    gBtUnlockFunc( mutex );
}

void btSetMutexLockFunc( btMutexLockFunc lockFunc )
{
    gBtLockFunc = lockFunc;
}
void btSetMutexUnlockFunc( btMutexLockFunc unlockFunc )
{
    gBtUnlockFunc = unlockFunc;
}

#endif //#if BT_THREADSAFE


