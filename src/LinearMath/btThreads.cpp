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

bool btIsAligned( const void* ptr, unsigned int alignment )
{
    return ( ( (unsigned int) ptr )&( alignment - 1 ) ) == 0;
}

#if BT_THREADSAFE

bool btThreadsAreRunning()
{
    return gThreadsRunning;
}


#if __cplusplus >= 201103L
#define USE_CPP11_ATOMICS 1
#elif defined( _MSC_VER ) && _MSC_VER >= 1800
#define USE_CPP11_ATOMICS 1
#elif defined( _MSC_VER ) && _MSC_VER >= 1600
#define USE_MSVC_INTRINSICS 1
#endif


#if USE_CPP11_ATOMICS
#include <atomic>
#include <thread>

void btMutex::lock()
{
    // note: this lock does not sleep the thread
    std::atomic_uchar* aDest = reinterpret_cast<std::atomic_uchar*>( &mLock );
    for ( ;; )
    {
        unsigned char expected = 0;
        if ( std::atomic_compare_exchange_weak_explicit( aDest, &expected, 1, std::memory_order_acq_rel, std::memory_order_acquire ) )
        {
            break;
        }
    }
}

void btMutex::unlock()
{
    std::atomic_uchar* aDest = reinterpret_cast<std::atomic_uchar*>( &mLock );
    std::atomic_store_explicit( aDest, 0, std::memory_order_release );
}


unsigned int btGetCurrentThreadId()
{
    std::thread::id id = std::this_thread::get_id();
    return id.hash();  // somewhat dicey (hash collision may be possible)
}

void btThreadYield()
{
    std::this_thread::yield();
}

int btAtomicLoadRelaxed( const int* src )
{
    // src must be 4-byte aligned for this to be treated as an atomic type
    btAssert( btIsAligned( src, 4 ) );  // must be 4-byte aligned or not atomic!
    const std::atomic_int* aSrc = reinterpret_cast<const std::atomic_int*>( src );
    return std::atomic_load_explicit( aSrc, std::memory_order_relaxed );
}

bool btAtomicCompareAndExchange32RelAcq( int* dest, int& expected, int desired )
{
    btAssert( btIsAligned( dest, 4 ) );  // must be 4-byte aligned or not atomic!
    std::atomic_int* aDest = reinterpret_cast<std::atomic_int*>( dest );
    return std::atomic_compare_exchange_weak_explicit( aDest, &expected, desired, std::memory_order_release, std::memory_order_acquire );
}

struct FloatIntUnion
{
    union
    {
        float mFloat;
        unsigned int mInt;
    };
};

inline void btThreadsafeFloatAdd( float* dest, float delta )
{
    // add a value to a float in memory using 32-bit compare-exchange
    if ( delta != 0.0f )
    {
        // dest must be 4-byte aligned for this to be treated as an atomic type
        std::atomic_uint* aDest = reinterpret_cast<std::atomic_uint*>( dest );
        FloatIntUnion expected;
        expected.mInt = std::atomic_load_explicit( aDest, std::memory_order_relaxed );
        FloatIntUnion desired;
        for ( ;; )
        {
            desired.mFloat = expected.mFloat + delta;
            if ( std::atomic_compare_exchange_weak_explicit( aDest, &expected.mInt, desired.mInt, std::memory_order_release, std::memory_order_acquire ) )
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
        float mFloat[ 2 ];
        unsigned long long mInt;
    };
};

inline void btThreadsafeFloat2Add( float* dest2Floats, float delta0, float delta1 )
{
    // add 2 floats atomically using a 64-bit compare-exchange
    if ( delta0 != 0.0f || delta1 != 0.0f )
    {
        // dest2Floats must be 8-byte aligned to be treated as an atomic
        std::atomic_ullong* aDest = reinterpret_cast<std::atomic_ullong*>( dest2Floats );
        Float2Int64Union expected;
        expected.mInt = std::atomic_load_explicit( aDest, std::memory_order_relaxed );
        Float2Int64Union desired;
        for ( ;; )
        {
            desired.mFloat[ 0 ] = expected.mFloat[ 0 ] + delta0;
            desired.mFloat[ 1 ] = expected.mFloat[ 1 ] + delta1;
            if ( std::atomic_compare_exchange_weak_explicit( aDest, &expected.mInt, desired.mInt, std::memory_order_release, std::memory_order_acquire ) )
            {
                break;
            }
        }
    }
}

void btThreadsafeVector3Add( btVector3* dest, const btVector3& delta )
{
#if ATOMIC_LLONG_LOCK_FREE
    // use 2 64-bit compare-exchange ops
    btAssert( btIsAligned( dest, 8 ) );
    btThreadsafeFloat2Add( reinterpret_cast<float*>( dest ), delta.x(), delta.y() );
    btThreadsafeFloatAdd( reinterpret_cast<float*>(dest) +2, delta.z() );
    //btThreadsafeFloat2Add( reinterpret_cast<float*>( dest )+2, delta.z(), 0.0f );
#else
    // use 3 32-bit compare-exchange ops
    btAssert( btIsAligned( dest, 4 ) );
    btThreadsafeFloatAdd( reinterpret_cast<float*>( dest ), delta.x() );
    btThreadsafeFloatAdd( reinterpret_cast<float*>(dest) +1, delta.y() );
    btThreadsafeFloatAdd( reinterpret_cast<float*>(dest) +2, delta.z() );
#endif
}


#elif USE_MSVC_INTRINSICS

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <intrin.h>


void btMutex::lock()
{
    // note: this lock does not sleep the thread
    volatile char* aDest = reinterpret_cast<char*>( &mLock );
    for ( ;; )
    {
        unsigned char expected = 0;
        if ( 0 == _InterlockedCompareExchange8( aDest, 1, 0) )
        {
            break;
        }
    }
}

void btMutex::unlock()
{
    volatile char* aDest = reinterpret_cast<char*>( &mLock );
    _InterlockedExchange8( aDest, 0 );
}




/*
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
*/

unsigned int btGetCurrentThreadId()
{
    return GetCurrentThreadId();
}


int btAtomicLoadRelaxed( const int* src )
{
    // src must be 4-byte aligned for this to be treated as an atomic type
    btAssert( btIsAligned( src, 4 ) );  // must be 4-byte aligned or not atomic!
    volatile const int* aSrc = src;
    return *aSrc;
}

bool btAtomicCompareAndExchange32RelAcq( int* dest, int& expected, int desired )
{
    btAssert( btIsAligned( dest, 4 ) );  // must be 4-byte aligned or not atomic!
    int origValue = _InterlockedCompareExchange( (unsigned int*) dest, desired, expected );
    if ( origValue == expected )
    {
        return true;
    }
    expected = origValue;
    return false;
}


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
            unsigned int origValue = _InterlockedCompareExchange( reinterpret_cast<volatile unsigned int*>( dest ), xchg.mInt, cmp.mInt );
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

#endif

#else

#error "no threading primitives defined -- unknown platform"

#endif  // #else // #if defined(WIN32) || defined(_WIN32)


void btInternalMutexLock( btMutex* mutex )
{
    mutex->lock();
}

void btInternalMutexUnlock( btMutex* mutex )
{
    mutex->unlock();
}

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


