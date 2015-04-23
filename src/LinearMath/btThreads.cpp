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

int gThreadsRunningCounter = 0;
btMutex gThreadsRunningCounterMutex;

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

bool btIsAligned( const void* ptr, unsigned int alignment )
{
    btAssert( ( alignment & ( alignment - 1 ) ) == 0 ); // alignment should be a power of 2
    return ( ( (size_t) ptr )&( alignment - 1 ) ) == 0;
}

bool btThreadsAreRunning()
{
    return gThreadsRunningCounter != 0;
}

#if BT_THREADSAFE

#if __cplusplus >= 201103L

// for anything claiming full C++11 compliance, use C++11 atomics
#define USE_CPP11_ATOMICS 1

#elif defined( _MSC_VER ) && _MSC_VER >= 1800

// MSVC 2013 does support C++11 atomics (even if it isn't fully C++11 compliant)
#define USE_CPP11_ATOMICS 1

#elif defined( _MSC_VER ) && _MSC_VER >= 1600

// some older MSVC versions can use MSVC intrinsics instead
#define USE_MSVC_INTRINSICS 1

#endif


#if USE_CPP11_ATOMICS

#include <atomic>

void btMutex::lock()
{
    // Lightweight mutex based on atomics and the C++11 memory model.
    // note: this lock does not sleep the thread.
    // Using ordinary system-provided mutexes like Windows critical sections was noticeably slower
    // presumably because when it fails to lock at first it would sleep the thread and trigger costly
    // context switches.
    // 
	std::atomic<unsigned char>* aDest = reinterpret_cast<std::atomic<unsigned char>*>(&mLock);
    for ( ;; )
    {
        unsigned char expected = 0;
        if ( std::atomic_compare_exchange_weak_explicit( aDest, &expected, (unsigned char)1, std::memory_order_acq_rel, std::memory_order_acquire ) )
        {
            break;
        }
    }
}

void btMutex::unlock()
{
	std::atomic<unsigned char>* aDest = reinterpret_cast<std::atomic<unsigned char>*>(&mLock);
    std::atomic_store_explicit( aDest, (unsigned char)0, std::memory_order_release );
}

bool btMutex::tryLock()
{
	std::atomic<unsigned char>* aDest = reinterpret_cast<std::atomic<unsigned char>*>(&mLock);
    unsigned char expected = 0;
    return std::atomic_compare_exchange_weak_explicit( aDest, &expected, (unsigned char)1, std::memory_order_acq_rel, std::memory_order_acquire );
}

int btAtomicLoadRelaxed( const int* src )
{
    // src must be 4-byte aligned for this to be treated as an atomic type
    btAssert( btIsAligned( src, 4 ) );  // must be 4-byte aligned or not atomic!
    const std::atomic<int>* aSrc = reinterpret_cast<const std::atomic<int>*>( src );
    return std::atomic_load_explicit( aSrc, std::memory_order_relaxed );
}

bool btAtomicCompareAndExchange32RelAcq( int* dest, int& expected, int desired )
{
    btAssert( btIsAligned( dest, 4 ) );  // must be 4-byte aligned or not atomic!
    std::atomic<int>* aDest = reinterpret_cast<std::atomic<int>*>( dest );
    return std::atomic_compare_exchange_weak_explicit( aDest, &expected, desired, std::memory_order_release, std::memory_order_acquire );
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

bool btMutex::tryLock()
{
    volatile char* aDest = reinterpret_cast<char*>( &mLock );
    return ( 0 == _InterlockedCompareExchange8( aDest, 1, 0) );
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


#else //#elif USE_MSVC_INTRINSICS

#error "no threading primitives defined -- unknown platform"

#endif  //#else //#elif USE_MSVC_INTRINSICS


void btMutexLockInternal( btMutex* mutex )
{
    mutex->lock();
}

btMutexLockFunc gBtLockFunc = btMutexLockInternal;

void btMutexLock( btMutex* mutex )
{
    // call through a user-hookable pointer
    gBtLockFunc( mutex );
}

void btMutexUnlock( btMutex* mutex )
{
    mutex->unlock();
}

bool btMutexTryLock( btMutex* mutex )
{
    return mutex->tryLock();
}

void btSetMutexLockFunc( btMutexLockFunc lockFunc )
{
    gBtLockFunc = lockFunc;
}

#else

void btMutex::lock()
{
}

void btMutex::unlock()
{
}

bool btMutex::tryLock()
{
    return true;
}

#endif //#if BT_THREADSAFE


