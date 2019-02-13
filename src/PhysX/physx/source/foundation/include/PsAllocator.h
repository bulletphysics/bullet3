//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSALLOCATOR_H
#define PSFOUNDATION_PSALLOCATOR_H

#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxAssert.h"
#include "PxFoundation.h"
#include "Ps.h"

#if(PX_WINDOWS_FAMILY || PX_XBOXONE)
#include <exception>
#include <typeinfo.h>
#endif
#if(PX_APPLE_FAMILY)
#include <typeinfo>
#endif

#include <new>

// Allocation macros going through user allocator
#if PX_CHECKED
#define PX_ALLOC(n, name) physx::shdfnd::NamedAllocator(name).allocate(n, __FILE__, __LINE__)
#else
#define PX_ALLOC(n, name) physx::shdfnd::NonTrackingAllocator().allocate(n, __FILE__, __LINE__)
#endif
#define PX_ALLOC_TEMP(n, name) PX_ALLOC(n, name)
#define PX_FREE(x) physx::shdfnd::NonTrackingAllocator().deallocate(x)
#define PX_FREE_AND_RESET(x)                                                                                           \
	{                                                                                                                  \
		PX_FREE(x);                                                                                                    \
		x = 0;                                                                                                         \
	}

// The following macros support plain-old-types and classes derived from UserAllocated.
#define PX_NEW(T) new (physx::shdfnd::ReflectionAllocator<T>(), __FILE__, __LINE__) T
#define PX_NEW_TEMP(T) PX_NEW(T)
#define PX_DELETE(x) delete x
#define PX_DELETE_AND_RESET(x)                                                                                         \
	{                                                                                                                  \
		PX_DELETE(x);                                                                                                  \
		x = 0;                                                                                                         \
	}
#define PX_DELETE_POD(x)                                                                                               \
	{                                                                                                                  \
		PX_FREE(x);                                                                                                    \
		x = 0;                                                                                                         \
	}
#define PX_DELETE_ARRAY(x)                                                                                             \
	{                                                                                                                  \
		PX_DELETE([] x);                                                                                               \
		x = 0;                                                                                                         \
	}

// aligned allocation
#define PX_ALIGNED16_ALLOC(n) physx::shdfnd::AlignedAllocator<16>().allocate(n, __FILE__, __LINE__)
#define PX_ALIGNED16_FREE(x) physx::shdfnd::AlignedAllocator<16>().deallocate(x)

//! placement new macro to make it easy to spot bad use of 'new'
#define PX_PLACEMENT_NEW(p, T) new (p) T

#if PX_DEBUG || PX_CHECKED
#define PX_USE_NAMED_ALLOCATOR 1
#else
#define PX_USE_NAMED_ALLOCATOR 0
#endif

// Don't use inline for alloca !!!
#if PX_WINDOWS_FAMILY
#include <malloc.h>
#define PxAlloca(x) _alloca(x)
#elif PX_LINUX || PX_ANDROID
#include <malloc.h>
#define PxAlloca(x) alloca(x)
#elif PX_APPLE_FAMILY
#include <alloca.h>
#define PxAlloca(x) alloca(x)
#elif PX_PS4
#include <memory.h>
#define PxAlloca(x) alloca(x)
#elif PX_XBOXONE
#include <malloc.h>
#define PxAlloca(x) alloca(x)
#elif PX_SWITCH
#include <malloc.h>
#define PxAlloca(x) alloca(x)
#endif

#define PxAllocaAligned(x, alignment) ((size_t(PxAlloca(x + alignment)) + (alignment - 1)) & ~size_t(alignment - 1))

namespace physx
{
namespace shdfnd
{

PX_FOUNDATION_API PxAllocatorCallback& getAllocator();

/**
Allocator used to access the global PxAllocatorCallback instance without providing additional information.
*/

class PX_FOUNDATION_API Allocator
{
  public:
	Allocator(const char* = 0)
	{
	}
	void* allocate(size_t size, const char* file, int line);
	void deallocate(void* ptr);
};

/*
 * Bootstrap allocator using malloc/free.
 * Don't use unless your objects get allocated before foundation is initialized.
 */
class RawAllocator
{
  public:
	RawAllocator(const char* = 0)
	{
	}
	void* allocate(size_t size, const char*, int)
	{
		// malloc returns valid pointer for size==0, no need to check
		return ::malloc(size);
	}
	void deallocate(void* ptr)
	{
		// free(0) is guaranteed to have no side effect, no need to check
		::free(ptr);
	}
};

/*
 * Allocator that simply calls straight back to the application without tracking.
 * This is used by the heap (Foundation::mNamedAllocMap) that tracks allocations
 * because it needs to be able to grow as a result of an allocation.
 * Making the hash table re-entrant to deal with this may not make sense.
 */
class NonTrackingAllocator
{
  public:
	PX_FORCE_INLINE NonTrackingAllocator(const char* = 0)
	{
	}
	PX_FORCE_INLINE void* allocate(size_t size, const char* file, int line)
	{
		return !size ? 0 : getAllocator().allocate(size, "NonTrackedAlloc", file, line);
	}
	PX_FORCE_INLINE void deallocate(void* ptr)
	{
		if(ptr)
			getAllocator().deallocate(ptr);
	}
};

/*
\brief	Virtual allocator callback used to provide run-time defined allocators to foundation types like Array or Bitmap.
        This is used by VirtualAllocator
*/
class VirtualAllocatorCallback
{
  public:
	VirtualAllocatorCallback()
	{
	}
	virtual ~VirtualAllocatorCallback()
	{
	}
	virtual void* allocate(const size_t size, const char* file, const int line) = 0;
	virtual void deallocate(void* ptr) = 0;
};

/*
\brief Virtual allocator to be used by foundation types to provide run-time defined allocators.
Due to the fact that Array extends its allocator, rather than contains a reference/pointer to it, the VirtualAllocator
must
be a concrete type containing a pointer to a virtual callback. The callback may not be available at instantiation time,
therefore
methods are provided to set the callback later.
*/
class VirtualAllocator
{
  public:
	VirtualAllocator(VirtualAllocatorCallback* callback = NULL) : mCallback(callback)
	{
	}

	void* allocate(const size_t size, const char* file, const int line)
	{
		PX_ASSERT(mCallback);
		if(size)
			return mCallback->allocate(size, file, line);
		return NULL;
	}
	void deallocate(void* ptr)
	{
		PX_ASSERT(mCallback);
		if(ptr)
			mCallback->deallocate(ptr);
	}

	void setCallback(VirtualAllocatorCallback* callback)
	{
		mCallback = callback;
	}
	VirtualAllocatorCallback* getCallback()
	{
		return mCallback;
	}

  private:
	VirtualAllocatorCallback* mCallback;
	VirtualAllocator& operator=(const VirtualAllocator&);
};

#if PX_USE_NAMED_ALLOCATOR // can be slow, so only use in debug/checked
class PX_FOUNDATION_API NamedAllocator
{
  public:
	NamedAllocator(const PxEMPTY);
	NamedAllocator(const char* name = 0); // todo: should not have default argument!
	NamedAllocator(const NamedAllocator&);
	~NamedAllocator();
	NamedAllocator& operator=(const NamedAllocator&);
	void* allocate(size_t size, const char* filename, int line);
	void deallocate(void* ptr);
};
#else
class NamedAllocator;
#endif // PX_DEBUG

/**
Allocator used to access the global PxAllocatorCallback instance using a static name derived from T.
*/

template <typename T>
class ReflectionAllocator
{
	static const char* getName()
	{
		if(!PxGetFoundation().getReportAllocationNames())
			return "<allocation names disabled>";
#if PX_GCC_FAMILY
		return __PRETTY_FUNCTION__;
#else
		// name() calls malloc(), raw_name() wouldn't
		return typeid(T).name();
#endif
	}

  public:
	ReflectionAllocator(const PxEMPTY)
	{
	}
	ReflectionAllocator(const char* = 0)
	{
	}
	inline ReflectionAllocator(const ReflectionAllocator&)
	{
	}
	void* allocate(size_t size, const char* filename, int line)
	{
		return size ? getAllocator().allocate(size, getName(), filename, line) : 0;
	}
	void deallocate(void* ptr)
	{
		if(ptr)
			getAllocator().deallocate(ptr);
	}
};

template <typename T>
struct AllocatorTraits
{
#if PX_USE_NAMED_ALLOCATOR
	typedef NamedAllocator Type;
#else
	typedef ReflectionAllocator<T> Type;
#endif
};

// if you get a build error here, you are trying to PX_NEW a class
// that is neither plain-old-type nor derived from UserAllocated
template <typename T, typename X>
union EnableIfPod
{
	int i;
	T t;
	typedef X Type;
};

} // namespace shdfnd
} // namespace physx

// Global placement new for ReflectionAllocator templated by
// plain-old-type. Allows using PX_NEW for pointers and built-in-types.
//
// ATTENTION: You need to use PX_DELETE_POD or PX_FREE to deallocate
// memory, not PX_DELETE. PX_DELETE_POD redirects to PX_FREE.
//
// Rationale: PX_DELETE uses global operator delete(void*), which we dont' want to overload.
// Any other definition of PX_DELETE couldn't support array syntax 'PX_DELETE([]a);'.
// PX_DELETE_POD was preferred over PX_DELETE_ARRAY because it is used
// less often and applies to both single instances and arrays.
template <typename T>
PX_INLINE void* operator new(size_t size, physx::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                             typename physx::shdfnd::EnableIfPod<T, int>::Type line)
{
	return alloc.allocate(size, fileName, line);
}

template <typename T>
PX_INLINE void* operator new [](size_t size, physx::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                                typename physx::shdfnd::EnableIfPod<T, int>::Type line)
{ return alloc.allocate(size, fileName, line); }

// If construction after placement new throws, this placement delete is being called.
template <typename T>
PX_INLINE void operator delete(void* ptr, physx::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                               typename physx::shdfnd::EnableIfPod<T, int>::Type line)
{
	PX_UNUSED(fileName);
	PX_UNUSED(line);

	alloc.deallocate(ptr);
}

// If construction after placement new throws, this placement delete is being called.
template <typename T>
PX_INLINE void operator delete [](void* ptr, physx::shdfnd::ReflectionAllocator<T> alloc, const char* fileName,
                                  typename physx::shdfnd::EnableIfPod<T, int>::Type line)
{
	PX_UNUSED(fileName);
	PX_UNUSED(line);

	alloc.deallocate(ptr);
}

#endif // #ifndef PSFOUNDATION_PSALLOCATOR_H
