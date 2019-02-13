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

#ifndef PSFOUNDATION_PSPOOL_H
#define PSFOUNDATION_PSPOOL_H

#include "PsArray.h"
#include "PsSort.h"
#include "PsBasicTemplates.h"
#include "PsInlineArray.h"

namespace physx
{
namespace shdfnd
{

/*!
Simple allocation pool
*/
template <class T, class Alloc = typename AllocatorTraits<T>::Type>
class PoolBase : public UserAllocated, public Alloc
{
	PX_NOCOPY(PoolBase)
  protected:
	PoolBase(const Alloc& alloc, uint32_t elementsPerSlab, uint32_t slabSize)
	: Alloc(alloc), mSlabs(alloc), mElementsPerSlab(elementsPerSlab), mUsed(0), mSlabSize(slabSize), mFreeElement(0)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(T) >= sizeof(size_t));
	}

  public:
	~PoolBase()
	{
		if(mUsed)
			disposeElements();

		for(void** slabIt = mSlabs.begin(), *slabEnd = mSlabs.end(); slabIt != slabEnd; ++slabIt)
			Alloc::deallocate(*slabIt);
	}

	// Allocate space for single object
	PX_INLINE T* allocate()
	{
		if(mFreeElement == 0)
			allocateSlab();
		T* p = reinterpret_cast<T*>(mFreeElement);
		mFreeElement = mFreeElement->mNext;
		mUsed++;
/**
Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data
definition for serialized classes is complete in checked builds.
*/
#if PX_CHECKED
		for(uint32_t i = 0; i < sizeof(T); ++i)
			reinterpret_cast<uint8_t*>(p)[i] = 0xcd;
#endif
		return p;
	}

	// Put space for a single element back in the lists
	PX_INLINE void deallocate(T* p)
	{
		if(p)
		{
			PX_ASSERT(mUsed);
			mUsed--;
			push(reinterpret_cast<FreeList*>(p));
		}
	}

	PX_INLINE T* construct()
	{
		T* t = allocate();
		return t ? new (t) T() : 0;
	}

	template <class A1>
	PX_INLINE T* construct(A1& a)
	{
		T* t = allocate();
		return t ? new (t) T(a) : 0;
	}

	template <class A1, class A2>
	PX_INLINE T* construct(A1& a, A2& b)
	{
		T* t = allocate();
		return t ? new (t) T(a, b) : 0;
	}

	template <class A1, class A2, class A3>
	PX_INLINE T* construct(A1& a, A2& b, A3& c)
	{
		T* t = allocate();
		return t ? new (t) T(a, b, c) : 0;
	}

	template <class A1, class A2, class A3>
	PX_INLINE T* construct(A1* a, A2& b, A3& c)
	{
		T* t = allocate();
		return t ? new (t) T(a, b, c) : 0;
	}

	template <class A1, class A2, class A3, class A4>
	PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d)
	{
		T* t = allocate();
		return t ? new (t) T(a, b, c, d) : 0;
	}

	template <class A1, class A2, class A3, class A4, class A5>
	PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d, A5& e)
	{
		T* t = allocate();
		return t ? new (t) T(a, b, c, d, e) : 0;
	}

	PX_INLINE void destroy(T* const p)
	{
		if(p)
		{
			p->~T();
			deallocate(p);
		}
	}

  protected:
	struct FreeList
	{
		FreeList* mNext;
	};

	// All the allocated slabs, sorted by pointer
	InlineArray<void*, 64, Alloc> mSlabs;

	uint32_t mElementsPerSlab;
	uint32_t mUsed;
	uint32_t mSlabSize;

	FreeList* mFreeElement; // Head of free-list

	// Helper function to get bitmap of allocated elements

	void push(FreeList* p)
	{
		p->mNext = mFreeElement;
		mFreeElement = p;
	}

	// Allocate a slab and segregate it into the freelist
	void allocateSlab()
	{
		T* slab = reinterpret_cast<T*>(Alloc::allocate(mSlabSize, __FILE__, __LINE__));

		mSlabs.pushBack(slab);

		// Build a chain of nodes for the freelist
		T* it = slab + mElementsPerSlab;
		while(--it >= slab)
			push(reinterpret_cast<FreeList*>(it));
	}

	/*
	Cleanup method. Go through all active slabs and call destructor for live objects,
	then free their memory
	*/
	void disposeElements()
	{
		Array<void*, Alloc> freeNodes(*this);
		while(mFreeElement)
		{
			freeNodes.pushBack(mFreeElement);
			mFreeElement = mFreeElement->mNext;
		}
		Alloc& alloc(*this);
		sort(freeNodes.begin(), freeNodes.size(), Less<void*>(), alloc);
		sort(mSlabs.begin(), mSlabs.size(), Less<void*>(), alloc);

		typename Array<void*, Alloc>::Iterator slabIt = mSlabs.begin(), slabEnd = mSlabs.end();
		for(typename Array<void*, Alloc>::Iterator freeIt = freeNodes.begin(); slabIt != slabEnd; ++slabIt)
		{
			for(T* tIt = reinterpret_cast<T*>(*slabIt), *tEnd = tIt + mElementsPerSlab; tIt != tEnd; ++tIt)
			{
				if(freeIt != freeNodes.end() && *freeIt == tIt)
					++freeIt;
				else
					tIt->~T();
			}
		}
	}

	/*
	Go through all slabs and call destructor if the slab is empty
	*/
	void releaseEmptySlabs()
	{
		Array<void*, Alloc> freeNodes(*this);
		Array<void*, Alloc> slabNodes(mSlabs, *this);
		while(mFreeElement)
		{
			freeNodes.pushBack(mFreeElement);
			mFreeElement = mFreeElement->mNext;
		}

		typename Array<void*, Alloc>::Iterator freeIt = freeNodes.begin(), freeEnd = freeNodes.end(),
		                                       lastCheck = freeNodes.end() - mElementsPerSlab;

		if(freeNodes.size() > mElementsPerSlab)
		{
			Alloc& alloc(*this);
			sort(freeNodes.begin(), freeNodes.size(), Less<void*>(), alloc);
			sort(slabNodes.begin(), slabNodes.size(), Less<void*>(), alloc);

			mSlabs.clear();
			for(void** slabIt = slabNodes.begin(), *slabEnd = slabNodes.end(); slabIt != slabEnd; ++slabIt)
			{
				while((freeIt < lastCheck) && (*slabIt > (*freeIt)))
				{
					push(reinterpret_cast<FreeList*>(*freeIt));
					freeIt++;
				}

				if(*slabIt == (*freeIt)) // the slab's first element in freeList
				{
					const size_t endSlabAddress = size_t(*slabIt) + mSlabSize;
					const size_t endFreeAddress = size_t(*(freeIt + mElementsPerSlab - 1));
					if(endFreeAddress + sizeof(T) == endSlabAddress)
					{ // all slab's element in freeList
						Alloc::deallocate(*slabIt);
						freeIt += mElementsPerSlab;
						continue;
					}
				}

				mSlabs.pushBack(*slabIt);
			}
		}

		while(freeIt != freeEnd)
		{
			push(reinterpret_cast<FreeList*>(*freeIt));
			++freeIt;
		}
	}
};

// original pool implementation
template <class T, class Alloc = typename AllocatorTraits<T>::Type>
class Pool : public PoolBase<T, Alloc>
{
  public:
	Pool(const Alloc& alloc = Alloc(), uint32_t elementsPerSlab = 32)
	: PoolBase<T, Alloc>(alloc, elementsPerSlab, elementsPerSlab * sizeof(T))
	{
	}
};

// allows specification of the slab size instead of the occupancy
template <class T, uint32_t slabSize, class Alloc = typename AllocatorTraits<T>::Type>
class Pool2 : public PoolBase<T, Alloc>
{
  public:
	Pool2(const Alloc& alloc = Alloc()) : PoolBase<T, Alloc>(alloc, slabSize / sizeof(T), slabSize)
	{
	}
};

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSPOOL_H
