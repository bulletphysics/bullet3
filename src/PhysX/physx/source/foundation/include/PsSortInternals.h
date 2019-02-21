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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSSORTINTERNALS_H
#define PSFOUNDATION_PSSORTINTERNALS_H

/** \addtogroup foundation
@{
*/

#include "foundation/PxAssert.h"
#include "foundation/PxIntrinsics.h"
#include "PsBasicTemplates.h"
#include "PsUserAllocated.h"

namespace physx
{
namespace shdfnd
{
namespace internal
{
template <class T, class Predicate>
PX_INLINE void median3(T* elements, int32_t first, int32_t last, Predicate& compare)
{
	/*
	This creates sentinels because we know there is an element at the start minimum(or equal)
	than the pivot and an element at the end greater(or equal) than the pivot. Plus the
	median of 3 reduces the chance of degenerate behavour.
	*/

	int32_t mid = (first + last) / 2;

	if(compare(elements[mid], elements[first]))
		swap(elements[first], elements[mid]);

	if(compare(elements[last], elements[first]))
		swap(elements[first], elements[last]);

	if(compare(elements[last], elements[mid]))
		swap(elements[mid], elements[last]);

	// keep the pivot at last-1
	swap(elements[mid], elements[last - 1]);
}

template <class T, class Predicate>
PX_INLINE int32_t partition(T* elements, int32_t first, int32_t last, Predicate& compare)
{
	median3(elements, first, last, compare);

	/*
	WARNING: using the line:

	T partValue = elements[last-1];

	and changing the scan loops to:

	while(comparator.greater(partValue, elements[++i]));
	while(comparator.greater(elements[--j], partValue);

	triggers a compiler optimizer bug on xenon where it stores a double to the stack for partValue
	then loads it as a single...:-(
	*/

	int32_t i = first;    // we know first is less than pivot(but i gets pre incremented)
	int32_t j = last - 1; // pivot is in last-1 (but j gets pre decremented)

	for(;;)
	{
		while(compare(elements[++i], elements[last - 1]))
			;
		while(compare(elements[last - 1], elements[--j]))
			;

		if(i >= j)
			break;

		PX_ASSERT(i <= last && j >= first);
		swap(elements[i], elements[j]);
	}
	// put the pivot in place

	PX_ASSERT(i <= last && first <= (last - 1));
	swap(elements[i], elements[last - 1]);

	return i;
}

template <class T, class Predicate>
PX_INLINE void smallSort(T* elements, int32_t first, int32_t last, Predicate& compare)
{
	// selection sort - could reduce to fsel on 360 with floats.

	for(int32_t i = first; i < last; i++)
	{
		int32_t m = i;
		for(int32_t j = i + 1; j <= last; j++)
			if(compare(elements[j], elements[m]))
				m = j;

		if(m != i)
			swap(elements[m], elements[i]);
	}
}

template <class Allocator>
class Stack
{
	Allocator mAllocator;
	uint32_t mSize, mCapacity;
	int32_t* mMemory;
	bool mRealloc;

  public:
	Stack(int32_t* memory, uint32_t capacity, const Allocator& inAllocator)
	: mAllocator(inAllocator), mSize(0), mCapacity(capacity), mMemory(memory), mRealloc(false)
	{
	}
	~Stack()
	{
		if(mRealloc)
			mAllocator.deallocate(mMemory);
	}

	void grow()
	{
		mCapacity *= 2;
		int32_t* newMem =
		    reinterpret_cast<int32_t*>(mAllocator.allocate(sizeof(int32_t) * mCapacity, __FILE__, __LINE__));
		intrinsics::memCopy(newMem, mMemory, mSize * sizeof(int32_t));
		if(mRealloc)
			mAllocator.deallocate(mMemory);
		mRealloc = true;
		mMemory = newMem;
	}

	PX_INLINE void push(int32_t start, int32_t end)
	{
		if(mSize >= mCapacity - 1)
			grow();
		mMemory[mSize++] = start;
		mMemory[mSize++] = end;
	}

	PX_INLINE void pop(int32_t& start, int32_t& end)
	{
		PX_ASSERT(!empty());
		end = mMemory[--mSize];
		start = mMemory[--mSize];
	}

	PX_INLINE bool empty()
	{
		return mSize == 0;
	}
};
} // namespace internal

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSSORTINTERNALS_H
