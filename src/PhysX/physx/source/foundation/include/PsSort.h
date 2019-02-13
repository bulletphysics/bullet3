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

#ifndef PSFOUNDATION_PSSORT_H
#define PSFOUNDATION_PSSORT_H

/** \addtogroup foundation
@{
*/

#include "PsSortInternals.h"
#include "PsAlloca.h"

#define PX_SORT_PARANOIA PX_DEBUG

/**
\brief Sorts an array of objects in ascending order, assuming
that the predicate implements the < operator:

\see Less, Greater
*/

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4706) // disable the warning that we did an assignment within a conditional expression, as
// this was intentional.
#endif

namespace physx
{
namespace shdfnd
{
template <class T, class Predicate, class Allocator>
void sort(T* elements, uint32_t count, const Predicate& compare, const Allocator& inAllocator,
          const uint32_t initialStackSize = 32)
{
	static const uint32_t SMALL_SORT_CUTOFF = 5; // must be >= 3 since we need 3 for median

	PX_ALLOCA(stackMem, int32_t, initialStackSize);
	internal::Stack<Allocator> stack(stackMem, initialStackSize, inAllocator);

	int32_t first = 0, last = int32_t(count - 1);
	if(last > first)
	{
		for(;;)
		{
			while(last > first)
			{
				PX_ASSERT(first >= 0 && last < int32_t(count));
				if(uint32_t(last - first) < SMALL_SORT_CUTOFF)
				{
					internal::smallSort(elements, first, last, compare);
					break;
				}
				else
				{
					const int32_t partIndex = internal::partition(elements, first, last, compare);

					// push smaller sublist to minimize stack usage
					if((partIndex - first) < (last - partIndex))
					{
						stack.push(first, partIndex - 1);
						first = partIndex + 1;
					}
					else
					{
						stack.push(partIndex + 1, last);
						last = partIndex - 1;
					}
				}
			}

			if(stack.empty())
				break;

			stack.pop(first, last);
		}
	}
#if PX_SORT_PARANOIA
	for(uint32_t i = 1; i < count; i++)
		PX_ASSERT(!compare(elements[i], elements[i - 1]));
#endif
}

template <class T, class Predicate>
void sort(T* elements, uint32_t count, const Predicate& compare)
{
	sort(elements, count, compare, typename shdfnd::AllocatorTraits<T>::Type());
}

template <class T>
void sort(T* elements, uint32_t count)
{
	sort(elements, count, shdfnd::Less<T>(), typename shdfnd::AllocatorTraits<T>::Type());
}

} // namespace shdfnd
} // namespace physx

#if PX_VC
#pragma warning(pop)
#endif

#endif // #ifndef PSFOUNDATION_PSSORT_H
