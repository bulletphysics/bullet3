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


#ifndef PX_PHYSICS_COMMON_PRIORITYQUEUE
#define PX_PHYSICS_COMMON_PRIORITYQUEUE

#include "PsBasicTemplates.h"
#include "CmPhysXCommon.h"
#include "PsAllocator.h"
#include "foundation/PxMemory.h"

namespace physx
{
namespace Cm
{
	template<class Element, class Comparator = Ps::Less<Element> >
	class PriorityQueueBase : protected Comparator // inherit so that stateless comparators take no space
	{
	public:
		PriorityQueueBase(const Comparator& less, Element* elements) : Comparator(less), mHeapSize(0), mDataPtr(elements)
		{
		}
		
		~PriorityQueueBase() 
		{
		}
		
		//! Get the element with the highest priority
		PX_FORCE_INLINE const Element top() const
		{
			return mDataPtr[0];
		}

		//! Get the element with the highest priority
		PX_FORCE_INLINE Element top()
		{
			return mDataPtr[0];
		}
		
		//! Check to whether the priority queue is empty
		PX_FORCE_INLINE bool empty() const
		{
			return (mHeapSize == 0);
		}
		
		//! Empty the priority queue
		PX_FORCE_INLINE void clear()
		{
			mHeapSize = 0;
		}  

		//! Insert a new element into the priority queue. Only valid when size() is less than Capacity
		PX_FORCE_INLINE void push(const Element& value)
		{
			PxU32 newIndex;
			PxU32 parentIndex = parent(mHeapSize);

			for (newIndex = mHeapSize; newIndex > 0 && compare(value, mDataPtr[parentIndex]); newIndex = parentIndex, parentIndex= parent(newIndex)) 
			{
				mDataPtr[ newIndex ] = mDataPtr[parentIndex];
			}
			mDataPtr[newIndex] = value; 
			mHeapSize++;
			PX_ASSERT(valid());
		}

		//! Delete the highest priority element. Only valid when non-empty.
		PX_FORCE_INLINE Element pop()
		{
			PX_ASSERT(mHeapSize > 0);
			PxU32 i, child;
			//try to avoid LHS
			PxU32 tempHs = mHeapSize-1;
			mHeapSize = tempHs;
			Element min = mDataPtr[0];
			Element last = mDataPtr[tempHs];
			
			for (i = 0; (child = left(i)) < tempHs; i = child) 
			{
				/* Find highest priority child */
				const PxU32 rightChild = child + 1;
			
				child += ((rightChild < tempHs) & compare((mDataPtr[rightChild]), (mDataPtr[child]))) ? 1 : 0;

				if(compare(last, mDataPtr[child]))
					break;

				mDataPtr[i] = mDataPtr[child];
			}
			mDataPtr[ i ] = last;
			
			PX_ASSERT(valid());
			return min;
		} 

		//! Make sure the priority queue sort all elements correctly
		bool valid() const
		{
			const Element& min = mDataPtr[0];
			for(PxU32 i=1; i<mHeapSize; ++i)
			{
				if(compare(mDataPtr[i], min))
					return false;
			}

			return true;
		}

		//! Return number of elements in the priority queue
		PxU32 size() const
		{
			return mHeapSize;
		}

	protected:

		PxU32 mHeapSize;
		Element* mDataPtr;
		
		PX_FORCE_INLINE bool compare(const Element& a, const Element& b) const
		{
			return Comparator::operator()(a,b);
		}

		static PX_FORCE_INLINE PxU32 left(PxU32 nodeIndex) 
		{
			return (nodeIndex << 1) + 1;
		}
		
		static PX_FORCE_INLINE PxU32 parent(PxU32 nodeIndex) 
		{
			return (nodeIndex - 1) >> 1;
		}
	private:
		PriorityQueueBase<Element, Comparator>& operator = (const PriorityQueueBase<Element, Comparator>);
	};

	template <typename Element, PxU32 Capacity, typename Comparator>
	class InlinePriorityQueue : public PriorityQueueBase<Element, Comparator>
	{
		Element mData[Capacity];
	public:
		InlinePriorityQueue(const Comparator& less = Comparator()) : PriorityQueueBase<Element, Comparator>(less, mData)
		{
		}

		PX_FORCE_INLINE void push(Element& elem)
		{
			PX_ASSERT(this->mHeapSize < Capacity);
			PriorityQueueBase<Element, Comparator>::push(elem);
		}
	private:
		InlinePriorityQueue<Element, Capacity, Comparator>& operator = (const InlinePriorityQueue<Element, Capacity, Comparator>);
	};

	template <typename Element, typename Comparator, typename Alloc = typename physx::shdfnd::AllocatorTraits<Element>::Type>
	class PriorityQueue : public PriorityQueueBase<Element, Comparator>, protected Alloc
	{
		PxU32 mCapacity;
	public:
		PriorityQueue(const Comparator& less = Comparator(), PxU32 initialCapacity = 0, Alloc alloc = Alloc()) 
			: PriorityQueueBase<Element, Comparator>(less, NULL), Alloc(alloc), mCapacity(initialCapacity)
		{
			if(initialCapacity > 0)
				this->mDataPtr = reinterpret_cast<Element*>(Alloc::allocate(sizeof(Element)*initialCapacity, __FILE__, __LINE__));
		}

		~PriorityQueue()
		{
			if(this->mDataPtr)
				this->deallocate(this->mDataPtr);
		}

		PX_FORCE_INLINE void push(Element& elem)
		{
			if(this->mHeapSize == mCapacity)
			{
				reserve((this->mHeapSize+1)*2);
			}
			PriorityQueueBase<Element, Comparator>::push(elem);
		}

		PX_FORCE_INLINE PxU32 capacity()
		{
			return mCapacity;
		}

		PX_FORCE_INLINE void reserve(const PxU32 newCapacity)
		{
			if(newCapacity > mCapacity)
			{
				Element* newElems = reinterpret_cast<Element*>(Alloc::allocate(sizeof(Element)*newCapacity, __FILE__, __LINE__));
				if(this->mDataPtr)
				{
					physx::PxMemCopy(newElems, this->mDataPtr, sizeof(Element) * this->mHeapSize);
					Alloc::deallocate(this->mDataPtr);
				}
				this->mDataPtr = newElems;
				mCapacity = newCapacity;
			}
		}

	private:
		PriorityQueue<Element, Comparator, Alloc>& operator = (const PriorityQueue<Element, Comparator, Alloc>);
	};
	
}
}

#endif
