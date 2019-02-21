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


#ifndef PX_PHYSICS_COMMON_QUEUE
#define PX_PHYSICS_COMMON_QUEUE

#include "foundation/PxAssert.h"
#include "PsAllocator.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"

namespace physx
{
namespace Cm
{

	template<class T, class AllocType = Ps::NonTrackingAllocator >
	class Queue: public Ps::UserAllocated
	{
	public:
		Queue(PxU32 maxEntries);
		~Queue();

		T 		popFront();
		T 		front();
		T 		popBack();
		T 		back();
		bool	pushBack(const T& element);
		bool	empty() const;
		PxU32	size() const;

	private:
		T*			mJobQueue;
		PxU32		mNum;
		PxU32		mHead;
		PxU32		mTail;
		PxU32		mMaxEntries;
		AllocType	mAllocator;
	};

	template<class T, class AllocType>
	Queue<T, AllocType>::Queue(PxU32 maxEntries):
		mNum(0),
		mHead(0),
		mTail(0),
		mMaxEntries(maxEntries)
	{
		mJobQueue = reinterpret_cast<T*>(mAllocator.allocate(sizeof(T)*mMaxEntries, __FILE__, __LINE__));
	}

	template<class T, class AllocType>
	Queue<T, AllocType>::~Queue()
	{
		if(mJobQueue)
			mAllocator.deallocate(mJobQueue);
	}

	template<class T, class AllocType>
	T Queue<T, AllocType>::popFront()
	{
		PX_ASSERT(mNum>0);

		mNum--;
		T& element = mJobQueue[mTail];
		mTail = (mTail+1) % (mMaxEntries);
		return element;
	}

	template<class T, class AllocType>
	T Queue<T, AllocType>::front()
	{
		PX_ASSERT(mNum>0);

		return mJobQueue[mTail];
	}

	template<class T, class AllocType>
	T Queue<T, AllocType>::popBack()
	{
		PX_ASSERT(mNum>0);

		mNum--;
		mHead = (mHead-1) % (mMaxEntries);
		return mJobQueue[mHead];
	}

	template<class T, class AllocType>
	T Queue<T, AllocType>::back()
	{
		PX_ASSERT(mNum>0);

		PxU32 headAccess = (mHead-1) % (mMaxEntries);
		return mJobQueue[headAccess];
	}

	template<class T, class AllocType>
	bool Queue<T, AllocType>::pushBack(const T& element)
	{
		if (mNum == mMaxEntries) return false;
		mJobQueue[mHead] = element;

		mNum++;
		mHead = (mHead+1) % (mMaxEntries);

		return true;
	}

	template<class T, class AllocType>
	bool Queue<T, AllocType>::empty() const
	{
		return mNum == 0;
	}

	template<class T, class AllocType>
	PxU32 Queue<T, AllocType>::size() const
	{
		return mNum;
	}


} // namespace Cm

}

#endif
