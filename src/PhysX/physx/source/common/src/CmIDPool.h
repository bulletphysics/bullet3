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


#ifndef PX_PHYSICS_COMMON_ID_POOL
#define PX_PHYSICS_COMMON_ID_POOL

#include "foundation/Px.h"
#include "CmPhysXCommon.h"
#include "PsArray.h"
#include "PsUserAllocated.h"

namespace physx
{
namespace Cm
{
	template<class FreeBuffer>
	class IDPoolBase : public Ps::UserAllocated
	{
	protected:
		PxU32			mCurrentID;
		FreeBuffer		mFreeIDs;
	public:
        IDPoolBase() : mCurrentID(0)	{}

		void	freeID(PxU32 id)
		{
			// Allocate on first call
			// Add released ID to the array of free IDs
			if(id == (mCurrentID - 1))
				--mCurrentID;
			else
				mFreeIDs.pushBack(id);
		}

		void	freeAll()
		{
			mCurrentID = 0;
			mFreeIDs.clear();
		}

		PxU32	getNewID()
		{
			// If recycled IDs are available, use them
			const PxU32 size = mFreeIDs.size();
			if(size)
			{
				// Recycle last ID
				return mFreeIDs.popBack();
			}
			// Else create a new ID
			return mCurrentID++;
		}

		PxU32 getNumUsedID()	const
		{
			return mCurrentID - mFreeIDs.size();
		}

		PxU32 getMaxID() const
		{
			return mCurrentID;
		}

	};

	//This class extends IDPoolBase. This is mainly used for when it is unsafe for the application to free the id immediately so that it can
	//defer the free process until it is safe to do so
	template<class FreeBuffer>
	class DeferredIDPoolBase : public IDPoolBase<FreeBuffer>
	{
		FreeBuffer mDeferredFreeIDs;
	public:
		//release an index into the deferred list
		void	deferredFreeID(PxU32 id)
		{
			mDeferredFreeIDs.pushBack(id);
		}

		//release the deferred indices into the free list
		void processDeferredIds()
		{
			const PxU32 deferredFreeIDCount = mDeferredFreeIDs.size();
			for(PxU32 a = 0; a < deferredFreeIDCount;++a)
			{
				IDPoolBase<FreeBuffer>::freeID(mDeferredFreeIDs[a]);
			}
			mDeferredFreeIDs.clear();
		}

		//release all indices
		void	freeAll()
		{
			mDeferredFreeIDs.clear();
			IDPoolBase<FreeBuffer>::freeAll();
		}

		PxU32 getNumUsedID()	const
		{
			return IDPoolBase<FreeBuffer>::getNumUsedID() - mDeferredFreeIDs.size();
		}

		FreeBuffer& getDeferredFreeIDs() { return mDeferredFreeIDs; }
	};

	//This is spu friendly fixed size array
	template <typename T, uint32_t N>
	class InlineFixedArray
	{
		T mArr[N];
		PxU32 mSize;
	public:

		InlineFixedArray() : mSize(0)
		{
		}

		~InlineFixedArray(){}

		void pushBack(const T& t)
		{
			PX_ASSERT(mSize < N);
			mArr[mSize++] = t;
		}

		T popBack()
		{
			PX_ASSERT(mSize > 0);
			return mArr[--mSize];
		}

		void clear() { mSize = 0; }

		T& operator [] (PxU32 index) { PX_ASSERT(index < N); return mArr[index]; }

		const T& operator [] (PxU32 index) const { PX_ASSERT(index < N); return mArr[index]; }

		PxU32 size() const { return mSize; }
	};

	//Fix size IDPool
	template<PxU32 Capacity>
	class InlineIDPool : public IDPoolBase<InlineFixedArray<PxU32, Capacity> >
	{
	public:
		PxU32 getNumRemainingIDs()
		{
			return Capacity - this->getNumUsedID();
		}
	};

	//Dynamic resize IDPool
	class IDPool : public IDPoolBase<Ps::Array<PxU32> >
	{
	};


	//This class is used to recycle indices. It supports deferred release, so that until processDeferredIds is called,
	//released indices will not be reallocated. This class will fail if the calling code request more id than the InlineDeferredIDPoll
	//has. It is the calling code's responsibility to ensure that this does not happen.
	template<PxU32 Capacity>
	class InlineDeferredIDPool : public DeferredIDPoolBase<InlineFixedArray<PxU32, Capacity> >
	{
	public:
		PxU32 getNumRemainingIDs()
		{
			return Capacity - IDPoolBase< InlineFixedArray<PxU32, Capacity> >::getNumUsedID();
		}
	};

	//Dynamic resize DeferredIDPool
	class DeferredIDPool : public DeferredIDPoolBase<Ps::Array<PxU32> >
	{

	};

} // namespace Cm

}

#endif
