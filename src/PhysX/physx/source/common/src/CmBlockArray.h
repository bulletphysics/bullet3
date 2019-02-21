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

#ifndef CM_BLOCK_ARRAY_H
#define CM_BLOCK_ARRAY_H

#include "foundation/PxAssert.h"
#include "foundation/PxMath.h"
#include "foundation/PxMemory.h"
#include "PsAllocator.h"
#include "PsUserAllocated.h"
#include "PsIntrinsics.h"
#include "PsMathUtils.h"
#include "CmPhysXCommon.h"
#include "PsArray.h"

namespace physx
{
namespace Cm
{

template <typename T>
class BlockArray
{
	Ps::Array<T*> mBlocks;
	PxU32 mSize;
	PxU32 mCapacity;
	PxU32 mSlabSize;

public:

	BlockArray(PxU32 slabSize = 2048) : mSize(0), mCapacity(0), mSlabSize(slabSize)
	{
		PX_ASSERT(slabSize > 0);
	}

	~BlockArray()
	{
		for (PxU32 a = 0; a < mBlocks.size(); ++a)
		{
			PX_FREE(mBlocks[a]);
		}
		mBlocks.resize(0);
	}

	void reserve(PxU32 capacity)
	{
		if (capacity > mCapacity)
		{
			PxU32 nbSlabsRequired = (capacity + mSlabSize - 1) / mSlabSize;

			PxU32 nbSlabsToAllocate = nbSlabsRequired - mBlocks.size();

			mCapacity += nbSlabsToAllocate * mSlabSize;

			for (PxU32 a = 0; a < nbSlabsToAllocate; ++a)
			{
				mBlocks.pushBack(reinterpret_cast<T*>(PX_ALLOC(sizeof(T) * mSlabSize, PX_DEBUG_EXP("BlockArray"))));
			}
		}
	}

	void resize(PxU32 size)
	{
		reserve(size);
		for (PxU32 a = mSize; a < size; ++a)
		{
			mBlocks[a / mSlabSize][a%mSlabSize] = T();
		}
		mSize = size;
	}

	void forceSize_Unsafe(PxU32 size)
	{
		PX_ASSERT(size <= mCapacity);
		mSize = size;
	}

	void remove(PxU32 idx)
	{
		PX_ASSERT(idx < mSize);
		for (PxU32 a = idx; a < mSize; ++a)
		{
			mBlocks[a / mSlabSize][a%mSlabSize] = mBlocks[(a + 1) / mSlabSize][(a + 1) % mSlabSize];
		}

		mSize--;
	}

	void replaceWithLast(PxU32 idx)
	{
		PX_ASSERT(idx < mSize);
		--mSize;
		mBlocks[idx / mSlabSize][idx%mSlabSize] = mBlocks[mSize / mSlabSize][mSize%mSlabSize];
	}

	T& operator [] (const PxU32 idx)
	{
		PX_ASSERT(idx < mSize);

		return mBlocks[idx / mSlabSize][idx%mSlabSize];
	}

	const T& operator [] (const PxU32 idx) const
	{
		PX_ASSERT(idx < mSize);

		return mBlocks[idx / mSlabSize][idx%mSlabSize];
	}

	void pushBack(const T& item)
	{
		reserve(mSize + 1);
		mBlocks[mSize / mSlabSize][mSize%mSlabSize] = item;
		mSize++;
	}

	PxU32 capacity() const { return mCapacity; }

	PxU32 size() const { return mSize;  }
};

}
}

#endif

