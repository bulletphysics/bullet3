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



#ifndef PXC_FRICTIONPATCHPOOL_H
#define PXC_FRICTIONPATCHPOOL_H

#include "foundation/PxSimpleTypes.h"
#include "PxvConfig.h"
#include "PsMutex.h"
#include "PsArray.h"

// Each narrow phase thread has an input stream of friction patches from the
// previous frame and an output stream of friction patches which will be
// saved for next frame. The patches persist for exactly one frame at which
// point they get thrown away.


// There is a stream pair per thread. A contact callback reserves space
// for its friction patches and gets a cookie in return that can stash
// for next frame. Cookies are valid for one frame only.
//
// note that all friction patches reserved are guaranteed to be contiguous;
// this might turn out to be a bit inefficient if we often have a large
// number of friction patches

#include "PxcNpMemBlockPool.h"

namespace physx
{

class FrictionPatchStreamPair
{
public:
	FrictionPatchStreamPair(PxcNpMemBlockPool& blockPool);

	// reserve can fail and return null. Read should never fail
	template<class FrictionPatch>
	FrictionPatch*		reserve(const PxU32 size);

	template<class FrictionPatch>
	const FrictionPatch* findInputPatches(const PxU8* ptr) const;
	void					reset();

	PxcNpMemBlockPool& getBlockPool() { return mBlockPool;}
private:
	PxcNpMemBlockPool&	mBlockPool;
	PxcNpMemBlock*		mBlock;
	PxU32				mUsed;

	FrictionPatchStreamPair& operator=(const FrictionPatchStreamPair&);
};

PX_FORCE_INLINE FrictionPatchStreamPair::FrictionPatchStreamPair(PxcNpMemBlockPool& blockPool):
  mBlockPool(blockPool), mBlock(NULL), mUsed(0)
{
}

PX_FORCE_INLINE void FrictionPatchStreamPair::reset()
{
	mBlock = NULL;
	mUsed = 0;
}

// reserve can fail and return null. Read should never fail
template <class FrictionPatch>
FrictionPatch* FrictionPatchStreamPair::reserve(const PxU32 size)
{
	if(size>PxcNpMemBlock::SIZE)
	{
		return reinterpret_cast<FrictionPatch*>(-1);
	}

	PX_ASSERT(size <= PxcNpMemBlock::SIZE);

	FrictionPatch* ptr = NULL;

	if(mBlock == NULL || mUsed + size > PxcNpMemBlock::SIZE)
	{
		mBlock = mBlockPool.acquireFrictionBlock();
		mUsed = 0;
	}

	if(mBlock)
	{
		ptr = reinterpret_cast<FrictionPatch*>(mBlock->data+mUsed);
		mUsed += size;
	}

	return ptr;
}

template <class FrictionPatch>
const FrictionPatch* FrictionPatchStreamPair::findInputPatches(const PxU8* ptr) const
{
	return reinterpret_cast<const FrictionPatch*>(ptr);
}

}

#endif
