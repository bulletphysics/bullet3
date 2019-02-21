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


#ifndef PXC_CONSTRAINTBLOCKPOOL_H
#define PXC_CONSTRAINTBLOCKPOOL_H

#include "PxvConfig.h"
#include "PsArray.h"
#include "PsMutex.h"
#include "PxcNpMemBlockPool.h"

namespace physx
{
class PxsConstraintBlockManager
{
public:
	PxsConstraintBlockManager(PxcNpMemBlockPool & blockPool):
		mBlockPool(blockPool)
	{
	}

	PX_FORCE_INLINE	void reset()
	{
		mBlockPool.releaseConstraintBlocks(mTrackingArray);
	}

	PxcNpMemBlockArray	mTrackingArray;
	PxcNpMemBlockPool&	mBlockPool;

private:
	PxsConstraintBlockManager& operator=(const PxsConstraintBlockManager&);
};

class PxcConstraintBlockStream
{
	PX_NOCOPY(PxcConstraintBlockStream)
public:
	PxcConstraintBlockStream(PxcNpMemBlockPool & blockPool) :
		mBlockPool	(blockPool),
		mBlock		(NULL),
		mUsed		(0)
	{
	}

	PX_FORCE_INLINE	PxU8*				reserve(PxU32 size, PxsConstraintBlockManager& manager)
										{
											size = (size+15)&~15;
											if(size>PxcNpMemBlock::SIZE)
												return mBlockPool.acquireExceptionalConstraintMemory(size);

											if(mBlock == NULL || size+mUsed>PxcNpMemBlock::SIZE)
											{
												mBlock = mBlockPool.acquireConstraintBlock(manager.mTrackingArray);
												PX_ASSERT(0==mBlock || mBlock->data == reinterpret_cast<PxU8*>(mBlock));
												mUsed = size;
												return reinterpret_cast<PxU8*>(mBlock);
											}
											PX_ASSERT(mBlock && mBlock->data == reinterpret_cast<PxU8*>(mBlock));
											PxU8* PX_RESTRICT result = mBlock->data+mUsed;
											mUsed += size;
											return result;
										}

	PX_FORCE_INLINE	void				reset()
										{
											mBlock = NULL;
											mUsed = 0;
										}

	PX_FORCE_INLINE PxcNpMemBlockPool&	getMemBlockPool()	{ return mBlockPool;	}

private:
			PxcNpMemBlockPool&			mBlockPool;
			PxcNpMemBlock*				mBlock;	// current constraint block
			PxU32						mUsed;	// number of bytes used in constraint block
			//Tracking peak allocations
			PxU32						mPeakUsed;
};

class PxcContactBlockStream
{
	PX_NOCOPY(PxcContactBlockStream)
public:
	PxcContactBlockStream(PxcNpMemBlockPool & blockPool):
		mBlockPool(blockPool),
		mBlock(NULL),
		mUsed(0)
	{
	}

	PX_FORCE_INLINE	PxU8*				reserve(PxU32 size)
										{
											size = (size+15)&~15;

											if(size>PxcNpMemBlock::SIZE)
												return mBlockPool.acquireExceptionalConstraintMemory(size);

											PX_ASSERT(size <= PxcNpMemBlock::SIZE);

											if(mBlock == NULL || size+mUsed>PxcNpMemBlock::SIZE)
											{
												mBlock = mBlockPool.acquireContactBlock();
												PX_ASSERT(0==mBlock || mBlock->data == reinterpret_cast<PxU8*>(mBlock));
												mUsed = size;
												return reinterpret_cast<PxU8*>(mBlock);
											}
											PX_ASSERT(mBlock && mBlock->data == reinterpret_cast<PxU8*>(mBlock));
											PxU8* PX_RESTRICT result = mBlock->data+mUsed;
											mUsed += size;
											return result;
										}

	PX_FORCE_INLINE	void				reset()
										{
											mBlock = NULL;
											mUsed = 0;
										}

	PX_FORCE_INLINE PxcNpMemBlockPool&	getMemBlockPool()	{ return mBlockPool;	}

private:
			PxcNpMemBlockPool&			mBlockPool;
			PxcNpMemBlock*				mBlock;	// current constraint block
			PxU32						mUsed;	// number of bytes used in constraint block
};

}

#endif
