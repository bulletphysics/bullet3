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



#ifndef PXC_NP_MEM_BLOCK_POOL_H
#define PXC_NP_MEM_BLOCK_POOL_H

#include "PxvConfig.h"
#include "PsArray.h"
#include "PxcScratchAllocator.h"

namespace physx
{
struct PxcNpMemBlock
{
	enum
	{
		SIZE = 16384
	};
	PxU8 data[SIZE];
};

typedef Ps::Array<PxcNpMemBlock*> PxcNpMemBlockArray;

class PxcNpMemBlockPool
{
	PX_NOCOPY(PxcNpMemBlockPool)
public:
	PxcNpMemBlockPool(PxcScratchAllocator& allocator);
	~PxcNpMemBlockPool();

	void			init(PxU32 initial16KDataBlocks, PxU32 maxBlocks);
	void			flush();
	void			setBlockCount(PxU32 count);
	PxU32			getUsedBlockCount() const;
	PxU32			getMaxUsedBlockCount() const;
	PxU32			getPeakConstraintBlockCount() const;
	void			releaseUnusedBlocks();

	PxcNpMemBlock*	acquireConstraintBlock();
	PxcNpMemBlock*	acquireConstraintBlock(PxcNpMemBlockArray& memBlocks);
	PxcNpMemBlock*	acquireContactBlock();
	PxcNpMemBlock*	acquireFrictionBlock();
	PxcNpMemBlock*	acquireNpCacheBlock();

	PxU8*			acquireExceptionalConstraintMemory(PxU32 size);

	void			acquireConstraintMemory();
	void			releaseConstraintMemory();
	void			releaseConstraintBlocks(PxcNpMemBlockArray& memBlocks);
	void			releaseContacts();
	void			swapFrictionStreams();
	void			swapNpCacheStreams();

	void			flushUnused();
	
private:


	Ps::Mutex				mLock;
	PxcNpMemBlockArray		mConstraints;
	PxcNpMemBlockArray		mContacts[2];
	PxcNpMemBlockArray		mFriction[2];
	PxcNpMemBlockArray		mNpCache[2];
	PxcNpMemBlockArray		mScratchBlocks;
	Ps::Array<PxU8*>		mExceptionalConstraints;

	PxcNpMemBlockArray		mUnused;

	PxU32					mNpCacheActiveStream;
	PxU32					mFrictionActiveStream;
	PxU32					mCCDCacheActiveStream;
	PxU32					mContactIndex;
	PxU32					mAllocatedBlocks;
	PxU32					mMaxBlocks;
	PxU32					mInitialBlocks;
	PxU32					mUsedBlocks;
	PxU32					mMaxUsedBlocks;
	PxcNpMemBlock*			mScratchBlockAddr;
	PxU32					mNbScratchBlocks;
	PxcScratchAllocator&	mScratchAllocator;

	PxU32					mPeakConstraintAllocations;
	PxU32					mConstraintAllocations;

	PxcNpMemBlock*	acquire(PxcNpMemBlockArray& trackingArray, PxU32* allocationCount = NULL, PxU32* peakAllocationCount = NULL, bool isScratchAllocation = false);
	void			release(PxcNpMemBlockArray& deadArray, PxU32* allocationCount = NULL);
};

}

#endif
