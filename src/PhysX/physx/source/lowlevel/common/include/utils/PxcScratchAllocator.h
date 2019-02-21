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


#ifndef PXC_SCRATCHALLOCATOR_H
#define PXC_SCRATCHALLOCATOR_H

#include "foundation/PxAssert.h"
#include "PxvConfig.h"
#include "PsMutex.h"
#include "PsArray.h"
#include "PsAllocator.h"

namespace physx
{
class PxcScratchAllocator
{
	PX_NOCOPY(PxcScratchAllocator)
public:
	PxcScratchAllocator() : mStack(PX_DEBUG_EXP("PxcScratchAllocator")), mStart(NULL), mSize(0)
	{
		mStack.reserve(64);
		mStack.pushBack(0);
	}

	void setBlock(void* addr, PxU32 size)
	{
		// if the stack is not empty then some scratch memory was not freed on the previous frame. That's 
		// likely indicative of a problem, because when the scratch block is too small the memory will have
		// come from the heap

		PX_ASSERT(mStack.size()==1);
		mStack.popBack();

		mStart = reinterpret_cast<PxU8*>(addr);
		mSize = size;
		mStack.pushBack(mStart + size);
	}

	void* allocAll(PxU32& size)
	{
		Ps::Mutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>0);
		size = PxU32(mStack.back()-mStart);

		if(size==0)
			return NULL;

		mStack.pushBack(mStart);
		return mStart;
	}


	void* alloc(PxU32 requestedSize, bool fallBackToHeap = false)
	{
		requestedSize = (requestedSize+15)&~15;

		Ps::Mutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>=1);

		PxU8* top = mStack.back();

		if(top - mStart >= ptrdiff_t(requestedSize))
		{
			PxU8* addr = top - requestedSize;
			mStack.pushBack(addr);
			return addr;
		}

		if(!fallBackToHeap)
			return NULL;

		return PX_ALLOC(requestedSize, "Scratch Block Fallback");
	}

	void free(void* addr)
	{
		PX_ASSERT(addr!=NULL);
		if(!isScratchAddr(addr))
		{
			PX_FREE(addr);
			return;
		}

		Ps::Mutex::ScopedLock lock(mLock);
		PX_ASSERT(mStack.size()>1);

		PxU32 i=mStack.size()-1;		
		while(mStack[i]<addr)
			i--;

		PX_ASSERT(mStack[i]==addr);
		mStack.remove(i);
	}


	bool isScratchAddr(void* addr) const
	{
		PxU8* a = reinterpret_cast<PxU8*>(addr);
		return a>= mStart && a<mStart+mSize;
	}

private:
	Ps::Mutex			mLock;
	Ps::Array<PxU8*>	mStack;
	PxU8*				mStart;
	PxU32				mSize;
};

}

#endif
