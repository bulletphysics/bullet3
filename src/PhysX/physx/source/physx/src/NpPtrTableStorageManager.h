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


#ifndef PX_PHYSICS_NP_PTRTABLESTORAGEMANAGER_H
#define PX_PHYSICS_NP_PTRTABLESTORAGEMANAGER_H

#include "CmPhysXCommon.h"
#include "PsMutex.h"
#include "PsUserAllocated.h"
#include "CmPtrTable.h"
#include "PsBitUtils.h"

namespace physx
{
class NpPtrTableStorageManager : public Cm::PtrTableStorageManager, public Ps::UserAllocated
{
	PX_NOCOPY(NpPtrTableStorageManager)

public:

	NpPtrTableStorageManager() {}
	~NpPtrTableStorageManager() {}

	void**	allocate(PxU32 capacity)
	{
		PX_ASSERT(Ps::isPowerOfTwo(capacity));

		Ps::Mutex::ScopedLock lock(mMutex);

		return capacity<=4*sizeof(void*)  ? reinterpret_cast<void**>(mPool4.construct())
			 : capacity<=16*sizeof(void*) ? reinterpret_cast<void**>(mPool16.construct())
			 : capacity<=64*sizeof(void*) ? reinterpret_cast<void**>(mPool64.construct())
			 : reinterpret_cast<void**>(PX_ALLOC(capacity*sizeof(void*), "CmPtrTable pointer array"));							
	}

	void deallocate(void** addr, PxU32 capacity)
	{
		PX_ASSERT(Ps::isPowerOfTwo(capacity));

		Ps::Mutex::ScopedLock lock(mMutex);

		if(capacity<=4*sizeof(void*))			mPool4.destroy(reinterpret_cast< PtrBlock<4>*>(addr));
		else if(capacity<=16*sizeof(void*))		mPool16.destroy(reinterpret_cast< PtrBlock<16>*>(addr));
		else if(capacity<=64*sizeof(void*))		mPool64.destroy(reinterpret_cast< PtrBlock<64>*>(addr));
		else									PX_FREE(addr);
	}

	// originalCapacity is the only way we know which pool the alloc request belongs to,
	// so if those are no longer going to match, we need to realloc.

	bool canReuse(PxU32 originalCapacity, PxU32 newCapacity)
	{
		PX_ASSERT(Ps::isPowerOfTwo(originalCapacity));
		PX_ASSERT(Ps::isPowerOfTwo(newCapacity));

		return poolId(originalCapacity) == poolId(newCapacity) && newCapacity<=64;
	}

private:
	Ps::Mutex mMutex;

	int poolId(PxU32 size)
	{
		return size<=4	? 0
			 : size<=16 ? 1
			 : size<=64 ? 2
			 : 3;
	}

	template<int N> class PtrBlock { void* ptr[N]; };

	Ps::Pool2<PtrBlock<4>, 4096 >		mPool4; 
	Ps::Pool2<PtrBlock<16>, 4096 >		mPool16;
	Ps::Pool2<PtrBlock<64>, 4096 >		mPool64;
};

}
#endif
