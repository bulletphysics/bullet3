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

#ifndef PXCUDACONTEXTMANAGER_PXGPUCOPYDESCQUEUE_H
#define PXCUDACONTEXTMANAGER_PXGPUCOPYDESCQUEUE_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "foundation/PxAssert.h"
#include "task/PxTaskDefine.h"
#include "task/PxGpuDispatcher.h"
#include "cudamanager/PxGpuCopyDesc.h"
#include "cudamanager/PxCudaContextManager.h"

/* forward decl to avoid including <cuda.h> */
typedef struct CUstream_st* CUstream;

namespace physx
{

PX_PUSH_PACK_DEFAULT

/// \brief Container class for queueing PxGpuCopyDesc instances in pinned (non-pageable) CPU memory
class PxGpuCopyDescQueue
{
public:
	/// \brief PxGpuCopyDescQueue constructor
	PxGpuCopyDescQueue(PxGpuDispatcher& d)
		: mDispatcher(d)
		, mBuffer(0)
		, mStream(0)
		, mReserved(0)
		, mOccupancy(0)
		, mFlushed(0)
	{
	}

	/// \brief PxGpuCopyDescQueue destructor
	~PxGpuCopyDescQueue()
	{
		if (mBuffer)
		{
			mDispatcher.getCudaContextManager()->getMemoryManager()->free(PxCudaBufferMemorySpace::T_PINNED_HOST, (size_t) mBuffer);
		}
	}

	/// \brief Reset the enqueued copy descriptor list
	///
	/// Must be called at least once before any copies are enqueued, and each time the launched
	/// copies are known to have been completed.  The recommended use case is to call this at the
	/// start of each simulation step.
	void reset(CUstream stream, uint32_t reserveSize)
	{
		if (reserveSize > mReserved)
		{
			if (mBuffer)
			{
				mDispatcher.getCudaContextManager()->getMemoryManager()->free(
				    PxCudaBufferMemorySpace::T_PINNED_HOST,
				    (size_t) mBuffer);
				mReserved = 0;
			}
			mBuffer = (PxGpuCopyDesc*) mDispatcher.getCudaContextManager()->getMemoryManager()->alloc(
			              PxCudaBufferMemorySpace::T_PINNED_HOST,
			              reserveSize * sizeof(PxGpuCopyDesc),
			              PX_ALLOC_INFO("PxGpuCopyDescQueue", GPU_UTIL));
			if (mBuffer)
			{
				mReserved = reserveSize;
			}
		}

		mOccupancy = 0;
		mFlushed = 0;
		mStream = stream;
	}

	/// \brief Enqueue the specified copy descriptor, or launch immediately if no room is available
	void enqueue(PxGpuCopyDesc& desc)
	{
		PX_ASSERT(desc.isValid());
		if (desc.bytes == 0)
		{
			return;
		}

		if (mOccupancy < mReserved)
		{
			mBuffer[ mOccupancy++ ] = desc;
		}
		else
		{
			mDispatcher.launchCopyKernel(&desc, 1, mStream);
		}
	}

	/// \brief Launch all copies queued since the last flush or reset
	void flushEnqueued()
	{
		if (mOccupancy > mFlushed)
		{
			mDispatcher.launchCopyKernel(mBuffer + mFlushed, mOccupancy - mFlushed, mStream);
			mFlushed = mOccupancy;
		}
	}

private:
	PxGpuDispatcher&	mDispatcher;
	PxGpuCopyDesc*	mBuffer;
	CUstream        mStream;
	uint32_t		mReserved;
	uint32_t		mOccupancy;
	uint32_t		mFlushed;

	void operator=(const PxGpuCopyDescQueue&); // prevent a warning...
};

PX_POP_PACK

} // end physx namespace

#endif // PX_SUPPORT_GPU_PHYSX
#endif // PXCUDACONTEXTMANAGER_PXGPUCOPYDESCQUEUE_H
