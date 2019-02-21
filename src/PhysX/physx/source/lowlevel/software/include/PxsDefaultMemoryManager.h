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


#ifndef PXS_DEFAULT_MEMORY_MANAGER_H
#define PXS_DEFAULT_MEMORY_MANAGER_H

#include "PxsMemoryManager.h"
#include "PsAllocator.h"
#include "PsArray.h"

namespace physx
{
	
	class PxsDefaultMemoryAllocator : public Ps::VirtualAllocatorCallback
	{
	public:

		PxsDefaultMemoryAllocator(const char* name = NULL)
		{
			PX_UNUSED(name);
#if 0 //PX_USE_NAMED_ALLOCATOR
			if (name)
				strcpy(mName, name);
			else
				strcpy(mName, "");
#endif
		}

		virtual ~PxsDefaultMemoryAllocator()
		{
		}

		virtual void* allocate(const size_t newByteSize, const char* filename, const int line)
		{
			PX_UNUSED(line);
			PX_UNUSED(filename);
#if 0 //PX_USE_NAMED_ALLOCATOR
			return PX_ALLOC(newByteSize, mName);
#else
			return PX_ALLOC(newByteSize, filename);
#endif
		}

		virtual void deallocate(void* ptr)
		{
			if (ptr)
				PX_FREE(ptr);
		}

#if 0 //PX_USE_NAMED_ALLOCATOR
		char mName[32];
#endif
	};


	class PxsDefaultMemoryManager : public PxsMemoryManager
	{
	public:
		virtual ~PxsDefaultMemoryManager();
		virtual Ps::VirtualAllocatorCallback* createHostMemoryAllocator(const PxU32 gpuComputeVersion = 0);
		virtual Ps::VirtualAllocatorCallback* createDeviceMemoryAllocator(const PxU32 gpuComputeVersion = 0);

		virtual void destroyMemoryAllocator();

		Ps::Array<Ps::VirtualAllocatorCallback*> mAllocators;

	};

}

#endif
