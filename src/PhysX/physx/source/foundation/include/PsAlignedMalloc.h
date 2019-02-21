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

#ifndef PSFOUNDATION_PSALIGNEDMALLOC_H
#define PSFOUNDATION_PSALIGNEDMALLOC_H

#include "PsUserAllocated.h"

/*!
Allocate aligned memory.
Alignment must be a power of 2!
-- should be templated by a base allocator
*/

namespace physx
{
namespace shdfnd
{
/**
Allocator, which is used to access the global PxAllocatorCallback instance
(used for dynamic data types template instantiation), which can align memory
*/

// SCS: AlignedMalloc with 3 params not found, seems not used on PC either
// disabled for now to avoid GCC error

template <uint32_t N, typename BaseAllocator = NonTrackingAllocator>
class AlignedAllocator : public BaseAllocator
{
  public:
	AlignedAllocator(const BaseAllocator& base = BaseAllocator()) : BaseAllocator(base)
	{
	}

	void* allocate(size_t size, const char* file, int line)
	{
		size_t pad = N - 1 + sizeof(size_t); // store offset for delete.
		uint8_t* base = reinterpret_cast<uint8_t*>(BaseAllocator::allocate(size + pad, file, line));
		if(!base)
			return NULL;

		uint8_t* ptr = reinterpret_cast<uint8_t*>(size_t(base + pad) & ~(size_t(N) - 1)); // aligned pointer, ensuring N
		// is a size_t
		// wide mask
		reinterpret_cast<size_t*>(ptr)[-1] = size_t(ptr - base); // store offset

		return ptr;
	}
	void deallocate(void* ptr)
	{
		if(ptr == NULL)
			return;

		uint8_t* base = reinterpret_cast<uint8_t*>(ptr) - reinterpret_cast<size_t*>(ptr)[-1];
		BaseAllocator::deallocate(base);
	}
};

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSALIGNEDMALLOC_H
