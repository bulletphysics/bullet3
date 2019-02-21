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

#ifndef PSFOUNDATION_PSALLOCA_H
#define PSFOUNDATION_PSALLOCA_H

#include "PsTempAllocator.h"

namespace physx
{
namespace shdfnd
{
template <typename T, typename Alloc = TempAllocator>
class ScopedPointer : private Alloc
{
  public:
	~ScopedPointer()
	{
		if(mOwned)
			Alloc::deallocate(mPointer);
	}

	operator T*() const
	{
		return mPointer;
	}

	T* mPointer;
	bool mOwned;
};

} // namespace shdfnd
} // namespace physx

/*! Stack allocation for \c count instances of \c type. Falling back to temp allocator if using more than 1kB. */
#ifdef __SPU__
#define PX_ALLOCA(var, type, count) type* var = reinterpret_cast<type*>(PxAlloca(sizeof(type) * (count)))
#else
#define PX_ALLOCA(var, type, count)                                                                                    \
	physx::shdfnd::ScopedPointer<type> var;                                                                            \
	{                                                                                                                  \
		uint32_t size = sizeof(type) * (count);                                                                        \
		var.mOwned = size > 1024;                                                                                      \
		if(var.mOwned)                                                                                                 \
			var.mPointer = reinterpret_cast<type*>(physx::shdfnd::TempAllocator().allocate(size, __FILE__, __LINE__)); \
		else                                                                                                           \
			var.mPointer = reinterpret_cast<type*>(PxAlloca(size));                                                    \
	}
#endif
#endif // #ifndef PSFOUNDATION_PSALLOCA_H
