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

#ifndef PXFOUNDATION_PXALLOCATORCALLBACK_H
#define PXFOUNDATION_PXALLOCATORCALLBACK_H

/** \addtogroup foundation
@{
*/

#include "foundation/Px.h"
#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Abstract base class for an application defined memory allocator that can be used by the Nv library.

\note The SDK state should not be modified from within any allocation/free function.

<b>Threading:</b> All methods of this class should be thread safe as it can be called from the user thread
or the physics processing thread(s).
*/

class PxAllocatorCallback
{
  public:
	/**
	\brief destructor
	*/
	virtual ~PxAllocatorCallback()
	{
	}

	/**
	\brief Allocates size bytes of memory, which must be 16-byte aligned.

	This method should never return NULL.  If you run out of memory, then
	you should terminate the app or take some other appropriate action.

	<b>Threading:</b> This function should be thread safe as it can be called in the context of the user thread
	and physics processing thread(s).

	\param size			Number of bytes to allocate.
	\param typeName		Name of the datatype that is being allocated
	\param filename		The source file which allocated the memory
	\param line			The source line which allocated the memory
	\return				The allocated block of memory.
	*/
	virtual void* allocate(size_t size, const char* typeName, const char* filename, int line) = 0;

	/**
	\brief Frees memory previously allocated by allocate().

	<b>Threading:</b> This function should be thread safe as it can be called in the context of the user thread
	and physics processing thread(s).

	\param ptr Memory to free.
	*/
	virtual void deallocate(void* ptr) = 0;
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXALLOCATORCALLBACK_H
