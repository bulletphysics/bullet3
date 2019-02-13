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

#ifndef PSFOUNDATION_PSBITUTILS_H
#define PSFOUNDATION_PSBITUTILS_H

#include "foundation/PxIntrinsics.h"
#include "foundation/PxAssert.h"
#include "PsIntrinsics.h"
#include "Ps.h"

namespace physx
{
namespace shdfnd
{
PX_INLINE uint32_t bitCount(uint32_t v)
{
	// from http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
	uint32_t const w = v - ((v >> 1) & 0x55555555);
	uint32_t const x = (w & 0x33333333) + ((w >> 2) & 0x33333333);
	return (((x + (x >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

PX_INLINE bool isPowerOfTwo(uint32_t x)
{
	return x != 0 && (x & (x - 1)) == 0;
}

// "Next Largest Power of 2
// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
// largest power of 2. For a 32-bit value:"
PX_INLINE uint32_t nextPowerOfTwo(uint32_t x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

/*!
Return the index of the highest set bit. Not valid for zero arg.
*/

PX_INLINE uint32_t lowestSetBit(uint32_t x)
{
	PX_ASSERT(x);
	return lowestSetBitUnsafe(x);
}

/*!
Return the index of the highest set bit. Not valid for zero arg.
*/

PX_INLINE uint32_t highestSetBit(uint32_t x)
{
	PX_ASSERT(x);
	return highestSetBitUnsafe(x);
}

// Helper function to approximate log2 of an integer value
// assumes that the input is actually power of two.
// todo: replace 2 usages with 'highestSetBit'
PX_INLINE uint32_t ilog2(uint32_t num)
{
	for(uint32_t i = 0; i < 32; i++)
	{
		num >>= 1;
		if(num == 0)
			return i;
	}

	PX_ASSERT(0);
	return uint32_t(-1);
}

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSBITUTILS_H
