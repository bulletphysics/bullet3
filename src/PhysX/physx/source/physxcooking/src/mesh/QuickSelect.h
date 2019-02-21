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

#ifndef QUICKSELECT_H
#define QUICKSELECT_H

#include "foundation/PxSimpleTypes.h"

// Google "wikipedia QuickSelect" for algorithm explanation
namespace physx { namespace quickSelect {


	#define SWAP32(x, y) { PxU32 tmp = y; y = x; x = tmp; }

	// left is the index of the leftmost element of the subarray
	// right is the index of the rightmost element of the subarray (inclusive)
	// number of elements in subarray = right-left+1
	template<typename LtEq>
	PxU32 partition(PxU32* PX_RESTRICT a, PxU32 left, PxU32 right, PxU32 pivotIndex, const LtEq& cmpLtEq)
	{
		PX_ASSERT(pivotIndex >= left && pivotIndex <= right);
		PxU32 pivotValue = a[pivotIndex];
		SWAP32(a[pivotIndex], a[right]) // Move pivot to end
		PxU32 storeIndex = left;
		for (PxU32 i = left; i < right; i++)  // left <= i < right
			if (cmpLtEq(a[i], pivotValue))
			{
				SWAP32(a[i], a[storeIndex]);
				storeIndex++;
			}
		SWAP32(a[storeIndex], a[right]); // Move pivot to its final place
		for (PxU32 i = left; i < storeIndex; i++)
			PX_ASSERT(cmpLtEq(a[i], a[storeIndex]));
		for (PxU32 i = storeIndex+1; i <= right; i++)
			PX_ASSERT(cmpLtEq(a[storeIndex], a[i]));
		return storeIndex;
	}

	// left is the index of the leftmost element of the subarray
	// right is the index of the rightmost element of the subarray (inclusive)
	// number of elements in subarray = right-left+1
	// recursive version
	template<typename LtEq>
	void quickFindFirstK(PxU32* PX_RESTRICT a, PxU32 left, PxU32 right, PxU32 k, const LtEq& cmpLtEq)
	{
		PX_ASSERT(k <= right-left+1);
		if (right > left)
		{
			// select pivotIndex between left and right
			PxU32 pivotIndex = (left + right) >> 1;
			PxU32 pivotNewIndex = partition(a, left, right, pivotIndex, cmpLtEq);
			// now all elements to the left of pivotNewIndex are < old value of a[pivotIndex] (bottom half values)
			if (pivotNewIndex > left + k) // new condition
				quickFindFirstK(a, left, pivotNewIndex-1, k, cmpLtEq);
			if (pivotNewIndex < left + k)
				quickFindFirstK(a, pivotNewIndex+1, right, k+left-pivotNewIndex-1, cmpLtEq);
		}
	}

	// non-recursive version
	template<typename LtEq>
	void quickSelectFirstK(PxU32* PX_RESTRICT a, PxU32 left, PxU32 right, PxU32 k, const LtEq& cmpLtEq)
	{
		PX_ASSERT(k <= right-left+1);
		for (;;)
		{
			PxU32 pivotIndex = (left+right) >> 1;
			PxU32 pivotNewIndex = partition(a, left, right, pivotIndex, cmpLtEq);
			PxU32 pivotDist = pivotNewIndex - left + 1;
			if (pivotDist == k)
				return;
			else if (k < pivotDist)
			{
				PX_ASSERT(pivotNewIndex > 0);
				right = pivotNewIndex - 1;
			}
			else
			{
				k = k - pivotDist;
				left = pivotNewIndex+1;
			}
		}
	}

} }  // namespace quickSelect, physx

#endif

