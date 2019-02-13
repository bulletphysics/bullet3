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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "CmRadixSortBuffered.h"
#include "PsAllocator.h"

using namespace physx;
using namespace Cm;

RadixSortBuffered::RadixSortBuffered()
: RadixSort()
{
}

RadixSortBuffered::~RadixSortBuffered()
{
	reset();
}

void RadixSortBuffered::reset()
{
	// Release everything
	if(mDeleteRanks)
	{
		PX_FREE_AND_RESET(mRanks2);
		PX_FREE_AND_RESET(mRanks);
	}
	mCurrentSize = 0;
	INVALIDATE_RANKS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Resizes the inner lists.
 *	\param		nb	[in] new size (number of dwords)
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RadixSortBuffered::Resize(PxU32 nb)
{
	if(mDeleteRanks)
	{
		// Free previously used ram
		PX_FREE_AND_RESET(mRanks2);
		PX_FREE_AND_RESET(mRanks);

		// Get some fresh one
		mRanks	= reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nb, "RadixSortBuffered:mRanks"));
		mRanks2	= reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nb, "RadixSortBuffered:mRanks2"));
	}

	return true;
}

PX_INLINE void RadixSortBuffered::CheckResize(PxU32 nb)
{
	PxU32 CurSize = CURRENT_SIZE;
	if(nb!=CurSize)
	{
		if(nb>CurSize)	Resize(nb);
		mCurrentSize = nb;
		INVALIDATE_RANKS;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Main sort routine.
 *	This one is for integer values. After the call, mRanks contains a list of indices in sorted order, i.e. in the order you may process your data.
 *	\param		input	[in] a list of integer values to sort
 *	\param		nb		[in] number of values to sort, must be < 2^31
 *	\param		hint	[in] RADIX_SIGNED to handle negative values, RADIX_UNSIGNED if you know your input buffer only contains positive values
 *	\return		Self-Reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RadixSortBuffered& RadixSortBuffered::Sort(const PxU32* input, PxU32 nb, RadixHint hint)
{
	// Checkings
	if(!input || !nb || nb&0x80000000)	return *this;

	// Resize lists if needed
	CheckResize(nb);

	//Set histogram buffers.
	PxU32 histogram[1024];
	PxU32* links[256];
	mHistogram1024=histogram;
	mLinks256=links;

	RadixSort::Sort(input,nb,hint);
	return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Main sort routine.
 *	This one is for floating-point values. After the call, mRanks contains a list of indices in sorted order, i.e. in the order you may process your data.
 *	\param		input2			[in] a list of floating-point values to sort
 *	\param		nb				[in] number of values to sort, must be < 2^31
 *	\return		Self-Reference
 *	\warning	only sorts IEEE floating-point values
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RadixSortBuffered& RadixSortBuffered::Sort(const float* input2, PxU32 nb)
{
	// Checkings
	if(!input2 || !nb || nb&0x80000000)	return *this;

	// Resize lists if needed
	CheckResize(nb);

	//Set histogram buffers.
	PxU32 histogram[1024];
	PxU32* links[256];
	mHistogram1024=histogram;
	mLinks256=links;

	RadixSort::Sort(input2,nb);
	return *this;
}

