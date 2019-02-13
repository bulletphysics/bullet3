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
#include "foundation/PxMemory.h"
#include "foundation/PxAssert.h"
#include "CmRadixSort.h"

using namespace physx;
using namespace Cm;

#if defined(__BIG_ENDIAN__) || defined(_XBOX)
	#define H0_OFFSET	768
	#define H1_OFFSET	512
	#define H2_OFFSET	256
	#define H3_OFFSET	0
	#define BYTES_INC	(3-j)
#else 
	#define H0_OFFSET	0
	#define H1_OFFSET	256
	#define H2_OFFSET	512
	#define H3_OFFSET	768
	#define BYTES_INC	j
#endif

#define CREATE_HISTOGRAMS(type, buffer)														\
	/* Clear counters/histograms */															\
	PxMemZero(mHistogram1024, 256*4*sizeof(PxU32));											\
																							\
	/* Prepare to count */																	\
	const PxU8* PX_RESTRICT p = reinterpret_cast<const PxU8*>(input);						\
	const PxU8* PX_RESTRICT pe = &p[nb*4];													\
	PxU32* PX_RESTRICT h0= &mHistogram1024[H0_OFFSET];	/* Histogram for first pass (LSB)*/	\
	PxU32* PX_RESTRICT h1= &mHistogram1024[H1_OFFSET];	/* Histogram for second pass	*/	\
	PxU32* PX_RESTRICT h2= &mHistogram1024[H2_OFFSET];	/* Histogram for third pass		*/	\
	PxU32* PX_RESTRICT h3= &mHistogram1024[H3_OFFSET];	/* Histogram for last pass (MSB)*/	\
																							\
	bool AlreadySorted = true;	/* Optimism... */											\
																							\
	if(INVALID_RANKS)																		\
	{																						\
		/* Prepare for temporal coherence */												\
		const type* PX_RESTRICT Running = reinterpret_cast<const type*>(buffer);			\
		type PrevVal = *Running;															\
																							\
		while(p!=pe)																		\
		{																					\
			/* Read input buffer in previous sorted order */								\
			const type Val = *Running++;													\
			/* Check whether already sorted or not */										\
			if(Val<PrevVal)	{ AlreadySorted = false; break; } /* Early out */				\
			/* Update for next iteration */													\
			PrevVal = Val;																	\
																							\
			/* Create histograms */															\
			h0[*p++]++;	h1[*p++]++;	h2[*p++]++;	h3[*p++]++;									\
		}																					\
																							\
		/* If all input values are already sorted, we just have to return and leave the */	\
		/* previous list unchanged. That way the routine may take advantage of temporal */	\
		/* coherence, for example when used to sort transparent faces.					*/	\
		if(AlreadySorted)																	\
		{																					\
			mNbHits++;																		\
			for(PxU32 i=0;i<nb;i++)	mRanks[i] = i;											\
			return *this;																	\
		}																					\
	}																						\
	else																					\
	{																						\
		/* Prepare for temporal coherence */												\
		const PxU32* PX_RESTRICT Indices = mRanks;											\
		type PrevVal = type(buffer[*Indices]);												\
																							\
		while(p!=pe)																		\
		{																					\
			/* Read input buffer in previous sorted order */								\
			const type Val = type(buffer[*Indices++]);										\
			/* Check whether already sorted or not */										\
			if(Val<PrevVal)	{ AlreadySorted = false; break; } /* Early out */				\
			/* Update for next iteration */													\
			PrevVal = Val;																	\
																							\
			/* Create histograms */															\
			h0[*p++]++;	h1[*p++]++;	h2[*p++]++;	h3[*p++]++;									\
		}																					\
																							\
		/* If all input values are already sorted, we just have to return and leave the */	\
		/* previous list unchanged. That way the routine may take advantage of temporal */	\
		/* coherence, for example when used to sort transparent faces.					*/	\
		if(AlreadySorted)	{ mNbHits++; return *this;	}									\
	}																						\
																							\
	/* Else there has been an early out and we must finish computing the histograms */		\
	while(p!=pe)																			\
	{																						\
		/* Create histograms without the previous overhead */								\
		h0[*p++]++;	h1[*p++]++;	h2[*p++]++;	h3[*p++]++;										\
	}

PX_INLINE const PxU32* CheckPassValidity(PxU32 pass, const PxU32* mHistogram1024, PxU32 nb, const void* input, PxU8& UniqueVal)
{
	// Shortcut to current counters
	const PxU32* CurCount = &mHistogram1024[pass<<8];

	// Check pass validity

	// If all values have the same byte, sorting is useless.
	// It may happen when sorting bytes or words instead of dwords.
	// This routine actually sorts words faster than dwords, and bytes
	// faster than words. Standard running time (O(4*n))is reduced to O(2*n)
	// for words and O(n) for bytes. Running time for floats depends on actual values...

	// Get first byte
	UniqueVal = *((reinterpret_cast<const PxU8*>(input))+pass);

	// Check that byte's counter
	if(CurCount[UniqueVal]==nb)
		return NULL;

	return CurCount;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RadixSort::RadixSort() : mCurrentSize(0), mRanks(NULL), mRanks2(NULL), mHistogram1024(0), mLinks256(0), mTotalCalls(0), mNbHits(0), mDeleteRanks(true)
{
	// Initialize indices
	INVALIDATE_RANKS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RadixSort::~RadixSort()
{
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

RadixSort& RadixSort::Sort(const PxU32* input, PxU32 nb, RadixHint hint)
{
	PX_ASSERT(mHistogram1024);
	PX_ASSERT(mLinks256);
	PX_ASSERT(mRanks);
	PX_ASSERT(mRanks2);

	// Checkings
	if(!input || !nb || nb&0x80000000)	return *this;

	// Stats
	mTotalCalls++;

	// Create histograms (counters). Counters for all passes are created in one run.
	// Pros:	read input buffer once instead of four times
	// Cons:	mHistogram1024 is 4Kb instead of 1Kb
	// We must take care of signed/unsigned values for temporal coherence.... I just
	// have 2 code paths even if just a single opcode changes. Self-modifying code, someone?
	if(hint==RADIX_UNSIGNED)	{ CREATE_HISTOGRAMS(PxU32, input);	}
	else						{ CREATE_HISTOGRAMS(PxI32, input);	}

	// Compute #negative values involved if needed
	PxU32 NbNegativeValues = 0;
	if(hint==RADIX_SIGNED)
	{
		// An efficient way to compute the number of negatives values we'll have to deal with is simply to sum the 128
		// last values of the last histogram. Last histogram because that's the one for the Most Significant Byte,
		// responsible for the sign. 128 last values because the 128 first ones are related to positive numbers.
		PxU32* PX_RESTRICT h3= &mHistogram1024[768];
		for(PxU32 i=128;i<256;i++)	NbNegativeValues += h3[i];	// 768 for last histogram, 128 for negative part
	}

	// Radix sort, j is the pass number (0=LSB, 3=MSB)
	for(PxU32 j=0;j<4;j++)
	{
//		CHECK_PASS_VALIDITY(j);
		PxU8 UniqueVal;
		const PxU32* PX_RESTRICT CurCount = CheckPassValidity(j, mHistogram1024, nb, input, UniqueVal);

		// Sometimes the fourth (negative) pass is skipped because all numbers are negative and the MSB is 0xFF (for example). This is
		// not a problem, numbers are correctly sorted anyway.
		if(CurCount)
		{
			PxU32** PX_RESTRICT Links256 = mLinks256;

			// Should we care about negative values?
			if(j!=3 || hint==RADIX_UNSIGNED)
			{
				// Here we deal with positive values only

				// Create offsets
				Links256[0] = mRanks2;
				for(PxU32 i=1;i<256;i++)
					Links256[i] = Links256[i-1] + CurCount[i-1];
			}
			else
			{
				// This is a special case to correctly handle negative integers. They're sorted in the right order but at the wrong place.

				// Create biased offsets, in order for negative numbers to be sorted as well
				Links256[0] = &mRanks2[NbNegativeValues];										// First positive number takes place after the negative ones
				for(PxU32 i=1;i<128;i++)
					Links256[i] = Links256[i-1] + CurCount[i-1];		// 1 to 128 for positive numbers

				// Fixing the wrong place for negative values
				Links256[128] = mRanks2;
				for(PxU32 i=129;i<256;i++)
					Links256[i] = Links256[i-1] + CurCount[i-1];
			}

			// Perform Radix Sort
			const PxU8* PX_RESTRICT InputBytes = reinterpret_cast<const PxU8*>(input);
            InputBytes += BYTES_INC;
			if(INVALID_RANKS)
			{
				for(PxU32 i=0;i<nb;i++)
					*Links256[InputBytes[i<<2]]++ = i;
				VALIDATE_RANKS;
			}
			else
			{
				PxU32* PX_RESTRICT Indices		= mRanks;
				PxU32* PX_RESTRICT IndicesEnd	= &mRanks[nb];
				while(Indices!=IndicesEnd)
				{
					const PxU32 id = *Indices++;
					*Links256[InputBytes[id<<2]]++ = id;
				}
			}

			// Swap pointers for next pass. Valid indices - the most recent ones - are in mRanks after the swap.
			PxU32* Tmp	= mRanks;	mRanks = mRanks2; mRanks2 = Tmp;
		}
	}
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

RadixSort& RadixSort::Sort(const float* input2, PxU32 nb)
{
	PX_ASSERT(mHistogram1024);
	PX_ASSERT(mLinks256);
	PX_ASSERT(mRanks);
	PX_ASSERT(mRanks2);

	// Checkings
	if(!input2 || !nb || nb&0x80000000)	return *this;

	// Stats
	mTotalCalls++;

	const PxU32* PX_RESTRICT input = reinterpret_cast<const PxU32*>(input2);

	// Allocate histograms & offsets on the stack
	//PxU32 mHistogram1024[256*4];
	//PxU32* mLinks256[256];

	// Create histograms (counters). Counters for all passes are created in one run.
	// Pros:	read input buffer once instead of four times
	// Cons:	mHistogram1024 is 4Kb instead of 1Kb
	// Floating-point values are always supposed to be signed values, so there's only one code path there.
	// Please note the floating point comparison needed for temporal coherence! Although the resulting asm code
	// is dreadful, this is surprisingly not such a performance hit - well, I suppose that's a big one on first
	// generation Pentiums....We can't make comparison on integer representations because, as Chris said, it just
	// wouldn't work with mixed positive/negative values....
	{ CREATE_HISTOGRAMS(float, input2); }

	// Compute #negative values involved if needed
	PxU32 NbNegativeValues = 0;
	// An efficient way to compute the number of negatives values we'll have to deal with is simply to sum the 128
	// last values of the last histogram. Last histogram because that's the one for the Most Significant Byte,
	// responsible for the sign. 128 last values because the 128 first ones are related to positive numbers.
	// ### is that ok on Apple ?!
	PxU32* PX_RESTRICT h3= &mHistogram1024[768];
	for(PxU32 i=128;i<256;i++)	NbNegativeValues += h3[i];	// 768 for last histogram, 128 for negative part

	// Radix sort, j is the pass number (0=LSB, 3=MSB)
	for(PxU32 j=0;j<4;j++)
	{
		PxU8 UniqueVal;
		const PxU32* PX_RESTRICT CurCount = CheckPassValidity(j, mHistogram1024, nb, input, UniqueVal);
		// Should we care about negative values?
		if(j!=3)
		{
			// Here we deal with positive values only
//			CHECK_PASS_VALIDITY(j);
//			const bool PerformPass = CheckPassValidity(j, mHistogram1024, nb, input);

			if(CurCount)
			{
				PxU32** PX_RESTRICT Links256 = mLinks256;

				// Create offsets
				Links256[0] = mRanks2;
				for(PxU32 i=1;i<256;i++)
					Links256[i] = Links256[i-1] + CurCount[i-1];

				// Perform Radix Sort
				const PxU8* PX_RESTRICT InputBytes = reinterpret_cast<const PxU8*>(input);
                InputBytes += BYTES_INC;
				if(INVALID_RANKS)
				{
					for(PxU32 i=0;i<nb;i++)
						*Links256[InputBytes[i<<2]]++ = i;
					VALIDATE_RANKS;
				}
				else
				{
					PxU32* PX_RESTRICT Indices		= mRanks;
					PxU32* PX_RESTRICT IndicesEnd	= &mRanks[nb];
					while(Indices!=IndicesEnd)
					{
						const PxU32 id = *Indices++;
						*Links256[InputBytes[id<<2]]++ = id;
					}
				}

				// Swap pointers for next pass. Valid indices - the most recent ones - are in mRanks after the swap.
				PxU32* Tmp	= mRanks;	mRanks = mRanks2; mRanks2 = Tmp;
			}
		}
		else
		{
			// This is a special case to correctly handle negative values
//			CHECK_PASS_VALIDITY(j);
//			const bool PerformPass = CheckPassValidity(j, mHistogram1024, nb, input);

			if(CurCount)
			{
				PxU32** PX_RESTRICT Links256 = mLinks256;

				// Create biased offsets, in order for negative numbers to be sorted as well
				Links256[0] = &mRanks2[NbNegativeValues];										// First positive number takes place after the negative ones
				for(PxU32 i=1;i<128;i++)
					Links256[i] = Links256[i-1] + CurCount[i-1];		// 1 to 128 for positive numbers

				// We must reverse the sorting order for negative numbers!
				Links256[255] = mRanks2;
				for(PxU32 i=0;i<127;i++)
					Links256[254-i] = Links256[255-i] + CurCount[255-i];		// Fixing the wrong order for negative values
				for(PxU32 i=128;i<256;i++)
					Links256[i] += CurCount[i];							// Fixing the wrong place for negative values

				// Perform Radix Sort
				if(INVALID_RANKS)
				{
					for(PxU32 i=0;i<nb;i++)
					{
						const PxU32 Radix = input[i]>>24;				// Radix byte, same as above. AND is useless here (PxU32).
						// ### cmp to be killed. Not good. Later.
						if(Radix<128)	*Links256[Radix]++ = i;			// Number is positive, same as above
						else			*(--Links256[Radix]) = i;		// Number is negative, flip the sorting order
					}
					VALIDATE_RANKS;
				}
				else
				{
					const PxU32* PX_RESTRICT Ranks = mRanks;
					for(PxU32 i=0;i<nb;i++)
					{
						const PxU32 Radix = input[Ranks[i]]>>24;				// Radix byte, same as above. AND is useless here (PxU32).
                        // ### cmp to be killed. Not good. Later.
						if(Radix<128)	*Links256[Radix]++ = Ranks[i];			// Number is positive, same as above
						else			*(--Links256[Radix]) = Ranks[i];		// Number is negative, flip the sorting order
					}
				}
				// Swap pointers for next pass. Valid indices - the most recent ones - are in mRanks after the swap.
				PxU32* Tmp	= mRanks;	mRanks = mRanks2; mRanks2 = Tmp;
			}
			else
			{
				// The pass is useless, yet we still have to reverse the order of current list if all values are negative.
				if(UniqueVal>=128)
				{
					if(INVALID_RANKS)
					{
						// ###Possible?
						for(PxU32 i=0;i<nb;i++)	mRanks2[i] = nb-i-1;
						VALIDATE_RANKS;
					}
					else
					{
						for(PxU32 i=0;i<nb;i++)	mRanks2[i] = mRanks[nb-i-1];
					}

					// Swap pointers for next pass. Valid indices - the most recent ones - are in mRanks after the swap.
					PxU32* Tmp	= mRanks;	mRanks = mRanks2; mRanks2 = Tmp;
				}
			}
		}
	}
	return *this;
}

bool RadixSort::SetBuffers(PxU32* ranks0, PxU32* ranks1, PxU32* histogram1024, PxU32** links256)
{
	if(!ranks0 || !ranks1 || !histogram1024 || !links256)	return false;

	mRanks			= ranks0;
	mRanks2			= ranks1;
	mHistogram1024	= histogram1024;
	mLinks256		= links256;
	mDeleteRanks	= false;
	INVALIDATE_RANKS;
	return true;
}

