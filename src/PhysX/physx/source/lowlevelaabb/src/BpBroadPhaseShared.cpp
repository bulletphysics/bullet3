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

#include "BpBroadPhaseShared.h"
#include "foundation/PxMemory.h"
#include "PsBitUtils.h"

using namespace physx;
using namespace Bp;

#define MBP_ALLOC(x)		PX_ALLOC(x, "MBP")
#define MBP_FREE(x)			if(x)	PX_FREE_AND_RESET(x)

static PX_FORCE_INLINE void storeDwords(PxU32* dest, PxU32 nb, PxU32 value)
{
	while(nb--)
		*dest++ = value;
}

///////////////////////////////////////////////////////////////////////////////

PairManagerData::PairManagerData() :
	mHashSize		(0),
	mMask			(0),
	mNbActivePairs	(0),
	mHashTable		(NULL),
	mNext			(NULL),
	mActivePairs	(NULL),
	mReservedMemory (0)
{
}

///////////////////////////////////////////////////////////////////////////////

PairManagerData::~PairManagerData()
{
	purge();
}

///////////////////////////////////////////////////////////////////////////////

void PairManagerData::purge()
{
	MBP_FREE(mNext);
	MBP_FREE(mActivePairs);
	MBP_FREE(mHashTable);
	mHashSize		= 0;
	mMask			= 0;
	mNbActivePairs	= 0;
}

///////////////////////////////////////////////////////////////////////////////

void PairManagerData::reallocPairs()
{
	MBP_FREE(mHashTable);
	mHashTable = reinterpret_cast<PxU32*>(MBP_ALLOC(mHashSize*sizeof(PxU32)));
	storeDwords(mHashTable, mHashSize, INVALID_ID);

	// Get some bytes for new entries
	InternalPair* newPairs	= reinterpret_cast<InternalPair*>(MBP_ALLOC(mHashSize * sizeof(InternalPair)));	PX_ASSERT(newPairs);
	PxU32* newNext			= reinterpret_cast<PxU32*>(MBP_ALLOC(mHashSize * sizeof(PxU32)));		PX_ASSERT(newNext);

	// Copy old data if needed
	if(mNbActivePairs)
		PxMemCopy(newPairs, mActivePairs, mNbActivePairs*sizeof(InternalPair));
	// ### check it's actually needed... probably only for pairs whose hash value was cut by the and
	// yeah, since hash(id0, id1) is a constant
	// However it might not be needed to recompute them => only less efficient but still ok
	for(PxU32 i=0;i<mNbActivePairs;i++)
	{
		const PxU32 hashValue = hash(mActivePairs[i].getId0(), mActivePairs[i].getId1()) & mMask;	// New hash value with new mask
		newNext[i] = mHashTable[hashValue];
		mHashTable[hashValue] = i;
	}

	// Delete old data
	MBP_FREE(mNext);
	MBP_FREE(mActivePairs);

	// Assign new pointer
	mActivePairs = newPairs;
	mNext = newNext;
}

///////////////////////////////////////////////////////////////////////////////

void PairManagerData::shrinkMemory()
{
	// Check correct memory against actually used memory
	const PxU32 correctHashSize = Ps::nextPowerOfTwo(mNbActivePairs);
	if(mHashSize==correctHashSize)
		return;

	if(mReservedMemory && correctHashSize < mReservedMemory)
		return;

	// Reduce memory used
	mHashSize = correctHashSize;
	mMask = mHashSize-1;

	reallocPairs();
}

///////////////////////////////////////////////////////////////////////////////

void PairManagerData::reserveMemory(PxU32 memSize)
{
	if(!memSize)
		return;

	if(!Ps::isPowerOfTwo(memSize))
		memSize = Ps::nextPowerOfTwo(memSize);

	mHashSize = memSize;
	mMask = mHashSize-1;

	mReservedMemory = memSize;

	reallocPairs();
}

///////////////////////////////////////////////////////////////////////////////

PX_NOINLINE PxU32 PairManagerData::growPairs(PxU32 fullHashValue)
{
	// Get more entries
	mHashSize = Ps::nextPowerOfTwo(mNbActivePairs+1);
	mMask = mHashSize-1;

	reallocPairs();

	// Recompute hash value with new hash size
	return fullHashValue & mMask;
}

///////////////////////////////////////////////////////////////////////////////

void PairManagerData::removePair(PxU32 /*id0*/, PxU32 /*id1*/, PxU32 hashValue, PxU32 pairIndex)
{
	// Walk the hash table to fix mNext
	{
		PxU32 offset = mHashTable[hashValue];
		PX_ASSERT(offset!=INVALID_ID);

		PxU32 previous=INVALID_ID;
		while(offset!=pairIndex)
		{
			previous = offset;
			offset = mNext[offset];
		}

		// Let us go/jump us
		if(previous!=INVALID_ID)
		{
			PX_ASSERT(mNext[previous]==pairIndex);
			mNext[previous] = mNext[pairIndex];
		}
		// else we were the first
		else mHashTable[hashValue] = mNext[pairIndex];
		// we're now free to reuse mNext[pairIndex] without breaking the list
	}
#if PX_DEBUG
	mNext[pairIndex]=INVALID_ID;
#endif
	// Invalidate entry

	// Fill holes
	{
		// 1) Remove last pair
		const PxU32 lastPairIndex = mNbActivePairs-1;
		if(lastPairIndex==pairIndex)
		{
			mNbActivePairs--;
		}
		else
		{
			const InternalPair* last = &mActivePairs[lastPairIndex];
			const PxU32 lastHashValue = hash(last->getId0(), last->getId1()) & mMask;

			// Walk the hash table to fix mNext
			PxU32 offset = mHashTable[lastHashValue];
			PX_ASSERT(offset!=INVALID_ID);

			PxU32 previous=INVALID_ID;
			while(offset!=lastPairIndex)
			{
				previous = offset;
				offset = mNext[offset];
			}

			// Let us go/jump us
			if(previous!=INVALID_ID)
			{
				PX_ASSERT(mNext[previous]==lastPairIndex);
				mNext[previous] = mNext[lastPairIndex];
			}
			// else we were the first
			else mHashTable[lastHashValue] = mNext[lastPairIndex];
			// we're now free to reuse mNext[lastPairIndex] without breaking the list

#if PX_DEBUG
			mNext[lastPairIndex]=INVALID_ID;
#endif

			// Don't invalidate entry since we're going to shrink the array

			// 2) Re-insert in free slot
			mActivePairs[pairIndex] = mActivePairs[lastPairIndex];
#if PX_DEBUG
			PX_ASSERT(mNext[pairIndex]==INVALID_ID);
#endif
			mNext[pairIndex] = mHashTable[lastHashValue];
			mHashTable[lastHashValue] = pairIndex;

			mNbActivePairs--;
		}
	}
}

