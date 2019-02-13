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

#include "CmPhysXCommon.h"
#include "BpBroadPhaseSapAux.h"
#include "PsFoundation.h"

namespace physx
{

namespace Bp
{

PX_FORCE_INLINE void PxBpHandleSwap(BpHandle& a, BpHandle& b)													
{ 
	const BpHandle c = a; a = b; b = c;		
}

PX_FORCE_INLINE void Sort(BpHandle& id0, BpHandle& id1)										
{ 
	if(id0>id1)	PxBpHandleSwap(id0, id1);						
}

PX_FORCE_INLINE bool DifferentPair(const BroadPhasePair& p, BpHandle id0, BpHandle id1)	
{ 
	return (id0!=p.mVolA) || (id1!=p.mVolB);						
}

PX_FORCE_INLINE int Hash32Bits_1(int key)
{
	key += ~(key << 15);
	key ^=  (key >> 10);
	key +=  (key << 3);
	key ^=  (key >> 6);
	key += ~(key << 11);
	key ^=  (key >> 16);
	return key;
}

PX_FORCE_INLINE PxU32 Hash(BpHandle id0, BpHandle id1)								
{ 
	return PxU32(Hash32Bits_1( int(PxU32(id0)|(PxU32(id1)<<16)) ));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SapPairManager::SapPairManager() :
	mHashTable				(NULL),
	mNext					(NULL),
	mHashSize				(0),
	mHashCapacity			(0),
	mMinAllowedHashCapacity	(0),
	mActivePairs			(NULL),
	mActivePairStates		(NULL),
	mNbActivePairs			(0),
	mActivePairsCapacity	(0),
	mMask					(0)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SapPairManager::~SapPairManager()
{
	PX_ASSERT(NULL==mHashTable);
	PX_ASSERT(NULL==mNext);
	PX_ASSERT(NULL==mActivePairs);
	PX_ASSERT(NULL==mActivePairStates);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SapPairManager::init(const PxU32 size)
{
	mHashTable=reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16(sizeof(BpHandle)*size), "BpHandle"));
	mNext=reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16(sizeof(BpHandle)*size), "BpHandle"));
	mActivePairs=reinterpret_cast<BroadPhasePair*>(PX_ALLOC(ALIGN_SIZE_16(sizeof(BroadPhasePair)*size), "BroadPhasePair"));
	mActivePairStates=reinterpret_cast<PxU8*>(PX_ALLOC(ALIGN_SIZE_16(sizeof(PxU8)*size), "BroadPhaseContextSap ActivePairStates"));
	mHashCapacity=size;
	mMinAllowedHashCapacity = size;
	mActivePairsCapacity=size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SapPairManager::release()
{
	PX_FREE(mHashTable);
	PX_FREE(mNext);
	PX_FREE(mActivePairs);
	PX_FREE(mActivePairStates);
	mHashTable				= NULL;
	mNext					= NULL;
	mActivePairs			= NULL;
	mActivePairStates		= NULL;
	mNext					= 0;
	mHashSize				= 0;
	mHashCapacity			= 0;
	mMinAllowedHashCapacity	= 0;
	mNbActivePairs			= 0;
	mActivePairsCapacity	= 0;
	mMask					= 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const BroadPhasePair* SapPairManager::FindPair(BpHandle id0, BpHandle id1) const
{
	if(0==mHashSize) return NULL;	// Nothing has been allocated yet

	// Order the ids
	Sort(id0, id1);

	// Compute hash value for this pair
	PxU32 HashValue = Hash(id0, id1) & mMask;
	PX_ASSERT(HashValue<mHashCapacity);

	// Look for it in the table
	PX_ASSERT(HashValue<mHashCapacity);
	PxU32 Offset = mHashTable[HashValue];
	PX_ASSERT(BP_INVALID_BP_HANDLE==Offset || Offset<mActivePairsCapacity);
	while(Offset!=BP_INVALID_BP_HANDLE && DifferentPair(mActivePairs[Offset], id0, id1))
	{
		PX_ASSERT(mActivePairs[Offset].mVolA!=BP_INVALID_BP_HANDLE);
		PX_ASSERT(Offset<mHashCapacity);
		Offset = mNext[Offset];		// Better to have a separate array for this
		PX_ASSERT(BP_INVALID_BP_HANDLE==Offset || Offset<mActivePairsCapacity);
	}
	if(Offset==BP_INVALID_BP_HANDLE)	return NULL;
	PX_ASSERT(Offset<mNbActivePairs);
	// Match mActivePairs[Offset] => the pair is persistent
	PX_ASSERT(Offset<mActivePairsCapacity);
	return &mActivePairs[Offset];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Internal version saving hash computation
PX_FORCE_INLINE BroadPhasePair* SapPairManager::FindPair(BpHandle id0, BpHandle id1, PxU32 hash_value) const
{
	if(0==mHashSize) return NULL;	// Nothing has been allocated yet

	// Look for it in the table
	PX_ASSERT(hash_value<mHashCapacity);
	PxU32 Offset = mHashTable[hash_value];
	PX_ASSERT(BP_INVALID_BP_HANDLE==Offset || Offset<mActivePairsCapacity);
	while(Offset!=BP_INVALID_BP_HANDLE && DifferentPair(mActivePairs[Offset], id0, id1))
	{
		PX_ASSERT(mActivePairs[Offset].mVolA!=BP_INVALID_BP_HANDLE);
		PX_ASSERT(Offset<mHashCapacity);
		Offset = mNext[Offset];		// Better to have a separate array for this
		PX_ASSERT(BP_INVALID_BP_HANDLE==Offset || Offset<mActivePairsCapacity);
	}
	if(Offset==BP_INVALID_BP_HANDLE)	return NULL;
	PX_ASSERT(Offset<mNbActivePairs);
	// Match mActivePairs[Offset] => the pair is persistent
	PX_ASSERT(Offset<mActivePairsCapacity);
	return &mActivePairs[Offset];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const BroadPhasePair* SapPairManager::AddPair(BpHandle id0, BpHandle id1, const PxU8 state)
{
	if(MAX_BP_HANDLE == mNbActivePairs)
	{
		PX_WARN_ONCE(MAX_BP_PAIRS_MESSAGE);
		return NULL;
	}

	// Order the ids
	Sort(id0, id1);

	PxU32 HashValue = Hash(id0, id1) & mMask;

	BroadPhasePair* P = FindPair(id0, id1, HashValue);
	if(P)
	{
		return P;	// Persistent pair
	}

	// This is a new pair
	if(mNbActivePairs >= mHashSize)
	{
		// Get more entries
		mHashSize = Ps::nextPowerOfTwo(mNbActivePairs+1);
		mMask = mHashSize-1;

		reallocPairs(mHashSize>mHashCapacity);

		// Recompute hash value with new hash size
		HashValue = Hash(id0, id1) & mMask;
	}

	PX_ASSERT(mNbActivePairs<mActivePairsCapacity);
	BroadPhasePair* p = &mActivePairs[mNbActivePairs];
	p->mVolA		= id0;	// ### CMOVs would be nice here
	p->mVolB		= id1;
	mActivePairStates[mNbActivePairs]=state;

	PX_ASSERT(mNbActivePairs<mHashSize);
	PX_ASSERT(mNbActivePairs<mHashCapacity);
	PX_ASSERT(HashValue<mHashCapacity);
	mNext[mNbActivePairs] = mHashTable[HashValue];
	mHashTable[HashValue] = BpHandle(mNbActivePairs++);
	return p;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SapPairManager::RemovePair(BpHandle /*id0*/, BpHandle /*id1*/, PxU32 hash_value, PxU32 pair_index)
{
	// Walk the hash table to fix mNext
	{
		PX_ASSERT(hash_value<mHashCapacity);
		PxU32 Offset = mHashTable[hash_value];
		PX_ASSERT(Offset!=BP_INVALID_BP_HANDLE);

		PxU32 Previous=BP_INVALID_BP_HANDLE;
		while(Offset!=pair_index)
		{
			Previous = Offset;
			PX_ASSERT(Offset<mHashCapacity);
			Offset = mNext[Offset];
		}

		// Let us go/jump us
		if(Previous!=BP_INVALID_BP_HANDLE)
		{
			PX_ASSERT(Previous<mHashCapacity);
			PX_ASSERT(pair_index<mHashCapacity);
			PX_ASSERT(mNext[Previous]==pair_index);
			mNext[Previous] = mNext[pair_index];
		}
		// else we were the first
		else
		{
			PX_ASSERT(hash_value<mHashCapacity);
			PX_ASSERT(pair_index<mHashCapacity);
			mHashTable[hash_value] = mNext[pair_index];
		}
	}
	// we're now free to reuse mNext[PairIndex] without breaking the list

#if PX_DEBUG
	PX_ASSERT(pair_index<mHashCapacity);
	mNext[pair_index]=BP_INVALID_BP_HANDLE;
#endif
	// Invalidate entry

	// Fill holes
	{
		// 1) Remove last pair
		const PxU32 LastPairIndex = mNbActivePairs-1;
		if(LastPairIndex==pair_index)
		{
			mNbActivePairs--;
		}
		else
		{
			PX_ASSERT(LastPairIndex<mActivePairsCapacity);
			const BroadPhasePair* Last = &mActivePairs[LastPairIndex];
			const PxU32 LastHashValue = Hash(Last->mVolA, Last->mVolB) & mMask;

			// Walk the hash table to fix mNext
			PX_ASSERT(LastHashValue<mHashCapacity);
			PxU32 Offset = mHashTable[LastHashValue];
			PX_ASSERT(Offset!=BP_INVALID_BP_HANDLE);

			PxU32 Previous=BP_INVALID_BP_HANDLE;
			while(Offset!=LastPairIndex)
			{
				Previous = Offset;
				PX_ASSERT(Offset<mHashCapacity);
				Offset = mNext[Offset];
			}

			// Let us go/jump us
			if(Previous!=BP_INVALID_BP_HANDLE)
			{
				PX_ASSERT(Previous<mHashCapacity);
				PX_ASSERT(LastPairIndex<mHashCapacity);
				PX_ASSERT(mNext[Previous]==LastPairIndex);
				mNext[Previous] = mNext[LastPairIndex];
			}
			// else we were the first
			else
			{
				PX_ASSERT(LastHashValue<mHashCapacity);
				PX_ASSERT(LastPairIndex<mHashCapacity);
				mHashTable[LastHashValue] = mNext[LastPairIndex];
			}
			// we're now free to reuse mNext[LastPairIndex] without breaking the list

#if PX_DEBUG
			PX_ASSERT(LastPairIndex<mHashCapacity);
			mNext[LastPairIndex]=BP_INVALID_BP_HANDLE;
#endif

			// Don't invalidate entry since we're going to shrink the array

			// 2) Re-insert in free slot
			PX_ASSERT(pair_index<mActivePairsCapacity);
			PX_ASSERT(LastPairIndex<mActivePairsCapacity);
			mActivePairs[pair_index] = mActivePairs[LastPairIndex];
			mActivePairStates[pair_index] = mActivePairStates[LastPairIndex];
#if PX_DEBUG
			PX_ASSERT(pair_index<mHashCapacity);
			PX_ASSERT(mNext[pair_index]==BP_INVALID_BP_HANDLE);
#endif
			PX_ASSERT(pair_index<mHashCapacity);
			PX_ASSERT(LastHashValue<mHashCapacity);
			mNext[pair_index] = mHashTable[LastHashValue];
			mHashTable[LastHashValue] = BpHandle(pair_index);

			mNbActivePairs--;
		}
	}
}

bool SapPairManager::RemovePair(BpHandle id0, BpHandle id1)
{
	// Order the ids
	Sort(id0, id1);

	const PxU32 HashValue = Hash(id0, id1) & mMask;
	const BroadPhasePair* P = FindPair(id0, id1, HashValue);
	if(!P)	return false;
	PX_ASSERT(P->mVolA==id0);
	PX_ASSERT(P->mVolB==id1);

	RemovePair(id0, id1, HashValue, GetPairIndex(P));

	shrinkMemory();

	return true;
}

bool SapPairManager::RemovePairs(const Cm::BitMap& removedAABBs)
{
	PxU32 i=0;
	while(i<mNbActivePairs)
	{
		const BpHandle id0 = mActivePairs[i].mVolA;
		const BpHandle id1 = mActivePairs[i].mVolB;
		if(removedAABBs.test(id0) || removedAABBs.test(id1))
		{
			const PxU32 HashValue = Hash(id0, id1) & mMask;
			RemovePair(id0, id1, HashValue, i);
		}
		else i++;
	}
	return true;
}

void SapPairManager::shrinkMemory()
{
	//Compute the hash size given the current number of active pairs.
	const PxU32 correctHashSize = Ps::nextPowerOfTwo(mNbActivePairs);

	//If we have the correct hash size then no action required.
	if(correctHashSize==mHashSize || (correctHashSize < mMinAllowedHashCapacity && mHashSize == mMinAllowedHashCapacity))	
		return;

	//The hash size can be reduced so take action.
	//Don't let the hash size fall below a threshold value.
	PxU32 newHashSize = correctHashSize;
	if(newHashSize < mMinAllowedHashCapacity)
	{
		newHashSize = mMinAllowedHashCapacity;
	}
	mHashSize = newHashSize;
	mMask = newHashSize-1;

	reallocPairs( (newHashSize > mMinAllowedHashCapacity) || (mHashSize <= (mHashCapacity >> 2)) || (mHashSize <= (mActivePairsCapacity >> 2)));
}

void SapPairManager::reallocPairs(const bool allocRequired)
{
	if(allocRequired)
	{
		PX_FREE(mHashTable);
		mHashCapacity=mHashSize;
		mActivePairsCapacity=mHashSize;
		mHashTable = reinterpret_cast<BpHandle*>(PX_ALLOC(mHashSize*sizeof(BpHandle), "BpHandle"));

		for(PxU32 i=0;i<mHashSize;i++)	
		{
			mHashTable[i] = BP_INVALID_BP_HANDLE;
		}

		// Get some bytes for new entries
		BroadPhasePair* NewPairs	= reinterpret_cast<BroadPhasePair*>(PX_ALLOC(mHashSize * sizeof(BroadPhasePair), "BroadPhasePair"));	PX_ASSERT(NewPairs);
		BpHandle* NewNext			= reinterpret_cast<BpHandle*>(PX_ALLOC(mHashSize * sizeof(BpHandle), "BpHandle"));						PX_ASSERT(NewNext);
		PxU8* NewPairStates			= reinterpret_cast<PxU8*>(PX_ALLOC(mHashSize * sizeof(PxU8), "SapPairStates"));							PX_ASSERT(NewPairStates);

		// Copy old data if needed
		if(mNbActivePairs) 
		{
			PxMemCopy(NewPairs, mActivePairs, mNbActivePairs*sizeof(BroadPhasePair));
			PxMemCopy(NewPairStates, mActivePairStates, mNbActivePairs*sizeof(PxU8));
		}

		// ### check it's actually needed... probably only for pairs whose hash value was cut by the and
		// yeah, since Hash(id0, id1) is a constant
		// However it might not be needed to recompute them => only less efficient but still ok
		for(PxU32 i=0;i<mNbActivePairs;i++)
		{
			const PxU32 HashValue = Hash(mActivePairs[i].mVolA, mActivePairs[i].mVolB) & mMask;	// New hash value with new mask
			NewNext[i] = mHashTable[HashValue];
			PX_ASSERT(HashValue<mHashCapacity);
			mHashTable[HashValue] = BpHandle(i);
		}

		// Delete old data
		PX_FREE(mNext);
		PX_FREE(mActivePairs);
		PX_FREE(mActivePairStates);

		// Assign new pointer
		mActivePairs = NewPairs;
		mActivePairStates = NewPairStates;
		mNext = NewNext;
	}
	else
	{
		for(PxU32 i=0;i<mHashSize;i++)	
		{
			mHashTable[i] = BP_INVALID_BP_HANDLE;
		}

		// ### check it's actually needed... probably only for pairs whose hash value was cut by the and
		// yeah, since Hash(id0, id1) is a constant
		// However it might not be needed to recompute them => only less efficient but still ok
		for(PxU32 i=0;i<mNbActivePairs;i++)
		{
			const PxU32 HashValue = Hash(mActivePairs[i].mVolA, mActivePairs[i].mVolB) & mMask;	// New hash value with new mask
			mNext[i] = mHashTable[HashValue];
			PX_ASSERT(HashValue<mHashCapacity);
			mHashTable[HashValue] = BpHandle(i);
		}
	}
}

void resizeCreatedDeleted(BroadPhasePair*& pairs, PxU32& maxNumPairs)
{
	PX_ASSERT(pairs);
	PX_ASSERT(maxNumPairs>0);
	const PxU32 newMaxNumPairs=2*maxNumPairs;
	BroadPhasePair* newPairs=reinterpret_cast<BroadPhasePair*>(PX_ALLOC(sizeof(BroadPhasePair)*newMaxNumPairs, "BroadPhasePair"));
	PxMemCopy(newPairs, pairs, sizeof(BroadPhasePair)*maxNumPairs);
	PX_FREE(pairs);
	pairs=newPairs;
	maxNumPairs=newMaxNumPairs;
}

void ComputeCreatedDeletedPairsLists
(const Bp::FilterGroup::Enum* PX_RESTRICT boxGroups, 
 const BpHandle* PX_RESTRICT dataArray, const PxU32 dataArraySize,
 PxcScratchAllocator* scratchAllocator,
 BroadPhasePair*& createdPairsList, PxU32& numCreatedPairs, PxU32& maxNumCreatedPairs,
 BroadPhasePair*& deletedPairsList, PxU32& numDeletedPairs, PxU32& maxNumDeletedPairs,
 PxU32& numActualDeletedPairs,
 SapPairManager& pairManager)
{
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	PX_UNUSED(boxGroups);
#endif

	for(PxU32 i=0;i<dataArraySize;i++)
	{
		const PxU32 ID = dataArray[i];
		PX_ASSERT(ID<pairManager.mNbActivePairs);

		const BroadPhasePair* PX_RESTRICT UP = pairManager.mActivePairs + ID;
		PX_ASSERT(pairManager.IsInArray(UP));

		if(pairManager.IsRemoved(UP))
		{
			if(!pairManager.IsNew(UP))
			{
				// No need to call "ClearInArray" in this case, since the pair will get removed anyway
				if(numDeletedPairs==maxNumDeletedPairs)
				{
					BroadPhasePair* newDeletedPairsList = reinterpret_cast<BroadPhasePair*>(scratchAllocator->alloc(sizeof(BroadPhasePair)*2*maxNumDeletedPairs, true));
					PxMemCopy(newDeletedPairsList, deletedPairsList, sizeof(BroadPhasePair)*maxNumDeletedPairs);
					scratchAllocator->free(deletedPairsList);
					deletedPairsList = newDeletedPairsList;
					maxNumDeletedPairs = 2*maxNumDeletedPairs;
				}

				PX_ASSERT(numDeletedPairs<maxNumDeletedPairs);
				//PX_ASSERT((uintptr_t)UP->mUserData != 0xcdcdcdcd);
				deletedPairsList[numDeletedPairs++] = BroadPhasePair(UP->mVolA,UP->mVolB/*, ID*/);
			}
		}
		else
		{
			pairManager.ClearInArray(UP);
			// Add => already there... Might want to create user data, though
			if(pairManager.IsNew(UP))
			{
#if !BP_SAP_TEST_GROUP_ID_CREATEUPDATE
				if(groupFiltering(boxGroups[UP->mVolA], boxGroups[UP->mVolB]))
#endif
				{
					if(numCreatedPairs==maxNumCreatedPairs)
					{
						BroadPhasePair* newCreatedPairsList = reinterpret_cast<BroadPhasePair*>(scratchAllocator->alloc(sizeof(BroadPhasePair)*2*maxNumCreatedPairs, true));
						PxMemCopy(newCreatedPairsList, createdPairsList, sizeof(BroadPhasePair)*maxNumCreatedPairs);
						scratchAllocator->free(createdPairsList);
						createdPairsList = newCreatedPairsList;
						maxNumCreatedPairs = 2*maxNumCreatedPairs;
					}

					PX_ASSERT(numCreatedPairs<maxNumCreatedPairs);
					createdPairsList[numCreatedPairs++] = BroadPhasePair(UP->mVolA,UP->mVolB/*, ID*/);
				}
				pairManager.ClearNew(UP);
			}
		}
	}

	//Record pairs that are to be deleted because they were simultaneously created and removed 
	//from different axis sorts.
	numActualDeletedPairs=numDeletedPairs;
	for(PxU32 i=0;i<dataArraySize;i++)
	{
		const PxU32 ID = dataArray[i];
		PX_ASSERT(ID<pairManager.mNbActivePairs);
		const BroadPhasePair* PX_RESTRICT UP = pairManager.mActivePairs + ID;
		if(pairManager.IsRemoved(UP) && pairManager.IsNew(UP))
		{
			PX_ASSERT(pairManager.IsInArray(UP));

			if(numActualDeletedPairs==maxNumDeletedPairs)
			{
				BroadPhasePair* newDeletedPairsList = reinterpret_cast<BroadPhasePair*>(scratchAllocator->alloc(sizeof(BroadPhasePair)*2*maxNumDeletedPairs, true));
				PxMemCopy(newDeletedPairsList, deletedPairsList, sizeof(BroadPhasePair)*maxNumDeletedPairs);
				scratchAllocator->free(deletedPairsList);
				deletedPairsList = newDeletedPairsList;
				maxNumDeletedPairs = 2*maxNumDeletedPairs;
			}

			PX_ASSERT(numActualDeletedPairs<=maxNumDeletedPairs);
			deletedPairsList[numActualDeletedPairs++] = BroadPhasePair(UP->mVolA,UP->mVolB/*, ID*/); //KS - should we even get here????
		}
	}

//	// #### try batch removal here
//	for(PxU32 i=0;i<numActualDeletedPairs;i++)
//	{
//		const BpHandle id0 = deletedPairsList[i].mVolA;
//		const BpHandle id1 = deletedPairsList[i].mVolB;
//#if PX_DEBUG
//		const bool Status = pairManager.RemovePair(id0, id1);
//		PX_ASSERT(Status);
//#else
//		pairManager.RemovePair(id0, id1);
//#endif
//	}

	//Only report deleted pairs from different groups.
#if !BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	for(PxU32 i=0;i<numDeletedPairs;i++)
	{
		const PxU32 id0 = deletedPairsList[i].mVolA;
		const PxU32 id1 = deletedPairsList[i].mVolB;
		if(!groupFiltering(boxGroups[id0], boxGroups[id1]))
		{
			while((numDeletedPairs-1) > i && boxGroups[deletedPairsList[numDeletedPairs-1].mVolA] == boxGroups[deletedPairsList[numDeletedPairs-1].mVolB])
			{
				numDeletedPairs--;
			}
			deletedPairsList[i]=deletedPairsList[numDeletedPairs-1];
			numDeletedPairs--;
		}
	}
#endif
}

void DeletePairsLists(const PxU32 numActualDeletedPairs, BroadPhasePair* deletedPairsList, SapPairManager& pairManager)
{
	// #### try batch removal here
	for(PxU32 i=0;i<numActualDeletedPairs;i++)
	{
		const BpHandle id0 = deletedPairsList[i].mVolA;
		const BpHandle id1 = deletedPairsList[i].mVolB;
#if PX_DEBUG
		const bool Status = pairManager.RemovePair(id0, id1);
		PX_ASSERT(Status);
#else
		pairManager.RemovePair(id0, id1);
#endif
	}
}

//#define PRINT_STATS
#ifdef PRINT_STATS
	#include <stdio.h>
	static PxU32 gNbIter = 0;
	static PxU32 gNbTests = 0;
	static  PxU32 gNbPairs = 0;
	#define	START_STATS				gNbIter = gNbTests = gNbPairs = 0;
	#define	INCREASE_STATS_NB_ITER	gNbIter++;
	#define	INCREASE_STATS_NB_TESTS	gNbTests++;
	#define	INCREASE_STATS_NB_PAIRS	gNbPairs++;
	#define	DUMP_STATS				printf("%d %d %d\n", gNbIter, gNbTests, gNbPairs);
#else
	#define	START_STATS
	#define	INCREASE_STATS_NB_ITER
	#define	INCREASE_STATS_NB_TESTS
	#define	INCREASE_STATS_NB_PAIRS
	#define	DUMP_STATS
#endif

void DataArray::Resize(PxcScratchAllocator* scratchAllocator)
{
	BpHandle* newDataArray = reinterpret_cast<BpHandle*>(scratchAllocator->alloc(sizeof(BpHandle)*mCapacity*2, true));
	PxMemCopy(newDataArray, mData, mCapacity*sizeof(BpHandle));
	scratchAllocator->free(mData);
	mData = newDataArray;
	mCapacity *= 2;
}

static PX_FORCE_INLINE int intersect2D(const BoxYZ& a, const BoxYZ& b)
{
	const bool b0 = b.mMaxY < a.mMinY;
	const bool b1 = a.mMaxY < b.mMinY;
	const bool b2 = b.mMaxZ < a.mMinZ;
	const bool b3 = a.mMaxZ < b.mMinZ;
//	const bool b4 = b0 || b1 || b2 || b3;
	const bool b4 = b0 | b1 | b2 | b3;
	return !b4;
}

void addPair(const BpHandle id0, const BpHandle id1, PxcScratchAllocator* scratchAllocator, SapPairManager& pairManager, DataArray& dataArray)
{
	const BroadPhasePair* UP = reinterpret_cast<const BroadPhasePair*>(pairManager.AddPair(id0, id1, SapPairManager::PAIR_UNKNOWN));

	//If the hash table has reached its limit then we're unable to add a new pair.
	if(NULL==UP)
		return;

	PX_ASSERT(UP);
	if(pairManager.IsUnknown(UP))
	{
		pairManager.ClearState(UP);
		pairManager.SetInArray(UP);
		dataArray.AddData(pairManager.GetPairIndex(UP), scratchAllocator);
		pairManager.SetNew(UP);
	}
	pairManager.ClearRemoved(UP);
}

void removePair(BpHandle id0, BpHandle id1, PxcScratchAllocator* scratchAllocator, SapPairManager& pairManager, DataArray& dataArray)
{
	const BroadPhasePair* UP = reinterpret_cast<const BroadPhasePair*>(pairManager.FindPair(id0, id1));
	if(UP)
	{
		if(!pairManager.IsInArray(UP))
		{
			pairManager.SetInArray(UP);
			dataArray.AddData(pairManager.GetPairIndex(UP), scratchAllocator);
		}
		pairManager.SetRemoved(UP);
	}
}

struct AddPairParams
{
	AddPairParams(const PxU32* remap0, const PxU32* remap1, PxcScratchAllocator* alloc, SapPairManager* pm, DataArray* da) :
		mRemap0				(remap0),
		mRemap1				(remap1),
		mScratchAllocator	(alloc),
		mPairManager		(pm),
		mDataArray			(da)
	{
	}

	const PxU32*			mRemap0;
	const PxU32*			mRemap1;
	PxcScratchAllocator*	mScratchAllocator;
	SapPairManager*			mPairManager;
	DataArray*				mDataArray;
};

static void addPair(const AddPairParams* PX_RESTRICT params, const BpHandle id0_, const BpHandle id1_)
{
	SapPairManager& pairManager = *params->mPairManager;

	const BroadPhasePair* UP = reinterpret_cast<const BroadPhasePair*>(pairManager.AddPair(params->mRemap0[id0_], params->mRemap1[id1_], SapPairManager::PAIR_UNKNOWN));

	//If the hash table has reached its limit then we're unable to add a new pair.
	if(NULL==UP)
		return;

	PX_ASSERT(UP);
	if(pairManager.IsUnknown(UP))
	{
		pairManager.ClearState(UP);
		pairManager.SetInArray(UP);
		params->mDataArray->AddData(pairManager.GetPairIndex(UP), params->mScratchAllocator);
		pairManager.SetNew(UP);
	}
	pairManager.ClearRemoved(UP);
}

// PT: TODO: use SIMD

AuxData::AuxData(PxU32 nb, const SapBox1D*const* PX_RESTRICT boxes, const BpHandle* PX_RESTRICT indicesSorted, const Bp::FilterGroup::Enum* PX_RESTRICT groupIds)
{
	// PT: TODO: use scratch allocator / etc
	BoxX* PX_RESTRICT boxX						= reinterpret_cast<BoxX*>(PX_ALLOC(sizeof(BoxX)*(nb+1), PX_DEBUG_EXP("mBoxX")));
	BoxYZ* PX_RESTRICT boxYZ					= reinterpret_cast<BoxYZ*>(PX_ALLOC(sizeof(BoxYZ)*nb, PX_DEBUG_EXP("mBoxYZ")));
	Bp::FilterGroup::Enum* PX_RESTRICT groups	= reinterpret_cast<Bp::FilterGroup::Enum*>(PX_ALLOC(sizeof(Bp::FilterGroup::Enum)*nb, PX_DEBUG_EXP("mGroups")));
	PxU32* PX_RESTRICT remap					= reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nb, PX_DEBUG_EXP("mRemap")));

	mBoxX = boxX;
	mBoxYZ = boxYZ;
	mGroups = groups;
	mRemap = remap;
	mNb = nb;

	const PxU32 axis0 = 0;
	const PxU32 axis1 = 2;
	const PxU32 axis2 = 1;

	const SapBox1D* PX_RESTRICT boxes0 = boxes[axis0];
	const SapBox1D* PX_RESTRICT boxes1 = boxes[axis1];
	const SapBox1D* PX_RESTRICT boxes2 = boxes[axis2];

	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 boxID = indicesSorted[i];
		groups[i] = groupIds[boxID];
		remap[i] = boxID;

		const SapBox1D& currentBoxX = boxes0[boxID];
		boxX[i].mMinX = currentBoxX.mMinMax[0];
		boxX[i].mMaxX = currentBoxX.mMinMax[1];

		const SapBox1D& currentBoxY = boxes1[boxID];
		boxYZ[i].mMinY = currentBoxY.mMinMax[0];
		boxYZ[i].mMaxY = currentBoxY.mMinMax[1];

		const SapBox1D& currentBoxZ = boxes2[boxID];
		boxYZ[i].mMinZ = currentBoxZ.mMinMax[0];
		boxYZ[i].mMaxZ = currentBoxZ.mMinMax[1];
	}
	boxX[nb].mMinX = 0xffffffff;
}

AuxData::~AuxData()
{
	PX_FREE(mRemap);
	PX_FREE(mGroups);
	PX_FREE(mBoxYZ);
	PX_FREE(mBoxX);
}

void performBoxPruningNewNew(	const AuxData* PX_RESTRICT auxData, PxcScratchAllocator* scratchAllocator,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								const bool* lut,
#endif
								SapPairManager& pairManager, BpHandle*& dataArray, PxU32& dataArraySize, PxU32& dataArrayCapacity)
{
	const PxU32 nb = auxData->mNb;
	if(!nb)
		return;

	DataArray da(dataArray, dataArraySize, dataArrayCapacity);

	START_STATS
	{
		BoxX* boxX = auxData->mBoxX;
		BoxYZ* boxYZ = auxData->mBoxYZ;
		Bp::FilterGroup::Enum* groups = auxData->mGroups;
		PxU32* remap = auxData->mRemap;

		AddPairParams params(remap, remap, scratchAllocator, &pairManager, &da);

		PxU32 runningIndex = 0;
		PxU32 index0 = 0;

		while(runningIndex<nb && index0<nb)
		{
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
			const Bp::FilterGroup::Enum group0 = groups[index0];
#endif
			const BoxX& boxX0 = boxX[index0];

			const BpHandle minLimit = boxX0.mMinX;
			while(boxX[runningIndex++].mMinX<minLimit);

			const BpHandle maxLimit = boxX0.mMaxX;
			PxU32 index1 = runningIndex;
			while(boxX[index1].mMinX <= maxLimit)
			{
				INCREASE_STATS_NB_ITER
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
				if(groupFiltering(group0, groups[index1], lut))
	#else
				if(groupFiltering(group0, groups[index1]))
	#endif
#endif
				{
					INCREASE_STATS_NB_TESTS
					if(intersect2D(boxYZ[index0], boxYZ[index1]))
/*					__m128i b = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&boxYZ[index0].mMinY));
					b = _mm_shuffle_epi32(b, 78);
					const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&boxYZ[index1].mMinY));
					const __m128i d = _mm_cmpgt_epi32(a, b);
					const int mask = _mm_movemask_epi8(d);
					if(mask==0x0000ff00)*/
					{
						INCREASE_STATS_NB_PAIRS
						addPair(&params, index0, index1);
					}
				}
				index1++;
			}
			index0++;
		}
	}
	DUMP_STATS

	dataArray = da.mData;
	dataArraySize = da.mSize;
	dataArrayCapacity = da.mCapacity;
}

template<int codepath>
static void bipartitePruning(
	const PxU32 nb0, const BoxX* PX_RESTRICT boxX0, const BoxYZ* PX_RESTRICT boxYZ0, const PxU32* PX_RESTRICT remap0, const Bp::FilterGroup::Enum* PX_RESTRICT groups0,
	const PxU32 nb1, const BoxX* PX_RESTRICT boxX1, const BoxYZ* PX_RESTRICT boxYZ1, const PxU32* PX_RESTRICT remap1, const Bp::FilterGroup::Enum* PX_RESTRICT groups1,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	const bool* lut,
#endif
	PxcScratchAllocator* scratchAllocator, SapPairManager& pairManager, DataArray& dataArray
	)
{
	AddPairParams params(remap0, remap1, scratchAllocator, &pairManager, &dataArray);

	PxU32 runningIndex = 0;
	PxU32 index0 = 0;

	while(runningIndex<nb1 && index0<nb0)
	{
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
		const Bp::FilterGroup::Enum group0 = groups0[index0];
#endif

		const BpHandle minLimit = boxX0[index0].mMinX;
		if(!codepath)
		{
			while(boxX1[runningIndex].mMinX<minLimit)
				runningIndex++;
		}
		else
		{
			while(boxX1[runningIndex].mMinX<=minLimit)
				runningIndex++;
		}

		const BpHandle maxLimit = boxX0[index0].mMaxX;
		PxU32 index1 = runningIndex;
		while(boxX1[index1].mMinX <= maxLimit)
		{
			INCREASE_STATS_NB_ITER
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
			if(groupFiltering(group0, groups1[index1], lut))
	#else
			if(groupFiltering(group0, groups1[index1]))
	#endif
#endif
			{
				INCREASE_STATS_NB_TESTS
				if(intersect2D(boxYZ0[index0], boxYZ1[index1]))
				{
					INCREASE_STATS_NB_PAIRS
					addPair(&params, index0, index1);
				}
			}
			index1++;
		}
		index0++;
	}
}

void performBoxPruningNewOld(	const AuxData* PX_RESTRICT auxData0, const AuxData* PX_RESTRICT auxData1, PxcScratchAllocator* scratchAllocator, 
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								const bool* lut,
#endif
								SapPairManager& pairManager, BpHandle*& dataArray, PxU32& dataArraySize, PxU32& dataArrayCapacity)
{
	const PxU32 nb0 = auxData0->mNb;
	const PxU32 nb1 = auxData1->mNb;

	if(!nb0 || !nb1)
		return;

	DataArray da(dataArray, dataArraySize, dataArrayCapacity);

	START_STATS
	{
		const BoxX* boxX0 = auxData0->mBoxX;
		const BoxYZ* boxYZ0 = auxData0->mBoxYZ;
		const Bp::FilterGroup::Enum* groups0 = auxData0->mGroups;
		const PxU32* remap0 = auxData0->mRemap;

		const BoxX* boxX1 = auxData1->mBoxX;
		const BoxYZ* boxYZ1 = auxData1->mBoxYZ;
		const Bp::FilterGroup::Enum* groups1 = auxData1->mGroups;
		const PxU32* remap1 = auxData1->mRemap;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		bipartitePruning<0>(nb0, boxX0, boxYZ0, remap0, groups0, nb1, boxX1, boxYZ1, remap1, groups1, lut, scratchAllocator, pairManager, da);
		bipartitePruning<1>(nb1, boxX1, boxYZ1, remap1, groups1, nb0, boxX0, boxYZ0, remap0, groups0, lut, scratchAllocator, pairManager, da);
#else
		bipartitePruning<0>(nb0, boxX0, boxYZ0, remap0, groups0, nb1, boxX1, boxYZ1, remap1, groups1, scratchAllocator, pairManager, da);
		bipartitePruning<1>(nb1, boxX1, boxYZ1, remap1, groups1, nb0, boxX0, boxYZ0, remap0, groups0, scratchAllocator, pairManager, da);
#endif
	}
	DUMP_STATS

	dataArray = da.mData;
	dataArraySize = da.mSize;
	dataArrayCapacity = da.mCapacity;
}

} //namespace Bp

} //namespace physx

