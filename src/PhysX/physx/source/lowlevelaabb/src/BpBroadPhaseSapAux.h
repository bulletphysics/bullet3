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

#ifndef BP_BROADPHASE_SAP_AUX_H
#define BP_BROADPHASE_SAP_AUX_H

#include "foundation/PxAssert.h"
#include "CmPhysXCommon.h"
#include "PsIntrinsics.h"
#include "PsUserAllocated.h"
#include "BpBroadPhase.h"
#include "BpBroadPhaseUpdate.h"
#include "CmBitMap.h"
#include "PxcScratchAllocator.h"

namespace physx
{
namespace Bp
{
#define NUM_SENTINELS 2

#define BP_SAP_USE_PREFETCH 1//prefetch in batchUpdate

#define BP_SAP_USE_OVERLAP_TEST_ON_REMOVES	1// "Useless" but faster overall because seriously reduces number of calls (from ~10000 to ~3 sometimes!)

//Set 1 to test for group ids in batchCreate/batchUpdate so we can avoid group id test in ComputeCreatedDeletedPairsLists
//Set 0 to neglect group id test in batchCreate/batchUpdate and delay test until ComputeCreatedDeletedPairsLists
#define BP_SAP_TEST_GROUP_ID_CREATEUPDATE 1

#define MAX_BP_HANDLE			0x3fffffff
#define PX_REMOVED_BP_HANDLE	0x3ffffffd
#define MAX_BP_PAIRS_MESSAGE "Only 4294967296 broadphase pairs are supported.  This limit has been exceeded and some pairs will be dropped \n"

PX_FORCE_INLINE	void setMinSentinel(ValType& v, BpHandle& d)
{
	v = 0x00000000;//0x00800000;  //0x00800000 is -FLT_MAX but setting it to 0 means we don't crash when we get a value outside the float range.
	d = (BP_INVALID_BP_HANDLE & ~1);
}

PX_FORCE_INLINE	void setMaxSentinel(ValType& v, BpHandle& d)
{										
	v = 0xffffffff;//0xff7fffff;  //0xff7fffff is +FLT_MAX but setting it to 0xffffffff means we don't crash when we get a value outside the float range.
	d = BP_INVALID_BP_HANDLE;
}

PX_FORCE_INLINE	BpHandle setData(PxU32 owner_box_id, const bool is_max)
{
	BpHandle d = BpHandle(owner_box_id<<1);
	if(is_max)	d |= 1;
	return d;
}

PX_FORCE_INLINE	bool isSentinel(const BpHandle& d)	
{ 
	return (d&~1)==(BP_INVALID_BP_HANDLE & ~1);	
}

PX_FORCE_INLINE	BpHandle isMax(const BpHandle& d) 	
{
	return BpHandle(d & 1);		
}

PX_FORCE_INLINE	BpHandle getOwner(const BpHandle& d) 	
{ 
	return BpHandle(d>>1);		
}

class SapBox1D
{
public:

	PX_FORCE_INLINE			SapBox1D()	{}
	PX_FORCE_INLINE			~SapBox1D()	{}

	BpHandle		mMinMax[2];//mMinMax[0]=min, mMinMax[1]=max
};

class SapPairManager
{
public:
							SapPairManager();
							~SapPairManager();

	void					init(const PxU32 size);
	void					release();

	void					shrinkMemory();

	const BroadPhasePair*	AddPair		(BpHandle id0, BpHandle id1, const PxU8 state);
	bool					RemovePair	(BpHandle id0, BpHandle id1);
	bool					RemovePairs	(const Cm::BitMap& removedAABBs);
	const BroadPhasePair*	FindPair	(BpHandle id0, BpHandle id1)	const;

	PX_FORCE_INLINE	PxU32	GetPairIndex(const BroadPhasePair* PX_RESTRICT pair)	const
	{
		return (PxU32((size_t(pair) - size_t(mActivePairs)))/sizeof(BroadPhasePair));
	}

	BpHandle*			mHashTable;
	BpHandle*			mNext;
	PxU32				mHashSize;
	PxU32				mHashCapacity;
	PxU32				mMinAllowedHashCapacity;
	BroadPhasePair*		mActivePairs;
	PxU8*				mActivePairStates;
	PxU32				mNbActivePairs;
	PxU32				mActivePairsCapacity;
	PxU32				mMask;

	BroadPhasePair*		FindPair	(BpHandle id0, BpHandle id1, PxU32 hash_value) const;
	void				RemovePair	(BpHandle id0, BpHandle id1, PxU32 hash_value, PxU32 pair_index);
	void				reallocPairs(const bool allocRequired);

	enum
	{
		PAIR_INARRAY=1,
		PAIR_REMOVED=2,
		PAIR_NEW=4,
		PAIR_UNKNOWN=8
	};

	PX_FORCE_INLINE bool IsInArray(const BroadPhasePair* PX_RESTRICT pair) const 
	{
		const PxU8 state=mActivePairStates[pair-mActivePairs];
		return state & PAIR_INARRAY ? true : false;
	}
	PX_FORCE_INLINE bool IsRemoved(const BroadPhasePair* PX_RESTRICT pair) const 
	{
		const PxU8 state=mActivePairStates[pair-mActivePairs];
		return state & PAIR_REMOVED ? true : false;
	}
	PX_FORCE_INLINE bool IsNew(const BroadPhasePair* PX_RESTRICT pair) const
	{
		const PxU8 state=mActivePairStates[pair-mActivePairs];
		return state & PAIR_NEW ? true : false;
	}
	PX_FORCE_INLINE bool IsUnknown(const BroadPhasePair* PX_RESTRICT pair) const
	{
		const PxU8 state=mActivePairStates[pair-mActivePairs];
		return state & PAIR_UNKNOWN ? true : false;
	}

	PX_FORCE_INLINE void ClearState(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs]=0;
	}

	PX_FORCE_INLINE void SetInArray(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs] |= PAIR_INARRAY;
	}
	PX_FORCE_INLINE void SetRemoved(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs] |= PAIR_REMOVED;
	}
	PX_FORCE_INLINE void SetNew(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs] |= PAIR_NEW;
	}
	PX_FORCE_INLINE void ClearInArray(const BroadPhasePair* PX_RESTRICT pair)
	{		
		mActivePairStates[pair-mActivePairs] &= ~PAIR_INARRAY;
	}
	PX_FORCE_INLINE void ClearRemoved(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs] &= ~PAIR_REMOVED;
	}
	PX_FORCE_INLINE void ClearNew(const BroadPhasePair* PX_RESTRICT pair)
	{
		mActivePairStates[pair-mActivePairs] &= ~PAIR_NEW;
	}
};

struct DataArray
{
	DataArray(BpHandle* data, PxU32 size, PxU32 capacity) : mData(data), mSize(size), mCapacity(capacity)	{}

	BpHandle*	mData;
	PxU32		mSize;
	PxU32		mCapacity;

	PX_NOINLINE		void	Resize(PxcScratchAllocator* scratchAllocator);

	PX_FORCE_INLINE	void	AddData(const PxU32 data, PxcScratchAllocator* scratchAllocator)
	{
		if(mSize==mCapacity)
			Resize(scratchAllocator);

		PX_ASSERT(mSize<mCapacity);
		mData[mSize++] = BpHandle(data);
	}
};

void addPair(const BpHandle id0, const BpHandle id1, PxcScratchAllocator* scratchAllocator, SapPairManager& pairManager, DataArray& dataArray);
void removePair(BpHandle id0, BpHandle id1, PxcScratchAllocator* scratchAllocator, SapPairManager& pairManager, DataArray& dataArray);

void ComputeCreatedDeletedPairsLists
(const Bp::FilterGroup::Enum* PX_RESTRICT boxGroups, 
 const BpHandle* PX_RESTRICT dataArray, const PxU32 dataArraySize,
 PxcScratchAllocator* scratchAllocator,
 BroadPhasePair* & createdPairsList, PxU32& numCreatedPairs, PxU32& maxNumCreatdPairs,
 BroadPhasePair* & deletedPairsList, PxU32& numDeletedPairs, PxU32& maxNumDeletedPairs,
 PxU32&numActualDeletedPairs,
 SapPairManager& pairManager);

void DeletePairsLists(const PxU32 numActualDeletedPairs, BroadPhasePair* deletedPairsList, SapPairManager& pairManager);

	struct BoxX
	{
		PxU32	mMinX;
		PxU32	mMaxX;
	};

	struct BoxYZ
	{
		PxU32	mMinY;
		PxU32	mMinZ;
		PxU32	mMaxY;
		PxU32	mMaxZ;
	};

	struct AuxData
	{
		AuxData(PxU32 nb, const SapBox1D*const* PX_RESTRICT boxes, const BpHandle* PX_RESTRICT indicesSorted, const Bp::FilterGroup::Enum* PX_RESTRICT groupIds);
		~AuxData();

		BoxX*					mBoxX;
		BoxYZ*					mBoxYZ;
		Bp::FilterGroup::Enum*	mGroups;
		PxU32*					mRemap;
		PxU32					mNb;
	};

void performBoxPruningNewNew(	const AuxData* PX_RESTRICT auxData, PxcScratchAllocator* scratchAllocator,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								const bool* lut,
#endif
								SapPairManager& pairManager, BpHandle*& dataArray, PxU32& dataArraySize, PxU32& dataArrayCapacity);

void performBoxPruningNewOld(	const AuxData* PX_RESTRICT auxData0, const AuxData* PX_RESTRICT auxData1, PxcScratchAllocator* scratchAllocator,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								const bool* lut,
#endif
								SapPairManager& pairManager, BpHandle*& dataArray, PxU32& dataArraySize, PxU32& dataArrayCapacity);

PX_FORCE_INLINE bool Intersect2D_Handle
(const BpHandle bDir1Min, const BpHandle bDir1Max, const BpHandle bDir2Min, const BpHandle bDir2Max,
 const BpHandle cDir1Min, const BpHandle cDir1Max, const BpHandle cDir2Min, const BpHandle cDir2Max)
{
	return (bDir1Max > cDir1Min && cDir1Max > bDir1Min && 
			bDir2Max > cDir2Min && cDir2Max > bDir2Min);        
}

} //namespace Bp

} //namespace physx

#endif //BP_BROADPHASE_SAP_AUX_H
