///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *	OPCODE - Optimized Collision Detection
 *	Copyright (C) 2001 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/Opcode.htm
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains an array-based version of the sweep-and-prune algorithm
 *	\file		OPC_ArraySAP.cpp
 *	\author		Pierre Terdiman
 *	\date		December, 2, 2007
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "StdAfx.h"

using namespace Opcode;

//#include "SAP_Utils.h"

#define INVALID_USER_ID	0xffff

inline_ void Sort(uword& id0, uword& id1)										{ if(id0>id1)	TSwap(id0, id1);						}
inline_ void Sort(uword& id0, uword& id1, const void*& obj0, const void*& obj1)	{ if(id0>id1)	{ TSwap(id0, id1); TSwap(obj0, obj1);	}	}


	struct Opcode::IAABB : public Allocateable
	{
		udword mMinX;
		udword mMinY;
		udword mMinZ;
		udword mMaxX;
		udword mMaxY;
		udword mMaxZ;

		inline_ udword	GetMin(udword i)	const	{	return (&mMinX)[i];	}
		inline_ udword	GetMax(udword i)	const	{	return (&mMaxX)[i];	}
	};



/*
	- already sorted for batch create?
	- better axis selection batch create
*/

//#define USE_WORDS		// Use words or dwords for box indices. Words save memory but seriously limit the max number of objects in the SAP.
#define USE_PREFETCH
#define USE_INTEGERS
#define PAIR_USER_DATA
#define USE_OVERLAP_TEST_ON_REMOVES	// "Useless" but faster overall because seriously reduces number of calls (from ~10000 to ~3 sometimes!)
#define RELEASE_ON_RESET	// Release memory instead of just doing a reset

#include "OPC_ArraySAP.h"

//#include "SAP_PairManager.h"
//#include "SAP_PairManager.cpp"


inline_ udword Hash(uword id0, uword id1)								{ return Hash32Bits_1( udword(id0)|(udword(id1)<<16) );		}
inline_ bool DifferentPair(const ASAP_Pair& p, uword id0, uword id1)	{ return (id0!=p.id0) || (id1!=p.id1);						}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ASAP_PairManager::ASAP_PairManager() :
	mHashSize		(0),
	mMask			(0),
	mHashTable		(null),
	mNext			(null),
	mNbActivePairs	(0),
	mActivePairs	(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ASAP_PairManager::~ASAP_PairManager()
{
	Purge();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ASAP_PairManager::Purge()
{
	ICE_FREE(mNext);
	ICE_FREE(mActivePairs);
	ICE_FREE(mHashTable);
	mHashSize		= 0;
	mMask			= 0;
	mNbActivePairs	= 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const ASAP_Pair* ASAP_PairManager::FindPair(uword id0, uword id1) const
{
	if(!mHashTable)	return null;	// Nothing has been allocated yet

	// Order the ids
	Sort(id0, id1);

	// Compute hash value for this pair
	udword HashValue = Hash(id0, id1) & mMask;

	// Look for it in the table
	udword Offset = mHashTable[HashValue];
	while(Offset!=INVALID_ID && DifferentPair(mActivePairs[Offset], id0, id1))
	{
		ASSERT(mActivePairs[Offset].id0!=INVALID_USER_ID);
		Offset = mNext[Offset];		// Better to have a separate array for this
	}
	if(Offset==INVALID_ID)	return null;
	ASSERT(Offset<mNbActivePairs);
	// Match mActivePairs[Offset] => the pair is persistent
	return &mActivePairs[Offset];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Internal version saving hash computation
inline_ ASAP_Pair* ASAP_PairManager::FindPair(uword id0, uword id1, udword hash_value) const
{
	if(!mHashTable)	return null;	// Nothing has been allocated yet

	// Look for it in the table
	udword Offset = mHashTable[hash_value];
	while(Offset!=INVALID_ID && DifferentPair(mActivePairs[Offset], id0, id1))
	{
		ASSERT(mActivePairs[Offset].id0!=INVALID_USER_ID);
		Offset = mNext[Offset];		// Better to have a separate array for this
	}
	if(Offset==INVALID_ID)	return null;
	ASSERT(Offset<mNbActivePairs);
	// Match mActivePairs[Offset] => the pair is persistent
	return &mActivePairs[Offset];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const ASAP_Pair* ASAP_PairManager::AddPair(uword id0, uword id1, const void* object0, const void* object1)
{
	// Order the ids
	Sort(id0, id1, object0, object1);

	udword HashValue = Hash(id0, id1) & mMask;

	ASAP_Pair* P = FindPair(id0, id1, HashValue);
	if(P)
	{
		return P;	// Persistent pair
	}

	// This is a new pair
	if(mNbActivePairs >= mHashSize)
	{
		// Get more entries
		mHashSize = NextPowerOfTwo(mNbActivePairs+1);
		mMask = mHashSize-1;

		ReallocPairs();

		// Recompute hash value with new hash size
		HashValue = Hash(id0, id1) & mMask;
	}

	ASAP_Pair* p = &mActivePairs[mNbActivePairs];
	p->id0		= id0;	// ### CMOVs would be nice here
	p->id1		= id1;
	p->object0	= object0;
	p->object1	= object1;

	mNext[mNbActivePairs] = mHashTable[HashValue];
	mHashTable[HashValue] = mNbActivePairs++;
	return p;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ASAP_PairManager::RemovePair(uword id0, uword id1, udword hash_value, udword pair_index)
{
	// Walk the hash table to fix mNext
	udword Offset = mHashTable[hash_value];
	ASSERT(Offset!=INVALID_ID);

	udword Previous=INVALID_ID;
	while(Offset!=pair_index)
	{
		Previous = Offset;
		Offset = mNext[Offset];
	}

	// Let us go/jump us
	if(Previous!=INVALID_ID)
	{
		ASSERT(mNext[Previous]==pair_index);
		mNext[Previous] = mNext[pair_index];
	}
	// else we were the first
	else mHashTable[hash_value] = mNext[pair_index];
	// we're now free to reuse mNext[PairIndex] without breaking the list

#ifdef _DEBUG
	mNext[pair_index]=INVALID_ID;
#endif
	// Invalidate entry

	// Fill holes
	if(1)
	{
		// 1) Remove last pair
		const udword LastPairIndex = mNbActivePairs-1;
		if(LastPairIndex==pair_index)
		{
			mNbActivePairs--;
		}
		else
		{
			const ASAP_Pair* Last = &mActivePairs[LastPairIndex];
			const udword LastHashValue = Hash(Last->id0, Last->id1) & mMask;

			// Walk the hash table to fix mNext
			udword Offset = mHashTable[LastHashValue];
			ASSERT(Offset!=INVALID_ID);

			udword Previous=INVALID_ID;
			while(Offset!=LastPairIndex)
			{
				Previous = Offset;
				Offset = mNext[Offset];
			}

			// Let us go/jump us
			if(Previous!=INVALID_ID)
			{
				ASSERT(mNext[Previous]==LastPairIndex);
				mNext[Previous] = mNext[LastPairIndex];
			}
			// else we were the first
			else mHashTable[LastHashValue] = mNext[LastPairIndex];
			// we're now free to reuse mNext[LastPairIndex] without breaking the list

#ifdef _DEBUG
			mNext[LastPairIndex]=INVALID_ID;
#endif

			// Don't invalidate entry since we're going to shrink the array

			// 2) Re-insert in free slot
			mActivePairs[pair_index] = mActivePairs[LastPairIndex];
#ifdef _DEBUG
			ASSERT(mNext[pair_index]==INVALID_ID);
#endif
			mNext[pair_index] = mHashTable[LastHashValue];
			mHashTable[LastHashValue] = pair_index;

			mNbActivePairs--;
		}
	}
}

bool ASAP_PairManager::RemovePair(uword id0, uword id1)
{
	// Order the ids
	Sort(id0, id1);

	const udword HashValue = Hash(id0, id1) & mMask;
	const ASAP_Pair* P = FindPair(id0, id1, HashValue);
	if(!P)	return false;
	ASSERT(P->id0==id0);
	ASSERT(P->id1==id1);

	RemovePair(id0, id1, HashValue, GetPairIndex(P));

	ShrinkMemory();
	return true;
}

bool ASAP_PairManager::RemovePairs(const BitArray& array)
{
	udword i=0;
	while(i<mNbActivePairs)
	{
		const uword id0 = mActivePairs[i].id0;
		const uword id1 = mActivePairs[i].id1;
		if(array.IsSet(id0) || array.IsSet(id1))
		{
			const udword HashValue = Hash(id0, id1) & mMask;
			RemovePair(id0, id1, HashValue, i);
		}
		else i++;
	}
	ShrinkMemory();
	return true;
}

void ASAP_PairManager::ShrinkMemory()
{
	// Check correct memory against actually used memory
	const udword CorrectHashSize = NextPowerOfTwo(mNbActivePairs);
	if(mHashSize==CorrectHashSize)	return;

	// Reduce memory used
	mHashSize = CorrectHashSize;
	mMask = mHashSize-1;

	ReallocPairs();
}

void ASAP_PairManager::ReallocPairs()
{
	ICE_FREE(mHashTable);
	mHashTable = (udword*)ICE_ALLOC(mHashSize*sizeof(udword));
	StoreDwords(mHashTable, mHashSize, INVALID_ID);

	// Get some bytes for new entries
	ASAP_Pair* NewPairs	= (ASAP_Pair*)ICE_ALLOC(mHashSize * sizeof(ASAP_Pair));	ASSERT(NewPairs);
	udword* NewNext		= (udword*)ICE_ALLOC(mHashSize * sizeof(udword));		ASSERT(NewNext);

	// Copy old data if needed
	if(mNbActivePairs)	CopyMemory(NewPairs, mActivePairs, mNbActivePairs*sizeof(ASAP_Pair));
	// ### check it's actually needed... probably only for pairs whose hash value was cut by the and
	// yeah, since Hash(id0, id1) is a constant
	// However it might not be needed to recompute them => only less efficient but still ok
	for(udword i=0;i<mNbActivePairs;i++)
	{
		const udword HashValue = Hash(mActivePairs[i].id0, mActivePairs[i].id1) & mMask;	// New hash value with new mask
		NewNext[i] = mHashTable[HashValue];
		mHashTable[HashValue] = i;
	}

	// Delete old data
	ICE_FREE(mNext);
	ICE_FREE(mActivePairs);

	// Assign new pointer
	mActivePairs = NewPairs;
	mNext = NewNext;
}


#ifdef USE_WORDS
	typedef uword			IndexType;
	#define	INVALID_INDEX	0xffff
#else
	typedef udword			IndexType;
	#define	INVALID_INDEX	0xffffffff
#endif

#ifdef USE_INTEGERS
	typedef udword	ValType;
	typedef IAABB	SAP_AABB;
#else
	typedef float	ValType;
	typedef AABB	SAP_AABB;
#endif

	struct Opcode::CreateData
	{
		udword	mHandle;
		AABB	mBox;
	};

	class Opcode::ASAP_EndPoint : public Allocateable
	{
		public:
		inline_					ASAP_EndPoint()		{}
		inline_					~ASAP_EndPoint()	{}

				ValType			mValue;		// Min or Max value
				udword			mData;		// Parent box | MinMax flag
		public:

		inline_	bool			IsSentinel()	const	{ return (mData&~3)==0xfffffffc;	}

		inline_	void			SetData(ValType v, udword owner_box_id, BOOL is_max)
								{
									mValue = v;
									mData = owner_box_id<<2;
									if(is_max)	mData |= 3;
								}
		inline_	BOOL			IsMax()		const	{ return mData & 3;		}
		inline_	udword			GetOwner()	const	{ return mData>>2;		}
	};

	class Opcode::ASAP_Box : public Allocateable
	{
		public:
		inline_					ASAP_Box()	{}
		inline_					~ASAP_Box()	{}

				IndexType		mMin[3];
				IndexType		mMax[3];
				void*			mObject;
				udword			mGUID;

		inline_	void			SetInvalid()			{ mMin[0]=INVALID_INDEX;			}
		inline_	bool			IsValid()		const	{ return mMin[0]!=INVALID_INDEX;	}

		inline_	ValType			GetMaxValue(udword i, const ASAP_EndPoint* base)	const
								{
									return base[mMax[i]].mValue;
								}

		inline_	ValType			GetMinValue(udword i, const ASAP_EndPoint* base)	const
								{
									return base[mMin[i]].mValue;
								}
#ifdef _DEBUG
				bool			HasBeenInserted()	const
								{
									assert(mMin[0]!=INVALID_INDEX);
									assert(mMax[0]!=INVALID_INDEX);
									assert(mMin[1]!=INVALID_INDEX);
									assert(mMax[1]!=INVALID_INDEX);
									assert(mMin[2]!=INVALID_INDEX);
									assert(mMax[2]!=INVALID_INDEX);
									return true;
								}
#endif
	};

inline_ BOOL Intersect1D_Min(const SAP_AABB& a, const ASAP_Box& b, const ASAP_EndPoint* const base, udword axis)
{
	if(b.GetMaxValue(axis, base) < a.GetMin(axis))
		return FALSE;
	return TRUE;
}

inline_ BOOL Intersect2D(const ASAP_Box& c, const ASAP_Box& b, udword axis1, udword axis2)
{
	if(		b.mMax[axis1] < c.mMin[axis1] || c.mMax[axis1] < b.mMin[axis1]
		||	b.mMax[axis2] < c.mMin[axis2] || c.mMax[axis2] < b.mMin[axis2])	return FALSE;
	return TRUE;
}


ArraySAP::ArraySAP()
{
	mNbBoxes	= 0;
	mMaxNbBoxes	= 0;
	mBoxes		= null;
	mEndPoints[0] = mEndPoints[1] = mEndPoints[2] = null;
	mFirstFree	= INVALID_ID;
}

ArraySAP::~ArraySAP()
{
	mNbBoxes	= 0;
	mMaxNbBoxes	= 0;
	DELETEARRAY(mBoxes);
	for(udword i=0;i<3;i++)
	{
		DELETEARRAY(mEndPoints[i]);
	}
}

void ArraySAP::ResizeBoxArray()
{
	const udword NewMaxBoxes = mMaxNbBoxes ? mMaxNbBoxes*2 : 64;

	ASAP_Box* NewBoxes = ICE_NEW_TMP(ASAP_Box)[NewMaxBoxes];
	const udword NbSentinels=2;
	ASAP_EndPoint* NewEndPointsX = ICE_NEW_TMP(ASAP_EndPoint)[NewMaxBoxes*2+NbSentinels];
	ASAP_EndPoint* NewEndPointsY = ICE_NEW_TMP(ASAP_EndPoint)[NewMaxBoxes*2+NbSentinels];
	ASAP_EndPoint* NewEndPointsZ = ICE_NEW_TMP(ASAP_EndPoint)[NewMaxBoxes*2+NbSentinels];

	if(mNbBoxes)
	{
		CopyMemory(NewBoxes, mBoxes, sizeof(ASAP_Box)*mNbBoxes);
		CopyMemory(NewEndPointsX, mEndPoints[0], sizeof(ASAP_EndPoint)*(mNbBoxes*2+NbSentinels));
		CopyMemory(NewEndPointsY, mEndPoints[1], sizeof(ASAP_EndPoint)*(mNbBoxes*2+NbSentinels));
		CopyMemory(NewEndPointsZ, mEndPoints[2], sizeof(ASAP_EndPoint)*(mNbBoxes*2+NbSentinels));
	}
	else
	{
		// Initialize sentinels
#ifdef USE_INTEGERS
		const udword Min = EncodeFloat(MIN_FLOAT);
		const udword Max = EncodeFloat(MAX_FLOAT);
#else
		const float Min = MIN_FLOAT;
		const float Max = MAX_FLOAT;
#endif
		NewEndPointsX[0].SetData(Min, INVALID_INDEX, FALSE);
		NewEndPointsX[1].SetData(Max, INVALID_INDEX, TRUE);
		NewEndPointsY[0].SetData(Min, INVALID_INDEX, FALSE);
		NewEndPointsY[1].SetData(Max, INVALID_INDEX, TRUE);
		NewEndPointsZ[0].SetData(Min, INVALID_INDEX, FALSE);
		NewEndPointsZ[1].SetData(Max, INVALID_INDEX, TRUE);
	}
	DELETEARRAY(mBoxes);
	DELETEARRAY(mEndPoints[2]);
	DELETEARRAY(mEndPoints[1]);
	DELETEARRAY(mEndPoints[0]);
	mBoxes = NewBoxes;
	mEndPoints[0] = NewEndPointsX;
	mEndPoints[1] = NewEndPointsY;
	mEndPoints[2] = NewEndPointsZ;

	mMaxNbBoxes = NewMaxBoxes;
}

inline_ BOOL Intersect(const IAABB& a, const IAABB& b, udword axis)
{
	if(b.GetMax(axis) < a.GetMin(axis) || a.GetMax(axis) < b.GetMin(axis))	return FALSE;
	return TRUE;
}

// ### TODO: the sorts here might be useless, as the values have been sorted already
bool ArraySAP::CompleteBoxPruning2(udword nb, const IAABB* array, const Axes& axes, const CreateData* batched)
{
	// Checkings
	if(!nb || !array)	return false;

	// Catch axes
	const udword Axis0 = axes.mAxis0;
	const udword Axis1 = axes.mAxis1;
	const udword Axis2 = axes.mAxis2;

	// Allocate some temporary data
	udword* PosList = (udword*)ICE_ALLOC_TMP(sizeof(udword)*(nb+1));

	// 1) Build main list using the primary axis
	for(udword i=0;i<nb;i++)	PosList[i] = array[i].GetMin(Axis0);
//PosList[nb++] = ConvertToSortable(MAX_FLOAT);

	// 2) Sort the list
//	static RadixSort r;
	RadixSort r;
	RadixSort* RS = &r;

//	const udword* Sorted = RS->Sort(PosList, nb, RADIX_SIGNED).GetRanks();
	const udword* Sorted = RS->Sort(PosList, nb, RADIX_UNSIGNED).GetRanks();

	// 3) Prune the list
	const udword* const LastSorted = &Sorted[nb];
	const udword* RunningAddress = Sorted;
	udword Index0, Index1;
	while(RunningAddress<LastSorted && Sorted<LastSorted)
	{
		Index0 = *Sorted++;

		while(RunningAddress<LastSorted && PosList[*RunningAddress++]<PosList[Index0]);
//		while(PosList[*RunningAddress++]<PosList[Index0]);

		if(RunningAddress<LastSorted)
		{
			const udword* RunningAddress2 = RunningAddress;

			while(RunningAddress2<LastSorted && PosList[Index1 = *RunningAddress2++]<=array[Index0].GetMax(Axis0))
//			while(PosList[Index1 = *RunningAddress2++]<=(udword)ConvertToSortable(array[Index0].GetMax(Axis0)))
			{
				if(Intersect(array[Index0], array[Index1], Axis1))
				{
					if(Intersect(array[Index0], array[Index1], Axis2))
					{
						const ASAP_Box* Box0 = mBoxes + batched[Index0].mHandle;
						const ASAP_Box* Box1 = mBoxes + batched[Index1].mHandle;
						AddPair(Box0->mObject, Box1->mObject, Box0->mGUID, Box1->mGUID);
					}
				}
			}
		}
	}

	ICE_FREE(PosList);
	return true;
}

bool ArraySAP::BipartiteBoxPruning2(udword nb0, const IAABB* array0, udword nb1, const IAABB* array1, const Axes& axes, const CreateData* batched, const udword* box_indices)
{
	// Checkings
	if(!nb0 || !array0 || !nb1 || !array1)	return false;

	// Catch axes
	const udword Axis0 = axes.mAxis0;
	const udword Axis1 = axes.mAxis1;
	const udword Axis2 = axes.mAxis2;

	// Allocate some temporary data
	udword* MinPosList0 = (udword*)ICE_ALLOC_TMP(sizeof(udword)*nb0);
	udword* MinPosList1 = (udword*)ICE_ALLOC_TMP(sizeof(udword)*nb1);

	// 1) Build main lists using the primary axis
	for(udword i=0;i<nb0;i++)	MinPosList0[i] = array0[i].GetMin(Axis0);
	for(udword i=0;i<nb1;i++)	MinPosList1[i] = array1[i].GetMin(Axis0);

	// 2) Sort the lists
/*	static RadixSort r0;
	static RadixSort r1;
	RadixSort* RS0 = &r0;
	RadixSort* RS1 = &r1;*/
	RadixSort r0;
	RadixSort r1;
	RadixSort* RS0 = &r0;
	RadixSort* RS1 = &r1;

	const udword* Sorted0 = RS0->Sort(MinPosList0, nb0, RADIX_UNSIGNED).GetRanks();
	const udword* Sorted1 = RS1->Sort(MinPosList1, nb1, RADIX_UNSIGNED).GetRanks();

	// 3) Prune the lists
	udword Index0, Index1;

	const udword* const LastSorted0 = &Sorted0[nb0];
	const udword* const LastSorted1 = &Sorted1[nb1];
	const udword* RunningAddress0 = Sorted0;
	const udword* RunningAddress1 = Sorted1;

	while(RunningAddress1<LastSorted1 && Sorted0<LastSorted0)
	{
		Index0 = *Sorted0++;

		while(RunningAddress1<LastSorted1 && MinPosList1[*RunningAddress1]<MinPosList0[Index0])	RunningAddress1++;

		const udword* RunningAddress2_1 = RunningAddress1;

		while(RunningAddress2_1<LastSorted1 && MinPosList1[Index1 = *RunningAddress2_1++]<=array0[Index0].GetMax(Axis0))
		{
			if(Intersect(array0[Index0], array1[Index1], Axis1))
			{
				if(Intersect(array0[Index0], array1[Index1], Axis2))
				{
					const ASAP_Box* Box0 = mBoxes + batched[Index0].mHandle;
					const ASAP_Box* Box1 = mBoxes + box_indices[Index1];
					AddPair(Box0->mObject, Box1->mObject, Box0->mGUID, Box1->mGUID);
				}
			}
		}
	}

	////

	while(RunningAddress0<LastSorted0 && Sorted1<LastSorted1)
	{
		Index0 = *Sorted1++;

		while(RunningAddress0<LastSorted0 && MinPosList0[*RunningAddress0]<=MinPosList1[Index0])	RunningAddress0++;

		const udword* RunningAddress2_0 = RunningAddress0;

		while(RunningAddress2_0<LastSorted0 && MinPosList0[Index1 = *RunningAddress2_0++]<=array1[Index0].GetMax(Axis0))
		{
			if(Intersect(array0[Index1], array1[Index0], Axis1))
			{
				if(Intersect(array0[Index1], array1[Index0], Axis2))
				{
					const ASAP_Box* Box0 = mBoxes + batched[Index1].mHandle;
					const ASAP_Box* Box1 = mBoxes + box_indices[Index0];
					AddPair(Box0->mObject, Box1->mObject, Box0->mGUID, Box1->mGUID);
				}
			}

		}
	}

	ICE_FREE(MinPosList0);
	ICE_FREE(MinPosList1);
	return true;
}

udword ArraySAP::AddObject(void* object, uword guid, const AABB& box)
{
	assert(!(size_t(object)&3));	// We will use the 2 LSBs

#ifdef _DEBUG
	int a = sizeof(ASAP_Box);		// 32
	int b = sizeof(ASAP_EndPoint);	// 8
#endif

	udword BoxIndex;
	if(mFirstFree!=INVALID_ID)
	{
		BoxIndex = mFirstFree;
		mFirstFree = mBoxes[BoxIndex].mGUID;
	}
	else
	{
		if(mNbBoxes==mMaxNbBoxes)
			ResizeBoxArray();
		BoxIndex = mNbBoxes;
	}

	ASAP_Box* Box = &mBoxes[BoxIndex];
	// Initialize box
	Box->mObject	= object;
	Box->mGUID		= guid;
	for(udword i=0;i<3;i++)
	{
		Box->mMin[i] = INVALID_INDEX;
		Box->mMax[i] = INVALID_INDEX;
	}

	mNbBoxes++;

	CreateData* CD = (CreateData*)mCreated.Reserve(sizeof(CreateData)/sizeof(udword));
	CD->mHandle = BoxIndex;
	CD->mBox = box;

	return BoxIndex;
}

void ArraySAP::InsertEndPoints(udword axis, const ASAP_EndPoint* end_points, udword nb_endpoints)
{
	ASAP_EndPoint* const BaseEP = mEndPoints[axis];

	const udword OldSize = mNbBoxes*2 - nb_endpoints;
	const udword NewSize = mNbBoxes*2;

	BaseEP[NewSize + 1] = BaseEP[OldSize + 1];

	sdword WriteIdx = NewSize;
	udword CurrInsIdx = 0;

	const ASAP_EndPoint* First = &BaseEP[0];
	const ASAP_EndPoint* Current = &BaseEP[OldSize];
	while(Current>=First)
	{
		const ASAP_EndPoint& Src = *Current;
		const ASAP_EndPoint& Ins = end_points[CurrInsIdx];

		// We need to make sure we insert maxs before mins to handle exactly equal endpoints correctly
		const bool ShouldInsert = Ins.IsMax() ? (Src.mValue <= Ins.mValue) : (Src.mValue < Ins.mValue);

		const ASAP_EndPoint& Moved = ShouldInsert ? Ins : Src;
		BaseEP[WriteIdx] = Moved;
		mBoxes[Moved.GetOwner()].mMin[axis + Moved.IsMax()] = WriteIdx--;

		if(ShouldInsert)
		{
			CurrInsIdx++;
			if(CurrInsIdx >= nb_endpoints)
				break;//we just inserted the last endpoint
		}
		else
		{
			Current--;
		}
	}
}

void ArraySAP::BatchCreate()
{
	udword NbBatched = mCreated.GetNbEntries();
	if(!NbBatched)	return;	// Early-exit if no object has been created
	NbBatched /= sizeof(CreateData)/sizeof(udword);
	const CreateData* Batched = (const CreateData*)mCreated.GetEntries();
	mCreated.Reset();

	{
	const udword NbEndPoints = NbBatched*2;
	ASAP_EndPoint* NewEPSorted = ICE_NEW_TMP(ASAP_EndPoint)[NbEndPoints];
	ASAP_EndPoint* Buffer = (ASAP_EndPoint*)ICE_ALLOC_TMP(sizeof(ASAP_EndPoint)*NbEndPoints);
	RadixSort RS;

	for(udword Axis=0;Axis<3;Axis++)
	{
		for(udword i=0;i<NbBatched;i++)
		{
			const udword BoxIndex = (udword)Batched[i].mHandle;
			assert(mBoxes[BoxIndex].mMin[Axis]==INVALID_INDEX);
			assert(mBoxes[BoxIndex].mMax[Axis]==INVALID_INDEX);

			const float MinValue = Batched[i].mBox.GetMin(Axis);
			const float MaxValue = Batched[i].mBox.GetMax(Axis);

			NewEPSorted[i*2+0].SetData(EncodeFloat(MinValue), BoxIndex, FALSE);
			NewEPSorted[i*2+1].SetData(EncodeFloat(MaxValue), BoxIndex, TRUE);
		}

		// Sort endpoints backwards
		{
			udword* Keys = (udword*)Buffer;
			for(udword i=0;i<NbEndPoints;i++)
				Keys[i] = NewEPSorted[i].mValue;

			const udword* Sorted = RS.Sort(Keys, NbEndPoints, RADIX_UNSIGNED).GetRanks();

			for(udword i=0;i<NbEndPoints;i++)
				Buffer[i] = NewEPSorted[Sorted[NbEndPoints-1-i]];
		}

		InsertEndPoints(Axis, Buffer, NbEndPoints);
	}

	ICE_FREE(Buffer);
	DELETEARRAY(NewEPSorted);
	}

#ifdef _DEBUG
	for(udword i=0;i<NbBatched;i++)
	{
		udword BoxIndex = (udword)Batched[i].mHandle;
		ASAP_Box* Box = mBoxes + BoxIndex;
		assert(Box->HasBeenInserted());
	}
	for(udword i=0;i<mNbBoxes*2+1;i++)
	{
		assert(mEndPoints[0][i].mValue <= mEndPoints[0][i+1].mValue);
		assert(mEndPoints[1][i].mValue <= mEndPoints[1][i+1].mValue);
		assert(mEndPoints[2][i].mValue <= mEndPoints[2][i+1].mValue);
	}
#endif

	if(1)
	{
		BitArray BA(mMaxNbBoxes);

		// Using box-pruning on array indices....
		IAABB* NewBoxes = ICE_NEW_TMP(IAABB)[NbBatched];
		for(udword i=0;i<NbBatched;i++)
		{
			const ASAP_Box* Box = mBoxes + (udword)Batched[i].mHandle;
			assert((udword)Batched[i].mHandle<mMaxNbBoxes);
			BA.SetBit((udword)Batched[i].mHandle);
			NewBoxes[i].mMinX = Box->mMin[0];
			NewBoxes[i].mMaxX = Box->mMax[0];
			NewBoxes[i].mMinY = Box->mMin[1];
			NewBoxes[i].mMaxY = Box->mMax[1];
			NewBoxes[i].mMinZ = Box->mMin[2];
			NewBoxes[i].mMaxZ = Box->mMax[2];
		}

		CompleteBoxPruning2(NbBatched, NewBoxes, Axes(AXES_XZY), Batched);

		// the old boxes are not the first ones in the array

		const udword NbOldBoxes = mNbBoxes - NbBatched;
		if(NbOldBoxes)
		{
			IAABB* OldBoxes = ICE_NEW_TMP(IAABB)[NbOldBoxes];
			udword* OldBoxesIndices = (udword*)ICE_ALLOC_TMP(sizeof(udword)*NbOldBoxes);
			udword Offset=0;
			udword i=0;
			while(i<NbOldBoxes)
			{


				if(!BA.IsSet(i))
				{
					const ASAP_Box* Box = mBoxes + i;
//					if(Box->mObject)
					if(Box->IsValid())
					{
						OldBoxesIndices[Offset] = i;

						OldBoxes[Offset].mMinX = Box->mMin[0];
						OldBoxes[Offset].mMaxX = Box->mMax[0];
						OldBoxes[Offset].mMinY = Box->mMin[1];
						OldBoxes[Offset].mMaxY = Box->mMax[1];
						OldBoxes[Offset].mMinZ = Box->mMin[2];
						OldBoxes[Offset].mMaxZ = Box->mMax[2];
						Offset++;
					}
				}
				i++;
			}
//			assert(i==NbOldBoxes);
			BipartiteBoxPruning2(NbBatched, NewBoxes, Offset, OldBoxes, Axes(AXES_XZY), Batched, OldBoxesIndices);

			ICE_FREE(OldBoxesIndices);
			DELETEARRAY(OldBoxes);
		}
		DELETEARRAY(NewBoxes);
	}
#ifdef RELEASE_ON_RESET
	mCreated.Empty();
#endif
}

void ArraySAP::BatchRemove()
{
	udword NbRemoved = mRemoved.GetNbEntries();
	if(!NbRemoved)	return;	// Early-exit if no object has been removed
	const udword* Removed = mRemoved.GetEntries();
	mRemoved.Reset();

	for(udword Axis=0;Axis<3;Axis++)
	{
		ASAP_EndPoint* const BaseEP = mEndPoints[Axis];
		udword MinMinIndex = MAX_UDWORD;
		for(udword i=0;i<NbRemoved;i++)
		{
			assert(Removed[i]<mMaxNbBoxes);
			const ASAP_Box* RemovedObject = mBoxes + Removed[i];
			const udword MinIndex = RemovedObject->mMin[Axis];
			assert(MinIndex<mMaxNbBoxes*2+2);
			const udword MaxIndex = RemovedObject->mMax[Axis];
			assert(MaxIndex<mMaxNbBoxes*2+2);
			assert(BaseEP[MinIndex].GetOwner()==Removed[i]);
			assert(BaseEP[MaxIndex].GetOwner()==Removed[i]);
			BaseEP[MinIndex].mData = 0xfffffffe;
			BaseEP[MaxIndex].mData = 0xfffffffe;
			if(MinIndex<MinMinIndex)	MinMinIndex = MinIndex;
		}

		udword ReadIndex = MinMinIndex;
		udword DestIndex = MinMinIndex;
		const udword Limit = mNbBoxes*2+2;
		while(ReadIndex!=Limit)
		{
			while(ReadIndex!=Limit && BaseEP[ReadIndex].mData == 0xfffffffe)
			{
				ReadIndex++;
			}
			if(ReadIndex!=Limit)
			{
				if(ReadIndex!=DestIndex)
				{
					BaseEP[DestIndex] = BaseEP[ReadIndex];
					assert(BaseEP[DestIndex].mData != 0xfffffffe);

					if(!BaseEP[DestIndex].IsSentinel())
					{
						udword BoxOwner = BaseEP[DestIndex].GetOwner();
						assert(BoxOwner<mMaxNbBoxes);
						ASAP_Box* Box = mBoxes + BoxOwner;

						Box->mMin[Axis + BaseEP[DestIndex].IsMax()] = DestIndex;
					}
				}
				DestIndex++;
				ReadIndex++;
			}
		}
	}

	BitArray BA(65536);
	const udword Saved = NbRemoved;
	while(NbRemoved--)
	{
		udword Index = *Removed++;
		assert(Index<mMaxNbBoxes);

		ASAP_Box* Object = mBoxes + Index;
		assert(Object->mGUID < 65536);
		BA.SetBit(Object->mGUID);

		Object->mGUID = mFirstFree;
//		Object->mObject = null;	// ###########
		Object->SetInvalid();
		mFirstFree = Index;
	}
	mNbBoxes -= Saved;
	mPairs.RemovePairs(BA);

#ifdef RELEASE_ON_RESET
	mRemoved.Empty();
#endif
}

bool ArraySAP::RemoveObject(udword handle)
{
	mRemoved.Add(handle);
	return true;
}

#ifdef USE_INTEGERS
bool ArraySAP::UpdateObject(udword handle, const AABB& box_)
#else
bool ArraySAP::UpdateObject(udword handle, const AABB& box)
#endif
{
	const ASAP_Box* Object = mBoxes + handle;
	assert(Object->HasBeenInserted());
	const void* UserObject = Object->mObject;
	const udword UserGUID = Object->mGUID;

#ifdef USE_INTEGERS
	IAABB box;
	box.mMinX = EncodeFloat(box_.GetMin(0));
	box.mMinY = EncodeFloat(box_.GetMin(1));
	box.mMinZ = EncodeFloat(box_.GetMin(2));
	box.mMaxX = EncodeFloat(box_.GetMax(0));
	box.mMaxY = EncodeFloat(box_.GetMax(1));
	box.mMaxZ = EncodeFloat(box_.GetMax(2));
#endif

	for(udword Axis=0;Axis<3;Axis++)
	{
		const udword Axis1 = (1  << Axis) & 3;
		const udword Axis2 = (1  << Axis1) & 3;

		ASAP_EndPoint* const BaseEP = mEndPoints[Axis];

		// Update min
		{
			ASAP_EndPoint* CurrentMin = BaseEP + Object->mMin[Axis];
			ASSERT(!CurrentMin->IsMax());

			const ValType Limit = box.GetMin(Axis);
			if(Limit < CurrentMin->mValue)
			{
				CurrentMin->mValue = Limit;

				// Min is moving left:
				ASAP_EndPoint Saved = *CurrentMin;
				udword EPIndex = (size_t(CurrentMin) - size_t(BaseEP))/sizeof(ASAP_EndPoint);
				const udword SavedIndex = EPIndex;

				while((--CurrentMin)->mValue > Limit)
				{
#ifdef USE_PREFETCH
					_prefetch(CurrentMin-1);
#endif
					ASAP_Box* id1 = mBoxes + CurrentMin->GetOwner();
					const BOOL IsMax = CurrentMin->IsMax();
					if(IsMax)
					{
						// Our min passed a max => start overlap
						if(Object!=id1
							&& Intersect2D(*Object, *id1, Axis1, Axis2)
							&& Intersect1D_Min(box, *id1, BaseEP, Axis)
							)
							AddPair(UserObject, id1->mObject, UserGUID, id1->mGUID);
					}

					id1->mMin[Axis + IsMax] = EPIndex--;
					*(CurrentMin+1) = *CurrentMin;
				}

				if(SavedIndex!=EPIndex)
				{
					mBoxes[Saved.GetOwner()].mMin[Axis + Saved.IsMax()] = EPIndex;
					BaseEP[EPIndex] = Saved;
				}
			}
			else if(Limit > CurrentMin->mValue)
			{
				CurrentMin->mValue = Limit;

				// Min is moving right:
				ASAP_EndPoint Saved = *CurrentMin;
				udword EPIndex = (size_t(CurrentMin) - size_t(BaseEP))/sizeof(ASAP_EndPoint);
				const udword SavedIndex = EPIndex;

				while((++CurrentMin)->mValue < Limit)
				{
#ifdef USE_PREFETCH
					_prefetch(CurrentMin+1);
#endif
					ASAP_Box* id1 = mBoxes + CurrentMin->GetOwner();
					const BOOL IsMax = CurrentMin->IsMax();
					if(IsMax)
					{
						// Our min passed a max => stop overlap
						if(Object!=id1
#ifdef USE_OVERLAP_TEST_ON_REMOVES
							&& Intersect2D(*Object, *id1, Axis1, Axis2)
#endif
							)
							RemovePair(UserObject, id1->mObject, UserGUID, id1->mGUID);
					}

					id1->mMin[Axis + IsMax] = EPIndex++;
					*(CurrentMin-1) = *CurrentMin;
				}

				if(SavedIndex!=EPIndex)
				{
					mBoxes[Saved.GetOwner()].mMin[Axis + Saved.IsMax()] = EPIndex;
					BaseEP[EPIndex] = Saved;
				}
			}
		}

		// Update max
		{
			ASAP_EndPoint* CurrentMax = BaseEP + Object->mMax[Axis];
			ASSERT(CurrentMax->IsMax());

			const ValType Limit = box.GetMax(Axis);
			if(Limit > CurrentMax->mValue)
			{
				CurrentMax->mValue = Limit;

				// Max is moving right:
				ASAP_EndPoint Saved = *CurrentMax;
				udword EPIndex = (size_t(CurrentMax) - size_t(BaseEP))/sizeof(ASAP_EndPoint);
				const udword SavedIndex = EPIndex;

				while((++CurrentMax)->mValue < Limit)
				{
#ifdef USE_PREFETCH
					_prefetch(CurrentMax+1);
#endif
					ASAP_Box* id1 = mBoxes + CurrentMax->GetOwner();
					const BOOL IsMax = CurrentMax->IsMax();
					if(!IsMax)
					{
						// Our max passed a min => start overlap
						if(Object!=id1
							&& Intersect2D(*Object, *id1, Axis1, Axis2)
							&& Intersect1D_Min(box, *id1, BaseEP, Axis)
							)
							AddPair(UserObject, id1->mObject, UserGUID, id1->mGUID);
					}

					id1->mMin[Axis + IsMax] = EPIndex++;
					*(CurrentMax-1) = *CurrentMax;
				}

				if(SavedIndex!=EPIndex)
				{
					mBoxes[Saved.GetOwner()].mMin[Axis + Saved.IsMax()] = EPIndex;
					BaseEP[EPIndex] = Saved;
				}
			}
			else if(Limit < CurrentMax->mValue)
			{
				CurrentMax->mValue = Limit;

				// Max is moving left:
				ASAP_EndPoint Saved = *CurrentMax;
				udword EPIndex = (size_t(CurrentMax) - size_t(BaseEP))/sizeof(ASAP_EndPoint);
				const udword SavedIndex = EPIndex;

				while((--CurrentMax)->mValue > Limit)
				{
#ifdef USE_PREFETCH
					_prefetch(CurrentMax-1);
#endif
					ASAP_Box* id1 = mBoxes + CurrentMax->GetOwner();
					const BOOL IsMax = CurrentMax->IsMax();
					if(!IsMax)
					{
						// Our max passed a min => stop overlap
						if(Object!=id1
#ifdef USE_OVERLAP_TEST_ON_REMOVES
							&& Intersect2D(*Object, *id1, Axis1, Axis2)
#endif
							)
							RemovePair(UserObject, id1->mObject, UserGUID, id1->mGUID);
					}

					id1->mMin[Axis + IsMax] = EPIndex--;
					*(CurrentMax+1) = *CurrentMax;
				}

				if(SavedIndex!=EPIndex)
				{
					mBoxes[Saved.GetOwner()].mMin[Axis + Saved.IsMax()] = EPIndex;
					BaseEP[EPIndex] = Saved;
				}
			}
		}
	}
	return true;
}

udword ArraySAP::DumpPairs(SAP_CreatePair create_cb, SAP_DeletePair delete_cb, void* cb_user_data, ASAP_Pair** pairs)
{
	BatchCreate();

	const udword* Entries = mData.GetEntries();
	const udword* Last = Entries + mData.GetNbEntries();
	mData.Reset();

	udword* ToRemove = (udword*)Entries;
	while(Entries!=Last)
	{
		const udword ID = *Entries++;
		ASAP_Pair* UP = mPairs.mActivePairs + ID;

		{
			ASSERT(UP->IsInArray());
			if(UP->IsRemoved())
			{
				// No need to call "ClearInArray" in this case, since the pair will get removed anyway

				// Remove
				if(delete_cb && !UP->IsNew())
				{
#ifdef PAIR_USER_DATA
					(delete_cb)(UP->GetObject0(), UP->GetObject1(), cb_user_data, UP->userData);
#else
					(delete_cb)(UP->GetObject0(), UP->GetObject1(), cb_user_data, null);
#endif
				}

				*ToRemove++ = udword(UP->id0)<<16|UP->id1;
			}
			else
			{
				UP->ClearInArray();
				// Add => already there... Might want to create user data, though
				if(UP->IsNew())
				{
					if(create_cb)
					{
#ifdef PAIR_USER_DATA
						UP->userData = (create_cb)(UP->GetObject0(), UP->GetObject1(), cb_user_data);
#else
						(create_cb)(UP->GetObject0(), UP->GetObject1(), cb_user_data);
#endif
					}
					UP->ClearNew();
				}
			}
		}
	}

	// #### try batch removal here
	Entries = mData.GetEntries();
	while(Entries!=ToRemove)
	{
		const udword ID = *Entries++;
		const udword id0 = ID>>16;
		const udword id1 = ID&0xffff;
		bool Status = mPairs.RemovePair(id0, id1);
		ASSERT(Status);
	}

#ifdef RELEASE_ON_RESET
	mData.Empty();
#endif

	BatchRemove();

	if(pairs)	*pairs = mPairs.mActivePairs;

	return mPairs.mNbActivePairs;
}
