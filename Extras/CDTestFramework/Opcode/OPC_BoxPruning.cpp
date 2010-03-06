/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for box pruning.
 *	\file		IceBoxPruning.cpp
 *	\author		Pierre Terdiman
 *	\date		January, 29, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	You could use a complex sweep-and-prune as implemented in I-Collide.
	You could use a complex hashing scheme as implemented in V-Clip or recently in ODE it seems.
	You could use a "Recursive Dimensional Clustering" algorithm as implemented in GPG2.

	Or you could use this.
	Faster ? I don't know. Probably not. It would be a shame. But who knows ?
	Easier ? Definitely. Enjoy the sheer simplicity.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

	inline_ void FindRunningIndex(udword& index, float* array, udword* sorted, int last, float max)
	{
		int First=index;
		while(First<=last)
		{
			index = (First+last)>>1;

			if(max>array[sorted[index]])	First	= index+1;
			else							last	= index-1;
		}
	}
// ### could be log(n) !
// and maybe use cmp integers

// InsertionSort has better coherence, RadixSort is better for one-shot queries.
#define PRUNING_SORTER	RadixSort
//#define PRUNING_SORTER	InsertionSort

// Static for coherence
static PRUNING_SORTER* gCompletePruningSorter = null;
static PRUNING_SORTER* gBipartitePruningSorter0 = null;
static PRUNING_SORTER* gBipartitePruningSorter1 = null;
inline_ PRUNING_SORTER* GetCompletePruningSorter()
{
	if(!gCompletePruningSorter)	gCompletePruningSorter = ICE_NEW(PRUNING_SORTER);
	return gCompletePruningSorter;
}
inline_ PRUNING_SORTER* GetBipartitePruningSorter0()
{
	if(!gBipartitePruningSorter0)	gBipartitePruningSorter0 = ICE_NEW(PRUNING_SORTER);
	return gBipartitePruningSorter0;
}
inline_ PRUNING_SORTER* GetBipartitePruningSorter1()
{
	if(!gBipartitePruningSorter1)	gBipartitePruningSorter1 = ICE_NEW(PRUNING_SORTER);
	return gBipartitePruningSorter1;
}
void ReleasePruningSorters()
{
	DELETESINGLE(gBipartitePruningSorter1);
	DELETESINGLE(gBipartitePruningSorter0);
	DELETESINGLE(gCompletePruningSorter);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
 *	\param		nb0		[in] number of boxes in the first set
 *	\param		array0	[in] array of boxes for the first set
 *	\param		nb1		[in] number of boxes in the second set
 *	\param		array1	[in] array of boxes for the second set
 *	\param		pairs	[out] array of overlapping pairs
 *	\param		axes	[in] projection order (0,2,1 is often best)
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Opcode::BipartiteBoxPruning(udword nb0, const AABB** array0, udword nb1, const AABB** array1, Pairs& pairs, const Axes& axes)
{
	// Checkings
	if(!nb0 || !array0 || !nb1 || !array1)	return false;

	// Catch axes
	udword Axis0 = axes.mAxis0;
	udword Axis1 = axes.mAxis1;
	udword Axis2 = axes.mAxis2;

	// Allocate some temporary data
	float* MinPosList0 = new float[nb0];
	float* MinPosList1 = new float[nb1];

	// 1) Build main lists using the primary axis
	for(udword i=0;i<nb0;i++)	MinPosList0[i] = array0[i]->GetMin(Axis0);
	for(udword i=0;i<nb1;i++)	MinPosList1[i] = array1[i]->GetMin(Axis0);

	// 2) Sort the lists
	PRUNING_SORTER* RS0 = GetBipartitePruningSorter0();
	PRUNING_SORTER* RS1 = GetBipartitePruningSorter1();
	const udword* Sorted0 = RS0->Sort(MinPosList0, nb0).GetRanks();
	const udword* Sorted1 = RS1->Sort(MinPosList1, nb1).GetRanks();

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

		while(RunningAddress2_1<LastSorted1 && MinPosList1[Index1 = *RunningAddress2_1++]<=array0[Index0]->GetMax(Axis0))
		{
			if(array0[Index0]->Intersect(*array1[Index1], Axis1))
			{
				if(array0[Index0]->Intersect(*array1[Index1], Axis2))
				{
					pairs.AddPair(Index0, Index1);
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

		while(RunningAddress2_0<LastSorted0 && MinPosList0[Index1 = *RunningAddress2_0++]<=array1[Index0]->GetMax(Axis0))
		{
			if(array0[Index1]->Intersect(*array1[Index0], Axis1))
			{
				if(array0[Index1]->Intersect(*array1[Index0], Axis2))
				{
					pairs.AddPair(Index1, Index0);
				}
			}

		}
	}

	DELETEARRAY(MinPosList1);
	DELETEARRAY(MinPosList0);

	return true;
}

#define ORIGINAL_VERSION
//#define JOAKIM

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
 *	\param		nb		[in] number of boxes
 *	\param		array	[in] array of boxes
 *	\param		pairs	[out] array of overlapping pairs
 *	\param		axes	[in] projection order (0,2,1 is often best)
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Opcode::CompleteBoxPruning(udword nb, const AABB** array, Pairs& pairs, const Axes& axes)
{
	// Checkings
	if(!nb || !array)	return false;

	// Catch axes
	udword Axis0 = axes.mAxis0;
	udword Axis1 = axes.mAxis1;
	udword Axis2 = axes.mAxis2;

#ifdef ORIGINAL_VERSION
	// Allocate some temporary data
//	float* PosList = new float[nb];
	float* PosList = new float[nb+1];

	// 1) Build main list using the primary axis
	for(udword i=0;i<nb;i++)	PosList[i] = array[i]->GetMin(Axis0);
PosList[nb++] = MAX_FLOAT;

	// 2) Sort the list
	PRUNING_SORTER* RS = GetCompletePruningSorter();
	const udword* Sorted = RS->Sort(PosList, nb).GetRanks();

	// 3) Prune the list
	const udword* const LastSorted = &Sorted[nb];
	const udword* RunningAddress = Sorted;
	udword Index0, Index1;
	while(RunningAddress<LastSorted && Sorted<LastSorted)
	{
		Index0 = *Sorted++;

//		while(RunningAddress<LastSorted && PosList[*RunningAddress++]<PosList[Index0]);
		while(PosList[*RunningAddress++]<PosList[Index0]);

		if(RunningAddress<LastSorted)
		{
			const udword* RunningAddress2 = RunningAddress;

//			while(RunningAddress2<LastSorted && PosList[Index1 = *RunningAddress2++]<=array[Index0]->GetMax(Axis0))
			while(PosList[Index1 = *RunningAddress2++]<=array[Index0]->GetMax(Axis0))
			{
//				if(Index0!=Index1)
//				{
					if(array[Index0]->Intersect(*array[Index1], Axis1))
					{
						if(array[Index0]->Intersect(*array[Index1], Axis2))
						{
							pairs.AddPair(Index0, Index1);
						}
					}
//				}
			}
		}
	}

	DELETEARRAY(PosList);
#endif

#ifdef JOAKIM
	// Allocate some temporary data
//	float* PosList = new float[nb];
	float* MinList = new float[nb+1];

	// 1) Build main list using the primary axis
	for(udword i=0;i<nb;i++)	MinList[i] = array[i]->GetMin(Axis0);
	MinList[nb] = MAX_FLOAT;

	// 2) Sort the list
	PRUNING_SORTER* RS = GetCompletePruningSorter();
	udword* Sorted = RS->Sort(MinList, nb+1).GetRanks();

	// 3) Prune the list
//	const udword* const LastSorted = &Sorted[nb];
//	const udword* const LastSorted = &Sorted[nb-1];
	const udword* RunningAddress = Sorted;
	udword Index0, Index1;

//	while(RunningAddress<LastSorted && Sorted<LastSorted)
//	while(RunningAddress<LastSorted)
	while(RunningAddress<&Sorted[nb])
//	while(Sorted<LastSorted)
	{
//		Index0 = *Sorted++;
		Index0 = *RunningAddress++;

//		while(RunningAddress<LastSorted && PosList[*RunningAddress++]<PosList[Index0]);
//		while(PosList[*RunningAddress++]<PosList[Index0]);
//RunningAddress = Sorted;
//		if(RunningAddress<LastSorted)
		{
			const udword* RunningAddress2 = RunningAddress;

//			while(RunningAddress2<LastSorted && PosList[Index1 = *RunningAddress2++]<=array[Index0]->GetMax(Axis0))

//			float CurrentMin = array[Index0]->GetMin(Axis0);
			float CurrentMax = array[Index0]->GetMax(Axis0);

			while(MinList[Index1 = *RunningAddress2] <= CurrentMax)
//			while(PosList[Index1 = *RunningAddress] <= CurrentMax)
			{
//				if(Index0!=Index1)
//				{
					if(array[Index0]->Intersect(*array[Index1], Axis1))
					{
						if(array[Index0]->Intersect(*array[Index1], Axis2))
						{
							pairs.AddPair(Index0, Index1);
						}
					}
//				}

				RunningAddress2++;
//				RunningAddress++;
			}
		}
	}

	DELETEARRAY(MinList);
#endif

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Brute-force versions are kept:
// - to check the optimized versions return the correct list of intersections
// - to check the speed of the optimized code against the brute-force one
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Brute-force bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
 *	\param		nb0		[in] number of boxes in the first set
 *	\param		array0	[in] array of boxes for the first set
 *	\param		nb1		[in] number of boxes in the second set
 *	\param		array1	[in] array of boxes for the second set
 *	\param		pairs	[out] array of overlapping pairs
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Opcode::BruteForceBipartiteBoxTest(udword nb0, const AABB** array0, udword nb1, const AABB** array1, Pairs& pairs)
{
	// Checkings
	if(!nb0 || !array0 || !nb1 || !array1)	return false;

	// Brute-force nb0*nb1 overlap tests
	for(udword i=0;i<nb0;i++)
	{
		for(udword j=0;j<nb1;j++)
		{
			if(array0[i]->Intersect(*array1[j]))	pairs.AddPair(i, j);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
 *	\param		nb		[in] number of boxes
 *	\param		array	[in] array of boxes
 *	\param		pairs	[out] array of overlapping pairs
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Opcode::BruteForceCompleteBoxTest(udword nb, const AABB** array, Pairs& pairs)
{
	// Checkings
	if(!nb || !array)	return false;

	// Brute-force n(n-1)/2 overlap tests
	for(udword i=0;i<nb;i++)
	{
		for(udword j=i+1;j<nb;j++)
		{
			if(array[i]->Intersect(*array[j]))	pairs.AddPair(i, j);
		}
	}
	return true;
}
