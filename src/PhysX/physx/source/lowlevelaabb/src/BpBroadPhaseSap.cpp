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

#include "foundation/PxMath.h"
#include "common/PxProfileZone.h"
#include "CmPhysXCommon.h"
#include "CmTmpMem.h"
#include "PxcScratchAllocator.h"
#include "PxSceneDesc.h"
#include "BpBroadPhaseSap.h"
#include "BpBroadPhaseSapAux.h"
#include "CmRadixSortBuffered.h"
#include "PsFoundation.h"
#include "PsAllocator.h"
//#include <stdio.h>

namespace physx
{
namespace Bp
{
#define DEFAULT_DATA_ARRAY_CAPACITY 1024
#define DEFAULT_CREATEDDELETED_PAIR_ARRAY_CAPACITY 64
#define DEFAULT_CREATEDDELETED1AXIS_CAPACITY 8192

BroadPhaseSap::BroadPhaseSap(
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID) :
	mScratchAllocator		(NULL),
	mSapUpdateWorkTask		(contextID),
	mSapPostUpdateWorkTask	(contextID),
	mContextID				(contextID)
{

	for(PxU32 i=0;i<3;i++)
		mBatchUpdateTasks[i].setContextId(contextID);

	//Boxes
	mBoxesSize=0;
	mBoxesSizePrev=0;
	mBoxesCapacity = (((maxNbStaticShapes + maxNbDynamicShapes) + 31) & ~31);
	mBoxEndPts[0] = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*mBoxesCapacity)), "SapBox1D"));
	mBoxEndPts[1] = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*mBoxesCapacity)), "SapBox1D"));
	mBoxEndPts[2] = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*mBoxesCapacity)), "SapBox1D"));
	for(PxU32 i=0; i<mBoxesCapacity;i++)
	{
		mBoxEndPts[0][i].mMinMax[0]=BP_INVALID_BP_HANDLE;
		mBoxEndPts[0][i].mMinMax[1]=BP_INVALID_BP_HANDLE;
		mBoxEndPts[1][i].mMinMax[0]=BP_INVALID_BP_HANDLE;
		mBoxEndPts[1][i].mMinMax[1]=BP_INVALID_BP_HANDLE;
		mBoxEndPts[2][i].mMinMax[0]=BP_INVALID_BP_HANDLE;
		mBoxEndPts[2][i].mMinMax[1]=BP_INVALID_BP_HANDLE;
	}
		
	//End points
	mEndPointsCapacity = mBoxesCapacity*2 + NUM_SENTINELS;

	mBoxesUpdated = reinterpret_cast<PxU8*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(PxU8)*mBoxesCapacity)), "BoxesUpdated"));
	mSortedUpdateElements = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*mEndPointsCapacity)), "SortedUpdateElements"));
	mActivityPockets = reinterpret_cast<BroadPhaseActivityPocket*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BroadPhaseActivityPocket)*mEndPointsCapacity)), "BroadPhaseActivityPocket"));

	mEndPointValues[0] = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(mEndPointsCapacity))), "ValType"));
	mEndPointValues[1] = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(mEndPointsCapacity))), "ValType"));
	mEndPointValues[2] = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(mEndPointsCapacity))), "ValType"));
	mEndPointDatas[0] = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(mEndPointsCapacity))), "BpHandle"));
	mEndPointDatas[1] = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(mEndPointsCapacity))), "BpHandle"));
	mEndPointDatas[2]=  reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(mEndPointsCapacity))), "BpHandle"));

	// Initialize sentinels
	setMinSentinel(mEndPointValues[0][0],mEndPointDatas[0][0]);
	setMaxSentinel(mEndPointValues[0][1],mEndPointDatas[0][1]);
	setMinSentinel(mEndPointValues[1][0],mEndPointDatas[1][0]);
	setMaxSentinel(mEndPointValues[1][1],mEndPointDatas[1][1]);
	setMinSentinel(mEndPointValues[2][0],mEndPointDatas[2][0]);
	setMaxSentinel(mEndPointValues[2][1],mEndPointDatas[2][1]);

	mListNext = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*mEndPointsCapacity)), "NextList"));
	mListPrev = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*mEndPointsCapacity)), "PrevList"));

	for(PxU32 a = 1; a < mEndPointsCapacity; ++a)
	{
		mListNext[a-1] = BpHandle(a);
		mListPrev[a] = BpHandle(a-1);
	}
	mListNext[mEndPointsCapacity-1] = BpHandle(mEndPointsCapacity-1);
	mListPrev[0] = 0;

	mDefaultPairsCapacity = PxMax(maxNbBroadPhaseOverlaps, PxU32(DEFAULT_CREATEDDELETED_PAIR_ARRAY_CAPACITY));

	mPairs.init(mDefaultPairsCapacity);

	mBatchUpdateTasks[2].set(this,2);
	mBatchUpdateTasks[1].set(this,1);
	mBatchUpdateTasks[0].set(this,0);
	mBatchUpdateTasks[2].setPairs(NULL, 0);
	mBatchUpdateTasks[1].setPairs(NULL, 0);
	mBatchUpdateTasks[0].setPairs(NULL, 0);

	//Initialise data array.
	mData = NULL;
	mDataSize = 0;
	mDataCapacity = 0;

	//Initialise pairs arrays.
	mCreatedPairsArray = NULL;
	mCreatedPairsCapacity = 0;
	mCreatedPairsSize = 0;
	mDeletedPairsArray = NULL;
	mDeletedPairsCapacity = 0;
	mDeletedPairsSize = 0;
	mActualDeletedPairSize = 0;

#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	mLUT = NULL;
#endif
}

BroadPhaseSap::~BroadPhaseSap()
{
	PX_FREE(mBoxEndPts[0]);
	PX_FREE(mBoxEndPts[1]);
	PX_FREE(mBoxEndPts[2]);

	PX_FREE(mEndPointValues[0]);
	PX_FREE(mEndPointValues[1]);
	PX_FREE(mEndPointValues[2]);
	PX_FREE(mEndPointDatas[0]);
	PX_FREE(mEndPointDatas[1]);
	PX_FREE(mEndPointDatas[2]);

	PX_FREE(mListNext);
	PX_FREE(mListPrev);

	PX_FREE(mSortedUpdateElements);
	PX_FREE(mActivityPockets);
	PX_FREE(mBoxesUpdated);

	mPairs.release();

	mBatchUpdateTasks[0].setPairs(NULL, 0);
	mBatchUpdateTasks[1].setPairs(NULL, 0);
	mBatchUpdateTasks[2].setPairs(NULL, 0);

	mData = NULL;

	mCreatedPairsArray = NULL;
	mDeletedPairsArray = NULL;
	
}

void BroadPhaseSap::destroy()
{
	this->~BroadPhaseSap();
	PX_FREE(this);
}

void BroadPhaseSap::resizeBuffers()
{
	const PxU32 defaultPairsCapacity = mDefaultPairsCapacity;

	mCreatedPairsArray = reinterpret_cast<BroadPhasePair*>(mScratchAllocator->alloc(sizeof(BroadPhasePair)*defaultPairsCapacity, true));
	mCreatedPairsCapacity = defaultPairsCapacity;
	mCreatedPairsSize = 0;

	mDeletedPairsArray = reinterpret_cast<BroadPhasePair*>(mScratchAllocator->alloc(sizeof(BroadPhasePair)*defaultPairsCapacity, true));
	mDeletedPairsCapacity = defaultPairsCapacity;
	mDeletedPairsSize = 0;

	mData = reinterpret_cast<BpHandle*>(mScratchAllocator->alloc(sizeof(BpHandle)*defaultPairsCapacity, true));
	mDataCapacity = defaultPairsCapacity;
	mDataSize = 0;

	mBatchUpdateTasks[0].setPairs(reinterpret_cast<BroadPhasePair*>(mScratchAllocator->alloc(sizeof(BroadPhasePair)*defaultPairsCapacity, true)), defaultPairsCapacity);
	mBatchUpdateTasks[0].setNumPairs(0);
	mBatchUpdateTasks[1].setPairs(reinterpret_cast<BroadPhasePair*>(mScratchAllocator->alloc(sizeof(BroadPhasePair)*defaultPairsCapacity, true)), defaultPairsCapacity);
	mBatchUpdateTasks[1].setNumPairs(0);
	mBatchUpdateTasks[2].setPairs(reinterpret_cast<BroadPhasePair*>(mScratchAllocator->alloc(sizeof(BroadPhasePair)*defaultPairsCapacity, true)), defaultPairsCapacity);
	mBatchUpdateTasks[2].setNumPairs(0);
}

void BroadPhaseSap::freeBuffers()
{
	if(mCreatedPairsArray) mScratchAllocator->free(mCreatedPairsArray);
	mCreatedPairsArray = NULL;
	mCreatedPairsSize = 0;
	mCreatedPairsCapacity = 0;

	if(mDeletedPairsArray) mScratchAllocator->free(mDeletedPairsArray);
	mDeletedPairsArray = NULL;
	mDeletedPairsSize = 0;
	mDeletedPairsCapacity = 0;
	mActualDeletedPairSize = 0;

	if(mData) mScratchAllocator->free(mData);
	mData = NULL;
	mDataSize = 0;
	mDataCapacity = 0;

	if(mBatchUpdateTasks[0].getPairs()) mScratchAllocator->free(mBatchUpdateTasks[0].getPairs());
	mBatchUpdateTasks[0].setPairs(NULL, 0);
	mBatchUpdateTasks[0].setNumPairs(0);
	if(mBatchUpdateTasks[1].getPairs()) mScratchAllocator->free(mBatchUpdateTasks[1].getPairs());
	mBatchUpdateTasks[1].setPairs(NULL, 0);
	mBatchUpdateTasks[1].setNumPairs(0);
	if(mBatchUpdateTasks[2].getPairs()) mScratchAllocator->free(mBatchUpdateTasks[2].getPairs());
	mBatchUpdateTasks[2].setPairs(NULL, 0);
	mBatchUpdateTasks[2].setNumPairs(0);

	//Shrink pair manager buffers it they are larger than needed but only let them shrink to a minimum size.
	mPairs.shrinkMemory();
}

PX_FORCE_INLINE static void shiftCoord3(const ValType val0, const BpHandle handle0,
										const ValType val1, const BpHandle handle1,
										const ValType val2, const BpHandle handle2,
										const PxF32* shift, ValType& oVal0, ValType& oVal1, ValType& oVal2)
{
	PX_ASSERT(!isSentinel(handle0));
	PX_ASSERT(!isSentinel(handle1));
	PX_ASSERT(!isSentinel(handle2));

	PxF32 fl0, fl1, fl2;
	ValType* PX_RESTRICT bpVal0 = PxUnionCast<ValType*, PxF32*>(&fl0);
	ValType* PX_RESTRICT bpVal1 = PxUnionCast<ValType*, PxF32*>(&fl1);
	ValType* PX_RESTRICT bpVal2 = PxUnionCast<ValType*, PxF32*>(&fl2);
	*bpVal0 = decodeFloat(val0);
	*bpVal1 = decodeFloat(val1);
	*bpVal2 = decodeFloat(val2);
	fl0 -= shift[0];
	fl1 -= shift[1];
	fl2 -= shift[2];
	oVal0 = (isMax(handle0)) ? (IntegerAABB::encodeFloatMax(*bpVal0) | 1) : ((IntegerAABB::encodeFloatMin(*bpVal0) + 1) & ~1);
	oVal1 = (isMax(handle1)) ? (IntegerAABB::encodeFloatMax(*bpVal1) | 1) : ((IntegerAABB::encodeFloatMin(*bpVal1) + 1) & ~1);
	oVal2 = (isMax(handle2)) ? (IntegerAABB::encodeFloatMax(*bpVal2) | 1) : ((IntegerAABB::encodeFloatMin(*bpVal2) + 1) & ~1);
}

PX_FORCE_INLINE static void testPostShiftOrder(const ValType prevVal, ValType& currVal, const BpHandle prevIsMax, const BpHandle currIsMax)
{
	if(currVal < prevVal)
	{
		//The order has been broken by the lossy shift.
		//Correct currVal so that it is greater than prevVal.
		//If currVal is a box max then ensure that the box is of finite extent.
		const ValType shiftCorrection = (prevIsMax==currIsMax) ? ValType(0) : ValType(1);
		currVal = prevVal + shiftCorrection;
	}
}

void BroadPhaseSap::shiftOrigin(const PxVec3& shift)
{
	//
	// Note: shifting the bounds does not necessarily preserve the order of the broadphase interval endpoints. The encoding of the float bounds is a lossy
	//       operation, thus it is not possible to get the original float values back and shift them. The only goal of this method is to shift the endpoints
	//       such that the order is preserved. The new intervals might no reflect the correct bounds! Since all bounds have been marked dirty, they will get
	//       recomputed in the next frame anyway. This method makes sure that the next frame update can start from a valid configuration that is close to
	//       the correct one and does not require too many swaps.
	//

	if(0==mBoxesSize)
	{
		return;
	}

	//
	// Note: processing all the axis at once improved performance on XBox 360 and PS3 because it allows to compensate for stalls
	//

	const PxF32 shiftAxis[3] = { shift.x, shift.y, shift.z };
	const BpHandle* PX_RESTRICT epData0 = mEndPointDatas[0];
	ValType* PX_RESTRICT epValues0 = mEndPointValues[0];
	const BpHandle* PX_RESTRICT epData1 = mEndPointDatas[1];
	ValType* PX_RESTRICT epValues1 = mEndPointValues[1];
	const BpHandle* PX_RESTRICT epData2 = mEndPointDatas[2];
	ValType* PX_RESTRICT epValues2 = mEndPointValues[2];

	//Shift the first value in the array of sorted values.
	{
		//Shifted min (first element must be a min by definition).
		shiftCoord3(epValues0[1], epData0[1], epValues1[1], epData1[1], epValues2[1], epData2[1], shiftAxis, epValues0[1], epValues1[1], epValues2[1]);
		PX_ASSERT(!isMax(epData0[1]));
		PX_ASSERT(!isMax(epData1[1]));
		PX_ASSERT(!isMax(epData2[1]));
	}

	//Shift the remainder.
	ValType prevVal0 = epValues0[1];
	BpHandle prevIsMax0 = isMax(epData0[1]);
	ValType prevVal1 = epValues1[1];
	BpHandle prevIsMax1 = isMax(epData1[1]);
	ValType prevVal2 = epValues2[1];
	BpHandle prevIsMax2 = isMax(epData2[1]);
	for(PxU32 i=2; i <= mBoxesSize*2; i++)
	{
		const BpHandle handle0 = epData0[i];
		const BpHandle handle1 = epData1[i];
		const BpHandle handle2 = epData2[i];
		PX_ASSERT(!isSentinel(handle0));
		PX_ASSERT(!isSentinel(handle1));
		PX_ASSERT(!isSentinel(handle2));

		//Get the relevant prev and curr values after the shift.
		const BpHandle currIsMax0 = isMax(epData0[i]);
		const BpHandle currIsMax1 = isMax(epData1[i]);
		const BpHandle currIsMax2 = isMax(epData2[i]);
		ValType currVal0, currVal1, currVal2;
		shiftCoord3(epValues0[i], handle0, epValues1[i], handle1, epValues2[i], handle2, shiftAxis, currVal0, currVal1, currVal2);
			
		//Test if the order has been preserved by the lossy shift.
		testPostShiftOrder(prevVal0, currVal0, prevIsMax0, currIsMax0);
		testPostShiftOrder(prevVal1, currVal1, prevIsMax1, currIsMax1);
		testPostShiftOrder(prevVal2, currVal2, prevIsMax2, currIsMax2);
		
		prevIsMax0 = currIsMax0;
		prevVal0 = currVal0;
		prevIsMax1 = currIsMax1;
		prevVal1 = currVal1;
		prevIsMax2 = currIsMax2;
		prevVal2 = currVal2;

		epValues0[i] = currVal0;
		epValues1[i] = currVal1;
		epValues2[i] = currVal2;
	}

	PX_ASSERT(isSelfOrdered());
}

#if PX_CHECKED
bool BroadPhaseSap::isValid(const BroadPhaseUpdateData& updateData) const
{
	//Test that the created bounds haven't been added already (without first being removed).
	const BpHandle* created=updateData.getCreatedHandles();
	const PxU32 numCreated=updateData.getNumCreatedHandles();
	for(PxU32 i=0;i<numCreated;i++)
	{
		const BpHandle id=created[i];

		//If id >=mBoxesCapacity then we need to resize to add this id, meaning that the id must be new.
		if(id<mBoxesCapacity)
		{
			for(PxU32 j=0;j<3;j++)
			{
				const SapBox1D& box1d=mBoxEndPts[j][id];
				if(box1d.mMinMax[0] != BP_INVALID_BP_HANDLE && box1d.mMinMax[0] != PX_REMOVED_BP_HANDLE)
				{
					//This box has been added already but without being removed.
					return false;
				}
				if(box1d.mMinMax[1] != BP_INVALID_BP_HANDLE && box1d.mMinMax[1] != PX_REMOVED_BP_HANDLE)
				{
					//This box has been added already but without being removed.
					return false;
				}
			}
		}
	}

	//Test that the updated bounds have valid ids.
	const BpHandle* updated=updateData.getUpdatedHandles();
	const PxU32 numUpdated=updateData.getNumUpdatedHandles();
	for(PxU32 i=0;i<numUpdated;i++)
	{
		const BpHandle id = updated[i];
		if(id >= mBoxesCapacity)
		{
			return false;
		}
	}

	//Test that the updated bounds have been been added without being removed.
	for(PxU32 i=0;i<numUpdated;i++)
	{
		const BpHandle id = updated[i];

		for(PxU32 j=0;j<3;j++)
		{
			const SapBox1D& box1d=mBoxEndPts[j][id];

			if(BP_INVALID_BP_HANDLE == box1d.mMinMax[0] || PX_REMOVED_BP_HANDLE == box1d.mMinMax[0])
			{
				//This box has either not been added or has been removed
				return false;
			}
			if(BP_INVALID_BP_HANDLE == box1d.mMinMax[1] || PX_REMOVED_BP_HANDLE == box1d.mMinMax[1])
			{
				//This box has either not been added or has been removed
				return false;
			}
		}
	}

	//Test that the removed bounds have valid ids.
	const BpHandle* removed=updateData.getRemovedHandles();
	const PxU32 numRemoved=updateData.getNumRemovedHandles();
	for(PxU32 i=0;i<numRemoved;i++)
	{
		const BpHandle id = removed[i];
		if(id >= mBoxesCapacity)
		{
			return false;
		}
	}

	//Test that the removed bounds have already been added and haven't been removed.
	for(PxU32 i=0;i<numRemoved;i++)
	{
		const BpHandle id = removed[i];

		for(PxU32 j=0;j<3;j++)
		{
			const SapBox1D& box1d=mBoxEndPts[j][id];

			if(BP_INVALID_BP_HANDLE == box1d.mMinMax[0] || PX_REMOVED_BP_HANDLE == box1d.mMinMax[0])
			{
				//This box has either not been added or has been removed
				return false;
			}
			if(BP_INVALID_BP_HANDLE == box1d.mMinMax[1] || PX_REMOVED_BP_HANDLE == box1d.mMinMax[1])
			{
				//This box has either not been added or has been removed
				return false;
			}
		}
	}
	return true;
}
#endif

void BroadPhaseSap::update(const PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, PxBaseTask* continuation, PxBaseTask* narrowPhaseUnblockTask)
{
#if PX_CHECKED
	PX_CHECK_AND_RETURN(scratchAllocator, "BroadPhaseSap::update - scratchAllocator must be non-NULL \n");
#endif

	if(narrowPhaseUnblockTask)
		narrowPhaseUnblockTask->removeReference();

	if(setUpdateData(updateData))
	{
		mScratchAllocator = scratchAllocator;

		resizeBuffers();

		mSapPostUpdateWorkTask.setBroadPhase(this);
		mSapUpdateWorkTask.setBroadPhase(this);

		mSapPostUpdateWorkTask.set(numCpuTasks);
		mSapUpdateWorkTask.set(numCpuTasks);

		mSapPostUpdateWorkTask.setContinuation(continuation);
		mSapUpdateWorkTask.setContinuation(&mSapPostUpdateWorkTask);

		mSapPostUpdateWorkTask.removeReference();
		mSapUpdateWorkTask.removeReference();
	}
}

void BroadPhaseSap::singleThreadedUpdate(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData)
{
#if PX_CHECKED
	PX_CHECK_AND_RETURN(scratchAllocator, "BroadPhaseSap::singleThreadedUpdate - scratchAllocator must be non-NULL \n");
#endif

	if(setUpdateData(updateData))
	{
		mScratchAllocator = scratchAllocator;
		resizeBuffers();
		update();
		postUpdate();
	}
}

bool BroadPhaseSap::setUpdateData(const BroadPhaseUpdateData& updateData) 
{
	PX_ASSERT(0==mCreatedPairsSize);
	PX_ASSERT(0==mDeletedPairsSize);

#if PX_CHECKED
	if(!BroadPhaseUpdateData::isValid(updateData, *this))
	{
		PX_CHECK_MSG(false, "Illegal BroadPhaseUpdateData \n");
		mCreated			= NULL;
		mCreatedSize		= 0;
		mUpdated			= NULL;
		mUpdatedSize		= 0;
		mRemoved			= NULL;
		mRemovedSize		= 0;
		mBoxBoundsMinMax	= updateData.getAABBs();
		mBoxGroups			= updateData.getGroups();
		return false;
	}
#endif

	//Copy across the data ptrs and sizes.
	mCreated			= updateData.getCreatedHandles();
	mCreatedSize		= updateData.getNumCreatedHandles();
	mUpdated			= updateData.getUpdatedHandles();
	mUpdatedSize		= updateData.getNumUpdatedHandles();
	mRemoved			= updateData.getRemovedHandles();
	mRemovedSize		= updateData.getNumRemovedHandles();
	mBoxBoundsMinMax	= updateData.getAABBs();
	mBoxGroups			= updateData.getGroups();
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	mLUT				= updateData.getLUT();
#endif
	mContactDistance	= updateData.getContactDistance();

	//Do we need more memory to store the positions of each box min/max in the arrays of sorted boxes min/max?
	if(updateData.getCapacity() > mBoxesCapacity)
	{
		const PxU32 oldBoxesCapacity=mBoxesCapacity;
		const PxU32 newBoxesCapacity=updateData.getCapacity();
		SapBox1D* newBoxEndPts0 = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*newBoxesCapacity)), "SapBox1D"));
		SapBox1D* newBoxEndPts1 = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*newBoxesCapacity)), "SapBox1D"));
		SapBox1D* newBoxEndPts2 = reinterpret_cast<SapBox1D*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(SapBox1D)*newBoxesCapacity)), "SapBox1D"));

		PxMemCopy(newBoxEndPts0, mBoxEndPts[0], sizeof(SapBox1D)*oldBoxesCapacity);
		PxMemCopy(newBoxEndPts1, mBoxEndPts[1], sizeof(SapBox1D)*oldBoxesCapacity);
		PxMemCopy(newBoxEndPts2, mBoxEndPts[2], sizeof(SapBox1D)*oldBoxesCapacity);
		for(PxU32 i=oldBoxesCapacity;i<newBoxesCapacity;i++)
		{
			newBoxEndPts0[i].mMinMax[0]=BP_INVALID_BP_HANDLE;
			newBoxEndPts0[i].mMinMax[1]=BP_INVALID_BP_HANDLE;
			newBoxEndPts1[i].mMinMax[0]=BP_INVALID_BP_HANDLE;
			newBoxEndPts1[i].mMinMax[1]=BP_INVALID_BP_HANDLE;
			newBoxEndPts2[i].mMinMax[0]=BP_INVALID_BP_HANDLE;
			newBoxEndPts2[i].mMinMax[1]=BP_INVALID_BP_HANDLE;
		}
		PX_FREE(mBoxEndPts[0]);
		PX_FREE(mBoxEndPts[1]);
		PX_FREE(mBoxEndPts[2]);
		mBoxEndPts[0] = newBoxEndPts0;
		mBoxEndPts[1] = newBoxEndPts1;
		mBoxEndPts[2] = newBoxEndPts2;
		mBoxesCapacity = newBoxesCapacity;

		

		PX_FREE(mBoxesUpdated);
		mBoxesUpdated = reinterpret_cast<PxU8*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(PxU8))*newBoxesCapacity), "Updated Boxes"));
	}

	//Do we need more memory for the array of sorted boxes?
	if(2*(mBoxesSize + mCreatedSize) + NUM_SENTINELS > mEndPointsCapacity)
	{
		const PxU32 newEndPointsCapacity = 2*(mBoxesSize + mCreatedSize) + NUM_SENTINELS;

		ValType* newEndPointValuesX = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(newEndPointsCapacity))), "BPValType"));
		ValType* newEndPointValuesY = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(newEndPointsCapacity))), "BPValType"));
		ValType* newEndPointValuesZ = reinterpret_cast<ValType*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(ValType)*(newEndPointsCapacity))), "BPValType"));
		BpHandle* newEndPointDatasX = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(newEndPointsCapacity))), "BpHandle"));
		BpHandle* newEndPointDatasY = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(newEndPointsCapacity))), "BpHandle"));
		BpHandle* newEndPointDatasZ = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*(newEndPointsCapacity))), "BpHandle"));

		PX_FREE(mListNext);
		PX_FREE(mListPrev);

		mListNext = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*newEndPointsCapacity)), "NextList"));
		mListPrev = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*newEndPointsCapacity)), "Prev"));


		for(PxU32 a = 1; a < newEndPointsCapacity; ++a)
		{
			mListNext[a-1] = BpHandle(a);
			mListPrev[a] = BpHandle(a-1);
		}
		mListNext[newEndPointsCapacity-1] = BpHandle(newEndPointsCapacity-1);
		mListPrev[0] = 0;

		PxMemCopy(newEndPointValuesX, mEndPointValues[0], sizeof(ValType)*(mBoxesSize*2+NUM_SENTINELS));
		PxMemCopy(newEndPointValuesY, mEndPointValues[1], sizeof(ValType)*(mBoxesSize*2+NUM_SENTINELS));
		PxMemCopy(newEndPointValuesZ, mEndPointValues[2], sizeof(ValType)*(mBoxesSize*2+NUM_SENTINELS));
		PxMemCopy(newEndPointDatasX, mEndPointDatas[0], sizeof(BpHandle)*(mBoxesSize*2+NUM_SENTINELS));
		PxMemCopy(newEndPointDatasY, mEndPointDatas[1], sizeof(BpHandle)*(mBoxesSize*2+NUM_SENTINELS));
		PxMemCopy(newEndPointDatasZ, mEndPointDatas[2], sizeof(BpHandle)*(mBoxesSize*2+NUM_SENTINELS));
		PX_FREE(mEndPointValues[0]);
		PX_FREE(mEndPointValues[1]);
		PX_FREE(mEndPointValues[2]);
		PX_FREE(mEndPointDatas[0]);
		PX_FREE(mEndPointDatas[1]);
		PX_FREE(mEndPointDatas[2]);
		mEndPointValues[0] = newEndPointValuesX;
		mEndPointValues[1] = newEndPointValuesY;
		mEndPointValues[2] = newEndPointValuesZ;
		mEndPointDatas[0] = newEndPointDatasX;
		mEndPointDatas[1] = newEndPointDatasY;
		mEndPointDatas[2] = newEndPointDatasZ;
		mEndPointsCapacity = newEndPointsCapacity;

		PX_FREE(mSortedUpdateElements);
		PX_FREE(mActivityPockets);
		mSortedUpdateElements = reinterpret_cast<BpHandle*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BpHandle)*newEndPointsCapacity)), "SortedUpdateElements"));
		mActivityPockets = reinterpret_cast<BroadPhaseActivityPocket*>(PX_ALLOC(ALIGN_SIZE_16((sizeof(BroadPhaseActivityPocket)*newEndPointsCapacity)), "BroadPhaseActivityPocket"));
	}

	PxMemZero(mBoxesUpdated, sizeof(PxU8) * (mBoxesCapacity));	

	for(PxU32 a=0;a<mUpdatedSize;a++)
	{
		const PxU32 handle=mUpdated[a];
		mBoxesUpdated[handle] = 1;
	}

	//Update the size of the sorted boxes arrays.
	PX_ASSERT(mBoxesSize==mBoxesSizePrev);
	mBoxesSize += mCreatedSize;
	PX_ASSERT(2*mBoxesSize+NUM_SENTINELS <= mEndPointsCapacity);

	return true;
}

void BroadPhaseSap::postUpdate()
{
	PX_PROFILE_ZONE("BroadPhase.SapPostUpdate", mContextID);

	DataArray da(mData, mDataSize, mDataCapacity);

	for(PxU32 i=0;i<3;i++)
	{
		const PxU32 numPairs=mBatchUpdateTasks[i].getPairsSize();
		const BroadPhasePair* PX_RESTRICT pairs=mBatchUpdateTasks[i].getPairs();
		for(PxU32 j=0;j<numPairs;j++)
		{
			const BroadPhasePair& pair=pairs[j];
			const BpHandle volA=pair.mVolA;
			const BpHandle volB=pair.mVolB;
			if(volA > volB)
				addPair(volA, volB, mScratchAllocator, mPairs, da);
			else
				removePair(volA, volB, mScratchAllocator, mPairs, da);
		}
	}

	mData = da.mData;
	mDataSize = da.mSize;
	mDataCapacity = da.mCapacity;

	batchCreate();

	//Compute the lists of created and deleted overlap pairs.

	ComputeCreatedDeletedPairsLists(
		mBoxGroups,
		mData,mDataSize,
		mScratchAllocator, 
		mCreatedPairsArray,mCreatedPairsSize,mCreatedPairsCapacity,
		mDeletedPairsArray,mDeletedPairsSize,mDeletedPairsCapacity,
		mActualDeletedPairSize,
		mPairs);

	//DeletePairsLists(mActualDeletedPairSize, mDeletedPairsArray, mPairs);

	PX_ASSERT(isSelfConsistent());
	mBoxesSizePrev=mBoxesSize;
}

void BroadPhaseSap::deletePairs()
{
	DeletePairsLists(mActualDeletedPairSize, mDeletedPairsArray, mPairs);
}

void BroadPhaseBatchUpdateWorkTask::runInternal()
{
	mPairsSize=0;
	mSap->batchUpdate(mAxis, mPairs, mPairsSize, mPairsCapacity);
}

void BroadPhaseSap::update()
{
	PX_PROFILE_ZONE("BroadPhase.SapUpdate", mContextID);

	batchRemove();

	//Check that the overlap pairs per axis have been reset.
	PX_ASSERT(0==mBatchUpdateTasks[0].getPairsSize());
	PX_ASSERT(0==mBatchUpdateTasks[1].getPairsSize());
	PX_ASSERT(0==mBatchUpdateTasks[2].getPairsSize());

	mBatchUpdateTasks[0].runInternal();
	mBatchUpdateTasks[1].runInternal();
	mBatchUpdateTasks[2].runInternal();
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE void InsertEndPoints(const ValType* PX_RESTRICT newEndPointValues, const BpHandle* PX_RESTRICT newEndPointDatas, PxU32 numNewEndPoints,
											ValType* PX_RESTRICT endPointValues, BpHandle* PX_RESTRICT endPointDatas, const PxU32 numEndPoints, SapBox1D* PX_RESTRICT boxes)
{
	ValType* const BaseEPValue = endPointValues;
	BpHandle* const BaseEPData = endPointDatas;

	const PxU32 OldSize = numEndPoints-NUM_SENTINELS;
	const PxU32 NewSize = numEndPoints-NUM_SENTINELS+numNewEndPoints;

	BaseEPValue[NewSize + 1] = BaseEPValue[OldSize + 1];
	BaseEPData[NewSize + 1] = BaseEPData[OldSize + 1];

	PxI32 WriteIdx = PxI32(NewSize);
	PxU32 CurrInsIdx = 0;

	//const SapValType* FirstValue = &BaseEPValue[0];
	const BpHandle* FirstData = &BaseEPData[0];
	const ValType* CurrentValue = &BaseEPValue[OldSize];
	const BpHandle* CurrentData = &BaseEPData[OldSize];
	while(CurrentData>=FirstData)
	{
		const ValType& SrcValue = *CurrentValue;
		const BpHandle& SrcData = *CurrentData;
		const ValType& InsValue = newEndPointValues[CurrInsIdx];
		const BpHandle& InsData = newEndPointDatas[CurrInsIdx];

		// We need to make sure we insert maxs before mins to handle exactly equal endpoints correctly
		const bool ShouldInsert = isMax(InsData) ? (SrcValue <= InsValue) : (SrcValue < InsValue);

		const ValType& MovedValue = ShouldInsert ? InsValue : SrcValue;
		const BpHandle& MovedData = ShouldInsert ? InsData : SrcData;
		BaseEPValue[WriteIdx] = MovedValue;
		BaseEPData[WriteIdx] = MovedData;
		boxes[getOwner(MovedData)].mMinMax[isMax(MovedData)] = BpHandle(WriteIdx--);

		if(ShouldInsert)
		{
			CurrInsIdx++;
			if(CurrInsIdx >= numNewEndPoints)
				break;//we just inserted the last endpoint
		}
		else
		{
			CurrentValue--;
			CurrentData--;
		}
	}
}

static PX_FORCE_INLINE bool Intersect3D(const ValType bDir1Min, const ValType bDir1Max, const ValType bDir2Min, const ValType bDir2Max, const ValType bDir3Min, const ValType bDir3Max,
										const ValType cDir1Min, const ValType cDir1Max, const ValType cDir2Min, const ValType cDir2Max, const ValType cDir3Min, const ValType cDir3Max)
{
	return (bDir1Max >= cDir1Min && cDir1Max >= bDir1Min && 
			bDir2Max >= cDir2Min && cDir2Max >= bDir2Min &&
			bDir3Max >= cDir3Min && cDir3Max >= bDir3Min);       
}

void BroadPhaseSap::ComputeSortedLists(	//const PxVec4& globalMin, const PxVec4& /*globalMax*/,
										BpHandle* PX_RESTRICT newBoxIndicesSorted, PxU32& newBoxIndicesCount, BpHandle* PX_RESTRICT oldBoxIndicesSorted, PxU32& oldBoxIndicesCount,
										bool& allNewBoxesStatics, bool& allOldBoxesStatics)
{
	//To help us gather the two lists of sorted boxes we are going to use a bitmap and our knowledge of the indices of the new boxes
	const PxU32 bitmapWordCount = ((mBoxesCapacity*2 + 31) & ~31)/32;
	Cm::TmpMem<PxU32, 8> bitMapMem(bitmapWordCount);
	PxU32* bitMapWords = bitMapMem.getBase();
	PxMemSet(bitMapWords, 0, sizeof(PxU32)*bitmapWordCount);
	Cm::BitMap bitmap;
	bitmap.setWords(bitMapWords, bitmapWordCount);

	const PxU32 axis0 = 0;
	const PxU32 axis1 = 2;
	const PxU32 axis2 = 1;

	const PxU32 insertAABBStart = 0;
	const PxU32 insertAABBEnd = mCreatedSize;
	const BpHandle* PX_RESTRICT createdAABBs = mCreated;
	SapBox1D** PX_RESTRICT asapBoxes = mBoxEndPts;
	const Bp::FilterGroup::Enum* PX_RESTRICT asapBoxGroupIds = mBoxGroups;
	BpHandle* PX_RESTRICT asapEndPointDatas = mEndPointDatas[axis0];
	const PxU32 numSortedEndPoints = mBoxesSize*2 + NUM_SENTINELS;

	//Set the bitmap for new box ids and compute the aabb (of the sorted handles/indices and not of the values) that bounds all new boxes.
	
	PxU32 globalAABBMinX = PX_MAX_U32;
	PxU32 globalAABBMinY = PX_MAX_U32;
	PxU32 globalAABBMinZ = PX_MAX_U32;
	PxU32 globalAABBMaxX = 0;
	PxU32 globalAABBMaxY = 0;
	PxU32 globalAABBMaxZ = 0;
	
	// PT: TODO: compute the global bounds from the initial data, more cache/SIMD-friendly
	// => maybe doesn't work, we're dealing with indices here not actual float values IIRC
	for(PxU32 i=insertAABBStart;i<insertAABBEnd;i++)
	{
		const PxU32 boxId = createdAABBs[i];
		bitmap.set(boxId);

		globalAABBMinX = PxMin(globalAABBMinX, PxU32(asapBoxes[axis0][boxId].mMinMax[0]));
		globalAABBMaxX = PxMax(globalAABBMaxX, PxU32(asapBoxes[axis0][boxId].mMinMax[1]));
		globalAABBMinY = PxMin(globalAABBMinY, PxU32(asapBoxes[axis1][boxId].mMinMax[0]));
		globalAABBMaxY = PxMax(globalAABBMaxY, PxU32(asapBoxes[axis1][boxId].mMinMax[1]));
		globalAABBMinZ = PxMin(globalAABBMinZ, PxU32(asapBoxes[axis2][boxId].mMinMax[0]));
		globalAABBMaxZ = PxMax(globalAABBMaxZ, PxU32(asapBoxes[axis2][boxId].mMinMax[1]));
	}

/*	PxU32 _globalAABBMinX = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMin.x));
	PxU32 _globalAABBMinY = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMin.y));
	PxU32 _globalAABBMinZ = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMin.z));
	PxU32 _globalAABBMaxX = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMax.x));
	PxU32 _globalAABBMaxY = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMax.y));
	PxU32 _globalAABBMaxZ = IntegerAABB::encodeFloatMin(PxUnionCast<PxU32, PxF32>(globalMax.z));
	(void)_globalAABBMinX;*/

	PxU32 oldStaticCount=0;
	PxU32 newStaticCount=0;

	//Assign the sorted end pts to the appropriate arrays.
	// PT: TODO: we could just do this loop before inserting the new endpts, i.e. no need for a bitmap etc
	// => but we need to insert the pts first to have valid mMinMax data in the above loop.
	// => but why do we iterate over endpoints and then skip the mins? Why not iterate directly over boxes?  ====> probably to get sorted results
	// => we could then just use the regular bounds data etc
	for(PxU32 i=1;i<numSortedEndPoints-1;i++)
	{
		//Make sure we haven't encountered a sentinel - 
		//they should only be at each end of the array.
		PX_ASSERT(!isSentinel(asapEndPointDatas[i]));
		PX_ASSERT(!isSentinel(asapEndPointDatas[i]));
		PX_ASSERT(!isSentinel(asapEndPointDatas[i]));

		if(!isMax(asapEndPointDatas[i]))
		{
			const BpHandle boxId = BpHandle(getOwner(asapEndPointDatas[i]));
			if(!bitmap.test(boxId))
			{
				if(Intersect3D(
					globalAABBMinX, globalAABBMaxX, globalAABBMinY, globalAABBMaxY, globalAABBMinZ, globalAABBMaxZ,
					asapBoxes[axis0][boxId].mMinMax[0],
					asapBoxes[axis0][boxId].mMinMax[1],
					asapBoxes[axis1][boxId].mMinMax[0],
					asapBoxes[axis1][boxId].mMinMax[1],
					asapBoxes[axis2][boxId].mMinMax[0],
					asapBoxes[axis2][boxId].mMinMax[1]))
				{
					oldBoxIndicesSorted[oldBoxIndicesCount++] = boxId;
					oldStaticCount += asapBoxGroupIds[boxId]==FilterGroup::eSTATICS ? 0 : 1;
				}
			}
			else
			{
				newBoxIndicesSorted[newBoxIndicesCount++] = boxId;
				newStaticCount += asapBoxGroupIds[boxId]==FilterGroup::eSTATICS ? 0 : 1;
			}
		}
	}

	allOldBoxesStatics = oldStaticCount ? false : true;
	allNewBoxesStatics = newStaticCount ? false : true;

	//Make sure that we've found the correct number of boxes.
	PX_ASSERT(newBoxIndicesCount==(insertAABBEnd-insertAABBStart));
	PX_ASSERT(oldBoxIndicesCount<=((numSortedEndPoints-NUM_SENTINELS)/2));
}

//#include "PsVecMath.h"
//using namespace shdfnd::aos;

void BroadPhaseSap::batchCreate()
{
	if(!mCreatedSize)
		return;	// Early-exit if no object has been created

//	PxVec4 globalMin, globalMax;
	{
	//Number of newly-created boxes (still to be sorted) and number of old boxes (already sorted).
	const PxU32 numNewBoxes = mCreatedSize;
	//const PxU32 numOldBoxes = mBoxesSize - mCreatedSize;

	//Array of newly-created box indices.
	const BpHandle* PX_RESTRICT created = mCreated;

	//Arrays of min and max coords for each box for each axis.
	const PxBounds3* PX_RESTRICT minMax = mBoxBoundsMinMax;

/*	{
		PxU32 nbToGo = numNewBoxes-1;
		const PxU32 lastBoxId = created[nbToGo];
		const PxVec3 lastMin = minMax[lastBoxId].minimum;
		const PxVec3 lastMax = minMax[lastBoxId].maximum;
		PxVec4 minI(lastMin.x, lastMin.y, lastMin.z, 0.0f);
		PxVec4 maxI(lastMax.x, lastMax.y, lastMax.z, 0.0f);
		const Vec4V dist4 = V4Load(mContactDistance[lastBoxId]);
		Vec4V resultMinV = V4Sub(V4LoadU(&lastMin.x), dist4);
		Vec4V resultMaxV = V4Add(V4LoadU(&lastMax.x), dist4);

		const BpHandle* src = created;
		while(nbToGo--)
		{
			const PxU32 boxId = *src++;
			const Vec4V d4 = V4Load(mContactDistance[boxId]);
			resultMinV = V4Min(resultMinV, V4Sub(V4LoadU(&minMax[boxId].minimum.x), d4));
			resultMaxV = V4Max(resultMaxV, V4Add(V4LoadU(&minMax[boxId].maximum.x), d4));
		}
		V4StoreU(resultMinV, &globalMin.x);
		V4StoreU(resultMaxV, &globalMax.x);
	}*/

	//Insert new boxes into sorted endpoints lists.
	{
		const PxU32 numEndPoints = numNewBoxes*2;

		Cm::TmpMem<ValType, 32> nepsv(numEndPoints), bv(numEndPoints);
		ValType* newEPSortedValues = nepsv.getBase();
		ValType* bufferValues = bv.getBase();

		// PT: TODO: use the scratch allocator
		Cm::RadixSortBuffered RS;

		for(PxU32 Axis=0;Axis<3;Axis++)
		{
			for(PxU32 i=0;i<numNewBoxes;i++)
			{
				const PxU32 boxIndex = PxU32(created[i]);
				PX_ASSERT(mBoxEndPts[Axis][boxIndex].mMinMax[0]==BP_INVALID_BP_HANDLE || mBoxEndPts[Axis][boxIndex].mMinMax[0]==PX_REMOVED_BP_HANDLE);
				PX_ASSERT(mBoxEndPts[Axis][boxIndex].mMinMax[1]==BP_INVALID_BP_HANDLE || mBoxEndPts[Axis][boxIndex].mMinMax[1]==PX_REMOVED_BP_HANDLE);

//				const ValType minValue = minMax[boxIndex].getMin(Axis);
//				const ValType maxValue = minMax[boxIndex].getMax(Axis);
				const PxReal contactDistance = mContactDistance[boxIndex];
				newEPSortedValues[i*2+0] = encodeMin(minMax[boxIndex], Axis, contactDistance);
				newEPSortedValues[i*2+1] = encodeMax(minMax[boxIndex], Axis, contactDistance);
			}

			// Sort endpoints backwards
			BpHandle* bufferDatas;
			{
				RS.invalidateRanks();	// PT: there's no coherence between axes
				const PxU32* Sorted = RS.Sort(newEPSortedValues, numEndPoints, Cm::RADIX_UNSIGNED).GetRanks();
				bufferDatas = RS.GetRecyclable();

				// PT: TODO: with two passes here we could reuse the "newEPSortedValues" buffer and drop "bufferValues"
				for(PxU32 i=0;i<numEndPoints;i++)
				{
					const PxU32 sortedIndex = Sorted[numEndPoints-1-i];
					bufferValues[i] = newEPSortedValues[sortedIndex];
					// PT: compute buffer data on-the-fly, store in recyclable buffer
					const PxU32 boxIndex = PxU32(created[sortedIndex>>1]);
					bufferDatas[i] = setData(boxIndex, (sortedIndex&1)!=0);
				}
			}

			InsertEndPoints(bufferValues, bufferDatas, numEndPoints, mEndPointValues[Axis], mEndPointDatas[Axis], 2*(mBoxesSize-mCreatedSize)+NUM_SENTINELS, mBoxEndPts[Axis]);
		}
	}

	//Some debug tests.
#if PX_DEBUG
	{
		for(PxU32 i=0;i<numNewBoxes;i++)
		{
			PxU32 BoxIndex = PxU32(created[i]);
			PX_ASSERT(mBoxEndPts[0][BoxIndex].mMinMax[0]!=BP_INVALID_BP_HANDLE && mBoxEndPts[0][BoxIndex].mMinMax[0]!=PX_REMOVED_BP_HANDLE);
			PX_ASSERT(mBoxEndPts[0][BoxIndex].mMinMax[1]!=BP_INVALID_BP_HANDLE && mBoxEndPts[0][BoxIndex].mMinMax[1]!=PX_REMOVED_BP_HANDLE);
			PX_ASSERT(mBoxEndPts[1][BoxIndex].mMinMax[0]!=BP_INVALID_BP_HANDLE && mBoxEndPts[1][BoxIndex].mMinMax[0]!=PX_REMOVED_BP_HANDLE);
			PX_ASSERT(mBoxEndPts[1][BoxIndex].mMinMax[1]!=BP_INVALID_BP_HANDLE && mBoxEndPts[1][BoxIndex].mMinMax[1]!=PX_REMOVED_BP_HANDLE);
			PX_ASSERT(mBoxEndPts[2][BoxIndex].mMinMax[0]!=BP_INVALID_BP_HANDLE && mBoxEndPts[2][BoxIndex].mMinMax[0]!=PX_REMOVED_BP_HANDLE);
			PX_ASSERT(mBoxEndPts[2][BoxIndex].mMinMax[1]!=BP_INVALID_BP_HANDLE && mBoxEndPts[2][BoxIndex].mMinMax[1]!=PX_REMOVED_BP_HANDLE);
		}
		for(PxU32 i=0;i<mBoxesSize*2+1;i++)
		{
			PX_ASSERT(mEndPointValues[0][i] <= mEndPointValues[0][i+1]);
			PX_ASSERT(mEndPointValues[1][i] <= mEndPointValues[1][i+1]);
			PX_ASSERT(mEndPointValues[2][i] <= mEndPointValues[2][i+1]);
		}
	}
#endif
	}

	// Perform box-pruning
	{
	// PT: TODO: use the scratch allocator in Cm::TmpMem

	//Number of newly-created boxes (still to be sorted) and number of old boxes (already sorted).
	const PxU32 numNewBoxes = mCreatedSize;
	const PxU32 numOldBoxes = mBoxesSize - mCreatedSize;

	//Gather two list of sorted boxes along the preferred axis direction: 
	//one list for new boxes and one list for existing boxes.
	//Only gather the existing boxes that overlap the bounding box of 
	//all new boxes.
	Cm::TmpMem<BpHandle, 8> oldBoxesIndicesSortedMem(numOldBoxes);
	Cm::TmpMem<BpHandle, 8> newBoxesIndicesSortedMem(numNewBoxes);
	BpHandle* oldBoxesIndicesSorted = oldBoxesIndicesSortedMem.getBase();
	BpHandle* newBoxesIndicesSorted = newBoxesIndicesSortedMem.getBase();
	PxU32 oldBoxCount = 0;
	PxU32 newBoxCount = 0;

	bool allNewBoxesStatics = false;
	bool allOldBoxesStatics = false;
	// PT: TODO: separate static/dynamic to speed things up, compute "minPosList" etc at the same time
	// PT: TODO: isn't "newBoxesIndicesSorted" the same as what we already computed in batchCreate() ?
	//Ready to gather the two lists now.
	ComputeSortedLists(/*globalMin, globalMax,*/ newBoxesIndicesSorted, newBoxCount, oldBoxesIndicesSorted, oldBoxCount, allNewBoxesStatics, allOldBoxesStatics);

	//Intersect new boxes with new boxes and new boxes with existing boxes.
	if(!allNewBoxesStatics || !allOldBoxesStatics)
	{
		const AuxData data0(newBoxCount, mBoxEndPts, newBoxesIndicesSorted, mBoxGroups);

		if(!allNewBoxesStatics)
		{
			performBoxPruningNewNew(&data0, mScratchAllocator,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
				mLUT,
#endif
				mPairs, mData, mDataSize, mDataCapacity);
		}

		// the old boxes are not the first ones in the array
		if(numOldBoxes)
		{
			if(oldBoxCount)
			{
				const AuxData data1(oldBoxCount, mBoxEndPts, oldBoxesIndicesSorted, mBoxGroups);

				performBoxPruningNewOld(&data0, &data1, mScratchAllocator,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
				mLUT,
#endif
					mPairs, mData, mDataSize, mDataCapacity);
			}
		}
	}
	}
}

///////////////////////////////////////////////////////////////////////////////

void BroadPhaseSap::batchRemove()
{
	if(!mRemovedSize)	return;	// Early-exit if no object has been removed

	//The box count is incremented when boxes are added to the create list but these boxes
	//haven't yet been added to the pair manager or the sorted axis lists.  We need to 
	//pretend that the box count is the value it was when the bp was last updated.
	//Then, at the end, we need to set the box count to the number that includes the boxes
	//in the create list and subtract off the boxes that have been removed.
	PxU32 currBoxesSize=mBoxesSize;
	mBoxesSize=mBoxesSizePrev;

	for(PxU32 Axis=0;Axis<3;Axis++)
	{
		ValType* const BaseEPValue = mEndPointValues[Axis];
		BpHandle* const BaseEPData = mEndPointDatas[Axis];
		PxU32 MinMinIndex = PX_MAX_U32;
		for(PxU32 i=0;i<mRemovedSize;i++)
		{
			PX_ASSERT(mRemoved[i]<mBoxesCapacity);

			const PxU32 MinIndex = mBoxEndPts[Axis][mRemoved[i]].mMinMax[0];
			PX_ASSERT(MinIndex<mBoxesCapacity*2+2);
			PX_ASSERT(getOwner(BaseEPData[MinIndex])==mRemoved[i]);

			const PxU32 MaxIndex = mBoxEndPts[Axis][mRemoved[i]].mMinMax[1];
			PX_ASSERT(MaxIndex<mBoxesCapacity*2+2);
			PX_ASSERT(getOwner(BaseEPData[MaxIndex])==mRemoved[i]);

			PX_ASSERT(MinIndex<MaxIndex);

			BaseEPData[MinIndex] = PX_REMOVED_BP_HANDLE;
			BaseEPData[MaxIndex] = PX_REMOVED_BP_HANDLE;

			if(MinIndex<MinMinIndex)	
				MinMinIndex = MinIndex;
		}

		PxU32 ReadIndex = MinMinIndex;
		PxU32 DestIndex = MinMinIndex;
		const PxU32 Limit = mBoxesSize*2+NUM_SENTINELS;
		while(ReadIndex!=Limit)
		{
			Ps::prefetchLine(&BaseEPData[ReadIndex],128);
			while(ReadIndex!=Limit && BaseEPData[ReadIndex] == PX_REMOVED_BP_HANDLE)
			{
				Ps::prefetchLine(&BaseEPData[ReadIndex],128);
				ReadIndex++;
			}
			if(ReadIndex!=Limit)
			{
				if(ReadIndex!=DestIndex)
				{
					BaseEPValue[DestIndex] = BaseEPValue[ReadIndex];
					BaseEPData[DestIndex] = BaseEPData[ReadIndex];
					PX_ASSERT(BaseEPData[DestIndex] != PX_REMOVED_BP_HANDLE);
					if(!isSentinel(BaseEPData[DestIndex]))
					{
						BpHandle BoxOwner = getOwner(BaseEPData[DestIndex]);
						PX_ASSERT(BoxOwner<mBoxesCapacity);
						mBoxEndPts[Axis][BoxOwner].mMinMax[isMax(BaseEPData[DestIndex])] = BpHandle(DestIndex);
					}
				}
				DestIndex++;
				ReadIndex++;
			}
		}
	}

	for(PxU32 i=0;i<mRemovedSize;i++)
	{
		const PxU32 handle=mRemoved[i];
		mBoxEndPts[0][handle].mMinMax[0]=PX_REMOVED_BP_HANDLE;
		mBoxEndPts[0][handle].mMinMax[1]=PX_REMOVED_BP_HANDLE;
		mBoxEndPts[1][handle].mMinMax[0]=PX_REMOVED_BP_HANDLE;
		mBoxEndPts[1][handle].mMinMax[1]=PX_REMOVED_BP_HANDLE;
		mBoxEndPts[2][handle].mMinMax[0]=PX_REMOVED_BP_HANDLE;
		mBoxEndPts[2][handle].mMinMax[1]=PX_REMOVED_BP_HANDLE;
	}

	const PxU32 bitmapWordCount=1+(mBoxesCapacity>>5);
	Cm::TmpMem<PxU32, 128> bitmapWords(bitmapWordCount);
	PxMemZero(bitmapWords.getBase(),sizeof(PxU32)*bitmapWordCount);
	Cm::BitMap bitmap;
	bitmap.setWords(bitmapWords.getBase(),bitmapWordCount);
	for(PxU32 i=0;i<mRemovedSize;i++)
	{
		PxU32 Index = mRemoved[i];
		PX_ASSERT(Index<mBoxesCapacity);
		PX_ASSERT(0==bitmap.test(Index));
		bitmap.set(Index);
	}
	mPairs.RemovePairs(bitmap);

	mBoxesSize=currBoxesSize;
	mBoxesSize-=mRemovedSize;
	mBoxesSizePrev=mBoxesSize-mCreatedSize;
}

static BroadPhasePair* resizeBroadPhasePairArray(const PxU32 oldMaxNb, const PxU32 newMaxNb, PxcScratchAllocator* scratchAllocator, BroadPhasePair* elements)
{
	PX_ASSERT(newMaxNb > oldMaxNb);
	PX_ASSERT(newMaxNb > 0);
	PX_ASSERT(0==((newMaxNb*sizeof(BroadPhasePair)) & 15)); 
	BroadPhasePair* newElements = reinterpret_cast<BroadPhasePair*>(scratchAllocator->alloc(sizeof(BroadPhasePair)*newMaxNb, true));
	PX_ASSERT(0==(uintptr_t(newElements) & 0x0f));
	PxMemCopy(newElements, elements, oldMaxNb*sizeof(BroadPhasePair));
	scratchAllocator->free(elements);
	return newElements;
}

#define PERFORM_COMPARISONS 1


void BroadPhaseSap::batchUpdate
(const PxU32 Axis, BroadPhasePair*& pairs, PxU32& pairsSize, PxU32& pairsCapacity)
{
	//Nothin updated so don't do anything
	if(mUpdatedSize == 0)
		return;

		//If number updated is sufficiently fewer than number of boxes (say less than 20%)
	if((mUpdatedSize*5) < mBoxesSize)
	{
		batchUpdateFewUpdates(Axis, pairs, pairsSize, pairsCapacity);
		return;
	}

	PxU32 numPairs=0;
	PxU32 maxNumPairs=pairsCapacity;

	const PxBounds3* PX_RESTRICT boxMinMax3D = mBoxBoundsMinMax;
	SapBox1D* boxMinMax2D[6]={mBoxEndPts[1],mBoxEndPts[2],mBoxEndPts[2],mBoxEndPts[0],mBoxEndPts[0],mBoxEndPts[1]};

	const SapBox1D* PX_RESTRICT boxMinMax0=boxMinMax2D[2*Axis+0];
	const SapBox1D* PX_RESTRICT boxMinMax1=boxMinMax2D[2*Axis+1];

#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE 
	const Bp::FilterGroup::Enum* PX_RESTRICT asapBoxGroupIds=mBoxGroups;
#endif

	SapBox1D* PX_RESTRICT asapBoxes=mBoxEndPts[Axis];

	ValType* PX_RESTRICT asapEndPointValues=mEndPointValues[Axis];
	BpHandle* PX_RESTRICT asapEndPointDatas=mEndPointDatas[Axis];

	ValType* const PX_RESTRICT BaseEPValues = asapEndPointValues;
	BpHandle* const PX_RESTRICT BaseEPDatas = asapEndPointDatas;			

	PxU8* PX_RESTRICT updated = mBoxesUpdated;

	//KS - can we lazy create these inside the loop? Might benefit us

	//There are no extents, jus the sentinels, so exit early.
	if(isSentinel(BaseEPDatas[1]))
		return;

	//We are going to skip the 1st element in the array (the sublist will be sorted)
	//but we must first update its value if it has moved
	//const PxU32 startIsMax = isMax(BaseEPDatas[1]);
	PX_ASSERT(!isMax(BaseEPDatas[1]));
	const BpHandle startHandle = getOwner(BaseEPDatas[1]);

	//KS - in theory, we should just be able to grab the min element but there's some issue where a body's max < min (i.e. an invalid extents) that
	//appears in a unit test
	// ValType ThisValue_ = boxMinMax3D[startHandle].getMin(Axis);
	ValType ThisValue_ = encodeMin(boxMinMax3D[startHandle], Axis, mContactDistance[startHandle]);

	BaseEPValues[1] = ThisValue_;
	
	PxU32 updateCounter = mUpdatedSize*2;

	updateCounter -= updated[startHandle];

	//We'll never overlap with this sentinel but it just ensures that we don't need to branch to see if
	//there's a pocket that we need to test against
	
	BroadPhaseActivityPocket* PX_RESTRICT currentPocket = mActivityPockets;

	currentPocket->mEndIndex = 0;
	currentPocket->mStartIndex = 0;

	BpHandle ind = 2;
	PxU8 wasUpdated = updated[startHandle];
	for(; !isSentinel(BaseEPDatas[ind]); ++ind)
	{
		BpHandle ThisData = BaseEPDatas[ind];

		const BpHandle handle = getOwner(ThisData);

		if(updated[handle] || wasUpdated)
		{
			wasUpdated = updated[handle];
			updateCounter -= wasUpdated;

			BpHandle ThisIndex = ind;

			const BpHandle startIsMax = isMax(ThisData);

			//Access and write back the updated values. TODO - can we avoid this when we're walking through inactive nodes?
			//BPValType ThisValue = boxMinMax1D[Axis][twoHandle+startIsMax];
			//BPValType ThisValue = startIsMax ? boxMinMax3D[handle].getMax(Axis) : boxMinMax3D[handle].getMin(Axis);
			//ValType ThisValue = boxMinMax3D[handle].getExtent(startIsMax, Axis);

			ValType ThisValue = startIsMax ? encodeMax(boxMinMax3D[handle], Axis, mContactDistance[handle])
										   : encodeMin(boxMinMax3D[handle], Axis, mContactDistance[handle]);

			BaseEPValues[ThisIndex] = ThisValue;

			PX_ASSERT(handle!=BP_INVALID_BP_HANDLE);

			//We always iterate back through the list...

			BpHandle CurrentIndex = mListPrev[ThisIndex];
			ValType CurrentValue = BaseEPValues[CurrentIndex];
			//PxBpHandle CurrentData = BaseEPDatas[CurrentIndex];

			if(CurrentValue > ThisValue)
			{
				wasUpdated = 1;
				//Get the bounds of the curr aabb.
				//Get the box1d of the curr aabb.
				/*const SapBox1D* PX_RESTRICT Object=&asapBoxes[handle];
				PX_ASSERT(Object->mMinMax[0]!=BP_INVALID_BP_HANDLE);
				PX_ASSERT(Object->mMinMax[1]!=BP_INVALID_BP_HANDLE);*/
				
				// const ValType boxMax=boxMinMax3D[handle].getMax(Axis);

				const ValType boxMax=encodeMax(boxMinMax3D[handle], Axis, mContactDistance[handle]);

				PxU32 endIndex = ind;
				PxU32 startIndex = ind;

#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
				const Bp::FilterGroup::Enum group = asapBoxGroupIds[handle];
#endif
				if(!isMax(ThisData))
				{
					do
					{
						BpHandle CurrentData = BaseEPDatas[CurrentIndex];
						const BpHandle IsMax = isMax(CurrentData);
						
	#if PERFORM_COMPARISONS
						if(IsMax)
						{		
							const BpHandle ownerId=getOwner(CurrentData);
							SapBox1D* PX_RESTRICT id1 = asapBoxes + ownerId;
							// Our min passed a max => start overlap

							if(
								BaseEPValues[id1->mMinMax[0]] < boxMax && 
								//2D intersection test using up-to-date values
								Intersect2D_Handle(boxMinMax0[handle].mMinMax[0], boxMinMax0[handle].mMinMax[1], boxMinMax1[handle].mMinMax[0], boxMinMax1[handle].mMinMax[1],
								            boxMinMax0[ownerId].mMinMax[0],boxMinMax0[ownerId].mMinMax[1],boxMinMax1[ownerId].mMinMax[0],boxMinMax1[ownerId].mMinMax[1])

	#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
		#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								&& groupFiltering(group, asapBoxGroupIds[ownerId], mLUT)
		#else
								&& groupFiltering(group, asapBoxGroupIds[ownerId])
		#endif
	#else
								&& handle!=ownerId
	#endif
								)
							{
								if(numPairs==maxNumPairs)
								{
									const PxU32 newMaxNumPairs=maxNumPairs*2;
									pairs = reinterpret_cast<BroadPhasePair*>(resizeBroadPhasePairArray(maxNumPairs, newMaxNumPairs, mScratchAllocator, pairs));
									maxNumPairs=newMaxNumPairs;
								}
								PX_ASSERT(numPairs<maxNumPairs);
								pairs[numPairs].mVolA=BpHandle(PxMax(handle, ownerId));
								pairs[numPairs].mVolB=BpHandle(PxMin(handle, ownerId));
								numPairs++;
								//AddPair(handle, getOwner(*CurrentMinData), mPairs, mData, mDataSize, mDataCapacity);
							}
						}
	#endif
						startIndex--;
						CurrentIndex = mListPrev[CurrentIndex];
						CurrentValue = BaseEPValues[CurrentIndex];
					}
					while(ThisValue < CurrentValue);
				}			
				else 
				{
					// Max is moving left:
					do
					{
						BpHandle CurrentData = BaseEPDatas[CurrentIndex];
						const BpHandle IsMax = isMax(CurrentData);
						
	#if PERFORM_COMPARISONS
						if(!IsMax)
						{
							// Our max passed a min => stop overlap
							const BpHandle ownerId=getOwner(CurrentData);

#if 1
							if(
#if BP_SAP_USE_OVERLAP_TEST_ON_REMOVES
								Intersect2D_Handle(boxMinMax0[handle].mMinMax[0], boxMinMax0[handle].mMinMax[1], boxMinMax1[handle].mMinMax[0], boxMinMax1[handle].mMinMax[1],
								       boxMinMax0[ownerId].mMinMax[0],boxMinMax0[ownerId].mMinMax[1],boxMinMax1[ownerId].mMinMax[0],boxMinMax1[ownerId].mMinMax[1])
#endif
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								&& groupFiltering(group, asapBoxGroupIds[ownerId], mLUT)
	#else
								&& groupFiltering(group, asapBoxGroupIds[ownerId])
	#endif
#else
								&& handle!=ownerId
#endif
								)
#endif
							{
								if(numPairs==maxNumPairs)
								{
									const PxU32 newMaxNumPairs=maxNumPairs*2;
									pairs = reinterpret_cast<BroadPhasePair*>(resizeBroadPhasePairArray(maxNumPairs, newMaxNumPairs, mScratchAllocator, pairs));
									maxNumPairs=newMaxNumPairs;
								}
								PX_ASSERT(numPairs<maxNumPairs);
								pairs[numPairs].mVolA=BpHandle(PxMin(handle, ownerId));
								pairs[numPairs].mVolB=BpHandle(PxMax(handle, ownerId));
								numPairs++;
								//RemovePair(handle, getOwner(*CurrentMaxData), mPairs, mData, mDataSize, mDataCapacity);
							}
						}
	#endif
						startIndex--;
						CurrentIndex = mListPrev[CurrentIndex];
						CurrentValue = BaseEPValues[CurrentIndex];
					}
					while(ThisValue < CurrentValue);
				}

				//This test is unnecessary. If we entered the outer loop, we're doing the swap in here
				{
					//Unlink from old position and re-link to new position
					BpHandle oldNextIndex = mListNext[ThisIndex];
					BpHandle oldPrevIndex = mListPrev[ThisIndex];

					BpHandle newNextIndex = mListNext[CurrentIndex];
					BpHandle newPrevIndex = CurrentIndex;
					
					//Unlink this node
					mListNext[oldPrevIndex] = oldNextIndex;
					mListPrev[oldNextIndex] = oldPrevIndex;

					//Link it to it's new place in the list
					mListNext[ThisIndex] = newNextIndex;
					mListPrev[ThisIndex] = newPrevIndex;
					mListPrev[newNextIndex] = ThisIndex;
					mListNext[newPrevIndex] = ThisIndex;
				}

				//There is a sentinel with 0 index, so we don't need
				//to worry about walking off the array				
				while(startIndex < currentPocket->mStartIndex)
				{
					currentPocket--;
				}
				//If our start index > currentPocket->mEndIndex, then we don't overlap so create a new pocket
				if(currentPocket == mActivityPockets || startIndex > (currentPocket->mEndIndex+1))
				{
					currentPocket++;
					currentPocket->mStartIndex = startIndex;
				}
				currentPocket->mEndIndex = endIndex;
			}// update max
			//ind++;
		}
		else if (updateCounter == 0) //We've updated all the bodies and neither this nor the previous body was updated, so we're done
			break;

	}// updated aabbs

	pairsSize=numPairs;
	pairsCapacity=maxNumPairs;


	BroadPhaseActivityPocket* pocket = mActivityPockets+1;

	while(pocket <= currentPocket)
	{
		for(PxU32 a = pocket->mStartIndex; a <= pocket->mEndIndex; ++a)
		{
			mListPrev[a] = BpHandle(a);
		}

		//Now copy all the data to the array, updating the remap table

		PxU32 CurrIndex = pocket->mStartIndex-1;
		for(PxU32 a = pocket->mStartIndex; a <= pocket->mEndIndex; ++a)
		{
			CurrIndex = mListNext[CurrIndex];
			PxU32 origIndex =  CurrIndex;
			BpHandle remappedIndex = mListPrev[origIndex];

			if(origIndex != a)
			{
				const BpHandle ownerId=getOwner(BaseEPDatas[remappedIndex]);
				const BpHandle IsMax = isMax(BaseEPDatas[remappedIndex]);
				ValType tmp = BaseEPValues[a];
				BpHandle tmpHandle = BaseEPDatas[a];

				BaseEPValues[a] = BaseEPValues[remappedIndex];
				BaseEPDatas[a] = BaseEPDatas[remappedIndex];

				BaseEPValues[remappedIndex] = tmp;
				BaseEPDatas[remappedIndex] = tmpHandle;

				mListPrev[remappedIndex] = mListPrev[a];
				//Write back remap index (should be an immediate jump to original index)
				mListPrev[mListPrev[a]] = remappedIndex;
				asapBoxes[ownerId].mMinMax[IsMax] = BpHandle(a);
			}
			
		}

		////Reset next and prev ptrs back
		for(PxU32 a = pocket->mStartIndex-1; a <= pocket->mEndIndex; ++a)
		{
			mListPrev[a+1] = BpHandle(a);
			mListNext[a] = BpHandle(a+1);
		}

		pocket++;
	}
	mListPrev[0] = 0;
}


void BroadPhaseSap::batchUpdateFewUpdates
(const PxU32 Axis, BroadPhasePair*& pairs, PxU32& pairsSize, PxU32& pairsCapacity)
{
	PxU32 numPairs=0;
	PxU32 maxNumPairs=pairsCapacity;

	const PxBounds3* PX_RESTRICT boxMinMax3D = mBoxBoundsMinMax;
	SapBox1D* boxMinMax2D[6]={mBoxEndPts[1],mBoxEndPts[2],mBoxEndPts[2],mBoxEndPts[0],mBoxEndPts[0],mBoxEndPts[1]};

#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE 
	const Bp::FilterGroup::Enum* PX_RESTRICT asapBoxGroupIds=mBoxGroups;
#endif

	SapBox1D* PX_RESTRICT asapBoxes=mBoxEndPts[Axis];

	/*const BPValType* PX_RESTRICT boxMinMax0=boxMinMax2D[2*Axis];
	const BPValType* PX_RESTRICT boxMinMax1=boxMinMax2D[2*Axis+1];*/

	ValType* PX_RESTRICT asapEndPointValues=mEndPointValues[Axis];
	BpHandle* PX_RESTRICT asapEndPointDatas=mEndPointDatas[Axis];

	ValType* const PX_RESTRICT BaseEPValues = asapEndPointValues;
	BpHandle* const PX_RESTRICT BaseEPDatas = asapEndPointDatas;			

	const SapBox1D* PX_RESTRICT boxMinMax0=boxMinMax2D[2*Axis+0];
	const SapBox1D* PX_RESTRICT boxMinMax1=boxMinMax2D[2*Axis+1];

	PxU8* PX_RESTRICT updated = mBoxesUpdated;

	const PxU32 endPointSize = mBoxesSize*2 + 1;

	//There are no extents, just the sentinels, so exit early.
	if(isSentinel(BaseEPDatas[1]))
		return;

	PxU32 ind_ = 0;

	PxU32 index = 1;

	if(mUpdatedSize < 512)
	{
		//The array of updated elements is small, so use qsort to sort them
		for(PxU32 a = 0; a < mUpdatedSize; ++a)
		{
			const PxU32 handle=mUpdated[a];

			const SapBox1D* Object=&asapBoxes[handle];
				
			PX_ASSERT(Object->mMinMax[0]!=BP_INVALID_BP_HANDLE);
			PX_ASSERT(Object->mMinMax[1]!=BP_INVALID_BP_HANDLE);

			//Get the bounds of the curr aabb.

//			const ValType boxMin=boxMinMax3D[handle].getMin(Axis);
//			const ValType boxMax=boxMinMax3D[handle].getMax(Axis);

			const ValType boxMin = encodeMin(boxMinMax3D[handle], Axis, mContactDistance[handle]);
			const ValType boxMax = encodeMax(boxMinMax3D[handle], Axis, mContactDistance[handle]);

			BaseEPValues[Object->mMinMax[0]] = boxMin;
			BaseEPValues[Object->mMinMax[1]] = boxMax;

			mSortedUpdateElements[ind_++] = Object->mMinMax[0];
			mSortedUpdateElements[ind_++] = Object->mMinMax[1];
		}
		Ps::sort(mSortedUpdateElements, ind_);
	}
	else
	{
		//The array of updated elements is large so use a bucket sort to sort them
		for(; index < endPointSize; ++index)
		{
			if(isSentinel( BaseEPDatas[index] ))
				break;
			BpHandle ThisData = BaseEPDatas[index];
			BpHandle owner = BpHandle(getOwner(ThisData));
			if(updated[owner])
			{
				//BPValType ThisValue = isMax(ThisData) ? boxMinMax3D[owner].getMax(Axis) : boxMinMax3D[owner].getMin(Axis);
				ValType ThisValue = isMax(ThisData) ? encodeMax(boxMinMax3D[owner], Axis, mContactDistance[owner])
													: encodeMin(boxMinMax3D[owner], Axis, mContactDistance[owner]);
				BaseEPValues[index] = ThisValue;
				mSortedUpdateElements[ind_++] = BpHandle(index);
			}
		}
	}

	const PxU32 updateCounter = ind_;
	
	//We'll never overlap with this sentinel but it just ensures that we don't need to branch to see if
	//there's a pocket that we need to test against
	BroadPhaseActivityPocket* PX_RESTRICT currentPocket = mActivityPockets;
	currentPocket->mEndIndex = 0;
	currentPocket->mStartIndex = 0;

	for(PxU32 a = 0; a < updateCounter; ++a)
	{
		BpHandle ind = mSortedUpdateElements[a];

		BpHandle NextData;
		BpHandle PrevData;
		do
		{
			BpHandle ThisData = BaseEPDatas[ind];

			const BpHandle handle = getOwner(ThisData);

			BpHandle ThisIndex = ind;
			ValType ThisValue = BaseEPValues[ThisIndex];

			//Get the box1d of the curr aabb.
			const SapBox1D* PX_RESTRICT Object=&asapBoxes[handle];

			PX_ASSERT(handle!=BP_INVALID_BP_HANDLE);

			PX_ASSERT(Object->mMinMax[0]!=BP_INVALID_BP_HANDLE);
			PX_ASSERT(Object->mMinMax[1]!=BP_INVALID_BP_HANDLE);
			PX_UNUSED(Object);

			//Get the bounds of the curr aabb.
			//const PxU32 twoHandle = 2*handle;
			
			const ValType boxMax=encodeMax(boxMinMax3D[handle], Axis, mContactDistance[handle]);

			//We always iterate back through the list...
			BpHandle CurrentIndex = mListPrev[ThisIndex];
			ValType CurrentValue = BaseEPValues[CurrentIndex];

			if(CurrentValue > ThisValue)
			{
				//We're performing some swaps so we need an activity pocket here. This structure allows us to keep track of the range of 
				//modifications in the sorted lists. Doesn't help when everything's moving but makes a really big difference to reconstituting the 
				//list when only a small number of things are moving

				PxU32 endIndex = ind;
				PxU32 startIndex = ind;

				//const BPValType* PX_RESTRICT box0MinMax0 = &boxMinMax0[twoHandle];
				//const BPValType* PX_RESTRICT box0MinMax1 = &boxMinMax1[twoHandle];
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
				const Bp::FilterGroup::Enum group = asapBoxGroupIds[handle];
#endif
				if(!isMax(ThisData))
				{
					do
					{
						BpHandle CurrentData = BaseEPDatas[CurrentIndex];
						const BpHandle IsMax = isMax(CurrentData);
						
	#if PERFORM_COMPARISONS
						if(IsMax)
						{		
							const BpHandle ownerId=getOwner(CurrentData);
							SapBox1D* PX_RESTRICT id1 = asapBoxes + ownerId;
							// Our min passed a max => start overlap

							if(
								BaseEPValues[id1->mMinMax[0]] < boxMax && 
								//2D intersection test using up-to-date values
								Intersect2D_Handle(boxMinMax0[handle].mMinMax[0], boxMinMax0[handle].mMinMax[1], boxMinMax1[handle].mMinMax[0], boxMinMax1[handle].mMinMax[1],
								       boxMinMax0[ownerId].mMinMax[0],boxMinMax0[ownerId].mMinMax[1],boxMinMax1[ownerId].mMinMax[0],boxMinMax1[ownerId].mMinMax[1])
	#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
		#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								&& groupFiltering(group, asapBoxGroupIds[ownerId], mLUT)
		#else
								&& groupFiltering(group, asapBoxGroupIds[ownerId])
		#endif
	#else
								&& Object!=id1
	#endif
								)
							{
								if(numPairs==maxNumPairs)
								{
									const PxU32 newMaxNumPairs=maxNumPairs*2;
									pairs = reinterpret_cast<BroadPhasePair*>(resizeBroadPhasePairArray(maxNumPairs, newMaxNumPairs, mScratchAllocator, pairs));
									maxNumPairs=newMaxNumPairs;
								}
								PX_ASSERT(numPairs<maxNumPairs);
								pairs[numPairs].mVolA=BpHandle(PxMax(handle, ownerId));
								pairs[numPairs].mVolB=BpHandle(PxMin(handle, ownerId));
								numPairs++;
								//AddPair(handle, getOwner(*CurrentMinData), mPairs, mData, mDataSize, mDataCapacity);
							}
						}
	#endif
						startIndex--;
						CurrentIndex = mListPrev[CurrentIndex];
						CurrentValue = BaseEPValues[CurrentIndex];
					}
					while(ThisValue < CurrentValue);
				}			
				else 
				{
					// Max is moving left:
					do
					{
						BpHandle CurrentData = BaseEPDatas[CurrentIndex];
						const BpHandle IsMax = isMax(CurrentData);
						
	#if PERFORM_COMPARISONS
						if(!IsMax)
						{
							// Our max passed a min => stop overlap
							const BpHandle ownerId=getOwner(CurrentData);

#if 1
							if(
#if BP_SAP_USE_OVERLAP_TEST_ON_REMOVES
								Intersect2D_Handle(boxMinMax0[handle].mMinMax[0], boxMinMax0[handle].mMinMax[1], boxMinMax1[handle].mMinMax[0], boxMinMax1[handle].mMinMax[1],
								       boxMinMax0[ownerId].mMinMax[0],boxMinMax0[ownerId].mMinMax[1],boxMinMax1[ownerId].mMinMax[0],boxMinMax1[ownerId].mMinMax[1])
#endif
#if BP_SAP_TEST_GROUP_ID_CREATEUPDATE
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
								&& groupFiltering(group, asapBoxGroupIds[ownerId], mLUT)
	#else
								&& groupFiltering(group, asapBoxGroupIds[ownerId])
	#endif
#else
								&& Object!=id1
#endif
								)
#endif
							{
								if(numPairs==maxNumPairs)
								{
									const PxU32 newMaxNumPairs=maxNumPairs*2;
									pairs = reinterpret_cast<BroadPhasePair*>(resizeBroadPhasePairArray(maxNumPairs, newMaxNumPairs, mScratchAllocator, pairs));
									maxNumPairs=newMaxNumPairs;
								}
								PX_ASSERT(numPairs<maxNumPairs);
								pairs[numPairs].mVolA=BpHandle(PxMin(handle, ownerId));
								pairs[numPairs].mVolB=BpHandle(PxMax(handle, ownerId));
								numPairs++;
								//RemovePair(handle, getOwner(*CurrentMaxData), mPairs, mData, mDataSize, mDataCapacity);
							}
						}
	#endif
						startIndex--;
						CurrentIndex = mListPrev[CurrentIndex];
						CurrentValue = BaseEPValues[CurrentIndex];
					}
					while(ThisValue < CurrentValue);
				}

				//This test is unnecessary. If we entered the outer loop, we're doing the swap in here
				{
					//Unlink from old position and re-link to new position
					BpHandle oldNextIndex = mListNext[ThisIndex];
					BpHandle oldPrevIndex = mListPrev[ThisIndex];

					BpHandle newNextIndex = mListNext[CurrentIndex];
					BpHandle newPrevIndex = CurrentIndex;
					
					//Unlink this node
					mListNext[oldPrevIndex] = oldNextIndex;
					mListPrev[oldNextIndex] = oldPrevIndex;

					//Link it to it's new place in the list
					mListNext[ThisIndex] = newNextIndex;
					mListPrev[ThisIndex] = newPrevIndex;
					mListPrev[newNextIndex] = ThisIndex;
					mListNext[newPrevIndex] = ThisIndex;
				}

				//Loop over the activity pocket stack to make sure this set of shuffles didn't 
				//interfere with the previous set. If it did, we roll this pocket into the previous
				//pockets. If everything in the scene is moving, we should result in just 1 pocket
				while(startIndex < currentPocket->mStartIndex)
				{
					currentPocket--;
				}
				//If our start index > currentPocket->mEndIndex, then we don't overlap so create a new pocket
				if(currentPocket == mActivityPockets || startIndex > (currentPocket->mEndIndex+1))
				{
					currentPocket++;
					currentPocket->mStartIndex = startIndex;
				}
				currentPocket->mEndIndex = endIndex;
			}// update max
			//Get prev and next ptr...

			NextData = BaseEPDatas[++ind];
			PrevData = BaseEPDatas[mListPrev[ind]];

		}while(!isSentinel(NextData) && !updated[getOwner(NextData)] && updated[getOwner(PrevData)]);
		
	}// updated aabbs

	pairsSize=numPairs;
	pairsCapacity=maxNumPairs;


	BroadPhaseActivityPocket* pocket = mActivityPockets+1;

	while(pocket <= currentPocket)
	{
		//PxU32 CurrIndex = mListPrev[pocket->mStartIndex];
		for(PxU32 a = pocket->mStartIndex; a <= pocket->mEndIndex; ++a)
		{
			mListPrev[a] = BpHandle(a);
		}

		//Now copy all the data to the array, updating the remap table
		PxU32 CurrIndex = pocket->mStartIndex-1;
		for(PxU32 a = pocket->mStartIndex; a <= pocket->mEndIndex; ++a)
		{
			CurrIndex = mListNext[CurrIndex];
			PxU32 origIndex =  CurrIndex;
			BpHandle remappedIndex = mListPrev[origIndex];

			if(origIndex != a)
			{
				const BpHandle ownerId=getOwner(BaseEPDatas[remappedIndex]);
				const BpHandle IsMax = isMax(BaseEPDatas[remappedIndex]);
				ValType tmp = BaseEPValues[a];
				BpHandle tmpHandle = BaseEPDatas[a];

				BaseEPValues[a] = BaseEPValues[remappedIndex];
				BaseEPDatas[a] = BaseEPDatas[remappedIndex];

				BaseEPValues[remappedIndex] = tmp;
				BaseEPDatas[remappedIndex] = tmpHandle;

				mListPrev[remappedIndex] = mListPrev[a];
				//Write back remap index (should be an immediate jump to original index)
				mListPrev[mListPrev[a]] = remappedIndex;
				asapBoxes[ownerId].mMinMax[IsMax] = BpHandle(a);
			}
			
		}

		for(PxU32 a = pocket->mStartIndex-1; a <= pocket->mEndIndex; ++a)
		{
			mListPrev[a+1] = BpHandle(a);
			mListNext[a] = BpHandle(a+1);
		}
		pocket++;
	}
}

#if PX_DEBUG

bool BroadPhaseSap::isSelfOrdered() const 
{
	if(0==mBoxesSize)
	{
		return true;
	}

	for(PxU32 Axis=0;Axis<3;Axis++)
	{
		PxU32 it=1;
		PX_ASSERT(mEndPointDatas[Axis]);
		while(!isSentinel(mEndPointDatas[Axis][it]))
		{
			//Test the array is sorted.
			const ValType prevVal=mEndPointValues[Axis][it-1];
			const ValType currVal=mEndPointValues[Axis][it];
			if(currVal<prevVal)
			{
				return false;
			}

			//Test the end point array is consistent.
			const BpHandle ismax=isMax(mEndPointDatas[Axis][it]);
			const BpHandle ownerId=getOwner(mEndPointDatas[Axis][it]);
			if(mBoxEndPts[Axis][ownerId].mMinMax[ismax]!=it)
			{
				return false;
			}

			//Test the mins are even, the maxes are odd, and the extents are finite.
			const ValType boxMin = mEndPointValues[Axis][mBoxEndPts[Axis][ownerId].mMinMax[0]];
			const ValType boxMax = mEndPointValues[Axis][mBoxEndPts[Axis][ownerId].mMinMax[1]];
			if(boxMin & 1)
			{
				return false;
			}
			if(0==(boxMax & 1))
			{
				return false;
			}
			if(boxMax<=boxMin)
			{
				return false;
			}

			it++;
		}
	}

	return true;
}

bool BroadPhaseSap::isSelfConsistent() const 
{
	if(0==mBoxesSize)
	{
		return true;
	}

	for(PxU32 Axis=0;Axis<3;Axis++)
	{
		PxU32 it=1;
		ValType prevVal=0;
		const PxBounds3* PX_RESTRICT boxMinMax = mBoxBoundsMinMax;
		const PxReal* PX_RESTRICT contactDistance = mContactDistance;
		PX_ASSERT(mEndPointDatas[Axis]);
		while(!isSentinel(mEndPointDatas[Axis][it]))
		{
			const BpHandle ownerId=getOwner(mEndPointDatas[Axis][it]);
			const BpHandle ismax=isMax(mEndPointDatas[Axis][it]);
			const ValType boxMinMaxs[2] = { encodeMin(boxMinMax[ownerId], Axis, contactDistance[ownerId]),
											encodeMax(boxMinMax[ownerId], Axis, contactDistance[ownerId]) };
//			const ValType boxMinMaxs[2] = { boxMinMax[ownerId].getMin(Axis), boxMinMax[ownerId].getMax(Axis) };
			const ValType test1=boxMinMaxs[ismax];
			const ValType test2=mEndPointValues[Axis][it];
			if(test1!=test2)
			{
				return false;
			}
			if(test2<prevVal)
			{
				return false;
			}
			prevVal=test2;

			if(mBoxEndPts[Axis][ownerId].mMinMax[ismax]!=it)
			{
				return false;
			}

			it++;
		}
	}

	for(PxU32 i=0;i<mCreatedPairsSize;i++)
	{
		const PxU32 a=mCreatedPairsArray[i].mVolA;
		const PxU32 b=mCreatedPairsArray[i].mVolB;
		IntegerAABB aabb0(mBoxBoundsMinMax[a], mContactDistance[a]);
		IntegerAABB aabb1(mBoxBoundsMinMax[b], mContactDistance[b]);
		if(!aabb0.intersects(aabb1))
		{
			return false;
		}
	}

	for(PxU32 i=0;i<mDeletedPairsSize;i++)
	{
		const PxU32 a=mDeletedPairsArray[i].mVolA;
		const PxU32 b=mDeletedPairsArray[i].mVolB;

		bool isDeleted=false;
		for(PxU32 j=0;j<mRemovedSize;j++)
		{
			if(a==mRemoved[j] || b==mRemoved[j])
			{
				isDeleted=true;
			}
		}

		if(!isDeleted)
		{
			IntegerAABB aabb0(mBoxBoundsMinMax[a], mContactDistance[a]);
			IntegerAABB aabb1(mBoxBoundsMinMax[b], mContactDistance[b]);
			if(aabb0.intersects(aabb1))
			{
				// with the past refactors this should have become illegal
				return false;
			}
		}
	}

	return true;
}
#endif

} //namespace Bp

} //namespace physx


