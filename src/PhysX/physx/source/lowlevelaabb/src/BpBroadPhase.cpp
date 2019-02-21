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

#include "BpBroadPhase.h"
#include "BpBroadPhaseSap.h"
#include "BpBroadPhaseMBP.h"
#include "PxSceneDesc.h"
#include "CmBitMap.h"

using namespace physx;
using namespace Bp;
using namespace Cm;

BroadPhase* createABP(
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID);

BroadPhase* createMBP4(
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID);

BroadPhase* BroadPhase::create(
	const PxBroadPhaseType::Enum bpType,
	const PxU32 maxNbRegions,
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID)
{
	PX_ASSERT(bpType==PxBroadPhaseType::eMBP || bpType == PxBroadPhaseType::eSAP || bpType == PxBroadPhaseType::eABP);

	if(bpType==PxBroadPhaseType::eABP)
		return createABP(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
//		return createMBP4(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
	else if(bpType==PxBroadPhaseType::eMBP)
		return PX_NEW(BroadPhaseMBP)(maxNbRegions, maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
	else
		return PX_NEW(BroadPhaseSap)(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
//		return createABP(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
}


#if PX_CHECKED
bool BroadPhaseUpdateData::isValid(const BroadPhaseUpdateData& updateData, const BroadPhase& bp)
{
	return (updateData.isValid() && bp.isValid(updateData));
}

static bool testHandles(PxU32 size, const BpHandle* handles, const PxU32 capacity, const Bp::FilterGroup::Enum* groups, const PxBounds3* bounds, BitMap& bitmap)
{
	if(!handles && size)
		return false;

/*	ValType minVal=0;
	ValType maxVal=0xffffffff;*/

	for(PxU32 i=0;i<size;i++)
	{
		const BpHandle h = handles[i];

		if(h>=capacity)
			return false;

		// Array in ascending order of id.
		if(i>0 && (h < handles[i-1]))
			return false;

		if(groups && groups[h]==FilterGroup::eINVALID)
			return false;

		bitmap.set(h);

		if(bounds)
		{
			for(PxU32 j=0;j<3;j++)
			{
				//Max must be greater than min.
				if(bounds[h].minimum[j]>bounds[h].maximum[j])
					return false;
#if 0
				//Bounds have an upper limit.
				if(bounds[created[i]].getMax(j)>=maxVal)
					return false;

				//Bounds have a lower limit.
				if(bounds[created[i]].getMin(j)<=minVal)
					return false;

				//Max must be odd.
				if(4 != (bounds[created[i]].getMax(j) & 4))
					return false;

				//Min must be even.
				if(0 != (bounds[created[i]].getMin(j) & 4))
					return false;
#endif
			}
		}
	}
	return true;
}

static bool testBitmap(const BitMap& bitmap, PxU32 size, const BpHandle* handles)
{
	while(size--)
	{
		const BpHandle h = *handles++;
		if(bitmap.test(h))
			return false;
	}
	return true;
}

bool BroadPhaseUpdateData::isValid() const 
{
	const PxBounds3* bounds = getAABBs();
	const PxU32 boxesCapacity = getCapacity();
	const Bp::FilterGroup::Enum* groups = getGroups();

	BitMap createdBitmap;	createdBitmap.resizeAndClear(boxesCapacity);
	BitMap updatedBitmap;	updatedBitmap.resizeAndClear(boxesCapacity);
	BitMap removedBitmap;	removedBitmap.resizeAndClear(boxesCapacity);

	if(!testHandles(getNumCreatedHandles(), getCreatedHandles(), boxesCapacity, groups, bounds, createdBitmap))
		return false;
	if(!testHandles(getNumUpdatedHandles(), getUpdatedHandles(), boxesCapacity, groups, bounds, updatedBitmap))
		return false;
	if(!testHandles(getNumRemovedHandles(), getRemovedHandles(), boxesCapacity, NULL, NULL, removedBitmap))
		return false;

	if(1)
	{
		// Created/updated
		if(!testBitmap(createdBitmap, getNumUpdatedHandles(), getUpdatedHandles()))
			return false;
		// Created/removed
		if(!testBitmap(createdBitmap, getNumRemovedHandles(), getRemovedHandles()))
			return false;
		// Updated/removed
		if(!testBitmap(updatedBitmap, getNumRemovedHandles(), getRemovedHandles()))
			return false;
	}
	return true;
}
#endif
