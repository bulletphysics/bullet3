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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "foundation/PxMemory.h"
#include "SqPruningPool.h"

using namespace physx;
using namespace Sq;
using namespace Cm;

PruningPool::PruningPool() :
	mNbObjects			(0),
	mMaxNbObjects		(0),
	mWorldBoxes			(NULL),
	mObjects			(NULL),
	mHandleToIndex		(NULL),
	mIndexToHandle		(NULL),
	mFirstRecycledHandle(INVALID_PRUNERHANDLE)
{
}

PruningPool::~PruningPool()
{
	PX_FREE_AND_RESET(mWorldBoxes);
	PX_FREE_AND_RESET(mObjects);
	PX_FREE_AND_RESET(mHandleToIndex);
	PX_FREE_AND_RESET(mIndexToHandle);
}

bool PruningPool::resize(PxU32 newCapacity)
{
	// PT: we always allocate one extra box, to make sure we can safely use V4 loads on the array
	PxBounds3*		newBoxes			= reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(newCapacity+1), "PxBounds3"));
	PrunerPayload*	newData				= reinterpret_cast<PrunerPayload*>(PX_ALLOC(sizeof(PrunerPayload)*newCapacity, "PrunerPayload*"));
	PrunerHandle*	newIndexToHandle	= reinterpret_cast<PrunerHandle*>(PX_ALLOC(sizeof(PrunerHandle)*newCapacity, "Pruner Index Mapping"));
	PoolIndex*		newHandleToIndex	= reinterpret_cast<PoolIndex*>(PX_ALLOC(sizeof(PoolIndex)*newCapacity, "Pruner Index Mapping"));
	if( (NULL==newBoxes) || (NULL==newData) || (NULL==newIndexToHandle) || (NULL==newHandleToIndex)
		)
	{
		PX_FREE_AND_RESET(newBoxes);
		PX_FREE_AND_RESET(newData);
		PX_FREE_AND_RESET(newIndexToHandle);
		PX_FREE_AND_RESET(newHandleToIndex);
		return false;
	}

	if(mWorldBoxes)		PxMemCopy(newBoxes, mWorldBoxes, mNbObjects*sizeof(PxBounds3));
	if(mObjects)		PxMemCopy(newData, mObjects, mNbObjects*sizeof(PrunerPayload));
	if(mIndexToHandle)	PxMemCopy(newIndexToHandle, mIndexToHandle, mNbObjects*sizeof(PrunerHandle));
	if(mHandleToIndex)	PxMemCopy(newHandleToIndex, mHandleToIndex, mMaxNbObjects*sizeof(PoolIndex));
	mMaxNbObjects = newCapacity;

	PX_FREE_AND_RESET(mWorldBoxes);
	PX_FREE_AND_RESET(mObjects);
	PX_FREE_AND_RESET(mHandleToIndex);
	PX_FREE_AND_RESET(mIndexToHandle);
	mWorldBoxes		= newBoxes;
	mObjects		= newData;
	mHandleToIndex	= newHandleToIndex;
	mIndexToHandle	= newIndexToHandle;

	return true;
}

void PruningPool::preallocate(PxU32 newCapacity)
{
	if(newCapacity>mMaxNbObjects)
		resize(newCapacity);
}

PxU32 PruningPool::addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* payload, PxU32 count)
{
	for(PxU32 i=0;i<count;i++)
	{
		if(mNbObjects==mMaxNbObjects) // increase the capacity on overflow
		{
			if(!resize(PxMax<PxU32>(mMaxNbObjects*2, 64)))
			{
				// pool can return an invalid handle if memory alloc fails
				// should probably have an error here or not handle this
				results[i] = INVALID_PRUNERHANDLE;	// PT: we need to write the potentially invalid handle to let users know which object failed first
				return i;
			}
		}
		PX_ASSERT(mNbObjects!=mMaxNbObjects);

		const PoolIndex index = mNbObjects++;

		// update mHandleToIndex and mIndexToHandle mappings
		PrunerHandle handle;
		if(mFirstRecycledHandle != INVALID_PRUNERHANDLE)
		{
			// mFirstRecycledHandle is an entry into a freelist for removed slots
			// this path is only taken if we have any removed slots
			handle = mFirstRecycledHandle;
			mFirstRecycledHandle = mHandleToIndex[handle];
		}
		else
		{
			handle = index;
		}

		// PT: TODO: investigate why we added mIndexToHandle/mHandleToIndex. The initial design with 'Prunable' objects didn't need these arrays.

		// PT: these 3 arrays are "parallel"
		mWorldBoxes		[index] = bounds[i]; // store the payload and AABB in parallel arrays
		mObjects		[index] = payload[i];
		mIndexToHandle	[index] = handle;

		mHandleToIndex[handle] = index;
		results[i] = handle;
	}
	return count;
}

PoolIndex PruningPool::removeObject(PrunerHandle h)
{
	PX_ASSERT(mNbObjects);

	// remove the object and its AABB by provided PrunerHandle and update mHandleToIndex and mIndexToHandle mappings
	const PoolIndex indexOfRemovedObject = mHandleToIndex[h]; // retrieve object's index from handle

	const PoolIndex indexOfLastObject = --mNbObjects; // swap the object at last index with index
	if(indexOfLastObject!=indexOfRemovedObject)
	{
		// PT: move last object's data to recycled spot (from removed object)

		// PT: the last object has moved so we need to handle the mappings for this object
		// PT: TODO: investigate where this double-mapping comes from. Should not be needed...

		// PT: these 3 arrays are "parallel"
		const PrunerHandle handleOfLastObject	= mIndexToHandle[indexOfLastObject];
		mWorldBoxes		[indexOfRemovedObject]	= mWorldBoxes	[indexOfLastObject];
		mObjects		[indexOfRemovedObject]	= mObjects		[indexOfLastObject];
		mIndexToHandle	[indexOfRemovedObject]	= handleOfLastObject;

		mHandleToIndex[handleOfLastObject]		= indexOfRemovedObject;
	}

	// mHandleToIndex also stores the freelist for removed handles (in place of holes formed by removed handles)
	mHandleToIndex[h] = mFirstRecycledHandle; // update linked list of available recycled handles
	mFirstRecycledHandle = h; // update the list head

	return indexOfLastObject;
}

void PruningPool::shiftOrigin(const PxVec3& shift)
{
	for(PxU32 i=0; i < mNbObjects; i++)
	{
		mWorldBoxes[i].minimum -= shift;
		mWorldBoxes[i].maximum -= shift;
	}
}
