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

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "CmPtrTable.h"
#include "CmUtils.h"
#include "PxMetaData.h"
#include "PsBitUtils.h"

using namespace physx;
using namespace Cm;

PtrTable::PtrTable()
: mList(NULL)
, mCount(0)
, mOwnsMemory(true)
, mBufferUsed(false)
{
}

PtrTable::~PtrTable()
{
	PX_ASSERT(mOwnsMemory);
	PX_ASSERT(mCount == 0);
	PX_ASSERT(mList == NULL);
}

void PtrTable::clear(PtrTableStorageManager& sm)
{
	if(mOwnsMemory && mCount>1)
	{
		PxU32 implicitCapacity = Ps::nextPowerOfTwo(PxU32(mCount)-1);
		sm.deallocate(mList, sizeof(void*)*implicitCapacity);
	}

	mList = NULL;
	mOwnsMemory = true;
	mCount = 0;
}

PxU32 PtrTable::find(const void* ptr) const
{
	const PxU32 nbPtrs = mCount;
	void*const * PX_RESTRICT ptrs = getPtrs();

	for(PxU32 i=0; i<nbPtrs; i++)
	{
		if(ptrs[i] == ptr)
			return i;
	}
	return 0xffffffff;
}

void PtrTable::exportExtraData(PxSerializationContext& stream)
{
	if(mCount>1)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mList, sizeof(void*)*mCount);
	}
}

void PtrTable::importExtraData(PxDeserializationContext& context)
{
	if(mCount>1)
		mList = context.readExtraData<void*, PX_SERIAL_ALIGN>(mCount);
}

void PtrTable::realloc(PxU32 oldCapacity, PxU32 newCapacity, PtrTableStorageManager& sm)
{
	PX_ASSERT((mOwnsMemory && oldCapacity) || (!mOwnsMemory && oldCapacity == 0));
	PX_ASSERT(newCapacity);

	if(mOwnsMemory && sm.canReuse(oldCapacity, newCapacity))
		return;

	void** newMem = sm.allocate(newCapacity * sizeof(void*));
	PxMemCopy(newMem, mList, mCount * sizeof(void*));

	if(mOwnsMemory)
		sm.deallocate(mList, oldCapacity*sizeof(void*));

	mList = newMem;
	mOwnsMemory = true;
}

void PtrTable::add(void* ptr, PtrTableStorageManager& sm)
{
	if(mCount == 0)												// 0 -> 1, easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mList == NULL);
		PX_ASSERT(!mBufferUsed);
		mSingle = ptr;
		mCount = 1;
		mBufferUsed = true;
		return;
	}
	
	if(mCount == 1)												// 1 -> 2, easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mBufferUsed);

		void* single = mSingle;
		mList = sm.allocate(2*sizeof(void*));
		mList[0] = single;
		mBufferUsed = false;
		mOwnsMemory = true;
	} 
	else 
	{
		PX_ASSERT(!mBufferUsed);

		if(!mOwnsMemory)										// don't own the memory, must always alloc
			realloc(0, Ps::nextPowerOfTwo(mCount), sm);			// we're guaranteed nextPowerOfTwo(x) > x

		else if(Ps::isPowerOfTwo(mCount))						// count is at implicit capacity, so realloc
			realloc(mCount, PxU32(mCount)*2, sm);				// ... to next higher power of 2

		PX_ASSERT(mOwnsMemory);
	}

	mList[mCount++] = ptr;
}

void PtrTable::replaceWithLast(PxU32 index, PtrTableStorageManager& sm)
{
	PX_ASSERT(mCount!=0);

	if(mCount == 1)												// 1 -> 0 easy case
	{
		PX_ASSERT(mOwnsMemory);
		PX_ASSERT(mBufferUsed);

		mList = NULL;
		mCount = 0;
		mBufferUsed = false;
	}
	else if(mCount == 2)										// 2 -> 1 easy case
	{
		PX_ASSERT(!mBufferUsed);
		void* ptr = mList[1-index];
		if(mOwnsMemory)
			sm.deallocate(mList, 2*sizeof(void*));
		mSingle = ptr;
		mCount = 1;
		mBufferUsed = true;
		mOwnsMemory = true;
	} 
	else
	{
		PX_ASSERT(!mBufferUsed);

		mList[index] = mList[--mCount];								// remove before adjusting memory

		if(!mOwnsMemory)											// don't own the memory, must alloc
			realloc(0, Ps::nextPowerOfTwo(PxU32(mCount)-1), sm);	// if currently a power of 2, don't jump to the next one

		else if(Ps::isPowerOfTwo(mCount))							// own the memory, and implicit capacity requires that we downsize
			realloc(PxU32(mCount)*2, PxU32(mCount), sm);			// ... from the next power of 2, which was the old implicit capacity

		PX_ASSERT(mOwnsMemory);
	}
}

void Cm::PtrTable::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PtrTable)

	PX_DEF_BIN_METADATA_ITEM(stream,	PtrTable, void,		mSingle,		PxMetaDataFlag::ePTR)		// PT: this is actually a union, beware
	PX_DEF_BIN_METADATA_ITEM(stream,	PtrTable, PxU16,	mCount,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PtrTable, bool,		mOwnsMemory,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PtrTable, bool,		mBufferUsed,	0)

	//------ Extra-data ------

	// mList
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, PtrTable, void, mBufferUsed, mCount, PxMetaDataFlag::eCONTROL_FLIP|PxMetaDataFlag::ePTR, PX_SERIAL_ALIGN)
}
