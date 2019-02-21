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


#ifndef PX_PHYSICS_COMMON_PTR_TABLE
#define PX_PHYSICS_COMMON_PTR_TABLE

#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"

namespace physx
{

class PxSerializationContext;
class PxDeserializationContext;


namespace Cm
{

class PtrTableStorageManager
{
	// This will typically be backed by a MultiPool implementation with fallback to the user
	// allocator. For MultiPool, when deallocating we want to know what the previously requested size was
	// so we can release into the right pool

public:

	// capacity is in bytes

	virtual void**	allocate(PxU32 capacity)												= 0;
	virtual void	deallocate(void** addr, PxU32 originalCapacity)							= 0;

	// whether memory allocated at one capacity can (and should) be safely reused at a different capacity
	// allows realloc-style reuse by clients.

	virtual bool	canReuse(PxU32 originalCapacity, PxU32 newCapacity)		= 0;
protected:
	virtual ~PtrTableStorageManager() {}
};



// specialized class to hold an array of pointers with extrinsic storage management, 
// serialization-compatible with 3.3.1 PtrTable
//
// note that extrinsic storage implies you *must* clear the table before the destructor runs
//
// capacity is implicit: 
// if the memory is not owned (i.e. came from deserialization) then the capacity is exactly mCount
// else if mCount==0,  capacity is 0
// else the capacity is the power of 2 >= mCount
// 
// one implication of this is that if we want to add or remove a pointer from unowned memory, we always realloc

struct PX_PHYSX_COMMON_API PtrTable
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PtrTable();
	~PtrTable();

	void	add(void* ptr, PtrTableStorageManager& sm);
	void	replaceWithLast(PxU32 index, PtrTableStorageManager& sm);
	void	clear(PtrTableStorageManager& sm);

	PxU32	find(const void* ptr) const;

	PX_FORCE_INLINE PxU32		getCount()	const	{ return mCount; }
	PX_FORCE_INLINE	void*const*	getPtrs()	const	{ return mCount == 1 ? &mSingle : mList;	}
	PX_FORCE_INLINE	void**		getPtrs()			{ return mCount == 1 ? &mSingle : mList;	}


	// SERIALIZATION

	// 3.3.1 compatibility fixup: this implementation ALWAYS sets 'ownsMemory' if the size is 0 or 1
	PtrTable(const PxEMPTY)
	{
		mOwnsMemory = mCount<2;
		if(mCount == 0)
			mList = NULL;
	}

	void	exportExtraData(PxSerializationContext& stream);
	void	importExtraData(PxDeserializationContext& context);

	static void getBinaryMetaData(physx::PxOutputStream& stream);

private:
	void realloc(PxU32 oldCapacity, PxU32 newCapacity, PtrTableStorageManager& sm);

	union
	{
		void*	mSingle;
		void**	mList;
	};

	PxU16	mCount;
	bool	mOwnsMemory;
	bool	mBufferUsed;		// dark magic in serialization requires this, otherwise redundant because it's logically equivalent to mCount == 1.

};

} // namespace Cm
#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(Cm::PtrTable)==8);
#endif

}

#endif
