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


#ifndef PX_PHYSICS_COMMON_REFCOUNTABLE
#define PX_PHYSICS_COMMON_REFCOUNTABLE

#include "foundation/PxAssert.h"
#include "PsAtomic.h"

namespace physx
{
namespace Cm
{

	// simple thread-safe reference count
	// when the ref count is zero, the object is in an undefined state (pending delete)

	class RefCountable
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:
// PX_SERIALIZATION
		RefCountable(const PxEMPTY) : mRefCount(1)	{}
		static	void	getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
		explicit RefCountable(PxU32 initialCount = 1)
			: mRefCount(PxI32(initialCount))
		{
			PX_ASSERT(mRefCount!=0);
		}

		virtual ~RefCountable() {}

		/**
		Calls 'delete this;'. It needs to be overloaded for classes also deriving from 
		PxBase and call 'Cm::deletePxBase(this);' instead.
		*/
		virtual	void onRefCountZero()
		{
			delete this;
		}

		void incRefCount()
		{
			physx::shdfnd::atomicIncrement(&mRefCount);
			// value better be greater than 1, or we've created a ref to an undefined object
			PX_ASSERT(mRefCount>1);
		}

		void decRefCount()
		{
			PX_ASSERT(mRefCount>0);
			if(physx::shdfnd::atomicDecrement(&mRefCount) == 0)
				onRefCountZero();
		}

		PX_FORCE_INLINE PxU32 getRefCount() const
		{
			return PxU32(mRefCount);
		}
	private:
		volatile PxI32 mRefCount;
	};


} // namespace Cm

}

#endif
