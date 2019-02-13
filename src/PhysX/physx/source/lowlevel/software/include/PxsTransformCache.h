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

#ifndef PXS_TRANSFORM_CACHE_H
#define PXS_TRANSFORM_CACHE_H

#include "CmPhysXCommon.h"
#include "CmIDPool.h"
#include "CmBitMap.h"
#include "PsUserAllocated.h"
#include "PsAllocator.h"

#define PX_DEFAULT_CACHE_SIZE 512

namespace physx
{
	struct PxsTransformFlag
	{
		enum Flags
		{
			eFROZEN = (1 << 0)
		};
	};

	struct PX_ALIGN_PREFIX(16) PxsCachedTransform
	{
		PxTransform transform;
		PxU32 flags;

		PX_FORCE_INLINE PxU32 isFrozen() const { return flags & PxsTransformFlag::eFROZEN; }
	}
	PX_ALIGN_SUFFIX(16);


	class PxsTransformCache : public Ps::UserAllocated
	{
		typedef PxU32 RefCountType;

	public:
		PxsTransformCache(Ps::VirtualAllocatorCallback& allocatorCallback) : mTransformCache(Ps::VirtualAllocator(&allocatorCallback)), mHasAnythingChanged(true)
		{
			/*mTransformCache.reserve(PX_DEFAULT_CACHE_SIZE);
			mTransformCache.forceSize_Unsafe(PX_DEFAULT_CACHE_SIZE);*/
			mUsedSize = 0;
		}

		void initEntry(PxU32 index)
		{
			PxU32 oldCapacity = mTransformCache.capacity();
			if (index >= oldCapacity)
			{
				PxU32 newCapacity = Ps::nextPowerOfTwo(index);
				mTransformCache.reserve(newCapacity);
				mTransformCache.forceSize_Unsafe(newCapacity);
			}
			mUsedSize = PxMax(mUsedSize, index + 1u);
		}


		PX_FORCE_INLINE void setTransformCache(const PxTransform& transform, const PxU32 flags, const PxU32 index)
		{
			mTransformCache[index].transform = transform;
			mTransformCache[index].flags = flags;
			mHasAnythingChanged = true;
		}

		PX_FORCE_INLINE const PxsCachedTransform& getTransformCache(const PxU32 index) const
		{
			return mTransformCache[index];
		}


		PX_FORCE_INLINE PxsCachedTransform& getTransformCache(const PxU32 index)
		{
			return mTransformCache[index];
		}

		PX_FORCE_INLINE void shiftTransforms(const PxVec3& shift)
		{
			for (PxU32 i = 0; i < mTransformCache.capacity(); i++)
			{
				mTransformCache[i].transform.p += shift;
			}
			mHasAnythingChanged = true;
		}

		PX_FORCE_INLINE PxU32 getTotalSize() const
		{
			return mUsedSize;
		}

		PX_FORCE_INLINE const PxsCachedTransform* getTransforms() const
		{
			return mTransformCache.begin();
		}

		PX_FORCE_INLINE PxsCachedTransform* getTransforms()
		{
			return mTransformCache.begin();
		}

		PX_FORCE_INLINE Ps::Array<PxsCachedTransform, Ps::VirtualAllocator>* getCachedTransformArray()
		{
			return &mTransformCache;
		}

		PX_FORCE_INLINE	void resetChangedState()	{ mHasAnythingChanged = false;	}
		PX_FORCE_INLINE	void setChangedState()		{ mHasAnythingChanged = true;	}
		PX_FORCE_INLINE	bool hasChanged()	const	{ return mHasAnythingChanged;	}

	private:
		Ps::Array<PxsCachedTransform, Ps::VirtualAllocator>	mTransformCache;
		PxU32												mUsedSize;
		bool												mHasAnythingChanged;
	};
}

#endif
