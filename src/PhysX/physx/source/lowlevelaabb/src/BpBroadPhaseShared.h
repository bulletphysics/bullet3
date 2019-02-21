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

#ifndef BP_BROADPHASE_SHARED_H
#define BP_BROADPHASE_SHARED_H

#include "BpBroadPhaseUpdate.h"
#include "PsUserAllocated.h"
#include "PsHash.h"
#include "PsVecMath.h"

namespace physx
{
namespace Bp
{
	#define	INVALID_ID		0xffffffff
	#define INVALID_USER_ID	0xffffffff

	struct InternalPair : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	PxU32	getId0()	const	{ return id0_isNew & ~PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	PxU32	getId1()	const	{ return id1_isUpdated & ~PX_SIGN_BITMASK;	}

		PX_FORCE_INLINE	PxU32	isNew()		const	{ return id0_isNew & PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	PxU32	isUpdated()	const	{ return id1_isUpdated & PX_SIGN_BITMASK;	}

		PX_FORCE_INLINE	void	setNewPair(PxU32 id0, PxU32 id1)
		{
			PX_ASSERT(!(id0 & PX_SIGN_BITMASK));
			PX_ASSERT(!(id1 & PX_SIGN_BITMASK));
			id0_isNew = id0 | PX_SIGN_BITMASK;
			id1_isUpdated = id1;
		}
		PX_FORCE_INLINE	void	setUpdated()		{ id1_isUpdated |= PX_SIGN_BITMASK;		}
		PX_FORCE_INLINE	void	clearUpdated()		{ id1_isUpdated &= ~PX_SIGN_BITMASK;	}
		PX_FORCE_INLINE	void	clearNew()			{ id0_isNew &= ~PX_SIGN_BITMASK;		}

		protected:
		PxU32		id0_isNew;
		PxU32		id1_isUpdated;
	};

	PX_FORCE_INLINE bool	differentPair(const InternalPair& p, PxU32 id0, PxU32 id1)	{ return (id0!=p.getId0()) || (id1!=p.getId1());	}
	PX_FORCE_INLINE PxU32	hash(PxU32 id0, PxU32 id1)									{ return PxU32(Ps::hash( (id0&0xffff)|(id1<<16)) );	}
	PX_FORCE_INLINE void	sort(PxU32& id0, PxU32& id1)								{ if(id0>id1)	Ps::swap(id0, id1);					}

	class PairManagerData
	{
		public:
										PairManagerData();
										~PairManagerData();

		PX_FORCE_INLINE	PxU32			getPairIndex(const InternalPair* pair)	const
										{
											return (PxU32((size_t(pair) - size_t(mActivePairs)))/sizeof(InternalPair));
										}

		// Internal version saving hash computation
		PX_FORCE_INLINE InternalPair*	findPair(PxU32 id0, PxU32 id1, PxU32 hashValue) const
										{
											if(!mHashTable)
												return NULL;	// Nothing has been allocated yet

											InternalPair* PX_RESTRICT activePairs = mActivePairs;
											const PxU32* PX_RESTRICT next = mNext;

											// Look for it in the table
											PxU32 offset = mHashTable[hashValue];
											while(offset!=INVALID_ID && differentPair(activePairs[offset], id0, id1))
											{
												PX_ASSERT(activePairs[offset].getId0()!=INVALID_USER_ID);
												offset = next[offset];		// Better to have a separate array for this
											}
											if(offset==INVALID_ID)
												return NULL;
											PX_ASSERT(offset<mNbActivePairs);
											// Match mActivePairs[offset] => the pair is persistent

											return &activePairs[offset];
										}

		PX_FORCE_INLINE	InternalPair*	addPairInternal(PxU32 id0, PxU32 id1)
										{
											// Order the ids
											sort(id0, id1);

											const PxU32 fullHashValue = hash(id0, id1);
											PxU32 hashValue = fullHashValue & mMask;

											{
												InternalPair* PX_RESTRICT p = findPair(id0, id1, hashValue);
												if(p)
												{
													p->setUpdated();
													return p;	// Persistent pair
												}
											}

											// This is a new pair
											if(mNbActivePairs >= mHashSize)
												hashValue = growPairs(fullHashValue);

											const PxU32 pairIndex = mNbActivePairs++;

											InternalPair* PX_RESTRICT p = &mActivePairs[pairIndex];
											p->setNewPair(id0, id1);
											mNext[pairIndex] = mHashTable[hashValue];
											mHashTable[hashValue] = pairIndex;
											return p;
										}

						PxU32			mHashSize;
						PxU32			mMask;
						PxU32			mNbActivePairs;
						PxU32*			mHashTable;
						PxU32*			mNext;
						InternalPair*	mActivePairs;
						PxU32			mReservedMemory;

						void			purge();
						void			reallocPairs();
						void			shrinkMemory();
						void			reserveMemory(PxU32 memSize);
		PX_NOINLINE		PxU32			growPairs(PxU32 fullHashValue);
						void			removePair(PxU32 id0, PxU32 id1, PxU32 hashValue, PxU32 pairIndex);
	};

	struct AABB_Xi : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	AABB_Xi()	{}
		PX_FORCE_INLINE	~AABB_Xi()	{}

		PX_FORCE_INLINE	void	initFromFloats(const void* PX_RESTRICT minX, const void* PX_RESTRICT maxX)
		{
			mMinX = encodeFloat(*reinterpret_cast<const PxU32*>(minX));
			mMaxX = encodeFloat(*reinterpret_cast<const PxU32*>(maxX));
		}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			initFromFloats(&min.x, &max.x);
		}

		PX_FORCE_INLINE	void	operator = (const AABB_Xi& box)
		{
			mMinX = box.mMinX;
			mMaxX = box.mMaxX;
		}

		PX_FORCE_INLINE	void	initSentinel()
		{
			mMinX = 0xffffffff;
		}

		PX_FORCE_INLINE bool	isSentinel()	const
		{
			return mMinX == 0xffffffff;
		}

		PxU32 mMinX;
		PxU32 mMaxX;
	};

	struct AABB_YZn : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	AABB_YZn()	{}
		PX_FORCE_INLINE	~AABB_YZn()	{}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			mMinY	= -min.y;
			mMinZ	= -min.z;
			mMaxY	= max.y;
			mMaxZ	= max.z;
		}

		PX_FORCE_INLINE	void	operator = (const AABB_YZn& box)
		{
			using namespace physx::shdfnd::aos;
			V4StoreA(V4LoadA(&box.mMinY), &mMinY);
		}

		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
	};

	struct AABB_YZr : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	AABB_YZr()	{}
		PX_FORCE_INLINE	~AABB_YZr()	{}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
			mMinY	= min.y;
			mMinZ	= min.z;
			mMaxY	= max.y;
			mMaxZ	= max.z;
		}

		PX_FORCE_INLINE	void	operator = (const AABB_YZr& box)
		{
			using namespace physx::shdfnd::aos;
			V4StoreA(V4LoadA(&box.mMinY), &mMinY);
		}

		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
	};

} //namespace Bp
} //namespace physx

#endif // BP_BROADPHASE_SHARED_H
