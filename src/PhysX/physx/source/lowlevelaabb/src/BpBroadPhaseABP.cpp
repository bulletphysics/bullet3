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

#include "foundation/PxProfiler.h"
#include "BpBroadPhaseABP.h"
#include "BpBroadPhaseShared.h"
#include "CmRadixSortBuffered.h"
#include "PsFoundation.h"
#include "PsVecMath.h"
#include "PxcScratchAllocator.h"

using namespace physx::shdfnd::aos;
using namespace physx;
using namespace Bp;
using namespace Cm;

#define CHECKPOINT(x)
//#include <stdio.h>
//#define CHECKPOINT(x)	printf(x);

//#pragma warning (disable : 4702)
#define	CODEALIGN16		//_asm	align 16
#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
	#define ABP_SIMD_OVERLAP
#endif

#define ABP_BATCHING		128
#define USE_ABP_BUCKETS		5000	// PT: don't use buckets below that number...
#ifdef USE_ABP_BUCKETS
	#define NB_BUCKETS	5
	// Regular version: 5 buckets a la bucket pruner (4 + cross bucket)
	// Alternative version: 4 buckets + dup objects a la MBP regions
//	#define USE_ALTERNATIVE_VERSION
	#define ABP_USE_INTEGER_XS2	// Works but questionable speedups
#else
	#define ABP_USE_INTEGER_XS
#endif
#define NB_SENTINELS		6

//#define RECURSE_LIMIT	20000

	typedef	PxU32	ABP_Index;

	static const bool gPrepareOverlapsFlag = true;
#ifdef ABP_SIMD_OVERLAP
	static const bool gUseRegularBPKernel = false;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = true;			// true to use "version 14" in box pruning series
#else
	// PT: tested on Switch, for some reason the regular version is fastest there
	static const bool gUseRegularBPKernel = true;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = false;			// true to use "version 14" in box pruning series

	//ABP_SIMD_OVERLAP
	//MBP.Add64KObjects                            13982 ( +0.0%)   4757795 ( +0.0%)  FAIL
	//MBP.AddBroadPhaseRegion                          0 ( +0.0%)   3213795 ( +0.0%)  FAIL
	//MBP.FinalizeOverlaps64KObjects                 507 ( +0.0%)   5650723 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KMixedObjects              59258 ( +0.0%)   5170179 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KObjects                   31351 ( +0.0%)   7122019 ( +0.0%)  FAIL
	//MBP.Remove64KObjects                          4993 ( +0.0%)   5281683 ( +0.0%)  FAIL
	//MBP.Update64KObjects                         13711 ( +0.0%)   5521699 ( +0.0%)  FAIL

	//gUseRegularBPKernel:
	//MBP.Add64KObjects                            14406 ( +0.0%)   4757795 ( +0.0%)  FAIL
	//MBP.AddBroadPhaseRegion                          0 ( +0.0%)   3213795 ( +0.0%)  FAIL
	//MBP.FinalizeOverlaps64KObjects                 504 ( +0.0%)   5650723 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KMixedObjects              48929 ( +0.0%)   5170179 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KObjects                   25636 ( +0.0%)   7122019 ( +0.0%)  FAIL
	//MBP.Remove64KObjects                          4878 ( +0.0%)   5281683 ( +0.0%)  FAIL
	//MBP.Update64KObjects                         13932 ( +0.0%)   5521699 ( +0.0%)  FAIL

	// false/true
	//MBP.Add64KObjects                            14278 ( +0.0%)   4757795 ( +0.0%)  FAIL
	//MBP.AddBroadPhaseRegion                          0 ( +0.0%)   3213795 ( +0.0%)  FAIL
	//MBP.FinalizeOverlaps64KObjects                 504 ( +0.0%)   5650723 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KMixedObjects              60331 ( +0.0%)   5170179 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KObjects                   32064 ( +0.0%)   7122019 ( +0.0%)  FAIL
	//MBP.Remove64KObjects                          4930 ( +0.0%)   5281683 ( +0.0%)  FAIL
	//MBP.Update64KObjects                         13673 ( +0.0%)   5521699 ( +0.0%)  FAIL

	// false/false
	//MBP.Add64KObjects                            13960 ( +0.0%)   4757795 ( +0.0%)  FAIL
	//MBP.AddBroadPhaseRegion                          0 ( +0.0%)   3213795 ( +0.0%)  FAIL
	//MBP.FinalizeOverlaps64KObjects                 503 ( +0.0%)   5650723 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KMixedObjects              48549 ( +0.0%)   5170179 ( +0.0%)  FAIL
	//MBP.FindOverlaps64KObjects                   25598 ( +0.0%)   7122019 ( +0.0%)  FAIL
	//MBP.Remove64KObjects                          4883 ( +0.0%)   5281683 ( +0.0%)  FAIL
	//MBP.Update64KObjects                         13667 ( +0.0%)   5521699 ( +0.0%)  FAIL
#endif

#ifdef ABP_USE_INTEGER_XS
	typedef	PxU32			PosXType;
	#define	SentinelValue	0xffffffff
#else
	typedef	float			PosXType;
	#define	SentinelValue	FLT_MAX
#endif

#ifdef ABP_USE_INTEGER_XS2
	typedef	PxU32			PosXType2;
	#define	SentinelValue2	0xffffffff
#else
	#ifdef ABP_USE_INTEGER_XS
	typedef	PxU32			PosXType2;
	#define	SentinelValue2	0xffffffff
	#else
	typedef	float			PosXType2;
	#define	SentinelValue2	FLT_MAX
	#endif
#endif

namespace internalABP{

	struct SIMD_AABB4 : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	void	initFrom2(const PxBounds3& box)
		{
#ifdef ABP_USE_INTEGER_XS
			mMinX = encodeFloat(PX_IR(box.minimum.x));
			mMaxX = encodeFloat(PX_IR(box.maximum.x));
			mMinY = box.minimum.y;
			mMinZ = box.minimum.z;
			mMaxY = box.maximum.y;
			mMaxZ = box.maximum.z;
#else
			mMinX = box.minimum.x;
			mMinY = box.minimum.y;
			mMinZ = box.minimum.z;
			mMaxX = box.maximum.x;
			mMaxY = box.maximum.y;
			mMaxZ = box.maximum.z;
#endif
		}

		PX_FORCE_INLINE	void	operator = (const SIMD_AABB4& box)
		{
			mMinX = box.mMinX;
			mMinY = box.mMinY;
			mMinZ = box.mMinZ;
			mMaxX = box.mMaxX;
			mMaxY = box.mMaxY;
			mMaxZ = box.mMaxZ;
		}

		PX_FORCE_INLINE	void	initSentinel()
		{
			mMinX = SentinelValue;
		}

		PX_FORCE_INLINE bool	isSentinel()	const
		{
			return mMinX == SentinelValue;
		}

#ifdef USE_ABP_BUCKETS
		// PT: to be able to compute bounds easily
		PosXType mMinX;
		float mMinY;
		float mMinZ;
		PosXType mMaxX;
		float mMaxY;
		float mMaxZ;
#else
		PosXType mMinX;
		PosXType mMaxX;
		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
#endif
	};

#define USE_SHARED_CLASSES
#ifdef USE_SHARED_CLASSES
	struct SIMD_AABB_X4 : public AABB_Xi
	{
		PX_FORCE_INLINE	void	initFrom(const SIMD_AABB4& box)
		{
	#ifdef ABP_USE_INTEGER_XS2
			initFromFloats(&box.mMinX, &box.mMaxX);
	#else
			mMinX = box.mMinX;
			mMaxX = box.mMaxX;
	#endif
		}
	};

	PX_ALIGN_PREFIX(16)
	#ifdef ABP_SIMD_OVERLAP
	struct SIMD_AABB_YZ4 : AABB_YZn
	{
		PX_FORCE_INLINE	void	initFrom(const SIMD_AABB4& box)
		{
	#ifdef ABP_SIMD_OVERLAP
			mMinY	= -box.mMinY;
			mMinZ	= -box.mMinZ;
	#else
			mMinY	= box.mMinY;
			mMinZ	= box.mMinZ;
	#endif
			mMaxY	= box.mMaxY;
			mMaxZ	= box.mMaxZ;
		}
	}
	#else
	struct SIMD_AABB_YZ4 : AABB_YZr
	{
		PX_FORCE_INLINE	void	initFrom(const SIMD_AABB4& box)
		{
			mMinY	= box.mMinY;
			mMinZ	= box.mMinZ;
			mMaxY	= box.mMaxY;
			mMaxZ	= box.mMaxZ;
		}
	}
	#endif
	PX_ALIGN_SUFFIX(16);
#else
	struct SIMD_AABB_X4 : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	void	initFromFloats(const void* PX_RESTRICT minX, const void* PX_RESTRICT maxX)
		{
			mMinX = encodeFloat(*reinterpret_cast<const PxU32*>(minX));
			mMaxX = encodeFloat(*reinterpret_cast<const PxU32*>(maxX));
		}

		PX_FORCE_INLINE	void	initFrom(const SIMD_AABB4& box)
		{
	#ifdef ABP_USE_INTEGER_XS2
			initFromFloats(&box.mMinX, &box.mMaxX);
	#else
			mMinX = box.mMinX;
			mMaxX = box.mMaxX;
	#endif
		}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
	#ifdef ABP_USE_INTEGER_XS2
			initFromFloats(&min.x, &max.x);
	#else
		#ifdef ABP_USE_INTEGER_XS
			initFromFloats(&min.x, &max.x);
		#else
			mMinX = min.x;
			mMaxX = max.x;
		#endif
	#endif
		}

		PX_FORCE_INLINE	void	operator = (const SIMD_AABB_X4& box)
		{
			mMinX = box.mMinX;
			mMaxX = box.mMaxX;
		}

		PX_FORCE_INLINE	void	initSentinel()
		{
			mMinX = SentinelValue2;
		}

		PX_FORCE_INLINE bool	isSentinel()	const
		{
			return mMinX == SentinelValue2;
		}

		PosXType2 mMinX;
		PosXType2 mMaxX;
	};

	struct SIMD_AABB_YZ4 : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	void	initFrom(const SIMD_AABB4& box)
		{
	#ifdef ABP_SIMD_OVERLAP
			mMinY	= -box.mMinY;
			mMinZ	= -box.mMinZ;
	#else
			mMinY	= box.mMinY;
			mMinZ	= box.mMinZ;
	#endif
			mMaxY	= box.mMaxY;
			mMaxZ	= box.mMaxZ;
		}

		PX_FORCE_INLINE	void	initFromPxVec4(const PxVec4& min, const PxVec4& max)
		{
	#ifdef ABP_SIMD_OVERLAP
			mMinY	= -min.y;
			mMinZ	= -min.z;
	#else
			mMinY	= min.y;
			mMinZ	= min.z;
	#endif
			mMaxY	= max.y;
			mMaxZ	= max.z;
		}

		PX_FORCE_INLINE	void	operator = (const SIMD_AABB_YZ4& box)
		{
			V4StoreA(V4LoadA(&box.mMinY), &mMinY);
		}

		float mMinY;
		float mMinZ;
		float mMaxY;
		float mMaxZ;
	};
#endif

#define MBP_ALLOC(x)		PX_ALLOC(x, "MBP")
#define MBP_ALLOC_TMP(x)	PX_ALLOC_TEMP(x, "MBP_TMP")
#define MBP_FREE(x)			if(x)	PX_FREE_AND_RESET(x)
#define DELETESINGLE(x)		if (x) { delete x;		x = NULL; }
#define DELETEARRAY(x)		if (x) { delete []x;	x = NULL; }

#define	INVALID_ID	0xffffffff

///////////////////////////////////////////////////////////////////////////////

#define DEFAULT_NB_ENTRIES	128

	class ABP_MM
	{
		public:
									ABP_MM();
									~ABP_MM();

			void*					frameAlloc(PxU32 size);
			void					frameFree(void* address);

			PxcScratchAllocator*	mScratchAllocator;
	};

ABP_MM::ABP_MM() :
	mScratchAllocator	(NULL)
{
}

ABP_MM::~ABP_MM()
{
}

void* ABP_MM::frameAlloc(PxU32 size)
{
	if(mScratchAllocator)
		return mScratchAllocator->alloc(size, true);
	return PX_ALLOC_TEMP(size, PX_DEBUG_EXP("frameAlloc"));
}

void ABP_MM::frameFree(void* address)
{
	if(mScratchAllocator)
		mScratchAllocator->free(address);
	else
		PX_FREE(address);
}

template<class T>
static T* resizeBoxesT(PxU32 oldNbBoxes, PxU32 newNbBoxes, const T* boxes)
{
	T* newBoxes = PX_NEW(T)[newNbBoxes];
	if(oldNbBoxes)
		PxMemCopy(newBoxes, boxes, oldNbBoxes*sizeof(T));
	DELETEARRAY(boxes);
	return newBoxes;
}

	class Boxes
	{
		public:
								Boxes();
								~Boxes();

		PX_FORCE_INLINE	void	init(const Boxes& boxes){ mSize = boxes.mSize; mCapacity = boxes.mCapacity;	}
		PX_FORCE_INLINE	PxU32	getSize()		const	{ return mSize;										}
		PX_FORCE_INLINE	PxU32	getCapacity()	const	{ return mCapacity;									}
		PX_FORCE_INLINE	bool	isFull()		const	{ return mSize==mCapacity;							}
		PX_FORCE_INLINE	void	reset()					{ mSize = mCapacity = 0;							}
		PX_FORCE_INLINE	PxU32	popBack()				{ return --mSize;									}

//		protected:
						PxU32	mSize;
						PxU32	mCapacity;
	};

Boxes::Boxes() :
	mSize		(0),
	mCapacity	(0)
{
}

Boxes::~Boxes()
{
	reset();
}

	class StraightBoxes : public Boxes
	{
		public:
												StraightBoxes();
												~StraightBoxes();

						void					init(PxU32 size, PxU32 capacity, SIMD_AABB4* boxes);
						void					reset();
						PxU32					resize();
						PxU32					resize(PxU32 incoming);
						bool					allocate(PxU32 nb);

		PX_FORCE_INLINE	const SIMD_AABB4*		getBoxes()		const	{ return mBoxes;	}
		PX_FORCE_INLINE	SIMD_AABB4*				getBoxes()				{ return mBoxes;	}

		PX_FORCE_INLINE	void					setBounds(PxU32 index, const SIMD_AABB4& box)
												{
													PX_ASSERT(index<mSize);
													mBoxes[index] = box;
												}

		PX_FORCE_INLINE	PxU32					pushBack(const SIMD_AABB4& box)
												{
													const PxU32 index = mSize++;
													setBounds(index, box);
													return index;
												}
		private:
						SIMD_AABB4*				mBoxes;
	};

StraightBoxes::StraightBoxes() :
	mBoxes		(NULL)
{
}

StraightBoxes::~StraightBoxes()
{
	reset();
}

void StraightBoxes::reset()
{
	DELETEARRAY(mBoxes);
	Boxes::reset();
}

void StraightBoxes::init(PxU32 size, PxU32 capacity, SIMD_AABB4* boxes)
{
	reset();
	mSize = size;
	mCapacity = capacity;
	mBoxes = boxes;
}

PxU32 StraightBoxes::resize()
{
	const PxU32 capacity = mCapacity;
	const PxU32 size = mSize;

//	const PxU32 newCapacity = capacity ? capacity + DEFAULT_NB_ENTRIES : DEFAULT_NB_ENTRIES;
//	const PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	const PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	// PT: we allocate one extra box for safe SIMD loads
	mBoxes = resizeBoxesT(size, newCapacity+1, mBoxes);
	mCapacity = newCapacity;
	return newCapacity;
}

PxU32 StraightBoxes::resize(PxU32 incoming)
{
	const PxU32 capacity = mCapacity;
	const PxU32 size = mSize;
	const PxU32 minCapacity = size + incoming;
	if(minCapacity<capacity)
		return capacity;

	PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	if(newCapacity<minCapacity)
		newCapacity=minCapacity;

	// PT: we allocate one extra box for safe SIMD loads
	mBoxes = resizeBoxesT(size, newCapacity+1, mBoxes);
	mCapacity = newCapacity;
	return newCapacity;
}

bool StraightBoxes::allocate(PxU32 nb)
{
	if(nb<=mSize)
		return false;
	DELETEARRAY(mBoxes);
	// PT: we allocate NB_SENTINELS more boxes than necessary here so we don't need to allocate one more for SIMD-load safety
	mBoxes = PX_NEW_TEMP(SIMD_AABB4)[nb+NB_SENTINELS];
	mSize = mCapacity = nb;
	return true;
}


	class SplitBoxes : public Boxes
	{
		public:
												SplitBoxes();
												~SplitBoxes();

						void					init(PxU32 size, PxU32 capacity, SIMD_AABB_X4* boxes_X, SIMD_AABB_YZ4* boxes_YZ);
						void					init(const SplitBoxes& boxes);
						void					reset(bool freeMemory = true);
						PxU32					resize();
						PxU32					resize(PxU32 incoming);
						bool					allocate(PxU32 nb);

		PX_FORCE_INLINE	const SIMD_AABB_X4*		getBoxes_X()	const	{ return mBoxes_X;	}
		PX_FORCE_INLINE	SIMD_AABB_X4*			getBoxes_X()			{ return mBoxes_X;	}
		PX_FORCE_INLINE	const SIMD_AABB_YZ4*	getBoxes_YZ()	const	{ return mBoxes_YZ;	}
		PX_FORCE_INLINE	SIMD_AABB_YZ4*			getBoxes_YZ()			{ return mBoxes_YZ;	}

		PX_FORCE_INLINE	void					setBounds(PxU32 index, const PxVec4& min, const PxVec4& max)
												{
													PX_ASSERT(index<mSize);
													mBoxes_X[index].initFromPxVec4(min, max);
													mBoxes_YZ[index].initFromPxVec4(min, max);
												}

		PX_FORCE_INLINE	void					setBounds(PxU32 index, const SIMD_AABB4& box)
												{
													PX_ASSERT(index<mSize);
													mBoxes_X[index].initFrom(box);
													mBoxes_YZ[index].initFrom(box);
												}

		PX_FORCE_INLINE	PxU32					pushBack(const SIMD_AABB4& box)
												{
													const PxU32 index = mSize++;
													setBounds(index, box);
													return index;
												}
		private:
						SIMD_AABB_X4*			mBoxes_X;
						SIMD_AABB_YZ4*			mBoxes_YZ;
	};

SplitBoxes::SplitBoxes() :
	mBoxes_X	(NULL),
	mBoxes_YZ	(NULL)
{
}

SplitBoxes::~SplitBoxes()
{
	reset();
}

void SplitBoxes::reset(bool freeMemory)
{
	if(freeMemory)
	{
		DELETEARRAY(mBoxes_YZ);
		DELETEARRAY(mBoxes_X);
	}
	mBoxes_X = NULL;
	mBoxes_YZ = NULL;
	Boxes::reset();
}

void SplitBoxes::init(PxU32 size, PxU32 capacity, SIMD_AABB_X4* boxes_X, SIMD_AABB_YZ4* boxes_YZ)
{
	reset();
	mSize = size;
	mCapacity = capacity;
	mBoxes_X = boxes_X;
	mBoxes_YZ = boxes_YZ;
}

void SplitBoxes::init(const SplitBoxes& boxes)
{
	reset();
	Boxes::init(boxes);
	mBoxes_X = const_cast<SIMD_AABB_X4*>(boxes.getBoxes_X());
	mBoxes_YZ = const_cast<SIMD_AABB_YZ4*>(boxes.getBoxes_YZ());
}

PxU32 SplitBoxes::resize()
{
	const PxU32 capacity = mCapacity;
	const PxU32 size = mSize;

//	const PxU32 newCapacity = capacity ? capacity + DEFAULT_NB_ENTRIES : DEFAULT_NB_ENTRIES;
//	const PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	const PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	mBoxes_X = resizeBoxesT(size, newCapacity, mBoxes_X);
	mBoxes_YZ = resizeBoxesT(size, newCapacity, mBoxes_YZ);
	mCapacity = newCapacity;
	return newCapacity;
}

PxU32 SplitBoxes::resize(PxU32 incoming)
{
	const PxU32 capacity = mCapacity;
	const PxU32 size = mSize;
	const PxU32 minCapacity = size + incoming;
	if(minCapacity<capacity)
		return capacity;

	PxU32 newCapacity = capacity ? capacity*2 : DEFAULT_NB_ENTRIES;
	if(newCapacity<minCapacity)
		newCapacity=minCapacity;

	mBoxes_X = resizeBoxesT(size, newCapacity, mBoxes_X);
	mBoxes_YZ = resizeBoxesT(size, newCapacity, mBoxes_YZ);
	mCapacity = newCapacity;
	return newCapacity;
}

bool SplitBoxes::allocate(PxU32 nb)
{
	if(nb<=mSize)
		return false;
	DELETEARRAY(mBoxes_YZ);
	DELETEARRAY(mBoxes_X);
	mBoxes_X = PX_NEW_TEMP(SIMD_AABB_X4)[nb+NB_SENTINELS];
	mBoxes_YZ = PX_NEW_TEMP(SIMD_AABB_YZ4)[nb];
	PX_ASSERT(!(reinterpret_cast<size_t>(mBoxes_YZ) & 15));
	mSize = mCapacity = nb;
	return true;
}

	typedef	SplitBoxes		StaticBoxes;
	typedef	SplitBoxes		DynamicBoxes;

///////////////////////////////////////////////////////////////////////////////

	struct ABP_Object : public Ps::UserAllocated
	{
		PX_FORCE_INLINE	ABP_Object() : mIndex(INVALID_ID)
		{
#if PX_DEBUG
		mUpdated = false;
#endif
		}

	private:
		PxU32		mIndex;			// Out-to-in, maps user handle to internal array. mIndex indexes either the static or dynamic array.
		// PT: the type won't be available for removed objects so we have to store it there. That uses 2 bits.
		// Then the "data" will need one more bit for marking sleeping objects so that leaves 28bits for the actual index.
		PX_FORCE_INLINE	void	setData(PxU32 index, FilterType::Enum type)
		{
//			mIndex = index;
			index <<= 2;
			index |= type;
			mIndex = index;
		}
	public:
		// PT: TODO: rename "index" to data everywhere
		PX_FORCE_INLINE	void	setActiveIndex(PxU32 index, FilterType::Enum type)
		{
			const PxU32 boxData = (index+index);
			setData(boxData, type);
		}

		PX_FORCE_INLINE	void	setSleepingIndex(PxU32 index, FilterType::Enum type)
		{
			const PxU32 boxData = (index+index)|1;
			PX_ASSERT(getType()==type);
			setData(boxData, type);
		}

		PX_FORCE_INLINE		FilterType::Enum	getType()	const
		{
			return FilterType::Enum(mIndex&3);
		}

		PX_FORCE_INLINE		PxU32	getData()	const
		{
			return mIndex>>2;
		}

		PX_FORCE_INLINE		void	invalidateIndex()
		{
			mIndex = INVALID_ID;
		}
		PX_FORCE_INLINE		bool	isValid()	const
		{
			return mIndex != INVALID_ID;
		}

#if PX_DEBUG
		bool		mUpdated;
#endif
	};

	typedef ABP_Object ABPEntry;

///////////////////////////////////////////////////////////////////////////////

//#define BIT_ARRAY_STACK	512

	static PX_FORCE_INLINE PxU32 bitsToDwords(PxU32 nbBits)
	{
		return (nbBits>>5) + ((nbBits&31) ? 1 : 0);
	}

	// Use that one instead of an array of bools. Takes less ram, nearly as fast [no bounds checkings and so on].
	class BitArray
	{
		public:
										BitArray();
										BitArray(PxU32 nbBits);
										~BitArray();

						bool			init(PxU32 nbBits);
						void			empty();
						void			resize(PxU32 nbBits);

		PX_FORCE_INLINE	void			checkResize(PxU32 bitNumber)
										{
											const PxU32 index = bitNumber>>5;
											if(index>=mSize)
												resize(bitNumber);
										}

		PX_FORCE_INLINE	void			setBitChecked(PxU32 bitNumber)
										{
											const PxU32 index = bitNumber>>5;
											if(index>=mSize)
												resize(bitNumber);
											mBits[index] |= 1<<(bitNumber&31);
										}

		PX_FORCE_INLINE	void			clearBitChecked(PxU32 bitNumber)
										{
											const PxU32 index = bitNumber>>5;
											if(index>=mSize)
												resize(bitNumber);
											mBits[index] &= ~(1<<(bitNumber&31));
										}
		// Data management
		PX_FORCE_INLINE	void			setBit(PxU32 bitNumber)					{ mBits[bitNumber>>5] |= 1<<(bitNumber&31);				}
		PX_FORCE_INLINE	void			clearBit(PxU32 bitNumber)				{ mBits[bitNumber>>5] &= ~(1<<(bitNumber&31));			}
		PX_FORCE_INLINE	void			toggleBit(PxU32 bitNumber)				{ mBits[bitNumber>>5] ^= 1<<(bitNumber&31);				}

		PX_FORCE_INLINE	void			clearAll()								{ PxMemZero(mBits, mSize*4);							}
		PX_FORCE_INLINE	void			setAll()								{ PxMemSet(mBits, 0xff, mSize*4);						}

		// Data access
		PX_FORCE_INLINE	Ps::IntBool		isSet(PxU32 bitNumber)			const	{ return Ps::IntBool(mBits[bitNumber>>5] & (1<<(bitNumber&31)));		}
		PX_FORCE_INLINE	Ps::IntBool		isSetChecked(PxU32 bitNumber)	const
										{
											const PxU32 index = bitNumber>>5;
											if(index>=mSize)
												return 0;
											return Ps::IntBool(mBits[index] & (1<<(bitNumber&31)));
										}

		PX_FORCE_INLINE	const PxU32*	getBits()						const	{ return mBits;											}
		PX_FORCE_INLINE	PxU32			getSize()						const	{ return mSize;											}

		// PT: replicate Cm::BitMap stuff for temp testing
						PxU32			findLast()						const
										{
											for(PxU32 i = mSize; i-- > 0;)
											{
												if(mBits[i])
													return (i<<5)+Ps::highestSetBit(mBits[i]);
											}
											return PxU32(0); 
										}
		protected:
						PxU32*			mBits;		//!< Array of bits
						PxU32			mSize;		//!< Size of the array in dwords
#ifdef BIT_ARRAY_STACK
						PxU32			mStack[BIT_ARRAY_STACK];
#endif
	};

///////////////////////////////////////////////////////////////////////////////

BitArray::BitArray() : mBits(NULL), mSize(0)
{
}

BitArray::BitArray(PxU32 nbBits) : mBits(NULL), mSize(0)
{
	init(nbBits);
}

BitArray::~BitArray()
{
	empty();
}

void BitArray::empty()
{
#ifdef BIT_ARRAY_STACK
	if(mBits!=mStack)
#endif
		MBP_FREE(mBits);
	mBits = NULL;
	mSize = 0;
}

bool BitArray::init(PxU32 nbBits)
{
	mSize = bitsToDwords(nbBits);
	// Get ram for n bits
#ifdef BIT_ARRAY_STACK
	if(mBits!=mStack)
#endif
		MBP_FREE(mBits);
#ifdef BIT_ARRAY_STACK
	if(mSize>BIT_ARRAY_STACK)
#endif
		mBits = reinterpret_cast<PxU32*>(MBP_ALLOC(sizeof(PxU32)*mSize));
#ifdef BIT_ARRAY_STACK
	else
		mBits = mStack;
#endif

	// Set all bits to 0
	clearAll();
	return true;
}

void BitArray::resize(PxU32 nbBits)
{
	const PxU32 newSize = bitsToDwords(nbBits+128);
	PxU32* newBits = NULL;
#ifdef BIT_ARRAY_STACK
	if(newSize>BIT_ARRAY_STACK)
#endif
	{
		// Old buffer was stack or allocated, new buffer is allocated
		newBits = reinterpret_cast<PxU32*>(MBP_ALLOC(sizeof(PxU32)*newSize));
		if(mSize)
			PxMemCopy(newBits, mBits, sizeof(PxU32)*mSize);
	}
#ifdef BIT_ARRAY_STACK
	else
	{
		newBits = mStack;
		if(mSize>BIT_ARRAY_STACK)
		{
			// Old buffer was allocated, new buffer is stack => copy to stack, shrink
			CopyMemory(newBits, mBits, sizeof(PxU32)*BIT_ARRAY_STACK);
		}
		else
		{
			// Old buffer was stack, new buffer is stack => keep working on the same stack buffer, nothing to do
		}
	}
#endif
	const PxU32 remain = newSize - mSize;
	if(remain)
		PxMemZero(newBits + mSize, remain*sizeof(PxU32));

#ifdef BIT_ARRAY_STACK
	if(mBits!=mStack)
#endif
		MBP_FREE(mBits);
	mBits = newBits;
	mSize = newSize;
}

///////////////////////////////////////////////////////////////////////////////

static ABP_Index* resizeMapping(PxU32 oldNbBoxes, PxU32 newNbBoxes, ABP_Index* mapping)
{
	ABP_Index* newMapping = reinterpret_cast<ABP_Index*>(MBP_ALLOC(sizeof(ABP_Index)*newNbBoxes));
	if(oldNbBoxes)
		PxMemCopy(newMapping, mapping, oldNbBoxes*sizeof(ABP_Index));
	MBP_FREE(mapping);
	return newMapping;
}

	struct ABP_Object;

	class ABP_PairManager : public PairManagerData
	{
		public:
														ABP_PairManager();
														~ABP_PairManager();

						InternalPair*					addPair						(PxU32 id0, PxU32 id1);
						void							computeCreatedDeletedPairs	(BroadPhaseABP* mbp, const BitArray& updated, const BitArray& removed);

						const Bp::FilterGroup::Enum*	mGroups;
						const ABP_Index*				mInToOut0;
						const ABP_Index*				mInToOut1;
						const ABPEntry*					mObjects;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
						const bool*						mLUT;
#endif
	};

	///////////////////////////////////////////////////////////////////////////

	struct ABP_SharedData
	{
		PX_FORCE_INLINE				ABP_SharedData() :
										mABP_Objects			(NULL),
										mABP_Objects_Capacity	(0)
									{
									}

						void		resize(BpHandle userID);

		PX_FORCE_INLINE	void		checkResize(PxU32 maxID)
									{
										if(mABP_Objects_Capacity<maxID+1)
											resize(maxID);
									}

						ABP_Object*	mABP_Objects;
						PxU32		mABP_Objects_Capacity;
						BitArray	mUpdatedObjects;	// Indexed by ABP_ObjectIndex
						BitArray	mRemovedObjects;	// Indexed by ABP_ObjectIndex
	};

void ABP_SharedData::resize(BpHandle userID)
{
	const PxU32 oldCapacity = mABP_Objects_Capacity;

	PxU32 newCapacity = mABP_Objects_Capacity ? mABP_Objects_Capacity*2 : 256;
	if(newCapacity<userID+1)
		newCapacity = userID+1;

	ABP_Object* newObjects = PX_NEW(ABP_Object)[newCapacity];
	if(mABP_Objects)
		PxMemCopy(newObjects, mABP_Objects, oldCapacity*sizeof(ABP_Object));
#if PX_DEBUG
	for(PxU32 i=oldCapacity;i<newCapacity;i++)
		newObjects[i].mUpdated = false;
#endif
	PX_DELETE_ARRAY(mABP_Objects);
	mABP_Objects = newObjects;
	mABP_Objects_Capacity = newCapacity;
}

	class BoxManager
	{
		public:
											BoxManager(FilterType::Enum type);
											~BoxManager();

						void				reset();
						void				setSourceData(const PxBounds3* bounds, const float* distances)
											{
												mAABBManagerBounds = bounds;
												mAABBManagerDistances = distances;
											}

						void				notifyStaticChange(ABP_Object* PX_RESTRICT /*objects*/){}

						void				addObjects(const BpHandle* PX_RESTRICT userIDs, PxU32 nb, ABP_SharedData* PX_RESTRICT sharedData);
						void				removeObject(ABPEntry& object, BpHandle userID);
						void				updateObject(ABPEntry& object, BpHandle userID);

						void				prepareData(RadixSortBuffered& rs, ABP_Object* PX_RESTRICT objects, PxU32 objectsCapacity, ABP_MM& memoryManager);

						void				finalize();

//		PX_FORCE_INLINE	PxU32				isThereWorkToDo()		const	{ return mNbUpdated;	}
		PX_FORCE_INLINE	bool				isThereWorkToDo()		const	{ return mNbUpdated || mNbRemovedSleeping;	}	// PT: temp & test, maybe we do that differently in the end
		PX_FORCE_INLINE	PxU32				getNbUpdatedBoxes()		const	{ return mNbUpdated;		}
		PX_FORCE_INLINE	PxU32				getNbNonUpdatedBoxes()	const	{ return mNbSleeping;		}
		PX_FORCE_INLINE	const DynamicBoxes&	getUpdatedBoxes()		const	{ return mUpdatedBoxes;		}
		PX_FORCE_INLINE	const DynamicBoxes&	getSleepingBoxes()		const	{ return mSleepingBoxes;	}
		PX_FORCE_INLINE	const ABP_Index*	getRemap_Updated()		const	{ return mInToOut_Updated;	}
		PX_FORCE_INLINE	const ABP_Index*	getRemap_Sleeping()		const	{ return mInToOut_Sleeping;	}
#ifdef USE_ABP_BUCKETS
		PX_FORCE_INLINE	const PxBounds3&	getUpdatedBounds()		const	{ return mUpdatedBounds;	}
#endif

		private:
						FilterType::Enum	mType;

						// PT: refs to source data (not owned). Currently separate arrays, ideally should be merged.
						const PxBounds3*	mAABBManagerBounds;
						const float*		mAABBManagerDistances;

						// New & updated objects
#ifdef USE_ABP_BUCKETS
						PxBounds3			mUpdatedBounds;		// Bounds around updated dynamic objects, computed in prepareData().
#endif
						ABP_Index*			mInToOut_Updated;	// Maps boxes to mABP_Objects
						PxU32				mNbUpdated;
						PxU32				mMaxNbUpdated;
						DynamicBoxes		mUpdatedBoxes;

						// Sleeping objects
						ABP_Index*			mInToOut_Sleeping;	// Maps boxes to mABP_Objects
						PxU32				mNbSleeping;
						DynamicBoxes		mSleepingBoxes;

						// Removed sleeping
						PxU32				mNbRemovedSleeping;

						void				purgeRemovedFromSleeping(ABP_Object* PX_RESTRICT objects, PxU32 objectsCapacity);
	};

BoxManager::BoxManager(FilterType::Enum type) :
	mType					(type),
	mAABBManagerBounds		(NULL),
	mAABBManagerDistances	(NULL),
	mInToOut_Updated		(NULL),
	mNbUpdated				(0),
	mMaxNbUpdated			(0),
	mInToOut_Sleeping		(NULL),
	mNbSleeping				(0),
	mNbRemovedSleeping		(0)
{
}

BoxManager::~BoxManager()
{
	reset();
}

void BoxManager::reset()
{
	mMaxNbUpdated = mNbUpdated = mNbSleeping = 0;
	PX_FREE_AND_RESET(mInToOut_Updated);
	PX_FREE_AND_RESET(mInToOut_Sleeping);
	mUpdatedBoxes.reset();
	mSleepingBoxes.reset();
}

static PX_FORCE_INLINE PxU32 isNewOrUpdated(PxU32 data)
{
	return data & PX_SIGN_BITMASK;
}

static PX_FORCE_INLINE PxU32 markAsNewOrUpdated(PxU32 data)
{
	return data | PX_SIGN_BITMASK;
}

static PX_FORCE_INLINE PxU32 removeNewOrUpdatedMark(PxU32 data)
{
	return data & ~PX_SIGN_BITMASK;
}

// BpHandle = index in main/shared arrays like mAABBManagerBounds / mAABBManagerDistances
PX_COMPILE_TIME_ASSERT(sizeof(BpHandle)==sizeof(ABP_Index));
void BoxManager::addObjects(const BpHandle* PX_RESTRICT userIDs, PxU32 nb, ABP_SharedData* PX_RESTRICT sharedData)
{
	// PT: we're called for each batch.

	// PT: TODO: fix the BpHandle/ABP_Index mix
	const PxU32 currentSize = mNbUpdated;
	const PxU32 currentCapacity = mMaxNbUpdated;
	const PxU32 newSize = currentSize + nb;

	ABP_Index* remap;
	if(newSize>currentCapacity)
	{
		const PxU32 minCapacity = PxMax(newSize, 1024u);
		const PxU32 newCapacity = PxMax(minCapacity, currentCapacity*2);
		PX_ASSERT(newCapacity>=newSize);
		mMaxNbUpdated = newCapacity;
		remap = resizeMapping(currentSize, newCapacity, mInToOut_Updated);
	}
	else
	{
		remap = mInToOut_Updated;
	}
	mInToOut_Updated = remap;
	mNbUpdated = newSize;

	// PT: we only copy the new handles for now. The bounds will be computed later in "prepareData".
	// PT: TODO: do we even need to copy them? Can't we just reuse the source ptr directly?
	{
		PX_ASSERT(currentSize+nb<=mMaxNbUpdated);
		remap += currentSize;
		PxU32 nbToGo = nb;
		while(nbToGo--)
		{
			const BpHandle userID = *userIDs++;

			PX_ASSERT(!isNewOrUpdated(userID));
			*remap++ = markAsNewOrUpdated(userID);

			if(sharedData)
				sharedData->mUpdatedObjects.setBit(userID);
		}
	}
}

// PT: TODO: inline this again
void BoxManager::removeObject(ABPEntry& object, BpHandle userID)
{
	PX_UNUSED(userID);
	const PxU32 boxData = object.getData();
	const PxU32 boxIndex = boxData>>1;
	if(boxData&1)
	{
		// Sleeping object.
		PX_ASSERT(boxIndex<mNbSleeping);
		PX_ASSERT(mInToOut_Sleeping[boxIndex]==userID);
		PX_ASSERT(mInToOut_Sleeping[boxIndex] != INVALID_ID);	// PT: can that happen if we update and remove an object in the same frame or does the AABB take care of it?
		mInToOut_Sleeping[boxIndex] = INVALID_ID;
		mNbRemovedSleeping++;
		PX_ASSERT(mNbRemovedSleeping<=mNbSleeping);
	}
	else
	{
		// PT: remove active object, i.e. one that was previously in "updated" arrays. 
		PX_ASSERT(boxIndex<mNbUpdated);
		PX_ASSERT(boxIndex<mMaxNbUpdated);
		PX_ASSERT(mInToOut_Updated[boxIndex]==userID);
		PX_ASSERT(mInToOut_Updated[boxIndex] != INVALID_ID);
		// PT: TODO: do we need this at all? We could use 'userID' to access the removed bitmap...
		mInToOut_Updated[boxIndex] = INVALID_ID;
	}
}

// PT: TODO: inline this again
void BoxManager::updateObject(ABPEntry& object, BpHandle userID)
{
	PX_UNUSED(userID);
	const PxU32 boxData = object.getData();
	const PxU32 boxIndex = boxData>>1;
	if(boxData&1)
	{
		// PT: benchmark for this codepath: MBP.UpdateSleeping

		// Sleeping object. We must reactivate it, i.e:
		// - remove it from the array of sleeping objects
		// - add it to the array of active/updated objects

		// First we remove:
		{
			PX_ASSERT(boxIndex<mNbSleeping);
			PX_ASSERT(mInToOut_Sleeping[boxIndex]==userID);
			PX_ASSERT(mInToOut_Sleeping[boxIndex] != INVALID_ID);
			mInToOut_Sleeping[boxIndex] = INVALID_ID;
			mNbRemovedSleeping++;
			PX_ASSERT(mNbRemovedSleeping<=mNbSleeping);
		}

		// Then we add
		// PT: TODO: revisit / improve this maybe
		addObjects(&userID, 1, NULL);	// Don't pass sharedData because the bitmap has already been updated by the calling code
	}
	else
	{
		// Active object, i.e. it was updated in previous frame and it's already in mInToOut_Updated array
		PX_ASSERT(boxIndex<mNbUpdated);
		PX_ASSERT(boxIndex<mMaxNbUpdated);
		PX_ASSERT(mInToOut_Updated[boxIndex]==userID);
		mInToOut_Updated[boxIndex] = markAsNewOrUpdated(mInToOut_Updated[boxIndex]);
	}
}


#if PX_DEBUG
static PX_FORCE_INLINE void computeMBPBounds_Check(SIMD_AABB4& aabb, const PxBounds3* PX_RESTRICT boundsXYZ, const PxReal* PX_RESTRICT contactDistances, const BpHandle index)
{
	const PxBounds3& b = boundsXYZ[index];
	const Vec4V contactDistanceV = V4Load(contactDistances[index]);
	const Vec4V inflatedMinV = V4Sub(V4LoadU(&b.minimum.x), contactDistanceV);
	const Vec4V inflatedMaxV = V4Add(V4LoadU(&b.maximum.x), contactDistanceV);	// PT: this one is safe because we allocated one more box in the array (in BoundsArray::initEntry)

	PX_ALIGN(16, PxVec4) boxMin;
	PX_ALIGN(16, PxVec4) boxMax;
	V4StoreA(inflatedMinV, &boxMin.x);
	V4StoreA(inflatedMaxV, &boxMax.x);

	aabb.mMinX = boxMin[0];
	aabb.mMinY = boxMin[1];
	aabb.mMinZ = boxMin[2];
	aabb.mMaxX = boxMax[0];
	aabb.mMaxY = boxMax[1];
	aabb.mMaxZ = boxMax[2];
}
#endif

static PX_FORCE_INLINE void initSentinels(SIMD_AABB_X4* PX_RESTRICT boxesX, const PxU32 size)
{
	for(PxU32 i=0;i<NB_SENTINELS;i++)
		boxesX[size+i].initSentinel();
}

void BoxManager::purgeRemovedFromSleeping(ABP_Object* PX_RESTRICT objects, PxU32 objectsCapacity)
{
	CHECKPOINT("purgeRemovedFromSleeping\n");

	PX_UNUSED(objectsCapacity);
	PX_ASSERT(mNbRemovedSleeping);
	PX_ASSERT(mNbSleeping);

	// PT: TODO: do we need to allocate separate buffers here?

	// PT: we reach this codepath when:
	// - no object has been added or updated
	// - sleeping objects have been removed
	// So we have to purge the removed objects from the sleeping array. We cannot entirely ignore the removals since we compute collisions
	// between sleeping arrays and active arrays for bipartite cases. So we either have to remove the invalid entries immediately, or make
	// sure they don't report collisions. We could ignore collisions when the remapped ID is "INVALID_ID" but that would be an additional
	// test for each potential pair, i.e. it's a constant cost. We cannot tweak the removed bounding boxes (e.g. mark them as empty) because
	// they are sorted, and the tweak would break the sorting and the collision loop. Keeping all removed objects in the array also means
	// there is more data to parse all the time, i.e. there is a performance cost again. So for now we just remove all deleted entries here.
	// ==> also tweaking the sleeping boxes might break the "merge sleeping" array code
	PX_ASSERT(mNbRemovedSleeping<=mNbSleeping);

	if(mNbRemovedSleeping==mNbSleeping)
	{
		// PT: remove everything
		mSleepingBoxes.reset();
		PX_FREE_AND_RESET(mInToOut_Sleeping);
		mNbSleeping = mNbRemovedSleeping = 0;
		return;
	}

	const PxU32 expectedTotal = mNbSleeping - mNbRemovedSleeping;
	PxU32 nbRemovedFound = 0;
	PxU32 nbSleepingLeft = 0;
	const PxU32 sleepCapacity = mSleepingBoxes.getCapacity();
	if(expectedTotal>=sleepCapacity/2)
	{	
		// PT: remove holes, keep same data buffers
		SIMD_AABB_X4* boxesX = mSleepingBoxes.getBoxes_X();
		SIMD_AABB_YZ4* boxesYZ = mSleepingBoxes.getBoxes_YZ();
		ABP_Index* remap = mInToOut_Sleeping;

		for(PxU32 i=0;i<mNbSleeping;i++)
		{
			const PxU32 boxIndex = remap[i];
			if(boxIndex==INVALID_ID)
			{
				nbRemovedFound++;
			}
			else
			{
				PX_ASSERT(nbSleepingLeft<expectedTotal);
				if(i!=nbSleepingLeft)
				{
					remap[nbSleepingLeft] = boxIndex;
					boxesX[nbSleepingLeft] = boxesX[i];
					boxesYZ[nbSleepingLeft] = boxesYZ[i];
				}

				{
					PX_ASSERT(boxIndex<objectsCapacity);
					objects[boxIndex].setSleepingIndex(nbSleepingLeft, mType);
				}
				nbSleepingLeft++;
			}
		}
		PX_ASSERT(nbSleepingLeft==expectedTotal);
		PX_ASSERT(nbSleepingLeft+nbRemovedFound==mNbSleeping);

		initSentinels(boxesX, expectedTotal);

		mSleepingBoxes.mSize = expectedTotal;
	}
	else
	{
		// PT: remove holes, get fresh memory buffers
		SIMD_AABB_X4* dstBoxesX = PX_NEW(SIMD_AABB_X4)[expectedTotal+NB_SENTINELS];
		SIMD_AABB_YZ4* dstBoxesYZ = PX_NEW(SIMD_AABB_YZ4)[expectedTotal+NB_SENTINELS];
		initSentinels(dstBoxesX, expectedTotal);
		BpHandle* PX_RESTRICT dstRemap = reinterpret_cast<BpHandle*>(PX_ALLOC(expectedTotal*sizeof(BpHandle), PX_DEBUG_EXP("tmp")));

		const SIMD_AABB_X4* PX_RESTRICT srcDataX = mSleepingBoxes.getBoxes_X();
		const SIMD_AABB_YZ4* PX_RESTRICT srcDataYZ = mSleepingBoxes.getBoxes_YZ();
		const ABP_Index* PX_RESTRICT srcRemap = mInToOut_Sleeping;

		for(PxU32 i=0;i<mNbSleeping;i++)
		{
			const PxU32 boxIndex = srcRemap[i];
			if(boxIndex==INVALID_ID)
			{
				nbRemovedFound++;
			}
			else
			{
				PX_ASSERT(nbSleepingLeft<expectedTotal);
				dstRemap[nbSleepingLeft] = boxIndex;
				dstBoxesX[nbSleepingLeft] = srcDataX[i];
				dstBoxesYZ[nbSleepingLeft] = srcDataYZ[i];

				{
					PX_ASSERT(boxIndex<objectsCapacity);
					objects[boxIndex].setSleepingIndex(nbSleepingLeft, mType);
				}
				nbSleepingLeft++;
			}
		}
		PX_ASSERT(nbSleepingLeft==expectedTotal);
		PX_ASSERT(nbSleepingLeft+nbRemovedFound==mNbSleeping);

		// PT: TODO: double check all this
		mSleepingBoxes.init(expectedTotal, expectedTotal, dstBoxesX, dstBoxesYZ);

		PX_FREE(mInToOut_Sleeping);
		mInToOut_Sleeping = dstRemap;
	}
	mNbSleeping = expectedTotal;
	mNbRemovedSleeping = 0;
}

static PX_FORCE_INLINE PosXType2 getNextCandidateSorted(PxU32 offsetSorted, const PxU32 nbSorted, const SIMD_AABB_X4* PX_RESTRICT sortedDataX, const PxU32* PX_RESTRICT sleepingIndices)
{
	return offsetSorted<nbSorted ? sortedDataX[sleepingIndices[offsetSorted]].mMinX : SentinelValue2;
}
static PX_FORCE_INLINE PosXType2 getNextCandidateNonSorted(PxU32 offsetNonSorted, const PxU32 nbToSort, const SIMD_AABB_X4* PX_RESTRICT toSortDataX)
{
	return offsetNonSorted<nbToSort ? toSortDataX[offsetNonSorted].mMinX : SentinelValue2;
}

PX_COMPILE_TIME_ASSERT(sizeof(BpHandle)==sizeof(float));
void BoxManager::prepareData(RadixSortBuffered& /*rs*/, ABP_Object* PX_RESTRICT objects, PxU32 objectsCapacity, ABP_MM& memoryManager)
{
	PX_UNUSED(objectsCapacity);

	// PT: mNbUpdated = number of objects in the updated buffer, could have been updated this frame or previous frame
	const PxU32 size = mNbUpdated;
	if(!size)
	{
		if(mNbRemovedSleeping)
		{
			// PT: benchmark for this codepath: MBP.RemoveHalfSleeping
			purgeRemovedFromSleeping(objects, objectsCapacity);
		}
		return;
	}

	PX_ASSERT(mAABBManagerBounds);
	PX_ASSERT(mAABBManagerDistances);
	PX_ASSERT(mInToOut_Updated);

	// Prepare new/updated objects
	const ABP_Index* PX_RESTRICT remap = mInToOut_Updated;
	const PxBounds3* PX_RESTRICT bounds = mAABBManagerBounds;
	const float* PX_RESTRICT distances = mAABBManagerDistances;

	float* PX_RESTRICT keys = NULL;

	// newOrUpdatedIDs: *userIDs* of objects that have been added or updated this frame.
	// sleepingIndices: *indices* (not userIDs) of non-updated objects within mInToOut_Updated
	PxU32* tempBuffer = NULL;
	PxU32* newOrUpdatedIDs = tempBuffer;
	PxU32* sleepingIndices = tempBuffer;

	// PT: mNbUpdated / mInToOut_Updated contains:
	// 1) objects added this frame (from addObject(s))
	// 2) objects updated this frame (from updateObject(s))
	// 3) objects updated the frame before, not updated this frame, i.e. they are now "sleeping"
	// 4) objects updated the frame before, then removed (from removeObject(s))
	//
	// We split the current array into separate groups:
	// - 1) & 2) go to "temp", count is "nbUpdated"
	// - 3) go to "temp2", count is "nbSleeping"
	// - 4) are filtered out. No special processing is needed because the updated data is always parsed/recreated here anyway.
	//   So if we don't actively add removed objects to the new buffers, they get removed as a side-effect.

	PxU32 nbUpdated = 0;	// PT: number of objects updated this frame
	PxU32 nbSleeping = 0;
	PxU32 nbRemoved = 0;	// PT: number of removed objects that were previously located in the udpated array

	// PT: TODO: could we do the work within mInToOut_Updated?
	// - updated objects have invalidated bounds so we don't need to preserve their order
	// - we need to preserve the order of sleeping objects to avoid re-sorting them
	// - we cannot use MTF since it breaks the order
	// - parse backward and move sleeping objects to the back? but then we might have to move the sleeping boxes at the same time

	for(PxU32 i=0;i<size;i++)
	{
		PX_ASSERT(i<mMaxNbUpdated);
		const PxU32 index = remap[i];
		if(index==INVALID_ID)
		{
			nbRemoved++;
		}
		else if(isNewOrUpdated(index))
		{
			// PT: new or updated object
			if(!keys)
				keys = reinterpret_cast<float*>(PX_ALLOC(size*sizeof(float), PX_DEBUG_EXP("tmp")));

			// PT: in this version we compute the key on-the-fly, i.e. it will be computed twice overall. We could make this
			// faster by merging bounds and distances inside the AABB manager.
			const BpHandle userID = removeNewOrUpdatedMark(index);

			keys[nbUpdated] = bounds[userID].minimum.x - distances[userID];

			if(!tempBuffer)
			{
				tempBuffer = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(size*sizeof(PxU32)));
				newOrUpdatedIDs = tempBuffer;
				sleepingIndices = tempBuffer;
			}

			newOrUpdatedIDs[size - 1 - nbUpdated] = userID;
#if PX_DEBUG
			SIMD_AABB4 aabb;
			computeMBPBounds_Check(aabb, bounds, distances, userID);
			PX_ASSERT(aabb.mMinX==keys[nbUpdated]);
#endif
			nbUpdated++;
		}
		else
		{
			if(!tempBuffer)
			{
				tempBuffer = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(size*sizeof(PxU32)));
				newOrUpdatedIDs = tempBuffer;
				sleepingIndices = tempBuffer;
			}

			// PT: sleeping object
			sleepingIndices[nbSleeping++] = i;
		}
	}
	PX_ASSERT(nbRemoved + nbUpdated + nbSleeping == size);

	// PT: we must process the sleeping objects first, because the bounds of new sleeping objects are located in the existing updated buffers.
	// PT: TODO: *HOWEVER* we could sort things right now and then reuse the "keys" buffer?

	if(nbSleeping)
	{
		// PT: must merge these guys to current sleeping array
		// They should already be in sorted order and we should already have the boxes.
#if PX_DEBUG
		const SIMD_AABB_YZ4* boxesYZ = mUpdatedBoxes.getBoxes_YZ();
		float prevKey = -FLT_MAX;
		for(PxU32 ii=0;ii<nbSleeping;ii++)
		{
			const PxU32 i = sleepingIndices[ii];	// PT: TODO: remove this indirection
			const PxU32 index = remap[i];
			PX_ASSERT(index!=INVALID_ID);
			PX_ASSERT(!(index & PX_SIGN_BITMASK));
			const BpHandle userID = index;

			const float key = bounds[userID].minimum.x - distances[userID];
			PX_ASSERT(key>=prevKey);
			prevKey = key;

			SIMD_AABB4 aabb;
			computeMBPBounds_Check(aabb, bounds, distances, userID);
			PX_ASSERT(aabb.mMinX==key);

	#ifdef ABP_SIMD_OVERLAP
			PX_ASSERT(boxesYZ[i].mMinY==-aabb.mMinY);
			PX_ASSERT(boxesYZ[i].mMinZ==-aabb.mMinZ);
	#else
			PX_ASSERT(boxesYZ[i].mMinY==aabb.mMinY);
			PX_ASSERT(boxesYZ[i].mMinZ==aabb.mMinZ);
	#endif
			PX_ASSERT(boxesYZ[i].mMaxY==aabb.mMaxY);
			PX_ASSERT(boxesYZ[i].mMaxZ==aabb.mMaxZ);
		}
#endif

		if(mNbSleeping)
		{
			// PT: benchmark for this codepath: MBP.MergeSleeping
			CHECKPOINT("Merging sleeping objects\n");

			// PT: here, we need to merge two arrays of sleeping objects together:
			// - the ones already contained inside mSleepingBoxes
			// - the new sleeping objects currently contained in mUpdatedBoxes
			// Both of them should already be sorted.
			// PT: TODO: super subtle stuff going on there, to revisit

			// PT: TODO: revisit names
			PxU32 offsetSorted = 0;
			const PxU32 nbSorted = nbSleeping;
			const SIMD_AABB_X4* PX_RESTRICT sortedDataX = mUpdatedBoxes.getBoxes_X();
			const SIMD_AABB_YZ4* PX_RESTRICT sortedDataYZ = mUpdatedBoxes.getBoxes_YZ();
			const ABP_Index* PX_RESTRICT sortedRemap = mInToOut_Updated;

			PxU32 offsetNonSorted = 0;
			const PxU32 nbToSort = mNbSleeping;
			const SIMD_AABB_X4* PX_RESTRICT toSortDataX = mSleepingBoxes.getBoxes_X();
			const SIMD_AABB_YZ4* PX_RESTRICT toSortDataYZ = mSleepingBoxes.getBoxes_YZ();
			const ABP_Index* PX_RESTRICT toSortRemap = mInToOut_Sleeping;

			PX_ASSERT(mNbRemovedSleeping<=mNbSleeping);
#if PX_DEBUG
			{
				PxU32 nbRemovedFound=0;
				for(PxU32 i=0;i<mNbSleeping;i++)
				{
					if(toSortRemap[i]==INVALID_ID)
						nbRemovedFound++;
				}
				PX_ASSERT(nbRemovedFound==mNbRemovedSleeping);
			}
#endif

			PosXType2 nextCandidateNonSorted = getNextCandidateNonSorted(offsetNonSorted, nbToSort, toSortDataX);

			PosXType2 nextCandidateSorted = getNextCandidateSorted(offsetSorted, nbSorted, sortedDataX, sleepingIndices);

			const PxU32 nbTotal = nbSorted + nbToSort - mNbRemovedSleeping;

			SIMD_AABB_X4* dstBoxesX = PX_NEW(SIMD_AABB_X4)[nbTotal+NB_SENTINELS];
			SIMD_AABB_YZ4* dstBoxesYZ = PX_NEW(SIMD_AABB_YZ4)[nbTotal+NB_SENTINELS];
			initSentinels(dstBoxesX, nbTotal);
			BpHandle* PX_RESTRICT dstRemap = reinterpret_cast<BpHandle*>(PX_ALLOC(nbTotal*sizeof(BpHandle), PX_DEBUG_EXP("tmp")));

			PxU32 i=0;
			PxU32 nbRemovedFound=0;
			PxU32 nbToGo = nbSorted + nbToSort;
			while(nbToGo--)
			{
				PxU32 boxIndex;
				{
					if(nextCandidateNonSorted<nextCandidateSorted)
					{
						boxIndex = toSortRemap[offsetNonSorted];
						if(boxIndex!=INVALID_ID)
						{
							dstRemap[i] = boxIndex;
							dstBoxesX[i] = toSortDataX[offsetNonSorted];
							dstBoxesYZ[i] = toSortDataYZ[offsetNonSorted];
						}
						else
							nbRemovedFound++;

						offsetNonSorted++;

						nextCandidateNonSorted = getNextCandidateNonSorted(offsetNonSorted, nbToSort, toSortDataX);
					}
					else
					{
						const PxU32 j = sleepingIndices[offsetSorted];
						PX_ASSERT(j<size);
						boxIndex = sortedRemap[j];
						PX_ASSERT(boxIndex!=INVALID_ID);
							dstRemap[i] = boxIndex;
							dstBoxesX[i] = sortedDataX[j];
							dstBoxesYZ[i] = sortedDataYZ[j];

						offsetSorted++;

						nextCandidateSorted = getNextCandidateSorted(offsetSorted, nbSorted, sortedDataX, sleepingIndices);
					}
				}

				if(boxIndex!=INVALID_ID)
				{
					PX_ASSERT(boxIndex<objectsCapacity);
					objects[boxIndex].setSleepingIndex(i, mType);
					i++;
				}
			}
			PX_ASSERT(i==nbTotal);
			PX_ASSERT(offsetSorted+offsetNonSorted==nbSorted+nbToSort);

#if PX_DEBUG
			{
				PosXType2 prevSorted = dstBoxesX[0].mMinX;
				for(PxU32 i2=1;i2<nbTotal;i2++)
				{
					PosXType2 v = dstBoxesX[i2].mMinX;
					PX_ASSERT(prevSorted<=v);
					prevSorted = v;
				}
			}
#endif
			// PT: TODO: double check all this
			mSleepingBoxes.init(nbTotal, nbTotal, dstBoxesX, dstBoxesYZ);

			PX_FREE(mInToOut_Sleeping);
			mInToOut_Sleeping = dstRemap;
			mNbSleeping = nbTotal;
			mNbRemovedSleeping = 0;
		}
		else
		{
			// PT: benchmark for this codepath: MBP.ActiveToSleeping
			CHECKPOINT("Active objects become sleeping objects\n");

			// PT: TODO: optimize allocs
			BpHandle* inToOut_Sleeping;
			if(mSleepingBoxes.allocate(nbSleeping))
			{
				inToOut_Sleeping = reinterpret_cast<BpHandle*>(PX_ALLOC(nbSleeping*sizeof(BpHandle), PX_DEBUG_EXP("tmp")));
				PX_FREE(mInToOut_Sleeping);
				mInToOut_Sleeping = inToOut_Sleeping;
			}
			else
			{
				inToOut_Sleeping = mInToOut_Sleeping;
			}

			const SIMD_AABB_X4* srcBoxesX = mUpdatedBoxes.getBoxes_X();
			const SIMD_AABB_YZ4* srcBoxesYZ = mUpdatedBoxes.getBoxes_YZ();
			SIMD_AABB_X4* dstBoxesX = mSleepingBoxes.getBoxes_X();
			SIMD_AABB_YZ4* dstBoxesYZ = mSleepingBoxes.getBoxes_YZ();
			initSentinels(dstBoxesX, nbSleeping);
			for(PxU32 ii=0;ii<nbSleeping;ii++)
			{
				const PxU32 i = sleepingIndices[ii];	// PT: TODO: remove this indirection
				const PxU32 index = remap[i];
				PX_ASSERT(index!=INVALID_ID);
				inToOut_Sleeping[ii] = index;
				dstBoxesX[ii] = srcBoxesX[i];
				dstBoxesYZ[ii] = srcBoxesYZ[i];
				{
					PX_ASSERT(index<objectsCapacity);
					objects[index].setSleepingIndex(ii, mType);
				}
			}
			mNbSleeping = nbSleeping;
		}
	}
	else
	{
		// PT: no sleeping objects in updated buffer
		if(mNbSleeping)
		{
			if(mNbRemovedSleeping)
			{
				// PT: benchmark for this codepath: MBP.UpdateSleeping
				purgeRemovedFromSleeping(objects, objectsCapacity);
			}
		}
		else
		{
			PX_ASSERT(!mNbRemovedSleeping);
		}
	}

	if(nbUpdated)
	{
		// PT: benchmark for this codepath: MBP.Update64KObjects
		CHECKPOINT("Create updated objects\n");

		// PT: we need to sort here because we reuse the "keys" buffer just afterwards
		PxU32* ranks0 = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(sizeof(PxU32)*nbUpdated));
		PxU32* ranks1 = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(sizeof(PxU32)*nbUpdated));
		StackRadixSort(rs, ranks0, ranks1);
		const PxU32* sorted = rs.Sort(keys, nbUpdated).GetRanks();

		// PT:
		// - shuffle the remap table, store it in sorted order (we can probably use the "recyclable" array here again)
		// - compute bounds on-the-fly, store them in sorted order

		// PT: TODO: the "keys" array can be much bigger than stricly necessary here
		BpHandle* inToOut_Updated_Sorted;
		if(mUpdatedBoxes.allocate(nbUpdated))
		{
			inToOut_Updated_Sorted = reinterpret_cast<BpHandle*>(keys);
			PX_FREE(mInToOut_Updated);
			mInToOut_Updated = inToOut_Updated_Sorted;
		}
		else
		{
			PX_FREE(keys);
			inToOut_Updated_Sorted = mInToOut_Updated;
		}
		SIMD_AABB_X4* PX_RESTRICT dstBoxesX = mUpdatedBoxes.getBoxes_X();
		initSentinels(dstBoxesX, nbUpdated);

#ifdef USE_ABP_BUCKETS
		Vec4V minV = V4Load(FLT_MAX);
		Vec4V maxV = V4Load(-FLT_MAX);
#endif
		for(PxU32 i=0;i<nbUpdated;i++)
		{
			const PxU32 sortedIndex = *sorted++;

			const BpHandle userID = newOrUpdatedIDs[size - 1 - sortedIndex];
			PX_ASSERT(i<size);
			inToOut_Updated_Sorted[i] = userID;

			{
				PX_ASSERT(userID<objectsCapacity);
				objects[userID].setActiveIndex(i, mType);
#if PX_DEBUG
				objects[userID].mUpdated = false;
#endif
			}
	
			// PT: TODO: refactor with computeMBPBounds?
			{
				const PxBounds3& b = bounds[userID];
				const Vec4V contactDistanceV = V4Load(distances[userID]);
				const Vec4V inflatedMinV = V4Sub(V4LoadU(&b.minimum.x), contactDistanceV);
				const Vec4V inflatedMaxV = V4Add(V4LoadU(&b.maximum.x), contactDistanceV);	// PT: this one is safe because we allocated one more box in the array (in BoundsArray::initEntry)
#ifdef USE_ABP_BUCKETS
				minV = V4Min(minV, inflatedMinV);
				maxV = V4Max(maxV, inflatedMaxV);
#endif
				// PT: TODO better
				PX_ALIGN(16, PxVec4) boxMin;
				PX_ALIGN(16, PxVec4) boxMax;
				V4StoreA(inflatedMinV, &boxMin.x);
				V4StoreA(inflatedMaxV, &boxMax.x);

				mUpdatedBoxes.setBounds(i, boxMin, boxMax);
			}
		}
#ifdef USE_ABP_BUCKETS
		PX_ALIGN(16, PxVec4) mergedMin;
		PX_ALIGN(16, PxVec4) mergedMax;
		V4StoreA(minV, &mergedMin.x);
		V4StoreA(maxV, &mergedMax.x);
		mUpdatedBounds.minimum = PxVec3(mergedMin.x, mergedMin.y, mergedMin.z);
		mUpdatedBounds.maximum = PxVec3(mergedMax.x, mergedMax.y, mergedMax.z);
#endif
		memoryManager.frameFree(ranks1);
		memoryManager.frameFree(ranks0);
	}
	else
	{
		// PT: benchmark for this codepath: MBP.MergeSleeping / MBP.Remove64KObjects
		CHECKPOINT("Free updated objects\n");

		if(keys)
			PX_FREE(keys);
		mUpdatedBoxes.reset();
		if(mInToOut_Updated)
			PX_FREE_AND_RESET(mInToOut_Updated);
	}
	mNbUpdated = mMaxNbUpdated = nbUpdated;

	if(tempBuffer)
		memoryManager.frameFree(tempBuffer);
}

void BoxManager::finalize()
{
}

	typedef	BoxManager			DynamicManager;
	typedef	BoxManager			StaticManager;

	class ABP : public Ps::UserAllocated
	{
												PX_NOCOPY(ABP)
		public:
												ABP();
												~ABP();

						void					preallocate(PxU32 nbObjects, PxU32 maxNbOverlaps);
						void					reset();
						void					freeBuffers();

						void					addStaticObjects(const BpHandle* userIDs, PxU32 nb, PxU32 maxID);
						void					addDynamicObjects(const BpHandle* userIDs, PxU32 nb, PxU32 maxID);
						void					addKinematicObjects(const BpHandle* userIDs, PxU32 nb, PxU32 maxID);
						void					removeObject(BpHandle userID);
						void					updateObject(BpHandle userID);
						void					findOverlaps(const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
							);
						PxU32					finalize(BroadPhaseABP* mbp);
						void					shiftOrigin(const PxVec3& shift);

						void					setTransientData(const PxBounds3* bounds, const PxReal* contactDistance/*, const Bp::FilterGroup::Enum* groups*/);

						void					Region_prepareOverlaps();
						void					Region_findOverlaps(ABP_PairManager& pairManager);

						ABP_MM					mMM;
						BoxManager				mSBM;
						DynamicManager			mDBM;
						RadixSortBuffered		mRS;
						DynamicManager			mKBM;
						ABP_SharedData			mShared;
						ABP_PairManager			mPairManager;

				const	PxBounds3*				mTransientBounds;
				const	PxReal*					mTransientContactDistance;
//				const	Bp::FilterGroup::Enum*	mTransientGroups;
	};

#ifdef ABP_SIMD_OVERLAP
	#define ABP_OVERLAP_TEST(x)	SIMD_OVERLAP_TEST(x)
#else
	#define ABP_OVERLAP_TEST(x)	if(intersect2D(box0, x))
#endif

///////////////////////////////////////////////////////////////////////////////

ABP_PairManager::ABP_PairManager() :
	mGroups			(NULL),
	mInToOut0		(NULL),
	mInToOut1		(NULL),
	mObjects		(NULL)
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	,mLUT			(NULL)
#endif
{
}

///////////////////////////////////////////////////////////////////////////////

ABP_PairManager::~ABP_PairManager()
{
}

///////////////////////////////////////////////////////////////////////////////

InternalPair* ABP_PairManager::addPair(PxU32 index0, PxU32 index1)
{
	PxU32 id0 = mInToOut0[index0];
	PxU32 id1 = mInToOut1[index1];
	PX_ASSERT(id0!=id1);
	PX_ASSERT(id0!=INVALID_ID);
	PX_ASSERT(id1!=INVALID_ID);
	PX_ASSERT(mGroups);

	{
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		if(!groupFiltering(mGroups[id0], mGroups[id1], mLUT))
#else
		if(!groupFiltering(mGroups[id0], mGroups[id1]))
#endif
			return NULL;
	}

	return addPairInternal(id0, id1);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_INTEL_FAMILY
	#define SIMD_OVERLAP_TEST_14a(box)	_mm_movemask_ps(_mm_cmpngt_ps(b, _mm_load_ps(box)))==15

	#define SIMD_OVERLAP_INIT_9c(box)	\
			__m128 b = _mm_shuffle_ps(_mm_load_ps(&box.mMinY), _mm_load_ps(&box.mMinY), 78);\
			const float Coeff = -1.0f;\
			b = _mm_mul_ps(b, _mm_load1_ps(&Coeff));

	#define SIMD_OVERLAP_TEST_9c(box)					\
			const __m128 a = _mm_load_ps(&box.mMinY);	\
			const __m128 d = _mm_cmpge_ps(a, b);		\
			if(_mm_movemask_ps(d)==15)
#else
	#define SIMD_OVERLAP_TEST_14a(box)	BAllEqFFFF(V4IsGrtr(b, V4LoadA(box)))

	#define SIMD_OVERLAP_INIT_9c(box)				\
			Vec4V b = V4PermZWXY(V4LoadA(&box.mMinY));	\
			b = V4Mul(b, V4Load(-1.0f));

	#define SIMD_OVERLAP_TEST_9c(box)				\
			const Vec4V a = V4LoadA(&box.mMinY);	\
			const Vec4V d = V4IsGrtrOrEq(a, b);		\
			if(BAllEqTTTT(d))
#endif

#ifdef ABP_SIMD_OVERLAP
	#define SIMD_OVERLAP_PRELOAD_BOX0	SIMD_OVERLAP_INIT_9c(box0)
	#define SIMD_OVERLAP_TEST(x)		SIMD_OVERLAP_TEST_9c(x)
#else
	#define SIMD_OVERLAP_PRELOAD_BOX0
#endif

#ifndef ABP_SIMD_OVERLAP
static PX_FORCE_INLINE int intersect2D(const SIMD_AABB_YZ4& a, const SIMD_AABB_YZ4& b)
{
/*	if(
		b.mMaxY < a.mMinY || a.mMaxY < b.mMinY
	||
		b.mMaxZ < a.mMinZ || a.mMaxZ < b.mMinZ
	)
		return 0;
	return 1;*/

	const bool b0 = b.mMaxY < a.mMinY;
	const bool b1 = a.mMaxY < b.mMinY;
	const bool b2 = b.mMaxZ < a.mMinZ;
	const bool b3 = a.mMaxZ < b.mMinZ;
//	const bool b4 = b0 || b1 || b2 || b3;
	const bool b4 = b0 | b1 | b2 | b3;
	return !b4;
}
#endif

static PX_FORCE_INLINE void outputPair(ABP_PairManager& pairManager, PxU32 index0, PxU32 index1)
{
	pairManager.addPair(index0, index1);
}

template<int codepath>
static void boxPruningKernel(	PxU32 nb0, PxU32 nb1,
								const SIMD_AABB_X4* PX_RESTRICT boxes0_X, const SIMD_AABB_X4* PX_RESTRICT boxes1_X,
								const SIMD_AABB_YZ4* PX_RESTRICT boxes0_YZ, const SIMD_AABB_YZ4* PX_RESTRICT boxes1_YZ,
								const ABP_Index* PX_RESTRICT inToOut0, const ABP_Index* PX_RESTRICT inToOut1,
								ABP_PairManager* PX_RESTRICT pairManager, const ABPEntry* PX_RESTRICT objects)
{
	pairManager->mInToOut0 = inToOut0;
	pairManager->mInToOut1 = inToOut1;
	pairManager->mObjects = objects;

	PxU32 index0 = 0;
	PxU32 runningIndex1 = 0;

	while(runningIndex1<nb1 && index0<nb0)
	{
		const SIMD_AABB_X4& box0_X = boxes0_X[index0];
		const PosXType2 maxLimit = box0_X.mMaxX;

		const PosXType2 minLimit = box0_X.mMinX;
		if(!codepath)
		{
			while(boxes1_X[runningIndex1].mMinX<minLimit)
				runningIndex1++;
		}
		else
		{
			while(boxes1_X[runningIndex1].mMinX<=minLimit)
				runningIndex1++;
		}

		const SIMD_AABB_YZ4& box0 = boxes0_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex1;

			while(boxes1_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes1_YZ[index1])
				{
					outputPair(*pairManager, index0, index1);
				}
				index1++;
			}
		}
		else
		{
			PxU32 Offset = 0;
			const char* const CurrentBoxListYZ = reinterpret_cast<const char*>(&boxes1_YZ[runningIndex1]);
			const char* const CurrentBoxListX = reinterpret_cast<const char*>(&boxes1_X[runningIndex1]);

			if(!gUnrollLoop)
			{
				while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)
				{
					const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2);
#ifdef ABP_SIMD_OVERLAP
					if(SIMD_OVERLAP_TEST_14a(box))
#else
					if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))
#endif
					{
						const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
						outputPair(*pairManager, index0, Index1);
					}
					Offset += 8;
				}
			}
			else
			{

#define BIP_VERSION4
#ifdef BIP_VERSION4
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(SIMD_OVERLAP_TEST_14a(box))	\
								goto label;	}
#else
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))	\
								goto label;	}
#endif
		goto StartLoop4;
		CODEALIGN16
FoundOverlap3:
		Offset += 8;
		CODEALIGN16
FoundOverlap2:
		Offset += 8;
		CODEALIGN16
FoundOverlap1:
		Offset += 8;
		CODEALIGN16
FoundOverlap0:
		Offset += 8;
		CODEALIGN16
FoundOverlap:
		{
			const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - 8 - reinterpret_cast<const char*>(boxes1_X))>>3;
			outputPair(*pairManager, index0, Index1);
		}
		CODEALIGN16
StartLoop4:
		while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset + 8*5)<=maxLimit)
		{
			BLOCK4(0, FoundOverlap0)
			BLOCK4(8, FoundOverlap1)
			BLOCK4(16, FoundOverlap2)
			BLOCK4(24, FoundOverlap3)
			Offset += 40;
			BLOCK4(-8, FoundOverlap)
		}
#undef BLOCK4
#endif

#ifdef ABP_SIMD_OVERLAP
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)			\
				{if(SIMD_OVERLAP_TEST_14a(reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2)))	\
						goto OverlapFound;										\
					Offset += 8;
#else
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)			\
				{if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2)))	\
						goto OverlapFound;										\
					Offset += 8;
#endif

		goto LoopStart;
		CODEALIGN16
OverlapFound:
		{
			const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
			outputPair(*pairManager, index0, Index1);
		}
		Offset += 8;
		CODEALIGN16
LoopStart:
		BLOCK
			BLOCK
				BLOCK
				}
			}
			goto LoopStart;
		}
#undef BLOCK
			}
		}

		index0++;
	}
}

static /*PX_FORCE_INLINE*/ void doBipartiteBoxPruning_Leaf(
		ABP_PairManager* PX_RESTRICT pairManager,
		const ABPEntry* PX_RESTRICT objects,
		PxU32 nb0,
		PxU32 nb1,
		const SIMD_AABB_X4* PX_RESTRICT boxes0_X,
		const SIMD_AABB_X4* PX_RESTRICT boxes1_X,
		const SIMD_AABB_YZ4* PX_RESTRICT boxes0_YZ,
		const SIMD_AABB_YZ4* PX_RESTRICT boxes1_YZ,
		const ABP_Index* PX_RESTRICT remap0,
		const ABP_Index* PX_RESTRICT remap1
		)
{
	PX_ASSERT(boxes0_X[nb0].isSentinel());
	PX_ASSERT(boxes1_X[nb1].isSentinel());
	boxPruningKernel<0>(nb0, nb1, boxes0_X, boxes1_X, boxes0_YZ, boxes1_YZ, remap0, remap1, pairManager, objects);
	boxPruningKernel<1>(nb1, nb0, boxes1_X, boxes0_X, boxes1_YZ, boxes0_YZ, remap1, remap0, pairManager, objects);
}

static PX_FORCE_INLINE void doBipartiteBoxPruning_Leaf(ABP_PairManager* PX_RESTRICT pairManager, const ABPEntry* PX_RESTRICT objects,
		PxU32 nb0, PxU32 nb1, const SplitBoxes& boxes0, const SplitBoxes& boxes1, const ABP_Index* PX_RESTRICT remap0, const ABP_Index* PX_RESTRICT remap1)
{
	doBipartiteBoxPruning_Leaf(pairManager, objects, nb0, nb1, boxes0.getBoxes_X(), boxes1.getBoxes_X(), boxes0.getBoxes_YZ(), boxes1.getBoxes_YZ(), remap0, remap1);
}

static void doCompleteBoxPruning_Leaf(	ABP_PairManager* PX_RESTRICT pairManager, PxU32 nb,
										const SIMD_AABB_X4* PX_RESTRICT boxes_X,
										const SIMD_AABB_YZ4* PX_RESTRICT boxes_YZ,
										const ABP_Index* PX_RESTRICT remap,
										const ABPEntry* PX_RESTRICT objects)
{
	pairManager->mInToOut0 = remap;
	pairManager->mInToOut1 = remap;
	pairManager->mObjects = objects;

	PxU32 index0 = 0;
	PxU32 runningIndex = 0;
	while(runningIndex<nb && index0<nb)
	{
		const SIMD_AABB_X4& box0_X = boxes_X[index0];
		const PosXType2 maxLimit = box0_X.mMaxX;

		const PosXType2 minLimit = box0_X.mMinX;
		while(boxes_X[runningIndex++].mMinX<minLimit);

		const SIMD_AABB_YZ4& box0 = boxes_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex;
			while(boxes_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes_YZ[index1])
				{
					outputPair(*pairManager, index0, index1);
				}
				index1++;
			}
		}
		else
		{
			PxU32 Offset = 0;
			const char* const CurrentBoxListYZ = reinterpret_cast<const char*>(&boxes_YZ[runningIndex]);
			const char* const CurrentBoxListX = reinterpret_cast<const char*>(&boxes_X[runningIndex]);

			if(!gUnrollLoop)
			{
				while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)
				{
					const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2);
#ifdef ABP_SIMD_OVERLAP
					if(SIMD_OVERLAP_TEST_14a(box))
#else
					if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))
#endif
					{
						const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
						outputPair(*pairManager, index0, Index);
					}
					Offset += 8;
				}
			}
			else
			{

#define VERSION4c
#ifdef VERSION4c
#define VERSION3	// Enable this as our safe loop
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
							if(SIMD_OVERLAP_TEST_14a(box))					\
								goto label;	}
#else
	#define BLOCK4(x, label)	{const SIMD_AABB_YZ4* box = reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2 + x*2);	\
							if(intersect2D(box0, *box))													\
								goto label;	}
#endif
		goto StartLoop4;
		CODEALIGN16
FoundOverlap3:
		Offset += 8;
		CODEALIGN16
FoundOverlap2:
		Offset += 8;
		CODEALIGN16
FoundOverlap1:
		Offset += 8;
		CODEALIGN16
FoundOverlap0:
		Offset += 8;
		CODEALIGN16
FoundOverlap:
		{
			const PxU32 Index = PxU32(CurrentBoxListX + Offset - 8 - reinterpret_cast<const char*>(boxes_X))>>3;
			outputPair(*pairManager, index0, Index);
		}
		CODEALIGN16
StartLoop4:
		while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset + 8*5)<=maxLimit)
		{
			BLOCK4(0, FoundOverlap0)
			BLOCK4(8, FoundOverlap1)
			BLOCK4(16, FoundOverlap2)
			BLOCK4(24, FoundOverlap3)
			Offset += 40;
			BLOCK4(-8, FoundOverlap)
		}
#endif

#define VERSION3
#ifdef VERSION3
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)			\
				{if(SIMD_OVERLAP_TEST_14a(reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2)))	\
						goto BeforeLoop;										\
					Offset += 8;
#else
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)			\
				{if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2)))	\
						goto BeforeLoop;										\
					Offset += 8;
#endif

		goto StartLoop;
		CODEALIGN16
BeforeLoop:
		{
			const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
			outputPair(*pairManager, index0, Index);
			Offset += 8;
		}
		CODEALIGN16
StartLoop:
		BLOCK
			BLOCK
				BLOCK
					BLOCK
						BLOCK
						}
					}
				}
			}
			goto StartLoop;
		}
#endif
			}
		}

		index0++;
	}
}

#ifdef USE_ABP_BUCKETS
static const PxU8 gCodes[] = {	4, 4, 4, 255, 4, 3, 2, 255,
								4, 1, 0, 255, 255, 255, 255, 255 };

static PX_FORCE_INLINE PxU8 classifyBoxNew(const SIMD_AABB_YZ4& boxYZ, const float limitY, const float limitZ)
{
#ifdef ABP_SIMD_OVERLAP
	// PT: mins have been negated for SIMD tests
	const bool upperPart = (-boxYZ.mMinZ) > limitZ;
	const bool rightPart = (-boxYZ.mMinY) > limitY;
#else
	const bool upperPart = boxYZ.mMinZ > limitZ;
	const bool rightPart = boxYZ.mMinY > limitY;
#endif
	const bool lowerPart = boxYZ.mMaxZ < limitZ;
	const bool leftPart = boxYZ.mMaxY < limitY;

	// Table-based box classification avoids many branches
	const PxU32 Code = PxU32(rightPart)|(PxU32(leftPart)<<1)|(PxU32(upperPart)<<2)|(PxU32(lowerPart)<<3);
	PX_ASSERT(gCodes[Code]!=255);
	return gCodes[Code];
}

//#include <stdio.h>
#ifdef RECURSE_LIMIT
static void CompleteBoxPruning_Recursive(
	ABP_MM& memoryManager,
	ABP_PairManager* PX_RESTRICT pairManager,
	PxU32 nb,
	const SIMD_AABB_X4* PX_RESTRICT listX,
	const SIMD_AABB_YZ4* PX_RESTRICT listYZ,
	const ABP_Index* PX_RESTRICT remap,
	const ABPEntry* PX_RESTRICT objects)
{
//	printf("CompleteBoxPruning_Recursive %d\n", nb);
	if(!nb)
		return;

	/*__declspec(align(16))*/ float mergedMin[4];
	/*__declspec(align(16))*/ float mergedMax[4];
	{
//#ifdef SAFE_VERSION
		Vec4V maxV = V4LoadA(&listYZ[0].mMinY);
		for(PxU32 i=1;i<nb;i++)
			maxV = V4Max(maxV, V4LoadA(&listYZ[i].mMinY));

		PX_ALIGN(16, PxVec4) tmp;
		V4StoreA(maxV, &tmp.x);

		mergedMin[1] = -tmp.x;
		mergedMin[2] = -tmp.y;
		mergedMax[1] = tmp.z;
		mergedMax[2] = tmp.w;
//#endif
	}
	const float limitY = (mergedMax[1] + mergedMin[1]) * 0.5f;
	const float limitZ = (mergedMax[2] + mergedMin[2]) * 0.5f;



	// PT: TODO: revisit allocs
	SIMD_AABB_X4* BoxListXBuffer = reinterpret_cast<SIMD_AABB_X4*>(memoryManager.frameAlloc(sizeof(SIMD_AABB_X4)*(nb+NB_SENTINELS*NB_BUCKETS)));
	SIMD_AABB_YZ4* BoxListYZBuffer = reinterpret_cast<SIMD_AABB_YZ4*>(memoryManager.frameAlloc(sizeof(SIMD_AABB_YZ4)*nb));

	PxU32 Counters[NB_BUCKETS];
	for(PxU32 i=0;i<NB_BUCKETS;i++)
		Counters[i] = 0;

	PxU32* Remap = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(sizeof(PxU32)*nb));
	PxU8* Indices = reinterpret_cast<PxU8*>(memoryManager.frameAlloc(sizeof(PxU8)*nb));
	for(PxU32 i=0;i<nb;i++)
	{
		const PxU8 index = classifyBoxNew(listYZ[i], limitY, limitZ);
		Indices[i] = index;
		Counters[index]++;
	}

	SIMD_AABB_X4* BoxListX[NB_BUCKETS];
	SIMD_AABB_YZ4* BoxListYZ[NB_BUCKETS];
	PxU32* RemapBase[NB_BUCKETS];
	{
		SIMD_AABB_X4* CurrentBoxListXBuffer = BoxListXBuffer;
		SIMD_AABB_YZ4* CurrentBoxListYZBuffer = BoxListYZBuffer;
		PxU32* CurrentRemap = Remap;
		for(PxU32 i=0;i<NB_BUCKETS;i++)
		{
			const PxU32 Nb = Counters[i];
			BoxListX[i] = CurrentBoxListXBuffer;
			BoxListYZ[i] = CurrentBoxListYZBuffer;
			RemapBase[i] =  CurrentRemap;
			CurrentBoxListXBuffer += Nb+NB_SENTINELS;
			CurrentBoxListYZBuffer += Nb;
			CurrentRemap += Nb;
		}
		PX_ASSERT(CurrentBoxListXBuffer == BoxListXBuffer + nb + NB_SENTINELS*NB_BUCKETS);
		PX_ASSERT(CurrentBoxListYZBuffer == BoxListYZBuffer + nb);
		PX_ASSERT(CurrentRemap == Remap + nb);
	}

	for(PxU32 i=0;i<NB_BUCKETS;i++)
		Counters[i] = 0;

	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 SortedIndex = i;
		const PxU32 TargetBucket = PxU32(Indices[SortedIndex]);

		const PxU32 IndexInTarget = Counters[TargetBucket]++;

		SIMD_AABB_X4* TargetBoxListX = BoxListX[TargetBucket];
		SIMD_AABB_YZ4* TargetBoxListYZ = BoxListYZ[TargetBucket];
		PxU32* TargetRemap = RemapBase[TargetBucket];

		TargetRemap[IndexInTarget] = remap[SortedIndex];
		TargetBoxListX[IndexInTarget] = listX[SortedIndex];
		TargetBoxListYZ[IndexInTarget] = listYZ[SortedIndex];
	}
	memoryManager.frameFree(Indices);

	for(PxU32 i=0;i<NB_BUCKETS;i++)
	{
		SIMD_AABB_X4* TargetBoxListX = BoxListX[i];
		const PxU32 IndexInTarget = Counters[i];
		for(PxU32 j=0;j<NB_SENTINELS;j++)
			TargetBoxListX[IndexInTarget+j].initSentinel();
	}

	{
		const PxU32 limit = RECURSE_LIMIT;
		for(PxU32 i=0;i<NB_BUCKETS;i++)
		{
			if(Counters[i]<limit || Counters[i]==nb)
				doCompleteBoxPruning_Leaf(	pairManager,
										Counters[i],
										BoxListX[i], BoxListYZ[i],
										RemapBase[i],
										objects);
			else
				CompleteBoxPruning_Recursive(memoryManager, pairManager,
										Counters[i],
										BoxListX[i], BoxListYZ[i],
										RemapBase[i],
										objects);

		}
	}

	{
		for(PxU32 i=0;i<NB_BUCKETS-1;i++)
		{
			doBipartiteBoxPruning_Leaf(pairManager, objects,
				Counters[i], Counters[NB_BUCKETS-1],
				BoxListX[i], BoxListX[NB_BUCKETS-1], BoxListYZ[i], BoxListYZ[NB_BUCKETS-1],
				RemapBase[i], RemapBase[NB_BUCKETS-1]
				);
		}
	}

	memoryManager.frameFree(Remap);
	memoryManager.frameFree(BoxListYZBuffer);
	memoryManager.frameFree(BoxListXBuffer);
}
#endif

#ifndef USE_ALTERNATIVE_VERSION
static void CompleteBoxPruning_Version16(
	ABP_MM& memoryManager,
	const PxBounds3& updatedBounds,
	ABP_PairManager* PX_RESTRICT pairManager,
	PxU32 nb,
	const SIMD_AABB_X4* PX_RESTRICT listX,
	const SIMD_AABB_YZ4* PX_RESTRICT listYZ,
	const ABP_Index* PX_RESTRICT remap,
	const ABPEntry* PX_RESTRICT objects)
{
	if(!nb)
		return;

	// PT: TODO: revisit allocs
	SIMD_AABB_X4* BoxListXBuffer = reinterpret_cast<SIMD_AABB_X4*>(memoryManager.frameAlloc(sizeof(SIMD_AABB_X4)*(nb+NB_SENTINELS*NB_BUCKETS)));
	SIMD_AABB_YZ4* BoxListYZBuffer = reinterpret_cast<SIMD_AABB_YZ4*>(memoryManager.frameAlloc(sizeof(SIMD_AABB_YZ4)*nb));

	const PxVec3& mergedMin = updatedBounds.minimum;
	const PxVec3& mergedMax = updatedBounds.maximum;
	const float limitY = (mergedMax[1] + mergedMin[1]) * 0.5f;
	const float limitZ = (mergedMax[2] + mergedMin[2]) * 0.5f;

	PxU32 Counters[NB_BUCKETS];
	for(PxU32 i=0;i<NB_BUCKETS;i++)
		Counters[i] = 0;

	PxU32* Remap = reinterpret_cast<PxU32*>(memoryManager.frameAlloc(sizeof(PxU32)*nb));
	PxU8* Indices = reinterpret_cast<PxU8*>(memoryManager.frameAlloc(sizeof(PxU8)*nb));
	for(PxU32 i=0;i<nb;i++)
	{
		const PxU8 index = classifyBoxNew(listYZ[i], limitY, limitZ);
		Indices[i] = index;
		Counters[index]++;
	}

	SIMD_AABB_X4* BoxListX[NB_BUCKETS];
	SIMD_AABB_YZ4* BoxListYZ[NB_BUCKETS];
	PxU32* RemapBase[NB_BUCKETS];
	{
		SIMD_AABB_X4* CurrentBoxListXBuffer = BoxListXBuffer;
		SIMD_AABB_YZ4* CurrentBoxListYZBuffer = BoxListYZBuffer;
		PxU32* CurrentRemap = Remap;
		for(PxU32 i=0;i<NB_BUCKETS;i++)
		{
			const PxU32 Nb = Counters[i];
			BoxListX[i] = CurrentBoxListXBuffer;
			BoxListYZ[i] = CurrentBoxListYZBuffer;
			RemapBase[i] =  CurrentRemap;
			CurrentBoxListXBuffer += Nb+NB_SENTINELS;
			CurrentBoxListYZBuffer += Nb;
			CurrentRemap += Nb;
		}
		PX_ASSERT(CurrentBoxListXBuffer == BoxListXBuffer + nb + NB_SENTINELS*NB_BUCKETS);
		PX_ASSERT(CurrentBoxListYZBuffer == BoxListYZBuffer + nb);
		PX_ASSERT(CurrentRemap == Remap + nb);
	}

	for(PxU32 i=0;i<NB_BUCKETS;i++)
		Counters[i] = 0;

	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 SortedIndex = i;
		const PxU32 TargetBucket = PxU32(Indices[SortedIndex]);

		const PxU32 IndexInTarget = Counters[TargetBucket]++;

		SIMD_AABB_X4* TargetBoxListX = BoxListX[TargetBucket];
		SIMD_AABB_YZ4* TargetBoxListYZ = BoxListYZ[TargetBucket];
		PxU32* TargetRemap = RemapBase[TargetBucket];

		TargetRemap[IndexInTarget] = remap[SortedIndex];
		TargetBoxListX[IndexInTarget] = listX[SortedIndex];
		TargetBoxListYZ[IndexInTarget] = listYZ[SortedIndex];
	}
	memoryManager.frameFree(Indices);

	for(PxU32 i=0;i<NB_BUCKETS;i++)
	{
		SIMD_AABB_X4* TargetBoxListX = BoxListX[i];
		const PxU32 IndexInTarget = Counters[i];
		for(PxU32 j=0;j<NB_SENTINELS;j++)
			TargetBoxListX[IndexInTarget+j].initSentinel();
	}

	{
		for(PxU32 i=0;i<NB_BUCKETS;i++)
		{
#ifdef RECURSE_LIMIT
			if(Counters[i]<RECURSE_LIMIT || Counters[i]==nb)
#endif
				doCompleteBoxPruning_Leaf(	pairManager,
										Counters[i],
										BoxListX[i], BoxListYZ[i],
										RemapBase[i],
										objects);
#ifdef RECURSE_LIMIT
			else
				CompleteBoxPruning_Recursive(memoryManager, pairManager,
										Counters[i],
										BoxListX[i], BoxListYZ[i],
										RemapBase[i],
										objects);
#endif

		}
	}

	{
		for(PxU32 i=0;i<NB_BUCKETS-1;i++)
		{
			doBipartiteBoxPruning_Leaf(pairManager, objects,
				Counters[i], Counters[NB_BUCKETS-1],
				BoxListX[i], BoxListX[NB_BUCKETS-1], BoxListYZ[i], BoxListYZ[NB_BUCKETS-1],
				RemapBase[i], RemapBase[NB_BUCKETS-1]
				);
		}
	}

	memoryManager.frameFree(Remap);
	memoryManager.frameFree(BoxListYZBuffer);
	memoryManager.frameFree(BoxListXBuffer);
}
#endif
#endif

#ifdef USE_ALTERNATIVE_VERSION
// PT: experimental version that adds all cross-bucket objects to all regular buckets
static void CompleteBoxPruning_Version16(
	ABP_MM& /*memoryManager*/,
	const PxBounds3& updatedBounds,
	ABP_PairManager* PX_RESTRICT pairManager,
	PxU32 nb,
	const SIMD_AABB_X4* PX_RESTRICT listX,
	const SIMD_AABB_YZ4* PX_RESTRICT listYZ,
	const ABP_Index* PX_RESTRICT remap,
	const ABPEntry* PX_RESTRICT objects)
{
	if(!nb)
		return;

	const PxVec3& mergedMin = updatedBounds.minimum;
	const PxVec3& mergedMax = updatedBounds.maximum;
	const float limitY = (mergedMax[1] + mergedMin[1]) * 0.5f;
	const float limitZ = (mergedMax[2] + mergedMin[2]) * 0.5f;

	PxU32 Counters[NB_BUCKETS];
	for(PxU32 i=0;i<NB_BUCKETS;i++)
		Counters[i] = 0;

	PxU8* Indices = (PxU8*)PX_ALLOC(sizeof(PxU8)*nb, PX_DEBUG_EXP("temp"));
	for(PxU32 i=0;i<nb;i++)
	{
		const PxU8 index = classifyBoxNew(listYZ[i], limitY, limitZ);
		Indices[i] = index;
		Counters[index]++;
	}

	PxU32 total = 0;
	PxU32 Counters2[4];
	for(PxU32 i=0;i<4;i++)
	{
		Counters2[i] = Counters[i] + Counters[4];
		total += Counters2[i];
	}

	// PT: TODO: revisit allocs
	SIMD_AABB_X4* BoxListXBuffer = (SIMD_AABB_X4*)PX_ALLOC(sizeof(SIMD_AABB_X4)*(total+NB_SENTINELS*NB_BUCKETS), PX_DEBUG_EXP("temp"));
	SIMD_AABB_YZ4* BoxListYZBuffer = (SIMD_AABB_YZ4*)PX_ALLOC(sizeof(SIMD_AABB_YZ4)*total, PX_DEBUG_EXP("temp"));

	PxU32* Remap = (PxU32*)PX_ALLOC(sizeof(PxU32)*total, PX_DEBUG_EXP("temp"));

	SIMD_AABB_X4* CurrentBoxListXBuffer = BoxListXBuffer;
	SIMD_AABB_YZ4* CurrentBoxListYZBuffer = BoxListYZBuffer;
	PxU32* CurrentRemap = Remap;
	SIMD_AABB_X4* BoxListX[4];
	SIMD_AABB_YZ4* BoxListYZ[4];
	PxU32* RemapBase[4];
	for(PxU32 i=0;i<4;i++)
	{
		const PxU32 Nb = Counters2[i];
		BoxListX[i] = CurrentBoxListXBuffer;
		BoxListYZ[i] = CurrentBoxListYZBuffer;
		RemapBase[i] =  CurrentRemap;
		CurrentBoxListXBuffer += Nb+NB_SENTINELS;
		CurrentBoxListYZBuffer += Nb;
		CurrentRemap += Nb;
	}
	PX_ASSERT(CurrentBoxListXBuffer == BoxListXBuffer + total + NB_SENTINELS*NB_BUCKETS);
	PX_ASSERT(CurrentBoxListYZBuffer == BoxListYZBuffer + total);
	PX_ASSERT(CurrentRemap == Remap + total);

	for(PxU32 i=0;i<4;i++)
		Counters2[i] = 0;

	for(PxU32 i=0;i<nb;i++)
	{
		const PxU32 SortedIndex = i;
		const PxU32 TargetBucket = PxU32(Indices[SortedIndex]);
		if(TargetBucket==4)
		{
			for(PxU32 j=0;j<4;j++)
			{
				const PxU32 IndexInTarget = Counters2[j]++;

				SIMD_AABB_X4* TargetBoxListX = BoxListX[j];
				SIMD_AABB_YZ4* TargetBoxListYZ = BoxListYZ[j];
				PxU32* TargetRemap = RemapBase[j];

				TargetRemap[IndexInTarget] = remap[SortedIndex];
				TargetBoxListX[IndexInTarget] = listX[SortedIndex];
				TargetBoxListYZ[IndexInTarget] = listYZ[SortedIndex];
			}
		}
		else
		{
			const PxU32 IndexInTarget = Counters2[TargetBucket]++;

			SIMD_AABB_X4* TargetBoxListX = BoxListX[TargetBucket];
			SIMD_AABB_YZ4* TargetBoxListYZ = BoxListYZ[TargetBucket];
			PxU32* TargetRemap = RemapBase[TargetBucket];

			TargetRemap[IndexInTarget] = remap[SortedIndex];
			TargetBoxListX[IndexInTarget] = listX[SortedIndex];
			TargetBoxListYZ[IndexInTarget] = listYZ[SortedIndex];
		}
	}
	PX_FREE(Indices);

	for(PxU32 i=0;i<4;i++)
	{
		SIMD_AABB_X4* TargetBoxListX = BoxListX[i];
		const PxU32 IndexInTarget = Counters2[i];
		for(PxU32 j=0;j<NB_SENTINELS;j++)
			TargetBoxListX[IndexInTarget+j].initSentinel();
	}

	{
		for(PxU32 i=0;i<4;i++)
		{
#ifdef RECURSE_LIMIT
			if(Counters2[i]<RECURSE_LIMIT || Counters2[i]==nb)
#endif
				doCompleteBoxPruning_Leaf(	pairManager,
									Counters2[i],
									BoxListX[i], BoxListYZ[i],
									RemapBase[i],
									objects);
#ifdef RECURSE_LIMIT
			else
				CompleteBoxPruning_Recursive(	pairManager,
									Counters2[i],
									BoxListX[i], BoxListYZ[i],
									RemapBase[i],
									objects);
#endif
		}
	}

	PX_FREE(Remap);
	PX_FREE(BoxListYZBuffer);
	PX_FREE(BoxListXBuffer);
}
#endif

static void doCompleteBoxPruning_(ABP_MM& memoryManager, ABP_PairManager* PX_RESTRICT pairManager, const ABPEntry* PX_RESTRICT objects, const DynamicManager& mDBM)
{
	const PxU32 nbUpdated = mDBM.getNbUpdatedBoxes();
	if(!nbUpdated)
		return;
	const PxU32 nbNonUpdated = mDBM.getNbNonUpdatedBoxes();
	const DynamicBoxes& updatedBoxes = mDBM.getUpdatedBoxes();

	const SIMD_AABB_X4* PX_RESTRICT updatedDynamicBoxes_X = updatedBoxes.getBoxes_X();
	const SIMD_AABB_YZ4* PX_RESTRICT updatedDynamicBoxes_YZ = updatedBoxes.getBoxes_YZ();

	// PT: find sleeping-dynamics-vs-active-dynamics overlaps
	if(nbNonUpdated)
	{
		doBipartiteBoxPruning_Leaf(	pairManager, objects,
								nbUpdated, nbNonUpdated,
								updatedBoxes, mDBM.getSleepingBoxes(),
								mDBM.getRemap_Updated(), mDBM.getRemap_Sleeping());
	}

	///////

	// PT: find active-dynamics-vs-active-dynamics overlaps
	if(1)
	{
		PX_UNUSED(memoryManager);
#ifdef USE_ABP_BUCKETS
		if(nbUpdated>USE_ABP_BUCKETS)
			CompleteBoxPruning_Version16(memoryManager, mDBM.getUpdatedBounds(), pairManager, nbUpdated,
								updatedDynamicBoxes_X,
								updatedDynamicBoxes_YZ,
								mDBM.getRemap_Updated(), objects);
		else
#endif
			doCompleteBoxPruning_Leaf(pairManager, nbUpdated,
								updatedDynamicBoxes_X,
								updatedDynamicBoxes_YZ,
								mDBM.getRemap_Updated(), objects);
	}
}

void ABP::Region_prepareOverlaps()
{
	if(		!mDBM.isThereWorkToDo()
		&&	!mKBM.isThereWorkToDo()
		&&	!mSBM.isThereWorkToDo()
		)
		return;

	if(mSBM.isThereWorkToDo())
	{
		mSBM.prepareData(mRS, mShared.mABP_Objects, mShared.mABP_Objects_Capacity, mMM);
		mDBM.notifyStaticChange(mShared.mABP_Objects);
		mKBM.notifyStaticChange(mShared.mABP_Objects);
	}

	mDBM.prepareData(mRS, mShared.mABP_Objects, mShared.mABP_Objects_Capacity, mMM);
	mKBM.prepareData(mRS, mShared.mABP_Objects, mShared.mABP_Objects_Capacity, mMM);

	mRS.reset();
}

// Finds static-vs-dynamic and dynamic-vs-dynamic overlaps
static void findAllOverlaps(ABP_MM& memoryManager, ABP_PairManager& pairManager, const ABP_SharedData& mShared, const StaticManager& mSBM, const DynamicManager& mDBM, bool doComplete, bool doBipartite)
{
	const PxU32 nbUpdatedBoxes = mDBM.getNbUpdatedBoxes();

	// PT: find dynamics-vs-dynamics overlaps
	if(doComplete)
		doCompleteBoxPruning_(memoryManager, &pairManager, mShared.mABP_Objects, mDBM);

	// PT: find dynamics-vs-statics overlaps
	if(doBipartite)
	{
		// PT: in previous versions we did active-dynamics-vs-all-statics here.
		if(nbUpdatedBoxes)
		{
			if(mSBM.getNbUpdatedBoxes())
			{
				doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
										nbUpdatedBoxes, mSBM.getNbUpdatedBoxes(),
										mDBM.getUpdatedBoxes(), mSBM.getUpdatedBoxes(),
										mDBM.getRemap_Updated(), mSBM.getRemap_Updated()
									);
			}
			if(mSBM.getNbNonUpdatedBoxes())
			{
				doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
										nbUpdatedBoxes, mSBM.getNbNonUpdatedBoxes(),
										mDBM.getUpdatedBoxes(), mSBM.getSleepingBoxes(),
										mDBM.getRemap_Updated(), mSBM.getRemap_Sleeping()
									);
			}
		}

		// PT: TODO: refactor this with kinematic stuff, etc.... bit tedious
		if(mSBM.getNbUpdatedBoxes() && mDBM.getNbNonUpdatedBoxes())
		{
			doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
									mDBM.getNbNonUpdatedBoxes(), mSBM.getNbUpdatedBoxes(),
									mDBM.getSleepingBoxes(), mSBM.getUpdatedBoxes(),
									mDBM.getRemap_Sleeping(), mSBM.getRemap_Updated()
								);
		}

	}
}

void ABP::Region_findOverlaps(ABP_PairManager& pairManager)
{
	if(!gPrepareOverlapsFlag)
		Region_prepareOverlaps();

	bool doKineKine = true;
	bool doStaticKine = true;
	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	{
		const bool* lut = mPairManager.mLUT;
		doStaticKine = lut[Bp::FilterType::KINEMATIC*4+Bp::FilterType::STATIC];
		doKineKine = lut[Bp::FilterType::KINEMATIC*4+Bp::FilterType::KINEMATIC];
	}
	#endif

	findAllOverlaps(mMM, pairManager, mShared, mSBM, mDBM, true, true);
	findAllOverlaps(mMM, pairManager, mShared, mSBM, mKBM, doKineKine, doStaticKine);

	const PxU32 nbUpdatedDynamics = mDBM.getNbUpdatedBoxes();
	const PxU32 nbNonUpdatedDynamics = mDBM.getNbNonUpdatedBoxes();
	const PxU32 nbUpdatedKinematics = mKBM.getNbUpdatedBoxes();
	const PxU32 nbNonUpdatedKinematics = mKBM.getNbNonUpdatedBoxes();

	if(nbUpdatedDynamics)
	{
		// Active dynamics vs active kinematics
		if(nbUpdatedKinematics)
		{
			doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
									nbUpdatedDynamics, nbUpdatedKinematics,
									mDBM.getUpdatedBoxes(), mKBM.getUpdatedBoxes(),
									mDBM.getRemap_Updated(), mKBM.getRemap_Updated()
								);
		}
		// Active dynamics vs inactive kinematics
		if(nbNonUpdatedKinematics)
		{
			doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
									nbUpdatedDynamics, nbNonUpdatedKinematics,
									mDBM.getUpdatedBoxes(), mKBM.getSleepingBoxes(),
									mDBM.getRemap_Updated(), mKBM.getRemap_Sleeping()
								);
		}
	}

	if(nbUpdatedKinematics && nbNonUpdatedDynamics)
	{
		// Inactive dynamics vs active kinematics
		doBipartiteBoxPruning_Leaf(	&pairManager, mShared.mABP_Objects,
								nbNonUpdatedDynamics, nbUpdatedKinematics,
								mDBM.getSleepingBoxes(), mKBM.getUpdatedBoxes(),
								mDBM.getRemap_Sleeping(), mKBM.getRemap_Updated()
							);
	}

	mSBM.finalize();
	mDBM.finalize();
	mKBM.finalize();
}

///////////////////////////////////////////////////////////////////////////

ABP::ABP() :
	mSBM						(FilterType::STATIC),
	mDBM						(FilterType::DYNAMIC),
	mKBM						(FilterType::KINEMATIC),
	mTransientBounds			(NULL),
	mTransientContactDistance	(NULL)
//	mTransientGroups			(NULL)
{
}

ABP::~ABP()
{
	reset();
}

void ABP::freeBuffers()
{
	mShared.mRemovedObjects.empty();
}

void ABP::preallocate(PxU32 nbObjects, PxU32 maxNbOverlaps)
{
	if(nbObjects)
	{
		PX_DELETE_ARRAY(mShared.mABP_Objects);
		ABP_Object* objects = PX_NEW(ABP_Object)[nbObjects];
		mShared.mABP_Objects = objects;
		mShared.mABP_Objects_Capacity = nbObjects;
#if PX_DEBUG
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i].mUpdated = false;
#endif
	}

	// PT: TODO: here we should preallocate the box arrays but we don't know how many of them will be static / dynamic...

	mPairManager.reserveMemory(maxNbOverlaps);
}

void ABP::addStaticObjects(const BpHandle* userID_, PxU32 nb, PxU32 maxID)
{
	mShared.checkResize(maxID);

	mSBM.addObjects(userID_, nb, NULL);
}

void ABP::addDynamicObjects(const BpHandle* userID_, PxU32 nb, PxU32 maxID)
{
	mShared.checkResize(maxID);

	mShared.mUpdatedObjects.checkResize(maxID);

	mDBM.addObjects(userID_, nb, &mShared);
}

void ABP::addKinematicObjects(const BpHandle* userID_, PxU32 nb, PxU32 maxID)
{
	mShared.checkResize(maxID);

	mShared.mUpdatedObjects.checkResize(maxID);

	mKBM.addObjects(userID_, nb, &mShared);
}

void ABP::removeObject(BpHandle userID)
{
	mShared.mUpdatedObjects.setBitChecked(userID);
	mShared.mRemovedObjects.setBitChecked(userID);

	PX_ASSERT(userID<mShared.mABP_Objects_Capacity);
	ABPEntry& object = mShared.mABP_Objects[userID];

	// PT: TODO better
	BoxManager* bm;
	const FilterType::Enum objectType = object.getType();
	if(objectType==FilterType::STATIC)
	{
		bm = &mSBM;
	}
	else if(objectType==FilterType::KINEMATIC)
	{
		bm = &mKBM;
	}
	else
	{
		bm = &mDBM;
	}
	bm->removeObject(object, userID);

	object.invalidateIndex();
#if PX_DEBUG
	object.mUpdated = false;
#endif
}

void ABP::updateObject(BpHandle userID)
{
	mShared.mUpdatedObjects.setBitChecked(userID);

	PX_ASSERT(userID<mShared.mABP_Objects_Capacity);
	ABPEntry& object = mShared.mABP_Objects[userID];

	// PT: TODO better
	BoxManager* bm;
	const FilterType::Enum objectType = object.getType();
	if(objectType==FilterType::STATIC)
	{
		bm = &mSBM;
	}
	else if(objectType==FilterType::KINEMATIC)
	{
		bm = &mKBM;
	}
	else
	{
		bm = &mDBM;
	}
	bm->updateObject(object, userID);
}

void ABP_PairManager::computeCreatedDeletedPairs(BroadPhaseABP* mbp, const BitArray& updated, const BitArray& removed)
{
	// PT: parse all currently active pairs. The goal here is to generate the found/lost pairs, compared to previous frame.
	PxU32 i=0;
	PxU32 nbActivePairs = mNbActivePairs;
	while(i<nbActivePairs)
	{
		InternalPair& p = mActivePairs[i];

		if(p.isNew())
		{
			// New pair

			// PT: 'isNew' is set to true in the 'addPair' function. In this case the pair did not previously
			// exist in the structure, and thus we must report the new pair to the client code.
			//
			// PT: group-based filtering is not needed here, since it has already been done in 'addPair'
			const PxU32 id0 = p.getId0();
			const PxU32 id1 = p.getId1();
			PX_ASSERT(id0!=INVALID_ID);
			PX_ASSERT(id1!=INVALID_ID);
			mbp->mCreated.pushBack(BroadPhasePair(id0, id1));

			p.clearNew();
			p.clearUpdated();
			i++;
		}
		else if(p.isUpdated())
		{
			// Persistent pair

			// PT: this pair already existed in the structure, and has been found again this frame. Since
			// MBP reports "all pairs" each frame (as opposed to SAP), this happens quite often, for each
			// active persistent pair.
			p.clearUpdated();
			i++;
		}
		else
		{
			// Lost pair

			// PT: if the pair is not new and not 'updated', it might be a lost (separated) pair. But this
			// is not always the case since we now handle "sleeping" objects directly within MBP. A pair
			// of sleeping objects does not generate an 'addPair' call, so it ends up in this codepath.
			// Nonetheless the sleeping pair should not be deleted. We can only delete pairs involving
			// objects that have been actually moved during the frame. This is the only case in which
			// a pair can indeed become 'lost'.
			const PxU32 id0 = p.getId0();
			const PxU32 id1 = p.getId1();
			PX_ASSERT(id0!=INVALID_ID);
			PX_ASSERT(id1!=INVALID_ID);

			// PT: if none of the involved objects have been updated, the pair is just sleeping: keep it and skip it.
			if(updated.isSetChecked(id0) || updated.isSetChecked(id1))
			{
				// PT: by design (for better or worse) we do not report pairs to the client when
				// one of the involved objects has been deleted. The pair must still be deleted
				// from the MBP structure though.
				if(!removed.isSetChecked(id0) && !removed.isSetChecked(id1))
				{
					// PT: doing the group-based filtering here is useless. The pair should not have
					// been added in the first place.
					mbp->mDeleted.pushBack(BroadPhasePair(id0, id1));
				}

				const PxU32 hashValue = hash(id0, id1) & mMask;
				removePair(id0, id1, hashValue, i);
				nbActivePairs--;
			}
			else i++;
		}
	}

	shrinkMemory();
}

void ABP::findOverlaps(const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
	)
{
	mPairManager.mGroups = groups;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	mPairManager.mLUT = lut;
#endif

	Region_findOverlaps(mPairManager);
}

PxU32 ABP::finalize(BroadPhaseABP* mbp)
{
	mPairManager.computeCreatedDeletedPairs(mbp, mShared.mUpdatedObjects, mShared.mRemovedObjects);

	mShared.mUpdatedObjects.clearAll();

	return mPairManager.mNbActivePairs;
}

void ABP::reset()
{
	mSBM.reset();
	mDBM.reset();
	mKBM.reset();

	PX_DELETE_ARRAY(mShared.mABP_Objects);
	mShared.mABP_Objects_Capacity = 0;
	mPairManager.purge();
	mShared.mUpdatedObjects.empty();
	mShared.mRemovedObjects.empty();
}

// PT: TODO: is is really ok to use "transient" data in this function?
void ABP::shiftOrigin(const PxVec3& shift)
{
	PX_UNUSED(shift);	// PT: unused because the bounds were pre-shifted before calling this function

	// PT: the AABB manager marks all objects as updated when we shift so the stuff below may not be necessary
}

void ABP::setTransientData(const PxBounds3* bounds, const PxReal* contactDistance/*, const Bp::FilterGroup::Enum* groups*/)
{
	mTransientBounds = bounds;
	mTransientContactDistance = contactDistance;
//	mTransientGroups = groups;

	mSBM.setSourceData(bounds, contactDistance);
	mDBM.setSourceData(bounds, contactDistance);
	mKBM.setSourceData(bounds, contactDistance);
}
///////////////////////////////////////////////////////////////////////////////

}

// Below is the PhysX wrapper = link between AABBManager and ABP

using namespace internalABP;

#define DEFAULT_CREATED_DELETED_PAIRS_CAPACITY 1024

BroadPhaseABP::BroadPhaseABP(	PxU32 maxNbBroadPhaseOverlaps,
								PxU32 maxNbStaticShapes,
								PxU32 maxNbDynamicShapes,
								PxU64 /*contextID*/) :
	mGroups		(NULL)
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	,mLUT		(NULL)
#endif
{
	mABP = PX_NEW(ABP)();

	const PxU32 nbObjects = maxNbStaticShapes + maxNbDynamicShapes;
	mABP->preallocate(nbObjects, maxNbBroadPhaseOverlaps);

	mCreated.reserve(DEFAULT_CREATED_DELETED_PAIRS_CAPACITY);
	mDeleted.reserve(DEFAULT_CREATED_DELETED_PAIRS_CAPACITY);
}

BroadPhaseABP::~BroadPhaseABP()
{
	DELETESINGLE(mABP);
}

void BroadPhaseABP::update(const PxU32 /*numCpuTasks*/, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* /*continuation*/, physx::PxBaseTask* narrowPhaseUnblockTask)
{
#if PX_CHECKED
	PX_CHECK_AND_RETURN(scratchAllocator, "BroadPhaseABP::update - scratchAllocator must be non-NULL \n");
#endif
	mABP->mMM.mScratchAllocator = scratchAllocator;

	// PT: TODO: move this out of update function
	if(narrowPhaseUnblockTask)
		narrowPhaseUnblockTask->removeReference();

	setUpdateData(updateData);

	update();
	postUpdate();
}

void BroadPhaseABP::singleThreadedUpdate(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData)
{
	mABP->mMM.mScratchAllocator = scratchAllocator;
	setUpdateData(updateData);
	update();
	postUpdate();
}

void BroadPhaseABP::removeObjects(const BroadPhaseUpdateData& updateData)
{
	PxU32 nbToGo = updateData.getNumRemovedHandles();
	if(!nbToGo)
		return;

	const BpHandle* PX_RESTRICT removed = updateData.getRemovedHandles();
	PX_ASSERT(removed);

	while(nbToGo--)
	{
		const BpHandle index = *removed++;
		PX_ASSERT(index+1<mABP->mShared.mABP_Objects_Capacity);	// PT: we allocated one more box on purpose
		mABP->removeObject(index);
	}
}

void BroadPhaseABP::updateObjects(const BroadPhaseUpdateData& updateData)
{
	const BpHandle* PX_RESTRICT updated = updateData.getUpdatedHandles();
	if(!updated)
		return;

	PxU32 nbToGo = updateData.getNumUpdatedHandles();
	while(nbToGo--)
	{
		const BpHandle index = *updated++;
		PX_ASSERT(index+1<mABP->mShared.mABP_Objects_Capacity);	// PT: we allocated one more box on purpose
		mABP->updateObject(index);
	}
}

void BroadPhaseABP::addObjects(const BroadPhaseUpdateData& updateData)
{
	PxU32 nbToGo = updateData.getNumCreatedHandles();
	if(!nbToGo)
		return;

	const BpHandle* PX_RESTRICT created = updateData.getCreatedHandles();
	PX_ASSERT(created);

	const Bp::FilterGroup::Enum* PX_RESTRICT groups = updateData.getGroups();

	struct Batch
	{
		PX_FORCE_INLINE	Batch() : mNb(0), mMaxIndex(0)	{}

		PxU32		mNb;
		PxU32		mMaxIndex;
		BpHandle	mIndices[ABP_BATCHING];

		PX_FORCE_INLINE void add(const BpHandle index, internalABP::ABP* PX_RESTRICT abp, FilterType::Enum type)
		{
			PxU32 nb = mNb;
			mMaxIndex = PxMax(mMaxIndex, index);
			mIndices[nb++] = index;
			if(nb==ABP_BATCHING)
			{
				mNb = 0;
				// PT: TODO: we could use a function ptr here
				if(type==FilterType::STATIC)
					abp->addStaticObjects(mIndices, ABP_BATCHING, mMaxIndex);
				else if(type==FilterType::KINEMATIC)
					abp->addKinematicObjects(mIndices, ABP_BATCHING, mMaxIndex);
				else
				{
					PX_ASSERT(type==FilterType::DYNAMIC || type==FilterType::AGGREGATE);
					abp->addDynamicObjects(mIndices, ABP_BATCHING, mMaxIndex);
				}

				mMaxIndex = 0;
			}
			else
				mNb = nb;
		}
	};
	Batch statics;
	Batch dynamics;
	Batch kinematics;

	Batch* batches[FilterType::COUNT];
	batches[FilterType::STATIC] = &statics;
	batches[FilterType::DYNAMIC] = &dynamics;
	batches[FilterType::AGGREGATE] = &dynamics;
	batches[FilterType::KINEMATIC] = &kinematics;

	while(nbToGo--)
	{
		const BpHandle index = *created++;
		PX_ASSERT(index+1<mABP->mShared.mABP_Objects_Capacity);	// PT: we allocated one more box on purpose
		const FilterType::Enum type = FilterType::Enum(groups[index]&3);
		batches[type]->add(index, mABP, type);
	}

	if(statics.mNb)
		mABP->addStaticObjects(statics.mIndices, statics.mNb, statics.mMaxIndex);
	if(kinematics.mNb)
		mABP->addKinematicObjects(kinematics.mIndices, kinematics.mNb, kinematics.mMaxIndex);
	if(dynamics.mNb)
		mABP->addDynamicObjects(dynamics.mIndices, dynamics.mNb, dynamics.mMaxIndex);
}

void BroadPhaseABP::setUpdateData(const BroadPhaseUpdateData& updateData)
{
	mABP->setTransientData(updateData.getAABBs(), updateData.getContactDistance()/*, updateData.getGroups()*/);

	const PxU32 newCapacity = updateData.getCapacity();
	mABP->mShared.checkResize(newCapacity);

#if PX_CHECKED
	// PT: WARNING: this must be done after the allocateMappingArray call
	if(!BroadPhaseUpdateData::isValid(updateData, *this))
	{
		PX_CHECK_MSG(false, "Illegal BroadPhaseUpdateData \n");
		return;
	}
#endif

	mGroups = updateData.getGroups();
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	mLUT = updateData.getLUT();
#endif

	removeObjects(updateData);
	addObjects(updateData);
	updateObjects(updateData);

	PX_ASSERT(!mCreated.size());
	PX_ASSERT(!mDeleted.size());

	if(gPrepareOverlapsFlag)
		mABP->Region_prepareOverlaps();
}

void BroadPhaseABP::update()
{
	mABP->findOverlaps(mGroups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, mLUT
#endif
		);
}

void BroadPhaseABP::postUpdate()
{
	mABP->finalize(this);
}

PxU32 BroadPhaseABP::getNbCreatedPairs() const
{
	return mCreated.size();
}

BroadPhasePair* BroadPhaseABP::getCreatedPairs()
{
	return mCreated.begin();
}

PxU32 BroadPhaseABP::getNbDeletedPairs() const
{
	return mDeleted.size();
}

BroadPhasePair* BroadPhaseABP::getDeletedPairs()
{
	return mDeleted.begin();
}

static void freeBuffer(Ps::Array<BroadPhasePair>& buffer)
{
	const PxU32 size = buffer.size();
	if(size>DEFAULT_CREATED_DELETED_PAIRS_CAPACITY)
	{
		buffer.reset();
		buffer.reserve(DEFAULT_CREATED_DELETED_PAIRS_CAPACITY);
	}
	else
	{
		buffer.clear();
	}
}

void BroadPhaseABP::freeBuffers()
{
	mABP->freeBuffers();
	freeBuffer(mCreated);
	freeBuffer(mDeleted);
}

#if PX_CHECKED
bool BroadPhaseABP::isValid(const BroadPhaseUpdateData& updateData) const
{
	const PxU32 nbObjects = mABP->mShared.mABP_Objects_Capacity;
	PX_UNUSED(nbObjects);
	const ABP_Object* PX_RESTRICT objects = mABP->mShared.mABP_Objects;
	const BpHandle* created = updateData.getCreatedHandles();
	if(created)
	{
		PxU32 nbToGo = updateData.getNumCreatedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *created++;
			PX_ASSERT(index<nbObjects);
			if(objects[index].isValid())
				return false;	// This object has been added already
		}
	}

	const BpHandle* updated = updateData.getUpdatedHandles();
	if(updated)
	{
		PxU32 nbToGo = updateData.getNumUpdatedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *updated++;
			PX_ASSERT(index<nbObjects);
			if(!objects[index].isValid())
				return false;	// This object has been removed already, or never been added
		}
	}

	const BpHandle* removed = updateData.getRemovedHandles();
	if(removed)
	{
		PxU32 nbToGo = updateData.getNumRemovedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *removed++;
			PX_ASSERT(index<nbObjects);
			if(!objects[index].isValid())
				return false;	// This object has been removed already, or never been added
		}
	}
	return true;
}
#endif

void BroadPhaseABP::shiftOrigin(const PxVec3& shift)
{
	mABP->shiftOrigin(shift);
}

PxU32 BroadPhaseABP::getCurrentNbPairs() const
{
	return mABP->mPairManager.mNbActivePairs;
}

void BroadPhaseABP::setScratchAllocator(PxcScratchAllocator* scratchAllocator)
{
	mABP->mMM.mScratchAllocator = scratchAllocator;
}



BroadPhase* createABP(
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID)
{
	return PX_NEW(BroadPhaseABP)(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
}

