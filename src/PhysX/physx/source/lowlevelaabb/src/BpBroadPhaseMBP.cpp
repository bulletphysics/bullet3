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

#include "BpBroadPhaseMBP.h"
#include "BpBroadPhaseShared.h"
#include "CmRadixSortBuffered.h"
#include "PsUtilities.h"
#include "PsFoundation.h"
#include "PsVecMath.h"

using namespace physx::shdfnd::aos;

//#define CHECK_NB_OVERLAPS
#define USE_FULLY_INSIDE_FLAG
//#define MBP_USE_NO_CMP_OVERLAP_3D	// Seems slower

//HWSCAN: reverse bits in fully-inside-flag bitmaps because the code gives us indices for which bits are set (and we want the opposite)
#define HWSCAN

using namespace physx;
using namespace Bp;
using namespace Cm;

	static PX_FORCE_INLINE MBP_Handle encodeHandle(MBP_ObjectIndex objectIndex, PxU32 flipFlop, bool isStatic)
	{
	/*	objectIndex += objectIndex;
		objectIndex |= flipFlop;
		return objectIndex;*/
		return (objectIndex<<2)|(flipFlop<<1)|PxU32(isStatic);
	}

	static PX_FORCE_INLINE MBP_ObjectIndex decodeHandle_Index(MBP_Handle handle)
	{
	//	return handle>>1;
		return handle>>2;
	}

	static PX_FORCE_INLINE PxU32 decodeHandle_IsStatic(MBP_Handle handle)
	{
		return handle&1;
	}

#define MBP_ALLOC(x)		PX_ALLOC(x, "MBP")
#define MBP_ALLOC_TMP(x)	PX_ALLOC_TEMP(x, "MBP_TMP")
#define MBP_FREE(x)			if(x)	PX_FREE_AND_RESET(x)
#define DELETESINGLE(x)		if (x) { delete x;		x = NULL; }
#define DELETEARRAY(x)		if (x) { delete []x;	x = NULL; }

#define	INVALID_ID	0xffffffff

	typedef	MBP_Index*	MBP_Mapping;

/*	PX_FORCE_INLINE PxU32 encodeFloat(const float val)
	{
		// We may need to check on -0 and 0
		// But it should make no practical difference.
		PxU32 ir = IR(val);

		if(ir & 0x80000000) //negative?
			ir = ~ir;//reverse sequence of negative numbers
		else
			ir |= 0x80000000; // flip sign

		return ir;
	}*/

struct RegionHandle : public Ps::UserAllocated
{
	PxU16	mHandle;			// Handle from region
	PxU16	mInternalBPHandle;	// Index of region data within mRegions
};

enum MBPFlags
{
	MBP_FLIP_FLOP	= (1<<1),
	MBP_REMOVED		= (1<<2)	// ### added for TA24714, not needed otherwise
};

// We have one of those for each of the "200K" objects so we should optimize this size as much as possible
struct MBP_Object : public Ps::UserAllocated
{
	BpHandle	mUserID;		// Handle sent to us by the AABB manager
	PxU16		mNbHandles;		// Number of regions the object is part of
	PxU16		mFlags;			// MBPFlags ### only 1 bit used in the end

	PX_FORCE_INLINE	bool	getFlipFlop()	const	{ return (mFlags & MBP_FLIP_FLOP)==0;	}

	union
	{
		RegionHandle	mHandle;
		PxU32			mHandlesIndex;
	};
};

// This one is used in each Region
struct MBPEntry : public Ps::UserAllocated
{
	PX_FORCE_INLINE	MBPEntry()
	{
		mMBPHandle = INVALID_ID;
	}

	// ### mIndex could be PxU16 but beware, we store mFirstFree there
	PxU32		mIndex;			// Out-to-in, maps user handle to internal array. mIndex indexes either the static or dynamic array.
	MBP_Handle	mMBPHandle;		// MBP-level handle (the one returned to users)
#if PX_DEBUG
	bool		mUpdated;
#endif

	PX_FORCE_INLINE		PxU32	isStatic()	const
	{
		return decodeHandle_IsStatic(mMBPHandle);
	}
};

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

#ifdef USE_FULLY_INSIDE_FLAG
static PX_FORCE_INLINE void setBit(BitArray& bitmap, MBP_ObjectIndex objectIndex)
{
	#ifdef HWSCAN
	bitmap.clearBitChecked(objectIndex);	//HWSCAN
	#else
	bitmap.setBitChecked(objectIndex);
	#endif
}

static PX_FORCE_INLINE void clearBit(BitArray& bitmap, MBP_ObjectIndex objectIndex)
{
	#ifdef HWSCAN
	bitmap.setBitChecked(objectIndex);	// HWSCAN
	#else
	bitmap.clearBitChecked(objectIndex);
	#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef MBP_SIMD_OVERLAP
	typedef	SIMD_AABB	MBP_AABB;
#else
	typedef	IAABB		MBP_AABB;
#endif

	struct MBPEntry;
	struct RegionHandle;
	struct MBP_Object;

	class MBP_PairManager : public PairManagerData
	{
		public:
											MBP_PairManager();
											~MBP_PairManager();

						InternalPair*		addPair						(PxU32 id0, PxU32 id1);
//						bool				removePair					(PxU32 id0, PxU32 id1);
						bool				computeCreatedDeletedPairs	(const MBP_Object* objects, BroadPhaseMBP* mbp, const BitArray& updated, const BitArray& removed);

						const Bp::FilterGroup::Enum*	mGroups;
						const MBP_Object*				mObjects;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
						const bool*						mLUT;
#endif
	};

	///////////////////////////////////////////////////////////////////////////

	#define STACK_BUFFER_SIZE	256
	struct MBPOS_TmpBuffers
	{
					MBPOS_TmpBuffers();
					~MBPOS_TmpBuffers();

		void		allocateSleeping(PxU32 nbSleeping, PxU32 nbSentinels);
		void		allocateUpdated(PxU32 nbUpdated, PxU32 nbSentinels);

		// PT: wtf, why doesn't the 128 version compile?
//		MBP_AABB	PX_ALIGN(128, mSleepingDynamicBoxes_Stack[STACK_BUFFER_SIZE]);
//		MBP_AABB	PX_ALIGN(128, mUpdatedDynamicBoxes_Stack[STACK_BUFFER_SIZE]);
		MBP_AABB	PX_ALIGN(16, mSleepingDynamicBoxes_Stack[STACK_BUFFER_SIZE]);
		MBP_AABB	PX_ALIGN(16, mUpdatedDynamicBoxes_Stack[STACK_BUFFER_SIZE]);
		MBP_Index	mInToOut_Dynamic_Sleeping_Stack[STACK_BUFFER_SIZE];

		PxU32		mNbSleeping;
		PxU32		mNbUpdated;
		MBP_Index*	mInToOut_Dynamic_Sleeping;
		MBP_AABB*	mSleepingDynamicBoxes;
		MBP_AABB*	mUpdatedDynamicBoxes;
	};

	struct BIP_Input
	{
		BIP_Input() :
			mObjects		(NULL),
			mNbUpdatedBoxes	(0),
			mNbStaticBoxes	(0),
			mDynamicBoxes	(NULL),
			mStaticBoxes	(NULL),
			mInToOut_Static	(NULL),
			mInToOut_Dynamic(NULL),
			mNeeded			(false)
		{
		}

		const MBPEntry*		mObjects;
		PxU32				mNbUpdatedBoxes;
		PxU32				mNbStaticBoxes;
		const MBP_AABB*		mDynamicBoxes;
		const MBP_AABB*		mStaticBoxes;
		const MBP_Index*	mInToOut_Static;
		const MBP_Index*	mInToOut_Dynamic;
		bool				mNeeded;
	};

	struct BoxPruning_Input
	{
		BoxPruning_Input() :
			mObjects					(NULL),
			mUpdatedDynamicBoxes		(NULL),
			mSleepingDynamicBoxes		(NULL),
			mInToOut_Dynamic			(NULL),
			mInToOut_Dynamic_Sleeping	(NULL),
			mNbUpdated					(0),
			mNbNonUpdated				(0),
			mNeeded						(false)
		{
		}

		const MBPEntry*		mObjects;
		const MBP_AABB*		mUpdatedDynamicBoxes;
		const MBP_AABB*		mSleepingDynamicBoxes;
		const MBP_Index*	mInToOut_Dynamic;
		const MBP_Index*	mInToOut_Dynamic_Sleeping;
		PxU32				mNbUpdated;
		PxU32				mNbNonUpdated;
		bool				mNeeded;

		BIP_Input			mBIPInput;
	};

	class Region : public Ps::UserAllocated
	{
							PX_NOCOPY(Region)
		public:
							Region();
							~Region();

		void				updateObject(const MBP_AABB& bounds, MBP_Index handle);
		MBP_Index			addObject(const MBP_AABB& bounds, MBP_Handle mbpHandle, bool isStatic);
		void				removeObject(MBP_Index handle);
		MBP_Handle			retrieveBounds(MBP_AABB& bounds, MBP_Index handle)	const;
		void				setBounds(MBP_Index handle, const MBP_AABB& bounds);
		void				prepareOverlaps();
		void				findOverlaps(MBP_PairManager& pairManager);

//		private:
		BoxPruning_Input	PX_ALIGN(16, mInput);
		PxU32				mNbObjects;
		PxU32				mMaxNbObjects;
		PxU32				mFirstFree;
		MBPEntry*			mObjects;			// All objects, indexed by user handle
		PxU32				mMaxNbStaticBoxes;
		PxU32				mNbStaticBoxes;
		PxU32				mMaxNbDynamicBoxes;
		PxU32				mNbDynamicBoxes;
		MBP_AABB*			mStaticBoxes;
		MBP_AABB*			mDynamicBoxes;
		MBP_Mapping			mInToOut_Static;	// Maps static boxes to mObjects
		MBP_Mapping			mInToOut_Dynamic;	// Maps dynamic boxes to mObjects
		PxU32*				mPosList;
		PxU32				mNbUpdatedBoxes;
		PxU32				mPrevNbUpdatedBoxes;
		BitArray			mStaticBits;
		RadixSortBuffered	mRS;
		bool				mNeedsSorting;
		bool				mNeedsSortingSleeping;
				
		MBPOS_TmpBuffers	mTmpBuffers;

		void				optimizeMemory();
		void				resizeObjects();
		void				staticSort();
		void				preparePruning(MBPOS_TmpBuffers& buffers);
		void				prepareBIPPruning(const MBPOS_TmpBuffers& buffers);
	};

	///////////////////////////////////////////////////////////////////////////

// We have one of those for each Region within the MBP
struct RegionData : public Ps::UserAllocated
{
	MBP_AABB	mBox;		// Volume of space controlled by this Region
	Region*		mBP;		// Pointer to Region itself
	Ps::IntBool	mOverlap;	// True if overlaps other regions
	void*		mUserData;	// Region identifier, reused to contain "first free ID"
};

	#define MAX_NB_MBP	256
//	#define MAX_NB_MBP	16

	class MBP : public Ps::UserAllocated
	{
		public:
												MBP();
												~MBP();

						void					preallocate(PxU32 nbRegions, PxU32 nbObjects, PxU32 maxNbOverlaps);
						void					reset();
						void					freeBuffers();

						PxU32					addRegion(const PxBroadPhaseRegion& region, bool populateRegion);
						bool					removeRegion(PxU32 handle);
						const Region*			getRegion(PxU32 i)		const;
		PX_FORCE_INLINE	PxU32					getNbRegions()			const	{ return mNbRegions;	}

						MBP_Handle				addObject(const MBP_AABB& box, BpHandle userID, bool isStatic);
						bool					removeObject(MBP_Handle handle);
						bool					updateObject(MBP_Handle handle, const MBP_AABB& box);
						bool					updateObjectAfterRegionRemoval(MBP_Handle handle, Region* removedRegion);
						bool					updateObjectAfterNewRegionAdded(MBP_Handle handle, const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex);
						void					prepareOverlaps();
						void					findOverlaps(const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
							);
						PxU32					finalize(BroadPhaseMBP* mbp);
						void					shiftOrigin(const PxVec3& shift);

						void					setTransientBounds(const PxBounds3* bounds, const PxReal* contactDistance); 
//		private:
						PxU32					mNbRegions;
						MBP_ObjectIndex			mFirstFreeIndex;	// First free recycled index for mMBP_Objects
						PxU32					mFirstFreeIndexBP;	// First free recycled index for mRegions
						Ps::Array<RegionData>	mRegions;
						Ps::Array<MBP_Object>	mMBP_Objects;
						MBP_PairManager			mPairManager;

						BitArray				mUpdatedObjects;	// Indexed by MBP_ObjectIndex
						BitArray				mRemoved;			// Indexed by MBP_ObjectIndex
						Ps::Array<PxU32>   		mHandles[MAX_NB_MBP+1];
						PxU32					mFirstFree[MAX_NB_MBP+1];
		PX_FORCE_INLINE	RegionHandle*			getHandles(MBP_Object& currentObject, PxU32 nbHandles);
						void					purgeHandles(MBP_Object* PX_RESTRICT object, PxU32 nbHandles);
						void					storeHandles(MBP_Object* PX_RESTRICT object, PxU32 nbHandles, const RegionHandle* PX_RESTRICT handles);

						Ps::Array<PxU32>		mOutOfBoundsObjects;	// These are BpHandle but the BP interface expects PxU32s
						void					addToOutOfBoundsArray(BpHandle id);

				const	PxBounds3*				mTransientBounds;
				const	PxReal*					mTransientContactDistance;

#ifdef USE_FULLY_INSIDE_FLAG
						BitArray				mFullyInsideBitmap;	// Indexed by MBP_ObjectIndex
#endif
						void					populateNewRegion(const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex);

#ifdef MBP_REGION_BOX_PRUNING
						void					buildRegionData();
						MBP_AABB				mSortedRegionBoxes[MAX_NB_MBP];
						PxU32					mSortedRegionIndices[MAX_NB_MBP];
						PxU32					mNbActiveRegions;
						bool					mDirtyRegions;
#endif
	};

#ifdef MBP_SIMD_OVERLAP
	#define MBP_OVERLAP_TEST(x)	SIMD_OVERLAP_TEST(x)
#else
	#define MBP_OVERLAP_TEST(x)	if(intersect2D(box0, x))
#endif

#define DEFAULT_NB_ENTRIES	128

#ifdef MBP_SIMD_OVERLAP
	static PX_FORCE_INLINE void initSentinel(SIMD_AABB& box)
	{
	//	box.mMinX = encodeFloat(FLT_MAX)>>1;
		box.mMinX = 0xffffffff;
	}
	#if PX_DEBUG
	static PX_FORCE_INLINE bool isSentinel(const SIMD_AABB& box)
	{
		return box.mMinX == 0xffffffff;
	}
	#endif
#else
	static PX_FORCE_INLINE void initSentinel(MBP_AABB& box)
	{
	//	box.mMinX = encodeFloat(FLT_MAX)>>1;
		box.mMinX = 0xffffffff;
	}
	#if PX_DEBUG
	static PX_FORCE_INLINE bool isSentinel(const MBP_AABB& box)
	{
		return box.mMinX == 0xffffffff;
	}
	#endif
#endif

///////////////////////////////////////////////////////////////////////////////

MBP_PairManager::MBP_PairManager() :
	mGroups			(NULL),
	mObjects		(NULL)
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	,mLUT			(NULL)
#endif
{
}

///////////////////////////////////////////////////////////////////////////////

MBP_PairManager::~MBP_PairManager()
{
}

///////////////////////////////////////////////////////////////////////////////

InternalPair* MBP_PairManager::addPair(PxU32 id0, PxU32 id1)
{
	PX_ASSERT(id0!=INVALID_ID);
	PX_ASSERT(id1!=INVALID_ID);
	PX_ASSERT(mGroups);
	PX_ASSERT(mObjects);

	{
		const MBP_ObjectIndex index0 = decodeHandle_Index(id0);
		const MBP_ObjectIndex index1 = decodeHandle_Index(id1);

		const BpHandle object0 = mObjects[index0].mUserID;
		const BpHandle object1 = mObjects[index1].mUserID;

#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		if(!groupFiltering(mGroups[object0], mGroups[object1], mLUT))
#else
		if(!groupFiltering(mGroups[object0], mGroups[object1]))
#endif
			return NULL;
	}

	return addPairInternal(id0, id1);
}

///////////////////////////////////////////////////////////////////////////////

/*bool MBP_PairManager::removePair(PxU32 id0, PxU32 id1)
{
	// Order the ids
	sort(id0, id1);

	const PxU32 hashValue = hash(id0, id1) & mMask;
	const InternalPair* p = findPair(id0, id1, hashValue);
	if(!p)
		return false;
	PX_ASSERT(p->getId0()==id0);
	PX_ASSERT(p->getId1()==id1);

	PairManagerData::removePair(id0, id1, hashValue, getPairIndex(p));

	shrinkMemory();
	return true;
}*/

///////////////////////////////////////////////////////////////////////////////

#ifdef MBP_SIMD_OVERLAP
	#define SIMD_OVERLAP_PRELOAD_BOX0												\
	__m128i b = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&box0.mMinY));		\
	b = _mm_shuffle_epi32(b, 78);

	// PT: technically we don't need the 16 bits from _mm_movemask_epi8, we only
	// need the 4 bits from _mm_movemask_ps. Would it be faster? In any case this
	// works thanks to the _mm_cmpgt_epi32 which puts the same values in each byte
	// of each separate 32bits components.
	#define SIMD_OVERLAP_TEST(x)													\
	const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&x.mMinY));	\
	const __m128i d = _mm_cmpgt_epi32(a, b);										\
	const int mask = _mm_movemask_epi8(d);											\
	if(mask==0x0000ff00)

#else
	#define SIMD_OVERLAP_PRELOAD_BOX0
#endif

#ifdef MBP_USE_NO_CMP_OVERLAP
/*static PX_FORCE_INLINE void initBox(IAABB& box, const PxBounds3& src)
{
	box.initFrom2(src);
}*/
#else
static PX_FORCE_INLINE void initBox(IAABB& box, const PxBounds3& src)
{
	box.initFrom(src);
}
#endif

Region::Region() :
	mNbObjects				(0),
	mMaxNbObjects			(0),
	mFirstFree				(INVALID_ID),
	mObjects				(NULL),
	mMaxNbStaticBoxes		(0),
	mNbStaticBoxes			(0),
	mMaxNbDynamicBoxes		(0),
	mNbDynamicBoxes			(0),
	mStaticBoxes			(NULL),
	mDynamicBoxes			(NULL),
	mInToOut_Static			(NULL),
	mInToOut_Dynamic		(NULL),
	mPosList				(NULL),
	mNbUpdatedBoxes			(0),
	mPrevNbUpdatedBoxes		(0),
	mNeedsSorting			(false),
	mNeedsSortingSleeping	(true)
{
}

Region::~Region()
{
	DELETEARRAY(mObjects);
	MBP_FREE(mPosList);
	MBP_FREE(mInToOut_Dynamic);
	MBP_FREE(mInToOut_Static);
	DELETEARRAY(mDynamicBoxes);
	DELETEARRAY(mStaticBoxes);
}

// Pre-sort static boxes
#define STACK_BUFFER_SIZE_STATIC_SORT	8192
	#define DEFAULT_NUM_DYNAMIC_BOXES 1024

void Region::staticSort()
{
	// For now this version is only compatible with:
	// MBP_USE_WORDS
	// MBP_USE_SENTINELS

	mNeedsSorting = false;

	const PxU32 nbStaticBoxes = mNbStaticBoxes;
	if(!nbStaticBoxes)
	{
		mStaticBits.empty();
		return;
	}

//	PxU32 Time;
//	StartProfile(Time);

	// Roadmap:
	// - gather updated/modified static boxes
	// - sort those, and those only
	// - merge sorted set with previously existing (and previously sorted set)

	// Separate things-to-sort and things-already-sorted
	const PxU32 totalSize = sizeof(PxU32)*nbStaticBoxes*4;
	PxU8 stackBuffer[STACK_BUFFER_SIZE_STATIC_SORT];
	PxU8* tempMemory = totalSize<=STACK_BUFFER_SIZE_STATIC_SORT ? stackBuffer : reinterpret_cast<PxU8*>(MBP_ALLOC_TMP(totalSize));
	PxU32* minPosList_ToSort = reinterpret_cast<PxU32*>(tempMemory);
	PxU32* minPosList_Sorted = reinterpret_cast<PxU32*>(tempMemory + sizeof(PxU32)*nbStaticBoxes);
	PxU32* boxIndices_ToSort = reinterpret_cast<PxU32*>(tempMemory + sizeof(PxU32)*nbStaticBoxes*2);
	PxU32* boxIndices_Sorted = reinterpret_cast<PxU32*>(tempMemory + sizeof(PxU32)*nbStaticBoxes*3);
	PxU32 nbToSort = 0;
	PxU32 nbSorted = 0;
	for(PxU32 i=0;i<nbStaticBoxes;i++)
	{
		if(mStaticBits.isSetChecked(i))	// ### optimize check in that thing
		{
			minPosList_ToSort[nbToSort] = mStaticBoxes[i].mMinX;
			boxIndices_ToSort[nbToSort] = i;
			nbToSort++;
		}
		else
		{
			minPosList_Sorted[nbSorted] = mStaticBoxes[i].mMinX;
			boxIndices_Sorted[nbSorted] = i;
			PX_ASSERT(nbSorted==0 || minPosList_Sorted[nbSorted-1]<=minPosList_Sorted[nbSorted]);
			nbSorted++;
		}
	}
	PX_ASSERT(nbSorted+nbToSort==nbStaticBoxes);

//	EndProfile(Time);
//	printf("Part1: %d\n", Time);

//	StartProfile(Time);

	// Sort things that need sorting
	const PxU32* sorted;
	RadixSortBuffered RS;
	if(nbToSort<DEFAULT_NUM_DYNAMIC_BOXES)
	{
		sorted = mRS.Sort(minPosList_ToSort, nbToSort, RADIX_UNSIGNED).GetRanks();
	}
	else
	{
		sorted = RS.Sort(minPosList_ToSort, nbToSort, RADIX_UNSIGNED).GetRanks();
	}

//	EndProfile(Time);
//	printf("Part2: %d\n", Time);

//	StartProfile(Time);

	// Allocate final buffers that wil contain the 2 (merged) streams
	MBP_Index* newMapping = reinterpret_cast<MBP_Index*>(MBP_ALLOC(sizeof(MBP_Index)*mMaxNbStaticBoxes));
	const PxU32 nbStaticSentinels = 2;
	MBP_AABB* sortedBoxes = PX_NEW(MBP_AABB)[mMaxNbStaticBoxes+nbStaticSentinels];
	initSentinel(sortedBoxes[nbStaticBoxes]);
	initSentinel(sortedBoxes[nbStaticBoxes+1]);

//	EndProfile(Time);
//	printf("Part2b: %d\n", Time);

//	StartProfile(Time);

	// Merge streams to final buffers
	PxU32 offsetSorted = 0;
	PxU32 offsetNonSorted = 0;

	PxU32 nextCandidateNonSorted = offsetNonSorted<nbToSort ? minPosList_ToSort[sorted[offsetNonSorted]] : 0xffffffff;
	PxU32 nextCandidateSorted = offsetSorted<nbSorted ? minPosList_Sorted[offsetSorted] : 0xffffffff;

	for(PxU32 i=0;i<nbStaticBoxes;i++)
	{
		PxU32 boxIndex;
		{
//			minPosList_Sorted[offsetSorted] = mStaticBoxes[boxIndices_Sorted[offsetSorted]].mMinX;

			if(nextCandidateNonSorted<nextCandidateSorted)
			{
				boxIndex = boxIndices_ToSort[sorted[offsetNonSorted]];
				offsetNonSorted++;

				nextCandidateNonSorted = offsetNonSorted<nbToSort ? minPosList_ToSort[sorted[offsetNonSorted]] : 0xffffffff;
			}
			else
			{
				boxIndex = boxIndices_Sorted[offsetSorted];
				offsetSorted++;

				nextCandidateSorted = offsetSorted<nbSorted ? minPosList_Sorted[offsetSorted] : 0xffffffff;
			}
		}

		const MBP_Index OwnerIndex = mInToOut_Static[boxIndex];
		sortedBoxes[i] = mStaticBoxes[boxIndex];
		newMapping[i] = OwnerIndex;

		PX_ASSERT(mObjects[OwnerIndex].mIndex==boxIndex);
		PX_ASSERT(mObjects[OwnerIndex].isStatic());
		mObjects[OwnerIndex].mIndex = i;
	}
	PX_ASSERT(offsetSorted+offsetNonSorted==nbStaticBoxes);

//	EndProfile(Time);
//	printf("Part3: %d\n", Time);

//	StartProfile(Time);

	if(tempMemory!=stackBuffer)
		MBP_FREE(tempMemory);

	DELETEARRAY(mStaticBoxes);
	mStaticBoxes = sortedBoxes;

	MBP_FREE(mInToOut_Static);
	mInToOut_Static = newMapping;

	mStaticBits.empty();

//	EndProfile(Time);
//	printf("Part4: %d\n", Time);
}

void Region::optimizeMemory()
{
	// TODO: resize static boxes/mapping, dynamic boxes/mapping, object array
}

void Region::resizeObjects()
{
	const PxU32 newMaxNbOjects = mMaxNbObjects ? mMaxNbObjects + DEFAULT_NB_ENTRIES : DEFAULT_NB_ENTRIES;
//	const PxU32 newMaxNbOjects = mMaxNbObjects ? mMaxNbObjects*2 : DEFAULT_NB_ENTRIES;
	MBPEntry* newObjects = PX_NEW(MBPEntry)[newMaxNbOjects];
	if(mNbObjects)
		PxMemCopy(newObjects, mObjects, mNbObjects*sizeof(MBPEntry));
#if PX_DEBUG
	for(PxU32 i=mNbObjects;i<newMaxNbOjects;i++)
		newObjects[i].mUpdated = false;
#endif
	DELETEARRAY(mObjects);
	mObjects = newObjects;
	mMaxNbObjects = newMaxNbOjects;
}

static MBP_AABB* resizeBoxes(PxU32 oldNbBoxes, PxU32 newNbBoxes, const MBP_AABB* boxes)
{
	MBP_AABB* newBoxes = PX_NEW(MBP_AABB)[newNbBoxes];
	if(oldNbBoxes)
		PxMemCopy(newBoxes, boxes, oldNbBoxes*sizeof(MBP_AABB));
	DELETEARRAY(boxes);
	return newBoxes;
}

static MBP_Index* resizeMapping(PxU32 oldNbBoxes, PxU32 newNbBoxes, MBP_Index* mapping)
{
	MBP_Index* newMapping = reinterpret_cast<MBP_Index*>(MBP_ALLOC(sizeof(MBP_Index)*newNbBoxes));
	if(oldNbBoxes)
		PxMemCopy(newMapping, mapping, oldNbBoxes*sizeof(MBP_Index));
	MBP_FREE(mapping);
	return newMapping;
}

static PX_FORCE_INLINE void MTF(MBP_AABB* PX_RESTRICT dynamicBoxes, MBP_Index* PX_RESTRICT inToOut_Dynamic, MBPEntry* PX_RESTRICT objects, const MBP_AABB& bounds, PxU32 frontIndex, MBPEntry& updatedObject)
{
	const PxU32 updatedIndex = updatedObject.mIndex;
	if(frontIndex!=updatedIndex)
	{
		const MBP_AABB box0 = dynamicBoxes[frontIndex];
		dynamicBoxes[frontIndex] = bounds;
		dynamicBoxes[updatedIndex] = box0;

		const MBP_Index index0 = inToOut_Dynamic[frontIndex];
		inToOut_Dynamic[frontIndex] = inToOut_Dynamic[updatedIndex];
		inToOut_Dynamic[updatedIndex] = index0;

		objects[index0].mIndex = updatedIndex;
		updatedObject.mIndex = frontIndex;
	}
	else
	{
		dynamicBoxes[frontIndex] = bounds;
	}
}

MBP_Index Region::addObject(const MBP_AABB& bounds, MBP_Handle mbpHandle, bool isStatic)
{
#ifdef MBP_USE_WORDS
	PX_ASSERT(mNbObjects<0xffff);
#endif
	PX_ASSERT((decodeHandle_IsStatic(mbpHandle) && isStatic) || (!decodeHandle_IsStatic(mbpHandle) && !isStatic));

	MBP_Index handle;
	if(mFirstFree!=INVALID_ID)
	{
		handle = MBP_Index(mFirstFree);
		mFirstFree = mObjects[handle].mIndex;
	}
	else
	{
		if(mMaxNbObjects==mNbObjects)
			resizeObjects();

		handle = MBP_Index(mNbObjects);
	}
	mNbObjects++;
	///

	PxU32 boxIndex;
	if(isStatic)
	{
		if(mMaxNbStaticBoxes==mNbStaticBoxes)
		{
			const PxU32 newMaxNbBoxes = mMaxNbStaticBoxes ? mMaxNbStaticBoxes + DEFAULT_NB_ENTRIES : DEFAULT_NB_ENTRIES;
//			const PxU32 newMaxNbBoxes = mMaxNbStaticBoxes ? mMaxNbStaticBoxes*2 : DEFAULT_NB_ENTRIES;
			mStaticBoxes = resizeBoxes(mNbStaticBoxes, newMaxNbBoxes, mStaticBoxes);
			mInToOut_Static = resizeMapping(mNbStaticBoxes, newMaxNbBoxes, mInToOut_Static);
			mMaxNbStaticBoxes = newMaxNbBoxes;
		}

		boxIndex = mNbStaticBoxes++;
		mStaticBoxes[boxIndex] = bounds;
		mInToOut_Static[boxIndex] = handle;
		mNeedsSorting = true;
		mStaticBits.setBitChecked(boxIndex);
	}
	else
	{
		if(mMaxNbDynamicBoxes==mNbDynamicBoxes)
		{
			const PxU32 newMaxNbBoxes = mMaxNbDynamicBoxes ? mMaxNbDynamicBoxes + DEFAULT_NB_ENTRIES : DEFAULT_NB_ENTRIES;
//			const PxU32 newMaxNbBoxes = mMaxNbDynamicBoxes ? mMaxNbDynamicBoxes*2 : DEFAULT_NB_ENTRIES;
			mDynamicBoxes = resizeBoxes(mNbDynamicBoxes, newMaxNbBoxes, mDynamicBoxes);
			mInToOut_Dynamic = resizeMapping(mNbDynamicBoxes, newMaxNbBoxes, mInToOut_Dynamic);
			mMaxNbDynamicBoxes = newMaxNbBoxes;

			MBP_FREE(mPosList);
			mPosList = reinterpret_cast<PxU32*>(MBP_ALLOC((newMaxNbBoxes+1)*sizeof(PxU32)));
		}

		boxIndex = mNbDynamicBoxes++;
		mDynamicBoxes[boxIndex] = bounds;
		mInToOut_Dynamic[boxIndex] = handle;
	}

	mObjects[handle].mIndex = boxIndex;
	mObjects[handle].mMBPHandle = mbpHandle;
#if PX_DEBUG
	mObjects[handle].mUpdated = !isStatic;
#endif

	if(!isStatic)
	{
		MTF(mDynamicBoxes, mInToOut_Dynamic, mObjects, bounds, mNbUpdatedBoxes, mObjects[handle]);
		mNbUpdatedBoxes++;
		mPrevNbUpdatedBoxes = 0;
		mNeedsSortingSleeping = true;
		PX_ASSERT(mNbUpdatedBoxes<=mNbDynamicBoxes);
	}
	return handle;
}

// Moves box 'lastIndex' to location 'removedBoxIndex'
static PX_FORCE_INLINE void remove(MBPEntry* PX_RESTRICT objects, MBP_Index* PX_RESTRICT mapping, MBP_AABB* PX_RESTRICT boxes, PxU32 removedBoxIndex, PxU32 lastIndex)
{
	const PxU32 movedBoxHandle = mapping[lastIndex];
	boxes[removedBoxIndex] = boxes[lastIndex];				// Relocate box data
	mapping[removedBoxIndex] = MBP_Index(movedBoxHandle);	// Relocate mapping data
	MBPEntry& movedObject = objects[movedBoxHandle];
	PX_ASSERT(movedObject.mIndex==lastIndex);				// Checks index of moved box was indeed its old location
	movedObject.mIndex = removedBoxIndex;					// Adjust index of moved box to reflect its new location
}

void Region::removeObject(MBP_Index handle)
{
	PX_ASSERT(handle<mMaxNbObjects);

	MBPEntry& object = mObjects[handle];
	/*const*/ PxU32 removedBoxIndex = object.mIndex;

	MBP_Index* PX_RESTRICT mapping;
	MBP_AABB* PX_RESTRICT boxes;
	PxU32 lastIndex;
	if(!object.isStatic())
	{
		mPrevNbUpdatedBoxes = 0;
		mNeedsSortingSleeping = true;

		PX_ASSERT(mInToOut_Dynamic[removedBoxIndex]==handle);
		const bool isUpdated = removedBoxIndex<mNbUpdatedBoxes;
		PX_ASSERT(isUpdated==object.mUpdated);
		if(isUpdated)
		{
			PX_ASSERT(mNbUpdatedBoxes);
			if(mNbUpdatedBoxes!=mNbDynamicBoxes)
			{
				// Removing the object will create this pattern, which is wrong:
				// UUUUUUUUUUUNNNNNNNNN......... original
				// UUUUUU.UUUUNNNNNNNNN......... remove U
				// UUUUUUNUUUUNNNNNNNN.......... move N
				//
				// What we want instead is:
				// UUUUUUUUUUUNNNNNNNNN......... original
				// UUUUUU.UUUUNNNNNNNNN......... remove U
				// UUUUUUUUUU.NNNNNNNNN......... move U
				// UUUUUUUUUUNNNNNNNNN.......... move N
				const PxU32 lastUpdatedIndex = mNbUpdatedBoxes-1;

				remove(mObjects, mInToOut_Dynamic, mDynamicBoxes, removedBoxIndex, lastUpdatedIndex);	// Move last U to removed U
				//Remove(mObjects, mInToOut_Dynamic, mDynamicBoxes, lastUpdatedIndex, --mNbDynamicBoxes);	// Move last N to last U
				removedBoxIndex = lastUpdatedIndex;
			}
			mNbUpdatedBoxes--;
		}

//		remove(mObjects, mInToOut_Dynamic, mDynamicBoxes, removedBoxIndex, --mNbDynamicBoxes);
		mapping = mInToOut_Dynamic;
		boxes = mDynamicBoxes;
		lastIndex = --mNbDynamicBoxes;

		// ### adjust size of mPosList ?
	}
	else
	{
		PX_ASSERT(mInToOut_Static[removedBoxIndex]==handle);

		mNeedsSorting = true;
		mStaticBits.setBitChecked(removedBoxIndex);

//		remove(mObjects, mInToOut_Static, mStaticBoxes, removedBoxIndex, --mNbStaticBoxes);
		mapping = mInToOut_Static;
		boxes = mStaticBoxes;
		lastIndex = --mNbStaticBoxes;
	}
	remove(mObjects, mapping, boxes, removedBoxIndex, lastIndex);

	object.mIndex		= mFirstFree;
	object.mMBPHandle	= INVALID_ID;
//	printf("Invalid: %d\n", handle);
	mFirstFree			= handle;
	mNbObjects--;

#if PX_DEBUG
	object.mUpdated = false;
#endif
}

void Region::updateObject(const MBP_AABB& bounds, MBP_Index handle)
{
	PX_ASSERT(handle<mMaxNbObjects);

	MBPEntry& object = mObjects[handle];
	if(!object.isStatic())
	{
		// MTF on updated box
		const bool isContinuouslyUpdated = object.mIndex<mPrevNbUpdatedBoxes;
		if(!isContinuouslyUpdated)
			mNeedsSortingSleeping = true;
//		printf("%d: %d\n", handle, isContinuouslyUpdated);

		const bool isUpdated = object.mIndex<mNbUpdatedBoxes;
		PX_ASSERT(isUpdated==object.mUpdated);
		if(!isUpdated)
		{
#if PX_DEBUG
			object.mUpdated = true;
#endif
			MTF(mDynamicBoxes, mInToOut_Dynamic, mObjects, bounds, mNbUpdatedBoxes, object);
			mNbUpdatedBoxes++;
			PX_ASSERT(mNbUpdatedBoxes<=mNbDynamicBoxes);
		}
		else
		{
			mDynamicBoxes[object.mIndex] = bounds;
		}
	}
	else
	{
		mStaticBoxes[object.mIndex] = bounds;
		mNeedsSorting = true;	// ### not always!
		mStaticBits.setBitChecked(object.mIndex);
	}
}

MBP_Handle Region::retrieveBounds(MBP_AABB& bounds, MBP_Index handle) const
{
	PX_ASSERT(handle<mMaxNbObjects);

	const MBPEntry& object = mObjects[handle];
	if(!object.isStatic())
		bounds = mDynamicBoxes[object.mIndex];
	else
		bounds = mStaticBoxes[object.mIndex];

	return object.mMBPHandle;
}

void Region::setBounds(MBP_Index handle, const MBP_AABB& bounds)
{
	PX_ASSERT(handle<mMaxNbObjects);

	const MBPEntry& object = mObjects[handle];
	if(!object.isStatic())
	{
		PX_ASSERT(object.mIndex < mNbDynamicBoxes);
		mDynamicBoxes[object.mIndex] = bounds;
	}
	else
	{
		PX_ASSERT(object.mIndex < mNbStaticBoxes);
		mStaticBoxes[object.mIndex] = bounds;
	}
}

#ifndef MBP_SIMD_OVERLAP
static PX_FORCE_INLINE Ps::IntBool intersect2D(const MBP_AABB& a, const MBP_AABB& b)
{
#ifdef MBP_USE_NO_CMP_OVERLAP
		// PT: warning, only valid with the special encoding in InitFrom2
		const PxU32 bits0 = (b.mMaxY - a.mMinY)&0x80000000;
		const PxU32 bits1 = (b.mMaxZ - a.mMinZ)&0x80000000;
		const PxU32 bits2 = (a.mMaxY - b.mMinY)&0x80000000;
		const PxU32 bits3 = (a.mMaxZ - b.mMinZ)&0x80000000;
		const PxU32 mask = bits0|(bits1>>1)|(bits2>>2)|(bits3>>3);
		return !mask;

	/*	const PxU32 d0 = (b.mMaxY<<16)|a.mMaxY;
		const PxU32 d0b = (b.mMaxZ<<16)|a.mMaxZ;
		const PxU32 d1 = (a.mMinY<<16)|b.mMinY;
		const PxU32 d1b = (a.mMinZ<<16)|b.mMinZ;
		const PxU32 mask = (d0 - d1) | (d0b - d1b);
		return !(mask & 0x80008000);*/
#else
	if(//mMaxX < a.mMinX || a.mMaxX < mMinX
//		||
		b.mMaxY < a.mMinY || a.mMaxY < b.mMinY
	||
		b.mMaxZ < a.mMinZ || a.mMaxZ < b.mMinZ
	)
		return FALSE;
	return TRUE;
#endif
}
#endif

#ifdef MBP_USE_NO_CMP_OVERLAP_3D
static PX_FORCE_INLINE bool intersect3D(const MBP_AABB& a, const MBP_AABB& b)
{
	// PT: warning, only valid with the special encoding in InitFrom2
	const PxU32 bits0 = (b.mMaxY - a.mMinY)&0x80000000;
	const PxU32 bits1 = (b.mMaxZ - a.mMinZ)&0x80000000;
	const PxU32 bits2 = (a.mMaxY - b.mMinY)&0x80000000;
	const PxU32 bits3 = (a.mMaxZ - b.mMinZ)&0x80000000;
	const PxU32 bits4 = (b.mMaxX - a.mMinX)&0x80000000;
	const PxU32 bits5 = (a.mMaxX - b.mMinX)&0x80000000;
	const PxU32 mask = bits0|(bits1>>1)|(bits2>>2)|(bits3>>3)|(bits4>>4)|(bits5>>5);
	return !mask;
}
#endif

#ifdef CHECK_NB_OVERLAPS
static PxU32 gNbOverlaps = 0;
#endif

static PX_FORCE_INLINE void outputPair(	MBP_PairManager& pairManager,
										PxU32 index0, PxU32 index1,
										const MBP_Index* PX_RESTRICT inToOut0, const MBP_Index* PX_RESTRICT inToOut1,
										const MBPEntry* PX_RESTRICT objects)
{
#ifdef CHECK_NB_OVERLAPS
	gNbOverlaps++;
#endif
	const MBP_Index objectIndex0 = inToOut0[index0];
	const MBP_Index objectIndex1 = inToOut1[index1];
	PX_ASSERT(objectIndex0!=objectIndex1);
	const MBP_Handle id0 = objects[objectIndex0].mMBPHandle;
	const MBP_Handle id1 = objects[objectIndex1].mMBPHandle;
//	printf("2: %d %d\n", index0, index1);
//	printf("3: %d %d\n", objectIndex0, objectIndex1);
	pairManager.addPair(id0, id1);
}

MBPOS_TmpBuffers::MBPOS_TmpBuffers() :
	mNbSleeping					(0),
	mNbUpdated					(0),
	mInToOut_Dynamic_Sleeping	(NULL),
	mSleepingDynamicBoxes		(NULL),
	mUpdatedDynamicBoxes		(NULL)
{
}

MBPOS_TmpBuffers::~MBPOS_TmpBuffers()
{
//	printf("mNbSleeping: %d\n", mNbSleeping);
	if(mInToOut_Dynamic_Sleeping!=mInToOut_Dynamic_Sleeping_Stack)
		MBP_FREE(mInToOut_Dynamic_Sleeping);

	if(mSleepingDynamicBoxes!=mSleepingDynamicBoxes_Stack)
		DELETEARRAY(mSleepingDynamicBoxes);

	if(mUpdatedDynamicBoxes!=mUpdatedDynamicBoxes_Stack)
		DELETEARRAY(mUpdatedDynamicBoxes);

	mNbSleeping = 0;
	mNbUpdated = 0;
}

void MBPOS_TmpBuffers::allocateSleeping(PxU32 nbSleeping, PxU32 nbSentinels)
{
	if(nbSleeping>mNbSleeping)
	{
		if(mInToOut_Dynamic_Sleeping!=mInToOut_Dynamic_Sleeping_Stack)
			MBP_FREE(mInToOut_Dynamic_Sleeping);
		if(mSleepingDynamicBoxes!=mSleepingDynamicBoxes_Stack)
			DELETEARRAY(mSleepingDynamicBoxes);

		if(nbSleeping+nbSentinels<=STACK_BUFFER_SIZE)
		{
			mSleepingDynamicBoxes = mSleepingDynamicBoxes_Stack;
			mInToOut_Dynamic_Sleeping = mInToOut_Dynamic_Sleeping_Stack;
		}
		else
		{
			mSleepingDynamicBoxes = PX_NEW_TEMP(MBP_AABB)[nbSleeping+nbSentinels];
			mInToOut_Dynamic_Sleeping = reinterpret_cast<MBP_Index*>(MBP_ALLOC(sizeof(MBP_Index)*nbSleeping));
		}
		mNbSleeping = nbSleeping;
	}
}

void MBPOS_TmpBuffers::allocateUpdated(PxU32 nbUpdated, PxU32 nbSentinels)
{
	if(nbUpdated>mNbUpdated)
	{
		if(mUpdatedDynamicBoxes!=mUpdatedDynamicBoxes_Stack)
			DELETEARRAY(mUpdatedDynamicBoxes);

		if(nbUpdated+nbSentinels<=STACK_BUFFER_SIZE)
			mUpdatedDynamicBoxes = mUpdatedDynamicBoxes_Stack;
		else
			mUpdatedDynamicBoxes = PX_NEW_TEMP(MBP_AABB)[nbUpdated+nbSentinels];

		mNbUpdated = nbUpdated;
	}
}

void Region::preparePruning(MBPOS_TmpBuffers& buffers)
{
PxU32 _saved = mNbUpdatedBoxes;
mNbUpdatedBoxes = 0;

	if(mPrevNbUpdatedBoxes!=_saved)
		mNeedsSortingSleeping = true;

	PxU32 nb = mNbDynamicBoxes;
	if(!nb)
	{
		mInput.mNeeded = false;
		mPrevNbUpdatedBoxes = 0;
		mNeedsSortingSleeping = true;
		return;
	}
	const MBP_AABB* PX_RESTRICT dynamicBoxes = mDynamicBoxes;
	PxU32* PX_RESTRICT posList = mPosList;

#if PX_DEBUG
	PxU32 verifyNbUpdated = 0;
	for(PxU32 i=0;i<mMaxNbObjects;i++)
	{
		if(mObjects[i].mUpdated)
			verifyNbUpdated++;
	}
	PX_ASSERT(verifyNbUpdated==_saved);
#endif

	// Build main list using the primary axis

	PxU32 nbUpdated = 0;
	PxU32 nbNonUpdated = 0;
	{
		nbUpdated = _saved;
		nbNonUpdated = nb - _saved;
		for(PxU32 i=0;i<nbUpdated;i++)
		{
#if PX_DEBUG
			const PxU32 objectIndex = mInToOut_Dynamic[i];
			PX_ASSERT(mObjects[objectIndex].mUpdated);
			mObjects[objectIndex].mUpdated = false;
#endif
			posList[i] = dynamicBoxes[i].mMinX;
		}
		if(mNeedsSortingSleeping)
		{
			for(PxU32 i=0;i<nbNonUpdated;i++)
			{
#if PX_DEBUG
				const PxU32 objectIndex = mInToOut_Dynamic[i];
				PX_ASSERT(!mObjects[objectIndex].mUpdated);
#endif
				PxU32 j = i + nbUpdated;
				posList[j] = dynamicBoxes[j].mMinX;
			}
		}
#if PX_DEBUG
		else
		{
			for(PxU32 i=0;i<nbNonUpdated;i++)
			{
				const PxU32 objectIndex = mInToOut_Dynamic[i];
				PX_ASSERT(!mObjects[objectIndex].mUpdated);
				PxU32 j = i + nbUpdated;
				PX_ASSERT(posList[j] == dynamicBoxes[j].mMinX);
			}
		}
#endif
	}
	PX_ASSERT(nbUpdated==verifyNbUpdated);
	PX_ASSERT(nbUpdated+nbNonUpdated==nb);
	mNbUpdatedBoxes = nbUpdated;
	if(!nbUpdated)
	{
		mInput.mNeeded = false;
		mPrevNbUpdatedBoxes = 0;
		mNeedsSortingSleeping = true;
		return;
	}

	mPrevNbUpdatedBoxes = mNbUpdatedBoxes;

	///////

	// ### TODO: no need to recreate those buffers each frame!
	MBP_Index* PX_RESTRICT inToOut_Dynamic_Sleeping = NULL;
	MBP_AABB* PX_RESTRICT sleepingDynamicBoxes = NULL;
	if(nbNonUpdated)
	{
		if(mNeedsSortingSleeping)
		{
			const PxU32* PX_RESTRICT sorted = mRS.Sort(posList+nbUpdated, nbNonUpdated, RADIX_UNSIGNED).GetRanks();

			const PxU32 nbSentinels = 2;
			buffers.allocateSleeping(nbNonUpdated, nbSentinels);
			sleepingDynamicBoxes = buffers.mSleepingDynamicBoxes;
			inToOut_Dynamic_Sleeping = buffers.mInToOut_Dynamic_Sleeping;
			for(PxU32 i=0;i<nbNonUpdated;i++)
			{
				const PxU32 sortedIndex = nbUpdated+sorted[i];
				sleepingDynamicBoxes[i] = dynamicBoxes[sortedIndex];
				inToOut_Dynamic_Sleeping[i] = mInToOut_Dynamic[sortedIndex];
			}
			initSentinel(sleepingDynamicBoxes[nbNonUpdated]);
			initSentinel(sleepingDynamicBoxes[nbNonUpdated+1]);
			mNeedsSortingSleeping = false;
		}
		else
		{
			sleepingDynamicBoxes = buffers.mSleepingDynamicBoxes;
			inToOut_Dynamic_Sleeping = buffers.mInToOut_Dynamic_Sleeping;
#if PX_DEBUG
			for(PxU32 i=0;i<nbNonUpdated-1;i++)
				PX_ASSERT(sleepingDynamicBoxes[i].mMinX<=sleepingDynamicBoxes[i+1].mMinX);
#endif
		}
	}
	else
	{
		mNeedsSortingSleeping = true;
	}

	///////

//	posList[nbUpdated] = MAX_PxU32;
//	nb = nbUpdated;

	// Sort the list
//	const PxU32* PX_RESTRICT sorted = mRS.Sort(posList, nbUpdated+1, RADIX_UNSIGNED).GetRanks();
	const PxU32* PX_RESTRICT sorted = mRS.Sort(posList, nbUpdated, RADIX_UNSIGNED).GetRanks();

	const PxU32 nbSentinels = 2;
	buffers.allocateUpdated(nbUpdated, nbSentinels);
	MBP_AABB* PX_RESTRICT updatedDynamicBoxes = buffers.mUpdatedDynamicBoxes;
	MBP_Index* PX_RESTRICT inToOut_Dynamic = reinterpret_cast<MBP_Index*>(mRS.GetRecyclable());
	for(PxU32 i=0;i<nbUpdated;i++)
	{
		const PxU32 sortedIndex = sorted[i];
		updatedDynamicBoxes[i] = dynamicBoxes[sortedIndex];
		inToOut_Dynamic[i] = mInToOut_Dynamic[sortedIndex];
	}
	initSentinel(updatedDynamicBoxes[nbUpdated]);
	initSentinel(updatedDynamicBoxes[nbUpdated+1]);
	dynamicBoxes = updatedDynamicBoxes;

	mInput.mObjects						= mObjects;					// Can be shared (1)
	mInput.mUpdatedDynamicBoxes			= updatedDynamicBoxes;		// Can be shared (2) => buffers.mUpdatedDynamicBoxes;
	mInput.mSleepingDynamicBoxes		= sleepingDynamicBoxes;
	mInput.mInToOut_Dynamic				= inToOut_Dynamic;			// Can be shared (3) => (MBP_Index*)mRS.GetRecyclable();
	mInput.mInToOut_Dynamic_Sleeping	= inToOut_Dynamic_Sleeping;
	mInput.mNbUpdated					= nbUpdated;				// Can be shared (4)
	mInput.mNbNonUpdated				= nbNonUpdated;
	mInput.mNeeded						= true;
}

void Region::prepareBIPPruning(const MBPOS_TmpBuffers& buffers)
{
	if(!mNbUpdatedBoxes || !mNbStaticBoxes)
	{
		mInput.mBIPInput.mNeeded = false;
		return;
	}

	mInput.mBIPInput.mObjects			= mObjects;						// Can be shared (1)
	mInput.mBIPInput.mNbUpdatedBoxes	= mNbUpdatedBoxes;				// Can be shared (4)
	mInput.mBIPInput.mNbStaticBoxes		= mNbStaticBoxes;
//	mInput.mBIPInput.mDynamicBoxes		= mDynamicBoxes;
	mInput.mBIPInput.mDynamicBoxes		= buffers.mUpdatedDynamicBoxes;	// Can be shared (2)
	mInput.mBIPInput.mStaticBoxes		= mStaticBoxes;
	mInput.mBIPInput.mInToOut_Static	= mInToOut_Static;
	mInput.mBIPInput.mInToOut_Dynamic	= reinterpret_cast<const MBP_Index*>(mRS.GetRecyclable());	// Can be shared (3)
	mInput.mBIPInput.mNeeded			= true;
}

static void doCompleteBoxPruning(MBP_PairManager* PX_RESTRICT pairManager, const BoxPruning_Input& input)
{
	const MBPEntry* PX_RESTRICT objects						= input.mObjects;
	const MBP_AABB* PX_RESTRICT updatedDynamicBoxes			= input.mUpdatedDynamicBoxes;
	const MBP_AABB* PX_RESTRICT sleepingDynamicBoxes		= input.mSleepingDynamicBoxes;
	const MBP_Index* PX_RESTRICT inToOut_Dynamic			= input.mInToOut_Dynamic;
	const MBP_Index* PX_RESTRICT inToOut_Dynamic_Sleeping	= input.mInToOut_Dynamic_Sleeping;
	const PxU32 nbUpdated 									= input.mNbUpdated;
	const PxU32 nbNonUpdated								= input.mNbNonUpdated;

	//

	// PT: find sleeping-dynamics-vs-active-dynamics overlaps
	if(nbNonUpdated)
	{
		const PxU32 nb0 = nbUpdated;
		const PxU32 nb1 = nbNonUpdated;

		//
		PxU32 index0 = 0;
		PxU32 runningIndex1 = 0;

		while(runningIndex1<nb1 && index0<nb0)
		{
			const MBP_AABB& box0 = updatedDynamicBoxes[index0];
			const PxU32 limit = box0.mMaxX;
			SIMD_OVERLAP_PRELOAD_BOX0

			const PxU32 l = box0.mMinX;
			while(sleepingDynamicBoxes[runningIndex1].mMinX<l)
				runningIndex1++;

			PxU32 index1 = runningIndex1;

			while(sleepingDynamicBoxes[index1].mMinX<=limit)
			{
				MBP_OVERLAP_TEST(sleepingDynamicBoxes[index1])
				{
					outputPair(*pairManager, index0, index1, inToOut_Dynamic, inToOut_Dynamic_Sleeping, objects);
				}
				index1++;
			}
			index0++;
		}

		////

		index0 = 0;
		PxU32 runningIndex0 = 0;
		while(runningIndex0<nb0 && index0<nb1)
		{
			const MBP_AABB& box0 = sleepingDynamicBoxes[index0];
			const PxU32 limit = box0.mMaxX;
			SIMD_OVERLAP_PRELOAD_BOX0

			const PxU32 l = box0.mMinX;
			while(updatedDynamicBoxes[runningIndex0].mMinX<=l)
				runningIndex0++;

			PxU32 index1 = runningIndex0;

			while(updatedDynamicBoxes[index1].mMinX<=limit)
			{
				MBP_OVERLAP_TEST(updatedDynamicBoxes[index1])
				{
					outputPair(*pairManager, index1, index0, inToOut_Dynamic, inToOut_Dynamic_Sleeping, objects);
				}
				index1++;
			}
			index0++;
		}
	}

	///////

	// PT: find active-dynamics-vs-active-dynamics overlaps

	PxU32 index0 = 0;
	PxU32 runningIndex = 0;
	while(runningIndex<nbUpdated && index0<nbUpdated)
	{
		const MBP_AABB& box0 = updatedDynamicBoxes[index0];
		const PxU32 limit = box0.mMaxX;

		SIMD_OVERLAP_PRELOAD_BOX0

		const PxU32 l = box0.mMinX;
		while(updatedDynamicBoxes[runningIndex++].mMinX<l);

		if(runningIndex<nbUpdated)
		{
			PxU32 index1 = runningIndex;
			while(updatedDynamicBoxes[index1].mMinX<=limit)
			{
				MBP_OVERLAP_TEST(updatedDynamicBoxes[index1])
				{
					outputPair(*pairManager, index0, index1, inToOut_Dynamic, inToOut_Dynamic, objects);
				}
				index1++;
			}
		}
		index0++;
	}
}

static void doBipartiteBoxPruning(MBP_PairManager* PX_RESTRICT pairManager, const BIP_Input& input)
{
	// ### crashes because the code expects the dynamic array to be sorted, but mDynamicBoxes is not
	// ### we should instead modify mNbUpdatedBoxes so that mNbUpdatedBoxes == mNbDynamicBoxes, and
	// ### then the proper sorting happens in CompleteBoxPruning (right?)

	const PxU32 nb0 = input.mNbUpdatedBoxes;
	const PxU32 nb1 = input.mNbStaticBoxes;

	const MBPEntry* PX_RESTRICT mObjects			= input.mObjects;
	const MBP_AABB* PX_RESTRICT dynamicBoxes		= input.mDynamicBoxes;
	const MBP_AABB* PX_RESTRICT staticBoxes			= input.mStaticBoxes;
	const MBP_Index* PX_RESTRICT inToOut_Static		= input.mInToOut_Static;
	const MBP_Index* PX_RESTRICT inToOut_Dynamic	= input.mInToOut_Dynamic;

	PX_ASSERT(isSentinel(staticBoxes[nb1]));
	PX_ASSERT(isSentinel(staticBoxes[nb1+1]));
//	const MBP_AABB Saved = staticBoxes[nb1];
//	const MBP_AABB Saved1 = staticBoxes[nb1+1];
//	initSentinel(((MBP_AABB* PX_RESTRICT)staticBoxes)[nb1]);
//	initSentinel(((MBP_AABB* PX_RESTRICT)staticBoxes)[nb1+1]);

	//
	PxU32 index0 = 0;
	PxU32 runningIndex1 = 0;

	while(runningIndex1<nb1 && index0<nb0)
	{
		const MBP_AABB& box0 = dynamicBoxes[index0];
		const PxU32 limit = box0.mMaxX;
		SIMD_OVERLAP_PRELOAD_BOX0

		const PxU32 l = box0.mMinX;
		while(staticBoxes[runningIndex1].mMinX<l)
			runningIndex1++;

		PxU32 index1 = runningIndex1;

		while(staticBoxes[index1].mMinX<=limit)
		{
			MBP_OVERLAP_TEST(staticBoxes[index1])
			{
				outputPair(*pairManager, index0, index1, inToOut_Dynamic, inToOut_Static, mObjects);
			}
			index1++;
		}
		index0++;
	}

	////

	index0 = 0;
	PxU32 runningIndex0 = 0;
	while(runningIndex0<nb0 && index0<nb1)
	{
		const MBP_AABB& box0 = staticBoxes[index0];
		const PxU32 limit = box0.mMaxX;
		SIMD_OVERLAP_PRELOAD_BOX0

		const PxU32 l = box0.mMinX;
		while(dynamicBoxes[runningIndex0].mMinX<=l)
			runningIndex0++;

		PxU32 index1 = runningIndex0;

		while(dynamicBoxes[index1].mMinX<=limit)
		{
			MBP_OVERLAP_TEST(dynamicBoxes[index1])
			{
				outputPair(*pairManager, index1, index0, inToOut_Dynamic, inToOut_Static, mObjects);
			}
			index1++;
		}
		index0++;
	}

//	MBP_FREE(inToOut_Dynamic);

//	((MBP_AABB* PX_RESTRICT)staticBoxes)[nb1] = Saved;
//	((MBP_AABB* PX_RESTRICT)staticBoxes)[nb1+1] = Saved1;
}

void Region::prepareOverlaps()
{
	if(!mNbUpdatedBoxes && !mNeedsSorting)
		return;

	if(mNeedsSorting)
	{
		staticSort();

		// PT: when a static object is added/removed/updated we need to compute the overlaps again
		// even if no dynamic box has been updated. The line below forces all dynamic boxes to be
		// sorted in PreparePruning() and tested for overlaps in BipartiteBoxPruning(). It would be
		// more efficient to:
		// a) skip the actual pruning in PreparePruning() (we only need to re-sort)
		// b) do BipartiteBoxPruning() with the new/modified boxes, not all of them
		// Well, not done yet.
		mNbUpdatedBoxes = mNbDynamicBoxes;
		mPrevNbUpdatedBoxes = 0;
		mNeedsSortingSleeping = true;
#if PX_DEBUG
		for(PxU32 i=0;i<mNbDynamicBoxes;i++)
		{
			const PxU32 objectIndex = mInToOut_Dynamic[i];
			mObjects[objectIndex].mUpdated = true;
		}
#endif
	}

	preparePruning(mTmpBuffers);
	prepareBIPPruning(mTmpBuffers);
}

void Region::findOverlaps(MBP_PairManager& pairManager)
{
	PX_ASSERT(!mNeedsSorting);
	if(!mNbUpdatedBoxes)
		return;

	if(mInput.mNeeded)
		doCompleteBoxPruning(&pairManager, mInput);

	if(mInput.mBIPInput.mNeeded)
		doBipartiteBoxPruning(&pairManager, mInput.mBIPInput);

	mNbUpdatedBoxes = 0;
}

///////////////////////////////////////////////////////////////////////////

MBP::MBP() :
	mNbRegions			(0),
	mFirstFreeIndex		(INVALID_ID),
	mFirstFreeIndexBP	(INVALID_ID)
#ifdef MBP_REGION_BOX_PRUNING
	,mNbActiveRegions	(0),
	mDirtyRegions		(true)
#endif
{
	for(PxU32 i=0;i<MAX_NB_MBP+1;i++)
		mFirstFree[i] = INVALID_ID;
}

MBP::~MBP()
{
/*	for(PxU32 i=1;i<MAX_NB_MBP;i++)
	{
		if(mHandles[i].GetNbEntries())
		{
			const PxU32 SizeOfBundle = sizeof(RegionHandle)*i;
//			printf("Handles %d: %d\n", i, mHandles[i].GetNbEntries()*sizeof(PxU32)/SizeOfBundle);
		}
	}*/

	reset();
}

void MBP::freeBuffers()
{
	mRemoved.empty();
	mOutOfBoundsObjects.clear();
}

void MBP::preallocate(PxU32 nbRegions, PxU32 nbObjects, PxU32 maxNbOverlaps)
{
	if(nbRegions)
	{
		mRegions.clear();
		mRegions.reserve(nbRegions);
	}

	if(nbObjects)
	{
		mMBP_Objects.clear();
		mMBP_Objects.reserve(nbObjects);
#ifdef USE_FULLY_INSIDE_FLAG
		mFullyInsideBitmap.init(nbObjects);
		mFullyInsideBitmap.clearAll();
#endif
	}

	mPairManager.reserveMemory(maxNbOverlaps);
}

PX_COMPILE_TIME_ASSERT(sizeof(BpHandle)<=sizeof(PxU32));
void MBP::addToOutOfBoundsArray(BpHandle id)
{
	PX_ASSERT(mOutOfBoundsObjects.find(PxU32(id)) == mOutOfBoundsObjects.end());
	mOutOfBoundsObjects.pushBack(PxU32(id));
}

static void setupOverlapFlags(PxU32 nbRegions, RegionData* PX_RESTRICT regions)
{
	for(PxU32 i=0;i<nbRegions;i++)
		regions[i].mOverlap = false;

	for(PxU32 i=0;i<nbRegions;i++)
	{
		if(!regions[i].mBP)
			continue;

		for(PxU32 j=i+1;j<nbRegions;j++)
		{
			if(!regions[j].mBP)
				continue;

			if(regions[i].mBox.intersectNoTouch(regions[j].mBox))	
			{
				regions[i].mOverlap = true;
				regions[j].mOverlap = true;
			}
		}
	}
}

//#define PRINT_STATS
#ifdef PRINT_STATS
#include <stdio.h>
#endif

// PT: TODO:
// - We could try to keep bounds around all objects (for each region), and then test the new region's bounds against these instead of
//   testing all objects one by one. These new bounds (around all objects of a given region) would be delicate to maintain though.
// - Just store these "fully inside flags" (i.e. objects) in a separate list? Or can we do MTF on objects? (probably not, else we
//   wouldn't have holes in the array due to removed objects)

// PT: automatically populate new region with overlapping objects.
// Brute-force version checking all existing objects, potentially optimized using "fully inside" flags.
//#define FIRST_VERSION
#ifdef FIRST_VERSION
void MBP::populateNewRegion(const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex)
{
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	const PxU32 nbObjects = mMBP_Objects.size();
	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();

#ifdef PRINT_STATS
	PxU32 nbObjectsFound = 0;
	PxU32 nbObjectsTested = 0;
#endif

#ifdef USE_FULLY_INSIDE_FLAG
	const PxU32* fullyInsideFlags = mFullyInsideBitmap.getBits();
#endif

	PxU32 j=0;
	while(j<nbObjects)
	{
#ifdef USE_FULLY_INSIDE_FLAG
		const PxU32 blockFlags = fullyInsideFlags[j>>5];
	#ifdef HWSCAN
		if(blockFlags==0)	//HWSCAN
	#else
		if(blockFlags==0xffffffff)
	#endif
		{
			j+=32;
			continue;
		}
		PxU32 nbToGo = PxMin(nbObjects - j, PxU32(32));
		PxU32 mask = 1;
		while(nbToGo--)
		{

		MBP_Object& currentObject = objects[j];
		// PT: if an object A is fully contained inside all the regions S it overlaps, we don't need to test it against the new region R.
		// The rationale is that even if R does overlap A, any new object B must touch S to overlap with A. So B would be added to S and
		// the (A,B) overlap would be detected in S, even if it's not detected in R.
		const PxU32 res = blockFlags & mask;
		PX_ASSERT((mFullyInsideBitmap.isSet(j) && res) || (!mFullyInsideBitmap.isSet(j) && !res));
		mask+=mask;
		j++;
	#ifdef HWSCAN
		if(!res)	//HWSCAN
	#else
		if(res)
	#endif
			continue;
		PX_ASSERT(!(currentObject.mFlags & MBP_REMOVED));
#else
		MBP_Object& currentObject = objects[j++];
		if(currentObject.mFlags & MBP_REMOVED)
			continue;	// PT: object is in the free list
#endif

#ifdef PRINT_STATS
		nbObjectsTested++;
#endif

		MBP_AABB bounds;
		MBP_Handle mbpHandle;

		const PxU32 nbHandles = currentObject.mNbHandles;
		if(nbHandles)
		{
			RegionHandle* PX_RESTRICT handles = getHandles(currentObject, nbHandles);
			// PT: no need to test all regions since they should contain the same info. Just retrieve bounds from the first one.
			PxU32 i=0;	//	for(PxU32 i=0;i<nbHandles;i++)
			{
				const RegionHandle& h = handles[i];
				const RegionData& currentRegion = regions[h.mInternalBPHandle];
				PX_ASSERT(currentRegion.mBP);

				mbpHandle = currentRegion.mBP->retrieveBounds(bounds, h.mHandle);
			}
		}
		else
		{
			PX_ASSERT(mManager);

			// PT: if the object is out-of-bounds, we're out-of-luck. We don't have the object bounds, so we need to retrieve them
			// from the AABB manager - and then re-encode them. This is not very elegant or efficient, but it should rarely happen
			// so this is good enough for now.
			const PxBounds3 decodedBounds = mManager->getBPBounds(currentObject.mUserID);

			bounds.initFrom2(decodedBounds);

			mbpHandle = currentObject.mHandlesIndex;
		}

		if(bounds.intersect(box))
		{
//			updateObject(mbpHandle, bounds);
			updateObjectAfterNewRegionAdded(mbpHandle, bounds, addedRegion, regionIndex);
#ifdef PRINT_STATS
			nbObjectsFound++;
#endif
		}

#ifdef USE_FULLY_INSIDE_FLAG
		}
#endif
	}
#ifdef PRINT_STATS
	printf("Populating new region with %d objects (tested %d/%d object)\n", nbObjectsFound, nbObjectsTested, nbObjects);
#endif
}
#endif

// PT: version using lowestSetBit
#define SECOND_VERSION
#ifdef SECOND_VERSION

/*	PX_FORCE_INLINE PxU32 lowestSetBitUnsafe64(PxU64 v)
	{
		unsigned long retval;
		_BitScanForward64(&retval, v);
		return retval;
	}*/

void MBP::populateNewRegion(const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex)
{
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	const PxU32 nbObjects = mMBP_Objects.size();
	PX_UNUSED(nbObjects);
	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();
	const PxU32* fullyInsideFlags = mFullyInsideBitmap.getBits();
//	const PxU64* fullyInsideFlags = (const PxU64*)mFullyInsideBitmap.getBits();
	if(!fullyInsideFlags)
		return;

	const PxU32 lastSetBit = mFullyInsideBitmap.findLast();
//	const PxU64 lastSetBit = mFullyInsideBitmap.findLast();

#ifdef PRINT_STATS
	PxU32 nbObjectsFound = 0;
	PxU32 nbObjectsTested = 0;
#endif

	for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
//	for(PxU64 w = 0; w <= lastSetBit >> 6; ++w)
	{
		for(PxU32 b = fullyInsideFlags[w]; b; b &= b-1)
//		for(PxU64 b = fullyInsideFlags[w]; b; b &= b-1)
		{
			const PxU32 index = PxU32(w<<5|Ps::lowestSetBit(b));
//			const PxU64 index = (PxU64)(w<<6|::lowestSetBitUnsafe64(b));
			PX_ASSERT(index<nbObjects);

			MBP_Object& currentObject = objects[index];
			// PT: if an object A is fully contained inside all the regions S it overlaps, we don't need to test it against the new region R.
			// The rationale is that even if R does overlap A, any new object B must touch S to overlap with A. So B would be added to S and
			// the (A,B) overlap would be detected in S, even if it's not detected in R.
			PX_ASSERT(!(currentObject.mFlags & MBP_REMOVED));
#ifdef HWSCAN
			PX_ASSERT(mFullyInsideBitmap.isSet(index));
#else
			PX_ASSERT(!mFullyInsideBitmap.isSet(index));
#endif

#ifdef PRINT_STATS
			nbObjectsTested++;
#endif

			MBP_AABB bounds;
			MBP_Handle mbpHandle;

			const PxU32 nbHandles = currentObject.mNbHandles;
			if(nbHandles)
			{
				RegionHandle* PX_RESTRICT handles = getHandles(currentObject, nbHandles);
				// PT: no need to test all regions since they should contain the same info. Just retrieve bounds from the first one.
				PxU32 i=0;	//	for(PxU32 i=0;i<nbHandles;i++)
				{
					const RegionHandle& h = handles[i];
					const RegionData& currentRegion = regions[h.mInternalBPHandle];
					PX_ASSERT(currentRegion.mBP);

					mbpHandle = currentRegion.mBP->retrieveBounds(bounds, h.mHandle);
				}
			}
			else
			{
				// PT: if the object is out-of-bounds, we're out-of-luck. We don't have the object bounds, so we need to retrieve them
				// from the AABB manager - and then re-encode them. This is not very elegant or efficient, but it should rarely happen
				// so this is good enough for now.
				const PxBounds3 rawBounds = mTransientBounds[currentObject.mUserID];
				PxVec3 c(mTransientContactDistance[currentObject.mUserID]);
				const PxBounds3 decodedBounds(rawBounds.minimum - c, rawBounds.maximum + c);
				bounds.initFrom2(decodedBounds);

				mbpHandle = currentObject.mHandlesIndex;
			}

			if(bounds.intersects(box))
			{
//				updateObject(mbpHandle, bounds);
				updateObjectAfterNewRegionAdded(mbpHandle, bounds, addedRegion, regionIndex);
#ifdef PRINT_STATS
				nbObjectsFound++;
#endif
			}
		}
	}
#ifdef PRINT_STATS
	printf("Populating new region with %d objects (tested %d/%d object)\n", nbObjectsFound, nbObjectsTested, nbObjects);
#endif
}
#endif

//#define THIRD_VERSION
#ifdef THIRD_VERSION
void MBP::populateNewRegion(const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex)
{
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	const PxU32 nbObjects = mMBP_Objects.size();
	PX_UNUSED(nbObjects);
	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();

	const PxU32* fullyInsideFlags = mFullyInsideBitmap.getBits();
	if(!fullyInsideFlags)
		return;

#ifdef PRINT_STATS
	PxU32 nbObjectsFound = 0;
	PxU32 nbObjectsTested = 0;
#endif

	Cm::BitMap bm;
	bm.importData(mFullyInsideBitmap.getSize(), (PxU32*)fullyInsideFlags);

	Cm::BitMap::Iterator it(bm);
	PxU32 index = it.getNext();
	while(index != Cm::BitMap::Iterator::DONE)
	{
		PX_ASSERT(index<nbObjects);

		MBP_Object& currentObject = objects[index];
		// PT: if an object A is fully contained inside all the regions S it overlaps, we don't need to test it against the new region R.
		// The rationale is that even if R does overlap A, any new object B must touch S to overlap with A. So B would be added to S and
		// the (A,B) overlap would be detected in S, even if it's not detected in R.
		PX_ASSERT(!(currentObject.mFlags & MBP_REMOVED));

#ifdef PRINT_STATS
		nbObjectsTested++;
#endif

		MBP_AABB bounds;
		MBP_Handle mbpHandle;

		const PxU32 nbHandles = currentObject.mNbHandles;
		if(nbHandles)
		{
			RegionHandle* PX_RESTRICT handles = getHandles(currentObject, nbHandles);
			// PT: no need to test all regions since they should contain the same info. Just retrieve bounds from the first one.
			PxU32 i=0;	//	for(PxU32 i=0;i<nbHandles;i++)
			{
				const RegionHandle& h = handles[i];
				const RegionData& currentRegion = regions[h.mInternalBPHandle];
				PX_ASSERT(currentRegion.mBP);

				mbpHandle = currentRegion.mBP->retrieveBounds(bounds, h.mHandle);
			}
		}
		else
		{
			PX_ASSERT(mManager);

			// PT: if the object is out-of-bounds, we're out-of-luck. We don't have the object bounds, so we need to retrieve them
			// from the AABB manager - and then re-encode them. This is not very elegant or efficient, but it should rarely happen
			// so this is good enough for now.
			const PxBounds3 decodedBounds = mManager->getBPBounds(currentObject.mUserID);

			bounds.initFrom2(decodedBounds);

			mbpHandle = currentObject.mHandlesIndex;
		}

		if(bounds.intersect(box))
		{
//			updateObject(mbpHandle, bounds);
			updateObjectAfterNewRegionAdded(mbpHandle, bounds, addedRegion, regionIndex);
#ifdef PRINT_STATS
			nbObjectsFound++;
#endif
		}
		index = it.getNext();
	}
#ifdef PRINT_STATS
	printf("Populating new region with %d objects (tested %d/%d object)\n", nbObjectsFound, nbObjectsTested, nbObjects);
#endif
}
#endif

PxU32 MBP::addRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	PxU32 regionHandle;
	RegionData* PX_RESTRICT buffer;

	if(mFirstFreeIndexBP!=INVALID_ID)
	{
		regionHandle = mFirstFreeIndexBP;

		buffer = mRegions.begin();
		buffer += regionHandle;

		mFirstFreeIndexBP = PxU32(size_t(buffer->mUserData));	// PT: this is safe, we previously stored a PxU32 in there
	}
	else
	{
		if(mNbRegions>=MAX_NB_MBP)
		{
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "MBP::addRegion: max number of regions reached.");
			return INVALID_ID;
		}

		regionHandle = mNbRegions++;	
		buffer = reserveContainerMemory<RegionData>(mRegions, 1);		
	}

	Region* newRegion = PX_NEW(Region);
	buffer->mBox.initFrom2(region.bounds);
	buffer->mBP			= newRegion;
	buffer->mUserData	= region.userData;

	setupOverlapFlags(mNbRegions, mRegions.begin());

	// PT: automatically populate new region with overlapping objects
	if(populateRegion)
		populateNewRegion(buffer->mBox, newRegion, regionHandle);

#ifdef MBP_REGION_BOX_PRUNING
	mDirtyRegions = true;
#endif

	return regionHandle;
}

// ### TODO: recycle regions, make sure objects are properly deleted/transferred, etc
// ### TODO: what happens if we delete a zone then immediately add it back? Do objects get deleted?
// ### TODO: in fact if we remove a zone but we keep the objects, what happens to their current overlaps? Are they kept or discarded?
bool MBP::removeRegion(PxU32 handle)
{
	if(handle>=mNbRegions)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "MBP::removeRegion: invalid handle.");
		return false;
	}

	RegionData* PX_RESTRICT region = mRegions.begin();
	region += handle;

	Region* bp = region->mBP;
	if(!bp)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "MBP::removeRegion: invalid handle.");
		return false;
	}

	PxBounds3 empty;
	empty.setEmpty();
	region->mBox.initFrom2(empty);

	{
		// We are going to remove the region but it can still contain objects. We need to update
		// those objects so that their handles and out-of-bounds status are modified.
		//
		// Unfortunately there is no way to iterate over active objects in a region, so we need
		// to iterate over the max amount of objects. ### TODO: optimize this
		const PxU32 maxNbObjects = bp->mMaxNbObjects;
		MBPEntry* PX_RESTRICT objects = bp->mObjects;
		for(PxU32 j=0;j<maxNbObjects;j++)
		{
			// The handle is INVALID_ID for non-active entries
			if(objects[j].mMBPHandle!=INVALID_ID)
			{
//				printf("Object to update!\n");
				updateObjectAfterRegionRemoval(objects[j].mMBPHandle, bp);
			}
		}
	}

	PX_DELETE(bp);
	region->mBP = NULL;
	region->mUserData = reinterpret_cast<void*>(size_t(mFirstFreeIndexBP));
	mFirstFreeIndexBP = handle;

#ifdef MBP_REGION_BOX_PRUNING
	mDirtyRegions = true;
#endif

	// A region has been removed so we need to update the overlap flags for all remaining regions
	// ### TODO: optimize this
	setupOverlapFlags(mNbRegions, mRegions.begin());
	return true;
}

const Region* MBP::getRegion(PxU32 i) const
{
	if(i>=mNbRegions)
		return NULL;

	const RegionData* PX_RESTRICT regions = mRegions.begin();
	return regions[i].mBP;
}

#ifdef MBP_REGION_BOX_PRUNING
void MBP::buildRegionData()
{
	const PxU32 size = mNbRegions;
	PxU32 nbValidRegions = 0;
	if(size)
	{
		const RegionData* PX_RESTRICT regions = mRegions.begin();

		// Gather valid regions
		PxU32 minPosList[MAX_NB_MBP];
		for(PxU32 i=0;i<size;i++)
		{
			if(regions[i].mBP)
				minPosList[nbValidRegions++] = regions[i].mBox.mMinX;
		}

		// Sort them
		RadixSortBuffered RS;
		const PxU32* sorted = RS.Sort(minPosList, nbValidRegions, RADIX_UNSIGNED).GetRanks();

		// Store sorted
		for(PxU32 i=0;i<nbValidRegions;i++)
		{
			const PxU32 sortedIndex = *sorted++;
			mSortedRegionBoxes[i] = regions[sortedIndex].mBox;
			mSortedRegionIndices[i] = sortedIndex;
		}
	}
	mNbActiveRegions = nbValidRegions;
	mDirtyRegions = false;
}
#endif

PX_FORCE_INLINE RegionHandle* MBP::getHandles(MBP_Object& currentObject, PxU32 nbHandles)
{
	RegionHandle* handles;
	if(nbHandles==1)
		handles = &currentObject.mHandle;
	else
	{
		const PxU32 handlesIndex = currentObject.mHandlesIndex;
		Ps::Array<PxU32>& c = mHandles[nbHandles];
		handles = reinterpret_cast<RegionHandle*>(c.begin()+handlesIndex);
	}
	return handles;
}

void MBP::purgeHandles(MBP_Object* PX_RESTRICT object, PxU32 nbHandles)
{
	if(nbHandles>1)
	{
		const PxU32 handlesIndex = object->mHandlesIndex;
		Ps::Array<PxU32>& c = mHandles[nbHandles];
		PxU32* recycled = c.begin() + handlesIndex;
		*recycled = mFirstFree[nbHandles];
		mFirstFree[nbHandles] = handlesIndex;
	}
}

void MBP::storeHandles(MBP_Object* PX_RESTRICT object, PxU32 nbHandles, const RegionHandle* PX_RESTRICT handles)
{
	if(nbHandles==1)
	{
		object->mHandle = handles[0];
	}
	else if(nbHandles)
	{
		Ps::Array<PxU32>& c = mHandles[nbHandles];
		const PxU32 firstFree = mFirstFree[nbHandles];
		PxU32* handlesMemory;
		if(firstFree!=INVALID_ID)
		{
			object->mHandlesIndex = firstFree;
			handlesMemory = c.begin() + firstFree;
			mFirstFree[nbHandles] = *handlesMemory;
		}
		else
		{
			const PxU32 handlesIndex = c.size();
			object->mHandlesIndex = handlesIndex;
			handlesMemory = reserveContainerMemory<PxU32>(c, sizeof(RegionHandle)*nbHandles/sizeof(PxU32));
		}
		PxMemCopy(handlesMemory, handles, sizeof(RegionHandle)*nbHandles);
	}
}

MBP_Handle MBP::addObject(const MBP_AABB& box, BpHandle userID, bool isStatic)
{
	MBP_ObjectIndex objectIndex;
	MBP_Object* objectMemory;
	PxU32 flipFlop;
	if(1)
	{
		if(mFirstFreeIndex!=INVALID_ID)
		{
			objectIndex = mFirstFreeIndex;
			MBP_Object* objects = mMBP_Objects.begin();
			objectMemory = &objects[objectIndex];
			PX_ASSERT(!objectMemory->mNbHandles);
			mFirstFreeIndex = objectMemory->mHandlesIndex;
			flipFlop = PxU32(objectMemory->getFlipFlop());
		}
		else
		{
			objectIndex = mMBP_Objects.size();
			objectMemory = reserveContainerMemory<MBP_Object>(mMBP_Objects, 1);		
			flipFlop = 0;
		}
	}
	else
	{
		// PT: must be possible to use the AABB-manager's ID directly. Something like this:
		objectIndex = userID;
		if(mMBP_Objects.capacity()<userID+1)
		{
			PxU32 newCap = mMBP_Objects.capacity() ? mMBP_Objects.capacity()*2 : 128;
			if(newCap<userID+1)
				newCap = userID+1;

			mMBP_Objects.reserve(newCap);
		}
			mMBP_Objects.forceSize_Unsafe(userID+1);
		objectMemory = &mMBP_Objects[userID];

		flipFlop = 0;
	}

	const MBP_Handle MBPObjectHandle = encodeHandle(objectIndex, flipFlop, isStatic);

//	mMBP_Objects.Shrink();

	PxU32 nbHandles = 0;
#ifdef USE_FULLY_INSIDE_FLAG
	bool newObjectIsFullyInsideRegions = true;
#endif

	const PxU32 nb = mNbRegions;
	const RegionData* PX_RESTRICT regions = mRegions.begin();

	RegionHandle tmpHandles[MAX_NB_MBP+1];
	for(PxU32 i=0;i<nb;i++)
	{
#ifdef MBP_USE_NO_CMP_OVERLAP_3D
		if(intersect3D(regions[i].mBox, box))
#else
		if(regions[i].mBox.intersects(box))
#endif
		{
#ifdef USE_FULLY_INSIDE_FLAG
			if(!box.isInside(regions[i].mBox))
				newObjectIsFullyInsideRegions = false;
#endif
#ifdef MBP_USE_WORDS
			if(regions[i].mBP->mNbObjects==0xffff)
				Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "MBP::addObject: 64K objects in single region reached. Some collisions might be lost.");
			else
#endif
			{
				RegionHandle& h = tmpHandles[nbHandles++];
				h.mHandle = regions[i].mBP->addObject(box, MBPObjectHandle, isStatic);
				h.mInternalBPHandle = Ps::to16(i);
			}
		}
	}
	storeHandles(objectMemory, nbHandles, tmpHandles);

	objectMemory->mNbHandles	= Ps::to16(nbHandles);
	PxU16 flags = 0;
	if(flipFlop)
		flags |= MBP_FLIP_FLOP;
#ifdef USE_FULLY_INSIDE_FLAG
	if(nbHandles && newObjectIsFullyInsideRegions)
		setBit(mFullyInsideBitmap, objectIndex);
	else
		clearBit(mFullyInsideBitmap, objectIndex);
#endif
	if(!nbHandles)
	{
		objectMemory->mHandlesIndex = MBPObjectHandle;
		addToOutOfBoundsArray(userID);
	}

	if(!isStatic)
		mUpdatedObjects.setBitChecked(objectIndex);

//	objectMemory->mUpdated	= !isStatic;
	objectMemory->mFlags	= flags;
	objectMemory->mUserID	= userID;

	return MBPObjectHandle;
}

bool MBP::removeObject(MBP_Handle handle)
{
	const MBP_ObjectIndex objectIndex = decodeHandle_Index(handle);

	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();
	MBP_Object& currentObject = objects[objectIndex];
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	// Parse previously overlapping regions. If still overlapping, update object. Else remove from region.
	const PxU32 nbHandles = currentObject.mNbHandles;
	if(nbHandles)
	{
		RegionHandle* handles = getHandles(currentObject, nbHandles);
		for(PxU32 i=0;i<nbHandles;i++)
		{
			const RegionHandle& h = handles[i];
			const RegionData& currentRegion = regions[h.mInternalBPHandle];
//			if(currentRegion.mBP)
			PX_ASSERT(currentRegion.mBP);
			currentRegion.mBP->removeObject(h.mHandle);
		}

		purgeHandles(&currentObject, nbHandles);
	}

	currentObject.mNbHandles	= 0;
	currentObject.mFlags		|= MBP_REMOVED;
	currentObject.mHandlesIndex	= mFirstFreeIndex;
//	if(!decodeHandle_IsStatic(handle))
//	if(!currentObject.IsStatic())
		mUpdatedObjects.setBitChecked(objectIndex);

	mFirstFreeIndex				= objectIndex;

	mRemoved.setBitChecked(objectIndex);	// PT: this is cleared each frame so it's not a replacement for the MBP_REMOVED flag

#ifdef USE_FULLY_INSIDE_FLAG
	// PT: when removing an object we mark it as "fully inside" so that it is automatically
	// discarded in the "populateNewRegion" function, without the need for MBP_REMOVED.
	setBit(mFullyInsideBitmap, objectIndex);
#endif
	return true;
}

static PX_FORCE_INLINE bool stillIntersects(PxU32 handle, PxU32& _nb, PxU32* PX_RESTRICT currentOverlaps)
{
	const PxU32 nb = _nb;
	for(PxU32 i=0;i<nb;i++)
	{
		if(currentOverlaps[i]==handle)
		{
			_nb = nb-1;
			currentOverlaps[i] = currentOverlaps[nb-1];
			return true;
		}
	}
	return false;
}

bool MBP::updateObject(MBP_Handle handle, const MBP_AABB& box)
{
	const MBP_ObjectIndex objectIndex = decodeHandle_Index(handle);
	const PxU32 isStatic = decodeHandle_IsStatic(handle);

	const PxU32 nbRegions = mNbRegions;
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();
	MBP_Object& currentObject = objects[objectIndex];

//	if(!isStatic)	// ### removed for PhysX integration (bugfix)
//	if(!currentObject.IsStatic())
	{
		mUpdatedObjects.setBitChecked(objectIndex);
	}

	// PT: fast path which should happen quite frequently. If:
	// - the object was touching a single region
	// - that region doesn't overlap other regions
	// - the object's new bounds is fully inside the region
	// then we know that the object can't touch another region, and we can use this fast-path that simply
	// updates one region and avoids iterating over/testing all the other ones.
	const PxU32 nbHandles = currentObject.mNbHandles;
	if(nbHandles==1)
	{
		const RegionHandle& h = currentObject.mHandle;
		const RegionData& currentRegion = regions[h.mInternalBPHandle];
		if(!currentRegion.mOverlap && box.isInside(currentRegion.mBox))
		{
#ifdef USE_FULLY_INSIDE_FLAG
			// PT: it is possible that this flag is not set already when reaching this place:
			// - object touches 2 regions
			// - then in one frame:
			//    - object moves fully inside one region
			//    - the other region is removed
			// => nbHandles changes from 2 to 1 while MBP_FULLY_INSIDE is not set
			setBit(mFullyInsideBitmap, objectIndex);
#endif
			currentRegion.mBP->updateObject(box, h.mHandle);
			return true;
		}
	}

	// Find regions overlapping object's new position
#ifdef USE_FULLY_INSIDE_FLAG
	bool objectIsFullyInsideRegions = true;
#endif
	PxU32 nbCurrentOverlaps = 0;
	PxU32 currentOverlaps[MAX_NB_MBP+1];
	// PT: here, we may still parse regions which have been removed. But their boxes have been set to empty,
	// so nothing will happen.
	for(PxU32 i=0;i<nbRegions;i++)
	{
#ifdef MBP_USE_NO_CMP_OVERLAP_3D
		if(intersect3D(regions[i].mBox, box))
#else
		if(regions[i].mBox.intersects(box))
#endif
		{
#ifdef USE_FULLY_INSIDE_FLAG
			if(!box.isInside(regions[i].mBox))
				objectIsFullyInsideRegions = false;
#endif
			PX_ASSERT(nbCurrentOverlaps<MAX_NB_MBP);
			currentOverlaps[nbCurrentOverlaps++] = i;
		}
	}

	// New data for this frame
	PxU32 nbNewHandles = 0;
	RegionHandle newHandles[MAX_NB_MBP+1];

	// Parse previously overlapping regions. If still overlapping, update object. Else remove from region.
	RegionHandle* handles = getHandles(currentObject, nbHandles);
	for(PxU32 i=0;i<nbHandles;i++)
	{
		const RegionHandle& h = handles[i];
		PX_ASSERT(h.mInternalBPHandle<nbRegions);
		const RegionData& currentRegion = regions[h.mInternalBPHandle];
		// We need to update object even if it then gets removed, as the removal
		// doesn't actually report lost pairs - and we need this.
//		currentRegion.mBP->UpdateObject(box, h.mHandle);
		if(stillIntersects(h.mInternalBPHandle, nbCurrentOverlaps, currentOverlaps))
		{
			currentRegion.mBP->updateObject(box, h.mHandle);
			// Still collides => keep handle for this frame
			newHandles[nbNewHandles++] = h;
		}
		else
		{
			PX_ASSERT(!currentRegion.mBox.intersects(box));
//			if(currentRegion.mBP)
			PX_ASSERT(currentRegion.mBP);
			currentRegion.mBP->removeObject(h.mHandle);
		}
	}

	// Add to new regions if needed
	for(PxU32 i=0;i<nbCurrentOverlaps;i++)
	{
//		if(currentOverlaps[i]==INVALID_ID)
//			continue;
		const PxU32 regionIndex = currentOverlaps[i];
		const MBP_Index BPHandle = regions[regionIndex].mBP->addObject(box, handle, isStatic!=0);
		newHandles[nbNewHandles].mHandle = Ps::to16(BPHandle);
		newHandles[nbNewHandles].mInternalBPHandle = Ps::to16(regionIndex);
		nbNewHandles++;
	}

	if(nbHandles==nbNewHandles)
	{
		for(PxU32 i=0;i<nbNewHandles;i++)
			handles[i] = newHandles[i];
	}
	else
	{
		purgeHandles(&currentObject, nbHandles);
		storeHandles(&currentObject, nbNewHandles, newHandles);
	}

	currentObject.mNbHandles = Ps::to16(nbNewHandles);
	if(!nbNewHandles && nbHandles)
	{
		currentObject.mHandlesIndex = handle;
		addToOutOfBoundsArray(currentObject.mUserID);
	}

//	for(PxU32 i=0;i<nbNewHandles;i++)
//		currentObject.mHandles[i] = newHandles[i];

#ifdef USE_FULLY_INSIDE_FLAG
	if(objectIsFullyInsideRegions && nbNewHandles)
		setBit(mFullyInsideBitmap, objectIndex);
	else
		clearBit(mFullyInsideBitmap, objectIndex);
#endif
	return true;
}

bool MBP::updateObjectAfterRegionRemoval(MBP_Handle handle, Region* removedRegion)
{
	PX_ASSERT(removedRegion);

	const MBP_ObjectIndex objectIndex = decodeHandle_Index(handle);

	const PxU32 nbRegions = mNbRegions;
	PX_UNUSED(nbRegions);
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();
	MBP_Object& currentObject = objects[objectIndex];

	// Mark the object as updated so that its pairs are considered for removal. If we don't do this an out-of-bounds object
	// resting on another non-out-of-bounds object still collides with that object and the memory associated with that pair
	// is not released. If we mark it as updated the pair is lost, and the out-of-bounds object falls through.
	//
	// However if we do this any pair involving the object will be marked as lost, even the ones involving other regions.
	// Typically the pair will then get lost one frame and get recreated the next frame.
//	mUpdatedObjects.setBitChecked(objectIndex);

	const PxU32 nbHandles = currentObject.mNbHandles;
	PX_ASSERT(nbHandles);

	// New handles
	PxU32 nbNewHandles = 0;
	RegionHandle newHandles[MAX_NB_MBP+1];

	// Parse previously overlapping regions. Keep all of them except removed one.
	RegionHandle* handles = getHandles(currentObject, nbHandles);
	for(PxU32 i=0;i<nbHandles;i++)
	{
		const RegionHandle& h = handles[i];
		PX_ASSERT(h.mInternalBPHandle<nbRegions);
		if(regions[h.mInternalBPHandle].mBP!=removedRegion)
			newHandles[nbNewHandles++] = h;
	}
#ifdef USE_FULLY_INSIDE_FLAG
	// PT: in theory we should update the inside flag here but we don't do that for perf reasons.
	// - If the flag is set, it means the object was fully inside all its regions. Removing one of them does not invalidate the flag.
	// - If the flag is not set, removing one region might allow us to set the flag now. However not doing so simply makes the
	//   populateNewRegion() function run a bit slower, it does not produce wrong results. This is only until concerned objects are
	//   updated again anyway, so we live with this.
#endif

	PX_ASSERT(nbNewHandles==nbHandles-1);
	purgeHandles(&currentObject, nbHandles);
	storeHandles(&currentObject, nbNewHandles, newHandles);

	currentObject.mNbHandles = Ps::to16(nbNewHandles);
	if(!nbNewHandles)
	{
		currentObject.mHandlesIndex = handle;
		addToOutOfBoundsArray(currentObject.mUserID);
#ifdef USE_FULLY_INSIDE_FLAG
		clearBit(mFullyInsideBitmap, objectIndex);
#endif
	}
	return true;
}

bool MBP::updateObjectAfterNewRegionAdded(MBP_Handle handle, const MBP_AABB& box, Region* addedRegion, PxU32 regionIndex)
{
	PX_ASSERT(addedRegion);

	const MBP_ObjectIndex objectIndex = decodeHandle_Index(handle);
	const PxU32 isStatic = decodeHandle_IsStatic(handle);

	MBP_Object* PX_RESTRICT objects = mMBP_Objects.begin();
	MBP_Object& currentObject = objects[objectIndex];

//	if(!isStatic)	// ### removed for PhysX integration (bugfix)
//	if(!currentObject.IsStatic())
	{
		mUpdatedObjects.setBitChecked(objectIndex);
	}

	// PT: here we know that we're touching one more region than before and we'll need to update the handles.
	// So there is no "fast path" in this case - well the whole function is a fast path if you want.
	//
	// We don't need to "find regions overlapping object's new position": we know it's going to be the
	// same as before, plus the newly added region ("addedRegion").

#ifdef USE_FULLY_INSIDE_FLAG
	// PT: we know that the object is not marked as "fully inside", otherwise this function would not have been called.
	#ifdef HWSCAN
	PX_ASSERT(mFullyInsideBitmap.isSet(objectIndex));	//HWSCAN
	#else
	PX_ASSERT(!mFullyInsideBitmap.isSet(objectIndex));
	#endif
#endif

	const PxU32 nbHandles = currentObject.mNbHandles;
	PxU32 nbNewHandles = 0;
	RegionHandle newHandles[MAX_NB_MBP+1];

	// PT: get previously overlapping regions. We didn't actually move so we're still overlapping as before.
	// We just need to get the handles here.
	RegionHandle* handles = getHandles(currentObject, nbHandles);
	for(PxU32 i=0;i<nbHandles;i++)
		newHandles[nbNewHandles++] = handles[i];

	// Add to new region
	{
#if PX_DEBUG
		const RegionData* PX_RESTRICT regions = mRegions.begin();
		const RegionData& currentRegion = regions[regionIndex];
		PX_ASSERT(currentRegion.mBox.intersects(box));
#endif
		const MBP_Index BPHandle = addedRegion->addObject(box, handle, isStatic!=0);
		newHandles[nbNewHandles].mHandle = Ps::to16(BPHandle);
		newHandles[nbNewHandles].mInternalBPHandle = Ps::to16(regionIndex);
		nbNewHandles++;
	}

	// PT: we know that we have one more handle than before, no need to test
	purgeHandles(&currentObject, nbHandles);
	storeHandles(&currentObject, nbNewHandles, newHandles);

	currentObject.mNbHandles = Ps::to16(nbNewHandles);

	// PT: we know that we have at least one handle (from the newly added region), so we can't be "out of bounds" here.
	PX_ASSERT(nbNewHandles);

#ifdef USE_FULLY_INSIDE_FLAG
	// PT: we know that the object was not "fully inside" before, so even if it is fully inside the new region, it
	// will not be fully inside all of them => no need to change its fully inside flag
	// TODO: an exception to this would be the case where the object was out-of-bounds, and it's now fully inside the new region
	// => we could set the flag in that case.
#endif
	return true;
}

bool MBP_PairManager::computeCreatedDeletedPairs(const MBP_Object* objects, BroadPhaseMBP* mbp, const BitArray& updated, const BitArray& removed)
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
			const MBP_ObjectIndex index0 = decodeHandle_Index(id0);
			const MBP_ObjectIndex index1 = decodeHandle_Index(id1);

			const BpHandle object0 = objects[index0].mUserID;
			const BpHandle object1 = objects[index1].mUserID;
			mbp->mCreated.pushBack(BroadPhasePair(object0, object1));

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

			const MBP_ObjectIndex index0 = decodeHandle_Index(id0);
			const MBP_ObjectIndex index1 = decodeHandle_Index(id1);
			// PT: if none of the involved objects have been updated, the pair is just sleeping: keep it and skip it.
			if(updated.isSetChecked(index0) || updated.isSetChecked(index1))
			{
				// PT: by design (for better or worse) we do not report pairs to the client when
				// one of the involved objects has been deleted. The pair must still be deleted
				// from the MBP structure though.
				if(!removed.isSetChecked(index0) && !removed.isSetChecked(index1))
				{
					// PT: doing the group-based filtering here is useless. The pair should not have
					// been added in the first place.
					const BpHandle object0 = objects[index0].mUserID;
					const BpHandle object1 = objects[index1].mUserID;
					mbp->mDeleted.pushBack(BroadPhasePair(object0, object1));
				}

				const PxU32 hashValue = hash(id0, id1) & mMask;
				PairManagerData::removePair(id0, id1, hashValue, i);
				nbActivePairs--;
			}
			else i++;
		}
	}

	shrinkMemory();
	return true;
}

void MBP::prepareOverlaps()
{
	const PxU32 nb = mNbRegions;
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	for(PxU32 i=0;i<nb;i++)
	{
		if(regions[i].mBP)
			regions[i].mBP->prepareOverlaps();
	}
}

void MBP::findOverlaps(const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
	)
{
	PxU32 nb = mNbRegions;
	const RegionData* PX_RESTRICT regions = mRegions.begin();
	const MBP_Object* objects = mMBP_Objects.begin();

	mPairManager.mObjects = objects;
	mPairManager.mGroups = groups;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	mPairManager.mLUT = lut;
#endif

	for(PxU32 i=0;i<nb;i++)
	{
		if(regions[i].mBP)
			regions[i].mBP->findOverlaps(mPairManager);
	}
}

PxU32 MBP::finalize(BroadPhaseMBP* mbp)
{
	const MBP_Object* objects = mMBP_Objects.begin();
	mPairManager.computeCreatedDeletedPairs(objects, mbp, mUpdatedObjects, mRemoved);

	mUpdatedObjects.clearAll();

	return mPairManager.mNbActivePairs;
}

void MBP::reset()
{
	PxU32 nb = mNbRegions;
	RegionData* PX_RESTRICT regions = mRegions.begin();
	while(nb--)
	{
//		printf("%d objects in region\n", regions->mBP->mNbObjects);
		DELETESINGLE(regions->mBP);
		regions++;
	}

	mNbRegions			= 0;
	mFirstFreeIndex		= INVALID_ID;
	mFirstFreeIndexBP	= INVALID_ID;
	for(PxU32 i=0;i<MAX_NB_MBP+1;i++)
	{
		mHandles[i].clear();
		mFirstFree[i] = INVALID_ID;
	}

	mRegions.clear();
	mMBP_Objects.clear();
	mPairManager.purge();
	mUpdatedObjects.empty();
	mRemoved.empty();
	mOutOfBoundsObjects.clear();
#ifdef USE_FULLY_INSIDE_FLAG
	mFullyInsideBitmap.empty();
#endif
}

void MBP::shiftOrigin(const PxVec3& shift)
{
	const PxU32 size = mNbRegions;
	RegionData* PX_RESTRICT regions = mRegions.begin();
	//
	// regions
	//
	for(PxU32 i=0; i < size; i++)
	{
		if(regions[i].mBP)
		{
			MBP_AABB& box = regions[i].mBox;
			PxBounds3 bounds;
			box.decode(bounds);

			bounds.minimum -= shift;
			bounds.maximum -= shift;

			box.initFrom2(bounds);
		}
	}

	//
	// object bounds
	//
	const PxU32 nbObjects = mMBP_Objects.size();
	MBP_Object* objects = mMBP_Objects.begin();

	for(PxU32 i=0; i < nbObjects; i++)
	{
		MBP_Object& obj = objects[i];

		const PxU32 nbHandles = obj.mNbHandles;
		if(nbHandles)
		{
			MBP_AABB bounds;
			const PxBounds3 rawBounds = mTransientBounds[obj.mUserID];
			PxVec3 c(mTransientContactDistance[obj.mUserID]);
			const PxBounds3 decodedBounds(rawBounds.minimum - c, rawBounds.maximum + c);
			bounds.initFrom2(decodedBounds);

			RegionHandle* PX_RESTRICT handles = getHandles(obj, nbHandles);
			for(PxU32 j=0; j < nbHandles; j++)
			{
				const RegionHandle& h = handles[j];
				const RegionData& currentRegion = regions[h.mInternalBPHandle];
				PX_ASSERT(currentRegion.mBP);
				currentRegion.mBP->setBounds(h.mHandle, bounds);
			}
		}
	}
}

void MBP::setTransientBounds(const PxBounds3* bounds, const PxReal* contactDistance)
{
	mTransientBounds = bounds;
	mTransientContactDistance = contactDistance;
}
///////////////////////////////////////////////////////////////////////////////

// Below is the PhysX wrapper = link between AABBManager and MBP

#define DEFAULT_CREATED_DELETED_PAIRS_CAPACITY 1024

BroadPhaseMBP::BroadPhaseMBP(	PxU32 maxNbRegions,
								PxU32 maxNbBroadPhaseOverlaps,
								PxU32 maxNbStaticShapes,
								PxU32 maxNbDynamicShapes,
								PxU64 contextID) :
	mMBPUpdateWorkTask		(contextID),
	mMBPPostUpdateWorkTask	(contextID),
	mMapping				(NULL),
	mCapacity				(0),
	mGroups					(NULL)
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	,mLUT					(NULL)
#endif
{
	mMBP = PX_NEW(MBP)();

	const PxU32 nbObjects = maxNbStaticShapes + maxNbDynamicShapes;
	mMBP->preallocate(maxNbRegions, nbObjects, maxNbBroadPhaseOverlaps);

	if(nbObjects)
		allocateMappingArray(nbObjects);

	mCreated.reserve(DEFAULT_CREATED_DELETED_PAIRS_CAPACITY);
	mDeleted.reserve(DEFAULT_CREATED_DELETED_PAIRS_CAPACITY);
}

BroadPhaseMBP::~BroadPhaseMBP()
{
	DELETESINGLE(mMBP);
	PX_FREE(mMapping);
}

void BroadPhaseMBP::allocateMappingArray(PxU32 newCapacity)
{
	PX_ASSERT(newCapacity>mCapacity);
	MBP_Handle* newMapping = reinterpret_cast<MBP_Handle*>(PX_ALLOC(sizeof(MBP_Handle)*newCapacity, "MBP"));
	if(mCapacity)
		PxMemCopy(newMapping, mMapping, mCapacity*sizeof(MBP_Handle));
	for(PxU32 i=mCapacity;i<newCapacity;i++)
		newMapping[i] = PX_INVALID_U32;
	PX_FREE(mMapping);
	mMapping = newMapping;
	mCapacity = newCapacity;
}

bool BroadPhaseMBP::getCaps(PxBroadPhaseCaps& caps) const
{
	caps.maxNbRegions			= 256;
	caps.maxNbObjects			= 0;
	caps.needsPredefinedBounds	= true;
	return true;
}

PxU32 BroadPhaseMBP::getNbRegions() const
{
	// PT: we need to count active regions here, as we only keep track of the total number of
	// allocated regions internally - and some of which might have been removed.
	const PxU32 size = mMBP->mNbRegions;
/*	const RegionData* PX_RESTRICT regions = (const RegionData*)mMBP->mRegions.GetEntries();
	PxU32 nbActiveRegions = 0;
	for(PxU32 i=0;i<size;i++)
	{
		if(regions[i].mBP)
			nbActiveRegions++;
	}
	return nbActiveRegions;*/
	return size;
}

PxU32 BroadPhaseMBP::getRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU32 size = mMBP->mNbRegions;
	const RegionData* PX_RESTRICT regions = mMBP->mRegions.begin();
	regions += startIndex;

	const PxU32 writeCount = PxMin(size, bufferSize);
	for(PxU32 i=0;i<writeCount;i++)
	{
		const MBP_AABB& box = regions[i].mBox;
		box.decode(userBuffer[i].region.bounds);
		if(regions[i].mBP)
		{
			PX_ASSERT(userBuffer[i].region.bounds.isValid());
			userBuffer[i].region.userData	= regions[i].mUserData;
			userBuffer[i].active			= true;
			userBuffer[i].overlap			= regions[i].mOverlap!=0;
			userBuffer[i].nbStaticObjects	= regions[i].mBP->mNbStaticBoxes;
			userBuffer[i].nbDynamicObjects	= regions[i].mBP->mNbDynamicBoxes;
		}
		else
		{
			userBuffer[i].region.bounds.setEmpty();
			userBuffer[i].region.userData	= NULL;
			userBuffer[i].active			= false;
			userBuffer[i].overlap			= false;
			userBuffer[i].nbStaticObjects	= 0;
			userBuffer[i].nbDynamicObjects	= 0;
		}
	}
	return writeCount;
}

PxU32 BroadPhaseMBP::addRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	return mMBP->addRegion(region, populateRegion);
}

bool BroadPhaseMBP::removeRegion(PxU32 handle)
{
	return mMBP->removeRegion(handle);
}

void BroadPhaseMBP::update(const PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation, physx::PxBaseTask* narrowPhaseUnblockTask)
{
#if PX_CHECKED
	PX_CHECK_AND_RETURN(scratchAllocator, "BroadPhaseMBP::update - scratchAllocator must be non-NULL \n");
#endif

	// PT: TODO: move this out of update function
	if(narrowPhaseUnblockTask)
		narrowPhaseUnblockTask->removeReference();

	setUpdateData(updateData);

	if(1)
	{
		update();
		postUpdate();
	}
	else
	{
		mMBPPostUpdateWorkTask.set(this, scratchAllocator, numCpuTasks);
		mMBPUpdateWorkTask.set(this, scratchAllocator, numCpuTasks);

		mMBPPostUpdateWorkTask.setContinuation(continuation);
		mMBPUpdateWorkTask.setContinuation(&mMBPPostUpdateWorkTask);

		mMBPPostUpdateWorkTask.removeReference();
		mMBPUpdateWorkTask.removeReference();
	}
}

void BroadPhaseMBP::singleThreadedUpdate(PxcScratchAllocator* /*scratchAllocator*/, const BroadPhaseUpdateData& updateData)
{
	// PT: TODO: the scratchAllocator isn't actually needed, is it?
	setUpdateData(updateData);
	update();
	postUpdate();
}

static PX_FORCE_INLINE void computeMBPBounds(MBP_AABB& aabb, const PxBounds3* PX_RESTRICT boundsXYZ, const PxReal* PX_RESTRICT contactDistances, const BpHandle index)
{
	const PxBounds3& b = boundsXYZ[index];
	const Vec4V contactDistanceV = V4Load(contactDistances[index]);
	const Vec4V inflatedMinV = V4Sub(V4LoadU(&b.minimum.x), contactDistanceV);
	const Vec4V inflatedMaxV = V4Add(V4LoadU(&b.maximum.x), contactDistanceV);	// PT: this one is safe because we allocated one more box in the array (in BoundsArray::initEntry)

	PX_ALIGN(16, PxVec4) boxMin;
	PX_ALIGN(16, PxVec4) boxMax;
	V4StoreA(inflatedMinV, &boxMin.x);
	V4StoreA(inflatedMaxV, &boxMax.x);

	const PxU32* PX_RESTRICT min = PxUnionCast<const PxU32*, const PxF32*>(&boxMin.x);
	const PxU32* PX_RESTRICT max = PxUnionCast<const PxU32*, const PxF32*>(&boxMax.x);
	//Avoid min=max by enforcing the rule that mins are even and maxs are odd.
	aabb.mMinX = IntegerAABB::encodeFloatMin(min[0])>>1;
	aabb.mMinY = IntegerAABB::encodeFloatMin(min[1])>>1;
	aabb.mMinZ = IntegerAABB::encodeFloatMin(min[2])>>1;
	aabb.mMaxX = (IntegerAABB::encodeFloatMax(max[0]) | (1<<2))>>1;
	aabb.mMaxY = (IntegerAABB::encodeFloatMax(max[1]) | (1<<2))>>1;
	aabb.mMaxZ = (IntegerAABB::encodeFloatMax(max[2]) | (1<<2))>>1;

/*	const IntegerAABB bounds(boundsXYZ[index], contactDistances[index]);

	aabb.mMinX	= bounds.mMinMax[IntegerAABB::MIN_X]>>1;
	aabb.mMinY	= bounds.mMinMax[IntegerAABB::MIN_Y]>>1;
	aabb.mMinZ	= bounds.mMinMax[IntegerAABB::MIN_Z]>>1;
	aabb.mMaxX	= bounds.mMinMax[IntegerAABB::MAX_X]>>1;
	aabb.mMaxY	= bounds.mMinMax[IntegerAABB::MAX_Y]>>1;
	aabb.mMaxZ	= bounds.mMinMax[IntegerAABB::MAX_Z]>>1;*/

/*
	aabb.mMinX	&= ~1;
	aabb.mMinY	&= ~1;
	aabb.mMinZ	&= ~1;
	aabb.mMaxX	|= 1;
	aabb.mMaxY	|= 1;
	aabb.mMaxZ	|= 1;
*/

/*#if PX_DEBUG
	PxBounds3 decodedBox;
	PxU32* bin = reinterpret_cast<PxU32*>(&decodedBox.minimum.x);
	bin[0] = decodeFloat(bounds.mMinMax[IntegerAABB::MIN_X]);
	bin[1] = decodeFloat(bounds.mMinMax[IntegerAABB::MIN_Y]);
	bin[2] = decodeFloat(bounds.mMinMax[IntegerAABB::MIN_Z]);
	bin[3] = decodeFloat(bounds.mMinMax[IntegerAABB::MAX_X]);
	bin[4] = decodeFloat(bounds.mMinMax[IntegerAABB::MAX_Y]);
	bin[5] = decodeFloat(bounds.mMinMax[IntegerAABB::MAX_Z]);

	MBP_AABB PrunerBox;
	PrunerBox.initFrom2(decodedBox);
	PX_ASSERT(PrunerBox.mMinX==aabb.mMinX);
	PX_ASSERT(PrunerBox.mMinY==aabb.mMinY);
	PX_ASSERT(PrunerBox.mMinZ==aabb.mMinZ);
	PX_ASSERT(PrunerBox.mMaxX==aabb.mMaxX);
	PX_ASSERT(PrunerBox.mMaxY==aabb.mMaxY);
	PX_ASSERT(PrunerBox.mMaxZ==aabb.mMaxZ);
#endif*/
}

void MBPUpdateWorkTask::runInternal()
{
	mMBP->update();
}

void MBPPostUpdateWorkTask::runInternal()
{
	mMBP->postUpdate();
}

void BroadPhaseMBP::removeObjects(const BroadPhaseUpdateData& updateData)
{
	const BpHandle* PX_RESTRICT removed = updateData.getRemovedHandles();
	if(removed)
	{
		PxU32 nbToGo = updateData.getNumRemovedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *removed++;
			PX_ASSERT(index+1<mCapacity);	// PT: we allocated one more box on purpose

			const bool status = mMBP->removeObject(mMapping[index]);
			PX_ASSERT(status);
			PX_UNUSED(status);

			mMapping[index] = PX_INVALID_U32;
		}
	}
}

void BroadPhaseMBP::updateObjects(const BroadPhaseUpdateData& updateData)
{
	const BpHandle* PX_RESTRICT updated = updateData.getUpdatedHandles();
	if(updated)
	{
		const PxBounds3* PX_RESTRICT boundsXYZ = updateData.getAABBs();
		PxU32 nbToGo = updateData.getNumUpdatedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *updated++;
			PX_ASSERT(index+1<mCapacity);	// PT: we allocated one more box on purpose

			MBP_AABB aabb;
			computeMBPBounds(aabb, boundsXYZ, updateData.getContactDistance(), index);

			const bool status = mMBP->updateObject(mMapping[index], aabb);
			PX_ASSERT(status);
			PX_UNUSED(status);
		}
	}
}

void BroadPhaseMBP::addObjects(const BroadPhaseUpdateData& updateData)
{
	const BpHandle* PX_RESTRICT created = updateData.getCreatedHandles();
	if(created)
	{
		const PxBounds3* PX_RESTRICT boundsXYZ = updateData.getAABBs();
		const Bp::FilterGroup::Enum* PX_RESTRICT groups = updateData.getGroups();

		PxU32 nbToGo = updateData.getNumCreatedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *created++;
			PX_ASSERT(index+1<mCapacity);	// PT: we allocated one more box on purpose

			MBP_AABB aabb;
			computeMBPBounds(aabb, boundsXYZ, updateData.getContactDistance(), index);

			const PxU32 group = groups[index];
			const bool isStatic = group==FilterGroup::eSTATICS;

			mMapping[index] = mMBP->addObject(aabb, index, isStatic);
		}
	}
}

void BroadPhaseMBP::setUpdateData(const BroadPhaseUpdateData& updateData)
{
	mMBP->setTransientBounds(updateData.getAABBs(), updateData.getContactDistance());

	const PxU32 newCapacity = updateData.getCapacity();
	if(newCapacity>mCapacity)
		allocateMappingArray(newCapacity);

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

	// ### TODO: handle groups inside MBP
	// ### TODO: get rid of AABB conversions

	removeObjects(updateData);
	addObjects(updateData);
	updateObjects(updateData);

	PX_ASSERT(!mCreated.size());
	PX_ASSERT(!mDeleted.size());

	mMBP->prepareOverlaps();
}

void BroadPhaseMBP::update()
{
#ifdef CHECK_NB_OVERLAPS
	gNbOverlaps = 0;
#endif
	mMBP->findOverlaps(mGroups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, mLUT
#endif
		);
#ifdef CHECK_NB_OVERLAPS
	printf("PPU: %d overlaps\n", gNbOverlaps);
#endif
}

void BroadPhaseMBP::postUpdate()
{
	{
		PxU32 Nb = mMBP->mNbRegions;
		const RegionData* PX_RESTRICT regions = mMBP->mRegions.begin();
		for(PxU32 i=0;i<Nb;i++)
		{
			if(regions[i].mBP)
				regions[i].mBP->mNbUpdatedBoxes = 0;
		}
	}

	mMBP->finalize(this);
}

PxU32 BroadPhaseMBP::getNbCreatedPairs() const
{
	return mCreated.size();
}

BroadPhasePair* BroadPhaseMBP::getCreatedPairs()
{
	return mCreated.begin();
}

PxU32 BroadPhaseMBP::getNbDeletedPairs() const
{
	return mDeleted.size();
}

BroadPhasePair* BroadPhaseMBP::getDeletedPairs()
{
	return mDeleted.begin();
}

PxU32 BroadPhaseMBP::getNbOutOfBoundsObjects() const
{
	return mMBP->mOutOfBoundsObjects.size();
}

const PxU32* BroadPhaseMBP::getOutOfBoundsObjects() const
{
	return mMBP->mOutOfBoundsObjects.begin();
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

void BroadPhaseMBP::freeBuffers()
{
	mMBP->freeBuffers();
	freeBuffer(mCreated);
	freeBuffer(mDeleted);
}

#if PX_CHECKED
bool BroadPhaseMBP::isValid(const BroadPhaseUpdateData& updateData) const
{
	const BpHandle* created = updateData.getCreatedHandles();
	if(created)
	{
		Ps::HashSet<BpHandle> set;
		PxU32 nbObjects = mMBP->mMBP_Objects.size();
		const MBP_Object* PX_RESTRICT objects = mMBP->mMBP_Objects.begin();
		while(nbObjects--)
		{
			if(!(objects->mFlags & MBP_REMOVED))
				set.insert(objects->mUserID);
			objects++;
		}

		PxU32 nbToGo = updateData.getNumCreatedHandles();
		while(nbToGo--)
		{
			const BpHandle index = *created++;
			PX_ASSERT(index<mCapacity);

			if(set.contains(index))
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
			PX_ASSERT(index<mCapacity);

			if(mMapping[index]==PX_INVALID_U32)
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
			PX_ASSERT(index<mCapacity);

			if(mMapping[index]==PX_INVALID_U32)
				return false;	// This object has been removed already, or never been added
		}
	}
	return true;
}
#endif


void BroadPhaseMBP::shiftOrigin(const PxVec3& shift)
{
	mMBP->shiftOrigin(shift);
}

PxU32 BroadPhaseMBP::getCurrentNbPairs() const
{
	return mMBP->mPairManager.mNbActivePairs;
}
