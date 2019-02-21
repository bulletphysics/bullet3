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

#include "BpAABBManager.h"

#define NB_SENTINELS	6

#include "CmRenderOutput.h"
#include "CmFlushPool.h"
#include "BpBroadPhaseMBPCommon.h"
#include "BpBroadPhase.h"
#include "BpBroadPhaseShared.h"
#include "PsFoundation.h"
#include "PsSort.h"
#include "PsHashSet.h"
#include "PsVecMath.h"
#include "GuInternal.h"
#include "common/PxProfileZone.h"
//#include <stdio.h>

using namespace physx;
using namespace Bp;
using namespace Cm;
using namespace Ps::aos;

static const bool gSingleThreaded = false;
#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
	#define ABP_SIMD_OVERLAP
#endif
#ifdef ABP_SIMD_OVERLAP
	typedef AABB_YZn	AABB_YZ;
#else
	typedef AABB_YZr	AABB_YZ;
#endif

#ifdef ABP_SIMD_OVERLAP
	static const bool gUseRegularBPKernel = false;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = true;			// true to use "version 14" in box pruning series
#else
	// PT: tested on Switch, for some reason the regular version is fastest there
	static const bool gUseRegularBPKernel = true;	// false to use "version 13" in box pruning series
	static const bool gUnrollLoop = false;			// true to use "version 14" in box pruning series
#endif

namespace physx
{
namespace Bp
{
static PX_FORCE_INLINE uint32_t hash(const Pair& p)
{
	return PxU32(Ps::hash( (p.mID0&0xffff)|(p.mID1<<16)) );
}

static PX_FORCE_INLINE uint32_t hash(const AggPair& p)
{
	return PxU32(Ps::hash( (p.mIndex0&0xffff)|(p.mIndex1<<16)) );
}

static PX_FORCE_INLINE bool shouldPairBeDeleted(const Ps::Array<Bp::FilterGroup::Enum, Ps::VirtualAllocator>& groups, ShapeHandle h0, ShapeHandle h1)
{
	PX_ASSERT(h0<groups.size());
	PX_ASSERT(h1<groups.size());
	return (groups[h0]==Bp::FilterGroup::eINVALID) || (groups[h1]==Bp::FilterGroup::eINVALID);
}

// PT: TODO: refactor with CCT version
template <class T> 
static void resetOrClear(T& a)
{
	const PxU32 c = a.capacity();
	const PxU32 s = a.size();
	if(s>=c/2)
		a.clear();
	else
		a.reset();
}

///

	typedef PxU32		InflatedType;

	// PT: TODO: revisit/optimize all that stuff once it works
	class Aggregate : public Ps::UserAllocated
	{
		public:
														Aggregate(BoundsIndex index, bool selfCollisions);
														~Aggregate();

						BoundsIndex						mIndex;
		private:
						Ps::Array<BoundsIndex>			mAggregated;	// PT: TODO: replace with linked list?
		public:
						PersistentSelfCollisionPairs*	mSelfCollisionPairs;
						PxU32							mDirtyIndex;	// PT: index in mDirtyAggregates
		private:
						AABB_Xi*						mInflatedBoundsX;
						AABB_YZ*						mInflatedBoundsYZ;
						PxU32							mAllocatedSize;
		public:
		PX_FORCE_INLINE	PxU32							getNbAggregated()		const	{ return mAggregated.size();					}
		PX_FORCE_INLINE	BoundsIndex						getAggregated(PxU32 i)	const	{ return mAggregated[i];						}
		PX_FORCE_INLINE	const BoundsIndex*				getIndices()			const	{ return mAggregated.begin();					}
		PX_FORCE_INLINE	void							addAggregated(BoundsIndex i)	{ mAggregated.pushBack(i);						}
		PX_FORCE_INLINE	bool							removeAggregated(BoundsIndex i)	{ return mAggregated.findAndReplaceWithLast(i);	}	// PT: TODO: optimize?
		PX_FORCE_INLINE	const PxBounds3&				getMergedBounds()		const	{ return mBounds;								}

		PX_FORCE_INLINE	void							resetDirtyState()				{ mDirtyIndex = PX_INVALID_U32;				}
		PX_FORCE_INLINE	bool							isDirty()				const	{ return mDirtyIndex != PX_INVALID_U32;		}
		PX_FORCE_INLINE void							markAsDirty(Ps::Array<Aggregate*>& dirtyAggregates)
														{
															if(!isDirty())
															{
																mDirtyIndex = dirtyAggregates.size();
																dirtyAggregates.pushBack(this);
															}
														}

						void							allocateBounds();
						void							computeBounds(const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances) /*PX_RESTRICT*/;

		PX_FORCE_INLINE	const AABB_Xi*					getBoundsX()	const	{ return mInflatedBoundsX;	}
		PX_FORCE_INLINE	const AABB_YZ*					getBoundsYZ()	const	{ return mInflatedBoundsYZ;	}
		PX_FORCE_INLINE	void							getSortedMinBounds()
														{
															if(mDirtySort)
																sortBounds();
														}
		private:
						PxBounds3						mBounds;
						bool							mDirtySort;

						void							sortBounds();
						PX_NOCOPY(Aggregate)
	};

///

namespace
{
#define MBP_ALLOC(x)		PX_ALLOC(x, "MBP")
#define MBP_ALLOC_TMP(x)	PX_ALLOC_TEMP(x, "MBP_TMP")
#define MBP_FREE(x)			if(x)	PX_FREE_AND_RESET(x)

	struct MBPEntry;
	struct RegionHandle;
	struct MBP_Object;

	class MBP_PairManager : public PairManagerData
	{
		public:
											MBP_PairManager()	{}
											~MBP_PairManager()	{}

		PX_FORCE_INLINE	InternalPair*		addPair(PxU32 id0, PxU32 id1);
	};

///////////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE InternalPair* MBP_PairManager::addPair(PxU32 id0, PxU32 id1)
{
	PX_ASSERT(id0!=INVALID_ID);
	PX_ASSERT(id1!=INVALID_ID);
	return addPairInternal(id0, id1);
}

///////////////////////////////////////////////////////////////////////////////

}

	typedef MBP_PairManager	PairArray;

///

class PersistentPairs : public Ps::UserAllocated
{
	public:
									PersistentPairs() : mTimestamp(PX_INVALID_U32), mShouldBeDeleted(false)	{}
	virtual							~PersistentPairs()														{}

	virtual			bool			update(AABBManager& /*manager*/, BpCacheData* /*data*/ = NULL) { return false; }


	PX_FORCE_INLINE	void			updatePairs(PxU32 timestamp, const PxBounds3* bounds, const float* contactDistances, const Bp::FilterGroup::Enum* groups,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
												const bool* lut,
#endif
												Ps::Array<VolumeData>& volumeData, Ps::Array<AABBOverlap>* createdOverlaps, Ps::Array<AABBOverlap>* destroyedOverlaps);
					void			outputDeletedOverlaps(Ps::Array<AABBOverlap>* overlaps, const Ps::Array<VolumeData>& volumeData);
	private:
	virtual			void			findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
		)	= 0;
	protected:
					PxU32			mTimestamp;
					MBP_PairManager	mPM;
	public:
					bool			mShouldBeDeleted;
};

/////
	#define PosXType2	PxU32

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

	#define	CODEALIGN16		//_asm	align 16
#ifdef ABP_SIMD_OVERLAP
	#define SIMD_OVERLAP_PRELOAD_BOX0	SIMD_OVERLAP_INIT_9c(box0)
	#define SIMD_OVERLAP_TEST(x)		SIMD_OVERLAP_TEST_9c(x)
#else
	#define SIMD_OVERLAP_PRELOAD_BOX0
#endif

#ifndef ABP_SIMD_OVERLAP
static PX_FORCE_INLINE int intersect2D(const AABB_YZ& a, const AABB_YZ& b)
{
	const bool b0 = b.mMaxY < a.mMinY;
	const bool b1 = a.mMaxY < b.mMinY;
	const bool b2 = b.mMaxZ < a.mMinZ;
	const bool b3 = a.mMaxZ < b.mMinZ;
//	const bool b4 = b0 || b1 || b2 || b3;
	const bool b4 = b0 | b1 | b2 | b3;
	return !b4;
}
#endif

#ifdef ABP_SIMD_OVERLAP
	#define ABP_OVERLAP_TEST(x)	SIMD_OVERLAP_TEST(x)
#else
	#define ABP_OVERLAP_TEST(x)	if(intersect2D(box0, x))
#endif

//#define BIP_VERSION_1

	struct outputPair_Bipartite
	{
#ifdef BIP_VERSION_1
		outputPair_Bipartite(PairArray* pairManager, Aggregate* aggregate0, Aggregate* aggregate1, const Bp::FilterGroup::Enum* groups, const bool* lut) :
#else
		outputPair_Bipartite(PairArray* pairManager, const BoundsIndex* remap0, const BoundsIndex* remap1, const Bp::FilterGroup::Enum* groups, const bool* lut) :
#endif
			mPairManager	(pairManager),
#ifdef BIP_VERSION_1
			mAggregate0		(aggregate0),
			mAggregate1		(aggregate1),
#else
			mRemap0			(remap0),
			mRemap1			(remap1),
#endif
			mGroups			(groups),
			mLUT			(lut)
		{
		}

		PX_FORCE_INLINE	void outputPair(PxU32 index0, PxU32 index1)
		{
#ifdef BIP_VERSION_1
			const PxU32 aggIndex0 = mAggregate0->getAggregated(index0);
			const PxU32 aggIndex1 = mAggregate1->getAggregated(index1);
#else
			const PxU32 aggIndex0 = mRemap0[index0];
			const PxU32 aggIndex1 = mRemap1[index1];
#endif
			if(groupFiltering(mGroups[aggIndex0], mGroups[aggIndex1], mLUT))
				mPairManager->addPair(aggIndex0, aggIndex1);
		}

		PairArray*						mPairManager;
#ifdef BIP_VERSION_1
		Aggregate*						mAggregate0;
		Aggregate*						mAggregate1;
#else
		const BoundsIndex*				mRemap0;
		const BoundsIndex*				mRemap1;
#endif
		const Bp::FilterGroup::Enum*	mGroups;
		const bool*						mLUT;
	};

template<int codepath>
static void boxPruningKernel(	PairArray* PX_RESTRICT pairManager, const bool* PX_RESTRICT lut,
#ifdef BIP_VERSION_1
								Aggregate* PX_RESTRICT aggregate0, Aggregate* PX_RESTRICT aggregate1,
#else
								PxU32 nb0, const BoundsIndex* PX_RESTRICT remap0, const AABB_Xi* PX_RESTRICT boxes0_X, const AABB_YZ* PX_RESTRICT boxes0_YZ,
								PxU32 nb1, const BoundsIndex* PX_RESTRICT remap1, const AABB_Xi* PX_RESTRICT boxes1_X, const AABB_YZ* PX_RESTRICT boxes1_YZ,
#endif
								const Bp::FilterGroup::Enum* PX_RESTRICT groups)
{
#ifdef BIP_VERSION_1
	outputPair_Bipartite pm(pairManager, aggregate0, aggregate1, groups, lut);

	const PxU32 nb0 =  aggregate0->getNbAggregated();
	const PxU32 nb1 =  aggregate1->getNbAggregated();
	const AABB_Xi* PX_RESTRICT boxes0_X = aggregate0->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes0_YZ = aggregate0->getBoundsYZ();
	const AABB_Xi* PX_RESTRICT boxes1_X = aggregate1->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes1_YZ = aggregate1->getBoundsYZ();
#else
//	outputPair_Bipartite pm(pairManager, aggregate0->getIndices(), aggregate1->getIndices(), groups, lut);
	outputPair_Bipartite pm(pairManager, remap0, remap1, groups, lut);
#endif

	PxU32 index0 = 0;
	PxU32 runningIndex1 = 0;

	while(runningIndex1<nb1 && index0<nb0)
	{
		const AABB_Xi& box0_X = boxes0_X[index0];
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

		const AABB_YZ& box0 = boxes0_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex1;

			while(boxes1_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes1_YZ[index1])
				{
					pm.outputPair(index0, index1);
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
					if(intersect2D(box0, *reinterpret_cast<const AABB_YZ*>(box)))
#endif
					{
						const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
						pm.outputPair(index0, Index1);
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
								if(intersect2D(box0, *reinterpret_cast<const AABB_YZ*>(box)))	\
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
			pm.outputPair(index0, Index1);
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
				{if(intersect2D(box0, *reinterpret_cast<const AABB_YZ*>(CurrentBoxListYZ + Offset*2)))	\
						goto OverlapFound;										\
					Offset += 8;
#endif

		goto LoopStart;
		CODEALIGN16
OverlapFound:
		{
			const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
			pm.outputPair(index0, Index1);
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

static PX_FORCE_INLINE void doBipartiteBoxPruning_Leaf(
		PairArray* PX_RESTRICT pairManager, const bool* PX_RESTRICT lut,
		Aggregate* PX_RESTRICT aggregate0, Aggregate* PX_RESTRICT aggregate1, const Bp::FilterGroup::Enum* PX_RESTRICT groups)
{
#ifdef BIP_VERSION_1
	boxPruningKernel<0>(pairManager, lut, aggregate0, aggregate1, groups);
	boxPruningKernel<1>(pairManager, lut, aggregate1, aggregate0, groups);
#else
	const PxU32 nb0 =  aggregate0->getNbAggregated();
	const PxU32 nb1 =  aggregate1->getNbAggregated();
	const BoundsIndex* PX_RESTRICT remap0 = aggregate0->getIndices();
	const BoundsIndex* PX_RESTRICT remap1 = aggregate1->getIndices();
	const AABB_Xi* PX_RESTRICT boxes0_X = aggregate0->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes0_YZ = aggregate0->getBoundsYZ();
	const AABB_Xi* PX_RESTRICT boxes1_X = aggregate1->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes1_YZ = aggregate1->getBoundsYZ();
	boxPruningKernel<0>(pairManager, lut, nb0, remap0, boxes0_X, boxes0_YZ, nb1, remap1, boxes1_X, boxes1_YZ, groups);
	boxPruningKernel<1>(pairManager, lut, nb1, remap1, boxes1_X, boxes1_YZ, nb0, remap0, boxes0_X, boxes0_YZ, groups);
#endif
}

	struct outputPair_Complete
	{
		outputPair_Complete(PairArray* pairManager, Aggregate* aggregate, const Bp::FilterGroup::Enum* groups, const bool* lut) :
			mPairManager	(pairManager),
			mAggregate		(aggregate),
			mGroups			(groups),
			mLUT			(lut)
		{
		}

		PX_FORCE_INLINE	void outputPair(PxU32 index0, PxU32 index1)
		{
			const PxU32 aggIndex0 = mAggregate->getAggregated(index0);
			const PxU32 aggIndex1 = mAggregate->getAggregated(index1);

			if(groupFiltering(mGroups[aggIndex0], mGroups[aggIndex1], mLUT))
				mPairManager->addPair(aggIndex0, aggIndex1);
		}

		PairArray*						mPairManager;
		Aggregate*						mAggregate;
		const Bp::FilterGroup::Enum*	mGroups;
		const bool*						mLUT;
	};

static void doCompleteBoxPruning_Leaf(	PairArray* PX_RESTRICT pairManager, const bool* PX_RESTRICT lut,
										Aggregate* PX_RESTRICT aggregate, const Bp::FilterGroup::Enum* PX_RESTRICT groups)
{
	outputPair_Complete pm(pairManager, aggregate, groups, lut);
	const PxU32 nb =  aggregate->getNbAggregated();
	const AABB_Xi* PX_RESTRICT boxes_X = aggregate->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes_YZ = aggregate->getBoundsYZ();

	PxU32 index0 = 0;
	PxU32 runningIndex = 0;
	while(runningIndex<nb && index0<nb)
	{
		const AABB_Xi& box0_X = boxes_X[index0];
		const PosXType2 maxLimit = box0_X.mMaxX;

		const PosXType2 minLimit = box0_X.mMinX;
		while(boxes_X[runningIndex++].mMinX<minLimit);

		const AABB_YZ& box0 = boxes_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex;
			while(boxes_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes_YZ[index1])
				{
					pm.outputPair(index0, index1);
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
					if(intersect2D(box0, *reinterpret_cast<const AABB_YZ*>(box)))
#endif
					{
						const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
						pm.outputPair(index0, Index);
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
	#define BLOCK4(x, label)	{const AABB_YZ* box = reinterpret_cast<const AABB_YZ*>(CurrentBoxListYZ + Offset*2 + x*2);	\
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
			pm.outputPair(index0, Index);
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
				{if(intersect2D(box0, *reinterpret_cast<const AABB_YZ*>(CurrentBoxListYZ + Offset*2)))	\
						goto BeforeLoop;										\
					Offset += 8;
#endif

		goto StartLoop;
		CODEALIGN16
BeforeLoop:
		{
			const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
			pm.outputPair(index0, Index);
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

/////

class PersistentActorAggregatePair : public PersistentPairs
{
	public:
								PersistentActorAggregatePair(Aggregate* aggregate, ShapeHandle actorHandle);
	virtual						~PersistentActorAggregatePair()	{}

	virtual			void		findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
		);
	virtual			bool		update(AABBManager& manager, BpCacheData* data);

					ShapeHandle	mAggregateHandle;
					ShapeHandle	mActorHandle;
					Aggregate*	mAggregate;
};

PersistentActorAggregatePair::PersistentActorAggregatePair(Aggregate* aggregate, ShapeHandle actorHandle) :
	mAggregateHandle	(aggregate->mIndex),
	mActorHandle		(actorHandle),
	mAggregate			(aggregate)
{
}

void PersistentActorAggregatePair::findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
	)
{
	if(0)
	{
		Aggregate singleActor(INVALID_ID, false);
		singleActor.addAggregated(mActorHandle);
		singleActor.allocateBounds();
		singleActor.computeBounds(bounds, contactDistances);
		singleActor.getSortedMinBounds();
		mAggregate->getSortedMinBounds();
		doBipartiteBoxPruning_Leaf(&pairs, lut, &singleActor, mAggregate, groups);
	}
	else
	{

	mAggregate->getSortedMinBounds();
	const PxU32 nb0 = mAggregate->getNbAggregated();
	const BoundsIndex* PX_RESTRICT remap0 = mAggregate->getIndices();
	const AABB_Xi* PX_RESTRICT boxes0_X = mAggregate->getBoundsX();
	const AABB_YZ* PX_RESTRICT boxes0_YZ = mAggregate->getBoundsYZ();

	PX_ALIGN(16, AABB_Xi inflatedBoundsX[1+NB_SENTINELS]);
	PX_ALIGN(16, AABB_YZ inflatedBoundsYZ);

	// Compute bounds
	{
		PX_ALIGN(16, PxVec4) boxMin;
		PX_ALIGN(16, PxVec4) boxMax;
		const BoundsIndex actorHandle = mActorHandle;
		const PxBounds3& b = bounds[actorHandle];
		const Vec4V offsetV = V4Load(contactDistances[actorHandle]);
		const Vec4V minimumV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		const Vec4V maximumV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		V4StoreA(minimumV, &boxMin.x);
		V4StoreA(maximumV, &boxMax.x);
		inflatedBoundsX[0].initFromPxVec4(boxMin, boxMax);
		inflatedBoundsYZ.initFromPxVec4(boxMin, boxMax);

		for(PxU32 i=0;i<NB_SENTINELS;i++)
			inflatedBoundsX[1+i].initSentinel();
	}

	const PxU32 nb1 = 1;
	const BoundsIndex* PX_RESTRICT remap1 = &mActorHandle;
	const AABB_Xi* PX_RESTRICT boxes1_X = inflatedBoundsX;
	const AABB_YZ* PX_RESTRICT boxes1_YZ = &inflatedBoundsYZ;

	boxPruningKernel<0>(&pairs, lut, nb0, remap0, boxes0_X, boxes0_YZ, nb1, remap1, boxes1_X, boxes1_YZ, groups);
	boxPruningKernel<1>(&pairs, lut, nb1, remap1, boxes1_X, boxes1_YZ, nb0, remap0, boxes0_X, boxes0_YZ, groups);

	}
}

bool PersistentActorAggregatePair::update(AABBManager& manager, BpCacheData* data)
{
	if(mShouldBeDeleted || shouldPairBeDeleted(manager.mGroups, mAggregateHandle, mActorHandle))
		return true;

	if(!mAggregate->getNbAggregated())	// PT: needed with lazy empty actors
		return true;

	if(mAggregate->isDirty() || manager.mChangedHandleMap.boundedTest(mActorHandle))
		manager.updatePairs(*this, data);

	return false;
}

/////

class PersistentAggregateAggregatePair : public PersistentPairs
{
	public:
								PersistentAggregateAggregatePair(Aggregate* aggregate0, Aggregate* aggregate1);
	virtual						~PersistentAggregateAggregatePair()	{}

	virtual			void		findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
		);
	virtual			bool		update(AABBManager& manager, BpCacheData*);

					ShapeHandle	mAggregateHandle0;
					ShapeHandle	mAggregateHandle1;
					Aggregate*	mAggregate0;
					Aggregate*	mAggregate1;
};

PersistentAggregateAggregatePair::PersistentAggregateAggregatePair(Aggregate* aggregate0, Aggregate* aggregate1) :
	mAggregateHandle0	(aggregate0->mIndex),
	mAggregateHandle1	(aggregate1->mIndex),
	mAggregate0			(aggregate0),
	mAggregate1			(aggregate1)
{
}

void PersistentAggregateAggregatePair::findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT /*bounds*/, const float* PX_RESTRICT /*contactDistances*/, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
	)
{
	mAggregate0->getSortedMinBounds();
	mAggregate1->getSortedMinBounds();
	doBipartiteBoxPruning_Leaf(&pairs, lut, mAggregate0, mAggregate1, groups);
}

bool PersistentAggregateAggregatePair::update(AABBManager& manager, BpCacheData* data)
{
	if(mShouldBeDeleted || shouldPairBeDeleted(manager.mGroups, mAggregateHandle0, mAggregateHandle1))
		return true;

	if(!mAggregate0->getNbAggregated() || !mAggregate1->getNbAggregated())	// PT: needed with lazy empty actors
		return true;

	if(mAggregate0->isDirty() || mAggregate1->isDirty())
		manager.updatePairs(*this, data);

	return false;
}

/////

class PersistentSelfCollisionPairs : public PersistentPairs
{
	public:
						PersistentSelfCollisionPairs(Aggregate* aggregate);
	virtual				~PersistentSelfCollisionPairs()	{}

	virtual	void		findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
		);

			Aggregate*	mAggregate;
};

PersistentSelfCollisionPairs::PersistentSelfCollisionPairs(Aggregate* aggregate) :
	mAggregate	(aggregate)
{
}

void PersistentSelfCollisionPairs::findOverlaps(PairArray& pairs, const PxBounds3* PX_RESTRICT/*bounds*/, const float* PX_RESTRICT/*contactDistances*/, const Bp::FilterGroup::Enum* PX_RESTRICT groups
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	, const bool* PX_RESTRICT lut
#endif
	)
{
	mAggregate->getSortedMinBounds();
	doCompleteBoxPruning_Leaf(&pairs, lut, mAggregate, groups);
}

/////

Aggregate::Aggregate(BoundsIndex index, bool selfCollisions) :
	mIndex			(index),
	mInflatedBoundsX	(NULL),
	mInflatedBoundsYZ	(NULL),
	mAllocatedSize	(0),
	mDirtySort		(false)
{
	resetDirtyState();
	mSelfCollisionPairs = selfCollisions ? PX_NEW(PersistentSelfCollisionPairs)(this) : NULL;
}

Aggregate::~Aggregate()
{
	PX_FREE_AND_RESET(mInflatedBoundsYZ);
	PX_FREE_AND_RESET(mInflatedBoundsX);

	if(mSelfCollisionPairs)
		PX_DELETE_AND_RESET(mSelfCollisionPairs);
}

void Aggregate::sortBounds()
{
	mDirtySort = false;
	const PxU32 nbObjects = getNbAggregated();
	if(nbObjects<2)
		return;

	{
		PX_ALLOCA(minPosBounds, InflatedType, nbObjects+1);
		bool alreadySorted =  true;
		InflatedType previousB = mInflatedBoundsX[0].mMinX;
		minPosBounds[0] = previousB;
		for(PxU32 i=1;i<nbObjects;i++)
		{
			const InflatedType minB = mInflatedBoundsX[i].mMinX;
			if(minB<previousB)
				alreadySorted = false;
			previousB = minB;
			minPosBounds[i] = minB;
		}
		if(alreadySorted)
			return;

		{
		Cm::RadixSortBuffered mRS;

		minPosBounds[nbObjects] = 0xffffffff;
		mRS.Sort(minPosBounds, nbObjects+1, /*RadixHint::*/RADIX_UNSIGNED);

		if(0)
		{
			Ps::Array<PxU32> copy = mAggregated;
			AABB_Xi* boundsXCopy = reinterpret_cast<AABB_Xi*>(PX_ALLOC(sizeof(AABB_Xi)*(nbObjects), "mInflatedBounds"));
			AABB_YZ* boundsYZCopy = reinterpret_cast<AABB_YZ*>(PX_ALLOC(sizeof(AABB_YZ)*(nbObjects), "mInflatedBounds"));
			PxMemCopy(boundsXCopy, mInflatedBoundsX, nbObjects*sizeof(AABB_Xi));
			PxMemCopy(boundsYZCopy, mInflatedBoundsYZ, nbObjects*sizeof(AABB_YZ));
			const PxU32* Sorted = mRS.GetRanks();
			for(PxU32 i=0;i<nbObjects;i++)
			{
				const PxU32 sortedIndex = Sorted[i];
				mAggregated[i] = copy[sortedIndex];
				mInflatedBoundsX[i] = boundsXCopy[sortedIndex];
				mInflatedBoundsYZ[i] = boundsYZCopy[sortedIndex];
			}
			PX_FREE(boundsYZCopy);
			PX_FREE(boundsXCopy);
		}
		else
		{
			Ps::Array<PxU32> copy = mAggregated;	// PT: TODO: revisit this, avoid the copy like we do for the other buffers
			AABB_Xi* sortedBoundsX = reinterpret_cast<AABB_Xi*>(PX_ALLOC(sizeof(AABB_Xi)*(nbObjects+NB_SENTINELS), "mInflatedBounds"));
			AABB_YZ* sortedBoundsYZ = reinterpret_cast<AABB_YZ*>(PX_ALLOC(sizeof(AABB_YZ)*(nbObjects), "mInflatedBounds"));

			const PxU32* Sorted = mRS.GetRanks();
			for(PxU32 i=0;i<nbObjects;i++)
			{
				const PxU32 sortedIndex = Sorted[i];
				mAggregated[i] = copy[sortedIndex];
				sortedBoundsX[i] = mInflatedBoundsX[sortedIndex];
				sortedBoundsYZ[i] = mInflatedBoundsYZ[sortedIndex];
			}
			for(PxU32 i=0;i<NB_SENTINELS;i++)
				sortedBoundsX[nbObjects+i].initSentinel();
			mAllocatedSize = nbObjects;
			PX_FREE(mInflatedBoundsYZ);
			PX_FREE(mInflatedBoundsX);
			mInflatedBoundsX = sortedBoundsX;
			mInflatedBoundsYZ = sortedBoundsYZ;
		}
		}
	}
}

void Aggregate::allocateBounds()
{
	const PxU32 size = getNbAggregated();
	if(size!=mAllocatedSize)
	{
		mAllocatedSize = size;
		PX_FREE(mInflatedBoundsYZ);
		PX_FREE(mInflatedBoundsX);
		mInflatedBoundsX = reinterpret_cast<AABB_Xi*>(PX_ALLOC(sizeof(AABB_Xi)*(size+NB_SENTINELS), "mInflatedBounds"));
		mInflatedBoundsYZ = reinterpret_cast<AABB_YZ*>(PX_ALLOC(sizeof(AABB_YZ)*(size), "mInflatedBounds"));
	}
}

void Aggregate::computeBounds(const PxBounds3* PX_RESTRICT bounds, const float* PX_RESTRICT contactDistances) /*PX_RESTRICT*/
{
//	PX_PROFILE_ZONE("Aggregate::computeBounds",0);

	const PxU32 size = getNbAggregated();
	PX_ASSERT(size);

	// PT: TODO: delay the conversion to integers until we sort (i.e. really need) the aggregated bounds?
	PX_ALIGN(16, PxVec4) boxMin;
	PX_ALIGN(16, PxVec4) boxMax;

	const PxU32 lookAhead = 4;
	Vec4V minimumV;
	Vec4V maximumV;
	{
		const BoundsIndex index0 = getAggregated(0);
		const PxU32 last = PxMin(lookAhead, size-1);
		for(PxU32 i=1;i<=last;i++)
		{
			const BoundsIndex index = getAggregated(i);
			Ps::prefetchLine(bounds + index, 0);
			Ps::prefetchLine(contactDistances + index, 0);
		}
		const PxBounds3& b = bounds[index0];
		const Vec4V offsetV = V4Load(contactDistances[index0]);
		minimumV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		maximumV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		V4StoreA(minimumV, &boxMin.x);
		V4StoreA(maximumV, &boxMax.x);
		mInflatedBoundsX[0].initFromPxVec4(boxMin, boxMax);
		mInflatedBoundsYZ[0].initFromPxVec4(boxMin, boxMax);
	}

	for(PxU32 i=1;i<size;i++)
	{
		const BoundsIndex index = getAggregated(i);
		if(i+lookAhead<size)
		{
			const BoundsIndex nextIndex = getAggregated(i+lookAhead);
			Ps::prefetchLine(bounds + nextIndex, 0);
			Ps::prefetchLine(contactDistances + nextIndex, 0);
		}
		const PxBounds3& b = bounds[index];
		const Vec4V offsetV = V4Load(contactDistances[index]);
		const Vec4V aggregatedBoundsMinV = V4Sub(V4LoadU(&b.minimum.x), offsetV);
		const Vec4V aggregatedBoundsMaxV = V4Add(V4LoadU(&b.maximum.x), offsetV);
		minimumV = V4Min(minimumV, aggregatedBoundsMinV);
		maximumV = V4Max(maximumV, aggregatedBoundsMaxV);
		V4StoreA(aggregatedBoundsMinV, &boxMin.x);
		V4StoreA(aggregatedBoundsMaxV, &boxMax.x);
		mInflatedBoundsX[i].initFromPxVec4(boxMin, boxMax);
		mInflatedBoundsYZ[i].initFromPxVec4(boxMin, boxMax);
	}

	StoreBounds(mBounds, minimumV, maximumV);
//	StoreBounds(boundsArray.begin()[mIndex], minimumV, maximumV);
//	boundsArray.setChangedState();

/*	if(0)
	{
		const PxBounds3& previousBounds = boundsArray.getBounds(mIndex);
		if(previousBounds.minimum==aggregateBounds.minimum && previousBounds.maximum==aggregateBounds.maximum)
		{
			// PT: same bounds as before
			printf("SAME BOUNDS\n");
		}
	}*/
	for(PxU32 i=0;i<NB_SENTINELS;i++)
		mInflatedBoundsX[size+i].initSentinel();
	mDirtySort = true;
}

/////

void AABBManager::reserveShapeSpace(PxU32 nbTotalBounds)
{
	nbTotalBounds = Ps::nextPowerOfTwo(nbTotalBounds);
	mGroups.resize(nbTotalBounds, Bp::FilterGroup::eINVALID);
	mVolumeData.resize(nbTotalBounds);					//KS - must be initialized so that userData is NULL for SQ-only shapes
	mContactDistance.resizeUninitialized(nbTotalBounds);
	mAddedHandleMap.resize(nbTotalBounds);
	mRemovedHandleMap.resize(nbTotalBounds);
}

#if PX_VC
#pragma warning(disable: 4355 )	// "this" used in base member initializer list
#endif

AABBManager::AABBManager(	BroadPhase& bp, BoundsArray& boundsArray, Ps::Array<PxReal, Ps::VirtualAllocator>& contactDistance,
							PxU32 maxNbAggregates, PxU32 maxNbShapes, Ps::VirtualAllocator& allocator, PxU64 contextID,
							PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode) :
	mPostBroadPhase2(contextID, *this),
	mPostBroadPhase3(contextID, this, "AABBManager::postBroadPhaseStage3"),
	mFinalizeUpdateTask			(contextID),
	mChangedHandleMap			(allocator),
	mGroups						(allocator),
	mContactDistance			(contactDistance),
	mVolumeData					(PX_DEBUG_EXP("AABBManager::mVolumeData")),
	mAddedHandles				(allocator),
	mUpdatedHandles				(allocator),
	mRemovedHandles				(allocator),
	mBroadPhase					(bp),
	mBoundsArray				(boundsArray),
	mOutOfBoundsObjects			(PX_DEBUG_EXP("AABBManager::mOutOfBoundsObjects")),
	mOutOfBoundsAggregates		(PX_DEBUG_EXP("AABBManager::mOutOfBoundsAggregates")),
	//mCreatedOverlaps			{ Ps::Array<Bp::AABBOverlap>(PX_DEBUG_EXP("AABBManager::mCreatedOverlaps")) },
	//mDestroyedOverlaps			{ Ps::Array<Bp::AABBOverlap>(PX_DEBUG_EXP("AABBManager::mDestroyedOverlaps")) },
	mScratchAllocator			(NULL),
	mNarrowPhaseUnblockTask		(NULL),
	mUsedSize					(0),
	mOriginShifted				(false),
	mPersistentStateChanged		(true),
	mNbAggregates				(0),
	mFirstFreeAggregate			(PX_INVALID_U32),
	mTimestamp					(0),
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	mAggregateGroupTide			(PxU32(Bp::FilterGroup::eAGGREGATE_BASE)),
#endif
	mContextID					(contextID)
{
	PX_UNUSED(maxNbAggregates);	// PT: TODO: use it or remove it
	reserveShapeSpace(PxMax(maxNbShapes, 1u));

//	mCreatedOverlaps.reserve(16000);
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	{
		const bool discardKineKine = kineKineFilteringMode==PxPairFilteringMode::eKILL ? true : false;
		const bool discardStaticKine = staticKineFilteringMode==PxPairFilteringMode::eKILL ? true : false;

		for(int j=0;j<Bp::FilterType::COUNT;j++)
			for(int i=0;i<Bp::FilterType::COUNT;i++)
				mLUT[j][i] = false;
		mLUT[Bp::FilterType::STATIC][Bp::FilterType::DYNAMIC] = mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::STATIC] = true;
		mLUT[Bp::FilterType::STATIC][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::STATIC] = !discardStaticKine;
		mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::KINEMATIC] = mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::DYNAMIC] = true;
		mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::DYNAMIC] = true;
		mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::KINEMATIC] = !discardKineKine;

		mLUT[Bp::FilterType::STATIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::STATIC] = true;
		mLUT[Bp::FilterType::KINEMATIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::KINEMATIC] = true;
		mLUT[Bp::FilterType::DYNAMIC][Bp::FilterType::AGGREGATE] = mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::DYNAMIC] = true;
		mLUT[Bp::FilterType::AGGREGATE][Bp::FilterType::AGGREGATE] = true;
	}
#else
	PX_UNUSED(kineKineFilteringMode);
	PX_UNUSED(staticKineFilteringMode);
#endif
}

static void releasePairs(AggPairMap& map)
{
	for(AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
		PX_DELETE(iter->second);
}

void AABBManager::destroy()
{
	releasePairs(mActorAggregatePairs);
	releasePairs(mAggregateAggregatePairs);

	const PxU32 nb = mAggregates.size();
	for(PxU32 i=0;i<nb;i++)
	{
		bool found = false;
		PxU32 currentFree = mFirstFreeAggregate;
		while(currentFree!=PX_INVALID_U32)
		{
			if(currentFree==i)
			{
				found = true;
				break;
			}
			currentFree = PxU32(reinterpret_cast<size_t>(mAggregates[currentFree]));
		}

		if(!found)
		{
			Aggregate* a = mAggregates[i];
			PX_DELETE(a);
		}
	}

	BpCacheData* entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	while (entry)
	{
		entry->~BpCacheData();
		PX_FREE(entry);
		entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	}

	PX_DELETE(this);
}

/*bool AABBManager::checkID(ShapeHandle id)
{
	for(AggPairMap::Iterator iter = mActorAggregatePairs.getIterator(); !iter.done(); ++iter)
	{
		PersistentActorAggregatePair* p = static_cast<PersistentActorAggregatePair*>(iter->second);
		if(p->mActorHandle==id || p->mAggregateHandle==id)
			return false;
	}

	for(AggPairMap::Iterator iter = mAggregateAggregatePairs.getIterator(); !iter.done(); ++iter)
	{
		PersistentAggregateAggregatePair* p = static_cast<PersistentAggregateAggregatePair*>(iter->second);
		if(p->mAggregateHandle0==id || p->mAggregateHandle1==id)
			return false;
	}
	return true;
}*/

static void removeAggregateFromDirtyArray(Aggregate* aggregate, Ps::Array<Aggregate*>& dirtyAggregates)
{
	// PT: TODO: do this lazily like for interactions?
	if(aggregate->isDirty())
	{
		const PxU32 dirtyIndex = aggregate->mDirtyIndex;
		PX_ASSERT(dirtyAggregates[dirtyIndex]==aggregate);
		dirtyAggregates.replaceWithLast(dirtyIndex);
		if(dirtyIndex<dirtyAggregates.size())
			dirtyAggregates[dirtyIndex]->mDirtyIndex = dirtyIndex;
		aggregate->resetDirtyState();
	}
	else
	{
		PX_ASSERT(!dirtyAggregates.findAndReplaceWithLast(aggregate));
	}
}

void AABBManager::reserveSpaceForBounds(BoundsIndex index)
{
	if ((index+1) >= mVolumeData.size())
		reserveShapeSpace(index+1);

	resetEntry(index); //KS - make sure this entry is flagged as invalid
}

// PT: TODO: what is the "userData" here?
bool AABBManager::addBounds(BoundsIndex index, PxReal contactDistance, Bp::FilterGroup::Enum group, void* userData, AggregateHandle aggregateHandle, ElementType::Enum volumeType)
{
//	PX_ASSERT(checkID(index));

	initEntry(index, contactDistance, group, userData);
	mVolumeData[index].setVolumeType(volumeType);

	if(aggregateHandle==PX_INVALID_U32)
	{
		mVolumeData[index].setSingleActor();

		addBPEntry(index);

		mPersistentStateChanged = true;
	}
	else
	{
#if PX_CHECKED
		if(aggregateHandle>=mAggregates.size())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "AABBManager::addBounds - aggregateId out of bounds\n");
			return false;
		}

/*		{
			PxU32 firstFreeAggregate = mFirstFreeAggregate;
			while(firstFreeAggregate!=PX_INVALID_U32)
			{
				if(firstFreeAggregate==aggregateHandle)
				{
					Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "AABBManager::destroyAggregate - aggregate has already been removed\n");
					return BP_INVALID_BP_HANDLE;
				}
				firstFreeAggregate = PxU32(reinterpret_cast<size_t>(mAggregates[firstFreeAggregate]));
			}
		}*/
#endif
		mVolumeData[index].setAggregated(aggregateHandle);

		mPersistentStateChanged = true;		// PT: TODO: do we need this here?

		Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);

		{
			// PT: schedule the aggregate for BP insertion here, if we just added its first shape
			if(!aggregate->getNbAggregated())
				addBPEntry(aggregate->mIndex);

			aggregate->addAggregated(index);

			// PT: new actor added to aggregate => mark dirty to recompute bounds later
			aggregate->markAsDirty(mDirtyAggregates);
		}
	}

	// PT: TODO: remove or use this return value. Currently useless since always true. Gives birth to unreachable code in callers.
	return true;
}

void AABBManager::removeBounds(BoundsIndex index)
{
	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	if(mVolumeData[index].isSingleActor())
	{
		removeBPEntry(index);

		mPersistentStateChanged = true;
	}
	else
	{
		PX_ASSERT(mVolumeData[index].isAggregated());

		const AggregateHandle aggregateHandle = mVolumeData[index].getAggregateOwner();
		Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
		bool status = aggregate->removeAggregated(index);
		(void)status;
//		PX_ASSERT(status);	// PT: can be false when >128 shapes

		// PT: remove empty aggregates, otherwise the BP will crash with empty bounds
		if(!aggregate->getNbAggregated())
		{
			removeBPEntry(aggregate->mIndex);
			removeAggregateFromDirtyArray(aggregate, mDirtyAggregates);
		}
		else
			aggregate->markAsDirty(mDirtyAggregates);	// PT: actor removed from aggregate => mark dirty to recompute bounds later

		mPersistentStateChanged = true;	// PT: TODO: do we need this here?
	}

	resetEntry(index);
}

// PT: TODO: the userData is actually a PxAggregate pointer. Maybe we could expose/use that.
AggregateHandle AABBManager::createAggregate(BoundsIndex index, Bp::FilterGroup::Enum group, void* userData, const bool selfCollisions)
{
//	PX_ASSERT(checkID(index));

	Aggregate* aggregate = PX_NEW(Aggregate)(index, selfCollisions);

	AggregateHandle handle;
	if(mFirstFreeAggregate==PX_INVALID_U32)
	{
		handle = mAggregates.size();
		mAggregates.pushBack(aggregate);
	}
	else
	{
		handle = mFirstFreeAggregate;
		mFirstFreeAggregate = PxU32(reinterpret_cast<size_t>(mAggregates[mFirstFreeAggregate]));
		mAggregates[handle] = aggregate;
	}

#ifdef BP_USE_AGGREGATE_GROUP_TAIL
/*	#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		PxU32 id = index;
		id<<=2;
		id|=FilterType::AGGREGATE;
		initEntry(index, 0.0f, Bp::FilterGroup::Enum(id), userData);
	#endif*/
	initEntry(index, 0.0f, getAggregateGroup(), userData);
	PX_UNUSED(group);
#else
	initEntry(index, 0.0f, group, userData);
#endif

	mVolumeData[index].setAggregate(handle);

	mBoundsArray.setBounds(PxBounds3::empty(), index);	// PT: no need to set mPersistentStateChanged since "setBounds" already does something similar

	mNbAggregates++;

	// PT: we don't add empty aggregates to mAddedHandleMap yet, since they make the BP crash.
	return handle;
}

bool AABBManager::destroyAggregate(BoundsIndex& index_, Bp::FilterGroup::Enum& group_, AggregateHandle aggregateHandle)
{
#if PX_CHECKED
	if(aggregateHandle>=mAggregates.size())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "AABBManager::destroyAggregate - aggregateId out of bounds\n");
		return false;
	}

	{
		PxU32 firstFreeAggregate = mFirstFreeAggregate;
		while(firstFreeAggregate!=PX_INVALID_U32)
		{
			if(firstFreeAggregate==aggregateHandle)
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "AABBManager::destroyAggregate - aggregate has already been removed\n");
				return false;
			}
			firstFreeAggregate = PxU32(reinterpret_cast<size_t>(mAggregates[firstFreeAggregate]));
		}
	}
#endif

	Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);

#if PX_CHECKED
	if(aggregate->getNbAggregated())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "AABBManager::destroyAggregate - aggregate still has bounds that needs removed\n");
		return false;
	}
#endif

	const BoundsIndex index = aggregate->mIndex;
	removeAggregateFromDirtyArray(aggregate, mDirtyAggregates);

	if(mAddedHandleMap.test(index))			// PT: if object had been added this frame...
		mAddedHandleMap.reset(index);		// PT: ...then simply revert the previous operation locally (it hasn't been passed to the BP yet).
	else if(aggregate->getNbAggregated())	// PT: else we need to remove it from the BP if it has been added there. If there's no aggregated
		mRemovedHandleMap.set(index);		// PT: shapes then the aggregate has never been added, or already removed.

	PX_DELETE(aggregate);
	mAggregates[aggregateHandle] = reinterpret_cast<Aggregate*>(size_t(mFirstFreeAggregate));
	mFirstFreeAggregate = aggregateHandle;

	// PT: TODO: shouldn't it be compared to mUsedSize?
	PX_ASSERT(index < mVolumeData.size());

	index_ = index;
	group_ = mGroups[index];

#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	releaseAggregateGroup(mGroups[index]);
#endif
	resetEntry(index);

	mPersistentStateChanged = true;

	PX_ASSERT(mNbAggregates);
	mNbAggregates--;

	return true;
}

void AABBManager::handleOriginShift()
{
	mOriginShifted = false;
	mPersistentStateChanged = true;
	// PT: TODO: isn't the following loop potentially updating removed objects?
	// PT: TODO: check that aggregates code is correct here
	for(PxU32 i=0; i<mUsedSize; i++)
	{
		if(mGroups[i] == Bp::FilterGroup::eINVALID)
			continue;

		{
			if(mVolumeData[i].isSingleActor())
			{
				if(!mAddedHandleMap.test(i))
					mUpdatedHandles.pushBack(i);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
			}
			else if(mVolumeData[i].isAggregate())
			{
				const AggregateHandle aggregateHandle = mVolumeData[i].getAggregate();
				Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
				if(aggregate->getNbAggregated())
				{
					aggregate->markAsDirty(mDirtyAggregates);
					aggregate->allocateBounds();
					aggregate->computeBounds(mBoundsArray.begin(), mContactDistance.begin());
					mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
					if(!mAddedHandleMap.test(i))
						mUpdatedHandles.pushBack(i);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}
}

void AggregateBoundsComputationTask::runInternal()
{
	const BoundsArray& boundArray = mManager->getBoundsArray();
	const float* contactDistances = mManager->getContactDistances();

	PxU32 size = mNbToGo;
	Aggregate** currentAggregate = mAggregates + mStart;
	while(size--)
	{
		if(size)
		{
			Aggregate* nextAggregate = *(currentAggregate+1);
			Ps::prefetchLine(nextAggregate, 0);
			Ps::prefetchLine(nextAggregate, 64);
		}

		(*currentAggregate)->computeBounds(boundArray.begin(), contactDistances);
		currentAggregate++;
	}
}

void FinalizeUpdateTask::runInternal()
{
	mManager->finalizeUpdate(mNumCpuTasks, mScratchAllocator, getContinuation(), mNarrowPhaseUnlockTask);
}

void AABBManager::startAggregateBoundsComputationTasks(PxU32 nbToGo, PxU32 numCpuTasks, Cm::FlushPool& flushPool)
{
	const PxU32 nbAggregatesPerTask = nbToGo > numCpuTasks ? nbToGo / numCpuTasks : nbToGo;

	// PT: TODO: better load balancing

	PxU32 start = 0;
	while(nbToGo)
	{
		AggregateBoundsComputationTask* T = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(AggregateBoundsComputationTask)), AggregateBoundsComputationTask(mContextID));

		const PxU32 nb = nbToGo < nbAggregatesPerTask ? nbToGo : nbAggregatesPerTask;
		T->Init(this, start, nb, mDirtyAggregates.begin());
		start += nb;
		nbToGo -= nb;

		T->setContinuation(&mFinalizeUpdateTask);
		T->removeReference();
	}
}

void AABBManager::updateAABBsAndBP(PxU32 numCpuTasks, Cm::FlushPool& flushPool, PxcScratchAllocator* scratchAllocator, bool hasContactDistanceUpdated, PxBaseTask* continuation, PxBaseTask* narrowPhaseUnlockTask)
{
	PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP", getContextId());

	mPersistentStateChanged = mPersistentStateChanged || hasContactDistanceUpdated;

	mScratchAllocator = scratchAllocator;
	mNarrowPhaseUnblockTask = narrowPhaseUnlockTask;

	const bool singleThreaded = gSingleThreaded || numCpuTasks<2;
	if(!singleThreaded)
	{
		PX_ASSERT(numCpuTasks);
		mFinalizeUpdateTask.Init(this, numCpuTasks, scratchAllocator, narrowPhaseUnlockTask);
		mFinalizeUpdateTask.setContinuation(continuation);
	}

	// Add
	{
		PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - add", getContextId());

		resetOrClear(mAddedHandles);

		const PxU32* bits = mAddedHandleMap.getWords();
		if(bits)
		{
			const PxU32 lastSetBit = mAddedHandleMap.findLast();
			for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for(PxU32 b = bits[w]; b; b &= b-1)
				{
					const BoundsIndex handle = PxU32(w<<5|Ps::lowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated());
					mAddedHandles.pushBack(handle);		// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	// Update
	{
		PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - update", getContextId());

		resetOrClear(mUpdatedHandles);
		if(!mOriginShifted)
		{
			// PT: TODO:
			// - intercept calls marking aggregateD shapes dirty, in order to mark their aggregates dirty at the same time. That way we don't discover
			// them while parsing the map, i.e. the map is already fully complete when the parsing begins (no need to parse twice etc).
			// - once this is done, aggregateD shapes can be ignored during parsing (since we only needed them to go to their aggregates)
			// - we can then handle aggregates while parsing the map, i.e. there's no need for sorting anymore.
			// - there will be some thoughts to do about the dirty aggregates coming from the added map parsing: we still need to compute their bounds,
			// but we don't want to add these to mUpdatedHandles (since they're already in  mAddedHandles)
			// - we still need the set of dirty aggregates post broadphase, but we don't want to re-parse the full map for self-collisions. So we may still
			// need to put dirty aggregates in an array, but that might be simplified now
			// - the 'isDirty' checks to updatePairs can use the update map though - but the boundedTest is probably more expensive than the current test

			// PT: TODO: another idea: just output all aggregate handles by default then have a pass on mUpdatedHandles to remove them if that wasn't actually necessary
			// ...or just drop the artificial requirement for aggregates...

			{
				PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - update - bitmap iteration", getContextId());

				const PxU32* bits = mChangedHandleMap.getWords();
				if(bits)
				{
					const PxU32 lastSetBit = mChangedHandleMap.findLast();
					for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
					{
						for(PxU32 b = bits[w]; b; b &= b-1)
						{
							const BoundsIndex handle = PxU32(w<<5|Ps::lowestSetBit(b));
							PX_ASSERT(!mRemovedHandleMap.test(handle));		// a handle may only be updated and deleted if it was just added.
							PX_ASSERT(!mVolumeData[handle].isAggregate());	// PT: make sure changedShapes doesn't contain aggregates

							if(mAddedHandleMap.test(handle))					// just-inserted handles may also be marked updated, so skip them
								continue;

							if(mVolumeData[handle].isSingleActor())
							{
								PX_ASSERT(mGroups[handle] != Bp::FilterGroup::eINVALID);
								mUpdatedHandles.pushBack(handle);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
							}
							else
							{
								PX_ASSERT(mVolumeData[handle].isAggregated());
								const AggregateHandle aggregateHandle = mVolumeData[handle].getAggregateOwner();
								Aggregate* aggregate = getAggregateFromHandle(aggregateHandle);
								// PT: an actor from the aggregate has been updated => mark dirty to recompute bounds later
								// PT: we don't recompute the bounds right away since multiple actors from the same aggregate may have changed.
								aggregate->markAsDirty(mDirtyAggregates);
							}
						}
					}
				}
			}

			const PxU32 size = mDirtyAggregates.size();
			if(size)
			{
				PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - update - dirty iteration", getContextId());
				for(PxU32 i=0;i<size;i++)
				{
					Aggregate* aggregate = mDirtyAggregates[i];
					if(i!=size-1)
					{
						Aggregate* nextAggregate = mDirtyAggregates[i];
						Ps::prefetchLine(nextAggregate, 0);
					}

					aggregate->allocateBounds();
					if(singleThreaded)
					{
						aggregate->computeBounds(mBoundsArray.begin(), mContactDistance.begin());
						mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
					}

					// PT: Can happen when an aggregate has been created and then its actors have been changed (with e.g. setLocalPose)
					// before a BP call.
					if(!mAddedHandleMap.test(aggregate->mIndex))
						mUpdatedHandles.pushBack(aggregate->mIndex);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}

				if(!singleThreaded)
					startAggregateBoundsComputationTasks(size, numCpuTasks, flushPool);

				// PT: we're already sorted if no dirty-aggregates are involved
				{
					PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - update - sort", getContextId());

					mPersistentStateChanged = true;	// PT: was previously set multiple times within 'computeBounds'
					// PT: TODO: remove this
					Ps::sort(mUpdatedHandles.begin(), mUpdatedHandles.size());
				}
			}
		}
		else
		{
			handleOriginShift();
		}
	}

	// Remove
	{
		PX_PROFILE_ZONE("AABBManager::updateAABBsAndBP - remove", getContextId());

		resetOrClear(mRemovedHandles);

		const PxU32* bits = mRemovedHandleMap.getWords();
		if(bits)
		{
			const PxU32 lastSetBit = mRemovedHandleMap.findLast();
			for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				for(PxU32 b = bits[w]; b; b &= b-1)
				{
					const BoundsIndex handle = PxU32(w<<5|Ps::lowestSetBit(b));
					PX_ASSERT(!mVolumeData[handle].isAggregated());
					mRemovedHandles.pushBack(handle);	// PT: TODO: BoundsIndex-to-ShapeHandle confusion here
				}
			}
		}
	}

	/////

	// PT: TODO: do we need to run these threads when we origin-shifted everything before?
	if(singleThreaded)
		finalizeUpdate(numCpuTasks, scratchAllocator, continuation, narrowPhaseUnlockTask);
	else
		mFinalizeUpdateTask.removeReference();
}

void AABBManager::finalizeUpdate(PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, PxBaseTask* continuation, PxBaseTask* narrowPhaseUnlockTask)
{
	PX_PROFILE_ZONE("AABBManager::finalizeUpdate", getContextId());

	const bool singleThreaded = gSingleThreaded || numCpuTasks<2;
	if(!singleThreaded)
	{
		const PxU32 size = mDirtyAggregates.size();
		for(PxU32 i=0;i<size;i++)
		{
			Aggregate* aggregate = mDirtyAggregates[i];
			mBoundsArray.begin()[aggregate->mIndex] = aggregate->getMergedBounds();
		}
	}

	const BroadPhaseUpdateData updateData(	mAddedHandles.begin(), mAddedHandles.size(),
											mUpdatedHandles.begin(), mUpdatedHandles.size(),
											mRemovedHandles.begin(), mRemovedHandles.size(),
											mBoundsArray.begin(), mGroups.begin(),
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
											&mLUT[0][0],
#endif
											mContactDistance.begin(), mBoundsArray.getCapacity(),
											mPersistentStateChanged || mBoundsArray.hasChanged());
	mPersistentStateChanged = false;

	PX_ASSERT(updateData.isValid());
	
	//KS - skip broad phase if there are no updated shapes.
	if (updateData.getNumCreatedHandles() != 0 || updateData.getNumRemovedHandles() != 0 || updateData.getNumUpdatedHandles() != 0)
		mBroadPhase.update(numCpuTasks, scratchAllocator, updateData, continuation, narrowPhaseUnlockTask);
	else
		narrowPhaseUnlockTask->removeReference();
}

static PX_FORCE_INLINE void createOverlap(Ps::Array<AABBOverlap>* overlaps, const Ps::Array<VolumeData>& volumeData, PxU32 id0, PxU32 id1)
{
//	overlaps.pushBack(AABBOverlap(volumeData[id0].userData, volumeData[id1].userData, handle));
	const ElementType::Enum volumeType = PxMax(volumeData[id0].getVolumeType(), volumeData[id1].getVolumeType());
	overlaps[volumeType].pushBack(AABBOverlap(reinterpret_cast<void*>(size_t(id0)), reinterpret_cast<void*>(size_t(id1))));
}

static PX_FORCE_INLINE void deleteOverlap(Ps::Array<AABBOverlap>* overlaps, const Ps::Array<VolumeData>& volumeData, PxU32 id0, PxU32 id1)
{
//	PX_ASSERT(volumeData[id0].userData);
//	PX_ASSERT(volumeData[id1].userData);
	if (volumeData[id0].getUserData() && volumeData[id1].getUserData())	// PT: TODO: no idea if this is the right thing to do or if it's normal to get null ptrs here
	{
		const ElementType::Enum volumeType = PxMax(volumeData[id0].getVolumeType(), volumeData[id1].getVolumeType());
//		overlaps.pushBack(AABBOverlap(volumeData[id0].userData, volumeData[id1].userData, handle));
		overlaps[volumeType].pushBack(AABBOverlap(reinterpret_cast<void*>(size_t(id0)), reinterpret_cast<void*>(size_t(id1))));
	}
}

void PersistentPairs::outputDeletedOverlaps(Ps::Array<AABBOverlap>* overlaps, const Ps::Array<VolumeData>& volumeData)
{
	const PxU32 nbActivePairs = mPM.mNbActivePairs;
	for(PxU32 i=0;i<nbActivePairs;i++)
	{
		const InternalPair& p = mPM.mActivePairs[i];
		deleteOverlap(overlaps, volumeData, p.getId0(), p.getId1());
	}
}

PX_FORCE_INLINE void PersistentPairs::updatePairs(	PxU32 timestamp, const PxBounds3* bounds, const float* contactDistances, const Bp::FilterGroup::Enum* groups,
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	const bool* lut,
#endif
									Ps::Array<VolumeData>& volumeData, Ps::Array<AABBOverlap>* createdOverlaps, Ps::Array<AABBOverlap>* destroyedOverlaps)
{
	if(mTimestamp==timestamp)
		return;

	mTimestamp = timestamp;

#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
	findOverlaps(mPM, bounds, contactDistances, groups, lut);
#else
	findOverlaps(mPM, bounds, contactDistances, groups);
#endif

	PxU32 i=0;
	PxU32 nbActivePairs = mPM.mNbActivePairs;
	while(i<nbActivePairs)
	{
		InternalPair& p = mPM.mActivePairs[i];
		const PxU32 id0 = p.getId0();
		const PxU32 id1 = p.getId1();

		if(p.isNew())
		{
			createOverlap(createdOverlaps, volumeData, id0, id1);

			p.clearNew();
			p.clearUpdated();

			i++;
		}
		else if(p.isUpdated())
		{
			p.clearUpdated();
			i++;
		}
		else
		{
			deleteOverlap(destroyedOverlaps, volumeData, id0, id1);

			const PxU32 hashValue = hash(id0, id1) & mPM.mMask;
			mPM.removePair(id0, id1, hashValue, i);
			nbActivePairs--;
		}
	}
	mPM.shrinkMemory();
}

PersistentActorAggregatePair* AABBManager::createPersistentActorAggregatePair(ShapeHandle volA, ShapeHandle volB)
{
	ShapeHandle	actorHandle;
	ShapeHandle	aggregateHandle;
	if(mVolumeData[volA].isAggregate())
	{
		aggregateHandle = volA;
		actorHandle = volB;
	}
	else
	{
		PX_ASSERT(mVolumeData[volB].isAggregate());
		aggregateHandle = volB;
		actorHandle = volA;
	}
	const AggregateHandle h = mVolumeData[aggregateHandle].getAggregate();
	Aggregate* aggregate = getAggregateFromHandle(h);
	PX_ASSERT(aggregate->mIndex==aggregateHandle);
	return PX_NEW(PersistentActorAggregatePair)(aggregate, actorHandle);	// PT: TODO: use a pool or something
}

PersistentAggregateAggregatePair* AABBManager::createPersistentAggregateAggregatePair(ShapeHandle volA, ShapeHandle volB)
{
	PX_ASSERT(mVolumeData[volA].isAggregate());
	PX_ASSERT(mVolumeData[volB].isAggregate());
	const AggregateHandle h0 = mVolumeData[volA].getAggregate();
	const AggregateHandle h1 = mVolumeData[volB].getAggregate();
	Aggregate* aggregate0 = getAggregateFromHandle(h0);
	Aggregate* aggregate1 = getAggregateFromHandle(h1);
	PX_ASSERT(aggregate0->mIndex==volA);
	PX_ASSERT(aggregate1->mIndex==volB);
	return PX_NEW(PersistentAggregateAggregatePair)(aggregate0, aggregate1);	// PT: TODO: use a pool or something
}

void AABBManager::updatePairs(PersistentPairs& p, BpCacheData* data)
{
	if (data)
	{
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), &mLUT[0][0], mVolumeData, data->mCreatedPairs, data->mDeletedPairs);
#else
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), mVolumeData, data->mCreatedPairs, data->mDeletedPairs);
#endif
	}
	else
	{
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), &mLUT[0][0], mVolumeData, mCreatedOverlaps, mDestroyedOverlaps);
#else
		p.updatePairs(mTimestamp, mBoundsArray.begin(), mContactDistance.begin(), mGroups.begin(), mVolumeData, mCreatedOverlaps, mDestroyedOverlaps);
#endif
	}
}

void AABBManager::processBPCreatedPair(const BroadPhasePair& pair)
{
	PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
	PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());
	const bool isSingleActorA = mVolumeData[pair.mVolA].isSingleActor();
	const bool isSingleActorB = mVolumeData[pair.mVolB].isSingleActor();

	if(isSingleActorA && isSingleActorB)
	{
		createOverlap(mCreatedOverlaps, mVolumeData, pair.mVolA, pair.mVolB);	// PT: regular actor-actor pair
		return;
	}

	// PT: TODO: check if this is needed
	ShapeHandle volA = pair.mVolA;
	ShapeHandle volB = pair.mVolB;
	if(volB<volA)
		Ps::swap(volA, volB);

	PersistentPairs* newPair;
	AggPairMap* pairMap;
	if(!isSingleActorA && !isSingleActorB)
	{
		pairMap = &mAggregateAggregatePairs;	// PT: aggregate-aggregate pair
		newPair = createPersistentAggregateAggregatePair(volA, volB);
	}
	else
	{
		pairMap = &mActorAggregatePairs;	// PT: actor-aggregate pair
		newPair = createPersistentActorAggregatePair(volA, volB);
	}
	bool status = pairMap->insert(AggPair(volA, volB), newPair);
	PX_UNUSED(status);
	PX_ASSERT(status);
	updatePairs(*newPair);
}

void AABBManager::processBPDeletedPair(const BroadPhasePair& pair)
{
	PX_ASSERT(!mVolumeData[pair.mVolA].isAggregated());
	PX_ASSERT(!mVolumeData[pair.mVolB].isAggregated());
	const bool isSingleActorA = mVolumeData[pair.mVolA].isSingleActor();
	const bool isSingleActorB = mVolumeData[pair.mVolB].isSingleActor();

	if(isSingleActorA && isSingleActorB)
	{
		deleteOverlap(mDestroyedOverlaps, mVolumeData, pair.mVolA, pair.mVolB);	// PT: regular actor-actor pair
		return;
	}

	// PT: TODO: check if this is needed
	ShapeHandle volA = pair.mVolA;
	ShapeHandle volB = pair.mVolB;
	if(volB<volA)
		Ps::swap(volA, volB);

	AggPairMap* pairMap;
	if(!isSingleActorA && !isSingleActorB)
		pairMap = &mAggregateAggregatePairs;	// PT: aggregate-aggregate pair
	else
		pairMap = &mActorAggregatePairs;		// PT: actor-aggregate pair

	PersistentPairs* p;
	{
		const AggPairMap::Entry* e = pairMap->find(AggPair(volA, volB));
		PX_ASSERT(e);
		p = e->second;
	}

	p->outputDeletedOverlaps(mDestroyedOverlaps, mVolumeData);
	p->mShouldBeDeleted = true;
}

struct CreatedPairHandler
{
	static PX_FORCE_INLINE void processPair(AABBManager& manager, const BroadPhasePair& pair) { manager.processBPCreatedPair(pair); }
};

struct DeletedPairHandler
{
	static PX_FORCE_INLINE void processPair(AABBManager& manager, const BroadPhasePair& pair) { manager.processBPDeletedPair(pair); }
};

template<class FunctionT>
static void processBPPairs(PxU32 nbPairs, const BroadPhasePair* pairs, AABBManager& manager)
{
	// PT: TODO: figure out this ShapeHandle/BpHandle thing. Is it ok to use "BP_INVALID_BP_HANDLE" for a "ShapeHandle"?
	ShapeHandle previousA = BP_INVALID_BP_HANDLE;
	ShapeHandle previousB = BP_INVALID_BP_HANDLE;

	while(nbPairs--)
	{
		PX_ASSERT(pairs->mVolA!=BP_INVALID_BP_HANDLE);
		PX_ASSERT(pairs->mVolB!=BP_INVALID_BP_HANDLE);
		// PT: TODO: why is that test needed now? GPU broadphase?
		if(pairs->mVolA != previousA || pairs->mVolB != previousB)
		{
			previousA = pairs->mVolA;
			previousB = pairs->mVolB;
			FunctionT::processPair(manager, *pairs);
		}
		pairs++;
	}
}

static void processAggregatePairs(AggPairMap& map, AABBManager& manager)
{
	// PT: TODO: hmmm we have a list of dirty aggregates but we don't have a list of dirty pairs.
	// PT: not sure how the 3.4 trunk solves this but let's just iterate all pairs for now
	// PT: atm we reuse this loop to delete removed interactions
	// PT: TODO: in fact we could handle all the "lost pairs" stuff right there with extra aabb-abb tests

	// PT: TODO: replace with decent hash map - or remove the hashmap entirely and use a linear array
	Ps::Array<AggPair> removedEntries;
	for(AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
	{
		PersistentPairs* p = iter->second;
		if(p->update(manager))
		{
			removedEntries.pushBack(iter->first);
			PX_DELETE(p);
		}
	}
	for(PxU32 i=0;i<removedEntries.size();i++)
	{
		bool status = map.erase(removedEntries[i]);
		PX_ASSERT(status);
		PX_UNUSED(status);
	}
}

struct PairData
{
	Ps::Array<AABBOverlap>* mArray;
	PxU32 mStartIdx;
	PxU32 mCount;

	PairData() : mArray(NULL), mStartIdx(0), mCount(0)
	{
	}
};

class ProcessAggPairsBase : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 16;

	PairData mCreatedPairs[2];
	PairData mDestroyedPairs[2];

	ProcessAggPairsBase(PxU64 contextID) : Cm::Task(contextID)
	{
	}

	void setCache(BpCacheData& data)
	{
		for (PxU32 i = 0; i < 2; ++i)
		{
			mCreatedPairs[i].mArray = &data.mCreatedPairs[i];
			mCreatedPairs[i].mStartIdx = data.mCreatedPairs[i].size();
			mDestroyedPairs[i].mArray = &data.mDeletedPairs[i];
			mDestroyedPairs[i].mStartIdx = data.mDeletedPairs[i].size();
		}
	}

	void updateCounters()
	{
		for (PxU32 i = 0; i < 2; ++i)
		{
			mCreatedPairs[i].mCount = mCreatedPairs[i].mArray->size() - mCreatedPairs[i].mStartIdx;
			mDestroyedPairs[i].mCount = mDestroyedPairs[i].mArray->size() - mDestroyedPairs[i].mStartIdx;
		}
	}
};


class ProcessAggPairsParallelTask : public ProcessAggPairsBase
{
public:
	PersistentPairs* mPersistentPairs[MaxPairs];
	Bp::AggPair mAggPairs[MaxPairs];
	PxU32 mNbPairs;
	AABBManager* mManager;
	AggPairMap* mMap;
	Ps::Mutex* mMutex;
	const char* mName;

	ProcessAggPairsParallelTask(PxU64 contextID, Ps::Mutex* mutex, AABBManager* manager, AggPairMap* map, const char* name) : ProcessAggPairsBase(contextID),
		mNbPairs(0), mManager(manager), mMap(map), mMutex(mutex), mName(name)
	{
	}


	void runInternal()
	{
		BpCacheData* data = mManager->getBpCacheData();

		setCache(*data);


		Ps::InlineArray<AggPair, MaxPairs> removedEntries;
		for (PxU32 i = 0; i < mNbPairs; ++i)
		{
			if (mPersistentPairs[i]->update(*mManager, data))
			{
				removedEntries.pushBack(mAggPairs[i]);
				PX_DELETE(mPersistentPairs[i]);
			}
		}

		updateCounters();

		mManager->putBpCacheData(data);


		if (removedEntries.size())
		{
			Ps::Mutex::ScopedLock lock(*mMutex);
			for (PxU32 i = 0; i < removedEntries.size(); i++)
			{
				bool status = mMap->erase(removedEntries[i]);
				PX_ASSERT(status);
				PX_UNUSED(status);
			}
		}


	}

	virtual const char* getName() const { return mName; }

};

class SortAggregateBoundsParallel : public Cm::Task
{
public:
	static const PxU32 MaxPairs = 16;
	Aggregate** mAggregates;
	PxU32 mNbAggs;

	SortAggregateBoundsParallel(PxU64 contextID, Aggregate** aggs, PxU32 nbAggs) : Cm::Task(contextID),
		mAggregates(aggs), mNbAggs(nbAggs)
	{
	}


	void runInternal()
	{
		PX_PROFILE_ZONE("SortBounds", mContextID);
		for (PxU32 i = 0; i < mNbAggs; i++)
		{
			Aggregate* aggregate = mAggregates[i];

			aggregate->getSortedMinBounds();
		}
	}

	virtual const char* getName() const { return "SortAggregateBoundsParallel"; }

};


class ProcessSelfCollisionPairsParallel : public ProcessAggPairsBase
{
public:
	Aggregate** mAggregates;
	PxU32 mNbAggs;
	AABBManager* mManager;

	ProcessSelfCollisionPairsParallel(PxU64 contextID, Aggregate** aggs, PxU32 nbAggs, AABBManager* manager) : ProcessAggPairsBase(contextID),
		mAggregates(aggs), mNbAggs(nbAggs), mManager(manager)
	{
	}


	void runInternal()
	{
		BpCacheData* data = mManager->getBpCacheData();
		setCache(*data);
		PX_PROFILE_ZONE("ProcessSelfCollisionPairs", mContextID);
		for (PxU32 i = 0; i < mNbAggs; i++)
		{
			Aggregate* aggregate = mAggregates[i];

			if (aggregate->mSelfCollisionPairs)
				mManager->updatePairs(*aggregate->mSelfCollisionPairs, data);
		}
		updateCounters();
		mManager->putBpCacheData(data);
	}

	virtual const char* getName() const { return "ProcessSelfCollisionPairsParallel"; }

};

static void processAggregatePairsParallel(AggPairMap& map, AABBManager& manager, Cm::FlushPool& flushPool,
	PxBaseTask* continuation, const char* taskName, Ps::Array<ProcessAggPairsBase*>& pairTasks)
{
	// PT: TODO: hmmm we have a list of dirty aggregates but we don't have a list of dirty pairs.
	// PT: not sure how the 3.4 trunk solves this but let's just iterate all pairs for now
	// PT: atm we reuse this loop to delete removed interactions
	// PT: TODO: in fact we could handle all the "lost pairs" stuff right there with extra aabb-abb tests

	// PT: TODO: replace with decent hash map - or remove the hashmap entirely and use a linear array

	manager.mMapLock.lock();

	ProcessAggPairsParallelTask* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessAggPairsParallelTask)), ProcessAggPairsParallelTask)(0, &manager.mMapLock, &manager, &map, taskName);

	PxU32 startIdx = pairTasks.size();

	for (AggPairMap::Iterator iter = map.getIterator(); !iter.done(); ++iter)
	{
		task->mAggPairs[task->mNbPairs] = iter->first;
		task->mPersistentPairs[task->mNbPairs++] = iter->second;
		if (task->mNbPairs == ProcessAggPairsParallelTask::MaxPairs)
		{
			pairTasks.pushBack(task);
			task->setContinuation(continuation);
			task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessAggPairsParallelTask)), ProcessAggPairsParallelTask)(0, &manager.mMapLock, &manager, &map, taskName);
		}
	}

	manager.mMapLock.unlock();

	for (PxU32 i = startIdx; i < pairTasks.size(); ++i)
	{
		pairTasks[i]->removeReference();
	}

	if (task->mNbPairs)
	{
		pairTasks.pushBack(task);
		task->setContinuation(continuation);
		task->removeReference();
		//task->runInternal();
	}
}

void AABBManager::postBroadPhase(PxBaseTask* continuation, PxBaseTask* narrowPhaseUnlockTask, Cm::FlushPool& flushPool)
{
	PX_PROFILE_ZONE("AABBManager::postBroadPhase", getContextId());

	//KS - There is a continuation task for discrete broad phase, but not for CCD broad phase. PostBroadPhase for CCD broad phase runs in-line.
	//This probably should be revisited but it means that we can't use the parallel codepath within CCD.
	if (continuation)
	{
		mPostBroadPhase3.setContinuation(continuation);
		mPostBroadPhase2.setContinuation(&mPostBroadPhase3);
	}

	mTimestamp++;

	// PT: TODO: consider merging mCreatedOverlaps & mDestroyedOverlaps
	// PT: TODO: revisit memory management of mCreatedOverlaps & mDestroyedOverlaps

	//KS - if we ran broad phase, fetch the results now
	if(mAddedHandles.size() != 0 || mUpdatedHandles.size() != 0 || mRemovedHandles.size() != 0)
		mBroadPhase.fetchBroadPhaseResults(narrowPhaseUnlockTask);

	for(PxU32 i=0; i<ElementType::eCOUNT; i++)
	{
		resetOrClear(mCreatedOverlaps[i]);
		resetOrClear(mDestroyedOverlaps[i]);
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - process deleted pairs", getContextId());
//		processBPPairs<CreatedPairHandler>(mBroadPhase.getNbCreatedPairs(), mBroadPhase.getCreatedPairs(), *this);
		processBPPairs<DeletedPairHandler>(mBroadPhase.getNbDeletedPairs(), mBroadPhase.getDeletedPairs(), *this);
	}

	{
		//If there is a continuation task, then this is not part of CCD, so we can trigger bounds to be recomputed in parallel before pair generation runs during
		//stage 2.			
		if (continuation)
		{
			const PxU32 size = mDirtyAggregates.size();
			for (PxU32 i = 0; i < size; i += SortAggregateBoundsParallel::MaxPairs)
			{
				const PxU32 nbToProcess = PxMin(size - i, SortAggregateBoundsParallel::MaxPairs);

				SortAggregateBoundsParallel* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(SortAggregateBoundsParallel)), SortAggregateBoundsParallel)
					(mContextID, &mDirtyAggregates[i], nbToProcess);

				task->setContinuation(&mPostBroadPhase2);
				task->removeReference();
			}
		}
	}



	if (continuation)
	{
		mPostBroadPhase2.setFlushPool(&flushPool);
		mPostBroadPhase3.removeReference();
		mPostBroadPhase2.removeReference();
	}
	else
	{
		postBpStage2(NULL, flushPool);
		postBpStage3(NULL);
	}

}

void PostBroadPhaseStage2Task::runInternal()
{
	mManager.postBpStage2(mCont, *mFlushPool);
}

void AABBManager::postBpStage2(PxBaseTask* continuation, Cm::FlushPool& flushPool)
{
	{
		const PxU32 size = mDirtyAggregates.size();
		for (PxU32 i = 0; i < size; i += ProcessSelfCollisionPairsParallel::MaxPairs)
		{
			const PxU32 nbToProcess = PxMin(size - i, ProcessSelfCollisionPairsParallel::MaxPairs);

			ProcessSelfCollisionPairsParallel* task = PX_PLACEMENT_NEW(flushPool.allocate(sizeof(ProcessSelfCollisionPairsParallel)), ProcessSelfCollisionPairsParallel)
				(mContextID, &mDirtyAggregates[i], nbToProcess, this);
			if (continuation)
			{
				task->setContinuation(continuation);
				task->removeReference();
			}
			else
				task->runInternal();
			mAggPairTasks.pushBack(task);
		}
	}

	{
		if (continuation)
			processAggregatePairsParallel(mAggregateAggregatePairs, *this, flushPool, continuation, "AggAggPairs", mAggPairTasks);
		else
			processAggregatePairs(mAggregateAggregatePairs, *this);
	}



	{
		if (continuation)
			processAggregatePairsParallel(mActorAggregatePairs, *this, flushPool, continuation, "AggActorPairs", mAggPairTasks);
		else
			processAggregatePairs(mActorAggregatePairs, *this);
	}


}

void AABBManager::postBpStage3(PxBaseTask*)
{

	{
		{
			PX_PROFILE_ZONE("SimpleAABBManager::postBroadPhase - aggregate self-collisions", getContextId());
			const PxU32 size = mDirtyAggregates.size();
			for (PxU32 i = 0; i < size; i++)
			{
				Aggregate* aggregate = mDirtyAggregates[i];
				aggregate->resetDirtyState();

			}
			resetOrClear(mDirtyAggregates);
		}

		{
			PX_PROFILE_ZONE("SimpleAABBManager::postBroadPhase - append pairs", getContextId());

			for (PxU32 a = 0; a < mAggPairTasks.size(); ++a)
			{
				ProcessAggPairsBase* task = mAggPairTasks[a];
				for (PxU32 t = 0; t < 2; t++)
				{
					for (PxU32 i = 0, startIdx = task->mCreatedPairs[t].mStartIdx; i < task->mCreatedPairs[t].mCount; ++i)
					{
						mCreatedOverlaps[t].pushBack((*task->mCreatedPairs[t].mArray)[i + startIdx]);
					}
					for (PxU32 i = 0, startIdx = task->mDestroyedPairs[t].mStartIdx; i < task->mDestroyedPairs[t].mCount; ++i)
					{
						mDestroyedOverlaps[t].pushBack((*task->mDestroyedPairs[t].mArray)[i + startIdx]);
					}
				}
			}

			mAggPairTasks.forceSize_Unsafe(0);


			Ps::InlineArray<BpCacheData*, 16> bpCache;
			BpCacheData* entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());

			while (entry)
			{
				entry->reset();
				bpCache.pushBack(entry);
				entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
			}

			//Now reinsert back into queue...
			for (PxU32 i = 0; i < bpCache.size(); ++i)
			{
				mBpThreadContextPool.push(*bpCache[i]);
			}
		}
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - process created pairs", getContextId());
		processBPPairs<CreatedPairHandler>(mBroadPhase.getNbCreatedPairs(), mBroadPhase.getCreatedPairs(), *this);
//		processBPPairs<DeletedPairHandler>(mBroadPhase.getNbDeletedPairs(), mBroadPhase.getDeletedPairs(), *this);
	}

	// PT: TODO: revisit this
	// Filter out pairs in mDestroyedOverlaps that already exist in mCreatedOverlaps. This should be done better using bitmaps
	// and some imposed ordering on previous operations. Later.
	// We could also have a dedicated function "reinsertBroadPhase()", which would preserve the existing interactions at Sc-level.
	if(1)
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - post-process", getContextId());

		PxU32 totalCreatedOverlaps = 0;
		for (PxU32 idx=0; idx<ElementType::eCOUNT; idx++)
			totalCreatedOverlaps += mCreatedOverlaps[idx].size();

		mCreatedPairs.clear();
		mCreatedPairs.reserve(totalCreatedOverlaps);
		
		for(PxU32 idx=0; idx<ElementType::eCOUNT; idx++)
		{
			const PxU32 nbDestroyedOverlaps = mDestroyedOverlaps[idx].size();
			{
				const PxU32 size = mCreatedOverlaps[idx].size();
				for (PxU32 i = 0; i < size; i++)
				{
					const PxU32 id0 = PxU32(size_t(mCreatedOverlaps[idx][i].mUserData0));
					const PxU32 id1 = PxU32(size_t(mCreatedOverlaps[idx][i].mUserData1));
					mCreatedOverlaps[idx][i].mUserData0 = mVolumeData[id0].getUserData();
					mCreatedOverlaps[idx][i].mUserData1 = mVolumeData[id1].getUserData();
					if (nbDestroyedOverlaps)
						mCreatedPairs.insert(Pair(id0, id1));
				}
			}
			PxU32 newSize = 0;
			for (PxU32 i = 0; i < nbDestroyedOverlaps; i++)
			{
				const PxU32 id0 = PxU32(size_t(mDestroyedOverlaps[idx][i].mUserData0));
				const PxU32 id1 = PxU32(size_t(mDestroyedOverlaps[idx][i].mUserData1));
				if (!mCreatedPairs.contains(Pair(id0, id1)))
				{
					mDestroyedOverlaps[idx][newSize].mUserData0 = mVolumeData[id0].getUserData();
					mDestroyedOverlaps[idx][newSize].mUserData1 = mVolumeData[id1].getUserData();
					newSize++;
				}
			}
			mDestroyedOverlaps[idx].forceSize_Unsafe(newSize);
		}
	}

	// Handle out-of-bounds objects
	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - out-of-bounds", getContextId());
		PxU32 nbObjects = mBroadPhase.getNbOutOfBoundsObjects();
		const PxU32* objects = mBroadPhase.getOutOfBoundsObjects();
		while(nbObjects--)
		{
			const PxU32 index = *objects++;
			if(!mRemovedHandleMap.test(index))
			{
				if(mVolumeData[index].isSingleActor())
					mOutOfBoundsObjects.pushBack(mVolumeData[index].getUserData());
				else
				{
					PX_ASSERT(mVolumeData[index].isAggregate());
					mOutOfBoundsAggregates.pushBack(mVolumeData[index].getUserData());
				}
			}
		}
	}

	{
		PX_PROFILE_ZONE("AABBManager::postBroadPhase - clear", getContextId());
		mAddedHandleMap.clear();
		mRemovedHandleMap.clear();
	}
}

void AABBManager::freeBuffers()
{
	// PT: TODO: investigate if we need more stuff here
	mBroadPhase.freeBuffers();
}

void AABBManager::shiftOrigin(const PxVec3& shift)
{
	mBroadPhase.shiftOrigin(shift);
	mOriginShifted = true;
}

BpCacheData* AABBManager::getBpCacheData()
{
	BpCacheData* rv = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	if (rv == NULL)
	{
		rv = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(BpCacheData), PX_DEBUG_EXP("BpCacheData")), BpCacheData)();
	}
	return rv;
}
void AABBManager::putBpCacheData(BpCacheData* data)
{
	mBpThreadContextPool.push(*data);
}
void AABBManager::resetBpCacheData()
{
	Ps::InlineArray<BpCacheData*, 16> bpCache;
	BpCacheData* entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	while (entry)
	{
		entry->reset();
		bpCache.pushBack(entry);
		entry = static_cast<BpCacheData*>(mBpThreadContextPool.pop());
	}

	//Now reinsert back into queue...
	for (PxU32 i = 0; i < bpCache.size(); ++i)
	{
		mBpThreadContextPool.push(*bpCache[i]);
	}
}

// PT: disabled this by default, since it bypasses all per-shape/per-actor visualization flags
//static const bool gVisualizeAggregateElems = false;

void AABBManager::visualize(Cm::RenderOutput& out)
{
	const PxTransform idt = PxTransform(PxIdentity);
	out << idt;

	const PxU32 N = mAggregates.size();
	for(PxU32 i=0;i<N;i++)
	{
		Aggregate* aggregate = mAggregates[i];
		if(aggregate->getNbAggregated())
		{
			out << PxU32(PxDebugColor::eARGB_GREEN);
			const PxBounds3& b = mBoundsArray.getBounds(aggregate->mIndex);
			out << Cm::DebugBox(b, true);
		}
	}

/*	const PxU32 N = mAggregateManager.getAggregatesCapacity();
	for(PxU32 i=0;i<N;i++)
	{
		const Aggregate* aggregate = mAggregateManager.getAggregate(i);
		if(aggregate->nbElems)
		{
			if(!mAggregatesUpdated.isInList(BpHandle(i)))
				out << PxU32(PxDebugColor::eARGB_GREEN);
			else
				out << PxU32(PxDebugColor::eARGB_RED);

			PxBounds3 decoded;
			const IntegerAABB& iaabb = mBPElems.getAABB(aggregate->bpElemId);
			iaabb.decode(decoded);

			out << Cm::DebugBox(decoded, true);

			if(gVisualizeAggregateElems)
			{
				PxU32 elem = aggregate->elemHeadID;
				while(BP_INVALID_BP_HANDLE!=elem)
				{
					out << PxU32(PxDebugColor::eARGB_CYAN);
					const IntegerAABB elemBounds = mAggregateElems.getAABB(elem);
					elemBounds.decode(decoded);
					out << Cm::DebugBox(decoded, true);
					elem = mAggregateElems.getNextId(elem);
				}
			}
		}
	}*/
}

} //namespace Bp

} //namespace physx

