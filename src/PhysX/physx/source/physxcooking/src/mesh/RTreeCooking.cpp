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

#include "foundation/PxBounds3.h"
#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "RTreeCooking.h"
#include "PsSort.h"
#include "PsMathUtils.h"
#include "PsAllocator.h"
#include "PsVecMath.h"
#include "PxTolerancesScale.h"
#include "QuickSelect.h"
#include "PsInlineArray.h"
#include "GuRTree.h"

#define PRINT_RTREE_COOKING_STATS 0 // AP: keeping this frequently used macro for diagnostics/benchmarking

#if PRINT_RTREE_COOKING_STATS
#include <stdio.h>
#endif

using namespace physx::Gu;
using namespace physx::shdfnd;
using namespace physx::shdfnd::aos;

namespace physx
{

// Intermediate non-quantized representation for RTree node in a page (final format is SIMD transposed page)
struct RTreeNodeNQ
{
	PxBounds3	bounds;
	PxI32		childPageFirstNodeIndex; // relative to the beginning of all build tree nodes array
	PxI32		leafCount; // -1 for empty nodes, 0 for non-terminal nodes, number of enclosed tris if non-zero (LeafTriangles), also means a terminal node

	struct U {}; // selector struct for uninitialized constructor
	RTreeNodeNQ(U) {} // uninitialized constructor
	RTreeNodeNQ() : bounds(PxBounds3::empty()), childPageFirstNodeIndex(-1), leafCount(0) {}
};

// SIMD version of bounds class
struct PxBounds3V
{
	struct U {}; // selector struct for uninitialized constructor
	Vec3V mn, mx;
	PxBounds3V(Vec3VArg mn_, Vec3VArg mx_) : mn(mn_), mx(mx_) {}
	PxBounds3V(U) {} // uninitialized constructor

	PX_FORCE_INLINE Vec3V getExtents() const { return V3Sub(mx, mn); }
	PX_FORCE_INLINE void include(const PxBounds3V& other) { mn = V3Min(mn, other.mn); mx = V3Max(mx, other.mx); }

	// convert vector extents to PxVec3
	PX_FORCE_INLINE const PxVec3 getMinVec3() const { PxVec3 ret; V3StoreU(mn, ret); return ret; }
	PX_FORCE_INLINE const PxVec3 getMaxVec3() const { PxVec3 ret; V3StoreU(mx, ret); return ret; }
};

static void buildFromBounds(
	Gu::RTree& resultTree, const PxBounds3V* allBounds, PxU32 numBounds,
	Array<PxU32>& resultPermute, RTreeCooker::RemapCallback* rc, Vec3VArg allMn, Vec3VArg allMx,
	PxReal sizePerfTradeOff, PxMeshCookingHint::Enum hint);

/////////////////////////////////////////////////////////////////////////
void RTreeCooker::buildFromTriangles(
	Gu::RTree& result, const PxVec3* verts, PxU32 numVerts, const PxU16* tris16, const PxU32* tris32, PxU32 numTris,
	Array<PxU32>& resultPermute, RTreeCooker::RemapCallback* rc, PxReal sizePerfTradeOff01, PxMeshCookingHint::Enum hint)
{
	PX_UNUSED(numVerts);
	Array<PxBounds3V> allBounds;
	allBounds.reserve(numTris);
	Vec3V allMn = Vec3V_From_FloatV(FMax()), allMx = Vec3V_From_FloatV(FNegMax());
	Vec3V eps = V3Splat(FLoad(5e-4f)); // AP scaffold: use PxTolerancesScale here?

	// build RTree AABB bounds from triangles, conservative bound inflation is also performed here
	for(PxU32 i = 0; i < numTris; i ++)
	{
		PxU32 i0, i1, i2;
		PxU32 i3 = i*3;
		if(tris16)
		{
			i0 = tris16[i3]; i1 = tris16[i3+1]; i2 = tris16[i3+2];
		} else
		{
			i0 = tris32[i3]; i1 = tris32[i3+1]; i2 = tris32[i3+2];
		}
		PX_ASSERT_WITH_MESSAGE(i0 < numVerts && i1 < numVerts && i2 < numVerts ,"Input mesh triangle's vertex index exceeds specified numVerts.");
		Vec3V v0 = V3LoadU(verts[i0]), v1 = V3LoadU(verts[i1]), v2 = V3LoadU(verts[i2]);
		Vec3V mn = V3Sub(V3Min(V3Min(v0, v1), v2), eps); // min over 3 verts, subtract eps to inflate
		Vec3V mx = V3Add(V3Max(V3Max(v0, v1), v2), eps); // max over 3 verts, add eps to inflate
		allMn = V3Min(allMn, mn); allMx = V3Max(allMx, mx);
		allBounds.pushBack(PxBounds3V(mn, mx));
	}

	buildFromBounds(result, allBounds.begin(), numTris, resultPermute, rc, allMn, allMx, sizePerfTradeOff01, hint);
}

/////////////////////////////////////////////////////////////////////////
// Fast but lower quality 4-way split sorting using repeated application of quickselect

// comparator template struct for sortin gbounds centers given a coordinate index (x,y,z=0,1,2)
struct BoundsLTE
{
	PxU32 coordIndex;
	const PxVec3* PX_RESTRICT boundCenters; // AP: precomputed centers are faster than recomputing the centers
	BoundsLTE(PxU32 coordIndex_, const PxVec3* boundCenters_)
		: coordIndex(coordIndex_), boundCenters(boundCenters_)
	{}

	PX_FORCE_INLINE bool operator()(const PxU32 & idx1, const PxU32 & idx2) const
	{
		PxF32 center1 = boundCenters[idx1][coordIndex];
		PxF32 center2 = boundCenters[idx2][coordIndex];
		return (center1 <= center2);
	}
};

// ======================================================================
// Quick sorting method
// recursive sorting procedure:
// 1. find min and max extent along each axis for the current cluster
// 2. split input cluster into two 3 times using quickselect, splitting off a quarter of the initial cluster size each time
// 3. the axis is potentialy different for each split using the following
//   approximate splitting heuristic - reduce max length by some estimated factor to encourage split along other axis
//   since we cut off between a quarter to a half of elements in this direction per split
//   the reduction for first split should be *0.75f but we use 0.8
//   to account for some node overlap. This is somewhat of an arbitrary choice and there's room for improvement.
// 4. recurse on new clusters (goto step 1)
//
struct SubSortQuick
{
	static const PxReal reductionFactors[RTREE_N-1];

	enum { NTRADEOFF = 9 };
	static const PxU32 stopAtTrisPerLeaf1[NTRADEOFF]; // presets for PxCookingParams::meshSizePerformanceTradeoff implementation

	const PxU32* permuteEnd;
	const PxU32* permuteStart;
	const PxBounds3V* allBounds;
	Array<PxVec3> boundCenters;
	PxU32 maxBoundsPerLeafPage;

	// initialize the context for the sorting routine
	SubSortQuick(PxU32* permute, const PxBounds3V* allBounds_, PxU32 allBoundsSize, PxReal sizePerfTradeOff01)
		: allBounds(allBounds_)
	{
		permuteEnd = permute + allBoundsSize;
		permuteStart = permute;
		PxU32 boundsCount = allBoundsSize;
		boundCenters.reserve(boundsCount); // AP - measured that precomputing centers helps with perf significantly (~20% on 1k verts)
		for(PxU32 i = 0; i < boundsCount; i++)
			boundCenters.pushBack( allBounds[i].getMinVec3() + allBounds[i].getMaxVec3() );
		PxU32 iTradeOff = PxMin<PxU32>( PxU32(PxMax<PxReal>(0.0f, sizePerfTradeOff01)*NTRADEOFF), NTRADEOFF-1 );
		maxBoundsPerLeafPage = stopAtTrisPerLeaf1[iTradeOff];
	}

	// implements the sorting/splitting procedure
	void sort4(
		PxU32* PX_RESTRICT permute, const PxU32 clusterSize, // beginning and size of current recursively processed cluster
		Array<RTreeNodeNQ>& resultTree, PxU32& maxLevels,
		PxBounds3V& subTreeBound, PxU32 level = 0)
	{
		if(level == 0)
			maxLevels = 1;
		else
			maxLevels = PxMax(maxLevels, level+1);

		PX_ASSERT(permute + clusterSize <= permuteEnd);
		PX_ASSERT(maxBoundsPerLeafPage >= RTREE_N-1);

		const PxU32 cluster4 = PxMax<PxU32>(clusterSize/RTREE_N, 1);

		PX_ASSERT(clusterSize > 0);
		// find min and max world bound for current cluster
		Vec3V mx = allBounds[permute[0]].mx, mn = allBounds[permute[0]].mn; PX_ASSERT(permute[0] < boundCenters.size());
		for(PxU32 i = 1; i < clusterSize; i ++)
		{
			PX_ASSERT(permute[i] < boundCenters.size());
			mx = V3Max(mx, allBounds[permute[i]].mx);
			mn = V3Min(mn, allBounds[permute[i]].mn);
		}
		PX_ALIGN_PREFIX(16) PxReal maxElem[4] PX_ALIGN_SUFFIX(16);
		V3StoreA(V3Sub(mx, mn), *reinterpret_cast<PxVec3*>(maxElem)); // compute the dimensions and store into a scalar maxElem array

		// split along the longest axis
		const PxU32 maxDiagElement = PxU32(maxElem[0] > maxElem[1] && maxElem[0] > maxElem[2] ? 0 : (maxElem[1] > maxElem[2] ? 1 : 2));
		BoundsLTE cmpLte(maxDiagElement, boundCenters.begin());

		const PxU32 startNodeIndex = resultTree.size();
		resultTree.resizeUninitialized(startNodeIndex+RTREE_N); // at each recursion level we add 4 nodes to the tree

		PxBounds3V childBound( (PxBounds3V::U()) ); // start off uninitialized for performance
		const PxI32 leftover = PxMax<PxI32>(PxI32(clusterSize - cluster4*(RTREE_N-1)), 0);
		PxU32 totalCount = 0;
		for(PxU32 i = 0; i < RTREE_N; i++)
		{
			// split off cluster4 count nodes out of the entire cluster for each i
			const PxU32 clusterOffset = cluster4*i;
			PxU32 count1; // cluster4 or leftover depending on whether it's the last cluster
			if(i < RTREE_N-1)
			{
				// only need to so quickSelect for the first pagesize-1 clusters
				if(clusterOffset <= clusterSize-1)
				{
					quickSelect::quickSelectFirstK(permute, clusterOffset, clusterSize-1, cluster4, cmpLte);
					// approximate heuristic - reduce max length by some estimated factor to encourage split along other axis
					// since we cut off a quarter of elements in this direction the reduction should be *0.75f but we use 0.8
					// to account for some node overlap. This is somewhat of an arbitrary choice though
					maxElem[cmpLte.coordIndex] *= reductionFactors[i];
					// recompute cmpLte.coordIndex from updated maxElements
					cmpLte.coordIndex = PxU32(maxElem[0] > maxElem[1] && maxElem[0] > maxElem[2] ? 0 : (maxElem[1] > maxElem[2] ? 1 : 2));
				}
				count1 = cluster4;
			} else
			{
				count1 = PxU32(leftover);
				// verify that leftover + sum of previous clusters adds up to clusterSize or leftover is 0
				// leftover can be 0 if clusterSize<RTREE_N, this is generally rare, can happen for meshes with < RTREE_N tris
				PX_ASSERT(leftover == 0 || cluster4*i + count1 == clusterSize);
			}

			RTreeNodeNQ& curNode = resultTree[startNodeIndex+i];

			totalCount += count1; // accumulate total node count
			if(count1 <= maxBoundsPerLeafPage) // terminal page according to specified maxBoundsPerLeafPage
			{
				if(count1 && totalCount <= clusterSize)
				{
					// this will be true most of the time except when the total number of triangles in the mesh is < PAGESIZE
					curNode.leafCount = PxI32(count1);
					curNode.childPageFirstNodeIndex = PxI32(clusterOffset + PxU32(permute-permuteStart));
					childBound = allBounds[permute[clusterOffset+0]];
					for(PxU32 i1 = 1; i1 < count1; i1++)
					{
						const PxBounds3V& bnd = allBounds[permute[clusterOffset+i1]];
						childBound.include(bnd);
					}
				} else
				{
					// since we are required to have PAGESIZE nodes per page for simd, we fill any leftover with empty nodes
					// we should only hit this if the total number of triangles in the mesh is < PAGESIZE
					childBound.mn = childBound.mx = V3Zero(); // shouldn't be necessary but setting just in case
					curNode.bounds.setEmpty();
					curNode.leafCount = -1;
					curNode.childPageFirstNodeIndex = -1; // using -1 for empty node
				}
			} else // not a terminal page, recurse on count1 nodes cluster
			{
				curNode.childPageFirstNodeIndex = PxI32(resultTree.size());
				curNode.leafCount = 0;
				sort4(permute+cluster4*i, count1, resultTree, maxLevels, childBound, level+1);
			}
			if(i == 0)
				subTreeBound = childBound; // initialize subTreeBound with first childBound
			else
				subTreeBound.include(childBound); // expand subTreeBound with current childBound

			// can use curNode since the reference change due to resizing in recursive call, need to recompute the pointer
			RTreeNodeNQ& curNode1 = resultTree[startNodeIndex+i];
			curNode1.bounds.minimum = childBound.getMinVec3(); // update node bounds using recursively computed childBound
			curNode1.bounds.maximum = childBound.getMaxVec3();
		}
	}
};

// heuristic size reduction factors for splitting heuristic (see how it's used above)
const PxReal SubSortQuick::reductionFactors[RTREE_N-1] = {0.8f, 0.7f, 0.6f};

// sizePerf trade-off presets for sorting routines
const PxU32 SubSortQuick::stopAtTrisPerLeaf1[SubSortQuick::NTRADEOFF] = {16, 14, 12, 10, 8, 7, 6, 5, 4};

/////////////////////////////////////////////////////////////////////////
// SAH sorting method
//
// Preset table: lower index=better size -> higher index = better perf
static const PxU32 NTRADEOFF = 15;
											//	%  -24 -23 -17 -15 -10  -8  -5  -3   0  +3  +3  +5  +7   +8    +9  - % raycast MeshSurface*Random benchmark perf
											//  K  717 734 752 777 793 811 824 866 903 939 971 1030 1087 1139 1266 - testzone size in K
											//  #    0   1   2   3   4   5   6   7   8   9  10  11  12   13     14 - preset number
static const PxU32 stopAtTrisPerPage[NTRADEOFF] = { 64, 60, 56, 48, 46, 44, 40, 36, 32, 28, 24, 20, 16,  12,    12};
static const PxU32 stopAtTrisPerLeaf[NTRADEOFF] = { 16, 14, 12, 10,  9,  8,  8,  6,  5,  5,  5,  4,  4,   4,     2}; // capped at 2 anyway

/////////////////////////////////////////////////////////////////////////
// comparator struct for sorting the bounds along a specified coordIndex (coordIndex=0,1,2 for X,Y,Z)
struct SortBoundsPredicate
{
	PxU32 coordIndex;
	const PxBounds3V* allBounds;
	SortBoundsPredicate(PxU32 coordIndex_, const PxBounds3V* allBounds_) : coordIndex(coordIndex_), allBounds(allBounds_)
	{}

	bool operator()(const PxU32 & idx1, const PxU32 & idx2) const
	{
		// using the bounds center for comparison
		PxF32 center1 = V3ReadXYZ(allBounds[idx1].mn)[coordIndex] + V3ReadXYZ(allBounds[idx1].mx)[coordIndex];
		PxF32 center2 = V3ReadXYZ(allBounds[idx2].mn)[coordIndex] + V3ReadXYZ(allBounds[idx2].mx)[coordIndex];
		return (center1 < center2);
	}
};


/////////////////////////////////////////////////////////////////////////
// auxiliary class for SAH build (SAH = surface area heuristic)
struct Interval
{
	PxU32 start, count;
	Interval(PxU32 s, PxU32 c) : start(s), count(c) {}
};

// SAH function - returns surface area for given AABB extents
static PX_FORCE_INLINE void PxSAH(const Vec3VArg v, PxF32& sah)
{
	FStore(V3Dot(v, V3PermZXY(v)), &sah); // v.x*v.y + v.y*v.z + v.x*v.z;
}

struct SubSortSAH
{
	PxU32* PX_RESTRICT permuteStart, *PX_RESTRICT tempPermute;
	const PxBounds3V* PX_RESTRICT allBounds;
	PxF32* PX_RESTRICT metricL;
	PxF32* PX_RESTRICT metricR;
	const PxU32* PX_RESTRICT xOrder, *PX_RESTRICT yOrder, *PX_RESTRICT zOrder;
	const PxU32* PX_RESTRICT xRanks, *PX_RESTRICT yRanks, *PX_RESTRICT zRanks;
	PxU32* PX_RESTRICT tempRanks;
	PxU32 nbTotalBounds;
	PxU32 iTradeOff;

	// precompute various values used during sort
	SubSortSAH(
		PxU32* permute, const PxBounds3V* allBounds_, PxU32 numBounds,
		const PxU32* xOrder_, const PxU32* yOrder_, const PxU32* zOrder_,
		const PxU32* xRanks_, const PxU32* yRanks_, const PxU32* zRanks_, PxReal sizePerfTradeOff01)
			: permuteStart(permute), allBounds(allBounds_),
			xOrder(xOrder_), yOrder(yOrder_), zOrder(zOrder_),
			xRanks(xRanks_), yRanks(yRanks_), zRanks(zRanks_), nbTotalBounds(numBounds)
	{
		metricL = reinterpret_cast<PxF32*>(PX_ALLOC(sizeof(PxF32)*numBounds, PX_DEBUG_EXP("metricL")));
		metricR = reinterpret_cast<PxF32*>(PX_ALLOC(sizeof(PxF32)*numBounds, PX_DEBUG_EXP("metricR")));
		tempPermute = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*(numBounds*2+1), PX_DEBUG_EXP("tempPermute")));
		tempRanks = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*numBounds, PX_DEBUG_EXP("tempRanks")));
		iTradeOff = PxMin<PxU32>( PxU32(PxMax<PxReal>(0.0f, sizePerfTradeOff01)*NTRADEOFF), NTRADEOFF-1 );
	}

	~SubSortSAH() // release temporarily used memory
	{
		PX_FREE_AND_RESET(metricL);
		PX_FREE_AND_RESET(metricR);
		PX_FREE_AND_RESET(tempPermute);
		PX_FREE_AND_RESET(tempRanks);
	}

	////////////////////////////////////////////////////////////////////
	// returns split position for second array start relative to permute ptr
	PxU32 split(PxU32* permute, PxU32 clusterSize)
	{
		if(clusterSize <= 1)
			return 0;
		if(clusterSize == 2)
			return 1;

		PxI32 minCount = clusterSize >= 4 ? 2 : 1;
		PxI32 splitStartL = minCount; // range=[startL->endL)
		PxI32 splitEndL = PxI32(clusterSize-minCount);
		PxI32 splitStartR = PxI32(clusterSize-splitStartL); // range=(endR<-startR], startR > endR
		PxI32 splitEndR = PxI32(clusterSize-splitEndL);
		PX_ASSERT(splitEndL-splitStartL == splitStartR-splitEndR);
		PX_ASSERT(splitStartL <= splitEndL);
		PX_ASSERT(splitStartR >= splitEndR);
		PX_ASSERT(splitEndR >= 1);
		PX_ASSERT(splitEndL < PxI32(clusterSize));

		// pick the best axis with some splitting metric
		// axis index is X=0, Y=1, Z=2
		PxF32 minMetric[3];
		PxU32 minMetricSplit[3];
		const PxU32* ranks3[3] = { xRanks, yRanks, zRanks };
		const PxU32* orders3[3] = { xOrder, yOrder, zOrder };
		for(PxU32 coordIndex = 0; coordIndex <= 2; coordIndex++)
		{
			SortBoundsPredicate sortPredicateLR(coordIndex, allBounds);

			const PxU32* rank = ranks3[coordIndex];
			const PxU32* order = orders3[coordIndex];

			// build ranks in tempPermute
			if(clusterSize == nbTotalBounds) // AP: about 4% perf gain from this optimization
			{
				// if this is a full cluster sort, we already have it done
				for(PxU32 i = 0; i < clusterSize; i ++)
					tempPermute[i] = order[i];
			} else
			{
				// sort the tempRanks
				for(PxU32 i = 0; i < clusterSize; i ++)
					tempRanks[i] = rank[permute[i]];
				Ps::sort(tempRanks, clusterSize);
				for(PxU32 i = 0; i < clusterSize; i ++) // convert back from ranks to indices
					tempPermute[i] = order[tempRanks[i]];
			}

			// we consider overlapping intervals for minimum sum of metrics
			// left interval is from splitStartL up to splitEndL
			// right interval is from splitStartR down to splitEndR


			// first compute the array metricL
			Vec3V boundsLmn = allBounds[tempPermute[0]].mn; // init with 0th bound
			Vec3V boundsLmx = allBounds[tempPermute[0]].mx; // init with 0th bound
			PxI32 ii;
			for(ii = 1; ii < splitStartL; ii++) // sweep right to include all bounds up to splitStartL-1
			{
				boundsLmn = V3Min(boundsLmn, allBounds[tempPermute[ii]].mn);
				boundsLmx = V3Max(boundsLmx, allBounds[tempPermute[ii]].mx);
			}

			PxU32 countL0 = 0;
			for(ii = splitStartL; ii <= splitEndL; ii++) // compute metric for inclusive bounds from splitStartL to splitEndL
			{
				boundsLmn = V3Min(boundsLmn, allBounds[tempPermute[ii]].mn);
				boundsLmx = V3Max(boundsLmx, allBounds[tempPermute[ii]].mx);
				PxSAH(V3Sub(boundsLmx, boundsLmn), metricL[countL0++]);
			}
			// now we have metricL

			// now compute the array metricR
			Vec3V boundsRmn = allBounds[tempPermute[clusterSize-1]].mn; // init with last bound
			Vec3V boundsRmx = allBounds[tempPermute[clusterSize-1]].mx; // init with last bound
			for(ii = PxI32(clusterSize-2); ii > splitStartR; ii--) // include bounds to the left of splitEndR down to splitStartR
			{
				boundsRmn = V3Min(boundsRmn, allBounds[tempPermute[ii]].mn);
				boundsRmx = V3Max(boundsRmx, allBounds[tempPermute[ii]].mx);
			}

			PxU32 countR0 = 0;
			for(ii = splitStartR; ii >= splitEndR; ii--) // continue sweeping left, including bounds and recomputing the metric
			{
				boundsRmn = V3Min(boundsRmn, allBounds[tempPermute[ii]].mn);
				boundsRmx = V3Max(boundsRmx, allBounds[tempPermute[ii]].mx);
				PxSAH(V3Sub(boundsRmx, boundsRmn), metricR[countR0++]);
			}

			PX_ASSERT((countL0 == countR0) && (countL0 == PxU32(splitEndL-splitStartL+1)));

			// now iterate over splitRange and compute the minimum sum of SAHLeft*countLeft + SAHRight*countRight
			PxU32 minMetricSplitPosition = 0;
			PxF32 minMetricLocal = PX_MAX_REAL;
			const PxI32 hsI32 = PxI32(clusterSize/2);
			const PxI32 splitRange = (splitEndL-splitStartL+1);
			for(ii = 0; ii < splitRange; ii++)
			{
				PxF32 countL = PxF32(ii+minCount); // need to add minCount since ii iterates over splitRange
				PxF32 countR = PxF32(splitRange-ii-1+minCount);
				PX_ASSERT(PxU32(countL + countR) == clusterSize);

				const PxF32 metric = (countL*metricL[ii] + countR*metricR[splitRange-ii-1]);
				const PxU32 splitPos = PxU32(ii+splitStartL);
				if(metric < minMetricLocal || 
					(metric <= minMetricLocal && // same metric but more even split
						PxAbs(PxI32(splitPos)-hsI32) < PxAbs(PxI32(minMetricSplitPosition)-hsI32)))
				{
					minMetricLocal = metric;
					minMetricSplitPosition = splitPos;
				}
			}

			minMetric[coordIndex] = minMetricLocal;
			minMetricSplit[coordIndex] = minMetricSplitPosition;

			// sum of axis lengths for both left and right AABBs
		}

		PxU32 winIndex = 2;
		if(minMetric[0] <= minMetric[1] && minMetric[0] <= minMetric[2])
			winIndex = 0;
		else if(minMetric[1] <= minMetric[2])
			winIndex = 1;

		const PxU32* rank = ranks3[winIndex];
		const PxU32* order = orders3[winIndex];
		if(clusterSize == nbTotalBounds) // AP: about 4% gain from this special case optimization
		{
			// if this is a full cluster sort, we already have it done
			for(PxU32 i = 0; i < clusterSize; i ++)
				permute[i] = order[i];
		} else
		{
			// sort the tempRanks
			for(PxU32 i = 0; i < clusterSize; i ++)
				tempRanks[i] = rank[permute[i]];
			Ps::sort(tempRanks, clusterSize);
			for(PxU32 i = 0; i < clusterSize; i ++)
				permute[i] = order[tempRanks[i]];
		}

		PxU32 splitPoint = minMetricSplit[winIndex];
		if(clusterSize == 3 && splitPoint == 0)
			splitPoint = 1; // special case due to rounding
		return splitPoint;
	}

	// compute surface area for a given split
	PxF32 computeSA(const PxU32* permute, const Interval& split) // both permute and i are relative
	{
		PX_ASSERT(split.count >= 1);
		Vec3V bmn = allBounds[permute[split.start]].mn;
		Vec3V bmx = allBounds[permute[split.start]].mx;
		for(PxU32 i = 1; i < split.count; i++)
		{
			const PxBounds3V& b1 = allBounds[permute[split.start+i]];
			bmn = V3Min(bmn, b1.mn); bmx = V3Max(bmx, b1.mx);
		}

		PxF32 ret; PxSAH(V3Sub(bmx, bmn), ret);
		return ret;
	}

	////////////////////////////////////////////////////////////////////
	// main SAH sort routine
	void sort4(PxU32* permute, PxU32 clusterSize,
		Array<RTreeNodeNQ>& resultTree, PxU32& maxLevels, PxU32 level = 0, RTreeNodeNQ* parentNode = NULL)
	{
		PX_UNUSED(parentNode);

		if(level == 0)
			maxLevels = 1;
		else
			maxLevels = PxMax(maxLevels, level+1);

		PxU32 splitPos[RTREE_N];
		for(PxU32 j = 0; j < RTREE_N; j++)
			splitPos[j] = j+1;

		if(clusterSize >= RTREE_N)
		{
			// split into RTREE_N number of regions via RTREE_N-1 subsequent splits
			// each split is represented as a current interval
			// we iterate over currently active intervals and compute it's surface area
			// then we split the interval with maximum surface area
			// AP scaffold: possible optimization - seems like computeSA can be cached for unchanged intervals
			InlineArray<Interval, 1024> splits;
			splits.pushBack(Interval(0, clusterSize));
			for(PxU32 iSplit = 0; iSplit < RTREE_N-1; iSplit++)
			{
				PxF32 maxSAH = -FLT_MAX;
				PxU32 maxSplit = 0xFFFFffff;
				for(PxU32 i = 0; i < splits.size(); i++)
				{
					if(splits[i].count == 1)
						continue;
					PxF32 SAH = computeSA(permute, splits[i])*splits[i].count;
					if(SAH > maxSAH)
					{
						maxSAH = SAH;
						maxSplit = i;
					}
				}
				PX_ASSERT(maxSplit != 0xFFFFffff);

				// maxSplit is now the index of the interval in splits array with maximum surface area
				// we now split it into 2 using the split() function
				Interval old = splits[maxSplit];
				PX_ASSERT(old.count > 1);
				PxU32 splitLocal = split(permute+old.start, old.count); // relative split pos

				PX_ASSERT(splitLocal >= 1);
				PX_ASSERT(old.count-splitLocal >= 1);
				splits.pushBack(Interval(old.start, splitLocal));
				splits.pushBack(Interval(old.start+splitLocal, old.count-splitLocal));
				splits.replaceWithLast(maxSplit);
				splitPos[iSplit] = old.start+splitLocal;
			}

			// verification code, make sure split counts add up to clusterSize
			PX_ASSERT(splits.size() == RTREE_N);
			PxU32 sum = 0;
			for(PxU32 j = 0; j < RTREE_N; j++)
				sum += splits[j].count;
			PX_ASSERT(sum == clusterSize);
		}
		else // clusterSize < RTREE_N
		{
			// make it so splitCounts based on splitPos add up correctly for small cluster sizes
			for(PxU32 i = clusterSize; i < RTREE_N-1; i++)
				splitPos[i] = clusterSize;
		}

		// sort splitPos index array using quicksort (just a few values)
		Ps::sort(splitPos, RTREE_N-1);
		splitPos[RTREE_N-1] = clusterSize; // splitCount[n] is computed as splitPos[n+1]-splitPos[n], so we need to add this last value

		// now compute splitStarts and splitCounts from splitPos[] array. Also perform a bunch of correctness verification
		PxU32 splitStarts[RTREE_N];
		PxU32 splitCounts[RTREE_N];
		splitStarts[0] = 0;
		splitCounts[0] = splitPos[0];
		PxU32 sumCounts = splitCounts[0];
		for(PxU32 j = 1; j < RTREE_N; j++)
		{
			splitStarts[j] = splitPos[j-1];
			PX_ASSERT(splitStarts[j-1]<=splitStarts[j]);
			splitCounts[j] = splitPos[j]-splitPos[j-1];
			PX_ASSERT(splitCounts[j] > 0 || clusterSize < RTREE_N);
			sumCounts += splitCounts[j];
			PX_ASSERT(splitStarts[j-1]+splitCounts[j-1]<=splitStarts[j]);
		}
		PX_ASSERT(sumCounts == clusterSize);
		PX_ASSERT(splitStarts[RTREE_N-1]+splitCounts[RTREE_N-1]<=clusterSize);

		// mark this cluster as terminal based on clusterSize <= stopAtTrisPerPage parameter for current iTradeOff user specified preset
		bool terminalClusterByTotalCount = (clusterSize <= stopAtTrisPerPage[iTradeOff]);
		// iterate over splitCounts for the current cluster, if any of counts exceed 16 (which is the maximum supported by LeafTriangles
		// we cannot mark this cluster as terminal (has to be split more)
		for(PxU32 s = 0; s < RTREE_N; s++)
			if(splitCounts[s] > 16) // LeafTriangles doesn't support > 16 tris
				terminalClusterByTotalCount = false;

		// iterate over all the splits
		for(PxU32 s = 0; s < RTREE_N; s++)
		{
			RTreeNodeNQ rtn;
			PxU32 splitCount = splitCounts[s];
			if(splitCount > 0) // splits shouldn't be empty generally
			{
				// sweep left to right and compute min and max SAH for each individual bound in current split
				PxBounds3V b = allBounds[permute[splitStarts[s]]];
				PxF32 sahMin; PxSAH(b.getExtents(), sahMin);
				PxF32 sahMax = sahMin;
				// AP scaffold - looks like this could be optimized (we are recomputing bounds top down)
				for(PxU32 i = 1; i < splitCount; i++)
				{
					PxU32 localIndex = i + splitStarts[s];
					const PxBounds3V& b1 = allBounds[permute[localIndex]];
					PxF32 sah1; PxSAH(b1.getExtents(), sah1);
					sahMin = PxMin(sahMin, sah1);
					sahMax = PxMax(sahMax, sah1);
					b.include(b1);
				}

				rtn.bounds.minimum = V3ReadXYZ(b.mn);
				rtn.bounds.maximum = V3ReadXYZ(b.mx);

				// if bounds differ widely (according to some heuristic preset), we continue splitting
				// this is important for a mixed cluster with large and small triangles
				bool okSAH = (sahMax/sahMin < 40.0f);
				if(!okSAH)
					terminalClusterByTotalCount = false; // force splitting this cluster

				bool stopSplitting = // compute the final splitting criterion
					splitCount <= 2 || (okSAH && splitCount <= 3) // stop splitting at 2 nodes or if SAH ratio is OK and splitCount <= 3
					|| terminalClusterByTotalCount || splitCount <= stopAtTrisPerLeaf[iTradeOff];
				if(stopSplitting)
				{
					// this is a terminal page then, mark as such
					// first node index is relative to the top level input array beginning
					rtn.childPageFirstNodeIndex = PxI32(splitStarts[s]+(permute-permuteStart));
					rtn.leafCount = PxI32(splitCount);
					PX_ASSERT(splitCount <= 16); // LeafTriangles doesn't support more
				}
				else
				{
					// this is not a terminal page, we will recompute this later, after we recurse on subpages (label ZZZ)
					rtn.childPageFirstNodeIndex = -1;
					rtn.leafCount = 0;
				}
			}
			else // splitCount == 0 at this point, this is an empty paddding node (with current presets it's very rare)
			{
				PX_ASSERT(splitCount == 0);
				rtn.bounds.setEmpty();
				rtn.childPageFirstNodeIndex = -1;
				rtn.leafCount = -1;
			}
			resultTree.pushBack(rtn); // push the new node into the resultTree array
		}

		if(terminalClusterByTotalCount) // abort recursion if terminal cluster
			return;

		// recurse on subpages
		PxU32 parentIndex = resultTree.size() - RTREE_N; // save the parentIndex as specified (array can be resized during recursion)
		for(PxU32 s = 0; s<RTREE_N; s++)
		{
			RTreeNodeNQ* sParent = &resultTree[parentIndex+s]; // array can be resized and relocated during recursion
			if(sParent->leafCount == 0) // only split pages that were marked as non-terminal during splitting (see "label ZZZ" above)
			{
				// all child nodes will be pushed inside of this recursive call,
				// so we set the child pointer for parent node to resultTree.size()
				sParent->childPageFirstNodeIndex = PxI32(resultTree.size());
				sort4(permute+splitStarts[s], splitCounts[s], resultTree, maxLevels, level+1, sParent);
			}
		}
	}
};




/////////////////////////////////////////////////////////////////////////
// initializes the input permute array with identity permutation
// and shuffles it so that new sorted index, newIndex = resultPermute[oldIndex]
static void buildFromBounds(
	Gu::RTree& result, const PxBounds3V* allBounds, PxU32 numBounds,
	Array<PxU32>& permute, RTreeCooker::RemapCallback* rc, Vec3VArg allMn, Vec3VArg allMx,
	PxReal sizePerfTradeOff01, PxMeshCookingHint::Enum hint)
{
	PX_UNUSED(sizePerfTradeOff01);
	PxBounds3V treeBounds(allMn, allMx);

	// start off with an identity permutation
	permute.resize(0);
	permute.reserve(numBounds+1);
	for(PxU32 j = 0; j < numBounds; j ++)
		permute.pushBack(j);
	const PxU32 sentinel = 0xABCDEF01;
	permute.pushBack(sentinel);

	// load sorted nodes into an RTreeNodeNQ tree representation
	// build the tree structure from sorted nodes
	const PxU32 pageSize = RTREE_N;
	Array<RTreeNodeNQ> resultTree;
	resultTree.reserve(numBounds*2);

	PxU32 maxLevels = 0;
	if(hint == PxMeshCookingHint::eSIM_PERFORMANCE) // use high quality SAH build
	{
		Array<PxU32> xRanks(numBounds), yRanks(numBounds), zRanks(numBounds), xOrder(numBounds), yOrder(numBounds), zOrder(numBounds);
		PxMemCopy(xOrder.begin(), permute.begin(), sizeof(xOrder[0])*numBounds);
		PxMemCopy(yOrder.begin(), permute.begin(), sizeof(yOrder[0])*numBounds);
		PxMemCopy(zOrder.begin(), permute.begin(), sizeof(zOrder[0])*numBounds);
		// sort by shuffling the permutation, precompute sorted ranks for x,y,z-orders
		Ps::sort(xOrder.begin(), xOrder.size(), SortBoundsPredicate(0, allBounds));
		for(PxU32 i = 0; i < numBounds; i++) xRanks[xOrder[i]] = i;
		Ps::sort(yOrder.begin(), yOrder.size(), SortBoundsPredicate(1, allBounds));
		for(PxU32 i = 0; i < numBounds; i++) yRanks[yOrder[i]] = i;
		Ps::sort(zOrder.begin(), zOrder.size(), SortBoundsPredicate(2, allBounds));
		for(PxU32 i = 0; i < numBounds; i++) zRanks[zOrder[i]] = i;

		SubSortSAH ss(permute.begin(), allBounds, numBounds,
			xOrder.begin(), yOrder.begin(), zOrder.begin(), xRanks.begin(), yRanks.begin(), zRanks.begin(), sizePerfTradeOff01);
		ss.sort4(permute.begin(), numBounds, resultTree, maxLevels);
	} else
	{ // use fast cooking path
		PX_ASSERT(hint == PxMeshCookingHint::eCOOKING_PERFORMANCE);
		SubSortQuick ss(permute.begin(), allBounds, numBounds, sizePerfTradeOff01);
		PxBounds3V discard((PxBounds3V::U()));
		ss.sort4(permute.begin(), permute.size()-1, resultTree, maxLevels, discard); // AP scaffold: need to implement build speed/runtime perf slider
	}

	PX_ASSERT(permute[numBounds] == sentinel); // verify we didn't write past the array
	permute.popBack(); // discard the sentinel value

	#if PRINT_RTREE_COOKING_STATS // stats code
		PxU32 totalLeafTris = 0;
		PxU32 numLeaves = 0;
		PxI32 maxLeafTris = 0;
		PxU32 numEmpty = 0;
		for(PxU32 i = 0; i < resultTree.size(); i++)
		{
			PxI32 leafCount = resultTree[i].leafCount;
			numEmpty += (resultTree[i].bounds.isEmpty());
			if(leafCount > 0)
			{
				numLeaves++;
				totalLeafTris += leafCount;
				if(leafCount > maxLeafTris)
					maxLeafTris = leafCount;
			}
		}

		printf("AABBs total/empty=%d/%d\n", resultTree.size(), numEmpty);
		printf("numTris=%d, numLeafAABBs=%d, avgTrisPerLeaf=%.2f, maxTrisPerLeaf = %d\n",
			numBounds, numLeaves, PxF32(totalLeafTris)/numLeaves, maxLeafTris);
	#endif

	PX_ASSERT(RTREE_N*sizeof(RTreeNodeQ) == sizeof(RTreePage)); // needed for nodePtrMultiplier computation to be correct
	const int nodePtrMultiplier = sizeof(RTreeNodeQ); // convert offset as count in qnodes to page ptr

	// Quantize the tree. AP scaffold - might be possible to merge this phase with the page pass below this loop
	Array<RTreeNodeQ> qtreeNodes;
	PxU32 firstEmptyIndex = PxU32(-1);
	PxU32 resultCount = resultTree.size();
	qtreeNodes.reserve(resultCount);

	for(PxU32 i = 0; i < resultCount; i++) // AP scaffold - eliminate this pass
	{
		RTreeNodeNQ & u = resultTree[i];
		RTreeNodeQ q;
		q.setLeaf(u.leafCount > 0); // set the leaf flag
		if(u.childPageFirstNodeIndex == -1) // empty node?
		{
			if(firstEmptyIndex == PxU32(-1))
				firstEmptyIndex = qtreeNodes.size();
			q.minx = q.miny = q.minz = FLT_MAX; // AP scaffold improvement - use empty 1e30 bounds instead and reference a valid leaf
			q.maxx = q.maxy = q.maxz = -FLT_MAX; // that will allow to remove the empty node test from the runtime

			q.ptr = firstEmptyIndex*nodePtrMultiplier; PX_ASSERT((q.ptr & 1) == 0);
			q.setLeaf(true); // label empty node as leaf node
		} else
		{
			// non-leaf node
			q.minx = u.bounds.minimum.x;
			q.miny = u.bounds.minimum.y;
			q.minz = u.bounds.minimum.z;
			q.maxx = u.bounds.maximum.x;
			q.maxy = u.bounds.maximum.y;
			q.maxz = u.bounds.maximum.z;
			if(u.leafCount > 0)
			{
				q.ptr = PxU32(u.childPageFirstNodeIndex);
				rc->remap(&q.ptr, q.ptr, PxU32(u.leafCount));
				PX_ASSERT(q.isLeaf()); // remap is expected to set the isLeaf bit
			}
			else
			{
				// verify that all children bounds are included in the parent bounds
				for(PxU32 s = 0; s < RTREE_N; s++)
				{
					const RTreeNodeNQ& child = resultTree[u.childPageFirstNodeIndex+s];
					PX_UNUSED(child);
					// is a sentinel node or is inside parent's bounds
					PX_ASSERT(child.leafCount == -1 || child.bounds.isInside(u.bounds));
				}

				q.ptr = PxU32(u.childPageFirstNodeIndex * nodePtrMultiplier);
				PX_ASSERT(q.ptr % RTREE_N == 0);
				q.setLeaf(false);
			}
		}
		qtreeNodes.pushBack(q);
	}

	// build the final rtree image
	result.mInvDiagonal = PxVec4(1.0f);
	PX_ASSERT(qtreeNodes.size() % RTREE_N == 0);
	result.mTotalNodes = qtreeNodes.size();
	result.mTotalPages = result.mTotalNodes / pageSize;
	result.mPages = static_cast<RTreePage*>(
		Ps::AlignedAllocator<128>().allocate(sizeof(RTreePage)*result.mTotalPages, __FILE__, __LINE__));
	result.mBoundsMin = PxVec4(V3ReadXYZ(treeBounds.mn), 0.0f);
	result.mBoundsMax = PxVec4(V3ReadXYZ(treeBounds.mx), 0.0f);
	result.mDiagonalScaler = (result.mBoundsMax - result.mBoundsMin) / 65535.0f;
	result.mPageSize = pageSize;
	result.mNumLevels = maxLevels;
	PX_ASSERT(result.mTotalNodes % pageSize == 0);
	result.mNumRootPages = 1;

	for(PxU32 j = 0; j < result.mTotalPages; j++)
	{
		RTreePage& page = result.mPages[j];
		for(PxU32 k = 0; k < RTREE_N; k ++)
		{
			const RTreeNodeQ& n = qtreeNodes[j*RTREE_N+k];
			page.maxx[k] = n.maxx;
			page.maxy[k] = n.maxy;
			page.maxz[k] = n.maxz;
			page.minx[k] = n.minx;
			page.miny[k] = n.miny;
			page.minz[k] = n.minz;
			page.ptrs[k] = n.ptr;
		}
	}

	//printf("Tree size=%d\n", result.mTotalPages*sizeof(RTreePage));
#if PX_DEBUG
	result.validate(); // make sure the child bounds are included in the parent and other validation
#endif
}

} // namespace physx
