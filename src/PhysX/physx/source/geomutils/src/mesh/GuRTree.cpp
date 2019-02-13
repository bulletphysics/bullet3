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

#include "foundation/PxPreprocessor.h"

#define RTREE_TEXT_DUMP_ENABLE		0
#if PX_P64_FAMILY
#define RTREE_PAGES_PER_POOL_SLAB	16384 // preallocate all pages in first batch to make sure we stay within 32 bits for relative pointers.. this is 2 megs
#else
#define RTREE_PAGES_PER_POOL_SLAB	128
#endif

#define INSERT_SCAN_LOOKAHEAD		1 // enable one level lookahead scan for determining which child page is best to insert a node into

#define RTREE_INFLATION_EPSILON 5e-4f

#include "GuRTree.h"
#include "PsSort.h"
#include "GuSerialize.h"
#include "CmUtils.h"
#include "PsUtilities.h"

using namespace physx;
#if PX_ENABLE_DYNAMIC_MESH_RTREE
using namespace shdfnd::aos;
#endif
using Ps::Array;
using Ps::sort;
using namespace Gu;

namespace physx
{
namespace Gu {

bool RTree::load(PxInputStream& stream, PxU32 meshVersion, bool mismatch_)	// PT: 'meshVersion' is the PX_MESH_VERSION from cooked file
{
	PX_UNUSED(meshVersion);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if(a!='R' || b!='T' || c!='R' || d!='E')
		return false;

	bool mismatch;
	PxU32 fileVersion;
	if(!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	readFloatBuffer(&mBoundsMin.x, 4, mismatch, stream);
	readFloatBuffer(&mBoundsMax.x, 4, mismatch, stream);
	readFloatBuffer(&mInvDiagonal.x, 4, mismatch, stream);
	readFloatBuffer(&mDiagonalScaler.x, 4, mismatch, stream);
	mPageSize = readDword(mismatch, stream);
	mNumRootPages = readDword(mismatch, stream);
	mNumLevels = readDword(mismatch, stream);
	mTotalNodes = readDword(mismatch, stream);
	mTotalPages = readDword(mismatch, stream);
	PxU32 unused = readDword(mismatch, stream); PX_UNUSED(unused); // backwards compatibility
	mPages = static_cast<RTreePage*>(Ps::AlignedAllocator<128>().allocate(sizeof(RTreePage)*mTotalPages, __FILE__, __LINE__));
	Cm::markSerializedMem(mPages, sizeof(RTreePage)*mTotalPages);
	for(PxU32 j=0; j<mTotalPages; j++)
	{
		readFloatBuffer(mPages[j].minx, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].miny, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].minz, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxx, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxy, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxz, RTREE_N, mismatch, stream);
		ReadDwordBuffer(mPages[j].ptrs, RTREE_N, mismatch, stream);
	}
	return true;
}

/////////////////////////////////////////////////////////////////////////
PxU32 RTree::computeBottomLevelCount(PxU32 multiplier) const
{
	PxU32 topCount = 0, curCount = mNumRootPages;
	const RTreePage* rightMostPage = &mPages[mNumRootPages-1];
	PX_ASSERT(rightMostPage);
	for (PxU32 level = 0; level < mNumLevels-1; level++)
	{
		topCount += curCount;
		PxU32 nc = rightMostPage->nodeCount();
		PX_ASSERT(nc > 0 && nc <= RTREE_N);
		// old version pointer, up to PX_MESH_VERSION 8
		PxU32 ptr = (rightMostPage->ptrs[nc-1]) * multiplier;
		PX_ASSERT(ptr % sizeof(RTreePage) == 0);
		const RTreePage* rightMostPageNext = mPages + (ptr / sizeof(RTreePage));
		curCount = PxU32(rightMostPageNext - rightMostPage);
		rightMostPage = rightMostPageNext;
	}

	return mTotalPages - topCount;
}

/////////////////////////////////////////////////////////////////////////
RTree::RTree(const PxEMPTY)
{
	mFlags |= USER_ALLOCATED;
}


// PX_SERIALIZATION
/////////////////////////////////////////////////////////////////////////
void RTree::exportExtraData(PxSerializationContext& stream)
{
	stream.alignData(128);
	stream.writeData(mPages, mTotalPages*sizeof(RTreePage));
}

/////////////////////////////////////////////////////////////////////////
void RTree::importExtraData(PxDeserializationContext& context)
{
	context.alignExtraData(128);
	mPages = context.readExtraData<RTreePage>(mTotalPages);
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE PxU32 RTreePage::nodeCount() const
{
	for (int j = 0; j < RTREE_N; j ++)
		if (minx[j] == MX)
			return PxU32(j);

	return RTREE_N;
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::clearNode(PxU32 nodeIndex)
{
	PX_ASSERT(nodeIndex < RTREE_N);
	minx[nodeIndex] = miny[nodeIndex] = minz[nodeIndex] = MX; // initialize empty node with sentinels
	maxx[nodeIndex] = maxy[nodeIndex] = maxz[nodeIndex] = MN;
	ptrs[nodeIndex] = 0;
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::getNode(const PxU32 nodeIndex, RTreeNodeQ& r) const
{
	PX_ASSERT(nodeIndex < RTREE_N);
	r.minx = minx[nodeIndex];
	r.miny = miny[nodeIndex];
	r.minz = minz[nodeIndex];
	r.maxx = maxx[nodeIndex];
	r.maxy = maxy[nodeIndex];
	r.maxz = maxz[nodeIndex];
	r.ptr  = ptrs[nodeIndex];
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::setEmpty(PxU32 startIndex)
{
	PX_ASSERT(startIndex < RTREE_N);
	for (PxU32 j = startIndex; j < RTREE_N; j ++)
		clearNode(j);
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::computeBounds(RTreeNodeQ& newBounds)
{
	RTreeValue _minx = MX, _miny = MX, _minz = MX, _maxx = MN, _maxy = MN, _maxz = MN;
	for (PxU32 j = 0; j < RTREE_N; j++)
	{
		if (isEmpty(j))
			continue;
		_minx = PxMin(_minx, minx[j]);
		_miny = PxMin(_miny, miny[j]);
		_minz = PxMin(_minz, minz[j]);
		_maxx = PxMax(_maxx, maxx[j]);
		_maxy = PxMax(_maxy, maxy[j]);
		_maxz = PxMax(_maxz, maxz[j]);
	}
	newBounds.minx = _minx;
	newBounds.miny = _miny;
	newBounds.minz = _minz;
	newBounds.maxx = _maxx;
	newBounds.maxy = _maxy;
	newBounds.maxz = _maxz;
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::adjustChildBounds(PxU32 index, const RTreeNodeQ& adjChild)
{
	PX_ASSERT(index < RTREE_N);
	minx[index] = adjChild.minx;
	miny[index] = adjChild.miny;
	minz[index] = adjChild.minz;
	maxx[index] = adjChild.maxx;
	maxy[index] = adjChild.maxy;
	maxz[index] = adjChild.maxz;
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::growChildBounds(PxU32 index, const RTreeNodeQ& child)
{
	PX_ASSERT(index < RTREE_N);
	minx[index] = PxMin(minx[index], child.minx);
	miny[index] = PxMin(miny[index], child.miny);
	minz[index] = PxMin(minz[index], child.minz);
	maxx[index] = PxMax(maxx[index], child.maxx);
	maxy[index] = PxMax(maxy[index], child.maxy);
	maxz[index] = PxMax(maxz[index], child.maxz);
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::copyNode(PxU32 targetIndex, const RTreePage& sourcePage, PxU32 sourceIndex)
{
	PX_ASSERT(targetIndex < RTREE_N);
	PX_ASSERT(sourceIndex < RTREE_N);
	minx[targetIndex] = sourcePage.minx[sourceIndex];
	miny[targetIndex] = sourcePage.miny[sourceIndex];
	minz[targetIndex] = sourcePage.minz[sourceIndex];
	maxx[targetIndex] = sourcePage.maxx[sourceIndex];
	maxy[targetIndex] = sourcePage.maxy[sourceIndex];
	maxz[targetIndex] = sourcePage.maxz[sourceIndex];
	ptrs[targetIndex] = sourcePage.ptrs[sourceIndex];
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreePage::setNode(PxU32 targetIndex, const RTreeNodeQ& sourceNode)
{
	PX_ASSERT(targetIndex < RTREE_N);
	minx[targetIndex] = sourceNode.minx;
	miny[targetIndex] = sourceNode.miny;
	minz[targetIndex] = sourceNode.minz;
	maxx[targetIndex] = sourceNode.maxx;
	maxy[targetIndex] = sourceNode.maxy;
	maxz[targetIndex] = sourceNode.maxz;
	ptrs[targetIndex] = sourceNode.ptr;
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreeNodeQ::grow(const RTreePage& page, int nodeIndex)
{
	PX_ASSERT(nodeIndex < RTREE_N);
	minx = PxMin(minx, page.minx[nodeIndex]);
	miny = PxMin(miny, page.miny[nodeIndex]);
	minz = PxMin(minz, page.minz[nodeIndex]);
	maxx = PxMax(maxx, page.maxx[nodeIndex]);
	maxy = PxMax(maxy, page.maxy[nodeIndex]);
	maxz = PxMax(maxz, page.maxz[nodeIndex]);
}

/////////////////////////////////////////////////////////////////////////
PX_FORCE_INLINE void RTreeNodeQ::grow(const RTreeNodeQ& node)
{
	minx = PxMin(minx, node.minx); miny = PxMin(miny, node.miny); minz = PxMin(minz, node.minz);
	maxx = PxMax(maxx, node.maxx); maxy = PxMax(maxy, node.maxy); maxz = PxMax(maxz, node.maxz);
}

/////////////////////////////////////////////////////////////////////////
#if PX_ENABLE_DYNAMIC_MESH_RTREE
void RTree::validateRecursive(PxU32 level, RTreeNodeQ parentBounds, RTreePage* page, CallbackRefit* cbLeaf)
#else
void RTree::validateRecursive(PxU32 level, RTreeNodeQ parentBounds, RTreePage* page)
#endif
{
	PX_UNUSED(parentBounds);

	static PxU32 validateCounter = 0; // this is to suppress a warning that recursive call has no side effects
	validateCounter++;

	RTreeNodeQ n;
	PxU32 pageNodeCount = page->nodeCount();
	for (PxU32 j = 0; j < pageNodeCount; j++)
	{
		page->getNode(j, n);
		if (page->isEmpty(j))
			continue;
		PX_ASSERT(n.minx >= parentBounds.minx); PX_ASSERT(n.miny >= parentBounds.miny); PX_ASSERT(n.minz >= parentBounds.minz);
		PX_ASSERT(n.maxx <= parentBounds.maxx); PX_ASSERT(n.maxy <= parentBounds.maxy); PX_ASSERT(n.maxz <= parentBounds.maxz);
		if (!n.isLeaf())
		{
			PX_ASSERT((n.ptr&1) == 0);
			RTreePage* childPage = reinterpret_cast<RTreePage*>(size_t(mPages) + n.ptr);
#if PX_ENABLE_DYNAMIC_MESH_RTREE
			validateRecursive(level+1, n, childPage, cbLeaf);
		} else if (cbLeaf)
		{
			Vec3V mnv, mxv;
			cbLeaf->recomputeBounds(page->ptrs[j] & ~1, mnv, mxv);
			PxVec3 mn3, mx3; V3StoreU(mnv, mn3); V3StoreU(mxv, mx3);
			const PxBounds3 lb(mn3, mx3);
			const PxVec3& mn = lb.minimum; const PxVec3& mx = lb.maximum; PX_UNUSED(mn); PX_UNUSED(mx);
			PX_ASSERT(mn.x >= n.minx); PX_ASSERT(mn.y >= n.miny); PX_ASSERT(mn.z >= n.minz);
			PX_ASSERT(mx.x <= n.maxx); PX_ASSERT(mx.y <= n.maxy); PX_ASSERT(mx.z <= n.maxz);
		}
#else
			validateRecursive(level+1, n, childPage);
		}
#endif
	}
	RTreeNodeQ recomputedBounds;
	page->computeBounds(recomputedBounds);
	PX_ASSERT((recomputedBounds.minx - parentBounds.minx)<=RTREE_INFLATION_EPSILON);
	PX_ASSERT((recomputedBounds.miny - parentBounds.miny)<=RTREE_INFLATION_EPSILON);
	PX_ASSERT((recomputedBounds.minz - parentBounds.minz)<=RTREE_INFLATION_EPSILON);
	PX_ASSERT((recomputedBounds.maxx - parentBounds.maxx)<=RTREE_INFLATION_EPSILON);
	PX_ASSERT((recomputedBounds.maxy - parentBounds.maxy)<=RTREE_INFLATION_EPSILON);
	PX_ASSERT((recomputedBounds.maxz - parentBounds.maxz)<=RTREE_INFLATION_EPSILON);
}

/////////////////////////////////////////////////////////////////////////
#if PX_ENABLE_DYNAMIC_MESH_RTREE
void RTree::validate(CallbackRefit* cbLeaf)
#else
void RTree::validate()
#endif
{
	for (PxU32 j = 0; j < mNumRootPages; j++)
	{
		RTreeNodeQ rootBounds;
		mPages[j].computeBounds(rootBounds);
#if PX_ENABLE_DYNAMIC_MESH_RTREE
		validateRecursive(0, rootBounds, mPages+j, cbLeaf);
#else
		validateRecursive(0, rootBounds, mPages+j);
#endif
	}
}

#if PX_ENABLE_DYNAMIC_MESH_RTREE
void RTree::refitAllStaticTree(CallbackRefit& cb, PxBounds3* retBounds)
{
	PxU8* treeNodes8 = reinterpret_cast<PxU8*>(mPages);

	// since pages are ordered we can scan back to front and the hierarchy will be updated
	for (PxI32 iPage = PxI32(mTotalPages)-1; iPage>=0; iPage--)
	{
		RTreePage& page = mPages[iPage];
		for (PxU32 j = 0; j < RTREE_N; j++)
		{
			if (page.isEmpty(j))
				continue;
			if (page.isLeaf(j))
			{
				Vec3V childMn, childMx;
				cb.recomputeBounds(page.ptrs[j]-1, childMn, childMx); // compute the bound around triangles
				PxVec3 mn3, mx3;
				V3StoreU(childMn, mn3);
				V3StoreU(childMx, mx3);
				page.minx[j] = mn3.x; page.miny[j] = mn3.y; page.minz[j] = mn3.z;
				page.maxx[j] = mx3.x; page.maxy[j] = mx3.y; page.maxz[j] = mx3.z;
			} else
			{
				const RTreePage* child = reinterpret_cast<const RTreePage*>(treeNodes8 + page.ptrs[j]);
				PX_COMPILE_TIME_ASSERT(RTREE_N == 4);
				bool first = true;
				for (PxU32 k = 0; k < RTREE_N; k++)
				{
					if (child->isEmpty(k))
						continue;
					if (first)
					{
						page.minx[j] = child->minx[k]; page.miny[j] = child->miny[k]; page.minz[j] = child->minz[k];
						page.maxx[j] = child->maxx[k]; page.maxy[j] = child->maxy[k]; page.maxz[j] = child->maxz[k];
						first = false;
					} else
					{
						page.minx[j] = PxMin(page.minx[j], child->minx[k]);
						page.miny[j] = PxMin(page.miny[j], child->miny[k]);
						page.minz[j] = PxMin(page.minz[j], child->minz[k]);
						page.maxx[j] = PxMax(page.maxx[j], child->maxx[k]);
						page.maxy[j] = PxMax(page.maxy[j], child->maxy[k]);
						page.maxz[j] = PxMax(page.maxz[j], child->maxz[k]);
					}
				}
			}
		}
	}

	if (retBounds)
	{
		RTreeNodeQ bound1;
		for (PxU32 ii = 0; ii<mNumRootPages; ii++)
		{
			mPages[ii].computeBounds(bound1);
			if (ii == 0)
			{
				retBounds->minimum = PxVec3(bound1.minx, bound1.miny, bound1.minz);
				retBounds->maximum = PxVec3(bound1.maxx, bound1.maxy, bound1.maxz);
			} else
			{
				retBounds->minimum = retBounds->minimum.minimum(PxVec3(bound1.minx, bound1.miny, bound1.minz));
				retBounds->maximum = retBounds->maximum.maximum(PxVec3(bound1.maxx, bound1.maxy, bound1.maxz));
			}
		}
	}

#if PX_CHECKED
	validate(&cb);
#endif
}
#endif // PX_ENABLE_DYNAMIC_MESH_RTREE

//~PX_SERIALIZATION
const RTreeValue RTreePage::MN = -PX_MAX_F32;
const RTreeValue RTreePage::MX = PX_MAX_F32;

} // namespace Gu

}
