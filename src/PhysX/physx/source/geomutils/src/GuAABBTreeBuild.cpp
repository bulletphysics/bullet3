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


#include "GuAABBTreeBuild.h"

#include "PsMathUtils.h"
#include "PsFoundation.h"
#include "GuInternal.h"

using namespace physx;
using namespace Gu;

NodeAllocator::NodeAllocator() : mPool(NULL), mCurrentSlabIndex(0), mTotalNbNodes(0)
{
}

NodeAllocator::~NodeAllocator()
{
	release();
}

void NodeAllocator::release()
{
	const PxU32 nbSlabs = mSlabs.size();
	for (PxU32 i = 0; i<nbSlabs; i++)
	{
		Slab& s = mSlabs[i];
		PX_DELETE_ARRAY(s.mPool);
	}

	mSlabs.reset();
	mCurrentSlabIndex = 0;
	mTotalNbNodes = 0;
}

void NodeAllocator::init(PxU32 nbPrimitives, PxU32 limit)
{
	const PxU32 maxSize = nbPrimitives * 2 - 1;	// PT: max possible #nodes for a complete tree
	const PxU32 estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / limit;
	mPool = PX_NEW(AABBTreeBuildNode)[estimatedFinalSize];
	PxMemZero(mPool, sizeof(AABBTreeBuildNode)*estimatedFinalSize);

	// Setup initial node. Here we have a complete permutation of the app's primitives.
	mPool->mNodeIndex = 0;
	mPool->mNbPrimitives = nbPrimitives;

	mSlabs.pushBack(Slab(mPool, 1, estimatedFinalSize));
	mCurrentSlabIndex = 0;
	mTotalNbNodes = 1;
}

// PT: TODO: inline this?
AABBTreeBuildNode* NodeAllocator::getBiNode()
{
	mTotalNbNodes += 2;
	Slab& currentSlab = mSlabs[mCurrentSlabIndex];
	if (currentSlab.mNbUsedNodes + 2 <= currentSlab.mMaxNbNodes)
	{
		AABBTreeBuildNode* biNode = currentSlab.mPool + currentSlab.mNbUsedNodes;
		currentSlab.mNbUsedNodes += 2;
		return biNode;
	}
	else
	{
		// Allocate new slab
		const PxU32 size = 1024;
		AABBTreeBuildNode* pool = PX_NEW(AABBTreeBuildNode)[size];
		PxMemZero(pool, sizeof(AABBTreeBuildNode)*size);

		mSlabs.pushBack(Slab(pool, 2, size));
		mCurrentSlabIndex++;
		return pool;
	}
}

static PX_FORCE_INLINE float getSplittingValue(const PxBounds3& global_box, PxU32 axis)
{
	// Default split value = middle of the axis (using only the box)
	return global_box.getCenter(axis);
}

static PxU32 split(const PxBounds3& box, PxU32 nb, PxU32* const PX_RESTRICT prims, PxU32 axis, const AABBTreeBuildParams& params)
{
	// Get node split value
	const float splitValue = getSplittingValue(box, axis);

	PxU32 nbPos = 0;
	// Loop through all node-related primitives. Their indices range from "mNodePrimitives[0]" to "mNodePrimitives[mNbPrimitives-1]",
	// with mNodePrimitives = mIndices + mNodeIndex (i.e. those indices map the global list in the tree params).

	// PT: to avoid calling the unsafe [] operator
	const size_t ptrValue = size_t(params.mCache) + axis * sizeof(float);
	const PxVec3* /*PX_RESTRICT*/ cache = reinterpret_cast<const PxVec3*>(ptrValue);

	for (PxU32 i = 0; i<nb; i++)
	{
		// Get index in global list
		const PxU32 index = prims[i];

		// Test against the splitting value. The primitive value is tested against the enclosing-box center.
		// [We only need an approximate partition of the enclosing box here.]
		const float primitiveValue = cache[index].x;
		PX_ASSERT(primitiveValue == params.mCache[index][axis]);

		// Reorganize the list of indices in this order: positive - negative.
		if (primitiveValue > splitValue)
		{
			// Swap entries
			prims[i] = prims[nbPos];
			prims[nbPos] = index;
			// Count primitives assigned to positive space
			nbPos++;
		}
	}
	return nbPos;
}

void AABBTreeBuildNode::subdivide(const AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& allocator, PxU32* const indices)
{
	PxU32* const PX_RESTRICT primitives = indices + mNodeIndex;
	const PxU32 nbPrims = mNbPrimitives;

	// Compute global box & means for current node. The box is stored in mBV.
	Vec4V meansV;
	{
		const PxBounds3* PX_RESTRICT boxes = params.mAABBArray;
		PX_ASSERT(boxes);
		PX_ASSERT(primitives);
		PX_ASSERT(nbPrims);

		Vec4V minV = V4LoadU(&boxes[primitives[0]].minimum.x);
		Vec4V maxV = V4LoadU(&boxes[primitives[0]].maximum.x);

		meansV = V4LoadU(&params.mCache[primitives[0]].x);

		for (PxU32 i = 1; i<nbPrims; i++)
		{
			const PxU32 index = primitives[i];
			const Vec4V curMinV = V4LoadU(&boxes[index].minimum.x);
			const Vec4V curMaxV = V4LoadU(&boxes[index].maximum.x);
			meansV = V4Add(meansV, V4LoadU(&params.mCache[index].x));
			minV = V4Min(minV, curMinV);
			maxV = V4Max(maxV, curMaxV);
		}

		StoreBounds(mBV, minV, maxV);

		const float coeff = 1.0f / float(nbPrims);
		meansV = V4Scale(meansV, FLoad(coeff));
	}

	// Check the user-defined limit. Also ensures we stop subdividing if we reach a leaf node.
	if (nbPrims <= params.mLimit)
		return;

	bool validSplit = true;
	PxU32 nbPos;
	{
		// Compute variances
		Vec4V varsV = V4Zero();
		for (PxU32 i = 0; i<nbPrims; i++)
		{
			const PxU32 index = primitives[i];
			Vec4V centerV = V4LoadU(&params.mCache[index].x);
			centerV = V4Sub(centerV, meansV);
			centerV = V4Mul(centerV, centerV);
			varsV = V4Add(varsV, centerV);
		}
		const float coeffNb1 = 1.0f / float(nbPrims - 1);
		varsV = V4Scale(varsV, FLoad(coeffNb1));
		PX_ALIGN(16, PxVec4) vars;
		V4StoreA(varsV, &vars.x);

		// Choose axis with greatest variance
		const PxU32 axis = Ps::largestAxis(PxVec3(vars.x, vars.y, vars.z));

		// Split along the axis
		nbPos = split(mBV, nbPrims, primitives, axis, params);

		// Check split validity
		if (!nbPos || nbPos == nbPrims)
			validSplit = false;
	}

	// Check the subdivision has been successful
	if (!validSplit)
	{
		// Here, all boxes lie in the same sub-space. Two strategies:
		// - if we are over the split limit, make an arbitrary 50-50 split
		// - else stop subdividing
		if (nbPrims>params.mLimit)
		{
			nbPos = nbPrims >> 1;
		}
		else return;
	}

	// Now create children and assign their pointers.
	mPos = allocator.getBiNode();

	stats.increaseCount(2);

	// Assign children
	PX_ASSERT(!isLeaf());
	AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(mPos);
	AABBTreeBuildNode* Neg = Pos + 1;
	Pos->mNodeIndex = mNodeIndex;
	Pos->mNbPrimitives = nbPos;
	Neg->mNodeIndex = mNodeIndex + nbPos;
	Neg->mNbPrimitives = mNbPrimitives - nbPos;
}

void AABBTreeBuildNode::_buildHierarchy(AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& nodeBase, PxU32* const indices)
{
	// Subdivide current node
	subdivide(params, stats, nodeBase, indices);

	// Recurse
	if (!isLeaf())
	{
		AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(getPos());
		PX_ASSERT(Pos);
		AABBTreeBuildNode* Neg = Pos + 1;
		Pos->_buildHierarchy(params, stats, nodeBase, indices);
		Neg->_buildHierarchy(params, stats, nodeBase, indices);
	}

	stats.mTotalPrims += mNbPrimitives;
}

bool Gu::initAABBTreeBuild(AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats, PxU32*&  indices)
{
	const PxU32 numPrimitives = params.mNbPrimitives;

	if(numPrimitives == 0)
		return false;	

	// indices already initialized!
	if(indices)
		return false;

	// Init stats
	stats.setCount(1);

	// Initialize indices. This list will be modified during build.
	indices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*numPrimitives, "AABB tree indices"));
	// Identity permutation
	for(PxU32 i=0;i<numPrimitives;i++)
		indices[i] = i;

	// Allocate a pool of nodes
	nodeAllocator.init(numPrimitives, params.mLimit);

	// Compute box centers only once and cache them
	params.mCache = reinterpret_cast<PxVec3*>(PX_ALLOC(sizeof(PxVec3)*(numPrimitives+1), "cache"));
	const float half = 0.5f;
	const FloatV halfV = FLoad(half);
	for(PxU32 i=0;i<numPrimitives;i++)
	{
		const Vec4V curMinV = V4LoadU(&params.mAABBArray[i].minimum.x);
		const Vec4V curMaxV = V4LoadU(&params.mAABBArray[i].maximum.x);
		const Vec4V centerV = V4Scale(V4Add(curMaxV, curMinV), halfV);
		V4StoreU(centerV, &params.mCache[i].x);
	}

	return true;
}

bool Gu::buildAABBTree(AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats, PxU32*&  indices)
{
	// initialize the build first
	if(!initAABBTreeBuild(params, nodeAllocator, stats, indices))
		return false;

	// Build the hierarchy
	nodeAllocator.mPool->_buildHierarchy(params, stats, nodeAllocator, indices);

	return true;
}

