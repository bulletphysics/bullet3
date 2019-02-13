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


#include "foundation/PxIO.h"
#include "BVHStructureBuilder.h"
#include "GuAABBTreeBuild.h"
#include "GuSerialize.h"
#include "GuBVHStructure.h"

using namespace physx;
using namespace Gu;

// A.B. move this to some common place?
#define NB_OBJECTS_PER_NODE	4

// PT: TODO: - check that this is compatible with Gu::computeBounds(..., SQ_PRUNER_INFLATION, ...)
// PT: TODO: - refactor with "inflateBounds" in GuBounds.cpp if possible
// PT: TODO: - use SQ_PRUNER_INFLATION instead of hardcoding "0.01f"
// A.B. move to common place
PX_FORCE_INLINE void inflateBounds(PxBounds3& dst, const PxBounds3& src)
{
	using namespace physx::shdfnd::aos;

	const Vec4V minV = V4LoadU(&src.minimum.x);
	const Vec4V maxV = V4LoadU(&src.maximum.x);
	const Vec4V eV = V4Scale(V4Sub(maxV, minV), FLoad(0.5f * 0.01f));

	V4StoreU(V4Sub(minV, eV), &dst.minimum.x);
	PX_ALIGN(16, PxVec4) max4;
	V4StoreA(V4Add(maxV, eV), &max4.x);
	dst.maximum = PxVec3(max4.x, max4.y, max4.z);
}

void flatten(const NodeAllocator& nodeAllocator, BVHNode* dest)
{
	// PT: gathers all build nodes allocated so far and flatten them to a linear destination array of smaller runtime nodes
	PxU32 offset = 0;
	const PxU32 nbSlabs = nodeAllocator.mSlabs.size();
	for(PxU32 s=0;s<nbSlabs;s++)
	{
		const NodeAllocator::Slab& currentSlab = nodeAllocator.mSlabs[s];

		AABBTreeBuildNode* pool = currentSlab.mPool;
		for(PxU32 i=0;i<currentSlab.mNbUsedNodes;i++)
		{
			dest[offset].mBV = pool[i].mBV;
			if(pool[i].isLeaf())
			{
				const PxU32 index = pool[i].mNodeIndex;

				const PxU32 nbPrims = pool[i].getNbPrimitives();
				PX_ASSERT(nbPrims<=16);

				dest[offset].mData = (index<<5)|((nbPrims&15)<<1)|1;
			}
			else
			{
				PX_ASSERT(pool[i].mPos);
				PxU32 localNodeIndex = 0xffffffff;
				PxU32 nodeBase = 0;
				for(PxU32 j=0;j<nbSlabs;j++)
				{
					if(pool[i].mPos>= nodeAllocator.mSlabs[j].mPool && pool[i].mPos < nodeAllocator.mSlabs[j].mPool + nodeAllocator.mSlabs[j].mNbUsedNodes)
					{
						localNodeIndex = PxU32(pool[i].mPos - nodeAllocator.mSlabs[j].mPool);
						break;
					}
					nodeBase += nodeAllocator.mSlabs[j].mNbUsedNodes;
				}
				const PxU32 nodeIndex = nodeBase + localNodeIndex;
				dest[offset].mData = nodeIndex<<1;
			}
			offset++;
		}
	}
}

BVHStructureBuilder::BVHStructureBuilder():
	mBounds(NULL),
	mNumVolumes(0),
	mNumNodes(0),
	mNodes(NULL),
	mIndices(NULL)
{
}

BVHStructureBuilder::~BVHStructureBuilder()
{
	PX_FREE_AND_RESET(mBounds);
	PX_FREE_AND_RESET(mNodes);
	PX_FREE_AND_RESET(mIndices);
}

bool BVHStructureBuilder::loadFromDesc(const PxBVHStructureDesc& desc)
{
	PX_ASSERT(desc.isValid());

	const PxU32 numPrimitives = desc.bounds.count;

	// allocate one more for safe SIMD vec4 load
	mBounds = reinterpret_cast<PxBounds3*>(PX_ALLOC((numPrimitives + 1) * sizeof(PxBounds3), "PxBounds3"));

	const PxU8* sB = reinterpret_cast<const PxU8*>(desc.bounds.data);

	for(PxU32 i = 0; i < numPrimitives; i++)
	{
		inflateBounds(mBounds[i], *reinterpret_cast<const PxBounds3*>(sB));
	
		sB += desc.bounds.stride;
	}
	mNumVolumes = numPrimitives;

	// build the BVH
	AABBTreeBuildParams params;
	params.mNbPrimitives = desc.bounds.count;
	params.mAABBArray = mBounds;
	params.mLimit = NB_OBJECTS_PER_NODE;
	BuildStats stats;
	NodeAllocator nodeAllocator;

	const bool buildStatus = buildAABBTree(params, nodeAllocator, stats, mIndices);
	PX_UNUSED(buildStatus);
	PX_ASSERT(buildStatus);

	// store the computed hierarchy
	mNumNodes =  stats.getCount();
	mNodes = reinterpret_cast<BVHNode*>(PX_ALLOC(sizeof(BVHNode)*mNumNodes, "AABB tree nodes"));
	PX_ASSERT(mNumNodes==nodeAllocator.mTotalNbNodes);

	// store the results into BVHNode list
	flatten(nodeAllocator, mNodes);
	nodeAllocator.release();

	return true;
}

// A.B. move to load code
#define PX_BVH_STRUCTURE_VERSION 1

bool BVHStructureBuilder::save(PxOutputStream& stream, bool endian) const
{
	// write header
	if(!writeHeader('B', 'V', 'H', 'S', PX_BVH_STRUCTURE_VERSION, endian, stream))
		return false;

	// write mData members
	writeDword(mNumVolumes, endian, stream);
	writeDword(mNumNodes, endian, stream);

	// write indices and bounds
	for(PxU32 i = 0; i < mNumVolumes; i++)
	{
		writeDword(mIndices[i], endian, stream);
	}

	for(PxU32 i = 0; i < mNumVolumes; i++)
	{
		writeFloatBuffer(&mBounds[i].minimum.x, 3, endian, stream);
		writeFloatBuffer(&mBounds[i].maximum.x, 3, endian, stream);
	}

	// write nodes
	for(PxU32 i = 0; i < mNumNodes; i++)
	{
		writeDword(mNodes[i].mData, endian, stream);

		writeFloatBuffer(&mNodes[i].mBV.minimum.x, 3, endian, stream);
		writeFloatBuffer(&mNodes[i].mBV.maximum.x, 3, endian, stream);
	}

	return true;
}

void BVHStructureBuilder::moveData(Gu::BVHStructureData& bvhData)
{
	bvhData.mBounds = mBounds;
	bvhData.mIndices = mIndices;
	bvhData.mNodes = mNodes;
	bvhData.mNumNodes = mNumNodes;
	bvhData.mNumVolumes = mNumVolumes;

	// set pointers to NULL so we do not release the memory that has been passed to physics create
	mBounds = NULL;
	mIndices = NULL;
	mNodes = NULL;
	mNumNodes = 0;
	mNumVolumes = 0;
}
