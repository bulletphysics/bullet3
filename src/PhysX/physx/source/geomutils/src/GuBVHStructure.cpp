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


#include "GuBVHStructure.h"
#include "GuAABBTreeBuild.h"
#include "GuAABBTreeQuery.h"
#include "GuSerialize.h"
#include "GuBounds.h"
#include "PsFoundation.h"
#include "CmUtils.h"

using namespace physx;
using namespace Gu;

BVHStructure::BVHStructure(GuMeshFactory* factory):	
	PxBVHStructure(PxType(PxConcreteType::eBVH_STRUCTURE), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMeshFactory(factory),
	mNumVolumes(0),
	mNumNodes(0),
	mBounds(NULL),
	mIndices(NULL),
	mVolumes(NULL),
	mNodes(NULL)
{
}

BVHStructure::BVHStructure(GuMeshFactory* factory, BVHStructureData& bvhData):
	PxBVHStructure(PxType(PxConcreteType::eBVH_STRUCTURE), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMeshFactory(factory),
	mNumVolumes(bvhData.mNumVolumes),
	mNumNodes(bvhData.mNumNodes),
	mBounds(bvhData.mBounds),
	mIndices(bvhData.mIndices),
	mVolumes(NULL),
	mNodes(bvhData.mNodes)
{
}

BVHStructure::~BVHStructure()
{	
}

bool BVHStructure::load(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('B', 'V', 'H', 'S', version, mismatch, stream))
		return false;

	// read numVolumes, numNodes together 
	ReadDwordBuffer(&mNumVolumes, 2, mismatch, stream);	

	// read indices
	mIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mNumVolumes, "BVH indices"));
	ReadDwordBuffer(mIndices, mNumVolumes, mismatch, stream);

	// read bounds
	mBounds = reinterpret_cast<PxBounds3*>(PX_ALLOC(sizeof(PxBounds3)*(mNumVolumes + 1), "BVH bounds"));
	readFloatBuffer(&mBounds[0].minimum.x, mNumVolumes*(3 + 3), mismatch, stream);

	// read nodes
	mNodes = reinterpret_cast<BVHNode*>(PX_ALLOC(sizeof(BVHNode)*mNumNodes, "BVH nodes"));
	for(PxU32 i = 0; i < mNumNodes; i++)
	{
		ReadDwordBuffer(&mNodes[i].mData, 1, mismatch, stream);

		readFloatBuffer(&mNodes[i].mBV.minimum.x, 3 + 3, mismatch, stream);		
	}
	return true;
}

void BVHStructure::release()
{
	decRefCount();
}

void BVHStructure::onRefCountZero()
{
	PX_FREE_AND_RESET(mBounds);
	PX_FREE_AND_RESET(mIndices);
	PX_FREE_AND_RESET(mNodes);
	PX_FREE_AND_RESET(mVolumes);

	mNumNodes = 0;
	mNumVolumes = 0;

	if(mMeshFactory->removeBVHStructure(*this))
	{
		const PxType type = getConcreteType();
		GuMeshFactory* mf = mMeshFactory;
		Cm::deletePxBase(this);
		mf->notifyFactoryListener(this, type);
		return;
	}

	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::BVHStructure::release: double deletion detected!");
}

void BVHStructure::createVolumes() const
{
	if(!mVolumes)
	{
		mVolumes = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mNumVolumes, "BVH volume list"));
		for(PxU32 i = 0; i < mNumVolumes; i++)
		{
			mVolumes[i] = i;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Query Implementation
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


PxU32 BVHStructure::raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal maxDist, PxU32 maxHits, PxU32* PX_RESTRICT rayHits) const
{
	createVolumes();

	BVHCallback cbk(rayHits, maxHits);
	BVHTree tree(mNodes, mIndices);
	AABBTreeRaycast<false, BVHTree, BVHNode, PxU32, BVHCallback>()(mVolumes, mBounds, tree, origin, unitDir, maxDist, PxVec3(0.0f), cbk);

	return cbk.mCurrentHitsCount;
}

PxU32 BVHStructure::sweep(const PxBounds3& aabb, const PxVec3& unitDir, PxReal maxDist, PxU32 maxHits, PxU32* PX_RESTRICT sweepHits) const
{
	createVolumes();

	const PxVec3 extents = aabb.getExtents();

	BVHCallback cbk(sweepHits, maxHits);
	BVHTree tree(mNodes, mIndices);

	AABBTreeRaycast<true, BVHTree, BVHNode, PxU32, BVHCallback>()(mVolumes, mBounds, tree, aabb.getCenter(), unitDir, maxDist, extents, cbk);

	return cbk.mCurrentHitsCount;
}


PxU32 BVHStructure::overlap(const PxBounds3& aabb, PxU32 maxHits, PxU32* PX_RESTRICT overlapHits) const
{
	createVolumes();
	
	BVHCallback cbk(overlapHits, maxHits);
	BVHTree tree(mNodes, mIndices);

	const Gu::AABBAABBTest test(aabb);
	AABBTreeOverlap<Gu::AABBAABBTest, BVHTree, BVHNode, PxU32, BVHCallback>()(mVolumes, mBounds, tree, test, cbk);

	return cbk.mCurrentHitsCount;
}

