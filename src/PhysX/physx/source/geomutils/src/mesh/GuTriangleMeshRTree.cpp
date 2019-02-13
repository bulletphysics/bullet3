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

#include "GuTriangleMesh.h"
#include "GuTriangleMeshRTree.h"
#if PX_ENABLE_DYNAMIC_MESH_RTREE
#include "GuConvexEdgeFlags.h"
#endif

using namespace physx;

namespace physx
{

Gu::RTreeTriangleMesh::RTreeTriangleMesh(GuMeshFactory& factory, TriangleMeshData& d)
:	TriangleMesh(factory, d)
{
	PX_ASSERT(d.mType==PxMeshMidPhase::eBVH33);

	RTreeTriangleData& rtreeData = static_cast<RTreeTriangleData&>(d);
	mRTree = rtreeData.mRTree;
	rtreeData.mRTree.mPages = NULL;
}

Gu::TriangleMesh* Gu::RTreeTriangleMesh::createObject(PxU8*& address, PxDeserializationContext& context)
{
	RTreeTriangleMesh* obj = new (address) RTreeTriangleMesh(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(RTreeTriangleMesh);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void Gu::RTreeTriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	mRTree.exportExtraData(stream);
	TriangleMesh::exportExtraData(stream);
}

void Gu::RTreeTriangleMesh::importExtraData(PxDeserializationContext& context)
{
	mRTree.importExtraData(context);
	TriangleMesh::importExtraData(context);
}

#if PX_ENABLE_DYNAMIC_MESH_RTREE
PxVec3 * Gu::RTreeTriangleMesh::getVerticesForModification()
{
	return const_cast<PxVec3*>(getVertices());
}

template<typename IndexType>
struct RefitCallback : Gu::RTree::CallbackRefit
{
	const PxVec3* newPositions;
	const IndexType* indices;

	RefitCallback(const PxVec3* aNewPositions, const IndexType* aIndices) : newPositions(aNewPositions), indices(aIndices) {}
	PX_FORCE_INLINE ~RefitCallback() {}

	virtual void recomputeBounds(PxU32 index, shdfnd::aos::Vec3V& aMn, shdfnd::aos::Vec3V& aMx)
	{
		using namespace shdfnd::aos;

		// Each leaf box has a set of triangles
		Gu::LeafTriangles currentLeaf; currentLeaf.Data = index;
		PxU32 nbTris = currentLeaf.GetNbTriangles();
		PxU32 baseTri = currentLeaf.GetTriangleIndex();
		PX_ASSERT(nbTris > 0);
		const IndexType* vInds = indices + 3 * baseTri;
		Vec3V vPos = V3LoadU(newPositions[vInds[0]]);
		Vec3V mn = vPos, mx = vPos;
		//PxBounds3 result(newPositions[vInds[0]], newPositions[vInds[0]]);
		vPos = V3LoadU(newPositions[vInds[1]]);
		mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		vPos = V3LoadU(newPositions[vInds[2]]);
		mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		for (PxU32 i = 1; i < nbTris; i++)
		{
			const IndexType* vInds1 = indices + 3 * (baseTri + i);
			vPos = V3LoadU(newPositions[vInds1[0]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
			vPos = V3LoadU(newPositions[vInds1[1]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
			vPos = V3LoadU(newPositions[vInds1[2]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		}

		aMn = mn;
		aMx = mx;
	}
};

PxBounds3 Gu::RTreeTriangleMesh::refitBVH()
{
	PxBounds3 meshBounds;
	if (has16BitIndices())
	{
		RefitCallback<PxU16> cb(mVertices, static_cast<const PxU16*>(mTriangles));
		mRTree.refitAllStaticTree(cb, &meshBounds);
	}
	else
	{
		RefitCallback<PxU32> cb(mVertices, static_cast<const PxU32*>(mTriangles));
		mRTree.refitAllStaticTree(cb, &meshBounds);
	}

	// reset edge flags and remember we did that using a mesh flag (optimization)
	if ((mRTree.mFlags & RTree::IS_EDGE_SET) == 0)
	{
		mRTree.mFlags |= RTree::IS_EDGE_SET;
		if(mExtraTrigData)
		{
			const PxU32 nbTris = getNbTriangles();
			for(PxU32 i = 0; i < nbTris; i++)
				mExtraTrigData[i] |= ETD_CONVEX_EDGE_ALL;
		}
	}

	mAABB = meshBounds;
	return meshBounds;
}
#endif

} // namespace physx
