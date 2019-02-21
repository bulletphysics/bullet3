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

#include "RTreeCooking.h"
#include "TriangleMeshBuilder.h"
#include "EdgeList.h"
#include "MeshCleaner.h"
#include "GuConvexEdgeFlags.h"
#include "PxTriangleMeshDesc.h"
#include "GuSerialize.h"
#include "Cooking.h"
#include "GuMeshData.h"
#include "GuTriangle32.h"
#include "GuRTree.h"
#include "GuInternal.h"
#include "GuBV4Build.h"
#include "GuBV32Build.h"
#include "PsFoundation.h"
#include "PsHashMap.h"
#include "PsSort.h"

namespace physx {

struct int3
{
	int x, y, z;
};

struct uint3
{
	unsigned int x, y, z;
};

PX_ALIGN_PREFIX(16)
struct uint4
{
	unsigned int x, y, z, w;
}
PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct float4
{
	float x, y, z, w;
}
PX_ALIGN_SUFFIX(16);

}

#include "GrbTriangleMeshCooking.h"

using namespace physx;
using namespace Gu;
using namespace Ps;

namespace physx {

TriangleMeshBuilder::TriangleMeshBuilder(TriangleMeshData& m, const PxCookingParams& params) :
	edgeList	(NULL),
	mParams		(params),
	mMeshData	(m)
{
}

TriangleMeshBuilder::~TriangleMeshBuilder()
{
	releaseEdgeList();
}

void TriangleMeshBuilder::remapTopology(const PxU32* order)
{
	if(!mMeshData.mNbTriangles)
		return;

	// Remap one array at a time to limit memory usage

	Gu::TriangleT<PxU32>* newTopo = reinterpret_cast<Gu::TriangleT<PxU32>*>(PX_ALLOC(mMeshData.mNbTriangles * sizeof(Gu::TriangleT<PxU32>), "Gu::TriangleT<PxU32>"));
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
		newTopo[i]	= reinterpret_cast<Gu::TriangleT<PxU32>*>(mMeshData.mTriangles)[order[i]];
	PX_FREE_AND_RESET(mMeshData.mTriangles);
	mMeshData.mTriangles = newTopo;

	if(mMeshData.mMaterialIndices)
	{
		PxMaterialTableIndex* newMat = PX_NEW(PxMaterialTableIndex)[mMeshData.mNbTriangles];
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMat[i] = mMeshData.mMaterialIndices[order[i]];
		PX_DELETE_POD(mMeshData.mMaterialIndices);
		mMeshData.mMaterialIndices = newMat;
	}

	if(!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
	{
		PxU32* newMap = PX_NEW(PxU32)[mMeshData.mNbTriangles];
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMap[i] = mMeshData.mFaceRemap ? mMeshData.mFaceRemap[order[i]] : order[i];
		PX_DELETE_POD(mMeshData.mFaceRemap);
		mMeshData.mFaceRemap = newMap;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TriangleMeshBuilder::cleanMesh(bool validate, PxTriangleMeshCookingResult::Enum* condition)
{
	PX_ASSERT(mMeshData.mFaceRemap == NULL);

	PxF32 meshWeldTolerance = 0.0f;
	if(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eWELD_VERTICES)
	{
		if(mParams.meshWeldTolerance == 0.f)
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "TriangleMesh: Enable mesh welding with 0 weld tolerance!");
		}
		else
		{
			meshWeldTolerance = mParams.meshWeldTolerance;
		}
	}
	MeshCleaner cleaner(mMeshData.mNbVertices, mMeshData.mVertices, mMeshData.mNbTriangles, reinterpret_cast<const PxU32*>(mMeshData.mTriangles), meshWeldTolerance);
	if(!cleaner.mNbTris)
		return false;

	if(validate)
	{
		// if we do only validate, we check if cleaning did not remove any verts or triangles. 
		// such a mesh can be then directly used for cooking without clean flag
		if((cleaner.mNbVerts != mMeshData.mNbVertices) || (cleaner.mNbTris != mMeshData.mNbTriangles))
		{
			return false;
		}
	}

	// PT: deal with the remap table
	{
		// PT: TODO: optimize this
		if(cleaner.mRemap)
		{
			const PxU32 newNbTris = cleaner.mNbTris;

			// Remap material array
			if(mMeshData.mMaterialIndices)
			{
				PxMaterialTableIndex* tmp = PX_NEW(PxMaterialTableIndex)[newNbTris];
				for(PxU32 i=0;i<newNbTris;i++)
					tmp[i] = mMeshData.mMaterialIndices[cleaner.mRemap[i]];

				PX_DELETE_POD(mMeshData.mMaterialIndices);
				mMeshData.mMaterialIndices = tmp;
			}

			if (!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
			{
				mMeshData.mFaceRemap = PX_NEW(PxU32)[newNbTris];
				PxMemCopy(mMeshData.mFaceRemap, cleaner.mRemap, newNbTris*sizeof(PxU32));
			}
		}
	}

	// PT: deal with geometry
	{
		if(mMeshData.mNbVertices!=cleaner.mNbVerts)
		{
			PX_FREE_AND_RESET(mMeshData.mVertices);
			mMeshData.allocateVertices(cleaner.mNbVerts);
		}
		PxMemCopy(mMeshData.mVertices, cleaner.mVerts, mMeshData.mNbVertices*sizeof(PxVec3));
	}

	// PT: deal with topology
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
		if(mMeshData.mNbTriangles!=cleaner.mNbTris)
		{
			PX_FREE_AND_RESET(mMeshData.mTriangles);
			mMeshData.allocateTriangles(cleaner.mNbTris, true);
		}

		const float testLength = 500.0f*500.0f*mParams.scale.length*mParams.scale.length;
		bool bigTriangle = false;
		const PxVec3* v = mMeshData.mVertices;
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
		{
			const PxU32 vref0 = cleaner.mIndices[i*3+0];
			const PxU32 vref1 = cleaner.mIndices[i*3+1];
			const PxU32 vref2 = cleaner.mIndices[i*3+2];
			PX_ASSERT(vref0!=vref1 && vref0!=vref2 && vref1!=vref2);

			reinterpret_cast<Gu::TriangleT<PxU32>*>(mMeshData.mTriangles)[i].v[0] = vref0;
			reinterpret_cast<Gu::TriangleT<PxU32>*>(mMeshData.mTriangles)[i].v[1] = vref1;
			reinterpret_cast<Gu::TriangleT<PxU32>*>(mMeshData.mTriangles)[i].v[2] = vref2;

			if(		(v[vref0] - v[vref1]).magnitudeSquared() >= testLength
				||	(v[vref1] - v[vref2]).magnitudeSquared() >= testLength
				||	(v[vref2] - v[vref0]).magnitudeSquared() >= testLength
				)
				bigTriangle = true;
		}
		if(bigTriangle)
		{
			if(condition)
				*condition = PxTriangleMeshCookingResult::eLARGE_TRIANGLE;
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "TriangleMesh: triangles are too big, reduce their size to increase simulation stability!");
		}
	}

	return true;
}

void TriangleMeshBuilder::createSharedEdgeData(bool buildAdjacencies, bool buildActiveEdges)
{
	if(buildAdjacencies) // building edges is required if buildAdjacencies is requested
		buildActiveEdges = true;

	PX_ASSERT(mMeshData.mExtraTrigData == NULL);
	PX_ASSERT(mMeshData.mAdjacencies == NULL);

	if(!buildActiveEdges)
		return;

	const PxU32 nTrigs = mMeshData.mNbTriangles;

	mMeshData.mExtraTrigData = PX_NEW(PxU8)[nTrigs];
	memset(mMeshData.mExtraTrigData, 0, sizeof(PxU8)*nTrigs); 

	const Gu::TriangleT<PxU32>* trigs = reinterpret_cast<const Gu::TriangleT<PxU32>*>(mMeshData.mTriangles);
	if(0x40000000 <= nTrigs)
	{
		//mesh is too big for this algo, need to be able to express trig indices in 30 bits, and still have an index reserved for "unused":
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "TriangleMesh: mesh is too big for this algo!");
		return;
	}
	
	createEdgeList();
	if(edgeList)
	{
		PX_ASSERT(edgeList->getNbFaces()==mMeshData.mNbTriangles);
		if(edgeList->getNbFaces()==mMeshData.mNbTriangles)
		{
			for(PxU32 i=0;i<edgeList->getNbFaces();i++)
			{
				const Gu::EdgeTriangleData& ET = edgeList->getEdgeTriangle(i);
				// Replicate flags
				if(Gu::EdgeTriangleAC::HasActiveEdge01(ET))	mMeshData.mExtraTrigData[i] |= Gu::ETD_CONVEX_EDGE_01;
				if(Gu::EdgeTriangleAC::HasActiveEdge12(ET))	mMeshData.mExtraTrigData[i] |= Gu::ETD_CONVEX_EDGE_12;
				if(Gu::EdgeTriangleAC::HasActiveEdge20(ET))	mMeshData.mExtraTrigData[i] |= Gu::ETD_CONVEX_EDGE_20;
			}
		}
	}

	// fill the adjacencies
	if(buildAdjacencies)
	{
		mMeshData.mAdjacencies = PX_NEW(PxU32)[nTrigs*3];
		memset(mMeshData.mAdjacencies, 0xFFFFffff, sizeof(PxU32)*nTrigs*3);		

		PxU32 NbEdges = edgeList->getNbEdges();
		const Gu::EdgeDescData* ED = edgeList->getEdgeToTriangles();
		const Gu::EdgeData* Edges = edgeList->getEdges();
		const PxU32* FBE = edgeList->getFacesByEdges();

		while(NbEdges--)
		{
			// Get number of triangles sharing current edge
			PxU32 Count = ED->Count;
			
			if(Count > 1)
			{
				PxU32 FaceIndex0 = FBE[ED->Offset+0];
				PxU32 FaceIndex1 = FBE[ED->Offset+1];

				const Gu::EdgeData& edgeData = *Edges;
				const Gu::TriangleT<PxU32>& T0 = trigs[FaceIndex0];
				const Gu::TriangleT<PxU32>& T1 = trigs[FaceIndex1];

				PxU32 offset0 = T0.findEdgeCCW(edgeData.Ref0,edgeData.Ref1);				
				PxU32 offset1 = T1.findEdgeCCW(edgeData.Ref0,edgeData.Ref1);

				mMeshData.setTriangleAdjacency(FaceIndex0, FaceIndex1, offset0);
				mMeshData.setTriangleAdjacency(FaceIndex1, FaceIndex0, offset1);
			}
			ED++;
			Edges++;
		}
	}

#if PX_DEBUG
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
	{
		const Gu::TriangleT<PxU32>& T = trigs[i];
		PX_UNUSED(T);
		const Gu::EdgeTriangleData& ET = edgeList->getEdgeTriangle(i);
		PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge01(ET) && (mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_01)) || (!Gu::EdgeTriangleAC::HasActiveEdge01(ET) && !(mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_01)));
		PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge12(ET) && (mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_12)) || (!Gu::EdgeTriangleAC::HasActiveEdge12(ET) && !(mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_12)));
		PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge20(ET) && (mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_20)) || (!Gu::EdgeTriangleAC::HasActiveEdge20(ET) && !(mMeshData.mExtraTrigData[i] & Gu::ETD_CONVEX_EDGE_20)));
	}
#endif
	return;
}

namespace GrbTrimeshCookerHelper
{

struct SortedNeighbor
{
	PxU32 v, a;		// vertex and adjacent vertex
	bool boundary;

	SortedNeighbor(PxU32 v_, PxU32 a_, bool b_): v(v_), a(a_), boundary(b_) {}

	// sort boundary edges to the front so that they are kept when duplicates are removed
	bool operator<(const SortedNeighbor& b) const
	{
		return (v<b.v || (v == b.v && a<b.a) || (v == b.v && a == b.a && boundary && !b.boundary));
	}
};

struct SharpEdgeRange
{
	PxU32 start, length;
	SharpEdgeRange(): start(0), length(0)  {}
	SharpEdgeRange(PxU32 s, PxU32 l): start(s), length(l)  {}
};

class LocalIndexer
{
public:
	bool insert(PxU32 meshIndex)		// returns true if this is a new index
	{
		bool isNew = mMeshToLocal.insert(meshIndex, mLocalToMesh.size());
		if(isNew)
			mLocalToMesh.pushBack(meshIndex);
		return isNew;
	}

	PxU32 meshIndex(PxU32 localIndex)
	{
		PX_ASSERT(localIndex<mLocalToMesh.size());
		return mLocalToMesh[localIndex];
	}

	PxU32 localIndex(PxU32 meshIndex)
	{
		PX_ASSERT(mMeshToLocal.find(meshIndex));
		return mMeshToLocal[meshIndex];
	}
	
	bool contains(PxU32 meshIndex)
	{
		return mMeshToLocal.find(meshIndex) != 0;
	}

	PxU32 size()
	{
		return mLocalToMesh.size();
	}

private:
	Ps::Array<PxU32> mLocalToMesh;
	Ps::HashMap<PxU32, PxU32> mMeshToLocal;
};

#include <stdio.h>


void findSharpVertices(
		Ps::Array<SortedNeighbor>& pairList,
		Ps::Array<SharpEdgeRange>& edgeRanges,
		/*const Ps::Array<Triangle>& triangles,*/
		const uint3* triIndices,
		const uint4* triAdjacencies,
		PxU32 nbTris,
		PxU32 nbVerts
		)
{
	// sort the edges which are sharp or boundary
	for(PxU32 i=0;i<nbTris;i++)
	{
		const uint4& triAdj = triAdjacencies[i];
		const uint3& triIdx = triIndices[i];

		if (!isEdgeNonconvex(triAdj.x))
		{
			pairList.pushBack(SortedNeighbor(triIdx.x, triIdx.y, triAdj.x == BOUNDARY));
			pairList.pushBack(SortedNeighbor(triIdx.y, triIdx.x, triAdj.x == BOUNDARY));
		}

		if (!isEdgeNonconvex(triAdj.y))
		{
			pairList.pushBack(SortedNeighbor(triIdx.y, triIdx.z, triAdj.y == BOUNDARY));
			pairList.pushBack(SortedNeighbor(triIdx.z, triIdx.y, triAdj.y == BOUNDARY));
		}

		if (!isEdgeNonconvex(triAdj.z))
		{
			pairList.pushBack(SortedNeighbor(triIdx.z, triIdx.x, triAdj.z == BOUNDARY));
			pairList.pushBack(SortedNeighbor(triIdx.x, triIdx.z, triAdj.z == BOUNDARY));
		}
	}

	Ps::sort(pairList.begin(), pairList.size());

	// remove duplicates - note that boundary edges are sorted earlier, so we keep them
	PxU32 unique = 1;
	for(PxU32 i=1;i<pairList.size();i++)
	{
		if(pairList[i].v != pairList[i-1].v || pairList[i].a != pairList[i-1].a)			
			pairList[unique++] = pairList[i];
	}
	pairList.resizeUninitialized(unique);

	// a vertex is marked for sharp vertex processing if it has a boundary edge or at least three convex edges
	edgeRanges.resize(nbVerts);
	for(PxU32 p = 0, u ; p<pairList.size(); p = u)
	{
		bool boundary = false;
		for(u=p+1; u<pairList.size() && pairList[u].v == pairList[p].v; u++)
			boundary |= pairList[u].boundary;
		if(boundary || u-p>=3)
			edgeRanges[pairList[p].v] = SharpEdgeRange(p, u-p);
	}
}

#if 0
PxU32 buildVertexConnectionNew_p1(
	Ps::Array<SortedNeighbor> & pairList,
	Ps::Array<SharpEdgeRange> & edgeRanges,
	LocalIndexer & vertexMap,

	const uint4 * triIndices,
	const uint4 * triAdjacencies,
	
	PxU32 nbTris,
	PxU32 nbVerts
	)
{
	findSharpVertices(
		pairList,
		edgeRanges,
		triIndices,
		triAdjacencies,
		nbTris,
		nbVerts
		);

	// add all the original triangles and vertices and record how big the core is
	for(PxU32 i=0; i<nbTris; i++)
	{
		const uint4 & triIdx = triIndices[i];
		vertexMap.insert(triIdx.x);
		vertexMap.insert(triIdx.y);
		vertexMap.insert(triIdx.z);
	}
	PxU32 nbCoreVerts = vertexMap.size();

	PX_ASSERT(nbCoreVerts == nbVerts);

	// add adjacent triangles
	for(PxU32 i=0;i<nbTris;i++)
	{
		const uint4 & triAdj = triAdjacencies[i];

#define IS_TRI(triAdjIdx) (( (triAdjIdx) != BOUNDARY ) && ( !((triAdjIdx) & NONCONVEX_FLAG) ))

		if(IS_TRI(triAdj.x))
		{
			const uint4 & triIdx = triIndices[triAdj.x];
			vertexMap.insert(triIdx.x);
			vertexMap.insert(triIdx.y);
			vertexMap.insert(triIdx.z);
		}

		if(IS_TRI(triAdj.y))
		{
			const uint4 & triIdx = triIndices[triAdj.y];
			vertexMap.insert(triIdx.x);
			vertexMap.insert(triIdx.y);
			vertexMap.insert(triIdx.z);
		}

		if(IS_TRI(triAdj.z))
		{
			const uint4 & triIdx = triIndices[triAdj.z];
			vertexMap.insert(triIdx.x);
			vertexMap.insert(triIdx.y);
			vertexMap.insert(triIdx.z);
			
		}

#undef IS_TRI
	}

	// add the neighbors of the sharp vertices
	PxU32 nbNeighbors = 0;
	for(PxU32 i=0;i<nbCoreVerts;i++)
	{
		PxU32 meshIndex = vertexMap.meshIndex(i);
		const SharpEdgeRange& er = edgeRanges[meshIndex];
		for(PxU32 j = 0;j<er.length;j++)
		{
			PX_ASSERT(pairList[er.start+j].v == meshIndex);
			vertexMap.insert(pairList[er.start + j].a);
		}
		nbNeighbors += er.length;
	}

	return nbNeighbors;
}

void buildVertexConnectionNew_p2(
	PxU32 * adjVertStart,
	PxU32 * vertValency,
	PxU32 * adjVertices,

	Ps::Array<SortedNeighbor>& pairList,
	Ps::Array<SharpEdgeRange>& edgeRanges,
	LocalIndexer & vertexMap,

	const uint4 * /*triIndices*/,
	const uint4 * /*triAdjacencies*/,
	
	PxU32 /*nbTris*/,
	PxU32 nbVerts,
	PxU32 /*nbNeighbors*/
	)
{
	PxU32 n = 0;
	for(PxU32 i=0;i<nbVerts;i++)
	{
		PxU32 meshIdx = vertexMap.meshIndex(i);
		const SharpEdgeRange& er = edgeRanges[vertexMap.meshIndex(i)];
		adjVertStart[meshIdx] = n;
		vertValency[meshIdx] = er.length;
		for(PxU32 j = 0;j<er.length;j++)
			adjVertices[n++] = pairList[er.start+j].a;
	}
}
#else


PxU32 buildVertexConnectionNew_p1(
	Ps::Array<SortedNeighbor> & pairList,
	Ps::Array<SharpEdgeRange> & edgeRanges,
	
	const uint3* triIndices,
	const uint4 * triAdjacencies,

	PxU32 nbTris,
	PxU32 nbVerts
	)
{
	findSharpVertices(
		pairList,
		edgeRanges,
		triIndices,
		triAdjacencies,
		nbTris,
		nbVerts
		);


	// add the neighbors of the sharp vertices
	PxU32 nbNeighbors = 0;
	for (PxU32 i = 0; i<nbVerts; i++)
	{
		const SharpEdgeRange& er = edgeRanges[i];
		nbNeighbors += er.length;
	}

	return nbNeighbors;
}

void buildVertexConnectionNew_p2(
	PxU32 * adjVertStart,
	PxU32 * vertValency,
	PxU32 * adjVertices,

	Ps::Array<SortedNeighbor>& pairList,
	Ps::Array<SharpEdgeRange>& edgeRanges,
	PxU32 nbVerts
	)
{
	PxU32 n = 0;
	for (PxU32 i = 0; i<nbVerts; i++)
	{
		const SharpEdgeRange& er = edgeRanges[i];
		adjVertStart[i] = n;
		vertValency[i] = er.length;
		for (PxU32 j = 0; j<er.length; j++)
			adjVertices[n++] = pairList[er.start + j].a;
	}
}
#endif

} // namespace GrbTrimeshCookerHelper

void TriangleMeshBuilder::recordTriangleIndices()
{
	if (mParams.buildGPUData)
	{
		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
		PX_ASSERT(mMeshData.mGRB_primIndices);

		//copy the original traingle indices to originalTrangles32
		PxMemCopy(mMeshData.mGRB_primIndices, mMeshData.mTriangles, sizeof(IndTri32) *mMeshData.mNbTriangles);


		if (mMeshData.mFaceRemap)
		{
			//We must have discarded some triangles so let's 
			mMeshData.mGRB_faceRemap = PX_NEW(PxU32)[mMeshData.mNbTriangles];
			PxMemCopy(mMeshData.mGRB_faceRemap, mMeshData.mFaceRemap, sizeof(PxU32)*mMeshData.mNbTriangles);
		}

	}
}

void TriangleMeshBuilder::createGRBData()
{

	const PxU32 & numTris = mMeshData.mNbTriangles;

	PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));


	// Core: Mesh data
	///////////////////////////////////////////////////////////////////////////////////

	// (by using adjacency info generated by physx cooker)
	PxVec3 * tempNormalsPerTri_prealloc = reinterpret_cast<PxVec3 *>(PX_ALLOC(numTris * sizeof(PxVec3), PX_DEBUG_EXP("tempNormalsPerTri_prealloc")));

	mMeshData.mGRB_primAdjacencies = PX_ALLOC(sizeof(uint4)*numTris, PX_DEBUG_EXP("GRB_triAdjacencies"));

	
	buildAdjacencies(
		reinterpret_cast<uint4 *>(mMeshData.mGRB_primAdjacencies),
		tempNormalsPerTri_prealloc,
		mMeshData.mVertices,
		reinterpret_cast<uint3*>(mMeshData.mGRB_primIndices),
		numTris
		);
	

	PX_FREE(tempNormalsPerTri_prealloc);

}

void TriangleMeshBuilder::createGRBMidPhaseAndData(const PxU32 originalTriangleCount)
{
	if (mParams.buildGPUData)
	{

		PX_ASSERT(!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

		BV32Tree* bv32Tree = PX_NEW(BV32Tree);
		mMeshData.mGRB_BV32Tree = bv32Tree;

		BV32TriangleMeshBuilder::createMidPhaseStructure(mParams, mMeshData, *bv32Tree);
		
		createGRBData();

		//create a remap table from GPU to CPU remap table
		PxU32* orignalToRemap = PX_NEW(PxU32)[originalTriangleCount];

		PX_ASSERT(mMeshData.mFaceRemap);


		for (PxU32 i = 0; i < mMeshData.mNbTriangles; ++i)
		{
			const PxU32 index = mMeshData.mFaceRemap[i];
			PX_ASSERT(index < originalTriangleCount);
			orignalToRemap[index] = i;
		}


		//map CPU remap triangle index to GPU remap triangle index
		for (PxU32 i = 0; i < mMeshData.mNbTriangles; ++i)
		{
			const PxU32 index = mMeshData.mGRB_faceRemap[i];
			mMeshData.mGRB_faceRemap[i] = orignalToRemap[index];
		}

#if BV32_VALIDATE
		IndTri32* grbTriIndices = reinterpret_cast<IndTri32*>(mMeshData.mGRB_triIndices);
		IndTri32* cpuTriIndices = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);
		//map CPU remap triangle index to GPU remap triangle index
		for (PxU32 i = 0; i < mMeshData.mNbTriangles; ++i)
		{
			PX_ASSERT(grbTriIndices[i].mRef[0] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[0]);
			PX_ASSERT(grbTriIndices[i].mRef[1] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[1]);
			PX_ASSERT(grbTriIndices[i].mRef[2] == cpuTriIndices[mMeshData.mGRB_faceRemap[i]].mRef[2]);
		}
#endif

		if (orignalToRemap)
			PX_DELETE_POD(orignalToRemap);

	}
}

void TriangleMeshBuilder::createEdgeList()
{
	Gu::EDGELISTCREATE create;
	create.NbFaces		= mMeshData.mNbTriangles;
	if(mMeshData.has16BitIndices())
	{
		create.DFaces		= NULL;
		create.WFaces		= reinterpret_cast<PxU16*>(mMeshData.mTriangles);
	}
	else
	{
		create.DFaces		= reinterpret_cast<PxU32*>(mMeshData.mTriangles);
		create.WFaces		= NULL;
	}
	create.FacesToEdges	= true;
	create.EdgesToFaces	= true;
	create.Verts		= mMeshData.mVertices;
	//create.Epsilon = 0.1f;
	//	create.Epsilon		= convexEdgeThreshold;
	edgeList = PX_NEW(Gu::EdgeListBuilder);
	if(!edgeList->init(create))
	{
		PX_DELETE(edgeList);
		edgeList = 0;
	}
}

void TriangleMeshBuilder::releaseEdgeList()
{
	PX_DELETE_AND_RESET(edgeList);
}

//
// When suppressTriangleMeshRemapTable is true, the face remap table is not created.  This saves a significant amount of memory,
// but the SDK will not be able to provide information about which mesh triangle is hit in collisions, sweeps or raycasts hits.
//
// The sequence is as follows:

bool TriangleMeshBuilder::loadFromDesc(const PxTriangleMeshDesc& _desc, PxTriangleMeshCookingResult::Enum* condition, bool validateMesh)
{
	const PxU32 originalTriangleCount = _desc.triangles.count;
	if(!_desc.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "TriangleMesh::loadFromDesc: desc.isValid() failed!");
		return false;
	}

	// verify the mesh params
	if(!mParams.midphaseDesc.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "TriangleMesh::loadFromDesc: mParams.midphaseDesc.isValid() failed!");
		return false;
	}

	// Create a local copy that we can modify
	PxTriangleMeshDesc desc = _desc;

	// Save simple params
	{
		// Handle implicit topology
		PxU32* topology = NULL;
		if(!desc.triangles.data)
		{
			// We'll create 32-bit indices
			desc.flags &= ~PxMeshFlag::e16_BIT_INDICES;
			desc.triangles.stride = sizeof(PxU32)*3;

			{
			// Non-indexed mesh => create implicit topology
			desc.triangles.count = desc.points.count/3;
			// Create default implicit topology
			topology = PX_NEW_TEMP(PxU32)[desc.points.count];
			for(PxU32 i=0;i<desc.points.count;i++)
				topology[i] = i;
			desc.triangles.data = topology;
			}
		}
		// Continue as usual using our new descriptor

		// Convert and clean the input mesh
		if (!importMesh(desc, mParams, condition, validateMesh))
			return false;

		// Cleanup if needed
		PX_DELETE_POD(topology);
	}


	//copy the original triangle indices to grb triangle indices if buildGRBData is true
	recordTriangleIndices();

	createMidPhaseStructure();

	// Compute local bounds
	computeLocalBounds(mMeshData); // AP scaffold: local bounds are already computed in builder.createRTree efficiently with SIMD

	createSharedEdgeData(mParams.buildTriangleAdjacencies, !(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE));

	createGRBMidPhaseAndData(originalTriangleCount);


	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TriangleMeshBuilder::save(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params) const
{
	// Export header
	if(!writeHeader('M', 'E', 'S', 'H', PX_MESH_VERSION, platformMismatch, stream))
		return false;

	// Export midphase ID
	writeDword(getMidphaseID(), platformMismatch, stream);

	// Export serialization flags
	PxU32 serialFlags = 0;
	if(mMeshData.mMaterialIndices)	serialFlags |= Gu::IMSF_MATERIALS;
	if(mMeshData.mFaceRemap)		serialFlags |= Gu::IMSF_FACE_REMAP;
	if(mMeshData.mAdjacencies)		serialFlags |= Gu::IMSF_ADJACENCIES;
	if (params.buildGPUData)		serialFlags |= Gu::IMSF_GRB_DATA;
	// Compute serialization flags for indices
	PxU32 maxIndex=0;
	const Gu::TriangleT<PxU32>* tris = reinterpret_cast<const Gu::TriangleT<PxU32>*>(mMeshData.mTriangles);
	for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
	{
		if(tris[i].v[0]>maxIndex)	maxIndex = tris[i].v[0];
		if(tris[i].v[1]>maxIndex)	maxIndex = tris[i].v[1];
		if(tris[i].v[2]>maxIndex)	maxIndex = tris[i].v[2];
	}

	bool force32 = (params.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES);
	if (maxIndex <= 0xFFFF && !force32)
		serialFlags |= (maxIndex <= 0xFF ? Gu::IMSF_8BIT_INDICES : Gu::IMSF_16BIT_INDICES);
	writeDword(serialFlags, platformMismatch, stream);

	// Export mesh
	writeDword(mMeshData.mNbVertices, platformMismatch, stream);
	writeDword(mMeshData.mNbTriangles, platformMismatch, stream);
	writeFloatBuffer(&mMeshData.mVertices->x, mMeshData.mNbVertices*3, platformMismatch, stream);
	if(serialFlags & Gu::IMSF_8BIT_INDICES)
	{
		const PxU32* indices = tris->v;
		for(PxU32 i=0;i<mMeshData.mNbTriangles*3;i++)
		{
			PxI8 data = PxI8(indices[i]);		
			stream.write(&data, sizeof(PxU8));	
		}
	}
	else if(serialFlags & Gu::IMSF_16BIT_INDICES)
	{
		const PxU32* indices = tris->v;
		for(PxU32 i=0;i<mMeshData.mNbTriangles*3;i++)
			writeWord(Ps::to16(indices[i]), platformMismatch, stream);
	}
	else
		writeIntBuffer(tris->v, mMeshData.mNbTriangles*3, platformMismatch, stream);

	if(mMeshData.mMaterialIndices)
		writeWordBuffer(mMeshData.mMaterialIndices, mMeshData.mNbTriangles, platformMismatch, stream);

	if(mMeshData.mFaceRemap)
	{
		PxU32 maxId = computeMaxIndex(mMeshData.mFaceRemap, mMeshData.mNbTriangles);
		writeDword(maxId, platformMismatch, stream);
		storeIndices(maxId, mMeshData.mNbTriangles, mMeshData.mFaceRemap, stream, platformMismatch);
//		writeIntBuffer(mMeshData.mFaceRemap, mMeshData.mNbTriangles, platformMismatch, stream);
	}

	if(mMeshData.mAdjacencies)
		writeIntBuffer(mMeshData.mAdjacencies, mMeshData.mNbTriangles*3, platformMismatch, stream);

	// Export midphase structure
	saveMidPhaseStructure(stream, platformMismatch);

	// Export local bounds
	writeFloat(mMeshData.mGeomEpsilon, platformMismatch, stream);

	writeFloat(mMeshData.mAABB.minimum.x, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.minimum.y, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.minimum.z, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.x, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.y, platformMismatch, stream);
	writeFloat(mMeshData.mAABB.maximum.z, platformMismatch, stream);

	if(mMeshData.mExtraTrigData)
	{
		writeDword(mMeshData.mNbTriangles, platformMismatch, stream);
		// No need to convert those bytes
		stream.write(mMeshData.mExtraTrigData, mMeshData.mNbTriangles*sizeof(PxU8));
	}
	else
		writeDword(0, platformMismatch, stream);

	// GRB write -----------------------------------------------------------------
	if (params.buildGPUData)
	{
		const PxU32* indices = reinterpret_cast<PxU32*>(mMeshData.mGRB_primIndices);
		if (serialFlags & Gu::IMSF_8BIT_INDICES)
		{
			for (PxU32 i = 0; i<mMeshData.mNbTriangles * 3; i++)
			{
				PxI8 data = PxI8(indices[i]);
				stream.write(&data, sizeof(PxU8));
			}
		}
		else if (serialFlags & Gu::IMSF_16BIT_INDICES)
		{
			for (PxU32 i = 0; i<mMeshData.mNbTriangles * 3; i++)
				writeWord(Ps::to16(indices[i]), platformMismatch, stream);
		}
		else
			writeIntBuffer(indices, mMeshData.mNbTriangles * 3, platformMismatch, stream);


		//writeIntBuffer(reinterpret_cast<PxU32*>(mMeshData.mGRB_triIndices), , mMeshData.mNbTriangles*3, platformMismatch, stream);

		//writeIntBuffer(reinterpret_cast<PxU32 *>(mMeshData.mGRB_triIndices), mMeshData.mNbTriangles*4, platformMismatch, stream);

		writeIntBuffer(reinterpret_cast<PxU32 *>(mMeshData.mGRB_primAdjacencies), mMeshData.mNbTriangles*4, platformMismatch, stream);
		writeIntBuffer(mMeshData.mGRB_faceRemap, mMeshData.mNbTriangles, platformMismatch, stream);

		//Export GPU midphase structure
		BV32Tree* bv32Tree = reinterpret_cast<BV32Tree*>(mMeshData.mGRB_BV32Tree);
		BV32TriangleMeshBuilder::saveMidPhaseStructure(bv32Tree, stream, platformMismatch);
	}

	// End of GRB write ----------------------------------------------------------

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4996)	// permitting use of gatherStrided until we have a replacement.
#endif

bool TriangleMeshBuilder::importMesh(const PxTriangleMeshDesc& desc,const PxCookingParams& params,PxTriangleMeshCookingResult::Enum* condition, bool validate)
{
	//convert and clean the input mesh
	//this is where the mesh data gets copied from user mem to our mem

	PxVec3* verts = mMeshData.allocateVertices(desc.points.count);
	Gu::TriangleT<PxU32>* tris = reinterpret_cast<Gu::TriangleT<PxU32>*>(mMeshData.allocateTriangles(desc.triangles.count, true, PxU32(params.buildGPUData)));

	//copy, and compact to get rid of strides:
	Cooking::gatherStrided(desc.points.data, verts, mMeshData.mNbVertices, sizeof(PxVec3), desc.points.stride);

#if PX_CHECKED
	// PT: check all input vertices are valid
	for(PxU32 i=0;i<desc.points.count;i++)
	{
		const PxVec3& p = verts[i];
		if(!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "input mesh contains corrupted vertex data");
			return false;
		}
	}
#endif

	//for trigs index stride conversion and eventual reordering is also needed, I don't think flexicopy can do that for us.

	Gu::TriangleT<PxU32>* dest = tris;
	const Gu::TriangleT<PxU32>* pastLastDest = tris + mMeshData.mNbTriangles;
	const PxU8* source = reinterpret_cast<const PxU8*>(desc.triangles.data);

	//4 combos of 16 vs 32 and flip vs no flip
	PxU32 c = (desc.flags & PxMeshFlag::eFLIPNORMALS)?PxU32(1):0;
	if (desc.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		//index stride conversion is also needed, I don't think flexicopy can do that for us.
		while (dest < pastLastDest)
		{
			const PxU16 * trig16 = reinterpret_cast<const PxU16*>(source);
			dest->v[0] = trig16[0];
			dest->v[1] = trig16[1+c];
			dest->v[2] = trig16[2-c];
			dest ++;
			source += desc.triangles.stride;
		}
	}
	else
	{
		while (dest < pastLastDest)
		{
			const PxU32 * trig32 = reinterpret_cast<const PxU32*>(source);
			dest->v[0] = trig32[0];
			dest->v[1] = trig32[1+c];
			dest->v[2] = trig32[2-c];
			dest ++;
			source += desc.triangles.stride;
		}
	}

	//copy the material index list if any:
	if(desc.materialIndices.data)
	{
		PxMaterialTableIndex* materials = mMeshData.allocateMaterials();
		Cooking::gatherStrided(desc.materialIndices.data, materials, mMeshData.mNbTriangles, sizeof(PxMaterialTableIndex), desc.materialIndices.stride);

		// Check material indices
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)	PX_ASSERT(materials[i]!=0xffff);
	}

	// Clean the mesh using ICE's MeshBuilder
	// This fixes the bug in ConvexTest06 where the inertia tensor computation fails for a mesh => it works with a clean mesh

	if (!(params.meshPreprocessParams & PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH) || validate)
	{
		if(!cleanMesh(validate, condition))
		{
			if(!validate)
				Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "cleaning the mesh failed");
			return false;
		}
	}
	else
	{
		// we need to fill the remap table if no cleaning was done
		if(params.suppressTriangleMeshRemapTable == false)
		{
			PX_ASSERT(mMeshData.mFaceRemap == NULL);
			mMeshData.mFaceRemap = PX_NEW(PxU32)[mMeshData.mNbTriangles];
			for (PxU32 i = 0; i < mMeshData.mNbTriangles; i++)
				mMeshData.mFaceRemap[i] = i;
		}
	}
	return true;
}

#if PX_VC
#pragma warning(pop)
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TriangleMeshBuilder::checkMeshIndicesSize()
{
	Gu::TriangleMeshData& m = mMeshData;

	// check if we can change indices from 32bits to 16bits
	if(m.mNbVertices <= 0xffff && !m.has16BitIndices())
	{
		const PxU32 numTriangles = m.mNbTriangles;
		PxU32* PX_RESTRICT indices32 = reinterpret_cast<PxU32*> (m.mTriangles);
		PxU32* PX_RESTRICT grbIndices32 = reinterpret_cast<PxU32*>(m.mGRB_primIndices);
		
		m.mTriangles = 0;					// force a realloc
		m.allocateTriangles(numTriangles, false, grbIndices32 != NULL ? 1u : 0u);
		PX_ASSERT(m.has16BitIndices());		// realloc'ing without the force32bit flag changed it.

		PxU16* PX_RESTRICT indices16 = reinterpret_cast<PxU16*> (m.mTriangles);
		for (PxU32 i = 0; i < numTriangles * 3; i++)
			indices16[i] = Ps::to16(indices32[i]);

		PX_FREE(indices32);

		if (grbIndices32)
		{
			PxU16* PX_RESTRICT grbIndices16 = reinterpret_cast<PxU16*> (m.mGRB_primIndices);
			for (PxU32 i = 0; i < numTriangles * 3; i++)
				grbIndices16[i] = Ps::to16(grbIndices32[i]);
		}

		PX_FREE(grbIndices32);

		onMeshIndexFormatChange();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BV4TriangleMeshBuilder::BV4TriangleMeshBuilder(const PxCookingParams& params) : TriangleMeshBuilder(mData, params)
{
}

BV4TriangleMeshBuilder::~BV4TriangleMeshBuilder()
{
}

void BV4TriangleMeshBuilder::onMeshIndexFormatChange()
{
	IndTri32* triangles32 = NULL;
	IndTri16* triangles16 = NULL;
	if(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
		triangles16 = reinterpret_cast<IndTri16*>(mMeshData.mTriangles);
	else
		triangles32 = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);

	mData.mMeshInterface.setPointers(triangles32, triangles16, mMeshData.mVertices);
}

void BV4TriangleMeshBuilder::createMidPhaseStructure()
{
	const float gBoxEpsilon = 2e-4f;
//	const float gBoxEpsilon = 0.1f;
	mData.mMeshInterface.initRemap();
	mData.mMeshInterface.setNbVertices(mMeshData.mNbVertices);
	mData.mMeshInterface.setNbTriangles(mMeshData.mNbTriangles);

	IndTri32* triangles32 = NULL;
	IndTri16* triangles16 = NULL;
	if (mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
	{
		triangles16 = reinterpret_cast<IndTri16*>(mMeshData.mTriangles);
	}
	else
	{
		triangles32 = reinterpret_cast<IndTri32*>(mMeshData.mTriangles);
	}

	mData.mMeshInterface.setPointers(triangles32, triangles16, mMeshData.mVertices);

	const PxU32 nbTrisPerLeaf = (mParams.midphaseDesc.getType() == PxMeshMidPhase::eBVH34) ? mParams.midphaseDesc.mBVH34Desc.numPrimsPerLeaf : 4;

	if(!BuildBV4Ex(mData.mBV4Tree, mData.mMeshInterface, gBoxEpsilon, nbTrisPerLeaf))
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "BV4 tree failed to build.");
		return;
	}

//	remapTopology(mData.mMeshInterface);

	const PxU32* order = mData.mMeshInterface.getRemap();
	if(mMeshData.mMaterialIndices)
	{
		PxMaterialTableIndex* newMat = PX_NEW(PxMaterialTableIndex)[mMeshData.mNbTriangles];
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMat[i] = mMeshData.mMaterialIndices[order[i]];
		PX_DELETE_POD(mMeshData.mMaterialIndices);
		mMeshData.mMaterialIndices = newMat;
	}

	if (!mParams.suppressTriangleMeshRemapTable || mParams.buildGPUData)
	{
		PxU32* newMap = PX_NEW(PxU32)[mMeshData.mNbTriangles];
		for(PxU32 i=0;i<mMeshData.mNbTriangles;i++)
			newMap[i] = mMeshData.mFaceRemap ? mMeshData.mFaceRemap[order[i]] : order[i];
		PX_DELETE_POD(mMeshData.mFaceRemap);
		mMeshData.mFaceRemap = newMap;
	}
	mData.mMeshInterface.releaseRemap();
}

void BV4TriangleMeshBuilder::saveMidPhaseStructure(PxOutputStream& stream, bool mismatch) const
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv4StructureVersion = 3;

	writeChunk('B', 'V', '4', ' ', stream);
	writeDword(bv4StructureVersion, mismatch, stream);

	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(mData.mBV4Tree.mLocalBounds.mExtentsMagnitude, mismatch, stream);

	writeDword(mData.mBV4Tree.mInitData, mismatch, stream);

	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mCenterOrMinCoeff.z, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.x, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.y, mismatch, stream);
	writeFloat(mData.mBV4Tree.mExtentsOrMaxCoeff.z, mismatch, stream);

	// PT: version 3
	writeDword(PxU32(mData.mBV4Tree.mQuantized), mismatch, stream);

	writeDword(mData.mBV4Tree.mNbNodes, mismatch, stream);

#ifdef GU_BV4_USE_SLABS
	// PT: we use BVDataPacked to get the size computation right, but we're dealing with BVDataSwizzled here!
	#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
	const PxU32 NodeSize = mData.mBV4Tree.mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
	#else
	const PxU32 NodeSize = sizeof(BVDataPackedQ);
	#endif
	stream.write(mData.mBV4Tree.mNodes, NodeSize*mData.mBV4Tree.mNbNodes);
	PX_ASSERT(!mismatch);
#else
	#error	Not implemented
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BV32TriangleMeshBuilder::createMidPhaseStructure(const PxCookingParams& params, Gu::TriangleMeshData&	meshData, Gu::BV32Tree& bv32Tree)
{
	const float gBoxEpsilon = 2e-4f;

	Gu::SourceMesh	meshInterface;
	//	const float gBoxEpsilon = 0.1f;
	meshInterface.initRemap();
	meshInterface.setNbVertices(meshData.mNbVertices);
	meshInterface.setNbTriangles(meshData.mNbTriangles);

	PX_ASSERT(!(meshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

	IndTri32* triangles32 = reinterpret_cast<IndTri32*>(meshData.mGRB_primIndices);

	meshInterface.setPointers(triangles32, NULL, meshData.mVertices);

	PxU32 nbTrisPerLeaf = 32;

	if (!BuildBV32Ex(bv32Tree, meshInterface, gBoxEpsilon, nbTrisPerLeaf))
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "BV32 tree failed to build.");
		return;
	}

	const PxU32* order = meshInterface.getRemap();

	if (!params.suppressTriangleMeshRemapTable || params.buildGPUData)
	{
		PxU32* newMap = PX_NEW(PxU32)[meshData.mNbTriangles];
		for (PxU32 i = 0; i<meshData.mNbTriangles; i++)
			newMap[i] = meshData.mGRB_faceRemap ? meshData.mGRB_faceRemap[order[i]] : order[i];
		PX_DELETE_POD(meshData.mGRB_faceRemap);
		meshData.mGRB_faceRemap = newMap;
	}
	
	meshInterface.releaseRemap();

}

void BV32TriangleMeshBuilder::saveMidPhaseStructure(Gu::BV32Tree* bv32Tree, PxOutputStream& stream, bool mismatch)
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv32StructureVersion = 2;

	writeChunk('B', 'V', '3', '2', stream);
	writeDword(bv32StructureVersion, mismatch, stream);

	writeFloat(bv32Tree->mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mExtentsMagnitude, mismatch, stream);

	writeDword(bv32Tree->mInitData, mismatch, stream);

	writeDword(bv32Tree->mNbPackedNodes, mismatch, stream);

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);
	for (PxU32 i = 0; i < bv32Tree->mNbPackedNodes; ++i)
	{
		BV32DataPacked& node = bv32Tree->mPackedNodes[i];

		const PxU32 nbElements = node.mNbNodes * 4;
		writeDword(node.mNbNodes, mismatch, stream);
		WriteDwordBuffer(node.mData, node.mNbNodes, mismatch, stream);
		writeFloatBuffer(&node.mCenter[0].x, nbElements, mismatch, stream);
		writeFloatBuffer(&node.mExtents[0].x, nbElements, mismatch, stream);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RTreeTriangleMeshBuilder::RTreeTriangleMeshBuilder(const PxCookingParams& params) : TriangleMeshBuilder(mData, params)
{
}

RTreeTriangleMeshBuilder::~RTreeTriangleMeshBuilder()
{
}

struct RTreeCookerRemap : RTreeCooker::RemapCallback 
{
	PxU32 mNbTris;
	RTreeCookerRemap(PxU32 numTris) : mNbTris(numTris)
	{
	}

	virtual void remap(PxU32* val, PxU32 start, PxU32 leafCount)
	{
		PX_ASSERT(leafCount > 0);
		PX_ASSERT(leafCount <= 16); // sanity check
		PX_ASSERT(start < mNbTris);
		PX_ASSERT(start+leafCount <= mNbTris);
		PX_ASSERT(val);
		LeafTriangles lt;
		// here we remap from ordered leaf index in the rtree to index in post-remap in triangles
		// this post-remap will happen later
		lt.SetData(leafCount, start);
		*val = lt.Data;
	}
};

void RTreeTriangleMeshBuilder::createMidPhaseStructure()
{
	const PxReal meshSizePerformanceTradeOff = mParams.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff;
	const PxMeshCookingHint::Enum meshCookingHint = mParams.midphaseDesc.mBVH33Desc.meshCookingHint;

	Array<PxU32> resultPermute;
	RTreeCookerRemap rc(mMeshData.mNbTriangles);
	RTreeCooker::buildFromTriangles(
		mData.mRTree,
		mMeshData.mVertices, mMeshData.mNbVertices,
		(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? reinterpret_cast<PxU16*>(mMeshData.mTriangles) : NULL,
		!(mMeshData.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? reinterpret_cast<PxU32*>(mMeshData.mTriangles) : NULL,
		mMeshData.mNbTriangles, resultPermute, &rc, meshSizePerformanceTradeOff, meshCookingHint);

	PX_ASSERT(resultPermute.size() == mMeshData.mNbTriangles);

	remapTopology(resultPermute.begin());
}

void RTreeTriangleMeshBuilder::saveMidPhaseStructure(PxOutputStream& stream, bool mismatch) const
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 rtreeStructureVersion = 2;

	// save the RTree root structure followed immediately by RTreePage pages to an output stream
	writeChunk('R', 'T', 'R', 'E', stream);

	writeDword(rtreeStructureVersion, mismatch, stream);
	const RTree& d = mData.mRTree;
	writeFloatBuffer(&d.mBoundsMin.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mBoundsMax.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mInvDiagonal.x, 4, mismatch, stream);
	writeFloatBuffer(&d.mDiagonalScaler.x, 4, mismatch, stream);
	writeDword(d.mPageSize, mismatch, stream);
	writeDword(d.mNumRootPages, mismatch, stream);
	writeDword(d.mNumLevels, mismatch, stream);
	writeDword(d.mTotalNodes, mismatch, stream);
	writeDword(d.mTotalPages, mismatch, stream);
	PxU32 unused = 0; writeDword(unused, mismatch, stream); // backwards compatibility
	for (PxU32 j = 0; j < d.mTotalPages; j++)
	{
		writeFloatBuffer(d.mPages[j].minx, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].miny, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].minz, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxx, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxy, RTREE_N, mismatch, stream);
		writeFloatBuffer(d.mPages[j].maxz, RTREE_N, mismatch, stream);
		WriteDwordBuffer(d.mPages[j].ptrs, RTREE_N, mismatch, stream);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
