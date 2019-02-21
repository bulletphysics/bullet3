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


#ifndef PX_COLLISION_GRBTRIANGLEMESHCOOKING
#define PX_COLLISION_GRBTRIANGLEMESHCOOKING

#include "GuMeshData.h"
#include "cooking/PxCooking.h"

namespace physx
{
	namespace Gu
	{

	}

// TODO avoroshilov: remove duplicate definitions
static const PxU32 BOUNDARY = 0xffffffff;
static const PxU32 NONCONVEX_FLAG = 0x80000000;

struct EdgeTriLookup
{
	PxU32 edgeId0, edgeId1;
	PxU32 triId;

	bool operator < (const EdgeTriLookup& edge1) const
	{
		return edgeId0 < edge1.edgeId0 || (edgeId0 == edge1.edgeId0 && edgeId1 < edge1.edgeId1);
	}

	bool operator <=(const EdgeTriLookup& edge1) const
	{
		return edgeId0 < edge1.edgeId0 || (edgeId0 == edge1.edgeId0 && edgeId1 <= edge1.edgeId1);
	}
};


static PxU32 binarySearch(const EdgeTriLookup* __restrict data, const PxU32 numElements, const EdgeTriLookup& value)
{
	PxU32 left = 0;
	PxU32 right = numElements;

	while ((right - left) > 1)
	{
		PxU32 pos = (left + right) / 2;
		const EdgeTriLookup& element = data[pos];
		if (element <= value)
		{
			left = pos;
		}
		else
		{
			right = pos;
		}
	}

	return left;
}

// slightly different behavior from collide2: boundary edges are filtered out

static PxU32 findAdjacent(const PxVec3* triVertices, const PxVec3* triNormals, const uint3* triIndices, 
	PxU32 nbTris, PxU32 i0, PxU32 i1, const PxPlane& plane,
	EdgeTriLookup* triLookups, PxU32 triangleIndex)
{
	PxU32 result = BOUNDARY;
	PxReal bestCos = -FLT_MAX;

	EdgeTriLookup lookup;
	lookup.edgeId0 = PxMin(i0, i1);
	lookup.edgeId1 = PxMax(i0, i1);

	PxU32 startIndex = binarySearch(triLookups, nbTris * 3, lookup);

	for (PxU32 a = startIndex; a > 0; --a)
	{
		if (triLookups[a - 1].edgeId0 == lookup.edgeId0 && triLookups[a - 1].edgeId1 == lookup.edgeId1)
			startIndex = a - 1;
		else
			break;
	}

	for (PxU32 a = startIndex; a < nbTris * 3; ++a)
	{
		EdgeTriLookup& edgeTri = triLookups[a];

		if (edgeTri.edgeId0 != lookup.edgeId0 || edgeTri.edgeId1 != lookup.edgeId1)
			break;

		if (edgeTri.triId == triangleIndex)
			continue;

		const uint3& triIdx = triIndices[edgeTri.triId];
		PxU32 vIdx0 = triIdx.x;
		PxU32 vIdx1 = triIdx.y;
		PxU32 vIdx2 = triIdx.z;

		PxU32 other = vIdx0 + vIdx1 + vIdx2 - (i0 + i1);

		if (plane.distance(triVertices[other]) >= 0)
			return NONCONVEX_FLAG | edgeTri.triId;

		PxReal c = plane.n.dot(triNormals[edgeTri.triId]);
		if (c>bestCos)
		{
			bestCos = c;
			result = edgeTri.triId;
		}

	}

	return result;
}


static void buildAdjacencies(uint4* triAdjacencies, PxVec3* tempNormalsPerTri_prealloc, const PxVec3* triVertices, const uint3* triIndices, PxU32 nbTris)
{
	//PxVec3 * triNormals = new PxVec3[nbTris];

	EdgeTriLookup* edgeLookups = reinterpret_cast<EdgeTriLookup*>(PX_ALLOC(sizeof(EdgeTriLookup) * nbTris * 3, PX_DEBUG_EXP("edgeLookups")));
	

	for (PxU32 i = 0; i < nbTris; i++)
	{
		const uint3& triIdx = triIndices[i];
		PxU32 vIdx0 = triIdx.x;
		PxU32 vIdx1 = triIdx.y;
		PxU32 vIdx2 = triIdx.z;

		tempNormalsPerTri_prealloc[i] = (triVertices[vIdx1] - triVertices[vIdx0]).cross(triVertices[vIdx2] - triVertices[vIdx0]).getNormalized();

		edgeLookups[i * 3].edgeId0 = PxMin(vIdx0, vIdx1);
		edgeLookups[i * 3].edgeId1 = PxMax(vIdx0, vIdx1);
		edgeLookups[i * 3].triId = i;

		edgeLookups[i * 3 + 1].edgeId0 = PxMin(vIdx1, vIdx2);
		edgeLookups[i * 3 + 1].edgeId1 = PxMax(vIdx1, vIdx2);
		edgeLookups[i * 3 + 1].triId = i;

		edgeLookups[i * 3 + 2].edgeId0 = PxMin(vIdx0, vIdx2);
		edgeLookups[i * 3 + 2].edgeId1 = PxMax(vIdx0, vIdx2);
		edgeLookups[i * 3 + 2].triId = i;
	}

	Ps::sort<EdgeTriLookup>(edgeLookups, PxU32(nbTris * 3));

	for (PxU32 i = 0; i < nbTris; i++)
	{
		const uint3& triIdx = triIndices[i];
		PxU32 vIdx0 = triIdx.x;
		PxU32 vIdx1 = triIdx.y;
		PxU32 vIdx2 = triIdx.z;

		PxPlane triPlane(triVertices[vIdx0], tempNormalsPerTri_prealloc[i]);
		uint4 triAdjIdx;

		triAdjIdx.x = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx0, vIdx1, triPlane, edgeLookups, i);
		triAdjIdx.y = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx1, vIdx2, triPlane, edgeLookups, i);
		triAdjIdx.z = findAdjacent(triVertices, tempNormalsPerTri_prealloc, triIndices, nbTris, vIdx2, vIdx0, triPlane, edgeLookups, i);
		triAdjIdx.w = 0;

		triAdjacencies[i] = triAdjIdx;
	}
	

	PX_FREE(edgeLookups);
}

static bool isEdgeNonconvex(PxU32 adjEdgeIndex)
{
	return (adjEdgeIndex != BOUNDARY) && (adjEdgeIndex & NONCONVEX_FLAG);
}

PX_INLINE void buildVertexConnection_p1(PxU32 * vertValency, PxU32 * vertNeighborStart, PxU32 & tempNumAdjVertices, const float4 * /*triVertices*/, const uint4 * triIndices, const uint4 * triAdjacencies, PxU32 nbVerts, PxU32 nbTris)
{
	tempNumAdjVertices = 0;
	memset(vertValency, 0, nbVerts*sizeof(PxU32));

	// Calculate max num of adjVerts
	for (PxU32 i = 0; i < nbTris; i++)
	{
		uint4 triIdx = triIndices[i];
		PxU32 vi0 = triIdx.x;
		PxU32 vi1 = triIdx.y;
		PxU32 vi2 = triIdx.z;

		uint4 triAdjIdx = triAdjacencies[i];

		PxU32 totalVertsAdded = 0;
		if (!isEdgeNonconvex(triAdjIdx.x))
		{
			++vertValency[vi0];
			++vertValency[vi1];
			totalVertsAdded += 2;
		}
		if (!isEdgeNonconvex(triAdjIdx.y))
		{
			++vertValency[vi1];
			++vertValency[vi2];
			totalVertsAdded += 2;
		}
		if (!isEdgeNonconvex(triAdjIdx.z))
		{
			++vertValency[vi2];
			++vertValency[vi0];
			totalVertsAdded += 2;
		}
		tempNumAdjVertices += totalVertsAdded;
	}
	PxU32 offset = 0;
	for (PxU32 i = 0; i < nbVerts; i++)
	{
		vertNeighborStart[i] = offset;
		offset += vertValency[i];
	}

	memset(vertValency, 0, nbVerts*sizeof(PxU32));
}

PX_INLINE PxU32 buildVertexConnection_p2(PxU32 * vertValency, PxU32 * vertNeighborStart, PxU32 * vertNeighboringPairs_prealloc, PxU32 tempNumAdjVertices, const float4 * /*triVertices*/, const uint4 * triIndices, const uint4 * triAdjacencies, PxU32 /*nbVerts*/, PxU32 nbTris)
{
	memset(vertNeighboringPairs_prealloc, 0xff, tempNumAdjVertices*2*sizeof(PxU32));

	PxU32 newAdjVertsNum = 0;
	for (PxU32 i = 0; i < nbTris; i++)
	{
		uint4 triIdx = triIndices[i];
		PxU32 vi[3] =
		{
			triIdx.x,
			triIdx.y,
			triIdx.z
		};
		uint4 triAdjIdx = triAdjacencies[i];
		PxU32 ta[3] =
		{
			triAdjIdx.x,
			triAdjIdx.y,
			triAdjIdx.z
		};

		for (int tvi = 0; tvi < 3; ++tvi)
		{
			PxU32 curIdx = vi[tvi];
			PxU32 nextIdx = vi[(tvi+1)%3];
			if (!isEdgeNonconvex(ta[tvi]))
			{
				bool matchFound = false;
				for (PxU32 valIdx = vertNeighborStart[curIdx], valIdxEnd = vertNeighborStart[curIdx] + vertValency[curIdx]; valIdx < valIdxEnd; ++valIdx)
				{
					if (vertNeighboringPairs_prealloc[valIdx*2+1] == nextIdx)
					{
						matchFound = true;
						break;
					}
				}

				if (!matchFound)
				{
					PxU32 curPairIdx;
					
					curPairIdx = vertNeighborStart[curIdx] + vertValency[curIdx];
					vertNeighboringPairs_prealloc[curPairIdx*2+0] = curIdx;
					vertNeighboringPairs_prealloc[curPairIdx*2+1] = nextIdx;
					++vertValency[curIdx];

					curPairIdx = vertNeighborStart[nextIdx] + vertValency[nextIdx];
					vertNeighboringPairs_prealloc[curPairIdx*2+0] = nextIdx;
					vertNeighboringPairs_prealloc[curPairIdx*2+1] = curIdx;
					++vertValency[nextIdx];

					newAdjVertsNum += 2;
				}
			}
		}
	}

	return newAdjVertsNum;
}

PX_INLINE void buildVertexConnection_p3(PxU32 * vertNeighbors, PxU32 * /*vertValency*/, PxU32 * vertNeighborStart, PxU32 * vertNeighboringPairs_prealloc, PxU32 tempNumAdjVertices, PxU32 newNumAdjVertices, const float4 * /*triVertices*/, const uint4 * /*triIndices*/, const uint4 * /*triAdjacencies*/, PxU32 /*nbVerts*/, PxU32 /*nbTris*/)
{
	PX_UNUSED(newNumAdjVertices);
	PxU32 prevVertex = 0xFFffFFff;
	PxU32 writingIndex = 0;
	for (PxU32 i = 0; i < tempNumAdjVertices; ++i)
	{
		PxU32 curPairIdx0 = vertNeighboringPairs_prealloc[i*2+0];
		if (curPairIdx0 == 0xFFffFFff)
		{
			continue;
		}

		PxU32 curPairIdx1 = vertNeighboringPairs_prealloc[i*2+1];
		vertNeighbors[writingIndex] = curPairIdx1;
		if (curPairIdx0 != prevVertex)
		{
			vertNeighborStart[curPairIdx0] = writingIndex;
		}
		prevVertex = curPairIdx0;

		++writingIndex;
	}

	PX_ASSERT(writingIndex == newNumAdjVertices);
}

} // namespace physx

#endif
