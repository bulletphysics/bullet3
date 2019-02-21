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


#include "PsUserAllocated.h"
#include "PsUtilities.h"
#include "PsMathUtils.h"
#include "PsVecMath.h"

#include "PxCooking.h"

#include "GuConvexMeshData.h"
#include "GuBigConvexData2.h"
#include "GuIntersectionRayPlane.h"
#include "GuSerialize.h"

#include "BigConvexDataBuilder.h"
#include "EdgeList.h"

#include "ConvexHullBuilder.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace physx;
using namespace Gu;
using namespace Ps::aos;

static const PxU32 gSupportVersion = 0;
static const PxU32 gVersion = 0;

BigConvexDataBuilder::BigConvexDataBuilder(const Gu::ConvexHullData* hull, BigConvexData* gm, const PxVec3* hullVerts) : mHullVerts(hullVerts)
{
	mSVM = gm;
	mHull = hull;
}

BigConvexDataBuilder::~BigConvexDataBuilder()
{
}

bool BigConvexDataBuilder::initialize()
{
	mSVM->mData.mSamples = PX_NEW(PxU8)[mSVM->mData.mNbSamples*2u];

#if PX_DEBUG
//	printf("SVM: %d bytes\n", mNbSamples*sizeof(PxU8)*2);
#endif

	return true;
}

bool BigConvexDataBuilder::save(PxOutputStream& stream, bool platformMismatch) const
{
	// Export header
	if(!WriteHeader('S', 'U', 'P', 'M', gSupportVersion, platformMismatch, stream))
		return false;

	// Save base gaussmap
//	if(!GaussMapBuilder::Save(stream, platformMismatch))	return false;
		// Export header
		if(!WriteHeader('G', 'A', 'U', 'S', gVersion, platformMismatch, stream))
			return false;

		// Export basic info
	//	stream.StoreDword(mSubdiv);
		writeDword(mSVM->mData.mSubdiv, platformMismatch, stream);		// PT: could now write Word here
	//	stream.StoreDword(mNbSamples);
		writeDword(mSVM->mData.mNbSamples, platformMismatch, stream);	// PT: could now write Word here

	// Save map data
	// It's an array of bytes so we don't care about 'PlatformMismatch'
	stream.write(mSVM->mData.mSamples, sizeof(PxU8)*mSVM->mData.mNbSamples*2);

	if(!saveValencies(stream, platformMismatch))	
		return false;

	return true;
}

//////////////////////////////////////////////////////////////////////////
// compute valencies for each vertex
// we dont compute the edges again here, we have them temporary stored in mHullDataFacesByAllEdges8 structure
bool BigConvexDataBuilder::computeValencies(const ConvexHullBuilder& meshBuilder)
{	
	// Create valencies
	const PxU32 numVertices = meshBuilder.mHull->mNbHullVertices;
	mSVM->mData.mNbVerts = numVertices;

	// Get ram for valencies and adjacent verts
	const PxU32 numAlignedVerts = (numVertices+3)&~3;
	const PxU32 TotalSize = sizeof(Gu::Valency)*numAlignedVerts + sizeof(PxU8)*meshBuilder.mHull->mNbEdges*2u;
	mSVM->mVBuffer = PX_ALLOC(TotalSize, "BigConvexData data");
	mSVM->mData.mValencies		= reinterpret_cast<Gu::Valency*>(mSVM->mVBuffer);
	mSVM->mData.mAdjacentVerts	= (reinterpret_cast<PxU8*>(mSVM->mVBuffer)) + sizeof(Gu::Valency)*numAlignedVerts;

	PxMemZero(mSVM->mData.mValencies, numVertices*sizeof(Gu::Valency));
	PxU8 vertexMarker[256];
	PxMemZero(vertexMarker,numVertices);

	// Compute valencies
	for (PxU32 i = 0; i < meshBuilder.mHull->mNbPolygons; i++)
	{
		const PxU32 numVerts = meshBuilder.mHullDataPolygons[i].mNbVerts;
		const PxU8* Data = meshBuilder.mHullDataVertexData8 + meshBuilder.mHullDataPolygons[i].mVRef8;
		for (PxU32 j = 0; j < numVerts; j++)
		{			
			mSVM->mData.mValencies[Data[j]].mCount++;
			PX_ASSERT(mSVM->mData.mValencies[Data[j]].mCount != 0xffff);
		}
	}

	// Create offsets
	mSVM->CreateOffsets();

	//		mNbAdjVerts = mOffsets[mNbVerts-1] + mValencies[mNbVerts-1];
	mSVM->mData.mNbAdjVerts = PxU32(mSVM->mData.mValencies[mSVM->mData.mNbVerts - 1].mOffset + mSVM->mData.mValencies[mSVM->mData.mNbVerts - 1].mCount);
	PX_ASSERT(mSVM->mData.mNbAdjVerts == PxU32(meshBuilder.mHull->mNbEdges * 2));

	// Create adjacent vertices	
	// parse the polygons and its vertices
	for (PxU32 i = 0; i < meshBuilder.mHull->mNbPolygons; i++)
	{
		PxU32 numVerts = meshBuilder.mHullDataPolygons[i].mNbVerts;
		const PxU8* Data = meshBuilder.mHullDataVertexData8 + meshBuilder.mHullDataPolygons[i].mVRef8;
		for (PxU32 j = 0; j < numVerts; j++)
		{
			const PxU8 vertexIndex = Data[j];
			PxU8 numAdj = 0;
			// if we did not parsed this vertex, traverse to the adjacent face and then 
			// again to next till we hit back the original polygon
			if(vertexMarker[vertexIndex] == 0)
			{
				PxU8 prevIndex = Data[(j+1)%numVerts];
				mSVM->mData.mAdjacentVerts[mSVM->mData.mValencies[vertexIndex].mOffset++] = prevIndex;
				numAdj++;
				// now traverse the neighbors	
				const PxU16 edgeIndex = PxU16(meshBuilder.mEdgeData16[meshBuilder.mHullDataPolygons[i].mVRef8 + j]*2);
				PxU8 n0 = meshBuilder.mHullDataFacesByEdges8[edgeIndex];
				PxU8 n1 = meshBuilder.mHullDataFacesByEdges8[edgeIndex + 1];
				
				PxU32 neighborPolygon = n0 == i ? n1 : n0;				
				while (neighborPolygon != i)
				{
					PxU32 numNeighborVerts = meshBuilder.mHullDataPolygons[neighborPolygon].mNbVerts;
					const PxU8* neighborData = meshBuilder.mHullDataVertexData8 + meshBuilder.mHullDataPolygons[neighborPolygon].mVRef8;
					PxU32 nextEdgeIndex = 0;
					// search in the neighbor face for the tested vertex
					for (PxU32 k = 0; k < numNeighborVerts; k++)
					{
						// search the vertexIndex
						if(neighborData[k] == vertexIndex)
						{
							const PxU8 nextIndex = neighborData[(k+1)%numNeighborVerts];
							// next index already there, pick the previous
							if(nextIndex == prevIndex)
							{
								prevIndex = k == 0 ? neighborData[numNeighborVerts - 1] : neighborData[k-1];
								nextEdgeIndex = k == 0 ? numNeighborVerts - 1 : k-1;
							}
							else
							{
								prevIndex = nextIndex;
								nextEdgeIndex = k;
							}
							mSVM->mData.mAdjacentVerts[mSVM->mData.mValencies[vertexIndex].mOffset++] = prevIndex;
							numAdj++;
							break;
						}
					}

					// now move to next neighbor
					const PxU16 edgeIndex2 = PxU16(meshBuilder.mEdgeData16[(meshBuilder.mHullDataPolygons[neighborPolygon].mVRef8 + nextEdgeIndex)]*2);
					n0 = meshBuilder.mHullDataFacesByEdges8[edgeIndex2];
					n1 = meshBuilder.mHullDataFacesByEdges8[edgeIndex2 + 1];

					neighborPolygon = n0 == neighborPolygon ? n1 : n0;
				}
				vertexMarker[vertexIndex] = numAdj;
			}
		}
	}

	// Recreate offsets
	mSVM->CreateOffsets();
	return true;
}

//////////////////////////////////////////////////////////////////////////
// compute the min dot product from the verts for given dir
void BigConvexDataBuilder::precomputeSample(const PxVec3& dir, PxU8& startIndex_, float negativeDir)
{
	PxU8 startIndex = startIndex_;
	
	const PxVec3* verts = mHullVerts;
	const Valency* valency = mSVM->mData.mValencies;
	const PxU8* adjacentVerts = mSVM->mData.mAdjacentVerts;

	// we have only 256 verts
	PxU32 smallBitMap[8] = {0,0,0,0,0,0,0,0};	
	
	float minimum = negativeDir * verts[startIndex].dot(dir);
	PxU32 initialIndex = startIndex;	
	do
	{
		initialIndex = startIndex;
		const PxU32 numNeighbours = valency[startIndex].mCount;
		const PxU32 offset = valency[startIndex].mOffset;

		for (PxU32 a = 0; a < numNeighbours; ++a)
		{			
			const PxU8 neighbourIndex = adjacentVerts[offset + a];			
			const float dist = negativeDir * verts[neighbourIndex].dot(dir);
			if (dist < minimum)
			{
				const PxU32 ind = PxU32(neighbourIndex >> 5);
				const PxU32 mask = PxU32(1 << (neighbourIndex & 31));
				if ((smallBitMap[ind] & mask) == 0)
				{
					smallBitMap[ind] |= mask;
					minimum = dist;
					startIndex = neighbourIndex;
				}
			}
		}

	} while (startIndex != initialIndex);	

	startIndex_ = startIndex;
}

//////////////////////////////////////////////////////////////////////////
// Precompute the min/max vertices for cube directions. 
bool BigConvexDataBuilder::precompute(PxU32 subdiv)
{
	mSVM->mData.mSubdiv = Ps::to16(subdiv);
	mSVM->mData.mNbSamples = Ps::to16(6 * subdiv*subdiv);

	if (!initialize())
		return false;

	PxU8 startIndex[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	PxU8 startIndex2[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	const float halfSubdiv = float(subdiv - 1) * 0.5f;
	for (PxU32 j = 0; j < subdiv; j++)
	{
		for (PxU32 i = j; i < subdiv; i++)
		{
			const float iSubDiv = 1.0f - i / halfSubdiv;
			const float jSubDiv = 1.0f - j / halfSubdiv;

			PxVec3 tempDir(1.0f, iSubDiv, jSubDiv);
			// we need to normalize only once, then we permute the components
			// as before for each i,j and j,i face direction
			tempDir.normalize();

			const PxVec3 dirs[12] = {
				PxVec3(-tempDir.x, tempDir.y, tempDir.z),
				PxVec3(tempDir.x, tempDir.y, tempDir.z),

				PxVec3(tempDir.z, -tempDir.x, tempDir.y),
				PxVec3(tempDir.z, tempDir.x, tempDir.y),

				PxVec3(tempDir.y, tempDir.z, -tempDir.x),
				PxVec3(tempDir.y, tempDir.z, tempDir.x),

				PxVec3(-tempDir.x, tempDir.z, tempDir.y),
				PxVec3(tempDir.x, tempDir.z, tempDir.y),

				PxVec3(tempDir.y, -tempDir.x, tempDir.z),
				PxVec3(tempDir.y, tempDir.x, tempDir.z),

				PxVec3(tempDir.z, tempDir.y, -tempDir.x),
				PxVec3(tempDir.z, tempDir.y, tempDir.x)
			};

			// compute in each direction + negative/positive dot, we have
			// then two start indexes, which are used then for hill climbing
			for (PxU32 dStep = 0; dStep < 12; dStep++)
			{
				precomputeSample(dirs[dStep], startIndex[dStep], 1.0f);
				precomputeSample(dirs[dStep], startIndex2[dStep], -1.0f);
			}			

			// decompose the vector results into face directions
			for (PxU32 k = 0; k < 6; k++)
			{
				const PxU32 ksub = k*subdiv*subdiv;
				const PxU32 offset = j + i*subdiv + ksub;
				const PxU32 offset2 = i + j*subdiv + ksub;
				PX_ASSERT(offset < mSVM->mData.mNbSamples);
				PX_ASSERT(offset2 < mSVM->mData.mNbSamples);

				mSVM->mData.mSamples[offset] = startIndex[k];
				mSVM->mData.mSamples[offset + mSVM->mData.mNbSamples] = startIndex2[k];

				mSVM->mData.mSamples[offset2] = startIndex[k + 6];
				mSVM->mData.mSamples[offset2 + mSVM->mData.mNbSamples] = startIndex2[k + 6];
			}
		}
	}
	return true;
}

static const PxU32 gValencyVersion = 2;

//////////////////////////////////////////////////////////////////////////

bool BigConvexDataBuilder::saveValencies(PxOutputStream& stream, bool platformMismatch) const
{
	// Export header
	if(!WriteHeader('V', 'A', 'L', 'E', gValencyVersion, platformMismatch, stream))
		return false;

	writeDword(mSVM->mData.mNbVerts, platformMismatch, stream);
	writeDword(mSVM->mData.mNbAdjVerts, platformMismatch, stream);

	{
		PxU16* temp = PX_NEW_TEMP(PxU16)[mSVM->mData.mNbVerts];
		for(PxU32 i=0;i<mSVM->mData.mNbVerts;i++)
			temp[i] = mSVM->mData.mValencies[i].mCount;

		const PxU32 maxIndex = computeMaxIndex(temp, mSVM->mData.mNbVerts);
		writeDword(maxIndex, platformMismatch, stream);
		StoreIndices(Ps::to16(maxIndex), mSVM->mData.mNbVerts, temp, stream, platformMismatch);

		PX_DELETE_POD(temp);
	}
	stream.write(mSVM->mData.mAdjacentVerts, mSVM->mData.mNbAdjVerts);

	return true;
}
