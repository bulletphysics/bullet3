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


#include "ConvexHullLib.h"
#include "Quantizer.h"
#include "PsAllocator.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMemory.h"

using namespace physx;

namespace local
{
	//////////////////////////////////////////////////////////////////////////
	// constants	
	static const float DISTANCE_EPSILON = 0.000001f;	// close enough to consider two floating point numbers to be 'the same'.
	static const float NORMAL_DISTANCE_EPSILON = 0.001f; // close enough to consider two floating point numbers to be 'the same' in normalized points cloud.
	static const float RESIZE_VALUE = 0.01f;			// if the provided points AABB is very thin resize it to this size

	//////////////////////////////////////////////////////////////////////////
	// checks if points form a valid AABB cube, if not construct a default CUBE
	static bool checkPointsAABBValidity(PxU32 numPoints, const PxVec3* points, PxU32 stride , float distanceEpsilon,
		float resizeValue, PxVec3& center, PxVec3& scale, PxU32& vcount, PxVec3* vertices, bool fCheck = false)
	{
		const char* vtx = reinterpret_cast<const char *> (points);
		PxBounds3 bounds;
		bounds.setEmpty();

		// get the bounding box		
		for (PxU32 i = 0; i < numPoints; i++)
		{
			const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
			vtx += stride;

			bounds.include(p);
		}

		PxVec3 dim = bounds.getDimensions();
		center = bounds.getCenter();

		// special case, the AABB is very thin or user provided us with only input 2 points
		// we construct an AABB cube and return it
		if ( dim.x < distanceEpsilon || dim.y < distanceEpsilon || dim.z < distanceEpsilon || numPoints < 3 )
		{
			float len = FLT_MAX;

			// pick the shortest size bigger than the distance epsilon
			if ( dim.x > distanceEpsilon && dim.x < len ) 
				len = dim.x;
			if ( dim.y > distanceEpsilon && dim.y < len ) 
				len = dim.y;
			if ( dim.z > distanceEpsilon && dim.z < len ) 
				len = dim.z;

			// if the AABB is small in all dimensions, resize it
			if ( len == FLT_MAX )
			{
				dim = PxVec3(resizeValue);
			}
			// if one edge is small, set to 1/5th the shortest non-zero edge.
			else
			{
				if ( dim.x < distanceEpsilon )
					dim.x = len * 0.05f;
				else
					dim.x *= 0.5f;
				if ( dim.y < distanceEpsilon )
					dim.y = len * 0.05f;
				else
					dim.y *= 0.5f;
				if ( dim.z < distanceEpsilon ) 
					dim.z = len * 0.05f;
				else
					dim.z *= 0.5f;
			}

			// construct the AABB
			const PxVec3 extPos = center + dim;
			const PxVec3 extNeg = center - dim;

			if(fCheck)
				vcount = 0;

			vertices[vcount++] = extNeg;			
			vertices[vcount++] = PxVec3(extPos.x,extNeg.y,extNeg.z);
			vertices[vcount++] = PxVec3(extPos.x,extPos.y,extNeg.z);			
			vertices[vcount++] = PxVec3(extNeg.x,extPos.y,extNeg.z);			
			vertices[vcount++] = PxVec3(extNeg.x,extNeg.y,extPos.z);			
			vertices[vcount++] = PxVec3(extPos.x,extNeg.y,extPos.z);			
			vertices[vcount++] = extPos;			
			vertices[vcount++] = PxVec3(extNeg.x,extPos.y,extPos.z);			
			return true; // return cube
		}
		else
		{
			scale = dim;
		}
		return false;
	}

}


//////////////////////////////////////////////////////////////////////////
// shift vertices around origin and normalize point cloud, remove duplicates!
bool ConvexHullLib::shiftAndcleanupVertices(PxU32 svcount, const PxVec3* svertices, PxU32 stride,
	PxU32& vcount, PxVec3* vertices, PxVec3& scale, PxVec3& center)
{
	mShiftedVerts = reinterpret_cast<PxVec3*> (PX_ALLOC_TEMP(sizeof(PxVec3)*svcount, "PxVec3"));	
	const char* vtx = reinterpret_cast<const char *> (svertices);
	PxBounds3 bounds;
	bounds.setEmpty();

	// get the bounding box		
	for (PxU32 i = 0; i < svcount; i++)
	{
		const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
		vtx += stride;

		bounds.include(p);
	}
	mOriginShift = bounds.getCenter();
	vtx = reinterpret_cast<const char *> (svertices);
	for (PxU32 i = 0; i < svcount; i++)
	{
		const PxVec3& p = *reinterpret_cast<const PxVec3 *> (vtx);
		vtx += stride;

		mShiftedVerts[i] = p - mOriginShift;
	}
	return cleanupVertices(svcount, mShiftedVerts, sizeof(PxVec3), vcount, vertices, scale, center);
}

//////////////////////////////////////////////////////////////////////////
// Shift verts/planes in the desc back
void ConvexHullLib::shiftConvexMeshDesc(PxConvexMeshDesc& desc)
{
	PX_ASSERT(mConvexMeshDesc.flags & PxConvexFlag::eSHIFT_VERTICES);

	PxVec3* points = reinterpret_cast<PxVec3*>(const_cast<void*>(desc.points.data));
	for(PxU32 i = 0; i < desc.points.count; i++)
	{
		points[i] = points[i] + mOriginShift;
	}

	PxHullPolygon* polygons = reinterpret_cast<PxHullPolygon*>(const_cast<void*>(desc.polygons.data));
	for(PxU32 i = 0; i < desc.polygons.count; i++)
	{
		polygons[i].mPlane[3] -= PxVec3(polygons[i].mPlane[0], polygons[i].mPlane[1], polygons[i].mPlane[2]).dot(mOriginShift);
	}
}

//////////////////////////////////////////////////////////////////////////
// normalize point cloud, remove duplicates!
bool ConvexHullLib::cleanupVertices(PxU32 svcount, const PxVec3* svertices, PxU32 stride,
	PxU32& vcount, PxVec3* vertices, PxVec3& scale, PxVec3& center)
{
	if (svcount == 0) 
		return false;

	const PxVec3* verticesToClean = svertices;
	PxU32 numVerticesToClean = svcount;
	Quantizer* quantizer = NULL;

	// if quantization is enabled, parse the input vertices and produce new qantized vertices, 
	// that will be then cleaned the same way
	if (mConvexMeshDesc.flags & PxConvexFlag::eQUANTIZE_INPUT)
	{
		quantizer = createQuantizer();
		PxU32 vertsOutCount;
		const PxVec3* vertsOut = quantizer->kmeansQuantize3D(svcount, svertices, stride,true, mConvexMeshDesc.quantizedCount, vertsOutCount);

		if (vertsOut)
		{
			numVerticesToClean = vertsOutCount;
			verticesToClean = vertsOut;
		}		
	}

	const float distanceEpsilon = local::DISTANCE_EPSILON * mCookingParams.scale.length;
	const float resizeValue = local::RESIZE_VALUE * mCookingParams.scale.length;
	const float normalEpsilon = local::NORMAL_DISTANCE_EPSILON; // used to determine if 2 points are the same

	vcount = 0;
	PxVec3 recip;

	scale = PxVec3(1.0f);

	// check for the AABB from points, if its very tiny return a resized CUBE
	if (local::checkPointsAABBValidity(numVerticesToClean, verticesToClean, stride, distanceEpsilon, resizeValue, center, scale, vcount, vertices, false))
	{
		if (quantizer)
			quantizer->release();
		return true;
	}

	recip[0] = 1 / scale[0];
	recip[1] = 1 / scale[1];
	recip[2] = 1 / scale[2];

	center = center.multiply(recip);

	// normalize the point cloud
	const char * vtx = reinterpret_cast<const char *> (verticesToClean);
	for (PxU32 i = 0; i<numVerticesToClean; i++)
	{
		const PxVec3& p = *reinterpret_cast<const PxVec3 *>(vtx);
		vtx+=stride;

		PxVec3 normalizedP = p.multiply(recip); // normalize

		PxU32 j;

		// parse the already stored vertices and check the distance
		for (j=0; j<vcount; j++)
		{
			PxVec3& v = vertices[j];

			const float dx = fabsf(normalizedP[0] - v[0] );
			const float dy = fabsf(normalizedP[1] - v[1] );
			const float dz = fabsf(normalizedP[2] - v[2] );

			if ( dx < normalEpsilon && dy < normalEpsilon && dz < normalEpsilon )
			{
				// ok, it is close enough to the old one
				// now let us see if it is further from the center of the point cloud than the one we already recorded.
				// in which case we keep this one instead.
				const float dist1 = (normalizedP - center).magnitudeSquared();
				const float dist2 = (v - center).magnitudeSquared();

				if ( dist1 > dist2 )
				{
					v = normalizedP;
				}
				break;
			}
		}

		// we dont have that vertex in the output, add it
		if ( j == vcount )
		{
			vertices[vcount] = normalizedP;
			vcount++;
		}
	}

	// scale the verts back
	for (PxU32 i = 0; i < vcount; i++)
	{
		vertices[i] = vertices[i].multiply(scale);
	}

	// ok..now make sure we didn't prune so many vertices it is now invalid.
	// note, that the output vertices are again scaled, we need to scale them back then
	local::checkPointsAABBValidity(vcount, vertices, sizeof(PxVec3), distanceEpsilon, resizeValue, center, scale, vcount, vertices, true);
	
	if (quantizer)
		quantizer->release();
	return true;
}

void ConvexHullLib::swapLargestFace(PxConvexMeshDesc& desc)
{
	const PxHullPolygon* polygons = reinterpret_cast<const PxHullPolygon*>(desc.polygons.data);
	PxHullPolygon* polygonsOut = const_cast<PxHullPolygon*>(polygons);

	PxU32 largestFace = 0;
	for (PxU32 i = 1; i < desc.polygons.count; i++)
	{		
		if(polygons[largestFace].mNbVerts < polygons[i].mNbVerts)
			largestFace = i;
	}

	// early exit if no swap needs to be done
	if(largestFace == 0)
		return;

	const PxU32* indices = reinterpret_cast<const PxU32*>(desc.indices.data);
	mSwappedIndices = reinterpret_cast<PxU32*> (PX_ALLOC_TEMP(sizeof(PxU32)*desc.indices.count, "PxU32"));	

	PxHullPolygon replacedPolygon = polygons[0];
	PxHullPolygon largestPolygon = polygons[largestFace];
	polygonsOut[0] = polygons[largestFace];
	polygonsOut[largestFace] = replacedPolygon;

	// relocate indices
	PxU16 indexBase = 0;
	for (PxU32 i = 0; i < desc.polygons.count; i++)
	{
		if(i == 0)
		{
			PxMemCopy(mSwappedIndices, &indices[largestPolygon.mIndexBase],sizeof(PxU32)*largestPolygon.mNbVerts);
			polygonsOut[0].mIndexBase = indexBase;
			indexBase += largestPolygon.mNbVerts;
		}
		else
		{
			if(i == largestFace)
			{
				PxMemCopy(&mSwappedIndices[indexBase], &indices[replacedPolygon.mIndexBase], sizeof(PxU32)*replacedPolygon.mNbVerts);
				polygonsOut[i].mIndexBase = indexBase;
				indexBase += replacedPolygon.mNbVerts;
			}
			else
			{
				PxMemCopy(&mSwappedIndices[indexBase], &indices[polygons[i].mIndexBase], sizeof(PxU32)*polygons[i].mNbVerts);
				polygonsOut[i].mIndexBase = indexBase;
				indexBase += polygons[i].mNbVerts;
			}
		}
	}

	PX_ASSERT(indexBase == desc.indices.count);
	
	desc.indices.data = mSwappedIndices;
}

ConvexHullLib::~ConvexHullLib()
{
	if (mSwappedIndices)
		PX_FREE(mSwappedIndices);

	if(mShiftedVerts)
		PX_FREE(mShiftedVerts);
}
