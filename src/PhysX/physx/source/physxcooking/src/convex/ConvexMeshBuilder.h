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


#ifndef PX_COLLISION_CONVEXMESHBUILDER
#define PX_COLLISION_CONVEXMESHBUILDER

#include "GuConvexMeshData.h"
#include "PxCooking.h"
#include "ConvexPolygonsBuilder.h"

namespace physx
{
	//////////////////////////////////////////////////////////////////////////
	// Convex mesh builder, creates the convex mesh from given polygons and creates internal data
	class ConvexMeshBuilder
	{
	public:
									ConvexMeshBuilder(const bool buildGRBData);
									~ConvexMeshBuilder();

				// loads the computed or given convex hull from descriptor. 
				// the descriptor does contain polygons directly, triangles are not allowed
				bool				build(const PxConvexMeshDesc&, PxU32 gaussMapVertexLimit, bool validateOnly = false, ConvexHullLib* hullLib = NULL);

				// save the convex mesh into stream
				bool				save(PxOutputStream& stream, bool platformMismatch)		const;

				// copy the convex mesh into internal convex mesh, which can be directly used then
				bool				copy(Gu::ConvexHullData& convexData, PxU32& nb);

				// loads the convex mesh from given polygons
				bool				loadConvexHull(const PxConvexMeshDesc&, ConvexHullLib* hullLib);

				// computed hull polygons from given triangles
				bool				computeHullPolygons(const PxU32& nbVerts,const PxVec3* verts, const PxU32& nbTriangles, const PxU32* triangles, PxAllocatorCallback& inAllocator,
										 PxU32& outNbVerts, PxVec3*& outVertices, PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& polygons);

				// compute big convex data
				bool				computeGaussMaps();

				// compute mass, inertia tensor
				void				computeMassInfo(bool lowerPrecision);
// TEST_INTERNAL_OBJECTS
				// internal objects
				void				computeInternalObjects();
//~TEST_INTERNAL_OBJECTS

				// return computed mass
				PxReal				getMass() const { return mMass; }

				// return computed inertia tensor
				const PxMat33&		getInertia() const { return mInertia; }

				// return big convex data
				BigConvexData*		getBigConvexData() const  { return mBigConvexData; }

				// set big convex data
				void				setBigConvexData(BigConvexData* data) { mBigConvexData = data; }

		mutable	ConvexPolygonsBuilder	hullBuilder;

	protected:
		Gu::ConvexHullData			mHullData;		

		BigConvexData*				mBigConvexData;		//!< optional, only for large meshes! PT: redundant with ptr in chull data? Could also be end of other buffer
		PxReal						mMass;				//this is mass assuming a unit density that can be scaled by instances!
		PxMat33						mInertia;			//in local space of mesh!

	};

}

#endif
