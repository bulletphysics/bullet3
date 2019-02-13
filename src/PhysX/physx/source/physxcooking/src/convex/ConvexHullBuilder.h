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


#ifndef PX_CONVEXHULLBUILDER_H
#define PX_CONVEXHULLBUILDER_H

#include "GuConvexMeshData.h"
#include "PsUserAllocated.h"
#include "PxCooking.h"

namespace physx
{
	struct PxHullPolygon;
	class ConvexHullLib;

	namespace Gu
	{
		struct EdgeDescData;
		struct ConvexHullData;
	} // namespace Gu

	struct HullTriangleData
	{
		PxU32	mRef[3];
	};

	class ConvexHullBuilder : public Ps::UserAllocated
	{
		public:
												ConvexHullBuilder(Gu::ConvexHullData* hull, const bool buildGRBData);
												~ConvexHullBuilder();

					bool						init(PxU32 nbVerts, const PxVec3* verts, const PxU32* indices, const PxU32 nbIndices, const PxU32 nbPolygons, 
													const PxHullPolygon* hullPolygons, bool doValidation = true, ConvexHullLib* hullLib = NULL);

					bool						save(PxOutputStream& stream, bool platformMismatch)	const;
					bool						copy(Gu::ConvexHullData& hullData, PxU32& nb);
					
					bool						createEdgeList(bool doValidation, PxU32 nbEdges);
					bool						checkHullPolygons()	const;										

					bool						calculateVertexMapTable(PxU32 nbPolygons, bool userPolygons = false);					

		PX_INLINE	PxU32						computeNbPolygons()		const
												{
													PX_ASSERT(mHull->mNbPolygons);
													return mHull->mNbPolygons;
												}

					PxVec3*						mHullDataHullVertices;
					Gu::HullPolygonData*		mHullDataPolygons;
					PxU8*						mHullDataVertexData8;
					PxU8*						mHullDataFacesByEdges8;
					PxU8*						mHullDataFacesByVertices8;

					PxU16*						mEdgeData16;	//!< Edge indices indexed by hull polygons
					PxU16*						mEdges;			//!< Edge to vertex mapping

					Gu::ConvexHullData*			mHull;
					bool						mBuildGRBData;
					
		protected:										
					bool						computeGeomCenter(PxVec3& , PxU32 numFaces, HullTriangleData* faces) const; 
	};
}

#endif	// PX_CONVEXHULLBUILDER_H

