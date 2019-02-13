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

#ifndef GU_CONVEXHELPER_H
#define GU_CONVEXHELPER_H

#include "GuShapeConvex.h"

namespace physx
{
namespace Gu
{
	class GeometryUnion;

	///////////////////////////////////////////////////////////////////////////

	PX_PHYSX_COMMON_API	void getScaledConvex(	PxVec3*& scaledVertices, PxU8*& scaledIndices, PxVec3* dstVertices, PxU8* dstIndices,
												bool idtConvexScale, const PxVec3* srcVerts, const PxU8* srcIndices, PxU32 nbVerts, const Cm::FastVertex2ShapeScaling& convexScaling);

	// PT: calling this correctly isn't trivial so let's macroize it. At least we limit the damage since it immediately calls a real function.
	#define GET_SCALEX_CONVEX(scaledVertices, stackIndices, idtScaling, nbVerts, scaling, srcVerts, srcIndices)	\
	getScaledConvex(scaledVertices, stackIndices,																\
					idtScaling ? NULL : reinterpret_cast<PxVec3*>(PxAlloca(nbVerts * sizeof(PxVec3))),			\
					idtScaling ? NULL : reinterpret_cast<PxU8*>(PxAlloca(nbVerts * sizeof(PxU8))),				\
					idtScaling, srcVerts, srcIndices, nbVerts, scaling);

	PX_PHYSX_COMMON_API bool getConvexData(const Gu::GeometryUnion& shape, Cm::FastVertex2ShapeScaling& scaling, PxBounds3& bounds, PolygonalData& polyData);

	struct ConvexEdge
	{
		PxU8	vref0;
		PxU8	vref1;
		PxVec3	normal;	// warning: non-unit vector!
	};

	PX_PHYSX_COMMON_API PxU32 findUniqueConvexEdges(PxU32 maxNbEdges, ConvexEdge* PX_RESTRICT edges, PxU32 numPolygons, const Gu::HullPolygonData* PX_RESTRICT polygons, const PxU8* PX_RESTRICT vertexData);
}
}

#endif
