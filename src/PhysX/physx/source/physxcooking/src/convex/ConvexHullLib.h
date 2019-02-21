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


#ifndef PX_CONVEXHULLLIB_H
#define PX_CONVEXHULLLIB_H

#include "PxConvexMeshDesc.h"
#include "PxCooking.h"
#include "CmPhysXCommon.h"

namespace physx
{	
	//////////////////////////////////////////////////////////////////////////
	// base class for the convex hull libraries - inflation based and quickhull
	class ConvexHullLib
	{		
		PX_NOCOPY(ConvexHullLib)
	public:
		// functions
		ConvexHullLib(const PxConvexMeshDesc& desc, const PxCookingParams& params)
			: mConvexMeshDesc(desc), mCookingParams(params), mSwappedIndices(NULL),
			mShiftedVerts(NULL)
		{
		}

		virtual ~ConvexHullLib();
			
		// computes the convex hull from provided points
		virtual PxConvexMeshCookingResult::Enum createConvexHull() = 0;

		// fills the PxConvexMeshDesc with computed hull data
		virtual void fillConvexMeshDesc(PxConvexMeshDesc& desc) = 0;

		// compute the edge list information if possible
		virtual bool createEdgeList(const PxU32 nbIndices, const PxU8* indices, PxU8** hullDataFacesByEdges8, PxU16** edgeData16, PxU16** edges) = 0;

		static const PxU32 gpuMaxVertsPerFace = 32;

	protected:

		// clean input vertices from duplicates, normalize etc.
		bool cleanupVertices(PxU32 svcount, // input vertex count
			const PxVec3* svertices, // vertices
			PxU32 stride,		// stride
			PxU32& vcount,		// output number of vertices
			PxVec3* vertices,	// location to store the results.			
			PxVec3& scale,		// scale
			PxVec3& center);	// center

		// shift vertices around origin and clean input vertices from duplicates, normalize etc.
		bool shiftAndcleanupVertices(PxU32 svcount, // input vertex count
			const PxVec3* svertices, // vertices
			PxU32 stride,		// stride
			PxU32& vcount,		// output number of vertices
			PxVec3* vertices,	// location to store the results.			
			PxVec3& scale,		// scale
			PxVec3& center);	// center

		void swapLargestFace(PxConvexMeshDesc& desc);

		void shiftConvexMeshDesc(PxConvexMeshDesc& desc);

	protected:
		const PxConvexMeshDesc&			mConvexMeshDesc;
		const PxCookingParams&			mCookingParams;
		PxU32*							mSwappedIndices;
		PxVec3							mOriginShift;
		PxVec3*							mShiftedVerts;
	};
}

#endif
