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

#ifndef PX_QUICKHULL_CONVEXHULLLIB_H
#define PX_QUICKHULL_CONVEXHULLLIB_H

#include "ConvexHullLib.h"
#include "Ps.h"
#include "PsArray.h"
#include "PsUserAllocated.h"

namespace local
{
	class QuickHull;
	struct QuickHullVertex;
}

namespace physx
{
	class ConvexHull;

	//////////////////////////////////////////////////////////////////////////
	// Quickhull lib constructs the hull from given input points. The resulting hull 
	// will only contain a subset of the input points. The algorithm does incrementally
	// adds most furthest vertices to the starting simplex. The produced hulls are build with high precision
	// and produce more stable and correct results, than the legacy algorithm. 
	class QuickHullConvexHullLib: public ConvexHullLib, public Ps::UserAllocated
	{
		PX_NOCOPY(QuickHullConvexHullLib)
	public:

		// functions
		QuickHullConvexHullLib(const PxConvexMeshDesc& desc, const PxCookingParams& params);

		~QuickHullConvexHullLib();

		// computes the convex hull from provided points
		virtual PxConvexMeshCookingResult::Enum createConvexHull();

		// fills the convexmeshdesc with computed hull data
		virtual void fillConvexMeshDesc(PxConvexMeshDesc& desc);

		// provide the edge list information
		virtual bool createEdgeList(const PxU32, const PxU8* , PxU8** , PxU16** , PxU16** );

	protected:
		// if vertex limit reached we need to expand the hull using the OBB slicing
		PxConvexMeshCookingResult::Enum expandHullOBB();

		// if vertex limit reached we need to expand the hull using the plane shifting
		PxConvexMeshCookingResult::Enum expandHull();

		// checks for collinearity and co planarity
		// returns true if the simplex was ok, we can reuse the computed tolerances and min/max values
		bool cleanupForSimplex(PxVec3* vertices, PxU32 vertexCount, local::QuickHullVertex* minimumVertex, 
			local::QuickHullVertex* maximumVertex, float& tolerance, float& planeTolerance);

		// fill the result desc from quick hull convex
		void fillConvexMeshDescFromQuickHull(PxConvexMeshDesc& desc);

		// fill the result desc from cropped hull convex
		void fillConvexMeshDescFromCroppedHull(PxConvexMeshDesc& desc);

	private:
		local::QuickHull*		mQuickHull;		// the internal quick hull representation
		ConvexHull*				mCropedConvexHull; //the hull cropped from OBB, used for vertex limit path

		PxU8*					mOutMemoryBuffer;   // memory buffer used for output data
		PxU16*					mFaceTranslateTable; // translation table mapping output faces to internal quick hull table
	};
}

#endif
