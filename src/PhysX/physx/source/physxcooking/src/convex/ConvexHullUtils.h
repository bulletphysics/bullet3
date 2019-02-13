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


#ifndef PX_CONVEXHULLUTILS_H
#define PX_CONVEXHULLUTILS_H

#include "foundation/PxMemory.h"
#include "foundation/PxPlane.h"

#include "CmPhysXCommon.h"

#include "PsUserAllocated.h"
#include "PsArray.h"
#include "PsMathUtils.h"

#include "PxConvexMeshDesc.h"

namespace physx
{

	//////////////////////////////////////////////////////////////////////////
	// helper class for hull construction, holds the vertices and planes together
	// while cropping the hull with planes
	class ConvexHull : public Ps::UserAllocated
	{
	public:

		// Helper class for halfedge representation
		class HalfEdge
		{
		public:
			PxI16 ea;         // the other half of the edge (index into edges list)
			PxU8 v;  // the vertex at the start of this edge (index into vertices list)
			PxU8 p;  // the facet on which this edge lies (index into facets list)
			HalfEdge(){}
			HalfEdge(PxI16 _ea, PxU8 _v, PxU8 _p) :ea(_ea), v(_v), p(_p){}
		};

		ConvexHull& operator = (const ConvexHull&);

		// construct the base cube hull from given max/min AABB
		ConvexHull(const PxVec3& bmin, const PxVec3& bmax, const Ps::Array<PxPlane>& inPlanes);

		// construct the base cube hull from given OBB
		ConvexHull(const PxVec3& extent, const PxTransform& transform, const Ps::Array<PxPlane>& inPlanes);

		// copy constructor
		ConvexHull(const ConvexHull& srcHull)
			: mInputPlanes(srcHull.getInputPlanes())
		{
			copyHull(srcHull);
		}

		// construct plain hull
		ConvexHull(const Ps::Array<PxPlane>& inPlanes)
			: mInputPlanes(inPlanes)
		{
		}

		// finds the candidate plane, returns -1 otherwise
		PxI32 findCandidatePlane(float planetestepsilon, float epsilon) const;

		// internal check of the hull integrity
		bool assertIntact(float epsilon) const;

		// return vertices
		const Ps::Array<PxVec3>& getVertices() const
		{
			return mVertices;
		}

		// return edges
		const Ps::Array<HalfEdge>& getEdges() const
		{
			return mEdges;
		}

		// return faces
		const Ps::Array<PxPlane>& getFacets() const
		{
			return mFacets;
		}

		// return input planes
		const Ps::Array<PxPlane>& getInputPlanes() const
		{
			return mInputPlanes;
		}

		// return vertices
		Ps::Array<PxVec3>& getVertices()
		{
			return mVertices;
		}

		// return edges
		Ps::Array<HalfEdge>& getEdges()
		{
			return mEdges;
		}

		// return faces
		Ps::Array<PxPlane>& getFacets()
		{
			return mFacets;
		}

		// returns the maximum number of vertices on a face
		PxU32 maxNumVertsPerFace() const;

		// copy the hull from source
		void copyHull(const ConvexHull& src)
		{
			mVertices.resize(src.getVertices().size());
			mEdges.resize(src.getEdges().size());
			mFacets.resize(src.getFacets().size());

			PxMemCopy(mVertices.begin(), src.getVertices().begin(), src.getVertices().size()*sizeof(PxVec3));
			PxMemCopy(mEdges.begin(), src.getEdges().begin(), src.getEdges().size()*sizeof(HalfEdge));
			PxMemCopy(mFacets.begin(), src.getFacets().begin(), src.getFacets().size()*sizeof(PxPlane));
		}

	private:
		Ps::Array<PxVec3>	mVertices;
		Ps::Array<HalfEdge> mEdges;
		Ps::Array<PxPlane>  mFacets;
		const Ps::Array<PxPlane>&	mInputPlanes;
	};

	//////////////////////////////////////////////////////////////////////////|
	// Crops the hull with a provided plane and with given epsilon
	// returns new hull if succeeded
	ConvexHull* convexHullCrop(const ConvexHull& convex, const PxPlane& slice, float planetestepsilon);

	//////////////////////////////////////////////////////////////////////////|
	// three planes intersection
	PX_FORCE_INLINE PxVec3 threePlaneIntersection(const PxPlane& p0, const PxPlane& p1, const PxPlane& p2)
	{
		PxMat33 mp = (PxMat33(p0.n, p1.n, p2.n)).getTranspose();
		PxMat33 mi = (mp).getInverse();
		PxVec3 b(p0.d, p1.d, p2.d);
		return -mi.transform(b);
	}

	//////////////////////////////////////////////////////////////////////////
	// Compute OBB around given convex hull
	bool computeOBBFromConvex(const PxConvexMeshDesc& desc, PxVec3& sides, PxTransform& matrix);
}

#endif
