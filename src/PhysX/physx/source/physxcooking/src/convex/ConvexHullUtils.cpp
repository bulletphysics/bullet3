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


#include "foundation/PxBounds3.h"
#include "foundation/PxMathUtils.h"

#include "ConvexHullUtils.h"
#include "VolumeIntegration.h"
#include "PsUtilities.h"
#include "PsVecMath.h"
#include "GuBox.h"
#include "GuConvexMeshData.h"

using namespace physx;
using namespace Ps::aos;

namespace local
{
	static const float MIN_ADJACENT_ANGLE = 3.0f;  // in degrees  - result wont have two adjacent facets within this angle of each other.
	static const float MAXDOT_MINANG = cosf(Ps::degToRad(MIN_ADJACENT_ANGLE)); // adjacent angle for dot product tests

	//////////////////////////////////////////////////////////////////////////
	// helper class for ConvexHullCrop
	class VertFlag
	{
	public:
		PxU8 planetest;
		PxU8 undermap;
		PxU8 overmap;
	};

	//////////////////////////////////////////////////////////////////////////|
	// helper class for ConvexHullCrop
	class EdgeFlag
	{
	public:
		PxI16 undermap;
	};

	//////////////////////////////////////////////////////////////////////////|
	// helper class for ConvexHullCrop
	class Coplanar
	{
	public:
		PxU16 ea;
		PxU8 v0;
		PxU8 v1;
	};

	//////////////////////////////////////////////////////////////////////////
	// plane test
	enum PlaneTestResult
	{
		eCOPLANAR = 0,
		eUNDER = 1 << 0,
		eOVER = 1 << 1
	};

	//////////////////////////////////////////////////////////////////////////
	// test where vertex lies in respect to the plane
	static PlaneTestResult planeTest(const PxPlane& p, const PxVec3& v, float epsilon)
	{
		const float a = v.dot(p.n) + p.d;
		PlaneTestResult flag = (a > epsilon) ? eOVER : ((a < -epsilon) ? eUNDER : eCOPLANAR);
		return flag;
	}

	// computes the OBB for this set of points relative to this transform matrix. SIMD version
	void computeOBBSIMD(PxU32 vcount, const Vec4V* points, Vec4V& sides, const QuatV& rot, Vec4V& trans)
	{
		PX_ASSERT(vcount);

		Vec4V minV = V4Load(FLT_MAX);
		Vec4V maxV = V4Load(FLT_MIN);
		for (PxU32 i = 0; i < vcount; i++)
		{
			const Vec4V& vertexV = points[i];
			const Vec4V t = V4Sub(vertexV, trans);
			const Vec4V v = Vec4V_From_Vec3V(QuatRotateInv(rot, Vec3V_From_Vec4V(t)));

			minV = V4Min(minV, v);
			maxV = V4Max(maxV, v);
		}
				
		sides = V4Sub(maxV, minV);

		Mat33V tmpMat;
		QuatGetMat33V(rot, tmpMat.col0, tmpMat.col1, tmpMat.col2);
		const FloatV coe = FLoad(0.5f);

		const Vec4V deltaVec = V4Sub(maxV, V4Scale(sides, coe));		

		const Vec4V t0 = V4Scale(Vec4V_From_Vec3V(tmpMat.col0), V4GetX(deltaVec));
		trans = V4Add(trans, t0);

		const Vec4V t1 = V4Scale(Vec4V_From_Vec3V(tmpMat.col1), V4GetY(deltaVec));
		trans = V4Add(trans, t1);

		const Vec4V t2 = V4Scale(Vec4V_From_Vec3V(tmpMat.col2), V4GetZ(deltaVec));
		trans = V4Add(trans, t2);
	}
}

//////////////////////////////////////////////////////////////////////////
// construct the base cube from given min/max
ConvexHull::ConvexHull(const PxVec3& bmin, const PxVec3& bmax, const Ps::Array<PxPlane>& inPlanes)
: mInputPlanes(inPlanes)
{
	// min max verts of the cube - 8 verts
	mVertices.pushBack(PxVec3(bmin.x, bmin.y, bmin.z)); // ---
	mVertices.pushBack(PxVec3(bmin.x, bmin.y, bmax.z)); // --+
	mVertices.pushBack(PxVec3(bmin.x, bmax.y, bmin.z)); // -+-
	mVertices.pushBack(PxVec3(bmin.x, bmax.y, bmax.z)); // -++
	mVertices.pushBack(PxVec3(bmax.x, bmin.y, bmin.z)); // +--
	mVertices.pushBack(PxVec3(bmax.x, bmin.y, bmax.z)); // +-+
	mVertices.pushBack(PxVec3(bmax.x, bmax.y, bmin.z)); // ++-
	mVertices.pushBack(PxVec3(bmax.x, bmax.y, bmax.z)); // +++

	// cube planes - 6 planes
	mFacets.pushBack(PxPlane(PxVec3(-1.f, 0, 0), bmin.x)); // 0,1,3,2
	mFacets.pushBack(PxPlane(PxVec3(1.f, 0, 0), -bmax.x)); // 6,7,5,4
	mFacets.pushBack(PxPlane(PxVec3(0, -1.f, 0), bmin.y)); // 0,4,5,1
	mFacets.pushBack(PxPlane(PxVec3(0, 1.f, 0), -bmax.y)); // 3,7,6,2
	mFacets.pushBack(PxPlane(PxVec3(0, 0, -1.f), bmin.z)); // 0,2,6,4
	mFacets.pushBack(PxPlane(PxVec3(0, 0, 1.f), -bmax.z)); // 1,5,7,3

	// cube edges - 24 edges
	mEdges.pushBack(HalfEdge(11, 0, 0));
	mEdges.pushBack(HalfEdge(23, 1, 0));
	mEdges.pushBack(HalfEdge(15, 3, 0));
	mEdges.pushBack(HalfEdge(16, 2, 0));

	mEdges.pushBack(HalfEdge(13, 6, 1));
	mEdges.pushBack(HalfEdge(21, 7, 1));
	mEdges.pushBack(HalfEdge(9, 5, 1));
	mEdges.pushBack(HalfEdge(18, 4, 1));

	mEdges.pushBack(HalfEdge(19, 0, 2));
	mEdges.pushBack(HalfEdge(6, 4, 2));
	mEdges.pushBack(HalfEdge(20, 5, 2));
	mEdges.pushBack(HalfEdge(0, 1, 2));

	mEdges.pushBack(HalfEdge(22, 3, 3));
	mEdges.pushBack(HalfEdge(4, 7, 3));
	mEdges.pushBack(HalfEdge(17, 6, 3));
	mEdges.pushBack(HalfEdge(2, 2, 3));

	mEdges.pushBack(HalfEdge(3, 0, 4));
	mEdges.pushBack(HalfEdge(14, 2, 4));
	mEdges.pushBack(HalfEdge(7, 6, 4));
	mEdges.pushBack(HalfEdge(8, 4, 4));

	mEdges.pushBack(HalfEdge(10, 1, 5));
	mEdges.pushBack(HalfEdge(5, 5, 5));
	mEdges.pushBack(HalfEdge(12, 7, 5));
	mEdges.pushBack(HalfEdge(1, 3, 5));
}

//////////////////////////////////////////////////////////////////////////
// create the initial convex hull from given OBB
ConvexHull::ConvexHull(const PxVec3& extent, const PxTransform& transform, const Ps::Array<PxPlane>& inPlanes)
	: mInputPlanes(inPlanes)
{
	// get the OBB corner points
	PxVec3 extentPoints[8];
	PxMat33 rot(transform.q);
	Gu::computeOBBPoints(extentPoints, transform.p, extent, rot.column0, rot.column1, rot.column2);

	mVertices.pushBack(PxVec3(extentPoints[0].x, extentPoints[0].y, extentPoints[0].z)); // ---
	mVertices.pushBack(PxVec3(extentPoints[4].x, extentPoints[4].y, extentPoints[4].z)); // --+
	mVertices.pushBack(PxVec3(extentPoints[3].x, extentPoints[3].y, extentPoints[3].z)); // -+-
	mVertices.pushBack(PxVec3(extentPoints[7].x, extentPoints[7].y, extentPoints[7].z)); // -++
	mVertices.pushBack(PxVec3(extentPoints[1].x, extentPoints[1].y, extentPoints[1].z)); // +--
	mVertices.pushBack(PxVec3(extentPoints[5].x, extentPoints[5].y, extentPoints[5].z)); // +-+
	mVertices.pushBack(PxVec3(extentPoints[2].x, extentPoints[2].y, extentPoints[2].z)); // ++-
	mVertices.pushBack(PxVec3(extentPoints[6].x, extentPoints[6].y, extentPoints[6].z)); // +++

	// cube planes - 6 planes
	PxPlane plane0(extentPoints[0], extentPoints[4], extentPoints[7]);	// 0,1,3,2
	mFacets.pushBack(PxPlane(plane0.n, plane0.d));

	PxPlane plane1(extentPoints[2], extentPoints[6], extentPoints[5]);	// 6,7,5,4
	mFacets.pushBack(PxPlane(plane1.n, plane1.d));

	PxPlane plane2(extentPoints[0], extentPoints[1], extentPoints[5]);	// 0,4,5,1
	mFacets.pushBack(PxPlane(plane2.n, plane2.d));

	PxPlane plane3(extentPoints[7], extentPoints[6], extentPoints[2]);	// 3,7,6,2
	mFacets.pushBack(PxPlane(plane3.n, plane3.d));

	PxPlane plane4(extentPoints[0], extentPoints[3], extentPoints[2]);	// 0,2,6,4
	mFacets.pushBack(PxPlane(plane4.n, plane4.d));

	PxPlane plane5(extentPoints[4], extentPoints[5], extentPoints[6]);	// 1,5,7,3
	mFacets.pushBack(PxPlane(plane5.n, plane5.d));

	// cube edges - 24 edges
	mEdges.pushBack(HalfEdge(11, 0, 0));
	mEdges.pushBack(HalfEdge(23, 1, 0));
	mEdges.pushBack(HalfEdge(15, 3, 0));
	mEdges.pushBack(HalfEdge(16, 2, 0));

	mEdges.pushBack(HalfEdge(13, 6, 1));
	mEdges.pushBack(HalfEdge(21, 7, 1));
	mEdges.pushBack(HalfEdge(9, 5, 1));
	mEdges.pushBack(HalfEdge(18, 4, 1));

	mEdges.pushBack(HalfEdge(19, 0, 2));
	mEdges.pushBack(HalfEdge(6, 4, 2));
	mEdges.pushBack(HalfEdge(20, 5, 2));
	mEdges.pushBack(HalfEdge(0, 1, 2));

	mEdges.pushBack(HalfEdge(22, 3, 3));
	mEdges.pushBack(HalfEdge(4, 7, 3));
	mEdges.pushBack(HalfEdge(17, 6, 3));
	mEdges.pushBack(HalfEdge(2, 2, 3));

	mEdges.pushBack(HalfEdge(3, 0, 4));
	mEdges.pushBack(HalfEdge(14, 2, 4));
	mEdges.pushBack(HalfEdge(7, 6, 4));
	mEdges.pushBack(HalfEdge(8, 4, 4));

	mEdges.pushBack(HalfEdge(10, 1, 5));
	mEdges.pushBack(HalfEdge(5, 5, 5));
	mEdges.pushBack(HalfEdge(12, 7, 5));
	mEdges.pushBack(HalfEdge(1, 3, 5));
}

//////////////////////////////////////////////////////////////////////////
// finds the candidate plane, returns -1 otherwise
PxI32 ConvexHull::findCandidatePlane(float planeTestEpsilon, float epsilon) const
{
	PxI32 p = -1;
	float md = 0.0f;
	PxU32 i, j;	
	for (i = 0; i < mInputPlanes.size(); i++)
	{
		float d = 0.0f;
		float dmax = 0.0f;
		float dmin = 0.0f;
		for (j = 0; j < mVertices.size(); j++)
		{
			dmax = PxMax(dmax, mVertices[j].dot(mInputPlanes[i].n) + mInputPlanes[i].d);
			dmin = PxMin(dmin, mVertices[j].dot(mInputPlanes[i].n) + mInputPlanes[i].d);
		}

		float dr = dmax - dmin;
		if (dr < planeTestEpsilon)
			dr = 1.0f; // shouldn't happen.
		d = dmax / dr;
		// we have a better candidate try another one
		if (d <= md)
			continue;
		// check if we dont have already that plane or if the normals are nearly the same
		for (j = 0; j<mFacets.size(); j++)
		{
			if (mInputPlanes[i] == mFacets[j])
			{
				d = 0.0f;
				continue;
			}
			if (mInputPlanes[i].n.dot(mFacets[j].n)> local::MAXDOT_MINANG)
			{
				for (PxU32 k = 0; k < mEdges.size(); k++)
				{
					if (mEdges[k].p != j)
						continue;
					if (mVertices[mEdges[k].v].dot(mInputPlanes[i].n) + mInputPlanes[i].d < 0)
					{
						d = 0; // so this plane wont get selected.
						break;
					}
				}
			}
		}
		if (d>md)
		{
			p = PxI32(i);
			md = d;
		}
	}
	return (md > epsilon) ? p : -1;
}

//////////////////////////////////////////////////////////////////////////
// internal hull check
bool ConvexHull::assertIntact(float epsilon) const
{
	PxU32 i;
	PxU32 estart = 0;
	for (i = 0; i < mEdges.size(); i++)
	{
		if (mEdges[estart].p != mEdges[i].p)
		{
			estart = i;
		}
		PxU32 inext = i + 1;
		if (inext >= mEdges.size() || mEdges[inext].p != mEdges[i].p)
		{
			inext = estart;
		}
		PX_ASSERT(mEdges[inext].p == mEdges[i].p);
		PxI16 nb = mEdges[i].ea;
		if (nb == 255 || nb == -1)
			return false;
		PX_ASSERT(nb != -1);
		PX_ASSERT(i == PxU32(mEdges[PxU32(nb)].ea));
		// Check that the vertex of the next edge is the vertex of the adjacent half edge.
		// Otherwise the two half edges are not really adjacent and we have a hole.
		PX_ASSERT(mEdges[PxU32(nb)].v == mEdges[inext].v);
		if (!(mEdges[PxU32(nb)].v == mEdges[inext].v))
			return false;
	}

	for (i = 0; i < mEdges.size(); i++)
	{
		PX_ASSERT(local::eCOPLANAR == local::planeTest(mFacets[mEdges[i].p], mVertices[mEdges[i].v], epsilon));
		if (local::eCOPLANAR != local::planeTest(mFacets[mEdges[i].p], mVertices[mEdges[i].v], epsilon))
			return false;
		if (mEdges[estart].p != mEdges[i].p)
		{
			estart = i;
		}
		PxU32 i1 = i + 1;
		if (i1 >= mEdges.size() || mEdges[i1].p != mEdges[i].p) {
			i1 = estart;
		}
		PxU32 i2 = i1 + 1;
		if (i2 >= mEdges.size() || mEdges[i2].p != mEdges[i].p) {
			i2 = estart;
		}
		if (i == i2)
			continue; // i sliced tangent to an edge and created 2 meaningless edges

		// check the face normal against the triangle from edges
		PxVec3 localNormal = (mVertices[mEdges[i1].v] - mVertices[mEdges[i].v]).cross(mVertices[mEdges[i2].v] - mVertices[mEdges[i1].v]);
		const float m = localNormal.magnitude();
		if (m == 0.0f)
			localNormal = PxVec3(1.f, 0.0f, 0.0f);
		localNormal *= (1.0f / m);
		if (localNormal.dot(mFacets[mEdges[i].p].n) <= 0.0f)
			return false;
	}
	return true;
}

// returns the maximum number of vertices on a face
PxU32 ConvexHull::maxNumVertsPerFace() const
{	
	PxU32 maxVerts = 0;
	PxU32 currentVerts = 0;
	PxU32 estart = 0;
	for (PxU32 i = 0; i < mEdges.size(); i++)
	{
		if (mEdges[estart].p != mEdges[i].p)
		{
			if(currentVerts > maxVerts)
			{
				maxVerts = currentVerts + 1;
			}
			currentVerts = 0;
			estart = i;
		}
		else
		{
			currentVerts++;
		}
	}
	return maxVerts;
}

//////////////////////////////////////////////////////////////////////////
// slice the input convexHull with the slice plane
ConvexHull* physx::convexHullCrop(const ConvexHull& convex, const PxPlane& slice, float planeTestEpsilon)
{
	static const PxU8 invalidIndex = PxU8(-1);
	PxU32 i;
	PxU32 vertCountUnder = 0; // Running count of the vertices UNDER the slicing plane.

	PX_ASSERT(convex.getEdges().size() < 480);

	// Arrays of mapping information associated with features in the input convex.
	// edgeflag[i].undermap  - output index of input edge convex->edges[i]
	// vertflag[i].undermap  - output index of input vertex convex->vertices[i]
	// vertflag[i].planetest - the side-of-plane classification of convex->vertices[i]		
	// (There are other members but they are unused.)
	local::EdgeFlag  edgeFlag[512];
	local::VertFlag  vertFlag[256];

	// Lists of output features. Populated during clipping.
	// Coplanar edges have one sibling in tmpunderedges and one in coplanaredges.
	// coplanaredges holds the sibling that belong to the new polygon created from slicing.
	ConvexHull::HalfEdge  tmpUnderEdges[512];  // The output edge list.
	PxPlane	  tmpUnderPlanes[128]; // The output plane list.
	local::Coplanar  coplanarEdges[512];  // The coplanar edge list.

	PxU32 coplanarEdgesNum = 0; // Running count of coplanar edges.

	// Created vertices on the slicing plane (stored for output after clipping).
	Ps::Array<PxVec3> createdVerts;

	// Logical OR of individual vertex flags.
	PxU32 convexClipFlags = 0;

	// Classify each vertex against the slicing plane as OVER | COPLANAR | UNDER.
	// OVER     - Vertex is over (outside) the slicing plane. Will not be output.
	// COPLANAR - Vertex is on the slicing plane. A copy will be output.
	// UNDER    - Vertex is under (inside) the slicing plane. Will be output.
	// We keep an array of information structures for each vertex in the input convex.
	// vertflag[i].undermap  - The (computed) index of convex->vertices[i] in the output.
	//                         invalidIndex for OVER vertices - they are not output.
	//                         initially invalidIndex for COPLANAR vertices - set later.
	// vertflag[i].overmap   - Unused - we don't care about the over part.
	// vertflag[i].planetest - The classification (clip flag) of convex->vertices[i].
	for (i = 0; i < convex.getVertices().size(); i++)
	{
		local::PlaneTestResult vertexClipFlag = local::planeTest(slice, convex.getVertices()[i], planeTestEpsilon);
		switch (vertexClipFlag)
		{
		case local::eOVER:
		case local::eCOPLANAR:
			vertFlag[i].undermap = invalidIndex; // Initially invalid for COPLANAR
			vertFlag[i].overmap = invalidIndex;
			break;
		case local::eUNDER:
			vertFlag[i].undermap = Ps::to8(vertCountUnder++);
			vertFlag[i].overmap = invalidIndex;
			break;
		}
		vertFlag[i].planetest = PxU8(vertexClipFlag);
		convexClipFlags |= vertexClipFlag;
	}

	// Check special case: everything UNDER or COPLANAR.
	// This way we know we wont end up with silly faces / edges later on.
	if ((convexClipFlags & local::eOVER) == 0)
	{
		// Just return a copy of the same convex.
		ConvexHull* dst = PX_NEW_TEMP(ConvexHull)(convex);
		return dst;
	}

	PxU16 underEdgeCount = 0; // Running count of output edges.
	PxU16 underPlanesCount = 0; // Running count of output planes.

	// Clipping Loop
	// =============
	//
	// for each plane
	//
	//    for each edge
	//
	//       if first UNDER & second !UNDER
	//          output current edge -> tmpunderedges
	//          if we have done the sibling
	//             connect current edge to its sibling
	//             set vout = first vertex of sibling
	//          else if second is COPLANAR
	//             if we havent already copied it
	//                copy second -> createdverts
	//             set vout = index of created vertex
	//          else
	//             generate a new vertex -> createdverts
	//             set vout = index of created vertex
	//          if vin is already set and vin != vout (non-trivial edge)
	//             output coplanar edge -> tmpunderedges (one sibling)
	//             set coplanaredge to new edge index (for connecting the other sibling)
	//
	//       else if first !UNDER & second UNDER
	//          if we have done the sibling
	//             connect current edge to its sibling
	//             set vin = second vertex of sibling (this is a bit of a pain)
	//          else if first is COPLANAR
	//             if we havent already copied it
	//                copy first -> createdverts
	//             set vin = index of created vertex
	//          else
	//             generate a new vertex -> createdverts
	//             set vin = index of created vertex
	//          if vout is already set and vin != vout (non-trivial edge)
	//             output coplanar edge -> tmpunderedges (one sibling)
	//             set coplanaredge to new edge index (for connecting the other sibling)
	//          output current edge -> tmpunderedges
	//
	//       else if first UNDER & second UNDER
	//          output current edge -> tmpunderedges
	//
	//    next edge
	//
	//    if part of current plane was UNDER
	//       output current plane -> tmpunderplanes
	//
	//    if coplanaredge is set
	//       output coplanar edge -> coplanaredges
	// 
	// next plane
	// 

	// Indexing is a bit tricky here:
	// 
	// e0           - index of the current edge
	// e1           - index of the next edge
	// estart       - index of the first edge in the current plane
	// currentplane - index of the current plane
	// enextface    - first edge of next plane

	PxU32 e0 = 0;

	for (PxU32 currentplane = 0; currentplane < convex.getFacets().size(); currentplane++)
	{

		PxU32 eStart = e0;
		PxU32 eNextFace = 0xffffffff;
		PxU32 e1 = e0 + 1;

		PxU8 vout = invalidIndex;
		PxU8 vin = invalidIndex;

		PxU32 coplanarEdge = invalidIndex;

		// Logical OR of individual vertex flags in the current plane.
		PxU32 planeSide = 0;

		do{

			// Next edge modulo logic
			if (e1 >= convex.getEdges().size() || convex.getEdges()[e1].p != currentplane)
			{
				eNextFace = e1;
				e1 = eStart;
			}

			const ConvexHull::HalfEdge& edge0 = convex.getEdges()[e0];
			const ConvexHull::HalfEdge& edge1 = convex.getEdges()[e1];
			const ConvexHull::HalfEdge& edgea = convex.getEdges()[PxU32(edge0.ea)];

			planeSide |= vertFlag[edge0.v].planetest;

			if (vertFlag[edge0.v].planetest == local::eUNDER && vertFlag[edge1.v].planetest != local::eUNDER)
			{
				// first is UNDER, second is COPLANAR or OVER

				// Output current edge.
				edgeFlag[e0].undermap = short(underEdgeCount);
				tmpUnderEdges[underEdgeCount].v = vertFlag[edge0.v].undermap;
				tmpUnderEdges[underEdgeCount].p = PxU8(underPlanesCount);
				PX_ASSERT(tmpUnderEdges[underEdgeCount].v != invalidIndex);

				if (PxU32(edge0.ea) < e0)
				{
					// We have already done the sibling.
					// Connect current edge to its sibling.
					PX_ASSERT(edgeFlag[edge0.ea].undermap != invalidIndex);
					tmpUnderEdges[underEdgeCount].ea = edgeFlag[edge0.ea].undermap;
					tmpUnderEdges[edgeFlag[edge0.ea].undermap].ea = short(underEdgeCount);
					// Set vout = first vertex of (output, clipped) sibling.
					vout = tmpUnderEdges[edgeFlag[edge0.ea].undermap].v;
				}
				else if (vertFlag[edge1.v].planetest == local::eCOPLANAR)
				{
					// Boundary case.
					// We output coplanar vertices once.
					if (vertFlag[edge1.v].undermap == invalidIndex)
					{
						createdVerts.pushBack(convex.getVertices()[edge1.v]);
						// Remember the index so we don't output it again.
						vertFlag[edge1.v].undermap = Ps::to8(vertCountUnder++);
					}
					vout = vertFlag[edge1.v].undermap;
				}
				else
				{
					// Add new vertex.
					const PxPlane& p0 = convex.getFacets()[edge0.p];
					const PxPlane& pa = convex.getFacets()[edgea.p];
					createdVerts.pushBack(threePlaneIntersection(p0, pa, slice));
					vout = Ps::to8(vertCountUnder++);
				}

				// We added an edge, increment the counter
				underEdgeCount++;

				if (vin != invalidIndex && vin != vout)
				{
					// We already have vin and a non-trivial edge
					// Output coplanar edge
					PX_ASSERT(vout != invalidIndex);
					coplanarEdge = underEdgeCount;
					tmpUnderEdges[underEdgeCount].v = vout;
					tmpUnderEdges[underEdgeCount].p = PxU8(underPlanesCount);
					tmpUnderEdges[underEdgeCount].ea = invalidIndex;
					underEdgeCount++;
				}
			}
			else if (vertFlag[edge0.v].planetest != local::eUNDER && vertFlag[edge1.v].planetest == local::eUNDER)
			{
				// First is OVER or COPLANAR, second is UNDER.

				if (PxU32(edge0.ea) < e0)
				{
					// We have already done the sibling.
					// We need the second vertex of the sibling.
					// Which is the vertex of the next edge in the adjacent poly.
					int nea = edgeFlag[edge0.ea].undermap + 1;
					int p = tmpUnderEdges[edgeFlag[edge0.ea].undermap].p;
					if (nea >= underEdgeCount || tmpUnderEdges[nea].p != p)
					{
						// End of polygon, next edge is first edge
						nea -= 2;
						while (nea > 0 && tmpUnderEdges[nea - 1].p == p)
							nea--;
					}
					vin = tmpUnderEdges[nea].v;
					PX_ASSERT(vin < vertCountUnder);
				}
				else if (vertFlag[edge0.v].planetest == local::eCOPLANAR)
				{
					// Boundary case.
					// We output coplanar vertices once.
					if (vertFlag[edge0.v].undermap == invalidIndex)
					{
						createdVerts.pushBack(convex.getVertices()[edge0.v]);
						// Remember the index so we don't output it again.
						vertFlag[edge0.v].undermap = Ps::to8(vertCountUnder++);
					}
					vin = vertFlag[edge0.v].undermap;
				}
				else
				{
					// Add new vertex.
					const PxPlane& p0 = convex.getFacets()[edge0.p];
					const PxPlane& pa = convex.getFacets()[edgea.p];
					createdVerts.pushBack(threePlaneIntersection(p0, pa, slice));
					vin = Ps::to8(vertCountUnder++);
				}

				if (vout != invalidIndex && vin != vout)
				{
					// We have been in and out, Add the coplanar edge
					coplanarEdge = underEdgeCount;
					tmpUnderEdges[underEdgeCount].v = vout;
					tmpUnderEdges[underEdgeCount].p = Ps::to8(underPlanesCount);
					tmpUnderEdges[underEdgeCount].ea = invalidIndex;
					underEdgeCount++;
				}

				// Output current edge.
				tmpUnderEdges[underEdgeCount].v = vin;
				tmpUnderEdges[underEdgeCount].p = Ps::to8(underPlanesCount);
				edgeFlag[e0].undermap = short(underEdgeCount);

				if (PxU32(edge0.ea) < e0)
				{
					// We have already done the sibling.
					// Connect current edge to its sibling.
					PX_ASSERT(edgeFlag[edge0.ea].undermap != invalidIndex);
					tmpUnderEdges[underEdgeCount].ea = edgeFlag[edge0.ea].undermap;
					tmpUnderEdges[edgeFlag[edge0.ea].undermap].ea = short(underEdgeCount);
				}

				PX_ASSERT(edgeFlag[e0].undermap == underEdgeCount);
				underEdgeCount++;
			}
			else if (vertFlag[edge0.v].planetest == local::eUNDER && vertFlag[edge1.v].planetest == local::eUNDER)
			{
				// Both UNDER

				// Output current edge.
				edgeFlag[e0].undermap = short(underEdgeCount);
				tmpUnderEdges[underEdgeCount].v = vertFlag[edge0.v].undermap;
				tmpUnderEdges[underEdgeCount].p = Ps::to8(underPlanesCount);
				if (PxU32(edge0.ea) < e0)
				{
					// We have already done the sibling.
					// Connect current edge to its sibling.
					PX_ASSERT(edgeFlag[edge0.ea].undermap != invalidIndex);
					tmpUnderEdges[underEdgeCount].ea = edgeFlag[edge0.ea].undermap;
					tmpUnderEdges[edgeFlag[edge0.ea].undermap].ea = short(underEdgeCount);
				}
				underEdgeCount++;
			}

			e0 = e1;
			e1++; // do the modulo at the beginning of the loop

		} while (e0 != eStart);

		e0 = eNextFace;

		if (planeSide & local::eUNDER)
		{
			// At least part of current plane is UNDER.
			// Output current plane.
			tmpUnderPlanes[underPlanesCount] = convex.getFacets()[currentplane];
			underPlanesCount++;
		}

		if (coplanarEdge != invalidIndex)
		{
			// We have a coplanar edge.
			// Add to coplanaredges for later processing.
			// (One sibling is in place but one is missing)
			PX_ASSERT(vin != invalidIndex);
			PX_ASSERT(vout != invalidIndex);
			PX_ASSERT(coplanarEdge != 511);
			coplanarEdges[coplanarEdgesNum].ea = PxU8(coplanarEdge);
			coplanarEdges[coplanarEdgesNum].v0 = vin;
			coplanarEdges[coplanarEdgesNum].v1 = vout;
			coplanarEdgesNum++;
		}

		// Reset coplanar edge infos for next poly
		vin = invalidIndex;
		vout = invalidIndex;
		coplanarEdge = invalidIndex;
	}

	// Add the new plane to the mix:
	if (coplanarEdgesNum > 0)
	{
		tmpUnderPlanes[underPlanesCount++] = slice;
	}

	// Sort the coplanar edges in winding order.
	for (i = 0; i < coplanarEdgesNum - 1; i++)
	{
		if (coplanarEdges[i].v1 != coplanarEdges[i + 1].v0)
		{
			PxU32 j = 0;
			for (j = i + 2; j < coplanarEdgesNum; j++)
			{
				if (coplanarEdges[i].v1 == coplanarEdges[j].v0)
				{
					local::Coplanar tmp = coplanarEdges[i + 1];
					coplanarEdges[i + 1] = coplanarEdges[j];
					coplanarEdges[j] = tmp;
					break;
				}
			}
			if (j >= coplanarEdgesNum)
			{
				// PX_ASSERT(j<coplanaredges_num);
				return NULL;
			}
		}
	}

	// PT: added this line to fix DE2904
	if (!vertCountUnder)
		return NULL;

	// Create the output convex.
	ConvexHull* punder = PX_NEW_TEMP(ConvexHull)(convex.getInputPlanes());
	ConvexHull& under = *punder;

	// Copy UNDER vertices
	PxU32 k = 0;
	for (i = 0; i < convex.getVertices().size(); i++)
	{
		if (vertFlag[i].planetest == local::eUNDER)
		{
			under.getVertices().pushBack(convex.getVertices()[i]);
			k++;
		}
	}

	// Copy created vertices
	i = 0;
	while (k < vertCountUnder)
	{
		under.getVertices().pushBack(createdVerts[i++]);
		k++;
	}

	PX_ASSERT(i == createdVerts.size());

	// Copy the output edges and output planes.
	under.getEdges().resize(underEdgeCount + coplanarEdgesNum);
	under.getFacets().resize(underPlanesCount);

	// Add the coplanar edge siblings that belong to the new polygon (coplanaredges).
	for (i = 0; i < coplanarEdgesNum; i++)
	{
		under.getEdges()[underEdgeCount + i].p = PxU8(underPlanesCount - 1);
		under.getEdges()[underEdgeCount + i].ea = short(coplanarEdges[i].ea);
		tmpUnderEdges[coplanarEdges[i].ea].ea = PxI16(underEdgeCount + i);
		under.getEdges()[underEdgeCount + i].v = coplanarEdges[i].v0;
	}

	PxMemCopy(under.getEdges().begin(), tmpUnderEdges, sizeof(ConvexHull::HalfEdge)*underEdgeCount);
	PxMemCopy(under.getFacets().begin(), tmpUnderPlanes, sizeof(PxPlane)*underPlanesCount);
	return punder;
}

bool physx::computeOBBFromConvex(const PxConvexMeshDesc& desc, PxVec3& sides, PxTransform& matrix)
{
	PxIntegrals integrals;
	// using the centroid of the convex for the volume integration solved accuracy issues in cases where the inertia tensor
	// ended up close to not being positive definite and after a few further transforms the diagonalized inertia tensor ended
	// up with negative values.

	const PxVec3* verts = (reinterpret_cast<const PxVec3*>(desc.points.data));
	const PxU32* ind = (reinterpret_cast<const PxU32*>(desc.indices.data));
	const PxHullPolygon* polygons = (reinterpret_cast<const PxHullPolygon*>(desc.polygons.data));
	PxVec3 mean(0.0f);
	for (PxU32 i = 0; i < desc.points.count; i++)
		mean += verts[i];
	mean *= (1.0f / desc.points.count);

	PxU8* indices = reinterpret_cast<PxU8*> (PX_ALLOC_TEMP(sizeof(PxU8)*desc.indices.count, "PxU8"));
	for (PxU32 i = 0; i < desc.indices.count; i++)
	{
		indices[i] = Ps::to8(ind[i]);
	}
	// we need to move the polygon data to internal format
	Gu::HullPolygonData* polygonData = reinterpret_cast<Gu::HullPolygonData*> (PX_ALLOC_TEMP(sizeof(Gu::HullPolygonData)*desc.polygons.count, "Gu::HullPolygonData"));
	for (PxU32 i = 0; i < desc.polygons.count; i++)
	{
		polygonData[i].mPlane = PxPlane(polygons[i].mPlane[0], polygons[i].mPlane[1], polygons[i].mPlane[2], polygons[i].mPlane[3]);
		polygonData[i].mNbVerts = Ps::to8(polygons[i].mNbVerts);
		polygonData[i].mVRef8 = polygons[i].mIndexBase;
	}

	PxConvexMeshDesc inDesc;
	inDesc.points.data = desc.points.data;
	inDesc.points.count = desc.points.count;

	inDesc.polygons.data = polygonData;
	inDesc.polygons.count = desc.polygons.count;

	inDesc.indices.data = indices;
	inDesc.indices.count = desc.indices.count;

	// compute volume integrals to get basis axis
	bool status = (desc.flags & PxConvexFlag::eFAST_INERTIA_COMPUTATION) ? 
		computeVolumeIntegralsEberlySIMD(inDesc, 1.0f, integrals, mean) : computeVolumeIntegralsEberly(inDesc, 1.0f, integrals, mean);
	if (status)
	{
		Vec4V* pointsV = reinterpret_cast<Vec4V*> (PX_ALLOC_TEMP(sizeof(Vec4V)*desc.points.count, "Vec4V"));
		for (PxU32 i = 0; i < desc.points.count; i++)
		{
			// safe to V4 load, same as volume integration - we allocate one more vector
			pointsV[i] = V4LoadU(&verts[i].x);
		}

		PxMat33 inertia;
		integrals.getOriginInertia(inertia);
		PxQuat inertiaQuat;
		PxDiagonalize(inertia, inertiaQuat);
		PxMat33 baseAxis(inertiaQuat);
		Vec4V center = V4LoadU(&integrals.COM.x);

		const PxU32 numSteps = 20;
		const float subStep = Ps::degToRad(float(360/numSteps));

		float bestVolume = 1e9;		

		for (PxU32 axis = 0; axis < 3; axis++)
		{
			for (PxU32 iStep = 0; iStep < numSteps; iStep++)
			{
				PxQuat quat(iStep*subStep, baseAxis[axis]);

				Vec4V transV = center;
				Vec4V psidesV;

				const QuatV rotV = QuatVLoadU(&quat.x);
				local::computeOBBSIMD(desc.points.count, pointsV, psidesV, rotV, transV);

				PxVec3 psides;
				V3StoreU(Vec3V_From_Vec4V(psidesV), psides);

				const float volume = psides[0] * psides[1] * psides[2]; // the volume of the cube

				if (volume <= bestVolume)
				{
					bestVolume = volume;
					sides = psides;					

					V4StoreU(rotV, &matrix.q.x);
					V3StoreU(Vec3V_From_Vec4V(transV), matrix.p);
				}
			}
		}

		PX_FREE_AND_RESET(pointsV);
	}
	else
	{
		PX_FREE_AND_RESET(indices);
		PX_FREE_AND_RESET(polygonData);
		return false;
	}

	PX_FREE_AND_RESET(indices);
	PX_FREE_AND_RESET(polygonData);
	return true;
}
