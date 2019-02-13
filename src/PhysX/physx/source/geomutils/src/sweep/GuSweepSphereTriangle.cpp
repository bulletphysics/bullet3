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

#include "GuSweepSphereTriangle.h"
#include "GuIntersectionRaySphere.h"
#include "GuIntersectionRayCapsule.h"
#include "GuIntersectionRayTriangle.h"
#include "GuCapsule.h"
#include "GuInternal.h"
#include "PsUtilities.h"
#include "GuDistancePointTriangle.h"

using namespace physx;
using namespace Gu;

// PT: using GU_CULLING_EPSILON_RAY_TRIANGLE fails here, in capsule-vs-mesh's triangle extrusion, when
// the sweep dir is almost the same as the capsule's dir (i.e. when we usually fallback to the sphere codepath).
// I suspect det becomes so small that we lose all accuracy when dividing by det and using the result in computing
// impact distance.
#define LOCAL_EPSILON 0.00001f

// PT: special version computing (u,v) even when the ray misses the tri. Version working on precomputed edges.
static PX_FORCE_INLINE PxU32 rayTriSpecial(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, PxReal& t, PxReal& u, PxReal& v)
{
	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	// the non-culling branch
//	if(det>-GU_CULLING_EPSILON_RAY_TRIANGLE && det<GU_CULLING_EPSILON_RAY_TRIANGLE)
	if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)
		return 0;
	const PxReal oneOverDet = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter
	u = (tvec.dot(pvec)) * oneOverDet;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter
	v = (dir.dot(qvec)) * oneOverDet;

	if(u<0.0f || u>1.0f)
		return 1;
	if(v<0.0f || u+v>1.0f)
		return 1;

	// Calculate t, ray intersects triangle
	t = (edge2.dot(qvec)) * oneOverDet;

	return 2;
}

// Returns true if sphere can be tested against triangle vertex, false if edge test should be performed
//
// Uses a conservative approach to work for "sliver triangles" (long & thin) as well.
static PX_FORCE_INLINE bool edgeOrVertexTest(const PxVec3& planeIntersectPoint, const PxVec3* PX_RESTRICT tri, PxU32 vertIntersectCandidate, PxU32 vert0, PxU32 vert1, PxU32& secondEdgeVert)
{
	{
		const PxVec3 edge0 = tri[vertIntersectCandidate] - tri[vert0];
		const PxReal edge0LengthSqr = edge0.dot(edge0);

		const PxVec3 diff = planeIntersectPoint - tri[vert0];

		if (edge0.dot(diff) < edge0LengthSqr)  // If the squared edge length is used for comparison, the edge vector does not need to be normalized
		{
			secondEdgeVert = vert0;
			return false;
		}
	}

	{
		const PxVec3 edge1 = tri[vertIntersectCandidate] - tri[vert1];
		const PxReal edge1LengthSqr = edge1.dot(edge1);

		const PxVec3 diff = planeIntersectPoint - tri[vert1];

		if (edge1.dot(diff) < edge1LengthSqr)
		{
			secondEdgeVert = vert1;
			return false;
		}
	}
	return true;
}

static PX_FORCE_INLINE bool testRayVsSphereOrCapsule(PxReal& impactDistance, bool testSphere, const PxVec3& center, PxReal radius, const PxVec3& dir, const PxVec3* PX_RESTRICT verts, PxU32 e0, PxU32 e1)
{
	if(testSphere)
	{
		PxReal t;
		if(intersectRaySphere(center, dir, PX_MAX_F32, verts[e0], radius, t))
		{
			impactDistance = t;
			return true;
		}
	}
	else
	{
		PxReal t;
		if(intersectRayCapsule(center, dir, verts[e0], verts[e1], radius, t))
		{
			if(t>=0.0f/* && t<MinDist*/)
			{
				impactDistance = t;
				return true;
			}
		}
	}
	return false;
}

bool Gu::sweepSphereVSTri(const PxVec3* PX_RESTRICT triVerts, const PxVec3& normal, const PxVec3& center, PxReal radius, const PxVec3& dir, PxReal& impactDistance, bool& directHit, bool testInitialOverlap)
{
	// Ok, this new version is now faster than the original code. Needs more testing though.

	directHit = false;
	const PxVec3 edge10 = triVerts[1] - triVerts[0];
	const PxVec3 edge20 = triVerts[2] - triVerts[0];

	if(testInitialOverlap)	// ### brute force version that always works, but we can probably do better
	{
		const PxVec3 cp = closestPtPointTriangle2(center, triVerts[0], triVerts[1], triVerts[2], edge10, edge20);
		if((cp - center).magnitudeSquared() <= radius*radius)
		{
			impactDistance = 0.0f;
			return true;
		}
	}

	#define INTERSECT_POINT (triVerts[1]*u) + (triVerts[2]*v) + (triVerts[0] * (1.0f-u-v))

	PxReal u,v;
	{
		PxVec3 R = normal * radius;
		if(dir.dot(R) >= 0.0f)
			R = -R;

		// The first point of the sphere to hit the triangle plane is the point of the sphere nearest to
		// the triangle plane. Hence, we use center - (normal*radius) below.

		// PT: casting against the extruded triangle in direction R is the same as casting from a ray moved by -R
		PxReal t;
		const PxU32 r = rayTriSpecial(center-R, dir, triVerts[0], edge10, edge20, t, u, v);
		if(!r)
			return false;
		if(r==2)
		{
			if(t<0.0f)
				return false;
			impactDistance = t;
			directHit = true;
			return true;
		}
	}

	//
	// Let's do some art!
	//
	// The triangle gets divided into the following areas (based on the barycentric coordinates (u,v)):
	//
	//               \   A0    /
	//                 \      /
	//                   \   /
	//                     \/ 0
	//            A02      *      A01
	//   u /              /   \          \ v
	//    *              /      \         *
	//                  /         \						.
	//               2 /            \ 1
	//          ------*--------------*-------
	//               /                 \				.
	//        A2    /        A12         \   A1
	//
	//
	// Based on the area where the computed triangle plane intersection point lies in, a different sweep test will be applied.
	//
	// A) A01, A02, A12  : Test sphere against the corresponding edge
	// B) A0, A1, A2     : Test sphere against the corresponding vertex
	//
	// Unfortunately, B) does not work for long, thin triangles. Hence there is some extra code which does a conservative check and
	// switches to edge tests if necessary.
	//

	bool TestSphere;
	PxU32 e0,e1;
	if(u<0.0f)
	{
		if(v<0.0f)
		{
			// 0 or 0-1 or 0-2
			e0 = 0;
			const PxVec3 intersectPoint = INTERSECT_POINT;
			TestSphere = edgeOrVertexTest(intersectPoint, triVerts, 0, 1, 2, e1);
		}
		else if(u+v>1.0f)
		{
			// 2 or 2-0 or 2-1
			e0 = 2;
			const PxVec3 intersectPoint = INTERSECT_POINT;
			TestSphere = edgeOrVertexTest(intersectPoint, triVerts, 2, 0, 1, e1);
		}
		else
		{
			// 0-2
			TestSphere = false;
			e0 = 0;
			e1 = 2;
		}
	}
	else
	{
		if(v<0.0f)
		{
			if(u+v>1.0f)
			{
				// 1 or 1-0 or 1-2
				e0 = 1;
				const PxVec3 intersectPoint = INTERSECT_POINT;
				TestSphere = edgeOrVertexTest(intersectPoint, triVerts, 1, 0, 2, e1);
			}
			else
			{
				// 0-1
				TestSphere = false;
				e0 = 0;
				e1 = 1;
			}
		}
		else
		{
			PX_ASSERT(u+v>=1.0f);	// Else hit triangle
			// 1-2
			TestSphere = false;
			e0 = 1;
			e1 = 2;
		}
	}
	return testRayVsSphereOrCapsule(impactDistance, TestSphere, center, radius, dir, triVerts, e0, e1);
}

bool Gu::sweepSphereTriangles(	PxU32 nbTris, const PxTriangle* PX_RESTRICT triangles,							// Triangle data
								const PxVec3& center, const PxReal radius,										// Sphere data
								const PxVec3& unitDir, PxReal distance,											// Ray data
								const PxU32* PX_RESTRICT cachedIndex,											// Cache data
								PxSweepHit& h, PxVec3& triNormalOut,												// Results
								bool isDoubleSided, bool meshBothSides, bool anyHit, bool testInitialOverlap)	// Query modifiers
{
	if(!nbTris)
		return false;

	const bool doBackfaceCulling = !isDoubleSided && !meshBothSides;

	PxU32 index = PX_INVALID_U32;
	const PxU32 initIndex = getInitIndex(cachedIndex, nbTris);

	PxReal curT = distance;
	const PxReal dpc0 = center.dot(unitDir);

	PxReal bestAlignmentValue = 2.0f;

	PxVec3 bestTriNormal(0.0f);

	for(PxU32 ii=0; ii<nbTris; ii++)	// We need i for returned triangle index
	{
		const PxU32 i = getTriangleIndex(ii, initIndex);

		const PxTriangle& currentTri = triangles[i];

		if(rejectTriangle(center, unitDir, curT, radius, currentTri.verts, dpc0))
			continue;

		PxVec3 triNormal;
		currentTri.denormalizedNormal(triNormal);

		// Backface culling
		if(doBackfaceCulling && (triNormal.dot(unitDir) > 0.0f))
			continue;

		const PxReal magnitude = triNormal.magnitude();
		if(magnitude==0.0f)
			continue;

		triNormal /= magnitude;

		PxReal currentDistance;
		bool unused;
		if(!sweepSphereVSTri(currentTri.verts, triNormal, center, radius, unitDir, currentDistance, unused, testInitialOverlap))
			continue;

		const PxReal distEpsilon = GU_EPSILON_SAME_DISTANCE; // pick a farther hit within distEpsilon that is more opposing than the previous closest hit
		const PxReal hitDot = computeAlignmentValue(triNormal, unitDir);
		if(!keepTriangle(currentDistance, hitDot, curT, bestAlignmentValue, distance, distEpsilon))
			continue;

		if(currentDistance==0.0f)
		{
			triNormalOut = -unitDir;
			return setInitialOverlapResults(h, unitDir, i);
		}

		curT = currentDistance;
		index = i;		
		bestAlignmentValue = hitDot; 
		bestTriNormal = triNormal;
		if(anyHit)
			break;
	}	
	return computeSphereTriangleImpactData(h, triNormalOut, index, curT, center, unitDir, bestTriNormal, triangles, isDoubleSided, meshBothSides);
}

static PX_FORCE_INLINE PxU32 rayQuadSpecial2(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, float& t, float& u, float& v)
{
	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const float det = edge1.dot(pvec);

	// the non-culling branch
	if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)
		return 0;
	const float OneOverDet = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter
	u = tvec.dot(pvec) * OneOverDet;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter
	v = dir.dot(qvec) * OneOverDet;

	if(u<0.0f || u>1.0f)
		return 1;
	if(v<0.0f || v>1.0f)
		return 1;

	// Calculate t, ray intersects triangle
	t = edge2.dot(qvec) * OneOverDet;

	return 2;
}

bool Gu::sweepSphereVSQuad(const PxVec3* PX_RESTRICT quadVerts, const PxVec3& normal, const PxVec3& center, float radius, const PxVec3& dir, float& impactDistance)
{
	// Quad formed by 2 tris:
	// p0 p1 p2
	// p2 p1 p3 = p3 p2 p1
	//
	//	p0___p2
	//	|   /|
	//	|  / |
	//	| /  |
	//	|/   |
	//	p1---p3
	//
	// Edge10 = p1 - p0
	// Edge20 = p2 - p0
	// Impact point = Edge10*u + Edge20*v + p0
	// => u is along Y, between 0.0 (p0;p2) and 1.0 (p1;p3)
	// => v is along X, between 0.0 (p0;p1) and 1.0 (p2;p3)
	//
	// For the second triangle,
	// Edge10b = p2 - p3 = -Edge10
	// Edge20b = p1 - p3 = -Edge20

	const PxVec3 Edge10 = quadVerts[1] - quadVerts[0];
	const PxVec3 Edge20 = quadVerts[2] - quadVerts[0];

	if(1)	// ### brute force version that always works, but we can probably do better
	{
		const float r2 = radius*radius;
		{
			const PxVec3 Cp = closestPtPointTriangle2(center, quadVerts[0], quadVerts[1], quadVerts[2], Edge10, Edge20);
			if((Cp - center).magnitudeSquared() <= r2)
			{
				impactDistance = 0.0f;
				return true;
			}
		}
		{
			const PxVec3 Cp = closestPtPointTriangle2(center, quadVerts[3], quadVerts[2], quadVerts[1], -Edge10, -Edge20);
			if((Cp - center).magnitudeSquared() <= r2)
			{
				impactDistance = 0.0f;
				return true;
			}
		}
	}

	float u,v;
	if(1)
	{
		PxVec3 R = normal * radius;
		if(dir.dot(R) >= 0.0f)
			R = -R;

		// The first point of the sphere to hit the quad plane is the point of the sphere nearest to
		// the quad plane. Hence, we use center - (normal*radius) below.

		// PT: casting against the extruded quad in direction R is the same as casting from a ray moved by -R
		float t;
		PxU32 r = rayQuadSpecial2(center-R, dir, quadVerts[0], Edge10, Edge20, t, u, v);
		if(!r)
			return false;
		if(r==2)
		{
			if(t<0.0f)
				return false;
			impactDistance = t;
			return true;
		}
	}

	#define INTERSECT_POINT_Q (quadVerts[1]*u) + (quadVerts[2]*v) + (quadVerts[0] * (1.0f-u-v))

	Ps::swap(u,v);
	bool TestSphere;
	PxU32 e0,e1;
	if(u<0.0f)
	{
		if(v<0.0f)
		{
			// 0 or 0-1 or 0-2
			e0 = 0;
			const PxVec3 intersectPoint = INTERSECT_POINT_Q;
			TestSphere = edgeOrVertexTest(intersectPoint, quadVerts, 0, 1, 2, e1);
		}
		else if(v>1.0f)
		{
			// 1 or 1-0 or 1-3
			e0 = 1;
			const PxVec3 intersectPoint = INTERSECT_POINT_Q;
			TestSphere = edgeOrVertexTest(intersectPoint, quadVerts, 1, 0, 3, e1);
		}
		else
		{
			// 0-1
			TestSphere = false;
			e0 = 0;
			e1 = 1;
		}
	}
	else if(u>1.0f)
	{
		if(v<0.0f)
		{
			// 2 or 2-0 or 2-3
			e0 = 2;
			const PxVec3 intersectPoint = INTERSECT_POINT_Q;
			TestSphere = edgeOrVertexTest(intersectPoint, quadVerts, 2, 0, 3, e1);
		}
		else if(v>1.0f)
		{
			// 3 or 3-1 or 3-2
			e0 = 3;
			const PxVec3 intersectPoint = INTERSECT_POINT_Q;
			TestSphere = edgeOrVertexTest(intersectPoint, quadVerts, 3, 1, 2, e1);
		}
		else
		{
			// 2-3
			TestSphere = false;
			e0 = 2;
			e1 = 3;
		}
	}
	else
	{
		if(v<0.0f)
		{
			// 0-2
			TestSphere = false;
			e0 = 0;
			e1 = 2;
		}
		else
		{
			PX_ASSERT(v>=1.0f);	// Else hit quad
			// 1-3
			TestSphere = false;
			e0 = 1;
			e1 = 3;
		}
	}
	return testRayVsSphereOrCapsule(impactDistance, TestSphere, center, radius, dir, quadVerts, e0, e1);
}

