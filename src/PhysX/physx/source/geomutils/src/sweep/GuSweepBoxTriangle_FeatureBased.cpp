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

#include "foundation/PxBounds3.h"
#include "GuSweepBoxTriangle_FeatureBased.h"
#include "GuIntersectionRayBox.h"
#include "PxTriangle.h"
#include "GuSweepTriangleUtils.h"
#include "GuInternal.h"

using namespace physx;
using namespace Gu;

#define LOCAL_EPSILON 0.00001f	// PT: this value makes the 'basicAngleTest' pass. Fails because of a ray almost parallel to a triangle

namespace
{
static const PxReal gFatTriangleCoeff = 0.02f;

static const PxVec3 gNearPlaneNormal[] = 
{
	PxVec3(1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, 1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, 1.0f),
	PxVec3(-1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, -1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, -1.0f)
};
}

#define	INVSQRT3 0.577350269189f	//!< 1 / sqrt(3)

/**
Returns vertex normals.
\return		24 floats (8 normals)
*/
static const PxF32* getBoxVertexNormals()
{
	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	static PxF32 VertexNormals[] = 
	{
		-INVSQRT3,	-INVSQRT3,	-INVSQRT3,
		INVSQRT3,	-INVSQRT3,	-INVSQRT3,
		INVSQRT3,	INVSQRT3,	-INVSQRT3,
		-INVSQRT3,	INVSQRT3,	-INVSQRT3,
		-INVSQRT3,	-INVSQRT3,	INVSQRT3,
		INVSQRT3,	-INVSQRT3,	INVSQRT3,
		INVSQRT3,	INVSQRT3,	INVSQRT3,
		-INVSQRT3,	INVSQRT3,	INVSQRT3
	};

	return VertexNormals;
}

static PxTriangle inflateTriangle(const PxTriangle& triangle, PxReal fat_coeff)
{
	PxTriangle fatTri = triangle;

	// Compute triangle center
	const PxVec3& p0 = triangle.verts[0];
	const PxVec3& p1 = triangle.verts[1];
	const PxVec3& p2 = triangle.verts[2];
	const PxVec3 center = (p0 + p1 + p2)*0.333333333f;

	// Don't normalize?
	// Normalize => add a constant border, regardless of triangle size
	// Don't => add more to big triangles
	for(PxU32 i=0;i<3;i++)
	{
		const PxVec3 v = fatTri.verts[i] - center;
		fatTri.verts[i] += v * fat_coeff;
	}
	return fatTri;
}

// PT: special version to fire N parallel rays against the same tri
static PX_FORCE_INLINE Ps::IntBool rayTriPrecaCull(	const PxVec3& orig, const PxVec3& dir,
													const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, const PxVec3& pvec,
													PxReal det, PxReal oneOverDet, PxReal& t)
{
	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	PxReal u = tvec.dot(pvec);
	if((u < 0.0f) || u>det)
		return 0;

	// Prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	PxReal v = dir.dot(qvec);
	if((v < 0.0f) || u+v>det)
		return 0;

	// Calculate t, scale parameters, ray intersects triangle
	t = edge2.dot(qvec);
	t *= oneOverDet;
	return 1;
}

static PX_FORCE_INLINE Ps::IntBool rayTriPrecaNoCull(	const PxVec3& orig, const PxVec3& dir,
														const PxVec3& vert0, const PxVec3& edge1, const PxVec3& edge2, const PxVec3& pvec,
														PxReal /*det*/, PxReal oneOverDet, PxReal& t)
{
	// Calculate distance from vert0 to ray origin
	const PxVec3 tvec = orig - vert0;

	// Calculate U parameter and test bounds
	PxReal u = (tvec.dot(pvec)) * oneOverDet;
	if((u < 0.0f) || u>1.0f)
		return 0;

	// prepare to test V parameter
	const PxVec3 qvec = tvec.cross(edge1);

	// Calculate V parameter and test bounds
	PxReal v = (dir.dot(qvec)) * oneOverDet;
	if((v < 0.0f) || u+v>1.0f)
		return 0;

	// Calculate t, ray intersects triangle
	t = (edge2.dot(qvec)) * oneOverDet;
	return 1;
}

// PT: specialized version where oneOverDir is available
// PT: why did we change the initial epsilon value?
#define LOCAL_EPSILON_RAY_BOX PX_EPS_F32
//#define LOCAL_EPSILON_RAY_BOX 0.0001f
static PX_FORCE_INLINE int intersectRayAABB2(const PxVec3& minimum, const PxVec3& maximum,
							const PxVec3& ro, const PxVec3& /*rd*/, const PxVec3& oneOverDir,
							float& tnear, float& tfar,
							bool fbx, bool fby, bool fbz)
{
	// PT: this unrolled loop is a lot faster on Xbox

	if(fbx)
		if(ro.x<minimum.x || ro.x>maximum.x)
		{
//			tnear = FLT_MAX;
			return -1;
		}
	if(fby)
		if(ro.y<minimum.y || ro.y>maximum.y)
		{
//			tnear = FLT_MAX;
			return -1;
		}
	if(fbz)
		if(ro.z<minimum.z || ro.z>maximum.z)
		{
//			tnear = FLT_MAX;
			return -1;
		}

	PxReal t1x = (minimum.x - ro.x) * oneOverDir.x;
	PxReal t2x = (maximum.x - ro.x) * oneOverDir.x;
	PxReal t1y = (minimum.y - ro.y) * oneOverDir.y;
	PxReal t2y = (maximum.y - ro.y) * oneOverDir.y;
	PxReal t1z = (minimum.z - ro.z) * oneOverDir.z;
	PxReal t2z = (maximum.z - ro.z) * oneOverDir.z;

	int bx;
	int by;
	int bz;

	if(t1x>t2x)
	{
		PxReal t=t1x; t1x=t2x; t2x=t;
		bx = 3;
	}
	else
	{
		bx = 0;
	}

	if(t1y>t2y)
	{
		PxReal t=t1y; t1y=t2y; t2y=t;
		by = 4;
	}
	else
	{
		by = 1;
	}

	if(t1z>t2z)
	{
		PxReal t=t1z; t1z=t2z; t2z=t;
		bz = 5;
	}
	else
	{
		bz = 2;
	}

	int ret;
	if(!fbx)
	{
//		if(t1x>tnear)	// PT: no need to test for the first value
		{
			tnear = t1x;
			ret = bx;
		}
//		tfar = Px::intrinsics::selectMin(tfar, t2x);
		tfar = t2x;		// PT: no need to test for the first value
	}
	else
	{
		ret=-1;
		tnear = -PX_MAX_F32;
		tfar = PX_MAX_F32;
	}

	if(!fby)
	{
		if(t1y>tnear)
		{
			tnear = t1y;
			ret = by;
		}
		tfar = physx::intrinsics::selectMin(tfar, t2y);
	}

	if(!fbz)
	{
		if(t1z>tnear)
		{
			tnear = t1z;
			ret = bz;
		}
		tfar = physx::intrinsics::selectMin(tfar, t2z);
	}

	if(tnear>tfar || tfar<LOCAL_EPSILON_RAY_BOX)
		return -1;

	return ret;
}

// PT: force-inlining this saved 500.000 cycles in the benchmark. Ok to inline, only used once anyway.
static PX_FORCE_INLINE bool intersectEdgeEdge3(const PxPlane& plane, const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& v1,
						const PxVec3& p3, const PxVec3& p4,
						PxReal& dist, PxVec3& ip, PxU32 i, PxU32 j, const PxReal coeff)
{
	// if colliding edge (p3,p4) does not cross plane return no collision
	// same as if p3 and p4 on same side of plane return 0
	//
	// Derivation:
	// d3 = d(p3, P) = (p3 | plane.n) - plane.d;		Reversed sign compared to Plane::Distance() because plane.d is negated.
	// d4 = d(p4, P) = (p4 | plane.n) - plane.d;		Reversed sign compared to Plane::Distance() because plane.d is negated.
	// if d3 and d4 have the same sign, they're on the same side of the plane => no collision
	// We test both sides at the same time by only testing Sign(d3 * d4).
	// ### put that in the Plane class
	// ### also check that code in the triangle class that might be similar
	const PxReal d3 = plane.distance(p3);

	const PxReal temp = d3 * plane.distance(p4);
	if(temp>0.0f)	return false;

	// if colliding edge (p3,p4) and plane are parallel return no collision
	const PxVec3 v2 = p4 - p3;

	const PxReal temp2 = plane.n.dot(v2);
	if(temp2==0.0f)	return false;	// ### epsilon would be better

	// compute intersection point of plane and colliding edge (p3,p4)
	ip = p3-v2*(d3/temp2);

	// compute distance of intersection from line (ip, -dir) to line (p1,p2)
	dist =	(v1[i]*(ip[j]-p1[j])-v1[j]*(ip[i]-p1[i])) * coeff;
	if(dist<0.0f)	return false;

	// compute intersection point on edge (p1,p2) line
	ip -= dist*dir;

	// check if intersection point (ip) is between edge (p1,p2) vertices
	const PxReal temp3 = (p1.x-ip.x)*(p2.x-ip.x)+(p1.y-ip.y)*(p2.y-ip.y)+(p1.z-ip.z)*(p2.z-ip.z);
	return temp3<0.0f;
}

namespace
{
static const PxReal gFatBoxEdgeCoeff = 0.01f;
#define	INVSQRT2 0.707106781188f	//!< 1 / sqrt(2)

static const PxVec3 EdgeNormals[] = 
{
	PxVec3(0,			-INVSQRT2,	-INVSQRT2),	// 0-1
	PxVec3(INVSQRT2,	0,			-INVSQRT2),	// 1-2
	PxVec3(0,			INVSQRT2,	-INVSQRT2),	// 2-3
	PxVec3(-INVSQRT2,	0,			-INVSQRT2),	// 3-0

	PxVec3(0,			INVSQRT2,	INVSQRT2),	// 7-6
	PxVec3(INVSQRT2,	0,			INVSQRT2),	// 6-5
	PxVec3(0,			-INVSQRT2,	INVSQRT2),	// 5-4
	PxVec3(-INVSQRT2,	0,			INVSQRT2),	// 4-7

	PxVec3(INVSQRT2,	-INVSQRT2,	0),			// 1-5
	PxVec3(INVSQRT2,	INVSQRT2,	0),			// 6-2
	PxVec3(-INVSQRT2,	INVSQRT2,	0),			// 3-7
	PxVec3(-INVSQRT2,	-INVSQRT2,	0)			// 4-0
};

static const PxVec3* getBoxLocalEdgeNormals()
{
	return EdgeNormals;
}
}

static PX_FORCE_INLINE void closestAxis2(const PxVec3& v, PxU32& j, PxU32& k)
{
	// find largest 2D plane projection
	const PxF32 absPx = physx::intrinsics::abs(v.x);
	const PxF32 absPy = physx::intrinsics::abs(v.y);
	const PxF32 absPz = physx::intrinsics::abs(v.z);
	//PxU32 m = 0;	//x biggest axis
	j = 1;
	k = 2;
	if( absPy > absPx && absPy > absPz)
	{
		//y biggest
		j = 2;
		k = 0;
		//m = 1;
	}
	else if(absPz > absPx)
	{
		//z biggest
		j = 0;
		k = 1;
		//m = 2;
	}
//		return m;
}

bool Gu::sweepBoxTriangle(	const PxTriangle& tri, const PxBounds3& box,
							const PxVec3& motion, const PxVec3& oneOverMotion,
							PxVec3& hit, PxVec3& normal, PxReal& d, bool isDoubleSided)
{
	// Create triangle normal
	PxVec3 denormalizedTriNormal;
	tri.denormalizedNormal(denormalizedTriNormal);

	// Backface culling
	const bool doBackfaceCulling = !isDoubleSided;
	if(doBackfaceCulling && (denormalizedTriNormal.dot(motion)) >= 0.0f)	// ">=" is important !
		return false;

	/////////////////////////

	PxVec3 boxVertices[8];
	computeBoxPoints(box, boxVertices);

	/////////////////////////

	// Make fat triangle
	const PxTriangle fatTri = inflateTriangle(tri, gFatTriangleCoeff);

	PxReal minDist = d;	// Initialize with current best distance
	int col = -1;

	// Box vertices VS triangle
	{
		// ### cull using box-plane distance ?
		const PxVec3 edge1 = fatTri.verts[1] - fatTri.verts[0];
		const PxVec3 edge2 = fatTri.verts[2] - fatTri.verts[0];
		const PxVec3 PVec = motion.cross(edge2);
		const PxReal Det = edge1.dot(PVec);

		// We can't use stamps here since we can still find a better TOI for a given vertex,
		// even if that vertex has already been tested successfully against another triangle.
		const PxVec3* VN = reinterpret_cast<const PxVec3*>(getBoxVertexNormals());

		const PxReal oneOverDet = Det!=0.0f ? 1.0f / Det : 0.0f;

		PxU32 hitIndex=0;
		if(doBackfaceCulling)
		{
			if(Det>=LOCAL_EPSILON)
			{
				for(PxU32 i=0;i<8;i++)
				{
					// Orientation culling
					if((VN[i].dot(denormalizedTriNormal) >= 0.0f))	// Can't rely on triangle normal for double-sided faces
						continue;

					// ### test this
					// ### ok, this causes the bug in level3's v-shaped desk. Not really a real "bug", it just happens
					// that this VF test fixes this case, so it's a bad idea to cull it. Oh, well.
					// If we use a penetration-depth code to fixup bad cases, we can enable this culling again. (also
					// if we find a better way to handle that desk)
					// Discard back vertices
//					if(VN[i].dot(motion)<0.0f)
//						continue;

					// Shoot a ray from vertex against triangle, in direction "motion"
					PxReal t;
					if(!rayTriPrecaCull(boxVertices[i], motion, fatTri.verts[0], edge1, edge2, PVec, Det, oneOverDet, t))
						continue;

					//if(t<=OffsetLength)	t=0.0f;
					// Only consider positive distances, closer than current best
					// ### we could test that first on tri vertices & discard complete tri if it's further than current best (or equal!)
					if(t < 0.0f || t > minDist)
						continue;

					minDist = t;
					col = 0;
//					hit = boxVertices[i] + t * motion;
					hitIndex = i;
				}
			}
		}
		else
		{
			if(Det<=-LOCAL_EPSILON || Det>=LOCAL_EPSILON)
			{
				for(PxU32 i=0;i<8;i++)
				{
					// ### test this
					// ### ok, this causes the bug in level3's v-shaped desk. Not really a real "bug", it just happens
					// that this VF test fixes this case, so it's a bad idea to cull it. Oh, well.
					// If we use a penetration-depth code to fixup bad cases, we can enable this culling again. (also
					// if we find a better way to handle that desk)
					// Discard back vertices
					//			if(!VN[i].SameDirection(motion))
					//				continue;

					// Shoot a ray from vertex against triangle, in direction "motion"
					PxReal t;
					if(!rayTriPrecaNoCull(boxVertices[i], motion, fatTri.verts[0], edge1, edge2, PVec, Det, oneOverDet, t))
						continue;

					//if(t<=OffsetLength)	t=0.0f;
					// Only consider positive distances, closer than current best
					// ### we could test that first on tri vertices & discard complete tri if it's further than current best (or equal!)
					if(t < 0.0f || t > minDist)
						continue;

					minDist = t;
					col = 0;
//					hit = boxVertices[i] + t * motion;
					hitIndex = i;
				}
			}
		}

		// Only copy this once, if needed
		if(col==0)
		{
			// PT: hit point on triangle
			hit = boxVertices[hitIndex] + minDist * motion;
			normal = denormalizedTriNormal;
		}
	}

	// Triangle vertices VS box
	{
		const PxVec3 negMotion = -motion;
		const PxVec3 negInvMotion = -oneOverMotion;

		// PT: precompute fabs-test for ray-box
		// - doing this outside of the ray-box function gets rid of 3 fabs/fcmp per call
		// - doing this with integer code removes the 3 remaining fabs/fcmps totally
		// - doing this outside reduces the LHS
		const bool b0 = physx::intrinsics::abs(negMotion.x)<LOCAL_EPSILON_RAY_BOX;
		const bool b1 = physx::intrinsics::abs(negMotion.y)<LOCAL_EPSILON_RAY_BOX;
		const bool b2 = physx::intrinsics::abs(negMotion.z)<LOCAL_EPSILON_RAY_BOX;

		// ### have this as a param ?
		const PxVec3& Min = box.minimum;
		const PxVec3& Max = box.maximum;

		const PxVec3* boxNormals = gNearPlaneNormal;

		// ### use stamps not to shoot shared vertices multiple times
		// ### discard non-convex verts
		for(PxU32 i=0;i<3;i++)
		{
			PxReal tnear, tfar;
			const int plane = ::intersectRayAABB2(Min, Max, tri.verts[i], negMotion, negInvMotion, tnear, tfar, b0, b1, b2);
			PX_ASSERT(plane == intersectRayAABB(Min, Max, tri.verts[i], negMotion, tnear, tfar));

			// The following works as well but we need to call "intersectRayAABB" to get a plane index compatible with BoxNormals.
			// We could fix this by unifying the plane indices returned by the different ray-aabb functions...
			//PxVec3 coord;
			//PxReal t;
			//PxU32 status = rayAABBIntersect2(Min, Max, tri.verts[i], -motion, coord, t);

			// ### don't test -1 ?
			if(plane==-1 || tnear<0.0f)	continue;
//			if(tnear<0.0f)	continue;
			if(tnear <= minDist)
			{
				minDist = tnear;	// ### warning, tnear => flips normals
				normal = boxNormals[plane];
				col = 1;

				// PT: hit point on triangle
				hit = tri.verts[i];
			}
		}
	}

	PxU32 saved_j = PX_INVALID_U32;
	PxU32 saved_k = PX_INVALID_U32;
	PxVec3 p1s;
	PxVec3 v1s;

	// Edge-vs-edge
	{
		// Loop through box edges
		const PxU8* PX_RESTRICT edges = getBoxEdges();
		const PxVec3* PX_RESTRICT edgeNormals = getBoxLocalEdgeNormals();
		for(PxU32 i=0;i<12;i++)	// 12 edges
		{
			// PT: TODO: skip this if edge is culled
			PxVec3 p1 = boxVertices[*edges++];
			PxVec3 p2 = boxVertices[*edges++];
			Ps::makeFatEdge(p1, p2, gFatBoxEdgeCoeff);

			if(edgeNormals[i].dot(motion) < 0.0f)
				continue;

			// While we're at it, precompute some more data for EE tests
			const PxVec3 v1 = p2 - p1;

			// Build plane P based on edge (p1, p2) and direction (dir)
			const PxVec3 planeNormal = v1.cross(motion);
			const PxPlane plane(planeNormal, -(planeNormal.dot(p1)));

			// find largest 2D plane projection
			PxU32 closest_i, closest_j;
		//	Ps::closestAxis(plane.normal, ii, jj);
			closestAxis2(planeNormal, closest_i, closest_j);

			const PxReal coeff = 1.0f / (v1[closest_i]*motion[closest_j] - v1[closest_j]*motion[closest_i]);

			// Loop through triangle edges
			for(PxU32 j=0; j<3; j++)
			{
				// Catch current triangle edge
				// j=0 => 0-1
				// j=1 => 1-2
				// j=2 => 2-0
				// => this is compatible with EdgeList
				const PxU32 k = Ps::getNextIndex3(j);

				PxReal dist;
				PxVec3 ip;
				if(intersectEdgeEdge3(plane, p1, p2, motion, v1, tri.verts[j], tri.verts[k], dist, ip, closest_i, closest_j, coeff))
				{
					if(dist<=minDist)
					{
						p1s = p1;
						v1s = v1;
						saved_j = j;
						saved_k = k;

						col = 2;
						minDist = dist;

						// PT: hit point on triangle
						hit = ip + motion*dist;
					}
				}
			}
		}
	}

	if(col==-1)
		return false;

	if(col==2)
	{
		PX_ASSERT(saved_j != PX_INVALID_U32);
		PX_ASSERT(saved_k != PX_INVALID_U32);
		const PxVec3& p3 = tri.verts[saved_j];
		const PxVec3& p4 = tri.verts[saved_k];
		computeEdgeEdgeNormal(normal, p1s, v1s, p3, p4-p3, motion, minDist);
	}

	d = minDist;
	return true;
}
