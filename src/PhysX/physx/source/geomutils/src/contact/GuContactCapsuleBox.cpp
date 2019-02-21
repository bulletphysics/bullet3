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

#include "GuContactBuffer.h"
#include "GuIntersectionRayBox.h"
#include "GuDistanceSegmentBox.h"
#include "GuInternal.h"
#include "GuContactMethodImpl.h"
#include "PsMathUtils.h"
#include "PsUtilities.h"
#include "GuGeometryUnion.h"
#include "GuBoxConversion.h"

using namespace physx;
using namespace Gu;

/*namespace Gu
{
const PxU8* getBoxEdges();
}*/

/////////
	/*#include "CmRenderOutput.h"
	#include "PxsContext.h"
	static void gVisualizeBox(const Box& box, PxcNpThreadContext& context, PxU32 color=0xffffff)
	{
		PxMat33 rot(box.base.column0, box.base.column1, box.base.column2);
		PxMat44 m(rot, box.origin);

		DebugBox db(box.extent);

		Cm::RenderOutput& out = context.mRenderOutput;
		out << color << m;
		out << db;
	}
	static void gVisualizeLine(const PxVec3& a, const PxVec3& b, PxcNpThreadContext& context, PxU32 color=0xffffff)
	{
		PxMat44 m = PxMat44::identity();

		Cm::RenderOutput& out = context.mRenderOutput;
		out << color << m << Cm::RenderOutput::LINES << a << b;
	}*/
/////////


static const PxReal fatBoxEdgeCoeff = 0.01f;

static bool intersectEdgeEdgePreca(const PxVec3& p1, const PxVec3& p2, const PxVec3& v1, const PxPlane& plane, PxU32 i, PxU32 j, float coeff, const PxVec3& dir, const PxVec3& p3, const PxVec3& p4, PxReal& dist, PxVec3& ip)
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
	PxReal temp = d3 * plane.distance(p4);
	if(temp>0.0f)	return false;

	// if colliding edge (p3,p4) and plane are parallel return no collision
	PxVec3 v2 = p4 - p3;

	temp = plane.n.dot(v2);
	if(temp==0.0f)	return false;	// ### epsilon would be better

	// compute intersection point of plane and colliding edge (p3,p4)
	ip = p3-v2*(d3/temp);

	// compute distance of intersection from line (ip, -dir) to line (p1,p2)
	dist =	(v1[i]*(ip[j]-p1[j])-v1[j]*(ip[i]-p1[i]))*coeff;
	if(dist<0.0f)	return false;

	// compute intersection point on edge (p1,p2) line
	ip -= dist*dir;

	// check if intersection point (ip) is between edge (p1,p2) vertices
	temp = (p1.x-ip.x)*(p2.x-ip.x)+(p1.y-ip.y)*(p2.y-ip.y)+(p1.z-ip.z)*(p2.z-ip.z);
	if(temp<0.0f)	return true;	// collision found

	return false;	// no collision
}


static bool GuTestAxis(const PxVec3& axis, const Segment& segment, PxReal radius, const Box& box, PxReal& depth)
{
	// Project capsule
	PxReal min0 = segment.p0.dot(axis);
	PxReal max0 = segment.p1.dot(axis);
	if(min0>max0) Ps::swap(min0, max0);
	min0 -= radius;
	max0 += radius;

	// Project box
	PxReal Min1, Max1;
	{
		const PxReal BoxCen = box.center.dot(axis);
		const PxReal BoxExt = 
				PxAbs(box.rot.column0.dot(axis)) * box.extents.x
			+	PxAbs(box.rot.column1.dot(axis)) * box.extents.y
			+	PxAbs(box.rot.column2.dot(axis)) * box.extents.z;

		Min1 = BoxCen - BoxExt; 
		Max1 = BoxCen + BoxExt; 
	}

	// Test projections
	if(max0<Min1 || Max1<min0)
		return false;

	const PxReal d0 = max0 - Min1;
	PX_ASSERT(d0>=0.0f);
	const PxReal d1 = Max1 - min0;
	PX_ASSERT(d1>=0.0f);
	depth = physx::intrinsics::selectMin(d0, d1);
	return true;
}


static bool GuCapsuleOBBOverlap3(const Segment& segment, PxReal radius, const Box& box, PxReal* t=NULL, PxVec3* pp=NULL)
{
	PxVec3 Sep(PxReal(0));
	PxReal PenDepth = PX_MAX_REAL;

	// Test normals
	for(PxU32 i=0;i<3;i++)
	{
		PxReal d;
		if(!GuTestAxis(box.rot[i], segment, radius, box, d))
			return false;

		if(d<PenDepth)
		{
			PenDepth = d;
			Sep = box.rot[i];
		}
	}

	// Test edges
	PxVec3 CapsuleAxis(segment.p1 - segment.p0);
	CapsuleAxis = CapsuleAxis.getNormalized();
	for(PxU32 i=0;i<3;i++)
	{
		PxVec3 Cross = CapsuleAxis.cross(box.rot[i]);
		if(!Ps::isAlmostZero(Cross))
		{
			Cross = Cross.getNormalized();
			PxReal d;
			if(!GuTestAxis(Cross, segment, radius, box, d))
				return false;

			if(d<PenDepth)
			{
				PenDepth = d;
				Sep = Cross;
			}
		}
	}

	const PxVec3 Witness = segment.computeCenter() - box.center;

	if(Sep.dot(Witness) < 0.0f)
		Sep = -Sep;

	if(t)
		*t = PenDepth;
	if(pp)
		*pp = Sep;

	return true;
}


static void GuGenerateVFContacts(	ContactBuffer& contactBuffer,
									//
									const Segment& segment,
									const PxReal radius,
									//
									const Box& worldBox,
									//
									const PxVec3& normal,
									const PxReal contactDistance)
{
	const PxVec3 Max = worldBox.extents;
	const PxVec3 Min = -worldBox.extents;

	const PxVec3 tmp2 = - worldBox.rot.transformTranspose(normal);

	const PxVec3* PX_RESTRICT Ptr = &segment.p0;
	for(PxU32 i=0;i<2;i++)
	{
		const PxVec3& Pos = Ptr[i];

		const PxVec3 tmp = worldBox.rot.transformTranspose(Pos - worldBox.center);
		PxReal tnear, tfar;
		int Res = intersectRayAABB(Min, Max, tmp, tmp2, tnear, tfar);

		if(Res!=-1 && tnear < radius + contactDistance)
		{
			contactBuffer.contact(Pos - tnear * normal, normal, tnear - radius);
		}
	}
}


// PT: this looks similar to PxcGenerateEEContacts2 but it is mandatory to properly handle thin capsules.
static void GuGenerateEEContacts(	ContactBuffer& contactBuffer,
									//
									const Segment& segment,
									const PxReal radius,
									//
									const Box& worldBox,
									//
									const PxVec3& normal)
{
	const PxU8* PX_RESTRICT Indices = getBoxEdges();

	PxVec3 Pts[8];
	worldBox.computeBoxPoints(Pts);

	PxVec3 s0 = segment.p0;
	PxVec3 s1 = segment.p1;
	Ps::makeFatEdge(s0, s1, fatBoxEdgeCoeff);

	// PT: precomputed part of edge-edge intersection test
//	const PxVec3 v1 = segment.p1 - segment.p0;
	const PxVec3 v1 = s1 - s0;
	PxPlane plane;
	plane.n = v1.cross(normal);
//		plane.d = -(plane.normal|segment.p0);
	plane.d = -(plane.n.dot(s0));

	PxU32 ii,jj;
	Ps::closestAxis(plane.n, ii, jj);

	const float coeff = 1.0f /(v1[ii]*normal[jj]-v1[jj]*normal[ii]);


	for(PxU32 i=0;i<12;i++)
	{
//		PxVec3 p1 = Pts[*Indices++];
//		PxVec3 p2 = Pts[*Indices++];
//		Ps::makeFatEdge(p1, p2, fatBoxEdgeCoeff);	// PT: TODO: make fat segment instead
		const PxVec3& p1 = Pts[*Indices++];
		const PxVec3& p2 = Pts[*Indices++];

//		PT: keep original code in case something goes wrong
//		PxReal dist;
//		PxVec3 ip;
//		if(intersectEdgeEdge(p1, p2, -normal, segment.p0, segment.p1, dist, ip))
//			contactBuffer.contact(ip, normal, - (radius + dist));

		PxReal dist;
		PxVec3 ip;

		
		if(intersectEdgeEdgePreca(s0, s1, v1, plane, ii, jj, coeff, normal, p1, p2, dist, ip))
//		if(intersectEdgeEdgePreca(segment.p0, segment.p1, v1, plane, ii, jj, coeff, normal, p1, p2, dist, ip))
		{
			contactBuffer.contact(ip-normal*dist, normal, - (radius + dist));
//			if(contactBuffer.count==2)	// PT: we only need 2 contacts to be stable
//				return;
		}
	}
}


static void GuGenerateEEContacts2(	ContactBuffer& contactBuffer,
									//
									const Segment& segment,
									const PxReal radius,
									//
									const Box& worldBox,
									//
									const PxVec3& normal,
									const PxReal contactDistance)
{
	const PxU8* PX_RESTRICT Indices = getBoxEdges();

	PxVec3 Pts[8];
	worldBox.computeBoxPoints(Pts);

	PxVec3 s0 = segment.p0;
	PxVec3 s1 = segment.p1;
	Ps::makeFatEdge(s0, s1, fatBoxEdgeCoeff);

	// PT: precomputed part of edge-edge intersection test
//		const PxVec3 v1 = segment.p1 - segment.p0;
		const PxVec3 v1 = s1 - s0;
		PxPlane plane;
		plane.n = -(v1.cross(normal));
//		plane.d = -(plane.normal|segment.p0);
		plane.d = -(plane.n.dot(s0));

		PxU32 ii,jj;
		Ps::closestAxis(plane.n, ii, jj);

		const float coeff = 1.0f /(v1[jj]*normal[ii]-v1[ii]*normal[jj]);

	for(PxU32 i=0;i<12;i++)
	{
//		PxVec3 p1 = Pts[*Indices++];
//		PxVec3 p2 = Pts[*Indices++];
//		Ps::makeFatEdge(p1, p2, fatBoxEdgeCoeff);	// PT: TODO: make fat segment instead
		const PxVec3& p1 = Pts[*Indices++];
		const PxVec3& p2 = Pts[*Indices++];

//		PT: keep original code in case something goes wrong
//		PxReal dist;
//		PxVec3 ip;
//		bool contact = intersectEdgeEdge(p1, p2, normal, segment.p0, segment.p1, dist, ip);
//		if(contact && dist < radius + contactDistance)
//			contactBuffer.contact(ip, normal, dist - radius);

		PxReal dist;
		PxVec3 ip;
//		bool contact = intersectEdgeEdgePreca(segment.p0, segment.p1, v1, plane, ii, jj, coeff, -normal, p1, p2, dist, ip);
		bool contact = intersectEdgeEdgePreca(s0, s1, v1, plane, ii, jj, coeff, -normal, p1, p2, dist, ip);
		if(contact && dist < radius + contactDistance)
		{
			contactBuffer.contact(ip-normal*dist, normal, dist - radius);
//			if(contactBuffer.count==2)	// PT: we only need 2 contacts to be stable
//				return;
		}
	}
}

namespace physx
{
namespace Gu
{
bool contactCapsuleBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	// Get actual shape data
	const PxCapsuleGeometry& shapeCapsule = shape0.get<const PxCapsuleGeometry>();
	const PxBoxGeometry& shapeBox = shape1.get<const PxBoxGeometry>();

	// PT: TODO: move computations to local space

	// Capsule data
	Segment worldSegment;
	getCapsuleSegment(transform0, shapeCapsule, worldSegment);
	const PxReal inflatedRadius = shapeCapsule.radius + params.mContactDistance;

	// Box data
	Box worldBox;
	buildFrom(worldBox, transform1.p, shapeBox.halfExtents, transform1.q);

	// Collision detection
	PxReal t;
	PxVec3 onBox;
	const PxReal squareDist = distanceSegmentBoxSquared(worldSegment.p0, worldSegment.p1, worldBox.center, worldBox.extents, worldBox.rot, &t, &onBox);
	
	if(squareDist >= inflatedRadius*inflatedRadius)
		return false;

	PX_ASSERT(contactBuffer.count==0);

	if(squareDist != 0.0f)
	{
		// PT: the capsule segment doesn't intersect the box => distance-based version
		const PxVec3 onSegment = worldSegment.getPointAt(t);
		onBox = worldBox.center + worldBox.rot.transform(onBox);

		PxVec3 normal = onSegment - onBox;
		PxReal normalLen = normal.magnitude();

		if(normalLen > 0.0f)
		{
			normal *= 1.0f/normalLen;

			// PT: generate VF contacts for segment's vertices vs box
			GuGenerateVFContacts(contactBuffer, worldSegment, shapeCapsule.radius, worldBox, normal, params.mContactDistance);

			// PT: early exit if we already have 2 stable contacts
			if(contactBuffer.count==2)
				return true;

			// PT: else generate slower EE contacts
			GuGenerateEEContacts2(contactBuffer, worldSegment, shapeCapsule.radius, worldBox, normal, params.mContactDistance);

			// PT: run VF case for box-vertex-vs-capsule only if we don't have any contact yet
			if(!contactBuffer.count)
				contactBuffer.contact(onBox, normal, sqrtf(squareDist) - shapeCapsule.radius);
		}
		else
		{
			// On linux we encountered the following:
			// For a case where a segment endpoint lies on the surface of a box, the squared distance between segment and box was tiny but still larger than 0.
			// However, the computation of the normal length was exactly 0. In that case we should have switched to the penetration based version so we do it now
			// instead.
			goto PenetrationBasedCode;
		}
	}
	else
	{
		PenetrationBasedCode:

		// PT: the capsule segment intersects the box => penetration-based version

		// PT: compute penetration vector (MTD)
		PxVec3 sepAxis;
		PxReal depth;
		if(!GuCapsuleOBBOverlap3(worldSegment, shapeCapsule.radius, worldBox, &depth, &sepAxis)) return false;

		// PT: generate VF contacts for segment's vertices vs box
		GuGenerateVFContacts(contactBuffer, worldSegment, shapeCapsule.radius, worldBox, sepAxis, params.mContactDistance);

		// PT: early exit if we already have 2 stable contacts
		if(contactBuffer.count==2)
			return true;

		// PT: else generate slower EE contacts
		GuGenerateEEContacts(contactBuffer, worldSegment, shapeCapsule.radius, worldBox, sepAxis);

		if(!contactBuffer.count)
		{
			contactBuffer.contact(worldSegment.computeCenter(), sepAxis, -(shapeCapsule.radius + depth));
			return true;
		}
	}
	return true;
}
}//Gu
}//physx
