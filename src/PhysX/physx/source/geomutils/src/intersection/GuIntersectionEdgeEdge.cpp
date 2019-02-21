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

#include "GuIntersectionEdgeEdge.h"
#include "PsMathUtils.h"
#include "CmPhysXCommon.h"

using namespace physx;

bool Gu::intersectEdgeEdge(const PxVec3& p1, const PxVec3& p2, const PxVec3& dir, const PxVec3& p3, const PxVec3& p4, PxReal& dist, PxVec3& ip)
{
	const PxVec3 v1 = p2 - p1;

	// Build plane P based on edge (p1, p2) and direction (dir)
	PxPlane plane;
	plane.n = v1.cross(dir);
	plane.d = -(plane.n.dot(p1));

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

	// find largest 2D plane projection
	PxU32 i,j;
	Ps::closestAxis(plane.n, i, j);

	// compute distance of intersection from line (ip, -dir) to line (p1,p2)
	dist =	(v1[i]*(ip[j]-p1[j])-v1[j]*(ip[i]-p1[i]))/(v1[i]*dir[j]-v1[j]*dir[i]);
	if(dist<0.0f)	return false;

	// compute intersection point on edge (p1,p2) line
	ip -= dist*dir;

	// check if intersection point (ip) is between edge (p1,p2) vertices
	temp = (p1.x-ip.x)*(p2.x-ip.x)+(p1.y-ip.y)*(p2.y-ip.y)+(p1.z-ip.z)*(p2.z-ip.z);
	if(temp<1e-3f)	return true;	// collision found

	return false;	// no collision
}
