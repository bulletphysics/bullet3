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

#ifndef GU_INTERSECTION_RAY_CAPSULE_H
#define GU_INTERSECTION_RAY_CAPSULE_H

#include "CmPhysXCommon.h"
#include "GuCapsule.h"
#include "GuDistancePointSegment.h"
#include "GuIntersectionRay.h"

namespace physx
{
namespace Gu
{
	PxU32 intersectRayCapsuleInternal(const PxVec3& origin, const PxVec3& dir, const PxVec3& p0, const PxVec3& p1, float radius, PxReal s[2]);

	PX_FORCE_INLINE bool intersectRayCapsule(const PxVec3& origin, const PxVec3& dir, const PxVec3& p0, const PxVec3& p1, float radius, PxReal& t)
	{
		// PT: move ray origin close to capsule, to solve accuracy issues.
		// We compute the distance D between the ray origin and the capsule's segment.
		// Then E = D - radius = distance between the ray origin and the capsule.
		// We can move the origin freely along 'dir' up to E units before touching the capsule.
		PxReal l = distancePointSegmentSquaredInternal(p0, p1 - p0, origin);
		l = PxSqrt(l) - radius;

		// PT: if this becomes negative or null, the ray starts inside the capsule and we can early exit
		if(l<=0.0f)
		{
			t = 0.0f;
			return true;
		}

		// PT: we remove an arbitrary GU_RAY_SURFACE_OFFSET units to E, to make sure we don't go close to the surface.
		// If we're moving in the direction of the capsule, the origin is now about GU_RAY_SURFACE_OFFSET units from it.
		// If we're moving away from the capsule, the ray won't hit the capsule anyway.
		// If l is smaller than GU_RAY_SURFACE_OFFSET we're close enough, accuracy is good, there is nothing to do.
		if(l>GU_RAY_SURFACE_OFFSET)
			l -= GU_RAY_SURFACE_OFFSET;
		else
			l = 0.0f;

		// PT: move origin closer to capsule and do the raycast
		PxReal s[2];
		const PxU32 nbHits = Gu::intersectRayCapsuleInternal(origin + l*dir, dir, p0, p1, radius, s);
		if(!nbHits)
			return false;

		// PT: keep closest hit only
		if(nbHits == 1)
			t = s[0];
		else
			t = (s[0] < s[1]) ? s[0] : s[1];

		// PT: fix distance (smaller than expected after moving ray close to capsule)
		t += l;
		return true;
	}

	PX_FORCE_INLINE bool intersectRayCapsule(const PxVec3& origin, const PxVec3& dir, const Gu::Capsule& capsule, PxReal& t)
	{
		return Gu::intersectRayCapsule(origin, dir, capsule.p0, capsule.p1, capsule.radius, t);
	}
}
}

#endif
