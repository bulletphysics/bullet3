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

#ifndef GU_SWEEP_TRIANGLE_UTILS_H
#define GU_SWEEP_TRIANGLE_UTILS_H

#include "CmPhysXCommon.h"
#include "GuSweepSharedTests.h"
#include "GuInternal.h"
#include "PxTriangle.h"
#include "PxQueryReport.h"

namespace physx
{

namespace Gu
{
	// PT: computes proper impact data for sphere-sweep-vs-tri, after the closest tri has been found.
	void computeSphereTriImpactData(PxVec3& hit, PxVec3& normal, const PxVec3& center, const PxVec3& dir, float t, const PxTriangle& tri);

	// PT: computes proper impact data for box-sweep-vs-tri, after the closest tri has been found.
	void computeBoxTriImpactData(PxVec3& hit, PxVec3& normal, const PxVec3& boxExtents, const PxVec3& localDir, const PxTriangle& triInBoxSpace, PxReal impactDist);

	// PT: computes impact normal between two edges. Produces better normals than just the EE cross product.
	// This version properly computes the closest points between two colliding edges and makes a normal from these.
	void computeEdgeEdgeNormal(PxVec3& normal, const PxVec3& p1, const PxVec3& p2_p1, const PxVec3& p3, const PxVec3& p4_p3, const PxVec3& dir, float d);

	// PT: small function just to avoid duplicating the code.
	// Returns index of first triangle we should process (when processing arrays of input triangles)
	PX_FORCE_INLINE PxU32 getInitIndex(const PxU32* PX_RESTRICT cachedIndex, PxU32 nbTris)
	{
		PxU32 initIndex = 0;	// PT: by default the first triangle to process is just the first one in the array
		if(cachedIndex)			// PT: but if we cached the last closest triangle from a previous call...
		{
			PX_ASSERT(*cachedIndex < nbTris);
			PX_UNUSED(nbTris);
			initIndex = *cachedIndex;	// PT: ...then we should start with that one, to potentially shrink the ray as early as possible
		}
		return initIndex;
	}

	// PT: quick triangle rejection for sphere-based sweeps.
	// Please refer to %SDKRoot%\InternalDocumentation\GU\cullTriangle.png for details & diagram.
	PX_FORCE_INLINE bool cullTriangle(const PxVec3* PX_RESTRICT triVerts, const PxVec3& dir, PxReal radius, PxReal t, const PxReal dpc0)
	{
		// PT: project triangle on axis
		const PxReal dp0 = triVerts[0].dot(dir);
		const PxReal dp1 = triVerts[1].dot(dir);
		const PxReal dp2 = triVerts[2].dot(dir);

		// PT: keep min value = earliest possible impact distance
		PxReal dp = dp0;
		dp = physx::intrinsics::selectMin(dp, dp1);
		dp = physx::intrinsics::selectMin(dp, dp2);

		// PT: make sure we keep triangles that are about as close as best current distance
		radius += 0.001f + GU_EPSILON_SAME_DISTANCE;

		// PT: if earliest possible impact distance for this triangle is already larger than
		// sphere's current best known impact distance, we can skip the triangle
		if(dp>dpc0 + t + radius)
		{
			//PX_ASSERT(resx == 0.0f);
			return false;
		}

		// PT: if triangle is fully located before the sphere's initial position, skip it too
		const PxReal dpc1 = dpc0 - radius;
		if(dp0<dpc1 && dp1<dpc1 && dp2<dpc1)
		{
			//PX_ASSERT(resx == 0.0f);
			return false;
		}

		//PX_ASSERT(resx != 0.0f);
		return true;
	}

	// PT: quick quad rejection for sphere-based sweeps. Same as for triangle, adapted for one more vertex.
	PX_FORCE_INLINE bool cullQuad(const PxVec3* PX_RESTRICT quadVerts, const PxVec3& dir, PxReal radius, PxReal t, const PxReal dpc0)
	{
		// PT: project quad on axis
		const PxReal dp0 = quadVerts[0].dot(dir);
		const PxReal dp1 = quadVerts[1].dot(dir);
		const PxReal dp2 = quadVerts[2].dot(dir);
		const PxReal dp3 = quadVerts[3].dot(dir);

		// PT: keep min value = earliest possible impact distance
		PxReal dp = dp0;
		dp = physx::intrinsics::selectMin(dp, dp1);
		dp = physx::intrinsics::selectMin(dp, dp2);
		dp = physx::intrinsics::selectMin(dp, dp3);

		// PT: make sure we keep quads that are about as close as best current distance
		radius += 0.001f;

		// PT: if earliest possible impact distance for this quad is already larger than
		// sphere's current best known impact distance, we can skip the quad
		if(dp>dpc0 + t + radius)
			return false;

		// PT: if quad is fully located before the sphere's initial position, skip it too
		const float dpc1 = dpc0 - radius;
		if(dp0<dpc1 && dp1<dpc1 && dp2<dpc1 && dp3<dpc1)
			return false;

		return true;
	}

	// PT: computes distance between a point 'point' and a segment. The segment is defined as a starting point 'p0'
	// and a direction vector 'dir' plus a length 't'. Segment's endpoint is p0 + dir * t.
	//
	//                     point
	//                      o
	//                   __/|
	//                __/ / |
	//             __/   /  |(B)
	//          __/  (A)/   |
	//       __/       /    |                dir
	//  p0 o/---------o---------------o--    -->
	//                t (t<=fT)       t (t>fT)
	//                return (A)^2    return (B)^2
	//
	//     |<-------------->|
	//             fT
	//
	PX_FORCE_INLINE PxReal squareDistance(const PxVec3& p0, const PxVec3& dir, PxReal t, const PxVec3& point)
	{
		PxVec3 diff = point - p0;
		PxReal fT = diff.dot(dir);
		fT = physx::intrinsics::selectMax(fT, 0.0f);
		fT = physx::intrinsics::selectMin(fT, t);
		diff -= fT*dir;
		return diff.magnitudeSquared();
	}

	// PT: quick triangle culling for sphere-based sweeps
	// Please refer to %SDKRoot%\InternalDocumentation\GU\coarseCulling.png for details & diagram.
	PX_FORCE_INLINE bool coarseCullingTri(const PxVec3& center, const PxVec3& dir, PxReal t, PxReal radius, const PxVec3* PX_RESTRICT triVerts)
	{
		// PT: compute center of triangle ### could be precomputed?
		const PxVec3 triCenter = (triVerts[0] + triVerts[1] + triVerts[2]) * (1.0f/3.0f);

		// PT: distance between the triangle center and the swept path (an LSS)
		// Same as: distancePointSegmentSquared(center, center+dir*t, TriCenter);
		PxReal d = PxSqrt(squareDistance(center, dir, t, triCenter)) - radius - 0.0001f;

		if (d < 0.0f)	// The triangle center lies inside the swept sphere
			return true;

		d*=d;

		// PT: coarse capsule-vs-triangle overlap test ### distances could be precomputed?
		if(1)
		{
			if(d <= (triCenter-triVerts[0]).magnitudeSquared())
				return true;
			if(d <= (triCenter-triVerts[1]).magnitudeSquared())
				return true;
			if(d <= (triCenter-triVerts[2]).magnitudeSquared())
				return true;
		}
		else
		{
			const float d0 = (triCenter-triVerts[0]).magnitudeSquared();
			const float d1 = (triCenter-triVerts[1]).magnitudeSquared();
			const float d2 = (triCenter-triVerts[2]).magnitudeSquared();
			float triRadius = physx::intrinsics::selectMax(d0, d1);
			triRadius = physx::intrinsics::selectMax(triRadius, d2);
			if(d <= triRadius)
				return true;
		}
		return false;
	}

	// PT: quick quad culling for sphere-based sweeps. Same as for triangle, adapted for one more vertex.
	PX_FORCE_INLINE bool coarseCullingQuad(const PxVec3& center, const PxVec3& dir, PxReal t, PxReal radius, const PxVec3* PX_RESTRICT quadVerts)
	{
		// PT: compute center of quad ### could be precomputed?
		const PxVec3 quadCenter = (quadVerts[0] + quadVerts[1] + quadVerts[2] + quadVerts[3]) * (1.0f/4.0f);

		// PT: distance between the quad center and the swept path (an LSS)
		PxReal d = PxSqrt(squareDistance(center, dir, t, quadCenter)) - radius - 0.0001f;

		if (d < 0.0f)	// The quad center lies inside the swept sphere
			return true;

		d*=d;

		// PT: coarse capsule-vs-quad overlap test ### distances could be precomputed?
		if(1)
		{
			if(d <= (quadCenter-quadVerts[0]).magnitudeSquared())
				return true;
			if(d <= (quadCenter-quadVerts[1]).magnitudeSquared())
				return true;
			if(d <= (quadCenter-quadVerts[2]).magnitudeSquared())
				return true;
			if(d <= (quadCenter-quadVerts[3]).magnitudeSquared())
				return true;
		}
		return false;
	}

	// PT: combined triangle culling for sphere-based sweeps
	PX_FORCE_INLINE bool rejectTriangle(const PxVec3& center, const PxVec3& unitDir, PxReal curT, PxReal radius, const PxVec3* PX_RESTRICT triVerts, const PxReal dpc0)
	{
		if(!coarseCullingTri(center, unitDir, curT, radius, triVerts))
			return true;
		if(!cullTriangle(triVerts, unitDir, radius, curT, dpc0))
			return true;
		return false;
	}

	// PT: combined quad culling for sphere-based sweeps
	PX_FORCE_INLINE bool rejectQuad(const PxVec3& center, const PxVec3& unitDir, PxReal curT, PxReal radius, const PxVec3* PX_RESTRICT quadVerts, const PxReal dpc0)
	{
		if(!coarseCullingQuad(center, unitDir, curT, radius, quadVerts))
			return true;
		if(!cullQuad(quadVerts, unitDir, radius, curT, dpc0))
			return true;
		return false;
	}

	PX_FORCE_INLINE bool shouldFlipNormal(const PxVec3& normal, bool meshBothSides, bool isDoubleSided, const PxVec3& triangleNormal, const PxVec3& dir)
	{
		// PT: this function assumes that input normal is opposed to the ray/sweep direction. This is always
		// what we want except when we hit a single-sided back face with 'meshBothSides' enabled.

		if(!meshBothSides || isDoubleSided)
			return false;

		PX_ASSERT(normal.dot(dir) <= 0.0f);	// PT: if this fails, the logic below cannot be applied
		PX_UNUSED(normal);
		return triangleNormal.dot(dir) > 0.0f;	// PT: true for back-facing hits
	}

	PX_FORCE_INLINE bool shouldFlipNormal(const PxVec3& normal, bool meshBothSides, bool isDoubleSided, const PxTriangle& triangle, const PxVec3& dir, const PxTransform* pose)
	{
		// PT: this function assumes that input normal is opposed to the ray/sweep direction. This is always
		// what we want except when we hit a single-sided back face with 'meshBothSides' enabled.

		if(!meshBothSides || isDoubleSided)
			return false;

		PX_ASSERT(normal.dot(dir) <= 0.0f);	// PT: if this fails, the logic below cannot be applied
		PX_UNUSED(normal);

		PxVec3 triangleNormal;
		triangle.denormalizedNormal(triangleNormal);

		if(pose)
			triangleNormal = pose->rotate(triangleNormal);

		return triangleNormal.dot(dir) > 0.0f;	// PT: true for back-facing hits
	}

	// PT: implements the spec for IO sweeps in a single place (to ensure consistency)
	PX_FORCE_INLINE bool setInitialOverlapResults(PxSweepHit& hit, const PxVec3& unitDir, PxU32 faceIndex)
	{
		// PT: please write these fields in the order they are listed in the struct.
		hit.faceIndex	= faceIndex;
		hit.flags		= PxHitFlag::eNORMAL|PxHitFlag::eFACE_INDEX;
		hit.normal		= -unitDir;
		hit.distance	= 0.0f;
		return true;	// PT: true indicates a hit, saves some lines in calling code
	}

	PX_FORCE_INLINE void computeBoxLocalImpact(	PxVec3& pos, PxVec3& normal, PxHitFlags& outFlags,
												const Box& box, const PxVec3& localDir, const PxTriangle& triInBoxSpace,
												const PxHitFlags inFlags, bool isDoubleSided, bool meshBothSides, PxReal impactDist)
	{
		if(inFlags & (PxHitFlag::eNORMAL|PxHitFlag::ePOSITION))
		{			
			PxVec3 localPos, localNormal;
			computeBoxTriImpactData(localPos, localNormal, box.extents, localDir, triInBoxSpace, impactDist);

			if(inFlags & PxHitFlag::eNORMAL)
			{
				localNormal.normalize();

				// PT: doing this after the 'rotate' minimizes errors when normal and dir are close to perpendicular
				// ....but we must do it before the rotate now, because triangleNormal is in box space (and thus we
				// need the normal with the proper orientation, in box space. We can't fix it after it's been rotated
				// to box space.
				// Technically this one is only here because of the EE cross product in the feature-based sweep.
				// PT: TODO: revisit corresponding code in computeImpactData, get rid of ambiguity
				// PT: TODO: this may not be needed anymore
				if((localNormal.dot(localDir))>0.0f)
					localNormal = -localNormal;

				// PT: this one is to ensure the normal respects the mesh-both-sides/double-sided convention
				if(shouldFlipNormal(localNormal, meshBothSides, isDoubleSided, triInBoxSpace, localDir, NULL))
					localNormal = -localNormal;

				normal = box.rotate(localNormal);
				outFlags |= PxHitFlag::eNORMAL;
			}

			if(inFlags & PxHitFlag::ePOSITION)
			{
				pos = box.transform(localPos);
				outFlags |= PxHitFlag::ePOSITION;
			}
		}
	}

} // namespace Gu

}

#endif
