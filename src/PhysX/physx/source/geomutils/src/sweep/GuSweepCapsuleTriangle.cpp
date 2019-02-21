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

#include "GuSweepCapsuleTriangle.h"
#include "GuIntersectionCapsuleTriangle.h"
#include "GuDistanceSegmentTriangle.h"
#include "GuDistanceSegmentTriangleSIMD.h"
#include "GuIntersectionTriangleBox.h"
#include "GuSweepSphereTriangle.h"
#include "GuInternal.h"

using namespace physx;
using namespace Gu;
using namespace physx::shdfnd::aos;

#define COLINEARITY_EPSILON 0.00001f

///////////////////////////////////////////////////////////////////////////////

#define OUTPUT_TRI(pp0, pp1, pp2){															\
	extrudedTris[nbExtrudedTris].verts[0] = pp0;											\
	extrudedTris[nbExtrudedTris].verts[1] = pp1;											\
	extrudedTris[nbExtrudedTris].verts[2] = pp2;											\
	extrudedTris[nbExtrudedTris].denormalizedNormal(extrudedTrisNormals[nbExtrudedTris]);	\
	nbExtrudedTris++;}

#define OUTPUT_TRI2(p0, p1, p2, d){				\
	PxTriangle& tri = extrudedTris[nbExtrudedTris];	\
	tri.verts[0] = p0;								\
	tri.verts[1] = p1;								\
	tri.verts[2] = p2;								\
	PxVec3 nrm;										\
	tri.denormalizedNormal(nrm);					\
	if(nrm.dot(d)>0.0f) {							\
	PxVec3 tmp = tri.verts[1];						\
	tri.verts[1] = tri.verts[2];					\
	tri.verts[2] = tmp;								\
	nrm = -nrm;										\
	}												\
	extrudedTrisNormals[nbExtrudedTris] = nrm;		\
	nbExtrudedTris++; }


bool Gu::sweepCapsuleTriangles_Precise(	PxU32 nbTris, const PxTriangle* PX_RESTRICT triangles,	// Triangle data
										const Capsule& capsule,									// Capsule data
										const PxVec3& unitDir, const PxReal distance,			// Ray data
										const PxU32* PX_RESTRICT cachedIndex,					// Cache data
										PxSweepHit& hit, PxVec3& triNormalOut,					// Results
										PxHitFlags hitFlags, bool isDoubleSided,				// Query modifiers
										const BoxPadded* cullBox)								// Cull data
{
	if(!nbTris)
		return false;

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool doBackfaceCulling = !isDoubleSided && !meshBothSides;
	const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;
	const bool testInitialOverlap = !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP);

	// PT: we can fallback to sphere sweep:
	// - if the capsule is degenerate (i.e. it's a sphere)
	// - if the sweep direction is the same as the capsule axis, in which case we can just sweep the top or bottom sphere

	const PxVec3 extrusionDir = (capsule.p0 - capsule.p1)*0.5f;	// Extrusion dir = capsule segment
	const PxReal halfHeight = extrusionDir.magnitude();
	bool mustExtrude = halfHeight!=0.0f;
	if(!mustExtrude)
	{
		// PT: capsule is a sphere. Switch to sphere path (intersectCapsuleTriangle doesn't work for degenerate capsules)
		return sweepSphereTriangles(nbTris, triangles, capsule.p0, capsule.radius, unitDir, distance, cachedIndex, hit, triNormalOut, isDoubleSided, meshBothSides, anyHit, testInitialOverlap);
	}
	else
	{
		const PxVec3 capsuleAxis = extrusionDir/halfHeight;
		const PxReal colinearity = PxAbs(capsuleAxis.dot(unitDir));
		mustExtrude = (colinearity < (1.0f - COLINEARITY_EPSILON));
	}

	const PxVec3 capsuleCenter = capsule.computeCenter();

	if(!mustExtrude)
	{
		CapsuleTriangleOverlapData params;
		params.init(capsule);
		// PT: unfortunately we need to do IO test with the *capsule*, even though we're in the sphere codepath. So we
		// can't directly reuse the sphere function.
		const PxVec3 sphereCenter = capsuleCenter + unitDir * halfHeight;
		// PT: this is a copy of 'sweepSphereTriangles' but with a capsule IO test. Saves double backface culling....
		{
			PxU32 index = PX_INVALID_U32;
			const PxU32 initIndex = getInitIndex(cachedIndex, nbTris);

			PxReal curT = distance;
			const PxReal dpc0 = sphereCenter.dot(unitDir);

			PxReal bestAlignmentValue = 2.0f;

			PxVec3 bestTriNormal(0.0f);

			for(PxU32 ii=0; ii<nbTris; ii++)	// We need i for returned triangle index
			{
				const PxU32 i = getTriangleIndex(ii, initIndex);

				const PxTriangle& currentTri = triangles[i];

				if(rejectTriangle(sphereCenter, unitDir, curT, capsule.radius, currentTri.verts, dpc0))
					continue;

				PxVec3 triNormal;
				currentTri.denormalizedNormal(triNormal);

				// Backface culling
				if(doBackfaceCulling && (triNormal.dot(unitDir) > 0.0f))
					continue;

				if(testInitialOverlap && intersectCapsuleTriangle(triNormal, currentTri.verts[0], currentTri.verts[1], currentTri.verts[2], capsule, params))
				{
					triNormalOut = -unitDir;
					return setInitialOverlapResults(hit, unitDir, i);
				}

				const PxReal magnitude = triNormal.magnitude();
				if(magnitude==0.0f)
					continue;

				triNormal /= magnitude;

				PxReal currentDistance;
				bool unused;
				if(!sweepSphereVSTri(currentTri.verts, triNormal, sphereCenter, capsule.radius, unitDir, currentDistance, unused, false))
					continue;

				const PxReal distEpsilon = GU_EPSILON_SAME_DISTANCE; // pick a farther hit within distEpsilon that is more opposing than the previous closest hit
				const PxReal hitDot = computeAlignmentValue(triNormal, unitDir);
				if(!keepTriangle(currentDistance, hitDot, curT, bestAlignmentValue, distance, distEpsilon))
					continue;

				curT = currentDistance;
				index = i;
				bestAlignmentValue = hitDot;
				bestTriNormal = triNormal;
				if(anyHit)
					break;
			}
			return computeSphereTriangleImpactData(hit, triNormalOut, index, curT, sphereCenter, unitDir, bestTriNormal, triangles, isDoubleSided, meshBothSides);
		}
	}

	// PT: extrude mesh on the fly. This is a modified copy of sweepSphereTriangles, unfortunately
	PxTriangle extrudedTris[7];
	PxVec3 extrudedTrisNormals[7];	// Not normalized

	hit.faceIndex = PX_INVALID_U32;
	const PxU32 initIndex = getInitIndex(cachedIndex, nbTris);

	const PxReal radius = capsule.radius;
	PxReal curT = distance;
	const PxReal dpc0 = capsuleCenter.dot(unitDir);

	// PT: we will copy the best triangle here. Using indices alone doesn't work
	// since we extrude on-the-fly (and we don't want to re-extrude later)
	PxTriangle bestTri;
	PxVec3 bestTriNormal(0.0f);
	PxReal mostOpposingHitDot = 2.0f;

	CapsuleTriangleOverlapData params;
	params.init(capsule);

	for(PxU32 ii=0; ii<nbTris; ii++)	// We need i for returned triangle index
	{
		const PxU32 i = getTriangleIndex(ii, initIndex);

		const PxTriangle& currentSrcTri = triangles[i];	// PT: src tri, i.e. non-extruded

///////////// PT: this part comes from "ExtrudeMesh"
		// Create triangle normal
		PxVec3 denormalizedNormal;
		currentSrcTri.denormalizedNormal(denormalizedNormal);

		// Backface culling
		if(doBackfaceCulling && (denormalizedNormal.dot(unitDir) > 0.0f))
			continue;

		if(cullBox)
		{
			if(!intersectTriangleBox(*cullBox, currentSrcTri.verts[0], currentSrcTri.verts[1], currentSrcTri.verts[2]))
				continue;
		}

		if(testInitialOverlap && intersectCapsuleTriangle(denormalizedNormal, currentSrcTri.verts[0], currentSrcTri.verts[1], currentSrcTri.verts[2], capsule, params))
		{
			triNormalOut = -unitDir;
			return setInitialOverlapResults(hit, unitDir, i);
		}

		// Extrude mesh on the fly
		PxU32 nbExtrudedTris=0;

		const PxVec3 p0 = currentSrcTri.verts[0] - extrusionDir;
		const PxVec3 p1 = currentSrcTri.verts[1] - extrusionDir;
		const PxVec3 p2 = currentSrcTri.verts[2] - extrusionDir;

		const PxVec3 p0b = currentSrcTri.verts[0] + extrusionDir;
		const PxVec3 p1b = currentSrcTri.verts[1] + extrusionDir;
		const PxVec3 p2b = currentSrcTri.verts[2] + extrusionDir;

		if(denormalizedNormal.dot(extrusionDir) >= 0.0f)	OUTPUT_TRI(p0b, p1b, p2b)
		else												OUTPUT_TRI(p0, p1, p2)

		// ### it's probably useless to extrude all the shared edges !!!!!
		//if(CurrentFlags & TriangleCollisionFlag::eACTIVE_EDGE12)
		{
			OUTPUT_TRI2(p1, p1b, p2b, unitDir)
			OUTPUT_TRI2(p1, p2b, p2, unitDir)
		}
		//if(CurrentFlags & TriangleCollisionFlag::eACTIVE_EDGE20)
		{
			OUTPUT_TRI2(p0, p2, p2b, unitDir)
			OUTPUT_TRI2(p0, p2b, p0b, unitDir)
		}
		//if(CurrentFlags & TriangleCollisionFlag::eACTIVE_EDGE01)
		{
			OUTPUT_TRI2(p0b, p1b, p1, unitDir)
			OUTPUT_TRI2(p0b, p1, p0, unitDir)
		}
/////////////

		// PT: TODO: this one is new, to fix the tweak issue. However this wasn't
		// here before so the perf hit should be analyzed.
		denormalizedNormal.normalize();
		const PxReal hitDot1 = computeAlignmentValue(denormalizedNormal, unitDir);

		for(PxU32 j=0;j<nbExtrudedTris;j++)
		{
			const PxTriangle& currentTri = extrudedTris[j];

			PxVec3& triNormal = extrudedTrisNormals[j];
			// Backface culling
			if(doBackfaceCulling && (triNormal.dot(unitDir)) > 0.0f)
				continue;

			// PT: beware, culling is only ok on the sphere I think
			if(rejectTriangle(capsuleCenter, unitDir, curT, radius, currentTri.verts, dpc0))
				continue;

			const PxReal magnitude = triNormal.magnitude();
			if(magnitude==0.0f)
				continue;

			triNormal /= magnitude;

			PxReal currentDistance;
			bool unused;
			if(!sweepSphereVSTri(currentTri.verts, triNormal, capsuleCenter, radius, unitDir, currentDistance, unused, false))
				continue;

			const PxReal distEpsilon = GU_EPSILON_SAME_DISTANCE; // pick a farther hit within distEpsilon that is more opposing than the previous closest hit			
			if(!keepTriangle(currentDistance, hitDot1, curT, mostOpposingHitDot, distance, distEpsilon))
				continue;

			curT = currentDistance;
			hit.faceIndex = i;
			mostOpposingHitDot = hitDot1; // arbitrary bias. works for hitDot1=-1, prevHitDot=0
			bestTri = currentTri;
			bestTriNormal = denormalizedNormal;
			if(anyHit)
				goto Exit;	// PT: using goto to have one test per hit, not test per triangle ('break' doesn't work here)
		}
	}
Exit:
	if(hit.faceIndex==PX_INVALID_U32)
		return false;	// We didn't touch any triangle

	hit.distance = curT;

	triNormalOut = bestTriNormal;

	// Compute impact data only once, using best triangle
	computeSphereTriImpactData(hit.position, hit.normal, capsuleCenter, unitDir, hit.distance, bestTri);

	// PT: by design, returned normal is opposed to the sweep direction.
	if(shouldFlipNormal(hit.normal, meshBothSides, isDoubleSided, bestTriNormal, unitDir))
		hit.normal = -hit.normal;

	// PT: revisit this
	if(hit.faceIndex!=PX_INVALID_U32)
	{
		// PT: we need to recompute a hit here because the hit between the *capsule* and the source mesh can be very
		// different from the hit between the *sphere* and the extruded mesh.

		// Touched tri
		const PxVec3& p0 = triangles[hit.faceIndex].verts[0];
		const PxVec3& p1 = triangles[hit.faceIndex].verts[1];
		const PxVec3& p2 = triangles[hit.faceIndex].verts[2];

		// AP: measured to be a bit faster than the scalar version
		const PxVec3 delta = unitDir*hit.distance;
		Vec3V pointOnSeg, pointOnTri;
		distanceSegmentTriangleSquared(
			V3LoadU(capsule.p0 + delta), V3LoadU(capsule.p1 + delta),
			V3LoadU(p0), V3LoadU(p1), V3LoadU(p2),
			pointOnSeg, pointOnTri);
		V3StoreU(pointOnTri, hit.position);

		hit.flags = PxHitFlag::eNORMAL|PxHitFlag::ePOSITION;
	}
	return true;
}
