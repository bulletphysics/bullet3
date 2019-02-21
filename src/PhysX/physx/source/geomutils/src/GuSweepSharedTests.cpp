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

#include "GuSweepTests.h"
#include "GuHeightFieldUtil.h"
#include "CmScaling.h"
#include "GuConvexMesh.h"
#include "GuIntersectionRayPlane.h"
#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuSweepMTD.h"
#include "PxConvexMeshGeometry.h"
#include "PxSphereGeometry.h"
#include "GuSweepSphereCapsule.h"
#include "GuSweepCapsuleCapsule.h"
#include "GuSweepTriangleUtils.h"
#include "GuSweepCapsuleTriangle.h"
#include "GuInternal.h"
#include "GuGJKRaycast.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace physx::shdfnd::aos;

static const PxReal gEpsilon = .01f;

static PxU32 computeSweepConvexPlane(
	const PxConvexMeshGeometry& convexGeom, ConvexHullData* hullData, const PxU32& nbPolys, const PxTransform& pose,
	const PxVec3& impact_, const PxVec3& unitDir)
{
	PX_ASSERT(nbPolys);

	const PxVec3 impact = impact_ - unitDir * gEpsilon;

	const PxVec3 localPoint = pose.transformInv(impact);
	const PxVec3 localDir = pose.rotateInv(unitDir);

	const FastVertex2ShapeScaling scaling(convexGeom.scale);

	PxU32 minIndex = 0;
	PxReal minD = PX_MAX_REAL;
	for(PxU32 j=0; j<nbPolys; j++)
	{
		const PxPlane& pl = hullData->mPolygons[j].mPlane;

		PxPlane plane;
		scaling.transformPlaneToShapeSpace(pl.n, pl.d, plane.n, plane.d);

		PxReal d = plane.distance(localPoint);
		if(d<0.0f)
			continue;

		const PxReal tweak = plane.n.dot(localDir) * gEpsilon;
		d += tweak;

		if(d<minD)
		{
			minIndex = j;
			minD = d;
		}
	}
	return minIndex;
}

static PX_FORCE_INLINE bool computeFaceIndex(PxSweepHit& sweepHit, const PxHitFlags hitFlags, const PxConvexMeshGeometry& convexGeom, ConvexHullData* hullData, const PxTransform& pose, const PxVec3& unitDir)
{
	if(hitFlags & PxHitFlag::eFACE_INDEX)
	{
		// PT: compute closest polygon using the same tweak as in swept-capsule-vs-mesh
		sweepHit.faceIndex = computeSweepConvexPlane(convexGeom, hullData, hullData->mNbPolygons, pose, sweepHit.position, unitDir);
		sweepHit.flags |= PxHitFlag::eFACE_INDEX;
	}
	return true;
}

static PX_FORCE_INLINE bool hasInitialOverlap(PxSweepHit& sweepHit, const PxVec3& unitDir,
											  const FloatVArg toi,
											  const Vec3VArg normal, const Vec3VArg closestA,
											  const PsTransformV& convexPose,
											  const bool isMtd, const bool impactPointOnTheOtherShape)
{
	sweepHit.flags = PxHitFlag::eNORMAL;

	const FloatV zero = FZero();
	if(FAllGrtrOrEq(zero, toi))
	{
		//ML: initial overlap
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const FloatV length = toi;
			const Vec3V worldPointA = convexPose.transform(closestA);
			const Vec3V worldNormal = V3Normalize(convexPose.rotate(normal));
			if(impactPointOnTheOtherShape)
			{
				const Vec3V destWorldPointA = V3NegScaleSub(worldNormal, length, worldPointA);
				V3StoreU(worldNormal, sweepHit.normal);
				V3StoreU(destWorldPointA, sweepHit.position);
			}
			else
			{
				const Vec3V destNormal = V3Neg(worldNormal);
				V3StoreU(destNormal, sweepHit.normal);
				V3StoreU(worldPointA, sweepHit.position);
			}
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
		sweepHit.faceIndex = 0xffffffff;
		return true;
	}
	return false;
}

/////////////////////////////////////////////////  sweepCapsule/Sphere  //////////////////////////////////////////////////////
bool sweepCapsule_SphereGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);

	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	const Sphere sphere(pose.p, sphereGeom.radius+inflation);

	if(!sweepSphereCapsule(sphere, lss, -unitDir, distance, sweepHit.distance, sweepHit.position, sweepHit.normal, hitFlags))
		return false;

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	if(isMtd)
	{
		sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;

		if(sweepHit.distance == 0.f)
		{
			//intialOverlap
			if(lss.p0 == lss.p1)
			{
				//sphere
				return computeSphere_SphereMTD(sphere, Sphere(lss.p0, lss.radius), sweepHit);
			}
			else
			{
				//capsule
				return computeSphere_CapsuleMTD(sphere, lss, sweepHit);
			}
		}
	}
	else
	{
		if(sweepHit.distance!=0.0f)
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
		else
			sweepHit.flags = PxHitFlag::eNORMAL;
	}
	return true;
}

bool sweepCapsule_PlaneGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);

	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	PX_UNUSED(geom);
//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);

	const PxPlane& worldPlane = getPlane(pose);

	const PxF32 capsuleRadius = lss.radius + inflation;

	PxU32 index = 0;
	PxVec3 pts[2];

	PxReal minDp = PX_MAX_REAL;

	sweepHit.faceIndex	= 0xFFFFffff; // spec says face index is undefined for planes

	// Find extreme point on the capsule
	// AP: removed if (lss.p0 == lss.p1 clause because it wasn't properly computing minDp)
	pts[0] = lss.p0;
	pts[1] = lss.p1;
	for(PxU32 i=0; i<2; i++)
	{
		const PxReal dp = pts[i].dot(worldPlane.n);
		if(dp<minDp)
		{
			minDp = dp;
			index = i;
		}
	}

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	if(isMtd)
	{
		//initial overlap with the plane
		if(minDp <= capsuleRadius - worldPlane.d)
		{
			sweepHit.flags = PxHitFlag::eNORMAL| PxHitFlag::ePOSITION;
			return computePlane_CapsuleMTD(worldPlane, lss, sweepHit);
		}
	}
	else
	{
		if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
		{
			// test if the capsule initially overlaps with plane
			if(minDp <= capsuleRadius - worldPlane.d)
			{
				sweepHit.flags		= PxHitFlag::eNORMAL;
				sweepHit.distance	= 0.0f;
				sweepHit.normal		= -unitDir;
				return true;
			}
		}
	}

	const PxVec3 ptOnCapsule = pts[index] - worldPlane.n*capsuleRadius;

	// Raycast extreme vertex against plane
	bool hitPlane = intersectRayPlane(ptOnCapsule, unitDir, worldPlane, sweepHit.distance, &sweepHit.position);
	if(hitPlane && sweepHit.distance > 0 && sweepHit.distance <= distance)
	{
		sweepHit.normal = worldPlane.n;
		sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
		return true;
	}
	return false;
}

bool sweepCapsule_CapsuleGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);

	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Capsule staticCapsule;
	getCapsule(staticCapsule, capsuleGeom, pose);
	staticCapsule.radius +=inflation;

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	PxU16 outFlags;
	if(!sweepCapsuleCapsule(lss, staticCapsule, -unitDir, distance, sweepHit.distance, sweepHit.position, sweepHit.normal, hitFlags, outFlags))
		return false;

	sweepHit.flags = PxHitFlags(outFlags);
	if(sweepHit.distance == 0.0f)
	{
		//initial overlap
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			return computeCapsule_CapsuleMTD(lss, staticCapsule, sweepHit);
		}
	}
	return true;
}

bool sweepCapsule_ConvexGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);

	using namespace Ps::aos;
	
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	ConvexHullData* hullData = &convexMesh->getHull();

	const Vec3V zeroV = V3Zero();
	const FloatV zero = FZero();
	const FloatV dist = FLoad(distance);
	const Vec3V worldDir = V3LoadU(unitDir);

	const PsTransformV capPose = loadTransformU(capsulePose_);
	const PsTransformV convexPose = loadTransformU(pose);

	const PsMatTransformV aToB(convexPose.transformInv(capPose));

	const FloatV capsuleHalfHeight = FLoad(capsuleGeom_.halfHeight);
	const FloatV capsuleRadius = FLoad(lss.radius);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);

	CapsuleV capsule(aToB.p, aToB.rotate( V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);
	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, convexGeom.scale.isIdentity());

	const Vec3V dir = convexPose.rotateInv(V3Neg(V3Scale(worldDir, dist)));

	bool isMtd = hitFlags & PxHitFlag::eMTD;

	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of convex hull
	LocalConvex<CapsuleV> convexA(capsule);
	LocalConvex<ConvexHullV> convexB(convexHull);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());
	if(!gjkRaycastPenetration< LocalConvex<CapsuleV>, LocalConvex<ConvexHullV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, lss.radius + inflation, isMtd))
		return false;

	if(hasInitialOverlap(sweepHit, unitDir, toi, normal, closestA, convexPose, isMtd, true))
		return true;

	sweepHit.flags |= PxHitFlag::ePOSITION;
	const Vec3V worldPointA = convexPose.transform(closestA);
	const FloatV length = FMul(dist, toi);
	const Vec3V destNormal = V3Normalize(convexPose.rotate(normal));
	const Vec3V destWorldPointA = V3ScaleAdd(worldDir, length, worldPointA);
	V3StoreU(destNormal, sweepHit.normal);
	V3StoreU(destWorldPointA, sweepHit.position);
	FStore(length, &sweepHit.distance);

	return computeFaceIndex(sweepHit, hitFlags, convexGeom, hullData, pose, unitDir);
}

/////////////////////////////////////////////////  sweepBox  //////////////////////////////////////////////////////

bool sweepBox_PlaneGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	PX_UNUSED(geom);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);

//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);

	sweepHit.faceIndex	= 0xFFFFffff; // spec says face index is undefined for planes

	PxPlane worldPlane = getPlane(pose);
	worldPlane.d -=inflation;

	// Find extreme point on the box
	PxVec3 boxPts[8];
	box.computeBoxPoints(boxPts);
	PxU32 index = 0;
	PxReal minDp = PX_MAX_REAL;
	for(PxU32 i=0;i<8;i++)
	{
		const PxReal dp = boxPts[i].dot(worldPlane.n);
	
		if(dp<minDp)
		{
			minDp = dp;
			index = i;
		}
	}

	bool isMtd = hitFlags & PxHitFlag::eMTD;

	if(isMtd)
	{
		// test if box initially overlap with plane
		if(minDp <= -worldPlane.d)
		{
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			//compute Mtd;
			return computePlane_BoxMTD(worldPlane, box, sweepHit);
		}
	}
	else
	{
		if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
		{
			// test if box initially overlap with plane
			if(minDp <= -worldPlane.d)
			{
				sweepHit.flags		= PxHitFlag::eNORMAL;
				sweepHit.distance	= 0.0f;
				sweepHit.normal		= -unitDir;
				return true;
			}
		}
	}

	// Raycast extreme vertex against plane
	bool hitPlane = intersectRayPlane(boxPts[index], unitDir, worldPlane, sweepHit.distance, &sweepHit.position);
	if(hitPlane && sweepHit.distance > 0 && sweepHit.distance <= distance)
	{
		sweepHit.normal = worldPlane.n;
		sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
		return true;
	}
	return false;
}

bool sweepBox_ConvexGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(boxGeom_);

	using namespace Ps::aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	ConvexHullData* hullData = &convexMesh->getHull();

	const Vec3V zeroV = V3Zero();
	const FloatV zero = FZero();

	const PsTransformV boxPose = loadTransformU(boxPose_);
	const PsTransformV convexPose = loadTransformU(pose);

	const PsMatTransformV aToB(convexPose.transformInv(boxPose));

	const Vec3V boxExtents = V3LoadU(box.extents);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);
	
	BoxV boxV(zeroV, boxExtents);
	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, convexGeom.scale.isIdentity());

	const Vec3V worldDir = V3LoadU(unitDir);
	const FloatV dist = FLoad(distance);
	const Vec3V dir = convexPose.rotateInv(V3Neg(V3Scale(worldDir, dist)));

	bool isMtd = hitFlags & PxHitFlag::eMTD;

	FloatV toi;
	Vec3V closestA, normal;
	RelativeConvex<BoxV> convexA(boxV, aToB);
	LocalConvex<ConvexHullV> convexB(convexHull);
	if(!gjkRaycastPenetration< RelativeConvex<BoxV>,LocalConvex<ConvexHullV> >(convexA, convexB, aToB.p, zero, zeroV, dir, toi, normal, closestA, inflation, isMtd))
		return false;

	if(hasInitialOverlap(sweepHit, unitDir, toi, normal, closestA, convexPose, isMtd, true))
		return true;

	sweepHit.flags |= PxHitFlag::ePOSITION;
	const Vec3V destNormal = V3Normalize(convexPose.rotate(normal));
	const FloatV length = FMul(dist, toi);
	const Vec3V worldPointA = convexPose.transform(closestA);
	const Vec3V destWorldPointA = V3ScaleAdd(worldDir, length, worldPointA);
	V3StoreU(destNormal, sweepHit.normal);
	V3StoreU(destWorldPointA, sweepHit.position);
	FStore(length, &sweepHit.distance);

	return computeFaceIndex(sweepHit, hitFlags, convexGeom, hullData, pose, unitDir);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Gu::sweepCapsuleTriangles(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxCapsuleGeometry))
{
	Capsule capsule;
	getCapsule(capsule, geom, pose);
	capsule.radius +=inflation;

	// Compute swept box
	Box capsuleBox;
	computeBoxAroundCapsule(capsule, capsuleBox);

	BoxPadded sweptBounds;
	computeSweptBox(sweptBounds, capsuleBox.extents, capsuleBox.center, capsuleBox.rot, unitDir, distance);

	PxVec3 triNormal;
	return sweepCapsuleTriangles_Precise(nbTris, triangles, capsule, unitDir, distance, cachedIndex, hit, triNormal, hitFlags, doubleSided, &sweptBounds);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool sweepConvex_SphereGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	ConvexHullData* hullData = &convexMesh->getHull();

	const Vec3V zeroV = V3Zero();
	const FloatV zero= FZero();

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);

	const FloatV sphereRadius = FLoad(sphereGeom.radius);

	const PsTransformV sphereTransf = loadTransformU(pose);
	const PsTransformV convexTransf = loadTransformU(convexPose);

	const PsMatTransformV aToB(convexTransf.transformInv(sphereTransf));

	const Vec3V worldDir = V3LoadU(unitDir);
	const FloatV dist = FLoad(distance);
	const Vec3V dir = convexTransf.rotateInv(V3Scale(worldDir, dist));

	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, convexGeom.scale.isIdentity());
	//CapsuleV capsule(zeroV, sphereRadius);
	CapsuleV capsule(aToB.p, sphereRadius);

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	FloatV toi;
	Vec3V closestA, normal;
	LocalConvex<CapsuleV> convexA(capsule);
	LocalConvex<ConvexHullV> convexB(convexHull);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());
	if(!gjkRaycastPenetration< LocalConvex<CapsuleV>, LocalConvex<ConvexHullV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, sphereGeom.radius+inflation, isMtd))
		return false;

	if(hasInitialOverlap(sweepHit, unitDir, toi, normal, closestA, convexPose, isMtd, false))
		return true;

	sweepHit.flags |= PxHitFlag::ePOSITION;
	const Vec3V destNormal = V3Neg(V3Normalize(convexTransf.rotate(normal)));
	const FloatV length = FMul(dist, toi);
	const Vec3V destWorldPointA = convexTransf.transform(closestA);
	V3StoreU(destNormal, sweepHit.normal);
	V3StoreU(destWorldPointA, sweepHit.position);
	FStore(length, &sweepHit.distance);
	sweepHit.faceIndex = 0xffffffff;
	return true;
}

bool sweepConvex_PlaneGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	PX_UNUSED(hitFlags);
	PX_UNUSED(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	ConvexHullData* hullData = &convexMesh->getHull();

	sweepHit.faceIndex	= 0xFFFFffff; // spec says face index is undefined for planes

	const PxVec3* PX_RESTRICT hullVertices = hullData->getHullVertices();
	PxU32 numHullVertices = hullData->mNbHullVertices;

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	const FastVertex2ShapeScaling convexScaling(convexGeom.scale);

	PxPlane plane = getPlane(pose);
	plane.d -=inflation;

	sweepHit.distance	= distance;
	bool status = false;
	bool initialOverlap = false;
	while(numHullVertices--)
	{
		const PxVec3& vertex = *hullVertices++;
		const PxVec3 worldPt = convexPose.transform(convexScaling * vertex);
		float t;
		PxVec3 pointOnPlane;
		if(intersectRayPlane(worldPt, unitDir, plane, t, &pointOnPlane))
		{	
			if(plane.distance(worldPt) <= 0.0f)
			{
				initialOverlap = true;
				break;
				//// Convex touches plane
				//sweepHit.distance		= 0.0f;
				//sweepHit.flags			= PxHitFlag::eNORMAL;
				//sweepHit.normal			= -unitDir;
				//return true;
			}

			if(t > 0.0f && t <= sweepHit.distance)
			{
				sweepHit.distance	= t;
				sweepHit.flags		= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
				sweepHit.position	= pointOnPlane;
				sweepHit.normal		= plane.n;
				status				= true;
			}
		}
	}

	if(initialOverlap)
	{
		if(isMtd)
		{
			sweepHit.flags		= PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return computePlane_ConvexMTD(plane, convexGeom, convexPose, sweepHit);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.flags		= PxHitFlag::eNORMAL;
			sweepHit.normal		= -unitDir;
			return true;
		}
	}
	return status;
}

bool sweepConvex_CapsuleGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	Capsule capsule;
	getCapsule(capsule, capsuleGeom, pose);

	// remove PxHitFlag::eFACE_INDEX, not neeeded to compute.
	PxHitFlags tempHitFlags = hitFlags;
	tempHitFlags &= ~PxHitFlag::eFACE_INDEX;

	if(!sweepCapsule_ConvexGeom(convexGeom, convexPose, capsuleGeom, pose, capsule, -unitDir, distance, sweepHit, tempHitFlags, inflation))
		return false;

	if(sweepHit.flags & PxHitFlag::ePOSITION)
		sweepHit.position += unitDir * sweepHit.distance;

	sweepHit.normal = -sweepHit.normal;
	sweepHit.faceIndex = 0xffffffff;
	return true;
}

bool sweepConvex_BoxGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	Box box;
	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);

	// remove PxHitFlag::eFACE_INDEX, not neeeded to compute.
	PxHitFlags tempHitFlags = hitFlags;
	tempHitFlags &= ~PxHitFlag::eFACE_INDEX;

	if(!sweepBox_ConvexGeom(convexGeom, convexPose, boxGeom, pose, box, -unitDir, distance, sweepHit, tempHitFlags, inflation))
		return false;

	if(sweepHit.flags & PxHitFlag::ePOSITION)
		sweepHit.position += unitDir * sweepHit.distance;

	sweepHit.normal = -sweepHit.normal;
	sweepHit.faceIndex = 0xffffffff;
	return true;
}

bool sweepConvex_ConvexGeom(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	using namespace Ps::aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	const PxConvexMeshGeometry& otherConvexGeom = static_cast<const PxConvexMeshGeometry&>(geom);
	ConvexMesh& otherConvexMesh = *static_cast<ConvexMesh*>(otherConvexGeom.convexMesh);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	ConvexHullData* hullData = &convexMesh->getHull();

	ConvexHullData* otherHullData = &otherConvexMesh.getHull();
	
	const Vec3V zeroV = V3Zero();
	const FloatV zero = FZero();

	const Vec3V otherVScale = V3LoadU_SafeReadW(otherConvexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV otherVQuat = QuatVLoadU(&otherConvexGeom.scale.rotation.x);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);

	const PsTransformV otherTransf = loadTransformU(pose);
	const PsTransformV convexTransf = loadTransformU(convexPose);

	const Vec3V worldDir = V3LoadU(unitDir);
	const FloatV dist = FLoad(distance);
	const Vec3V dir = convexTransf.rotateInv(V3Scale(worldDir, dist));

	const PsMatTransformV aToB(convexTransf.transformInv(otherTransf));
	
	ConvexHullV otherConvexHull(otherHullData, zeroV, otherVScale, otherVQuat, otherConvexGeom.scale.isIdentity());
	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, convexGeom.scale.isIdentity());

	const bool isMtd = hitFlags & PxHitFlag::eMTD;

	FloatV toi;
	Vec3V closestA, normal;
	RelativeConvex<ConvexHullV> convexA(otherConvexHull, aToB);
	LocalConvex<ConvexHullV> convexB(convexHull);
	if(!gjkRaycastPenetration< RelativeConvex<ConvexHullV>, LocalConvex<ConvexHullV> >(convexA, convexB, aToB.p, zero, zeroV, dir, toi, normal, closestA, inflation, isMtd))
		return false;

	if(hasInitialOverlap(sweepHit, unitDir, toi, normal, closestA, convexPose, isMtd, false))
		return true;

	sweepHit.flags |= PxHitFlag::ePOSITION;
	const Vec3V worldPointA = convexTransf.transform(closestA);
	const Vec3V destNormal = V3Neg(V3Normalize(convexTransf.rotate(normal)));
	const FloatV length = FMul(dist, toi);
	V3StoreU(destNormal, sweepHit.normal);
	V3StoreU(worldPointA, sweepHit.position);
	FStore(length, &sweepHit.distance);

	return computeFaceIndex(sweepHit, hitFlags, otherConvexGeom, otherHullData, pose, unitDir);
}
