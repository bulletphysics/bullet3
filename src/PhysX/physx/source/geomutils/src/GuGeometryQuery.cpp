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

#include "PxGeometryQuery.h"
#include "GuInternal.h"
#include "GuOverlapTests.h"
#include "GuSweepTests.h"
#include "GuRaycastTests.h"
#include "GuBoxConversion.h"
#include "GuTriangleMesh.h"
#include "GuMTD.h"
#include "GuBounds.h"
#include "GuDistancePointSegment.h"
#include "GuConvexMesh.h"
#include "GuDistancePointBox.h"
#include "PsFPU.h"
#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxTriangleMeshGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxHeightFieldGeometry.h"

using namespace physx;
using namespace Gu;

extern GeomSweepFuncs gGeomSweepFuncs;
extern GeomOverlapTable gGeomOverlapMethodTable[];
extern RaycastFunc gRaycastMap[PxGeometryType::eGEOMETRY_COUNT];

bool PxGeometryQuery::isValid(const PxGeometry& geom)
{
	switch(geom.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
			if(!sphereGeom.isValid())
				return false;
			break;
		}
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);
			if(!capsuleGeom.isValid())
				return false;
			break;
		}
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
			if(!boxGeom.isValid())
				return false;
			break;
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);
			if(!convexGeom.isValid())
				return false;
			break;
		}
		case PxGeometryType::ePLANE:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			break;
	}	
	return true;
}

bool PxGeometryQuery::sweep(const PxVec3& unitDir, const PxReal distance,
							const PxGeometry& geom0, const PxTransform& pose0,
							const PxGeometry& geom1, const PxTransform& pose1,
							PxSweepHit& sweepHit, PxHitFlags hitFlags,
							const PxReal inflation)
{
	PX_SIMD_GUARD;
	PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxGeometryQuery::sweep(): pose0 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxGeometryQuery::sweep(): pose1 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(unitDir.isFinite(), "PxGeometryQuery::sweep(): unitDir is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(distance), "PxGeometryQuery::sweep(): distance is not valid.", false);
	PX_CHECK_AND_RETURN_VAL((distance >= 0.0f && !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP)) || distance > 0.0f,
		"PxGeometryQuery::sweep(): sweep distance must be >=0 or >0 with eASSUME_NO_INITIAL_OVERLAP.", 0);
#if PX_CHECKED
	if(!PxGeometryQuery::isValid(geom0))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Provided geometry 0 is not valid");
		return false;
	}
	if(!PxGeometryQuery::isValid(geom1))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Provided geometry 1 is not valid");
		return false;
	}
#endif // PX_CHECKED

	const GeomSweepFuncs& sf = gGeomSweepFuncs;

	switch(geom0.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);

			// PT: TODO: technically this capsule with 0.0 half-height is invalid ("isValid" returns false)
			const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);

			const Capsule worldCapsule(pose0.p, pose0.p, sphereGeom.radius);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];

			return func(geom1, pose1, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation);
		}

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);

			Capsule worldCapsule;
			getCapsule(worldCapsule, capsuleGeom, pose0);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];

			return func(geom1, pose1, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation);
		}

		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);

			Box box;
			buildFrom(box, pose0.p, boxGeom.halfExtents, pose0.q);

			const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
			const SweepBoxFunc func = precise ? sf.preciseBoxMap[geom1.getType()] : sf.boxMap[geom1.getType()];

			return func(geom1, pose1, boxGeom, pose0, box, unitDir, distance, sweepHit, hitFlags, inflation);
		}

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);

			const SweepConvexFunc func = sf.convexMap[geom1.getType()];

			return func(geom1, pose1, convexGeom, pose0, unitDir, distance, sweepHit, hitFlags, inflation);
		}
		case PxGeometryType::ePLANE:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_CHECK_MSG(false, "PxGeometryQuery::sweep(): first geometry object parameter must be sphere, capsule, box or convex geometry.");
	}

	return false;
}

///////////////////////////////////////////////////////////////////////////////

bool PxGeometryQuery::overlap(	const PxGeometry& geom0, const PxTransform& pose0,
								const PxGeometry& geom1, const PxTransform& pose1)
{
	PX_SIMD_GUARD;
	return Gu::overlap(geom0, pose0, geom1, pose1, gGeomOverlapMethodTable);
}

///////////////////////////////////////////////////////////////////////////////
PxU32 PxGeometryQuery::raycast(	const PxVec3& rayOrigin, const PxVec3& rayDir,
								const PxGeometry& geom, const PxTransform& pose,
								PxReal maxDist, PxHitFlags hitFlags,
								PxU32 maxHits, PxRaycastHit* PX_RESTRICT rayHits)
{
	PX_SIMD_GUARD;
	PX_CHECK_AND_RETURN_VAL(rayDir.isFinite(), "PxGeometryQuery::raycast(): rayDir is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(rayOrigin.isFinite(), "PxGeometryQuery::raycast(): rayOrigin is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxGeometryQuery::raycast(): pose is not valid.", 0);
	PX_CHECK_AND_RETURN_VAL(maxDist >= 0.0f, "PxGeometryQuery::raycast(): maxDist is negative.", false);
	PX_CHECK_AND_RETURN_VAL(PxIsFinite(maxDist), "PxGeometryQuery::raycast(): maxDist is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs(rayDir.magnitudeSquared()-1)<1e-4f, "PxGeometryQuery::raycast(): ray direction must be unit vector.", false);

	const RaycastFunc func = gRaycastMap[geom.getType()];
	return func(geom, pose, rayOrigin, rayDir, maxDist, hitFlags, maxHits, rayHits);
}

///////////////////////////////////////////////////////////////////////////////

bool pointConvexDistance(PxVec3& normal_, PxVec3& closestPoint_, PxReal& sqDistance, const PxVec3& pt, const ConvexMesh* convexMesh, const PxMeshScale& meshScale, const PxTransform& convexPose);

PxReal PxGeometryQuery::pointDistance(const PxVec3& point, const PxGeometry& geom, const PxTransform& pose, PxVec3* closestPoint)
{
	PX_SIMD_GUARD;
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxGeometryQuery::pointDistance(): pose is not valid.", false);

	switch(geom.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

			const PxReal r = sphereGeom.radius;

			PxVec3 delta = point - pose.p;
			const PxReal d = delta.magnitude();
			if(d<=r)
				return 0.0f;

			if(closestPoint)
			{
				delta /= d;
				*closestPoint = pose.p + delta * r;
			}

			return (d - r)*(d - r);
		}
		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& capsGeom = static_cast<const PxCapsuleGeometry&>(geom);

			Capsule capsule;
			getCapsule(capsule, capsGeom, pose);

			const PxReal r = capsGeom.radius;

			PxReal param;
			const PxReal sqDistance = distancePointSegmentSquared(capsule, point, &param);
			if(sqDistance<=r*r)
				return 0.0f;

			const PxReal d = physx::intrinsics::sqrt(sqDistance);

			if(closestPoint)
			{
				const PxVec3 cp = capsule.getPointAt(param);

				PxVec3 delta = point - cp;
				delta.normalize();

				*closestPoint = cp + delta * r;
			}
			return (d - r)*(d - r);
		}
		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

			Box obb;
			buildFrom(obb, pose.p, boxGeom.halfExtents, pose.q);

			PxVec3 boxParam;
			const PxReal sqDistance = distancePointBoxSquared(point, obb, &boxParam);
			if(closestPoint && sqDistance!=0.0f)
			{
				*closestPoint = obb.transform(boxParam);
			}
			return sqDistance;
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

			PxVec3 normal, cp;
			PxReal sqDistance;
			const bool intersect = pointConvexDistance(normal, cp, sqDistance, point, static_cast<ConvexMesh*>(convexGeom.convexMesh), convexGeom.scale, pose);
			if(!intersect && closestPoint)
				*closestPoint = cp;
			return sqDistance;
		}
		case PxGeometryType::ePLANE:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_CHECK_MSG(false, "PxGeometryQuery::pointDistance(): geometry object parameter must be sphere, capsule box or convex geometry.");
			break;
	}
	return -1.0f;
}

///////////////////////////////////////////////////////////////////////////////

PxBounds3 PxGeometryQuery::getWorldBounds(const PxGeometry& geom, const PxTransform& pose, float inflation)
{
	PX_SIMD_GUARD;
	PX_CHECK_AND_RETURN_VAL(pose.isValid(), "PxGeometryQuery::getWorldBounds(): pose is not valid.", PxBounds3::empty());

	PxBounds3 bounds;
	Gu::computeBounds(bounds, geom, pose, 0.0f, NULL, inflation);
	PX_ASSERT(bounds.isValid());
	return bounds;
}

///////////////////////////////////////////////////////////////////////////////

extern GeomMTDFunc	gGeomMTDMethodTable[][PxGeometryType::eGEOMETRY_COUNT];

bool PxGeometryQuery::computePenetration(	PxVec3& mtd, PxF32& depth,
											const PxGeometry& geom0, const PxTransform& pose0,
											const PxGeometry& geom1, const PxTransform& pose1)
{
	PX_SIMD_GUARD;
	PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxGeometryQuery::computePenetration(): pose0 is not valid.", false);
	PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxGeometryQuery::computePenetration(): pose1 is not valid.", false);

	if(geom0.getType() > geom1.getType())
	{
		GeomMTDFunc mtdFunc = gGeomMTDMethodTable[geom1.getType()][geom0.getType()];
		PX_ASSERT(mtdFunc);
		if(!mtdFunc(mtd, depth, geom1, pose1, geom0, pose0))
			return false;
		mtd = -mtd;
		return true;
	}
	else
	{
		GeomMTDFunc mtdFunc = gGeomMTDMethodTable[geom0.getType()][geom1.getType()];
		PX_ASSERT(mtdFunc);
		return mtdFunc(mtd, depth, geom0, pose0, geom1, pose1);
	}
}
