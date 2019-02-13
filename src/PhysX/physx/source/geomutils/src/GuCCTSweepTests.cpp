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

#include "GuSweepTests.h"
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "PxSphereGeometry.h"
#include "GuDistanceSegmentBox.h"
#include "GuDistancePointBox.h"
#include "GuSweepBoxSphere.h"
#include "GuSweepCapsuleBox.h"
#include "GuSweepBoxBox.h"
#include "GuSweepBoxTriangle_SAT.h"
#include "GuSweepTriangleUtils.h"
#include "GuInternal.h"
#include "PsVecMath.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace Ps::aos;

static const bool gValidateBoxRadiusComputation = false;

///////////////////////////////////////////

bool sweepCapsule_BoxGeom_Precise(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	PX_UNUSED(inflation);
	PX_UNUSED(capsulePose_);
	PX_UNUSED(capsuleGeom_);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	if (lss.p0 == lss.p1)  // The capsule is actually a sphere
	{
		//TODO: Check if this is really faster than using a "sphere-aware" version of sweepCapsuleBox

		Box box;	buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);
		if(!sweepBoxSphere(box, lss.radius, lss.p0, unitDir, distance, sweepHit.distance, sweepHit.normal, hitFlags))
			return false;

		sweepHit.normal = -sweepHit.normal;
		sweepHit.flags = PxHitFlag::eNORMAL;
		
		if(hitFlags & PxHitFlag::ePOSITION && sweepHit.distance!=0.0f)
		{
			// The sweep test doesn't compute the impact point automatically, so we have to do it here.
			const PxVec3 newSphereCenter = lss.p0 + unitDir * sweepHit.distance;
			PxVec3 closest;
			const PxReal d = distancePointBoxSquared(newSphereCenter, box.center, box.extents, box.rot, &closest);
			PX_UNUSED(d);
			// Compute point on the box, after sweep
			closest = box.rotate(closest);
			sweepHit.position = closest + box.center;
			sweepHit.flags |= PxHitFlag::ePOSITION;
		}
	}
	else
	{
		if(!sweepCapsuleBox(lss, pose, boxGeom.halfExtents, unitDir, distance, sweepHit.position, sweepHit.distance, sweepHit.normal, hitFlags))
			return false;

		sweepHit.flags = PxHitFlag::eNORMAL;
		
		if((hitFlags & PxHitFlag::ePOSITION) && sweepHit.distance!=0.0f)
		{
			// The sweep test doesn't compute the impact point automatically, so we have to do it here.
			Capsule movedCaps = lss;
			movedCaps.p0 += unitDir * sweepHit.distance;
			movedCaps.p1 += unitDir * sweepHit.distance;

			Box box;
			buildFrom(box, pose.p, boxGeom.halfExtents, pose.q);

			PxVec3 closest;
			const PxReal d = distanceSegmentBoxSquared(movedCaps, box, NULL, &closest);
			PX_UNUSED(d);
			// Compute point on the box, after sweep
			closest = pose.q.rotate(closest);
			sweepHit.position = closest + pose.p;
			sweepHit.flags |= PxHitFlag::ePOSITION;
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool sweepBox_SphereGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);

	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	// PT: move to relative space
	const Box relBox(box.center - pose.p, box.extents, box.rot);

	const PxReal sphereRadius = sphereGeom.radius + inflation;

	if(!sweepBoxSphere(relBox, sphereRadius, PxVec3(0), -unitDir, distance, sweepHit.distance, sweepHit.normal, hitFlags))
		return false;

	sweepHit.flags = PxHitFlag::eNORMAL;

	if((hitFlags & PxHitFlag::ePOSITION) && sweepHit.distance!=0.0f)
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		const PxVec3 motion = sweepHit.distance * unitDir;
		const PxVec3 newSphereCenter = - motion;
		PxVec3 closest;
		const PxReal d = distancePointBoxSquared(newSphereCenter, relBox.center, relBox.extents, relBox.rot, &closest);
		PX_UNUSED(d);
		// Compute point on the box, after sweep
		sweepHit.position = relBox.rotate(closest) + box.center + motion;	// PT: undo move to local space here
		sweepHit.flags |= PxHitFlag::ePOSITION;
	}
	return true;
}

bool sweepBox_CapsuleGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	PX_UNUSED(inflation);
	PX_UNUSED(boxGeom_);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	// PT: move to relative space
	const PxVec3 delta = box.center - pose.p;
	Box relBox(delta, box.extents, box.rot);

	Capsule capsule;
	const PxVec3 halfHeightVector = getCapsuleHalfHeightVector(pose, capsuleGeom);
	capsule.p0 = halfHeightVector;
	capsule.p1 = -halfHeightVector;
	capsule.radius = capsuleGeom.radius;

	// PT: TODO: remove this. We convert to PxTansform here but inside sweepCapsuleBox we convert back to a matrix.
	const PxTransform boxWorldPose(delta, boxPose_.q);

	PxVec3 n;
	if(!sweepCapsuleBox(capsule, boxWorldPose, relBox.extents, -unitDir, distance, sweepHit.position, sweepHit.distance, n, hitFlags))
		return false;

	sweepHit.normal = -n;
	sweepHit.flags = PxHitFlag::eNORMAL;

	if((hitFlags & PxHitFlag::ePOSITION) && sweepHit.distance!=0.0f)
	{
		// The sweep test doesn't compute the impact point automatically, so we have to do it here.
		relBox.center += (unitDir * sweepHit.distance);
		PxVec3 closest;
		const PxReal d = distanceSegmentBoxSquared(capsule, relBox, NULL, &closest);
		PX_UNUSED(d);
		// Compute point on the box, after sweep
		sweepHit.position = relBox.transform(closest) + pose.p;	// PT: undo move to local space here
		sweepHit.flags |= PxHitFlag::ePOSITION;
	}
	return true;
}

bool sweepBox_BoxGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	PX_UNUSED(inflation);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	// PT: move to local space
	const Box relBox(box.center - pose.p, box.extents, box.rot);
	Box staticBox;	buildFrom(staticBox, PxVec3(0), boxGeom.halfExtents, pose.q);

	if(!sweepBoxBox(relBox, staticBox, unitDir, distance, hitFlags, sweepHit))
		return false;

	if(sweepHit.distance!=0.0f)
		sweepHit.position += pose.p;	// PT: undo move to local space
	return true;
}

// PT: test: new version for CCT, based on code for general sweeps. Just to check it works or not with rotations
// TODO: refactor this and the similar code in sweptBox for box-vs-mesh. Not so easy though.
static bool sweepBoxVsTriangles(PxU32 nbTris, const PxTriangle* triangles, const Box& box, const PxVec3& unitDir, const PxReal distance, PxSweepHit& sweepHit,
								PxHitFlags hitFlags, bool isDoubleSided, const PxU32* cachedIndex)
{
	if(!nbTris)
		return false;

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool doBackfaceCulling = !isDoubleSided && !meshBothSides;

	// Move to AABB space
	Matrix34 worldToBox;
	computeWorldToBoxMatrix(worldToBox, box);

	const PxVec3 localDir = worldToBox.rotate(unitDir);
	const PxVec3 localMotion = localDir * distance;

	bool status = false;
	sweepHit.distance = distance;	//was PX_MAX_F32, but that may trigger an assert in the caller!

	const PxVec3 oneOverMotion(
		localDir.x!=0.0f ? 1.0f/localMotion.x : 0.0f,
		localDir.y!=0.0f ? 1.0f/localMotion.y : 0.0f,
		localDir.z!=0.0f ? 1.0f/localMotion.z : 0.0f);

// PT: experimental code, don't clean up before I test it more and validate it

// Project box
/*float boxRadius0 =
			PxAbs(dir.x) * box.extents.x
		+	PxAbs(dir.y) * box.extents.y
		+	PxAbs(dir.z) * box.extents.z;*/

float boxRadius =
			PxAbs(localDir.x) * box.extents.x
		+	PxAbs(localDir.y) * box.extents.y
		+	PxAbs(localDir.z) * box.extents.z;

if(gValidateBoxRadiusComputation)	// PT: run this to check the box radius is correctly computed
{
	PxVec3 boxVertices2[8];
	box.computeBoxPoints(boxVertices2);
	float dpmin = FLT_MAX;
	float dpmax = -FLT_MAX;
	for(int i=0;i<8;i++)
	{
		const float dp = boxVertices2[i].dot(unitDir);
		if(dp<dpmin)	dpmin = dp;
		if(dp>dpmax)	dpmax = dp;
	}
	const float goodRadius = (dpmax-dpmin)/2.0f;
	PX_UNUSED(goodRadius);
}

const float dpc0 = box.center.dot(unitDir);
float localMinDist = 1.0f;
#if PX_DEBUG
	PxU32 totalTestsExpected = nbTris;
	PxU32 totalTestsReal = 0;
	PX_UNUSED(totalTestsExpected);
	PX_UNUSED(totalTestsReal);
#endif

	const PxU32 idx = cachedIndex ? *cachedIndex : 0;

	PxVec3 bestTriNormal(0.0f);

	for(PxU32 ii=0;ii<nbTris;ii++)
	{
		const PxU32 triangleIndex = getTriangleIndex(ii, idx);

		const PxTriangle& tri = triangles[triangleIndex];

		if(!cullTriangle(tri.verts, unitDir, boxRadius, localMinDist*distance, dpc0))
			continue;

#if PX_DEBUG
		totalTestsReal++;
#endif
		// Move to box space
		const PxTriangle currentTriangle(
			worldToBox.transform(tri.verts[0]),
			worldToBox.transform(tri.verts[1]),
			worldToBox.transform(tri.verts[2]));

		PxF32 t = PX_MAX_F32;		// could be better!
		if(triBoxSweepTestBoxSpace(currentTriangle, box.extents, localMotion, oneOverMotion, localMinDist, t, doBackfaceCulling))
		{
			if(t < localMinDist)
			{
				// PT: test if shapes initially overlap
				if(t==0.0f)
					return setInitialOverlapResults(sweepHit, unitDir, triangleIndex);

				localMinDist		= t;
				sweepHit.distance	= t * distance;
				sweepHit.faceIndex	= triangleIndex;
				status				= true;

				// PT: TODO: optimize this.... already computed in triBoxSweepTestBoxSpace...
				currentTriangle.denormalizedNormal(bestTriNormal);

				if(hitFlags & PxHitFlag::eMESH_ANY)
					break;
			}
		}
	}

	if(status)
	{
		sweepHit.flags = PxHitFlags(0);

		// PT: TODO: refactor with computeBoxLocalImpact (TA34704)
		if(hitFlags & (PxHitFlag::eNORMAL|PxHitFlag::ePOSITION))
		{			
			const PxTriangle& tri = triangles[sweepHit.faceIndex];

			// Move to box space
			const PxTriangle currentTriangle(
				worldToBox.transform(tri.verts[0]),
				worldToBox.transform(tri.verts[1]),
				worldToBox.transform(tri.verts[2]));

			computeBoxTriImpactData(sweepHit.position, sweepHit.normal, box.extents, localDir, currentTriangle, sweepHit.distance);

			if(hitFlags & PxHitFlag::eNORMAL)
			{
				PxVec3 localNormal = sweepHit.normal;	// PT: both local space & local variable
				localNormal.normalize();

				if(shouldFlipNormal(localNormal, meshBothSides, isDoubleSided, bestTriNormal, localDir))
					localNormal = -localNormal;

				sweepHit.normal = box.rotate(localNormal);
				sweepHit.flags |= PxHitFlag::eNORMAL;
			}

			if(hitFlags & PxHitFlag::ePOSITION)
			{
				sweepHit.position = box.rotate(sweepHit.position) + box.center;
				sweepHit.flags |= PxHitFlag::ePOSITION;
			}
		}
	}
	return status;
}

bool sweepBox_HeightFieldGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(inflation);
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);

	const PxHeightFieldGeometry& heightFieldGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	// Compute swept box
	Box sweptBox;
	computeSweptBox(sweptBox, box.extents, box.center, box.rot, unitDir, distance);

	//### Temp hack until we can directly collide the OBB against the HF
	const PxTransform sweptBoxTR = sweptBox.getTransform();
	const PxBounds3 bounds = PxBounds3::poseExtent(sweptBoxTR, sweptBox.extents);

	sweepHit.distance = PX_MAX_F32;

	struct LocalReport : EntityReport<PxU32>
	{
		virtual bool onEvent(PxU32 nb, PxU32* indices)
		{
			for(PxU32 i=0; i<nb; i++)
			{
				const PxU32 triangleIndex = indices[i];

				PxTriangle currentTriangle;	// in world space
				mHFUtil->getTriangle(*mPose, currentTriangle, NULL, NULL, triangleIndex, true, true);

				PxSweepHit sweepHit_;
				const bool b = sweepBoxVsTriangles(1, &currentTriangle, mBox, mDir, mDist, sweepHit_, mHitFlags, mIsDoubleSided, NULL);
				if(b && sweepHit_.distance<mHit->distance)
				{
					*mHit = sweepHit_;
					mHit->faceIndex	= triangleIndex;
					mStatus			= true;
				}
			}
			return true;
		}

		const HeightFieldUtil*	mHFUtil;
		const PxTransform*		mPose;
		PxSweepHit*				mHit;
		bool					mStatus;
		Box						mBox;
		PxVec3					mDir;
		float					mDist;
		PxHitFlags				mHitFlags;
		bool					mIsDoubleSided;
	} myReport;

	HeightFieldUtil hfUtil(heightFieldGeom);

	myReport.mBox		= box;
	myReport.mDir		= unitDir;
	myReport.mDist		= distance;
	myReport.mHitFlags	= hitFlags;
	myReport.mHFUtil	= &hfUtil;
	myReport.mStatus	= false;
	myReport.mPose		= &pose;
	myReport.mHit		= &sweepHit;
	const PxU32 meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	myReport.mIsDoubleSided = (heightFieldGeom.heightFieldFlags & PxMeshGeometryFlag::eDOUBLE_SIDED) || meshBothSides;

	hfUtil.overlapAABBTriangles(pose, bounds, GuHfQueryFlags::eWORLD_SPACE, &myReport);

	return myReport.mStatus;
}

bool Gu::sweepBoxTriangles_Precise(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxBoxGeometry))
{
	PX_UNUSED(inflation);

	Box box;
	buildFrom(box, pose.p, geom.halfExtents, pose.q);

	return sweepBoxVsTriangles(nbTris, triangles, box, unitDir, distance, hit, hitFlags, doubleSided, cachedIndex);
}
