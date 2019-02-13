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
#include "PxSphereGeometry.h"
#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuVecTriangle.h"
#include "GuSweepTriangleUtils.h"
#include "GuInternal.h"
#include "PsFoundation.h"
#include "GuGJKRaycast.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace physx::shdfnd::aos;  

bool sweepCapsule_BoxGeom(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(hitFlags);

	using namespace Ps::aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents0 = V3LoadU(boxGeom.halfExtents);
	const FloatV dist = FLoad(distance);
	const Vec3V worldDir = V3LoadU(unitDir);

	const PsTransformV capPos = loadTransformU(capsulePose_);
	const PsTransformV boxPos = loadTransformU(pose);

	const PsMatTransformV aToB(boxPos.transformInv(capPos));

	const FloatV capsuleHalfHeight = FLoad(capsuleGeom_.halfHeight);
	const FloatV capsuleRadius = FLoad(lss.radius);

	BoxV box(zeroV, boxExtents0);
	CapsuleV capsule(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	const Vec3V dir = boxPos.rotateInv(V3Neg(V3Scale(worldDir, dist)));

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi = FMax();
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	LocalConvex<CapsuleV> convexA(capsule);
	LocalConvex<BoxV> convexB(box);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), box.getCenter());
	if(!gjkRaycastPenetration<LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, 
		closestA, lss.radius + inflation, isMtd))
		return false;

	sweepHit.flags = PxHitFlag::eNORMAL;
	if(FAllGrtrOrEq(zero, toi))
	{
		//initial overlap
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const Vec3V worldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = boxPos.rotate(normal);
			const FloatV length = toi;
			const Vec3V destWorldPointA = V3NegScaleSub(destNormal, length, worldPointA);
			V3StoreU(destWorldPointA, sweepHit.position);
			V3StoreU(destNormal, sweepHit.normal);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V worldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = boxPos.rotate(normal);
		const FloatV length = FMul(dist, toi);
		const Vec3V destWorldPointA = V3ScaleAdd(worldDir, length, worldPointA);
		V3StoreU(destNormal, sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool sweepBox_SphereGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	PX_UNUSED(hitFlags);
	PX_UNUSED(boxGeom_);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const FloatV sphereRadius = FLoad(sphereGeom.radius);

	const PsTransformV spherePos = loadTransformU(pose);
	const PsTransformV boxPos = loadTransformU(boxPose_);

	const PsMatTransformV aToB(boxPos.transformInv(spherePos));

	BoxV boxV(zeroV, boxExtents);
	CapsuleV capsuleV(aToB.p, sphereRadius);

	//transform into b space
	const Vec3V dir = boxPos.rotateInv(V3Scale(unitDirV, worldDist));

	bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const Vec3V initialSearchDir = V3Sub(capsuleV.getCenter(), boxV.getCenter());
	LocalConvex<CapsuleV> convexA(capsuleV);
	LocalConvex<BoxV> convexB(boxV);
	if(!gjkRaycastPenetration< LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, sphereGeom.radius+inflation, isMtd))
		return false;

	sweepHit.flags = PxHitFlag::eNORMAL;

	//initial overlap
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const Vec3V destWorldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = V3Neg(boxPos.rotate(normal));
			const FloatV length = toi;
			V3StoreU(destNormal, sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = V3Neg(boxPos.rotate(normal));
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(destNormal, sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

bool sweepBox_CapsuleGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	using namespace Ps::aos;
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	PX_UNUSED(hitFlags);
	PX_UNUSED(boxGeom_);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	const FloatV capsuleHalfHeight = FLoad(capsuleGeom.halfHeight);
	const FloatV capsuleRadius = FLoad(capsuleGeom.radius);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const PsTransformV capPos = loadTransformU(pose);
	const PsTransformV boxPos = loadTransformU(boxPose_);

	const PsMatTransformV aToB(boxPos.transformInv(capPos));

	BoxV boxV(zeroV, boxExtents);
	CapsuleV capsuleV(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	//transform into b space
	const Vec3V dir = boxPos.rotateInv(V3Scale(unitDirV, worldDist));

	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	const Vec3V initialSearchDir = V3Sub(capsuleV.getCenter(), boxV.getCenter());
	LocalConvex<CapsuleV> convexA(capsuleV);
	LocalConvex<BoxV> convexB(boxV);
	if(!gjkRaycastPenetration< LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, dir, toi, normal, closestA, capsuleGeom.radius+inflation, isMtd))
		return false;

	sweepHit.flags = PxHitFlag::eNORMAL;

	//initial overlap
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			//initial overlap is toi < 0 
			const FloatV length = toi;
			const Vec3V destWorldPointA = boxPos.transform(closestA);
			const Vec3V destNormal = boxPos.rotate(normal);
			V3StoreU(V3Neg(destNormal), sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
		return true;
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxPos.transform(closestA);
		const Vec3V destNormal = boxPos.rotate(normal);
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(V3Neg(destNormal), sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;	
}

bool sweepBox_BoxGeom(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	PX_UNUSED(boxGeom_);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents0 = V3LoadU(boxGeom.halfExtents);
	const Vec3V boxExtents1 = V3LoadU(box.extents);
	const FloatV worldDist = FLoad(distance);
	const Vec3V  unitDirV = V3LoadU(unitDir);

	const PsTransformV boxTrans0 = loadTransformU(pose);
	const PsTransformV boxTrans1 = loadTransformU(boxPose_);

	const PsMatTransformV aToB(boxTrans1.transformInv(boxTrans0));

	BoxV box0(zeroV, boxExtents0);
	BoxV box1(zeroV, boxExtents1);

	//transform into b space
	const Vec3V dir = boxTrans1.rotateInv(V3Scale(unitDirV, worldDist));
	const bool isMtd = hitFlags & PxHitFlag::eMTD;
	FloatV toi;
	Vec3V closestA, normal;//closestA and normal is in the local space of box
	RelativeConvex<BoxV> convexA(box0, aToB);
	LocalConvex<BoxV> convexB(box1);
	if(!gjkRaycastPenetration<RelativeConvex<BoxV>, LocalConvex<BoxV> >(convexA, convexB, aToB.p, zero, zeroV, dir, toi, normal, closestA, inflation, isMtd))
		return false;
	
	sweepHit.flags = PxHitFlag::eNORMAL;
	if(FAllGrtrOrEq(zero, toi))
	{
		if(isMtd)
		{
			sweepHit.flags |= PxHitFlag::ePOSITION;
			const FloatV length = toi;
			const Vec3V destWorldPointA = boxTrans1.transform(closestA);
			const Vec3V destNormal = V3Normalize(boxTrans1.rotate(normal));
			V3StoreU(V3Neg(destNormal), sweepHit.normal);
			V3StoreU(destWorldPointA, sweepHit.position);
			FStore(length, &sweepHit.distance);
		}
		else
		{
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -unitDir;
		}
	}
	else
	{
		sweepHit.flags |= PxHitFlag::ePOSITION;
		const Vec3V destWorldPointA = boxTrans1.transform(closestA);
		const Vec3V destNormal = V3Normalize(boxTrans1.rotate(normal));
		const FloatV length = FMul(worldDist, toi);
		V3StoreU(V3Neg(destNormal), sweepHit.normal);
		V3StoreU(destWorldPointA, sweepHit.position);
		FStore(length, &sweepHit.distance);
	}
	return true;
}

bool Gu::sweepBoxTriangles(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxBoxGeometry))
{
	PX_UNUSED(hitFlags);

	if(!nbTris)
		return false;

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool doBackfaceCulling = !doubleSided && !meshBothSides;

	Box box;
	buildFrom(box, pose.p, geom.halfExtents, pose.q);

	PxSweepHit sweepHit;
	// Move to AABB space
	Matrix34 worldToBox;
	computeWorldToBoxMatrix(worldToBox, box);

	const PxVec3 localDir = worldToBox.rotate(unitDir);
	const PxVec3 localMotion = localDir * distance;

	const Vec3V base0 = V3LoadU(worldToBox.m.column0);
	const Vec3V base1 = V3LoadU(worldToBox.m.column1);
	const Vec3V base2 = V3LoadU(worldToBox.m.column2);
	const Mat33V matV(base0, base1, base2);
	const Vec3V p	  = V3LoadU(worldToBox.p);
	const PsMatTransformV worldToBoxV(p, matV);

	const FloatV zero = FZero();
	const Vec3V zeroV = V3Zero();
	const Vec3V boxExtents = V3LoadU(box.extents);
	const Vec3V boxDir = V3LoadU(localDir);
	const FloatV inflationV = FLoad(inflation);
	const Vec3V absBoxDir = V3Abs(boxDir);
	const FloatV boxRadiusV = FAdd(V3Dot(absBoxDir, boxExtents), inflationV);
	BoxV boxV(zeroV, boxExtents);

#if PX_DEBUG
	PxU32 totalTestsExpected = nbTris;
	PxU32 totalTestsReal = 0;
	PX_UNUSED(totalTestsExpected);
	PX_UNUSED(totalTestsReal);
#endif

	Vec3V boxLocalMotion = V3LoadU(localMotion);
	Vec3V minClosestA = zeroV, minNormal = zeroV;
	PxU32 minTriangleIndex = 0;
	PxVec3 bestTriNormal(0.0f);
	FloatV dist = FLoad(distance);

	const PsTransformV boxPos = loadTransformU(pose);

	bool status = false;

	const PxU32 idx = cachedIndex ? *cachedIndex : 0;

	for(PxU32 ii=0;ii<nbTris;ii++)
	{
		const PxU32 triangleIndex = getTriangleIndex(ii, idx);

		const Vec3V localV0 =  V3LoadU(triangles[triangleIndex].verts[0]);
		const Vec3V localV1 =  V3LoadU(triangles[triangleIndex].verts[1]);
		const Vec3V localV2 =  V3LoadU(triangles[triangleIndex].verts[2]);

		const Vec3V triV0 = worldToBoxV.transform(localV0);
		const Vec3V triV1 = worldToBoxV.transform(localV1);
		const Vec3V triV2 = worldToBoxV.transform(localV2);

		const Vec3V triNormal = V3Cross(V3Sub(triV2, triV1),V3Sub(triV0, triV1)); 

		if(doBackfaceCulling && FAllGrtrOrEq(V3Dot(triNormal, boxLocalMotion), zero)) // backface culling
			continue;

		const FloatV dp0 = V3Dot(triV0, boxDir);
		const FloatV dp1 = V3Dot(triV1, boxDir);
		const FloatV dp2 = V3Dot(triV2, boxDir);
		
		const FloatV dp = FMin(dp0, FMin(dp1, dp2));

		const Vec3V dpV = V3Merge(dp0, dp1, dp2);

		const FloatV temp1 = FAdd(boxRadiusV, dist);
		const BoolV con0 = FIsGrtr(dp, temp1);
		const BoolV con1 = V3IsGrtr(zeroV, dpV);

		if(BAllEqTTTT(BOr(con0, con1)))
			continue;

#if PX_DEBUG
		totalTestsReal++;
#endif

		TriangleV triangleV(triV0, triV1, triV2);
		
		FloatV lambda;   
		Vec3V closestA, normal;//closestA and normal is in the local space of convex hull
		LocalConvex<TriangleV> convexA(triangleV);
		LocalConvex<BoxV> convexB(boxV);
		const Vec3V initialSearchDir = V3Sub(triangleV.getCenter(), boxV.getCenter());
		if(gjkRaycastPenetration<LocalConvex<TriangleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, zero, zeroV, boxLocalMotion, lambda, normal, closestA, inflation, false))
		{
			//hitCount++;
		
			if(FAllGrtrOrEq(zero, lambda))
			{
				hit.distance	= 0.0f;
				hit.faceIndex	= triangleIndex;
				hit.normal		= -unitDir;
				hit.flags		= PxHitFlag::eNORMAL;
				return true;
			}

			dist = FMul(dist, lambda);
			boxLocalMotion = V3Scale(boxDir, dist);  
			minClosestA = closestA;
			minNormal = normal;
			minTriangleIndex = triangleIndex;
			V3StoreU(triNormal, bestTriNormal);
			status = true;
			if(hitFlags & PxHitFlag::eMESH_ANY)
				break;
		}
	}

	if(!status)
		return false;

	hit.faceIndex	= minTriangleIndex;
	const Vec3V destNormal = V3Neg(V3Normalize(boxPos.rotate(minNormal)));
	const Vec3V destWorldPointA = boxPos.transform(minClosestA);
	V3StoreU(destNormal, hit.normal);
	V3StoreU(destWorldPointA, hit.position);
	FStore(dist, &hit.distance);

	// PT: by design, returned normal is opposed to the sweep direction.
	if(shouldFlipNormal(hit.normal, meshBothSides, doubleSided, bestTriNormal, unitDir))
		hit.normal = -hit.normal;

	hit.flags = PxHitFlag::ePOSITION|PxHitFlag::eNORMAL;
	return true;
}

bool sweepCapsule_SphereGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_PlaneGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_CapsuleGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_BoxGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_BoxGeom_Precise	(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_ConvexGeom		(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_MeshGeom			(GU_CAPSULE_SWEEP_FUNC_PARAMS);
bool sweepCapsule_HeightFieldGeom	(GU_CAPSULE_SWEEP_FUNC_PARAMS);

bool sweepBox_SphereGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_SphereGeom_Precise	(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_PlaneGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_CapsuleGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_CapsuleGeom_Precise	(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_BoxGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_BoxGeom_Precise		(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_ConvexGeom			(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_MeshGeom				(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_HeightFieldGeom		(GU_BOX_SWEEP_FUNC_PARAMS);
bool sweepBox_HeightFieldGeom_Precise(GU_BOX_SWEEP_FUNC_PARAMS);

bool sweepConvex_SphereGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_PlaneGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_CapsuleGeom		(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_BoxGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_ConvexGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_MeshGeom			(GU_CONVEX_SWEEP_FUNC_PARAMS);
bool sweepConvex_HeightFieldGeom	(GU_CONVEX_SWEEP_FUNC_PARAMS);

static bool sweepCapsule_HeightfieldUnregistered(GU_CAPSULE_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(capsuleGeom_);
	PX_UNUSED(capsulePose_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(lss);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Height Field Sweep test called with height fields unregistered ");
	return false;
}
static bool sweepBox_HeightfieldUnregistered(GU_BOX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(boxPose_);
	PX_UNUSED(boxGeom_);
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(box);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Height Field Sweep test called with height fields unregistered ");
	return false;
}
static bool sweepConvex_HeightfieldUnregistered(GU_CONVEX_SWEEP_FUNC_PARAMS)
{
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(convexGeom);
	PX_UNUSED(convexPose);
	PX_UNUSED(unitDir);
	PX_UNUSED(distance);
	PX_UNUSED(sweepHit);
	PX_UNUSED(hitFlags);
	PX_UNUSED(inflation);
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Height Field Sweep test called with height fields unregistered ");
	return false;
}

Gu::GeomSweepFuncs gGeomSweepFuncs =
{
	{
		sweepCapsule_SphereGeom,
		sweepCapsule_PlaneGeom,
		sweepCapsule_CapsuleGeom,
		sweepCapsule_BoxGeom,
		sweepCapsule_ConvexGeom,
		sweepCapsule_MeshGeom,
		sweepCapsule_HeightfieldUnregistered
	},
	{
		sweepCapsule_SphereGeom,
		sweepCapsule_PlaneGeom,
		sweepCapsule_CapsuleGeom,
		sweepCapsule_BoxGeom_Precise,
		sweepCapsule_ConvexGeom,
		sweepCapsule_MeshGeom ,
		sweepCapsule_HeightfieldUnregistered
	},
	{
		sweepBox_SphereGeom,
		sweepBox_PlaneGeom,
		sweepBox_CapsuleGeom,
		sweepBox_BoxGeom,
		sweepBox_ConvexGeom,
		sweepBox_MeshGeom,		
		sweepBox_HeightfieldUnregistered
	},
	{
		sweepBox_SphereGeom_Precise,
		sweepBox_PlaneGeom,
		sweepBox_CapsuleGeom_Precise,
		sweepBox_BoxGeom_Precise,
		sweepBox_ConvexGeom,
		sweepBox_MeshGeom,		
		sweepBox_HeightfieldUnregistered
	},
	{
		sweepConvex_SphereGeom,		// 0
		sweepConvex_PlaneGeom,		// 1
		sweepConvex_CapsuleGeom,	// 2
		sweepConvex_BoxGeom,		// 3
		sweepConvex_ConvexGeom,		// 4
		sweepConvex_MeshGeom,		// 5			
		sweepConvex_HeightfieldUnregistered	// 6
	}
};

PX_PHYSX_COMMON_API const GeomSweepFuncs& Gu::getSweepFuncTable()
{
	return gGeomSweepFuncs;
}

void registerHeightFields_Sweeps()
{
	gGeomSweepFuncs.capsuleMap[PxGeometryType::eHEIGHTFIELD] = sweepCapsule_HeightFieldGeom;
	gGeomSweepFuncs.preciseCapsuleMap[PxGeometryType::eHEIGHTFIELD] = sweepCapsule_HeightFieldGeom;
	gGeomSweepFuncs.boxMap[PxGeometryType::eHEIGHTFIELD] = sweepBox_HeightFieldGeom;
	gGeomSweepFuncs.preciseBoxMap[PxGeometryType::eHEIGHTFIELD] = sweepBox_HeightFieldGeom_Precise;
	gGeomSweepFuncs.convexMap[PxGeometryType::eHEIGHTFIELD] = sweepConvex_HeightFieldGeom;
}
