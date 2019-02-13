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

#include "GuOverlapTests.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionSphereBox.h"
#include "GuDistancePointSegment.h"
#include "GuDistanceSegmentBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuSphere.h"
#include "GuBoxConversion.h"
#include "GuInternal.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecBox.h"
#include "GuConvexMesh.h"
#include "GuHillClimbing.h"
#include "GuGJK.h"

using namespace physx;
using namespace Cm;
using namespace Gu;

// PT: TODO: why don't we use ShapeData for overlaps?

//returns the maximal vertex in shape space
// PT: this function should be removed. We already have 2 different project hull functions in PxcShapeConvex & GuGJKObjectSupport, this one looks like a weird mix of both!
static PxVec3 projectHull_(	const ConvexHullData& hull,
							float& minimum, float& maximum,
							const PxVec3& localDir, // expected to be normalized
							const PxMat33& vert2ShapeSkew)
{
	PX_ASSERT(localDir.isNormalized());

	//use property that x|My == Mx|y for symmetric M to avoid having to transform vertices.
	const PxVec3 vertexSpaceDir = vert2ShapeSkew * localDir;

	const PxVec3* Verts = hull.getHullVertices();
	const PxVec3* bestVert = NULL;

	if(!hull.mBigConvexRawData)	// Brute-force, local space. Experiments show break-even point is around 32 verts.
	{
		PxU32 NbVerts = hull.mNbHullVertices;
		float min_ = PX_MAX_F32;
		float max_ = -PX_MAX_F32;
		while(NbVerts--)
		{
			const float dp = (*Verts).dot(vertexSpaceDir);
			min_ = physx::intrinsics::selectMin(min_, dp);
			if(dp > max_)	{ max_ = dp; bestVert = Verts; }

			Verts++;
		}
		minimum = min_;
		maximum = max_;

		PX_ASSERT(bestVert != NULL);

		return vert2ShapeSkew * *bestVert;
	}
	else //*/if(1)	// This version is better for objects with a lot of vertices
	{
		const PxU32 Offset = ComputeCubemapNearestOffset(vertexSpaceDir, hull.mBigConvexRawData->mSubdiv);
		PxU32 MinID = hull.mBigConvexRawData->mSamples[Offset];
		PxU32 MaxID = hull.mBigConvexRawData->getSamples2()[Offset];

		localSearch(MinID, -vertexSpaceDir, Verts, hull.mBigConvexRawData);
		localSearch(MaxID, vertexSpaceDir, Verts, hull.mBigConvexRawData);

		minimum = (Verts[MinID].dot(vertexSpaceDir));
		maximum = (Verts[MaxID].dot(vertexSpaceDir));

		PX_ASSERT(maximum >= minimum);

		return vert2ShapeSkew * Verts[MaxID];
	}
}

static bool intersectSphereConvex(const PxTransform& sphereTransform, float radius, const ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
						   PxVec3*)
{
	using namespace Ps::aos;
	const Vec3V zeroV = V3Zero();
	const ConvexHullData* hullData = &mesh.getHull();
	const FloatV sphereRadius = FLoad(radius);
	const Vec3V vScale	= V3LoadU_SafeReadW(meshScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&meshScale.rotation.x);

	const PsMatTransformV aToB(convexGlobalPose.transformInv(sphereTransform));
	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, meshScale.isIdentity());
	CapsuleV capsule(aToB.p, sphereRadius);

	Vec3V contactA, contactB, normal;
	FloatV dist;
	LocalConvex<CapsuleV> convexA(capsule);
	LocalConvex<ConvexHullV> convexB(convexHull);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());

	GjkStatus status = gjk(convexA, convexB, initialSearchDir, FZero(), contactA, contactB, normal, dist);

	return status == GJK_CONTACT;
}

static bool intersectCapsuleConvex(	const PxCapsuleGeometry& capsGeom, const PxTransform& capsGlobalPose,
									const ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
									PxVec3*)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();
	const ConvexHullData* hull = &mesh.getHull();

	const FloatV capsuleHalfHeight = FLoad(capsGeom.halfHeight);
	const FloatV capsuleRadius = FLoad(capsGeom.radius);

	const Vec3V vScale	= V3LoadU_SafeReadW(meshScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&meshScale.rotation.x);

	const PsMatTransformV aToB(convexGlobalPose.transformInv(capsGlobalPose));

	ConvexHullV convexHull(hull, zeroV, vScale, vQuat, meshScale.isIdentity());
	CapsuleV capsule(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	Vec3V contactA, contactB, normal;
	FloatV dist;
	LocalConvex<CapsuleV> convexA(capsule);
	LocalConvex<ConvexHullV> convexB(convexHull);
	const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());
	
	GjkStatus  status = gjk(convexA, convexB, initialSearchDir, FZero(), contactA, contactB, normal, dist);

	return status == GJK_CONTACT;
}

static bool intersectBoxConvex(const PxBoxGeometry& boxGeom, const PxTransform& boxGlobalPose,
								const ConvexMesh& mesh, const PxMeshScale& meshScale, const PxTransform& convexGlobalPose,
								PxVec3*)
{
	// AP: see archived non-GJK version in //sw/physx/dev/pterdiman/graveyard/contactConvexBox.cpp
	using namespace Ps::aos;
	const Vec3V zeroV = V3Zero();
	const ConvexHullData* hull = &mesh.getHull();

	const Vec3V vScale	= V3LoadU_SafeReadW(meshScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&meshScale.rotation.x);
	const Vec3V boxExtents = V3LoadU(boxGeom.halfExtents);
	const PsMatTransformV aToB(convexGlobalPose.transformInv(boxGlobalPose));

	ConvexHullV convexHull(hull, zeroV, vScale, vQuat, meshScale.isIdentity());
	BoxV box(zeroV, boxExtents);

	Vec3V contactA, contactB, normal;
	FloatV dist;
	RelativeConvex<BoxV> convexA(box, aToB);
	LocalConvex<ConvexHullV> convexB(convexHull);

	GjkStatus status = gjk(convexA, convexB, aToB.p, FZero(), contactA, contactB, normal, dist);

	//PX_PRINTF("BOX status = %i, overlap = %i, PxVec3(%f, %f, %f)\n", status, overlap, boxGlobalPose.p.x, boxGlobalPose.p.y, boxGlobalPose.p.z);

	return status == GJK_CONTACT;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxVec3* getCachedAxis(TriggerCache* cache)
{
	if(cache && cache->state==TRIGGER_OVERLAP)
		return &cache->dir;
	else
		return NULL;
}

static PX_FORCE_INLINE bool updateTriggerCache(bool overlap, TriggerCache* cache)
{
	if(cache)
	{
		if(overlap)
			cache->state = TRIGGER_OVERLAP;
		else
			cache->state = TRIGGER_DISJOINT;
	}
	return overlap;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sphere-vs-shape

static bool GeomOverlapCallback_SphereSphere(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eSPHERE);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom0 = static_cast<const PxSphereGeometry&>(geom0);
	const PxSphereGeometry& sphereGeom1 = static_cast<const PxSphereGeometry&>(geom1);

	const PxVec3 delta = pose1.p - pose0.p;
	const PxReal r = sphereGeom0.radius + sphereGeom1.radius;
	return delta.magnitudeSquared() <= r*r;	// PT: objects are defined as closed, so we return 'true' in case of equality
}

static bool GeomOverlapCallback_SpherePlane(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::ePLANE);
	PX_UNUSED(cache);
	PX_UNUSED(geom1);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);

	return getPlane(pose1).distance(pose0.p) <= sphereGeom.radius;	// PT: objects are defined as closed, so we return 'true' in case of equality
}

static bool GeomOverlapCallback_SphereCapsule(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	// PT: TODO: remove this useless conversion
	const PxVec3 capsuleHalfHeightVector = getCapsuleHalfHeightVector(pose1, capsuleGeom);
	const PxReal r = sphereGeom.radius + capsuleGeom.radius;

	return distancePointSegmentSquared(capsuleHalfHeightVector, -capsuleHalfHeightVector, pose0.p - pose1.p) <= r*r;	// PT: objects are defined as closed, so we return 'true' in case of equality
}

static bool GeomOverlapCallback_SphereBox(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	// PT: TODO: remove this useless conversion
	Box obb;
	buildFrom(obb, pose1.p, boxGeom.halfExtents, pose1.q);

	return intersectSphereBox(Sphere(pose0.p, sphereGeom.radius), obb);
}

static bool GeomOverlapCallback_SphereConvex(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1.f);

	const bool overlap = intersectSphereConvex(pose0, sphereGeom.radius,
		*cm,
		convexGeom.scale, pose1,
		&cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Plane-vs-shape

static bool GeomOverlapCallback_PlaneCapsule(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);
	PX_UNUSED(cache);
	PX_UNUSED(geom0);

//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	// PT: TODO: remove this useless conversion
	Capsule capsule;
	getCapsule(capsule, capsuleGeom, pose1);

	const PxPlane plane = getPlane(pose0);

	// We handle the capsule-plane collision with 2 sphere-plane collisions.
	// Seems ok so far, since plane is infinite.

	if(plane.distance(capsule.p0) <= capsule.radius)	// PT: objects are defined as closed, so we return 'true' in case of equality
		return true;

	if(plane.distance(capsule.p1) <= capsule.radius)	// PT: objects are defined as closed, so we return 'true' in case of equality
		return true;

	return false;
}

/*static bool intersectPlaneBox(const PxPlane& plane, const Box& box)
{
	PxVec3 pts[8];
	box.computeBoxPoints(pts);

	for(PxU32 i=0;i<8;i++)
	{
		if(plane.distance(pts[i]) <= 0.0f)	// PT: objects are defined as closed, so we return 'true' in case of equality
			return true;
	}
	return false;
}*/

static bool GeomOverlapCallback_PlaneBox(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);
	PX_UNUSED(cache);
	PX_UNUSED(geom0);

//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	// I currently use the same code as for contact generation but maybe we could do something faster (in theory testing
	// only 2 pts is enough).

	const Matrix34 absPose(pose1);
	const PxPlane worldPlane = getPlane(pose0);

	for(int vx=-1; vx<=1; vx+=2)
		for(int vy=-1; vy<=1; vy+=2)
			for(int vz=-1; vz<=1; vz+=2)
			{
				const PxVec3 v = absPose.transform(PxVec3(PxReal(vx),PxReal(vy),PxReal(vz)).multiply(boxGeom.halfExtents));

				if(worldPlane.distance(v) <= 0.0f)	// PT: objects are defined as closed, so we return 'true' in case of equality
					return true;
			}
	return false;
}

static bool GeomOverlapCallback_PlaneConvex(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);
	PX_UNUSED(cache);
	PX_UNUSED(geom0);

//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	
	//find plane normal in shape space of convex:
	const PxTransform plane2convex = pose1.getInverse().transform(pose0);

	const PxPlane shapeSpacePlane = getPlane(plane2convex);

	PxReal minimum, maximum;
	projectHull_(cm->getHull(), minimum, maximum, shapeSpacePlane.n, convexGeom.scale.toMat33());

	return (minimum <= -shapeSpacePlane.d);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Capsule-vs-shape

static bool GeomOverlapCallback_CapsuleCapsule(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);
	PX_UNUSED(cache);

	const PxCapsuleGeometry& capsuleGeom0 = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom1 = static_cast<const PxCapsuleGeometry&>(geom1);

	// PT: move computation to local space for improved accuracy
	const PxVec3 delta = pose1.p - pose0.p;

	// PT: TODO: remove this useless conversion
	const PxVec3 capsuleHalfHeightVector0 = getCapsuleHalfHeightVector(pose0, capsuleGeom0);
	const PxVec3 capsuleHalfHeightVector1 = getCapsuleHalfHeightVector(pose1, capsuleGeom1);

	const PxReal squareDist = distanceSegmentSegmentSquared(-capsuleHalfHeightVector0, capsuleHalfHeightVector0*2.0f,
															delta-capsuleHalfHeightVector1, capsuleHalfHeightVector1*2.0f);
	const PxReal r = capsuleGeom0.radius + capsuleGeom1.radius;
	return squareDist <= r*r;	// PT: objects are defined as closed, so we return 'true' in case of equality
}

static bool GeomOverlapCallback_CapsuleBox(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);
	PX_UNUSED(cache);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	// PT: move computation to local space for improved accuracy
	const PxVec3 delta = pose1.p - pose0.p;

	// PT: TODO: remove this useless conversion
	const PxVec3 capsuleHalfHeightVector = getCapsuleHalfHeightVector(pose0, capsuleGeom);

	// PT: TODO: remove this useless conversion
	const PxMat33 obbRot(pose1.q);

	// PT: objects are defined as closed, so we return 'true' in case of equality
	return distanceSegmentBoxSquared(capsuleHalfHeightVector, -capsuleHalfHeightVector, delta, boxGeom.halfExtents, obbRot) <= capsuleGeom.radius*capsuleGeom.radius;
}

static bool GeomOverlapCallback_CapsuleConvex(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	
	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0,0,1.0f);

	const bool overlap = intersectCapsuleConvex(capsuleGeom, pose0, *cm, convexGeom.scale, pose1, &cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Box-vs-shape

static bool GeomOverlapCallback_BoxBox(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);
	PX_UNUSED(cache);

	const PxBoxGeometry& boxGeom0 = static_cast<const PxBoxGeometry&>(geom0);
	const PxBoxGeometry& boxGeom1 = static_cast<const PxBoxGeometry&>(geom1);

	// PT: TODO: remove this useless conversion
	return intersectOBBOBB(	boxGeom0.halfExtents, pose0.p, PxMat33Padded(pose0.q),
							boxGeom1.halfExtents, pose1.p, PxMat33Padded(pose1.q), true);
}

static bool GeomOverlapCallback_BoxConvex(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	PxVec3 cachedSepAxis;
	PxVec3* tmp = getCachedAxis(cache);
	if(tmp)
		cachedSepAxis = *tmp;
	else
		cachedSepAxis = PxVec3(0.0f, 0.0f, 1.0f);

	const bool overlap = intersectBoxConvex(boxGeom, pose0, *cm, convexGeom.scale, pose1, &cachedSepAxis);

	if(cache && overlap)
		cache->dir = cachedSepAxis;

	return updateTriggerCache(overlap, cache);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Convex-vs-shape
static bool GeomOverlapCallback_ConvexConvex(GU_OVERLAP_FUNC_PARAMS)
{
	using namespace Ps::aos;
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const Vec3V zeroV = V3Zero();
	const PxConvexMeshGeometry& convexGeom0 = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom1 = static_cast<const PxConvexMeshGeometry&>(geom1);
	const ConvexMesh* cm0 = static_cast<ConvexMesh*>(convexGeom0.convexMesh);	
	const ConvexMesh* cm1 = static_cast<ConvexMesh*>(convexGeom1.convexMesh);	

	bool overlap;
	{
		const ConvexHullData* hullData0 = &cm0->getHull();
		const ConvexHullData* hullData1 = &cm1->getHull();

		const Vec3V vScale0 = V3LoadU_SafeReadW(convexGeom0.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat0 = QuatVLoadU(&convexGeom0.scale.rotation.x);
		const Vec3V vScale1 = V3LoadU_SafeReadW(convexGeom1.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat1 = QuatVLoadU(&convexGeom1.scale.rotation.x);

		const QuatV q0 = QuatVLoadU(&pose0.q.x);
		const Vec3V p0 = V3LoadU(&pose0.p.x);

		const QuatV q1 = QuatVLoadU(&pose1.q.x);
		const Vec3V p1 = V3LoadU(&pose1.p.x);

		const PsTransformV transf0(p0, q0);
		const PsTransformV transf1(p1, q1);

		const PsMatTransformV aToB(transf1.transformInv(transf0));

		ConvexHullV convexHull0(hullData0, zeroV, vScale0, vQuat0, convexGeom0.scale.isIdentity());
		ConvexHullV convexHull1(hullData1, zeroV, vScale1, vQuat1, convexGeom1.scale.isIdentity());

		Vec3V contactA, contactB, normal;
		FloatV dist;
		RelativeConvex<ConvexHullV> convexA(convexHull0, aToB);
		LocalConvex<ConvexHullV> convexB(convexHull1);
		
		GjkStatus status = gjk(convexA, convexB, aToB.p,  FZero(), contactA, contactB, normal, dist);
		overlap = (status == GJK_CONTACT);
	}

	return updateTriggerCache(overlap, cache);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool GeomOverlapCallback_NotSupported(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ALWAYS_ASSERT_MESSAGE("NOT SUPPORTED");
	PX_UNUSED(cache);
	PX_UNUSED(pose0);
	PX_UNUSED(pose1);
	PX_UNUSED(geom0);
	PX_UNUSED(geom1);
	return false;
}

static bool GeomOverlapCallback_HeightfieldUnregistered(GU_OVERLAP_FUNC_PARAMS)
{
	PX_UNUSED(cache);
	PX_UNUSED(geom0);
	PX_UNUSED(geom1);
	PX_UNUSED(pose0);
	PX_UNUSED(pose1);
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Height Field Overlap test called with height fields unregistered ");
	return false;
}

bool GeomOverlapCallback_SphereMesh			(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_CapsuleMesh		(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_BoxMesh			(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_ConvexMesh			(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_SphereHeightfield	(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_CapsuleHeightfield	(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_BoxHeightfield		(GU_OVERLAP_FUNC_PARAMS);
bool GeomOverlapCallback_ConvexHeightfield	(GU_OVERLAP_FUNC_PARAMS);

GeomOverlapTable gGeomOverlapMethodTable[] = 
{
	//PxGeometryType::eSPHERE
	{
		GeomOverlapCallback_SphereSphere,		//PxGeometryType::eSPHERE
		GeomOverlapCallback_SpherePlane,		//PxGeometryType::ePLANE
		GeomOverlapCallback_SphereCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_SphereBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_SphereConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_SphereMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_HeightfieldUnregistered,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::ePLANE
	{
		0,										//PxGeometryType::eSPHERE
		GeomOverlapCallback_NotSupported,		//PxGeometryType::ePLANE
		GeomOverlapCallback_PlaneCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_PlaneBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_PlaneConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_NotSupported,		//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		GeomOverlapCallback_CapsuleCapsule,		//PxGeometryType::eCAPSULE
		GeomOverlapCallback_CapsuleBox,			//PxGeometryType::eBOX
		GeomOverlapCallback_CapsuleConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_CapsuleMesh,		//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_HeightfieldUnregistered,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eBOX
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		GeomOverlapCallback_BoxBox,				//PxGeometryType::eBOX
		GeomOverlapCallback_BoxConvex,			//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_BoxMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_HeightfieldUnregistered,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		GeomOverlapCallback_ConvexConvex,		//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_ConvexMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		GeomOverlapCallback_HeightfieldUnregistered,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		0,										//PxGeometryType::eCONVEXMESH
		GeomOverlapCallback_NotSupported,		//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,										//PxGeometryType::eSPHERE
		0,										//PxGeometryType::ePLANE
		0,										//PxGeometryType::eCAPSULE
		0,										//PxGeometryType::eBOX
		0,										//PxGeometryType::eCONVEXMESH
		0,										//PxGeometryType::eTRIANGLEMESH
		GeomOverlapCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},
};

const GeomOverlapTable* Gu::getOverlapFuncTable()
{
	return gGeomOverlapMethodTable;
}

void registerHeightFields_Raycasts();
void registerHeightFields_Sweeps();
void Gu::registerHeightFields()
{
	registerHeightFields_Raycasts();
	registerHeightFields_Sweeps();

	gGeomOverlapMethodTable[PxGeometryType::eSPHERE][PxGeometryType::eHEIGHTFIELD] = GeomOverlapCallback_SphereHeightfield;
	gGeomOverlapMethodTable[PxGeometryType::eCAPSULE][PxGeometryType::eHEIGHTFIELD] = GeomOverlapCallback_CapsuleHeightfield;
	gGeomOverlapMethodTable[PxGeometryType::eBOX][PxGeometryType::eHEIGHTFIELD] = GeomOverlapCallback_BoxHeightfield;
	gGeomOverlapMethodTable[PxGeometryType::eCONVEXMESH][PxGeometryType::eHEIGHTFIELD] = GeomOverlapCallback_ConvexHeightfield;
}
