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

#include "GuMidphaseInterface.h"
#include "GuInternal.h"
#include "PxSphereGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "GuIntersectionRayCapsule.h"
#include "GuIntersectionRaySphere.h"
#include "GuIntersectionRayPlane.h"
#include "GuHeightFieldUtil.h"
#include "GuDistancePointSegment.h"
#include "GuConvexMesh.h"
#include "CmScaling.h"

using namespace physx;
using namespace Gu;

////////////////////////////////////////////////// raycasts //////////////////////////////////////////////////////////////////
PxU32 raycast_box(GU_RAY_FUNC_PARAMS)
{
	PX_UNUSED(maxHits);
	PX_ASSERT(geom.getType() == PxGeometryType::eBOX);
	PX_ASSERT(maxHits && hits);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);

	const PxTransform& absPose = pose;

	PxVec3 localOrigin = rayOrigin - absPose.p;
	localOrigin = absPose.q.rotateInv(localOrigin);

	const PxVec3 localDir = absPose.q.rotateInv(rayDir);

	PxVec3 localImpact;
	PxReal t;
	PxU32 rval = rayAABBIntersect2(-boxGeom.halfExtents, boxGeom.halfExtents, localOrigin, localDir, localImpact, t);
	if(!rval)
		return 0;

	if(t>maxDist)
		return 0;

	hits->distance	= t; //worldRay.orig.distance(hit.worldImpact);	//should be the same, assuming ray dir was normalized!!
	hits->faceIndex	= 0xffffffff;
	hits->u			= 0.0f;
	hits->v			= 0.0f;

	PxHitFlags outFlags = PxHitFlags(0);
	if((hitFlags & PxHitFlag::ePOSITION))
	{
		outFlags |= PxHitFlag::ePOSITION;
		if(t!=0.0f)
			hits->position = absPose.transform(localImpact);
		else
			hits->position = rayOrigin;
	}

	// Compute additional information if needed
	if(hitFlags & PxHitFlag::eNORMAL)
	{
		outFlags |= PxHitFlag::eNORMAL;

		//Because rayAABBIntersect2 set t = 0 if start point inside shape
		if(t == 0)
		{
			hits->normal = -rayDir;
		}
		else
		{
			//local space normal is:
			rval--;
			PxVec3 n(0.0f);
			n[rval] = PxReal((localImpact[rval] > 0.0f) ? 1.0f : -1.0f);
			hits->normal = absPose.q.rotate(n);
		}
	}
	else
	{
		hits->normal = PxVec3(0.0f);
	}
	hits->flags	= outFlags;
	return 1;
}

PxU32 raycast_sphere(GU_RAY_FUNC_PARAMS)
{
	PX_UNUSED(maxHits);
	PX_ASSERT(geom.getType() == PxGeometryType::eSPHERE);
	PX_ASSERT(maxHits && hits);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);

	if(!intersectRaySphere(rayOrigin, rayDir, maxDist, pose.p, sphereGeom.radius, hits->distance, &hits->position))
		return 0;

	/*	// PT: should be useless now
	hit.distance	= worldRay.orig.distance(hit.worldImpact);
	if(hit.distance>maxDist)
	return false;
	*/
	// PT: we can't avoid computing the position here since it's needed to compute the normal anyway
	hits->faceIndex	= 0xffffffff;
	hits->u			= 0.0f;
	hits->v			= 0.0f;

	// Compute additional information if needed
	PxHitFlags outFlags = PxHitFlag::ePOSITION;
	if(hitFlags & PxHitFlag::eNORMAL)
	{
		// User requested impact normal
		//Because intersectRaySphere set distance = 0 if start point inside shape
		if(hits->distance == 0.0f)
		{
			hits->normal = -rayDir;
		}
		else
		{
			hits->normal = hits->position - pose.p;
			hits->normal.normalize();
		}
		outFlags |= PxHitFlag::eNORMAL;
	}
	else
	{
		hits->normal = PxVec3(0.0f);
	}
	hits->flags = outFlags;

	return 1;
}

PxU32 raycast_capsule(GU_RAY_FUNC_PARAMS)
{
	PX_UNUSED(maxHits);
	PX_ASSERT(geom.getType() == PxGeometryType::eCAPSULE);
	PX_ASSERT(maxHits && hits);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);

	// TODO: PT: could we simplify this ?
	Capsule capsule;
	getCapsuleSegment(pose, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	PxReal t = 0.0f;
	if(!intersectRayCapsule(rayOrigin, rayDir, capsule, t))
		return 0;

	if(t<0.0f || t>maxDist)
		return 0;

	// PT: we can't avoid computing the position here since it's needed to compute the normal anyway
	hits->position	= rayOrigin + rayDir*t;	// PT: will be rayOrigin for t=0.0f (i.e. what the spec wants)
	hits->distance	= t;
	hits->faceIndex	= 0xffffffff;
	hits->u			= 0.0f;
	hits->v			= 0.0f;

	// Compute additional information if needed
	PxHitFlags outFlags = PxHitFlag::ePOSITION;
	if(hitFlags & PxHitFlag::eNORMAL)
	{
		outFlags |= PxHitFlag::eNORMAL;

		if(t==0.0f)
		{
			hits->normal = -rayDir;
		}
		else
		{
			PxReal capsuleT;
			distancePointSegmentSquared(capsule, hits->position, &capsuleT);
			capsule.computePoint(hits->normal, capsuleT);
			hits->normal = hits->position - hits->normal;	 //this should never be zero. It should have a magnitude of the capsule radius.
			hits->normal.normalize();
		}
	}
	else
	{
		hits->normal = PxVec3(0.0f);
	}
	hits->flags = outFlags;

	return 1;
}

PxU32 raycast_plane(GU_RAY_FUNC_PARAMS)
{
	PX_UNUSED(hitFlags);
	PX_UNUSED(maxHits);
	PX_ASSERT(geom.getType() == PxGeometryType::ePLANE);
	PX_ASSERT(maxHits && hits);
	PX_UNUSED(geom);
//	const PxPlaneGeometry& planeGeom = static_cast<const PxPlaneGeometry&>(geom);

	// Perform backface culling so that we can pick objects beyond planes
	const PxPlane plane = getPlane(pose);
	if(rayDir.dot(plane.n)>=0.0f)
		return false;

	PxReal distanceAlongLine;
	if(!intersectRayPlane(rayOrigin, rayDir, plane, distanceAlongLine, &hits->position))
		return 0;

	/*
	PxReal test = worldRay.orig.distance(hit.worldImpact);

	PxReal dd;
	PxVec3 pp;
	PxSegmentPlaneIntersect(worldRay.orig, worldRay.orig+worldRay.dir*1000.0f, plane, dd, pp);
	*/

	if(distanceAlongLine<0.0f)
		return 0;

	if(distanceAlongLine>maxDist)
		return 0;

	hits->distance	= distanceAlongLine;
	hits->faceIndex	= 0xffffffff;
	hits->u			= 0.0f;
	hits->v			= 0.0f;
	hits->flags		= PxHitFlag::ePOSITION|PxHitFlag::eNORMAL;
	hits->normal	= plane.n;
	return 1;
}

PxU32 raycast_convexMesh(GU_RAY_FUNC_PARAMS)
{ 
	PX_UNUSED(maxHits);
	PX_ASSERT(geom.getType() == PxGeometryType::eCONVEXMESH);
	PX_ASSERT(maxHits && hits);
	PX_ASSERT(PxAbs(rayDir.magnitudeSquared()-1)<1e-4f);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

	ConvexMesh* convexMesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	PxRaycastHit& hit = *hits;
	
	//scaling: transform the ray to vertex space
	const Cm::Matrix34 world2vertexSkew = convexGeom.scale.getInverse() * pose.getInverse();	

	//ConvexMesh* cmesh = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	const PxU32 nPolys = convexMesh->getNbPolygonsFast();
	const HullPolygonData* PX_RESTRICT polysEA = convexMesh->getPolygons();
	const HullPolygonData* polys = polysEA;

	const PxVec3 vrayOrig = world2vertexSkew.transform(rayOrigin);
	const PxVec3 vrayDir = world2vertexSkew.rotate(rayDir);

	/*
	Purely convex planes based algorithm
	Iterate all planes of convex, with following rules:
	* determine of ray origin is inside them all or not.  
	* planes parallel to ray direction are immediate early out if we're on the outside side (plane normal is sep axis)
	* else 
		- for all planes the ray direction "enters" from the front side, track the one furthest along the ray direction (A)
		- for all planes the ray direction "exits" from the back side, track the one furthest along the negative ray direction (B)
	if the ray origin is outside the convex and if along the ray, A comes before B, the directed line stabs the convex at A
	*/
	bool originInsideAllPlanes = true;
	PxReal latestEntry = -FLT_MAX;
	PxReal earliestExit = FLT_MAX;
//	PxU32 bestPolygonIndex = 0;
	hit.faceIndex	= 0xffffffff;

	for(PxU32 i=0;i<nPolys;i++)
	{
		const HullPolygonData& poly = polys[i];
		const PxPlane& vertSpacePlane = poly.mPlane;

		const PxReal distToPlane = vertSpacePlane.distance(vrayOrig);
		const PxReal dn = vertSpacePlane.n.dot(vrayDir);
		const PxReal distAlongRay = -distToPlane/dn;	// PT: TODO: potential divide by zero here!

		// PT: TODO: this is computed again in the last branch!
		if(distToPlane > 0.0f)
			originInsideAllPlanes = false;	//origin not behind plane == ray starts outside the convex.

		if(dn > 1E-7f)	//the ray direction "exits" from the back side
		{
			earliestExit = physx::intrinsics::selectMin(earliestExit, distAlongRay);
		}
		else if(dn < -1E-7f)	//the ray direction "enters" from the front side
		{
			if(distAlongRay > latestEntry)
			{
				latestEntry = distAlongRay;
				hit.faceIndex = i;
			}
		}
		else
		{
			//plane normal and ray dir are orthogonal
			if(distToPlane > 0.0f)	
				return 0;	//a plane is parallel with ray -- and we're outside the ray -- we definitely miss the entire convex!
		}
	}

	if(originInsideAllPlanes)	//ray starts inside convex
	{
		hit.distance	= 0.0f;
		hit.faceIndex	= 0xffffffff;
		hit.u			= 0.0f;
		hit.v			= 0.0f;
		hit.position	= rayOrigin;
		hit.normal		= -rayDir;
		hit.flags		= PxHitFlag::eNORMAL|PxHitFlag::ePOSITION;
		return 1;
	}

	// AP: changed to latestEntry < maxDist-1e-5f so that we have a conservatively negative result near end of ray
	if(latestEntry < earliestExit && latestEntry > 0.0f && latestEntry < maxDist-1e-5f)
	{
		PxHitFlags outFlags = PxHitFlag::eFACE_INDEX;
		if(hitFlags & PxHitFlag::ePOSITION)
		{
			outFlags |= PxHitFlag::ePOSITION;
			const PxVec3 pointOnPlane = vrayOrig + latestEntry * vrayDir;
			hit.position = pose.transform(convexGeom.scale.toMat33() * pointOnPlane);
		}
		hit.distance	= latestEntry;
		hit.u			= 0.0f;
		hit.v			= 0.0f;
		hit.normal		= PxVec3(0.0f);

		// Compute additional information if needed
		if(hitFlags & PxHitFlag::eNORMAL)
		{
			outFlags |= PxHitFlag::eNORMAL;
			//when we have nonuniform scaling we actually have to transform by the transpose of the inverse of vertex2worldSkew.M == transpose of world2vertexSkew:
			hit.normal = world2vertexSkew.rotateTranspose(polys[hit.faceIndex].mPlane.n);
			hit.normal.normalize();
		}
		hit.flags = outFlags;
		return 1;
	}
	return 0;
}

PxU32 raycast_triangleMesh(GU_RAY_FUNC_PARAMS) 
{
	PX_ASSERT(geom.getType() == PxGeometryType::eTRIANGLEMESH);
	PX_ASSERT(PxAbs(rayDir.magnitudeSquared()-1)<1e-4f);

	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	return Midphase::raycastTriangleMesh(meshData, meshGeom, pose, rayOrigin, rayDir, maxDist, hitFlags, maxHits, hits);
}

namespace
{
	struct HFTraceSegmentCallback 
	{
		PX_NOCOPY(HFTraceSegmentCallback)
	public:
		PxRaycastHit*			mHits;
		const PxU32				mMaxHits;
		PxU32					mNbHits;
		const HeightFieldUtil&	mUtil;
		const PxTransform&		mPose;
		const PxVec3&			mRayDir;
		const PxVec3&			mLocalRayDir;
		const PxVec3&			mLocalRayOrig;		
		const PxHitFlags		mHitFlags;
		const bool				mIsDoubleSided;

		HFTraceSegmentCallback(	PxRaycastHit* hits, PxU32 maxHits, const PxHitFlags hitFlags, const HeightFieldUtil& hfUtil, const PxTransform& pose,
								const PxVec3& rayDir, const PxVec3& localRayDir, const PxVec3& localRayOrig,
								bool isDoubleSided) :
			mHits			(hits),
			mMaxHits		(maxHits),
			mNbHits			(0),
			mUtil			(hfUtil),
			mPose			(pose),
			mRayDir			(rayDir),
			mLocalRayDir	(localRayDir),
			mLocalRayOrig	(localRayOrig),
			mHitFlags		(hitFlags),
			mIsDoubleSided	(isDoubleSided)
		{
			PX_ASSERT(maxHits > 0);
		}

		PX_FORCE_INLINE bool onEvent(PxU32 , PxU32*)
		{
			return true;
		}

		PX_FORCE_INLINE bool underFaceHit(const HeightFieldUtil&, const PxVec3&, const PxVec3&, PxF32, PxF32, PxF32, PxU32)
		{
			return true;	// true means continue traversal
		}

		PxAgain faceHit(const HeightFieldUtil&, const PxVec3& aHitPoint, PxU32 aTriangleIndex, PxReal u, PxReal v)
		{
			// traversal is strictly sorted so there's no need to sort hits
			if(mNbHits >= mMaxHits)
				return false; // false = stop traversal

			PxRaycastHit& hit = mHits[mNbHits++];
			hit.position	= aHitPoint;
			hit.faceIndex	= aTriangleIndex;
			hit.u			= u;
			hit.v			= v;
			hit.flags		= PxHitFlag::eUV | PxHitFlag::eFACE_INDEX; // UVs and face index are always set

			if(mHitFlags & PxHitFlag::eNORMAL)
			{
				// We need the normal for the dot product.
				PxVec3 normal = mPose.q.rotate(mUtil.getNormalAtShapePoint(hit.position.x, hit.position.z)); 
				normal.normalize();
				if(mIsDoubleSided && normal.dot(mRayDir) > 0.0f) // comply with normal spec for double sided (should always face opposite rayDir)
					hit.normal = -normal;
				else
					hit.normal = normal;
				hit.flags |= PxHitFlag::eNORMAL;
			}

			hit.distance = physx::intrinsics::selectMax(0.f, (hit.position - mLocalRayOrig).dot(mLocalRayDir));

			if(mHitFlags & PxHitFlag::ePOSITION)
			{
				hit.position = mPose.transform(hit.position);
				hit.flags |= PxHitFlag::ePOSITION;
			}
			return (mNbHits < mMaxHits); // true = continue traversal, false = stop traversal
		}
	};   
}

PxU32 raycast_heightField(GU_RAY_FUNC_PARAMS)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	PX_ASSERT(maxHits && hits);
	PX_UNUSED(maxHits);

	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const PxTransform invAbsPose = pose.getInverse();
	const PxVec3 localRayOrig = invAbsPose.transform(rayOrigin);
	const PxVec3 localRayDir = invAbsPose.rotate(rayDir);

	const bool isDoubleSided = hfGeom.heightFieldFlags.isSet(PxMeshGeometryFlag::eDOUBLE_SIDED);
	const bool bothSides = isDoubleSided || (hitFlags & PxHitFlag::eMESH_BOTH_SIDES);

	const HeightFieldTraceUtil hfUtil(hfGeom);

	PxVec3 normRayDir = localRayDir;
	normRayDir.normalizeSafe(); // nothing will happen if length is < PX_NORMALIZATION_EPSILON
	
	// pretest if we intersect HF bounds. If no early exit, if yes move the origin and shorten the maxDist
	// to deal with precision issues with large maxDist
	PxBounds3 hfLocalBounds;
	hfUtil.computeLocalBounds(hfLocalBounds);

	// PT: inflate the bounds like we do in the scene-tree (see PX-1179)
	const PxVec3 center = hfLocalBounds.getCenter();
	const PxVec3 extents = hfLocalBounds.getExtents() * 1.01f;	//SQ_PRUNER_INFLATION;
	hfLocalBounds.minimum = center - extents;
	hfLocalBounds.maximum = center + extents;

	PxVec3 localImpact;
	PxReal t;	// closest intersection, t==0 hit inside
	PxU32 rval = rayAABBIntersect2(hfLocalBounds.minimum, hfLocalBounds.maximum, localRayOrig, localRayDir, localImpact, t);
	// early exit we miss the AABB
	if (!rval)
		return 0;
	if (t > maxDist)
		return 0;

	// PT: if eMESH_ANY is used then eMESH_MULTIPLE won't be, and we'll stop the query after 1 hit is found. There is no difference
	// between 'any hit' and 'closest hit' for HFs since hits are reported in order.
	HFTraceSegmentCallback callback(hits, hitFlags.isSet(PxHitFlag::eMESH_MULTIPLE) ? maxHits : 1, hitFlags, hfUtil, pose,
									rayDir, localRayDir, localRayOrig,
									isDoubleSided); // make sure we return only 1 hit without eMESH_MULTIPLE

	PxReal offset = 0.0f;
	PxReal maxDistOffset = maxDist;
	PxVec3 localRayOrigOffset = localRayOrig;

	// if we don't start inside the AABB box, offset the start pos, because of precision issues with large maxDist
	if(t > 0.0f)
	{
		offset = t - GU_RAY_SURFACE_OFFSET;
		// move the rayOrig to offset start pos
		localRayOrigOffset = localRayOrig + normRayDir*offset;
	}

	// shorten the maxDist of the offset that was cut off and clip it
	// we pick either the original maxDist, if maxDist is huge we clip it
	maxDistOffset = PxMin(maxDist - offset, GU_RAY_SURFACE_OFFSET + 2.0f * PxMax(hfLocalBounds.maximum.x - hfLocalBounds.minimum.x, PxMax(hfLocalBounds.maximum.y - hfLocalBounds.minimum.y, hfLocalBounds.maximum.z - hfLocalBounds.minimum.z)));

	hfUtil.traceSegment<HFTraceSegmentCallback, false, false>(localRayOrigOffset, normRayDir, maxDistOffset, 
		&callback, hfLocalBounds, !bothSides);
	return callback.mNbHits;
}

static PxU32 raycast_heightField_unregistered(GU_RAY_FUNC_PARAMS)
{
	PX_UNUSED(geom);
	PX_UNUSED(pose);
	PX_UNUSED(rayOrigin);
	PX_UNUSED(rayDir);
	PX_UNUSED(maxDist);
	PX_UNUSED(hitFlags);
	PX_UNUSED(maxHits);
	PX_UNUSED(hits);
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Height Field Raycast test called with height fields unregistered ");
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PT: table is not static because it's accessed as 'extern' within Gu (bypassing the function call).
RaycastFunc gRaycastMap[PxGeometryType::eGEOMETRY_COUNT] =
{
	raycast_sphere,
	raycast_plane,
	raycast_capsule,
	raycast_box,
	raycast_convexMesh,
	raycast_triangleMesh,	
	raycast_heightField_unregistered
};

// PT: the function is used by external modules (Np, CCT, Sq)
const Gu::GeomRaycastTable& Gu::getRaycastFuncTable()
{
	return gRaycastMap;
}

void registerHeightFields_Raycasts()
{
	gRaycastMap[PxGeometryType::eHEIGHTFIELD] = raycast_heightField;
}
