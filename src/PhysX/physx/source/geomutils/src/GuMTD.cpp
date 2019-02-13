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

#include "GuMTD.h"
#include "GuSphere.h"
#include "GuCapsule.h"
#include "GuDistancePointSegment.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistanceSegmentBox.h"


#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuInternal.h"

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuBoxConversion.h"
#include "GuGeometryUnion.h"
#include "GuShapeConvex.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactGen.h"
#include "GuConvexMesh.h"
#include "GuGJK.h"

#include "PsUtilities.h"
#include "PsVecTransform.h"
#include "PsMathUtils.h"
#include "PxMeshScale.h"
#include "PxConvexMeshGeometry.h"

using namespace physx;
using namespace Gu;

static PX_FORCE_INLINE PxF32 manualNormalize(PxVec3& mtd, const PxVec3& normal, PxReal lenSq)
{
	const PxF32 len = PxSqrt(lenSq);

	// We do a *manual* normalization to check for singularity condition
	if(lenSq < 1e-6f)
		mtd = PxVec3(1.0f, 0.0f, 0.0f);			// PT: zero normal => pick up random one
	else
		mtd = normal * 1.0f / len;

	return len;
}

static PX_FORCE_INLINE float validateDepth(float depth)
{
	// PT: penetration depth must always be positive or null, but FPU accuracy being what it is, we sometimes
	// end up with very small, epsilon-sized negative depths. We clamp those to zero, since they don't indicate
	// real bugs in the MTD functions. However anything larger than epsilon is wrong, and caught with an assert.
	const float epsilon = 1.e-3f;

	//ML: because we are shrunking the shape in this moment, so the depth might be larger than eps, this condition is no longer valid
	//PX_ASSERT(depth>=-epsilon);
	PX_UNUSED(epsilon);
	return PxMax(depth, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////

// PT: the function names should follow the order in which the PxGeometryTypes are listed,
// i.e. computeMTD_Type0Type1 with Type0<=Type1. This is to guarantee that the proper results
// (following the desired convention) are returned from the PxGeometryQuery-level call.

///////////////////////////////////////////////////////////////////////////////

static bool computeMTD_SphereSphere(PxVec3& mtd, PxF32& depth, const Sphere& sphere0, const Sphere& sphere1)
{
	const PxVec3 delta = sphere0.center - sphere1.center;
	const PxReal d2 = delta.magnitudeSquared();
	const PxReal radiusSum = sphere0.radius + sphere1.radius;

	if(d2 > radiusSum*radiusSum)
		return false;

	const PxF32 d = manualNormalize(mtd, delta, d2);

	depth = validateDepth(radiusSum - d);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool computeMTD_SphereCapsule(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const Capsule& capsule)
{
	const PxReal radiusSum = sphere.radius + capsule.radius;

	PxReal u;
	const PxReal d2 = distancePointSegmentSquared(capsule, sphere.center, &u);

	if(d2 > radiusSum*radiusSum)
		return false;

	const PxVec3 normal = sphere.center - capsule.getPointAt(u);
	
	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 d = manualNormalize(mtd, normal, lenSq);

	depth = validateDepth(radiusSum - d);
	return true;
}

///////////////////////////////////////////////////////////////////////////////


//This version is ported 1:1 from novodex
static PX_FORCE_INLINE bool ContactSphereBox(const PxVec3& sphereOrigin, 
							 PxReal sphereRadius,
							 const PxVec3& boxExtents,
//							 const PxcCachedTransforms& boxCacheTransform, 
							 const PxTransform& boxTransform, 
							 PxVec3& point, 
							 PxVec3& normal, 
							 PxReal& separation, 
							 PxReal contactDistance)
{

	//returns true on contact
	const PxVec3 delta = sphereOrigin - boxTransform.p; // s1.center - s2.center;
	PxVec3 dRot = boxTransform.rotateInv(delta); //transform delta into OBB body coords.

	//check if delta is outside ABB - and clip the vector to the ABB.
	bool outside = false;

	if (dRot.x < -boxExtents.x)
	{ 
		outside = true; 
		dRot.x = -boxExtents.x;
	}
	else if (dRot.x >  boxExtents.x)
	{ 
		outside = true; 
		dRot.x = boxExtents.x;
	}

	if (dRot.y < -boxExtents.y)
	{ 
		outside = true; 
		dRot.y = -boxExtents.y;
	}
	else if (dRot.y >  boxExtents.y)
	{ 
		outside = true; 
		dRot.y = boxExtents.y;
	}

	if (dRot.z < -boxExtents.z)
	{ 
		outside = true; 
		dRot.z =-boxExtents.z;
	}
	else if (dRot.z >  boxExtents.z)
	{ 
		outside = true; 
		dRot.z = boxExtents.z;
	}

	if (outside) //if clipping was done, sphere center is outside of box.
	{
		point = boxTransform.rotate(dRot); //get clipped delta back in world coords.
		normal = delta - point; //what we clipped away.	
		const PxReal lenSquared = normal.magnitudeSquared();
		const PxReal inflatedDist = sphereRadius + contactDistance;
		if (lenSquared > inflatedDist * inflatedDist) 
			return false;	//disjoint

		//normalize to make it into the normal:
		separation = PxRecipSqrt(lenSquared);
		normal *= separation;	
		separation *= lenSquared;
		//any plane that touches the sphere is tangential, so a vector from contact point to sphere center defines normal.
		//we could also use point here, which has same direction.
		//this is either a faceFace or a vertexFace contact depending on whether the box's face or vertex collides, but we did not distinguish. 
		//We'll just use vertex face for now, this info isn't really being used anyway.
		//contact point is point on surface of cube closest to sphere center.
		point += boxTransform.p;
		separation -= sphereRadius;
		return true;
	}
	else
	{
		//center is in box, we definitely have a contact.
		PxVec3 locNorm;	//local coords contact normal

		PxVec3 absdRot;
		absdRot = PxVec3(PxAbs(dRot.x), PxAbs(dRot.y), PxAbs(dRot.z));
		PxVec3 distToSurface = boxExtents - absdRot;	//dist from embedded center to box surface along 3 dimensions.

		//find smallest element of distToSurface
		if (distToSurface.y < distToSurface.x)
		{
			if (distToSurface.y < distToSurface.z)
			{
				//y
				locNorm = PxVec3(0.0f, dRot.y > 0.0f ? 1.0f : -1.0f, 0.0f);
				separation = -distToSurface.y;
			}
			else
			{
				//z
				locNorm = PxVec3(0.0f,0.0f, dRot.z > 0.0f ? 1.0f : -1.0f);
				separation = -distToSurface.z;
			}
		}
		else
		{
			if (distToSurface.x < distToSurface.z)
			{
				//x
				locNorm = PxVec3(dRot.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
				separation = -distToSurface.x;
			}
			else
			{
				//z
				locNorm = PxVec3(0.0f,0.0f, dRot.z > 0.0f ? 1.0f : -1.0f);
				separation = -distToSurface.z;
			}
		}
		//separation so far is just the embedding of the center point; we still have to push out all of the radius.
		point = sphereOrigin;
		normal = boxTransform.rotate(locNorm);
		separation -= sphereRadius;
		return true;
	}
}

static bool computeMTD_SphereBox(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const Box& box)
{
	PxVec3 point;
	if(!ContactSphereBox(	sphere.center, sphere.radius,
							box.extents, PxTransform(box.center, PxQuat(box.rot)),
							point, mtd, depth, 0.0f))
		return false;
	depth = validateDepth(-depth);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool computeMTD_CapsuleCapsule(PxVec3& mtd, PxF32& depth, const Capsule& capsule0, const Capsule& capsule1)
{
	PxReal s,t;
	const PxReal d2 = distanceSegmentSegmentSquared(capsule0, capsule1, &s, &t);

	const PxReal radiusSum = capsule0.radius + capsule1.radius;

	if(d2 > radiusSum*radiusSum)
		return false;

	const PxVec3 normal = capsule0.getPointAt(s) - capsule1.getPointAt(t);

	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 d = manualNormalize(mtd, normal, lenSq);

	depth = validateDepth(radiusSum - d);
	return true;
}

///////////////////////////////////////////////////////////////////////////////


static PX_FORCE_INLINE void reorderMTD(PxVec3& mtd, const PxVec3& center0, const PxVec3& center1)
{
	const PxVec3 witness = center0 - center1;
	if(mtd.dot(witness) < 0.0f)
		mtd = -mtd;
}

static PX_FORCE_INLINE void projectBox(PxReal& min, PxReal& max, const PxVec3& axis, const Box& box)
{
	const PxReal boxCen = box.center.dot(axis);
	const PxReal boxExt = 
			PxAbs(box.rot.column0.dot(axis)) * box.extents.x
		+	PxAbs(box.rot.column1.dot(axis)) * box.extents.y
		+	PxAbs(box.rot.column2.dot(axis)) * box.extents.z;

	min = boxCen - boxExt; 
	max = boxCen + boxExt; 
}

static bool PxcTestAxis(const PxVec3& axis, const Segment& segment, PxReal radius, const Box& box, PxReal& depth)
{
	// Project capsule
	PxReal min0 = segment.p0.dot(axis);
	PxReal max0 = segment.p1.dot(axis);
	if(min0>max0) Ps::swap(min0, max0);
	min0 -= radius;
	max0 += radius;

	// Project box
	PxReal Min1, Max1;
	projectBox(Min1, Max1, axis, box);

	// Test projections
	if(max0<Min1 || Max1<min0)
		return false;

	const PxReal d0 = max0 - Min1;
	PX_ASSERT(d0>=0.0f);
	const PxReal d1 = Max1 - min0;
	PX_ASSERT(d1>=0.0f);
	depth = physx::intrinsics::selectMin(d0, d1);
	return true;
}

static bool PxcCapsuleOBBOverlap3(const Segment& segment, PxReal radius, const Box& box, PxReal* t=NULL, PxVec3* pp=NULL)
{
	PxVec3 Sep(0.0f);
	PxReal PenDepth = PX_MAX_REAL;

	// Test normals
	for(PxU32 i=0;i<3;i++)
	{
		PxReal d;
		if(!PxcTestAxis(box.rot[i], segment, radius, box, d))
			return false;

		if(d<PenDepth)
		{
			PenDepth = d;
			Sep = box.rot[i];
		}
	}

	// Test edges
	PxVec3 CapsuleAxis(segment.p1 - segment.p0);
	CapsuleAxis = CapsuleAxis.getNormalized();
	for(PxU32 i=0;i<3;i++)
	{
		PxVec3 Cross = CapsuleAxis.cross(box.rot[i]);
		if(!Ps::isAlmostZero(Cross))
		{
			Cross = Cross.getNormalized();
			PxReal d;
			if(!PxcTestAxis(Cross, segment, radius, box, d))
				return false;

			if(d<PenDepth)
			{
				PenDepth = d;
				Sep = Cross;
			}
		}
	}

	reorderMTD(Sep, segment.computeCenter(), box.center);

	if(t)
		*t = validateDepth(PenDepth);
	if(pp)
		*pp = Sep;

	return true;
}

static bool computeMTD_CapsuleBox(PxVec3& mtd, PxF32& depth, const Capsule& capsule, const Box& box)
{
	PxReal t;
	PxVec3 onBox;

	const PxReal d2 = distanceSegmentBoxSquared(capsule.p0, capsule.p1, box.center, box.extents, box.rot, &t, &onBox);	

	if(d2 > capsule.radius*capsule.radius)
		return false;

	if(d2 != 0.0f)
	{
		// PT: the capsule segment doesn't intersect the box => distance-based version
		const PxVec3 onSegment = capsule.getPointAt(t);
		onBox = box.center + box.rot.transform(onBox);

		PxVec3 normal = onSegment - onBox;
		PxReal normalLen = normal.magnitude();

		if(normalLen != 0.0f)
		{
			normal *= 1.0f/normalLen;

			mtd = normal;
			depth = validateDepth(capsule.radius - PxSqrt(d2));
			return true;
		}
	}

	// PT: the capsule segment intersects the box => penetration-based version
	return PxcCapsuleOBBOverlap3(capsule, capsule.radius, box, &depth, &mtd);
}

///////////////////////////////////////////////////////////////////////////////

static bool PxcTestAxis(const PxVec3& axis, const Box& box0, const Box& box1, PxReal& depth)
{
	// Project box
	PxReal min0, max0;
	projectBox(min0, max0, axis, box0);

	// Project box
	PxReal Min1, Max1;
	projectBox(Min1, Max1, axis, box1);

	// Test projections
	if(max0<Min1 || Max1<min0)
		return false;

	const PxReal d0 = max0 - Min1;
	PX_ASSERT(d0>=0.0f);
	const PxReal d1 = Max1 - min0;
	PX_ASSERT(d1>=0.0f);
	depth = physx::intrinsics::selectMin(d0, d1);
	return true;
}

static PX_FORCE_INLINE bool testBoxBoxAxis(PxVec3& mtd, PxF32& depth, const PxVec3& axis, const Box& box0, const Box& box1)
{
	PxF32 d;
	if(!PxcTestAxis(axis, box0, box1, d))
		return false;
	if(d<depth)
	{
		depth = d;
		mtd = axis;
	}
	return true;
}

static bool computeMTD_BoxBox(PxVec3& _mtd, PxF32& _depth, const Box& box0, const Box& box1)
{
	PxVec3 mtd;
	PxF32 depth = PX_MAX_F32;

	if(!testBoxBoxAxis(mtd, depth, box0.rot.column0, box0, box1))
		return false;
	if(!testBoxBoxAxis(mtd, depth, box0.rot.column1, box0, box1))
		return false;
	if(!testBoxBoxAxis(mtd, depth, box0.rot.column2, box0, box1))
		return false;

	if(!testBoxBoxAxis(mtd, depth, box1.rot.column0, box0, box1))
		return false;
	if(!testBoxBoxAxis(mtd, depth, box1.rot.column1, box0, box1))
		return false;
	if(!testBoxBoxAxis(mtd, depth, box1.rot.column2, box0, box1))
		return false;

	for(PxU32 j=0;j<3;j++)
	{
		for(PxU32 i=0;i<3;i++)
		{
			PxVec3 cross = box0.rot[i].cross(box1.rot[j]);
			if(!Ps::isAlmostZero(cross))
			{
				cross = cross.getNormalized();

				if(!testBoxBoxAxis(mtd, depth, cross, box0, box1))
					return false;
			}
		}
	}


	reorderMTD(mtd, box1.center, box0.center);

	_mtd	= -mtd;
	_depth	= validateDepth(depth);

	return true;
}

///////////////////////////////////////////////////////////////////////////////

using namespace physx::shdfnd::aos;

bool pointConvexDistance(PxVec3& normal_, PxVec3& closestPoint_, PxReal& sqDistance, const PxVec3& pt, const ConvexMesh* convexMesh, const PxMeshScale& meshScale, const PxTransform& convexPose)
{
	const PxTransform transform0(pt);
  
	PxVec3 onSegment, onConvex;

	using namespace Ps::aos;
	const Vec3V zeroV = V3Zero();
	Vec3V closA, closB, normalV;
	GjkStatus status;
	FloatV dist;
	{
		const ConvexHullData* hullData = &convexMesh->getHull();
		const Vec3V vScale	= V3LoadU_SafeReadW(meshScale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat = QuatVLoadU(&meshScale.rotation.x);
		const ConvexHullV convexHull_(hullData, zeroV, vScale, vQuat, meshScale.isIdentity());

		const PsMatTransformV aToB(convexPose.transformInv(transform0)); 

		//const CapsuleV capsule(zeroV, zeroV, FZero());//this is a point
		const CapsuleV capsule_(aToB.p, FZero());//this is a point
		LocalConvex<CapsuleV> capsule(capsule_);
		LocalConvex<ConvexHullV> convexHull(convexHull_);
		
		status = gjk<LocalConvex<CapsuleV>, LocalConvex<ConvexHullV> >(capsule, convexHull, aToB.p, FMax(), closA, closB, normalV, dist);
	}

	bool intersect = status == GJK_CONTACT;
	if(intersect)
	{
		sqDistance = 0.0f;
	}
	else
	{
		const FloatV sqDist = FMul(dist, dist);
		FStore(sqDist, &sqDistance);
		V3StoreU(normalV, normal_);
		V3StoreU(closB, closestPoint_);

		normal_ = convexPose.rotate(normal_);
		closestPoint_ = convexPose.transform(closestPoint_);
	}

	return intersect;
}

static bool computeMTD_SphereConvex(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose)
{
	PxReal d2;
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
	PxVec3 dummy;
	if(!pointConvexDistance(mtd, dummy, d2, sphere.center, convexMesh, convexGeom.scale, convexPose))
	{
		if(d2 > sphere.radius*sphere.radius)
			return false;

		depth = validateDepth(sphere.radius - PxSqrt(d2));
		mtd = -mtd;
		return true;
	}

	// PT: if we reach this place, the sphere center touched the convex => switch to penetration-based code
	PxU32 nbPolygons = convexMesh->getNbPolygonsFast();
	const HullPolygonData* polygons = convexMesh->getPolygons();
	const PxVec3 localSphereCenter = convexPose.transformInv(sphere.center);
	PxReal dmax = -PX_MAX_F32;
	while(nbPolygons--)
	{
		const HullPolygonData& polygon = *polygons++;
		const PxF32 d = polygon.mPlane.distance(localSphereCenter);
		if(d>dmax)		
		{
			dmax = d;
			mtd = convexPose.rotate(polygon.mPlane.n);
		}
	}
	depth = validateDepth(sphere.radius - dmax);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

//ML : capsule will be in the local space of convexHullV
static bool internalComputeMTD_CapsuleConvex(const CapsuleV& capsule, const bool idtScale,  ConvexHullV& convexHullV, const Ps::aos::PsTransformV& transf1,
									   Ps::aos::FloatV& penetrationDepth, Ps::aos::Vec3V& normal)
{
	PolygonalData polyData;
	getPCMConvexData(convexHullV, idtScale, polyData);

	PxU8 buff[sizeof(SupportLocalImpl<ConvexHullV>)];

	SupportLocal* map = (idtScale ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHullV), transf1, convexHullV.vertex2Shape, convexHullV.shape2Vertex, idtScale)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullV>)(convexHullV, transf1, convexHullV.vertex2Shape, convexHullV.shape2Vertex, idtScale)));

	return computeMTD(capsule, polyData, map, penetrationDepth, normal); 
}

static bool computeMTD_CapsuleConvex(PxVec3& mtd, PxF32& depth, const Capsule& capsule, const PxTransform& capsulePose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose)
{
	const FloatV capsuleHalfHeight = FLoad(capsule.length()*0.5f);
	const FloatV capsuleRadius = FLoad(capsule.radius);

	const Vec3V zeroV = V3Zero();
	// Convex mesh
		const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
		const ConvexHullData* hull = &convexMesh->getHull();
		const Vec3V vScale	= V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat	= QuatVLoadU(&convexGeom.scale.rotation.x);
		ConvexHullV convexHullV(hull, zeroV, vScale, vQuat, convexGeom.scale.isIdentity());
	//~Convex mesh


	const QuatV q0 = QuatVLoadU(&capsulePose.q.x);
	const Vec3V p0 = V3LoadU(&capsulePose.p.x);

	const QuatV q1 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p1 = V3LoadU(&convexPose.p.x);

	const PsTransformV transf0(p0, q0);
	const PsTransformV transf1(p1, q1);
	const PsTransformV curRTrans(transf1.transformInv(transf0));
	const PsMatTransformV aToB(curRTrans);

	Vec3V normal = zeroV;
	FloatV penetrationDepth = FZero();

	CapsuleV capsuleV(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);

	const bool idtScale = convexGeom.scale.isIdentity();
	bool hasContacts = internalComputeMTD_CapsuleConvex(capsuleV, idtScale, convexHullV, transf1, penetrationDepth, normal);
	if(hasContacts)
	{
		FStore(penetrationDepth, &depth);
		depth = validateDepth(depth);
		V3StoreU(normal, mtd);
	}

	return hasContacts;
}

///////////////////////////////////////////////////////////////////////////////
static bool internalComputeMTD_BoxConvex(const PxVec3 halfExtents, const BoxV& box, const bool idtScale,  ConvexHullV& convexHullV, const Ps::aos::PsTransformV& transf0, const Ps::aos::PsTransformV& transf1,
									   Ps::aos::FloatV& penetrationDepth, Ps::aos::Vec3V& normal)
{
	PolygonalData polyData0;
	PCMPolygonalBox polyBox0(halfExtents);
	polyBox0.getPolygonalData(&polyData0);
	polyData0.mPolygonVertexRefs = gPCMBoxPolygonData;

	PolygonalData polyData1;
	getPCMConvexData(convexHullV, idtScale, polyData1);

	Mat33V identity =  M33Identity();
	SupportLocalImpl<BoxV> map0(box, transf0, identity, identity, true);

	PxU8 buff[sizeof(SupportLocalImpl<ConvexHullV>)];

	SupportLocal* map1 = (idtScale ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHullV), transf1, convexHullV.vertex2Shape, convexHullV.shape2Vertex, idtScale)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullV>)(convexHullV, transf1, convexHullV.vertex2Shape, convexHullV.shape2Vertex, idtScale)));

	return computeMTD(polyData0, polyData1, &map0, map1, penetrationDepth, normal); 

}

static bool computeMTD_BoxConvex(PxVec3& mtd, PxF32& depth, const Box& box, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose)
{
	const Vec3V zeroV = V3Zero();
	const PxTransform boxPose = box.getTransform();
	const Vec3V boxExtents = V3LoadU(box.extents);
	BoxV boxV(zeroV, boxExtents);

	// Convex mesh
		const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
		const ConvexHullData* hull = &convexMesh->getHull();
		const Vec3V vScale	= V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat	= QuatVLoadU(&convexGeom.scale.rotation.x);
		ConvexHullV convexHullV(hull, zeroV, vScale, vQuat, convexGeom.scale.isIdentity()); 
	//~Convex mesh


	const QuatV q0 = QuatVLoadU(&boxPose.q.x);
	const Vec3V p0 = V3LoadU(&boxPose.p.x);

	const QuatV q1 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p1 = V3LoadU(&convexPose.p.x);

	const PsTransformV transf0(p0, q0);
	const PsTransformV transf1(p1, q1);

	Vec3V normal=zeroV;
	FloatV penetrationDepth=FZero();

	const bool idtScale = convexGeom.scale.isIdentity();
	bool hasContacts = internalComputeMTD_BoxConvex(box.extents, boxV, idtScale, convexHullV, transf0, transf1, penetrationDepth, normal);
	if(hasContacts)
	{
		FStore(penetrationDepth, &depth);
		depth = validateDepth(depth);
		V3StoreU(normal, mtd);
	}

	return hasContacts;

}


static bool internalComputeMTD_ConvexConvex(const bool idtScale0, const bool idtScale1, ConvexHullV& convexHullV0, ConvexHullV& convexHullV1, const Ps::aos::PsTransformV& transf0, const Ps::aos::PsTransformV& transf1,
									   Ps::aos::FloatV& penetrationDepth, Ps::aos::Vec3V& normal)
{
	PolygonalData polyData0, polyData1;
	getPCMConvexData(convexHullV0, idtScale0, polyData0);
	getPCMConvexData(convexHullV1, idtScale1, polyData1);

	PxU8 buff0[sizeof(SupportLocalImpl<ConvexHullV>)];
	PxU8 buff1[sizeof(SupportLocalImpl<ConvexHullV>)];

	SupportLocal* map0 = (idtScale0 ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff0, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHullV0), transf0, convexHullV0.vertex2Shape, convexHullV0.shape2Vertex, idtScale0)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff0, SupportLocalImpl<ConvexHullV>)(convexHullV0, transf0, convexHullV0.vertex2Shape, convexHullV0.shape2Vertex, idtScale0)));

	SupportLocal* map1 = (idtScale1 ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHullV1), transf1, convexHullV1.vertex2Shape, convexHullV1.shape2Vertex, idtScale1)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullV>)(convexHullV1, transf1, convexHullV1.vertex2Shape, convexHullV1.shape2Vertex, idtScale1)));

	return computeMTD(polyData0, polyData1, map0, map1, penetrationDepth, normal); 
}

///////////////////////////////////////////////////////////////////////////////
static bool computeMTD_ConvexConvex(PxVec3& mtd, PxF32& depth, const PxConvexMeshGeometry& convexGeom0, const PxTransform& convexPose0, const PxConvexMeshGeometry& convexGeom1, const PxTransform& convexPose1)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();
	// Convex mesh
		const ConvexMesh* convexMesh0 = static_cast<const ConvexMesh*>(convexGeom0.convexMesh);
		const ConvexHullData* hull0 = &convexMesh0->getHull();
		const Vec3V vScale0	= V3LoadU_SafeReadW(convexGeom0.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat0	= QuatVLoadU(&convexGeom0.scale.rotation.x);
		ConvexHullV convexHullV0(hull0, zeroV, vScale0, vQuat0, convexGeom0.scale.isIdentity());
	//~Convex mesh

	// Convex mesh
		const ConvexMesh* convexMesh1 = static_cast<const ConvexMesh*>(convexGeom1.convexMesh);
		const ConvexHullData* hull1 = &convexMesh1->getHull();
		const Vec3V vScale1	= V3LoadU_SafeReadW(convexGeom1.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
		const QuatV vQuat1	= QuatVLoadU(&convexGeom1.scale.rotation.x);
		ConvexHullV convexHullV1(hull1, zeroV, vScale1, vQuat1, convexGeom1.scale.isIdentity());
	//~Convex mesh

	const QuatV q0 = QuatVLoadU(&convexPose0.q.x);
	const Vec3V p0 = V3LoadU(&convexPose0.p.x);

	const QuatV q1 = QuatVLoadU(&convexPose1.q.x);
	const Vec3V p1 = V3LoadU(&convexPose1.p.x);

	const PsTransformV transf0(p0, q0);
	const PsTransformV transf1(p1, q1);

	Vec3V normal = zeroV;
	FloatV penetrationDepth = FZero();


	const bool idtScale0 = convexGeom0.scale.isIdentity();
	const bool idtScale1 = convexGeom1.scale.isIdentity();

	bool hasContacts = internalComputeMTD_ConvexConvex(idtScale0, idtScale1, convexHullV0, convexHullV1, transf0, transf1, penetrationDepth, normal);

	if(hasContacts)
	{
		FStore(penetrationDepth, &depth);
		depth = validateDepth(depth);
		V3StoreU(normal, mtd);
	}
	return hasContacts;
}

///////////////////////////////////////////////////////////////////////////////

static bool computeMTD_SpherePlane(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const PxPlane& plane)
{
	const PxReal d = plane.distance(sphere.center);
	if(d>sphere.radius)
		return false;

	mtd		= plane.n;
	depth	= validateDepth(sphere.radius - d);
	return true;
}

static bool computeMTD_PlaneBox(PxVec3& mtd, PxF32& depth, const PxPlane& plane, const Box& box)
{
	PxVec3 pts[8];
	box.computeBoxPoints(pts);

	PxReal dmin = plane.distance(pts[0]);
	for(PxU32 i=1;i<8;i++)
	{
		const PxReal d = plane.distance(pts[i]);
		dmin = physx::intrinsics::selectMin(dmin, d);
	}
	if(dmin>0.0f)
		return false;

	mtd		= -plane.n;
	depth	= validateDepth(-dmin);
	return true;
}

static bool computeMTD_PlaneCapsule(PxVec3& mtd, PxF32& depth, const PxPlane& plane, const Capsule& capsule)
{
	const PxReal d0 = plane.distance(capsule.p0);
	const PxReal d1 = plane.distance(capsule.p1);
	const PxReal dmin = physx::intrinsics::selectMin(d0, d1) - capsule.radius;
	if(dmin>0.0f)
		return false;

	mtd		= -plane.n;
	depth	= validateDepth(-dmin);
	return true;
}

static bool computeMTD_PlaneConvex(PxVec3& mtd, PxF32& depth, const PxPlane& plane, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose)
{
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
	PxU32 nbVerts = convexMesh->getNbVerts();
	const PxVec3* PX_RESTRICT verts = convexMesh->getVerts();

	PxReal dmin = plane.distance(convexPose.transform(verts[0]));
	for(PxU32 i=1;i<nbVerts;i++)
	{
		const PxReal d = plane.distance(convexPose.transform(verts[i]));
		dmin = physx::intrinsics::selectMin(dmin, d);
	}
	if(dmin>0.0f)
		return false;

	mtd		= -plane.n;
	depth	= validateDepth(-dmin);
	return true;
}

///////////////////////////////////////////////////////////////////////////////

static bool processContacts(PxVec3& mtd, PxF32& depth, PxU32 nbContacts, const ContactPoint* contacts)
{
	if(nbContacts)
	{
		PxVec3 mn(0.0f), mx(0.0f);
		for(PxU32 i=0; i<nbContacts; i++)
		{
			const ContactPoint& ct = contacts[i];
			PxVec3 depenetration = ct.separation * ct.normal;
			
			mn = mn.minimum(depenetration);
			mx = mx.maximum(depenetration);
		}

		// even if we are already moving in separation direction, we should still depenetrate
		// so no dot velocity test
		// here we attempt to equalize the separations pushing in opposing directions along each axis
		PxVec3 mn1, mx1;
		mn1.x = (mn.x == 0.0f) ? mx.x : mn.x;
		mn1.y = (mn.y == 0.0f) ? mx.y : mn.y;
		mn1.z = (mn.z == 0.0f) ? mx.z : mn.z;
		mx1.x = (mx.x == 0.0f) ? mn.x : mx.x;
		mx1.y = (mx.y == 0.0f) ? mn.y : mx.y;
		mx1.z = (mx.z == 0.0f) ? mn.z : mx.z;
		PxVec3 sepDir((mn1 + mx1)*0.5f);

		if(sepDir.magnitudeSquared() < 1e-10f)
		{

			return false;

		}
		mtd = -sepDir.getNormalized();
		depth = sepDir.magnitude();
	}
	return true;
}

static bool computeMTD_SphereMesh(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose)
{
	GeometryUnion shape0;
	shape0.set(PxSphereGeometry(sphere.radius));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactSphereMesh(shape0, shape1, PxTransform(sphere.center), meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_CapsuleMesh(PxVec3& mtd, PxF32& depth, const Capsule& capsule, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose)
{
	PxReal halfHeight;
	const PxTransform capsuleTransform = PxTransformFromSegment(capsule.p0, capsule.p1, &halfHeight);

	GeometryUnion shape0;
	shape0.set(PxCapsuleGeometry(capsule.radius, halfHeight));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactCapsuleMesh(shape0, shape1, capsuleTransform, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_BoxMesh(PxVec3& mtd, PxF32& depth, const Box& box, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose)
{
	const PxTransform boxPose(box.center, PxQuat(box.rot));

	GeometryUnion shape0;
	shape0.set(PxBoxGeometry(box.extents));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactBoxMesh(shape0, shape1, boxPose, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_ConvexMesh(PxVec3& mtd, PxF32& depth, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose)
{
	GeometryUnion shape0;
	shape0.set(convexGeom);

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactConvexMesh(shape0, shape1, convexPose, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_SphereHeightField(PxVec3& mtd, PxF32& depth, const Sphere& sphere, const PxHeightFieldGeometry& meshGeom, const PxTransform& meshPose)
{
	GeometryUnion shape0;
	shape0.set(PxSphereGeometry(sphere.radius));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	const PxTransform spherePose(sphere.center);

	if(!contactSphereHeightfield(shape0, shape1, spherePose, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_CapsuleHeightField(PxVec3& mtd, PxF32& depth, const Capsule& capsule, const PxHeightFieldGeometry& meshGeom, const PxTransform& meshPose)
{
	PxReal halfHeight;
	const PxTransform capsuleTransform = PxTransformFromSegment(capsule.p0, capsule.p1, &halfHeight);

	GeometryUnion shape0;
	shape0.set(PxCapsuleGeometry(capsule.radius, halfHeight));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactCapsuleHeightfield(shape0, shape1, capsuleTransform, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_BoxHeightField(PxVec3& mtd, PxF32& depth, const Box& box, const PxHeightFieldGeometry& meshGeom, const PxTransform& meshPose)
{
	const PxTransform boxPose(box.center, PxQuat(box.rot));

	GeometryUnion shape0;
	shape0.set(PxBoxGeometry(box.extents));

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactBoxHeightfield(shape0, shape1, boxPose, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}

static bool computeMTD_ConvexHeightField(PxVec3& mtd, PxF32& depth, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, const PxHeightFieldGeometry& meshGeom, const PxTransform& meshPose)
{
	GeometryUnion shape0;
	shape0.set(convexGeom);

	GeometryUnion shape1;
	shape1.set(meshGeom);

	Cache cache;

	ContactBuffer contactBuffer;
	contactBuffer.reset();

	if(!contactConvexHeightfield(shape0, shape1, convexPose, meshPose, NarrowPhaseParams(0.0f, 0.0f, 1.0f), cache, contactBuffer, NULL))
		return false;

	if(!processContacts(mtd, depth, contactBuffer.count, contactBuffer.contacts))
		return false;

	return contactBuffer.count!=0;
}


static bool GeomMTDCallback_NotSupported(GU_MTD_FUNC_PARAMS)
{
	PX_ALWAYS_ASSERT_MESSAGE("NOT SUPPORTED");
	PX_UNUSED(mtd); PX_UNUSED(depth); PX_UNUSED(geom0); PX_UNUSED(geom1); PX_UNUSED(pose0); PX_UNUSED(pose1);

	return false;
}

static bool GeomMTDCallback_SphereSphere(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eSPHERE);

	const PxSphereGeometry& sphereGeom0 = static_cast<const PxSphereGeometry&>(geom0);
	const PxSphereGeometry& sphereGeom1 = static_cast<const PxSphereGeometry&>(geom1);

	return computeMTD_SphereSphere(mtd, depth, Sphere(pose0.p, sphereGeom0.radius), Sphere(pose1.p, sphereGeom1.radius));
}

static bool GeomMTDCallback_SpherePlane(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::ePLANE);
	PX_UNUSED(geom1);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	return computeMTD_SpherePlane(mtd, depth, Sphere(pose0.p, sphereGeom.radius), getPlane(pose1));
}

static bool GeomMTDCallback_SphereCapsule(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	Capsule capsule;
	getCapsuleSegment(pose1, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	return computeMTD_SphereCapsule(mtd, depth, Sphere(pose0.p, sphereGeom.radius), capsule);
}

static bool GeomMTDCallback_SphereBox(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	Box obb;
	buildFrom(obb, pose1.p, boxGeom.halfExtents, pose1.q);

	return computeMTD_SphereBox(mtd, depth, Sphere(pose0.p, sphereGeom.radius), obb);
}

static bool GeomMTDCallback_SphereConvex(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	return computeMTD_SphereConvex(mtd, depth, Sphere(pose0.p, sphereGeom.radius), convexGeom, pose1);
}

static bool GeomMTDCallback_SphereMesh(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);	

	return computeMTD_SphereMesh(mtd, depth, Sphere(pose0.p, sphereGeom.radius), meshGeom, pose1);
}

static bool GeomMTDCallback_PlaneCapsule(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);
	PX_UNUSED(geom0);
	
	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom1);

	Capsule capsule;
	getCapsuleSegment(pose1, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	return computeMTD_PlaneCapsule(mtd, depth, getPlane(pose0), capsule);
}

static bool GeomMTDCallback_PlaneBox(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);
	PX_UNUSED(geom0);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	Box obb;
	buildFrom(obb, pose1.p, boxGeom.halfExtents, pose1.q);

	return computeMTD_PlaneBox(mtd, depth, getPlane(pose0), obb);
}

static bool GeomMTDCallback_PlaneConvex(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::ePLANE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);
	PX_UNUSED(geom0);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	return computeMTD_PlaneConvex(mtd, depth, getPlane(pose0), convexGeom, pose1);
}

static bool GeomMTDCallback_CapsuleCapsule(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCAPSULE);

	const PxCapsuleGeometry& capsuleGeom0 = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxCapsuleGeometry& capsuleGeom1 = static_cast<const PxCapsuleGeometry&>(geom1);

	Capsule capsule0;
	getCapsuleSegment(pose0, capsuleGeom0, capsule0);
	capsule0.radius = capsuleGeom0.radius;

	Capsule capsule1;
	getCapsuleSegment(pose1, capsuleGeom1, capsule1);
	capsule1.radius = capsuleGeom1.radius;

	return computeMTD_CapsuleCapsule(mtd, depth, capsule0, capsule1);
}

static bool GeomMTDCallback_CapsuleBox(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom1);

	Capsule capsule;
	getCapsuleSegment(pose0, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	Box obb;
	buildFrom(obb, pose1.p, boxGeom.halfExtents, pose1.q);

	return computeMTD_CapsuleBox(mtd, depth, capsule, obb);
}

static bool GeomMTDCallback_CapsuleConvex(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	Capsule capsule;
	getCapsuleSegment(pose0, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	return computeMTD_CapsuleConvex(mtd, depth, capsule, pose0, convexGeom, pose1);
}

static bool GeomMTDCallback_CapsuleMesh(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);	

	Capsule capsule;
	getCapsuleSegment(pose0, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	return computeMTD_CapsuleMesh(mtd, depth, capsule, meshGeom, pose1);
}

static bool GeomMTDCallback_BoxBox(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eBOX);

	const PxBoxGeometry& boxGeom0 = static_cast<const PxBoxGeometry&>(geom0);
	const PxBoxGeometry& boxGeom1 = static_cast<const PxBoxGeometry&>(geom1);

	Box obb0;
	buildFrom(obb0, pose0.p, boxGeom0.halfExtents, pose0.q);

	Box obb1;
	buildFrom(obb1, pose1.p, boxGeom1.halfExtents, pose1.q);

	return computeMTD_BoxBox(mtd, depth, obb0, obb1);
}

static bool GeomMTDCallback_BoxConvex(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom1);

	Box obb;
	buildFrom(obb, pose0.p, boxGeom.halfExtents, pose0.q);

	return computeMTD_BoxConvex(mtd, depth, obb, convexGeom, pose1);
}

static bool GeomMTDCallback_BoxMesh(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);	

	Box obb;
	buildFrom(obb, pose0.p, boxGeom.halfExtents, pose0.q);

	return computeMTD_BoxMesh(mtd, depth, obb, meshGeom, pose1);
}

static bool GeomMTDCallback_ConvexConvex(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eCONVEXMESH);

	const PxConvexMeshGeometry& convexGeom0 = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxConvexMeshGeometry& convexGeom1 = static_cast<const PxConvexMeshGeometry&>(geom1);

	return computeMTD_ConvexConvex(mtd, depth, convexGeom0, pose0, convexGeom1, pose1);
}

static bool GeomMTDCallback_ConvexMesh(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eTRIANGLEMESH);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom1);	

	return computeMTD_ConvexMesh(mtd, depth, convexGeom, pose0, meshGeom, pose1);
}

static bool GeomMTDCallback_SphereHeightField(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom1);	

	const Sphere sphere(pose0.p, sphereGeom.radius);

	return computeMTD_SphereHeightField(mtd, depth, sphere, meshGeom, pose1);
}

static bool GeomMTDCallback_CapsuleHeightField(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom1);	

	Capsule capsule;
	getCapsuleSegment(pose0, capsuleGeom, capsule);
	capsule.radius = capsuleGeom.radius;

	return computeMTD_CapsuleHeightField(mtd, depth, capsule, meshGeom, pose1);
}

static bool GeomMTDCallback_BoxHeightField(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom1);	

	Box obb;
	buildFrom(obb, pose0.p, boxGeom.halfExtents, pose0.q);

	return computeMTD_BoxHeightField(mtd, depth, obb, meshGeom, pose1);
}

static bool GeomMTDCallback_ConvexHeightField(GU_MTD_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom1);	

	return computeMTD_ConvexHeightField(mtd, depth, convexGeom, pose0, meshGeom, pose1);
}

Gu::GeomMTDFunc gGeomMTDMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		GeomMTDCallback_SphereSphere,		//PxGeometryType::eSPHERE
		GeomMTDCallback_SpherePlane,		//PxGeometryType::ePLANE
		GeomMTDCallback_SphereCapsule,		//PxGeometryType::eCAPSULE
		GeomMTDCallback_SphereBox,			//PxGeometryType::eBOX
		GeomMTDCallback_SphereConvex,		//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_SphereMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_SphereHeightField,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::ePLANE
	{
		0,									//PxGeometryType::eSPHERE
		GeomMTDCallback_NotSupported,		//PxGeometryType::ePLANE
		GeomMTDCallback_PlaneCapsule,		//PxGeometryType::eCAPSULE
		GeomMTDCallback_PlaneBox,			//PxGeometryType::eBOX
		GeomMTDCallback_PlaneConvex,		//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_NotSupported,		//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		GeomMTDCallback_CapsuleCapsule,		//PxGeometryType::eCAPSULE
		GeomMTDCallback_CapsuleBox,			//PxGeometryType::eBOX
		GeomMTDCallback_CapsuleConvex,		//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_CapsuleMesh,		//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_CapsuleHeightField,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eBOX
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		GeomMTDCallback_BoxBox,				//PxGeometryType::eBOX
		GeomMTDCallback_BoxConvex,			//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_BoxMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_BoxHeightField,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		GeomMTDCallback_ConvexConvex,		//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_ConvexMesh,			//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_ConvexHeightField,	//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		GeomMTDCallback_NotSupported,		//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		GeomMTDCallback_NotSupported,		//PxGeometryType::eHEIGHTFIELD
	},
};
