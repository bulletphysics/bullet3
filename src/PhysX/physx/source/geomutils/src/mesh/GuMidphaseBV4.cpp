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

#include "GuBV4.h"
using namespace physx;
using namespace Gu;

#include "PsVecMath.h"
using namespace physx::shdfnd::aos;

#include "GuSweepMesh.h"
#include "GuBV4Build.h"
#include "GuBV4_Common.h"
#include "GuSphere.h"
#include "GuCapsule.h"
#include "GuBoxConversion.h"
#include "GuConvexUtilsInternal.h"
#include "GuVecTriangle.h"
#include "GuIntersectionTriangleBox.h"
#include "GuIntersectionCapsuleTriangle.h"
#include "GuIntersectionRayBox.h"
#include "PxTriangleMeshGeometry.h"
#include "CmScaling.h"
#include "GuTriangleMeshBV4.h"

// This file contains code specific to the BV4 midphase.

// PT: TODO: revisit/inline static sweep functions (TA34704)

using namespace physx;
using namespace Gu;
using namespace Cm;

#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
Ps::IntBool	BV4_RaycastSingle		(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxRaycastHit* PX_RESTRICT hit, float maxDist, float geomEpsilon, PxU32 flags, PxHitFlags hitFlags);
PxU32		BV4_RaycastAll			(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxRaycastHit* PX_RESTRICT hits, PxU32 maxNbHits, float maxDist, float geomEpsilon, PxU32 flags, PxHitFlags hitFlags);
void		BV4_RaycastCB			(const PxVec3& origin, const PxVec3& dir, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, float maxDist, float geomEpsilon, PxU32 flags, MeshRayCallback callback, void* userData);

Ps::IntBool	BV4_OverlapSphereAny	(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned);
PxU32		BV4_OverlapSphereAll	(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow);
void		BV4_OverlapSphereCB		(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, MeshOverlapCallback callback, void* userData);

Ps::IntBool	BV4_OverlapBoxAny		(const Box& box, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned);
PxU32		BV4_OverlapBoxAll		(const Box& box, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow);
void		BV4_OverlapBoxCB		(const Box& box, const BV4Tree& tree, MeshOverlapCallback callback, void* userData);

Ps::IntBool	BV4_OverlapCapsuleAny	(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned);
PxU32		BV4_OverlapCapsuleAll	(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow);
void		BV4_OverlapCapsuleCB	(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, MeshOverlapCallback callback, void* userData);

Ps::IntBool	BV4_SphereSweepSingle	(const Sphere& sphere, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepHit* PX_RESTRICT hit, PxU32 flags);
void		BV4_SphereSweepCB		(const Sphere& sphere, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting);

Ps::IntBool	BV4_BoxSweepSingle		(const Box& box, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepHit* PX_RESTRICT hit, PxU32 flags);
void		BV4_BoxSweepCB			(const Box& box, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting);

Ps::IntBool	BV4_CapsuleSweepSingle	(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags);
Ps::IntBool	BV4_CapsuleSweepSingleAA(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags);
void		BV4_CapsuleSweepCB		(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags);
void		BV4_CapsuleSweepAACB	(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags);

void		BV4_GenericSweepCB_Old	(const PxVec3& origin, const PxVec3& extents, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, MeshSweepCallback callback, void* userData);
void		BV4_GenericSweepCB		(const Box& box, const PxVec3& dir, float maxDist, const BV4Tree& tree, MeshSweepCallback callback, void* userData, bool anyHit);

static PX_FORCE_INLINE void setIdentity(PxMat44& m)
{
	m.column0 = PxVec4(1.0f, 0.0f, 0.0f, 0.0f);
	m.column1 = PxVec4(0.0f, 1.0f, 0.0f, 0.0f);
	m.column2 = PxVec4(0.0f, 0.0f, 1.0f, 0.0f);
	m.column3 = PxVec4(0.0f, 0.0f, 0.0f, 1.0f);
}

static PX_FORCE_INLINE void setRotation(PxMat44& m, const PxQuat& q)
{
	const PxReal x = q.x;
	const PxReal y = q.y;
	const PxReal z = q.z;
	const PxReal w = q.w;

	const PxReal x2 = x + x;
	const PxReal y2 = y + y;
	const PxReal z2 = z + z;

	const PxReal xx = x2*x;
	const PxReal yy = y2*y;
	const PxReal zz = z2*z;

	const PxReal xy = x2*y;
	const PxReal xz = x2*z;
	const PxReal xw = x2*w;

	const PxReal yz = y2*z;
	const PxReal yw = y2*w;
	const PxReal zw = z2*w;

	m.column0 = PxVec4(1.0f - yy - zz, xy + zw, xz - yw, 0.0f);
	m.column1 = PxVec4(xy - zw, 1.0f - xx - zz, yz + xw, 0.0f);
	m.column2 = PxVec4(xz + yw, yz - xw, 1.0f - xx - yy, 0.0f);
}

#define IEEE_1_0	0x3f800000	//!< integer representation of 1.0
static PX_FORCE_INLINE const PxMat44* setupWorldMatrix(PxMat44& world, const float* meshPos, const float* meshRot)
{
//	world = PxMat44(PxIdentity);
	setIdentity(world);

	bool isIdt = true;
	if(meshRot)
	{
		const PxU32* Bin = reinterpret_cast<const PxU32*>(meshRot);
		if(Bin[0]!=0 || Bin[1]!=0 || Bin[2]!=0 || Bin[3]!=IEEE_1_0)
		{
//			const PxQuat Q(meshRot[0], meshRot[1], meshRot[2], meshRot[3]);
//			world = PxMat44(Q);
			setRotation(world, PxQuat(meshRot[0], meshRot[1], meshRot[2], meshRot[3]));
			isIdt = false;
		}
	}

	if(meshPos)
	{
		const PxU32* Bin = reinterpret_cast<const PxU32*>(meshPos);
		if(Bin[0]!=0 || Bin[1]!=0 || Bin[2]!=0)
		{
//			world.setPosition(PxVec3(meshPos[0], meshPos[1], meshPos[2]));
			world.column3.x = meshPos[0];
			world.column3.y = meshPos[1];
			world.column3.z = meshPos[2];
			isIdt = false;
		}
	}
	return isIdt ? NULL : &world;
}

static PX_FORCE_INLINE PxU32 setupFlags(bool anyHit, bool doubleSided, bool meshBothSides)
{
	PxU32 flags = 0;
	if(anyHit)
		flags |= QUERY_MODIFIER_ANY_HIT;
	if(doubleSided)
		flags |= QUERY_MODIFIER_DOUBLE_SIDED;
	if(meshBothSides)
		flags |= QUERY_MODIFIER_MESH_BOTH_SIDES;
	return flags;
}

static Ps::IntBool boxSweepVsMesh(SweepHit& h, const BV4Tree& tree, const float* meshPos, const float* meshRot, const Box& box, const PxVec3& dir, float maxDist, bool anyHit, bool doubleSided, bool meshBothSides)
{
	BV4_ALIGN16(PxMat44 World);
	const PxMat44* TM = setupWorldMatrix(World, meshPos, meshRot);

	const PxU32 flags = setupFlags(anyHit, doubleSided, meshBothSides);
	return BV4_BoxSweepSingle(box, dir, maxDist, tree, TM, &h, flags);
}

static Ps::IntBool sphereSweepVsMesh(SweepHit& h, const BV4Tree& tree, const PxVec3& center, float radius, const PxVec3& dir, float maxDist, const PxMat44* TM, const PxU32 flags)
{
	// PT: TODO: avoid this copy (TA34704)
	const Sphere tmp(center, radius);

	return BV4_SphereSweepSingle(tmp, dir, maxDist, tree, TM, &h, flags);
}

static bool capsuleSweepVsMesh(SweepHit& h, const BV4Tree& tree, const Capsule& capsule, const PxVec3& dir, float maxDist, const PxMat44* TM, const PxU32 flags)
{
	Capsule localCapsule;
	computeLocalCapsule(localCapsule, capsule, TM);

	// PT: TODO: optimize
	PxVec3 localDir, unused;
	computeLocalRay(localDir, unused, dir, dir, TM);

	const PxVec3 capsuleDir = localCapsule.p1 - localCapsule.p0;
	PxU32 nbNullComponents = 0;
	const float epsilon = 1e-3f;
	if(PxAbs(capsuleDir.x)<epsilon)
		nbNullComponents++;
	if(PxAbs(capsuleDir.y)<epsilon)
		nbNullComponents++;
	if(PxAbs(capsuleDir.z)<epsilon)
		nbNullComponents++;

	// PT: TODO: consider passing TM to BV4_CapsuleSweepSingleXX just to do the final transforms there instead
	// of below. It would make the parameters slightly inconsistent (local input + world TM) but it might make
	// the code better overall, more aligned with the "unlimited results" version.
	Ps::IntBool status;
	if(nbNullComponents==2)
	{
		status = BV4_CapsuleSweepSingleAA(localCapsule, localDir, maxDist, tree, &h, flags);
	}
	else
	{
		status = BV4_CapsuleSweepSingle(localCapsule, localDir, maxDist, tree, &h, flags);
	}
	if(status && TM)
	{
		h.mPos		= TM->transform(h.mPos);
		h.mNormal	= TM->rotate(h.mNormal);
	}
	return status!=0;
}

static PX_FORCE_INLINE void boxSweepVsMeshCBOld(const BV4Tree& tree, const float* meshPos, const float* meshRot, const PxVec3& center, const PxVec3& extents, const PxVec3& dir, float maxDist, MeshSweepCallback callback, void* userData)
{
	BV4_ALIGN16(PxMat44 World);
	const PxMat44* TM = setupWorldMatrix(World, meshPos, meshRot);

	BV4_GenericSweepCB_Old(center, extents, dir, maxDist, tree, TM, callback, userData);
}

//

static PX_FORCE_INLINE bool raycastVsMesh(PxRaycastHit& hitData, const BV4Tree& tree, const float* meshPos, const float* meshRot, const PxVec3& orig, const PxVec3& dir, float maxDist, float geomEpsilon, bool doubleSided, PxHitFlags hitFlags)
{
	BV4_ALIGN16(PxMat44 World);
	const PxMat44* TM = setupWorldMatrix(World, meshPos, meshRot);

	const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;
	const PxU32 flags = setupFlags(anyHit, doubleSided, false);
	
	if(!BV4_RaycastSingle(orig, dir, tree, TM, &hitData, maxDist, geomEpsilon, flags, hitFlags))
		return false;

	return true;
}

/*static PX_FORCE_INLINE PxU32 raycastVsMeshAll(PxRaycastHit* hits, PxU32 maxNbHits, const BV4Tree& tree, const float* meshPos, const float* meshRot, const PxVec3& orig, const PxVec3& dir, float maxDist, float geomEpsilon, bool doubleSided, PxHitFlags hitFlags)
{
	BV4_ALIGN16(PxMat44 World);
	const PxMat44* TM = setupWorldMatrix(World, meshPos, meshRot);

	const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;
	const PxU32 flags = setupFlags(anyHit, doubleSided, false);
	
	return BV4_RaycastAll(orig, dir, tree, TM, hits, maxNbHits, maxDist, geomEpsilon, flags, hitFlags);
}*/

static PX_FORCE_INLINE void raycastVsMeshCB(const BV4Tree& tree, const PxVec3& orig, const PxVec3& dir, float maxDist, float geomEpsilon, bool doubleSided, MeshRayCallback callback, void* userData)
{
	const PxU32 flags = setupFlags(false, doubleSided, false);
	BV4_RaycastCB(orig, dir, tree, NULL, maxDist, geomEpsilon, flags, callback, userData);
}

struct BV4RaycastCBParams
{
	PX_FORCE_INLINE BV4RaycastCBParams(	PxRaycastHit* hits, PxU32 maxHits, const PxMeshScale* scale, const PxTransform* pose,
										const Cm::Matrix34* world2vertexSkew, PxU32 hitFlags,
										const PxVec3& rayDir, bool isDoubleSided, float distCoeff) :
		mDstBase			(hits),
		mHitNum				(0),
		mMaxHits			(maxHits),
		mScale				(scale),
		mPose				(pose),
		mWorld2vertexSkew	(world2vertexSkew),
		mHitFlags			(hitFlags),
		mRayDir				(rayDir),
		mIsDoubleSided		(isDoubleSided),
		mDistCoeff			(distCoeff)
	{
	}

	PxRaycastHit*		mDstBase;
	PxU32				mHitNum;
	PxU32				mMaxHits;
	const PxMeshScale*	mScale;
	const PxTransform*	mPose;
	const Cm::Matrix34*	mWorld2vertexSkew;
	PxU32				mHitFlags;
	const PxVec3&		mRayDir;
	bool				mIsDoubleSided;
	float				mDistCoeff;

private:
	BV4RaycastCBParams& operator=(const BV4RaycastCBParams&);
};

static PX_FORCE_INLINE PxVec3 processLocalNormal(const Cm::Matrix34* PX_RESTRICT world2vertexSkew, const PxTransform* PX_RESTRICT pose, const PxVec3& localNormal, const PxVec3& rayDir, const bool isDoubleSided)
{
	PxVec3 normal;
	if(world2vertexSkew)
		normal = world2vertexSkew->rotateTranspose(localNormal);
	else
		normal = pose->rotate(localNormal);
	normal.normalize();

	// PT: figure out correct normal orientation (DE7458)
	// - if the mesh is single-sided the normal should be the regular triangle normal N, regardless of eMESH_BOTH_SIDES.
	// - if the mesh is double-sided the correct normal can be either N or -N. We take the one opposed to ray direction.
	if(isDoubleSided && normal.dot(rayDir) > 0.0f)
		normal = -normal;
	return normal;
}

static HitCode gRayCallback(void* userData, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2, PxU32 triangleIndex, float dist, float u, float v)
{
	BV4RaycastCBParams* params = reinterpret_cast<BV4RaycastCBParams*>(userData);

//const bool last = params->mHitNum == params->mMaxHits;

	//not worth concatenating to do 1 transform: PxMat34Legacy vertex2worldSkew = scaling.getVertex2WorldSkew(absPose);
	// PT: TODO: revisit this for N hits
	PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxRaycastHit)] PX_ALIGN_SUFFIX(16);
	PxRaycastHit& hit = reinterpret_cast<PxRaycastHit&>(buffer);
//PxRaycastHit& hit = last ? (PxRaycastHit&)buffer : params->mDstBase[params->mHitNum];

	hit.distance = dist * params->mDistCoeff;
	hit.u = u;
	hit.v = v;
	hit.faceIndex = triangleIndex;

	PxVec3 localImpact = (1.0f - u - v)*lp0 + u*lp1 + v*lp2;
	if(params->mWorld2vertexSkew)
	{
		localImpact = params->mScale->transform(localImpact);
		if(params->mScale->hasNegativeDeterminant())
			Ps::swap<PxReal>(hit.u, hit.v); // have to swap the UVs though since they were computed in mesh local space
	}

	hit.position = params->mPose->transform(localImpact);
	hit.flags = PxHitFlag::ePOSITION|PxHitFlag::eUV|PxHitFlag::eFACE_INDEX;

	PxVec3 normal(0.0f);
	// Compute additional information if needed
	if(params->mHitFlags & PxHitFlag::eNORMAL)
	{
		const PxVec3 localNormal = (lp1 - lp0).cross(lp2 - lp0);
		normal = processLocalNormal(params->mWorld2vertexSkew, params->mPose, localNormal, params->mRayDir, params->mIsDoubleSided);
		hit.flags |= PxHitFlag::eNORMAL;
	}
	hit.normal = normal;

	// PT: no callback => store results in provided buffer
	if(params->mHitNum == params->mMaxHits)
//	if(last)
		return HIT_EXIT;

	params->mDstBase[params->mHitNum++] = hit;
//	params->mHitNum++;

	return HIT_NONE;
}

PxU32 physx::Gu::raycast_triangleMesh_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
											const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
											PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4TriangleMesh* meshData = static_cast<const BV4TriangleMesh*>(mesh);

	const bool multipleHits = (maxHits > 1);
	const bool idtScale = meshGeom.scale.isIdentity();

	const bool isDoubleSided = meshGeom.meshFlags.isSet(PxMeshGeometryFlag::eDOUBLE_SIDED);
	const bool bothSides = isDoubleSided || (hitFlags & PxHitFlag::eMESH_BOTH_SIDES);

	const BV4Tree& tree = static_cast<const BV4TriangleMesh*>(meshData)->getBV4Tree();
	if(idtScale && !multipleHits)
	{
		bool b = raycastVsMesh(*hits, tree, &pose.p.x, &pose.q.x, rayOrigin, rayDir, maxDist, meshData->getGeomEpsilon(), bothSides, hitFlags);
		if(b)
		{
			PxHitFlags dstFlags = PxHitFlag::ePOSITION|PxHitFlag::eUV|PxHitFlag::eFACE_INDEX;

			// PT: TODO: pass flags to BV4 code (TA34704)
			if(hitFlags & PxHitFlag::eNORMAL)
			{
				dstFlags |= PxHitFlag::eNORMAL;
				if(isDoubleSided)
				{
					PxVec3 normal = hits->normal;
					// PT: figure out correct normal orientation (DE7458)
					// - if the mesh is single-sided the normal should be the regular triangle normal N, regardless of eMESH_BOTH_SIDES.
					// - if the mesh is double-sided the correct normal can be either N or -N. We take the one opposed to ray direction.
					if(normal.dot(rayDir) > 0.0f)
						normal = -normal;
					hits->normal = normal;
				}
			}
			else
			{
				hits->normal = PxVec3(0.0f);
			}
			hits->flags = dstFlags;
		}
		return PxU32(b);
	}

/*
	if(idtScale && multipleHits)
	{
		PxU32 nbHits = raycastVsMeshAll(hits, maxHits, tree, &pose.p.x, &pose.q.x, rayOrigin, rayDir, maxDist, meshData->getGeomEpsilon(), bothSides, hitFlags);

		return nbHits;
	}
*/

	//scaling: transform the ray to vertex space
	PxVec3 orig, dir;
	Cm::Matrix34 world2vertexSkew;
	Cm::Matrix34* world2vertexSkewP = NULL;
	PxReal distCoeff = 1.0f;
	if(idtScale)
	{
		orig = pose.transformInv(rayOrigin);
		dir = pose.rotateInv(rayDir);
	}
	else
	{
		world2vertexSkew = meshGeom.scale.getInverse() * pose.getInverse();
		world2vertexSkewP = &world2vertexSkew;
		orig = world2vertexSkew.transform(rayOrigin);
		dir = world2vertexSkew.rotate(rayDir);
		{
			distCoeff = dir.normalize();
			maxDist *= distCoeff;
			maxDist += 1e-3f;
			distCoeff = 1.0f/distCoeff;
		}
	}

	if(!multipleHits)
	{
		bool b = raycastVsMesh(*hits, tree, NULL, NULL, orig, dir, maxDist, meshData->getGeomEpsilon(), bothSides, hitFlags);
		if(b)
		{
			hits->distance	*= distCoeff;
			hits->position	= pose.transform(meshGeom.scale.transform(hits->position));
			PxHitFlags dstFlags = PxHitFlag::ePOSITION|PxHitFlag::eUV|PxHitFlag::eFACE_INDEX;

			if(meshGeom.scale.hasNegativeDeterminant())
				Ps::swap<PxReal>(hits->u, hits->v); // have to swap the UVs though since they were computed in mesh local space

			// PT: TODO: pass flags to BV4 code (TA34704)
			// Compute additional information if needed
			if(hitFlags & PxHitFlag::eNORMAL)
			{
				dstFlags |= PxHitFlag::eNORMAL;
				hits->normal = processLocalNormal(world2vertexSkewP, &pose, hits->normal, rayDir, isDoubleSided);
			}
			else
			{
				hits->normal = PxVec3(0.0f);
			}
			hits->flags = dstFlags;
		}
		return PxU32(b);
	}

	BV4RaycastCBParams callback(hits, maxHits, &meshGeom.scale, &pose, world2vertexSkewP, hitFlags, rayDir, isDoubleSided, distCoeff);

	raycastVsMeshCB(	static_cast<const BV4TriangleMesh*>(meshData)->getBV4Tree(),
						orig, dir,
						maxDist, meshData->getGeomEpsilon(), bothSides,
						gRayCallback, &callback);
	return callback.mHitNum;
}

namespace
{
struct IntersectShapeVsMeshCallback
{
	IntersectShapeVsMeshCallback(LimitedResults* results, bool flipNormal) :	 mResults(results), mAnyHits(false), mFlipNormal(flipNormal)	{}

	LimitedResults*	mResults;
	bool			mAnyHits;
	bool			mFlipNormal;

	PX_FORCE_INLINE	bool	recordHit(PxU32 faceIndex, Ps::IntBool hit)
	{
		if(hit)
		{
			mAnyHits = true;
			if(mResults)
				mResults->add(faceIndex);
			else
				return false; // abort traversal if we are only interested in firstContact (mResults is NULL)
		}
		return true; // if we are here, either no triangles were hit or multiple results are expected => continue traversal
	}
};

// PT: TODO: get rid of this (TA34704)
struct IntersectSphereVsMeshCallback : IntersectShapeVsMeshCallback
{
	PX_FORCE_INLINE IntersectSphereVsMeshCallback(const PxMeshScale& meshScale, const PxTransform& meshTransform, const Sphere& sphere, LimitedResults* r, bool flipNormal) 
		: IntersectShapeVsMeshCallback(r, flipNormal)
	{
		mVertexToShapeSkew = meshScale.toMat33();
		mLocalCenter = meshTransform.transformInv(sphere.center);	// sphereCenterInMeshSpace
		mSphereRadius2 = sphere.radius*sphere.radius;
	}

	PxMat33		mVertexToShapeSkew;
	PxVec3		mLocalCenter;	// PT: sphere center in local/mesh space
	PxF32		mSphereRadius2;

	PX_FORCE_INLINE PxAgain processHit(PxU32 faceIndex, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2)
	{
		const Vec3V v0 = V3LoadU(mVertexToShapeSkew * av0);
		const Vec3V v1 = V3LoadU(mVertexToShapeSkew * (mFlipNormal ? av2 : av1));
		const Vec3V v2 = V3LoadU(mVertexToShapeSkew * (mFlipNormal ? av1 : av2));

		FloatV dummy1, dummy2;
		Vec3V closestP;
		PxReal dist2;
		FStore(distancePointTriangleSquared(V3LoadU(mLocalCenter), v0, v1, v2, dummy1, dummy2, closestP), &dist2);
		return recordHit(faceIndex, dist2 <= mSphereRadius2);
	}
};

// PT: TODO: get rid of this (TA34704)
struct IntersectCapsuleVsMeshCallback : IntersectShapeVsMeshCallback
{
	PX_FORCE_INLINE IntersectCapsuleVsMeshCallback(const PxMeshScale& meshScale, const PxTransform& meshTransform, const Capsule& capsule, LimitedResults* r, bool flipNormal)
		: IntersectShapeVsMeshCallback(r, flipNormal)
	{
		mVertexToShapeSkew = meshScale.toMat33();

		// transform world capsule to mesh shape space
		mLocalCapsule.p0		= meshTransform.transformInv(capsule.p0);
		mLocalCapsule.p1		= meshTransform.transformInv(capsule.p1);
		mLocalCapsule.radius	= capsule.radius;
		mParams.init(mLocalCapsule);
	}

	PxMat33						mVertexToShapeSkew;
	Capsule						mLocalCapsule;		// PT: capsule in mesh/local space
	CapsuleTriangleOverlapData	mParams;

	PX_FORCE_INLINE PxAgain processHit(PxU32 faceIndex, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2)
	{
		const PxVec3 v0 = mVertexToShapeSkew * av0;
		const PxVec3 v1 = mVertexToShapeSkew * (mFlipNormal ? av2 : av1);
		const PxVec3 v2 = mVertexToShapeSkew * (mFlipNormal ? av1 : av2);
		const PxVec3 normal = (v0 - v1).cross(v0 - v2);
		bool hit = intersectCapsuleTriangle(normal, v0, v1, v2, mLocalCapsule, mParams);
		return recordHit(faceIndex, hit);
	}
};

// PT: TODO: get rid of this (TA34704)
struct IntersectBoxVsMeshCallback : IntersectShapeVsMeshCallback
{
	PX_FORCE_INLINE IntersectBoxVsMeshCallback(const PxMeshScale& meshScale, const PxTransform& meshTransform, const Box& box, LimitedResults* r, bool flipNormal)
		: IntersectShapeVsMeshCallback(r, flipNormal)
	{
		const PxMat33 vertexToShapeSkew = meshScale.toMat33();

		// mesh scale needs to be included - inverse transform and optimize the box
		const PxMat33 vertexToWorldSkew_Rot = PxMat33Padded(meshTransform.q) * vertexToShapeSkew;
		const PxVec3& vertexToWorldSkew_Trans = meshTransform.p;

		Matrix34 tmp;
		buildMatrixFromBox(tmp, box);
		const Matrix34 inv = tmp.getInverseRT();
		const Matrix34 _vertexToWorldSkew(vertexToWorldSkew_Rot, vertexToWorldSkew_Trans);

		mVertexToBox = inv * _vertexToWorldSkew;
		mBoxCenter = PxVec3(0.0f);
		mBoxExtents = box.extents; // extents do not change
	}

	Matrix34	mVertexToBox;
	Vec3p		mBoxExtents, mBoxCenter;

	PX_FORCE_INLINE PxAgain processHit(PxU32 faceIndex, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2)
	{
		const Vec3p v0 = mVertexToBox.transform(av0);
		const Vec3p v1 = mVertexToBox.transform(mFlipNormal ? av2 : av1);
		const Vec3p v2 = mVertexToBox.transform(mFlipNormal ? av1 : av2);

		// PT: this one is safe because we're using Vec3p for all parameters
		const Ps::IntBool hit = intersectTriangleBox_Unsafe(mBoxCenter, mBoxExtents, v0, v1, v2);
		return recordHit(faceIndex, hit);
	}
};
}

static bool gSphereVsMeshCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, const PxU32* /*vertexIndices*/)
{
	IntersectSphereVsMeshCallback* callback = reinterpret_cast<IntersectSphereVsMeshCallback*>(userData);
	return !callback->processHit(triangleIndex, p0, p1, p2);
}

static bool gCapsuleVsMeshCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, const PxU32* /*vertexIndices*/)
{
	IntersectCapsuleVsMeshCallback* callback = reinterpret_cast<IntersectCapsuleVsMeshCallback*>(userData);
	return !callback->processHit(triangleIndex, p0, p1, p2);
}

static bool gBoxVsMeshCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, const PxU32* /*vertexIndices*/)
{
	IntersectBoxVsMeshCallback* callback = reinterpret_cast<IntersectBoxVsMeshCallback*>(userData);
	return !callback->processHit(triangleIndex, p0, p1, p2);
}

bool physx::Gu::intersectSphereVsMesh_BV4(const Sphere& sphere, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	PX_ASSERT(triMesh.getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4Tree& tree = static_cast<const BV4TriangleMesh&>(triMesh).getBV4Tree();

	if(meshScale.isIdentity())
	{
		BV4_ALIGN16(PxMat44 World);
		const PxMat44* TM = setupWorldMatrix(World, &meshTransform.p.x, &meshTransform.q.x);
		if(results)
		{
			const PxU32 nbResults = BV4_OverlapSphereAll(sphere, tree, TM, results->mResults, results->mMaxResults, results->mOverflow);
			results->mNbResults = nbResults;
			return nbResults!=0;
		}
		else
		{
			return BV4_OverlapSphereAny(sphere, tree, TM)!=0;
		}
	}
	else
	{
		// PT: TODO: we don't need to use this callback here (TA34704)
		IntersectSphereVsMeshCallback callback(meshScale, meshTransform, sphere, results, meshScale.hasNegativeDeterminant());

		const Box worldOBB_(sphere.center, PxVec3(sphere.radius), PxMat33(PxIdentity));
		Box vertexOBB;
		computeVertexSpaceOBB(vertexOBB, worldOBB_, meshTransform, meshScale);

		BV4_OverlapBoxCB(vertexOBB, tree, gSphereVsMeshCallback, &callback);
		return callback.mAnyHits;
	}
}

bool physx::Gu::intersectBoxVsMesh_BV4(const Box& box, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	PX_ASSERT(triMesh.getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4Tree& tree = static_cast<const BV4TriangleMesh&>(triMesh).getBV4Tree();

	if(meshScale.isIdentity())
	{
		BV4_ALIGN16(PxMat44 World);
		const PxMat44* TM = setupWorldMatrix(World, &meshTransform.p.x, &meshTransform.q.x);
		if(results)
		{
			const PxU32 nbResults = BV4_OverlapBoxAll(box, tree, TM, results->mResults, results->mMaxResults, results->mOverflow);
			results->mNbResults = nbResults;
			return nbResults!=0;
		}
		else
		{
			return BV4_OverlapBoxAny(box, tree, TM)!=0;
		}
	}
	else
	{
		// PT: TODO: we don't need to use this callback here (TA34704)
		IntersectBoxVsMeshCallback callback(meshScale, meshTransform, box, results, meshScale.hasNegativeDeterminant());

		Box vertexOBB; // query box in vertex space
		computeVertexSpaceOBB(vertexOBB, box, meshTransform, meshScale);

		BV4_OverlapBoxCB(vertexOBB, tree, gBoxVsMeshCallback, &callback);
		return callback.mAnyHits;
	}
}

bool physx::Gu::intersectCapsuleVsMesh_BV4(const Capsule& capsule, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	PX_ASSERT(triMesh.getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4Tree& tree = static_cast<const BV4TriangleMesh&>(triMesh).getBV4Tree();

	if(meshScale.isIdentity())
	{
		BV4_ALIGN16(PxMat44 World);
		const PxMat44* TM = setupWorldMatrix(World, &meshTransform.p.x, &meshTransform.q.x);
		if(results)
		{
			const PxU32 nbResults = BV4_OverlapCapsuleAll(capsule, tree, TM, results->mResults, results->mMaxResults, results->mOverflow);
			results->mNbResults = nbResults;
			return nbResults!=0;
		}
		else
		{
			return BV4_OverlapCapsuleAny(capsule, tree, TM)!=0;
		}
	}
	else
	{
		// PT: TODO: we don't need to use this callback here (TA34704)
		IntersectCapsuleVsMeshCallback callback(meshScale, meshTransform, capsule, results, meshScale.hasNegativeDeterminant());

		// make vertex space OBB
		Box vertexOBB;
		Box worldOBB_;
		worldOBB_.create(capsule); // AP: potential optimization (meshTransform.inverse is already in callback.mCapsule)
		computeVertexSpaceOBB(vertexOBB, worldOBB_, meshTransform, meshScale);

		BV4_OverlapBoxCB(vertexOBB, tree, gCapsuleVsMeshCallback, &callback);
		return callback.mAnyHits;
	}
}

// PT: TODO: get rid of this (TA34704)
static bool gVolumeCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, const PxU32* vertexIndices)
{
	MeshHitCallback<PxRaycastHit>* callback = reinterpret_cast<MeshHitCallback<PxRaycastHit>*>(userData);
	PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxRaycastHit)] PX_ALIGN_SUFFIX(16);
	PxRaycastHit& hit = reinterpret_cast<PxRaycastHit&>(buffer);
	hit.faceIndex = triangleIndex;
	PxReal dummy;
	return !callback->processHit(hit, p0, p1, p2, dummy, vertexIndices);
}

void physx::Gu::intersectOBB_BV4(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned)
{
	PX_UNUSED(checkObbIsAligned);
	PX_UNUSED(bothTriangleSidesCollide);
	BV4_OverlapBoxCB(obb, static_cast<const BV4TriangleMesh*>(mesh)->getBV4Tree(), gVolumeCallback, &callback);
}




#include "GuVecCapsule.h"
#include "GuSweepMTD.h"

static bool gCapsuleMeshSweepCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, /*const PxU32* vertexIndices,*/ float& dist)
{
	SweepCapsuleMeshHitCallback* callback = reinterpret_cast<SweepCapsuleMeshHitCallback*>(userData);
	PxRaycastHit meshHit;
	meshHit.faceIndex = triangleIndex;
	return !callback->SweepCapsuleMeshHitCallback::processHit(meshHit, p0, p1, p2, dist, NULL/*vertexIndices*/);
}

// PT: TODO: refactor/share bits of this (TA34704)
bool physx::Gu::sweepCapsule_MeshGeom_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
											const Capsule& lss, const PxVec3& unitDir, const PxReal distance,
											PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4TriangleMesh* meshData = static_cast<const BV4TriangleMesh*>(mesh);

	const Capsule inflatedCapsule(lss.p0, lss.p1, lss.radius + inflation);

	const bool isIdentity = triMeshGeom.scale.isIdentity();
	bool isDoubleSided = (triMeshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED);
	const PxU32 meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;

	if(isIdentity)
	{
		const BV4Tree& tree = meshData->getBV4Tree();
		const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;

		BV4_ALIGN16(PxMat44 World);
		const PxMat44* TM = setupWorldMatrix(World, &pose.p.x, &pose.q.x);

		const PxU32 flags = setupFlags(anyHit, isDoubleSided, meshBothSides!=0);

		SweepHit hitData;
		if(lss.p0==lss.p1)
		{
			if(!sphereSweepVsMesh(hitData, tree, inflatedCapsule.p0, inflatedCapsule.radius, unitDir, distance, TM, flags))
				return false;
		}
		else
		{
			if(!capsuleSweepVsMesh(hitData, tree, inflatedCapsule, unitDir, distance, TM, flags))
				return false;
		}

		sweepHit.distance = hitData.mDistance;
		sweepHit.position = hitData.mPos;
		sweepHit.normal = hitData.mNormal;
		sweepHit.faceIndex = hitData.mTriangleID;

		if(hitData.mDistance==0.0f)
		{
			sweepHit.flags = PxHitFlag::eNORMAL;

			if(meshBothSides)
				isDoubleSided = true;

			// PT: TODO: consider using 'setInitialOverlapResults' here
			bool hasContacts = false;
			if(hitFlags & PxHitFlag::eMTD)
			{
				const Vec3V p0 = V3LoadU(inflatedCapsule.p0);
				const Vec3V p1 = V3LoadU(inflatedCapsule.p1);
				const FloatV radius = FLoad(lss.radius);
				CapsuleV capsuleV;
				capsuleV.initialize(p0, p1, radius);

				//we need to calculate the MTD
				hasContacts = computeCapsule_TriangleMeshMTD(triMeshGeom, pose, capsuleV, inflatedCapsule.radius, isDoubleSided, sweepHit);
			}
			setupSweepHitForMTD(sweepHit, hasContacts, unitDir);
		}
		else
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		return true;
	}

	// compute sweptAABB
	const PxVec3 localP0 = pose.transformInv(inflatedCapsule.p0);
	const PxVec3 localP1 = pose.transformInv(inflatedCapsule.p1);
	PxVec3 sweepOrigin = (localP0+localP1)*0.5f;
	PxVec3 sweepDir = pose.rotateInv(unitDir);
	PxVec3 sweepExtents = PxVec3(inflatedCapsule.radius) + (localP0-localP1).abs()*0.5f;
	PxReal distance1 = distance;
	PxReal distCoef = 1.0f;
	Matrix34 poseWithScale;
	if(!isIdentity)
	{
		poseWithScale = pose * triMeshGeom.scale;
		distance1 = computeSweepData(triMeshGeom, sweepOrigin, sweepExtents, sweepDir, distance);
		distCoef = distance1 / distance;
	} else
		poseWithScale = Matrix34(pose);

	SweepCapsuleMeshHitCallback callback(sweepHit, poseWithScale, distance, isDoubleSided, inflatedCapsule, unitDir, hitFlags, triMeshGeom.scale.hasNegativeDeterminant(), distCoef);

	boxSweepVsMeshCBOld(meshData->getBV4Tree(), NULL, NULL, sweepOrigin, sweepExtents, sweepDir, distance1, gCapsuleMeshSweepCallback, &callback);

	if(meshBothSides)
		isDoubleSided = true;

	return callback.finalizeHit(sweepHit, inflatedCapsule, triMeshGeom, pose, isDoubleSided);
}

#include "GuSweepSharedTests.h"
static bool gBoxMeshSweepCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, /*const PxU32* vertexIndices,*/ float& dist)
{
	SweepBoxMeshHitCallback* callback = reinterpret_cast<SweepBoxMeshHitCallback*>(userData);
	PxRaycastHit meshHit;
	meshHit.faceIndex = triangleIndex;
	return !callback->SweepBoxMeshHitCallback::processHit(meshHit, p0, p1, p2, dist, NULL/*vertexIndices*/);
}

// PT: TODO: refactor/share bits of this (TA34704)
bool physx::Gu::sweepBox_MeshGeom_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
										const Box& box, const PxVec3& unitDir, const PxReal distance,
										PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4TriangleMesh* meshData = static_cast<const BV4TriangleMesh*>(mesh);

	const bool isIdentity = triMeshGeom.scale.isIdentity();

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool isDoubleSided = triMeshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED;

	if(isIdentity && inflation==0.0f)
	{
		const bool anyHit = hitFlags & PxHitFlag::eMESH_ANY;

		// PT: TODO: this is wrong, we shouldn't actually sweep the inflated version
//		const PxVec3 inflated = (box.extents + PxVec3(inflation)) * 1.01f;
		// PT: TODO: avoid this copy
//		const Box tmp(box.center, inflated, box.rot);

		SweepHit hitData;
//		if(!boxSweepVsMesh(hitData, meshData->getBV4Tree(), &pose.p.x, &pose.q.x, tmp, unitDir, distance, anyHit, isDoubleSided, meshBothSides))
		if(!boxSweepVsMesh(hitData, meshData->getBV4Tree(), &pose.p.x, &pose.q.x, box, unitDir, distance, anyHit, isDoubleSided, meshBothSides))
			return false;

		sweepHit.distance = hitData.mDistance;
		sweepHit.position = hitData.mPos;
		sweepHit.normal = hitData.mNormal;
		sweepHit.faceIndex = hitData.mTriangleID;

		if(hitData.mDistance==0.0f)
		{
			sweepHit.flags = PxHitFlag::eNORMAL;

			const bool bothTriangleSidesCollide = isDoubleSided || meshBothSides;
			const PxTransform boxTransform = box.getTransform();

			bool hasContacts = false;
			if(hitFlags & PxHitFlag::eMTD)
				hasContacts = computeBox_TriangleMeshMTD(triMeshGeom, pose, box, boxTransform, inflation, bothTriangleSidesCollide,  sweepHit);

			setupSweepHitForMTD(sweepHit, hasContacts, unitDir);
		}
		else
		{
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
		}
		return true;
	}

	// PT: TODO: revisit this codepath, we don't need to sweep an AABB all the time (TA34704)

	Matrix34 meshToWorldSkew;
	PxVec3 sweptAABBMeshSpaceExtents, meshSpaceOrigin, meshSpaceDir;

	// Input sweep params: geom, pose, box, unitDir, distance
	// We convert the origin from world space to mesh local space
	// and convert the box+pose to mesh space AABB
	if(isIdentity)
	{
		meshToWorldSkew = Matrix34(pose);
		PxMat33 worldToMeshRot(pose.q.getConjugate()); // extract rotation matrix from pose.q
		meshSpaceOrigin = worldToMeshRot.transform(box.center - pose.p);
		meshSpaceDir = worldToMeshRot.transform(unitDir) * distance;
		PxMat33 boxToMeshRot = worldToMeshRot * box.rot;
		sweptAABBMeshSpaceExtents = boxToMeshRot.column0.abs() * box.extents.x + 
						   boxToMeshRot.column1.abs() * box.extents.y + 
						   boxToMeshRot.column2.abs() * box.extents.z;
	}
	else
	{
		meshToWorldSkew = pose * triMeshGeom.scale;
		const PxMat33 meshToWorldSkew_Rot = PxMat33Padded(pose.q) * triMeshGeom.scale.toMat33();
		const PxVec3& meshToWorldSkew_Trans = pose.p;

		PxMat33 worldToVertexSkew_Rot;
		PxVec3 worldToVertexSkew_Trans;
		getInverse(worldToVertexSkew_Rot, worldToVertexSkew_Trans, meshToWorldSkew_Rot, meshToWorldSkew_Trans);

		//make vertex space OBB
		Box vertexSpaceBox1;
		const Matrix34 worldToVertexSkew(worldToVertexSkew_Rot, worldToVertexSkew_Trans);
		vertexSpaceBox1 = transform(worldToVertexSkew, box);
		// compute swept aabb
		sweptAABBMeshSpaceExtents = vertexSpaceBox1.computeAABBExtent();

		meshSpaceOrigin = worldToVertexSkew.transform(box.center);
		meshSpaceDir = worldToVertexSkew.rotate(unitDir*distance); // also applies scale to direction/length
	}

	sweptAABBMeshSpaceExtents += PxVec3(inflation); // inflate the bounds with additive inflation
	sweptAABBMeshSpaceExtents *= 1.01f; // fatten the bounds to account for numerical discrepancies

	PxReal dirLen = PxMax(meshSpaceDir.magnitude(), 1e-5f);
	PxReal distCoeff = 1.0f;
	if (!isIdentity)
		distCoeff = dirLen / distance;

	// Move to AABB space
	Matrix34 worldToBox;
	computeWorldToBoxMatrix(worldToBox, box);

	const bool bothTriangleSidesCollide = isDoubleSided || meshBothSides;

	const Matrix34Padded meshToBox = worldToBox*meshToWorldSkew;
	const PxTransform boxTransform = box.getTransform();	// PT: TODO: this is not needed when there's no hit (TA34704)

	const PxVec3 localDir = worldToBox.rotate(unitDir);
	const PxVec3 localDirDist = localDir*distance;
	SweepBoxMeshHitCallback callback( // using eMULTIPLE with shrinkMaxT
		CallbackMode::eMULTIPLE, meshToBox, distance, bothTriangleSidesCollide, box, localDirDist, localDir, unitDir, hitFlags, inflation, triMeshGeom.scale.hasNegativeDeterminant(), distCoeff);

	const PxVec3 dir = meshSpaceDir/dirLen;
	boxSweepVsMeshCBOld(meshData->getBV4Tree(), NULL, NULL, meshSpaceOrigin, sweptAABBMeshSpaceExtents, dir, dirLen, gBoxMeshSweepCallback, &callback);

	return callback.finalizeHit(sweepHit, triMeshGeom, pose, boxTransform, localDir, meshBothSides, isDoubleSided);
}

static bool gConvexVsMeshSweepCallback(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, /*const PxU32* vertexIndices,*/ float& dist)
{
	SweepConvexMeshHitCallback* callback = reinterpret_cast<SweepConvexMeshHitCallback*>(userData);
	PX_ALIGN_PREFIX(16)	char buffer[sizeof(PxRaycastHit)] PX_ALIGN_SUFFIX(16);
	PxRaycastHit& hit = reinterpret_cast<PxRaycastHit&>(buffer);
	hit.faceIndex = triangleIndex;
	return !callback->SweepConvexMeshHitCallback::processHit(hit, p0, p1, p2, dist, NULL/*vertexIndices*/);
}

void physx::Gu::sweepConvex_MeshGeom_BV4(const TriangleMesh* mesh, const Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool anyHit)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH34);
	const BV4TriangleMesh* meshData = static_cast<const BV4TriangleMesh*>(mesh);
	BV4_GenericSweepCB(hullBox, localDir, distance, meshData->getBV4Tree(), gConvexVsMeshSweepCallback, &callback, anyHit);
}

#endif

