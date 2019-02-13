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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "GuSweepMesh.h"
#include "GuIntersectionRayTriangle.h"
#include "GuIntersectionCapsuleTriangle.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionRayBoxSIMD.h"
#include "GuSphere.h"
#include "GuBoxConversion.h"
#include "GuConvexUtilsInternal.h"
#include "GuVecTriangle.h"
#include "GuIntersectionTriangleBox.h"
#include "GuSIMDHelpers.h"
#include "GuTriangleVertexPointers.h"
#include "GuRTree.h"
#include "GuTriangleMeshRTree.h"
#include "GuInternal.h"

// This file contains code specific to the RTree midphase.

using namespace physx;
using namespace Cm;
using namespace Gu;
using namespace physx::shdfnd::aos;

struct MeshRayCollider
{
	template <int tInflate, int tRayTest>
	PX_PHYSX_COMMON_API static void collide(
		const PxVec3& orig, const PxVec3& dir, // dir is not normalized (full length), both in mesh space (unless meshWorld is non-zero)
		PxReal maxT, // maxT is from [0,1], if maxT is 0.0f, AABB traversal will be used
		bool bothTriangleSidesCollide, const RTreeTriangleMesh* mesh, MeshHitCallback<PxRaycastHit>& callback,
		const PxVec3* inflate = NULL);

	PX_PHYSX_COMMON_API static void collideOBB(
		const Box& obb, bool bothTriangleSidesCollide, const RTreeTriangleMesh* mesh, MeshHitCallback<PxRaycastHit>& callback,
		bool checkObbIsAligned = true); // perf hint, pass false if obb is rarely axis aligned
};

class SimpleRayTriOverlap
{
public:
	PX_FORCE_INLINE SimpleRayTriOverlap(const PxVec3& origin, const PxVec3& dir, bool bothSides, PxReal geomEpsilon)
		: mOrigin(origin), mDir(dir), mBothSides(bothSides), mGeomEpsilon(geomEpsilon)
	{
	}

	PX_FORCE_INLINE Ps::IntBool overlap(const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, PxRaycastHit& hit) const
	{
		if(!intersectRayTriangle(mOrigin, mDir, vert0, vert1, vert2, hit.distance, hit.u, hit.v, !mBothSides, mGeomEpsilon))
			return false;

		if(hit.distance< 0.0f) // test if the ray intersection t is negative
			return false;

		return true;
	}

	PxVec3	mOrigin;
	PxVec3	mDir;
	bool	mBothSides;
	PxReal	mGeomEpsilon;
};

using Gu::RTree;

// This callback comes from RTree and decodes LeafTriangle indices stored in rtree into actual triangles
// This callback is needed because RTree doesn't know that it stores triangles since it's a general purpose spatial index

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

template <int tInflate, bool tRayTest>
struct RayRTreeCallback : RTree::CallbackRaycast, RTree::Callback
{
	MeshHitCallback<PxRaycastHit>& outerCallback;
	PxI32 has16BitIndices;
	const void* mTris;
	const PxVec3* mVerts;
	const PxVec3* mInflate;
	const SimpleRayTriOverlap rayCollider;
	PxReal maxT;
	PxRaycastHit closestHit; // recorded closest hit over the whole traversal (only for callback mode eCLOSEST)
	PxVec3 cv0, cv1, cv2;	// PT: make sure these aren't last in the class, to safely V4Load them
	PxU32 cis[3];
	bool hadClosestHit;
	const bool closestMode;
	Vec3V inflateV, rayOriginV, rayDirV;

	RayRTreeCallback(
		PxReal geomEpsilon, MeshHitCallback<PxRaycastHit>& callback,
		PxI32 has16BitIndices_, const void* tris, const PxVec3* verts,
		const PxVec3& origin, const PxVec3& dir, PxReal maxT_, bool bothSides, const PxVec3* inflate)
		:	outerCallback(callback), has16BitIndices(has16BitIndices_),
			mTris(tris), mVerts(verts), mInflate(inflate), rayCollider(origin, dir, bothSides, geomEpsilon),
			maxT(maxT_), closestMode(callback.inClosestMode())
	{
		PX_ASSERT(closestHit.distance == PX_MAX_REAL);
		hadClosestHit = false;
		if (tInflate)
			inflateV = V3LoadU(*mInflate);
		rayOriginV = V3LoadU(rayCollider.mOrigin);
		rayDirV = V3LoadU(rayCollider.mDir);
	}

	PX_FORCE_INLINE void getVertIndices(PxU32 triIndex, PxU32& i0, PxU32 &i1, PxU32 &i2)
	{
		if(has16BitIndices)
		{
			const PxU16* p = reinterpret_cast<const PxU16*>(mTris) + triIndex*3;
			i0 = p[0]; i1 = p[1]; i2 = p[2];
		}
		else
		{
			const PxU32* p = reinterpret_cast<const PxU32*>(mTris) + triIndex*3;
			i0 = p[0]; i1 = p[1]; i2 = p[2];
		}
	}

	virtual PX_FORCE_INLINE bool processResults(PxU32 NumTouched, PxU32* Touched, PxF32& newMaxT)
	{
		PX_ASSERT(NumTouched > 0);
		// Loop through touched leaves
		PxRaycastHit tempHit;
		for(PxU32 leaf = 0; leaf<NumTouched; leaf++)
		{
			// Each leaf box has a set of triangles
			LeafTriangles currentLeaf;
			currentLeaf.Data = Touched[leaf];
			PxU32 nbLeafTris = currentLeaf.GetNbTriangles();			
			PxU32 baseLeafTriIndex = currentLeaf.GetTriangleIndex();

			for(PxU32 i = 0; i < nbLeafTris; i++)
			{
				PxU32 i0, i1, i2;
				const PxU32 triangleIndex = baseLeafTriIndex+i;
				getVertIndices(triangleIndex, i0, i1, i2);

				const PxVec3& v0 = mVerts[i0], &v1 = mVerts[i1], &v2 = mVerts[i2];
				const PxU32 vinds[3] = { i0, i1, i2 };

				if (tRayTest)
				{
					Ps::IntBool overlap;
					if (tInflate)
					{
						// AP: mesh skew is already included here (ray is pre-transformed)
						Vec3V v0v = V3LoadU(v0), v1v = V3LoadU(v1), v2v = V3LoadU(v2);
						Vec3V minB = V3Min(V3Min(v0v, v1v), v2v), maxB = V3Max(V3Max(v0v, v1v), v2v);

						// PT: we add an epsilon to max distance, to make sure we don't reject triangles that are just at the same
						// distance as best triangle so far. We need to keep all of these to make sure we return the one with the
						// best normal.
						const float relativeEpsilon = GU_EPSILON_SAME_DISTANCE * PxMax(1.0f, maxT);
						FloatV tNear, tFar;
						overlap = intersectRayAABB2(
							V3Sub(minB, inflateV), V3Add(maxB, inflateV), rayOriginV, rayDirV, FLoad(maxT+relativeEpsilon), tNear, tFar);
						if (overlap)
						{
							// can't clip to tFar here because hitting the AABB doesn't guarantee that we can clip
							// (since we can still miss the actual tri)
							tempHit.distance = maxT;
							tempHit.faceIndex = triangleIndex;
							tempHit.u = tempHit.v = 0.0f;
						}
					} else
						overlap = rayCollider.overlap(v0, v1, v2, tempHit) && tempHit.distance <= maxT;
					if(!overlap)
						continue;
				}
				tempHit.faceIndex = triangleIndex;
				tempHit.flags = PxHitFlag::ePOSITION;
				// Intersection point is valid if dist < segment's length
				// We know dist>0 so we can use integers
				if (closestMode)
				{
					if(tempHit.distance < closestHit.distance)
					{
						closestHit = tempHit;
						newMaxT = PxMin(tempHit.distance, newMaxT);
						cv0 = v0; cv1 = v1; cv2 = v2;
						cis[0] = vinds[0]; cis[1] = vinds[1]; cis[2] = vinds[2];
						hadClosestHit = true;
					}
				} else
				{
					PxReal shrunkMaxT = newMaxT;
					PxAgain again = outerCallback.processHit(tempHit, v0, v1, v2, shrunkMaxT, vinds);
					if (!again)
						return false;
					if (shrunkMaxT < newMaxT)
					{
						newMaxT = shrunkMaxT;
						maxT = shrunkMaxT;
					}
				}

				if (outerCallback.inAnyMode()) // early out if in ANY mode
					return false;
			}

		} // for(PxU32 leaf = 0; leaf<NumTouched; leaf++)

		return true;
	}

	virtual bool processResults(PxU32 numTouched, PxU32* touched)
	{
		PxF32 dummy;
		return RayRTreeCallback::processResults(numTouched, touched, dummy);
	}


	virtual ~RayRTreeCallback()
	{
		if (hadClosestHit)
		{
			PX_ASSERT(outerCallback.inClosestMode());
			outerCallback.processHit(closestHit, cv0, cv1, cv2, maxT, cis);
		}
	}

private:
	RayRTreeCallback& operator=(const RayRTreeCallback&);
};

#if PX_VC 
     #pragma warning(pop) 
#endif

void MeshRayCollider::collideOBB(
	const Box& obb, bool bothTriangleSidesCollide, const RTreeTriangleMesh* mi, MeshHitCallback<PxRaycastHit>& callback,
	bool checkObbIsAligned)
{
	const PxU32 maxResults = RTREE_N; // maxResults=rtree page size for more efficient early out
	PxU32 buf[maxResults];
	RayRTreeCallback<false, false> rTreeCallback(
		mi->getGeomEpsilon(), callback, mi->has16BitIndices(), mi->getTrianglesFast(), mi->getVerticesFast(),
		PxVec3(0), PxVec3(0), 0.0f, bothTriangleSidesCollide, NULL);
	if (checkObbIsAligned && PxAbs(PxQuat(obb.rot).w) > 0.9999f)
	{
		PxVec3 aabbExtents = obb.computeAABBExtent();
		mi->getRTree().traverseAABB(obb.center - aabbExtents, obb.center + aabbExtents, maxResults, buf, &rTreeCallback);
	} else
		mi->getRTree().traverseOBB(obb, maxResults, buf, &rTreeCallback);
}

template <int tInflate, int tRayTest>
void MeshRayCollider::collide(
	const PxVec3& orig, const PxVec3& dir, PxReal maxT, bool bothSides,
	const RTreeTriangleMesh* mi, MeshHitCallback<PxRaycastHit>& callback,
	const PxVec3* inflate)
{
	const PxU32 maxResults = RTREE_N; // maxResults=rtree page size for more efficient early out
	PxU32 buf[maxResults];
	if (maxT == 0.0f) // AABB traversal path
	{
		RayRTreeCallback<tInflate, false> rTreeCallback(
			mi->getGeomEpsilon(), callback, mi->has16BitIndices(), mi->getTrianglesFast(), mi->getVerticesFast(),
			orig, dir, maxT, bothSides, inflate);
		PxVec3 inflate1 = tInflate ? *inflate : PxVec3(0); // both maxT and inflate can be zero, so need to check tInflate
		mi->getRTree().traverseAABB(orig-inflate1, orig+inflate1, maxResults, buf, &rTreeCallback);
	}
	else // ray traversal path
	{
		RayRTreeCallback<tInflate, tRayTest> rTreeCallback(
			mi->getGeomEpsilon(), callback, mi->has16BitIndices(), mi->getTrianglesFast(), mi->getVerticesFast(),
			orig, dir, maxT, bothSides, inflate);
		mi->getRTree().traverseRay<tInflate>(orig, dir, maxResults, buf, &rTreeCallback, inflate, maxT);
	}
}


#define TINST(a,b) \
template void MeshRayCollider::collide<a,b>( \
	const PxVec3& orig, const PxVec3& dir, PxReal maxT, bool bothSides, const RTreeTriangleMesh* mesh, \
	MeshHitCallback<PxRaycastHit>& callback, const PxVec3* inflate);

TINST(0,0)
TINST(1,0)
TINST(0,1)
TINST(1,1)

#undef TINST

#include "GuRaycastTests.h"
#include "PxTriangleMeshGeometry.h"
#include "GuTriangleMesh.h"
#include "CmScaling.h"

struct RayMeshColliderCallback  : public MeshHitCallback<PxRaycastHit>
{
	PxRaycastHit*		mDstBase;
	PxU32				mHitNum;
	PxU32				mMaxHits;
	const PxMeshScale*	mScale;
	const PxTransform*	mPose;
	const Matrix34*		mWorld2vertexSkew;
	PxU32				mHitFlags;
	const PxVec3&		mRayDir;
	bool				mIsDoubleSided;
	float				mDistCoeff;

	RayMeshColliderCallback(
		CallbackMode::Enum mode_, PxRaycastHit* hits, PxU32 maxHits, const PxMeshScale* scale, const PxTransform* pose,
		const Matrix34* world2vertexSkew, PxU32 hitFlags, const PxVec3& rayDir, bool isDoubleSided, float distCoeff) :
			MeshHitCallback<PxRaycastHit>	(mode_),
			mDstBase						(hits),
			mHitNum							(0),
			mMaxHits						(maxHits),
			mScale							(scale),
			mPose							(pose),
			mWorld2vertexSkew				(world2vertexSkew),
			mHitFlags						(hitFlags),
			mRayDir							(rayDir),
			mIsDoubleSided					(isDoubleSided),
			mDistCoeff						(distCoeff)
	{
	}

	// return false for early out
	virtual bool processHit(
		const PxRaycastHit& lHit, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2, PxReal&, const PxU32*)
	{
		const PxReal u = lHit.u, v = lHit.v;
		const PxVec3 localImpact = (1.0f - u - v)*lp0 + u*lp1 + v*lp2;

		//not worth concatenating to do 1 transform: PxMat34Legacy vertex2worldSkew = scaling.getVertex2WorldSkew(absPose);
		// PT: TODO: revisit this for N hits
		PxRaycastHit hit = lHit;
		hit.position	= mPose->transform(mScale->transform(localImpact));
		hit.flags		= PxHitFlag::ePOSITION|PxHitFlag::eUV|PxHitFlag::eFACE_INDEX;
		hit.normal		= PxVec3(0.0f);
		hit.distance	*= mDistCoeff;

		// Compute additional information if needed
		if(mHitFlags & PxHitFlag::eNORMAL)
		{
			// User requested impact normal
			const PxVec3 localNormal = (lp1 - lp0).cross(lp2 - lp0);

			if(mWorld2vertexSkew)
			{
				hit.normal = mWorld2vertexSkew->rotateTranspose(localNormal);
				if (mScale->hasNegativeDeterminant())
					Ps::swap<PxReal>(hit.u, hit.v); // have to swap the UVs though since they were computed in mesh local space
			}
			else
				hit.normal = hit.normal = mPose->rotate(localNormal);
			hit.normal.normalize();

			// PT: figure out correct normal orientation (DE7458)
			// - if the mesh is single-sided the normal should be the regular triangle normal N, regardless of eMESH_BOTH_SIDES.
			// - if the mesh is double-sided the correct normal can be either N or -N. We take the one opposed to ray direction.
			if(mIsDoubleSided && hit.normal.dot(mRayDir) > 0.0f)
				hit.normal = -hit.normal;

			hit.flags |= PxHitFlag::eNORMAL;
		}

		// PT: no callback => store results in provided buffer
		if(mHitNum == mMaxHits)
			return false;

		mDstBase[mHitNum++] = hit;
		return true;
	}

private:
	RayMeshColliderCallback& operator=(const RayMeshColliderCallback&);
};

PxU32 physx::Gu::raycast_triangleMesh_RTREE(const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
											const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
											PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH33);

	const RTreeTriangleMesh* meshData = static_cast<const RTreeTriangleMesh*>(mesh);

	//scaling: transform the ray to vertex space

	PxVec3 orig, dir;
	Matrix34 world2vertexSkew;
	Matrix34* world2vertexSkewP = NULL;
	PxReal distCoeff = 1.0f;
	if(meshGeom.scale.isIdentity())
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
			distCoeff = 1.0f / distCoeff;
		}
	}

	const bool isDoubleSided = meshGeom.meshFlags.isSet(PxMeshGeometryFlag::eDOUBLE_SIDED);
	const bool bothSides = isDoubleSided || (hitFlags & PxHitFlag::eMESH_BOTH_SIDES);

	RayMeshColliderCallback callback(
		(maxHits > 1) ? CallbackMode::eMULTIPLE : (hitFlags & PxHitFlag::eMESH_ANY ? CallbackMode::eANY : CallbackMode::eCLOSEST),
		hits, maxHits, &meshGeom.scale, &pose, world2vertexSkewP, hitFlags, rayDir, isDoubleSided, distCoeff);

	MeshRayCollider::collide<0, 1>(orig, dir, maxDist, bothSides, static_cast<const RTreeTriangleMesh*>(meshData), callback, NULL);
	return callback.mHitNum;
}


static PX_INLINE void computeSweptAABBAroundOBB(
	const Box& obb, PxVec3& sweepOrigin, PxVec3& sweepExtents, PxVec3& sweepDir, PxReal& sweepLen)
{
	PxU32 other1, other2;
	// largest axis of the OBB is the sweep direction, sum of abs of two other is the swept AABB extents
	PxU32 lai = Ps::largestAxis(obb.extents, other1, other2);
	PxVec3 longestAxis = obb.rot[lai]*obb.extents[lai];
	PxVec3 absOther1 = obb.rot[other1].abs()*obb.extents[other1];
	PxVec3 absOther2 = obb.rot[other2].abs()*obb.extents[other2];
	sweepOrigin = obb.center - longestAxis;
	sweepExtents = absOther1 + absOther2 + PxVec3(GU_MIN_AABB_EXTENT); // see comments for GU_MIN_AABB_EXTENT
	sweepLen = 2.0f; // length is already included in longestAxis
	sweepDir = longestAxis;
}

enum { eSPHERE, eCAPSULE, eBOX }; // values for tSCB

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
	#pragma warning( disable : 4512 ) // assignment operator could not be generated
#endif

namespace
{
struct IntersectShapeVsMeshCallback : MeshHitCallback<PxRaycastHit>
{
	PX_NOCOPY(IntersectShapeVsMeshCallback)
public:
	IntersectShapeVsMeshCallback(const PxMat33& vertexToShapeSkew, LimitedResults* results, bool flipNormal)
		:	MeshHitCallback<PxRaycastHit>(CallbackMode::eMULTIPLE),
			mVertexToShapeSkew	(vertexToShapeSkew),
			mResults			(results),
			mAnyHits			(false),
			mFlipNormal			(flipNormal)
	{
	}
	virtual	~IntersectShapeVsMeshCallback(){}

	const PxMat33&	mVertexToShapeSkew; // vertex to box without translation for boxes
	LimitedResults*	mResults;
	bool			mAnyHits;
	bool			mFlipNormal;

	PX_FORCE_INLINE	bool	recordHit(const PxRaycastHit& aHit, Ps::IntBool hit)
	{
		if(hit)
		{
			mAnyHits = true;
			if(mResults)
				mResults->add(aHit.faceIndex);
			else
				return false; // abort traversal if we are only interested in firstContact (mResults is NULL)
		}
		return true; // if we are here, either no triangles were hit or multiple results are expected => continue traversal
	}
};

template<bool tScaleIsIdentity>
struct IntersectSphereVsMeshCallback : IntersectShapeVsMeshCallback
{
	IntersectSphereVsMeshCallback(const PxMat33& m, LimitedResults* r, bool flipNormal) : IntersectShapeVsMeshCallback(m, r, flipNormal)	{}
	virtual	~IntersectSphereVsMeshCallback(){}
	PxF32					mMinDist2;
	PxVec3					mLocalCenter;	// PT: sphere center in local/mesh space

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxRaycastHit& aHit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal&, const PxU32*)
	{
		const Vec3V v0 = V3LoadU(tScaleIsIdentity ? av0 : mVertexToShapeSkew * av0);
		const Vec3V v1 = V3LoadU(tScaleIsIdentity ? av1 : mVertexToShapeSkew * (mFlipNormal ? av2 : av1));
		const Vec3V v2 = V3LoadU(tScaleIsIdentity ? av2 : mVertexToShapeSkew * (mFlipNormal ? av1 : av2));

		FloatV dummy1, dummy2;
		Vec3V closestP;
		PxReal dist2;
		FStore(distancePointTriangleSquared(V3LoadU(mLocalCenter), v0, v1, v2, dummy1, dummy2, closestP), &dist2);
		return recordHit(aHit, dist2 <= mMinDist2);
	}
};

template<bool tScaleIsIdentity>
struct IntersectCapsuleVsMeshCallback : IntersectShapeVsMeshCallback
{
	IntersectCapsuleVsMeshCallback(const PxMat33& m, LimitedResults* r, bool flipNormal) : IntersectShapeVsMeshCallback(m, r, flipNormal)	{}
	virtual	~IntersectCapsuleVsMeshCallback(){}

	Capsule						mLocalCapsule;		// PT: capsule in mesh/local space
	CapsuleTriangleOverlapData	mParams;

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxRaycastHit& aHit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal&, const PxU32*)
	{
		bool hit;
		if(tScaleIsIdentity)
		{
			const PxVec3 normal = (av0 - av1).cross(av0 - av2);
			hit = intersectCapsuleTriangle(normal, av0, av1, av2, mLocalCapsule, mParams);
		}
		else
		{
			const PxVec3 v0 = mVertexToShapeSkew * av0;
			const PxVec3 v1 = mVertexToShapeSkew * (mFlipNormal ? av2 : av1);
			const PxVec3 v2 = mVertexToShapeSkew * (mFlipNormal ? av1 : av2);
			const PxVec3 normal = (v0 - v1).cross(v0 - v2);
			hit = intersectCapsuleTriangle(normal, v0, v1, v2, mLocalCapsule, mParams);
		}
		return recordHit(aHit, hit);
	}
};

template<bool tScaleIsIdentity>
struct IntersectBoxVsMeshCallback : IntersectShapeVsMeshCallback
{
	IntersectBoxVsMeshCallback(const PxMat33& m, LimitedResults* r, bool flipNormal) : IntersectShapeVsMeshCallback(m, r, flipNormal)	{}
	virtual	~IntersectBoxVsMeshCallback(){}

	Matrix34	mVertexToBox;
	Vec3p		mBoxExtents, mBoxCenter;

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxRaycastHit& aHit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal&, const PxU32*)
	{
		Vec3p v0, v1, v2;
		if(tScaleIsIdentity)
		{
			v0 = mVertexToShapeSkew * av0; // transform from skewed mesh vertex to box space,
			v1 = mVertexToShapeSkew * av1; // this includes inverse skew, inverse mesh shape transform and inverse box basis
			v2 = mVertexToShapeSkew * av2;
		}
		else
		{
			v0 = mVertexToBox.transform(av0);
			v1 = mVertexToBox.transform(mFlipNormal ? av2 : av1);
			v2 = mVertexToBox.transform(mFlipNormal ? av1 : av2);
		}

		// PT: this one is safe because we're using Vec3p for all parameters
		const Ps::IntBool hit = intersectTriangleBox_Unsafe(mBoxCenter, mBoxExtents, v0, v1, v2);
		return recordHit(aHit, hit);
	}
};
}

#if PX_VC 
     #pragma warning(pop) 
#endif

template<int tSCB, bool idtMeshScale>
static bool intersectAnyVsMeshT(
	const Sphere* worldSphere, const Capsule* worldCapsule, const Box* worldOBB,
	const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale,
	LimitedResults* results)
{
	const bool flipNormal = meshScale.hasNegativeDeterminant();
	PxMat33 shapeToVertexSkew, vertexToShapeSkew;
	if (!idtMeshScale && tSCB != eBOX)
	{
		vertexToShapeSkew = meshScale.toMat33();
		shapeToVertexSkew = vertexToShapeSkew.getInverse();
	}

	if (tSCB == eSPHERE)
	{
		IntersectSphereVsMeshCallback<idtMeshScale> callback(vertexToShapeSkew, results, flipNormal);
		// transform sphere center from world to mesh shape space 
		const PxVec3 center = meshTransform.transformInv(worldSphere->center);

		// callback will transform verts
		callback.mLocalCenter = center;
		callback.mMinDist2 = worldSphere->radius*worldSphere->radius;

		PxVec3 sweepOrigin, sweepDir, sweepExtents;
		PxReal sweepLen;
		if (!idtMeshScale)
		{
			// AP: compute a swept AABB around an OBB around a skewed sphere
			// TODO: we could do better than an AABB around OBB actually because we can slice off the corners..
			const Box worldOBB_(worldSphere->center, PxVec3(worldSphere->radius), PxMat33(PxIdentity));
			Box vertexOBB;
			computeVertexSpaceOBB(vertexOBB, worldOBB_, meshTransform, meshScale);
			computeSweptAABBAroundOBB(vertexOBB, sweepOrigin, sweepExtents, sweepDir, sweepLen);
		} else
		{
			sweepOrigin = center;
			sweepDir = PxVec3(1.0f,0,0);
			sweepLen = 0.0f;
			sweepExtents = PxVec3(PxMax(worldSphere->radius, GU_MIN_AABB_EXTENT));
		}

		MeshRayCollider::collide<1, 1>(sweepOrigin, sweepDir, sweepLen, true, static_cast<const RTreeTriangleMesh*>(&triMesh), callback, &sweepExtents);

		return callback.mAnyHits;
	}
	else if (tSCB == eCAPSULE)
	{
		IntersectCapsuleVsMeshCallback<idtMeshScale> callback(vertexToShapeSkew, results, flipNormal);
		const PxF32 radius = worldCapsule->radius;

		// transform world capsule to mesh shape space
		callback.mLocalCapsule.p0		= meshTransform.transformInv(worldCapsule->p0);
		callback.mLocalCapsule.p1		= meshTransform.transformInv(worldCapsule->p1);
		callback.mLocalCapsule.radius	= radius;
		callback.mParams.init(callback.mLocalCapsule);

		if (idtMeshScale)
		{
			// traverse a sweptAABB around the capsule
			const PxVec3 radius3(radius);
			MeshRayCollider::collide<1, 0>(callback.mLocalCapsule.p0, callback.mLocalCapsule.p1-callback.mLocalCapsule.p0, 1.0f, true, static_cast<const RTreeTriangleMesh*>(&triMesh), callback, &radius3);
		}
		else
		{
			// make vertex space OBB
			Box vertexOBB;
			Box worldOBB_;
			worldOBB_.create(*worldCapsule); // AP: potential optimization (meshTransform.inverse is already in callback.mCapsule)
			computeVertexSpaceOBB(vertexOBB, worldOBB_, meshTransform, meshScale);

			MeshRayCollider::collideOBB(vertexOBB, true, static_cast<const RTreeTriangleMesh*>(&triMesh), callback);
		}
		return callback.mAnyHits;
	}
	else if (tSCB == eBOX)
	{
		Box vertexOBB; // query box in vertex space
		if (idtMeshScale)
		{
			// mesh scale is identity - just inverse transform the box without optimization
			vertexOBB = transformBoxOrthonormal(*worldOBB, meshTransform.getInverse());
			// mesh vertices will be transformed from skewed vertex space directly to box AABB space
			// box inverse rotation is baked into the vertexToShapeSkew transform
			// if meshScale is not identity, vertexOBB already effectively includes meshScale transform
			PxVec3 boxCenter;
			getInverse(vertexToShapeSkew, boxCenter, vertexOBB.rot, vertexOBB.center);
			IntersectBoxVsMeshCallback<idtMeshScale> callback(vertexToShapeSkew, results, flipNormal);

			callback.mBoxCenter = -boxCenter;
			callback.mBoxExtents = worldOBB->extents; // extents do not change

			MeshRayCollider::collideOBB(vertexOBB, true, static_cast<const RTreeTriangleMesh*>(&triMesh), callback);

			return callback.mAnyHits;
		} else
		{
			computeVertexSpaceOBB(vertexOBB, *worldOBB, meshTransform, meshScale);

			// mesh scale needs to be included - inverse transform and optimize the box
			const PxMat33 vertexToWorldSkew_Rot = PxMat33Padded(meshTransform.q) * meshScale.toMat33();
			const PxVec3& vertexToWorldSkew_Trans = meshTransform.p;

			Matrix34 tmp;
			buildMatrixFromBox(tmp, *worldOBB);
			const Matrix34 inv = tmp.getInverseRT();
			const Matrix34 _vertexToWorldSkew(vertexToWorldSkew_Rot, vertexToWorldSkew_Trans);

			IntersectBoxVsMeshCallback<idtMeshScale> callback(vertexToShapeSkew, results, flipNormal);
			callback.mVertexToBox = inv * _vertexToWorldSkew;
			callback.mBoxCenter = PxVec3(0.0f);
			callback.mBoxExtents = worldOBB->extents; // extents do not change

			MeshRayCollider::collideOBB(vertexOBB, true, static_cast<const RTreeTriangleMesh*>(&triMesh), callback);

			return callback.mAnyHits;
		}
	}
	else
	{
		PX_ASSERT(0);
		return false;
	}
}

template<int tSCB>
static bool intersectAnyVsMesh(
	const Sphere* worldSphere, const Capsule* worldCapsule, const Box* worldOBB,
	const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale,
	LimitedResults* results)
{
	PX_ASSERT(triMesh.getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH33);
	if (meshScale.isIdentity())
		return intersectAnyVsMeshT<tSCB, true>(worldSphere, worldCapsule, worldOBB, triMesh, meshTransform, meshScale, results);
	else
		return intersectAnyVsMeshT<tSCB, false>(worldSphere, worldCapsule, worldOBB, triMesh, meshTransform, meshScale, results);
}

bool physx::Gu::intersectSphereVsMesh_RTREE(const Sphere& sphere, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	return intersectAnyVsMesh<eSPHERE>(&sphere, NULL, NULL, triMesh, meshTransform, meshScale, results);
}

bool physx::Gu::intersectBoxVsMesh_RTREE(const Box& box, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	return intersectAnyVsMesh<eBOX>(NULL, NULL, &box, triMesh, meshTransform, meshScale, results);
}

bool physx::Gu::intersectCapsuleVsMesh_RTREE(const Capsule& capsule, const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
{
	return intersectAnyVsMesh<eCAPSULE>(NULL, &capsule, NULL, triMesh, meshTransform, meshScale, results);
}

void physx::Gu::intersectOBB_RTREE(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned)
{
	MeshRayCollider::collideOBB(obb, bothTriangleSidesCollide, static_cast<const RTreeTriangleMesh*>(mesh), callback, checkObbIsAligned);
}

// PT: TODO: refactor/share bits of this
bool physx::Gu::sweepCapsule_MeshGeom_RTREE(const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
											const Capsule& lss, const PxVec3& unitDir, const PxReal distance,
											PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH33);
	const RTreeTriangleMesh* meshData = static_cast<const RTreeTriangleMesh*>(mesh);

	const Capsule inflatedCapsule(lss.p0, lss.p1, lss.radius + inflation);

	const bool isIdentity = triMeshGeom.scale.isIdentity();
	bool isDoubleSided = (triMeshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED);
	const PxU32 meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;	

	// compute sweptAABB
	const PxVec3 localP0 = pose.transformInv(inflatedCapsule.p0);
	const PxVec3 localP1 = pose.transformInv(inflatedCapsule.p1);
	PxVec3 sweepOrigin = (localP0+localP1)*0.5f;
	PxVec3 sweepDir = pose.rotateInv(unitDir);
	PxVec3 sweepExtents = PxVec3(inflatedCapsule.radius) + (localP0-localP1).abs()*0.5f;
	PxReal distance1 = distance;
	PxReal distCoeff = 1.0f;
	Matrix34 poseWithScale;
	if(!isIdentity)
	{
		poseWithScale = pose * triMeshGeom.scale;
		distance1 = computeSweepData(triMeshGeom, sweepOrigin, sweepExtents, sweepDir, distance);
		distCoeff = distance1 / distance;
	} else
		poseWithScale = Matrix34(pose);

	SweepCapsuleMeshHitCallback callback(sweepHit, poseWithScale, distance, isDoubleSided, inflatedCapsule, unitDir, hitFlags, triMeshGeom.scale.hasNegativeDeterminant(), distCoeff);

	MeshRayCollider::collide<1, 1>(sweepOrigin, sweepDir, distance1, true, meshData, callback, &sweepExtents);

	if(meshBothSides)
		isDoubleSided = true;

	return callback.finalizeHit(sweepHit, inflatedCapsule, triMeshGeom, pose, isDoubleSided);
}

#include "GuSweepSharedTests.h"

// PT: TODO: refactor/share bits of this
bool physx::Gu::sweepBox_MeshGeom_RTREE(const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
										const Box& box, const PxVec3& unitDir, const PxReal distance,
										PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH33);
	const RTreeTriangleMesh* meshData = static_cast<const RTreeTriangleMesh*>(mesh);

	const bool isIdentity = triMeshGeom.scale.isIdentity();

	const bool meshBothSides = hitFlags & PxHitFlag::eMESH_BOTH_SIDES;
	const bool isDoubleSided = triMeshGeom.meshFlags & PxMeshGeometryFlag::eDOUBLE_SIDED;

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
	const PxTransform boxTransform = box.getTransform();

	const PxVec3 localDir = worldToBox.rotate(unitDir);
	const PxVec3 localDirDist = localDir*distance;
	SweepBoxMeshHitCallback callback( // using eMULTIPLE with shrinkMaxT
		CallbackMode::eMULTIPLE, meshToBox, distance, bothTriangleSidesCollide, box, localDirDist, localDir, unitDir, hitFlags, inflation, triMeshGeom.scale.hasNegativeDeterminant(), distCoeff);

	MeshRayCollider::collide<1, 1>(meshSpaceOrigin, meshSpaceDir/dirLen, dirLen, bothTriangleSidesCollide, meshData, callback, &sweptAABBMeshSpaceExtents);

	return callback.finalizeHit(sweepHit, triMeshGeom, pose, boxTransform, localDir, meshBothSides, isDoubleSided);
}

#include "GuInternal.h"
void physx::Gu::sweepConvex_MeshGeom_RTREE(const TriangleMesh* mesh, const Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool)
{
	PX_ASSERT(mesh->getConcreteType()==PxConcreteType::eTRIANGLE_MESH_BVH33);
	const RTreeTriangleMesh* meshData = static_cast<const RTreeTriangleMesh*>(mesh);

	// create temporal bounds
	Box querySweptBox;
	computeSweptBox(querySweptBox, hullBox.extents, hullBox.center, hullBox.rot, localDir, distance);	

	MeshRayCollider::collideOBB(querySweptBox, true, meshData, callback);
}
