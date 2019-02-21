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

#include "GuSweepTriangleUtils.h"
#include "GuSweepBoxTriangle_FeatureBased.h"
#include "GuSweepBoxTriangle_SAT.h"
#include "GuBV4_BoxOverlap_Internal.h"

// PT: for box-sweeps please refer to %SDKRoot%\InternalDocumentation\GU\Sweep strategies.ppt.
// We use:
// - method 3 if the box is an AABB (SWEEP_AABB_IMPL is defined)
// - method 2 if the box is an OBB (SWEEP_AABB_IMPL is undefined)

#ifdef SWEEP_AABB_IMPL
	// PT: TODO: refactor structure (TA34704)
	struct RayParams
	{
		BV4_ALIGN16(Vec3p	mCenterOrMinCoeff_PaddedAligned);
		BV4_ALIGN16(Vec3p	mExtentsOrMaxCoeff_PaddedAligned);
	#ifndef GU_BV4_USE_SLABS
		BV4_ALIGN16(Vec3p	mData2_PaddedAligned);
		BV4_ALIGN16(Vec3p	mFDir_PaddedAligned);
		BV4_ALIGN16(Vec3p	mData_PaddedAligned);
		BV4_ALIGN16(Vec3p	mLocalDir_PaddedAligned);
	#endif
		BV4_ALIGN16(Vec3p	mOrigin_Padded);		// PT: TODO: this one could be switched to PaddedAligned & V4LoadA (TA34704)
	};

	#include "GuBV4_AABBAABBSweepTest.h"
#else
	#include "GuBV4_BoxBoxOverlapTest.h"
#endif

#include "GuBV4_BoxSweep_Params.h"

static PX_FORCE_INLINE Vec4V multiply3x3V(const Vec4V p, const PxMat33& mat_Padded)
{
	const FloatV xxxV = V4GetX(p);
	const FloatV yyyV = V4GetY(p);
	const FloatV zzzV = V4GetZ(p);

	Vec4V ResV = V4Scale(V4LoadU_Safe(&mat_Padded.column0.x), xxxV);
	ResV = V4Add(ResV, V4Scale(V4LoadU_Safe(&mat_Padded.column1.x), yyyV));
	ResV = V4Add(ResV, V4Scale(V4LoadU_Safe(&mat_Padded.column2.x), zzzV));

	return ResV;
}

// PT: TODO: __fastcall removed to make it compile everywhere. Revisit.
static bool /*__fastcall*/ triBoxSweep(BoxSweepParams* PX_RESTRICT params, PxU32 primIndex, bool nodeSorting=true)
{
	PxU32 VRef0, VRef1, VRef2;
	getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

	const PxVec3& p0 = params->mVerts[VRef0];
	const PxVec3& p1 = params->mVerts[VRef1];
	const PxVec3& p2 = params->mVerts[VRef2];

	// Don't bother doing the actual sweep test if the triangle is too far away
	if(1)
	{
		const float dp0 = p0.dot(params->mLocalDir_Padded);
		const float dp1 = p1.dot(params->mLocalDir_Padded);
		const float dp2 = p2.dot(params->mLocalDir_Padded);

		float TriMin = PxMin(dp0, dp1);
		TriMin = PxMin(TriMin, dp2);

		if(TriMin >= params->mOffset + params->mStabbedFace.mDistance)
			return false;
	}

	TrianglePadded triBoxSpace;
	const Vec4V transModelToBoxV = V4LoadU_Safe(&params->mTModelToBox_Padded.x);
	const Vec4V v0V = V4Add(multiply3x3V(V4LoadU_Safe(&p0.x), params->mRModelToBox_Padded), transModelToBoxV);
	V4StoreU_Safe(v0V, &triBoxSpace.verts[0].x);
	const Vec4V v1V = V4Add(multiply3x3V(V4LoadU_Safe(&p1.x), params->mRModelToBox_Padded), transModelToBoxV);
	V4StoreU_Safe(v1V, &triBoxSpace.verts[1].x);
	const Vec4V v2V = V4Add(multiply3x3V(V4LoadU_Safe(&p2.x), params->mRModelToBox_Padded), transModelToBoxV);
	V4StoreU_Safe(v2V, &triBoxSpace.verts[2].x);

	float Dist;
	if(triBoxSweepTestBoxSpace_inlined(triBoxSpace, params->mOriginalExtents_Padded, params->mOriginalDir_Padded*params->mStabbedFace.mDistance, params->mOneOverDir_Padded, 1.0f, Dist, params->mBackfaceCulling))
	{
		// PT: TODO: these muls & divs may not be needed at all - we just pass the unit dir/inverse dir to the sweep code. Revisit. (TA34704)
		Dist *= params->mStabbedFace.mDistance;
		params->mOneOverDir_Padded = params->mOneOverOriginalDir / Dist;
		params->mStabbedFace.mDistance = Dist;
		params->mStabbedFace.mTriangleID = primIndex;
		// PT: TODO: revisit this (TA34704)
		params->mP0 = triBoxSpace.verts[0];
		params->mP1 = triBoxSpace.verts[1];
		params->mP2 = triBoxSpace.verts[2];
//		V4StoreU_Safe(v0V, &params->mP0.x);
//		V4StoreU_Safe(v1V, &params->mP1.x);
//		V4StoreU_Safe(v2V, &params->mP2.x);

		if(nodeSorting)
		{
#ifdef SWEEP_AABB_IMPL
	#ifndef GU_BV4_USE_SLABS
			setupRayData(params, Dist, params->mOrigin_Padded, params->mLocalDir_PaddedAligned);
	#endif
#else
			params->ShrinkOBB(Dist);
#endif
		}
		return true;
	}
	return false;
}

namespace
{
class LeafFunction_BoxSweepClosest
{
public:
	static PX_FORCE_INLINE void doLeafTest(BoxSweepParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			triBoxSweep(params, primIndex);
			primIndex++;
		}while(nbToGo--);
	}
};

class LeafFunction_BoxSweepAny
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(BoxSweepParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(triBoxSweep(params, primIndex))
				return 1;
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

// PT: TODO: refactor with sphere/capsule versions (TA34704)
static PX_FORCE_INLINE bool computeImpactData(const Box& box, const PxVec3& dir, SweepHit* PX_RESTRICT hit, const BoxSweepParams* PX_RESTRICT params, bool isDoubleSided, bool meshBothSides)
{
	if(params->mStabbedFace.mTriangleID==PX_INVALID_U32)
		return false;	// We didn't touch any triangle

	if(hit)
	{
		const float t = params->mStabbedFace.mDistance;
		hit->mTriangleID = params->mStabbedFace.mTriangleID;
		hit->mDistance = t;

		if(t==0.0f)
		{
			hit->mPos = PxVec3(0.0f);
			hit->mNormal = -dir;
		}
		else
		{
			// PT: TODO: revisit/optimize/use this (TA34704)
			const PxTriangle triInBoxSpace(params->mP0, params->mP1, params->mP2);
			PxHitFlags outFlags = PxHitFlag::Enum(0);
			computeBoxLocalImpact(hit->mPos, hit->mNormal, outFlags, box, params->mOriginalDir_Padded, triInBoxSpace, PxHitFlag::ePOSITION|PxHitFlag::eNORMAL, isDoubleSided, meshBothSides, t);
		}
	}
	return true;
}

template<class ParamsT>
static PX_FORCE_INLINE void setupBoxSweepParams(ParamsT* PX_RESTRICT params, const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree* PX_RESTRICT tree, const SourceMesh* PX_RESTRICT mesh, PxU32 flags)
{
	params->mStabbedFace.mTriangleID = PX_INVALID_U32;
	setupParamsFlags(params, flags);

	setupMeshPointersAndQuantizedCoeffs(params, mesh, tree);

	prepareSweepData(localBox, localDir, maxDist, params);

#ifdef SWEEP_AABB_IMPL
	params->mOrigin_Padded = localBox.center;
	#ifndef GU_BV4_USE_SLABS
	params->mLocalDir_PaddedAligned = localDir;
	setupRayData(params, maxDist, localBox.center, localDir);
	#endif
#endif
}

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#ifdef SWEEP_AABB_IMPL
	#include "GuBV4_ProcessStreamOrdered_SegmentAABB_Inflated.h"
	#include "GuBV4_ProcessStreamNoOrder_SegmentAABB_Inflated.h"
	#ifdef GU_BV4_USE_SLABS
		#include "GuBV4_Slabs_KajiyaNoOrder.h"
		#include "GuBV4_Slabs_KajiyaOrdered.h"
	#endif
#else
	#include "GuBV4_ProcessStreamOrdered_OBBOBB.h"
	#include "GuBV4_ProcessStreamNoOrder_OBBOBB.h"
	#ifdef GU_BV4_USE_SLABS
		#include "GuBV4_Slabs_SwizzledNoOrder.h"
		#include "GuBV4_Slabs_SwizzledOrdered.h"
	#endif
#endif

#ifdef SWEEP_AABB_IMPL
	#define GU_BV4_PROCESS_STREAM_RAY_NO_ORDER
	#define GU_BV4_PROCESS_STREAM_RAY_ORDERED
#else
	#define GU_BV4_PROCESS_STREAM_NO_ORDER
	#define GU_BV4_PROCESS_STREAM_ORDERED
#endif
#include "GuBV4_Internal.h"

#ifdef SWEEP_AABB_IMPL
Ps::IntBool Sweep_AABB_BV4(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags)
#else
Ps::IntBool Sweep_OBB_BV4(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags)
#endif
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	BoxSweepParams Params;
	setupBoxSweepParams(&Params, localBox, localDir, maxDist, &tree, mesh, flags);

	if(tree.mNodes)
	{
#ifdef SWEEP_AABB_IMPL
		if(Params.mEarlyExit)
			processStreamRayNoOrder<1, LeafFunction_BoxSweepAny>(tree, &Params);
		else
			processStreamRayOrdered<1, LeafFunction_BoxSweepClosest>(tree, &Params);
#else
		if(Params.mEarlyExit)
			processStreamNoOrder<LeafFunction_BoxSweepAny>(tree, &Params);
		else
			processStreamOrdered<LeafFunction_BoxSweepClosest>(tree, &Params);
#endif
	}
	else
		doBruteForceTests<LeafFunction_BoxSweepAny, LeafFunction_BoxSweepClosest>(mesh->getNbTriangles(), &Params);

	return computeImpactData(localBox, localDir, hit, &Params, (flags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (flags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
}



// PT: box sweep callback version - currently not used

namespace
{
	struct BoxSweepParamsCB : BoxSweepParams
	{
		// PT: these new members are only here to call computeImpactData during traversal :( 
		// PT: TODO: most of them may not be needed
		Box						mBoxCB;		// Box in original space (maybe not local/mesh space)
		PxVec3					mDirCB;		// Dir in original space (maybe not local/mesh space)
		const PxMat44*			mWorldm_Aligned;
		PxU32					mFlags;

		SweepUnlimitedCallback	mCallback;
		void*					mUserData;
		float					mMaxDist;
		bool					mNodeSorting;
	};

class LeafFunction_BoxSweepCB
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(BoxSweepParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(triBoxSweep(params, primIndex, params->mNodeSorting))
			{
				// PT: TODO: in this version we must compute the impact data immediately,
				// which is a terrible idea in general, but I'm not sure what else I can do.
				SweepHit hit;
				const bool b = computeImpactData(params->mBoxCB, params->mDirCB, &hit, params, (params->mFlags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (params->mFlags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
				PX_ASSERT(b);

				// PT: then replicate part from BV4_BoxSweepSingle:
				if(b && params->mWorldm_Aligned)
				{
					// Move to world space
					// PT: TODO: optimize (TA34704)
					hit.mPos = params->mWorldm_Aligned->transform(hit.mPos);
					hit.mNormal = params->mWorldm_Aligned->rotate(hit.mNormal);
				}

				reportUnlimitedCallbackHit(params, hit);
			}

			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};

}

// PT: for design decisions in this function, refer to the comments of BV4_GenericSweepCB().
// PT: 'worldm_Aligned' is only here to move back results to world space, but input is already in local space.
#ifdef SWEEP_AABB_IMPL
void Sweep_AABB_BV4_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting)
#else
void Sweep_OBB_BV4_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting)
#endif
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	BoxSweepParamsCB Params;
	Params.mBoxCB			= localBox;
	Params.mDirCB			= localDir;
	Params.mWorldm_Aligned	= worldm_Aligned;
	Params.mFlags			= flags;

	Params.mCallback		= callback;
	Params.mUserData		= userData;
	Params.mMaxDist			= maxDist;
	Params.mNodeSorting		= nodeSorting;
	setupBoxSweepParams(&Params, localBox, localDir, maxDist, &tree, mesh, flags);

	PX_ASSERT(!Params.mEarlyExit);

	if(tree.mNodes)
	{
		if(nodeSorting)
		{
#ifdef SWEEP_AABB_IMPL
			processStreamRayOrdered<1, LeafFunction_BoxSweepCB>(tree, &Params);
#else
			processStreamOrdered<LeafFunction_BoxSweepCB>(tree, &Params);
#endif
		}
		else
		{
#ifdef SWEEP_AABB_IMPL
			processStreamRayNoOrder<1, LeafFunction_BoxSweepCB>(tree, &Params);
#else
			processStreamNoOrder<LeafFunction_BoxSweepCB>(tree, &Params);
#endif
		}
	}
	else
		doBruteForceTests<LeafFunction_BoxSweepCB, LeafFunction_BoxSweepCB>(mesh->getNbTriangles(), &Params);
}




// New callback-based box sweeps. Reuses code above, allow early exits. Some init code may be done in vain
// since the leaf tests are not performed (we don't do box-sweeps-vs-tri since the box is only a BV around
// the actual shape, say a convex)

namespace
{
struct GenericSweepParamsCB : BoxSweepParams
{
	MeshSweepCallback	mCallback;
	void*				mUserData;
};

class LeafFunction_BoxSweepClosestCB
{
public:
	static PX_FORCE_INLINE void doLeafTest(GenericSweepParamsCB* PX_RESTRICT params, PxU32 prim_index)
	{
		PxU32 nbToGo = getNbPrimitives(prim_index);
		do
		{
			// PT: in the regular version we'd do a box-vs-triangle sweep test here
			// Instead we just grab the triangle and send it to the callback
			//
			// This can be used for regular "closest hit" sweeps, when the scale is not identity or
			// when the box is just around a more complex shape (e.g. convex). In this case we want
			// the calling code to compute a convex-triangle distance, and then we want to shrink
			// the ray/box while doing an ordered traversal.
			//
			// For "sweep all" or "sweep any" purposes we want to either report all hits or early exit
			// as soon as we find one. There is no need for shrinking or ordered traversals here.

			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, prim_index, params->mTris32, params->mTris16);

			const PxVec3& p0 = params->mVerts[VRef0];
			const PxVec3& p1 = params->mVerts[VRef1];
			const PxVec3& p2 = params->mVerts[VRef2];

			// Don't bother doing the actual sweep test if the triangle is too far away
			const float dp0 = p0.dot(params->mLocalDir_Padded);
			const float dp1 = p1.dot(params->mLocalDir_Padded);
			const float dp2 = p2.dot(params->mLocalDir_Padded);

			float TriMin = PxMin(dp0, dp1);
			TriMin = PxMin(TriMin, dp2);

			if(TriMin < params->mOffset + params->mStabbedFace.mDistance)
			{
//				const PxU32 vrefs[3] = { VRef0, VRef1, VRef2 };
				float Dist = params->mStabbedFace.mDistance;
				if((params->mCallback)(params->mUserData, p0, p1, p2, prim_index, /*vrefs,*/ Dist))
					return;	// PT: TODO: we return here but the ordered path doesn't really support early exits (TA34704)

				if(Dist<params->mStabbedFace.mDistance)
				{
					params->mStabbedFace.mDistance = Dist;
					params->mStabbedFace.mTriangleID = prim_index;
#ifdef SWEEP_AABB_IMPL
	#ifndef GU_BV4_USE_SLABS
					setupRayData(params, Dist, params->mOrigin_Padded, params->mLocalDir_PaddedAligned);
	#endif
#else
					params->ShrinkOBB(Dist);
#endif
				}
			}

			prim_index++;
		}while(nbToGo--);
	}
};

class LeafFunction_BoxSweepAnyCB
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(GenericSweepParamsCB* PX_RESTRICT params, PxU32 prim_index)
	{
		PxU32 nbToGo = getNbPrimitives(prim_index);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, prim_index, params->mTris32, params->mTris16);

			const PxVec3& p0 = params->mVerts[VRef0];
			const PxVec3& p1 = params->mVerts[VRef1];
			const PxVec3& p2 = params->mVerts[VRef2];

			{
//				const PxU32 vrefs[3] = { VRef0, VRef1, VRef2 };
				float Dist = params->mStabbedFace.mDistance;
				if((params->mCallback)(params->mUserData, p0, p1, p2, prim_index, /*vrefs,*/ Dist))
					return 1;
			}

			prim_index++;
		}while(nbToGo--);

		return 0;
	}
};
}

#ifdef SWEEP_AABB_IMPL
void GenericSweep_AABB_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, MeshSweepCallback callback, void* userData, PxU32 flags)
#else
void GenericSweep_OBB_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, MeshSweepCallback callback, void* userData, PxU32 flags)
#endif
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	GenericSweepParamsCB Params;
	Params.mCallback	= callback;
	Params.mUserData	= userData;
	setupBoxSweepParams(&Params, localBox, localDir, maxDist, &tree, mesh, flags);

	if(tree.mNodes)
	{
#ifdef SWEEP_AABB_IMPL
		if(Params.mEarlyExit)
			processStreamRayNoOrder<1, LeafFunction_BoxSweepAnyCB>(tree, &Params);
		else
			processStreamRayOrdered<1, LeafFunction_BoxSweepClosestCB>(tree, &Params);
#else
		if(Params.mEarlyExit)
			processStreamNoOrder<LeafFunction_BoxSweepAnyCB>(tree, &Params);
		else
			processStreamOrdered<LeafFunction_BoxSweepClosestCB>(tree, &Params);
#endif
	}
	else
		doBruteForceTests<LeafFunction_BoxSweepAnyCB, LeafFunction_BoxSweepClosestCB>(mesh->getNbTriangles(), &Params);
}
