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

#include "GuBV4.h"
using namespace physx;
using namespace Gu;

#if PX_INTEL_FAMILY  && !defined(PX_SIMD_DISABLED)

#include "PsVecMath.h"
using namespace physx::shdfnd::aos;

#include "GuBV4_Common.h"
#include "GuSphere.h"
#include "GuDistancePointTriangle.h"
#include "PsVecMath.h"

using namespace physx::shdfnd::aos;

#if PX_VC
#pragma warning ( disable : 4324 )
#endif

// Sphere overlap any

struct SphereParams
{
	const IndTri32*	PX_RESTRICT	mTris32;
	const IndTri16*	PX_RESTRICT	mTris16;
	const PxVec3*	PX_RESTRICT	mVerts;

	BV4_ALIGN16(Vec3p	mCenterOrMinCoeff_PaddedAligned);
	BV4_ALIGN16(Vec3p	mExtentsOrMaxCoeff_PaddedAligned);

	BV4_ALIGN16(PxVec3	mCenter_PaddedAligned);		float	mRadius2;
#ifdef GU_BV4_USE_SLABS
	BV4_ALIGN16(PxVec3	mCenter_PaddedAligned2);	float	mRadius22;
#endif
};

#ifndef GU_BV4_USE_SLABS
#ifndef GU_BV4_QUANTIZED_TREE
// PT: TODO: refactor with bucket pruner code (TA34704)
static PX_FORCE_INLINE Ps::IntBool BV4_SphereAABBOverlap(const PxVec3& center, const PxVec3& extents, const SphereParams* PX_RESTRICT params)
{
	const Vec4V mCenter = V4LoadA_Safe(&params->mCenter_PaddedAligned.x);
	const FloatV mRadius2 = FLoad(params->mRadius2);

	const Vec4V boxCenter = V4LoadU(&center.x);
	const Vec4V boxExtents = V4LoadU(&extents.x);

	const Vec4V offset = V4Sub(mCenter, boxCenter);
	const Vec4V closest = V4Clamp(offset, V4Neg(boxExtents), boxExtents);
	const Vec4V d = V4Sub(offset, closest);
	
	const PxU32 test = (PxU32)_mm_movemask_ps(FIsGrtrOrEq(mRadius2, V4Dot3(d, d)));
	return (test & 0x7) == 0x7;
}
#endif
#endif

static PX_FORCE_INLINE Ps::IntBool __SphereTriangle(const SphereParams* PX_RESTRICT params, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2)
{
	{
		const float sqrDist = (p0 - params->mCenter_PaddedAligned).magnitudeSquared();
		if(sqrDist <= params->mRadius2)
			return 1;
	}

	const PxVec3 edge10 = p1 - p0;
	const PxVec3 edge20 = p2 - p0;
	const PxVec3 cp = closestPtPointTriangle2(params->mCenter_PaddedAligned, p0, p1, p2, edge10, edge20);
	const float sqrDist = (cp - params->mCenter_PaddedAligned).magnitudeSquared();
	return sqrDist <= params->mRadius2;
}

// PT: TODO: evaluate if SIMD distance function would be faster here (TA34704)
// PT: TODO: __fastcall removed to make it compile everywhere. Revisit.
static /*PX_FORCE_INLINE*/ Ps::IntBool /*__fastcall*/ __SphereTriangle(const SphereParams* PX_RESTRICT params, PxU32 primIndex)
{
	PxU32 VRef0, VRef1, VRef2;
	getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

	return __SphereTriangle(params, params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2]);
}

namespace
{
class LeafFunction_SphereOverlapAny
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const SphereParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(__SphereTriangle(params, primIndex))
				return 1;
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

template<class ParamsT>
static PX_FORCE_INLINE void setupSphereParams(ParamsT* PX_RESTRICT params, const Sphere& sphere, const BV4Tree* PX_RESTRICT tree, const PxMat44* PX_RESTRICT worldm_Aligned, const SourceMesh* PX_RESTRICT mesh)
{
	computeLocalSphere(params->mRadius2, params->mCenter_PaddedAligned, sphere, worldm_Aligned);

#ifdef GU_BV4_USE_SLABS
	params->mCenter_PaddedAligned2 = params->mCenter_PaddedAligned*2.0f;
	params->mRadius22 = params->mRadius2*4.0f;
#endif

	setupMeshPointersAndQuantizedCoeffs(params, mesh, tree);
}

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"

	static PX_FORCE_INLINE Ps::IntBool BV4_SphereAABBOverlap(const Vec4V boxCenter, const Vec4V boxExtents, const SphereParams* PX_RESTRICT params)
	{
		const Vec4V mCenter = V4LoadA_Safe(&params->mCenter_PaddedAligned2.x);
		const FloatV mRadius2 = FLoad(params->mRadius22);

		const Vec4V offset = V4Sub(mCenter, boxCenter);
		const Vec4V closest = V4Clamp(offset, V4Neg(boxExtents), boxExtents);
		const Vec4V d = V4Sub(offset, closest);

		const PxU32 test = PxU32(_mm_movemask_ps(FIsGrtrOrEq(mRadius2, V4Dot3(d, d))));
		return (test & 0x7) == 0x7;
	}
#else
	#ifdef GU_BV4_QUANTIZED_TREE
	static PX_FORCE_INLINE Ps::IntBool BV4_SphereAABBOverlap(const BVDataPacked* PX_RESTRICT node, const SphereParams* PX_RESTRICT params)
	{
		const __m128i testV = _mm_load_si128((__m128i*)node->mAABB.mData);
		const __m128i qextentsV = _mm_and_si128(testV, _mm_set1_epi32(0x0000ffff));
		const __m128i qcenterV = _mm_srai_epi32(testV, 16);
		const Vec4V boxCenter = V4Mul(_mm_cvtepi32_ps(qcenterV), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
		const Vec4V boxExtents = V4Mul(_mm_cvtepi32_ps(qextentsV), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));

		const Vec4V mCenter = V4LoadA_Safe(&params->mCenter_PaddedAligned.x);
		const FloatV mRadius2 = FLoad(params->mRadius2);

		const Vec4V offset = V4Sub(mCenter, boxCenter);
		const Vec4V closest = V4Clamp(offset, V4Neg(boxExtents), boxExtents);
		const Vec4V d = V4Sub(offset, closest);

		const PxU32 test = (PxU32)_mm_movemask_ps(FIsGrtrOrEq(mRadius2, V4Dot3(d, d)));
		return (test & 0x7) == 0x7;
	}
	#endif
#endif

#include "GuBV4_ProcessStreamNoOrder_SphereAABB.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_SwizzledNoOrder.h"
#endif

#define GU_BV4_PROCESS_STREAM_NO_ORDER
#include "GuBV4_Internal.h"

Ps::IntBool BV4_OverlapSphereAny(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	SphereParams Params;
	setupSphereParams(&Params, sphere, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		return processStreamNoOrder<LeafFunction_SphereOverlapAny>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		return LeafFunction_SphereOverlapAny::doLeafTest(&Params, nbTris);
	}
}

// Sphere overlap all

struct SphereParamsAll : SphereParams
{
	PxU32	mNbHits;
	PxU32	mMaxNbHits;
	PxU32*	mHits;
};

namespace
{
class LeafFunction_SphereOverlapAll
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(SphereParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(__SphereTriangle(params, primIndex))
			{
				SphereParamsAll* ParamsAll = static_cast<SphereParamsAll*>(params);
				if(ParamsAll->mNbHits==ParamsAll->mMaxNbHits)
					return 1;
				ParamsAll->mHits[ParamsAll->mNbHits] = primIndex;
				ParamsAll->mNbHits++;
			}
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

PxU32 BV4_OverlapSphereAll(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	SphereParamsAll Params;
	Params.mNbHits		= 0;
	Params.mMaxNbHits	= size;
	Params.mHits		= results;

	setupSphereParams(&Params, sphere, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		overflow = processStreamNoOrder<LeafFunction_SphereOverlapAll>(tree, &Params)!=0;
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		overflow = LeafFunction_SphereOverlapAll::doLeafTest(&Params, nbTris)!=0;
	}
	return Params.mNbHits;
}


// Sphere overlap - callback version

struct SphereParamsCB : SphereParams
{
	MeshOverlapCallback	mCallback;
	void*				mUserData;
};

namespace
{
class LeafFunction_SphereOverlapCB
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const SphereParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			const PxVec3& p0 = params->mVerts[VRef0];
			const PxVec3& p1 = params->mVerts[VRef1];
			const PxVec3& p2 = params->mVerts[VRef2];

			if(__SphereTriangle(params, p0, p1, p2))
			{
				const PxU32 vrefs[3] = { VRef0, VRef1, VRef2 };
				if((params->mCallback)(params->mUserData, p0, p1, p2, primIndex, vrefs))
					return 1;
			}
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

// PT: this one is currently not used
void BV4_OverlapSphereCB(const Sphere& sphere, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, MeshOverlapCallback callback, void* userData)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	SphereParamsCB Params;
	Params.mCallback	= callback;
	Params.mUserData	= userData;
	setupSphereParams(&Params, sphere, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		processStreamNoOrder<LeafFunction_SphereOverlapCB>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		LeafFunction_SphereOverlapCB::doLeafTest(&Params, nbTris);
	}
}

#endif
