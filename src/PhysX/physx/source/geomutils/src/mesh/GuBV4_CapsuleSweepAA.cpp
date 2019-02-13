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
#include "GuSweepSphereTriangle.h"
using namespace physx;
using namespace Gu;

#if PX_INTEL_FAMILY  && !defined(PX_SIMD_DISABLED)

#include "PsVecMath.h"
using namespace physx::shdfnd::aos;

#include "GuBV4_Common.h"
#include "GuInternal.h"

#define SWEEP_AABB_IMPL

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

#include "GuBV4_BoxSweep_Params.h"

namespace
{
	struct CapsuleSweepParams : BoxSweepParams
	{
		Capsule	mLocalCapsule;
		PxVec3	mCapsuleCenter;
		PxVec3	mExtrusionDir;
		float	mBestAlignmentValue;
		float	mBestDistance;
		float	mMaxDist;
	};
}

#include "GuBV4_CapsuleSweep_Internal.h"
#include "GuBV4_AABBAABBSweepTest.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamOrdered_SegmentAABB_Inflated.h"
#include "GuBV4_ProcessStreamNoOrder_SegmentAABB_Inflated.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_KajiyaNoOrder.h"
	#include "GuBV4_Slabs_KajiyaOrdered.h"
#endif

#define GU_BV4_PROCESS_STREAM_RAY_NO_ORDER
#define GU_BV4_PROCESS_STREAM_RAY_ORDERED
#include "GuBV4_Internal.h"

Ps::IntBool BV4_CapsuleSweepSingleAA(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleSweepParams Params;
	setupCapsuleParams(&Params, capsule, dir, maxDist, &tree, mesh, flags);

	if(tree.mNodes)
	{
		if(Params.mEarlyExit)
			processStreamRayNoOrder<1, LeafFunction_CapsuleSweepAny>(tree, &Params);
		else
			processStreamRayOrdered<1, LeafFunction_CapsuleSweepClosest>(tree, &Params);
	}
	else
		doBruteForceTests<LeafFunction_CapsuleSweepAny, LeafFunction_CapsuleSweepClosest>(mesh->getNbTriangles(), &Params);

	return computeImpactDataT<ImpactFunctionCapsule>(capsule, dir, hit, &Params, NULL, (flags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (flags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
}

#endif
