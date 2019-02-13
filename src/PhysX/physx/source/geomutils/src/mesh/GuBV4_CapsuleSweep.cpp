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

#include "GuSIMDHelpers.h"
#include "GuInternal.h"

#include "GuBV4_BoxOverlap_Internal.h"
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
#include "GuBV4_BoxBoxOverlapTest.h"

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamOrdered_OBBOBB.h"
#include "GuBV4_ProcessStreamNoOrder_OBBOBB.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_SwizzledNoOrder.h"
	#include "GuBV4_Slabs_SwizzledOrdered.h"
#endif

#define GU_BV4_PROCESS_STREAM_NO_ORDER
#define GU_BV4_PROCESS_STREAM_ORDERED
#include "GuBV4_Internal.h"

Ps::IntBool BV4_CapsuleSweepSingle(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleSweepParams Params;
	setupCapsuleParams(&Params, capsule, dir, maxDist, &tree, mesh, flags);

	if(tree.mNodes)
	{
		if(Params.mEarlyExit)
			processStreamNoOrder<LeafFunction_CapsuleSweepAny>(tree, &Params);
		else
			processStreamOrdered<LeafFunction_CapsuleSweepClosest>(tree, &Params);
	}
	else
		doBruteForceTests<LeafFunction_CapsuleSweepAny, LeafFunction_CapsuleSweepClosest>(mesh->getNbTriangles(), &Params);

	return computeImpactDataT<ImpactFunctionCapsule>(capsule, dir, hit, &Params, NULL, (flags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (flags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
}

// PT: capsule sweep callback version - currently not used

namespace
{
	struct CapsuleSweepParamsCB : CapsuleSweepParams
	{
		// PT: these new members are only here to call computeImpactDataT during traversal :( 
		// PT: TODO: most of them may not be needed
		// PT: TODO: for example mCapsuleCB probably dup of mLocalCapsule
		Capsule					mCapsuleCB;		// Capsule in original space (maybe not local/mesh space)
		PxVec3					mDirCB;			// Dir in original space (maybe not local/mesh space)
		const PxMat44*			mWorldm_Aligned;
		PxU32					mFlags;

		SweepUnlimitedCallback	mCallback;
		void*					mUserData;
		bool					mNodeSorting;
	};

class LeafFunction_CapsuleSweepCB
{
public:

	static PX_FORCE_INLINE Ps::IntBool doLeafTest(CapsuleSweepParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(triCapsuleSweep(params, primIndex, params->mNodeSorting))
			{
				// PT: TODO: in this version we must compute the impact data immediately,
				// which is a terrible idea in general, but I'm not sure what else I can do.
				SweepHit hit;
				const bool b = computeImpactDataT<ImpactFunctionCapsule>(params->mCapsuleCB, params->mDirCB, &hit, params, params->mWorldm_Aligned, (params->mFlags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (params->mFlags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
				PX_ASSERT(b);
				PX_UNUSED(b);

				reportUnlimitedCallbackHit(params, hit);
			}
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

// PT: for design decisions in this function, refer to the comments of BV4_GenericSweepCB().
void BV4_CapsuleSweepCB(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleSweepParamsCB Params;
	Params.mCapsuleCB		= capsule;
	Params.mDirCB			= dir;
	Params.mWorldm_Aligned	= worldm_Aligned;
	Params.mFlags			= flags;

	Params.mCallback		= callback;
	Params.mUserData		= userData;
	Params.mMaxDist			= maxDist;
	Params.mNodeSorting		= nodeSorting;
	setupCapsuleParams(&Params, capsule, dir, maxDist, &tree, mesh, flags);

	PX_ASSERT(!Params.mEarlyExit);

	if(tree.mNodes)
	{
		if(nodeSorting)
			processStreamOrdered<LeafFunction_CapsuleSweepCB>(tree, &Params);
		else
			processStreamNoOrder<LeafFunction_CapsuleSweepCB>(tree, &Params);
	}
	else
		doBruteForceTests<LeafFunction_CapsuleSweepCB, LeafFunction_CapsuleSweepCB>(mesh->getNbTriangles(), &Params);
}

#endif
