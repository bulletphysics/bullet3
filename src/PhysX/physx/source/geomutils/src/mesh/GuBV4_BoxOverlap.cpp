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

#if PX_INTEL_FAMILY  && !defined(PX_SIMD_DISABLED)

#include "PsVecMath.h"
using namespace physx::shdfnd::aos;

#include "GuInternal.h"
#include "GuDistancePointSegment.h"
#include "GuIntersectionCapsuleTriangle.h"
#include "GuIntersectionTriangleBox.h"

#include "GuBV4_BoxOverlap_Internal.h"
#include "GuBV4_BoxBoxOverlapTest.h"

// Box overlap any

struct OBBParams : OBBTestParams
{
	const IndTri32*	PX_RESTRICT	mTris32;
	const IndTri16*	PX_RESTRICT	mTris16;
	const PxVec3*	PX_RESTRICT	mVerts;

	PxMat33			mRModelToBox_Padded;	//!< Rotation from model space to obb space
	Vec3p			mTModelToBox_Padded;	//!< Translation from model space to obb space
};

// PT: TODO: this used to be inlined so we lost some perf by moving to PhysX's version. Revisit. (TA34704)
Ps::IntBool intersectTriangleBoxBV4(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2,
									const PxMat33& rotModelToBox, const PxVec3& transModelToBox, const PxVec3& extents);
namespace
{
class LeafFunction_BoxOverlapAny
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const OBBParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			if(intersectTriangleBoxBV4(params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params->mRModelToBox_Padded, params->mTModelToBox_Padded, params->mBoxExtents_PaddedAligned))
				return 1;
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

template<class ParamsT>
static PX_FORCE_INLINE void setupBoxParams(ParamsT* PX_RESTRICT params, const Box& localBox, const BV4Tree* PX_RESTRICT tree, const SourceMesh* PX_RESTRICT mesh)
{
	invertBoxMatrix(params->mRModelToBox_Padded, params->mTModelToBox_Padded, localBox);
	params->mTBoxToModel_PaddedAligned = localBox.center;

	setupMeshPointersAndQuantizedCoeffs(params, mesh, tree);

	params->precomputeBoxData(localBox.extents, &localBox.rot);
}

///////////////////////////////////////////////////////////////////////////////

#include "GuBV4_BoxBoxOverlapTest.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamNoOrder_OBBOBB.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_SwizzledNoOrder.h"
#endif

#define GU_BV4_PROCESS_STREAM_NO_ORDER
#include "GuBV4_Internal.h"

Ps::IntBool BV4_OverlapBoxAny(const Box& box, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	Box localBox;
	computeLocalBox(localBox, box, worldm_Aligned);

	OBBParams Params;
	setupBoxParams(&Params, localBox, &tree, mesh);

	if(tree.mNodes)
		return processStreamNoOrder<LeafFunction_BoxOverlapAny>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		return LeafFunction_BoxOverlapAny::doLeafTest(&Params, nbTris);
	}
}


// Box overlap all

struct OBBParamsAll : OBBParams
{
	PxU32	mNbHits;
	PxU32	mMaxNbHits;
	PxU32*	mHits;
};

namespace
{
class LeafFunction_BoxOverlapAll
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(OBBParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			if(intersectTriangleBoxBV4(params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params->mRModelToBox_Padded, params->mTModelToBox_Padded, params->mBoxExtents_PaddedAligned))
			{
				OBBParamsAll* ParamsAll = static_cast<OBBParamsAll*>(params);
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

PxU32 BV4_OverlapBoxAll(const Box& box, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	Box localBox;
	computeLocalBox(localBox, box, worldm_Aligned);

	OBBParamsAll Params;
	Params.mNbHits		= 0;
	Params.mMaxNbHits	= size;
	Params.mHits		= results;
	setupBoxParams(&Params, localBox, &tree, mesh);

	if(tree.mNodes)
		overflow = processStreamNoOrder<LeafFunction_BoxOverlapAll>(tree, &Params)!=0;
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		overflow = LeafFunction_BoxOverlapAll::doLeafTest(&Params, nbTris)!=0;
	}
	return Params.mNbHits;
}

// Box overlap - callback version

struct OBBParamsCB : OBBParams
{
	MeshOverlapCallback	mCallback;
	void*				mUserData;
};

namespace
{
class LeafFunction_BoxOverlapCB
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const OBBParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			if(intersectTriangleBoxBV4(params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params->mRModelToBox_Padded, params->mTModelToBox_Padded, params->mBoxExtents_PaddedAligned))
			{
				const PxU32 vrefs[3] = { VRef0, VRef1, VRef2 };
				if((params->mCallback)(params->mUserData, params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], primIndex, vrefs))
					return 1;
			}
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

void BV4_OverlapBoxCB(const Box& localBox, const BV4Tree& tree, MeshOverlapCallback callback, void* userData)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	OBBParamsCB Params;
	Params.mCallback	= callback;
	Params.mUserData	= userData;
	setupBoxParams(&Params, localBox, &tree, mesh);

	if(tree.mNodes)
		processStreamNoOrder<LeafFunction_BoxOverlapCB>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		LeafFunction_BoxOverlapCB::doLeafTest(&Params, nbTris);
	}
}

// Capsule overlap any

struct CapsuleParamsAny : OBBParams
{
	Capsule						mLocalCapsule;	// Capsule in mesh space
	CapsuleTriangleOverlapData	mData;
};

// PT: TODO: try to refactor this one with the PhysX version (TA34704)
static bool CapsuleVsTriangle_SAT(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const CapsuleParamsAny* PX_RESTRICT params)
{
//	PX_ASSERT(capsule.p0!=capsule.p1);

	{
		const PxReal d2 = distancePointSegmentSquaredInternal(params->mLocalCapsule.p0, params->mData.mCapsuleDir, p0);
		if(d2<=params->mLocalCapsule.radius*params->mLocalCapsule.radius)
			return 1;
	}

	const PxVec3 N = (p0 - p1).cross(p0 - p2);

	if(!testAxis(p0, p1, p2, params->mLocalCapsule, N))
		return 0;

	const float BDotB = params->mData.mBDotB;
	const float oneOverBDotB = params->mData.mOneOverBDotB;
	const PxVec3& capP0 = params->mLocalCapsule.p0;
	const PxVec3& capDir = params->mData.mCapsuleDir;

	if(!testAxis(p0, p1, p2, params->mLocalCapsule, computeEdgeAxis(p0, p1 - p0, capP0, capDir, BDotB, oneOverBDotB)))
		return 0;

	if(!testAxis(p0, p1, p2, params->mLocalCapsule, computeEdgeAxis(p1, p2 - p1, capP0, capDir, BDotB, oneOverBDotB)))
		return 0;

	if(!testAxis(p0, p1, p2, params->mLocalCapsule, computeEdgeAxis(p2, p0 - p2, capP0, capDir, BDotB, oneOverBDotB)))
		return 0;

	return 1;
}

static Ps::IntBool PX_FORCE_INLINE __CapsuleTriangle(const CapsuleParamsAny* PX_RESTRICT params, PxU32 primIndex)
{
	PxU32 VRef0, VRef1, VRef2;
	getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);
	return CapsuleVsTriangle_SAT(params->mVerts[VRef0], params->mVerts[VRef1], params->mVerts[VRef2], params);
}

namespace
{
class LeafFunction_CapsuleOverlapAny
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const OBBParams* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(__CapsuleTriangle(static_cast<const CapsuleParamsAny*>(params), primIndex))
				return 1;
			primIndex++;
		}while(nbToGo--);

		return 0;
	}
};
}

template<class ParamsT>
static PX_FORCE_INLINE void setupCapsuleParams(ParamsT* PX_RESTRICT params, const Capsule& capsule, const BV4Tree* PX_RESTRICT tree, const PxMat44* PX_RESTRICT worldm_Aligned, const SourceMesh* PX_RESTRICT mesh)
{
	computeLocalCapsule(params->mLocalCapsule, capsule, worldm_Aligned);

	params->mData.init(params->mLocalCapsule);

	Box localBox;
	computeBoxAroundCapsule(params->mLocalCapsule, localBox);

	setupBoxParams(params, localBox, tree, mesh);
}

Ps::IntBool BV4_OverlapCapsuleAny(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleParamsAny Params;
	setupCapsuleParams(&Params, capsule, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		return processStreamNoOrder<LeafFunction_CapsuleOverlapAny>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		return LeafFunction_CapsuleOverlapAny::doLeafTest(&Params, nbTris);
	}
}


// Capsule overlap all

struct CapsuleParamsAll : CapsuleParamsAny
{
	PxU32	mNbHits;
	PxU32	mMaxNbHits;
	PxU32*	mHits;
};

namespace
{
class LeafFunction_CapsuleOverlapAll
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(OBBParams* PX_RESTRICT params, PxU32 primIndex)
	{
		CapsuleParamsAll* ParamsAll = static_cast<CapsuleParamsAll*>(params);

		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			if(__CapsuleTriangle(ParamsAll, primIndex))
			{
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

PxU32 BV4_OverlapCapsuleAll(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, PxU32* results, PxU32 size, bool& overflow)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleParamsAll Params;
	Params.mNbHits		= 0;
	Params.mMaxNbHits	= size;
	Params.mHits		= results;
	setupCapsuleParams(&Params, capsule, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		overflow = processStreamNoOrder<LeafFunction_CapsuleOverlapAll>(tree, &Params)!=0;
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		overflow = LeafFunction_CapsuleOverlapAll::doLeafTest(&Params, nbTris)!=0;
	}
	return Params.mNbHits;
}

// Capsule overlap - callback version

struct CapsuleParamsCB : CapsuleParamsAny
{
	MeshOverlapCallback	mCallback;
	void*				mUserData;
};

namespace
{
class LeafFunction_CapsuleOverlapCB
{
public:
	static PX_FORCE_INLINE Ps::IntBool doLeafTest(const CapsuleParamsCB* PX_RESTRICT params, PxU32 primIndex)
	{
		PxU32 nbToGo = getNbPrimitives(primIndex);
		do
		{
			PxU32 VRef0, VRef1, VRef2;
			getVertexReferences(VRef0, VRef1, VRef2, primIndex, params->mTris32, params->mTris16);

			const PxVec3& p0 = params->mVerts[VRef0];
			const PxVec3& p1 = params->mVerts[VRef1];
			const PxVec3& p2 = params->mVerts[VRef2];

			if(CapsuleVsTriangle_SAT(p0, p1, p2, params))
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
void BV4_OverlapCapsuleCB(const Capsule& capsule, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, MeshOverlapCallback callback, void* userData)
{
	const SourceMesh* PX_RESTRICT mesh = tree.mMeshInterface;

	CapsuleParamsCB Params;
	Params.mCallback	= callback;
	Params.mUserData	= userData;
	setupCapsuleParams(&Params, capsule, &tree, worldm_Aligned, mesh);

	if(tree.mNodes)
		processStreamNoOrder<LeafFunction_CapsuleOverlapCB>(tree, &Params);
	else
	{
		const PxU32 nbTris = mesh->getNbTriangles();
		PX_ASSERT(nbTris<16);
		LeafFunction_CapsuleOverlapCB::doLeafTest(&Params, nbTris);
	}
}

#endif
