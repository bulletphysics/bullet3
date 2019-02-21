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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef GU_BV4_COMMON_H
#define GU_BV4_COMMON_H

#include "foundation/PxMat44.h"
#include "GuBox.h"
#include "GuSphere.h"
#include "GuCapsule.h"
#include "GuSIMDHelpers.h"

#define BV4_ALIGN16(x)	PX_ALIGN_PREFIX(16)	x PX_ALIGN_SUFFIX(16)

namespace physx
{
namespace Gu
{
	enum QueryModifierFlag
	{
		QUERY_MODIFIER_ANY_HIT			= (1<<0),
		QUERY_MODIFIER_DOUBLE_SIDED		= (1<<1),
		QUERY_MODIFIER_MESH_BOTH_SIDES	= (1<<2)
	};

	template<class ParamsT>
	PX_FORCE_INLINE void setupParamsFlags(ParamsT* PX_RESTRICT params, PxU32 flags)
	{
		params->mBackfaceCulling	= (flags & (QUERY_MODIFIER_DOUBLE_SIDED|QUERY_MODIFIER_MESH_BOTH_SIDES)) ? 0 : 1u;
		params->mEarlyExit			= flags & QUERY_MODIFIER_ANY_HIT;
	}

	enum HitCode
	{
		HIT_NONE		= 0,	//!< No hit
		HIT_CONTINUE	= 1,	//!< Hit found, but keep looking for closer one
		HIT_EXIT		= 2		//!< Hit found, you can early-exit (raycast any)
	};

	class RaycastHitInternal : public physx::shdfnd::UserAllocated
	{
		public:
		PX_FORCE_INLINE		RaycastHitInternal()	{}
		PX_FORCE_INLINE		~RaycastHitInternal()	{}

				float		mDistance;
				PxU32		mTriangleID;
	};

	class SweepHit : public physx::shdfnd::UserAllocated
	{
		public:
		PX_FORCE_INLINE		SweepHit()		{}
		PX_FORCE_INLINE		~SweepHit()		{}

				PxU32		mTriangleID;	//!< Index of touched face
				float		mDistance;		//!< Impact distance

				PxVec3		mPos;
				PxVec3		mNormal;
	};

	typedef		HitCode		(*MeshRayCallback)			(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, float dist, float u, float v);
	typedef		bool		(*MeshOverlapCallback)		(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, const PxU32* vertexIndices);
	typedef		bool		(*MeshSweepCallback)		(void* userData, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxU32 triangleIndex, /*const PxU32* vertexIndices,*/ float& dist);
	typedef		bool		(*SweepUnlimitedCallback)	(void* userData, const SweepHit& hit);

	template<class ParamsT>
	PX_FORCE_INLINE	void reportUnlimitedCallbackHit(ParamsT* PX_RESTRICT params, const SweepHit& hit)
	{
		// PT: we can't reuse the MeshSweepCallback here since it's designed for doing the sweep test inside the callback
		// (in the user's code) rather than inside the traversal code. So we use the SweepUnlimitedCallback instead to
		// report the already fully computed hit to users.
		// PT: TODO: this may not be very efficient, since computing the full hit is expensive. If we use this codepath
		// to implement the Epic Tweak, the resulting code will not be optimal.
		(params->mCallback)(params->mUserData, hit);

		// PT: the existing traversal code already shrunk the ray. For real "sweep all" calls we must undo that by reseting the max dist.
		// (params->mStabbedFace.mDistance is used in computeImpactDataX code, so we need it before that point - we can't simply avoid
		// modifying this value before this point).
		if(!params->mNodeSorting)
			params->mStabbedFace.mDistance = params->mMaxDist;
	}

	PX_FORCE_INLINE void invertPRMatrix(PxMat44* PX_RESTRICT dest, const PxMat44* PX_RESTRICT src)
	{
		const float m30 = src->column3.x;
		const float m31 = src->column3.y;
		const float m32 = src->column3.z;

		const float m00 = src->column0.x;
		const float m01 = src->column0.y;
		const float m02 = src->column0.z;

		dest->column0.x = m00;
		dest->column1.x = m01;
		dest->column2.x = m02;
		dest->column3.x = -(m30*m00 + m31*m01 + m32*m02);

		const float m10 = src->column1.x;
		const float m11 = src->column1.y;
		const float m12 = src->column1.z;

		dest->column0.y = m10;
		dest->column1.y = m11;
		dest->column2.y = m12;
		dest->column3.y = -(m30*m10 + m31*m11 + m32*m12);

		const float m20 = src->column2.x;
		const float m21 = src->column2.y;
		const float m22 = src->column2.z;

		dest->column0.z = m20;
		dest->column1.z = m21;
		dest->column2.z = m22;
		dest->column3.z = -(m30*m20 + m31*m21 + m32*m22);

		dest->column0.w = 0.0f;
		dest->column1.w = 0.0f;
		dest->column2.w = 0.0f;
		dest->column3.w = 1.0f;
	}

	PX_FORCE_INLINE void invertBoxMatrix(PxMat33& m, PxVec3& t, const Gu::Box& box)
	{
		const float m30 = box.center.x;
		const float m31 = box.center.y;
		const float m32 = box.center.z;

		const float m00 = box.rot.column0.x;
		const float m01 = box.rot.column0.y;
		const float m02 = box.rot.column0.z;

		m.column0.x = m00;
		m.column1.x = m01;
		m.column2.x = m02;
		t.x = -(m30*m00 + m31*m01 + m32*m02);

		const float m10 = box.rot.column1.x;
		const float m11 = box.rot.column1.y;
		const float m12 = box.rot.column1.z;

		m.column0.y = m10;
		m.column1.y = m11;
		m.column2.y = m12;
		t.y = -(m30*m10 + m31*m11 + m32*m12);

		const float m20 = box.rot.column2.x;
		const float m21 = box.rot.column2.y;
		const float m22 = box.rot.column2.z;

		m.column0.z = m20;
		m.column1.z = m21;
		m.column2.z = m22;
		t.z = -(m30*m20 + m31*m21 + m32*m22);
	}

#ifdef GU_BV4_USE_SLABS
	// PT: this class moved here to make things compile with pedantic compilers.

	// PT: now duplicated because not easy to do otherwise

	struct BVDataSwizzledQ : public physx::shdfnd::UserAllocated
	{
		struct Data
		{
			PxI16	mMin;	//!< Quantized min
			PxI16	mMax;	//!< Quantized max
		};

		Data		mX[4];
		Data		mY[4];
		Data		mZ[4];

		PxU32		mData[4];	

		PX_FORCE_INLINE	PxU32	isLeaf(PxU32 i)				const	{ return mData[i]&1;								}
		PX_FORCE_INLINE	PxU32	getPrimitive(PxU32 i)		const	{ return mData[i]>>1;								}
		PX_FORCE_INLINE	PxU32	getChildOffset(PxU32 i)		const	{ return mData[i]>>GU_BV4_CHILD_OFFSET_SHIFT_COUNT;	}
		PX_FORCE_INLINE	PxU32	getChildType(PxU32 i)		const	{ return (mData[i]>>1)&3;							}
		PX_FORCE_INLINE	PxU32	getChildData(PxU32 i)		const	{ return mData[i];									}
		PX_FORCE_INLINE	PxU32	decodePNSNoShift(PxU32 i)	const	{ return mData[i];									}
	};

	#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
	struct BVDataSwizzledNQ : public physx::shdfnd::UserAllocated
	{
		float		mMinX[4];
		float		mMinY[4];
		float		mMinZ[4];
		float		mMaxX[4];
		float		mMaxY[4];
		float		mMaxZ[4];

		PxU32		mData[4];	

		PX_FORCE_INLINE	PxU32	isLeaf(PxU32 i)				const	{ return mData[i]&1;								}
		PX_FORCE_INLINE	PxU32	getPrimitive(PxU32 i)		const	{ return mData[i]>>1;								}
		PX_FORCE_INLINE	PxU32	getChildOffset(PxU32 i)		const	{ return mData[i]>>GU_BV4_CHILD_OFFSET_SHIFT_COUNT;	}
		PX_FORCE_INLINE	PxU32	getChildType(PxU32 i)		const	{ return (mData[i]>>1)&3;							}
		PX_FORCE_INLINE	PxU32	getChildData(PxU32 i)		const	{ return mData[i];									}
		PX_FORCE_INLINE	PxU32	decodePNSNoShift(PxU32 i)	const	{ return mData[i];									}
	};
	#endif

#else
	#define SSE_CONST4(name, val) static const __declspec(align(16)) PxU32 name[4] = { (val), (val), (val), (val) }
	#define SSE_CONST(name) *(const __m128i *)&name
	#define SSE_CONSTF(name) *(const __m128 *)&name
#endif

	PX_FORCE_INLINE PxU32 getNbPrimitives(PxU32& primIndex)
	{
		PxU32 NbToGo = (primIndex & 15)-1;
		primIndex>>=4;
		return NbToGo;
	}

	template<class ParamsT>
	PX_FORCE_INLINE	void setupMeshPointersAndQuantizedCoeffs(ParamsT* PX_RESTRICT params, const SourceMesh* PX_RESTRICT mesh, const BV4Tree* PX_RESTRICT tree)
	{
		using namespace physx::shdfnd::aos;
		
		params->mTris32	= mesh->getTris32();
		params->mTris16	= mesh->getTris16();
		params->mVerts	= mesh->getVerts();

		V4StoreA_Safe(V4LoadU_Safe(&tree->mCenterOrMinCoeff.x), &params->mCenterOrMinCoeff_PaddedAligned.x);
		V4StoreA_Safe(V4LoadU_Safe(&tree->mExtentsOrMaxCoeff.x), &params->mExtentsOrMaxCoeff_PaddedAligned.x);
	}

	PX_FORCE_INLINE void rotateBox(Gu::Box& dst, const PxMat44& m, const Gu::Box& src)
	{
		// The extents remain constant
		dst.extents = src.extents;
		// The center gets x-formed
		dst.center = m.transform(src.center);
		// Combine rotations
		// PT: TODO: revisit.. this is awkward... grab 3x3 part of 4x4 matrix (TA34704)
		const PxMat33 tmp(	PxVec3(m.column0.x, m.column0.y, m.column0.z),
							PxVec3(m.column1.x, m.column1.y, m.column1.z),
							PxVec3(m.column2.x, m.column2.y, m.column2.z));
		dst.rot = tmp * src.rot;
	}

	PX_FORCE_INLINE PxVec3 inverseRotate(const PxMat44* PX_RESTRICT src, const PxVec3& p)
	{
		const float m00 = src->column0.x;
		const float m01 = src->column0.y;
		const float m02 = src->column0.z;

		const float m10 = src->column1.x;
		const float m11 = src->column1.y;
		const float m12 = src->column1.z;

		const float m20 = src->column2.x;
		const float m21 = src->column2.y;
		const float m22 = src->column2.z;

		return PxVec3(	m00*p.x + m01*p.y + m02*p.z,
						m10*p.x + m11*p.y + m12*p.z,
						m20*p.x + m21*p.y + m22*p.z);
	}

	PX_FORCE_INLINE PxVec3 inverseTransform(const PxMat44* PX_RESTRICT src, const PxVec3& p)
	{
		const float m30 = src->column3.x;
		const float m31 = src->column3.y;
		const float m32 = src->column3.z;

		const float m00 = src->column0.x;
		const float m01 = src->column0.y;
		const float m02 = src->column0.z;

		const float m10 = src->column1.x;
		const float m11 = src->column1.y;
		const float m12 = src->column1.z;

		const float m20 = src->column2.x;
		const float m21 = src->column2.y;
		const float m22 = src->column2.z;

		return PxVec3(	m00*p.x + m01*p.y + m02*p.z -(m30*m00 + m31*m01 + m32*m02),
						m10*p.x + m11*p.y + m12*p.z -(m30*m10 + m31*m11 + m32*m12),
						m20*p.x + m21*p.y + m22*p.z -(m30*m20 + m31*m21 + m32*m22));
	}

	PX_FORCE_INLINE	void computeLocalRay(PxVec3& localDir, PxVec3& localOrigin, const PxVec3& dir, const PxVec3& origin, const PxMat44* PX_RESTRICT worldm_Aligned)
	{
		if(worldm_Aligned)
		{
			localDir = inverseRotate(worldm_Aligned, dir);
			localOrigin = inverseTransform(worldm_Aligned, origin);
		}
		else
		{
			localDir	= dir;
			localOrigin	= origin;
		}
	}

	PX_FORCE_INLINE void computeLocalSphere(float& radius2, PxVec3& local_center, const Sphere& sphere, const PxMat44* PX_RESTRICT worldm_Aligned)
	{
		radius2 = sphere.radius * sphere.radius;
		if(worldm_Aligned)
		{
			local_center = inverseTransform(worldm_Aligned, sphere.center);
		}
		else
		{
			local_center = sphere.center;
		}
	}

	PX_FORCE_INLINE void computeLocalCapsule(Capsule& localCapsule, const Capsule& capsule, const PxMat44* PX_RESTRICT worldm_Aligned)
	{
		localCapsule.radius = capsule.radius;
		if(worldm_Aligned)
		{
			localCapsule.p0 = inverseTransform(worldm_Aligned, capsule.p0);
			localCapsule.p1 = inverseTransform(worldm_Aligned, capsule.p1);
		}
		else
		{
			localCapsule.p0 = capsule.p0;
			localCapsule.p1 = capsule.p1;
		}
	}

	PX_FORCE_INLINE void computeLocalBox(Gu::Box& dst, const Gu::Box& src, const PxMat44* PX_RESTRICT worldm_Aligned)
	{
		if(worldm_Aligned)
		{
			PxMat44 invWorldM;
			invertPRMatrix(&invWorldM, worldm_Aligned);

			rotateBox(dst, invWorldM, src);
		}
		else
		{
			dst = src;	// PT: TODO: check asm for operator= (TA34704)
		}
	}

	template<class ImpactFunctionT, class ShapeT, class ParamsT>
	static PX_FORCE_INLINE bool computeImpactDataT(const ShapeT& shape, const PxVec3& dir, SweepHit* PX_RESTRICT hit, const ParamsT* PX_RESTRICT params, const PxMat44* PX_RESTRICT worldm, bool isDoubleSided, bool meshBothSides)
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
				// PT: TODO: we shouldn't compute impact in world space, and in fact moving this to local space is necessary if we want to reuse this for box-sweeps (TA34704)
				TrianglePadded WP;
				if(worldm)
				{
					WP.verts[0] = worldm->transform(params->mP0);
					WP.verts[1] = worldm->transform(params->mP1);
					WP.verts[2] = worldm->transform(params->mP2);
				}
				else
				{
					WP.verts[0] = params->mP0;
					WP.verts[1] = params->mP1;
					WP.verts[2] = params->mP2;
				}

				PxVec3 impactNormal;
				ImpactFunctionT::computeImpact(hit->mPos, impactNormal, shape, dir, t, WP);

				// PT: by design, returned normal is opposed to the sweep direction.
				if(shouldFlipNormal(impactNormal, meshBothSides, isDoubleSided, params->mBestTriNormal, dir))
					impactNormal = -impactNormal;

				hit->mNormal = impactNormal;
			}
		}
		return true;
	}

	// PT: we don't create a structure for small meshes with just a few triangles. We use brute-force tests on these.
	template<class LeafFunction_AnyT, class LeafFunction_ClosestT, class ParamsT>
	void doBruteForceTests(PxU32 nbTris, ParamsT* PX_RESTRICT params)
	{
		PX_ASSERT(nbTris<16);
		if(params->mEarlyExit)
			LeafFunction_AnyT::doLeafTest(params, nbTris);
		else
			LeafFunction_ClosestT::doLeafTest(params, nbTris);
	}

#if PX_INTEL_FAMILY
#ifndef GU_BV4_USE_SLABS
	template<class ParamsT>
	PX_FORCE_INLINE void setupRayData(ParamsT* PX_RESTRICT params, float max_dist, const PxVec3& origin, const PxVec3& dir)
	{
		const float Half = 0.5f*max_dist;
		const FloatV HalfV = FLoad(Half);
		const Vec4V DataV = V4Scale(V4LoadU(&dir.x), HalfV);
		const Vec4V Data2V = V4Add(V4LoadU(&origin.x), DataV);
		const PxU32 MaskI = 0x7fffffff;
		const Vec4V FDirV = _mm_and_ps(_mm_load1_ps((float*)&MaskI), DataV);
		V4StoreA_Safe(DataV, &params->mData_PaddedAligned.x);
		V4StoreA_Safe(Data2V, &params->mData2_PaddedAligned.x);
		V4StoreA_Safe(FDirV, &params->mFDir_PaddedAligned.x);
	}
#endif
#endif

}
}

#endif // GU_BV4_COMMON_H
