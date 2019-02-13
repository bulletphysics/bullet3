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


#ifndef GU_BVH_STRUCTURE_H
#define GU_BVH_STRUCTURE_H

/** \addtogroup geomutils
@{
*/

#include "PsUserAllocated.h"
#include "CmRefCountable.h"
#include "CmPhysXCommon.h"
#include "GuMeshFactory.h"
#include "PsVecMath.h"
#include "PxBVHStructure.h"

namespace physx
{
namespace Gu
{
	struct BVHNode
	{
		public:
		PX_FORCE_INLINE	PxU32						isLeaf()								const	{ return mData&1;			}

		PX_FORCE_INLINE	const PxU32*				getPrimitives(const PxU32* base)		const	{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32*						getPrimitives(PxU32* base)						{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32						getNbPrimitives()						const	{ return (mData>>1)&15;		}

		PX_FORCE_INLINE	PxU32						getPosIndex()							const	{ return mData>>1;			}
		PX_FORCE_INLINE	PxU32						getNegIndex()							const	{ return (mData>>1) + 1;			}
		PX_FORCE_INLINE	const BVHNode*				getPos(const BVHNode* base)	const	{ return base + (mData>>1);	}
		PX_FORCE_INLINE	const BVHNode*				getNeg(const BVHNode* base)	const	{ const BVHNode* P = getPos(base); return P ? P+1 : NULL;}

		PX_FORCE_INLINE	BVHNode*					getPos(BVHNode* base)				{ return base + (mData >> 1); }
		PX_FORCE_INLINE	BVHNode*					getNeg(BVHNode* base)				{ BVHNode* P = getPos(base); return P ? P + 1 : NULL; }


		PX_FORCE_INLINE	void						getAABBCenterExtentsV(shdfnd::aos::Vec3V* center, shdfnd::aos::Vec3V* extents) const
													{
														const shdfnd::aos::Vec4V minV = shdfnd::aos::V4LoadU(&mBV.minimum.x);
														const shdfnd::aos::Vec4V maxV = shdfnd::aos::V4LoadU(&mBV.maximum.x);

														const float half = 0.5f;
														const shdfnd::aos::FloatV halfV = shdfnd::aos::FLoad(half);

														*extents = shdfnd::aos::Vec3V_From_Vec4V(shdfnd::aos::V4Scale(shdfnd::aos::V4Sub(maxV, minV), halfV));
														*center = shdfnd::aos::Vec3V_From_Vec4V(shdfnd::aos::V4Scale(shdfnd::aos::V4Add(maxV, minV), halfV));
													}

		PX_FORCE_INLINE	void						getAABBCenterExtentsV2(shdfnd::aos::Vec3V* center, shdfnd::aos::Vec3V* extents) const
													{
														const shdfnd::aos::Vec4V minV = shdfnd::aos::V4LoadU(&mBV.minimum.x);
														const shdfnd::aos::Vec4V maxV = shdfnd::aos::V4LoadU(&mBV.maximum.x);

														*extents = shdfnd::aos::Vec3V_From_Vec4V(shdfnd::aos::V4Sub(maxV, minV));
														*center = shdfnd::aos::Vec3V_From_Vec4V(shdfnd::aos::V4Add(maxV, minV));
													}

		PX_FORCE_INLINE	void						getAABBMinMaxV(shdfnd::aos::Vec4V* minV, shdfnd::aos::Vec4V* maxV) const
													{
														*minV = shdfnd::aos::V4LoadU(&mBV.minimum.x);
														*maxV = shdfnd::aos::V4LoadU(&mBV.maximum.x);
													}

						PxBounds3					mBV;	// Global bounding-volume enclosing all the node-related primitives
						PxU32						mData;	// 27 bits node or prim index|4 bits #prims|1 bit leaf
	};

	struct BVHTree
	{
		BVHTree(const BVHNode* node, const PxU32* indices):
			mRootNode(node),
			mIndices(indices)
		{
		}


		const BVHNode*	getNodes() const { return mRootNode; }
		const PxU32*	getIndices() const { return mIndices; }

		const BVHNode*	mRootNode;
		const PxU32*	mIndices;
	};

	struct BVHCallback
	{
		BVHCallback(PxU32* hits, PxU32 numMaxHits):
			mHits(hits),
			mNbMaxHits(numMaxHits),
			mCurrentHitsCount(0)
		{
		}

		bool invoke(PxReal& , PxU32 payload)
		{
			mHits[mCurrentHitsCount++] = payload;
			if(mCurrentHitsCount == mNbMaxHits)
				return false;

			return true;
		}

		PxU32*		mHits;
		PxU32		mNbMaxHits;
		PxU32		mCurrentHitsCount;
	};

	struct BVHStructureData
	{
		PxU32				mNumVolumes;
		PxU32				mNumNodes;
		PxBounds3*			mBounds;
		PxU32*				mIndices;		
		BVHNode*			mNodes;
	};
/**
\brief Represents a BVH.
*/
	class BVHStructure: public PxBVHStructure, public Ps::UserAllocated, public Cm::RefCountable
	{
	public:
		/**
		\brief Constructor
		*/
		BVHStructure(GuMeshFactory* factory);
		BVHStructure(GuMeshFactory* factory, BVHStructureData& data);

		/**
		\brief Destructor
		*/
		~BVHStructure();


		bool							load(PxInputStream& desc);

		void							release();

//		PxBVHStructure
		PxU32							getNbBounds() const { return mNumVolumes; }
		const PxBounds3*				getBounds() const { return mBounds; }
		PxU32							raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal maxDist, PxU32 maxHits, PxU32* PX_RESTRICT rayHits) const;
		PxU32							sweep(const PxBounds3& aabb, const PxVec3& unitDir, PxReal maxDist, PxU32 maxHits, PxU32* PX_RESTRICT sweepHits) const;
		PxU32							overlap(const PxBounds3& aabb, PxU32 maxHits, PxU32* PX_RESTRICT overlapHits) const;
//		~PxBVHStructure

// Cm::RefCountable
		virtual	void					onRefCountZero();
//~Cm::RefCountable

		const BVHNode*					getNodes() const { return mNodes; }
		const PxU32*					getIndices() const { return mIndices; }

	private:
		void							createVolumes() const;

	private:
		GuMeshFactory*					mMeshFactory;

		PxU32							mNumVolumes;
		PxU32							mNumNodes;
		PxBounds3*						mBounds;
		PxU32*							mIndices;
		mutable PxU32*					mVolumes;			// used just for queries
		BVHNode*						mNodes;
	};
}

}

/** @} */
#endif
