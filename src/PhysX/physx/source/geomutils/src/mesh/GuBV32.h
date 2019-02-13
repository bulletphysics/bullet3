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

#ifndef GU_BV32_H
#define GU_BV32_H

#include "foundation/PxBounds3.h"
#include "PxSerialFramework.h"
#include "PsUserAllocated.h"
#include "GuBV4.h"
#include "CmPhysXCommon.h"
#include "PsArray.h"
#include "foundation/PxVec4.h"

namespace physx
{
	namespace Gu
	{
		struct BV32Data : public physx::shdfnd::UserAllocated
		{
			PxVec3		mCenter;
			PxU32		mNbLeafNodes;
			PxVec3		mExtents;
			size_t		mData;
			

			PX_FORCE_INLINE BV32Data() : mNbLeafNodes(0), mData(PX_INVALID_U32)
			{
				setEmpty();
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			isLeaf()			const	{ return mData & 1; }

			//if the node is leaf, 
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getNbReferencedTriangles()	const	{ PX_ASSERT(isLeaf()); return PxU32((mData >>1)&63); }
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getTriangleStartIndex()	const	{ PX_ASSERT(isLeaf()); return PxU32(mData >> 7); }

			//PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getPrimitive()		const	{ return mData >> 1; }
			//if the node isn't leaf, we will get the childOffset
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getChildOffset()	const	{ PX_ASSERT(!isLeaf()); return PxU32(mData >> GU_BV4_CHILD_OFFSET_SHIFT_COUNT); }
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getNbChildren()		const	{ PX_ASSERT(!isLeaf()); return ((mData) & ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1))>>1; }
			
			PX_CUDA_CALLABLE PX_FORCE_INLINE	void			getMinMax(PxVec3& min, PxVec3& max)			const
			{
				min = mCenter - mExtents;
				max = mCenter + mExtents;
			}

			PX_FORCE_INLINE	void setEmpty()
			{
				mCenter = PxVec3(0.0f, 0.0f, 0.0f);
				mExtents = PxVec3(-1.0f, -1.0f, -1.0f);
			}
			
		};

		PX_ALIGN_PREFIX(16)
		struct BV32DataPacked
		{
			PxVec4 mCenter[32];
			PxVec4 mExtents[32];
			PxU32 mData[32];
			PxU32 mNbNodes;
			PxU32 pad[3];

			PX_CUDA_CALLABLE PX_FORCE_INLINE BV32DataPacked() : mNbNodes(0)
			{
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			isLeaf(const PxU32 index)			const	{ return mData[index] & 1; }
			//if the node is leaf, 
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getNbReferencedTriangles(const PxU32 index)	const	{ PX_ASSERT(isLeaf(index)); return (mData[index] >> 1) & 63; }
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getTriangleStartIndex(const PxU32 index)	const	{ PX_ASSERT(isLeaf(index)); return (mData[index] >> 7); }
			//if the node isn't leaf, we will get the childOffset
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getChildOffset(const PxU32 index)	const	{ PX_ASSERT(!isLeaf(index)); return mData[index] >> GU_BV4_CHILD_OFFSET_SHIFT_COUNT; }
			PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32			getNbChildren(const PxU32 index)		const	{ PX_ASSERT(!isLeaf(index)); return ((mData[index])& ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1)) >> 1; }
		} 
		PX_ALIGN_SUFFIX(16);

		class BV32Tree : public physx::shdfnd::UserAllocated
		{
		public:
			// PX_SERIALIZATION
			BV32Tree(const PxEMPTY);
			void			exportExtraData(PxSerializationContext&);
			void			importExtraData(PxDeserializationContext& context);
			static			void			getBinaryMetaData(PxOutputStream& stream);
			//~PX_SERIALIZATION

			PX_PHYSX_COMMON_API				BV32Tree();
			PX_PHYSX_COMMON_API				BV32Tree(SourceMesh* meshInterface, const PxBounds3& localBounds);
			PX_PHYSX_COMMON_API				~BV32Tree();

			bool			load(PxInputStream& stream, bool mismatch);

			void			calculateLeafNode(BV32Data& node);
			void			createSOAformatNode(BV32DataPacked& packedData, const BV32Data& node, const PxU32 childOffset, PxU32& currentIndex, PxU32& nbPackedNodes);

			void			reset();
			void			operator = (BV32Tree& v);

			bool			init(SourceMesh* meshInterface, const PxBounds3& localBounds);
			void			release();

			SourceMesh*		mMeshInterface;
			LocalBounds		mLocalBounds;

			PxU32			mNbNodes;
			BV32Data*		mNodes;
			BV32DataPacked*	mPackedNodes;
			PxU32			mNbPackedNodes;
			PxU32			mInitData;
			bool			mUserAllocated;	// PT: please keep these 4 bytes right after mCenterOrMinCoeff/mExtentsOrMaxCoeff for safe V4 loading
			bool			mPadding[3];
		};

	} // namespace Gu
}

#endif // GU_BV32_H
