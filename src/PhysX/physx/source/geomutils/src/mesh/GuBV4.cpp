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

#include "foundation/PxMemory.h"
#include "GuBV4.h"
#include "GuBV4_Common.h"
#include "GuSerialize.h"
#include "CmUtils.h"
#include "PsUtilities.h"

using namespace physx;
using namespace Gu;

#define DELETEARRAY(x)		if (x) { delete []x;	x = NULL; }

SourceMeshBase::SourceMeshBase()
{
	mNbVerts = 0;
	mVerts = NULL; 
	mRemap = NULL;
}

SourceMeshBase::~SourceMeshBase()
{
	PX_FREE_AND_RESET(mRemap);
}

SourceMesh::SourceMesh()
{
	reset();
}

SourceMesh::~SourceMesh()
{

}

void SourceMesh::reset()
{
	mNbVerts		= 0;
	mVerts			= NULL;
	mNbTris			= 0;
	mTriangles32	= NULL;
	mTriangles16	= NULL;
	mRemap = NULL;
}

void SourceMesh::operator=(SourceMesh& v)
{
	mNbVerts		= v.mNbVerts;
	mVerts			= v.mVerts;
	mNbTris			= v.mNbTris;
	mTriangles32	= v.mTriangles32;
	mTriangles16	= v.mTriangles16;
	mRemap			= v.mRemap;
	v.reset();
}

void SourceMesh::remapTopology(const PxU32* order)
{
	if(!mNbTris)
		return;

	if(mTriangles32)
	{
		IndTri32* newTopo = PX_NEW(IndTri32)[mNbTris];
		for(PxU32 i=0;i<mNbTris;i++)
			newTopo[i] = mTriangles32[order[i]];

		PxMemCopy(mTriangles32, newTopo, sizeof(IndTri32)*mNbTris);
		DELETEARRAY(newTopo);
	}
	else
	{
		PX_ASSERT(mTriangles16);
		IndTri16* newTopo = PX_NEW(IndTri16)[mNbTris];
		for(PxU32 i=0;i<mNbTris;i++)
			newTopo[i] = mTriangles16[order[i]];

		PxMemCopy(mTriangles16, newTopo, sizeof(IndTri16)*mNbTris);
		DELETEARRAY(newTopo);
	}

	{
		PxU32* newMap = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mNbTris, "OPC2"));
		for(PxU32 i=0;i<mNbTris;i++)
			newMap[i] = mRemap ? mRemap[order[i]] : order[i];

		PX_FREE_AND_RESET(mRemap);
		mRemap = newMap;
	}
}

bool SourceMesh::isValid() const
{
	if(!mNbTris || !mNbVerts)			return false;
	if(!mVerts)							return false;
	if(!mTriangles32 && !mTriangles16)	return false;
	return true;
}

/////

BV4Tree::BV4Tree(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	reset();
	init(meshInterface, localBounds);
}

BV4Tree::BV4Tree()
{
	reset();
}

void BV4Tree::release()
{
	if(!mUserAllocated)
	{
#ifdef GU_BV4_USE_SLABS
		PX_FREE(mNodes);
//		PX_DELETE_AND_RESET(mNodes);
#else
		DELETEARRAY(mNodes);
#endif
	}

	mNodes = NULL;
	mNbNodes = 0;
	reset();
}

BV4Tree::~BV4Tree()
{
	release();
}

void BV4Tree::reset()
{
	mMeshInterface		= NULL;
	mNbNodes			= 0;
	mNodes				= NULL;
	mInitData			= 0;
	mCenterOrMinCoeff	= PxVec3(0.0f);
	mExtentsOrMaxCoeff	= PxVec3(0.0f);
	mUserAllocated		= false;
	mQuantized			= false;
}

void BV4Tree::operator=(BV4Tree& v)
{
	mMeshInterface		= v.mMeshInterface;
	mLocalBounds		= v.mLocalBounds;
	mNbNodes			= v.mNbNodes;
	mNodes				= v.mNodes;
	mInitData			= v.mInitData;
	mCenterOrMinCoeff	= v.mCenterOrMinCoeff;
	mExtentsOrMaxCoeff	= v.mExtentsOrMaxCoeff;
	mUserAllocated		= v.mUserAllocated;
	mQuantized			= v.mQuantized;
	v.reset();
}

bool BV4Tree::init(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	mMeshInterface	= meshInterface;
	mLocalBounds.init(localBounds);
	return true;
}


// PX_SERIALIZATION
BV4Tree::BV4Tree(const PxEMPTY) : mLocalBounds(PxEmpty)
{
	mUserAllocated = true;
}

void BV4Tree::exportExtraData(PxSerializationContext& stream)
{
	if(mNbNodes)
	{
		stream.alignData(16);
#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
		const PxU32 nodeSize = mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
#else
		const PxU32 nodeSize = sizeof(BVDataPackedQ);
#endif
		stream.writeData(mNodes, mNbNodes*nodeSize);
	}
}

void BV4Tree::importExtraData(PxDeserializationContext& context)
{
	if(mNbNodes)
	{
		context.alignExtraData(16);
#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
		if(mQuantized)
			mNodes = context.readExtraData<BVDataPackedQ>(mNbNodes);
		else
			mNodes = context.readExtraData<BVDataPackedNQ>(mNbNodes);
#else
		mNodes = context.readExtraData<BVDataPackedQ>(mNbNodes);
#endif
	}
}
//~PX_SERIALIZATION

bool BV4Tree::load(PxInputStream& stream, bool mismatch_)
{
	PX_ASSERT(!mUserAllocated);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if(a!='B' || b!='V' || c!='4' || d!=' ')
		return false;

	bool mismatch;
	PxU32 fileVersion;
	if(!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	readFloatBuffer(&mLocalBounds.mCenter.x, 3, mismatch, stream);
	mLocalBounds.mExtentsMagnitude = readFloat(mismatch, stream);

	mInitData = readDword(mismatch, stream);

	readFloatBuffer(&mCenterOrMinCoeff.x, 3, mismatch, stream);
	readFloatBuffer(&mExtentsOrMaxCoeff.x, 3, mismatch, stream);

	// PT: version 3
	if(fileVersion>=3)
	{
		const PxU32 Quantized = readDword(mismatch, stream);
		mQuantized = Quantized!=0;
	}
	else
		mQuantized = true;

	const PxU32 nbNodes = readDword(mismatch, stream);
	mNbNodes = nbNodes;

	if(nbNodes)
	{
		PxU32 dataSize = 0;
#ifdef GU_BV4_USE_SLABS
	#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
		const PxU32 nodeSize = mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
	#else
		const PxU32 nodeSize = sizeof(BVDataPackedQ);
	#endif
		dataSize = nodeSize*nbNodes;
		void* nodes = PX_ALLOC(dataSize, "BV4 nodes");	// PT: PX_NEW breaks alignment here
//		BVDataPacked* nodes = reinterpret_cast<BVDataPacked*>(PX_ALLOC(sizeof(BVDataPacked)*nbNodes, "BV4 nodes"));	// PT: PX_NEW breaks alignment here
		mNodes = nodes;
#else
		BVDataPacked* nodes = PX_NEW(BVDataPacked)[nbNodes];
		mNodes = nodes;
#endif
//		Cm::markSerializedMem(nodes, dataSize);

		stream.read(nodes, dataSize);
		PX_ASSERT(!mismatch);
	}
	else mNodes = NULL;

	return true;
}

void BV4Tree::refit(float epsilon)
{
	if(mQuantized)
		return;

#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
	// PT: TODO: SIMD
	const PxVec3 eps(epsilon);
	PX_ASSERT(!(mNbNodes&3));
	PxU32 nb = mNbNodes/4;
	BVDataSwizzledNQ* data = reinterpret_cast<BVDataSwizzledNQ*>(mNodes);
	while(nb--)
	{
		BVDataSwizzledNQ* current = data + nb;

		for(PxU32 j=0;j<4;j++)
		{
			if(current->getChildData(j)==PX_INVALID_U32)
				continue;

			PxBounds3 refitBox;
			refitBox.setEmpty();

			if(current->isLeaf(j))
			{
				PxU32 primIndex = current->getPrimitive(j);

				PxU32 nbToGo = getNbPrimitives(primIndex);
				VertexPointers VP;
				do
				{
					PX_ASSERT(primIndex<mMeshInterface->getNbTriangles());
					mMeshInterface->getTriangle(VP, primIndex);

					refitBox.include(*VP.Vertex[0]);
					refitBox.include(*VP.Vertex[1]);
					refitBox.include(*VP.Vertex[2]);

					primIndex++;
				}while(nbToGo--);
			}
			else
			{
				PxU32 childOffset = current->getChildOffset(j);
				PX_ASSERT(!(childOffset&3));
				childOffset>>=2;
				PX_ASSERT(childOffset>nb);
				const PxU32 childType = current->getChildType(j);

				const BVDataSwizzledNQ* next = data + childOffset;
				{
					if(childType>1)
					{
						const PxBounds3 childBox(	PxVec3(next->mMinX[3], next->mMinY[3], next->mMinZ[3]),
													PxVec3(next->mMaxX[3], next->mMaxY[3], next->mMaxZ[3]));
						refitBox.include(childBox);
					}

					if(childType>0)
					{
						const PxBounds3 childBox(	PxVec3(next->mMinX[2], next->mMinY[2], next->mMinZ[2]),
													PxVec3(next->mMaxX[2], next->mMaxY[2], next->mMaxZ[2]));
						refitBox.include(childBox);
					}

					{
						const PxBounds3 childBox(	PxVec3(next->mMinX[1], next->mMinY[1], next->mMinZ[1]),
													PxVec3(next->mMaxX[1], next->mMaxY[1], next->mMaxZ[1]));
						refitBox.include(childBox);
					}

					{
						const PxBounds3 childBox(	PxVec3(next->mMinX[0], next->mMinY[0], next->mMinZ[0]),
													PxVec3(next->mMaxX[0], next->mMaxY[0], next->mMaxZ[0]));
						refitBox.include(childBox);
					}
				}
			}
			refitBox.minimum -= eps;
			refitBox.maximum += eps;

			current->mMinX[j] = refitBox.minimum.x;
			current->mMinY[j] = refitBox.minimum.y;
			current->mMinZ[j] = refitBox.minimum.z;
			current->mMaxX[j] = refitBox.maximum.x;
			current->mMaxY[j] = refitBox.maximum.y;
			current->mMaxZ[j] = refitBox.maximum.z;
		}
	}
#else
	PX_UNUSED(epsilon);
#endif
}

