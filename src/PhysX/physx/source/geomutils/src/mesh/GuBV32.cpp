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

#include "foundation/PxMemory.h"
#include "GuBV32.h"
#include "GuSerialize.h"
#include "CmUtils.h"
#include "PsUtilities.h"

using namespace physx;
using namespace Gu;

#define DELETEARRAY(x)		if (x) { delete []x;	x = NULL; }


BV32Tree::BV32Tree(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	reset();
	init(meshInterface, localBounds);
}

BV32Tree::BV32Tree()
{
	reset();
}

void BV32Tree::release()
{
	if (!mUserAllocated)
	{
		DELETEARRAY(mNodes);
		PX_FREE_AND_RESET(mPackedNodes);
	}
	mNodes = NULL;
	mNbNodes = 0;
}

BV32Tree::~BV32Tree()
{
	release();
}

void BV32Tree::reset()
{
	mMeshInterface = NULL;
	mNbNodes = 0;
	mNodes = NULL;
	mNbPackedNodes = 0;
	mPackedNodes = NULL;
	mInitData = 0;
	mUserAllocated = false;
}

void BV32Tree::operator=(BV32Tree& v)
{
	mMeshInterface = v.mMeshInterface;
	mLocalBounds = v.mLocalBounds;
	mNbNodes = v.mNbNodes;
	mNodes = v.mNodes;
	mInitData = v.mInitData;
	mUserAllocated = v.mUserAllocated;
	v.reset();
}

bool BV32Tree::init(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	mMeshInterface = meshInterface;
	mLocalBounds.init(localBounds);
	return true;
}

// PX_SERIALIZATION
BV32Tree::BV32Tree(const PxEMPTY)
{
	mUserAllocated = true;
}

void BV32Tree::exportExtraData(PxSerializationContext& stream)
{
	stream.alignData(16);
	stream.writeData(mPackedNodes, mNbNodes*sizeof(BV32DataPacked));
}

void BV32Tree::importExtraData(PxDeserializationContext& context)
{
	context.alignExtraData(16);
	mPackedNodes = context.readExtraData<BV32DataPacked>(mNbNodes);
}
//~PX_SERIALIZATION

bool BV32Tree::load(PxInputStream& stream, bool mismatch_)
{
	PX_ASSERT(!mUserAllocated);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if(a != 'B' || b != 'V' || c != '3' || d != '2')
		return false;

	bool mismatch;
	PxU32 fileVersion;
	if(!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	mLocalBounds.mCenter.x = readFloat(mismatch, stream);
	mLocalBounds.mCenter.y = readFloat(mismatch, stream);
	mLocalBounds.mCenter.z = readFloat(mismatch, stream);
	mLocalBounds.mExtentsMagnitude = readFloat(mismatch, stream);

	mInitData = readDword(mismatch, stream);

	/*const PxU32 nbNodes = readDword(mismatch, stream);
	mNbNodes = nbNodes;

	if (nbNodes)
	{
		BV32Data* nodes = PX_NEW(BV32Data)[nbNodes];

		mNodes = nodes;
		Cm::markSerializedMem(nodes, sizeof(BV32Data)*nbNodes);

		for (PxU32 i = 0; i<nbNodes; i++)
		{
			BV32Data& node = nodes[i];

			readFloatBuffer(&node.mCenter.x, 3, mismatch, stream);
			node.mData = readDword(mismatch, stream);
			readFloatBuffer(&node.mExtents.x, 3, mismatch, stream);
		}
	}*/


	//read SOA format node data
	const PxU32 nbPackedNodes = readDword(mismatch, stream);
	mNbPackedNodes = nbPackedNodes;

	if (nbPackedNodes)
	{
		mPackedNodes = reinterpret_cast<BV32DataPacked*>(PX_ALLOC(sizeof(BV32DataPacked)*nbPackedNodes, "BV32DataPacked"));

		Cm::markSerializedMem(mPackedNodes, sizeof(BV32DataPacked)*nbPackedNodes);

		for (PxU32 i = 0; i < nbPackedNodes; ++i)
		{
			BV32DataPacked& node = mPackedNodes[i];
			node.mNbNodes = readDword(mismatch, stream);
			PX_ASSERT(node.mNbNodes > 0);
			ReadDwordBuffer(node.mData, node.mNbNodes, mismatch, stream);
			const PxU32 nbElements = 4 * node.mNbNodes;
			readFloatBuffer(&node.mCenter[0].x, nbElements, mismatch, stream);
			readFloatBuffer(&node.mExtents[0].x, nbElements, mismatch, stream);
			
		}
	}

	return true;
}


void BV32Tree::calculateLeafNode(BV32Data& node)
{
	if (!node.isLeaf())
	{
		const PxU32 nbChildren = node.getNbChildren();
		const PxU32 offset = node.getChildOffset();
		//calcualte how many children nodes are leaf nodes
		PxU32 nbLeafNodes = 0;
		for (PxU32 i = 0; i < nbChildren; ++i)
		{
			BV32Data& child = mNodes[offset + i];

			if (child.isLeaf())
			{
				nbLeafNodes++;
			}
		}

		node.mNbLeafNodes = nbLeafNodes;
		for (PxU32 i = 0; i < nbChildren; ++i)
		{
			BV32Data& child = mNodes[offset + i];
			calculateLeafNode(child);
		}

	}
}



void BV32Tree::createSOAformatNode(BV32DataPacked& packedData, const BV32Data& node, const PxU32 childOffset, PxU32& currentIndex, PxU32& nbPackedNodes)
{
	
	//found the next 32 nodes and fill it in SOA format
	
	const PxU32 nbChildren = node.getNbChildren();
	const PxU32 offset = node.getChildOffset();


	for (PxU32 i = 0; i < nbChildren; ++i)
	{
		BV32Data& child = mNodes[offset + i];

		packedData.mCenter[i] = PxVec4(child.mCenter, 0.f);
		packedData.mExtents[i] = PxVec4(child.mExtents, 0.f);
		packedData.mData[i] = PxU32(child.mData);
	}

	packedData.mNbNodes = nbChildren;
	
	PxU32 NbToGo = 0;
	PxU32 NextIDs[32];
	memset(NextIDs, PX_INVALID_U32, sizeof(PxU32) * 32);
	const BV32Data* ChildNodes[32];
	memset(ChildNodes, 0, sizeof(BV32Data*) * 32);
	

	for (PxU32 i = 0; i< nbChildren; i++)
	{
		BV32Data& child = mNodes[offset + i];

		if (!child.isLeaf())
		{
			const PxU32 NextID = currentIndex;

			const PxU32 ChildSize = child.getNbChildren() - child.mNbLeafNodes;
			currentIndex += ChildSize;

			//packedData.mData[i] = (packedData.mData[i] & ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1)) | (NextID << GU_BV4_CHILD_OFFSET_SHIFT_COUNT);
			packedData.mData[i] = (packedData.mData[i] & ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1)) | ((childOffset + NbToGo) << GU_BV4_CHILD_OFFSET_SHIFT_COUNT);

			NextIDs[NbToGo] = NextID;
			ChildNodes[NbToGo] = &child;
			NbToGo++;
		}
	}

	nbPackedNodes += NbToGo;
	for (PxU32 i = 0; i < NbToGo; ++i)
	{
		const BV32Data& child = *ChildNodes[i];
	
		BV32DataPacked& childData = mPackedNodes[childOffset+i];
		
		createSOAformatNode(childData, child, NextIDs[i], currentIndex, nbPackedNodes);

	}

}
