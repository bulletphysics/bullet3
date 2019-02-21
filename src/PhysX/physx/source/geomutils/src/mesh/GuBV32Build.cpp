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

#include "foundation/PxVec4.h"
#include "GuBV32Build.h"
#include "GuBV32.h"
#include "PxTriangle.h"
#include "CmPhysXCommon.h"
#include "PsBasicTemplates.h"
#include "GuCenterExtents.h"
#include "GuBV4Build.h"
#include "PsAllocator.h"

using namespace physx;
using namespace Gu;

#include "PsVecMath.h"
using namespace physx::shdfnd::aos;

#define DELETESINGLE(x)	if (x) { delete x;		x = NULL; }
#define DELETEARRAY(x)	if (x) { delete []x;	x = NULL; }

struct BV32Node : public physx::shdfnd::UserAllocated
{
	BV32Node() : mNbChildBVNodes(0)
	{}

	BV32Data	mBVData[32];
	PxU32		mNbChildBVNodes;

	PX_FORCE_INLINE	size_t			isLeaf(PxU32 i)			const	{ return mBVData[i].mData & 1; }
	PX_FORCE_INLINE	PxU32			getPrimitive(PxU32 i)	const	{ return PxU32(mBVData[i].mData >> 1); }
	PX_FORCE_INLINE	const BV32Node*	getChild(PxU32 i)		const	{ return reinterpret_cast<BV32Node*>(mBVData[i].mData); }


	PxU32 getSize() const
	{
		return sizeof(BV32Data)*mNbChildBVNodes;
	}
};


static void fillInNodes(const AABBTreeNode* current_node, const PxU32 startIndex, const PxU32 endIndex, const AABBTreeNode** NODES, PxU32& stat)
{

	if (startIndex + 1 == endIndex)
	{
		//fill in nodes
		const AABBTreeNode* P = current_node->getPos();
		const AABBTreeNode* N = current_node->getNeg();
		NODES[startIndex] = P;
		NODES[endIndex] = N;
		stat += 2;
	}
	else
	{
		const AABBTreeNode* P = current_node->getPos();
		const AABBTreeNode* N = current_node->getNeg();
		const PxU32 midIndex = startIndex + ((endIndex - startIndex) / 2);
		if (!P->isLeaf())
			fillInNodes(P, startIndex, midIndex, NODES, stat);
		else
		{
			NODES[startIndex] = P;
			stat++;
		}

		if (!N->isLeaf())
			fillInNodes(N, midIndex + 1, endIndex, NODES, stat);
		else
		{
			NODES[midIndex + 1] = N;
			stat++;
		}
	}
}



static void setPrimitive(const AABBTree& source, BV32Node* node32, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	const PxU32 nbPrims = node->getNbPrimitives();
	PX_ASSERT(nbPrims<=32);
	const PxU32* indexBase = source.getIndices();
	const PxU32* prims = node->getPrimitives();
	const PxU32 offset = PxU32(prims - indexBase);
	
#if BV32_VALIDATE
	for (PxU32 j = 0; j<nbPrims; j++)
	{
		PX_ASSERT(prims[j] == offset + j);
	}
#endif
	const PxU32 primitiveIndex = (offset << 6) | (nbPrims & 63);

	node32->mBVData[i].mCenter = node->getAABB().getCenter();
	node32->mBVData[i].mExtents = node->getAABB().getExtents();
	if (epsilon != 0.0f)
		node32->mBVData[i].mExtents += PxVec3(epsilon, epsilon, epsilon);
	node32->mBVData[i].mData = (primitiveIndex << 1) | 1;
}

static BV32Node* setNode(const AABBTree& source, BV32Node* node32, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	BV32Node* child = NULL;

	if (node)
	{
		if (node->isLeaf())
		{
			setPrimitive(source, node32, i, node, epsilon);
		}
		else
		{
			node32->mBVData[i].mCenter = node->getAABB().getCenter();
			node32->mBVData[i].mExtents = node->getAABB().getExtents();
			if (epsilon != 0.0f)
				node32->mBVData[i].mExtents += PxVec3(epsilon, epsilon, epsilon);

			child = PX_NEW(BV32Node);
			node32->mBVData[i].mData = size_t(child);
		}
	}
	
	return child;
}


static void _BuildBV32(const AABBTree& source, BV32Node* tmp, const AABBTreeNode* current_node, float epsilon, PxU32& nbNodes)
{
	PX_ASSERT(!current_node->isLeaf());

	const AABBTreeNode* NODES[32];
	memset(NODES, 0, sizeof(AABBTreeNode*) * 32);

	fillInNodes(current_node, 0, 31, NODES, tmp->mNbChildBVNodes);

	PxU32 left = 0;
	PxU32 right = 31;

	while (left < right)
	{
		
		//sweep from the front
		while (left<right)
		{
			//found a hole
			if (NODES[left] == NULL)
				break;
			left++;
		}

		//sweep from the back
		while (left < right)
		{
			//found a node
			if (NODES[right])
				break;
			right--;
		}

		if (left != right)
		{
			//swap left and right
			const AABBTreeNode* node = NODES[right];
			NODES[right] = NODES[left];
			NODES[left] = node;
		}

	}

	nbNodes += tmp->mNbChildBVNodes;

	for (PxU32 i = 0; i < tmp->mNbChildBVNodes; ++i)
	{
		const AABBTreeNode* tempNode = NODES[i];
		BV32Node* Child = setNode(source, tmp, i, tempNode, epsilon);
		if (Child)
		{
			_BuildBV32(source, Child, tempNode, epsilon, nbNodes);
		}
	}

}

//
//static void validateTree(const AABBTree& Source, const AABBTreeNode* currentNode)
//{
//	if (currentNode->isLeaf())
//	{
//		const PxU32* indexBase = Source.getIndices();
//		const PxU32* prims = currentNode->getPrimitives();
//		const PxU32 offset = PxU32(prims - indexBase);
//		const PxU32 nbPrims = currentNode->getNbPrimitives();
//		for (PxU32 j = 0; j<nbPrims; j++)
//		{
//			PX_ASSERT(prims[j] == offset + j);
//		}
//	}
//	else
//	{
//		const AABBTreeNode* pos = currentNode->getPos();
//		validateTree(Source, pos);
//		const AABBTreeNode* neg = currentNode->getNeg();
//		validateTree(Source, neg);
//	}
//}

#if BV32_VALIDATE
static void validateNodeBound(const BV32Node* currentNode, SourceMesh* mesh)
{
	const PxU32 nbNodes = currentNode->mNbChildBVNodes;
	for (PxU32 i = 0; i < nbNodes; ++i)
	{
		const BV32Node* node = currentNode->getChild(i);
		if (currentNode->isLeaf(i))
		{
			BV32Data data = currentNode->mBVData[i];
			PxU32 nbTriangles = data.getNbReferencedTriangles();
			PxU32 startIndex = data.getTriangleStartIndex();
			const IndTri32* triIndices = mesh->getTris32();
			const PxVec3* verts = mesh->getVerts();
			PxVec3 min(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32);
			PxVec3 max(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
			for (PxU32 j = 0; j < nbTriangles; ++j)
			{
				IndTri32 index = triIndices[startIndex + j];

				for (PxU32 k = 0; k < 3; ++k)
				{
					const PxVec3& v = verts[index.mRef[k]];

					min.x = (min.x > v.x) ? v.x : min.x;
					min.y = (min.y > v.y) ? v.y : min.y;
					min.z = (min.z > v.z) ? v.z : min.z;

					max.x = (max.x < v.x) ? v.x : max.x;
					max.y = (max.y > v.y) ? v.y : max.y;
					max.z = (max.z > v.z) ? v.z : max.z;
				}
			}

			PxVec3 dMin, dMax;
			data.getMinMax(dMin, dMax);
			PX_ASSERT(dMin.x <= min.x && dMin.y <= min.y && dMin.z <= min.z);
			PX_ASSERT(dMax.x >= max.x && dMax.y >= max.y && dMax.z >= min.z);

		}
		else
		{
			validateNodeBound(node, mesh);
		}
	}
}
#endif

static bool BuildBV32Internal(BV32Tree& bv32Tree, const AABBTree& Source, SourceMesh* mesh, float epsilon)
{
	if (mesh->getNbTriangles() <= 32)
	{
		bv32Tree.mNbPackedNodes = 1;
		bv32Tree.mPackedNodes = reinterpret_cast<BV32DataPacked*>(PX_ALLOC(sizeof(BV32DataPacked), "BV32DataPacked"));
		BV32DataPacked& packedData = bv32Tree.mPackedNodes[0];
		packedData.mNbNodes = 1;
		packedData.mCenter[0] = PxVec4(Source.getBV().getCenter(), 0.f);
		packedData.mExtents[0] = PxVec4(Source.getBV().getExtents(), 0.f);
		packedData.mData[0] = (mesh->getNbTriangles() << 1) | 1;
		return bv32Tree.init(mesh, Source.getBV());
	}

	{
		struct Local
		{
			static void _CheckMD(const AABBTreeNode* current_node, PxU32& md, PxU32& cd)
			{
				cd++;
				md = PxMax(md, cd);

				if (current_node->getPos())	{ _CheckMD(current_node->getPos(), md, cd);	cd--; }
				if (current_node->getNeg())	{ _CheckMD(current_node->getNeg(), md, cd);	cd--; }
			}

			static void _Check(AABBTreeNode* current_node)
			{
				if (current_node->isLeaf())
					return;

				AABBTreeNode* P = const_cast<AABBTreeNode*>(current_node->getPos());
				AABBTreeNode* N = const_cast<AABBTreeNode*>(current_node->getNeg());
				{
					PxU32 MDP = 0;	PxU32 CDP = 0;	_CheckMD(P, MDP, CDP);
					PxU32 MDN = 0;	PxU32 CDN = 0;	_CheckMD(N, MDN, CDN);

					if (MDP>MDN)
						//					if(MDP<MDN)
					{
						Ps::swap(*P, *N);
						Ps::swap(P, N);
					}
				}
				_Check(P);
				_Check(N);
			}
		};
		Local::_Check(const_cast<AABBTreeNode*>(Source.getNodes()));
	}


	PxU32 nbNodes = 1;
	BV32Node* Root32 = PX_NEW(BV32Node);


	_BuildBV32(Source, Root32, Source.getNodes(), epsilon, nbNodes);

#if BV32_VALIDATE
	validateNodeBound(Root32, mesh);
#endif

	if (!bv32Tree.init(mesh, Source.getBV()))
		return false;
	BV32Tree* T = &bv32Tree;

	// Version with variable-sized nodes in single stream
	{
		struct Local
		{
			static void _Flatten(BV32Data* const dest, const PxU32 box_id, PxU32& current_id, const BV32Node* current, PxU32& max_depth, PxU32& current_depth, const PxU32 nb_nodes)
			{
				// Entering a new node => increase depth
				current_depth++;
				// Keep track of max depth
				if (current_depth>max_depth)
					max_depth = current_depth;

				for (PxU32 i = 0; i<current->mNbChildBVNodes; i++)
				{
					dest[box_id + i].mCenter = current->mBVData[i].mCenter;
					dest[box_id + i].mExtents = current->mBVData[i].mExtents;
					dest[box_id + i].mData = PxU32(current->mBVData[i].mData);

					PX_ASSERT(box_id + i < nb_nodes);
				}

				PxU32 NbToGo = 0;
				PxU32 NextIDs[32];
				memset(NextIDs, PX_INVALID_U32, sizeof(PxU32)*32); 
				const BV32Node* ChildNodes[32];
				memset(ChildNodes, 0, sizeof(BV32Node*)*32);

				BV32Data* data = dest + box_id;
				for (PxU32 i = 0; i<current->mNbChildBVNodes; i++)
				{
					PX_ASSERT(current->mBVData[i].mData != PX_INVALID_U32);

					if (!current->isLeaf(i))
					{

						const BV32Node* ChildNode = current->getChild(i);

						const PxU32 NextID = current_id;

						const PxU32 ChildSize = ChildNode->mNbChildBVNodes;
						current_id += ChildSize;

						const PxU32 ChildType = ChildNode->mNbChildBVNodes << 1;
						data[i].mData = size_t(ChildType + (NextID << GU_BV4_CHILD_OFFSET_SHIFT_COUNT));
						//PX_ASSERT(data[i].mData == size_t(ChildType+(NextID<<3)));

						PX_ASSERT(box_id + i < nb_nodes);

						NextIDs[NbToGo] = NextID;
						ChildNodes[NbToGo] = ChildNode;
						NbToGo++;
					}
				}

			

				for (PxU32 i = 0; i<NbToGo; i++)
				{
					_Flatten(dest, NextIDs[i], current_id, ChildNodes[i], max_depth, current_depth, nb_nodes);
					current_depth--;
				}

				DELETESINGLE(current);
			}
		};


		PxU32 CurID = Root32->mNbChildBVNodes+1;

		BV32Data* Nodes = PX_NEW(BV32Data)[nbNodes];
		Nodes[0].mCenter = Source.getBV().getCenter();
		Nodes[0].mExtents = Source.getBV().getExtents();

		const PxU32 ChildType = Root32->mNbChildBVNodes << 1;
		Nodes[0].mData = size_t(ChildType + (1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT));

		const PxU32 nbChilden = Nodes[0].getNbChildren();

		PX_UNUSED(nbChilden);


		T->mInitData = CurID;
		PxU32 MaxDepth = 0;
		PxU32 CurrentDepth = 0;

		Local::_Flatten(Nodes, 1, CurID, Root32, MaxDepth, CurrentDepth, nbNodes);

		PX_ASSERT(CurID == nbNodes);

		T->mNbNodes = nbNodes;

		T->mNodes = Nodes;
	}

	
	bv32Tree.calculateLeafNode(bv32Tree.mNodes[0]);
	
	bv32Tree.mPackedNodes = reinterpret_cast<BV32DataPacked*>(PX_ALLOC(sizeof(BV32DataPacked)*nbNodes, "BV32DataPacked"));
	bv32Tree.mNbPackedNodes = nbNodes;

	PxU32 nbPackedNodes = 1;
	PxU32 currentIndex = bv32Tree.mNodes[0].getNbChildren() - bv32Tree.mNodes[0].mNbLeafNodes + 1;
	BV32DataPacked& packedData = bv32Tree.mPackedNodes[0];
	bv32Tree.createSOAformatNode(packedData, bv32Tree.mNodes[0], 1, currentIndex, nbPackedNodes);

	bv32Tree.mNbPackedNodes = nbPackedNodes;

	PX_ASSERT(nbPackedNodes == currentIndex);
	PX_ASSERT(nbPackedNodes > 0);

	return true;
}

/////

struct ReorderData32
{
	const SourceMesh*	mMesh;
	PxU32*				mOrder;
	PxU32				mNbTrisPerLeaf;
	PxU32				mIndex;
	PxU32				mNbTris;
	PxU32				mStats[32];
};

static bool gReorderCallback(const AABBTreeNode* current, PxU32 /*depth*/, void* userData)
{
	ReorderData32* Data = reinterpret_cast<ReorderData32*>(userData);
	if (current->isLeaf())
	{
		const PxU32 n = current->getNbPrimitives();
		PX_ASSERT(n > 0);
		PX_ASSERT(n <= Data->mNbTrisPerLeaf);
		Data->mStats[n-1]++;
		PxU32* Prims = const_cast<PxU32*>(current->getPrimitives());

		for (PxU32 i = 0; i<n; i++)
		{
			PX_ASSERT(Prims[i]<Data->mNbTris);
			Data->mOrder[Data->mIndex] = Prims[i];
			PX_ASSERT(Data->mIndex<Data->mNbTris);
			Prims[i] = Data->mIndex;
			Data->mIndex++;
		}
	}
	return true;
}


bool physx::Gu::BuildBV32Ex(BV32Tree& tree, SourceMesh& mesh, float epsilon, PxU32 nbTrisPerLeaf)
{
	const PxU32 nbTris = mesh.mNbTris;

	AABBTree Source;
	if (!Source.buildFromMesh(mesh, nbTrisPerLeaf))
		return false;


	{
		PxU32* order = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nbTris, "BV32"));
		ReorderData32 RD;
		RD.mMesh = &mesh;
		RD.mOrder = order;
		RD.mNbTrisPerLeaf = nbTrisPerLeaf;
		RD.mIndex = 0;
		RD.mNbTris = nbTris;
		for (PxU32 i = 0; i<32; i++)
			RD.mStats[i] = 0;
		Source.walk(gReorderCallback, &RD);
		PX_ASSERT(RD.mIndex == nbTris);
		mesh.remapTopology(order);
		PX_FREE(order);
		//		for(PxU32 i=0;i<16;i++)
		//			printf("%d: %d\n", i, RD.mStats[i]);
	}


	//if (mesh.getNbTriangles() <= nbTrisPerLeaf)
	//	return tree.init(&mesh, Source.getBV());

	return BuildBV32Internal(tree, Source, &mesh, epsilon);
}
