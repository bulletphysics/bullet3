/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for optimized trees. Implements 4 trees:
 *	- normal
 *	- no leaf
 *	- quantized
 *	- no leaf / quantized
 *
 *	\file		OPC_OptimizedTree.cpp
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A standard AABB tree.
 *
 *	\class		AABBCollisionTree
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A no-leaf AABB tree.
 *
 *	\class		AABBNoLeafTree
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A quantized AABB tree.
 *
 *	\class		AABBQuantizedTree
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A quantized no-leaf AABB tree.
 *
 *	\class		AABBQuantizedNoLeafTree
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

//! Compilation flag:
//! - true to fix quantized boxes (i.e. make sure they enclose the original ones)
//! - false to see the effects of quantization errors (faster, but wrong results in some cases)
static bool gFixQuantized = true;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds an implicit tree from a standard one. An implicit tree is a complete tree (2*N-1 nodes) whose negative
 *	box pointers and primitive pointers have been made implicit, hence packing 3 pointers in one.
 *
 *	Layout for implicit trees:
 *	Node:
 *			- box
 *			- data (32-bits value)
 *
 *	if data's LSB = 1 =>	remaining bits are a primitive pointer
 *	else					remaining bits are a P-node pointer, and N = P + 1
 *
 *	\relates	AABBCollisionNode
 *	\fn			_BuildCollisionTree(AABBCollisionNode* linear, const udword box_id, udword& current_id, const AABBTreeNode* current_node)
 *	\param		linear			[in] base address of destination nodes
 *	\param		box_id			[in] index of destination node
 *	\param		current_id		[in] current running index
 *	\param		current_node	[in] current node from input tree
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void _BuildCollisionTree(AABBCollisionNode* linear, const udword box_id, udword& current_id, const AABBTreeNode* current_node)
{
	// Current node from input tree is "current_node". Must be flattened into "linear[boxid]".

	// Store the AABB
	current_node->GetAABB()->GetCenter(linear[box_id].mAABB.mCenter);
	current_node->GetAABB()->GetExtents(linear[box_id].mAABB.mExtents);
	// Store remaining info
	if(current_node->IsLeaf())
	{
		// The input tree must be complete => i.e. one primitive/leaf
		ASSERT(current_node->GetNbPrimitives()==1);
		// Get the primitive index from the input tree
		udword PrimitiveIndex = current_node->GetPrimitives()[0];
		// Setup box data as the primitive index, marked as leaf
		linear[box_id].mData = (PrimitiveIndex<<1)|1;
	}
	else
	{
		// To make the negative one implicit, we must store P and N in successive order
		udword PosID = current_id++;	// Get a new id for positive child
		udword NegID = current_id++;	// Get a new id for negative child
		// Setup box data as the forthcoming new P pointer
		linear[box_id].mData = (udword)&linear[PosID];
		// Make sure it's not marked as leaf
		ASSERT(!(linear[box_id].mData&1));
		// Recurse with new IDs
		_BuildCollisionTree(linear, PosID, current_id, current_node->GetPos());
		_BuildCollisionTree(linear, NegID, current_id, current_node->GetNeg());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds a "no-leaf" tree from a standard one. This is a tree whose leaf nodes have been removed.
 *
 *	Layout for no-leaf trees:
 *
 *	Node:
 *			- box
 *			- P pointer => a node (LSB=0) or a primitive (LSB=1)
 *			- N pointer => a node (LSB=0) or a primitive (LSB=1)
 *
 *	\relates	AABBNoLeafNode
 *	\fn			_BuildNoLeafTree(AABBNoLeafNode* linear, const udword box_id, udword& current_id, const AABBTreeNode* current_node)
 *	\param		linear			[in] base address of destination nodes
 *	\param		box_id			[in] index of destination node
 *	\param		current_id		[in] current running index
 *	\param		current_node	[in] current node from input tree
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void _BuildNoLeafTree(AABBNoLeafNode* linear, const udword box_id, udword& current_id, const AABBTreeNode* current_node)
{
	const AABBTreeNode* P = current_node->GetPos();
	const AABBTreeNode* N = current_node->GetNeg();
	// Leaf nodes here?!
	ASSERT(P);
	ASSERT(N);
	// Internal node => keep the box
	current_node->GetAABB()->GetCenter(linear[box_id].mAABB.mCenter);
	current_node->GetAABB()->GetExtents(linear[box_id].mAABB.mExtents);

	if(P->IsLeaf())
	{
		// The input tree must be complete => i.e. one primitive/leaf
		ASSERT(P->GetNbPrimitives()==1);
		// Get the primitive index from the input tree
		udword PrimitiveIndex = P->GetPrimitives()[0];
		// Setup prev box data as the primitive index, marked as leaf
		linear[box_id].mPosData = (PrimitiveIndex<<1)|1;
	}
	else
	{
		// Get a new id for positive child
		udword PosID = current_id++;
		// Setup box data
		linear[box_id].mPosData = (udword)&linear[PosID];
		// Make sure it's not marked as leaf
		ASSERT(!(linear[box_id].mPosData&1));
		// Recurse
		_BuildNoLeafTree(linear, PosID, current_id, P);
	}

	if(N->IsLeaf())
	{
		// The input tree must be complete => i.e. one primitive/leaf
		ASSERT(N->GetNbPrimitives()==1);
		// Get the primitive index from the input tree
		udword PrimitiveIndex = N->GetPrimitives()[0];
		// Setup prev box data as the primitive index, marked as leaf
		linear[box_id].mNegData = (PrimitiveIndex<<1)|1;
	}
	else
	{
		// Get a new id for negative child
		udword NegID = current_id++;
		// Setup box data
		linear[box_id].mNegData = (udword)&linear[NegID];
		// Make sure it's not marked as leaf
		ASSERT(!(linear[box_id].mNegData&1));
		// Recurse
		_BuildNoLeafTree(linear, NegID, current_id, N);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBCollisionTree::AABBCollisionTree() : mNodes(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBCollisionTree::~AABBCollisionTree()
{
	DELETEARRAY(mNodes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds the collision tree from a generic AABB tree.
 *	\param		tree			[in] generic AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBCollisionTree::Build(AABBTree* tree)
{
	// Checkings
	if(!tree)	return false;
	// Check the input tree is complete
	udword NbTriangles	= tree->GetNbPrimitives();
	udword NbNodes		= tree->GetNbNodes();
	if(NbNodes!=NbTriangles*2-1)	return false;

	// Get nodes
	if(mNbNodes!=NbNodes)	// Same number of nodes => keep moving
	{
		mNbNodes = NbNodes;
		DELETEARRAY(mNodes);
		mNodes = new AABBCollisionNode[mNbNodes];
		CHECKALLOC(mNodes);
	}

	// Build the tree
	udword CurID = 1;
	_BuildCollisionTree(mNodes, 0, CurID, tree);
	ASSERT(CurID==mNbNodes);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the collision tree after vertices have been modified.
 *	\param		mesh_interface	[in] mesh interface for current model
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBCollisionTree::Refit(const MeshInterface* mesh_interface)
{
	ASSERT(!"Not implemented since AABBCollisionTrees have twice as more nodes to refit as AABBNoLeafTrees!");
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Walks the tree and call the user back for each node.
 *	\param		callback	[in] walking callback
 *	\param		user_data	[in] callback's user data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBCollisionTree::Walk(GenericWalkingCallback callback, void* user_data) const
{
	if(!callback)	return false;

	struct Local
	{
		static void _Walk(const AABBCollisionNode* current_node, GenericWalkingCallback callback, void* user_data)
		{
			if(!current_node || !(callback)(current_node, user_data))	return;

			if(!current_node->IsLeaf())
			{
				_Walk(current_node->GetPos(), callback, user_data);
				_Walk(current_node->GetNeg(), callback, user_data);
			}
		}
	};
	Local::_Walk(mNodes, callback, user_data);
	return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBNoLeafTree::AABBNoLeafTree() : mNodes(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBNoLeafTree::~AABBNoLeafTree()
{
	DELETEARRAY(mNodes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds the collision tree from a generic AABB tree.
 *	\param		tree			[in] generic AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBNoLeafTree::Build(AABBTree* tree)
{
	// Checkings
	if(!tree)	return false;
	// Check the input tree is complete
	udword NbTriangles	= tree->GetNbPrimitives();
	udword NbNodes		= tree->GetNbNodes();
	if(NbNodes!=NbTriangles*2-1)	return false;

	// Get nodes
	if(mNbNodes!=NbTriangles-1)	// Same number of nodes => keep moving
	{
		mNbNodes = NbTriangles-1;
		DELETEARRAY(mNodes);
		mNodes = new AABBNoLeafNode[mNbNodes];
		CHECKALLOC(mNodes);
	}

	// Build the tree
	udword CurID = 1;
	_BuildNoLeafTree(mNodes, 0, CurID, tree);
	ASSERT(CurID==mNbNodes);

	return true;
}

inline_ void ComputeMinMax_OT(Point& min, Point& max, const VertexPointers& vp)
{
	// Compute triangle's AABB = a leaf box
#ifdef OPC_USE_FCOMI	// a 15% speedup on my machine, not much
	min.x = FCMin3(vp.Vertex[0]->x, vp.Vertex[1]->x, vp.Vertex[2]->x);
	max.x = FCMax3(vp.Vertex[0]->x, vp.Vertex[1]->x, vp.Vertex[2]->x);

	min.y = FCMin3(vp.Vertex[0]->y, vp.Vertex[1]->y, vp.Vertex[2]->y);
	max.y = FCMax3(vp.Vertex[0]->y, vp.Vertex[1]->y, vp.Vertex[2]->y);

	min.z = FCMin3(vp.Vertex[0]->z, vp.Vertex[1]->z, vp.Vertex[2]->z);
	max.z = FCMax3(vp.Vertex[0]->z, vp.Vertex[1]->z, vp.Vertex[2]->z);
#else
	min = *vp.Vertex[0];
	max = *vp.Vertex[0];
	min.Min(*vp.Vertex[1]);
	max.Max(*vp.Vertex[1]);
	min.Min(*vp.Vertex[2]);
	max.Max(*vp.Vertex[2]);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the collision tree after vertices have been modified.
 *	\param		mesh_interface	[in] mesh interface for current model
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBNoLeafTree::Refit(const MeshInterface* mesh_interface)
{
	// Checkings
	if(!mesh_interface)	return false;

	// Bottom-up update
	VertexPointers VP;
	Point Min,Max;
	Point Min_,Max_;
	udword Index = mNbNodes;
	while(Index--)
	{
		AABBNoLeafNode& Current = mNodes[Index];

		if(Current.HasPosLeaf())
		{
			mesh_interface->GetTriangle(VP, Current.GetPosPrimitive());
			ComputeMinMax_OT(Min, Max, VP);
		}
		else
		{
			const CollisionAABB& CurrentBox = Current.GetPos()->mAABB;
			CurrentBox.GetMin(Min);
			CurrentBox.GetMax(Max);
		}

		if(Current.HasNegLeaf())
		{
			mesh_interface->GetTriangle(VP, Current.GetNegPrimitive());
			ComputeMinMax_OT(Min_, Max_, VP);
		}
		else
		{
			const CollisionAABB& CurrentBox = Current.GetNeg()->mAABB;
			CurrentBox.GetMin(Min_);
			CurrentBox.GetMax(Max_);
		}
#ifdef OPC_USE_FCOMI
		Min.x = FCMin2(Min.x, Min_.x);
		Max.x = FCMax2(Max.x, Max_.x);
		Min.y = FCMin2(Min.y, Min_.y);
		Max.y = FCMax2(Max.y, Max_.y);
		Min.z = FCMin2(Min.z, Min_.z);
		Max.z = FCMax2(Max.z, Max_.z);
#else
		Min.Min(Min_);
		Max.Max(Max_);
#endif
		Current.mAABB.SetMinMax(Min, Max);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Walks the tree and call the user back for each node.
 *	\param		callback	[in] walking callback
 *	\param		user_data	[in] callback's user data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBNoLeafTree::Walk(GenericWalkingCallback callback, void* user_data) const
{
	if(!callback)	return false;

	struct Local
	{
		static void _Walk(const AABBNoLeafNode* current_node, GenericWalkingCallback callback, void* user_data)
		{
			if(!current_node || !(callback)(current_node, user_data))	return;

			if(!current_node->HasPosLeaf())	_Walk(current_node->GetPos(), callback, user_data);
			if(!current_node->HasNegLeaf())	_Walk(current_node->GetNeg(), callback, user_data);
		}
	};
	Local::_Walk(mNodes, callback, user_data);
	return true;
}

// Quantization notes:
// - We could use the highest bits of mData to store some more quantized bits. Dequantization code
//   would be slightly more complex, but number of overlap tests would be reduced (and anyhow those
//   bits are currently wasted). Of course it's not possible if we move to 16 bits mData.
// - Something like "16 bits floats" could be tested, to bypass the int-to-float conversion.
// - A dedicated BV-BV test could be used, dequantizing while testing for overlap. (i.e. it's some
//   lazy-dequantization which may save some work in case of early exits). At the very least some
//   muls could be saved by precomputing several more matrices. But maybe not worth the pain.
// - Do we need to dequantize anyway? Not doing the extents-related muls only implies the box has
//   been scaled, for example.
// - The deeper we move into the hierarchy, the smaller the extents should be. May not need a fixed
//   number of quantization bits. Even better, could probably be best delta-encoded.


// Find max values. Some people asked why I wasn't simply using the first node. Well, I can't.
// I'm not looking for (min, max) values like in a standard AABB, I'm looking for the extremal
// centers/extents in order to quantize them. The first node would only give a single center and
// a single extents. While extents would be the biggest, the center wouldn't.
#define FIND_MAX_VALUES																			\
	/* Get max values */																		\
	Point CMax(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);												\
	Point EMax(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);												\
	for(udword i=0;i<mNbNodes;i++)																\
	{																							\
		if(fabsf(Nodes[i].mAABB.mCenter.x)>CMax.x)	CMax.x = fabsf(Nodes[i].mAABB.mCenter.x);	\
		if(fabsf(Nodes[i].mAABB.mCenter.y)>CMax.y)	CMax.y = fabsf(Nodes[i].mAABB.mCenter.y);	\
		if(fabsf(Nodes[i].mAABB.mCenter.z)>CMax.z)	CMax.z = fabsf(Nodes[i].mAABB.mCenter.z);	\
		if(fabsf(Nodes[i].mAABB.mExtents.x)>EMax.x)	EMax.x = fabsf(Nodes[i].mAABB.mExtents.x);	\
		if(fabsf(Nodes[i].mAABB.mExtents.y)>EMax.y)	EMax.y = fabsf(Nodes[i].mAABB.mExtents.y);	\
		if(fabsf(Nodes[i].mAABB.mExtents.z)>EMax.z)	EMax.z = fabsf(Nodes[i].mAABB.mExtents.z);	\
	}

#define INIT_QUANTIZATION													\
	udword nbc=15;	/* Keep one bit for sign */								\
	udword nbe=15;	/* Keep one bit for fix */								\
	if(!gFixQuantized) nbe++;												\
																			\
	/* Compute quantization coeffs */										\
	Point CQuantCoeff, EQuantCoeff;											\
	CQuantCoeff.x = CMax.x!=0.0f ? float((1<<nbc)-1)/CMax.x : 0.0f;			\
	CQuantCoeff.y = CMax.y!=0.0f ? float((1<<nbc)-1)/CMax.y : 0.0f;			\
	CQuantCoeff.z = CMax.z!=0.0f ? float((1<<nbc)-1)/CMax.z : 0.0f;			\
	EQuantCoeff.x = EMax.x!=0.0f ? float((1<<nbe)-1)/EMax.x : 0.0f;			\
	EQuantCoeff.y = EMax.y!=0.0f ? float((1<<nbe)-1)/EMax.y : 0.0f;			\
	EQuantCoeff.z = EMax.z!=0.0f ? float((1<<nbe)-1)/EMax.z : 0.0f;			\
	/* Compute and save dequantization coeffs */							\
	mCenterCoeff.x = CQuantCoeff.x!=0.0f ? 1.0f / CQuantCoeff.x : 0.0f;		\
	mCenterCoeff.y = CQuantCoeff.y!=0.0f ? 1.0f / CQuantCoeff.y : 0.0f;		\
	mCenterCoeff.z = CQuantCoeff.z!=0.0f ? 1.0f / CQuantCoeff.z : 0.0f;		\
	mExtentsCoeff.x = EQuantCoeff.x!=0.0f ? 1.0f / EQuantCoeff.x : 0.0f;	\
	mExtentsCoeff.y = EQuantCoeff.y!=0.0f ? 1.0f / EQuantCoeff.y : 0.0f;	\
	mExtentsCoeff.z = EQuantCoeff.z!=0.0f ? 1.0f / EQuantCoeff.z : 0.0f;	\

#define PERFORM_QUANTIZATION														\
	/* Quantize */																	\
	mNodes[i].mAABB.mCenter[0] = sword(Nodes[i].mAABB.mCenter.x * CQuantCoeff.x);	\
	mNodes[i].mAABB.mCenter[1] = sword(Nodes[i].mAABB.mCenter.y * CQuantCoeff.y);	\
	mNodes[i].mAABB.mCenter[2] = sword(Nodes[i].mAABB.mCenter.z * CQuantCoeff.z);	\
	mNodes[i].mAABB.mExtents[0] = uword(Nodes[i].mAABB.mExtents.x * EQuantCoeff.x);	\
	mNodes[i].mAABB.mExtents[1] = uword(Nodes[i].mAABB.mExtents.y * EQuantCoeff.y);	\
	mNodes[i].mAABB.mExtents[2] = uword(Nodes[i].mAABB.mExtents.z * EQuantCoeff.z);	\
	/* Fix quantized boxes */														\
	if(gFixQuantized)																\
	{																				\
		/* Make sure the quantized box is still valid */							\
		Point Max = Nodes[i].mAABB.mCenter + Nodes[i].mAABB.mExtents;				\
		Point Min = Nodes[i].mAABB.mCenter - Nodes[i].mAABB.mExtents;				\
		/* For each axis */															\
		for(udword j=0;j<3;j++)														\
		{	/* Dequantize the box center */											\
			float qc = float(mNodes[i].mAABB.mCenter[j]) * mCenterCoeff[j];			\
			bool FixMe=true;														\
			do																		\
			{	/* Dequantize the box extent */										\
				float qe = float(mNodes[i].mAABB.mExtents[j]) * mExtentsCoeff[j];	\
				/* Compare real & dequantized values */								\
				if(qc+qe<Max[j] || qc-qe>Min[j])	mNodes[i].mAABB.mExtents[j]++;	\
				else								FixMe=false;					\
				/* Prevent wrapping */												\
				if(!mNodes[i].mAABB.mExtents[j])									\
				{																	\
					mNodes[i].mAABB.mExtents[j]=0xffff;								\
					FixMe=false;													\
				}																	\
			}while(FixMe);															\
		}																			\
	}

#define REMAP_DATA(member)											\
	/* Fix data */													\
	Data = Nodes[i].member;											\
	if(!(Data&1))													\
	{																\
		/* Compute box number */									\
		udword Nb = (Data - udword(Nodes))/Nodes[i].GetNodeSize();	\
		Data = udword(&mNodes[Nb]);									\
	}																\
	/* ...remapped */												\
	mNodes[i].member = Data;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBQuantizedTree::AABBQuantizedTree() : mNodes(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBQuantizedTree::~AABBQuantizedTree()
{
	DELETEARRAY(mNodes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds the collision tree from a generic AABB tree.
 *	\param		tree			[in] generic AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedTree::Build(AABBTree* tree)
{
	// Checkings
	if(!tree)	return false;
	// Check the input tree is complete
	udword NbTriangles	= tree->GetNbPrimitives();
	udword NbNodes		= tree->GetNbNodes();
	if(NbNodes!=NbTriangles*2-1)	return false;

	// Get nodes
	mNbNodes = NbNodes;
	DELETEARRAY(mNodes);
	AABBCollisionNode* Nodes = new AABBCollisionNode[mNbNodes];
	CHECKALLOC(Nodes);

	// Build the tree
	udword CurID = 1;
	_BuildCollisionTree(Nodes, 0, CurID, tree);

	// Quantize
	{
		mNodes = new AABBQuantizedNode[mNbNodes];
		CHECKALLOC(mNodes);

		// Get max values
		FIND_MAX_VALUES

		// Quantization
		INIT_QUANTIZATION

		// Quantize
		udword Data;
		for(udword i=0;i<mNbNodes;i++)
		{
			PERFORM_QUANTIZATION
			REMAP_DATA(mData)
		}

		DELETEARRAY(Nodes);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the collision tree after vertices have been modified.
 *	\param		mesh_interface	[in] mesh interface for current model
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedTree::Refit(const MeshInterface* mesh_interface)
{
	ASSERT(!"Not implemented since requantizing is painful !");
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Walks the tree and call the user back for each node.
 *	\param		callback	[in] walking callback
 *	\param		user_data	[in] callback's user data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedTree::Walk(GenericWalkingCallback callback, void* user_data) const
{
	if(!callback)	return false;

	struct Local
	{
		static void _Walk(const AABBQuantizedNode* current_node, GenericWalkingCallback callback, void* user_data)
		{
			if(!current_node || !(callback)(current_node, user_data))	return;

			if(!current_node->IsLeaf())
			{
				_Walk(current_node->GetPos(), callback, user_data);
				_Walk(current_node->GetNeg(), callback, user_data);
			}
		}
	};
	Local::_Walk(mNodes, callback, user_data);
	return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBQuantizedNoLeafTree::AABBQuantizedNoLeafTree() : mNodes(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AABBQuantizedNoLeafTree::~AABBQuantizedNoLeafTree()
{
	DELETEARRAY(mNodes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds the collision tree from a generic AABB tree.
 *	\param		tree			[in] generic AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedNoLeafTree::Build(AABBTree* tree)
{
	// Checkings
	if(!tree)	return false;
	// Check the input tree is complete
	udword NbTriangles	= tree->GetNbPrimitives();
	udword NbNodes		= tree->GetNbNodes();
	if(NbNodes!=NbTriangles*2-1)	return false;

	// Get nodes
	mNbNodes = NbTriangles-1;
	DELETEARRAY(mNodes);
	AABBNoLeafNode* Nodes = new AABBNoLeafNode[mNbNodes];
	CHECKALLOC(Nodes);

	// Build the tree
	udword CurID = 1;
	_BuildNoLeafTree(Nodes, 0, CurID, tree);
	ASSERT(CurID==mNbNodes);

	// Quantize
	{
		mNodes = new AABBQuantizedNoLeafNode[mNbNodes];
		CHECKALLOC(mNodes);

		// Get max values
		FIND_MAX_VALUES

		// Quantization
		INIT_QUANTIZATION

		// Quantize
		udword Data;
		for(udword i=0;i<mNbNodes;i++)
		{
			PERFORM_QUANTIZATION
			REMAP_DATA(mPosData)
			REMAP_DATA(mNegData)
		}

		DELETEARRAY(Nodes);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Refits the collision tree after vertices have been modified.
 *	\param		mesh_interface	[in] mesh interface for current model
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedNoLeafTree::Refit(const MeshInterface* mesh_interface)
{
	ASSERT(!"Not implemented since requantizing is painful !");
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Walks the tree and call the user back for each node.
 *	\param		callback	[in] walking callback
 *	\param		user_data	[in] callback's user data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBQuantizedNoLeafTree::Walk(GenericWalkingCallback callback, void* user_data) const
{
	if(!callback)	return false;

	struct Local
	{
		static void _Walk(const AABBQuantizedNoLeafNode* current_node, GenericWalkingCallback callback, void* user_data)
		{
			if(!current_node || !(callback)(current_node, user_data))	return;

			if(!current_node->HasPosLeaf())	_Walk(current_node->GetPos(), callback, user_data);
			if(!current_node->HasNegLeaf())	_Walk(current_node->GetNeg(), callback, user_data);
		}
	};
	Local::_Walk(mNodes, callback, user_data);
	return true;
}
