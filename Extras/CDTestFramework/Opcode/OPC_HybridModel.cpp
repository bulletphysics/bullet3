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
 *	Contains code for hybrid models.
 *	\file		OPC_HybridModel.cpp
 *	\author		Pierre Terdiman
 *	\date		May, 18, 2003
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	An hybrid collision model.
 *	
 *	The problem :
 *	
 *	Opcode really shines for mesh-mesh collision, especially when meshes are deeply overlapping
 *	(it typically outperforms RAPID in those cases).
 *	
 *	Unfortunately this is not the typical scenario in games.
 *	
 *	For close-proximity cases, especially for volume-mesh queries, it's relatively easy to run faster
 *	than Opcode, that suffers from a relatively high setup time.
 *	
 *	In particular, Opcode's "vanilla" trees in those cases -can- run faster. They can also use -less-
 *	memory than the optimized ones, when you let the system stop at ~10 triangles / leaf for example
 *	(i.e. when you don't use "complete" trees). However, those trees tend to fragment memory quite a
 *	lot, increasing cache misses : since they're not "complete", we can't predict the final number of
 *	nodes and we have to allocate nodes on-the-fly. For the same reasons we can't use Opcode's "optimized"
 *	trees here, since they rely on a known layout to perform the "optimization".
 *	
 *	Hybrid trees :
 *	
 *	Hybrid trees try to combine best of both worlds :
 *	
 *	- they use a maximum limit of 16 triangles/leaf. "16" is used so that we'll be able to save the
 *	number of triangles using 4 bits only.
 *	
 *	- they're still "complete" trees thanks to a two-passes building phase. First we create a "vanilla"
 *	AABB-tree with Opcode, limited to 16 triangles/leaf. Then we create a *second* vanilla tree, this
 *	time using the leaves of the first one. The trick is : this second tree is now "complete"... so we
 *	can further transform it into an Opcode's optimized tree.
 *	
 *	- then we run the collision queries on that standard Opcode tree. The only difference is that leaf
 *	nodes contain indices to leaf nodes of another tree. Also, we have to skip all primitive tests in
 *	Opcode optimized trees, since our leaves don't contain triangles anymore.
 *	
 *	- finally, for each collided leaf, we simply loop through 16 triangles max, and collide them with
 *	the bounding volume used in the query (we only support volume-vs-mesh queries here, not mesh-vs-mesh)
 *	
 *	All of that is wrapped in this "hybrid model" that contains the minimal data required for this to work.
 *	It's a mix between old "vanilla" trees, and old "optimized" trees.
 *
 *	Extra advantages:
 *
 *	- If we use them for dynamic models, we're left with a very small number of leaf nodes to refit. It
 *	might be a bit faster since we have less nodes to write back.
 *
 *	- In rigid body simulation, using temporal coherence and sleeping objects greatly reduce the actual
 *	influence of one tree over another (i.e. the speed difference is often invisible). So memory is really
 *	the key element to consider, and in this regard hybrid trees are just better.
 *	
 *	Information to take home:
 *	- they use less ram
 *	- they're not slower (they're faster or slower depending on cases, overall there's no significant
 *	difference *as long as objects don't interpenetrate too much* - in which case Opcode's optimized trees
 *	are still notably faster)
 *
 *	\class		HybridModel
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		May, 18, 2003
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridModel::HybridModel() :
	mNbLeaves		(0),
	mNbPrimitives	(0),
	mTriangles		(null),
	mIndices		(null)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridModel::~HybridModel()
{
	Release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Releases everything.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HybridModel::Release()
{
	ReleaseBase();
	DELETEARRAY(mIndices);
	DELETEARRAY(mTriangles);
	mNbLeaves		= 0;
	mNbPrimitives	= 0;
}

	struct Internal
	{
		Internal()
		{
			mNbLeaves	= 0;
			mLeaves		= null;
			mTriangles	= null;
			mBase		= null;
		}
		~Internal()
		{
			DELETEARRAY(mLeaves);
		}

		udword			mNbLeaves;
		AABB*			mLeaves;
		LeafTriangles*	mTriangles;
		const udword*	mBase;
	};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Builds a collision model.
 *	\param		create		[in] model creation structure
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Build(const OPCODECREATE& create)
{
	// 1) Checkings
	if(!create.mIMesh || !create.mIMesh->IsValid())	return false;

	// Look for degenerate faces.
	udword NbDegenerate = create.mIMesh->CheckTopology();
	if(NbDegenerate)	Log("OPCODE WARNING: found %d degenerate faces in model! Collision might report wrong results!\n", NbDegenerate);
	// We continue nonetheless.... 

	Release();	// Make sure previous tree has been discarded

	// 1-1) Setup mesh interface automatically
	SetMeshInterface(create.mIMesh);

	bool Status = false;
	AABBTree* LeafTree = null;
	Internal Data;

	// 2) Build a generic AABB Tree.
	mSource = new AABBTree;
	CHECKALLOC(mSource);

	// 2-1) Setup a builder. Our primitives here are triangles from input mesh,
	// so we use an AABBTreeOfTrianglesBuilder.....
	{
		AABBTreeOfTrianglesBuilder TB;
		TB.mIMesh			= create.mIMesh;
		TB.mNbPrimitives	= create.mIMesh->GetNbTriangles();
		TB.mSettings		= create.mSettings;
		TB.mSettings.mLimit	= 16;	// ### Hardcoded, but maybe we could let the user choose 8 / 16 / 32 ...
		if(!mSource->Build(&TB))	goto FreeAndExit;
	}

	// 2-2) Here's the trick : create *another* AABB tree using the leaves of the first one (which are boxes, this time)
	struct Local
	{
		// A callback to count leaf nodes
		static bool CountLeaves(const AABBTreeNode* current, udword depth, void* user_data)
		{
			if(current->IsLeaf())
			{
				Internal* Data = (Internal*)user_data;
				Data->mNbLeaves++;
			}
			return true;
		}

		// A callback to setup leaf nodes in our internal structures
		static bool SetupLeafData(const AABBTreeNode* current, udword depth, void* user_data)
		{
			if(current->IsLeaf())
			{
				Internal* Data = (Internal*)user_data;

				// Get current leaf's box
				Data->mLeaves[Data->mNbLeaves] = *current->GetAABB();

				// Setup leaf data
				udword Index = (udword(current->GetPrimitives()) - udword(Data->mBase))/sizeof(udword);
				Data->mTriangles[Data->mNbLeaves].SetData(current->GetNbPrimitives(), Index);

				Data->mNbLeaves++;
			}
			return true;
		}
	};

	// Walk the tree & count number of leaves
	Data.mNbLeaves = 0;
	mSource->Walk(Local::CountLeaves, &Data);
	mNbLeaves = Data.mNbLeaves;	// Keep track of it

	// Special case for 1-leaf meshes
	if(mNbLeaves==1)
	{
		mModelCode |= OPC_SINGLE_NODE;
		Status = true;
		goto FreeAndExit;
	}

	// Allocate our structures
	Data.mLeaves = new AABB[Data.mNbLeaves];		CHECKALLOC(Data.mLeaves);
	mTriangles = new LeafTriangles[Data.mNbLeaves];	CHECKALLOC(mTriangles);

	// Walk the tree again & setup leaf data
	Data.mTriangles	= mTriangles;
	Data.mBase		= mSource->GetIndices();
	Data.mNbLeaves	= 0;	// Reset for incoming walk
	mSource->Walk(Local::SetupLeafData, &Data);

	// Handle source indices
	{
		bool MustKeepIndices = true;
		if(create.mCanRemap)
		{
			// We try to get rid of source indices (saving more ram!) by reorganizing triangle arrays...
			// Remap can fail when we use callbacks => keep track of indices in that case (it still
			// works, only using more memory)
			if(create.mIMesh->RemapClient(mSource->GetNbPrimitives(), mSource->GetIndices()))
			{
				MustKeepIndices = false;
			}
		}

		if(MustKeepIndices)
		{
			// Keep track of source indices (from vanilla tree)
			mNbPrimitives = mSource->GetNbPrimitives();
			mIndices = new udword[mNbPrimitives];
			CopyMemory(mIndices, mSource->GetIndices(), mNbPrimitives*sizeof(udword));
		}
	}

	// Now, create our optimized tree using previous leaf nodes
	LeafTree = new AABBTree;
	CHECKALLOC(LeafTree);
	{
		AABBTreeOfAABBsBuilder TB;	// Now using boxes !
		TB.mSettings		= create.mSettings;
		TB.mSettings.mLimit	= 1;	// We now want a complete tree so that we can "optimize" it
		TB.mNbPrimitives	= Data.mNbLeaves;
		TB.mAABBArray		= Data.mLeaves;
		if(!LeafTree->Build(&TB))	goto FreeAndExit;
	}

	// 3) Create an optimized tree according to user-settings
	if(!CreateTree(create.mNoLeaf, create.mQuantized))	goto FreeAndExit;

	// 3-2) Create optimized tree
	if(!mTree->Build(LeafTree))	goto FreeAndExit;

	// Finally ok...
	Status = true;

FreeAndExit:	// Allow me this one...
	DELETESINGLE(LeafTree);

	// 3-3) Delete generic tree if needed
	if(!create.mKeepOriginal)	DELETESINGLE(mSource);

	return Status;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Gets the number of bytes used by the tree.
 *	\return		amount of bytes used
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
udword HybridModel::GetUsedBytes() const
{
	udword UsedBytes = 0;
	if(mTree)		UsedBytes += mTree->GetUsedBytes();
	if(mIndices)	UsedBytes += mNbPrimitives * sizeof(udword);	// mIndices
	if(mTriangles)	UsedBytes += mNbLeaves * sizeof(LeafTriangles);	// mTriangles
	return UsedBytes;
}

inline_ void ComputeMinMax_HM(Point& min, Point& max, const VertexPointers& vp)
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
 *	Refits the collision model. This can be used to handle dynamic meshes. Usage is:
 *	1. modify your mesh vertices (keep the topology constant!)
 *	2. refit the tree (call this method)
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool HybridModel::Refit()
{
	if(!mIMesh)	return false;
	if(!mTree)	return false;

	if(IsQuantized())	return false;
	if(HasLeafNodes())	return false;

	const LeafTriangles* LT = GetLeafTriangles();
	const udword* Indices = GetIndices();

	// Bottom-up update
	VertexPointers VP;
	Point Min,Max;
	Point Min_,Max_;
	udword Index = mTree->GetNbNodes();
	AABBNoLeafNode* Nodes = (AABBNoLeafNode*)((AABBNoLeafTree*)mTree)->GetNodes();
	while(Index--)
	{
		AABBNoLeafNode& Current = Nodes[Index];

		if(Current.HasPosLeaf())
		{
			const LeafTriangles& CurrentLeaf = LT[Current.GetPosPrimitive()];

			Min.SetPlusInfinity();
			Max.SetMinusInfinity();

			Point TmpMin, TmpMax;

			// Each leaf box has a set of triangles
			udword NbTris = CurrentLeaf.GetNbTriangles();
			if(Indices)
			{
				const udword* T = &Indices[CurrentLeaf.GetTriangleIndex()];

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					mIMesh->GetTriangle(VP, *T++);
					ComputeMinMax_HM(TmpMin, TmpMax, VP);
					Min.Min(TmpMin);
					Max.Max(TmpMax);
				}
			}
			else
			{
				udword BaseIndex = CurrentLeaf.GetTriangleIndex();

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					mIMesh->GetTriangle(VP, BaseIndex++);
					ComputeMinMax_HM(TmpMin, TmpMax, VP);
					Min.Min(TmpMin);
					Max.Max(TmpMax);
				}
			}
		}
		else
		{
			const CollisionAABB& CurrentBox = Current.GetPos()->mAABB;
			CurrentBox.GetMin(Min);
			CurrentBox.GetMax(Max);
		}

		if(Current.HasNegLeaf())
		{
			const LeafTriangles& CurrentLeaf = LT[Current.GetNegPrimitive()];

			Min_.SetPlusInfinity();
			Max_.SetMinusInfinity();

			Point TmpMin, TmpMax;

			// Each leaf box has a set of triangles
			udword NbTris = CurrentLeaf.GetNbTriangles();
			if(Indices)
			{
				const udword* T = &Indices[CurrentLeaf.GetTriangleIndex()];

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					mIMesh->GetTriangle(VP, *T++);
					ComputeMinMax_HM(TmpMin, TmpMax, VP);
					Min_.Min(TmpMin);
					Max_.Max(TmpMax);
				}
			}
			else
			{
				udword BaseIndex = CurrentLeaf.GetTriangleIndex();

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					mIMesh->GetTriangle(VP, BaseIndex++);
					ComputeMinMax_HM(TmpMin, TmpMax, VP);
					Min_.Min(TmpMin);
					Max_.Max(TmpMax);
				}
			}
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
