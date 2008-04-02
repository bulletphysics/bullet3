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
 *	Contains code for an LSS collider.
 *	\file		OPC_LSSCollider.cpp
 *	\author		Pierre Terdiman
 *	\date		December, 28, 2002
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a lss-vs-tree collider.
 *
 *	\class		LSSCollider
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		December, 28, 2002
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

#include "OPC_LSSAABBOverlap.h"
#include "OPC_LSSTriOverlap.h"

#define SET_CONTACT(prim_index, flag)									\
	/* Set contact status */											\
	mFlags |= flag;														\
	mTouchedPrimitives->Add(prim_index);

//! LSS-triangle overlap test
#define LSS_PRIM(prim_index, flag)										\
	/* Request vertices from the app */									\
	VertexPointers VP;	mIMesh->GetTriangle(VP, prim_index);			\
																		\
	/* Perform LSS-tri overlap test */									\
	if(LSSTriOverlap(*VP.Vertex[0], *VP.Vertex[1], *VP.Vertex[2]))		\
	{																	\
		SET_CONTACT(prim_index, flag)									\
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
LSSCollider::LSSCollider()
{
//	mCenter.Zero();
//	mRadius2 = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
LSSCollider::~LSSCollider()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Generic collision query for generic OPCODE models. After the call, access the results:
 *	- with GetContactStatus()
 *	- with GetNbTouchedPrimitives()
 *	- with GetTouchedPrimitives()
 *
 *	\param		cache			[in/out] an lss cache
 *	\param		lss				[in] collision lss in local space
 *	\param		model			[in] Opcode model to collide with
 *	\param		worldl			[in] lss world matrix, or null
 *	\param		worldm			[in] model's world matrix, or null
 *	\return		true if success
 *	\warning	SCALE NOT SUPPORTED. The matrices must contain rotation & translation parts only.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LSSCollider::Collide(LSSCache& cache, const LSS& lss, const Model& model, const Matrix4x4* worldl, const Matrix4x4* worldm)
{
	// Checkings
	if(!Setup(&model))	return false;

	// Init collision query
	if(InitQuery(cache, lss, worldl, worldm))	return true;

	if(!model.HasLeafNodes())
	{
		if(model.IsQuantized())
		{
			const AABBQuantizedNoLeafTree* Tree = (const AABBQuantizedNoLeafTree*)model.GetTree();

			// Setup dequantization coeffs
			mCenterCoeff	= Tree->mCenterCoeff;
			mExtentsCoeff	= Tree->mExtentsCoeff;

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes());
			else						_Collide(Tree->GetNodes());
		}
		else
		{
			const AABBNoLeafTree* Tree = (const AABBNoLeafTree*)model.GetTree();

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes());
			else						_Collide(Tree->GetNodes());
		}
	}
	else
	{
		if(model.IsQuantized())
		{
			const AABBQuantizedTree* Tree = (const AABBQuantizedTree*)model.GetTree();

			// Setup dequantization coeffs
			mCenterCoeff	= Tree->mCenterCoeff;
			mExtentsCoeff	= Tree->mExtentsCoeff;

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes());
			else						_Collide(Tree->GetNodes());
		}
		else
		{
			const AABBCollisionTree* Tree = (const AABBCollisionTree*)model.GetTree();

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes());
			else						_Collide(Tree->GetNodes());
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes a collision query :
 *	- reset stats & contact status
 *	- setup matrices
 *	- check temporal coherence
 *
 *	\param		cache		[in/out] an lss cache
 *	\param		lss			[in] lss in local space
 *	\param		worldl		[in] lss world matrix, or null
 *	\param		worldm		[in] model's world matrix, or null
 *	\return		TRUE if we can return immediately
 *	\warning	SCALE NOT SUPPORTED. The matrices must contain rotation & translation parts only.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL LSSCollider::InitQuery(LSSCache& cache, const LSS& lss, const Matrix4x4* worldl, const Matrix4x4* worldm)
{
	// 1) Call the base method
	VolumeCollider::InitQuery();

	// 2) Compute LSS in model space:
	// - Precompute R^2
	mRadius2 = lss.mRadius * lss.mRadius;
	// - Compute segment
	mSeg.mP0 = lss.mP0;
	mSeg.mP1 = lss.mP1;
	// -> to world space
	if(worldl)
	{
		mSeg.mP0 *= *worldl;
		mSeg.mP1 *= *worldl;
	}
	// -> to model space
	if(worldm)
	{
		// Invert model matrix
		Matrix4x4 InvWorldM;
		InvertPRMatrix(InvWorldM, *worldm);

		mSeg.mP0 *= InvWorldM;
		mSeg.mP1 *= InvWorldM;
	}

	// 3) Setup destination pointer
	mTouchedPrimitives = &cache.TouchedPrimitives;

	// 4) Special case: 1-triangle meshes [Opcode 1.3]
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		if(!SkipPrimitiveTests())
		{
			// We simply perform the BV-Prim overlap test each time. We assume single triangle has index 0.
			mTouchedPrimitives->Reset();

			// Perform overlap test between the unique triangle and the LSS (and set contact status if needed)
			LSS_PRIM(udword(0), OPC_CONTACT)

			// Return immediately regardless of status
			return TRUE;
		}
	}

	// 5) Check temporal coherence :
	if(TemporalCoherenceEnabled())
	{
		// Here we use temporal coherence
		// => check results from previous frame before performing the collision query
		if(FirstContactEnabled())
		{
			// We're only interested in the first contact found => test the unique previously touched face
			if(mTouchedPrimitives->GetNbEntries())
			{
				// Get index of previously touched face = the first entry in the array
				udword PreviouslyTouchedFace = mTouchedPrimitives->GetEntry(0);

				// Then reset the array:
				// - if the overlap test below is successful, the index we'll get added back anyway
				// - if it isn't, then the array should be reset anyway for the normal query
				mTouchedPrimitives->Reset();

				// Perform overlap test between the cached triangle and the LSS (and set contact status if needed)
				LSS_PRIM(PreviouslyTouchedFace, OPC_TEMPORAL_CONTACT)

				// Return immediately if possible
				if(GetContactStatus())	return TRUE;
			}
			// else no face has been touched during previous query
			// => we'll have to perform a normal query
		}
		else
		{
			// We're interested in all contacts =>test the new real LSS N(ew) against the previous fat LSS P(revious):

			// ### rewrite this

			LSS Test(mSeg, lss.mRadius);	// in model space
			LSS Previous(cache.Previous, sqrtf(cache.Previous.mRadius));

//			if(cache.Previous.Contains(Test))
			if(IsCacheValid(cache) && Previous.Contains(Test))
			{
				// - if N is included in P, return previous list
				// => we simply leave the list (mTouchedFaces) unchanged

				// Set contact status if needed
				if(mTouchedPrimitives->GetNbEntries())	mFlags |= OPC_TEMPORAL_CONTACT;

				// In any case we don't need to do a query
				return TRUE;
			}
			else
			{
				// - else do the query using a fat N

				// Reset cache since we'll about to perform a real query
				mTouchedPrimitives->Reset();

				// Make a fat sphere so that coherence will work for subsequent frames
				mRadius2 *= cache.FatCoeff;
//				mRadius2 = (lss.mRadius * cache.FatCoeff)*(lss.mRadius * cache.FatCoeff);


				// Update cache with query data (signature for cached faces)
				cache.Previous.mP0 = mSeg.mP0;
				cache.Previous.mP1 = mSeg.mP1;
				cache.Previous.mRadius = mRadius2;
			}
		}
	}
	else
	{
		// Here we don't use temporal coherence => do a normal query
		mTouchedPrimitives->Reset();
	}

	return FALSE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Collision query for vanilla AABB trees.
 *	\param		cache		[in/out] an lss cache
 *	\param		lss			[in] collision lss in world space
 *	\param		tree		[in] AABB tree
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LSSCollider::Collide(LSSCache& cache, const LSS& lss, const AABBTree* tree)
{
	// This is typically called for a scene tree, full of -AABBs-, not full of triangles.
	// So we don't really have "primitives" to deal with. Hence it doesn't work with
	// "FirstContact" + "TemporalCoherence".
	ASSERT( !(FirstContactEnabled() && TemporalCoherenceEnabled()) );

	// Checkings
	if(!tree)	return false;

	// Init collision query
	if(InitQuery(cache, lss))	return true;

	// Perform collision query
	_Collide(tree);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks the LSS completely contains the box. In which case we can end the query sooner.
 *	\param		bc	[in] box center
 *	\param		be	[in] box extents
 *	\return		true if the LSS contains the whole box
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL LSSCollider::LSSContainsBox(const Point& bc, const Point& be)
{
	// Not implemented
	return FALSE;
}

#define TEST_BOX_IN_LSS(center, extents)	\
	if(LSSContainsBox(center, extents))		\
	{										\
		/* Set contact status */			\
		mFlags |= OPC_CONTACT;				\
		_Dump(node);						\
		return;								\
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for normal AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_Collide(const AABBCollisionNode* node)
{
	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents))	return;

	TEST_BOX_IN_LSS(node->mAABB.mCenter, node->mAABB.mExtents)

	if(node->IsLeaf())
	{
		LSS_PRIM(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_Collide(node->GetPos());

		if(ContactFound()) return;

		_Collide(node->GetNeg());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for normal AABB trees, without primitive tests.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_CollideNoPrimitiveTest(const AABBCollisionNode* node)
{
	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents))	return;

	TEST_BOX_IN_LSS(node->mAABB.mCenter, node->mAABB.mExtents)

	if(node->IsLeaf())
	{
		SET_CONTACT(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_CollideNoPrimitiveTest(node->GetPos());

		if(ContactFound()) return;

		_CollideNoPrimitiveTest(node->GetNeg());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_Collide(const AABBQuantizedNode* node)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(Center, Extents))	return;

	TEST_BOX_IN_LSS(Center, Extents)

	if(node->IsLeaf())
	{
		LSS_PRIM(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_Collide(node->GetPos());

		if(ContactFound()) return;

		_Collide(node->GetNeg());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized AABB trees, without primitive tests.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_CollideNoPrimitiveTest(const AABBQuantizedNode* node)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(Center, Extents))	return;

	TEST_BOX_IN_LSS(Center, Extents)

	if(node->IsLeaf())
	{
		SET_CONTACT(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_CollideNoPrimitiveTest(node->GetPos());

		if(ContactFound()) return;

		_CollideNoPrimitiveTest(node->GetNeg());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_Collide(const AABBNoLeafNode* node)
{
	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents))	return;

	TEST_BOX_IN_LSS(node->mAABB.mCenter, node->mAABB.mExtents)

	if(node->HasPosLeaf())	{ LSS_PRIM(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetPos());

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ LSS_PRIM(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetNeg());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for no-leaf AABB trees, without primitive tests.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_CollideNoPrimitiveTest(const AABBNoLeafNode* node)
{
	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents))	return;

	TEST_BOX_IN_LSS(node->mAABB.mCenter, node->mAABB.mExtents)

	if(node->HasPosLeaf())	{ SET_CONTACT(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetPos());

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ SET_CONTACT(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetNeg());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_Collide(const AABBQuantizedNoLeafNode* node)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(Center, Extents))	return;

	TEST_BOX_IN_LSS(Center, Extents)

	if(node->HasPosLeaf())	{ LSS_PRIM(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetPos());

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ LSS_PRIM(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetNeg());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized no-leaf AABB trees, without primitive tests.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_CollideNoPrimitiveTest(const AABBQuantizedNoLeafNode* node)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Perform LSS-AABB overlap test
	if(!LSSAABBOverlap(Center, Extents))	return;

	TEST_BOX_IN_LSS(Center, Extents)

	if(node->HasPosLeaf())	{ SET_CONTACT(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetPos());

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ SET_CONTACT(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetNeg());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for vanilla AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LSSCollider::_Collide(const AABBTreeNode* node)
{
	// Perform LSS-AABB overlap test
	Point Center, Extents;
	node->GetAABB()->GetCenter(Center);
	node->GetAABB()->GetExtents(Extents);
	if(!LSSAABBOverlap(Center, Extents))	return;

	if(node->IsLeaf() || LSSContainsBox(Center, Extents))
	{
		mFlags |= OPC_CONTACT;
		mTouchedPrimitives->Add(node->GetPrimitives(), node->GetNbPrimitives());
	}
	else
	{
		_Collide(node->GetPos());
		_Collide(node->GetNeg());
	}
}






///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridLSSCollider::HybridLSSCollider()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridLSSCollider::~HybridLSSCollider()
{
}

bool HybridLSSCollider::Collide(LSSCache& cache, const LSS& lss, const HybridModel& model, const Matrix4x4* worldl, const Matrix4x4* worldm)
{
	// We don't want primitive tests here!
	mFlags |= OPC_NO_PRIMITIVE_TESTS;

	// Checkings
	if(!Setup(&model))	return false;

	// Init collision query
	if(InitQuery(cache, lss, worldl, worldm))	return true;

	// Special case for 1-leaf trees
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		// Here we're supposed to perform a normal query, except our tree has a single node, i.e. just a few triangles
		udword Nb = mIMesh->GetNbTriangles();

		// Loop through all triangles
		for(udword i=0;i<Nb;i++)
		{
			LSS_PRIM(i, OPC_CONTACT)
		}
		return true;
	}

	// Override destination array since we're only going to get leaf boxes here
	mTouchedBoxes.Reset();
	mTouchedPrimitives = &mTouchedBoxes;

	// Now, do the actual query against leaf boxes
	if(!model.HasLeafNodes())
	{
		if(model.IsQuantized())
		{
			const AABBQuantizedNoLeafTree* Tree = (const AABBQuantizedNoLeafTree*)model.GetTree();

			// Setup dequantization coeffs
			mCenterCoeff	= Tree->mCenterCoeff;
			mExtentsCoeff	= Tree->mExtentsCoeff;

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes());
		}
		else
		{
			const AABBNoLeafTree* Tree = (const AABBNoLeafTree*)model.GetTree();

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes());
		}
	}
	else
	{
		if(model.IsQuantized())
		{
			const AABBQuantizedTree* Tree = (const AABBQuantizedTree*)model.GetTree();

			// Setup dequantization coeffs
			mCenterCoeff	= Tree->mCenterCoeff;
			mExtentsCoeff	= Tree->mExtentsCoeff;

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes());
		}
		else
		{
			const AABBCollisionTree* Tree = (const AABBCollisionTree*)model.GetTree();

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes());
		}
	}

	// We only have a list of boxes so far
	if(GetContactStatus())
	{
		// Reset contact status, since it currently only reflects collisions with leaf boxes
		Collider::InitQuery();

		// Change dest container so that we can use built-in overlap tests and get collided primitives
		cache.TouchedPrimitives.Reset();
		mTouchedPrimitives = &cache.TouchedPrimitives;

		// Read touched leaf boxes
		udword Nb = mTouchedBoxes.GetNbEntries();
		const udword* Touched = mTouchedBoxes.GetEntries();

		const LeafTriangles* LT = model.GetLeafTriangles();
		const udword* Indices = model.GetIndices();

		// Loop through touched leaves
		while(Nb--)
		{
			const LeafTriangles& CurrentLeaf = LT[*Touched++];

			// Each leaf box has a set of triangles
			udword NbTris = CurrentLeaf.GetNbTriangles();
			if(Indices)
			{
				const udword* T = &Indices[CurrentLeaf.GetTriangleIndex()];

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					udword TriangleIndex = *T++;
					LSS_PRIM(TriangleIndex, OPC_CONTACT)
				}
			}
			else
			{
				udword BaseIndex = CurrentLeaf.GetTriangleIndex();

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					udword TriangleIndex = BaseIndex++;
					LSS_PRIM(TriangleIndex, OPC_CONTACT)
				}
			}
		}
	}

	return true;
}
