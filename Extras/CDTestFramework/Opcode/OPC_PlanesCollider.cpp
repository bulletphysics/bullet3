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
 *	Contains code for a planes collider.
 *	\file		OPC_PlanesCollider.cpp
 *	\author		Pierre Terdiman
 *	\date		January, 1st, 2002
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a Planes-vs-tree collider.
 *
 *	\class		PlanesCollider
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		January, 1st, 2002
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

#include "OPC_PlanesAABBOverlap.h"
#include "OPC_PlanesTriOverlap.h"

#define SET_CONTACT(prim_index, flag)		\
	/* Set contact status */				\
	mFlags |= flag;							\
	mTouchedPrimitives->Add(prim_index);

//! Planes-triangle test
#define PLANES_PRIM(prim_index, flag)		\
	/* Request vertices from the app */		\
	mIMesh->GetTriangle(mVP, prim_index);	\
	/* Perform triangle-box overlap test */	\
	if(PlanesTriOverlap(clip_mask))			\
	{										\
		SET_CONTACT(prim_index, flag)		\
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PlanesCollider::PlanesCollider() :
	mPlanes		(null),
	mNbPlanes	(0)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PlanesCollider::~PlanesCollider()
{
	DELETEARRAY(mPlanes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Validates current settings. You should call this method after all the settings and callbacks have been defined.
 *	\return		null if everything is ok, else a string describing the problem
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* PlanesCollider::ValidateSettings()
{
	if(TemporalCoherenceEnabled() && !FirstContactEnabled())	return "Temporal coherence only works with ""First contact"" mode!";

	return VolumeCollider::ValidateSettings();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Generic collision query for generic OPCODE models. After the call, access the results:
 *	- with GetContactStatus()
 *	- with GetNbTouchedPrimitives()
 *	- with GetTouchedPrimitives()
 *
 *	\param		cache		[in/out] a planes cache
 *	\param		planes		[in] list of planes in world space
 *	\param		nb_planes	[in] number of planes
 *	\param		model		[in] Opcode model to collide with
 *	\param		worldm		[in] model's world matrix, or null
 *	\return		true if success
 *	\warning	SCALE NOT SUPPORTED. The matrices must contain rotation & translation parts only.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PlanesCollider::Collide(PlanesCache& cache, const Plane* planes, udword nb_planes, const Model& model, const Matrix4x4* worldm)
{
	// Checkings
	if(!Setup(&model))	return false;

	// Init collision query
	if(InitQuery(cache, planes, nb_planes, worldm))	return true;

	udword PlaneMask = (1<<nb_planes)-1;

	if(!model.HasLeafNodes())
	{
		if(model.IsQuantized())
		{
			const AABBQuantizedNoLeafTree* Tree = (const AABBQuantizedNoLeafTree*)model.GetTree();

			// Setup dequantization coeffs
			mCenterCoeff	= Tree->mCenterCoeff;
			mExtentsCoeff	= Tree->mExtentsCoeff;

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
			else						_Collide(Tree->GetNodes(), PlaneMask);
		}
		else
		{
			const AABBNoLeafTree* Tree = (const AABBNoLeafTree*)model.GetTree();

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
			else						_Collide(Tree->GetNodes(), PlaneMask);
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
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
			else						_Collide(Tree->GetNodes(), PlaneMask);
		}
		else
		{
			const AABBCollisionTree* Tree = (const AABBCollisionTree*)model.GetTree();

			// Perform collision query
			if(SkipPrimitiveTests())	_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
			else						_Collide(Tree->GetNodes(), PlaneMask);
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes a collision query :
 *	- reset stats & contact status
 *	- compute planes in model space
 *	- check temporal coherence
 *
 *	\param		cache		[in/out] a planes cache
 *	\param		planes		[in] list of planes
 *	\param		nb_planes	[in] number of planes
 *	\param		worldm		[in] model's world matrix, or null
 *	\return		TRUE if we can return immediately
 *	\warning	SCALE NOT SUPPORTED. The matrix must contain rotation & translation parts only.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL PlanesCollider::InitQuery(PlanesCache& cache, const Plane* planes, udword nb_planes, const Matrix4x4* worldm)
{
	// 1) Call the base method
	VolumeCollider::InitQuery();

	// 2) Compute planes in model space
	if(nb_planes>mNbPlanes)
	{
		DELETEARRAY(mPlanes);
		mPlanes = new Plane[nb_planes];
	}
	mNbPlanes = nb_planes;

	if(worldm)
	{
		Matrix4x4 InvWorldM;
		InvertPRMatrix(InvWorldM, *worldm);

//		for(udword i=0;i<nb_planes;i++)	mPlanes[i] = planes[i] * InvWorldM;
		for(udword i=0;i<nb_planes;i++)	TransformPlane(mPlanes[i], planes[i], InvWorldM);
	}
	else CopyMemory(mPlanes, planes, nb_planes*sizeof(Plane));

	// 3) Setup destination pointer
	mTouchedPrimitives = &cache.TouchedPrimitives;

	// 4) Special case: 1-triangle meshes [Opcode 1.3]
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		if(!SkipPrimitiveTests())
		{
			// We simply perform the BV-Prim overlap test each time. We assume single triangle has index 0.
			mTouchedPrimitives->Reset();

			// Perform overlap test between the unique triangle and the planes (and set contact status if needed)
			udword clip_mask = (1<<mNbPlanes)-1;
			PLANES_PRIM(udword(0), OPC_CONTACT)

			// Return immediately regardless of status
			return TRUE;
		}
	}

	// 4) Check temporal coherence:
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

				// Perform overlap test between the cached triangle and the planes (and set contact status if needed)
				udword clip_mask = (1<<mNbPlanes)-1;
				PLANES_PRIM(PreviouslyTouchedFace, OPC_TEMPORAL_CONTACT)

				// Return immediately if possible
				if(GetContactStatus())	return TRUE;
			}
			// else no face has been touched during previous query
			// => we'll have to perform a normal query
		}
		else mTouchedPrimitives->Reset();
	}
	else
	{
		// Here we don't use temporal coherence => do a normal query
		mTouchedPrimitives->Reset();
	}

	return FALSE;
}

#define TEST_CLIP_MASK																					\
	/* If the box is completely included, so are its children. We don't need to do extra tests, we */	\
	/* can immediately output a list of visible children. Those ones won't need to be clipped. */		\
	if(!OutClipMask)																					\
	{																									\
		/* Set contact status */																		\
		mFlags |= OPC_CONTACT;																			\
		_Dump(node);																					\
		return;																							\
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for normal AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_Collide(const AABBCollisionNode* node, udword clip_mask)
{
	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->IsLeaf())
	{
		PLANES_PRIM(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_Collide(node->GetPos(), OutClipMask);

		if(ContactFound()) return;

		_Collide(node->GetNeg(), OutClipMask);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for normal AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_CollideNoPrimitiveTest(const AABBCollisionNode* node, udword clip_mask)
{
	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->IsLeaf())
	{
		SET_CONTACT(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_CollideNoPrimitiveTest(node->GetPos(), OutClipMask);

		if(ContactFound()) return;

		_CollideNoPrimitiveTest(node->GetNeg(), OutClipMask);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_Collide(const AABBQuantizedNode* node, udword clip_mask)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(Center, Extents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->IsLeaf())
	{
		PLANES_PRIM(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_Collide(node->GetPos(), OutClipMask);

		if(ContactFound()) return;

		_Collide(node->GetNeg(), OutClipMask);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_CollideNoPrimitiveTest(const AABBQuantizedNode* node, udword clip_mask)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(Center, Extents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->IsLeaf())
	{
		SET_CONTACT(node->GetPrimitive(), OPC_CONTACT)
	}
	else
	{
		_CollideNoPrimitiveTest(node->GetPos(), OutClipMask);

		if(ContactFound()) return;

		_CollideNoPrimitiveTest(node->GetNeg(), OutClipMask);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_Collide(const AABBNoLeafNode* node, udword clip_mask)
{
	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->HasPosLeaf())	{ PLANES_PRIM(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetPos(), OutClipMask);

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ PLANES_PRIM(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetNeg(), OutClipMask);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_CollideNoPrimitiveTest(const AABBNoLeafNode* node, udword clip_mask)
{
	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(node->mAABB.mCenter, node->mAABB.mExtents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->HasPosLeaf())	{ SET_CONTACT(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetPos(), OutClipMask);

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ SET_CONTACT(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetNeg(), OutClipMask);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_Collide(const AABBQuantizedNoLeafNode* node, udword clip_mask)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(Center, Extents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->HasPosLeaf())	{ PLANES_PRIM(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetPos(), OutClipMask);

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ PLANES_PRIM(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_Collide(node->GetNeg(), OutClipMask);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive collision query for quantized no-leaf AABB trees.
 *	\param		node	[in] current collision node
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PlanesCollider::_CollideNoPrimitiveTest(const AABBQuantizedNoLeafNode* node, udword clip_mask)
{
	// Dequantize box
	const QuantizedAABB& Box = node->mAABB;
	const Point Center(float(Box.mCenter[0]) * mCenterCoeff.x, float(Box.mCenter[1]) * mCenterCoeff.y, float(Box.mCenter[2]) * mCenterCoeff.z);
	const Point Extents(float(Box.mExtents[0]) * mExtentsCoeff.x, float(Box.mExtents[1]) * mExtentsCoeff.y, float(Box.mExtents[2]) * mExtentsCoeff.z);

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
	udword OutClipMask;
	if(!PlanesAABBOverlap(Center, Extents, OutClipMask, clip_mask))	return;

	TEST_CLIP_MASK

	// Else the box straddles one or several planes, so we need to recurse down the tree.
	if(node->HasPosLeaf())	{ SET_CONTACT(node->GetPosPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetPos(), OutClipMask);

	if(ContactFound()) return;

	if(node->HasNegLeaf())	{ SET_CONTACT(node->GetNegPrimitive(), OPC_CONTACT) }
	else					_CollideNoPrimitiveTest(node->GetNeg(), OutClipMask);
}







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridPlanesCollider::HybridPlanesCollider()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HybridPlanesCollider::~HybridPlanesCollider()
{
}

bool HybridPlanesCollider::Collide(PlanesCache& cache, const Plane* planes, udword nb_planes, const HybridModel& model, const Matrix4x4* worldm)
{
	// We don't want primitive tests here!
	mFlags |= OPC_NO_PRIMITIVE_TESTS;

	// Checkings
	if(!Setup(&model))	return false;

	// Init collision query
	if(InitQuery(cache, planes, nb_planes, worldm))	return true;

	// Special case for 1-leaf trees
	if(mCurrentModel && mCurrentModel->HasSingleNode())
	{
		// Here we're supposed to perform a normal query, except our tree has a single node, i.e. just a few triangles
		udword Nb = mIMesh->GetNbTriangles();

		// Loop through all triangles
		udword clip_mask = (1<<mNbPlanes)-1;
		for(udword i=0;i<Nb;i++)
		{
			PLANES_PRIM(i, OPC_CONTACT)
		}
		return true;
	}

	// Override destination array since we're only going to get leaf boxes here
	mTouchedBoxes.Reset();
	mTouchedPrimitives = &mTouchedBoxes;

	udword PlaneMask = (1<<nb_planes)-1;

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
			_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
		}
		else
		{
			const AABBNoLeafTree* Tree = (const AABBNoLeafTree*)model.GetTree();

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
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
			_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
		}
		else
		{
			const AABBCollisionTree* Tree = (const AABBCollisionTree*)model.GetTree();

			// Perform collision query - we don't want primitive tests here!
			_CollideNoPrimitiveTest(Tree->GetNodes(), PlaneMask);
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
		udword clip_mask = (1<<mNbPlanes)-1;
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
					PLANES_PRIM(TriangleIndex, OPC_CONTACT)
				}
			}
			else
			{
				udword BaseIndex = CurrentLeaf.GetTriangleIndex();

				// Loop through triangles and test each of them
				while(NbTris--)
				{
					udword TriangleIndex = BaseIndex++;
					PLANES_PRIM(TriangleIndex, OPC_CONTACT)
				}
			}
		}
	}

	return true;
}
