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
 *	Contains code for tree builders.
 *	\file		OPC_TreeBuilders.cpp
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A builder for AABB-trees of vertices.
 *
 *	\class		AABBTreeOfVerticesBuilder
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A builder for AABB-trees of AABBs.
 *
 *	\class		AABBTreeOfAABBsBuilder
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	A builder for AABB-trees of triangles.
 *
 *	\class		AABBTreeOfTrianglesBuilder
 *	\author		Pierre Terdiman
 *	\version	1.3
 *	\date		March, 20, 2001
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfAABBsBuilder::ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	global_box = mAABBArray[primitives[0]];

	// Loop through boxes
	for(udword i=1;i<nb_prims;i++)
	{
		// Update global box
		global_box.Add(mAABBArray[primitives[i]]);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfAABBsBuilder::GetSplittingValue(udword index, udword axis) const
{
	// For an AABB, the splitting value is the middle of the given axis,
	// i.e. the corresponding component of the center point
	return mAABBArray[index].GetCenter(axis);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfTrianglesBuilder::ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	Point Min(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	Point Max(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);

	// Loop through triangles
	VertexPointers VP;
	while(nb_prims--)
	{
		// Get current triangle-vertices
		mIMesh->GetTriangle(VP, *primitives++);
		// Update global box
		Min.Min(*VP.Vertex[0]).Min(*VP.Vertex[1]).Min(*VP.Vertex[2]);
		Max.Max(*VP.Vertex[0]).Max(*VP.Vertex[1]).Max(*VP.Vertex[2]);
	}
	global_box.SetMinMax(Min, Max);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfTrianglesBuilder::GetSplittingValue(udword index, udword axis) const
{
/*	// Compute center of triangle
	Point Center;
	mTriList[index].Center(mVerts, Center);
	// Return value
	return Center[axis];*/

	// Compute correct component from center of triangle
//	return	(mVerts[mTriList[index].mVRef[0]][axis]
//			+mVerts[mTriList[index].mVRef[1]][axis]
//			+mVerts[mTriList[index].mVRef[2]][axis])*INV3;

	VertexPointers VP;
	mIMesh->GetTriangle(VP, index);

	// Compute correct component from center of triangle
	return	((*VP.Vertex[0])[axis]
			+(*VP.Vertex[1])[axis]
			+(*VP.Vertex[2])[axis])*INV3;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given node.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[in] global AABB enclosing the set of input primitives
 *	\param		axis			[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfTrianglesBuilder::GetSplittingValue(const udword* primitives, udword nb_prims, const AABB& global_box, udword axis)	const
{
	if(mSettings.mRules&SPLIT_GEOM_CENTER)
	{
		// Loop through triangles
		float SplitValue = 0.0f;
		VertexPointers VP;
		for(udword i=0;i<nb_prims;i++)
		{
			// Get current triangle-vertices
			mIMesh->GetTriangle(VP, primitives[i]);
			// Update split value
			SplitValue += (*VP.Vertex[0])[axis];
			SplitValue += (*VP.Vertex[1])[axis];
			SplitValue += (*VP.Vertex[2])[axis];
		}
		return SplitValue / float(nb_prims*3);
	}
	else return AABBTreeBuilder::GetSplittingValue(primitives, nb_prims, global_box, axis);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfVerticesBuilder::ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	global_box.SetEmpty();

	// Loop through vertices
	for(udword i=0;i<nb_prims;i++)
	{
		// Update global box
		global_box.Extend(mVertexArray[primitives[i]]);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfVerticesBuilder::GetSplittingValue(udword index, udword axis) const
{
	// For a vertex, the splitting value is simply the vertex coordinate.
	return mVertexArray[index][axis];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given node.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[in] global AABB enclosing the set of input primitives
 *	\param		axis			[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfVerticesBuilder::GetSplittingValue(const udword* primitives, udword nb_prims, const AABB& global_box, udword axis)	const
{
	if(mSettings.mRules&SPLIT_GEOM_CENTER)
	{
		// Loop through vertices
		float SplitValue = 0.0f;
		for(udword i=0;i<nb_prims;i++)
		{
			// Update split value
			SplitValue += mVertexArray[primitives[i]][axis];
		}
		return SplitValue / float(nb_prims);
	}
	else return AABBTreeBuilder::GetSplittingValue(primitives, nb_prims, global_box, axis);
}
