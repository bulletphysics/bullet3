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
 *	\file		OPC_TreeBuilders.h
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_TREEBUILDERS_H__
#define __OPC_TREEBUILDERS_H__

	//! Tree splitting rules
	enum SplittingRules
	{
		// Primitive split
		SPLIT_LARGEST_AXIS		= (1<<0),		//!< Split along the largest axis
		SPLIT_SPLATTER_POINTS	= (1<<1),		//!< Splatter primitive centers (QuickCD-style)
		SPLIT_BEST_AXIS			= (1<<2),		//!< Try largest axis, then second, then last
		SPLIT_BALANCED			= (1<<3),		//!< Try to keep a well-balanced tree
		SPLIT_FIFTY				= (1<<4),		//!< Arbitrary 50-50 split
		// Node split
		SPLIT_GEOM_CENTER		= (1<<5),		//!< Split at geometric center (else split in the middle)
		//
		SPLIT_FORCE_DWORD		= 0x7fffffff
	};

	//! Simple wrapper around build-related settings [Opcode 1.3]
	struct OPCODE_API BuildSettings
	{
		inline_	BuildSettings() : mLimit(1), mRules(SPLIT_FORCE_DWORD)	{}

		udword	mLimit;		//!< Limit number of primitives / node. If limit is 1, build a complete tree (2*N-1 nodes)
		udword	mRules;		//!< Building/Splitting rules (a combination of SplittingRules flags)
	};

	class OPCODE_API AABBTreeBuilder
	{
		public:
		//! Constructor
													AABBTreeBuilder() :
														mNbPrimitives(0),
														mNodeBase(null),
														mCount(0),
														mNbInvalidSplits(0)		{}
		//! Destructor
		virtual										~AABBTreeBuilder()			{}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the AABB of a set of primitives.
		 *	\param		primitives		[in] list of indices of primitives
		 *	\param		nb_prims		[in] number of indices
		 *	\param		global_box		[out] global AABB enclosing the set of input primitives
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual						bool			ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box)	const	= 0;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the splitting value along a given axis for a given primitive.
		 *	\param		index			[in] index of the primitive to split
		 *	\param		axis			[in] axis index (0,1,2)
		 *	\return		splitting value
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual						float			GetSplittingValue(udword index, udword axis)	const	= 0;

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
		virtual						float			GetSplittingValue(const udword* primitives, udword nb_prims, const AABB& global_box, udword axis)	const
													{
														// Default split value = middle of the axis (using only the box)
														return global_box.GetCenter(axis);
													}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Validates node subdivision. This is called each time a node is considered for subdivision, during tree building.
		 *	\param		primitives		[in] list of indices of primitives
		 *	\param		nb_prims		[in] number of indices
		 *	\param		global_box		[in] global AABB enclosing the set of input primitives
		 *	\return		TRUE if the node should be subdivised
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual						BOOL			ValidateSubdivision(const udword* primitives, udword nb_prims, const AABB& global_box)
													{
														// Check the user-defined limit
														if(nb_prims<=mSettings.mLimit)	return FALSE;

														return TRUE;
													}

									BuildSettings	mSettings;			//!< Splitting rules & split limit [Opcode 1.3]
									udword			mNbPrimitives;		//!< Total number of primitives.
									void*			mNodeBase;			//!< Address of node pool [Opcode 1.3]
		// Stats
		inline_						void			SetCount(udword nb)				{ mCount=nb;				}
		inline_						void			IncreaseCount(udword nb)		{ mCount+=nb;				}
		inline_						udword			GetCount()				const	{ return mCount;			}
		inline_						void			SetNbInvalidSplits(udword nb)	{ mNbInvalidSplits=nb;		}
		inline_						void			IncreaseNbInvalidSplits()		{ mNbInvalidSplits++;		}
		inline_						udword			GetNbInvalidSplits()	const	{ return mNbInvalidSplits;	}

		private:
									udword			mCount;				//!< Stats: number of nodes created
									udword			mNbInvalidSplits;	//!< Stats: number of invalid splits
	};

	class OPCODE_API AABBTreeOfVerticesBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
													AABBTreeOfVerticesBuilder() : mVertexArray(null)	{}
		//! Destructor
		virtual										~AABBTreeOfVerticesBuilder()						{}

		override(AABBTreeBuilder)	bool			ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box)	const;
		override(AABBTreeBuilder)	float			GetSplittingValue(udword index, udword axis)									const;
		override(AABBTreeBuilder)	float			GetSplittingValue(const udword* primitives, udword nb_prims, const AABB& global_box, udword axis)	const;

		const						Point*			mVertexArray;		//!< Shortcut to an app-controlled array of vertices.
	};

	class OPCODE_API AABBTreeOfAABBsBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
													AABBTreeOfAABBsBuilder() : mAABBArray(null)	{}
		//! Destructor
		virtual										~AABBTreeOfAABBsBuilder()					{}

		override(AABBTreeBuilder)	bool			ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box)	const;
		override(AABBTreeBuilder)	float			GetSplittingValue(udword index, udword axis)									const;

		const						AABB*			mAABBArray;			//!< Shortcut to an app-controlled array of AABBs.
	};

	class OPCODE_API AABBTreeOfTrianglesBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
													AABBTreeOfTrianglesBuilder() : mIMesh(null)										{}
		//! Destructor
		virtual										~AABBTreeOfTrianglesBuilder()													{}

		override(AABBTreeBuilder)	bool			ComputeGlobalBox(const udword* primitives, udword nb_prims, AABB& global_box)	const;
		override(AABBTreeBuilder)	float			GetSplittingValue(udword index, udword axis)									const;
		override(AABBTreeBuilder)	float			GetSplittingValue(const udword* primitives, udword nb_prims, const AABB& global_box, udword axis)	const;

		const				MeshInterface*			mIMesh;			//!< Shortcut to an app-controlled mesh interface
	};

#endif // __OPC_TREEBUILDERS_H__
