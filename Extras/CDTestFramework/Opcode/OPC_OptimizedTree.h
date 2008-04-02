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
 *	Contains code for optimized trees.
 *	\file		OPC_OptimizedTree.h
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_OPTIMIZEDTREE_H__
#define __OPC_OPTIMIZEDTREE_H__

	//! Common interface for a node of an implicit tree
	#define IMPLEMENT_IMPLICIT_NODE(base_class, volume)														\
		public:																								\
		/* Constructor / Destructor */																		\
		inline_								base_class() : mData(0)	{}										\
		inline_								~base_class()			{}										\
		/* Leaf test */																						\
		inline_			BOOL				IsLeaf()		const	{ return mData&1;					}	\
		/* Data access */																					\
		inline_			const base_class*	GetPos()		const	{ return (base_class*)mData;		}	\
		inline_			const base_class*	GetNeg()		const	{ return ((base_class*)mData)+1;	}	\
		inline_			udword				GetPrimitive()	const	{ return (mData>>1);				}	\
		/* Stats */																							\
		inline_			udword				GetNodeSize()	const	{ return SIZEOFOBJECT;				}	\
																											\
						volume				mAABB;															\
						udword				mData;

	//! Common interface for a node of a no-leaf tree
	#define IMPLEMENT_NOLEAF_NODE(base_class, volume)														\
		public:																								\
		/* Constructor / Destructor */																		\
		inline_								base_class() : mPosData(0), mNegData(0)	{}						\
		inline_								~base_class()							{}						\
		/* Leaf tests */																					\
		inline_			BOOL				HasPosLeaf()		const	{ return mPosData&1;			}	\
		inline_			BOOL				HasNegLeaf()		const	{ return mNegData&1;			}	\
		/* Data access */																					\
		inline_			const base_class*	GetPos()			const	{ return (base_class*)mPosData;	}	\
		inline_			const base_class*	GetNeg()			const	{ return (base_class*)mNegData;	}	\
		inline_			udword				GetPosPrimitive()	const	{ return (mPosData>>1);			}	\
		inline_			udword				GetNegPrimitive()	const	{ return (mNegData>>1);			}	\
		/* Stats */																							\
		inline_			udword				GetNodeSize()		const	{ return SIZEOFOBJECT;			}	\
																											\
						volume				mAABB;															\
						udword				mPosData;														\
						udword				mNegData;

	class OPCODE_API AABBCollisionNode
	{
		IMPLEMENT_IMPLICIT_NODE(AABBCollisionNode, CollisionAABB)

		inline_			float				GetVolume()		const	{ return mAABB.mExtents.x * mAABB.mExtents.y * mAABB.mExtents.z;	}
		inline_			float				GetSize()		const	{ return mAABB.mExtents.SquareMagnitude();	}
		inline_			udword				GetRadius()		const
											{
												udword* Bits = (udword*)&mAABB.mExtents.x;
												udword Max = Bits[0];
												if(Bits[1]>Max)	Max = Bits[1];
												if(Bits[2]>Max)	Max = Bits[2];
												return Max;
											}

		// NB: using the square-magnitude or the true volume of the box, seems to yield better results
		// (assuming UNC-like informed traversal methods). I borrowed this idea from PQP. The usual "size"
		// otherwise, is the largest box extent. In SOLID that extent is computed on-the-fly each time it's
		// needed (the best approach IMHO). In RAPID the rotation matrix is permuted so that Extent[0] is
		// always the greatest, which saves looking for it at runtime. On the other hand, it yields matrices
		// whose determinant is not 1, i.e. you can't encode them anymore as unit quaternions. Not a very
		// good strategy.
	};

	class OPCODE_API AABBQuantizedNode
	{
		IMPLEMENT_IMPLICIT_NODE(AABBQuantizedNode, QuantizedAABB)

		inline_			uword				GetSize()		const
											{
												const uword* Bits = mAABB.mExtents;
												uword Max = Bits[0];
												if(Bits[1]>Max)	Max = Bits[1];
												if(Bits[2]>Max)	Max = Bits[2];
												return Max;
											}
		// NB: for quantized nodes I don't feel like computing a square-magnitude with integers all
		// over the place.......!
	};

	class OPCODE_API AABBNoLeafNode
	{
		IMPLEMENT_NOLEAF_NODE(AABBNoLeafNode, CollisionAABB)
	};

	class OPCODE_API AABBQuantizedNoLeafNode
	{
		IMPLEMENT_NOLEAF_NODE(AABBQuantizedNoLeafNode, QuantizedAABB)
	};

	//! Common interface for a collision tree
	#define IMPLEMENT_COLLISION_TREE(base_class, node)																\
		public:																										\
		/* Constructor / Destructor */																				\
													base_class();													\
		virtual										~base_class();													\
		/* Builds from a standard tree */																			\
		override(AABBOptimizedTree)	bool			Build(AABBTree* tree);											\
		/* Refits the tree */																						\
		override(AABBOptimizedTree)	bool			Refit(const MeshInterface* mesh_interface);						\
		/* Walks the tree */																						\
		override(AABBOptimizedTree)	bool			Walk(GenericWalkingCallback callback, void* user_data) const;	\
		/* Data access */																							\
		inline_						const node*		GetNodes()		const	{ return mNodes;					}	\
		/* Stats */																									\
		override(AABBOptimizedTree)	udword			GetUsedBytes()	const	{ return mNbNodes*sizeof(node);		}	\
		private:																									\
									node*			mNodes;

	typedef		bool				(*GenericWalkingCallback)	(const void* current, void* user_data);

	class OPCODE_API AABBOptimizedTree
	{
		public:
		// Constructor / Destructor
											AABBOptimizedTree() :
												mNbNodes	(0)
																							{}
		virtual								~AABBOptimizedTree()							{}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Builds the collision tree from a generic AABB tree.
		 *	\param		tree			[in] generic AABB tree
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual			bool				Build(AABBTree* tree)											= 0;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Refits the collision tree after vertices have been modified.
		 *	\param		mesh_interface	[in] mesh interface for current model
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual			bool				Refit(const MeshInterface* mesh_interface)						= 0;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Walks the tree and call the user back for each node.
		 *	\param		callback	[in] walking callback
		 *	\param		user_data	[in] callback's user data
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual			bool				Walk(GenericWalkingCallback callback, void* user_data) const	= 0;

		// Data access
		virtual			udword				GetUsedBytes()		const										= 0;
		inline_			udword				GetNbNodes()		const						{ return mNbNodes;	}

		protected:
						udword				mNbNodes;
	};

	class OPCODE_API AABBCollisionTree : public AABBOptimizedTree
	{
		IMPLEMENT_COLLISION_TREE(AABBCollisionTree, AABBCollisionNode)
	};

	class OPCODE_API AABBNoLeafTree : public AABBOptimizedTree
	{
		IMPLEMENT_COLLISION_TREE(AABBNoLeafTree, AABBNoLeafNode)
	};

	class OPCODE_API AABBQuantizedTree : public AABBOptimizedTree
	{
		IMPLEMENT_COLLISION_TREE(AABBQuantizedTree, AABBQuantizedNode)

		public:
						Point				mCenterCoeff;
						Point				mExtentsCoeff;
	};

	class OPCODE_API AABBQuantizedNoLeafTree : public AABBOptimizedTree
	{
		IMPLEMENT_COLLISION_TREE(AABBQuantizedNoLeafTree, AABBQuantizedNoLeafNode)

		public:
						Point				mCenterCoeff;
						Point				mExtentsCoeff;
	};

#endif // __OPC_OPTIMIZEDTREE_H__
