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
 *	Contains code for a sphere collider.
 *	\file		OPC_SphereCollider.h
 *	\author		Pierre Terdiman
 *	\date		June, 2, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_SPHERECOLLIDER_H__
#define __OPC_SPHERECOLLIDER_H__

	struct OPCODE_API SphereCache : VolumeCache
	{
					SphereCache() : Center(0.0f,0.0f,0.0f), FatRadius2(0.0f), FatCoeff(1.1f)	{}
					~SphereCache()																{}

		// Cached faces signature
		Point		Center;		//!< Sphere used when performing the query resulting in cached faces
		float		FatRadius2;	//!< Sphere used when performing the query resulting in cached faces
		// User settings
		float		FatCoeff;	//!< mRadius2 multiplier used to create a fat sphere
	};

	class OPCODE_API SphereCollider : public VolumeCollider
	{
		public:
		// Constructor / Destructor
											SphereCollider();
		virtual								~SphereCollider();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Generic collision query for generic OPCODE models. After the call, access the results:
		 *	- with GetContactStatus()
		 *	- with GetNbTouchedPrimitives()
		 *	- with GetTouchedPrimitives()
		 *
		 *	\param		cache			[in/out] a sphere cache
		 *	\param		sphere			[in] collision sphere in local space
		 *	\param		model			[in] Opcode model to collide with
		 *	\param		worlds			[in] sphere's world matrix, or null
		 *	\param		worldm			[in] model's world matrix, or null
		 *	\return		true if success
		 *	\warning	SCALE NOT SUPPORTED. The matrices must contain rotation & translation parts only.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							bool			Collide(SphereCache& cache, const Sphere& sphere, const Model& model, const Matrix4x4* worlds=null, const Matrix4x4* worldm=null);

		// 
							bool			Collide(SphereCache& cache, const Sphere& sphere, const AABBTree* tree);
		protected:
		// Sphere in model space
							Point			mCenter;			//!< Sphere center
							float			mRadius2;			//!< Sphere radius squared
		// Internal methods
							void			_Collide(const AABBCollisionNode* node);
							void			_Collide(const AABBNoLeafNode* node);
							void			_Collide(const AABBQuantizedNode* node);
							void			_Collide(const AABBQuantizedNoLeafNode* node);
							void			_Collide(const AABBTreeNode* node);
							void			_CollideNoPrimitiveTest(const AABBCollisionNode* node);
							void			_CollideNoPrimitiveTest(const AABBNoLeafNode* node);
							void			_CollideNoPrimitiveTest(const AABBQuantizedNode* node);
							void			_CollideNoPrimitiveTest(const AABBQuantizedNoLeafNode* node);
			// Overlap tests
		inline_				BOOL			SphereContainsBox(const Point& bc, const Point& be);
		inline_				BOOL			SphereAABBOverlap(const Point& center, const Point& extents);
							BOOL			SphereTriOverlap(const Point& vert0, const Point& vert1, const Point& vert2);
			// Init methods
							BOOL			InitQuery(SphereCache& cache, const Sphere& sphere, const Matrix4x4* worlds=null, const Matrix4x4* worldm=null);
	};

	class OPCODE_API HybridSphereCollider : public SphereCollider
	{
		public:
		// Constructor / Destructor
											HybridSphereCollider();
		virtual								~HybridSphereCollider();

							bool			Collide(SphereCache& cache, const Sphere& sphere, const HybridModel& model, const Matrix4x4* worlds=null, const Matrix4x4* worldm=null);
		protected:
							Container		mTouchedBoxes;
	};

#endif // __OPC_SPHERECOLLIDER_H__
