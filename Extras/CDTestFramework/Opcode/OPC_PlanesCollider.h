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
 *	\file		OPC_PlanesCollider.h
 *	\author		Pierre Terdiman
 *	\date		January, 1st, 2002
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_PLANESCOLLIDER_H__
#define __OPC_PLANESCOLLIDER_H__

	struct OPCODE_API PlanesCache : VolumeCache
	{
					PlanesCache()
					{
					}
	};

	class OPCODE_API PlanesCollider : public VolumeCollider
	{
		public:
		// Constructor / Destructor
											PlanesCollider();
		virtual								~PlanesCollider();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Generic collision query for generic OPCODE models. After the call, access the results:
		 *	- with GetContactStatus()
		 *	- with GetNbTouchedPrimitives()
		 *	- with GetTouchedPrimitives()
		 *
		 *	\param		cache			[in/out] a planes cache
		 *	\param		planes			[in] list of planes in world space
		 *	\param		nb_planes		[in] number of planes
		 *	\param		model			[in] Opcode model to collide with
		 *	\param		worldm			[in] model's world matrix, or null
		 *	\return		true if success
		 *	\warning	SCALE NOT SUPPORTED. The matrices must contain rotation & translation parts only.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							bool			Collide(PlanesCache& cache, const Plane* planes, udword nb_planes, const Model& model, const Matrix4x4* worldm=null);

		// Mutant box-with-planes collision queries
		inline_				bool			Collide(PlanesCache& cache, const OBB& box, const Model& model, const Matrix4x4* worldb=null, const Matrix4x4* worldm=null)
											{
												Plane PL[6];

												if(worldb)
												{
													// Create a new OBB in world space
													OBB WorldBox;
													box.Rotate(*worldb, WorldBox);
													// Compute planes from the sides of the box
													WorldBox.ComputePlanes(PL);
												}
												else
												{
													// Compute planes from the sides of the box
													box.ComputePlanes(PL);
												}

												// Collide with box planes
												return Collide(cache, PL, 6, model, worldm);
											}
		// Settings

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Validates current settings. You should call this method after all the settings and callbacks have been defined for a collider.
		 *	\return		null if everything is ok, else a string describing the problem
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		override(Collider)	const char*		ValidateSettings();

		protected:
		// Planes in model space
							udword			mNbPlanes;
							Plane*			mPlanes;
		// Leaf description
							VertexPointers	mVP;
		// Internal methods
							void			_Collide(const AABBCollisionNode* node, udword clip_mask);
							void			_Collide(const AABBNoLeafNode* node, udword clip_mask);
							void			_Collide(const AABBQuantizedNode* node, udword clip_mask);
							void			_Collide(const AABBQuantizedNoLeafNode* node, udword clip_mask);
							void			_CollideNoPrimitiveTest(const AABBCollisionNode* node, udword clip_mask);
							void			_CollideNoPrimitiveTest(const AABBNoLeafNode* node, udword clip_mask);
							void			_CollideNoPrimitiveTest(const AABBQuantizedNode* node, udword clip_mask);
							void			_CollideNoPrimitiveTest(const AABBQuantizedNoLeafNode* node, udword clip_mask);
			// Overlap tests
		inline_				BOOL			PlanesAABBOverlap(const Point& center, const Point& extents, udword& out_clip_mask, udword in_clip_mask);
		inline_				BOOL			PlanesTriOverlap(udword in_clip_mask);
			// Init methods
							BOOL			InitQuery(PlanesCache& cache, const Plane* planes, udword nb_planes, const Matrix4x4* worldm=null);
	};

	class OPCODE_API HybridPlanesCollider : public PlanesCollider
	{
		public:
		// Constructor / Destructor
											HybridPlanesCollider();
		virtual								~HybridPlanesCollider();

							bool			Collide(PlanesCache& cache, const Plane* planes, udword nb_planes, const HybridModel& model, const Matrix4x4* worldm=null);
		protected:
							Container		mTouchedBoxes;
	};

#endif // __OPC_PLANESCOLLIDER_H__
