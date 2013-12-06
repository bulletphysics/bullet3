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
 *	Contains code to perform "picking".
 *	\file		OPC_Picking.h
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_PICKING_H__
#define __OPC_PICKING_H__

#ifdef OPC_RAYHIT_CALLBACK

	enum CullMode
	{
		CULLMODE_NONE	= 0,
		CULLMODE_CW		= 1,
		CULLMODE_CCW	= 2
	};

	typedef CullMode (*CullModeCallback)(udword triangle_index, void* user_data);

	OPCODE_API	bool SetupAllHits		(RayCollider& collider, CollisionFaces& contacts);
	OPCODE_API	bool SetupClosestHit	(RayCollider& collider, CollisionFace& closest_contact);
	OPCODE_API	bool SetupShadowFeeler	(RayCollider& collider);
	OPCODE_API	bool SetupInOutTest		(RayCollider& collider);

	OPCODE_API	bool Picking(
						CollisionFace& picked_face,
						const Ray& world_ray, const Model& model, const Matrix4x4* world,
						float min_dist, float max_dist, const Point& view_point, CullModeCallback callback, void* user_data);
#endif

#endif //__OPC_PICKING_H__