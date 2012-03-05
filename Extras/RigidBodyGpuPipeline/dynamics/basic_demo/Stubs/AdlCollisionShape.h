/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

#include "Stubs/AdlMath.h"
#include "Stubs/AdlAabb.h"


_MEM_CLASSALIGN16
class CollisionShape
{
	public:
		_MEM_ALIGNED_ALLOCATOR16;

		enum Type
		{
			SHAPE_HEIGHT_FIELD,
			SHAPE_CONVEX_HEIGHT_FIELD,
			SHAPE_PLANE,
			MAX_NUM_SHAPE_TYPES,
		};

		CollisionShape( Type type, float collisionMargin = 0.0025f ) : m_type( type ){ m_collisionMargin = collisionMargin; }
		virtual ~CollisionShape(){}
		virtual float queryDistance(const float4& p) const = 0;
		virtual bool queryDistanceWithNormal(const float4& p, float4& normalOut) const = 0;

	public:
		Type m_type;
		Aabb m_aabb;
		float m_collisionMargin;
};

#endif
