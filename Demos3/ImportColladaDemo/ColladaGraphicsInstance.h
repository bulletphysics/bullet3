/*
Bullet Collision Detection and Physics Library http://bulletphysics.org
This file is Copyright (c) 2014 Google Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

//original author: Erwin Coumans
*/

#ifndef COLLADA_GRAPHICS_INSTANCE_H
#define COLLADA_GRAPHICS_INSTANCE_H

#include "btMatrix4x4.h"

struct ColladaGraphicsInstance
{
	ColladaGraphicsInstance()
		:m_shapeIndex(-1)
	{
		m_worldTransform.setIdentity();
	}
	btMatrix4x4	m_worldTransform;
	int m_shapeIndex;//could be index into array of GLInstanceGraphicsShape
	float m_color[4];
};

#endif //COLLADA_GRAPHICS_INSTANCE_H
