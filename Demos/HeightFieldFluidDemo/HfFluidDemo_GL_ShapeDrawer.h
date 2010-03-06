/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#ifndef HFFLUID_GL_SHAPE_DRAWER_H
#define HFFLUID_GL_SHAPE_DRAWER_H

#include "GL_ShapeDrawer.h"

///experimental buyancy fluid demo
/// OpenGL shape drawing
class HfFluidDemo_GL_ShapeDrawer : public GL_ShapeDrawer
{
	public:
		HfFluidDemo_GL_ShapeDrawer();

		virtual ~HfFluidDemo_GL_ShapeDrawer();

		///drawOpenGL might allocate temporary memoty, stores pointer in shape userpointer
		virtual void drawOpenGL(btScalar* m, const btCollisionShape* shape, const btVector3& color,int	debugMode,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax);
		virtual void drawShadow(btScalar* m, const btVector3& extrusion,const btCollisionShape* shape,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax);
};

#endif //HFFLUID_GL_SHAPE_DRAWER_H
