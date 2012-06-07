/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef DEBUG_CAST_RESULT_H
#define DEBUG_CAST_RESULT_H

#include "BulletCollision/NarrowPhaseCollision/btConvexCast.h"
#include "LinearMath/btTransform.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#ifdef WIN32
#include <windows.h>
#endif
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#endif
struct btDebugCastResult : public btConvexCast::CastResult
{

	btTransform	m_fromTrans;
	const btPolyhedralConvexShape* m_shape;
	btVector3	m_linVel;
	btVector3 m_angVel;
	GL_ShapeDrawer* m_shapeDrawer;

	btDebugCastResult(const btTransform& fromTrans,const btPolyhedralConvexShape* shape,
					const btVector3& linVel,const btVector3& angVel,GL_ShapeDrawer* drawer)
	:m_fromTrans(fromTrans),
	m_shape(shape),
	m_linVel(linVel),
	m_angVel(angVel),
	m_shapeDrawer(drawer)
	{
	}

	virtual void drawCoordSystem(const btTransform& tr)  
	{
		btScalar m[16];
		tr.getOpenGLMatrix(m);
		glPushMatrix();
		btglLoadMatrix(m);
		glBegin(GL_LINES);
		btglColor3(1, 0, 0);
		btglVertex3(0, 0, 0);
		btglVertex3(1, 0, 0);
		btglColor3(0, 1, 0);
		btglVertex3(0, 0, 0);
		btglVertex3(0, 1, 0);
		btglColor3(0, 0, 1);
		btglVertex3(0, 0, 0);
		btglVertex3(0, 0, 1);
		glEnd();
		glPopMatrix();
	}

	virtual void	DebugDraw(btScalar	fraction)
	{
		btVector3 worldBoundsMin(-1000,-1000,-1000);
		btVector3 worldBoundsMax(1000,1000,1000);

	
		ATTRIBUTE_ALIGNED16(btScalar) m[16];
		btTransform hitTrans;
		btTransformUtil::integrateTransform(m_fromTrans,m_linVel,m_angVel,fraction,hitTrans);
		hitTrans.getOpenGLMatrix(m);
		if (m_shapeDrawer)
			m_shapeDrawer->drawOpenGL(m,m_shape,btVector3(1,0,0),btIDebugDraw::DBG_NoDebug,worldBoundsMin,worldBoundsMax);
	}
};


#endif //DEBUG_CAST_RESULT_H
