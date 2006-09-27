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

	btDebugCastResult(const btTransform& fromTrans,const btPolyhedralConvexShape* shape,
					const btVector3& linVel,const btVector3& angVel)
	:m_fromTrans(fromTrans),
	m_shape(shape),
	m_linVel(linVel),
	m_angVel(angVel)
	{
	}

	virtual void DrawCoordSystem(const btTransform& tr)  
	{
		float m[16];
		tr.getOpenGLMatrix(m);
		glPushMatrix();
		glLoadMatrixf(m);
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(1, 0, 0);
		glColor3f(0, 1, 0);
		glVertex3d(0, 0, 0);
		glVertex3d(0, 1, 0);
		glColor3f(0, 0, 1);
		glVertex3d(0, 0, 0);
		glVertex3d(0, 0, 1);
		glEnd();
		glPopMatrix();
	}

	virtual void	DebugDraw(btScalar	fraction)
	{
	
		float m[16];
		btTransform hitTrans;
		btTransformUtil::IntegrateTransform(m_fromTrans,m_linVel,m_angVel,fraction,hitTrans);
		hitTrans.getOpenGLMatrix(m);
		GL_ShapeDrawer::DrawOpenGL(m,m_shape,btVector3(1,0,0),btIDebugDraw::DBG_NoDebug);
	
	}
};


#endif //DEBUG_CAST_RESULT_H
