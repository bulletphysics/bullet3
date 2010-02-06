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
#include "GL_Simplex1to4.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "GL_ShapeDrawer.h"
#ifdef _WIN32
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#endif
#include "GlutStuff.h"
#include "LinearMath/btTransform.h"

GL_Simplex1to4::GL_Simplex1to4()
:m_simplexSolver(0)
{
}

///
/// Debugging method calcClosest calculates the closest point to the origin, using m_simplexSolver
///
void	GL_Simplex1to4::calcClosest(btScalar* m)
{
	btTransform tr;
	tr.setFromOpenGLMatrix(m);
	


			GL_ShapeDrawer::drawCoordSystem();
			
			if (m_simplexSolver)
			{
				m_simplexSolver->reset();
				bool res;

				btVector3 v;

				for (int i=0;i<m_numVertices;i++)
				{
					v =  tr(m_vertices[i]);
					m_simplexSolver->addVertex(v,v,btVector3(0.f,0.f,0.f));
					res = m_simplexSolver->closest(v);
				}

				//draw v?
				glDisable(GL_LIGHTING);
				glBegin(GL_LINES);
				btglColor3(1.f, 0.f, 0.f);
				btglVertex3(0.f, 0.f, 0.f);
				btglVertex3(v.x(),v.y(),v.z());
				glEnd();
				
				glEnable(GL_LIGHTING);


			}

}
