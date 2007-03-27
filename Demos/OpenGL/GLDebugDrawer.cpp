
#include "GLDebugDrawer.h"
#include "LinearMath/btPoint3.h"

#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "BMF_Api.h"
#include <stdio.h> //printf debugging
GLDebugDrawer::GLDebugDrawer()
:m_debugMode(0)
{

}
void	GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
	if (m_debugMode > 0)
	{
		glBegin(GL_LINES);
		glColor3f(color.getX(), color.getY(), color.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();
	}
}

void	GLDebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;

}

void	GLDebugDrawer::reportErrorWarning(const char* warningString)
{
	printf(warningString);
}

void	GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
	if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
	{
		btVector3 to=pointOnB+normalOnB*distance;
		const btVector3&from = pointOnB;
		glBegin(GL_LINES);
		glColor3f(color.getX(), color.getY(), color.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();


		glRasterPos3f(from.x(),  from.y(),  from.z());
		char buf[12];
		sprintf(buf," %d",lifeTime);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


	}
}




