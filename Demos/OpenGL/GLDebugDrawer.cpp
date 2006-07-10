
#include "GLDebugDrawer.h"
#include "SimdPoint3.h"

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
void	GLDebugDrawer::DrawLine(const SimdVector3& from,const SimdVector3& to,const SimdVector3& color)
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

void	GLDebugDrawer::SetDebugMode(int debugMode)
{
	m_debugMode = debugMode;

}

void	GLDebugDrawer::DrawContactPoint(const SimdVector3& pointOnB,const SimdVector3& normalOnB,float distance,int lifeTime,const SimdVector3& color)
{
	if (m_debugMode & IDebugDraw::DBG_DrawContactPoints)
	{
		SimdVector3 to=pointOnB+normalOnB*distance;
		const SimdVector3&from = pointOnB;
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
