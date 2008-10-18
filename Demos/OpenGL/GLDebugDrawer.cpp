
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
//	if (m_debugMode > 0)
	{
		glBegin(GL_LINES);
		glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();
	}
}

void	GLDebugDrawer::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha)
{
//	if (m_debugMode > 0)
	{
		const btVector3	n=cross(b-a,c-a).normalized();
		glBegin(GL_TRIANGLES);		
		glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
		glNormal3d(n.getX(),n.getY(),n.getZ());
		glVertex3d(a.getX(),a.getY(),a.getZ());
		glVertex3d(b.getX(),b.getY(),b.getZ());
		glVertex3d(c.getX(),c.getY(),c.getZ());
		glEnd();
	}
}

void	GLDebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;

}

void	GLDebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
	glRasterPos3f(location.x(),  location.y(),  location.z());
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void	GLDebugDrawer::reportErrorWarning(const char* warningString)
{
	printf(warningString);
}

void	GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
	
	{
		btVector3 to=pointOnB+normalOnB*distance;
		const btVector3&from = pointOnB;
		glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
		//glColor4f(0,0,0,1.f);

		glBegin(GL_LINES);
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();

		
		glRasterPos3f(from.x(),  from.y(),  from.z());
		char buf[12];
		sprintf(buf," %d",lifeTime);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


	}
}





