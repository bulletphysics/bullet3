
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


void GLDebugDrawer::drawAabb(const btVector3& from,const btVector3& to,const btVector3& color)
{

	btVector3 halfExtents = (to-from)* 0.5f;
	btVector3 center = (to+from) *0.5f;
	int i,j;

	btVector3 edgecoord(1.f,1.f,1.f),pa,pb;
	for (i=0;i<4;i++)
	{
		for (j=0;j<3;j++)
		{
			pa = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],		
				edgecoord[2]*halfExtents[2]);
			pa+=center;

			int othercoord = j%3;
			edgecoord[othercoord]*=-1.f;
			pb = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],	
				edgecoord[2]*halfExtents[2]);
			pb+=center;

			drawLine(pa,pb,color);
		}
		edgecoord = btVector3(-1.f,-1.f,-1.f);
		if (i<3)
			edgecoord[i]*=-1.f;
	}


}

