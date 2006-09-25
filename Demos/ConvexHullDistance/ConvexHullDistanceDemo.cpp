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


///
/// Convex Hull Distance Demo shows distance calculation between two convex hulls of points.
/// GJK with the VoronoiSimplexSolver is used.
///

#include "GL_Simplex1to4.h"
#include "LinearMath/SimdQuaternion.h"
#include "LinearMath/SimdTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletDynamics/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"

#include "GL_ShapeDrawer.h"
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
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

PolyhedralConvexShape*	shapePtr[maxNumObjects];

SimdTransform tr[numObjects];
int screenWidth = 640.f;
int screenHeight = 480.f;


int main(int argc,char** argv)
{
	clientResetScene();

	SimdMatrix3x3 basisA;
	basisA.setIdentity();

	SimdMatrix3x3 basisB;
	basisB.setIdentity();

	tr[0].setBasis(basisA);
	tr[1].setBasis(basisB);

	SimdPoint3	points0[3]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1)};
	SimdPoint3	points1[5]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1),SimdPoint3(0,0,-1),SimdPoint3(-1,-1,0)};
	
	ConvexHullShape	hullA(points0,3);
	ConvexHullShape	hullB(points1,5);

	shapePtr[0] = &hullA;
	shapePtr[1] = &hullB;
	

	SimdTransform tr;
	tr.setIdentity();


	return glutmain(argc, argv,screenWidth,screenHeight,"Convex Hull Distance Demo");
}

//to be implemented by the demo

void clientMoveAndDisplay()
{
	
	clientDisplay();
}


static VoronoiSimplexSolver sGjkSimplexSolver;
SimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;



void clientDisplay(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	//GL_ShapeDrawer::DrawCoordSystem();

	float m[16];
	int i;

	GjkPairDetector	convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,0);

	SimdVector3 seperatingAxis(0.00000000f,0.059727669f,0.29259586f);
	convexConvex.SetCachedSeperatingAxis(seperatingAxis);

	PointCollector gjkOutput;
	GjkPairDetector::ClosestPointInput input;
	input.m_transformA = tr[0];
	input.m_transformB = tr[1];

	convexConvex.GetClosestPoints(input ,gjkOutput,0);

	if (gjkOutput.m_hasResult)
	{
		SimdVector3 endPt = gjkOutput.m_pointInWorld +
			gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

		 glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3d(gjkOutput.m_pointInWorld.x(), gjkOutput.m_pointInWorld.y(),gjkOutput.m_pointInWorld.z());
		glVertex3d(endPt.x(),endPt.y(),endPt.z());
		glEnd();

	}

	for (i=0;i<numObjects;i++)
	{
		
		tr[i].getOpenGLMatrix( m );

		GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i],SimdVector3(1,1,1),getDebugMode());


	}

	simplex.SetSimplexSolver(&sGjkSimplexSolver);
	SimdPoint3 ybuf[4],pbuf[4],qbuf[4];
	int numpoints = sGjkSimplexSolver.getSimplex(pbuf,qbuf,ybuf);
	simplex.Reset();
	
	for (i=0;i<numpoints;i++)
		simplex.AddVertex(ybuf[i]);

	SimdTransform ident;
	ident.setIdentity();
	ident.getOpenGLMatrix(m);
	GL_ShapeDrawer::DrawOpenGL(m,&simplex,SimdVector3(1,1,1),getDebugMode());


	SimdQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	tr[0].setRotation(orn);
	tr[1].setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

void clientResetScene()
{
	tr[0].setOrigin(SimdVector3(0.0f,3.f,7.f));
	tr[1].setOrigin(SimdVector3(0.0f,9.f,2.f));
}

void clientKeyboard(unsigned char key, int x, int y)
{
	defaultKeyboard(key, x, y);
}


void clientMouseFunc(int button, int state, int x, int y)
{

}

void	clientMotionFunc(int x,int y)
{
}
