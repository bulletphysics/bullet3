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
/// Collision Demo shows a degenerate case, where the Simplex solver has to deal with near-affine dependent cases
/// See the define CATCH_DEGENERATE_TETRAHEDRON in Bullet's VoronoiSimplexSolver.cpp
///


///This low-level internal demo does intentionally NOT use the btBulletCollisionCommon.h header
///It needs internal access
#include "GL_Simplex1to4.h"
#include "LinearMath/SimdQuaternion.h"
#include "LinearMath/SimdTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"

#include "CollisionDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "LinearMath/GenIDebugDraw.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

PolyhedralConvexShape*	shapePtr[maxNumObjects];

SimdTransform tr[numObjects];
int screenWidth = 640;
int screenHeight = 480;

void DrawRasterizerLine(float const* , float const*, int)
{

}

int main(int argc,char** argv)
{
	CollisionDemo* colDemo = new CollisionDemo();

	colDemo->setCameraDistance(18.f);

	colDemo->initPhysics();

	
	
	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Demo",colDemo);
}

void CollisionDemo::initPhysics()
{
	m_debugMode |= IDebugDraw::DBG_DrawWireframe;
	m_azi = 250.f;
	m_ele = 25.f;

	tr[0].setOrigin(SimdVector3(0.0013328250f,8.1363249f,7.0390840f));
	tr[1].setOrigin(SimdVector3(0.00000000f,9.1262732f,2.0343180f));

	//tr[0].setOrigin(SimdVector3(0,0,0));
	//tr[1].setOrigin(SimdVector3(0,10,0));

	SimdMatrix3x3 basisA;
	basisA.setValue(0.99999958f,0.00022980258f,0.00090992288f,
		-0.00029313788f,0.99753088f,0.070228584f,
		-0.00089153741f,-0.070228823f,0.99753052f);

	SimdMatrix3x3 basisB;
	basisB.setValue(1.0000000f,4.4865553e-018f,-4.4410586e-017f,
		4.4865495e-018f,0.97979438f,0.20000751f,
		4.4410586e-017f,-0.20000751f,0.97979438f);

	tr[0].setBasis(basisA);
	tr[1].setBasis(basisB);



	SimdVector3 boxHalfExtentsA(1.0000004768371582f,1.0000004768371582f,1.0000001192092896f);
	SimdVector3 boxHalfExtentsB(3.2836332321166992f,3.2836332321166992f,3.2836320400238037f);

	BoxShape*	boxA = new BoxShape(boxHalfExtentsA);
	BoxShape*	boxB = new BoxShape(boxHalfExtentsB);
	shapePtr[0] = boxA;
	shapePtr[1] = boxB;

}

void CollisionDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static VoronoiSimplexSolver sGjkSimplexSolver;
SimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;



void CollisionDemo::displayCallback(void) {

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
		//glVertex3d(gjkOutputm_pointInWorld.x(), gjkOutputm_pointInWorld.y(),gjkOutputm_pointInWorld.z());
		//glVertex3d(gjkOutputm_pointInWorld.x(), gjkOutputm_pointInWorld.y(),gjkOutputm_pointInWorld.z());
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

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

