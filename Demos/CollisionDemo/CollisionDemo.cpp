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
/// See the define CATCH_DEGENERATE_TETRAHEDRON in Bullet's btVoronoiSimplexSolver.cpp
///


//#define CHECK_GENSHER_TRIANGLE_CASE 1


///This low-level internal demo does intentionally NOT use the btBulletCollisionCommon.h header
///It needs internal access
#include "GL_Simplex1to4.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "LinearMath/btTransformUtil.h"

#include "CollisionDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "LinearMath/btIDebugDraw.h"
#include "../OpenGL/GLDebugDrawer.h"
GLDebugDrawer debugDrawer;


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btPolyhedralConvexShape*	shapePtr[maxNumObjects];

btTransform tr[numObjects];
int screenWidth = 640;
int screenHeight = 480;

void DrawRasterizerLine(float const* , float const*, int)
{

}

int main(int argc,char** argv)
{
	CollisionDemo* colDemo = new CollisionDemo();

#ifdef CHECK_GENSHER_TRIANGLE_CASE
	colDemo->setCameraDistance(8.f);
#else
	colDemo->setCameraDistance(18.f);
	
#endif //
	colDemo->initPhysics();

	
	
	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Demo",colDemo);
}

void CollisionDemo::initPhysics()
{
	setTexturing(false);
	setShadows(false);

	//m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
#ifdef CHECK_GENSHER_TRIANGLE_CASE
	m_azi = 140.f;
#else
	m_azi = 250.f;
#endif
	m_ele = 25.f;

	m_azi = 0;
	m_ele = 0;

	tr[0].setIdentity();
	tr[0].setOrigin(btVector3(10,0,0));
	tr[1].setIdentity();
	tr[1].setOrigin(btVector3(0,0,0));


#ifdef CHECK_GENSHER_TRIANGLE_CASE
	tr[0].setIdentity();
	tr[1].setIdentity();
#endif //CHECK_GENSHER_TRIANGLE_CASE

	btVector3 boxHalfExtentsA(1,1,1);//1.0000004768371582f,1.0000004768371582f,1.0000001192092896f);
	btVector3 boxHalfExtentsB(4,4,4);//3.2836332321166992f,3.2836332321166992f,3.2836320400238037f);

#ifndef CHECK_GENSHER_TRIANGLE_CASE
  	btBoxShape*	boxA = new btBoxShape(boxHalfExtentsA);
  	btBoxShape*	boxB = new btBoxShape(boxHalfExtentsB);
#endif




	

#ifdef CHECK_GENSHER_TRIANGLE_CASE
	shapePtr[0] = trishapeA;
	shapePtr[1] = trishapeB;
#else
	shapePtr[0] = boxA;
	shapePtr[1] = boxB;
#endif

}

void CollisionDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

static btScalar gContactBreakingThreshold=.02f;

int checkPerturbation = 1;
int numPerturbationIterations = 20;
void CollisionDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);
    
   btVoronoiSimplexSolver sGjkSimplexSolver;
   btGjkEpaPenetrationDepthSolver epaSolver;
   btPointCollector gjkOutput; 
   btVector3 worldBoundsMin(-1000,-1000,-1000);
		btVector3 worldBoundsMax(1000,1000,1000);
   {
	   btGjkPairDetector convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,&epaSolver); 
	   
	   btGjkPairDetector::ClosestPointInput input; 
	   input.m_transformA = tr[0]; 
	   input.m_transformB = tr[1]; 
	    
		
	   convexConvex.getClosestPoints(input, gjkOutput, 0); 
   }
    
   if (gjkOutput.m_hasResult) 
   { 
        printf("bullet: %10.10f\n", gjkOutput.m_distance);
 		btVector3 endPt = gjkOutput.m_pointInWorld +
		gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

		debugDrawer.drawLine(gjkOutput.m_pointInWorld,endPt,btVector3(1,0,0));
		debugDrawer.drawSphere(gjkOutput.m_pointInWorld,0.05,btVector3(0,1,0));
		debugDrawer.drawSphere(endPt,0.05,btVector3(0,0,1));

		bool perturbeA = false;//true;
		const btScalar angleLimit = 0.125f * SIMD_PI;
		btScalar perturbeAngle;
		btScalar radiusA = shapePtr[0]->getAngularMotionDisc();
		btScalar radiusB = shapePtr[1]->getAngularMotionDisc();
		
		if (radiusA < radiusB)
		{
			perturbeAngle = gContactBreakingThreshold /radiusA;
			perturbeA = true;
		} else
		{
			perturbeAngle = gContactBreakingThreshold / radiusB;
			perturbeA = false;
		}
		if ( perturbeAngle > angleLimit ) 
				perturbeAngle = angleLimit;

		btVector3 v0,v1;
		btPlaneSpace1(gjkOutput.m_normalOnBInWorld,v0,v1);
		
		
		int i;
		for ( i=0;i<numPerturbationIterations;i++)
		{

			btGjkPairDetector::ClosestPointInput input; 
			input.m_transformA = tr[0]; 
			input.m_transformB = tr[1]; 
			sGjkSimplexSolver.reset();
			
			btQuaternion perturbeRot(v0,perturbeAngle);
			btScalar iterationAngle = i*(SIMD_2_PI/btScalar(numPerturbationIterations));
			btQuaternion rotq(gjkOutput.m_normalOnBInWorld,iterationAngle);
			if (perturbeA)
			{
				input.m_transformA.setBasis( btMatrix3x3(rotq*perturbeRot*rotq.inverse())*tr[0].getBasis());
			} else
			{
				input.m_transformB.setBasis( btMatrix3x3(rotq.inverse()*perturbeRot*rotq)*tr[1].getBasis());
			}
			debugDrawer.drawTransform(input.m_transformA,10.0);
			btGjkPairDetector convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,&epaSolver); 
			input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
			gjkOutput.m_distance = BT_LARGE_FLOAT;
			convexConvex.getClosestPoints(input, gjkOutput, 0); 
			
			

			
			{
				btScalar m[16];
						
				input.m_transformA.getOpenGLMatrix( m );
				m_shapeDrawer->drawOpenGL(m,shapePtr[0],btVector3(0,1,0),getDebugMode(),worldBoundsMin,worldBoundsMax);
			
			}

			if (1)//gjkOutput.m_hasResult) 
			{ 
				printf("bullet: %10.10f\n", gjkOutput.m_distance);
				btVector3 startPt,endPt;
				if (perturbeA)
				{
 					btVector3 endPtOrg = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
					endPt = (tr[0]*input.m_transformA.inverse())(endPtOrg);
					btScalar depth = (endPt -  gjkOutput.m_pointInWorld).dot(gjkOutput.m_normalOnBInWorld);
					startPt = endPt-gjkOutput.m_normalOnBInWorld*depth;
				} else
				{
					endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
					startPt = (tr[1]*input.m_transformB.inverse())(gjkOutput.m_pointInWorld);
					btScalar depth = (endPt -  startPt).dot(gjkOutput.m_normalOnBInWorld);
				}

				
				debugDrawer.drawLine(startPt,endPt,btVector3(1,0,0));
				debugDrawer.drawSphere(startPt,0.05,btVector3(0,1,0));
				debugDrawer.drawSphere(endPt,0.05,btVector3(0,0,1));
		  }
		}

   } 
   static int looper = 0;
   if (looper++>10)
   {
	looper =0;
	checkPerturbation++;
	if (checkPerturbation>numPerturbationIterations)
		checkPerturbation=0;
   }

   GL_ShapeDrawer::drawCoordSystem();

	btScalar m[16];
	int i;

	



	for (i=0;i<numObjects;i++)
	{
		tr[i].getOpenGLMatrix( m );
		m_shapeDrawer->drawOpenGL(m,shapePtr[i],btVector3(1,1,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
	}


	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	//let it rotate
	//tr[0].setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

