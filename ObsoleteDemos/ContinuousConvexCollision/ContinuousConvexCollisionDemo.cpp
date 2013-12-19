/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
 */


/*
	Continuous Convex Collision Demo demonstrates an efficient continuous collision detection algorithm.
	Both linear and angular velocities are supported. Convex Objects are sampled using Supporting Vertex.
	Motion using Exponential Map.
	Future ideas: Comparison with Screwing Motion. 
	Also comparision with Algebraic CCD and Interval Arithmetic methods (Stephane Redon)
*/


///This low level demo need internal access, and intentionally doesn't include the btBulletCollisionCommon.h headerfile
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"

#include "LinearMath/btTransformUtil.h"
#include "DebugCastResult.h"

#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"

#include "GL_ShapeDrawer.h"
#include "ContinuousConvexCollision.h"
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

btVector3 angVels[numObjects];
btVector3 linVels[numObjects];

btPolyhedralConvexShape*	shapePtr[maxNumObjects];


btTransform	fromTrans[maxNumObjects];
btTransform	toTrans[maxNumObjects];


int screenWidth = 640;
int screenHeight = 480;


int main(int argc,char** argv)
{
	btContinuousConvexCollisionDemo* ccdDemo = new btContinuousConvexCollisionDemo();

	ccdDemo->setCameraDistance(40.f);

	ccdDemo->initPhysics();
	
	return glutmain(argc, argv,screenWidth,screenHeight,"Continuous Convex Collision Demo",ccdDemo);
}


void	btContinuousConvexCollisionDemo::initPhysics()
{
	fromTrans[0].setOrigin(btVector3(0,10,20));
	  toTrans[0].setOrigin(btVector3(0,10,-20));
	fromTrans[1].setOrigin(btVector3(-2,7,0));
	  toTrans[1].setOrigin(btVector3(-2,10,0));

	  btMatrix3x3 identBasis;
	identBasis.setIdentity();

	btMatrix3x3 basisA;
	basisA.setIdentity();
	basisA.setEulerZYX(0.f,-SIMD_HALF_PI,0.f);

	fromTrans[0].setBasis(identBasis);
	  toTrans[0].setBasis(basisA);

	fromTrans[1].setBasis(identBasis);
	  toTrans[1].setBasis(identBasis);

	toTrans[1].setBasis(identBasis);
	btVector3 boxHalfExtentsA(10,1,1);
	btVector3 boxHalfExtentsB(1.1f,1.1f,1.1f);
	btBoxShape*	boxA = new btBoxShape(boxHalfExtentsA);
//	btBU_Simplex1to4* boxA = new btBU_Simplex1to4(btVector3(-2,0,-2),btVector3(2,0,-2),btVector3(0,0,2),btVector3(0,2,0));
//	btBU_Simplex1to4* boxA = new btBU_Simplex1to4(btVector3(-12,0,0),btVector3(12,0,0));
	

	btBoxShape*	boxB = new btBoxShape(boxHalfExtentsB);

	shapePtr[0] = boxA;
	shapePtr[1] = boxB;

	shapePtr[0]->setMargin(0.01f);
	shapePtr[1]->setMargin(0.01f);

	for (int i=0;i<numObjects;i++)
	{
		btTransformUtil::calculateVelocity(fromTrans[i],toTrans[i],1.f,linVels[i],angVels[i]);
	}

}

//to be implemented by the demo

void btContinuousConvexCollisionDemo::clientMoveAndDisplay()
{
	displayCallback();
}


static btVoronoiSimplexSolver sVoronoiSimplexSolver;

btSimplexSolverInterface& gGjkSimplexSolver = sVoronoiSimplexSolver;

bool drawLine= false;

int minlines = 0;

int maxlines = 512;


void btContinuousConvexCollisionDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	//GL_ShapeDrawer::drawCoordSystem();

	btScalar m[16];
	int i;

	btVector3 worldBoundsMin(-1000,-1000,-1000);
	btVector3 worldBoundsMax(1000,1000,1000);


	/*for (i=0;i<numObjects;i++)
	{
		fromTrans[i].getOpenGLMatrix( m );
		m_shapeDrawer.drawOpenGL(m,shapePtr[i]);
	}
*/

	if (getDebugMode()==btIDebugDraw::DBG_DrawAabb)
	{
		i=0;//for (i=1;i<numObjects;i++)
		{
			//for each object, subdivide the from/to transform in 10 equal steps

			int numSubSteps = 10;
			for (int s=0;s<10;s++)
			{
				btScalar subStep = s * 1.f/(float)numSubSteps;
				btTransform interpolatedTrans;
				
				btTransformUtil::integrateTransform(fromTrans[i],linVels[i],angVels[i],subStep,interpolatedTrans);

				//fromTrans[i].getOpenGLMatrix(m);
				//m_shapeDrawer.drawOpenGL(m,shapePtr[i]);

				//toTrans[i].getOpenGLMatrix(m);
				//m_shapeDrawer.drawOpenGL(m,shapePtr[i]);

				interpolatedTrans.getOpenGLMatrix( m );
				m_shapeDrawer->drawOpenGL(m,shapePtr[i],btVector3(1,0,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
			}
		}
	}

	
	btMatrix3x3 mat;
	mat.setEulerZYX(yaw,pitch,roll);
	btQuaternion orn;
	mat.getRotation(orn);
	orn.setEuler(yaw,pitch,roll);
	fromTrans[1].setRotation(orn);
	toTrans[1].setRotation(orn);
	

	if (m_stepping || m_singleStep)
	{
		m_singleStep = false;
		pitch += 0.005f;
//		yaw += 0.01f;
	}
//	btVector3 fromA(-25,11,0);
//	btVector3 toA(-15,11,0);

//	btQuaternion ornFromA(0.f,0.f,0.f,1.f);
//	btQuaternion ornToA(0.f,0.f,0.f,1.f);

//	btTransform	rayFromWorld(ornFromA,fromA);
//	btTransform	rayToWorld(ornToA,toA);

	btTransform	rayFromWorld = fromTrans[0];
	btTransform	rayToWorld = toTrans[0];
	

	if (drawLine)
	{
		glBegin(GL_LINES);
		glColor3f(0, 0, 1);
		glVertex3d(rayFromWorld.getOrigin().x(), rayFromWorld.getOrigin().y(),rayFromWorld.getOrigin().z());
		glVertex3d(rayToWorld.getOrigin().x(),rayToWorld.getOrigin().y(),rayToWorld.getOrigin().z());
		glEnd();
	}

	//now perform a raycast on the shapes, in local (shape) space
	gGjkSimplexSolver.reset();
	
	//choose one of the following lines


	for (i=0;i<numObjects;i++)
	{	
		fromTrans[i].getOpenGLMatrix(m);
		m_shapeDrawer->drawOpenGL(m,shapePtr[i],btVector3(1,1,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
	}

	btDebugCastResult	rayResult1(fromTrans[0],shapePtr[0],linVels[0],angVels[0],m_shapeDrawer);
	

	for (i=1;i<numObjects;i++)
	{
		btConvexCast::CastResult	rayResult2;
		btConvexCast::CastResult*	rayResultPtr;
		if (btIDebugDraw::DBG_DrawAabb)
		{
			rayResultPtr = &rayResult1;
		} else
		{
			rayResultPtr = &rayResult2;
		}

		//GjkConvexCast	convexCaster(&gGjkSimplexSolver);
		//SubsimplexConvexCast convexCaster(&gGjkSimplexSolver);

		//optional
		btConvexPenetrationDepthSolver* penetrationDepthSolver = 0;
		btContinuousConvexCollision convexCaster(shapePtr[0],shapePtr[i],&gGjkSimplexSolver,penetrationDepthSolver );

		gGjkSimplexSolver.reset();
	
		
	
		if (convexCaster.calcTimeOfImpact(fromTrans[0],toTrans[0],fromTrans[i] ,toTrans[i] ,*rayResultPtr))
		{

			glDisable(GL_DEPTH_TEST);

			btTransform hitTrans;
			btTransformUtil::integrateTransform(fromTrans[0],linVels[0],angVels[0],rayResultPtr->m_fraction,hitTrans);

			hitTrans.getOpenGLMatrix(m);
			m_shapeDrawer->drawOpenGL(m,shapePtr[0],btVector3(0,1,0),getDebugMode(),worldBoundsMin,worldBoundsMax);

			btTransformUtil::integrateTransform(fromTrans[i],linVels[i],angVels[i],rayResultPtr->m_fraction,hitTrans);

			hitTrans.getOpenGLMatrix(m);
			m_shapeDrawer->drawOpenGL(m,shapePtr[i],btVector3(0,1,1),getDebugMode(),worldBoundsMin,worldBoundsMax);
	

		}
	}

	swapBuffers();
}


