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
	LinearConvexCastDemo implements an efficient continuous collision detection algorithm.
	Both linear and angular velocities are supported. Gjk or Simplex based methods.
	Motion using Exponential Map.
	Comparison with Screwing Motion. 
	Also comparision with Algebraic CCD and Interval Arithmetic methods (Stephane Redon)
*/


///Low level demo, doesn't include btBulletCollisionCommon.h
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"

#ifdef USE_ALGEBRAIC_CCD
#include "NarrowPhaseCollision/BU_CollisionPair.h"
#endif //USE_ALGEBRAIC_CCD

#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"

#include "GL_ShapeDrawer.h"
#include "LinearConvexCastDemo.h"
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;


btPolyhedralConvexShape*	shapePtr[maxNumObjects];

btTransform tr[numObjects];
int screenWidth = 640;
int screenHeight = 480;
void DrawRasterizerLine(float const* , float const*, int)
{

}

int main(int argc,char** argv)
{

	LinearConvexCastDemo* linearCastDemo = new LinearConvexCastDemo();

	linearCastDemo->initPhysics();


	return glutmain(argc, argv,screenWidth,screenHeight,"Linear Convex Cast Demo",linearCastDemo);
}

void LinearConvexCastDemo::initPhysics()
{
	setCameraDistance(30.f);
	tr[0].setOrigin(btVector3(0,0,0));
	tr[1].setOrigin(btVector3(0,10,0));

	btMatrix3x3 basisA;
	basisA.setValue(0.99999958f,0.00022980258f,0.00090992288f,
		-0.00029313788f,0.99753088f,0.070228584f,
		-0.00089153741f,-0.070228823f,0.99753052f);

	btMatrix3x3 basisB;
	basisB.setValue(1.0000000f,4.4865553e-018f,-4.4410586e-017f,
		4.4865495e-018f,0.97979438f,0.20000751f,
		4.4410586e-017f,-0.20000751f,0.97979438f);

	tr[0].setBasis(basisA);
	tr[1].setBasis(basisB);



	btVector3 boxHalfExtentsA(0.2,4,4);
	btVector3 boxHalfExtentsB(6,6,6);

	btBoxShape*	boxA = new btBoxShape(boxHalfExtentsA);
/*	btBU_Simplex1to4	boxB;
	boxB.addVertex(btPoint3(-5,0,-5));
	boxB.addVertex(btPoint3(5,0,-5));
	boxB.addVertex(btPoint3(0,0,5));
	boxB.addVertex(btPoint3(0,5,0));
*/

	btBoxShape*	boxB = new btBoxShape(boxHalfExtentsB);
	shapePtr[0] = boxA;
	shapePtr[1] = boxB;

	shapePtr[0]->setMargin(0.01f);
	shapePtr[1]->setMargin(0.01f);

	btTransform tr;
	tr.setIdentity();
}

//to be implemented by the demo

void LinearConvexCastDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"

static btVoronoiSimplexSolver sVoronoiSimplexSolver;

btSimplexSolverInterface& gGjkSimplexSolver = sVoronoiSimplexSolver;

bool drawLine= false;

void LinearConvexCastDemo::displayCallback(void) 
{
	updateCamera();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	//GL_ShapeDrawer::drawCoordSystem();

	btScalar m[16];
	int i;

	for (i=0;i<numObjects;i++)
	{
		tr[i].getOpenGLMatrix( m );
		GL_ShapeDrawer::drawOpenGL(m,shapePtr[i],btVector3(1,1,1),getDebugMode());
	}

	
	int shapeIndex = 1;

	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	tr[shapeIndex].setRotation(orn);
	

	if (m_stepping || m_singleStep)
	{
		m_singleStep = false;
		pitch += 0.005f;
		yaw += 0.01f;
	}

	btVector3 fromA(-25,11,0);
	btVector3 toA(15,11,0);

	btQuaternion ornFromA(0.f,0.f,0.f,1.f);
	btQuaternion ornToA(0.f,0.f,0.f,1.f);

	btTransform	rayFromWorld(ornFromA,fromA);
	btTransform	rayToWorld(ornToA,toA);

	tr[0] = rayFromWorld;

	if (drawLine)
	{
		glBegin(GL_LINES);
		glColor3f(0, 0, 1);
		glVertex3d(rayFromWorld.getOrigin().x(), rayFromWorld.getOrigin().y(),rayFromWorld.getOrigin().z());
		glVertex3d(rayToWorld.getOrigin().x(),rayToWorld.getOrigin().y(),rayToWorld.getOrigin().z());
		glEnd();
	}

	//now perform a raycast on the shapes, in local (shape) space
	
	//choose one of the following lines

	

	for (i=1;i<numObjects;i++)
	{
		btContinuousConvexCollision convexCaster0(shapePtr[0],shapePtr[i],&gGjkSimplexSolver,0);
		btGjkConvexCast	convexCaster1(shapePtr[0],shapePtr[i],&gGjkSimplexSolver);
		
		//BU_CollisionPair (algebraic version) is currently broken, will look into this
		//BU_CollisionPair convexCaster2(shapePtr[0],shapePtr[i]);
		btSubsimplexConvexCast convexCaster3(shapePtr[0],shapePtr[i],&gGjkSimplexSolver);
				
		gGjkSimplexSolver.reset();

		btConvexCast::CastResult rayResult;
		
	

		if (convexCaster3.calcTimeOfImpact(rayFromWorld,rayToWorld,tr[i],tr[i],rayResult))
		{

			glDisable(GL_DEPTH_TEST);
			btVector3 hitPoint;
			hitPoint.setInterpolate3(rayFromWorld.getOrigin(),rayToWorld.getOrigin(),rayResult.m_fraction);
			
			//draw the raycast result
			glBegin(GL_LINES);
			glColor3f(1, 1, 1);
			glVertex3d(rayFromWorld.getOrigin().x(), rayFromWorld.getOrigin().y(),rayFromWorld.getOrigin().z());
			glVertex3d(hitPoint.x(),hitPoint.y(),hitPoint.z());
			glEnd();
			glEnable(GL_DEPTH_TEST);

			btTransform	toTransWorld;
			toTransWorld = tr[0];
			toTransWorld.setOrigin(hitPoint);

			toTransWorld.getOpenGLMatrix( m );
			GL_ShapeDrawer::drawOpenGL(m,shapePtr[0],btVector3(0,1,1),getDebugMode());


		}
	}

	glFlush();
    glutSwapBuffers();
}
