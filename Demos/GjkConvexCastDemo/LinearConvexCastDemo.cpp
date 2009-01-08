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

#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"



#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"



#include "GL_ShapeDrawer.h"
#include "LinearConvexCastDemo.h"
#include "GlutStuff.h"

static btVoronoiSimplexSolver sVoronoiSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sVoronoiSimplexSolver;

static float yaw=0.f,pitch=0.f,roll=0.f;
static const int maxNumObjects = 4;
static const int numObjects = 2;

static btPolyhedralConvexShape*	shapePtr[maxNumObjects];

static btTransform tr[numObjects];



void LinearConvexCastDemo::initPhysics()
{

	setCameraDistance(10.f);

	tr[0].setIdentity();
	tr[0].setOrigin( btVector3( 0.0f, 5.5f, 0.0f ) );

	tr[1].setIdentity();
	tr[1].setOrigin( btVector3( 0.0f, 0.0f, 0.0f ) );

	// Pyramide
	float r = 1.0f;
	float h = 2.0f;

	btConvexHullShape* shapeA = new btConvexHullShape;
	shapeA->addPoint( btVector3( 0.0f,  0.75f * h, 0.0f ) );
	shapeA->addPoint( btVector3(   -r, -0.25f * h,    r ) );
	shapeA->addPoint( btVector3(    r, -0.25f * h,    r ) );
	shapeA->addPoint( btVector3(    r, -0.25f * h,   -r ) );
	shapeA->addPoint( btVector3(   -r, -0.25f * h,   -r ) );



	// Triangle
	btConvexHullShape* shapeB = new btConvexHullShape;
	shapeB->addPoint( btVector3(  0.0f,  1.0f, 0.0f ) );
	shapeB->addPoint( btVector3(  1.0f, -1.0f, 0.0f ) );
	shapeB->addPoint( btVector3( -1.0f, -1.0f, 0.0f ) );

	shapePtr[0] = shapeA;
	shapePtr[1] = shapeB;

	shapePtr[0]->setMargin( 0.01f );
	shapePtr[1]->setMargin( 0.01f );
}

//to be implemented by the demo
void LinearConvexCastDemo::clientMoveAndDisplay()
{
	displayCallback();
}

LinearConvexCastDemo::~LinearConvexCastDemo()
{
	delete shapePtr[0];
	delete shapePtr[1];
}

void LinearConvexCastDemo::displayCallback(void)
{
	updateCamera();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_LIGHTING);

	GL_ShapeDrawer::drawCoordSystem();

	

	static btScalar angle = 0.f;
	angle+=getDeltaTimeMicroseconds()/1000000.0;

	tr[1].setRotation(btQuaternion(btVector3(1,0,0),angle));

	btTransform toA, toB;
	toA = tr[0];
	toA.setOrigin( btVector3( 0.0f, 0.f, 0.0f  ) );
	toB = tr[1];
	toB.setOrigin( btVector3( 0.0f, 0.0f, 0.0f  ) );


	gGjkSimplexSolver.reset();
	
	
	btVector3 worldBoundsMin(-1000,-1000,-1000);
	btVector3 worldBoundsMax(1000,1000,1000);


	//btGjkConvexCast convexCaster(shapePtr[ 0 ], shapePtr[ 1 ], &gGjkSimplexSolver );
	btSubsimplexConvexCast convexCaster( shapePtr[ 0 ], shapePtr[ 1 ], &gGjkSimplexSolver );

	btConvexCast::CastResult result;

	result.m_hitPoint.setValue(0,0,0);

	convexCaster.calcTimeOfImpact( tr[ 0 ], toA, tr[ 1 ], toB, result );

	btScalar m1[16], m2[16],m3[16];
	tr[ 0 ].getOpenGLMatrix( m1 );
	tr[ 1 ].getOpenGLMatrix( m2 );

	btSphereShape	sphere(0.2);

	btTransform tmp = tr[0];
	tmp.setOrigin(result.m_hitPoint);
	tmp.getOpenGLMatrix(m3);
	m_shapeDrawer->drawOpenGL( m3, &sphere, btVector3( 1, 0, 1 ), getDebugMode() ,worldBoundsMin,worldBoundsMax);


	m_shapeDrawer->drawOpenGL( m1, shapePtr[ 0 ], btVector3( 1, 0, 0 ), getDebugMode() ,worldBoundsMin,worldBoundsMax);
	m_shapeDrawer->drawOpenGL( m2, shapePtr[ 1 ], btVector3( 1, 0, 0 ), getDebugMode() ,worldBoundsMin,worldBoundsMax);

	btVector3 originA, originB;
	originA.setInterpolate3( tr[ 0 ].getOrigin(), toA.getOrigin(), result.m_fraction );
	originB.setInterpolate3( tr[ 1 ].getOrigin(), toB.getOrigin(), result.m_fraction );

	btTransform A = tr[ 0 ];
	A.setOrigin( originA );

	btTransform B = tr[ 1 ];
	B.setOrigin( originB );

	A.getOpenGLMatrix( m1 );
	B.getOpenGLMatrix( m2 );

	m_shapeDrawer->drawOpenGL( m1, shapePtr[ 0 ], btVector3( 1, 1, 0 ), getDebugMode() ,worldBoundsMin,worldBoundsMax);
	m_shapeDrawer->drawOpenGL( m2, shapePtr[ 1 ], btVector3( 1, 1, 0 ), getDebugMode() ,worldBoundsMin,worldBoundsMax);

	glFlush();
    glutSwapBuffers();
}
