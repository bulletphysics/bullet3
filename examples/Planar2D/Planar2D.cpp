/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletCollision/CollisionShapes/btBox2dShape.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h"

#include "BulletCollision/CollisionShapes/btBox2dShape.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 1

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "Planar2D.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h>  //printf debugging

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class GL_DialogDynamicsWorld;

#include "../CommonInterfaces/CommonRigidBodyBase.h"

class Planar2D : public CommonRigidBodyBase
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	btBroadphaseInterface* m_broadphase;

	btCollisionDispatcher* m_dispatcher;

	btConstraintSolver* m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btConvex2dConvex2dAlgorithm::CreateFunc* m_convexAlgo2d;
	btVoronoiSimplexSolver* m_simplexSolver;
	btMinkowskiPenetrationDepthSolver* m_pdSolver;
	btBox2dBox2dCollisionAlgorithm::CreateFunc* m_box2dbox2dAlgo;

public:
	Planar2D(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~Planar2D()
	{
		exitPhysics();
	}

	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
		float dist = 9;
		float pitch = -11;
		float yaw = 539;
		float targetPos[3] = {8.6, 10.5, -20.6};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void Planar2D::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_simplexSolver = new btVoronoiSimplexSolver();
	m_pdSolver = new btMinkowskiPenetrationDepthSolver();

	m_convexAlgo2d = new btConvex2dConvex2dAlgorithm::CreateFunc(m_simplexSolver, m_pdSolver);
	m_box2dbox2dAlgo = new btBox2dBox2dCollisionAlgorithm::CreateFunc();

	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE, CONVEX_2D_SHAPE_PROXYTYPE, m_convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE, CONVEX_2D_SHAPE_PROXYTYPE, m_convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE, BOX_2D_SHAPE_PROXYTYPE, m_convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE, BOX_2D_SHAPE_PROXYTYPE, m_box2dbox2dAlgo);

	m_broadphase = new btDbvtBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	//m_dynamicsWorld->getSolverInfo().m_erp = 1.f;
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(50.), btScalar(150.)));
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -43, 0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btScalar u = btScalar(1 * SCALING - 0.04);
		btVector3 points[3] = {btVector3(0, u, 0), btVector3(-u, -u, 0), btVector3(u, -u, 0)};
		btConvexShape* childShape0 = new btBoxShape(btVector3(btScalar(SCALING * 1), btScalar(SCALING * 1), btScalar(0.04)));
		btConvexShape* colShape = new btConvex2dShape(childShape0);
		//btCollisionShape* colShape = new btBox2dShape(btVector3(SCALING*1,SCALING*1,0.04));
		btConvexShape* childShape1 = new btConvexHullShape(&points[0].getX(), 3);
		btConvexShape* colShape2 = new btConvex2dShape(childShape1);
		btConvexShape* childShape2 = new btCylinderShapeZ(btVector3(btScalar(SCALING * 1), btScalar(SCALING * 1), btScalar(0.04)));
		btConvexShape* colShape3 = new btConvex2dShape(childShape2);

		m_collisionShapes.push_back(colShape);
		m_collisionShapes.push_back(colShape2);
		m_collisionShapes.push_back(colShape3);

		m_collisionShapes.push_back(childShape0);
		m_collisionShapes.push_back(childShape1);
		m_collisionShapes.push_back(childShape2);

		//btUniformScalingShape* colShape = new btUniformScalingShape(convexColShape,1.f);
		colShape->setMargin(btScalar(0.03));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		//		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		//		float start_y = START_POS_Y;
		//		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		btVector3 x(-ARRAY_SIZE_X, 8.0f, -20.f);
		btVector3 y;
		btVector3 deltaX(SCALING * 1, SCALING * 2, 0.f);
		btVector3 deltaY(SCALING * 2, 0.0f, 0.f);

		for (int i = 0; i < ARRAY_SIZE_X; ++i)
		{
			y = x;

			for (int j = i; j < ARRAY_SIZE_Y; ++j)
			{
				startTransform.setOrigin(y - btVector3(-10, 0, 0));

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(0, 0, 0);
				switch (j % 3)
				{
#if 1
					case 0:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
						break;
					case 1:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass, myMotionState, colShape3, localInertia);
						break;
#endif
					default:
						rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass, myMotionState, colShape2, localInertia);
				}
				btRigidBody* body = new btRigidBody(rbInfo);
				//body->setContactProcessingThreshold(colShape->getContactBreakingThreshold());
				body->setActivationState(ISLAND_SLEEPING);
				body->setLinearFactor(btVector3(1, 1, 0));
				body->setAngularFactor(btVector3(0, 0, 1));

				m_dynamicsWorld->addRigidBody(body);
				body->setActivationState(ISLAND_SLEEPING);

				//	y += -0.8*deltaY;
				y += deltaY;
			}

			x += deltaX;
		}
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Planar2D::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	if (m_dynamicsWorld)
		for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;

	delete m_convexAlgo2d;
	delete m_pdSolver;
	delete m_simplexSolver;
	delete m_box2dbox2dAlgo;

	m_dynamicsWorld = 0;
	m_solver = 0;
	m_broadphase = 0;
	m_dispatcher = 0;
	m_collisionConfiguration = 0;
	m_convexAlgo2d = 0;
	m_pdSolver = 0;
	m_simplexSolver = 0;
	m_box2dbox2dAlgo = 0;
}

CommonExampleInterface* Planar2DCreateFunc(struct CommonExampleOptions& options)
{
	return new Planar2D(options.m_guiHelper);
}
