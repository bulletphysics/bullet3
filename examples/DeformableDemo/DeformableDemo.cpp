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

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "DeformableDemo.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The DeformableDemo shows the use of rolling friction.
///Spheres will come to a rest on a sloped plane using a constraint. Damping cannot achieve the same.
///Generally it is best to leave the rolling friction coefficient zero (or close to zero).
class DeformableDemo : public CommonRigidBodyBase
{
public:
	DeformableDemo(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~DeformableDemo()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
//        float dist = 30;
//        float pitch = -14;
//        float yaw = 0;
//        float targetPos[3] = {0, 0, 0};
        float dist = 45;
        float pitch = -45;
        float yaw = 100;
        float targetPos[3] = {0,0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    virtual const btDeformableRigidDynamicsWorld* getDeformableDynamicsWorld() const
    {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btDeformableRigidDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual btDeformableRigidDynamicsWorld* getDeformableDynamicsWorld()
    {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btDeformableRigidDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual void renderScene()
    {
        CommonRigidBodyBase::renderScene();
        btDeformableRigidDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            //if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};

void DeformableDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
//    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDeformableRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, deformableBodySolver);
	//	m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    getDeformableDynamicsWorld()->getSoftDynamicsWorld()->getWorldInfo().m_gravity.setValue(0, -10, 0);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    {
        ///create a few basic rigid bodies
//        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(10.), btScalar(5.), btScalar(25.)));
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(25.), btScalar(50.)));

        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -30, 0));
//        groundTransform.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI * 0.03));
        //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
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
        body->setFriction(.5);

        //add the body to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
//
//    {
//        ///create a few basic rigid bodies
//        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(100.), btScalar(50.)));
//
//        m_collisionShapes.push_back(groundShape);
//
//        btTransform groundTransform;
//        groundTransform.setIdentity();
//        groundTransform.setOrigin(btVector3(0, 0, -54));
//        //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
//        btScalar mass(0.);
//
//        //rigidbody is dynamic if and only if mass is non zero, otherwise static
//        bool isDynamic = (mass != 0.f);
//
//        btVector3 localInertia(0, 0, 0);
//        if (isDynamic)
//            groundShape->calculateLocalInertia(mass, localInertia);
//
//        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
//        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
//        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
//        btRigidBody* body = new btRigidBody(rbInfo);
//        body->setFriction(.1);
//        //add the body to the dynamics world
//        m_dynamicsWorld->addRigidBody(body);
//    }
//
//    {
//        // add a simple deformable body
//        const btVector3 s(3,2,1); // side length
//        const btVector3 p(0,30,0); // origin;
//        const btVector3 h = s * 0.5;
//        const btVector3 c[] = {p + h * btVector3(-1, -1, -1),
//            p + h * btVector3(+1, -1, -1),
//            p + h * btVector3(-1, +1, -1),
//            p + h * btVector3(+1, +1, -1),
//            p + h * btVector3(-1, -1, +1),
//            p + h * btVector3(+1, -1, +1),
//            p + h * btVector3(-1, +1, +1),
//            p + h * btVector3(+1, +1, +1)};
//        btSoftBody* psb = btSoftBodyHelpers::CreateFromConvexHull(getDeformableDynamicsWorld()->getSoftDynamicsWorld()->getWorldInfo(), c, 8);
//        psb->generateBendingConstraints(2);
//        psb->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
//        psb->setTotalMass(150);
//        getDeformableDynamicsWorld()->addSoftBody(psb);
//    }
    {
        const btScalar s = 8;
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getSoftDynamicsWorld()->getWorldInfo(), btVector3(-s, 0, -s),
                                                         btVector3(+s, 0, -s),
                                                         btVector3(-s, 0, +s),
                                                         btVector3(+s, 0, +s),
                                                         10, 10,
                                                         //        31,31,
                                                          1 + 2 + 4 + 8, true);
//                                                         0, true);
        
        psb->getCollisionShape()->setMargin(0.5);
//        btSoftBody::Material* pm = psb->appendMaterial();
//        pm->m_kLST = 0.4 * 1000;
//        pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
        psb->generateBendingConstraints(2);
        psb->setTotalMass(1);
        psb->setDampingCoefficient(0.01);
        getDeformableDynamicsWorld()->addSoftBody(psb);
    }
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void DeformableDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
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
}


class CommonExampleInterface* DeformableCreateFunc(struct CommonExampleOptions& options)
{
	return new DeformableDemo(options.m_guiHelper);
}
