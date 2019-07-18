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

#include "Pinch.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The Pinch shows the use of rolling friction.
///Spheres will come to a rest on a sloped plane using a constraint. Damping cannot achieve the same.
///Generally it is best to leave the rolling friction coefficient zero (or close to zero).

struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

struct TetraBunny
{
#include "../SoftDemo/bunny.inl"
};


class Pinch : public CommonRigidBodyBase
{
public:
	Pinch(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~Pinch()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 25;
        float pitch = -45;
        float yaw = 100;
        float targetPos[3] = {0, -3, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
    }
    
    void createGrip()
    {
        int count = 2;
        float mass = 2;
        btCollisionShape* shape[] = {
            new btBoxShape(btVector3(3, 3, 0.5)),
        };
        static const int nshapes = sizeof(shape) / sizeof(shape[0]);
        for (int i = 0; i < count; ++i)
        {
            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(10, 0, 0));
            startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
            createRigidBody(mass, startTransform, shape[i % nshapes]);
        }
    }
    
    virtual const btDeformableRigidDynamicsWorld* getDeformableDynamicsWorld() const
    {
        return (btDeformableRigidDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual btDeformableRigidDynamicsWorld* getDeformableDynamicsWorld()
    {
        return (btDeformableRigidDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual void renderScene()
    {
        CommonRigidBodyBase::renderScene();
        btDeformableRigidDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};

void dynamics(btScalar time, btDeformableRigidDynamicsWorld* world)
{
    btAlignedObjectArray<btRigidBody*>& rbs = world->getNonStaticRigidBodies();
    if (rbs.size()<2)
        return;
    btRigidBody* rb0 = rbs[0];
    btScalar pressTime = 0.9;
    btTransform rbTransform;
    rbTransform.setIdentity();
    btVector3 translation = btVector3(0.5,3,4);
    btVector3 velocity = btVector3(0,5,0);
    if (time < pressTime)
    {
        velocity = btVector3(0,0,-2);
        translation += velocity * time;
    }
    else
        translation += btVector3(0,0,-2) * pressTime + (time-pressTime)*velocity;
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb0->setCenterOfMassTransform(rbTransform);
    rb0->setAngularVelocity(btVector3(0,0,0));
    rb0->setLinearVelocity(velocity);
    
    btRigidBody* rb1 = rbs[1];
    translation = btVector3(0.5,3,-4);
    velocity = btVector3(0,5,0);
    if (time < pressTime)
    {
        velocity = btVector3(0,0,2);
        translation += velocity * time;
    }
    else
        translation += btVector3(0,0,2) * pressTime + (time-pressTime)*velocity;
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb1->setCenterOfMassTransform(rbTransform);
    rb1->setAngularVelocity(btVector3(0,0,0));
    rb1->setLinearVelocity(velocity);
    
    rb0->setFriction(2);
    rb1->setFriction(2);
}

void Pinch::initPhysics()
{
	m_guiHelper->setUpAxis(1);

    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver();
	m_solver = sol;

	m_dynamicsWorld = new btDeformableRigidDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
	//	m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity.setValue(0, -10, 0);
    
    getDeformableDynamicsWorld()->before_solver_callbacks.push_back(dynamics);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    //create a ground
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));

        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -25, 0));
        groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
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
        body->setFriction(0.5);

        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    // create a soft block
    {
        btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(getDeformableDynamicsWorld()->getWorldInfo(),
                                                                  TetraCube::getElements(),
                                                                  0,
                                                                  TetraCube::getNodes(),
                                                                  false, true, true);
        getDeformableDynamicsWorld()->addSoftBody(psb);
        psb->scale(btVector3(2, 2, 2));
        psb->translate(btVector3(0, 4, 0));
        psb->getCollisionShape()->setMargin(0.1);
        psb->setTotalMass(1);
        psb->setSpringStiffness(10);
        psb->setDampingCoefficient(0.01);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = 0.5;
        // add a grippers
        createGrip();
    }
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Pinch::exitPhysics()
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



class CommonExampleInterface* PinchCreateFunc(struct CommonExampleOptions& options)
{
	return new Pinch(options.m_guiHelper);
}


