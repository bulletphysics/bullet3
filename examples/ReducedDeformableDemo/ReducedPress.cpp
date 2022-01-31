/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "ReducedPress.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedDeformableBody.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedDeformableBodyHelpers.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The BasicTest shows the contact between volumetric deformable objects and rigid objects.
static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0;
static int num_modes = 20;

class ReducedPress : public CommonDeformableBodyBase
{
public:
    ReducedPress(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~ReducedPress()
    {
    }
    void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 5;
        float pitch = -10;
        float yaw = 90;
        
        // float dist = 25;
        // float pitch = -30;
        // float yaw = 100;
        float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
        // float internalTimeStep = 1. / 60.f;
        // m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
    }
    
    void createGrip()
    {
        int count = 2;
        float mass = 1e6;
        btCollisionShape* shape = new btBoxShape(btVector3(1, 0.25, 1));
        {
            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(0,2,0));
            // startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
            createRigidBody(mass, startTransform, shape);
        }
        // {
        //     btTransform startTransform;
        //     startTransform.setIdentity();
        //     startTransform.setOrigin(btVector3(0,1,-2));
        //     startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
        //     createRigidBody(mass, startTransform, shape);
        // }
        
    }

    void Ctor_RbUpStack()
    {
        float mass = 8;
        btCollisionShape* shape = new btBoxShape(btVector3(0.25, 2, 0.5));
        btTransform startTransform;
        startTransform.setIdentity();

        startTransform.setOrigin(btVector3(0,9.5,0));
        btRigidBody* rb1 = createRigidBody(mass, startTransform, shape);
        rb1->setLinearVelocity(btVector3(0, 0, 0));
        rb1->setFriction(0.7);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(deformableWorld->getSoftBodyArray()[i]);
            {
                btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }

            for (int p = 0; p < rsb->m_nodeRigidContacts.size(); ++p)
            {
                deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[rsb->m_contactNodesList[p]].m_x, 0.1, btVector3(0, 1, 0));
            }
        }
    }

    static void GripperDynamics(btScalar time, btDeformableMultiBodyDynamicsWorld* world);
};

void ReducedPress::GripperDynamics(btScalar time, btDeformableMultiBodyDynamicsWorld* world)
{
    btAlignedObjectArray<btRigidBody*>& rbs = world->getNonStaticRigidBodies();
    if (rbs.size()>1)
        return;
    btRigidBody* rb0 = rbs[0];
    btScalar pressTime = 0.75;
    btTransform rbTransform;
    rbTransform.setIdentity();
    btVector3 translation;
    btVector3 velocity;
    
    btVector3 initialTranslationLeft = btVector3(0,2,0);            // inner face has z=2
    btVector3 pinchVelocityLeft = btVector3(0,-1,0);
    
    if (time < pressTime)
    {
        velocity = pinchVelocityLeft;
        translation = initialTranslationLeft + pinchVelocityLeft * time;
    }
    else
    {
        velocity = btVector3(0, 0, 0);
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime;
    }

    rbTransform.setOrigin(translation);
    // rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb0->setCenterOfMassTransform(rbTransform);
    rb0->setAngularVelocity(btVector3(0,0,0));
    rb0->setLinearVelocity(velocity);
        
    rb0->setFriction(20);
}

void ReducedPress::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    btReducedDeformableBodySolver* reducedSoftBodySolver = new btReducedDeformableBodySolver();
    btVector3 gravity = btVector3(0, 0, 0);
    reducedSoftBodySolver->setGravity(gravity);

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
    getDeformableDynamicsWorld()->setSolverCallback(GripperDynamics);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric reduced deformable body
    {   
        btReducedDeformableBody* rsb = btReducedDeformableBodyHelpers::createReducedCube(getDeformableDynamicsWorld()->getWorldInfo(), num_modes);

        getDeformableDynamicsWorld()->addSoftBody(rsb);
        rsb->getCollisionShape()->setMargin(0.015);
        
        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 1, 0));
        // init_transform.setRotation(btQuaternion(0, SIMD_PI / 2.0, SIMD_PI / 2.0));
        // init_transform.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 2.0));
        rsb->transform(init_transform);

        rsb->setStiffnessScale(100);
        rsb->setDamping(damping_alpha, damping_beta);

        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);
        
        // rsb->setRigidAngularVelocity(btVector3(1, 0, 0));
    }

    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(true);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_friction = 1;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;
    
    // grippers
    createGrip();

    // rigid block
    // Ctor_RbUpStack();

    // {
    //     float mass = 10;
    //     btCollisionShape* shape = new btBoxShape(btVector3(0.25, 2, 0.5));
    //     btTransform startTransform;
    //     startTransform.setIdentity();
    //     startTransform.setOrigin(btVector3(0,4,0));
    //     btRigidBody* rb1 = createRigidBody(mass, startTransform, shape);
    //     rb1->setLinearVelocity(btVector3(0, 0, 0));
    // }

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

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void ReducedPress::exitPhysics()
{
    //cleanup in the reverse order of creation/initialization
    removePickingConstraint();
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
    // delete forces
    for (int j = 0; j < m_forces.size(); j++)
    {
        btDeformableLagrangianForce* force = m_forces[j];
        delete force;
    }
    m_forces.clear();
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



class CommonExampleInterface* ReducedPressCreateFunc(struct CommonExampleOptions& options)
{
    return new ReducedPress(options.m_guiHelper);
}


