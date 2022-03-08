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

#include "ReducedGrasp.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The BasicTest shows the contact between volumetric deformable objects and rigid objects.
// static btScalar E = 50;
// static btScalar nu = 0.3;
static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0001;
static int num_modes = 20;

class ReducedGrasp : public CommonDeformableBodyBase
{
public:
    ReducedGrasp(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~ReducedGrasp()
    {
    }
    void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 10;
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
        btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 0.25));
        {
            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(0,1,0));
            startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
            createRigidBody(mass, startTransform, shape);
        }
        {
            btTransform startTransform;
            startTransform.setIdentity();
            startTransform.setOrigin(btVector3(0,1,-4));
            startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
            createRigidBody(mass, startTransform, shape);
        }
        
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
            // btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(deformableWorld->getSoftBodyArray()[i]);
            // {
            //     btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
            //     btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            // }

            // for (int p = 0; p < rsb->m_nodeRigidContacts.size(); ++p)
            // {
            //     deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[rsb->m_contactNodesList[p]].m_x, 0.1, btVector3(0, 1, 0));
            // }

            btSoftBody* psb = static_cast<btSoftBody*>(deformableWorld->getSoftBodyArray()[i]);
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }

    static void GripperDynamics(btScalar time, btDeformableMultiBodyDynamicsWorld* world);
};

void ReducedGrasp::GripperDynamics(btScalar time, btDeformableMultiBodyDynamicsWorld* world)
{
    btAlignedObjectArray<btRigidBody*>& rbs = world->getNonStaticRigidBodies();
    if (rbs.size()<2)
        return;
    btRigidBody* rb0 = rbs[0];
    // btScalar pressTime = 0.9;
    // btScalar pressTime = 0.96;
    btScalar pressTime = 1.26;
    btScalar liftTime = 2.5;
    btScalar shiftTime = 6;
    btScalar holdTime = 7;
    btScalar dropTime = 10;
    // btScalar holdTime = 500;
    // btScalar dropTime = 1000;
    btTransform rbTransform;
    rbTransform.setIdentity();
    btVector3 translation;
    btVector3 velocity;
    
    btVector3 initialTranslationLeft = btVector3(0,1,0);            // inner face has z=2
    btVector3 initialTranslationRight = btVector3(0,1,-4);          // inner face has z=-2
    btVector3 pinchVelocityLeft = btVector3(0,0,-1);
    btVector3 pinchVelocityRight = btVector3(0,0,1);
    btVector3 liftVelocity = btVector3(0,4,0);
    btVector3 shiftVelocity = btVector3(0,0,2);
    btVector3 holdVelocity = btVector3(0,0,0);
    btVector3 openVelocityLeft = btVector3(0,0,4);
    btVector3 openVelocityRight = btVector3(0,0,-4);
    
    if (time < pressTime)
    {
        velocity = pinchVelocityLeft;
        translation = initialTranslationLeft + pinchVelocityLeft * time;
    }
    // else
    // {
    //     velocity = btVector3(0, 0, 0);
    //     translation = initialTranslationLeft + pinchVelocityLeft * pressTime;
    // }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (time - pressTime);
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityLeft;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb0->setCenterOfMassTransform(rbTransform);
    rb0->setAngularVelocity(btVector3(0,0,0));
    rb0->setLinearVelocity(velocity);
    
    btRigidBody* rb1 = rbs[1];
    if (time < pressTime)
    {
        velocity = pinchVelocityRight;
        translation = initialTranslationRight + pinchVelocityRight * time;
    }
    // else
    // {
    //     velocity = btVector3(0, 0, 0);
    //     translation = initialTranslationRight + pinchVelocityRight * pressTime;
    // }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (time - pressTime);
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityRight;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb1->setCenterOfMassTransform(rbTransform);
    rb1->setAngularVelocity(btVector3(0,0,0));
    rb1->setLinearVelocity(velocity);
    
    rb0->setFriction(20);
    rb1->setFriction(20);
}

void ReducedGrasp::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    // btDeformableBodySolver* solver = new btDeformableBodySolver();
    btReducedDeformableBodySolver* solver = new btReducedDeformableBodySolver();

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(solver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, solver);
    btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
    getDeformableDynamicsWorld()->setSolverCallback(GripperDynamics);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric reduced deformable body
    {   
        std::string file_path("../../../data/reduced_cube/");
        std::string vtk_file("cube_mesh.vtk");
        btReducedDeformableBody* rsb = btReducedDeformableBodyHelpers::createReducedDeformableObject(
                                            getDeformableDynamicsWorld()->getWorldInfo(),
                                            file_path,
                                            vtk_file,
                                            num_modes,
                                            false);
                                            
        getDeformableDynamicsWorld()->addSoftBody(rsb);
        rsb->getCollisionShape()->setMargin(0.015);
        
        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 1, -2));
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
    }

    // create full deformable cube
    {
        // std::string filepath("../../../examples/SoftDemo/cube/");
        // std::string filename = filepath + "mesh.vtk";
        // btSoftBody* psb = btSoftBodyHelpers::CreateFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), filename.c_str());
        
        // // psb->scale(btVector3(2, 2, 2));
        // psb->translate(btVector3(0, 1, -2));
        // psb->getCollisionShape()->setMargin(0.05);
        // psb->setTotalMass(28.6);
        // psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        // psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        // psb->m_cfg.kDF = .5;
        // psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        // psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        // getDeformableDynamicsWorld()->addSoftBody(psb);
        // btSoftBodyHelpers::generateBoundaryFaces(psb);
        
        // btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        // getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        // m_forces.push_back(gravity_force);
        
        // btScalar E = 10000;
        // btScalar nu = 0.3;
        // btScalar lambda = E * nu / ((1 + nu) * (1 - 2 * nu));
        // btScalar mu = E / (2 * (1 + nu));
        // btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(lambda, mu, 0.02);
        // // neohookean->setPoissonRatio(0.3);
        // // neohookean->setYoungsModulus(25);
        // neohookean->setDamping(0.01);
        // psb->m_cfg.drag = 0.001;
        // getDeformableDynamicsWorld()->addForce(psb, neohookean);
        // m_forces.push_back(neohookean);
    }

    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(false);
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

void ReducedGrasp::exitPhysics()
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



class CommonExampleInterface* ReducedGraspCreateFunc(struct CommonExampleOptions& options)
{
    return new ReducedGrasp(options.m_guiHelper);
}


