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

#include "DeformableContact.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The DeformableContact shows the contact between deformable objects

class DeformableContact : public CommonDeformableBodyBase
{
public:
    DeformableContact(struct GUIHelperInterface* helper)
    : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~DeformableContact()
    {
    }
    void initPhysics();
    
    void exitPhysics();
    
    void resetCamera()
    {
        float dist = 12;
        float pitch = -50;
        float yaw = 120;
        float targetPos[3] = {0, -3, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void stepSimulation(float deltaTime)
    {
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        
		
		btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                //btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), fDrawFlags::Faces);// StddeformableWorld->getDrawFlags());
            }
        }
		
    }
};

void DeformableContact::initPhysics()
{
    m_guiHelper->setUpAxis(1);
    
    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    
    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    
    m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();
    
    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
    m_solver = sol;
    
    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    btVector3 gravity = btVector3(0, -10, 0);
    m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
		getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    {
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150), btScalar(25.), btScalar(150)));
        
        m_collisionShapes.push_back(groundShape);
        
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -32, 0));
        groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
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
        body->setFriction(2);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    // create a piece of cloth
    {
        btScalar s = 4;
        btScalar h = 0;
        
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
                                                         btVector3(+s, h, -s),
                                                         btVector3(-s, h, +s),
                                                         btVector3(+s, h, +s),
                                                         20,20,
                                                         1 + 2 + 4 + 8, true);
        
        psb->getCollisionShape()->setMargin(0.05);
        psb->generateBendingConstraints(2);
        psb->setSpringStiffness(10);
        psb->setTotalMass(1);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = 0;
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        getDeformableDynamicsWorld()->addSoftBody(psb);
        
        btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(10,1, true);
        getDeformableDynamicsWorld()->addForce(psb, mass_spring);
        m_forces.push_back(mass_spring);
        
        btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);

        
        h = 2;
        s = 2;
        btSoftBody* psb2 = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
                                                          btVector3(+s, h, -s),
                                                          btVector3(-s, h, +s),
                                                          btVector3(+s, h, +s),
                                                          10,10,
                                                          0, true);
        psb2->getCollisionShape()->setMargin(0.05);
        psb2->generateBendingConstraints(2);
        psb2->setSpringStiffness(10);
        psb2->setTotalMass(1);
        psb2->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb2->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb2->m_cfg.kDF = 0.5;
        psb2->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb2->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb2->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        psb->translate(btVector3(3.5,0,0));
        getDeformableDynamicsWorld()->addSoftBody(psb2);
        
        btDeformableMassSpringForce* mass_spring2 = new btDeformableMassSpringForce(10,1, true);
        getDeformableDynamicsWorld()->addForce(psb2, mass_spring2);
        m_forces.push_back(mass_spring2);
        
        btDeformableGravityForce* gravity_force2 =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb2, gravity_force2);
        m_forces.push_back(gravity_force2);
    }
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	int numInstances = m_guiHelper->getRenderInterface()->getTotalNumInstances();
	double rgbaColors[3][4] = { { 1, 0, 0, 1 } , { 0, 1, 0, 1 } ,{ 0, 0, 1, 1 } };

	for (int i = 0; i < numInstances; i++)
	{
		m_guiHelper->changeInstanceFlags(i, B3_INSTANCE_DOUBLE_SIDED);
	}
	
}

void DeformableContact::exitPhysics()
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

class CommonExampleInterface* DeformableContactCreateFunc(struct CommonExampleOptions& options)
{
    return new DeformableContact(options.m_guiHelper);
}


