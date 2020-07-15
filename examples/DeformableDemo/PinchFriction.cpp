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
#include "PinchFriction.h"
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

///The PinchFriction shows the frictional contacts among volumetric deformable objects

struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

class PinchFriction : public CommonDeformableBodyBase
{
    btAlignedObjectArray<btDeformableLagrangianForce*> m_forces;
public:
    PinchFriction(struct GUIHelperInterface* helper)
    : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~PinchFriction()
    {
    }
    void initPhysics();
    
    void exitPhysics();
    
    void resetCamera()
    {
        float dist = 25;
        float pitch = -30;
        float yaw = 100;
        float targetPos[3] = {0, -0, 0};
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
        float mass = 1e6;
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
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
    }
    
    virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
    {
        return false;
    }
    virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
    {
        return false;
    }
    virtual void removePickingConstraint(){}
};

void dynamics2(btScalar time, btDeformableMultiBodyDynamicsWorld* world)
{
    btAlignedObjectArray<btRigidBody*>& rbs = world->getNonStaticRigidBodies();
    if (rbs.size()<2)
        return;
    btRigidBody* rb0 = rbs[0];
    btScalar pressTime = 0.45;
    btScalar liftTime = 5;
    btScalar shiftTime = 1.75;
    btScalar holdTime = 7.5;
    btScalar dropTime = 8.3;
    btTransform rbTransform;
    rbTransform.setIdentity();
    btVector3 translation;
    btVector3 velocity;
    
    btVector3 initialTranslationLeft = btVector3(0.5,3,4);
    btVector3 initialTranslationRight = btVector3(0.5,3,-4);
    btVector3 PinchFrictionVelocityLeft = btVector3(0,0,-2);
    btVector3 PinchFrictionVelocityRight = btVector3(0,0,2);
    btVector3 liftVelocity = btVector3(0,2,0);
    btVector3 shiftVelocity = btVector3(0,0,0);
    btVector3 holdVelocity = btVector3(0,0,0);
    btVector3 openVelocityLeft = btVector3(0,0,4);
    btVector3 openVelocityRight = btVector3(0,0,-4);
    
    if (time < pressTime)
    {
        velocity = PinchFrictionVelocityLeft;
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * time;
    }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * pressTime + liftVelocity * (time - pressTime);
        
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityLeft;
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationLeft + PinchFrictionVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb0->setCenterOfMassTransform(rbTransform);
    rb0->setAngularVelocity(btVector3(0,0,0));
    rb0->setLinearVelocity(velocity);
    
    btRigidBody* rb1 = rbs[1];
    if (time < pressTime)
    {
        velocity = PinchFrictionVelocityRight;
        translation = initialTranslationRight + PinchFrictionVelocityRight * time;
    }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationRight + PinchFrictionVelocityRight * pressTime + liftVelocity * (time - pressTime);
        
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationRight + PinchFrictionVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationRight + PinchFrictionVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityRight;
        translation = initialTranslationRight + PinchFrictionVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationRight + PinchFrictionVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb1->setCenterOfMassTransform(rbTransform);
    rb1->setAngularVelocity(btVector3(0,0,0));
    rb1->setLinearVelocity(velocity);
    
    rb0->setFriction(200);
    rb1->setFriction(200);
}

void PinchFriction::initPhysics()
{
    m_guiHelper->setUpAxis(1);
    
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    
    m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();
    
    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
    m_solver = sol;
    
    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    btVector3 gravity = btVector3(0, -10, 0);
    m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
    getDeformableDynamicsWorld()->setSolverCallback(dynamics2);
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
        
        psb->scale(btVector3(2, 2, 1));
        psb->translate(btVector3(0, 2.1, 2.2));
        psb->getCollisionShape()->setMargin(0.025);
        psb->setSpringStiffness(10);
        psb->setTotalMass(.6);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = 2;
        btSoftBodyHelpers::generateBoundaryFaces(psb);
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        getDeformableDynamicsWorld()->addSoftBody(psb);

        btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);
        
        btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(6,6,.003);
        getDeformableDynamicsWorld()->addForce(psb, neohookean);
        m_forces.push_back(neohookean);
    }
    
    // create a second soft block
    {
        btSoftBody* psb2 = btSoftBodyHelpers::CreateFromTetGenData(getDeformableDynamicsWorld()->getWorldInfo(),
                                                                  TetraCube::getElements(),
                                                                  0,
                                                                  TetraCube::getNodes(),
                                                                  false, true, true);
        
        psb2->scale(btVector3(2, 2, 1));
        psb2->translate(btVector3(0, 2.1, -2.2));
        psb2->getCollisionShape()->setMargin(0.025);
        psb2->setTotalMass(.6);
        psb2->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb2->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb2->m_cfg.kDF = 2;
        psb2->setSpringStiffness(10);
        psb2->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb2->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb2->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        btSoftBodyHelpers::generateBoundaryFaces(psb2);
        getDeformableDynamicsWorld()->addSoftBody(psb2);
        
        btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb2, gravity_force);
        m_forces.push_back(gravity_force);
        
        btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(6,6,.003);
        getDeformableDynamicsWorld()->addForce(psb2, neohookean);
        m_forces.push_back(neohookean);
    }
    
    // create a third soft block
    {
        btSoftBody* psb3 = btSoftBodyHelpers::CreateFromTetGenData(getDeformableDynamicsWorld()->getWorldInfo(),
                                                                   TetraCube::getElements(),
                                                                   0,
                                                                   TetraCube::getNodes(),
                                                                   false, true, true);
        
        psb3->scale(btVector3(2, 2, 1));
        psb3->translate(btVector3(0, 2.1, 0));
        psb3->getCollisionShape()->setMargin(0.025);
        psb3->setTotalMass(.6);
        psb3->setSpringStiffness(10);
        psb3->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb3->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb3->m_cfg.kDF = 2;
        psb3->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb3->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        psb3->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
        btSoftBodyHelpers::generateBoundaryFaces(psb3);
        getDeformableDynamicsWorld()->addSoftBody(psb3);
        
        btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb3, gravity_force);
        m_forces.push_back(gravity_force);
        
        btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(6,6,.003);
        getDeformableDynamicsWorld()->addForce(psb3, neohookean);
        m_forces.push_back(neohookean);
    }
    getDeformableDynamicsWorld()->setImplicit(false);
    // add a pair of grippers
    createGrip();
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void PinchFriction::exitPhysics()
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



class CommonExampleInterface* PinchFrictionCreateFunc(struct CommonExampleOptions& options)
{
    return new PinchFriction(options.m_guiHelper);
}


