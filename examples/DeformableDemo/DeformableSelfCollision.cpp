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
#include "DeformableSelfCollision.h"
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

///The DeformableSelfCollision shows deformable self collisions
class DeformableSelfCollision : public CommonDeformableBodyBase
{
public:
    DeformableSelfCollision(struct GUIHelperInterface* helper)
    : CommonDeformableBodyBase(helper)
    {
        m_maxPickingForce = btScalar(0.004);
    }
    virtual ~DeformableSelfCollision()
    {
    }
    void initPhysics();
    
    void exitPhysics();
    
    void resetCamera()
    {
        float dist = 2.0;
        float pitch = -8;
        float yaw = 100;
        float targetPos[3] = {0, -1.0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void stepSimulation(float deltaTime)
    {
        float internalTimeStep = 1.f / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
    }
    
    void addCloth(const btVector3& origin);
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
    }
};

void DeformableSelfCollision::initPhysics()
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
    //    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
    //    m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
    btVector3 gravity = btVector3(0, btScalar(-9.8), 0);
    m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
    
    //    getDeformableDynamicsWorld()->before_solver_callbacks.push_back(dynamics);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    {
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(2.5), btScalar(150.)));
        groundShape->setMargin(btScalar(0.02));
        m_collisionShapes.push_back(groundShape);
        
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -3.5, 0));
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
        body->setFriction(4);
        
        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    addCloth(btVector3(0, btScalar(-0.2), 0));
    addCloth(btVector3(0, btScalar(-0.1), 0));
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}
void DeformableSelfCollision::addCloth(const btVector3& origin)
// create a piece of cloth
{
    const btScalar s = btScalar(0.6);
    const btScalar h = 0;
    
    btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -2*s),
                                                     btVector3(+s, h, -2*s),
                                                     btVector3(-s, h, +2*s),
                                                     btVector3(+s, h, +2*s),
                                                     15,30,
//                                                     4,4,
                                                     0, true, 0.0);

    
    psb->getCollisionShape()->setMargin(btScalar(0.02));
    psb->generateBendingConstraints(2);
    psb->setTotalMass(.5);
    psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
    psb->m_cfg.kCHR = 1; // collision hardness with rigid body
    psb->m_cfg.kDF = btScalar(0.1);
//    psb->rotate(btQuaternion(0, SIMD_PI / 2, 0));
    btTransform clothTransform;
    clothTransform.setIdentity();
    clothTransform.setOrigin(btVector3(0,btScalar(0.2),0)+origin);
    psb->transform(clothTransform);
    psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
    psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
    psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
    psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
    getDeformableDynamicsWorld()->addSoftBody(psb);
    psb->setSelfCollision(true);
    
    btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(2,btScalar(0.2), true);
    psb->setSpringStiffness(4);
    getDeformableDynamicsWorld()->addForce(psb, mass_spring);
    m_forces.push_back(mass_spring);
    btVector3 gravity = btVector3(0, btScalar(-9.8), 0);
    btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
    getDeformableDynamicsWorld()->addForce(psb, gravity_force);
    getDeformableDynamicsWorld()->setUseProjection(true);
    m_forces.push_back(gravity_force);
}

void DeformableSelfCollision::exitPhysics()
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



class CommonExampleInterface* DeformableSelfCollisionCreateFunc(struct CommonExampleOptions& options)
{
    return new DeformableSelfCollision(options.m_guiHelper);
}


