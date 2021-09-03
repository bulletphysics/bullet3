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

#include "ReducedCollide.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedSoftBody.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedSoftBodyHelpers.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedSoftBodySolver.h"
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
static btScalar damping_beta = 0.0;
static btScalar COLLIDING_VELOCITY = 4;
static int start_mode = 6;
static int num_modes = 1;

class ReducedCollide : public CommonDeformableBodyBase
{
public:
    ReducedCollide(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~ReducedCollide()
    {
    }
    void initPhysics();

    void exitPhysics();

    // TODO: disable pick force, non-interactive for now.
    bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld) {
        return false;
    } 

    void resetCamera()
    {
        float dist = 15;
        float pitch = -10;
        float yaw = 90;
        float targetPos[3] = {0, 3, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void Ctor_RbUpStack()
    {
        float mass = 8;
        btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,-2,0));
        // startTransform.setRotation(btQuaternion(btVector3(1, 0, 1), SIMD_PI / 3.0));
        btRigidBody* rb = createRigidBody(mass, startTransform, shape);
        rb->setActivationState(DISABLE_DEACTIVATION);
        rb->setLinearVelocity(btVector3(0, +COLLIDING_VELOCITY, 0));
    }
    
    void stepSimulation(float deltaTime)
    {
      float internalTimeStep = 1. / 60.f;
      m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        // int flag = 0;
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btReducedSoftBody* rsb = static_cast<btReducedSoftBody*>(deformableWorld->getSoftBodyArray()[i]);
            {
                btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
                // btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), flag);
                btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags()); 

                for (int p = 0; p < rsb->m_fixedNodes.size(); ++p)
                {
                    deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[rsb->m_fixedNodes[p]].m_x, 0.2, btVector3(1, 0, 0));
                }
                for (int p = 0; p < rsb->m_nodeRigidContacts.size(); ++p)
                {
                    deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[rsb->m_contactNodesList[p]].m_x, 0.2, btVector3(0, 1, 0));
                }

                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 0, 0), 0.1, btVector3(1, 1, 1));
                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 5, 0), 0.1, btVector3(1, 1, 1));
                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 10, 0), 0.1, btVector3(1, 1, 1));
            }
        }
    }
};

void ReducedCollide::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    btReducedSoftBodySolver* reducedSoftBodySolver = new btReducedSoftBodySolver();
    btVector3 gravity = btVector3(0, 0, 0);
    reducedSoftBodySolver->setGravity(gravity);

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
    m_dynamicsWorld->setGravity(gravity);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric reduced deformable body
    {   
        std::string filepath("../../../examples/SoftDemo/beam/");
        std::string filename = filepath + "mesh.vtk";
        btReducedSoftBody* rsb = btReducedSoftBodyHelpers::createFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), filename.c_str());
        
        rsb->setReducedModes(start_mode, num_modes, rsb->m_nodes.size());
        btReducedSoftBodyHelpers::readReducedDeformableInfoFromFiles(rsb, filepath.c_str());

        getDeformableDynamicsWorld()->addSoftBody(rsb);
        rsb->getCollisionShape()->setMargin(0.1);
        
        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 2, 0));
        rsb->transform(init_transform);

        rsb->setStiffnessScale(10);
        rsb->setDamping(damping_alpha, damping_beta);

        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);
        
        rsb->setRigidVelocity(btVector3(0, -COLLIDING_VELOCITY, 0));
        // rsb->setRigidAngularVelocity(btVector3(1, 0, 0));
        
        // btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
        // getDeformableDynamicsWorld()->addForce(rsb, gravity_force);
        // m_forces.push_back(gravity_force);
    }

    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(true);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.3;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = true;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;
    // add a few rigid bodies
    Ctor_RbUpStack();
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    
    // {
    //     SliderParams slider("Young's Modulus", &E);
    //     slider.m_minVal = 0;
    //     slider.m_maxVal = 2000;
    //     if (m_guiHelper->getParameterInterface())
    //         m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    // }
    // {
    //     SliderParams slider("Poisson Ratio", &nu);
    //     slider.m_minVal = 0.05;
    //     slider.m_maxVal = 0.49;
    //     if (m_guiHelper->getParameterInterface())
    //         m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    // }
    // {
    //     SliderParams slider("Mass Damping", &damping_alpha);
    //     slider.m_minVal = 0;
    //     slider.m_maxVal = 1;
    //     if (m_guiHelper->getParameterInterface())
    //         m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    // }
    // {
    //     SliderParams slider("Stiffness Damping", &damping_beta);
    //     slider.m_minVal = 0;
    //     slider.m_maxVal = 0.1;
    //     if (m_guiHelper->getParameterInterface())
    //         m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    // }
}

void ReducedCollide::exitPhysics()
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



class CommonExampleInterface* ReducedCollideCreateFunc(struct CommonExampleOptions& options)
{
    return new ReducedCollide(options.m_guiHelper);
}

