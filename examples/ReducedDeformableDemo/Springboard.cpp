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

#include "Springboard.h"
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

static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0001;
static int num_modes = 20;

class Springboard : public CommonDeformableBodyBase
{
    btScalar sim_time;
    bool first_step;

public:
    Springboard(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
        sim_time = 0;
        first_step = true;
    }
    virtual ~Springboard()
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
        float dist = 10;
        float pitch = 0;
        float yaw = 90;
        float targetPos[3] = {0, 3, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void Ctor_RbUpStack()
    {
        float mass = 2;
        btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,8,1));
        btRigidBody* rb1 = createRigidBody(mass, startTransform, shape);
        rb1->setActivationState(DISABLE_DEACTIVATION);
    }
    
    void stepSimulation(float deltaTime)
    {      
      float internalTimeStep = 1. / 60.f;
      m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);

      {
        sim_time += internalTimeStep;
        btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(getDeformableDynamicsWorld()->getSoftBodyArray()[0]);
        
        // std::ofstream myfile("fixed_node.txt", std::ios_base::app);
        // myfile << sim_time << "\t" << rsb->m_nodes[0].m_x[0] - rsb->m_x0[0][0] << "\t" 
        //                            << rsb->m_nodes[0].m_x[1] - rsb->m_x0[0][1] << "\t" 
        //                            << rsb->m_nodes[0].m_x[2] - rsb->m_x0[0][2] << "\n";
        // myfile.close();
      }
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

                for (int p = 0; p < rsb->m_fixedNodes.size(); ++p)
                {
                    deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[rsb->m_fixedNodes[p]].m_x, 0.2, btVector3(1, 0, 0));
                    // std::cout << rsb->m_nodes[rsb->m_fixedNodes[p]].m_x[0] << "\t" << rsb->m_nodes[rsb->m_fixedNodes[p]].m_x[1] << "\t" << rsb->m_nodes[rsb->m_fixedNodes[p]].m_x[2] << "\n";
                }
                // deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 0, 0), 0.1, btVector3(1, 1, 1));
                // deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 2, 0), 0.1, btVector3(1, 1, 1));
                // deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 4, 0), 0.1, btVector3(1, 1, 1));
            }
        }
    }
};

void Springboard::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    btReducedDeformableBodySolver* reducedSoftBodySolver = new btReducedDeformableBodySolver();
    btVector3 gravity = btVector3(0, -10, 0);
    reducedSoftBodySolver->setGravity(gravity);

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
    m_dynamicsWorld->setGravity(gravity);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric reduced deformable body
    {   
        std::string file_path("../../../data/reduced_beam/");
        std::string vtk_file("beam_mesh_origin.vtk");
        btReducedDeformableBody* rsb = btReducedDeformableBodyHelpers::createReducedDeformableObject(
                                            getDeformableDynamicsWorld()->getWorldInfo(),
                                            file_path,
                                            vtk_file,
                                            num_modes,
                                            false);

        getDeformableDynamicsWorld()->addSoftBody(rsb);
        rsb->getCollisionShape()->setMargin(0.1);
        
        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 4, 0));
        // init_transform.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 2.0));
        rsb->transform(init_transform);

        rsb->setStiffnessScale(200);
        rsb->setDamping(damping_alpha, damping_beta);
        
        // set fixed nodes
        rsb->setFixedNodes(0);
        rsb->setFixedNodes(1);
        rsb->setFixedNodes(2);
        rsb->setFixedNodes(3);
        
        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);
        
        // rsb->setVelocity(btVector3(0, -COLLIDING_VELOCITY, 0));
        // rsb->setRigidVelocity(btVector3(0, 1, 0));
        // rsb->setRigidAngularVelocity(btVector3(1, 0, 0));
    }
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(false);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.3;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;
    // add a few rigid bodies
    Ctor_RbUpStack();

    // create a static rigid box as the ground
    {
        // btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50), btScalar(50), btScalar(50)));
        btBoxShape* groundShape = createBoxShape(btVector3(btScalar(10), btScalar(2), btScalar(10)));
        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, 0, 0));
        {
            btScalar mass(0.);
            createRigidBody(mass, groundTransform, groundShape, btVector4(0,0,0,0));
        }
    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Springboard::exitPhysics()
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



class CommonExampleInterface* ReducedSpringboardCreateFunc(struct CommonExampleOptions& options)
{
    return new Springboard(options.m_guiHelper);
}


