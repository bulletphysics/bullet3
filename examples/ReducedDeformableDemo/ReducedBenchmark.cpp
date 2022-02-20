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

#include "ReducedBenchmark.h"
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
static btScalar COLLIDING_VELOCITY = 0;
static int num_modes = 20;
static bool run_reduced = true;

class ReducedBenchmark : public CommonDeformableBodyBase
{
    btVector3 m_gravity;
public:
    ReducedBenchmark(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
    }
    virtual ~ReducedBenchmark()
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
        // float dist = 6;
        // float pitch = -10;
        // float yaw = 90;
        // float targetPos[3] = {0, 2, 0};
        float dist = 10;
        float pitch = -30;
        float yaw = 125;
        float targetPos[3] = {0, 2, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void Ctor_RbUpStack(const btVector3& origin)
    {
        float mass = 10;
        btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
        // btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
        btTransform startTransform;
        startTransform.setIdentity();
        // startTransform.setOrigin(btVector3(0, 12, 0));
        // btRigidBody* rb0 = createRigidBody(mass, startTransform, shape);
        // rb0->setLinearVelocity(btVector3(0, 0, 0));

        startTransform.setOrigin(origin);
        // startTransform.setRotation(btQuaternion(btVector3(1, 0, 1), SIMD_PI / 4.0));
        btRigidBody* rb1 = createRigidBody(mass, startTransform, shape);
        rb1->setActivationState(DISABLE_DEACTIVATION);
        // rb1->setLinearVelocity(btVector3(0, 0, 4));
    }

    void createDeform(const btVector3& origin, const btQuaternion& rotation)
    {

        if (run_reduced)
        {   
            std::string file_path("../../../data/reduced_torus/");
            std::string vtk_file("torus_mesh.vtk");
            btReducedDeformableBody* rsb = btReducedDeformableBodyHelpers::createReducedDeformableObject(
                                                getDeformableDynamicsWorld()->getWorldInfo(),
                                                file_path,
                                                vtk_file,
                                                num_modes,
                                                false);

            getDeformableDynamicsWorld()->addSoftBody(rsb);
            rsb->getCollisionShape()->setMargin(0.01);
            // rsb->scale(btVector3(1, 1, 0.5));

            rsb->setTotalMass(10);

            btTransform init_transform;
            init_transform.setIdentity();
            init_transform.setOrigin(origin);
            init_transform.setRotation(rotation);
            rsb->transformTo(init_transform);

            rsb->setStiffnessScale(5);
            rsb->setDamping(damping_alpha, damping_beta);
            // rsb->scale(btVector3(0.5, 0.5, 0.5));

            rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
            rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
            rsb->m_cfg.kDF = 0;
            rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
            rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
            rsb->m_sleepingThreshold = 0;
            btSoftBodyHelpers::generateBoundaryFaces(rsb);

            std::cout << "Running reduced deformable\n";
        }
        else    // create full deformable cube
        {
            std::string filepath("../../../data/reduced_torus/");
            std::string filename = filepath + "torus_mesh.vtk";
            btSoftBody* psb = btSoftBodyHelpers::CreateFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), filename.c_str());
            
            btTransform init_transform;
            init_transform.setIdentity();
            init_transform.setOrigin(origin);
            init_transform.setRotation(rotation);
            psb->transform(init_transform);
            psb->getCollisionShape()->setMargin(0.015);
            psb->setTotalMass(10);
            psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
            psb->m_cfg.kCHR = 1; // collision hardness with rigid body
            psb->m_cfg.kDF = .5;
            psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
            psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
            getDeformableDynamicsWorld()->addSoftBody(psb);
            btSoftBodyHelpers::generateBoundaryFaces(psb);
            
            btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(m_gravity);
            getDeformableDynamicsWorld()->addForce(psb, gravity_force);
            m_forces.push_back(gravity_force);
            
            btScalar E = 10000;
            btScalar nu = 0.3;
            btScalar lambda = E * nu / ((1 + nu) * (1 - 2 * nu));
            btScalar mu = E / (2 * (1 + nu));
            btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(lambda, mu, 0.01);
            // neohookean->setPoissonRatio(0.3);
            // neohookean->setYoungsModulus(25);
            neohookean->setDamping(0.01);
            psb->m_cfg.drag = 0.001;
            getDeformableDynamicsWorld()->addForce(psb, neohookean);
            m_forces.push_back(neohookean);

            std::cout << "Running full deformable\n";
        }

        // btReducedDeformableBody* rsb = btReducedDeformableBodyHelpers::createReducedTorus(getDeformableDynamicsWorld()->getWorldInfo(), num_modes);

        // getDeformableDynamicsWorld()->addSoftBody(rsb);
        // rsb->getCollisionShape()->setMargin(0.01);
        // // rsb->scale(btVector3(1, 1, 0.5));

        // rsb->setTotalMass(10);

        // btTransform init_transform;
        // init_transform.setIdentity();
        // init_transform.setOrigin(origin);
        // init_transform.setRotation(rotation);
        // rsb->transformTo(init_transform);

        // rsb->setStiffnessScale(5);
        // rsb->setDamping(damping_alpha, damping_beta);
        // // rsb->scale(btVector3(0.5, 0.5, 0.5));

        // rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        // rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        // rsb->m_cfg.kDF = 0;
        // rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        // rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        // rsb->m_sleepingThreshold = 0;
        // btSoftBodyHelpers::generateBoundaryFaces(rsb);
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
            btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(deformableWorld->getSoftBodyArray()[i]);
            {
                btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags()); 
            }
        }
    }
};

void ReducedBenchmark::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();

    if (run_reduced)
    {
        btReducedDeformableBodySolver* solver = new btReducedDeformableBodySolver();

        btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
        sol->setDeformableSolver(solver);
        m_solver = sol;
        m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, solver);
    }
    else
    {
        btDeformableBodySolver* solver = new btDeformableBodySolver();

        btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
        sol->setDeformableSolver(solver);
        m_solver = sol;
        m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, solver);
    }

    // m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, solver);
    btVector3 gravity = btVector3(0, -10, 0);
    m_gravity = gravity;
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    // 3x3 torus
    createDeform(btVector3(4, 4, -4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(4, 4, 0), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(4, 4, 4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(0, 4, -4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(0, 4, 0), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(0, 4, 4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(-4, 4, -4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(-4, 4, 0), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    createDeform(btVector3(-4, 4, 4), btQuaternion(SIMD_PI / 2.0, SIMD_PI / 2.0, 0));
    
    // create a static rigid box as the ground
    {
        // btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50), btScalar(50), btScalar(50)));
        btBoxShape* groundShape = createBoxShape(btVector3(btScalar(20), btScalar(2), btScalar(20)));
        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        // groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI / 6.0));
        // groundTransform.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_PI / 6.0));
        groundTransform.setOrigin(btVector3(0, 0, 0));
        // groundTransform.setOrigin(btVector3(0, 0, 6));
        // groundTransform.setOrigin(btVector3(0, -50, 0));
        {
            btScalar mass(0.);
            createRigidBody(mass, groundTransform, groundShape, btVector4(0,0,0,0));
        }
    }

    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(false);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_friction = 0.5;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    m_dynamicsWorld->setGravity(gravity);
}

void ReducedBenchmark::exitPhysics()
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



class CommonExampleInterface* ReducedBenchmarkCreateFunc(struct CommonExampleOptions& options)
{
    return new ReducedBenchmark(options.m_guiHelper);
}


