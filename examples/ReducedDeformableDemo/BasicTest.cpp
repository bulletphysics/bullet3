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

#include "BasicTest.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedSoftBody.h"
#include "BulletSoftBody/BulletReducedSoftBody/btReducedSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The BasicTest shows the contact between volumetric deformable objects and rigid objects.
static btScalar E = 50;
static btScalar nu = 0.3;
// static btScalar damping_alpha = 0.1;
// static btScalar damping_beta = 0.01;
static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0;
static btScalar COLLIDING_VELOCITY = 0;

class BasicTest : public CommonDeformableBodyBase
{
    typedef btAlignedObjectArray<btSoftBody::Node> tNodeArray;

    // btDeformableLinearElasticityForce* m_linearElasticity;
    // btDeformableMassSpringForce* m_massSpring;

    static const unsigned int m_startMode = 6;  // actual mode# should +1
    static const unsigned int m_nReduced = 2;
    static const unsigned int selected_mode = 0;
    unsigned int m_nFull;
    btScalar sim_time;
    bool first_step;

    // compute reduced degree of freedoms
    void mapToReducedDofs(btReducedSoftBody* rsb)
    {
      btAssert(rsb->m_reducedDofs.size() == m_nReduced);
      for (int j = 0; j < m_nReduced; ++j) 
      {
        rsb->m_reducedDofs[j] = 0;
        for (int i = 0; i < m_nFull; ++i)
          for (int k = 0; k < 3; ++k)
            rsb->m_reducedDofs[j] += rsb->m_modes[j][3 * i + k] * (rsb->m_nodes[i].m_x[k] - rsb->m_x0[3 * i + k]);
      }
    }
    
    // compute full degree of freedoms
    void mapToFullDofs(btReducedSoftBody* rsb)
    {
      btAssert(rsb->m_nodes.size() == m_nFull);
      btAlignedObjectArray<btVector3> delta_x;
      delta_x.resize(m_nFull);
      for (int i = 0; i < m_nFull; ++i)
      {
        for (int k = 0; k < 3; ++k)
        {
          // compute displacement
          delta_x[i][k] = 0;
          for (int j = 0; j < m_nReduced; ++j)   
            delta_x[i][k] += rsb->m_modes[j][3 * i + k] * rsb->m_reducedDofs[j];
          // get new coordinates
          rsb->m_nodes[i].m_x[k] = rsb->m_x0[3 * i + k] + delta_x[i][k];
        }
      }
      
    }

    // get deformed shape
    void getDeformedShape(btReducedSoftBody* rsb, const int mode_n, const btScalar scale = 1)
    {
      for (int i = 0; i < rsb->m_nodes.size(); ++i)
        for (int k = 0; k < 3; ++k)
          rsb->m_nodes[i].m_x[k] += rsb->m_modes[mode_n][3 * i + k] * scale;
    }

public:
    BasicTest(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
        m_nFull = 0;
        sim_time = 0;
        first_step = true;
    }
    virtual ~BasicTest()
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
        float dist = 20;
        float pitch = 0;
        float yaw = 90;
        float targetPos[3] = {0, 3, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void Ctor_RbUpStack()
    {
        float mass = 0.5;
        btCollisionShape* shape = new btBoxShape(btVector3(2, 2, 2));
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,-2,0));
        btRigidBody* rb = createRigidBody(mass, startTransform, shape);
        rb->setLinearVelocity(btVector3(0,+COLLIDING_VELOCITY, 0));
    }
    
    void stepSimulation(float deltaTime)
    {
      btReducedSoftBody* rsb = static_cast<btDeformableMultiBodyDynamicsWorld*>(m_dynamicsWorld)->getSoftBodyArray()[0];
      
      // TODO: remove this. very hacky way of adding initial deformation
      if (first_step && !rsb->m_bUpdateRtCst) 
      {
        // getDeformedShape(rsb, 0, 0.5);
        first_step = false;
        mapToReducedDofs(rsb);
        // std::cout << rsb->m_reducedDofs[0] << "\n";
      }

      // compute reduced dofs
      rsb->m_reducedDofs.resize(m_nReduced);
      rsb->m_reducedVelocity.resize(m_nReduced);
      sim_time += deltaTime;
      // std::cout << rsb->m_eigenvalues[0] << "\t" << sim_time << "\t" << deltaTime  << "\t" << sin(rsb->m_eigenvalues[0] * sim_time) << "\n";
      
      float internalTimeStep = 1. / 60.f;
      m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
      // float internalTimeStep = 1;
      // m_dynamicsWorld->stepSimulation(1, 1, internalTimeStep);

      // map reduced dof back to full
      mapToFullDofs(rsb);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* rsb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};

void BasicTest::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    btVector3 gravity = btVector3(0, -10, 0);
    m_dynamicsWorld->setGravity(gravity);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric soft body
    {        
        std::string filename("../../../examples/SoftDemo/mesh.vtk");
        btReducedSoftBody* rsb = btReducedSoftBodyHelpers::CreateFromVtkFile(getDeformableDynamicsWorld()->getWorldInfo(), filename.c_str());
        m_nFull = rsb->m_nodes.size();
        rsb->m_reducedModel = true;

        // read in eigenmodes, stiffness and mass matrices
        std::string eigenvalues_file("../../../examples/SoftDemo/eigenvalues.bin");
        btReducedSoftBodyHelpers::readBinary(rsb->m_eigenvalues, m_startMode, m_nReduced, 3 * m_nFull, eigenvalues_file.c_str());

        std::string Kr_file("../../../examples/SoftDemo/K_r_diag_mat.bin");
        btReducedSoftBodyHelpers::readBinary(rsb->m_Kr, m_startMode, m_nReduced, 3 * m_nFull, Kr_file.c_str());

        std::string Mr_file("../../../examples/SoftDemo/M_r_diag_mat.bin");
        btReducedSoftBodyHelpers::readBinary(rsb->m_Mr, m_startMode, m_nReduced, 3 * m_nFull, Mr_file.c_str());

        std::string modes_file("../../../examples/SoftDemo/modes.bin");
        btReducedSoftBodyHelpers::readBinaryModes(rsb->m_modes, m_startMode, m_nReduced, 3 * m_nFull, modes_file.c_str());	// default to 3D

        // get rest position
        rsb->m_x0.resize(3 * rsb->m_nodes.size());
        for (int i = 0; i < rsb->m_nodes.size(); ++i)
          for (int k = 0; k < 3; ++k)
            rsb->m_x0[3 * i + k] = rsb->m_nodes[i].m_x[k];

        getDeformableDynamicsWorld()->addSoftBody(rsb);
        rsb->scale(btVector3(2, 2, 2));
        rsb->translate(btVector3(0, 7, 0));
        rsb->getCollisionShape()->setMargin(0.1);
        rsb->setTotalMass(0.5);
        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);

        std::string M_file("../../../examples/SoftDemo/M_diag_mat.bin");
        btAlignedObjectArray<btScalar> mass_array;
        btReducedSoftBodyHelpers::readBinary(mass_array, 0, 3 * m_nFull, 3 * m_nFull, M_file.c_str());
        // assign mass to nodes
        for (int i = 0; i < rsb->m_nodes.size(); ++i)
          rsb->m_nodes[i].m_im = mass_array[3 * i];   // here we use m_im as the actual mass not the mass inverse
        
        rsb->setVelocity(btVector3(0, -COLLIDING_VELOCITY, 0));
        
        btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(rsb, gravity_force);
        m_forces.push_back(gravity_force);
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
    // Ctor_RbUpStack();        // TODO: no rigid body for now
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

void BasicTest::exitPhysics()
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



class CommonExampleInterface* ReducedBasicTestCreateFunc(struct CommonExampleOptions& options)
{
    return new BasicTest(options.m_guiHelper);
}


