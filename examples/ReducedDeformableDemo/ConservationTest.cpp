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

#include "ConservationTest.h"
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
#include <random>

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0;
static int start_mode = 6;
static int num_modes = 20;

class ConservationTest : public CommonDeformableBodyBase
{
    btScalar sim_time;
    bool first_step;

    // get deformed shape
    void getDeformedShape(btReducedDeformableBody* rsb, const int mode_n, const btScalar scale = 1)
    {
      // for (int i = 0; i < rsb->m_nodes.size(); ++i)
      //   for (int k = 0; k < 3; ++k)
      //     rsb->m_nodes[i].m_x[k] += rsb->m_modes[mode_n][3 * i + k] * scale;
      
      // rsb->m_reducedDofs[mode_n] = scale;
      // rsb->m_reducedDofsBuffer[mode_n] = scale;

      srand(1);
      for (int r = 0; r < rsb->m_nReduced; r++)
      {
        rsb->m_reducedDofs[r] = (btScalar(rand()) / btScalar(RAND_MAX) - 0.5);
        rsb->m_reducedDofsBuffer[r] = rsb->m_reducedDofs[r];
      }

      rsb->mapToFullPosition(rsb->getRigidTransform());
      // std::cout << "-----------\n";
      // std::cout << rsb->m_nodes[0].m_x[0] << '\t' << rsb->m_nodes[0].m_x[1] << '\t' << rsb->m_nodes[0].m_x[2] << '\n';
      // std::cout << "-----------\n";
    }

public:
    ConservationTest(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
        sim_time = 0;
        first_step = true;
    }
    virtual ~ConservationTest()
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

    void checkMomentum(btReducedDeformableBody* rsb)
    {
      btVector3 x_com(0, 0, 0);
      btVector3 total_linear(0, 0, 0);
      btVector3 total_angular(0, 0, 0);
      {
        std::ofstream myfile("center_of_mass.txt", std::ios_base::app);
        for (int i = 0; i < rsb->m_nFull; ++i)
        {
          x_com += rsb->m_nodalMass[i] * rsb->m_nodes[i].m_x;
        }
        x_com /= rsb->getTotalMass();
        myfile << sim_time << "\t" << x_com[0] << "\t" << x_com[1] << "\t" << x_com[2] << "\n";
        myfile.close();
      }
      {
        std::ofstream myfile("linear_momentum.txt", std::ios_base::app);
        for (int i = 0; i < rsb->m_nFull; ++i)
        {
          total_linear += rsb->m_nodalMass[i] * rsb->m_nodes[i].m_v;
        }
        myfile << sim_time << "\t" << total_linear[0] << "\t" << total_linear[1] << "\t" << total_linear[2] << "\n";
        myfile.close();
      }
      {
        std::ofstream myfile("angular_momentum.txt", std::ios_base::app);
        // btVector3 ri(0, 0, 0);
        // for (int i = 0; i < rsb->m_nFull; ++i)
        // { 
        //   ri = rsb->m_nodes[i].m_x - x_com;
        //   total_angular += rsb->m_nodalMass[i] * ri.cross(rsb->m_nodes[i].m_v);
        // }
        total_angular = rsb->computeTotalAngularMomentum();
        myfile << sim_time << "\t" << total_angular[0] << "\t" << total_angular[1] << "\t" << total_angular[2] << "\n";
        myfile.close();
      }
      {
        btVector3 angular_rigid(0, 0, 0);
        std::ofstream myfile("angular_momentum_rigid.txt", std::ios_base::app);
        btVector3 ri(0, 0, 0);
        for (int i = 0; i < rsb->m_nFull; ++i)
        { 
          ri = rsb->m_nodes[i].m_x - x_com;
          btMatrix3x3 ri_star = Cross(ri);
          angular_rigid += rsb->m_nodalMass[i] * (ri_star.transpose() * ri_star * rsb->getAngularVelocity());
        }
        myfile << sim_time << "\t" << angular_rigid[0] << "\t" << angular_rigid[1] << "\t" << angular_rigid[2] << "\n";
        myfile.close();
      }

      {
        std::ofstream myfile("reduced_velocity.txt", std::ios_base::app);
        myfile << sim_time << "\t" << rsb->m_reducedVelocity[0] << "\t" << rsb->m_reducedDofs[0] << "\n";
        myfile.close();
      }
    }
    
    void stepSimulation(float deltaTime)
    {
      // add initial deformation
      btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(static_cast<btDeformableMultiBodyDynamicsWorld*>(m_dynamicsWorld)->getSoftBodyArray()[0]);
      if (first_step /* && !rsb->m_bUpdateRtCst*/) 
      {
        getDeformedShape(rsb, 0, 1);
        first_step = false;
      }
      
      float internalTimeStep = 1. / 240.f;
      m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);

      sim_time += internalTimeStep;
      checkMomentum(rsb);
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

                btVector3 origin = rsb->getRigidTransform().getOrigin();
                btVector3 line_x = rsb->getRigidTransform().getBasis() * 2 * btVector3(1, 0, 0) + origin;
                btVector3 line_y = rsb->getRigidTransform().getBasis() * 2 * btVector3(0, 1, 0) + origin;
                btVector3 line_z = rsb->getRigidTransform().getBasis() * 2 * btVector3(0, 0, 1) + origin;

                deformableWorld->getDebugDrawer()->drawLine(origin, line_x, btVector3(1, 0, 0));
                deformableWorld->getDebugDrawer()->drawLine(origin, line_y, btVector3(0, 1, 0));
                deformableWorld->getDebugDrawer()->drawLine(origin, line_z, btVector3(0, 0, 1));

                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 0, 0), 0.1, btVector3(1, 1, 1));
                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 2, 0), 0.1, btVector3(1, 1, 1));
                deformableWorld->getDebugDrawer()->drawSphere(btVector3(0, 4, 0), 0.1, btVector3(1, 1, 1));
            }
        }
    }
};

void ConservationTest::initPhysics()
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
        rsb->transformTo(init_transform);

        rsb->setStiffnessScale(100);
        rsb->setDamping(damping_alpha, damping_beta);
        
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

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void ConservationTest::exitPhysics()
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



class CommonExampleInterface* ReducedConservationTestCreateFunc(struct CommonExampleOptions& options)
{
    return new ConservationTest(options.m_guiHelper);
}


