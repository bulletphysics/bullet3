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

#include "ModeVisualizer.h"
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


static int num_modes = 20;
static btScalar visualize_mode = 0;
static btScalar frequency_scale = 1;

class ModeVisualizer : public CommonDeformableBodyBase
{
    btScalar sim_time;

    // get deformed shape
    void getDeformedShape(btReducedDeformableBody* rsb, const int mode_n, const btScalar time_term = 1)
    {
      for (int i = 0; i < rsb->m_nodes.size(); ++i)
        for (int k = 0; k < 3; ++k)
          rsb->m_nodes[i].m_x[k] = rsb->m_x0[i][k] + rsb->m_modes[mode_n][3 * i + k] * time_term;
    }

    btVector3 computeMassWeightedColumnSum(btReducedDeformableBody* rsb, const int mode_n)
    {
        btVector3 sum(0, 0, 0);
        for (int i = 0; i < rsb->m_nodes.size(); ++i)
        {
            for (int k = 0; k < 3; ++k)
            {
                sum[k] += rsb->m_nodalMass[i] * rsb->m_modes[mode_n][3 * i + k];
            }
        }
        return sum;
    }

public:
    ModeVisualizer(struct GUIHelperInterface* helper)
        : CommonDeformableBodyBase(helper)
    {
        sim_time = 0;
    }
    virtual ~ModeVisualizer()
    {
    }
    void initPhysics();

    void exitPhysics();

    // disable pick force. non-interactive example.
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
    
    void stepSimulation(float deltaTime)
    {
      btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(static_cast<btDeformableMultiBodyDynamicsWorld*>(m_dynamicsWorld)->getSoftBodyArray()[0]);

      sim_time += deltaTime;
      int n_mode = floor(visualize_mode);
      btScalar scale = sin(sqrt(rsb->m_eigenvalues[n_mode]) * sim_time / frequency_scale);
      getDeformedShape(rsb, n_mode, scale);
    //   btVector3 mass_weighted_column_sum = computeMassWeightedColumnSum(rsb, visualize_mode);
    //   std::cout << "mode=" << int(visualize_mode) << "\t" << mass_weighted_column_sum[0] << "\t"
    //                                                       << mass_weighted_column_sum[1] << "\t"
    //                                                       << mass_weighted_column_sum[2] << "\n";
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

void ModeVisualizer::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();
    btReducedDeformableBodySolver* reducedSoftBodySolver = new btReducedDeformableBodySolver();

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric soft body
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
      rsb->getCollisionShape()->setMargin(0.1);

      btTransform init_transform;
      init_transform.setIdentity();
      init_transform.setOrigin(btVector3(0, 2, 0));
    //   init_transform.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 2.0));
      rsb->transform(init_transform);
      btSoftBodyHelpers::generateBoundaryFaces(rsb);
    }
    getDeformableDynamicsWorld()->setImplicit(false);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    
    {
      SliderParams slider("Visualize Mode", &visualize_mode);
      slider.m_minVal = 0;
      slider.m_maxVal = num_modes - 1;
      if (m_guiHelper->getParameterInterface())
          m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
      SliderParams slider("Frequency Reduction", &frequency_scale);
      slider.m_minVal = 1;
      slider.m_maxVal = 1e3;
      if (m_guiHelper->getParameterInterface())
          m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
}

void ModeVisualizer::exitPhysics()
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



class CommonExampleInterface* ReducedModeVisualizerCreateFunc(struct CommonExampleOptions& options)
{
    return new ModeVisualizer(options.m_guiHelper);
}


