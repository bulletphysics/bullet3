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

#include "Collide.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The Collide shows the contact between volumetric deformable objects and rigid objects.
static btScalar E = 50;
static btScalar nu = 0.3;
static btScalar damping_alpha = 0.1;
static btScalar damping_beta = 0.01;
static btScalar COLLIDING_VELOCITY = 15;

struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

class Collide : public CommonDeformableBodyBase
{
	btDeformableLinearElasticityForce* m_linearElasticity;

public:
	Collide(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
        m_linearElasticity = 0;
	}
	virtual ~Collide()
	{
	}
	void initPhysics();

	void exitPhysics();

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
		m_linearElasticity->setPoissonRatio(nu);
		m_linearElasticity->setYoungsModulus(E);
		m_linearElasticity->setDamping(damping_alpha, damping_beta);
        float internalTimeStep = 1. / 60.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
};

void Collide::initPhysics()
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
    btVector3 gravity = btVector3(0, 0, 0);
    m_dynamicsWorld->setGravity(gravity);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric soft body
    {
        btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(getDeformableDynamicsWorld()->getWorldInfo(),
                                                                  TetraCube::getElements(),
                                                                  0,
                                                                  TetraCube::getNodes(),
                                                                  false, true, true);
        getDeformableDynamicsWorld()->addSoftBody(psb);
        psb->scale(btVector3(2, 2, 2));
        psb->translate(btVector3(0, 7, 0));
        psb->getCollisionShape()->setMargin(0.1);
        psb->setTotalMass(0.5);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 0;
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
		psb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(psb);
        
        psb->setVelocity(btVector3(0, -COLLIDING_VELOCITY, 0));
        
        btDeformableLinearElasticityForce* linearElasticity = new btDeformableLinearElasticityForce(100,100,0.01);
		m_linearElasticity = linearElasticity;
        getDeformableDynamicsWorld()->addForce(psb, linearElasticity);
        m_forces.push_back(linearElasticity);
    }
    getDeformableDynamicsWorld()->setImplicit(true);
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
	
//	{
//		SliderParams slider("Young's Modulus", &E);
//		slider.m_minVal = 0;
//		slider.m_maxVal = 2000;
//		if (m_guiHelper->getParameterInterface())
//			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//	}
//	{
//		SliderParams slider("Poisson Ratio", &nu);
//		slider.m_minVal = 0.05;
//		slider.m_maxVal = 0.49;
//		if (m_guiHelper->getParameterInterface())
//			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//	}
//	{
//		SliderParams slider("Mass Damping", &damping_alpha);
//		slider.m_minVal = 0.001;
//		slider.m_maxVal = 0.01;
//		if (m_guiHelper->getParameterInterface())
//			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//	}
//    {
//        SliderParams slider("Stiffness Damping", &damping_beta);
//        slider.m_minVal = 0.001;
//        slider.m_maxVal = 0.01;
//        if (m_guiHelper->getParameterInterface())
//            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//    }
    {
        SliderParams slider("Young's Modulus", &E);
        slider.m_minVal = 0;
        slider.m_maxVal = 2000;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
        SliderParams slider("Poisson Ratio", &nu);
        slider.m_minVal = 0.05;
        slider.m_maxVal = 0.49;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
        SliderParams slider("Mass Damping", &damping_alpha);
        slider.m_minVal = 0;
        slider.m_maxVal = 1;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    {
        SliderParams slider("Stiffness Damping", &damping_beta);
        slider.m_minVal = 0;
        slider.m_maxVal = 0.1;
        if (m_guiHelper->getParameterInterface())
            m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
}

void Collide::exitPhysics()
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



class CommonExampleInterface* CollideCreateFunc(struct CommonExampleOptions& options)
{
	return new Collide(options.m_guiHelper);
}


