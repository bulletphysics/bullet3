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
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.h"
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
static int num_modes = 20;

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

	btMultiBody* createFeatherstoneMultiBody_testMultiDof(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical = false, bool floating = false);
	void addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);

    // TODO: disable pick force, non-interactive for now.
    bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld) {
        return false;
    } 

    void resetCamera()
    {
        // float dist = 20;
        // float pitch = -10;
        float dist = 10;
        float pitch = -5;
        float yaw = 90;
        float targetPos[3] = {0, 0, 0};

        // float dist = 5;
		// float pitch = -35;
		// float yaw = 50;
		// float targetPos[3] = {-3, 2.8, -2.5};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    
    void Ctor_RbUpStack()
    {
        float mass = 10;

        btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
        // btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
        btVector3 localInertia(0, 0, 0);
		if (mass != 0.f)
			shape->calculateLocalInertia(mass, localInertia);

        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,-2,0));
        // startTransform.setRotation(btQuaternion(btVector3(1, 0, 1), SIMD_PI / 3.0));
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody(body, 1, 1+2);

        body->setActivationState(DISABLE_DEACTIVATION);
        body->setLinearVelocity(btVector3(0, COLLIDING_VELOCITY, 0));
        // body->setFriction(1);
    }

    void rigidBar()
    {
        float mass = 10;

        btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.25, 2));
        btVector3 localInertia(0, 0, 0);
		if (mass != 0.f)
			shape->calculateLocalInertia(mass, localInertia);

        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,10,0));
        // startTransform.setRotation(btQuaternion(btVector3(1, 0, 1), SIMD_PI / 3.0));
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody(body, 1, 1+2);

        body->setActivationState(DISABLE_DEACTIVATION);
        body->setLinearVelocity(btVector3(0, 0, 0));
        // body->setFriction(0);
    }

    void createGround()
    {
        // float mass = 55;
        float mass = 0;

        btCollisionShape* shape = new btBoxShape(btVector3(10, 2, 10));
        btVector3 localInertia(0, 0, 0);
		if (mass != 0.f)
			shape->calculateLocalInertia(mass, localInertia);

        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,-2,0));
        // startTransform.setRotation(btQuaternion(btVector3(1, 0, 1), SIMD_PI / 3.0));
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody(body, 1, 1+2);

        body->setActivationState(DISABLE_DEACTIVATION);
        body->setLinearVelocity(btVector3(0, 0, 0));
        // body->setFriction(1);
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
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btReducedDeformableBody* rsb = static_cast<btReducedDeformableBody*>(deformableWorld->getSoftBodyArray()[i]);
            {
                btSoftBodyHelpers::DrawFrame(rsb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags()); 
            }

            for (int p = 0; p < rsb->m_contactNodesList.size(); ++p)
            {
                int index = rsb->m_contactNodesList[p];
                deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[index].m_x, 0.2, btVector3(0, 1, 0));
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
    btReducedDeformableBodySolver* reducedSoftBodySolver = new btReducedDeformableBodySolver();
    btVector3 gravity = btVector3(0, 0, 0);
    reducedSoftBodySolver->setGravity(gravity);

    btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
    m_solver = sol;

    m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
    m_dynamicsWorld->setGravity(gravity);
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;
    m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    // create volumetric reduced deformable body
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
        // rsb->scale(btVector3(0.5, 0.5, 0.5));

        rsb->setStiffnessScale(100);
        rsb->setDamping(damping_alpha, damping_beta);

        rsb->setTotalMass(15);

        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 4, 0));
        // init_transform.setRotation(btQuaternion(0, SIMD_PI / 2.0, SIMD_PI / 2.0));
        rsb->transformTo(init_transform);

        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);
        
        rsb->setRigidVelocity(btVector3(0, -COLLIDING_VELOCITY, 0));
        // rsb->setRigidAngularVelocity(btVector3(1, 0, 0));
        b3Printf("total mass: %e", rsb->getTotalMass());
    }
    // rigidBar();

    // add a few rigid bodies
    Ctor_RbUpStack();
    
    // create ground
    // createGround();

    // create multibody
    // {
    //     bool damping = false;
    //     bool gyro = true;
    //     int numLinks = 0;
    //     bool spherical = true;  //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
    //     bool multibodyOnly = true;
    //     bool canSleep = false;
    //     bool selfCollide = true;
    //     bool multibodyConstraint = false;
    //     btVector3 linkHalfExtents(0.05, 0.37, 0.1);
    //     btVector3 baseHalfExtents(1, 1, 1);
    //     // btVector3 baseHalfExtents(2.5, 0.5, 2.5);
    //     // btVector3 baseHalfExtents(0.05, 0.37, 0.1);

    //     bool g_floatingBase = true;
    //     // btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, numLinks, btVector3(0, 4, 0), linkHalfExtents, baseHalfExtents, spherical, g_floatingBase);
    //     btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, numLinks, btVector3(0.f, 4.f, 0.f), baseHalfExtents, linkHalfExtents, spherical, g_floatingBase);
    //     //mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm

    //     mbC->setCanSleep(canSleep);
    //     mbC->setHasSelfCollision(selfCollide);
    //     mbC->setUseGyroTerm(gyro);
    //     //
    //     if (!damping)
    //     {
    //         mbC->setLinearDamping(0.f);
    //         mbC->setAngularDamping(0.f);
    //     }
    //     else
    //     {
    //         mbC->setLinearDamping(0.1f);
    //         mbC->setAngularDamping(0.9f);
    //     }
    //     //
    //     //////////////////////////////////////////////
    //     // if (numLinks > 0)
    //     // {
    //     //     btScalar q0 = 45.f * SIMD_PI / 180.f;
    //     //     if (!spherical)
    //     //     {
    //     //         mbC->setJointPosMultiDof(0, &q0);
    //     //     }
    //     //     else
    //     //     {
    //     //         btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
    //     //         quat0.normalize();
    //     //         mbC->setJointPosMultiDof(0, quat0);
    //     //     }
    //     // }
    //     ///
    //     addColliders_testMultiDof(mbC, m_dynamicsWorld, baseHalfExtents, linkHalfExtents);
    // }

    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(false);
    getDeformableDynamicsWorld()->getSolverInfo().m_friction = 1;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-3;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 100;
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

btMultiBody* ReducedCollide::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 10;

	if (baseMass)
	{
		btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;

	btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	// btQuaternion baseOriQuat(btVector3(0, 0, 1), -SIMD_PI / 6.0);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);

	//init the links
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI / 180.f;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();
	/////

	for (int i = 0; i < numLinks; ++i)
	{
		if (!spherical)
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}

	pMultiBody->finalizeMultiDof();
    pMultiBody->setBaseVel(vel);

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void ReducedCollide::addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		btScalar quat[4] = {-world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w()};

		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			pWorld->addCollisionObject(col, 2, 1 + 2);

			col->setFriction(1);
			pMultiBody->setBaseCollider(col);
		}
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
		local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		btVector3 posr = local_origin[i + 1];
		//	float pos[4]={posr.x(),posr.y(),posr.z(),1};

		btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(1);
		pWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}



class CommonExampleInterface* ReducedCollideCreateFunc(struct CommonExampleOptions& options)
{
    return new ReducedCollide(options.m_guiHelper);
}


