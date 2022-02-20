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

#include "ReducedMotorGrasp.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodyHelpers.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../CommonInterfaces/CommonFileIOInterface.h"
#include "Bullet3Common/b3FileUtils.h"

///The ReducedMotorGrasp shows grasping a volumetric deformable objects with multibody gripper with moter constraints.
static btScalar sGripperVerticalVelocity = 0.f;
static btScalar sGripperClosingTargetVelocity = 0.f;
static btScalar damping_alpha = 0.0;
static btScalar damping_beta = 0.0001;
static int num_modes = 20;
static float friction = 1.;
struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

struct TetraBunny
{
#include "../SoftDemo/bunny.inl"
};

static bool supportsJointMotor(btMultiBody* mb, int mbLinkIndex)
{
    bool canHaveMotor = (mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::eRevolute
                         || mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::ePrismatic);
    return canHaveMotor;
}

class ReducedMotorGrasp : public CommonDeformableBodyBase
{
    btAlignedObjectArray<btDeformableLagrangianForce*> m_forces;
public:
	ReducedMotorGrasp(struct GUIHelperInterface* helper)
    :CommonDeformableBodyBase(helper)
	{
	}
	virtual ~ReducedMotorGrasp()
	{
	}
	void initPhysics();

	void exitPhysics();

    void Ctor_RbUpStack()
    {
        float mass = 8;
        btCollisionShape* shape = new btBoxShape(btVector3(2, 0.25, 0.5));
        btTransform startTransform;
        startTransform.setIdentity();

        startTransform.setOrigin(btVector3(0,0.25,0));
        btRigidBody* rb1 = createRigidBody(mass, startTransform, shape);
        rb1->setLinearVelocity(btVector3(0, 0, 0));
        rb1->setFriction(0.7);
    }

	void resetCamera()
	{
        // float dist = 0.3;
        // float pitch = -45;
        // float yaw = 100;
        // float targetPos[3] = {0, -0.1, 0};
        float dist = 0.4;
        float pitch = -25;
        float yaw = 90;
        float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    btMultiBody* createFeatherstoneMultiBody(btMultiBodyDynamicsWorld* pWorld,const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool floating);
    
    void addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
    
    btMultiBody* createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating);
    
    void stepSimulation(float deltaTime)
    {
        double fingerTargetVelocities[2] = {sGripperVerticalVelocity, sGripperClosingTargetVelocity};
        int num_multiBody = getDeformableDynamicsWorld()->getNumMultibodies();
        for (int i = 0; i < num_multiBody; ++i)
        {
            btMultiBody* mb = getDeformableDynamicsWorld()->btMultiBodyDynamicsWorld::getMultiBody(i);
            mb->setBaseVel(btVector3(0,sGripperVerticalVelocity, 0));
            int dofIndex = 6;  //skip the 3 linear + 3 angular degree of freedom entries of the base
            for (int link = 0; link < mb->getNumLinks(); link++)
            {
                if (supportsJointMotor(mb, link))
                {
                    btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;
                    if (motor)
                    {
                        if (dofIndex == 6)
                        {
                            motor->setVelocityTarget(-fingerTargetVelocities[1], 1);
                            motor->setMaxAppliedImpulse(10);
                        }
                        if (dofIndex == 7)
                        {
                            motor->setVelocityTarget(fingerTargetVelocities[1], 1);
                            motor->setMaxAppliedImpulse(10);
                        }
                        motor->setMaxAppliedImpulse(25);
                    }
                }
                dofIndex += mb->getLink(link).m_dofCount;
            }
        }
        
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
        // float internalTimeStep = 1. / 60.f;
        // m_dynamicsWorld->stepSimulation(deltaTime, 1, internalTimeStep);
    }
    
    void createGrip()
    {
        int count = 2;
        float mass = 2;
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
    
    virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
    {
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
    {
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
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
				btSoftBodyHelpers::Draw(rsb, deformableWorld->getDebugDrawer(), fDrawFlags::Faces);// deformableWorld->getDrawFlags());
            }
            // for (int p = 0; p < rsb->m_contactNodesList.size(); ++p)
            // {
            //     int index = rsb->m_contactNodesList[p];
            //     deformableWorld->getDebugDrawer()->drawSphere(rsb->m_nodes[index].m_x, 0.2, btVector3(0, 1, 0));
            // }
        }
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


void ReducedMotorGrasp::initPhysics()
{
	m_guiHelper->setUpAxis(1);
    
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btReducedDeformableBodySolver* reducedSoftBodySolver = new btReducedDeformableBodySolver();
    // btVector3 gravity = btVector3(0, 0, 0);
    btVector3 gravity = btVector3(0, -9.81, 0);
    reducedSoftBodySolver->setGravity(gravity);

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(reducedSoftBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, reducedSoftBodySolver);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
    // getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.1;
    // getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0;
    // getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 150;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_maxPickingForce = 0.001;
    // build a gripper
    {
        bool damping = true;
        bool gyro = false;
        bool canSleep = false;
        bool selfCollide = true;
        int numLinks = 2;
        // btVector3 linkHalfExtents(0.02, 0.018, .003);
        // btVector3 baseHalfExtents(0.02, 0.002, .002);
        btVector3 linkHalfExtents(0.03, 0.04, 0.006);
        btVector3 baseHalfExtents(0.02, 0.015, 0.015);
        btVector3 basePosition(0, 0.3, 0);
        // btMultiBody* mbC = createFeatherstoneMultiBody(getDeformableDynamicsWorld(), btVector3(0.f, 0.05f,0.f), baseHalfExtents, linkHalfExtents, false);
        btMultiBody* mbC = createFeatherstoneMultiBody(getDeformableDynamicsWorld(), basePosition, baseHalfExtents, linkHalfExtents, false);
        
        mbC->setCanSleep(canSleep);
        mbC->setHasSelfCollision(selfCollide);
        mbC->setUseGyroTerm(gyro);

        for (int i = 0; i < numLinks; i++)
        {
            int mbLinkIndex = i;
            double maxMotorImpulse = 1;

            if (supportsJointMotor(mbC, mbLinkIndex))
            {
                int dof = 0;
                btScalar desiredVelocity = 0.f;
                btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mbC, mbLinkIndex, dof, desiredVelocity, maxMotorImpulse);
                motor->setPositionTarget(0, 0);
                motor->setVelocityTarget(0, 1);
                mbC->getLink(mbLinkIndex).m_userPtr = motor;
                getDeformableDynamicsWorld()->addMultiBodyConstraint(motor);
                motor->finalizeMultiDof();
            }
        }
        
        if (!damping)
        {
            mbC->setLinearDamping(0.0f);
            mbC->setAngularDamping(0.0f);
        }
        else
        {
            mbC->setLinearDamping(0.04f);
            mbC->setAngularDamping(0.04f);
        }
        btScalar q0 = 0.f * SIMD_PI / 180.f;
        if (numLinks > 0)
            mbC->setJointPosMultiDof(0, &q0);
        addColliders(mbC, getDeformableDynamicsWorld(), baseHalfExtents, linkHalfExtents);
    }
    
    //create a ground
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(10.), btScalar(5.), btScalar(10.)));
        groundShape->setMargin(0.001);
        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -5.1, 0));
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
        m_dynamicsWorld->addRigidBody(body,1,1+2);
    }

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
        rsb->getCollisionShape()->setMargin(0.001);

        rsb->setStiffnessScale(100);
        rsb->setDamping(damping_alpha, damping_beta);

        rsb->scale(btVector3(0.075, 0.075, 0.075));
        rsb->setTotalMass(1);

        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(0, 0.1, 0));
        // init_transform.setRotation(btQuaternion(SIMD_PI / 2.0, 0, SIMD_PI / 2.0));
        rsb->transform(init_transform);
        
        // rsb->setRigidVelocity(btVector3(0, 1, 0));

        rsb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        rsb->m_cfg.kCHR = 1; // collision hardness with rigid body
        rsb->m_cfg.kDF = 0;
        rsb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        rsb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
        rsb->m_sleepingThreshold = 0;
        btSoftBodyHelpers::generateBoundaryFaces(rsb);
    }

    // Ctor_RbUpStack();
    
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
    getDeformableDynamicsWorld()->setUseProjection(false);
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_erp = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_cfm = 0.2;
    getDeformableDynamicsWorld()->getSolverInfo().m_friction = 1;
    getDeformableDynamicsWorld()->getSolverInfo().m_deformable_maxErrorReduction = btScalar(200);
    getDeformableDynamicsWorld()->getSolverInfo().m_leastSquaresResidualThreshold = 1e-6;
    getDeformableDynamicsWorld()->getSolverInfo().m_splitImpulse = false;
    getDeformableDynamicsWorld()->getSolverInfo().m_numIterations = 200;
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    
    {
        SliderParams slider("Moving velocity", &sGripperVerticalVelocity);
        // slider.m_minVal = -.02;
        // slider.m_maxVal = .02;
        slider.m_minVal = -.2;
        slider.m_maxVal = .2;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    
    {
        SliderParams slider("Closing velocity", &sGripperClosingTargetVelocity);
        slider.m_minVal = -1;
        slider.m_maxVal = 1;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
    
}

void ReducedMotorGrasp::exitPhysics()
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

btMultiBody* ReducedMotorGrasp::createFeatherstoneMultiBody(btMultiBodyDynamicsWorld* pWorld, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool floating)
{
    //init the base
    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 55;
    float linkMass = 55;
    int numLinks = 2;
    
    if (baseMass)
    {
        btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
        pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
        delete pTempBox;
    }
    
    bool canSleep = false;
    btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
    
    btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
    pMultiBody->setBasePos(basePosition);
    pMultiBody->setWorldToBaseRot(baseOriQuat);
    
    //init the links
    btVector3 hingeJointAxis(1, 0, 0);
    
    btVector3 linkInertiaDiag(0.f, 0.f, 0.f);
    
    btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
    pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
    delete pTempBox;
    
    //y-axis assumed up
    btAlignedObjectArray<btVector3> parentComToCurrentCom;
    parentComToCurrentCom.push_back(btVector3(0, -linkHalfExtents[1] * 2.f, -baseHalfExtents[2] * 2.f));
    parentComToCurrentCom.push_back(btVector3(0, -linkHalfExtents[1] * 2.f, +baseHalfExtents[2] * 2.f));//par body's COM to cur body's COM offset

    
    btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1]*2.f, 0);                         //cur body's COM to cur body's PIV offset
    
    btAlignedObjectArray<btVector3> parentComToCurrentPivot;
    parentComToCurrentPivot.push_back(btVector3(parentComToCurrentCom[0] - currentPivotToCurrentCom));
    parentComToCurrentPivot.push_back(btVector3(parentComToCurrentCom[1] - currentPivotToCurrentCom));//par body's COM to cur body's PIV offset
    
    //////
    btScalar q0 = 0.f * SIMD_PI / 180.f;
    btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
    quat0.normalize();
    /////
    
    for (int i = 0; i < numLinks; ++i)
    {
        pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot[i], currentPivotToCurrentCom, true);
    }
    pMultiBody->finalizeMultiDof();
    ///
    pWorld->addMultiBody(pMultiBody);
    ///
    return pMultiBody;
}

void ReducedMotorGrasp::addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);
    
    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();
    
    {
        btScalar quat[4] = {-world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w()};
        
        if (1)
        {
            btCollisionShape* box = new btBoxShape(baseHalfExtents);
            box->setMargin(0.001);
            btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
            col->setCollisionShape(box);
            
            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(local_origin[0]);
            tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
            col->setWorldTransform(tr);
            
            pWorld->addCollisionObject(col, 2, 1 + 2);
            
            col->setFriction(friction);
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
        
        btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};
        
        btCollisionShape* box = new btBoxShape(linkHalfExtents);
        box->setMargin(0.001);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
        
        col->setCollisionShape(box);
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);
        col->setFriction(friction);
        pWorld->addCollisionObject(col, 2, 1 + 2);
        
        pMultiBody->getLink(i).m_collider = col;
    }
}

class CommonExampleInterface* ReducedMotorGraspCreateFunc(struct CommonExampleOptions& options)
{
	return new ReducedMotorGrasp(options.m_guiHelper);
}


