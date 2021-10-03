/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "MultiBodyBaseline.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "../SoftDemo/BunnyMesh.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../Utils/b3ResourcePath.h"
///The MultiBodyBaseline demo deformable bodies self-collision
static bool g_floatingBase = true;
static float friction = 1.;
class MultiBodyBaseline : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
    btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
public:
	MultiBodyBaseline(struct GUIHelperInterface* helper)
		: CommonMultiBodyBase(helper)
	{
	}
    
	virtual ~MultiBodyBaseline()
	{
	}
    
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 30;
        float pitch = -30;
        float yaw = 100;
        float targetPos[3] = {0, -10, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    virtual void stepSimulation(float deltaTime);
    
    btMultiBody* createFeatherstoneMultiBody_testMultiDof(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical = false, bool floating = false);
    
    void addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);

};

void MultiBodyBaseline::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btMultiBodyConstraintSolver* sol;
    sol = new btMultiBodyConstraintSolver;
	m_solver = sol;
    
    btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration);
    m_dynamicsWorld = world;
    //    m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

    {
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));

        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -40, 0));
        groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
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

    
    {
        bool damping = true;
        bool gyro = false;
        int numLinks = 4;
        bool spherical = false;  //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
        bool canSleep = false;
        bool selfCollide = true;
        btVector3 linkHalfExtents(1, 1, 1);
        btVector3 baseHalfExtents(1, 1, 1);
        
        btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, numLinks, btVector3(0.f, 10.f,0.f), linkHalfExtents, baseHalfExtents, spherical, g_floatingBase);
        
        mbC->setCanSleep(canSleep);
        mbC->setHasSelfCollision(selfCollide);
        mbC->setUseGyroTerm(gyro);
        //
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

        if (numLinks > 0)
        {
            btScalar q0 = 0.f * SIMD_PI / 180.f;
            if (!spherical)
            {
                mbC->setJointPosMultiDof(0, &q0);
            }
            else
            {
                btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
                quat0.normalize();
                mbC->setJointPosMultiDof(0, quat0);
            }
        }
        ///
        addColliders_testMultiDof(mbC, m_dynamicsWorld, baseHalfExtents, linkHalfExtents);
    }

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void MultiBodyBaseline::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

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

void MultiBodyBaseline::stepSimulation(float deltaTime)
{
//    getDeformableDynamicsWorld()->getMultiBodyDynamicsWorld()->stepSimulation(deltaTime);
    m_dynamicsWorld->stepSimulation(deltaTime, 5, 1./250.);
}


btMultiBody* MultiBodyBaseline::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating)
{
    //init the base
    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = .1f;
    
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
    btVector3 vel(0, 0, 0);
    //    pMultiBody->setBaseVel(vel);
    
    //init the links
    btVector3 hingeJointAxis(1, 0, 0);
    float linkMass = .1f;
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
    
    ///
    pWorld->addMultiBody(pMultiBody);
    ///
    return pMultiBody;
}

void MultiBodyBaseline::addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);
    
    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();
    
    {
        //    float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
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
        //    float pos[4]={posr.x(),posr.y(),posr.z(),1};
        
        btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};
        
        btCollisionShape* box = new btBoxShape(linkHalfExtents);
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
class CommonExampleInterface* MultiBodyBaselineCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiBodyBaseline(options.m_guiHelper);
}
