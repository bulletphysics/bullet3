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

#include "Pinch.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"

///The Pinch shows the frictional contact between kinematic rigid objects with deformable objects

struct TetraCube
{
#include "../SoftDemo/cube.inl"
};

class Pinch : public CommonDeformableBodyBase
{
public:
	Pinch(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
	}
	virtual ~Pinch()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 25;
        float pitch = -30;
        float yaw = 100;
        float targetPos[3] = {0, -0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
    }
    
    void createGrip()
    {
        int count = 2;
        float mass = 1e6;
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

void dynamics(btScalar time, btDeformableMultiBodyDynamicsWorld* world)
{
    btAlignedObjectArray<btRigidBody*>& rbs = world->getNonStaticRigidBodies();
    if (rbs.size()<2)
        return;
    btRigidBody* rb0 = rbs[0];
    btScalar pressTime = 0.9;
    btScalar liftTime = 2.5;
    btScalar shiftTime = 3.5;
    btScalar holdTime = 4.5*1000;
    btScalar dropTime = 5.3*1000;
    btTransform rbTransform;
    rbTransform.setIdentity();
    btVector3 translation;
    btVector3 velocity;
    
    btVector3 initialTranslationLeft = btVector3(0.5,3,4);
    btVector3 initialTranslationRight = btVector3(0.5,3,-4);
    btVector3 pinchVelocityLeft = btVector3(0,0,-2);
    btVector3 pinchVelocityRight = btVector3(0,0,2);
    btVector3 liftVelocity = btVector3(0,5,0);
    btVector3 shiftVelocity = btVector3(0,0,5);
    btVector3 holdVelocity = btVector3(0,0,0);
    btVector3 openVelocityLeft = btVector3(0,0,4);
    btVector3 openVelocityRight = btVector3(0,0,-4);
    
    if (time < pressTime)
    {
        velocity = pinchVelocityLeft;
        translation = initialTranslationLeft + pinchVelocityLeft * time;
    }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (time - pressTime);
        
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityLeft;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationLeft + pinchVelocityLeft * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityLeft * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb0->setCenterOfMassTransform(rbTransform);
    rb0->setAngularVelocity(btVector3(0,0,0));
    rb0->setLinearVelocity(velocity);
    
    btRigidBody* rb1 = rbs[1];
    if (time < pressTime)
    {
        velocity = pinchVelocityRight;
        translation = initialTranslationRight + pinchVelocityRight * time;
    }
    else if (time < liftTime)
    {
        velocity = liftVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (time - pressTime);
        
    }
    else if (time < shiftTime)
    {
        velocity = shiftVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (time - liftTime);
    }
    else if (time < holdTime)
    {
        velocity = btVector3(0,0,0);
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (time - shiftTime);
    }
    else if (time < dropTime)
    {
        velocity = openVelocityRight;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (time - holdTime);
    }
    else
    {
        velocity = holdVelocity;
        translation = initialTranslationRight + pinchVelocityRight * pressTime + liftVelocity * (liftTime-pressTime) + shiftVelocity * (shiftTime - liftTime) + holdVelocity * (holdTime - shiftTime)+ openVelocityRight * (dropTime - holdTime);
    }
    rbTransform.setOrigin(translation);
    rbTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
    rb1->setCenterOfMassTransform(rbTransform);
    rb1->setAngularVelocity(btVector3(0,0,0));
    rb1->setLinearVelocity(velocity);
    
    rb0->setFriction(20);
    rb1->setFriction(20);
}

void Pinch::initPhysics()
{
	m_guiHelper->setUpAxis(1);

    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
    btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
    getDeformableDynamicsWorld()->setSolverCallback(dynamics);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    //create a ground
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));

        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -25, 0));
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
        m_dynamicsWorld->addRigidBody(body);
    }
    
    // create a soft block
    {
        btScalar verts[24] = {0.f, 0.f, 0.f,
            1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            0.f, 0.f, 1.f,
            1.f, 1.f, 0.f,
            0.f, 1.f, 1.f,
            1.f, 0.f, 1.f,
            1.f, 1.f, 1.f
        };
        int triangles[60] = {0, 6, 3,
            0,1,6,
            7,5,3,
            7,3,6,
            4,7,6,
            4,6,1,
            7,2,5,
            7,4,2,
            0,3,2,
            2,3,5,
            0,2,4,
            0,4,1,
            0,6,5,
            0,6,4,
            3,4,2,
            3,4,7,
            2,7,3,
            2,7,1,
            4,5,0,
            4,5,6,
        };
//       btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(getDeformableDynamicsWorld()->getWorldInfo(), &verts[0], &triangles[0], 20);
////
        btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(getDeformableDynamicsWorld()->getWorldInfo(),
                                                                  TetraCube::getElements(),
                                                                  0,
                                                                  TetraCube::getNodes(),
                                                                  false, true, true);
        
        psb->scale(btVector3(2, 2, 2));
        psb->translate(btVector3(0, 4, 0));
        psb->getCollisionShape()->setMargin(0.01);
        psb->setTotalMass(1);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = .5;
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        getDeformableDynamicsWorld()->addSoftBody(psb);
        btSoftBodyHelpers::generateBoundaryFaces(psb);
        
        btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);
        
        btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(8,3, 0.02);
        neohookean->setPoissonRatio(0.3);
        neohookean->setYoungsModulus(25);
        neohookean->setDamping(0.01);
        psb->m_cfg.drag = 0.001;
        getDeformableDynamicsWorld()->addForce(psb, neohookean);
        m_forces.push_back(neohookean);
        // add a grippers
        createGrip();
    }
    getDeformableDynamicsWorld()->setImplicit(false);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Pinch::exitPhysics()
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



class CommonExampleInterface* PinchCreateFunc(struct CommonExampleOptions& options)
{
	return new Pinch(options.m_guiHelper);
}


