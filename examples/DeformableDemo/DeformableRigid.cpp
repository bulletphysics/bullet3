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
#include "DeformableRigid.h"
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

///The DeformableRigid shows contact between deformable objects and rigid objects.
class DeformableRigid : public CommonDeformableBodyBase
{
public:
	DeformableRigid(struct GUIHelperInterface* helper)
    :CommonDeformableBodyBase(helper)
	{
	}
	virtual ~DeformableRigid()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
        float dist = 20;
        float pitch = -45;
        float yaw = 100;
        float targetPos[3] = {0, -3, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
    
    void stepSimulation(float deltaTime)
    {
        //use a smaller internal timestep, there are stability issues
        float internalTimeStep = 1. / 240.f;
        m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
        
//
//        btCollisionShape* boxShape = new btBoxShape(btVector3(1, 1, 1));
//        boxShape->setMargin(1e-3);
//        if (0)
//        {
//        btVector3 p(0.99,1.01,0.99);
//        for (int i = 0; i < 40; ++i)
//        {
//            p[1] -= 0.001;
//            btScalar margin(.000001);
//            btTransform trans;
//            trans.setIdentity();
//            btGjkEpaSolver2::sResults results;
//            const btConvexShape* csh = static_cast<const btConvexShape*>(boxShape);
//            btScalar d = btGjkEpaSolver2::SignedDistance(p, margin, csh, trans, results);
//            printf("d = %f\n", d);
//            printf("----\n");
//        }
//        }
//
//        btVector3 p(.991,1.01,.99);
//        for (int i = 0; i < 40; ++i)
//        {
//            p[1] -= 0.001;
//            btScalar margin(.006);
//            btTransform trans;
//            trans.setIdentity();
//            btScalar dst;
//            btGjkEpaSolver2::sResults results;
//            btTransform point_transform;
//            point_transform.setIdentity();
//            point_transform.setOrigin(p);
//            btSphereShape sphere(margin);
//            btVector3 guess(0,0,0);
//            const btConvexShape* csh = static_cast<const btConvexShape*>(boxShape);
//            btGjkEpaSolver2::SignedDistance(&sphere, point_transform, csh, trans, guess, results);
//            dst = results.distance-csh->getMargin();
//            dst -= margin;
//            printf("d = %f\n", dst);
//             printf("----\n");
//        }
    }
    
    void Ctor_RbUpStack(int count)
    {
        float mass = .2;
        
        btCompoundShape* cylinderCompound = new btCompoundShape;
        btCollisionShape* cylinderShape = new btCylinderShapeX(btVector3(2, .5, .5));
        btCollisionShape* boxShape = new btBoxShape(btVector3(2, .5, .5));
        btTransform localTransform;
        localTransform.setIdentity();
        cylinderCompound->addChildShape(localTransform, boxShape);
        btQuaternion orn(SIMD_HALF_PI, 0, 0);
        localTransform.setRotation(orn);
        //    localTransform.setOrigin(btVector3(1,1,1));
        cylinderCompound->addChildShape(localTransform, cylinderShape);
        
        btCollisionShape* shape[] = {
            new btBoxShape(btVector3(1, 1, 1)),
            new btSphereShape(0.75),
            cylinderCompound
        };
//        static const int nshapes = sizeof(shape) / sizeof(shape[0]);
//        for (int i = 0; i < count; ++i)
//        {
//            btTransform startTransform;
//            startTransform.setIdentity();
//            startTransform.setOrigin(btVector3(0, 2+ 2 * i, 0));
//            startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
//            createRigidBody(mass, startTransform, shape[i % nshapes]);
//        }
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(1, 1.5, 1));
        createRigidBody(mass, startTransform, shape[0]);
        startTransform.setOrigin(btVector3(1, 1.5, -1));
        createRigidBody(mass, startTransform, shape[0]);
        startTransform.setOrigin(btVector3(-1, 1.5, 1));
        createRigidBody(mass, startTransform, shape[0]);
        startTransform.setOrigin(btVector3(-1, 1.5, -1));
        createRigidBody(mass, startTransform, shape[0]);
        startTransform.setOrigin(btVector3(0, 3.5, 0));
        createRigidBody(mass, startTransform, shape[0]);
    }
    
    virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
    {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
    {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual void renderScene()
    {
        CommonDeformableBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            //if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), fDrawFlags::Faces);// deformableWorld->getDrawFlags());
            }
        }
    }
};

void DeformableRigid::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
//    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
	//	m_dynamicsWorld->getSolverInfo().m_singleAxisDeformableThreshold = 0.f;//faster but lower quality
    btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
    getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.Reset();
    
//    getDeformableDynamicsWorld()->before_solver_callbacks.push_back(dynamics);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    {
        ///create a ground
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));

        m_collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -42, 0));
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
        body->setFriction(1);

        //add the ground to the dynamics world
        m_dynamicsWorld->addRigidBody(body);
    }
    
    // create a piece of cloth
    if(1)
    {
        bool onGround = false;
        const btScalar s = 4;
        const btScalar h = 0;
        
        btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
                                                         btVector3(+s, h, -s),
                                                         btVector3(-s, h, +s),
                                                         btVector3(+s, h, +s),
//                                                         3,3,
                                                         20,20,
                                                         1 + 2 + 4 + 8, true);
//                                                          0, true);

        if (onGround)
            psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, 0, -s),
                                                 btVector3(+s, 0, -s),
                                                 btVector3(-s, 0, +s),
                                                 btVector3(+s, 0, +s),
//                                                 20,20,
                                                 2,2,
                                                 0, true);
        
        psb->getCollisionShape()->setMargin(0.05);
        psb->generateBendingConstraints(2);
        psb->setTotalMass(1);
        psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
        psb->m_cfg.kCHR = 1; // collision hardness with rigid body
        psb->m_cfg.kDF = 2;
        psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
        psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
        getDeformableDynamicsWorld()->addSoftBody(psb);
        
        btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(15,0.5, true);
        getDeformableDynamicsWorld()->addForce(psb, mass_spring);
        m_forces.push_back(mass_spring);
        
        btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
        getDeformableDynamicsWorld()->addForce(psb, gravity_force);
        m_forces.push_back(gravity_force);
        // add a few rigid bodies
    }
    Ctor_RbUpStack(10);
    getDeformableDynamicsWorld()->setImplicit(false);
    getDeformableDynamicsWorld()->setLineSearch(false);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void DeformableRigid::exitPhysics()
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



class CommonExampleInterface* DeformableRigidCreateFunc(struct CommonExampleOptions& options)
{
	return new DeformableRigid(options.m_guiHelper);
}


