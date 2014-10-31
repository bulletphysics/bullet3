//test addJointTorque
#include "MultiBodyVehicle.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

btScalar gVehicleBaseMass = 100.f;
btScalar gVehicleWheelMass = 5.f;
float friction = 1.f;
btVector3 gVehicleBaseHalfExtents(1, 0.1, 2);
btVector3 gVehicleWheelHalfExtents(0.2, 0.2, 0.2);

btVector3 gVehicleWheelOffset(0, 0, 0.5);
btVector3 wheelAttachmentPosInWorld[4] = {
    btVector3(1, 0, 2.),
    btVector3(-1, 0, 2.),
    btVector3(1, 0, -2.),
    btVector3(-1, 0, -2.)
};


MultiBodyVehicleSetup::MultiBodyVehicleSetup()
{
}

MultiBodyVehicleSetup::~MultiBodyVehicleSetup()
{

}



class btMultiBody* MultiBodyVehicleSetup::createMultiBodyVehicle()
{
    class btMultiBodyDynamicsWorld* world = m_dynamicsWorld;
    int numWheels = 4;
    
    int totalLinks = numWheels;//number of body parts (links) (in)directly attached to the base, NOT including the base/root itself
    
    btCollisionShape* chassis = new btBoxShape(gVehicleBaseHalfExtents);//CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
    m_collisionShapes.push_back(chassis);
    btCollisionShape* wheel = new btCylinderShapeX(gVehicleWheelHalfExtents);//CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
    m_collisionShapes.push_back(wheel);
    
    
    btVector3 baseLocalInertia(0, 0, 0);
    chassis->calculateLocalInertia(gVehicleBaseMass, baseLocalInertia);
    
    bool multiDof = false;
    bool isFixedBase = false;
    bool canSleep = false;
    
    btMultiBody * bod = new btMultiBody(totalLinks, gVehicleBaseMass, baseLocalInertia, isFixedBase, canSleep);// , multiDof);
    bod->setHasSelfCollision(false);
    
    btQuaternion baseOrn(0, 0, 0, 1);
    btVector3 basePos(0, 0, 0);
    bod->setBasePos(basePos);
    
    bod->setWorldToBaseRot(baseOrn);
    btVector3 vel(0, 0, 0);
    bod->setBaseVel(vel);
    
    {
        
       
        
        
        int linkNum = 0;
    
        
        btVector3 wheelJointAxisWorld(1, 0, 0);
        btQuaternion parent_to_child = baseOrn.inverse();//??
        for (int j = 0; j < numWheels; j++, linkNum++)
        {
            int parent_link_num = -1;
            
            float initial_joint_angle = 0.0;
            
            btVector3 localWheelInertia(0, 0, 0);
            wheel->calculateLocalInertia(gVehicleWheelMass, localWheelInertia);
            bool disableParentCollision = true;
            btVector3 pivotToChildCOM(0, 0, 0.25);
            btVector3 pivotToWheelCOM(0, 0, 0);
            {
                bod->setupRevolute(linkNum, gVehicleWheelMass, localWheelInertia, parent_link_num, parent_to_child, wheelJointAxisWorld,
                                   wheelAttachmentPosInWorld[j], pivotToWheelCOM, disableParentCollision);
            }
            bod->setJointPos(linkNum, initial_joint_angle);
           
	    if (j<2)
{ 
            btMultiBodyJointMotor* con = new btMultiBodyJointMotor(bod, linkNum, 1., 50);
            world->addMultiBodyConstraint(con);
}	            
        }
       
        
    }
    
    //add a collider for the base
    {
        
        btAlignedObjectArray<btQuaternion> world_to_local;
        world_to_local.resize(totalLinks + 1);
        
        btAlignedObjectArray<btVector3> local_origin;
        local_origin.resize(totalLinks + 1);
        world_to_local[0] = bod->getWorldToBaseRot();
        local_origin[0] = bod->getBasePos();
        {
            
            float pos[4] = { local_origin[0].x(), local_origin[0].y(), local_origin[0].z(), 1 };
            float quat[4] = { -world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w() };
            
            
            if (1)
            {
                
                btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod, -1);
                col->setCollisionShape(chassis);
                btTransform tr;
                tr.setIdentity();
                tr.setOrigin(local_origin[0]);
                tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
                col->setWorldTransform(tr);
                world->addCollisionObject(col, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);// 2, 1 + 2);
                col->setFriction(friction);
                bod->setBaseCollider(col);
            }
        }
        
        //initialize local coordinate frames, relative to parent
        for (int i = 0; i<bod->getNumLinks(); i++)
        {
            const int parent = bod->getParent(i);
            world_to_local[i + 1] = bod->getParentToLocalRot(i) * world_to_local[parent + 1];
            local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), bod->getRVector(i)));
        }
        
        int linkIndex = 0;
        
     
        
        for (int j = 0; j<numWheels; j++, linkIndex++)
        {
            
            btVector3 posr = local_origin[linkIndex + 1];
            float pos[4] = { posr.x(), posr.y(), posr.z(), 1 };
            float quat[4] = { -world_to_local[linkIndex + 1].x(), -world_to_local[linkIndex + 1].y(), -world_to_local[linkIndex + 1].z(), world_to_local[linkIndex + 1].w() };
            
            btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bod, linkIndex);
            
            col->setCollisionShape(wheel);
            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(posr);
            tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
            col->setWorldTransform(tr);
            col->setFriction(friction);
            world->addCollisionObject(col, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);// 2, 1 + 2);
            bod->getLink(linkIndex).m_collider = col;
        }
    }
    
    
    world->addMultiBody(bod);
//    world->setGravity(btVector3(0,0,0));
    
    return bod;
}


void MultiBodyVehicleSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
    int upAxis = 1;

    btVector4 colors[4] =
    {
        btVector4(1,0,0,1),
        btVector4(0,1,0,1),
        btVector4(0,1,1,1),
        btVector4(1,1,0,1),
    };
    int curColor = 0;



    gfxBridge.setUpAxis(upAxis);

	this->createEmptyDynamicsWorld();
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        //btIDebugDraw::DBG_DrawConstraints
        +btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawContactPoints
        +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);

    
    createMultiBodyVehicle();
    
    if (1)
    {
        btVector3 groundHalfExtents(20,20,20);
        groundHalfExtents[upAxis]=1.f;
        btBoxShape* box = new btBoxShape(groundHalfExtents);
        box->initializePolyhedralFeatures();
        
        gfxBridge.createCollisionShapeGraphicsObject(box);
        btTransform start; start.setIdentity();
        btVector3 groundOrigin(0,0,0);
        groundOrigin[upAxis]=-1.5;
        start.setOrigin(groundOrigin);
        btRigidBody* body =  createRigidBody(0,start,box);
        btVector4 color = colors[curColor];
        curColor++;
        curColor&=3;
        gfxBridge.createRigidBodyGraphicsObject(body,color);
    }

}

void MultiBodyVehicleSetup::stepSimulation(float deltaTime)
{
      m_dynamicsWorld->stepSimulation(deltaTime);
}

