/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
	Draft high-level generic physics C-API. For low-level access, use the physics SDK native API's.
	Work in progress, functionality will be added on demand.

	If possible, use the richer Bullet C++ API, by including <src/btBulletDynamicsCommon.h>
*/

#include "Bullet-C-Api.h"
#include "btBulletDynamicsCommon.h"

/*
	Create and Delete a Physics SDK	
*/

struct	btPhysicsSdk
{

//	btDispatcher*				m_dispatcher;
//	btOverlappingPairCache*		m_pairCache;
//	btConstraintSolver*			m_constraintSolver

	btVector3	m_worldAabbMin;
	btVector3	m_worldAabbMax;


	//todo: version, hardware/optimization settings etc?
	btPhysicsSdk()
		:m_worldAabbMin(-1000,-1000,-1000),
		m_worldAabbMax(1000,1000,1000)
	{

	}

	
};

plPhysicsSdkHandle	plNewBulletSdk()
{
	return (plPhysicsSdkHandle)new btPhysicsSdk;
}

void		plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk)
{
	btPhysicsSdk* phys = reinterpret_cast<btPhysicsSdk*>(physicsSdk);
	delete phys;	
}


/* Dynamics World */
plDynamicsWorldHandle plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdkHandle)
{
	btPhysicsSdk* physicsSdk = reinterpret_cast<btPhysicsSdk*>(physicsSdkHandle);
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btDispatcher*				dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface*		pairCache = new btAxisSweep3(physicsSdk->m_worldAabbMin,physicsSdk->m_worldAabbMax);
	btConstraintSolver*			constraintSolver = new btSequentialImpulseConstraintSolver();

	return (plDynamicsWorldHandle) new btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver);
}
void           plDeleteDynamicsWorld(plDynamicsWorldHandle world)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	delete dynamicsWorld;
}

void	plStepSimulation(plDynamicsWorldHandle world,	plReal	timeStep)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	assert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep);
}

void plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	assert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	assert(body);

	dynamicsWorld->addRigidBody(body);
}

void plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btDynamicsWorld* dynamicsWorld = reinterpret_cast< btDynamicsWorld* >(world);
	assert(dynamicsWorld);
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	assert(body);

	dynamicsWorld->removeRigidBody(body);
}

/* Rigid Body  */

plRigidBodyHandle plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape )
{
	btTransform trans;
	trans.setIdentity();
	btVector3 localInertia(0,0,0);
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	assert(shape);
	if (mass)
	{
		shape->calculateLocalInertia(mass,localInertia);
	}
	btRigidBody* body = new btRigidBody(mass, 0,shape,localInertia);
	body->setWorldTransform(trans);
	body->setUserPointer(user_data);
	return (plRigidBodyHandle) body;
}

void plDeleteRigidBody(plRigidBodyHandle cbody)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(cbody);
	assert(body);
	delete body;
}


/* Collision Shape definition */

plCollisionShapeHandle plNewSphereShape(plReal radius)
{
	return (plCollisionShapeHandle) new btSphereShape(radius);
	
}
	
plCollisionShapeHandle plNewBoxShape(plReal x, plReal y, plReal z)
{
	return (plCollisionShapeHandle) new btBoxShape(btVector3(x,y,z));
}

plCollisionShapeHandle plNewCapsuleShape(plReal radius, plReal height)
{
	//capsule is convex hull of 2 spheres, so use btMultiSphereShape
	btVector3 inertiaHalfExtents(radius,height,radius);
	const int numSpheres = 2;
	btVector3 positions[numSpheres] = {btVector3(0,height,0),btVector3(0,-height,0)};
	btScalar radi[numSpheres] = {radius,radius};
	return (plCollisionShapeHandle) new btMultiSphereShape(inertiaHalfExtents,positions,radi,numSpheres);
}
plCollisionShapeHandle plNewConeShape(plReal radius, plReal height)
{
	return (plCollisionShapeHandle) new btConeShape(radius,height);
}

plCollisionShapeHandle plNewCylinderShape(plReal radius, plReal height)
{
	return (plCollisionShapeHandle) new btCylinderShape(btVector3(radius,height,radius));
}

/* Convex Meshes */
plCollisionShapeHandle plNewConvexHullShape()
{
	return (plCollisionShapeHandle) new btConvexHullShape();
}


/* Concave static triangle meshes */
plMeshInterfaceHandle		   plNewMeshInterface()
{
	return 0;
}

plCollisionShapeHandle plNewCompoundShape()
{
	return (plCollisionShapeHandle) new btCompoundShape();
}

void	plAddChildShape(plCollisionShapeHandle compoundShapeHandle,plCollisionShapeHandle childShapeHandle, plVector3 childPos,plQuaternion childOrn)
{
	btCollisionShape* colShape = reinterpret_cast<btCollisionShape*>(compoundShapeHandle);
	btAssert(colShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);
	btCompoundShape* compoundShape = reinterpret_cast<btCompoundShape*>(colShape);
	btCollisionShape* childShape = reinterpret_cast<btCollisionShape*>(childShapeHandle);
	btTransform	localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(childPos[0],childPos[1],childPos[2]));
	localTrans.setRotation(btQuaternion(childOrn[0],childOrn[1],childOrn[2],childOrn[3]));
	compoundShape->addChildShape(localTrans,childShape);
}

void plSetEuler(plReal yaw,plReal pitch,plReal roll, plQuaternion orient)
{
	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	orient[0] = orn.getX();
	orient[1] = orn.getY();
	orient[2] = orn.getZ();
	orient[3] = orn.getW();

}


//	extern  void		plAddTriangle(plMeshInterfaceHandle meshHandle, plVector3 v0,plVector3 v1,plVector3 v2);
//	extern  plCollisionShapeHandle plNewStaticTriangleMeshShape(plMeshInterfaceHandle);


void		plAddVertex(plCollisionShapeHandle cshape, plReal x,plReal y,plReal z)
{
	btCollisionShape* colShape = reinterpret_cast<btCollisionShape*>( cshape);
	btAssert(colShape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btConvexHullShape* convexHullShape = reinterpret_cast<btConvexHullShape*>( cshape);
	convexHullShape->addPoint(btPoint3(x,y,z));

}

void plDeleteShape(plCollisionShapeHandle cshape)
{
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	assert(shape);
	delete shape;
}
void plSetScaling(plCollisionShapeHandle cshape, plVector3 cscaling)
{
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	assert(shape);
	btVector3 scaling(cscaling[0],cscaling[1],cscaling[2]);
	shape->setLocalScaling(scaling);	
}



void plSetPosition(plRigidBodyHandle object, const plVector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btVector3 pos(position[0],position[1],position[2]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
}

void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	btQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	btTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	plGetPosition(plRigidBodyHandle object,plVector3 position)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void plGetOrientation(plRigidBodyHandle object,plQuaternion orientation)
{
	btRigidBody* body = reinterpret_cast< btRigidBody* >(object);
	btAssert(body);
	const btQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}



//plRigidBodyHandle plRayCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);

//	extern  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);
