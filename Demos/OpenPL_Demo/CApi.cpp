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

plPhysicsSdkHandle	plNewBulletSdk()
{
	return 0;
}

void		plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk)
{
	
}

/* Dynamics World */
plDynamicsWorldHandle plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdk)
{
	return (plDynamicsWorldHandle) new btDiscreteDynamicsWorld;
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
	btVector3 localInertia;
	btCollisionShape* shape = reinterpret_cast<btCollisionShape*>( cshape);
	assert(shape);
	shape->calculateLocalInertia(mass,localInertia);
	btRigidBody* body = new btRigidBody(mass, trans,shape,localInertia);
	body->m_userObjectPointer = user_data;
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
}
void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation)
{
}


//plRigidBodyHandle plRayCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);

//	extern  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);
