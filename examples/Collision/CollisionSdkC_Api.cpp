#include "CollisionSdkC_Api.h"
#include "Internal/CollisionSdkInterface.h"
#include "Internal/Bullet2CollisionSdk.h"

	/* Collision World */
	
plCollisionWorldHandle plCreateCollisionWorld(plCollisionSdkHandle collisionSdkHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	return sdk->createCollisionWorld();
}

void           plDeleteCollisionWorld(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	sdk->deleteCollisionWorld(worldHandle);
}

plCollisionSdkHandle plCreateBullet2CollisionSdk()
{
	return Bullet2CollisionSdk::createBullet2SdkHandle();
}

void plDeleteCollisionSdk(plCollisionSdkHandle collisionSdkHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	delete sdk;
}

plCollisionShapeHandle plCreateSphereShape(plCollisionSdkHandle collisionSdkHandle, plReal radius)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	return sdk->createSphereShape(radius);
	
}

void plDeleteShape(plCollisionSdkHandle collisionSdkHandle, plCollisionShapeHandle shapeHandle)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	sdk->deleteShape(shapeHandle);
}

plCollisionObjectHandle plCreateCollisionObject(  plCollisionSdkHandle collisionSdkHandle,  void* user_data,  plCollisionShapeHandle cshape ,plVector3 childPos,plQuaternion childOrn)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	return sdk->createCollisionObject(user_data, cshape, childPos, childOrn);
	
}

void plDeleteCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionObjectHandle body)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	sdk->deleteCollisionObject(body);
}

void plAddCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	sdk->addCollisionObject(world,object);
}
void plRemoveCollisionObject(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world, plCollisionObjectHandle object)
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	sdk->removeCollisionObject(world,object);
}

/* Collision Queries */
int plCollide(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle worldHandle, plCollisionObjectHandle colA, plCollisionObjectHandle colB,
                     lwContactPoint* pointsOut, int pointCapacity)
{
    CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
    return sdk->collide(worldHandle, colA,colB,pointsOut,pointCapacity);
}

void plWorldCollide(plCollisionSdkHandle collisionSdkHandle, plCollisionWorldHandle world,
                           plNearCallback filter, void* userData)
{
    CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
    sdk->collideWorld(world,filter,userData);
}
