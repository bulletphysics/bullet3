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

plCollisionObjectHandle plCreateCollisionObject(  plCollisionSdkHandle collisionSdkHandle,  void* user_data,  plCollisionShapeHandle cshape )
{
	CollisionSdkInterface* sdk = (CollisionSdkInterface*) collisionSdkHandle;
	return sdk->createCollisionObject(user_data, cshape);
	
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


//plCollisionSdkHandle plCreateRealTimeBullet3CollisionSdk();
//plCollisionSdkHandle plCreateCustomCollisionSdk();


#if 0
	extern  void           plDeleteCollisionWorld(plCollisionWorldHandle world);
	
	
	extern  void plAddCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	extern  void plRemoveCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	
	
	/* Collision Object  */
	
	extern  plCollisionObjectHandle plCreateCollisionObject(  plCollisionSdkHandle sdk,  void* user_data,  plCollisionShapeHandle cshape );
	extern  void plDeleteCollisionObject(plCollisionSdkHandle sdk, plCollisionObjectHandle body);
	
	
	/* Collision Shape definition */
	
	extern  plCollisionShapeHandle plNewSphereShape(plCollisionSdkHandle sdk, plReal radius);
	extern  plCollisionShapeHandle plNewCapsuleShape(plCollisionSdkHandle sdk, plReal radius, plReal height);
	extern  plCollisionShapeHandle plNewPlaneShape(plCollisionSdkHandle sdk, plReal planeNormalX, 
												   plReal planeNormalY, 
												   plReal planeNormalZ, 
												   plReal planeConstant);
	extern  plCollisionShapeHandle plNewCompoundShape(plCollisionSdkHandle sdk);
	extern  void    plAddChildShape(plCollisionShapeHandle compoundShape,plCollisionShapeHandle childShape, plVector3 childPos,plQuaternion childOrn);
	
	extern  void plDeleteShape(plCollisionShapeHandle shape);
	
	
	
	/* Contact Results */
	
	struct lwContactPoint
	{
		plVector3 m_ptOnAWorld;
		plVector3 m_ptOnBWorld;
		plVector3 m_normalOnB;
		plReal  m_distance;
	};
	
	/* Collision Filtering */
	typedef void(*plNearCallback)(plCollisionSdkHandle sdk, void* userData, plCollisionObjectHandle objA, plCollisionObjectHandle objB);
	
	
	/* Collision Queries */
	extern int plCollide(plCollisionSdkHandle sdk, plCollisionObjectHandle colA, plCollisionObjectHandle colB, 
						 lwContactPoint* pointsOut, int pointCapacity);
	
	extern void plWorldCollide(plCollisionWorldHandle world,
							   plNearCallback filter, void* userData);
	
	


#endif

