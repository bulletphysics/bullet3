#ifndef COLLISION_SDK_INTERFACE_H
#define COLLISION_SDK_INTERFACE_H

#include "../CollisionSdkC_Api.h"

class CollisionSdkInterface
{
public:
	virtual ~CollisionSdkInterface()
	{
	}

	virtual plCollisionWorldHandle createCollisionWorld(int maxNumObjsCapacity, int maxNumShapesCapacity, int maxNumPairsCapacity) = 0;

	virtual void deleteCollisionWorld(plCollisionWorldHandle worldHandle) = 0;

	virtual plCollisionShapeHandle createSphereShape(plCollisionWorldHandle worldHandle, plReal radius) = 0;

	virtual plCollisionShapeHandle createPlaneShape(plCollisionWorldHandle worldHandle,
													plReal planeNormalX,
													plReal planeNormalY,
													plReal planeNormalZ,
													plReal planeConstant) = 0;

	virtual plCollisionShapeHandle createCapsuleShape(plCollisionWorldHandle worldHandle,
													  plReal radius,
													  plReal height,
													  int capsuleAxis) = 0;

	virtual plCollisionShapeHandle createCompoundShape(plCollisionWorldHandle worldHandle) = 0;
	virtual void addChildShape(plCollisionWorldHandle worldHandle, plCollisionShapeHandle compoundShape, plCollisionShapeHandle childShape, plVector3 childPos, plQuaternion childOrn) = 0;

	virtual void deleteShape(plCollisionWorldHandle worldHandle, plCollisionShapeHandle shape) = 0;

	virtual void addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object) = 0;
	virtual void removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object) = 0;

	virtual plCollisionObjectHandle createCollisionObject(plCollisionWorldHandle worldHandle, void* userPointer, int userIndex, plCollisionShapeHandle cshape,
														  plVector3 startPosition, plQuaternion startOrientation) = 0;
	virtual void deleteCollisionObject(plCollisionObjectHandle body) = 0;
	virtual void setCollisionObjectTransform(plCollisionWorldHandle world, plCollisionObjectHandle body,
											 plVector3 position, plQuaternion orientation) = 0;

	virtual int collide(plCollisionWorldHandle world, plCollisionObjectHandle colA, plCollisionObjectHandle colB,
						lwContactPoint* pointsOut, int pointCapacity) = 0;

	virtual void collideWorld(plCollisionWorldHandle world,
							  plNearCallback filter, void* userData) = 0;
};

#endif  //COLLISION_SDK_INTERFACE_H
