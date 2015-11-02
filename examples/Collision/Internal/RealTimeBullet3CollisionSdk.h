#ifndef REAL_TIME_COLLISION_SDK_H
#define REAL_TIME_COLLISION_SDK_H

#include "CollisionSdkInterface.h"

class RealTimeBullet3CollisionSdk : public CollisionSdkInterface
{
	struct RealTimeBullet3CollisionSdkInternalData* m_internalData;
	
public:
	
	RealTimeBullet3CollisionSdk();
	
	virtual ~RealTimeBullet3CollisionSdk();
	
	virtual plCollisionWorldHandle createCollisionWorld(int maxNumObjsCapacity, int maxNumShapesCapacity, int maxNumPairsCapacity);
	
	virtual void deleteCollisionWorld(plCollisionWorldHandle worldHandle);

	virtual plCollisionShapeHandle createSphereShape(plCollisionWorldHandle worldHandle, plReal radius);
	virtual plCollisionShapeHandle createPlaneShape(plCollisionWorldHandle worldHandle, 
													plReal planeNormalX, 
													plReal planeNormalY, 
													plReal planeNormalZ, 
													plReal planeConstant);
	virtual plCollisionShapeHandle createCapsuleShape(plCollisionWorldHandle worldHandle, 
													plReal radius,
													plReal height,
													int capsuleAxis);
	
	virtual plCollisionShapeHandle createCompoundShape(plCollisionWorldHandle worldHandle);
	virtual void addChildShape(plCollisionWorldHandle worldHandle,plCollisionShapeHandle compoundShape, plCollisionShapeHandle childShape,plVector3 childPos,plQuaternion childOrn);

	virtual void deleteShape(plCollisionWorldHandle worldHandle, plCollisionShapeHandle shape);
	
	virtual void addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	virtual  void removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	
    virtual  plCollisionObjectHandle createCollisionObject( plCollisionWorldHandle worldHandle, void* userPointer, int userIndex,  plCollisionShapeHandle cshape ,
                                                           plVector3 startPosition,plQuaternion startOrientation );
    virtual  void deleteCollisionObject(plCollisionObjectHandle body);
	virtual void setCollisionObjectTransform(plCollisionWorldHandle world, plCollisionObjectHandle body,
												plVector3 position,plQuaternion orientation );
	
    virtual int collide(plCollisionWorldHandle world,plCollisionObjectHandle colA, plCollisionObjectHandle colB,
                        lwContactPoint* pointsOut, int pointCapacity);
	
    virtual void collideWorld( plCollisionWorldHandle world,
                              plNearCallback filter, void* userData);

	static plCollisionSdkHandle createRealTimeBullet3CollisionSdkHandle();
};


#endif //REAL_TIME_COLLISION_SDK_H
