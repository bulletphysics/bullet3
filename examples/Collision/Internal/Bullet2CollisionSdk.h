#ifndef BULLET2_COLLISION_SDK_H
#define BULLET2_COLLISION_SDK_H

#include "CollisionSdkInterface.h"

class Bullet2CollisionSdk : public CollisionSdkInterface
{
	struct Bullet2CollisionSdkInternalData* m_internalData;
	
public:
	
	Bullet2CollisionSdk();
	
	virtual ~Bullet2CollisionSdk();
	
	virtual plCollisionWorldHandle createCollisionWorld();
	
	virtual void deleteCollisionWorld(plCollisionWorldHandle worldHandle);

	virtual plCollisionShapeHandle createSphereShape(plReal radius);
	
	virtual void deleteShape(plCollisionShapeHandle shape);
	
	virtual void addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	virtual  void removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object);
	
    virtual  plCollisionObjectHandle createCollisionObject(  void* user_data,  plCollisionShapeHandle cshape ,
                                                           plVector3 startPosition,plQuaternion startOrientation );
    virtual  void deleteCollisionObject(plCollisionObjectHandle body);
	
    virtual int collide(plCollisionWorldHandle world,plCollisionObjectHandle colA, plCollisionObjectHandle colB,
                        lwContactPoint* pointsOut, int pointCapacity);
	
    virtual void collideWorld( plCollisionWorldHandle world,
                              plNearCallback filter, void* userData);

	static plCollisionSdkHandle createBullet2SdkHandle();
};

#endif //BULLET2_COLLISION_SDK_H
