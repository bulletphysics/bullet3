#ifndef COLLISION_SDK_INTERFACE_H
#define COLLISION_SDK_INTERFACE_H

#include "../CollisionSdkC_Api.h"

class CollisionSdkInterface
{
public:
	
	virtual ~CollisionSdkInterface()
	{
	}
	
	virtual plCollisionWorldHandle createCollisionWorld() = 0;
	
	virtual void deleteCollisionWorld(plCollisionWorldHandle worldHandle) = 0;
	
	virtual plCollisionShapeHandle createSphereShape(plReal radius) = 0;
	
	virtual void deleteShape(plCollisionShapeHandle shape) = 0;
	
	virtual void addCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)=0;
	virtual  void removeCollisionObject(plCollisionWorldHandle world, plCollisionObjectHandle object)=0;
	
	virtual  plCollisionObjectHandle createCollisionObject(  void* user_data,  plCollisionShapeHandle cshape )=0;
	virtual  void deleteCollisionObject(plCollisionObjectHandle body)=0;
	

};

#endif //COLLISION_SDK_INTERFACE_H

