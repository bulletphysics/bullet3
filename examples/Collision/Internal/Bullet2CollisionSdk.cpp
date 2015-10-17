#include "Bullet2CollisionSdk.h"
#include "btBulletCollisionCommon.h"

struct Bullet2CollisionSdkInternalData
{
	btCollisionConfiguration* m_collisionConfig;
	btCollisionDispatcher* m_dispatcher;
	btBroadphaseInterface* m_aabbBroadphase;

	btCollisionWorld* m_collisionWorld;
	
	Bullet2CollisionSdkInternalData()
		:m_aabbBroadphase(0),
		m_dispatcher(0),
		m_collisionWorld(0)
	{
	}
};

	
Bullet2CollisionSdk::Bullet2CollisionSdk()
{
	m_internalData = new Bullet2CollisionSdkInternalData;
}
	
Bullet2CollisionSdk::~Bullet2CollisionSdk()
{
	delete m_internalData;
	m_internalData = 0;
}
	
plCollisionWorldHandle Bullet2CollisionSdk::createCollisionWorld()
{
	m_internalData->m_collisionConfig = new btDefaultCollisionConfiguration;
	m_internalData->m_dispatcher = new btCollisionDispatcher(m_internalData->m_collisionConfig);
	m_internalData->m_aabbBroadphase = new btDbvtBroadphase();
	m_internalData->m_collisionWorld = new btCollisionWorld(m_internalData->m_dispatcher,
															m_internalData->m_aabbBroadphase,
															m_internalData->m_collisionConfig);
	return (plCollisionWorldHandle) m_internalData->m_collisionWorld;
}
	
void Bullet2CollisionSdk::deleteCollisionWorld(plCollisionWorldHandle worldHandle)
{
	btCollisionWorld* world = (btCollisionWorld*) worldHandle;
	btAssert(m_internalData->m_collisionWorld == world);
	
	if (m_internalData->m_collisionWorld == world)
	{
		delete m_internalData->m_collisionWorld;
		m_internalData->m_collisionWorld=0;
		delete m_internalData->m_aabbBroadphase;
		m_internalData->m_aabbBroadphase=0;
		delete m_internalData->m_dispatcher;
		m_internalData->m_dispatcher=0;
		delete m_internalData->m_collisionConfig;
		m_internalData->m_collisionConfig=0;
	} 
}

plCollisionShapeHandle Bullet2CollisionSdk::createSphereShape(plReal radius)
{
	btSphereShape* sphereShape = new btSphereShape(radius);
	return (plCollisionShapeHandle) sphereShape;
}

void Bullet2CollisionSdk::deleteShape(plCollisionShapeHandle shapeHandle)
{
	btCollisionShape* shape = (btCollisionShape*) shapeHandle;
	delete shape;
}

void Bullet2CollisionSdk::addCollisionObject(plCollisionWorldHandle worldHandle, plCollisionObjectHandle objectHandle)
{
	btCollisionWorld* world = (btCollisionWorld*) worldHandle;
	btCollisionObject* colObj = (btCollisionObject*) objectHandle;
	btAssert(world && colObj); 
	if (world == m_internalData->m_collisionWorld && colObj)
	{
		world->addCollisionObject(colObj);
	}
}
void Bullet2CollisionSdk::removeCollisionObject(plCollisionWorldHandle worldHandle, plCollisionObjectHandle objectHandle)
{
	btCollisionWorld* world = (btCollisionWorld*) worldHandle;
	btCollisionObject* colObj = (btCollisionObject*) objectHandle;
	btAssert(world && colObj); 
	if (world == m_internalData->m_collisionWorld && colObj)
	{
		world->removeCollisionObject(colObj);
	} 
}

plCollisionObjectHandle Bullet2CollisionSdk::createCollisionObject(  void* user_data,  plCollisionShapeHandle shapeHandle )
{
	btCollisionShape* colShape =  (btCollisionShape*) shapeHandle;
	btAssert(colShape);
	if (colShape)
	{
		btCollisionObject* colObj= new btCollisionObject;
		colObj->setCollisionShape(colShape);
		colObj->setWorldTransform(btTransform::getIdentity());
		return (plCollisionObjectHandle) colObj;
	}
	return 0;
}

void Bullet2CollisionSdk::deleteCollisionObject(plCollisionObjectHandle bodyHandle)
{
	btCollisionObject* colObj = (btCollisionObject*) bodyHandle;
	delete colObj;
}


plCollisionSdkHandle Bullet2CollisionSdk::createBullet2SdkHandle()
{
	return (plCollisionSdkHandle) new Bullet2CollisionSdk;
}
