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

plCollisionObjectHandle Bullet2CollisionSdk::createCollisionObject(  void* user_data,  plCollisionShapeHandle shapeHandle ,
                                                       plVector3 startPosition,plQuaternion startOrientation )

{
	btCollisionShape* colShape =  (btCollisionShape*) shapeHandle;
	btAssert(colShape);
	if (colShape)
	{
		btCollisionObject* colObj= new btCollisionObject;
		colObj->setCollisionShape(colShape);
        btTransform tr;
        tr.setOrigin(btVector3(startPosition[0],startPosition[1],startPosition[2]));
        tr.setRotation(btQuaternion(startOrientation[0],startOrientation[1],startOrientation[2],startOrientation[3]));
		colObj->setWorldTransform(tr);
		return (plCollisionObjectHandle) colObj;
	}
	return 0;
}

void Bullet2CollisionSdk::deleteCollisionObject(plCollisionObjectHandle bodyHandle)
{
	btCollisionObject* colObj = (btCollisionObject*) bodyHandle;
	delete colObj;
}

struct   Bullet2ContactResultCallback : public btCollisionWorld::ContactResultCallback
{
    int m_numContacts;
    lwContactPoint* m_pointsOut;
    int m_pointCapacity;
    
    Bullet2ContactResultCallback() :m_numContacts(0)
    {
    }
    virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
    {
        if (m_numContacts<m_pointCapacity)
        {
            lwContactPoint& ptOut = m_pointsOut[m_numContacts];
            ptOut.m_distance = cp.m_distance1;
            ptOut.m_normalOnB[0] = cp.m_normalWorldOnB.getX();
            ptOut.m_normalOnB[1] = cp.m_normalWorldOnB.getY();
            ptOut.m_normalOnB[2] = cp.m_normalWorldOnB.getZ();
            m_numContacts++;
        }
        
        return 1.f;
    }
};

int Bullet2CollisionSdk::collide(plCollisionWorldHandle worldHandle,plCollisionObjectHandle colA, plCollisionObjectHandle colB,
                    lwContactPoint* pointsOut, int pointCapacity)
{
    btCollisionWorld* world = (btCollisionWorld*) worldHandle;
    btCollisionObject* colObjA = (btCollisionObject*) colA;
    btCollisionObject* colObjB = (btCollisionObject*) colB;
    btAssert(world && colObjA && colObjB);
    if (world == m_internalData->m_collisionWorld && colObjA && colObjB)
    {
        Bullet2ContactResultCallback cb;
        world->contactPairTest(colObjA,colObjB,cb);
        return cb.m_numContacts;
    }
    return 0;
}

static plNearCallback gTmpFilter;
static int gContactCount = 0;

void Bullet2NearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
    if (gTmpFilter)
    {
        gContactCount++;
    }
}

void Bullet2CollisionSdk::collideWorld( plCollisionWorldHandle worldHandle,
                          plNearCallback filter, void* userData)
{
    btCollisionWorld* world = (btCollisionWorld*) worldHandle;
    //chain the near-callback
    gTmpFilter = filter;
    gContactCount = 0;
    
    m_internalData->m_dispatcher->setNearCallback(Bullet2NearCallback);
    world->performDiscreteCollisionDetection();
    gTmpFilter = 0;
}

plCollisionSdkHandle Bullet2CollisionSdk::createBullet2SdkHandle()
{
	return (plCollisionSdkHandle) new Bullet2CollisionSdk;
}
