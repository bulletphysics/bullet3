


#include "PhysicsServerExample.h"




#include "PhysicsServer.h"

#include "SharedMemoryCommon.h"
//const char* blaatnaam = "basename";
struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btDefaultSerializer* m_memSerializer;

	UrdfLinkNameMapUtil():m_mb(0),m_memSerializer(0)
	{
	}
};

class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;

	
	bool m_wantsShutdown;

	
public:
    
	PhysicsServerExample(GUIHelperInterface* helper);
    
	virtual ~PhysicsServerExample();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
    
    
	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0};//-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
    
    virtual bool wantsTermination();
};

PhysicsServerExample::PhysicsServerExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsShutdown(false)
{
	b3Printf("Started PhysicsServer\n");
}



PhysicsServerExample::~PhysicsServerExample()
{
	bool deInitializeSharedMemory = true;
	m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
}

void	PhysicsServerExample::initPhysics()
{
	///for this testing we use Z-axis up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

    createEmptyDynamicsWorld();
	//todo: create a special debug drawer that will cache the lines, so we can send the debug info over the wire
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	btVector3 grav(0,0,0);
	grav[upAxis] = 0;//-9.8;
	this->m_dynamicsWorld->setGravity(grav);
    
	bool allowSharedMemoryInitialization = true;
	m_physicsServer.connectSharedMemory(allowSharedMemoryInitialization, m_dynamicsWorld,m_guiHelper);
  
}


bool PhysicsServerExample::wantsTermination()
{
    return m_wantsShutdown;
}



void	PhysicsServerExample::stepSimulation(float deltaTime)
{
    m_physicsServer.processClientCommands();
}


class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsServerExample(options.m_guiHelper);
}


