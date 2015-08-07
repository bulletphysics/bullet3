


#include "PhysicsServerExample.h"




#include "PhysicsServer.h"

#include "SharedMemoryCommon.h"


class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;

    bool m_wantsShutdown;

    bool m_isConnected;
	
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
    virtual bool isConnected();
	virtual void	renderScene();
	virtual void    exitPhysics(){}

	virtual void	physicsDebugDraw(int debugFlags);
	virtual bool	mouseMoveCallback(float x,float y){return false;};
	virtual bool	mouseButtonCallback(int button, int state, float x, float y){return false;}
	virtual bool	keyboardCallback(int key, int state){return false;}

	virtual void setSharedMemoryKey(int key)
	{
		m_physicsServer.setSharedMemoryKey(key);
	}


};

PhysicsServerExample::PhysicsServerExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsShutdown(false),
m_isConnected(false)
{
	b3Printf("Started PhysicsServer\n");
}



PhysicsServerExample::~PhysicsServerExample()
{
	bool deInitializeSharedMemory = true;
	m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
    m_isConnected = false;
}

bool PhysicsServerExample::isConnected()
{
    return m_isConnected;
}

void	PhysicsServerExample::initPhysics()
{
	///for this testing we use Z-axis up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

#if 0
	
    createEmptyDynamicsWorld();

	//todo: create a special debug drawer that will cache the lines, so we can send the debug info over the wire
	btVector3 grav(0,0,0);
	grav[upAxis] = 0;//-9.8;
	this->m_dynamicsWorld->setGravity(grav);
    
#endif
	
	m_isConnected = m_physicsServer.connectSharedMemory( m_guiHelper);
  
}


bool PhysicsServerExample::wantsTermination()
{
    return m_wantsShutdown;
}



void	PhysicsServerExample::stepSimulation(float deltaTime)
{
    m_physicsServer.processClientCommands();
}

void PhysicsServerExample::renderScene()
{
	///debug rendering
	m_physicsServer.renderScene();
}

void    PhysicsServerExample::physicsDebugDraw(int debugDrawFlags)
{
	///debug rendering
	m_physicsServer.physicsDebugDraw(debugDrawFlags);

}

extern int gSharedMemoryKey;

class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
  	PhysicsServerExample* example = new PhysicsServerExample(options.m_guiHelper);
	if (gSharedMemoryKey>=0)
	{
		example->setSharedMemoryKey(gSharedMemoryKey);
	}
	return example;
	
}


