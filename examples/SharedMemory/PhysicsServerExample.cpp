


#include "PhysicsServerExample.h"



#include "PhysicsServerSharedMemory.h"

#include "SharedMemoryCommon.h"


class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;

    bool m_wantsShutdown;

    bool m_isConnected;
    btClock m_clock;
	bool m_replay;
	
public:
    
	PhysicsServerExample(GUIHelperInterface* helper);
    
	virtual ~PhysicsServerExample();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
    void enableCommandLogging()
	{
		m_physicsServer.enableCommandLogging(true,"BulletPhysicsCommandLog.bin");
	}

	void replayFromLogFile()
	{
		m_replay = true;
		m_physicsServer.replayFromLogFile("BulletPhysicsCommandLog.bin");
	}
	

    
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
	
	btVector3	getRayTo(int x,int y);
	
	virtual bool	mouseMoveCallback(float x,float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
		if (!renderer)
		{
			btAssert(0);
			return false;
		}

		btVector3 rayTo = getRayTo(int(x), int(y));
		btVector3 rayFrom;
		renderer->getActiveCamera()->getCameraPosition(rayFrom);
		m_physicsServer.movePickedBody(rayFrom,rayTo);
		return false;
	};

	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
		if (!renderer)
		{
			btAssert(0);
			return false;
		}
		
		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

	
		if (state==1)
		{
			if(button==0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL) ))
			{
				btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x),int(y));

				m_physicsServer.pickBody(rayFrom, rayTo);


			}
		} else
		{
			if (button==0)
			{
				m_physicsServer.removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}
	virtual bool	keyboardCallback(int key, int state){return false;}

	virtual void setSharedMemoryKey(int key)
	{
		m_physicsServer.setSharedMemoryKey(key);
	}


};

PhysicsServerExample::PhysicsServerExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsShutdown(false),
m_isConnected(false),
m_replay(false)
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
	if (m_replay)
	{
		for (int i=0;i<100;i++)
			m_physicsServer.processClientCommands();
	} else
	{
		btClock rtc;
		btScalar endTime = rtc.getTimeMilliseconds() + deltaTime*btScalar(800);

		while (rtc.getTimeMilliseconds()<endTime)
		{
			m_physicsServer.processClientCommands();
		}
	}
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



btVector3	PhysicsServerExample::getRayTo(int x,int y)
{
	CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		
	if (!renderer)
	{
		btAssert(0);
		return btVector3(0,0,0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3 camPos,camTarget;
	renderer->getActiveCamera()->getCameraPosition(camPos);
	renderer->getActiveCamera()->getCameraTargetPosition(camTarget);

	btVector3	rayFrom = camPos;
	btVector3 rayForward = (camTarget-camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 cameraUp=btVector3(0,0,0);
	cameraUp[m_guiHelper->getAppInterface()->getUpAxis()]=1;

	btVector3 vertical = cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;
	float width = float(renderer->getScreenWidth());
	float height = float (renderer->getScreenHeight());

	aspect =  width / height;

	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/width;
	btVector3 dVert = vertical * 1.f/height;


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}


extern int gSharedMemoryKey;

class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
  	PhysicsServerExample* example = new PhysicsServerExample(options.m_guiHelper);
	if (gSharedMemoryKey>=0)
	{
		example->setSharedMemoryKey(gSharedMemoryKey);
	}
	if (options.m_option & PHYSICS_SERVER_ENABLE_COMMAND_LOGGING)
	{
		example->enableCommandLogging();
	}
	if (options.m_option & PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG)
	{
		example->replayFromLogFile();
	}
	return example;
	
}
