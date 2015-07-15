


#include "RobotControlExample.h"



#include "../CommonInterfaces/CommonParameterInterface.h"
#include "PhysicsServer.h"
#include "PhysicsClient.h"
#include "SharedMemoryCommon.h"
//const char* blaatnaam = "basename";




class RobotControlExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;
	PhysicsClientSharedMemory	m_physicsClient;
	
	bool m_wantsShutdown;
	   
	btAlignedObjectArray<int> m_userCommandRequests;

	void	createButton(const char* name, int id, bool isTrigger );
	
public:
    
	RobotControlExample(GUIHelperInterface* helper);
    
	virtual ~RobotControlExample();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
    void enqueueCommand(int command)
	{
		m_userCommandRequests.push_back(command);
		b3Printf("User put command request %d on queue (queue length = %d)\n",command, m_userCommandRequests.size());
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
};


void MyCallback2(int buttonId, bool buttonState, void* userPtr)
{
	RobotControlExample* cl = (RobotControlExample*) userPtr;
	switch (buttonId)
	{
	case  CMD_LOAD_URDF:
	case CMD_CREATE_BOX_COLLISION_SHAPE:
	case CMD_REQUEST_ACTUAL_STATE:
	case CMD_STEP_FORWARD_SIMULATION:
	case CMD_SHUTDOWN:
    case CMD_SEND_DESIRED_STATE:
	case CMD_SEND_BULLET_DATA_STREAM:
		{
			cl->enqueueCommand(buttonId);
			break;
		}

	default:
		{
			b3Error("Unknown buttonId");
			btAssert(0);
		}
	};
}
void	RobotControlExample::createButton(const char* name, int buttonId, bool isTrigger )
{
	ButtonParams button(name,buttonId,  isTrigger);
	button.m_callback = MyCallback2;
	button.m_userPointer = this;
	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}

RobotControlExample::RobotControlExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsShutdown(false)
{
	
	bool useServer = true;
}



RobotControlExample::~RobotControlExample()
{
	bool deInitializeSharedMemory = true;
	m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
}

void	RobotControlExample::initPhysics()
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
  
	if (m_guiHelper && m_guiHelper->getParameterInterface())
	{
		bool isTrigger = false;
		
		createButton("Load URDF",CMD_LOAD_URDF,  isTrigger);
		createButton("Step Sim",CMD_STEP_FORWARD_SIMULATION,  isTrigger);
		createButton("Send Bullet Stream",CMD_SEND_BULLET_DATA_STREAM,  isTrigger);
		createButton("Get State",CMD_REQUEST_ACTUAL_STATE,  isTrigger);
		createButton("Send Desired State",CMD_SEND_DESIRED_STATE,  isTrigger);
		createButton("Create Box Collider",CMD_CREATE_BOX_COLLISION_SHAPE,isTrigger);
		
	} else
	{
		m_userCommandRequests.push_back(CMD_LOAD_URDF);
		m_userCommandRequests.push_back(CMD_REQUEST_ACTUAL_STATE);
		m_userCommandRequests.push_back(CMD_SEND_DESIRED_STATE);
		m_userCommandRequests.push_back(CMD_REQUEST_ACTUAL_STATE);
		//m_userCommandRequests.push_back(CMD_SET_JOINT_FEEDBACK);
		m_userCommandRequests.push_back(CMD_CREATE_BOX_COLLISION_SHAPE);
		//m_userCommandRequests.push_back(CMD_CREATE_RIGID_BODY);
		m_userCommandRequests.push_back(CMD_STEP_FORWARD_SIMULATION);
		m_userCommandRequests.push_back(CMD_REQUEST_ACTUAL_STATE);
		m_userCommandRequests.push_back(CMD_SHUTDOWN);
		
	}

	if (!m_physicsClient.connect())
	{
		b3Warning("Cannot eonnect to physics client");
	}

}


bool RobotControlExample::wantsTermination()
{
    return m_wantsShutdown;
}



void	RobotControlExample::stepSimulation(float deltaTime)
{
    m_physicsServer.processClientCommands();

	if (m_physicsClient.isConnected())
    {
		m_physicsClient.processServerStatus();
    
		if (m_physicsClient.canSubmitCommand())
		{
			if (m_userCommandRequests.size())
			{
				b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
				int command = m_userCommandRequests[0];

				//a manual 'pop_front', we don't use 'remove' because it will re-order the commands
				for (int i=1;i<m_userCommandRequests.size();i++)
				{
					m_userCommandRequests[i-1] = m_userCommandRequests[i];
				}

				m_userCommandRequests.pop_back();
				SharedMemoryCommand tmp;
				tmp.m_type = command;
				m_physicsClient.submitClientCommand(tmp);
			}
		}
	}
}


class CommonExampleInterface*    RobotControlExampleCreateFunc(struct CommonExampleOptions& options)
{
    return new RobotControlExample(options.m_guiHelper);
}


