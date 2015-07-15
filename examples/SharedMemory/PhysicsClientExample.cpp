
#include "PhysicsClientExample.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "SharedMemoryCommon.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "PhysicsClient.h"
#include "SharedMemoryCommands.h"

class PhysicsClientExample : public SharedMemoryCommon
{
protected:
	PhysicsClientSharedMemory	m_physicsClient;

	
	bool m_wantsTermination;
    btAlignedObjectArray<int> m_userCommandRequests;

	void	createButton(const char* name, int id, bool isTrigger );

public:
    
	PhysicsClientExample(GUIHelperInterface* helper);
	virtual ~PhysicsClientExample();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
    
    virtual bool wantsTermination()
    {
        return m_wantsTermination;
    }
	void enqueueCommand(int command);
	
};


void MyCallback(int buttonId, bool buttonState, void* userPtr)
{
	PhysicsClientExample* cl = (PhysicsClientExample*) userPtr;
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


void PhysicsClientExample::enqueueCommand(int command)
{
	m_userCommandRequests.push_back(command);
	b3Printf("User put command request %d on queue (queue length = %d)\n",command, m_userCommandRequests.size());
}


PhysicsClientExample::PhysicsClientExample(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_wantsTermination(false)
{
	b3Printf("Started PhysicsClientExample\n");
}

PhysicsClientExample::~PhysicsClientExample()
{
    b3Printf("~PhysicsClientExample\n");
}

void	PhysicsClientExample::createButton(const char* name, int buttonId, bool isTrigger )
{
	ButtonParams button(name,buttonId,  isTrigger);
	button.m_callback = MyCallback;
	button.m_userPointer = this;
	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}
void	PhysicsClientExample::initPhysics()
{
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


void	PhysicsClientExample::stepSimulation(float deltaTime)
{
    
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


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClientExample(options.m_guiHelper);
}
