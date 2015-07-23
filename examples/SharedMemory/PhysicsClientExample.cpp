
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
    btAlignedObjectArray<SharedMemoryCommand> m_userCommandRequests;

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
	void enqueueCommand(const SharedMemoryCommand& orgCommand);
	
};


void MyCallback(int buttonId, bool buttonState, void* userPtr)
{
	PhysicsClientExample* cl = (PhysicsClientExample*) userPtr;

	SharedMemoryCommand command;

	switch (buttonId)
	{
	case  CMD_LOAD_URDF:
		{
			command.m_type =CMD_LOAD_URDF;
			sprintf(command.m_urdfArguments.m_urdfFileName,"hinge.urdf");//r2d2.urdf");
			command.m_urdfArguments.m_initialPosition[0] = 0.0;
			command.m_urdfArguments.m_initialPosition[1] = 0.0;
			command.m_urdfArguments.m_initialPosition[2] = 0.0;
			command.m_urdfArguments.m_initialOrientation[0] = 0.0;
			command.m_urdfArguments.m_initialOrientation[1] = 0.0;
			command.m_urdfArguments.m_initialOrientation[2] = 0.0;
			command.m_urdfArguments.m_initialOrientation[3] = 1.0;
			command.m_urdfArguments.m_useFixedBase = false;
			command.m_urdfArguments.m_useMultiBody = true;
			cl->enqueueCommand(command);
			break;
		}
	case CMD_CREATE_BOX_COLLISION_SHAPE:
		{
			command.m_type =CMD_CREATE_BOX_COLLISION_SHAPE;
			cl->enqueueCommand(command);
			break;
		}
	case CMD_REQUEST_ACTUAL_STATE:
		{
			command.m_type =CMD_REQUEST_ACTUAL_STATE;
			cl->enqueueCommand(command);
			break;
		};
	case CMD_STEP_FORWARD_SIMULATION:
		{
			command.m_type =CMD_STEP_FORWARD_SIMULATION;
			cl->enqueueCommand(command);
			break;
		}
	
    case CMD_SEND_DESIRED_STATE:
		{
				command.m_type =CMD_SEND_DESIRED_STATE;
				int controlMode = CONTROL_MODE_VELOCITY;//CONTROL_MODE_TORQUE;

				command.m_sendDesiredStateCommandArgument.m_controlMode = controlMode;
				//todo: expose a drop box in the GUI for this
				switch (controlMode)
				{
				case CONTROL_MODE_VELOCITY:
					{
						for (int i=0;i<MAX_DEGREE_OF_FREEDOM;i++)
						{
							command.m_sendDesiredStateCommandArgument.m_desiredStateQdot[i] = 1;
							command.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[i] = 100;
						}
						break;
					}
				case CONTROL_MODE_TORQUE:
					{
						for (int i=0;i<MAX_DEGREE_OF_FREEDOM;i++)
						{
							command.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[i] = 100;
						}
						break;
					}
				default:
					{
						b3Printf("Unknown control mode in client CMD_SEND_DESIRED_STATE");
						btAssert(0);
					}
				}
			cl->enqueueCommand(command);		
			break;
		}
	case CMD_SEND_BULLET_DATA_STREAM:
		{
			command.m_type = buttonId;
			cl->enqueueCommand(command);
			break;
		}

	default:
		{
			b3Error("Unknown buttonId");
			btAssert(0);
		}
	};
}

 void PhysicsClientExample::enqueueCommand(const SharedMemoryCommand& orgCommand)
	{
		m_userCommandRequests.push_back(orgCommand);
		SharedMemoryCommand& cmd = m_userCommandRequests[m_userCommandRequests.size()-1];

		b3Printf("User put command request %d on queue (queue length = %d)\n",cmd.m_type, m_userCommandRequests.size());
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
		/*
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
		*/

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
		SharedMemoryStatus status;
		bool hasStatus = m_physicsClient.processServerStatus(status);
		if (hasStatus && status.m_type == CMD_URDF_LOADING_COMPLETED)
		{
			for (int i=0;i<m_physicsClient.getNumPoweredJoints();i++)
			{
				PoweredJointInfo info;
				m_physicsClient.getPoweredJointInfo(i,info);
				b3Printf("1-DOF PoweredJoint %s at q-index %d and u-index %d\n",info.m_jointName,info.m_qIndex,info.m_uIndex);
				
			}
		}
    
		if (m_physicsClient.canSubmitCommand())
		{
			if (m_userCommandRequests.size())
			{
				b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
				SharedMemoryCommand command = m_userCommandRequests[0];

				//a manual 'pop_front', we don't use 'remove' because it will re-order the commands
				for (int i=1;i<m_userCommandRequests.size();i++)
				{
					m_userCommandRequests[i-1] = m_userCommandRequests[i];
				}

				m_userCommandRequests.pop_back();
				
				m_physicsClient.submitClientCommand(command);
			}
		}
	}
	

}


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClientExample(options.m_guiHelper);
}
