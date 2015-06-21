
#include "PhysicsClient.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "SharedMemoryCommon.h"
#include "../CommonInterfaces/CommonParameterInterface.h"


class PhysicsClient : public SharedMemoryCommon
{
	SharedMemoryInterface* m_sharedMemory;
	SharedMemoryExampleData*   m_testBlock1;
    int m_counter;
    bool m_wantsTermination;
	btAlignedObjectArray<int> m_userCommandRequests;

	bool m_serverLoadUrdfOK;
	bool m_waitingForServer;
	


    
public:
    
	PhysicsClient(GUIHelperInterface* helper);
	virtual ~PhysicsClient();
    
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
	void submitCommand(int command);
	
};


void MyCallback(int buttonId, bool buttonState, void* userPtr)
{
	PhysicsClient* cl = (PhysicsClient*) userPtr;
	switch (buttonId)
	{
	case  CMD_LOAD_URDF:
	{
		cl->submitCommand(CMD_LOAD_URDF);
		break;
	}
	case CMD_REQUEST_ACTUAL_STATE:
		{
			cl->submitCommand(CMD_REQUEST_ACTUAL_STATE);
			break;
		}
	case CMD_STEP_FORWARD_SIMULATION:
		{
			cl->submitCommand(CMD_STEP_FORWARD_SIMULATION);
			break;
		}
	case CMD_SHUTDOWN:
		{
			cl->submitCommand(CMD_SHUTDOWN);
			break;
		}

	default:
		{
			b3Error("Unknown buttonId");
			btAssert(0);
		}
	};
}


void PhysicsClient::submitCommand(int command)
{
	m_userCommandRequests.push_back(command);
	b3Printf("User submitted command request  %d (outstanding %d)\n",command, m_userCommandRequests.size());
}


PhysicsClient::PhysicsClient(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_testBlock1(0),
m_counter(0),
m_wantsTermination(false),
m_serverLoadUrdfOK(false),
m_waitingForServer(false)
{
	b3Printf("Started PhysicsClient");
#ifdef _WIN32
	m_sharedMemory = new Win32SharedMemoryClient();
#else
	m_sharedMemory = new PosixSharedMemory();
#endif
}

PhysicsClient::~PhysicsClient()
{
    b3Printf("~PhysicsClient");
    m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
	delete m_sharedMemory;
}


void	PhysicsClient::initPhysics()
{
	{
		bool isTrigger = false;
		ButtonParams button("Load URDF",CMD_LOAD_URDF,  isTrigger);
		button.m_callback = MyCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
    }

	{
		bool isTrigger = false;
		ButtonParams button("Step Sim",CMD_STEP_FORWARD_SIMULATION,  isTrigger);
		button.m_callback = MyCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
    }

	{
		bool isTrigger = false;
		ButtonParams button("Get State",CMD_REQUEST_ACTUAL_STATE,  isTrigger);
		button.m_callback = MyCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
    }
	/*
	{
		bool isTrigger = false;
		ButtonParams button("Send Desired State",CMD_SEND_DESIRED_STATE,  isTrigger);
		button.m_callback = MyCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
    }
	*/
	{
		bool isTrigger = false;
		ButtonParams button("Shut Down",CMD_SHUTDOWN,  isTrigger);
		button.m_callback = MyCallback;
		button.m_userPointer = this;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);
    }


    m_testBlock1 = (SharedMemoryExampleData*)m_sharedMemory->allocateSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    if (m_testBlock1)
    {
     //   btAssert(m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER);
        if (m_testBlock1->m_magicId !=SHARED_MEMORY_MAGIC_NUMBER)
        {
            b3Error("Error: please start server before client");
            m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
            m_testBlock1 = 0;
        } else
		{
			b3Printf("Shared Memory status is OK");
		}
    }
    
}

void	PhysicsClient::stepSimulation(float deltaTime)
{
    
    
    if (m_testBlock1)
    {
        //check progress and submit further commands
        //we ignore overflow right now
        
        if (m_testBlock1->m_numServerCommands> m_testBlock1->m_numProcessedServerCommands)
        {
            btAssert(m_testBlock1->m_numServerCommands==m_testBlock1->m_numProcessedServerCommands+1);
        
            const SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
            
            //consume the command
            switch (serverCmd.m_type)
            {

				case CMD_URDF_LOADING_COMPLETED:
				{
					m_serverLoadUrdfOK = true;
					b3Printf("Server loading the URDF OK");
					break;
				}
                case CMD_STEP_FORWARD_SIMULATION_COMPLETED:
                {
                    break;
                }
                case CMD_URDF_LOADING_FAILED:
                {
                    b3Printf("Server failed loading the URDF...");
					m_serverLoadUrdfOK = false;
                    break;
                }
				case CMD_ACTUAL_STATE_UPDATE_COMPLETED:
					{
						b3Printf("Received actual state");
						
						int numQ = m_testBlock1->m_serverCommands[0].m_sendActualStateArgs.m_numDegreeOfFreedomQ;
						int numU = m_testBlock1->m_serverCommands[0].m_sendActualStateArgs.m_numDegreeOfFreedomU;
						b3Printf("size Q = %d, size U = %d\n", numQ,numU);
						char msg[1024];

						sprintf(msg,"Q=[");
						
						for (int i=0;i<numQ;i++)
						{
							if (i<numQ-1)
							{
								sprintf(msg,"%s%f,",msg,m_testBlock1->m_actualStateQ[i]);
							} else
							{
								sprintf(msg,"%s%f",msg,m_testBlock1->m_actualStateQ[i]);
							}
						}
						sprintf(msg,"%s]",msg);
						
						b3Printf(msg);
						
						break;
					}
                default:
                {
                    b3Error("Unknown server command");
                    btAssert(0);
                }
            };
			
			
            m_testBlock1->m_numProcessedServerCommands++;

			if (m_testBlock1->m_numServerCommands == m_testBlock1->m_numProcessedServerCommands)
			{
				m_waitingForServer = false;
			} else
			{
				m_waitingForServer = true;
			}
        }
    
		if (!m_waitingForServer)
		{
			//process outstanding requests
			if (m_userCommandRequests.size())
			{
				b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
				int command = m_userCommandRequests[0];
				m_userCommandRequests.remove(command);
				switch (command)
				{
				case CMD_LOAD_URDF:
					{
						if (!m_serverLoadUrdfOK)
						{
							m_testBlock1->m_clientCommands[0].m_type =CMD_LOAD_URDF;
							sprintf(m_testBlock1->m_clientCommands[0].m_urdfArguments.m_urdfFileName,"r2d2.urdf");
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_useFixedBase = false;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_useMultiBody = true;

							m_testBlock1->m_numClientCommands++;
							b3Printf("Client created CMD_LOAD_URDF");
							m_waitingForServer = true;
						} else
						{
							b3Warning("Server already loaded URDF, no client command submitted");
						}
						break;
					}
				case CMD_REQUEST_ACTUAL_STATE:
					{
						if (m_serverLoadUrdfOK)
						{
							b3Printf("Requesting actual state");
							m_testBlock1->m_clientCommands[0].m_type =CMD_REQUEST_ACTUAL_STATE;
							m_testBlock1->m_numClientCommands++;

						} else
						{
							b3Warning("No URDF loaded");
						}
						break;
					}
				case CMD_STEP_FORWARD_SIMULATION:
					{
						if (m_serverLoadUrdfOK)
						{
						
							m_testBlock1->m_clientCommands[0].m_type =CMD_STEP_FORWARD_SIMULATION;
							m_testBlock1->m_clientCommands[0].m_stepSimulationArguments.m_deltaTimeInSeconds = 1./60.;
							m_testBlock1->m_numClientCommands++;
							b3Printf("client created CMD_STEP_FORWARD_SIMULATION %d\n", m_counter++);
							m_waitingForServer = true;
						} else
						{
							b3Warning("No URDF loaded yet, no client CMD_STEP_FORWARD_SIMULATION submitted");
						}
						break;
					}
				case CMD_SHUTDOWN:
					{
						m_wantsTermination = true;
						m_testBlock1->m_clientCommands[0].m_type =CMD_SHUTDOWN;
						m_testBlock1->m_numClientCommands++;
						m_waitingForServer = true;
						m_serverLoadUrdfOK = false;
						b3Printf("client created CMD_SHUTDOWN\n");
						break;
					}
				default:
					{
						b3Error("unknown command requested");
					}
				}
			}
		}

	  }
	

}


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClient(options.m_guiHelper);
}
