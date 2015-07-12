
#include "PhysicsClient.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "SharedMemoryCommon.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../Utils/b3ResourcePath.h"
#include "../Extras/Serialize/BulletFileLoader/btBulletFile.h"

class PhysicsClient : public SharedMemoryCommon
{
protected:
	SharedMemoryInterface* m_sharedMemory;
	SharedMemoryExampleData*   m_testBlock1;
    int m_counter;
    bool m_wantsTermination;
	btAlignedObjectArray<int> m_userCommandRequests;

	bool m_serverLoadUrdfOK;
	bool m_waitingForServer;
	
	void	processServerCommands();
	void	createClientCommand();

    
	void	createButton(const char* name, int id, bool isTrigger );

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
	case CMD_CREATE_BOX_COLLISION_SHAPE:
	case CMD_REQUEST_ACTUAL_STATE:
	case CMD_STEP_FORWARD_SIMULATION:
	case CMD_SHUTDOWN:
	case CMD_SEND_BULLET_DATA_STREAM:
		{
			cl->submitCommand(buttonId);
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
	b3Printf("Started PhysicsClient\n");
#ifdef _WIN32
	m_sharedMemory = new Win32SharedMemoryClient();
#else
	m_sharedMemory = new PosixSharedMemory();
#endif
}

PhysicsClient::~PhysicsClient()
{
    b3Printf("~PhysicsClient\n");
    m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
	delete m_sharedMemory;
}

void	PhysicsClient::createButton(const char* name, int buttonId, bool isTrigger )
{
	ButtonParams button(name,buttonId,  isTrigger);
	button.m_callback = MyCallback;
	button.m_userPointer = this;
	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}
void	PhysicsClient::initPhysics()
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
		//m_userCommandRequests.push_back(CMD_SEND_DESIRED_STATE);
		m_userCommandRequests.push_back(CMD_REQUEST_ACTUAL_STATE);
		//m_userCommandRequests.push_back(CMD_SET_JOINT_FEEDBACK);
		m_userCommandRequests.push_back(CMD_CREATE_BOX_COLLISION_SHAPE);
		//m_userCommandRequests.push_back(CMD_CREATE_RIGID_BODY);
		m_userCommandRequests.push_back(CMD_STEP_FORWARD_SIMULATION);
		m_userCommandRequests.push_back(CMD_REQUEST_ACTUAL_STATE);
		m_userCommandRequests.push_back(CMD_SHUTDOWN);
		
	}


    m_testBlock1 = (SharedMemoryExampleData*)m_sharedMemory->allocateSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    if (m_testBlock1)
    {
     //   btAssert(m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER);
        if (m_testBlock1->m_magicId !=SHARED_MEMORY_MAGIC_NUMBER)
        {
            b3Error("Error: please start server before client\n");
            m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
            m_testBlock1 = 0;
        } else
		{
			b3Printf("Shared Memory status is OK\n");
		}
    } else
	{
		m_wantsTermination = true;
	}
}

void	PhysicsClient::processServerCommands()
{
	btAssert(m_testBlock1);

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
				b3Printf("Server loading the URDF OK\n");

				if (serverCmd.m_dataStreamArguments.m_streamChunkLength>0)
					{
						bParse::btBulletFile* bf = new bParse::btBulletFile(this->m_testBlock1->m_bulletStreamDataServerToClient,serverCmd.m_dataStreamArguments.m_streamChunkLength);
						bf->setFileDNAisMemoryDNA();
						bf->parse(false);
						for (int i=0;i<bf->m_multiBodies.size();i++)
						{
							int flag = bf->getFlags();

							if ((flag&bParse::FD_DOUBLE_PRECISION)!=0)
							{
								btMultiBodyDoubleData* mb = (btMultiBodyDoubleData*)bf->m_multiBodies[i];
								if (mb->m_baseName)
								{
									b3Printf("mb->m_baseName = %s\n",mb->m_baseName);
								}
								for (int link=0;link<mb->m_numLinks;link++)
								{
									if (mb->m_links[link].m_linkName)
									{
										b3Printf("mb->m_links[%d].m_linkName = %s\n",link,mb->m_links[link].m_linkName);
									}
									if (mb->m_links[link].m_jointName)
									{
										b3Printf("mb->m_links[%d].m_jointName = %s\n",link,mb->m_links[link].m_jointName);
									}
								}
							} else
							{
								btMultiBodyFloatData* mb = (btMultiBodyFloatData*) bf->m_multiBodies[i];
								if (mb->m_baseName)
								{
									b3Printf("mb->m_baseName = %s\n",mb->m_baseName);
								}
								for (int link=0;link<mb->m_numLinks;link++)
								{
									if (mb->m_links[link].m_linkName)
									{
										b3Printf("mb->m_links[%d].m_linkName = %s\n",link,mb->m_links[link].m_linkName);
									}
									b3Printf("link [%d] type = %d",link, mb->m_links[link].m_jointType);
									if (mb->m_links[link].m_jointName)
									{
										b3Printf("mb->m_links[%d].m_jointName = %s\n",link,mb->m_links[link].m_jointName);
									}
								}
							}
						}
						printf("ok!\n");
					}
				break;
			}
			case CMD_STEP_FORWARD_SIMULATION_COMPLETED:
			{
				break;
			}
			case CMD_URDF_LOADING_FAILED:
			{
				b3Printf("Server failed loading the URDF...\n");
				m_serverLoadUrdfOK = false;
				break;
			}

			case CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED:
				{
					b3Printf("Server received bullet data stream OK\n");

					


					break;
				}
			case CMD_BULLET_DATA_STREAM_RECEIVED_FAILED:
				{
					b3Printf("Server failed receiving bullet data stream\n");

					break;
				}
                    

			case CMD_ACTUAL_STATE_UPDATE_COMPLETED:
				{
					b3Printf("Received actual state\n");
						
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
					b3Printf("\n");
					break;
				}
			default:
			{
				b3Error("Unknown server command\n");
				btAssert(0);
			}
		};
			
			
		m_testBlock1->m_numProcessedServerCommands++;
		//we don't have more than 1 command outstanding (in total, either server or client)
		btAssert(m_testBlock1->m_numProcessedServerCommands == m_testBlock1->m_numServerCommands);

		if (m_testBlock1->m_numServerCommands == m_testBlock1->m_numProcessedServerCommands)
		{
			m_waitingForServer = false;
		} else
		{
			m_waitingForServer = true;
		}
	}
}

void	PhysicsClient::createClientCommand()
{
	if (!m_waitingForServer)
		{
			//process outstanding requests
			if (m_userCommandRequests.size())
			{
				b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
				int command = m_userCommandRequests[0];
				
				//don't use 'remove' because it will re-order the commands
				//m_userCommandRequests.remove(command);
				//pop_front
				for (int i=1;i<m_userCommandRequests.size();i++)
				{
					m_userCommandRequests[i-1] = m_userCommandRequests[i];
				}

				m_userCommandRequests.pop_back();
				m_waitingForServer = true;

				switch (command)
				{
				case CMD_LOAD_URDF:
					{
						if (!m_serverLoadUrdfOK)
						{
							m_testBlock1->m_clientCommands[0].m_type =CMD_LOAD_URDF;
							sprintf(m_testBlock1->m_clientCommands[0].m_urdfArguments.m_urdfFileName,"r2d2.urdf");
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialPosition[0] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialPosition[1] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialPosition[2] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialOrientation[0] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialOrientation[1] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialOrientation[2] = 0.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_initialOrientation[3] = 1.0;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_useFixedBase = false;
							m_testBlock1->m_clientCommands[0].m_urdfArguments.m_useMultiBody = true;

							m_testBlock1->m_numClientCommands++;
							b3Printf("Client created CMD_LOAD_URDF\n");
						} else
						{
							b3Warning("Server already loaded URDF, no client command submitted\n");
						}
						break;
					}
				case CMD_CREATE_BOX_COLLISION_SHAPE:
					{
						if (m_serverLoadUrdfOK)
						{
							b3Printf("Requesting create box collision shape\n");
							m_testBlock1->m_clientCommands[0].m_type =CMD_CREATE_BOX_COLLISION_SHAPE;
							m_testBlock1->m_numClientCommands++;
						} else
						{
							b3Warning("No URDF loaded\n");
						}
						break;
					}
				case CMD_SEND_BULLET_DATA_STREAM:
					{
						b3Printf("Sending a Bullet Data Stream\n");
						///The idea is to pass a stream of chunks from client to server
						///over shared memory. The server will process it
						///Initially we will just copy an entire .bullet file into shared
						///memory but we can also send individual chunks one at a time
						///so it becomes a streaming solution
						///In addition, we can make a separate API to create those chunks
						///if needed, instead of using a 3d modeler or the Bullet SDK btSerializer
						
						char relativeFileName[1024];
						const char* fileName = "slope.bullet";
						bool fileFound = b3ResourcePath::findResourcePath(fileName,relativeFileName,1024);
						if (fileFound)
						{
							FILE *fp = fopen(relativeFileName, "rb");
							if (fp)
							{
								fseek(fp, 0L, SEEK_END);
								int mFileLen = ftell(fp);
								fseek(fp, 0L, SEEK_SET);
								if (mFileLen<SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE)
								{
								
									fread(m_testBlock1->m_bulletStreamDataClientToServer, mFileLen, 1, fp);

									fclose(fp);

									m_testBlock1->m_clientCommands[0].m_type =CMD_SEND_BULLET_DATA_STREAM;
									m_testBlock1->m_clientCommands[0].m_dataStreamArguments.m_streamChunkLength = mFileLen;
									m_testBlock1->m_numClientCommands++;
									b3Printf("Send bullet data stream command\n");
								} else
								{
									b3Warning("Bullet file size (%d) exceeds of streaming memory chunk size (%d)\n", mFileLen,SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
								}
							} else
							{
								b3Warning("Cannot open file %s\n", relativeFileName);
							}
						} else
						{
							b3Warning("Cannot find file %s\n", fileName);
						}
						
						break;
					}
				case CMD_REQUEST_ACTUAL_STATE:
					{
						if (m_serverLoadUrdfOK)
						{
							b3Printf("Requesting actual state\n");
							m_testBlock1->m_clientCommands[0].m_type =CMD_REQUEST_ACTUAL_STATE;
							m_testBlock1->m_numClientCommands++;

						} else
						{
							b3Warning("No URDF loaded\n");
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
						} else
						{
							b3Warning("No URDF loaded yet, no client CMD_STEP_FORWARD_SIMULATION submitted\n");
						}
						break;
					}
				case CMD_SHUTDOWN:
					{
						m_wantsTermination = true;
						m_testBlock1->m_clientCommands[0].m_type =CMD_SHUTDOWN;
						m_testBlock1->m_numClientCommands++;
						m_serverLoadUrdfOK = false;
						b3Printf("client created CMD_SHUTDOWN\n");
						break;
					}
				default:
					{
						b3Error("unknown command requested\n");
					}
				}
			}
		}

}
void	PhysicsClient::stepSimulation(float deltaTime)
{
    
   // btAssert(m_testBlock1);

    if (m_testBlock1)
    {
		processServerCommands();
    
		if (!m_waitingForServer)
		{
			createClientCommand();
		}
	}
	

}


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClient(options.m_guiHelper);
}
