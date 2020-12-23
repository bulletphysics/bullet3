
#include "GraphicsServerExample.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btTransform.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "GraphicsSharedMemoryBlock.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryPublic.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
#include "../Utils/b3Clock.h"

#ifdef BT_ENABLE_CLSOCKET

#include "PassiveSocket.h"  // Include header for active socket object definition
#include <stdio.h>
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "RemoteGUIHelper.h"
#include "GraphicsSharedMemoryPublic.h"
#include "GraphicsSharedMemoryCommands.h"


bool gVerboseNetworkMessagesServer = true;

void MySerializeInt(unsigned int sz, unsigned char* output)
{
	unsigned int tmp = sz;
	output[0] = tmp & 255;
	tmp = tmp >> 8;
	output[1] = tmp & 255;
	tmp = tmp >> 8;
	output[2] = tmp & 255;
	tmp = tmp >> 8;
	output[3] = tmp & 255;
}

void submitStatus(CActiveSocket* pClient, GraphicsSharedMemoryStatus& serverStatus, b3AlignedObjectArray<char>& buffer)
{
	b3AlignedObjectArray<unsigned char> packetData;
	unsigned char* statBytes = (unsigned char*)&serverStatus;


	//create packetData with [int packetSizeInBytes, status, streamBytes)
	packetData.resize(4 + sizeof(GraphicsSharedMemoryStatus) + serverStatus.m_numDataStreamBytes);
	int sz = packetData.size();
	int curPos = 0;

	if (gVerboseNetworkMessagesServer)
	{
		printf("buffer.size = %d\n", buffer.size());
		printf("serverStatus packed size = %d\n", sz);
	}

	MySerializeInt(sz, &packetData[curPos]);
	curPos += 4;
	for (int i = 0; i < sizeof(GraphicsSharedMemoryStatus); i++)
	{
		packetData[i + curPos] = statBytes[i];
	}
	curPos += sizeof(GraphicsSharedMemoryStatus);
	if (gVerboseNetworkMessagesServer)
   	 printf("serverStatus.m_numDataStreamBytes=%d\n", serverStatus.m_numDataStreamBytes);
	for (int i = 0; i < serverStatus.m_numDataStreamBytes; i++)
	{
		packetData[i + curPos] = buffer[i];
	}

	pClient->Send(&packetData[0], packetData.size());
	if (gVerboseNetworkMessagesServer)
		printf("pClient->Send serverStatus: %d\n", packetData.size());
}


#endif //BT_ENABLE_CLSOCKET


#define MAX_GRAPHICS_SHARED_MEMORY_BLOCKS 1


struct TCPArgs
{
	TCPArgs()
		: m_cs(0),
		m_port(6667),
		m_numClientCommands(0),
		m_numServerCommands(0),
		m_cmdPtr(0)
	{
		m_dataSlots.resize(10);
	}
	
	void submitCommand()
	{
		m_cs->lock();
		m_serverStatus.m_type = GFX_CMD_CLIENT_COMMAND_FAILED;
		m_serverStatus.m_numDataStreamBytes = 0;
		btAssert(m_numServerCommands == m_numClientCommands);
		m_numClientCommands++;
		m_cs->unlock();
	}
	void processCommand()
	{
		m_cs->lock();
		btAssert(m_numServerCommands == (m_numClientCommands - 1));
		m_numServerCommands++;
		m_cs->unlock();
	}
	bool isCommandOutstanding()
	{
		m_cs->lock();
		bool result = m_numClientCommands > m_numServerCommands;
		m_cs->unlock();
		return result;
	}


	b3CriticalSection* m_cs;
	int m_port;
	b3AlignedObjectArray< b3AlignedObjectArray<unsigned char> > m_dataSlots;
	int m_numClientCommands;
	int m_numServerCommands;
	GraphicsSharedMemoryCommand* m_cmdPtr;
	GraphicsSharedMemoryStatus m_serverStatus;
};

struct TCPThreadLocalStorage
{
	int threadId;
};


enum TCPCommunicationEnums
{
	eTCPRequestTerminate = 11,
	eTCPIsUnInitialized,
	eTCPIsInitialized,
	eTCPInitializationFailed,
	eTCPHasTerminated
};

void TCPThreadFunc(void* userPtr, void* lsMemory)
{
	printf("TCPThreadFunc thread started\n");

	TCPArgs* args = (TCPArgs*)userPtr;
	//int workLeft = true;
	b3Clock clock;
	clock.reset();
	b3Clock sleepClock;
	bool init = true;
	if (init)
	{
		unsigned int cachedSharedParam = eTCPIsInitialized;

		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eTCPIsInitialized);
		args->m_cs->unlock();

		double deltaTimeInSeconds = 0;
		int numCmdSinceSleep1ms = 0;
		unsigned long long int prevTime = clock.getTimeMicroseconds();

#ifdef BT_ENABLE_CLSOCKET
		b3Clock clock;
		double timeOutInSeconds = 10;

		
		bool isPhysicsClientConnected = true;
		bool exitRequested = false;

		
		if (!isPhysicsClientConnected)
		{
			printf("TCP thread error connecting to shared memory. Machine needs a reboot?\n");
		}
		btAssert(isPhysicsClientConnected);
		
		printf("Starting TCP server using port %d\n", args->m_port);

		CPassiveSocket socket;
		CActiveSocket* pClient = NULL;

		//--------------------------------------------------------------------------
		// Initialize our socket object
		//--------------------------------------------------------------------------
		socket.Initialize();

		socket.Listen("localhost", args->m_port);

		socket.SetBlocking();

		int curNumErr = 0;

#endif

		do
		{
		  
			{
				b3Clock::usleep(0);
			}
			///////////////////////////////

#ifdef BT_ENABLE_CLSOCKET

			{
				b3Clock::usleep(0);

				if ((pClient = socket.Accept()) != NULL)
				{
					socket.SetReceiveTimeout(60, 0);// (1, 0);
					socket.SetSendTimeout(60, 0);

					b3AlignedObjectArray<char> bytesReceived;

					int clientPort = socket.GetClientPort();
					if (gVerboseNetworkMessagesServer)
						printf("connected from %s:%d\n", socket.GetClientAddr(), clientPort);

					if (pClient->Receive(4))
					{
						int clientKey = *(int*)pClient->GetData();

						if (clientKey == GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER)
						{
							printf("Client version OK %d\n", clientKey);
						}
						else
						{
							printf("Server version (%d) mismatches Client Version (%d)\n", GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER, clientKey);
							continue;
						}
					}

					//----------------------------------------------------------------------
					// Receive request from the client.
					//----------------------------------------------------------------------
					
					while (cachedSharedParam != eTCPRequestTerminate)
					{
						
						bool receivedData = false;

						int maxLen = 4 + sizeof(GraphicsSharedMemoryCommand) + GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE;
          
            
						if (pClient->Receive(maxLen))
						{
						  
							//heuristic to detect disconnected clients
							CSimpleSocket::CSocketError err = pClient->GetSocketError();

							if (err != CSimpleSocket::SocketSuccess || !pClient->IsSocketValid())
							{
								b3Clock::usleep(100);

								curNumErr++;

								if (curNumErr > 100)
								{
									///printf("TCP Connection error = %d, curNumErr = %d\n", (int)err, curNumErr);
									
								}
							}

							curNumErr = 0;
							char* msg2 = (char*)pClient->GetData();
							int numBytesRec2 = pClient->GetBytesReceived();
							if (gVerboseNetworkMessagesServer)
							  printf("numBytesRec2=%d\n", numBytesRec2);
							if (numBytesRec2 < 0)
							{
								numBytesRec2 = 0;
							}
							int curSize = bytesReceived.size();
							bytesReceived.resize(bytesReceived.size() + numBytesRec2);
							for (int i = 0; i < numBytesRec2; i++)
							{
								bytesReceived[curSize + i] = msg2[i];
							}

							if (bytesReceived.size() >= 4)
							{
								int numBytesRec = bytesReceived.size();
								if (numBytesRec >= 10)
								{
									if (strncmp(&bytesReceived[0], "disconnect", 10) == 0)
									{
										printf("Disconnect request received\n");
										bytesReceived.clear();
										break;
									}

										
								}

								if (gVerboseNetworkMessagesServer)
								{
									printf("received message length [%d]\n", numBytesRec);
								}

								receivedData = true;

								
								args->m_cmdPtr = 0;
								

								int type = *(int*)&bytesReceived[0];

								
								if (numBytesRec == sizeof(GraphicsSharedMemoryCommand))
								{
									args->m_cmdPtr = (GraphicsSharedMemoryCommand*)&bytesReceived[0];
								}
								
								if (args->m_cmdPtr)
								{
									
									b3AlignedObjectArray<char> buffer;
									buffer.resize(GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
									bool hasStatus = true;
									if (gVerboseNetworkMessagesServer)
										printf("processing command:");
									switch (args->m_cmdPtr->m_type)
									{
									case GFX_CMD_0:
									{
										int axis = args->m_cmdPtr->m_upAxisYCommand.m_enableUpAxisY ? 1 : 2;
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										bool done = false;
										//guiHelper.setUpAxis(axis);

										
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_0\n");
										break;
									}

									case GFX_CMD_SET_VISUALIZER_FLAG:
									{
										//disable single step rendering for GraphicsServer
										if (args->m_cmdPtr->m_visualizerFlagCommand.m_visualizerFlag == COV_ENABLE_SINGLE_STEP_RENDERING)
										{
											args->m_cmdPtr->m_visualizerFlagCommand.m_visualizerFlag = 0;
										}
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										//guiHelper.setVisualizerFlag(
										//	args->m_cmdPtr->m_visualizerFlagCommand.m_visualizerFlag,
										//	args->m_cmdPtr->m_visualizerFlagCommand.m_enable);
										//serverStatus.m_type = GFX_CMD_CLIENT_COMMAND_COMPLETED;
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_SET_VISUALIZER_FLAG\n");
										break;
									}
									case GFX_CMD_UPLOAD_DATA:
									{
										int slot = args->m_cmdPtr->m_uploadDataCommand.m_dataSlot;

										int numBytes = args->m_cmdPtr->m_uploadDataCommand.m_numBytes;


										submitStatus(pClient, args->m_serverStatus, buffer);

										//now receive numBytes 
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_UPLOAD_DATA receiving data\n");
										int received = 0;
										int offset = 0;
										args->m_dataSlots[slot].resize(numBytes);
										while (received < numBytes)
										{
											if (pClient->Receive(args->m_cmdPtr->m_uploadDataCommand.m_numBytes))
											{
												//heuristic to detect disconnected clients
												CSimpleSocket::CSocketError err = pClient->GetSocketError();
												if (err != CSimpleSocket::SocketSuccess || !pClient->IsSocketValid())
												{
													curNumErr++;
													//printf("TCP Connection error = %d, curNumErr = %d\n", (int)err, curNumErr);
												}
												char* msg2 = (char*)pClient->GetData();
												int numBytesRec2 = pClient->GetBytesReceived();
												if (gVerboseNetworkMessagesServer)
													printf("received %d bytes (total=%d)\n", numBytesRec2, received);
												
												for (int i = 0; i < numBytesRec2; i++)
												{
													args->m_dataSlots[slot][i+ offset] = msg2[i];
												}
												offset += numBytesRec2;
												received += numBytesRec2;
											}
										}
										if (gVerboseNetworkMessagesServer)
											printf("received all bytes!\n");
										args->m_serverStatus.m_type = GFX_CMD_CLIENT_COMMAND_COMPLETED;
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_UPLOAD_DATA\n");
										break;
									}
									case GFX_CMD_REGISTER_TEXTURE:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										//const unsigned char* texels = (const unsigned char*)&args->m_dataSlots[0][0];
										//args->m_serverStatus.m_registerTextureStatus.m_textureId = guiHelper.registerTexture(texels, args->m_cmdPtr->m_registerTextureCommand.m_width,
										//	args->m_cmdPtr->m_registerTextureCommand.m_height);
										//serverStatus.m_type = GFX_CMD_REGISTER_TEXTURE_COMPLETED;
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_REGISTER_TEXTURE\n");
										break;
									}
									case GFX_CMD_REGISTER_GRAPHICS_SHAPE:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_REGISTER_GRAPHICS_SHAPE\n");
										break;
									}
									case GFX_CMD_REGISTER_GRAPHICS_INSTANCE:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}

										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_REGISTER_GRAPHICS_INSTANCE\n");
										break;
									}
									case GFX_CMD_SYNCHRONIZE_TRANSFORMS:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_SYNCHRONIZE_TRANSFORMS\n");
										break;
									}
									case GFX_CMD_REMOVE_ALL_GRAPHICS_INSTANCES:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_REMOVE_ALL_GRAPHICS_INSTANCES\n");
										break;
									}
									case GFX_CMD_REMOVE_SINGLE_GRAPHICS_INSTANCE:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_REMOVE_SINGLE_GRAPHICS_INSTANCE\n");
										break;
									}
									case GFX_CMD_CHANGE_RGBA_COLOR:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_CHANGE_RGBA_COLOR\n");
										break;
									}
									case GFX_CMD_CHANGE_SCALING:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_CHANGE_SCALING\n");
										break;
									}

									case GFX_CMD_GET_CAMERA_INFO:
									{
										args->submitCommand();
										while (args->isCommandOutstanding())
										{
											clock.usleep(0);
										}
										//bool RemoteGUIHelper::getCameraInfo(int* width, int* height, 
										//	float viewMatrix[16], float projectionMatrix[16], 
										//	float camUp[3], float camForward[3], float hor[3], float vert[3], 
										//	float* yaw, float* pitch, float* camDist, float camTarget[3]) const
#if 0
										guiHelper.getCameraInfo(&serverStatus.m_getCameraInfoStatus.width,
											&serverStatus.m_getCameraInfoStatus.height,
											serverStatus.m_getCameraInfoStatus.viewMatrix,
											serverStatus.m_getCameraInfoStatus.projectionMatrix,
											serverStatus.m_getCameraInfoStatus.camUp,
											serverStatus.m_getCameraInfoStatus.camForward,
											serverStatus.m_getCameraInfoStatus.hor,
											serverStatus.m_getCameraInfoStatus.vert,
											&serverStatus.m_getCameraInfoStatus.yaw,
											&serverStatus.m_getCameraInfoStatus.pitch,
											&serverStatus.m_getCameraInfoStatus.camDist,
											serverStatus.m_getCameraInfoStatus.camTarget);
										serverStatus.m_type = GFX_CMD_GET_CAMERA_INFO_COMPLETED;
#endif
										if (gVerboseNetworkMessagesServer)
											printf("GFX_CMD_GET_CAMERA_INFO\n");
										break;
									}

									case GFX_CMD_INVALID:
									case GFX_CMD_MAX_CLIENT_COMMANDS:
									default:
									{
										printf("UNKNOWN COMMAND!\n");
										btAssert(0);
										hasStatus = false;
									}
									}


									double startTimeSeconds = clock.getTimeInSeconds();
									double curTimeSeconds = clock.getTimeInSeconds();

									if (gVerboseNetworkMessagesServer)
									{
										//printf("buffer.size = %d\n", buffer.size());
										printf("serverStatus.m_numDataStreamBytes = %d\n", args->m_serverStatus.m_numDataStreamBytes);
									}
									if (hasStatus)
									{
										submitStatus(pClient, args->m_serverStatus, buffer);
									}
                         
									bytesReceived.clear();
								}
								else
								{
									//likely an incomplete packet, let's append more bytes
									//printf("received packet with unknown contents\n");
								}
							}
						}
						if (!receivedData)
						{
							//printf("Didn't receive data.\n");
						}
					}

					printf("Disconnecting client.\n");
					pClient->Close();
					delete pClient;
				}
			}

#endif //BT_ENABLE_CLSOCKET


			///////////////////////////////
			args->m_cs->lock();
			cachedSharedParam = args->m_cs->getSharedParam(0);
			args->m_cs->unlock();

		} while (cachedSharedParam != eTCPRequestTerminate);
#ifdef BT_ENABLE_CLSOCKET	
		socket.Close();
		socket.Shutdown(CSimpleSocket::Both);
#endif
	}
	else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eTCPInitializationFailed);
		args->m_cs->unlock();
	}
		
	printf("TCPThreadFunc thread exit\n");
	//do nothing
}

void* TCPlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new TCPThreadLocalStorage;
}

void TCPlsMemoryReleaseFunc(void* ptr)
{
	TCPThreadLocalStorage* p = (TCPThreadLocalStorage*)ptr;
	delete p;
}


#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"

b3ThreadSupportInterface* createTCPThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("TCPThreads",
		TCPThreadFunc,
		TCPlsMemoryFunc,
		TCPlsMemoryReleaseFunc,
		numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;
}

#elif defined(_WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createTCPThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("TCPThreads", TCPThreadFunc, TCPlsMemoryFunc, TCPlsMemoryReleaseFunc, numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;
}
#endif


class GraphicsServerExample : public CommonExampleInterface
{
	CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	
	bool m_verboseOutput;

	float m_x;
	float m_y;
	float m_z;
	
	b3ThreadSupportInterface* m_threadSupport;
	TCPArgs m_args;
	
public:
	GraphicsServerExample(GUIHelperInterface* guiHelper)
		: m_guiHelper(guiHelper),
		  m_x(0),
		  m_y(0),
		  m_z(0)
	{
		m_verboseOutput = true;
		
		m_app = guiHelper->getAppInterface();
		m_app->setUpAxis(2);

		m_threadSupport = createTCPThreadSupport(1);
		m_args.m_cs = m_threadSupport->createCriticalSection();
		m_args.m_cs->setSharedParam(0, eTCPIsUnInitialized);
		m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)&this->m_args, 0);

		bool isUninitialized = true;

		while (isUninitialized)
		{
			m_args.m_cs->lock();
			isUninitialized = (m_args.m_cs->getSharedParam(0) == eTCPIsUnInitialized);
			m_args.m_cs->unlock();
#ifdef _WIN32
			b3Clock::usleep(1000);
#endif
	}

	}
	virtual ~GraphicsServerExample()
	{
		
		m_args.m_cs->setSharedParam(0, eTCPRequestTerminate);

		int numActiveThreads = 1;
		while (numActiveThreads)
		{
			int arg0, arg1;
			if (m_threadSupport->isTaskCompleted(&arg0, &arg1, 0))
			{
				numActiveThreads--;
				printf("numActiveThreads = %d\n", numActiveThreads);
			}
			else
			{
				b3Clock::usleep(0);
			}
		};

		m_threadSupport->deleteCriticalSection(m_args.m_cs);

		delete m_threadSupport;
		m_threadSupport = 0;

	}

	virtual void initPhysics()
	{
	}
	virtual void exitPhysics()
	{
	}
	
	void submitServerStatus(GraphicsSharedMemoryStatus& status, int blockIndex)
	{
	}

	bool processCommand(const struct GraphicsSharedMemoryCommand& clientCmd, struct GraphicsSharedMemoryStatus& serverStatusOut)
	{
		//printf("processed command of type:%d\n", clientCmd.m_type);
		B3_PROFILE("processCommand");
		switch (clientCmd.m_type)
		{
			case GFX_CMD_0:
			{
				//either Y or Z can be up axis
				int upAxis = (clientCmd.m_upAxisYCommand.m_enableUpAxisY) ? 1 : 2;
				m_guiHelper->setUpAxis(upAxis);
				serverStatusOut.m_type = GFX_CMD_CLIENT_COMMAND_COMPLETED;
				m_args.processCommand();
				break;
			}
			case GFX_CMD_SET_VISUALIZER_FLAG:
			{
				if ((clientCmd.m_visualizerFlagCommand.m_visualizerFlag != COV_ENABLE_RENDERING) &&
					(clientCmd.m_visualizerFlagCommand.m_visualizerFlag != COV_ENABLE_SINGLE_STEP_RENDERING))
				{
					//printf("clientCmd.m_visualizerFlag.m_visualizerFlag: %d, clientCmd.m_visualizerFlag.m_enable %d\n",
					//	clientCmd.m_visualizerFlagCommand.m_visualizerFlag, clientCmd.m_visualizerFlagCommand.m_enable);

					this->m_guiHelper->setVisualizerFlag(clientCmd.m_visualizerFlagCommand.m_visualizerFlag, clientCmd.m_visualizerFlagCommand.m_enable);
				}
				m_args.processCommand();
				break;

			}

			case GFX_CMD_UPLOAD_DATA:
			{
#if 0
				//printf("uploadData command: curSize=%d, offset=%d, slot=%d", clientCmd.m_uploadDataCommand.m_numBytes, clientCmd.m_uploadDataCommand.m_dataOffset, clientCmd.m_uploadDataCommand.m_dataSlot);
				int dataSlot = clientCmd.m_uploadDataCommand.m_dataSlot;
				int dataOffset = clientCmd.m_uploadDataCommand.m_dataOffset;
				m_dataSlots.resize(dataSlot + 1);
				btAssert(m_dataSlots[dataSlot].size() >= dataOffset);
				m_dataSlots[dataSlot].resize(clientCmd.m_uploadDataCommand.m_numBytes + clientCmd.m_uploadDataCommand.m_dataOffset);
				for (int i = 0; i < clientCmd.m_uploadDataCommand.m_numBytes; i++)
				{
					m_dataSlots[dataSlot][dataOffset + i] = bufferServerToClient[i];
				}
#endif
				break;
			}
			case GFX_CMD_REGISTER_TEXTURE:
			{
				
				int dataSlot = 0;
				int sizeData = m_args.m_dataSlots[dataSlot].size();
				btAssert(sizeData > 0);
				serverStatusOut.m_type = GFX_CMD_REGISTER_TEXTURE_FAILED;
				if (sizeData)
				{
					unsigned char* texels = &m_args.m_dataSlots[dataSlot][0];
					int textureId = this->m_guiHelper->registerTexture(texels, clientCmd.m_registerTextureCommand.m_width, clientCmd.m_registerTextureCommand.m_height);
					serverStatusOut.m_type = GFX_CMD_REGISTER_TEXTURE_COMPLETED;
					serverStatusOut.m_registerTextureStatus.m_textureId = textureId;
				}
				
				m_args.processCommand();
				break;
			}

			case GFX_CMD_REGISTER_GRAPHICS_SHAPE:
			{
				int verticesSlot = 0;
				int indicesSlot = 1;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_SHAPE_FAILED;
				const float* vertices = (const float*)&m_args.m_dataSlots[verticesSlot][0];
				const int* indices = (const int*)&m_args.m_dataSlots[indicesSlot][0];
				int numVertices = clientCmd.m_registerGraphicsShapeCommand.m_numVertices;
				int numIndices = clientCmd.m_registerGraphicsShapeCommand.m_numIndices;
				int primitiveType = clientCmd.m_registerGraphicsShapeCommand.m_primitiveType;
				int textureId = clientCmd.m_registerGraphicsShapeCommand.m_textureId;
				int shapeId = this->m_guiHelper->registerGraphicsShape(vertices, numVertices, indices, numIndices, primitiveType, textureId);
				serverStatusOut.m_registerGraphicsShapeStatus.m_shapeId = shapeId;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_SHAPE_COMPLETED;
				m_args.processCommand();
				break;
			}

			case GFX_CMD_REGISTER_GRAPHICS_INSTANCE:
			{
				int graphicsInstanceId = m_guiHelper->registerGraphicsInstance(clientCmd.m_registerGraphicsInstanceCommand.m_shapeIndex,
					clientCmd.m_registerGraphicsInstanceCommand.m_position,
					clientCmd.m_registerGraphicsInstanceCommand.m_quaternion,
					clientCmd.m_registerGraphicsInstanceCommand.m_color,
					clientCmd.m_registerGraphicsInstanceCommand.m_scaling);
				serverStatusOut.m_registerGraphicsInstanceStatus.m_graphicsInstanceId = graphicsInstanceId;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_INSTANCE_COMPLETED;
				m_args.processCommand();
				break;
			}
			case GFX_CMD_SYNCHRONIZE_TRANSFORMS:
			{
				GUISyncPosition* positions = (GUISyncPosition*)&m_args.m_dataSlots[0][0];
				for (int i = 0; i < clientCmd.m_syncTransformsCommand.m_numPositions; i++)
				{
					m_app->m_renderer->writeSingleInstanceTransformToCPU(positions[i].m_pos, positions[i].m_orn, positions[i].m_graphicsInstanceId);
				}
				m_args.processCommand();
				break;
			}
			case GFX_CMD_REMOVE_ALL_GRAPHICS_INSTANCES:
			{
				m_guiHelper->removeAllGraphicsInstances();
				m_args.processCommand();
				break;
			}
			case GFX_CMD_REMOVE_SINGLE_GRAPHICS_INSTANCE:
			{
				m_app->m_renderer->removeGraphicsInstance(clientCmd.m_removeGraphicsInstanceCommand.m_graphicsUid);
				m_args.processCommand();
				break;
			}
			case GFX_CMD_CHANGE_RGBA_COLOR:
			{
				m_guiHelper->changeRGBAColor(clientCmd.m_changeRGBAColorCommand.m_graphicsUid, clientCmd.m_changeRGBAColorCommand.m_rgbaColor);
				m_args.processCommand();
				break;
			}

			case GFX_CMD_CHANGE_SCALING:
			{
				m_guiHelper->changeScaling(clientCmd.m_changeScalingCommand.m_graphicsUid, clientCmd.m_changeScalingCommand.m_scaling);
				m_args.processCommand();
				break;
			}

			case GFX_CMD_GET_CAMERA_INFO:
			{
				serverStatusOut.m_type = GFX_CMD_GET_CAMERA_INFO_FAILED;
				
				if (m_guiHelper->getCameraInfo(
					&serverStatusOut.m_getCameraInfoStatus.width,
					&serverStatusOut.m_getCameraInfoStatus.height,
					serverStatusOut.m_getCameraInfoStatus.viewMatrix,
					serverStatusOut.m_getCameraInfoStatus.projectionMatrix,
					serverStatusOut.m_getCameraInfoStatus.camUp,
					serverStatusOut.m_getCameraInfoStatus.camForward,
					serverStatusOut.m_getCameraInfoStatus.hor,
					serverStatusOut.m_getCameraInfoStatus.vert,
					&serverStatusOut.m_getCameraInfoStatus.yaw,
					&serverStatusOut.m_getCameraInfoStatus.pitch,
					&serverStatusOut.m_getCameraInfoStatus.camDist,
					serverStatusOut.m_getCameraInfoStatus.camTarget))
				{
					serverStatusOut.m_type = GFX_CMD_GET_CAMERA_INFO_COMPLETED;
				}
				m_args.processCommand();

				break;
			}
			default:
			{
				printf("unsupported command:%d\n", clientCmd.m_type);
			}
		}
		return true;
	}

	void processClientCommands()
	{
		int timeStamp = 0;
		bool hasStatus = false;
		if (m_args.isCommandOutstanding())
		{
			processCommand(*m_args.m_cmdPtr, m_args.m_serverStatus);
		}
		//serverStatusOut.m_type = GFX_CMD_CLIENT_COMMAND_FAILED;
		//bool hasStatus = processCommand(clientCmd, serverStatusOut);
		if (hasStatus)
		{
			//submitServerStatus(serverStatusOut);
		}
	}

	virtual void stepSimulation(float deltaTime)
	{
		B3_PROFILE("stepSimulation");
		processClientCommands();
		m_x += 0.01f;
		m_y += 0.01f;
		m_z += 0.01f;
	}


	virtual void renderScene()
	{
		B3_PROFILE("renderScene");
		{

			B3_PROFILE("writeTransforms");
			m_app->m_renderer->writeTransforms();
		}
		{
			B3_PROFILE("m_renderer->renderScene");
			m_app->m_renderer->renderScene();
		}
		
	}

	

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
		
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool keyboardCallback(int key, int state)
	{
		return false;
	}

	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -32;
		float yaw = 136;
		float targetPos[3] = {0, 0, 0};
		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
	
	
};







CommonExampleInterface* GraphicsServerCreateFuncBullet(struct CommonExampleOptions& options)
{
	return new GraphicsServerExample(options.m_guiHelper);
}
