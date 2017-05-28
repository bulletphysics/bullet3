#include "PassiveSocket.h"       // Include header for active socket object definition

#include <stdio.h>
#include "../../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3CommandLineArgs.h"

#ifdef NO_SHARED_MEMORY
#include "PhysicsServerCommandProcessor.h"
typedef PhysicsServerCommandProcessor MyCommandProcessor;
#else
#include "SharedMemoryCommandProcessor.h"
typedef SharedMemoryCommandProcessor MyCommandProcessor;
#endif //NO_SHARED_MEMORY

#include "SharedMemoryCommands.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "PhysicsServerCommandProcessor.h"
#include "../Utils/b3Clock.h"


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

int main(int argc, char *argv[])
{
    
    b3CommandLineArgs parseArgs(argc,argv);
    b3Clock clock;
    double timeOutInSeconds = 10;
    
    DummyGUIHelper guiHelper;
    MyCommandProcessor* sm = new MyCommandProcessor;
    sm->setGuiHelper(&guiHelper);
    
    int port = 6667;
    parseArgs.GetCmdLineArgument("port",port);
	
    gVerboseNetworkMessagesServer = parseArgs.CheckCmdLineFlag("verbose");
    
#ifndef NO_SHARED_MEMORY
    int key = 0;
    if (parseArgs.GetCmdLineArgument("sharedMemoryKey",key))
    {
        sm->setSharedMemoryKey(key);
    }
#endif//NO_SHARED_MEMORY
    
    bool isPhysicsClientConnected = sm->connect();
    bool exitRequested = false;
    
    if (isPhysicsClientConnected)
    {
    
		printf("Starting TCP server using port %d\n", port);
		
        CPassiveSocket socket;
        CActiveSocket *pClient = NULL;
        
        //--------------------------------------------------------------------------
        // Initialize our socket object
        //--------------------------------------------------------------------------
        socket.Initialize();
        
        socket.Listen("localhost", port);
        //socket.SetBlocking();
        
        int curNumErr = 0;
        
        
        while (!exitRequested)
        {
            b3Clock::usleep(0);

            if ((pClient = socket.Accept()) != NULL)
            {
                b3AlignedObjectArray<char> bytesReceived;

                int clientPort = socket.GetClientPort();
                printf("connected from %s:%d\n", socket.GetClientAddr(),clientPort);

                if (pClient->Receive(4))
                {
					int clientKey = *(int*)pClient->GetData();

					if (clientKey==SHARED_MEMORY_MAGIC_NUMBER)
					{
						printf("Client version OK %d\n", clientKey);
					} else
					{
						printf("Server version (%d) mismatches Client Version (%d)\n", SHARED_MEMORY_MAGIC_NUMBER,clientKey);
						continue;
					}
				}
                
                //----------------------------------------------------------------------
                // Receive request from the client.
                //----------------------------------------------------------------------
                while (1)
                {
                    //printf("try receive\n");
                    bool receivedData = false;
                    
					int maxLen = 4 + sizeof(SharedMemoryStatus)+SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE;
       

                    
                    if (pClient->Receive(maxLen))
                    {

						//heuristic to detect disconnected clients
						CSimpleSocket::CSocketError err = pClient->GetSocketError();
					
						if (err != CSimpleSocket::SocketSuccess || !pClient->IsSocketValid())
						{
							b3Clock::usleep(100);
                        
							curNumErr++;
                       
							if (curNumErr>100)
							{
								printf("TCP Connection error = %d, curNumErr = %d\n", (int)err, curNumErr);
                            
								break;
							}
						}

                        curNumErr = 0;
                        char* msg2 = (char*) pClient->GetData();
                        int numBytesRec2 = pClient->GetBytesReceived();

						int curSize = bytesReceived.size();
						bytesReceived.resize(bytesReceived.size()+numBytesRec2);
						for (int i=0;i<numBytesRec2;i++)
						{
							bytesReceived[curSize+i] = msg2[i];
						}

						if (bytesReceived.size() >= 4)
						{
							int numBytesRec = bytesReceived.size();
							if (numBytesRec>=10)
							{
								if (strncmp(&bytesReceived[0],"disconnect",10)==0)
								{
									printf("Disconnect request received\n");
									bytesReceived.clear();
									break;
								}

								if (strncmp(&bytesReceived[0],"terminateserver",10)==0)
								{
									printf("Terminate server request received\n");
									exitRequested = true;
									bytesReceived.clear();
									break;
								}
							}

							if (gVerboseNetworkMessagesServer)
							{
								printf("received message length [%d]\n",numBytesRec);
							}
                        
							receivedData = true;
                        
							SharedMemoryCommand cmd;
                        
							SharedMemoryCommand* cmdPtr = 0;
                        
							//performance test
							if (numBytesRec == sizeof(int))
							{
								cmdPtr = &cmd;
								cmd.m_type = *(int*)&bytesReceived[0];
							}
                        
							if (numBytesRec == sizeof(SharedMemoryCommand))
							{
								cmdPtr = (SharedMemoryCommand*)&bytesReceived[0];
							}
							if (cmdPtr)
							{
								SharedMemoryStatus serverStatus;
								b3AlignedObjectArray<char> buffer;
								buffer.resize(SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
                            
								bool hasStatus = sm->processCommand(*cmdPtr,serverStatus, &buffer[0], buffer.size());
                            
								double startTimeSeconds = clock.getTimeInSeconds();
								double curTimeSeconds  = clock.getTimeInSeconds();
                            
								while ((!hasStatus) && ((curTimeSeconds - startTimeSeconds) <timeOutInSeconds))
								{
									hasStatus = sm->receiveStatus(serverStatus, &buffer[0], buffer.size());
									curTimeSeconds = clock.getTimeInSeconds();
								}
								if (gVerboseNetworkMessagesServer)
								{
									printf("buffer.size = %d\n", buffer.size());
									printf("serverStatus.m_numDataStreamBytes = %d\n", serverStatus.m_numDataStreamBytes);
								}
								if (hasStatus)
								{
									b3AlignedObjectArray<unsigned char> packetData;
									unsigned char* statBytes = (unsigned char*)&serverStatus;
                                
									if (cmdPtr->m_type == CMD_STEP_FORWARD_SIMULATION)
									{
										packetData.resize(4 + sizeof(int));
										int sz = packetData.size();
										int curPos = 0;
                                    
										MySerializeInt(sz, &packetData[curPos]);
										curPos += 4;
										for (int i = 0; i < sizeof(int); i++)
										{
											packetData[i + curPos] = statBytes[i];
										}
										curPos += sizeof(int);
                                    
										pClient->Send( &packetData[0], packetData.size() );
                                   
									}
									else
									{
										//create packetData with [int packetSizeInBytes, status, streamBytes)
										packetData.resize(4 + sizeof(SharedMemoryStatus) + serverStatus.m_numDataStreamBytes);
										int sz = packetData.size();
										int curPos = 0;
                                    
										MySerializeInt(sz, &packetData[curPos]);
										curPos += 4;
										for (int i = 0; i < sizeof(SharedMemoryStatus); i++)
										{
											packetData[i + curPos] = statBytes[i];
										}
										curPos += sizeof(SharedMemoryStatus);
                                    
										for (int i = 0; i < serverStatus.m_numDataStreamBytes; i++)
										{
											packetData[i + curPos] = buffer[i];
										}
                                    
										pClient->Send( &packetData[0], packetData.size() );
									}
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
        
        socket.Close();
        socket.Shutdown(CSimpleSocket::Both);
    } else
	{
		printf("Error: cannot connect to shared memory physics server.");
	}
    
    delete sm;

    return 0;
}

