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

#define MAX_PACKET 4096

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
    if (parseArgs.GetCmdLineArgument("port",port))
    {
        printf("Using TCP port %d\n", port);
    }
    
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
        
        CPassiveSocket socket;
        CActiveSocket *pClient = NULL;
        
        //--------------------------------------------------------------------------
        // Initialize our socket object
        //--------------------------------------------------------------------------
        socket.Initialize();
        
        socket.Listen("localhost", port);
        
        while (!exitRequested)
        {
            b3Clock::usleep(0);

            if ((pClient = socket.Accept()) != NULL)
            {
                int clientPort = socket.GetClientPort();
                printf("connected from %s:%d\n", socket.GetClientAddr(),clientPort);
                
                //----------------------------------------------------------------------
                // Receive request from the client.
                //----------------------------------------------------------------------
                while (1)
                {
                    //printf("try receive\n");
                    bool receivedData = false;
                    
                    if (pClient->Receive(MAX_PACKET))
                    {
                        char* msg = (char*) pClient->GetData();
                        int numBytesRec = pClient->GetBytesReceived();
                        if (gVerboseNetworkMessagesServer)
                        {
                            printf("received message length [%d]\n",numBytesRec);
                        }
                        
                        receivedData = true;
                        
                        if (strncmp(msg,"stop",4)==0)
                        {
                            printf("Stop request received\n");
                            exitRequested = true;
                            break;
                        }
                        
                        
                        SharedMemoryCommand cmd;
                        
                        SharedMemoryCommand* cmdPtr = 0;
                        
                        //performance test
                        if (numBytesRec == sizeof(int))
                        {
                            cmdPtr = &cmd;
                            cmd.m_type = *(int*)pClient->GetData();
                        }
                        
                        if (numBytesRec == sizeof(SharedMemoryCommand))
                        {
                            cmdPtr = (SharedMemoryCommand*)pClient->GetData();
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
                        }
                        else
                        {
                            printf("received packet with unknown contents\n");
                        }
                    }
                    if (!receivedData)
                    {
                        printf("Didn't receive data.\n");
                        break;
                    } 
                }
                printf("Disconnecting client.\n");
                pClient->Close();
                delete pClient;
                
            }
        }
        
        socket.Close();
        socket.Shutdown(CSimpleSocket::Both);
    }
    
    delete sm;

    return 0;
}

