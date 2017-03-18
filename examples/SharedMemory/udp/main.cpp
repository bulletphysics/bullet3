/* server.cpp */
#include <stdio.h>
#include <enet/enet.h>
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
	
bool gVerboseNetworkMessagesServer = false;

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

	int port = 1234;
	if (parseArgs.GetCmdLineArgument("port",port))
	{
		printf("Using UDP port %d\n", port);
	}

	gVerboseNetworkMessagesServer = parseArgs.CheckCmdLineFlag("verbose");

#ifndef NO_SHARED_MEMORY
	int key = 0;
	if (parseArgs.GetCmdLineArgument("sharedMemoryKey",key))
	{
		sm->setSharedMemoryKey(key);
	}
#endif//NO_SHARED_MEMORY

//	PhysicsDirect* sm = new PhysicsDirect(sdk);

	//PhysicsClientSharedMemory* sm = new PhysicsClientSharedMemory();

	bool isPhysicsClientConnected = sm->connect();

	if (isPhysicsClientConnected)
	{
		
		ENetAddress address;
		ENetHost *server;
		ENetEvent event;
		int serviceResult;

		puts("Starting server");

		if (enet_initialize() != 0)
		{
			puts("Error initialising enet");
			exit(EXIT_FAILURE);
		}


		/* Bind the server to the default localhost.     */
		/* A specific host address can be specified by   */
		/* enet_address_set_host (& address, "x.x.x.x"); */
		address.host = ENET_HOST_ANY;
		/* Bind the server to port 1234. */
		address.port = port;

		server = enet_host_create(&address,
			32,   /* number of clients */
			2,    /* number of channels */
			0,    /* Any incoming bandwith */
			0);   /* Any outgoing bandwith */

		if (server == NULL)
		{
			puts("Could not create server host");
			exit(EXIT_FAILURE);
		}


		while (true)
		{
			b3Clock::usleep(0);

			serviceResult = 1;

			/* Keep doing host_service until no events are left */
			while (serviceResult > 0)
			{
				/* Wait up to 1000 milliseconds for an event. */
				serviceResult = enet_host_service(server, &event, 0);
				if (serviceResult > 0)
				{

					switch (event.type)
					{
                        case ENET_EVENT_TYPE_CONNECT:
                        {
						printf("A new client connected from %x:%u.\n",
							event.peer->address.host,
							event.peer->address.port);

						/* Store any relevant client information here. */
						event.peer->data = (void*)"Client information";

						break;
                        }
					case ENET_EVENT_TYPE_RECEIVE:
                        {
						if (gVerboseNetworkMessagesServer)
						{
							int dataLen = (int)event.packet->dataLength;

							printf("A packet of length %u containing '%s' was "
								"received from %s on channel %u.\n",
								dataLen,
								event.packet->data,
								event.peer->data,
								event.channelID);
						}
						SharedMemoryCommand cmd;

						SharedMemoryCommand* cmdPtr = 0;

						//performance test
						if (event.packet->dataLength == sizeof(int))
						{
							cmdPtr = &cmd;
							cmd.m_type = *(int*)event.packet->data;
						}

						if (event.packet->dataLength == sizeof(SharedMemoryCommand))
						{
							cmdPtr = (SharedMemoryCommand*)event.packet->data;
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
									ENetPacket *packet = enet_packet_create(&packetData[0], packetData.size(), ENET_PACKET_FLAG_RELIABLE);
									enet_peer_send(event.peer, 0, packet);
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

									ENetPacket *packet = enet_packet_create(&packetData[0], packetData.size(), ENET_PACKET_FLAG_RELIABLE);
									enet_peer_send(event.peer, 0, packet);
									//enet_host_broadcast(server, 0, packet);
								}
							}
						}
						else
						{
							printf("received packet with unknown contents\n");
						}


						/* Tell all clients about this message */
						//enet_host_broadcast(server, 0, event.packet);

						break;
                        }
					case ENET_EVENT_TYPE_DISCONNECT:
                        {
						printf("%s disconnected.\n", event.peer->data);

						/* Reset the peer's client information. */

						event.peer->data = NULL;

						break;
                        }
                    default:
                        {
                        
                        }
					}
				}
				else if (serviceResult > 0)
				{
					puts("Error with servicing the server");
					exit(EXIT_FAILURE);
				}
			}

		}

		enet_host_destroy(server);
		enet_deinitialize();
	}
	delete sm;

	return 0;

}
