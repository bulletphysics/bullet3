#include "PhysicsClientUDP.h"
#include <enet/enet.h>
#include <stdio.h>
#include <string.h>
#include "../Utils/b3Clock.h"
#include "PhysicsClient.h"
//#include "LinearMath/btVector3.h"
#include "SharedMemoryCommands.h"
#include <string>
#include "Bullet3Common/b3Logging.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
void UDPThreadFunc(void* userPtr, void* lsMemory);
void* UDPlsMemoryFunc();
void UDPlsMemoryReleaseFunc(void* ptr);

bool gVerboseNetworkMessagesClient = false;

#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"

b3ThreadSupportInterface* createUDPThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("UDPThread",
																  UDPThreadFunc,
																  UDPlsMemoryFunc,
																  UDPlsMemoryReleaseFunc,
																  numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;
}

#elif defined(_WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createUDPThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("UDPThread", UDPThreadFunc, UDPlsMemoryFunc, UDPlsMemoryReleaseFunc, numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;
}
#endif

struct UDPThreadLocalStorage
{
	int threadId;
};

unsigned int b3DeserializeInt(const unsigned char* input)
{
	unsigned int tmp = (input[3] << 24) + (input[2] << 16) + (input[1] << 8) + input[0];
	return tmp;
}

struct UdpNetworkedInternalData
{
	ENetHost* m_client;
	ENetAddress m_address;
	ENetPeer* m_peer;
	ENetEvent m_event;
	bool m_isConnected;

	b3ThreadSupportInterface* m_threadSupport;

	b3CriticalSection* m_cs;

	UdpNetworkedInternalData* m_udpInternalData;

	SharedMemoryCommand m_clientCmd;
	bool m_hasCommand;

	bool m_hasStatus;
	SharedMemoryStatus m_lastStatus;
	b3AlignedObjectArray<char> m_stream;

	std::string m_hostName;
	int m_port;
	double m_timeOutInSeconds;

	UdpNetworkedInternalData()
		: m_client(0),
		  m_peer(0),
		  m_isConnected(false),
		  m_threadSupport(0),
		  m_hasCommand(false),
		  m_hasStatus(false),
		  m_timeOutInSeconds(60)
	{
	}

	bool connectUDP()
	{
		if (m_isConnected)
			return true;

		if (enet_initialize() != 0)
		{
			fprintf(stderr, "Error initialising enet");

			exit(EXIT_FAILURE);
		}

		m_client = enet_host_create(NULL,       /* create a client host */
									1,          /* number of clients */
									2,          /* number of channels */
									57600 / 8,  /* incoming bandwith */
									14400 / 8); /* outgoing bandwith */

		if (m_client == NULL)
		{
			fprintf(stderr, "Could not create client host");
			return false;
		}

		enet_address_set_host(&m_address, m_hostName.c_str());
		m_address.port = m_port;

		m_peer = enet_host_connect(m_client,
								   &m_address, /* address to connect to */
								   2,          /* number of channels */
								   0);         /* user data supplied to
						 the receiving host */

		if (m_peer == NULL)
		{
			fprintf(stderr,
					"No available peers for initiating an ENet "
					"connection.\n");
			return false;
		}

		/* Try to connect to server within 5 seconds */
		if (enet_host_service(m_client, &m_event, 5000) > 0 &&
			m_event.type == ENET_EVENT_TYPE_CONNECT)
		{
			puts("Connection to server succeeded.");
		}
		else
		{
			/* Either the 5 seconds are up or a disconnect event was */
			/* received. Reset the peer in the event the 5 seconds   */
			/* had run out without any significant event.            */
			enet_peer_reset(m_peer);

			fprintf(stderr, "Connection to server failed.");
			return false;
		}

		int serviceResult = enet_host_service(m_client, &m_event, 0);

		if (serviceResult > 0)
		{
			switch (m_event.type)
			{
				case ENET_EVENT_TYPE_CONNECT:
					printf("A new client connected from %x:%u.\n",
						   m_event.peer->address.host,
						   m_event.peer->address.port);
					m_event.peer->data = (void*)"New User";
					break;

				case ENET_EVENT_TYPE_RECEIVE:

					if (gVerboseNetworkMessagesClient)
					{
						printf(
							"A packet of length %lu containing '%s' was "
							"received from %s on channel %u.\n",
							m_event.packet->dataLength,
							(char*)m_event.packet->data,
							(char*)m_event.peer->data,
							m_event.channelID);
					}
					/* Clean up the packet now that we're done using it.
				> */
					enet_packet_destroy(m_event.packet);

					break;

				case ENET_EVENT_TYPE_DISCONNECT:
					printf("%s disconnected.\n", (char*)m_event.peer->data);

					break;
				default:
				{
					printf("unknown event type: %d.\n", m_event.type);
				}
			}
		}
		else if (serviceResult > 0)
		{
			puts("Error with servicing the client");
			return false;
		}

		m_isConnected = true;
		return m_isConnected;
	}

	bool checkData()
	{
		bool hasStatus = false;

		int serviceResult = enet_host_service(m_client, &m_event, 0);

		if (serviceResult > 0)
		{
			switch (m_event.type)
			{
				case ENET_EVENT_TYPE_CONNECT:
					printf("A new client connected from %x:%u.\n",
						   m_event.peer->address.host,
						   m_event.peer->address.port);

					m_event.peer->data = (void*)"New User";
					break;

				case ENET_EVENT_TYPE_RECEIVE:
				{
					if (gVerboseNetworkMessagesClient)
					{
						printf(
							"A packet of length %lu containing '%s' was "
							"received from %s on channel %u.\n",
							m_event.packet->dataLength,
							(char*)m_event.packet->data,
							(char*)m_event.peer->data,
							m_event.channelID);
					}

					int packetSizeInBytes = b3DeserializeInt(m_event.packet->data);

					if (packetSizeInBytes == m_event.packet->dataLength)
					{
						SharedMemoryStatus* statPtr = (SharedMemoryStatus*)&m_event.packet->data[4];
						if (statPtr->m_type == CMD_STEP_FORWARD_SIMULATION_COMPLETED)
						{
							SharedMemoryStatus dummy;
							dummy.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;
							m_lastStatus = dummy;
							m_stream.resize(0);
						}
						else
						{
							m_lastStatus = *statPtr;
							int streamOffsetInBytes = 4 + sizeof(SharedMemoryStatus);
							int numStreamBytes = packetSizeInBytes - streamOffsetInBytes;
							m_stream.resize(numStreamBytes);
							for (int i = 0; i < numStreamBytes; i++)
							{
								m_stream[i] = m_event.packet->data[i + streamOffsetInBytes];
							}
						}
					}
					else
					{
						printf("unknown status message received\n");
					}
					enet_packet_destroy(m_event.packet);
					hasStatus = true;
					break;
				}
				case ENET_EVENT_TYPE_DISCONNECT:
				{
					printf("%s disconnected.\n", (char*)m_event.peer->data);

					break;
				}
				default:
				{
					printf("unknown event type: %d.\n", m_event.type);
				}
			}
		}
		else if (serviceResult > 0)
		{
			puts("Error with servicing the client");
		}

		return hasStatus;
	}
};

enum UDPThreadEnums
{
	eUDPRequestTerminate = 13,
	eUDPIsUnInitialized,
	eUDPIsInitialized,
	eUDPInitializationFailed,
	eUDPHasTerminated
};

enum UDPCommandEnums
{
	eUDPIdle = 13,
	eUDP_ConnectRequest,
	eUDP_Connected,
	eUDP_ConnectionFailed,
	eUDP_DisconnectRequest,
	eUDP_Disconnected,

};

void UDPThreadFunc(void* userPtr, void* lsMemory)
{
	printf("UDPThreadFunc thread started\n");
	//	UDPThreadLocalStorage* localStorage = (UDPThreadLocalStorage*)lsMemory;

	UdpNetworkedInternalData* args = (UdpNetworkedInternalData*)userPtr;
	//	int workLeft = true;
	b3Clock clock;
	clock.reset();
	bool init = true;
	if (init)
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eUDPIsInitialized);
		args->m_cs->unlock();

		double deltaTimeInSeconds = 0;

		do
		{
			b3Clock::usleep(0);

			deltaTimeInSeconds += double(clock.getTimeMicroseconds()) / 1000000.;

			{
				clock.reset();
				deltaTimeInSeconds = 0.f;
				switch (args->m_cs->getSharedParam(1))
				{
					case eUDP_ConnectRequest:
					{
						bool connected = args->connectUDP();
						if (connected)
						{
							args->m_cs->setSharedParam(1, eUDP_Connected);
						}
						else
						{
							args->m_cs->setSharedParam(1, eUDP_ConnectionFailed);
						}
						break;
					}
					default:
					{
					}
				};

				if (args->m_isConnected)
				{
					args->m_cs->lock();
					bool hasCommand = args->m_hasCommand;
					args->m_cs->unlock();

					if (hasCommand)
					{
						int sz = 0;
						ENetPacket* packet = 0;

						if (args->m_clientCmd.m_type == CMD_STEP_FORWARD_SIMULATION)
						{
							sz = sizeof(int);
							packet = enet_packet_create(&args->m_clientCmd.m_type, sz, ENET_PACKET_FLAG_RELIABLE);
						}
						else
						{
							sz = sizeof(SharedMemoryCommand);
							packet = enet_packet_create(&args->m_clientCmd, sz, ENET_PACKET_FLAG_RELIABLE);
						}
						int res;
						res = enet_peer_send(args->m_peer, 0, packet);
						args->m_cs->lock();
						args->m_hasCommand = false;
						args->m_cs->unlock();
					}

					bool hasNewStatus = args->checkData();
					if (hasNewStatus)
					{
						if (args->m_hasStatus)
						{
							//overflow: last status hasn't been processed yet
							b3Assert(0);
							printf("Error: received new status but previous status not processed yet");
						}
						else
						{
							args->m_cs->lock();
							args->m_hasStatus = hasNewStatus;
							args->m_cs->unlock();
						}
					}
				}
			}

		} while (args->m_cs->getSharedParam(0) != eUDPRequestTerminate);
	}
	else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eUDPInitializationFailed);
		args->m_cs->unlock();
	}

	printf("finished\n");
}

void* UDPlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new UDPThreadLocalStorage;
}

void UDPlsMemoryReleaseFunc(void* ptr)
{
	UDPThreadLocalStorage* p = (UDPThreadLocalStorage*)ptr;
	delete p;
}

UdpNetworkedPhysicsProcessor::UdpNetworkedPhysicsProcessor(const char* hostName, int port)
{
	m_data = new UdpNetworkedInternalData;
	if (hostName)
	{
		m_data->m_hostName = hostName;
	}
	m_data->m_port = port;
}

UdpNetworkedPhysicsProcessor::~UdpNetworkedPhysicsProcessor()
{
	disconnect();
	delete m_data;
}

bool UdpNetworkedPhysicsProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	if (gVerboseNetworkMessagesClient)
	{
		printf("PhysicsClientUDP::processCommand\n");
	}
	//	int sz = sizeof(SharedMemoryCommand);

	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double timeOutInSeconds = m_data->m_timeOutInSeconds;

	m_data->m_cs->lock();
	m_data->m_clientCmd = clientCmd;
	m_data->m_hasCommand = true;
	m_data->m_cs->unlock();

	while ((m_data->m_hasCommand) && (clock.getTimeInSeconds() - startTime < timeOutInSeconds))
	{
		b3Clock::usleep(0);
	}

#if 0


	bool hasStatus = false;

	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double timeOutInSeconds = m_data->m_timeOutInSeconds;

	const SharedMemoryStatus* stat = 0;
	while ((!hasStatus) && (clock.getTimeInSeconds() - startTime < timeOutInSeconds))
	{
		hasStatus = receiveStatus(serverStatusOut, bufferServerToClient, bufferSizeInBytes);
		b3Clock::usleep(100);
	}
	return hasStatus;

#endif

	return false;
}

bool UdpNetworkedPhysicsProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = false;
	if (m_data->m_hasStatus)
	{
		if (gVerboseNetworkMessagesClient)
		{
			printf("UdpNetworkedPhysicsProcessor::receiveStatus\n");
		}

		hasStatus = true;
		serverStatusOut = m_data->m_lastStatus;
		int numStreamBytes = m_data->m_stream.size();

		if (numStreamBytes < bufferSizeInBytes)
		{
			for (int i = 0; i < numStreamBytes; i++)
			{
				bufferServerToClient[i] = m_data->m_stream[i];
			}
		}
		else
		{
			printf("Error: steam buffer overflow\n");
		}

		m_data->m_cs->lock();
		m_data->m_hasStatus = false;
		m_data->m_cs->unlock();
	}

	return hasStatus;
}

void UdpNetworkedPhysicsProcessor::renderScene(int renderFlags)
{
}

void UdpNetworkedPhysicsProcessor::physicsDebugDraw(int debugDrawFlags)
{
}

void UdpNetworkedPhysicsProcessor::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
}

bool UdpNetworkedPhysicsProcessor::isConnected() const
{
	return m_data->m_isConnected;
}

bool UdpNetworkedPhysicsProcessor::connect()
{
	if (m_data->m_threadSupport == 0)
	{
		m_data->m_threadSupport = createUDPThreadSupport(1);

		m_data->m_cs = m_data->m_threadSupport->createCriticalSection();
		m_data->m_cs->setSharedParam(0, eUDPIsUnInitialized);
		m_data->m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)m_data, 0);

		while (m_data->m_cs->getSharedParam(0) == eUDPIsUnInitialized)
		{
			b3Clock::usleep(1000);
		}

		m_data->m_cs->lock();
		m_data->m_cs->setSharedParam(1, eUDP_ConnectRequest);
		m_data->m_cs->unlock();

		while (m_data->m_cs->getSharedParam(1) == eUDP_ConnectRequest)
		{
			b3Clock::usleep(1000);
		}
	}
	unsigned int sharedParam = m_data->m_cs->getSharedParam(1);
	bool isConnected = (sharedParam == eUDP_Connected);
	return isConnected;
}

void UdpNetworkedPhysicsProcessor::disconnect()
{
	if (m_data->m_threadSupport)
	{
		m_data->m_cs->lock();
		m_data->m_cs->setSharedParam(0, eUDPRequestTerminate);
		m_data->m_cs->unlock();

		int numActiveThreads = 1;

		while (numActiveThreads)
		{
			int arg0, arg1;
			if (m_data->m_threadSupport->isTaskCompleted(&arg0, &arg1, 0))
			{
				numActiveThreads--;
				printf("numActiveThreads = %d\n", numActiveThreads);
			}
			else
			{
				b3Clock::usleep(1000);
			}
		};

		printf("stopping threads\n");

		delete m_data->m_threadSupport;
		m_data->m_threadSupport = 0;
		m_data->m_isConnected = false;
	}
}

void UdpNetworkedPhysicsProcessor::setTimeOut(double timeOutInSeconds)
{
	m_data->m_timeOutInSeconds = timeOutInSeconds;
}
