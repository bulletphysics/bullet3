#include "PhysicsClientTCP.h"

#include "ActiveSocket.h" 

#include <stdio.h>
#include <string.h>
#include "../Utils/b3Clock.h"
#include "PhysicsClient.h"
//#include "LinearMath/btVector3.h"
#include "SharedMemoryCommands.h"
#include <string>
#include "Bullet3Common/b3Logging.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
void	TCPThreadFunc(void* userPtr, void* lsMemory);
void*	TCPlsMemoryFunc();
bool gVerboseNetworkMessagesClient2 = false;

#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"

b3ThreadSupportInterface* createTCPThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("TCPThread",
		TCPThreadFunc,
		TCPlsMemoryFunc,
		numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;

}


#elif defined( _WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createTCPThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("TCPThread", TCPThreadFunc, TCPlsMemoryFunc, numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;

}
#endif



struct TCPThreadLocalStorage
{
	int threadId;
};



unsigned int b3DeserializeInt2(const unsigned char* input)
{
	unsigned int tmp = (input[3] << 24) + (input[2] << 16) + (input[1] << 8) + input[0];
	return tmp;
}

struct	TcpNetworkedInternalData
{
    /*
	ENetHost*	m_client;
	ENetAddress	m_address;
	ENetPeer*	m_peer;
	ENetEvent 	m_event;
     */
    CActiveSocket m_tcpSocket;
    
	bool		m_isConnected;

	b3ThreadSupportInterface* m_threadSupport;

	b3CriticalSection* m_cs;

	TcpNetworkedInternalData* m_tcpInternalData;
	

	SharedMemoryCommand m_clientCmd;
	bool m_hasCommand;

	bool		m_hasStatus;
	SharedMemoryStatus m_lastStatus;
	b3AlignedObjectArray<char> m_stream;

	std::string m_hostName;
	int m_port;

	TcpNetworkedInternalData()
		:
		m_isConnected(false),
		m_threadSupport(0),
		m_hasCommand(false),
		m_hasStatus(false)
	{

	}

	bool connectTCP()
	{
		if (m_isConnected)
			return true;

        m_tcpSocket.Initialize();
        
        m_isConnected = m_tcpSocket.Open(m_hostName.c_str(),m_port);

        m_isConnected = true;
		return m_isConnected;
	}

	bool checkData()
	{
		bool hasStatus = false;

		//int serviceResult = enet_host_service(m_client, &m_event, 0);
        int maxLen = 4 + sizeof(SharedMemoryStatus)+SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE;
        
        int recBytes = m_tcpSocket.Receive(maxLen);

        if (gVerboseNetworkMessagesClient2)
        {
            printf("A packet of length %d bytes received\n", recBytes);
        }
        
        unsigned char* data = (unsigned char*)m_tcpSocket.GetData();
        
        int packetSizeInBytes = b3DeserializeInt2(data);

        if (packetSizeInBytes == recBytes)
        {

            SharedMemoryStatus* statPtr = (SharedMemoryStatus*)&data[4];
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
                    m_stream[i] = data[i + streamOffsetInBytes];
                }
            }
        }
        else
        {
            printf("unknown status message received\n");
        }
        
		return hasStatus;
	}

};

enum TCPThreadEnums
{
	eTCPRequestTerminate = 13,
	eTCPIsUnInitialized,
	eTCPIsInitialized,
	eTCPInitializationFailed,
	eTCPHasTerminated
};



enum TCPCommandEnums
{
	eTCPIdle = 13,
	eTCP_ConnectRequest,
	eTCP_Connected,
	eTCP_ConnectionFailed,
	eTCP_DisconnectRequest,
	eTCP_Disconnected,

};

void	TCPThreadFunc(void* userPtr, void* lsMemory)
{
	printf("TCPThreadFunc thread started\n");

	TcpNetworkedInternalData* args = (TcpNetworkedInternalData*)userPtr;
//	int workLeft = true;
	b3Clock clock;
	clock.reset();
	bool init = true;
	if (init)
	{

		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eTCPIsInitialized);
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
					case eTCP_ConnectRequest:
					{
						bool connected = args->connectTCP();
						if (connected)
						{
							args->m_cs->setSharedParam(1, eTCP_Connected);
						}
						else
						{
							args->m_cs->setSharedParam(1, eTCP_ConnectionFailed);
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
                        unsigned char* data = 0;
                        

						if (args->m_clientCmd.m_type == CMD_STEP_FORWARD_SIMULATION)
						{
							sz = sizeof(int);
                            data = (unsigned char*) &args->m_clientCmd.m_type;
						}
						else
						{
							sz = sizeof(SharedMemoryCommand);
                            data = (unsigned char*)&args->m_clientCmd;
    					}
						int res;
						
                        args->m_tcpSocket.Send((const uint8 *)data,sz);
                        
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

		} while (args->m_cs->getSharedParam(0) != eTCPRequestTerminate);
	}
	else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eTCPInitializationFailed);
		args->m_cs->unlock();
	}


	printf("finished\n");

}



void*	TCPlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new TCPThreadLocalStorage;
}







TcpNetworkedPhysicsProcessor::TcpNetworkedPhysicsProcessor(const char* hostName, int port)
{
	m_data = new TcpNetworkedInternalData;
	if (hostName)
	{
		m_data->m_hostName = hostName;
	}
	m_data->m_port = port;

}

TcpNetworkedPhysicsProcessor::~TcpNetworkedPhysicsProcessor()
{
	disconnect();
	delete m_data;
}

bool TcpNetworkedPhysicsProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	if(gVerboseNetworkMessagesClient2)
	{
		printf("PhysicsClientTCP::processCommand\n");
	}
//	int sz = sizeof(SharedMemoryCommand);
	int timeout = 1024 * 1024 * 1024;

	m_data->m_cs->lock();
	m_data->m_clientCmd = clientCmd;
	m_data->m_hasCommand = true;
	m_data->m_cs->unlock();

	while (m_data->m_hasCommand &&  (timeout-- > 0))
	{
		b3Clock::usleep(0);
	}

#if 0

	timeout = 1024 * 1024 * 1024;

	bool hasStatus = false;

	const SharedMemoryStatus* stat = 0;
	while ((!hasStatus) && (timeout-- > 0))
	{
		hasStatus = receiveStatus(serverStatusOut, bufferServerToClient, bufferSizeInBytes);
		b3Clock::usleep(100);
	}
	return hasStatus;

#endif

	return false;
}

bool TcpNetworkedPhysicsProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = false;
	if (m_data->m_hasStatus)
	{
		if (gVerboseNetworkMessagesClient2)
		{
			printf("TcpNetworkedPhysicsProcessor::receiveStatus\n");
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


void TcpNetworkedPhysicsProcessor::renderScene()
{
}

void   TcpNetworkedPhysicsProcessor::physicsDebugDraw(int debugDrawFlags)
{
}

void TcpNetworkedPhysicsProcessor::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
}

bool TcpNetworkedPhysicsProcessor::isConnected() const
{
	return m_data->m_isConnected;
}


bool TcpNetworkedPhysicsProcessor::connect()
{
	if (m_data->m_threadSupport==0)
	{
		m_data->m_threadSupport = createTCPThreadSupport(1);
	
		m_data->m_cs = m_data->m_threadSupport->createCriticalSection();
		m_data->m_cs->setSharedParam(0, eTCPIsUnInitialized);
		m_data->m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*) m_data, 0);

		while (m_data->m_cs->getSharedParam(0) == eTCPIsUnInitialized)
		{
			b3Clock::usleep(1000);
		}

		m_data->m_cs->lock();
		m_data->m_cs->setSharedParam(1, eTCP_ConnectRequest);
		m_data->m_cs->unlock();

		while (m_data->m_cs->getSharedParam(1) == eTCP_ConnectRequest)
		{
			b3Clock::usleep(1000);
		}

	}
	unsigned int sharedParam = m_data->m_cs->getSharedParam(1);
	bool isConnected = (sharedParam == eTCP_Connected);
	return isConnected;
}

void TcpNetworkedPhysicsProcessor::disconnect()
{
	if (m_data->m_threadSupport)
	{
		m_data->m_cs->lock();
		m_data->m_cs->setSharedParam(0, eTCPRequestTerminate);
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





