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
#include "Bullet3Common/b3AlignedObjectArray.h"

unsigned int b3DeserializeInt2(const unsigned char* input)
{
	unsigned int tmp = (input[3] << 24) + (input[2] << 16) + (input[1] << 8) + input[0];
	return tmp;
}
bool gVerboseNetworkMessagesClient2 = false;

struct TcpNetworkedInternalData
{
	/*
	ENetHost*	m_client;
	ENetAddress	m_address;
	ENetPeer*	m_peer;
	ENetEvent 	m_event;
     */
	CActiveSocket m_tcpSocket;

	bool m_isConnected;

	TcpNetworkedInternalData* m_tcpInternalData;

	SharedMemoryCommand m_clientCmd;
	bool m_hasCommand;

	SharedMemoryStatus m_lastStatus;
	b3AlignedObjectArray<char> m_stream;

	std::string m_hostName;
	int m_port;

	b3AlignedObjectArray<unsigned char> m_tempBuffer;
	double m_timeOutInSeconds;

	TcpNetworkedInternalData()
		: m_isConnected(false),
		  m_hasCommand(false),
		  m_timeOutInSeconds(60)
	{
	}

	bool connectTCP()
	{
		if (m_isConnected)
			return true;

		m_tcpSocket.Initialize();

		m_isConnected = m_tcpSocket.Open(m_hostName.c_str(), m_port);
		if (m_isConnected)
		{
			m_tcpSocket.SetSendTimeout(m_timeOutInSeconds, 0);
			m_tcpSocket.SetReceiveTimeout(m_timeOutInSeconds, 0);
		}
		int key = SHARED_MEMORY_MAGIC_NUMBER;
		m_tcpSocket.Send((uint8*)&key, 4);

		return m_isConnected;
	}

	bool checkData()
	{
		bool hasStatus = false;

		//int serviceResult = enet_host_service(m_client, &m_event, 0);
		int maxLen = 4 + sizeof(SharedMemoryStatus) + SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE;

		int rBytes = m_tcpSocket.Receive(maxLen);
		if (rBytes <= 0)
			return false;

		//append to tmp buffer
		//recBytes

		unsigned char* d2 = (unsigned char*)m_tcpSocket.GetData();

		int curSize = m_tempBuffer.size();
		m_tempBuffer.resize(curSize + rBytes);
		for (int i = 0; i < rBytes; i++)
		{
			m_tempBuffer[curSize + i] = d2[i];
		}

		int packetSizeInBytes = -1;

		if (m_tempBuffer.size() >= 4)
		{
			packetSizeInBytes = b3DeserializeInt2(&m_tempBuffer[0]);
		}

		if (m_tempBuffer.size() == packetSizeInBytes)
		{
			unsigned char* data = &m_tempBuffer[0];
			if (gVerboseNetworkMessagesClient2)
			{
				printf("A packet of length %d bytes received\n", m_tempBuffer.size());
			}

			hasStatus = true;
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
			m_tempBuffer.clear();
		}
		return hasStatus;
	}
};

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
	if (gVerboseNetworkMessagesClient2)
	{
		printf("PhysicsClientTCP::processCommand\n");
	}

	{
		int sz = 0;
		unsigned char* data = 0;
		m_data->m_tempBuffer.clear();

		if (clientCmd.m_type == CMD_STEP_FORWARD_SIMULATION)
		{
			sz = sizeof(int);
			data = (unsigned char*)&clientCmd.m_type;
		}
		else
		{
			if (clientCmd.m_type == CMD_REQUEST_VR_EVENTS_DATA)
			{
				sz = 3 * sizeof(int) + sizeof(smUint64_t) + 16;
				data = (unsigned char*)&clientCmd;
			}
			else
			{
				sz = sizeof(SharedMemoryCommand);
				data = (unsigned char*)&clientCmd;
			}
		}

		m_data->m_tcpSocket.Send((const uint8*)data, sz);
	}

	return false;
}

bool TcpNetworkedPhysicsProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = m_data->checkData();

	if (hasStatus)
	{
		if (gVerboseNetworkMessagesClient2)
		{
			printf("TcpNetworkedPhysicsProcessor::receiveStatus\n");
		}

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
	}

	return hasStatus;
}

void TcpNetworkedPhysicsProcessor::renderScene(int renderFlags)
{
}

void TcpNetworkedPhysicsProcessor::physicsDebugDraw(int debugDrawFlags)
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
	bool isConnected = m_data->connectTCP();
	return isConnected;
}

void TcpNetworkedPhysicsProcessor::disconnect()
{
	const char msg[16] = "disconnect";
	m_data->m_tcpSocket.Send((const uint8*)msg, 10);
	m_data->m_tcpSocket.Close();
	m_data->m_isConnected = false;
}

void TcpNetworkedPhysicsProcessor::setTimeOut(double timeOutInSeconds)
{
	m_data->m_timeOutInSeconds = timeOutInSeconds;
}
