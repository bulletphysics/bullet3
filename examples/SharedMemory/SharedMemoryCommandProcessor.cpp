#include "SharedMemoryCommandProcessor.h"

#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3Scalar.h"

#include "SharedMemoryBlock.h"

struct SharedMemoryCommandProcessorInternalData
{
	int m_sharedMemoryKey;
	bool m_isConnected;
	SharedMemoryInterface* m_sharedMemory;
	bool m_ownsSharedMemory;
	bool m_verboseOutput;
	bool m_waitingForServer;
	SharedMemoryStatus m_lastServerStatus;
	SharedMemoryBlock* m_testBlock1;
	SendActualStateSharedMemoryStorage m_cachedState;
	
	SharedMemoryCommandProcessorInternalData()
		: m_sharedMemoryKey(SHARED_MEMORY_KEY),
		  m_isConnected(false),
		  m_sharedMemory(0),
		  m_ownsSharedMemory(false),
		  m_verboseOutput(false),
		  m_waitingForServer(false),
		  m_testBlock1(0)
	{
	}
};

SharedMemoryCommandProcessor::SharedMemoryCommandProcessor()
{
	m_data = new SharedMemoryCommandProcessorInternalData;
	m_data->m_sharedMemoryKey = SHARED_MEMORY_KEY;
#ifdef _WIN32
	m_data->m_sharedMemory = new Win32SharedMemoryClient();
#else
	m_data->m_sharedMemory = new PosixSharedMemory();
#endif
	m_data->m_ownsSharedMemory = true;
}

SharedMemoryCommandProcessor::~SharedMemoryCommandProcessor()
{
	if (m_data->m_isConnected)
	{
		disconnect();
	}
	if (m_data->m_ownsSharedMemory)
	{
		delete m_data->m_sharedMemory;
	}
	delete m_data;
}

bool SharedMemoryCommandProcessor::connect()
{
	if (m_data->m_isConnected)
		return true;

	bool allowCreation = false;
	m_data->m_testBlock1 = (SharedMemoryBlock*)m_data->m_sharedMemory->allocateSharedMemory(
		m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE, allowCreation);

	if (m_data->m_testBlock1)
	{
		if (m_data->m_testBlock1->m_magicId != SHARED_MEMORY_MAGIC_NUMBER)
		{
			if ((m_data->m_testBlock1->m_magicId < 211705023) &&
				(m_data->m_testBlock1->m_magicId >= 201705023))
			{
				b3Error("Error: physics server version mismatch (expected %d got %d)\n", SHARED_MEMORY_MAGIC_NUMBER, m_data->m_testBlock1->m_magicId);
			}
			else
			{
				b3Error("Error connecting to shared memory: please start server before client\n");
			}
			m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey,
														SHARED_MEMORY_SIZE);
			m_data->m_testBlock1 = 0;
			return false;
		}
		else
		{
			if (m_data->m_verboseOutput)
			{
				b3Printf("Connected to existing shared memory, status OK.\n");
			}
			m_data->m_isConnected = true;
		}
	}
	else
	{
		b3Error("Cannot connect to shared memory");
		return false;
	}
	return true;
}

void SharedMemoryCommandProcessor::disconnect()
{
	if (m_data->m_isConnected && m_data->m_sharedMemory)
	{
		m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE);
	}
	m_data->m_isConnected = false;
}

bool SharedMemoryCommandProcessor::isConnected() const
{
	return m_data->m_isConnected;
}

bool SharedMemoryCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	if (!m_data->m_waitingForServer)
	{
		if (&m_data->m_testBlock1->m_clientCommands[0] != &clientCmd)
		{
			m_data->m_testBlock1->m_clientCommands[0] = clientCmd;
		}
		m_data->m_testBlock1->m_numClientCommands++;
		m_data->m_waitingForServer = true;
	}

	return false;
}

bool SharedMemoryCommandProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	m_data->m_lastServerStatus.m_dataStream = 0;
	m_data->m_lastServerStatus.m_numDataStreamBytes = 0;

	if (!m_data->m_testBlock1)
	{
		//m_data->m_lastServerStatus.m_type = CMD_SHARED_MEMORY_NOT_INITIALIZED;
		//return &m_data->m_lastServerStatus;
		//serverStatusOut = m_data->m_lastServerStatus;
		return false;
	}

	if (!m_data->m_waitingForServer)
	{
		return false;
	}

	if (m_data->m_testBlock1->m_magicId != SHARED_MEMORY_MAGIC_NUMBER)
	{
		//m_data->m_lastServerStatus.m_type = CMD_SHARED_MEMORY_NOT_INITIALIZED;
		//return &m_data->m_lastServerStatus;
		return false;
	}

	if (m_data->m_testBlock1->m_numServerCommands >
		m_data->m_testBlock1->m_numProcessedServerCommands)
	{
		b3Assert(m_data->m_testBlock1->m_numServerCommands ==
				 m_data->m_testBlock1->m_numProcessedServerCommands + 1);

		const SharedMemoryStatus& serverCmd = m_data->m_testBlock1->m_serverCommands[0];
		if (serverCmd.m_type == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
		{
			SendActualStateSharedMemoryStorage* serverState = (SendActualStateSharedMemoryStorage*)m_data->m_testBlock1->m_bulletStreamDataServerToClientRefactor;
			m_data->m_cachedState = *serverState;
			//ideally we provided a 'getCachedState' but that would require changing the API, so we store a pointer instead.
			m_data->m_testBlock1->m_serverCommands[0].m_sendActualStateArgs.m_stateDetails = &m_data->m_cachedState;
		}

		m_data->m_lastServerStatus = serverCmd;
		m_data->m_lastServerStatus.m_dataStream = m_data->m_testBlock1->m_bulletStreamDataServerToClientRefactor;

		for (int i = 0; i < m_data->m_lastServerStatus.m_numDataStreamBytes; i++)
		{
			bufferServerToClient[i] = m_data->m_testBlock1->m_bulletStreamDataServerToClientRefactor[i];
		}

		m_data->m_testBlock1->m_numProcessedServerCommands++;
		// we don't have more than 1 command outstanding (in total, either server or client)
		b3Assert(m_data->m_testBlock1->m_numProcessedServerCommands ==
				 m_data->m_testBlock1->m_numServerCommands);

		if (m_data->m_testBlock1->m_numServerCommands ==
			m_data->m_testBlock1->m_numProcessedServerCommands)
		{
			m_data->m_waitingForServer = false;
		}
		else
		{
			m_data->m_waitingForServer = true;
		}

		serverStatusOut = m_data->m_lastServerStatus;

		return true;
	}
	return false;
}

void SharedMemoryCommandProcessor::renderScene(int renderFlags)
{
}

void SharedMemoryCommandProcessor::physicsDebugDraw(int debugDrawFlags)
{
}

void SharedMemoryCommandProcessor::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
}

void SharedMemoryCommandProcessor::setSharedMemoryInterface(class SharedMemoryInterface* sharedMem)
{
	if (m_data->m_sharedMemory && m_data->m_ownsSharedMemory)
	{
		delete m_data->m_sharedMemory;
	}
	m_data->m_ownsSharedMemory = false;
	m_data->m_sharedMemory = sharedMem;
}

void SharedMemoryCommandProcessor::setSharedMemoryKey(int key)
{
	m_data->m_sharedMemoryKey = key;
}

void SharedMemoryCommandProcessor::setTimeOut(double /*timeOutInSeconds*/)
{
}
