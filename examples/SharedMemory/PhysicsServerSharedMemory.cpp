#include "PhysicsServerSharedMemory.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"

#include "../CommonInterfaces/CommonRenderInterface.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btTransform.h"

#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryBlock.h"

#include "PhysicsServerCommandProcessor.h"



struct PhysicsServerSharedMemoryInternalData
{
	
	///end handle management
	
	
	SharedMemoryInterface* m_sharedMemory;
    SharedMemoryBlock* m_testBlock1;
	int m_sharedMemoryKey;
	bool m_isConnected;
	bool m_verboseOutput;
	PhysicsServerCommandProcessor* m_commandProcessor;
	
	PhysicsServerSharedMemoryInternalData()
		:m_sharedMemory(0),
		m_testBlock1(0),
		m_sharedMemoryKey(SHARED_MEMORY_KEY),
		m_isConnected(false),
		m_verboseOutput(false),
		m_commandProcessor(0)
		
	{
    
	}

	SharedMemoryStatus& createServerStatus(int statusType, int sequenceNumber, int timeStamp)
	{
		SharedMemoryStatus& serverCmd =m_testBlock1->m_serverCommands[0];
		serverCmd .m_type = statusType;
		serverCmd.m_sequenceNumber = sequenceNumber;
		serverCmd.m_timeStamp = timeStamp;
		return serverCmd;
	}
	void submitServerStatus(SharedMemoryStatus& status)
	{
		m_testBlock1->m_numServerCommands++;
	}

};


PhysicsServerSharedMemory::PhysicsServerSharedMemory()
{
	m_data = new PhysicsServerSharedMemoryInternalData();

#ifdef _WIN32
	m_data->m_sharedMemory = new Win32SharedMemoryServer();
#else
	m_data->m_sharedMemory = new PosixSharedMemory();
#endif
	
	m_data->m_commandProcessor = new PhysicsServerCommandProcessor;
	m_data->m_commandProcessor ->createEmptyDynamicsWorld();


}

PhysicsServerSharedMemory::~PhysicsServerSharedMemory()
{
	m_data->m_commandProcessor->deleteDynamicsWorld();
	delete m_data->m_commandProcessor;
	delete m_data;
}

void PhysicsServerSharedMemory::setSharedMemoryKey(int key)
{
	m_data->m_sharedMemoryKey = key;
}


bool PhysicsServerSharedMemory::connectSharedMemory( struct GUIHelperInterface* guiHelper)
{
	
	m_data->m_commandProcessor->setGuiHelper(guiHelper);

	
	bool allowCreation = true;
	

    if (m_data->m_isConnected)
    {
        b3Warning("connectSharedMemory, while already connected");
        return m_data->m_isConnected;
    }
    
    
	int counter = 0;
	do 
	{

		m_data->m_testBlock1 = (SharedMemoryBlock*)m_data->m_sharedMemory->allocateSharedMemory(m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE,allowCreation);
		if (m_data->m_testBlock1)
		{
			int magicId =m_data->m_testBlock1->m_magicId;
			if (m_data->m_verboseOutput)
			{
				b3Printf("magicId = %d\n", magicId);
			}
        
			if (m_data->m_testBlock1->m_magicId !=SHARED_MEMORY_MAGIC_NUMBER)
			{
				InitSharedMemoryBlock(m_data->m_testBlock1);
				if (m_data->m_verboseOutput)
				{
					b3Printf("Created and initialized shared memory block\n");
				}
				m_data->m_isConnected = true;
			} else
			{
				m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE);
				m_data->m_testBlock1 = 0;
				m_data->m_isConnected = false;
			}
		} else
		{
			b3Error("Cannot connect to shared memory");
			m_data->m_isConnected = false;
		}
	} while (counter++ < 10 && !m_data->m_isConnected);

	if (!m_data->m_isConnected)
	{
		b3Error("Server cannot connect to shared memory.\n");
	}
	
	return m_data->m_isConnected;
}


void PhysicsServerSharedMemory::disconnectSharedMemory(bool deInitializeSharedMemory)
{
	m_data->m_commandProcessor->setGuiHelper(0);

	if (m_data->m_verboseOutput)
	{
		b3Printf("releaseSharedMemory1\n");
	}
	if (m_data->m_testBlock1)
	{
		if (m_data->m_verboseOutput)
		{
			b3Printf("m_testBlock1\n");
		}
		if (deInitializeSharedMemory)
		{
			m_data->m_testBlock1->m_magicId = 0;
			if (m_data->m_verboseOutput)
			{
				b3Printf("De-initialized shared memory, magic id = %d\n",m_data->m_testBlock1->m_magicId);
			}
		}
		btAssert(m_data->m_sharedMemory);
		m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE);
	}
	if (m_data->m_sharedMemory)
	{
		if (m_data->m_verboseOutput)
		{
			b3Printf("m_sharedMemory\n");
		}
		delete m_data->m_sharedMemory;
		m_data->m_sharedMemory = 0;
		m_data->m_testBlock1 = 0;
	}
}

void PhysicsServerSharedMemory::releaseSharedMemory()
{
	if (m_data->m_verboseOutput)
	{
		b3Printf("releaseSharedMemory1\n");
	}
    if (m_data->m_testBlock1)
    {
		if (m_data->m_verboseOutput)
		{
			b3Printf("m_testBlock1\n");
		}
        m_data->m_testBlock1->m_magicId = 0;
		if (m_data->m_verboseOutput)
		{
			b3Printf("magic id = %d\n",m_data->m_testBlock1->m_magicId);
		}
        btAssert(m_data->m_sharedMemory);
		m_data->m_sharedMemory->releaseSharedMemory(	m_data->m_sharedMemoryKey
, SHARED_MEMORY_SIZE);
    }
    if (m_data->m_sharedMemory)
    {
		if (m_data->m_verboseOutput)
		{
			b3Printf("m_sharedMemory\n");
		}
        delete m_data->m_sharedMemory;
        m_data->m_sharedMemory = 0;
        m_data->m_testBlock1 = 0;
    }
}






void PhysicsServerSharedMemory::processClientCommands()
{
	if (m_data->m_isConnected && m_data->m_testBlock1)
    {
#if 0
		m_data->m_commandProcessor->processLogCommand();

		if (m_data->m_logPlayback)
		{
			if (m_data->m_testBlock1->m_numServerCommands>m_data->m_testBlock1->m_numProcessedServerCommands)
			{
				m_data->m_testBlock1->m_numProcessedServerCommands++;
			}
			//push a command from log file
			bool hasCommand = m_data->m_logPlayback->processNextCommand(&m_data->m_testBlock1->m_clientCommands[0]);
			if (hasCommand)
			{
				m_data->m_testBlock1->m_numClientCommands++;
			}
		}
#endif
        ///we ignore overflow of integer for now
        if (m_data->m_testBlock1->m_numClientCommands> m_data->m_testBlock1->m_numProcessedClientCommands)
        {
           

            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            btAssert(m_data->m_testBlock1->m_numClientCommands==m_data->m_testBlock1->m_numProcessedClientCommands+1);
            
			const SharedMemoryCommand& clientCmd =m_data->m_testBlock1->m_clientCommands[0];

			m_data->m_testBlock1->m_numProcessedClientCommands++;
			//todo, timeStamp 
			int timeStamp = 0;
			SharedMemoryStatus& serverStatusOut = m_data->createServerStatus(CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
			bool hasStatus = m_data->m_commandProcessor->processCommand(clientCmd, serverStatusOut,&m_data->m_testBlock1->m_bulletStreamDataServerToClientRefactor[0],SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
			if (hasStatus)
			{
				m_data->submitServerStatus(serverStatusOut);
			}
			       
        }
    }
}

void PhysicsServerSharedMemory::renderScene()
{
	m_data->m_commandProcessor->renderScene();

	
	
}

void    PhysicsServerSharedMemory::physicsDebugDraw(int debugDrawFlags)
{
	m_data->m_commandProcessor->physicsDebugDraw(debugDrawFlags);
}


bool PhysicsServerSharedMemory::pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	return m_data->m_commandProcessor->pickBody(rayFromWorld,rayToWorld);
}

bool PhysicsServerSharedMemory::movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	return m_data->m_commandProcessor->movePickedBody(rayFromWorld,rayToWorld);
}
void PhysicsServerSharedMemory::removePickingConstraint()
{
	m_data->m_commandProcessor->removePickingConstraint();
}

void PhysicsServerSharedMemory::enableCommandLogging(bool enable, const char* fileName)
{
	m_data->m_commandProcessor->enableCommandLogging(enable,fileName);
}

void PhysicsServerSharedMemory::replayFromLogFile(const char* fileName)
{
	m_data->m_commandProcessor->replayFromLogFile(fileName);
}
