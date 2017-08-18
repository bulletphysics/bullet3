#include "PhysicsServerSharedMemory.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"

#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btTransform.h"

#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryBlock.h"

#include "PhysicsServerCommandProcessor.h"

//number of shared memory blocks == number of simultaneous connections
#define MAX_SHARED_MEMORY_BLOCKS 2

struct PhysicsServerSharedMemoryInternalData
{
	
	///end handle management
	
	
	SharedMemoryInterface* m_sharedMemory;
	bool m_ownsSharedMemory;

    SharedMemoryBlock* m_testBlocks[MAX_SHARED_MEMORY_BLOCKS];
	int m_sharedMemoryKey;
	bool m_areConnected[MAX_SHARED_MEMORY_BLOCKS];
	bool m_verboseOutput;
	CommandProcessorInterface* m_commandProcessor;
	CommandProcessorCreationInterface* m_commandProcessorCreator;

	PhysicsServerSharedMemoryInternalData()
		:m_sharedMemory(0),
		m_ownsSharedMemory(false),
  		m_sharedMemoryKey(SHARED_MEMORY_KEY),
    	m_verboseOutput(false),
		m_commandProcessor(0)
		
	{
 
        for (int i=0;i<MAX_SHARED_MEMORY_BLOCKS;i++)
        {
            m_testBlocks[i]=0;
            m_areConnected[i]=false;
        }
	}

	SharedMemoryStatus& createServerStatus(int statusType, int sequenceNumber, int timeStamp, int blockIndex)
	{
		SharedMemoryStatus& serverCmd =m_testBlocks[blockIndex]->m_serverCommands[0];
		serverCmd .m_type = statusType;
		serverCmd.m_sequenceNumber = sequenceNumber;
		serverCmd.m_timeStamp = timeStamp;
		return serverCmd;
	}
	void submitServerStatus(SharedMemoryStatus& status,int blockIndex)
	{
		m_testBlocks[blockIndex]->m_numServerCommands++;
	}

};


PhysicsServerSharedMemory::PhysicsServerSharedMemory(CommandProcessorCreationInterface* commandProcessorCreator, SharedMemoryInterface* sharedMem, int bla)
{
	m_data = new PhysicsServerSharedMemoryInternalData();
	m_data->m_commandProcessorCreator = commandProcessorCreator;
	if (sharedMem)
	{
		m_data->m_sharedMemory = sharedMem;
		m_data->m_ownsSharedMemory = false;
	} else
	{
#ifdef _WIN32
	m_data->m_sharedMemory = new Win32SharedMemoryServer();
#else
	m_data->m_sharedMemory = new PosixSharedMemory();
#endif
	m_data->m_ownsSharedMemory = true;
	}


	m_data->m_commandProcessor = commandProcessorCreator->createCommandProcessor();

}

PhysicsServerSharedMemory::~PhysicsServerSharedMemory()
{
    if (m_data->m_sharedMemory)
    {
        if (m_data->m_verboseOutput)
        {
            b3Printf("m_sharedMemory\n");
        }
        if (m_data->m_ownsSharedMemory)
        {
            delete m_data->m_sharedMemory;
        }
        m_data->m_sharedMemory = 0;
    }

	m_data->m_commandProcessorCreator->deleteCommandProcessor(m_data->m_commandProcessor);
    delete m_data;
}

/*void PhysicsServerSharedMemory::resetDynamicsWorld()
{
	m_data->m_commandProcessor->deleteDynamicsWorld();
	m_data->m_commandProcessor ->createEmptyDynamicsWorld();
}
*/
void PhysicsServerSharedMemory::setSharedMemoryKey(int key)
{
	m_data->m_sharedMemoryKey = key;
}


bool PhysicsServerSharedMemory::connectSharedMemory( struct GUIHelperInterface* guiHelper)
{
	
	m_data->m_commandProcessor->setGuiHelper(guiHelper);

	
	bool allowCreation = true;
    bool allConnected = false;
	int numConnected = 0;
	
	 
	int counter = 0;
    for (int block=0;block<MAX_SHARED_MEMORY_BLOCKS;block++)
    {
        if (m_data->m_areConnected[block])
        {
            allConnected = true;
			numConnected++;
            b3Warning("connectSharedMemory, while already connected");
            continue;
        }
        do
        {

            m_data->m_testBlocks[block] = (SharedMemoryBlock*)m_data->m_sharedMemory->allocateSharedMemory(m_data->m_sharedMemoryKey+block, SHARED_MEMORY_SIZE,allowCreation);
            if (m_data->m_testBlocks[block])
            {
                int magicId =m_data->m_testBlocks[block]->m_magicId;
                if (m_data->m_verboseOutput)
                {
                    b3Printf("magicId = %d\n", magicId);
                }
            
                if (m_data->m_testBlocks[block]->m_magicId !=SHARED_MEMORY_MAGIC_NUMBER)
                {
                    InitSharedMemoryBlock(m_data->m_testBlocks[block]);
                    if (m_data->m_verboseOutput)
                    {
                        b3Printf("Created and initialized shared memory block\n");
                    }
                    m_data->m_areConnected[block] = true;
					numConnected++;
                } else
                {
                    m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey+block, SHARED_MEMORY_SIZE);
                    m_data->m_testBlocks[block] = 0;
                    m_data->m_areConnected[block] = false;
                }
            } else
            {
                b3Error("Cannot connect to shared memory");
                m_data->m_areConnected[block] = false;
            }
        } while (counter++ < 10 && !m_data->m_areConnected[block]);
        if (!m_data->m_areConnected[block])
        {
            b3Error("Server cannot connect to shared memory.\n");
        }
    }
	
	allConnected = (numConnected==MAX_SHARED_MEMORY_BLOCKS);
	
	return allConnected;
}


void PhysicsServerSharedMemory::disconnectSharedMemory(bool deInitializeSharedMemory)
{
	//m_data->m_commandProcessor->deleteDynamicsWorld();

	m_data->m_commandProcessor->setGuiHelper(0);

	if (m_data->m_verboseOutput)
	{
		b3Printf("releaseSharedMemory1\n");
	}
    for (int block = 0;block<MAX_SHARED_MEMORY_BLOCKS;block++)
    {
        if (m_data->m_testBlocks[block])
        {
            if (m_data->m_verboseOutput)
            {
                b3Printf("m_testBlock1\n");
            }
            if (deInitializeSharedMemory)
            {
                m_data->m_testBlocks[block]->m_magicId = 0;
                if (m_data->m_verboseOutput)
                {
                    b3Printf("De-initialized shared memory, magic id = %d\n",m_data->m_testBlocks[block]->m_magicId);
                }
            }
            btAssert(m_data->m_sharedMemory);
            m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey+block, SHARED_MEMORY_SIZE);
        }
        m_data->m_testBlocks[block] = 0;
        m_data->m_areConnected[block] = false;
        
    }
}

void PhysicsServerSharedMemory::releaseSharedMemory()
{
    disconnectSharedMemory(true);
}


void PhysicsServerSharedMemory::stepSimulationRealTime(double dtInSec, const struct b3VRControllerEvent* vrEvents, int numVREvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents)
{
	m_data->m_commandProcessor->stepSimulationRealTime(dtInSec,vrEvents, numVREvents, keyEvents,numKeyEvents,mouseEvents, numMouseEvents);
}

void PhysicsServerSharedMemory::enableRealTimeSimulation(bool enableRealTimeSim)
{
	m_data->m_commandProcessor->enableRealTimeSimulation(enableRealTimeSim);
}

bool PhysicsServerSharedMemory::isRealTimeSimulationEnabled() const
{
	return m_data->m_commandProcessor->isRealTimeSimulationEnabled();
}



void PhysicsServerSharedMemory::processClientCommands()
{
    for (int block = 0;block<MAX_SHARED_MEMORY_BLOCKS;block++)
    {
        
        if (m_data->m_areConnected[block] && m_data->m_testBlocks[block])
        {
            m_data->m_commandProcessor->replayLogCommand(&m_data->m_testBlocks[block]->m_bulletStreamDataServerToClientRefactor[0],SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
            
            ///we ignore overflow of integer for now
            if (m_data->m_testBlocks[block]->m_numClientCommands> m_data->m_testBlocks[block]->m_numProcessedClientCommands)
            {
               
				//BT_PROFILE("processClientCommand");

                //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
                btAssert(m_data->m_testBlocks[block]->m_numClientCommands==m_data->m_testBlocks[block]->m_numProcessedClientCommands+1);
                
                const SharedMemoryCommand& clientCmd =m_data->m_testBlocks[block]->m_clientCommands[0];

                m_data->m_testBlocks[block]->m_numProcessedClientCommands++;
                //todo, timeStamp 
                int timeStamp = 0;
                SharedMemoryStatus& serverStatusOut = m_data->createServerStatus(CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,clientCmd.m_sequenceNumber,timeStamp,block);
                bool hasStatus = m_data->m_commandProcessor->processCommand(clientCmd, serverStatusOut,&m_data->m_testBlocks[block]->m_bulletStreamDataServerToClientRefactor[0],SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
                if (hasStatus)
                {
                    m_data->submitServerStatus(serverStatusOut,block);
                }
                       
            }
        }
    }
}

void PhysicsServerSharedMemory::renderScene(int renderFlags)
{
	m_data->m_commandProcessor->renderScene(renderFlags);

}

void	PhysicsServerSharedMemory::syncPhysicsToGraphics()
{
	m_data->m_commandProcessor->syncPhysicsToGraphics();
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

const btVector3& PhysicsServerSharedMemory::getVRTeleportPosition() const
{
	return m_data->m_commandProcessor->getVRTeleportPosition();
}
void PhysicsServerSharedMemory::setVRTeleportPosition(const btVector3& vrTeleportPos)
{
	m_data->m_commandProcessor->setVRTeleportPosition(vrTeleportPos);
}

const btQuaternion& PhysicsServerSharedMemory::getVRTeleportOrientation() const
{
	return m_data->m_commandProcessor->getVRTeleportOrientation();

}
void PhysicsServerSharedMemory::setVRTeleportOrientation(const btQuaternion& vrTeleportOrn)
{
	m_data->m_commandProcessor->setVRTeleportOrientation(vrTeleportOrn);
}

	
