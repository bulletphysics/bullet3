
#include "PhysicsServer.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "PosixSharedMemory.h"



class PhysicsServer : public CommonMultiBodyBase
{
	SharedMemoryInterface* m_sharedMemory;
    SharedMemoryExampleData* m_testBlock1;
	
public:
    
	PhysicsServer(GUIHelperInterface* helper);
    
	virtual ~PhysicsServer();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
    
};

PhysicsServer::PhysicsServer(GUIHelperInterface* helper)
:CommonMultiBodyBase(helper),
m_testBlock1(0)
{
	b3Printf("Started PhysicsServer\n");
	m_sharedMemory = new PosixSharedMemory();
}

PhysicsServer::~PhysicsServer()
{
    if (m_testBlock1)
    {
        m_testBlock1->m_magicId = 0;
        b3Printf("magic id = %d\n",m_testBlock1->m_magicId);
    }
    m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
	delete m_sharedMemory;    
}

void	PhysicsServer::initPhysics()
{
    m_testBlock1 = (SharedMemoryExampleData*) m_sharedMemory->allocateSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    
//    btAssert(m_testBlock1);
    if (m_testBlock1)
    {
      //  btAssert(m_testBlock1->m_magicId != SHARED_MEMORY_MAGIC_NUMBER);
        if (m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER)
        {
            b3Printf("Warning: shared memory is already initialized, did you already spawn a server?\n");
        }
        
        m_testBlock1->m_numClientCommands = 0;
        m_testBlock1->m_numServerCommands = 0;
        m_testBlock1->m_numProcessedClientCommands=0;
        m_testBlock1->m_numProcessedServerCommands=0;
        
        m_testBlock1->m_magicId = SHARED_MEMORY_MAGIC_NUMBER;
        b3Printf("Shared memory succesfully allocated\n");
    } else
    {
        b3Error("Couldn't allocated shared memory, is it implemented on your operating system?\n");
    }
}

void	PhysicsServer::stepSimulation(float deltaTime)
{
    
    if (m_testBlock1)
    {
        ///we ignore overflow of integer for now
        if (m_testBlock1->m_numClientCommands> m_testBlock1->m_numProcessedClientCommands)
        {
            
            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            btAssert(m_testBlock1->m_numClientCommands==m_testBlock1->m_numProcessedClientCommands+1);
            
            //consume the command
            switch (m_testBlock1->m_clientCommands[0])
            {
                case CMD_LOAD_URDF:
                {
                    b3Printf("Processed CMD_LOAD_URDF");
                    
                    //load the actual URDF and send a report: completed or failed

                    m_testBlock1->m_serverCommands[0] =CMD_URDF_LOADING_COMPLETED;
                    m_testBlock1->m_numServerCommands++;
                    
                    //CMD_URDF_LOADING_COMPLETED,
                    //CMD_URDF_LOADING_FAILED,
                }
                default:
                {
                
                }
            };
            
            m_testBlock1->m_numProcessedClientCommands++;
            
            //process the command right now
            
        }
    }
}


class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsServer(options.m_guiHelper);
}


