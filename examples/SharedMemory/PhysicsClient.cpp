
#include "PhysicsClient.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "SharedMemoryCommon.h"

class PhysicsClient : public SharedMemoryCommon
{
	SharedMemoryInterface* m_sharedMemory;
	SharedMemoryExampleData*   m_testBlock1;
    int m_counter;
    bool m_wantsTermination;
    
public:
    
	PhysicsClient(GUIHelperInterface* helper);
	virtual ~PhysicsClient();
    
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
    
    virtual bool wantsTermination()
    {
        return m_wantsTermination;
    }
};

PhysicsClient::PhysicsClient(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_testBlock1(0),
m_counter(0),
m_wantsTermination(false)
{
	b3Printf("Started PhysicsClient");
#ifdef _WIN32
	m_sharedMemory = new Win32SharedMemoryClient();
#else
	m_sharedMemory = new PosixSharedMemory();
#endif
}

PhysicsClient::~PhysicsClient()
{
    b3Printf("~PhysicsClient");
    m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
	delete m_sharedMemory;
}

void	PhysicsClient::initPhysics()
{
    m_testBlock1 = (SharedMemoryExampleData*)m_sharedMemory->allocateSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    if (m_testBlock1)
    {
     //   btAssert(m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER);
        if (m_testBlock1->m_magicId !=SHARED_MEMORY_MAGIC_NUMBER)
        {
            b3Error("Error: please start server before client");
            m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
            m_testBlock1 = 0;
            
        } else
        {
            //submit a 'load urdf' command to get things started
            
            b3Printf("Client created CMD_LOAD_URDF");
            m_testBlock1->m_clientCommands[0].m_type =CMD_LOAD_URDF;
            sprintf(m_testBlock1->m_clientCommands[0].m_urdfArguments.m_urdfFileName,"r2d2.urdf");
            m_testBlock1->m_numClientCommands++;
        }
    }
    
}

void	PhysicsClient::stepSimulation(float deltaTime)
{
    
    
    if (m_testBlock1)
    {
        //check progress and submit further commands
        //we ignore overflow right now
        
        if (m_testBlock1->m_numServerCommands> m_testBlock1->m_numProcessedServerCommands)
        {
            btAssert(m_testBlock1->m_numServerCommands==m_testBlock1->m_numProcessedServerCommands+1);
        
            const SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
            
            //consume the command
            switch (serverCmd.m_type)
            {

                case CMD_STEP_FORWARD_SIMULATION_COMPLETED:
                case CMD_URDF_LOADING_COMPLETED:
                {
                    //submit a 'step simulation' request
                    
                    if (m_counter<10)
                    {
                        m_testBlock1->m_clientCommands[0].m_type =CMD_STEP_FORWARD_SIMULATION;
                        m_testBlock1->m_clientCommands[0].m_stepSimulationArguments.m_deltaTimeInSeconds = 1./60.;
                        m_testBlock1->m_numClientCommands++;
                        m_counter++;
                        
                    } else
                    {
                        m_wantsTermination = true;
                        m_testBlock1->m_clientCommands[0].m_type =CMD_SHUTDOWN;
                        m_testBlock1->m_numClientCommands++;
                    }
                    
                    break;
                }
                case CMD_URDF_LOADING_FAILED:
                {
                    b3Printf("Server failed loading the URDF...");
                    break;
                }
                default:
                {
                    b3Error("Unknown server command");
                    btAssert(0);
                }
            };
            m_testBlock1->m_numProcessedServerCommands++;
        }
        
    }
    
    

}


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClient(options.m_guiHelper);
}
