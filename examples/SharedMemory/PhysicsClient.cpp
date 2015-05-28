
#include "PhysicsClient.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "PosixSharedMemory.h"

class PhysicsClient : public CommonMultiBodyBase
{
	SharedMemoryInterface* m_sharedMemory;
	SharedMemoryExampleData*   m_testBlock1;
    
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
    
};

PhysicsClient::PhysicsClient(GUIHelperInterface* helper)
:CommonMultiBodyBase(helper),
m_testBlock1(0)
{
	b3Printf("Started PhysicsClient");
	m_sharedMemory = new PosixSharedMemory();
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
        btAssert(m_testBlock1->m_magicId == SHARED_MEMORY_MAGIC_NUMBER);
    }
    
}

void	PhysicsClient::stepSimulation(float deltaTime)
{
    
    static int once = true;
    
    if (m_testBlock1)
    {
        if (once)
        {
            once=false;
        
            b3Printf("Client created CMD_LOAD_URDF");
            m_testBlock1->m_clientCommands[0] =CMD_LOAD_URDF;
            m_testBlock1->m_numClientCommands++;
        }
    }

}


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsClient(options.m_guiHelper);
}
