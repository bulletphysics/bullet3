
#include "PhysicsServer.h"

#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"

#include "../Importers/ImportURDFDemo/MyURDFImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"

#include "SharedMemoryCommon.h"


class PhysicsServer : public SharedMemoryCommon
{
	SharedMemoryInterface* m_sharedMemory;
    SharedMemoryExampleData* m_testBlock1;
	bool m_wantsShutdown;

public:
    
	PhysicsServer(GUIHelperInterface* helper);
    
	virtual ~PhysicsServer();
    
	virtual void	initPhysics();
    
	virtual void	stepSimulation(float deltaTime);
    
    void releaseSharedMemory();
    
    bool loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                  bool useMultiBody, bool useFixedBase);
    
	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
    
    virtual bool wantsTermination();
};

PhysicsServer::PhysicsServer(GUIHelperInterface* helper)
:SharedMemoryCommon(helper),
m_testBlock1(0),
m_wantsShutdown(false)
{
	b3Printf("Started PhysicsServer\n");
	bool useServer = true;
	#ifdef _WIN32
	m_sharedMemory = new Win32SharedMemoryServer();
	#else
	m_sharedMemory = new PosixSharedMemory();
	#endif
}

void PhysicsServer::releaseSharedMemory()
{
    if (m_testBlock1)
    {
        m_testBlock1->m_magicId = 0;
        b3Printf("magic id = %d\n",m_testBlock1->m_magicId);
        btAssert(m_sharedMemory);
        m_sharedMemory->releaseSharedMemory(SHARED_MEMORY_KEY, SHARED_MEMORY_SIZE);
    }
    if (m_sharedMemory)
    {

        delete m_sharedMemory;
        m_sharedMemory = 0;
        m_testBlock1 = 0;
    }
}

PhysicsServer::~PhysicsServer()
{
    releaseSharedMemory();
}

void	PhysicsServer::initPhysics()
{
    createEmptyDynamicsWorld();
    
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


bool PhysicsServer::wantsTermination()
{
    return m_wantsShutdown;
}

bool PhysicsServer::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase)
{
 
    MyURDFImporter u2b(m_guiHelper);
    bool loadOk =  u2b.loadURDF(fileName);
    if (loadOk)
    {
        b3Printf("loaded %s OK!", fileName);
        
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(pos);
        tr.setRotation(orn);
        int rootLinkIndex = u2b.getRootLinkIndex();
        //                      printf("urdf root link index = %d\n",rootLinkIndex);
        MyMultiBodyCreator creation(m_guiHelper);
        
        ConvertURDF2Bullet(u2b,creation, tr,m_dynamicsWorld,useMultiBody,u2b.getPathPrefix());
        btMultiBody* mb = creation.getBulletMultiBody();

        return true;
    }
    
    return false;
}

void	PhysicsServer::stepSimulation(float deltaTime)
{

    bool wantsShutdown = false;
    
    if (m_testBlock1)
    {
        ///we ignore overflow of integer for now
        if (m_testBlock1->m_numClientCommands> m_testBlock1->m_numProcessedClientCommands)
        {
            
            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            btAssert(m_testBlock1->m_numClientCommands==m_testBlock1->m_numProcessedClientCommands+1);
            
            const SharedMemoryCommand& clientCmd =m_testBlock1->m_clientCommands[0];
             m_testBlock1->m_numProcessedClientCommands++;
            
            //consume the command
            switch (clientCmd.m_type)
            {
                case CMD_LOAD_URDF:
                {
                    b3Printf("Processed CMD_LOAD_URDF:%s",clientCmd.m_urdfArguments.m_urdfFileName);
                    
                    //load the actual URDF and send a report: completed or failed

                    
                    bool completedOk = loadUrdf(clientCmd.m_urdfArguments.m_urdfFileName,
                                               btVector3(0,0,0), btQuaternion(0,0,0,1),true,true );
                    SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
 
                    if (completedOk)
                    {
                        serverCmd.m_type =CMD_URDF_LOADING_COMPLETED;
                    } else
                    {
                        serverCmd.m_type =CMD_URDF_LOADING_FAILED;
                    
                    }
                    m_testBlock1->m_numServerCommands++;
                    break;
                }
                case CMD_STEP_FORWARD_SIMULATION:
                {
                   
                    b3Printf("Step simulation request");
                    double timeStep = clientCmd.m_stepSimulationArguments.m_deltaTimeInSeconds;
                    m_dynamicsWorld->stepSimulation(timeStep);
                    
                    SharedMemoryCommand& serverCmd =m_testBlock1->m_serverCommands[0];
                    
                    serverCmd.m_type =CMD_STEP_FORWARD_SIMULATION_COMPLETED;
                    m_testBlock1->m_numServerCommands++;
 
                    break;
                }
                case CMD_SHUTDOWN:
                {
                    wantsShutdown = true;
                    break;
                }
                default:
                {
                    b3Error("Unsupported command encountered");
                    btAssert(0);
                }
            };
            
           
            
            //process the command right now
            
        }
    }
    if (wantsShutdown)
    {
        m_wantsShutdown = true;
        releaseSharedMemory();
    }
}


class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{
    return new PhysicsServer(options.m_guiHelper);
}


