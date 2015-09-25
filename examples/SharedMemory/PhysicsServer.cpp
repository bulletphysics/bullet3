#include "PhysicsServer.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btTransform.h"

#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "LinearMath/btSerializer.h"
#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryBlock.h"

struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btDefaultSerializer* m_memSerializer;

	UrdfLinkNameMapUtil():m_mb(0),m_memSerializer(0)
	{
	}
};


struct SharedMemoryDebugDrawer : public btIDebugDraw
{

	int m_debugMode;
	btAlignedObjectArray<SharedMemLines> m_lines2;

	SharedMemoryDebugDrawer ()
		:m_debugMode(0)
	{
	}
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{
	}

	virtual void	reportErrorWarning(const char* warningString)
	{
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}
	
	virtual void	setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}
	
	virtual int		getDebugMode() const
	{
		return m_debugMode;
	}
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		SharedMemLines line;
		line.m_from = from;
		line.m_to = to;
		line.m_color = color;
		m_lines2.push_back(line);
	}
};
struct PhysicsServerInternalData
{
	SharedMemoryInterface* m_sharedMemory;
    SharedMemoryBlock* m_testBlock1;
	bool m_isConnected;
	btScalar m_physicsDeltaTime;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_multiBodyJointFeedbacks;

	btAlignedObjectArray<btBulletWorldImporter*> m_worldImporters;
	btAlignedObjectArray<UrdfLinkNameMapUtil*> m_urdfLinkNameMapper;
	btHashMap<btHashInt, btMultiBodyJointMotor*>	m_multiBodyJointMotorMap;
	btAlignedObjectArray<std::string*> m_strings;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btMultiBodyConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	SharedMemoryDebugDrawer*		m_debugDrawer;

    btTransform m_rootLocalInertialFrame;

    
	struct GUIHelperInterface* m_guiHelper;
	int m_sharedMemoryKey;

	bool m_verboseOutput;
	
	
	//data for picking objects
	class btRigidBody*	m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	class btMultiBodyPoint2Point*		m_pickingMultiBodyPoint2Point;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	bool m_prevCanSleep;

	PhysicsServerInternalData()
		:m_sharedMemory(0),
		m_testBlock1(0),
		m_isConnected(false),
		m_physicsDeltaTime(1./240.),
		m_dynamicsWorld(0),
		m_debugDrawer(0),
		m_guiHelper(0),
		m_sharedMemoryKey(SHARED_MEMORY_KEY),
		m_verboseOutput(false),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_pickingMultiBodyPoint2Point(0)
	{
        m_rootLocalInertialFrame.setIdentity();
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
	m_data = new PhysicsServerInternalData();

#ifdef _WIN32
	m_data->m_sharedMemory = new Win32SharedMemoryServer();
#else
	m_data->m_sharedMemory = new PosixSharedMemory();
#endif
	
	createEmptyDynamicsWorld();


}

PhysicsServerSharedMemory::~PhysicsServerSharedMemory()
{
	deleteDynamicsWorld();
	delete m_data;
}

void PhysicsServerSharedMemory::setSharedMemoryKey(int key)
{
	m_data->m_sharedMemoryKey = key;
}


void PhysicsServerSharedMemory::createEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_data->m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();
	
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_data->m_dispatcher = new	btCollisionDispatcher(m_data->m_collisionConfiguration);
	
	m_data->m_broadphase = new btDbvtBroadphase();
	
	m_data->m_solver = new btMultiBodyConstraintSolver;
	
	m_data->m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);

	m_data->m_debugDrawer = new SharedMemoryDebugDrawer();
	m_data->m_dynamicsWorld->setDebugDrawer(m_data->m_debugDrawer);
	
	m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
}

void PhysicsServerSharedMemory::deleteDynamicsWorld()
{
	
	for (int i=0;i<m_data->m_multiBodyJointFeedbacks.size();i++)
	{
		delete m_data->m_multiBodyJointFeedbacks[i];
	}
	m_data->m_multiBodyJointFeedbacks.clear();
	
	
	for (int i=0;i<m_data->m_worldImporters.size();i++)
	{
		delete m_data->m_worldImporters[i];
	}
	m_data->m_worldImporters.clear();
	
	for (int i=0;i<m_data->m_urdfLinkNameMapper.size();i++)
	{
		delete m_data->m_urdfLinkNameMapper[i];
	}
	m_data->m_urdfLinkNameMapper.clear();
	
	m_data->m_multiBodyJointMotorMap.clear();
	
	for (int i=0;i<m_data->m_strings.size();i++)
	{
		delete m_data->m_strings[i];
	}
	m_data->m_strings.clear();

	
	if (m_data->m_dynamicsWorld)
	{
		
		int i;
		for (i = m_data->m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
		{
			m_data->m_dynamicsWorld->removeConstraint(m_data->m_dynamicsWorld->getConstraint(i));
		}
		for (i = m_data->m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_data->m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_data->m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}
	}
	//delete collision shapes
	for (int j = 0; j<m_data->m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_data->m_collisionShapes[j];
		delete shape;
	}
	m_data->m_collisionShapes.clear();

	delete m_data->m_dynamicsWorld;
	m_data->m_dynamicsWorld=0;

	delete m_data->m_debugDrawer;
	m_data->m_debugDrawer=0;

	delete m_data->m_solver;
	m_data->m_solver=0;

	delete m_data->m_broadphase;
	m_data->m_broadphase=0;

	delete m_data->m_dispatcher;
	m_data->m_dispatcher=0;

	delete m_data->m_collisionConfiguration;
	m_data->m_collisionConfiguration=0;

}

bool PhysicsServerSharedMemory::connectSharedMemory( struct GUIHelperInterface* guiHelper)
{
	m_data->m_guiHelper = guiHelper;
	
	bool allowCreation = true;
	bool allowConnectToExistingSharedMemory = false;

    if (m_data->m_isConnected)
    {
        b3Warning("connectSharedMemory, while already connected");
        return m_data->m_isConnected;
    }
    
    
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
			b3Error("Server cannot connect to existing shared memory, disconnecting shared memory.\n");
            m_data->m_sharedMemory->releaseSharedMemory(m_data->m_sharedMemoryKey, SHARED_MEMORY_SIZE);
            m_data->m_testBlock1 = 0;
            m_data->m_isConnected = false;
		}
    } else
	{
		b3Error("Cannot connect to shared memory");
		m_data->m_isConnected = false;
	}
	return m_data->m_isConnected;
}


void PhysicsServerSharedMemory::disconnectSharedMemory(bool deInitializeSharedMemory)
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


bool PhysicsServerSharedMemory::supportsJointMotor(btMultiBody* mb, int mbLinkIndex)
{
	bool canHaveMotor = (mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::eRevolute
			||mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::ePrismatic);
	return canHaveMotor;

}

//for testing, create joint motors for revolute and prismatic joints
void	PhysicsServerSharedMemory::createJointMotors(btMultiBody* mb)
{
	int numLinks = mb->getNumLinks();
	for (int i=0;i<numLinks;i++)
	{
		int mbLinkIndex = i;
							
		if (supportsJointMotor(mb,mbLinkIndex))
		{
			float maxMotorImpulse = 0.f;
			int dof = 0;
			btScalar desiredVelocity = 0.f;
			btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb,mbLinkIndex,dof,desiredVelocity,maxMotorImpulse);
			//motor->setMaxAppliedImpulse(0);
			m_data->m_multiBodyJointMotorMap.insert(mbLinkIndex,motor);
			m_data->m_dynamicsWorld->addMultiBodyConstraint(motor);
		}

	}
}



bool PhysicsServerSharedMemory::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase)
{
	btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadUrdf: No valid m_dynamicsWorld");
		return false;
	}

    BulletURDFImporter u2b(m_data->m_guiHelper);

   
    bool loadOk =  u2b.loadURDF(fileName, useFixedBase);
    if (loadOk)
    {
        {
            btScalar mass = 0;
            m_data->m_rootLocalInertialFrame.setIdentity();
            btVector3 localInertiaDiagonal(0,0,0);
            int urdfLinkIndex = u2b.getRootLinkIndex();
            u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,m_data->m_rootLocalInertialFrame);
        }
		if (m_data->m_verboseOutput)
		{
			b3Printf("loaded %s OK!", fileName);
		}
        
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(pos);
        tr.setRotation(orn);
        //int rootLinkIndex = u2b.getRootLinkIndex();
        //                      printf("urdf root link index = %d\n",rootLinkIndex);
		MyMultiBodyCreator creation(m_data->m_guiHelper);
        
        ConvertURDF2Bullet(u2b,creation, tr,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix());
        btMultiBody* mb = creation.getBulletMultiBody();
		if (useMultiBody)
		{
			if (mb)
			{
				createJointMotors(mb);

				//serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire
			    UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
			    m_data->m_urdfLinkNameMapper.push_back(util);
			    util->m_mb = mb;
			    util->m_memSerializer = new btDefaultSerializer(SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE,(unsigned char*)m_data->m_testBlock1->m_bulletStreamDataServerToClient);
			    //disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
				util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(),0);

			    for (int i=0;i<mb->getNumLinks();i++)
                {
					//disable serialization of the collision objects
                   util->m_memSerializer->m_skipPointers.insert(mb->getLink(i).m_collider,0);
				   int urdfLinkIndex = creation.m_mb2urdfLink[i];
				   std::string* linkName = new std::string(u2b.getLinkName(urdfLinkIndex).c_str());
				   m_data->m_strings.push_back(linkName);
				   util->m_memSerializer->registerNameForPointer(linkName->c_str(),linkName->c_str());
				   mb->getLink(i).m_linkName = linkName->c_str();

				   std::string* jointName = new std::string(u2b.getJointName(urdfLinkIndex).c_str());
				   m_data->m_strings.push_back(jointName);
				   util->m_memSerializer->registerNameForPointer(jointName->c_str(),jointName->c_str());
				   mb->getLink(i).m_jointName = jointName->c_str();
                }
				
				std::string* baseName = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
				m_data->m_strings.push_back(baseName);

				
				util->m_memSerializer->registerNameForPointer(baseName->c_str(),baseName->c_str());
				mb->setBaseName(baseName->c_str());
				
				
                util->m_memSerializer->insertHeader();
                
                int len = mb->calculateSerializeBufferSize();
                btChunk* chunk = util->m_memSerializer->allocate(len,1);
                const char* structType = mb->serialize(chunk->m_oldPtr, util->m_memSerializer);
                util->m_memSerializer->finalizeChunk(chunk,structType,BT_MULTIBODY_CODE,mb);
                
				return true;
			} else
			{
				b3Warning("No multibody loaded from URDF. Could add btRigidBody+btTypedConstraint solution later.");
				return false;
			}
		} else
		{
			btAssert(0);
			
			return true;
		}
        
    }
    
    return false;
}







void PhysicsServerSharedMemory::processClientCommands()
{
	if (m_data->m_isConnected && m_data->m_testBlock1)
    {
        ///we ignore overflow of integer for now
        if (m_data->m_testBlock1->m_numClientCommands> m_data->m_testBlock1->m_numProcessedClientCommands)
        {
            
            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            btAssert(m_data->m_testBlock1->m_numClientCommands==m_data->m_testBlock1->m_numProcessedClientCommands+1);
            
			const SharedMemoryCommand& clientCmd =m_data->m_testBlock1->m_clientCommands[0];
			m_data->m_testBlock1->m_numProcessedClientCommands++;
			//no timestamp yet
            int timeStamp = 0;

            //consume the command
			switch (clientCmd.m_type)
            {
				case CMD_SEND_BULLET_DATA_STREAM:
                {
					if (m_data->m_verboseOutput)
					{
						b3Printf("Processed CMD_SEND_BULLET_DATA_STREAM length %d",clientCmd.m_dataStreamArguments.m_streamChunkLength);
					}
					
					btBulletWorldImporter* worldImporter = new btBulletWorldImporter(m_data->m_dynamicsWorld);
					m_data->m_worldImporters.push_back(worldImporter);
					bool completedOk = worldImporter->loadFileFromMemory(m_data->m_testBlock1->m_bulletStreamDataClientToServer,clientCmd.m_dataStreamArguments.m_streamChunkLength);
					
                    if (completedOk)
                    {
						SharedMemoryStatus& status = m_data->createServerStatus(CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
						m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
						m_data->submitServerStatus(status);
                    } else
                    {
						SharedMemoryStatus& status = m_data->createServerStatus(CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,clientCmd.m_sequenceNumber,timeStamp);
                        m_data->submitServerStatus(status);
                    }
                    
					break;
				}
				case CMD_REQUEST_DEBUG_LINES:
					{
						int curFlags =m_data->m_debugDrawer->getDebugMode();
                        
                        int debugMode = clientCmd.m_requestDebugLinesArguments.m_debugMode;//clientCmd.btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb;
                        int startingLineIndex = clientCmd.m_requestDebugLinesArguments.m_startingLineIndex;
                        if (startingLineIndex<0)
                        {
                            b3Warning("startingLineIndex should be non-negative");
                            startingLineIndex = 0;
                        }
                        
                        if (clientCmd.m_requestDebugLinesArguments.m_startingLineIndex==0)
                        {
                            m_data->m_debugDrawer->m_lines2.resize(0);
                            //|btIDebugDraw::DBG_DrawAabb|
                            //	btIDebugDraw::DBG_DrawConstraints |btIDebugDraw::DBG_DrawConstraintLimits ;
                            m_data->m_debugDrawer->setDebugMode(debugMode);
                            m_data->m_dynamicsWorld->debugDrawWorld();
                            m_data->m_debugDrawer->setDebugMode(curFlags);
                        }

                        //9 floats per line: 3 floats for 'from', 3 floats for 'to' and 3 floats for 'color'
                        int maxNumLines = SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE/(sizeof(float)*9)-1;
                        if (startingLineIndex >m_data->m_debugDrawer->m_lines2.size())
                        {
                            b3Warning("m_startingLineIndex exceeds total number of debug lines");
                            startingLineIndex =m_data->m_debugDrawer->m_lines2.size();
                        }
                        
                        int numLines = btMin(maxNumLines,m_data->m_debugDrawer->m_lines2.size()-startingLineIndex);
                        
                        if (numLines)
                        {

							float* linesFrom = (float*)&m_data->m_testBlock1->m_bulletStreamDataServerToClient[0];
							float* linesTo = (float*)(&m_data->m_testBlock1->m_bulletStreamDataServerToClient[0]+numLines*3*sizeof(float));
							float* linesColor = (float*)(&m_data->m_testBlock1->m_bulletStreamDataServerToClient[0]+2*numLines*3*sizeof(float));

							for (int i=0;i<numLines;i++)
							{
								linesFrom[i*3] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_from.x();
								linesTo[i*3] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_to.x();
								linesColor[i*3] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_color.x();

								linesFrom[i*3+1] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_from.y();
								linesTo[i*3+1] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_to.y();
								linesColor[i*3+1] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_color.y();

								linesFrom[i*3+2] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_from.z();
								linesTo[i*3+2] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_to.z();
								linesColor[i*3+2] = m_data->m_debugDrawer->m_lines2[i+startingLineIndex].m_color.z();
                            }
						}
                        
                        SharedMemoryStatus& status = m_data->createServerStatus(CMD_DEBUG_LINES_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
                        status.m_sendDebugLinesArgs.m_numDebugLines = numLines;
                        status.m_sendDebugLinesArgs.m_startingLineIndex = startingLineIndex;
                        status.m_sendDebugLinesArgs.m_numRemainingDebugLines = m_data->m_debugDrawer->m_lines2.size()-(startingLineIndex+numLines);
                        m_data->submitServerStatus(status);
                        
						break;
					}
                case CMD_LOAD_URDF:
                {
					//at the moment, we only load 1 urdf / robot
					if (m_data->m_urdfLinkNameMapper.size())
					{
						SharedMemoryStatus& status = m_data->createServerStatus(CMD_URDF_LOADING_FAILED,clientCmd.m_sequenceNumber,timeStamp);
						m_data->submitServerStatus(status);
						break;
					}
                    const UrdfArgs& urdfArgs = clientCmd.m_urdfArguments;
					if (m_data->m_verboseOutput)
					{
						b3Printf("Processed CMD_LOAD_URDF:%s", urdfArgs.m_urdfFileName);
					}
					btAssert((clientCmd.m_updateFlags&URDF_ARGS_FILE_NAME) !=0);
					btAssert(urdfArgs.m_urdfFileName);
					btVector3 initialPos(0,0,0);
					btQuaternion initialOrn(0,0,0,1);
					if (clientCmd.m_updateFlags & URDF_ARGS_INITIAL_POSITION)
					{
						initialPos[0] = urdfArgs.m_initialPosition[0];
						initialPos[1] = urdfArgs.m_initialPosition[1];
						initialPos[2] = urdfArgs.m_initialPosition[2];
					}
					if (clientCmd.m_updateFlags & URDF_ARGS_INITIAL_ORIENTATION)
					{
						initialOrn[0] = urdfArgs.m_initialOrientation[0];
						initialOrn[1] = urdfArgs.m_initialOrientation[1];
						initialOrn[2] = urdfArgs.m_initialOrientation[2];
						initialOrn[3] = urdfArgs.m_initialOrientation[3];
					}
					bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? urdfArgs.m_useMultiBody : true;
					bool useFixedBase = (clientCmd.m_updateFlags & URDF_ARGS_USE_FIXED_BASE) ? urdfArgs.m_useFixedBase: false;

                    //load the actual URDF and send a report: completed or failed
                    bool completedOk = loadUrdf(urdfArgs.m_urdfFileName,
                                               initialPos,initialOrn,
                                               useMultiBody, useFixedBase);
                    SharedMemoryStatus& serverCmd =m_data->m_testBlock1->m_serverCommands[0];
 
                    if (completedOk)
                    {
						if (m_data->m_urdfLinkNameMapper.size())
						{
							serverCmd.m_dataStreamArguments.m_streamChunkLength = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
						}
						m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
						SharedMemoryStatus& status = m_data->createServerStatus(CMD_URDF_LOADING_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
						m_data->submitServerStatus(status);
                        
                    } else
                    {
						SharedMemoryStatus& status = m_data->createServerStatus(CMD_URDF_LOADING_FAILED,clientCmd.m_sequenceNumber,timeStamp);
						m_data->submitServerStatus(status);
                    }
                    
                    

                    
                    break;
                }
                case CMD_CREATE_SENSOR:
                {
					if (m_data->m_verboseOutput)
					{
						b3Printf("Processed CMD_CREATE_SENSOR");
					}
                    
                    if (m_data->m_dynamicsWorld->getNumMultibodies()>0)
                    {
                        btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(0);
                        btAssert(mb);
                        for (int i=0;i<clientCmd.m_createSensorArguments.m_numJointSensorChanges;i++)
                        {
                            int jointIndex = clientCmd.m_createSensorArguments.m_jointIndex[i];
                            if (clientCmd.m_createSensorArguments.m_enableJointForceSensor[i])
                            {
                               if (mb->getLink(jointIndex).m_jointFeedback)
                               {
                                   b3Warning("CMD_CREATE_SENSOR: sensor for joint [%d] already enabled", jointIndex);
                               } else
                               {
                                   btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
                                   fb->m_reactionForces.setZero();
                                   mb->getLink(jointIndex).m_jointFeedback = fb;
                                   m_data->m_multiBodyJointFeedbacks.push_back(fb);
                               };
                                
                            } else
                            {
                                if (mb->getLink(jointIndex).m_jointFeedback)
                                {
                                    m_data->m_multiBodyJointFeedbacks.remove(mb->getLink(jointIndex).m_jointFeedback);
                                    delete mb->getLink(jointIndex).m_jointFeedback;
                                    mb->getLink(jointIndex).m_jointFeedback=0;
                                } else
                                {
                                     b3Warning("CMD_CREATE_SENSOR: cannot perform sensor removal request, no sensor on joint [%d]", jointIndex);
                                };

                            }
                        }
                        
                    } else
                    {
                        b3Warning("No btMultiBody in the world. btRigidBody/btTypedConstraint sensor not hooked up yet");
                    }

#if 0
                    //todo(erwincoumans) here is some sample code to hook up a force/torque sensor for btTypedConstraint/btRigidBody
                    /*
                     for (int i=0;i<m_data->m_dynamicsWorld->getNumConstraints();i++)
                     {
                     btTypedConstraint* c = m_data->m_dynamicsWorld->getConstraint(i);
                     btJointFeedback* fb = new btJointFeedback();
                     m_data->m_jointFeedbacks.push_back(fb);
                     c->setJointFeedback(fb);
                     
                     
                     }
                     */
#endif
                   
                    SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);
                    break;
                }
                case CMD_SEND_DESIRED_STATE:
                    {
						if (m_data->m_verboseOutput)
						{
                            b3Printf("Processed CMD_SEND_DESIRED_STATE");
						}
                            if (m_data->m_dynamicsWorld->getNumMultibodies()>0)
                            {
                                btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(0);
                                btAssert(mb);
                                
                                switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
                                {
                                case CONTROL_MODE_TORQUE:
                                    {
										if (m_data->m_verboseOutput)
										{
											b3Printf("Using CONTROL_MODE_TORQUE");
										}
                                        mb->clearForcesAndTorques();
                                        
                                        int torqueIndex = 0;
										btVector3 f(clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[0],
                                                    clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[1],
                                                    clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[2]);
                                        btVector3 t(clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[3],
                                                    clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[4],
                                                    clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[5]);
                                        torqueIndex+=6;
                                        mb->addBaseForce(f);
                                        mb->addBaseTorque(t);
                                        for (int link=0;link<mb->getNumLinks();link++)
                                        {
                                            
                                            for (int dof=0;dof<mb->getLink(link).m_dofCount;dof++)
                                            {               
                                                double torque = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[torqueIndex];
                                                mb->addJointTorqueMultiDof(link,dof,torque);
                                                torqueIndex++;
                                            }
                                        }
                                        break;
                                    }
								case CONTROL_MODE_VELOCITY:
									{
										if (m_data->m_verboseOutput)
										{
											b3Printf("Using CONTROL_MODE_VELOCITY");
										}
                                        
										int numMotors = 0;
										//find the joint motors and apply the desired velocity and maximum force/torque
										if (m_data->m_dynamicsWorld->getNumMultibodies()>0)
										{
											btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(0);
											int dofIndex = 6;//skip the 3 linear + 3 angular degree of freedom entries of the base
											for (int link=0;link<mb->getNumLinks();link++)
											{
												if (supportsJointMotor(mb,link))
												{
													
													btMultiBodyJointMotor** motorPtr  = m_data->m_multiBodyJointMotorMap[link];
													if (motorPtr)
													{
														btMultiBodyJointMotor* motor = *motorPtr;
														btScalar desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex];
														motor->setVelocityTarget(desiredVelocity);

														btScalar maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex]*m_data->m_physicsDeltaTime;
														motor->setMaxAppliedImpulse(maxImp);
														numMotors++;

													}
												}
												dofIndex += mb->getLink(link).m_dofCount;
											}
										}
										break;
									}
								case CONTROL_MODE_POSITION_VELOCITY_PD:
									{
										if (m_data->m_verboseOutput)
										{
											b3Printf("Using CONTROL_MODE_POSITION_VELOCITY_PD");
										}
										//compute the force base on PD control
										 mb->clearForcesAndTorques();

										int numMotors = 0;
										//find the joint motors and apply the desired velocity and maximum force/torque
										if (m_data->m_dynamicsWorld->getNumMultibodies()>0)
										{
											btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(0);
											int velIndex = 6;//skip the 3 linear + 3 angular degree of freedom velocity entries of the base
											int posIndex = 7;//skip 3 positional and 4 orientation (quaternion) positional degrees of freedom of the base
											for (int link=0;link<mb->getNumLinks();link++)
											{
												if (supportsJointMotor(mb,link))
												{
													
													btMultiBodyJointMotor** motorPtr  = m_data->m_multiBodyJointMotorMap[link];
													if (motorPtr)
													{
														btMultiBodyJointMotor* motor = *motorPtr;
													
                                                        btScalar desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
                                                        btScalar desiredPosition = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex];
                                                        
                                                        btScalar kp = clientCmd.m_sendDesiredStateCommandArgument.m_Kp[velIndex];
                                                        btScalar kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[velIndex];

                                                        int dof1 = 0;
                                                        btScalar currentPosition = mb->getJointPosMultiDof(link)[dof1];
                                                        btScalar currentVelocity = mb->getJointVelMultiDof(link)[dof1];
                                                        btScalar positionStabiliationTerm = (desiredPosition-currentPosition)/m_data->m_physicsDeltaTime;
                                                        btScalar velocityError = (desiredVelocity - currentVelocity);
                                                        
                                                        desiredVelocity =   kp * positionStabiliationTerm +
                                                                            kd * velocityError;
                                                        
                                                        motor->setVelocityTarget(desiredVelocity);
                                                        
                                                        btScalar maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex]*m_data->m_physicsDeltaTime;
                                                        
                                                        motor->setMaxAppliedImpulse(1000);//maxImp);
                                                        numMotors++;
                                                    }

												}
												velIndex += mb->getLink(link).m_dofCount;
												posIndex += mb->getLink(link).m_posVarCount;
											}
										}

										break;
									}
                                default:
                                    {
                                        b3Warning("m_controlMode not implemented yet");
                                        break;
                                    }
                                    
                                }
                            }
                            
							SharedMemoryStatus& status = m_data->createServerStatus(CMD_DESIRED_STATE_RECEIVED_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
							m_data->submitServerStatus(status);
                        break;
                    }
				case CMD_REQUEST_ACTUAL_STATE:
					{
						if (m_data->m_verboseOutput)
						{
							b3Printf("Sending the actual state (Q,U)");
						}
						if (m_data->m_dynamicsWorld->getNumMultibodies()>0)
						{
							btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(0);
							SharedMemoryStatus& serverCmd = m_data->createServerStatus(CMD_ACTUAL_STATE_UPDATE_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);

							serverCmd.m_sendActualStateArgs.m_bodyUniqueId = 0;
							int totalDegreeOfFreedomQ = 0;
							int totalDegreeOfFreedomU = 0; 
							
							//always add the base, even for static (non-moving objects)
							//so that we can easily move the 'fixed' base when needed
							//do we don't use this conditional "if (!mb->hasFixedBase())"
							{
								btTransform tr;
								tr.setOrigin(mb->getBasePos());
								tr.setRotation(mb->getWorldToBaseRot().inverse());
								
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[0] =
                                    m_data->m_rootLocalInertialFrame.getOrigin()[0];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[1] =
                                    m_data->m_rootLocalInertialFrame.getOrigin()[1];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[2] =
                                    m_data->m_rootLocalInertialFrame.getOrigin()[2];

                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[3] =
                                    m_data->m_rootLocalInertialFrame.getRotation()[0];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[4] =
                                    m_data->m_rootLocalInertialFrame.getRotation()[1];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[5] =
                                    m_data->m_rootLocalInertialFrame.getRotation()[2];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[6] =
                                    m_data->m_rootLocalInertialFrame.getRotation()[3];


                                
								//base position in world space, carthesian
								serverCmd.m_sendActualStateArgs.m_actualStateQ[0] = tr.getOrigin()[0];
								serverCmd.m_sendActualStateArgs.m_actualStateQ[1] = tr.getOrigin()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQ[2] = tr.getOrigin()[2];

								//base orientation, quaternion x,y,z,w, in world space, carthesian
								serverCmd.m_sendActualStateArgs.m_actualStateQ[3] = tr.getRotation()[0]; 
								serverCmd.m_sendActualStateArgs.m_actualStateQ[4] = tr.getRotation()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQ[5] = tr.getRotation()[2];
								serverCmd.m_sendActualStateArgs.m_actualStateQ[6] = tr.getRotation()[3];
								totalDegreeOfFreedomQ +=7;//pos + quaternion

								//base linear velocity (in world space, carthesian)
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = mb->getBaseVel()[0];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = mb->getBaseVel()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = mb->getBaseVel()[2];
	
								//base angular velocity (in world space, carthesian)
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = mb->getBaseOmega()[0];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = mb->getBaseOmega()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = mb->getBaseOmega()[2];
								totalDegreeOfFreedomU += 6;//3 linear and 3 angular DOF
							}
							for (int l=0;l<mb->getNumLinks();l++)
							{
								for (int d=0;d<mb->getLink(l).m_posVarCount;d++)
								{
									serverCmd.m_sendActualStateArgs.m_actualStateQ[totalDegreeOfFreedomQ++] = mb->getJointPosMultiDof(l)[d];
								}
								for (int d=0;d<mb->getLink(l).m_dofCount;d++)
								{
									serverCmd.m_sendActualStateArgs.m_actualStateQdot[totalDegreeOfFreedomU++] = mb->getJointVelMultiDof(l)[d];
								}
                                
                                if (0 == mb->getLink(l).m_jointFeedback)
                                {
                                    for (int d=0;d<6;d++)
                                    {
                                        serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+d]=0;
                                    }
                                } else
                                {
                                    btVector3 sensedForce = mb->getLink(l).m_jointFeedback->m_reactionForces.getLinear();
                                    btVector3 sensedTorque = mb->getLink(l).m_jointFeedback->m_reactionForces.getAngular();
                                    
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+0] = sensedForce[0];
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+1] = sensedForce[1];
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+2] = sensedForce[2];
                                    
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+3] = sensedTorque[0];
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+4] = sensedTorque[1];
                                    serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+5] = sensedTorque[2];
                                }
							}

							serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
							serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;
							
							m_data->submitServerStatus(serverCmd);
							
						} else
						{

							b3Warning("Request state but no multibody available");
							SharedMemoryStatus& serverCmd = m_data->createServerStatus(CMD_ACTUAL_STATE_UPDATE_FAILED,clientCmd.m_sequenceNumber,timeStamp);
							m_data->submitServerStatus(serverCmd);
						}

						break;
					}
                case CMD_STEP_FORWARD_SIMULATION:
                {
                   
					if (m_data->m_verboseOutput)
					{
						b3Printf("Step simulation request");
					}
                    m_data->m_dynamicsWorld->stepSimulation(m_data->m_physicsDeltaTime,0);
                    
					SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_STEP_FORWARD_SIMULATION_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);

                    break;
                }
					
				case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				{
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DELTA_TIME)
					{
						m_data->m_physicsDeltaTime = clientCmd.m_physSimParamArgs.m_deltaTime;
					}
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_GRAVITY)
					{
						btVector3 grav(clientCmd.m_physSimParamArgs.m_gravityAcceleration[0],
									   clientCmd.m_physSimParamArgs.m_gravityAcceleration[1],
									   clientCmd.m_physSimParamArgs.m_gravityAcceleration[2]);
						this->m_data->m_dynamicsWorld->setGravity(grav);
						if (m_data->m_verboseOutput)
						{
							b3Printf("Updated Gravity: %f,%f,%f",grav[0],grav[1],grav[2]);
						}

					}
					
					
					SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);
					break;
					
				};
				case CMD_INIT_POSE:
				{
					if (m_data->m_verboseOutput)
					{
						b3Printf("Server Init Pose not implemented yet");
					}
					int body_unique_id = clientCmd.m_sendDesiredStateCommandArgument.m_bodyUniqueId;
					if (m_data->m_dynamicsWorld->getNumMultibodies()>body_unique_id)
					{
						btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(body_unique_id);
						mb->setBasePos(btVector3(
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[0],
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[1],
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[2]));
						mb->setWorldToBaseRot(btQuaternion(
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[3],
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[4],
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[5],
								clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[6]));
					}
					
					SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);

					break;
				}
					

                case CMD_RESET_SIMULATION:
                {
					//clean up all data
					if (m_data && m_data->m_guiHelper && m_data->m_guiHelper->getRenderInterface())
                    {
                        m_data->m_guiHelper->getRenderInterface()->removeAllInstances();
                    }
                    deleteDynamicsWorld();
					createEmptyDynamicsWorld();
					
                    SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_RESET_SIMULATION_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);

                    break;
                }
				case CMD_CREATE_BOX_COLLISION_SHAPE:
					{
                        btVector3 halfExtents(1,1,1);
                        if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_HALF_EXTENTS)
                        {
                            halfExtents = btVector3(
                                                  clientCmd.m_createBoxShapeArguments.m_halfExtentsX,
                                                  clientCmd.m_createBoxShapeArguments.m_halfExtentsY,
                                                  clientCmd.m_createBoxShapeArguments.m_halfExtentsZ);
                        }
						btTransform startTrans;
						startTrans.setIdentity();
                        if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_INITIAL_POSITION)
                        {
                            startTrans.setOrigin(btVector3(
                                                 clientCmd.m_createBoxShapeArguments.m_initialPosition[0],
                                                 clientCmd.m_createBoxShapeArguments.m_initialPosition[1],
                                                 clientCmd.m_createBoxShapeArguments.m_initialPosition[2]));
                        }

                        if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_INITIAL_ORIENTATION)
                        {
                            
                            startTrans.setRotation(btQuaternion(
                                                           clientCmd.m_createBoxShapeArguments.m_initialOrientation[0],
                                                                 clientCmd.m_createBoxShapeArguments.m_initialOrientation[1],
                                                                 clientCmd.m_createBoxShapeArguments.m_initialOrientation[2],
                                                                 clientCmd.m_createBoxShapeArguments.m_initialOrientation[3]));
                        }

						btBulletWorldImporter* worldImporter = new btBulletWorldImporter(m_data->m_dynamicsWorld);
						m_data->m_worldImporters.push_back(worldImporter);

						btCollisionShape* shape = worldImporter->createBoxShape(halfExtents);
						btScalar mass = 0.f;
						bool isDynamic = (mass>0);
						worldImporter->createRigidBody(isDynamic,mass,startTrans,shape,0);
						m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
						
						SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,clientCmd.m_sequenceNumber,timeStamp);
						m_data->submitServerStatus(serverCmd);

						break;
					}
                case CMD_PICK_BODY:
                    {
                        pickBody(btVector3(clientCmd.m_pickBodyArguments.m_rayFromWorld[0],
                                           clientCmd.m_pickBodyArguments.m_rayFromWorld[1],
                                           clientCmd.m_pickBodyArguments.m_rayFromWorld[2]),
                                 btVector3(clientCmd.m_pickBodyArguments.m_rayToWorld[0],
                                           clientCmd.m_pickBodyArguments.m_rayToWorld[1],
                                           clientCmd.m_pickBodyArguments.m_rayToWorld[2]));

                        SharedMemoryStatus &serverCmd =
                                m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,
                                                           clientCmd.m_sequenceNumber, timeStamp);
                        m_data->submitServerStatus(serverCmd);
                        break;
                    }
                case CMD_MOVE_PICKED_BODY:
                    {
                        movePickedBody(btVector3(clientCmd.m_pickBodyArguments.m_rayFromWorld[0],
                                                 clientCmd.m_pickBodyArguments.m_rayFromWorld[1],
                                                 clientCmd.m_pickBodyArguments.m_rayFromWorld[2]),
                                       btVector3(clientCmd.m_pickBodyArguments.m_rayToWorld[0],
                                                 clientCmd.m_pickBodyArguments.m_rayToWorld[1],
                                                 clientCmd.m_pickBodyArguments.m_rayToWorld[2]));

                        SharedMemoryStatus &serverCmd =
                                m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,
                                                           clientCmd.m_sequenceNumber, timeStamp);
                        m_data->submitServerStatus(serverCmd);
                        break;
                    }
                case CMD_REMOVE_PICKING_CONSTRAINT_BODY:
                    {
                        removePickingConstraint();

                        SharedMemoryStatus &serverCmd =
                                m_data->createServerStatus(CMD_CLIENT_COMMAND_COMPLETED,
                                                           clientCmd.m_sequenceNumber, timeStamp);
                        m_data->submitServerStatus(serverCmd);
                        break;
                    }
                default:
                {
                    b3Error("Unknown command encountered");
					
					SharedMemoryStatus& serverCmd =m_data->createServerStatus(CMD_UNKNOWN_COMMAND_FLUSHED,clientCmd.m_sequenceNumber,timeStamp);
					m_data->submitServerStatus(serverCmd);

                }
            };
            
        }
    }
}

void PhysicsServerSharedMemory::renderScene()
{
	if (m_data->m_guiHelper)
	{
		m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
		
		m_data->m_guiHelper->render(m_data->m_dynamicsWorld);
	}
	
}

void    PhysicsServerSharedMemory::physicsDebugDraw(int debugDrawFlags)
{
#if 0
	if (m_data->m_dynamicsWorld)
	{
		if (m_data->m_dynamicsWorld->getDebugDrawer())
		{
			//m_data->m_debugDrawer->m_lines.clear();
			//m_data->m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
		}
		m_data->m_dynamicsWorld->debugDrawWorld();
	}
#endif
}


bool PhysicsServerSharedMemory::pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	if (m_data->m_dynamicsWorld==0)
		return false;

	btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

	m_data->m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, rayCallback);
	if (rayCallback.hasHit())
	{

		btVector3 pickPos = rayCallback.m_hitPointWorld;
		btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
		if (body)
		{
			//other exclusions?
			if (!(body->isStaticObject() || body->isKinematicObject()))
			{
				m_data->m_pickedBody = body;
				m_data->m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
				//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
				btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
				btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
				m_data->m_dynamicsWorld->addConstraint(p2p, true);
				m_data->m_pickedConstraint = p2p;
				btScalar mousePickClamping = 30.f;
				p2p->m_setting.m_impulseClamp = mousePickClamping;
				//very weak constraint for picking
				p2p->m_setting.m_tau = 0.001f;
			}
		} else
		{
			btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
			if (multiCol && multiCol->m_multiBody)
			{
						
				m_data->m_prevCanSleep = multiCol->m_multiBody->getCanSleep();
				multiCol->m_multiBody->setCanSleep(false);

				btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

				btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody,multiCol->m_link,0,pivotInA,pickPos);
				//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
				//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
				//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
				//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
				btScalar scaling=1;
				p2p->setMaxAppliedImpulse(2*scaling);
		
				btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
				world->addMultiBodyConstraint(p2p);
				m_data->m_pickingMultiBodyPoint2Point =p2p; 
			}
		}



		//					pickObject(pickPos, rayCallback.m_collisionObject);
		m_data->m_oldPickingPos = rayToWorld;
		m_data->m_hitPos = pickPos;
		m_data->m_oldPickingDist = (pickPos - rayFromWorld).length();
		//					printf("hit !\n");
		//add p2p
	}
	return false;
}
bool PhysicsServerSharedMemory::movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	if (m_data->m_pickedBody  && m_data->m_pickedConstraint)
	{
		btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_data->m_pickedConstraint);
		if (pickCon)
		{
			//keep it at the same picking distance
		
			btVector3 dir = rayToWorld-rayFromWorld;
			dir.normalize();
			dir *= m_data->m_oldPickingDist;

			btVector3 newPivotB = rayFromWorld + dir;
			pickCon->setPivotB(newPivotB);
		}
	}
		
	if (m_data->m_pickingMultiBodyPoint2Point)
	{
		//keep it at the same picking distance

		
		btVector3 dir = rayToWorld-rayFromWorld;
		dir.normalize();
		dir *= m_data->m_oldPickingDist;

		btVector3 newPivotB = rayFromWorld + dir;
			
		m_data->m_pickingMultiBodyPoint2Point->setPivotInB(newPivotB);
	}
		
	return false;
}
void PhysicsServerSharedMemory::removePickingConstraint()
{
	if (m_data->m_pickedConstraint)
	{
		m_data->m_dynamicsWorld->removeConstraint(m_data->m_pickedConstraint);
		delete m_data->m_pickedConstraint;
		m_data->m_pickedConstraint = 0;
		m_data->m_pickedBody = 0;
	}
	if (m_data->m_pickingMultiBodyPoint2Point)
	{
		m_data->m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(m_data->m_prevCanSleep);
		btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
		world->removeMultiBodyConstraint(m_data->m_pickingMultiBodyPoint2Point);
		delete m_data->m_pickingMultiBodyPoint2Point;
		m_data->m_pickingMultiBodyPoint2Point = 0;
	}
}
