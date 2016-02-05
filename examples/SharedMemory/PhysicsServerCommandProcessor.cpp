#include "PhysicsServerCommandProcessor.h"


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
#include "SharedMemoryCommands.h"

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


struct InteralBodyData
{
	btMultiBody* m_multiBody;
	btRigidBody* m_rigidBody;
	int m_testData;

	btTransform m_rootLocalInertialFrame;

	InteralBodyData()
		:m_multiBody(0),
		m_rigidBody(0),
		m_testData(0)
	{
		m_rootLocalInertialFrame.setIdentity();
	}
};

///todo: templatize this
struct InternalBodyHandle : public InteralBodyData
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_nextFreeHandle;
	void SetNextFree(int next) 
	{
		m_nextFreeHandle = next;
	}
	int	GetNextFree() const 
	{
		return m_nextFreeHandle;
	}
};

class btCommandChunk
{
public:
	int		m_chunkCode;
	int		m_length;
	void	*m_oldPtr;
	int		m_dna_nr;
	int		m_number;
};


class bCommandChunkPtr4
{
public:
	bCommandChunkPtr4(){}
	int code;
	int len;
	union
	{
		int m_uniqueInt;
	};
	int dna_nr;
	int nr;
};

// ----------------------------------------------------- //
class bCommandChunkPtr8
{
public:
	bCommandChunkPtr8(){}
	int code,  len;
	union
	{
		int	m_uniqueInts[2];
	};
	int dna_nr, nr;
};



struct CommandLogger
{
	FILE* m_file;
	
	void	writeHeader(unsigned char* buffer) const
	{

#ifdef  BT_USE_DOUBLE_PRECISION
		memcpy(buffer, "BT3CMDd", 7);
#else
		memcpy(buffer, "BT3CMDf", 7);
#endif //BT_USE_DOUBLE_PRECISION

		int littleEndian= 1;
		littleEndian= ((char*)&littleEndian)[0];

		if (sizeof(void*)==8)
		{
			buffer[7] = '-';
		} else
		{
			buffer[7] = '_';
		}

		if (littleEndian)
		{
			buffer[8]='v';
		} else
		{
			buffer[8]='V';
		}

		buffer[9] = 0;
		buffer[10] = 0;
		buffer[11] = 0;
		
		int ver = btGetVersion();
		if (ver>=0 && ver<999)
		{
			sprintf((char*)&buffer[9],"%d",ver);
		}
		
	}
	
	void logCommand(const SharedMemoryCommand& command)
	{
		btCommandChunk chunk;
		chunk.m_chunkCode = command.m_type;
		chunk.m_oldPtr = 0;
		chunk.m_dna_nr = 0;
		chunk.m_length = sizeof(SharedMemoryCommand);
		chunk.m_number = 1;
		fwrite((const char*)&chunk,sizeof(btCommandChunk), 1,m_file);
		fwrite((const char*)&command,sizeof(SharedMemoryCommand),1,m_file);
	}

	CommandLogger(const char* fileName)
	{
		m_file = fopen(fileName,"wb");
		unsigned char buf[15];
		buf[12] = 12;
		buf[13] = 13;
		buf[14] = 14;
		writeHeader(buf);
		fwrite(buf,12,1,m_file);
	}
	virtual ~CommandLogger()
	{
		fclose(m_file);
	}
};


struct CommandLogPlayback
{
	unsigned char m_header[12];
	FILE* m_file;
	bool m_bitsVary;
	bool m_fileIs64bit;


	CommandLogPlayback(const char* fileName)
	{
		m_file = fopen(fileName,"rb");
		if (m_file)
		{
			fread(m_header,12,1,m_file);
		}
		unsigned char c = m_header[7];
		m_fileIs64bit =  (c=='-');
		
		const bool VOID_IS_8 = ((sizeof(void*)==8));
		m_bitsVary = (VOID_IS_8 != m_fileIs64bit);
		


	}
	virtual ~CommandLogPlayback()
	{
		if (m_file)
		{
			fclose(m_file);
			m_file=0;
		}
	}
	bool processNextCommand(SharedMemoryCommand* cmd)
	{
		if (m_file)
		{
			size_t s = 0;
		

			if (m_fileIs64bit)
			{
				bCommandChunkPtr8 chunk8;
				s = fread((void*)&chunk8,sizeof(bCommandChunkPtr8),1,m_file);
			} else
			{
				bCommandChunkPtr4 chunk4;
				s = fread((void*)&chunk4,sizeof(bCommandChunkPtr4),1,m_file);
			}
		
			if (s==1)
			{
				s = fread(cmd,sizeof(SharedMemoryCommand),1,m_file);
				return (s==1);
			}
		}
		return false;

	}
};

struct PhysicsServerCommandProcessorInternalData
{
	///handle management
	btAlignedObjectArray<InternalBodyHandle>	m_bodyHandles;
	int m_numUsedHandles;						// number of active handles
	
	int	m_firstFreeHandle;		// free handles list
	InternalBodyHandle* getHandle(int handle)
	{
		btAssert(handle>=0);
		btAssert(handle<m_bodyHandles.size());
		if ((handle<0) || (handle>=m_bodyHandles.size()))
		{
			return 0;
		}
		return &m_bodyHandles[handle];
	}
	const InternalBodyHandle* getHandle(int handle) const
	{
		return &m_bodyHandles[handle];
	}

	void increaseHandleCapacity(int extraCapacity)
	{
		int curCapacity = m_bodyHandles.size();
		btAssert(curCapacity == m_numUsedHandles);
		int newCapacity = curCapacity + extraCapacity;
		m_bodyHandles.resize(newCapacity);

		{
			for (int i = curCapacity; i < newCapacity; i++)
				m_bodyHandles[i].SetNextFree(i + 1);


			m_bodyHandles[newCapacity - 1].SetNextFree(-1);
		}
		m_firstFreeHandle = curCapacity;
	}
	void initHandles()
	{
		m_numUsedHandles = 0;
		m_firstFreeHandle = -1;
		
		increaseHandleCapacity(1);
	}

	void exitHandles()
	{
		m_bodyHandles.resize(0);
		m_firstFreeHandle = -1;
		m_numUsedHandles = 0;
	}

	int allocHandle()
	{
		btAssert(m_firstFreeHandle>=0);

		int handle = m_firstFreeHandle;
		m_firstFreeHandle = getHandle(handle)->GetNextFree();
		m_numUsedHandles++;

		if (m_firstFreeHandle<0)
		{
			int curCapacity = m_bodyHandles.size();
			int additionalCapacity= m_bodyHandles.size();
			increaseHandleCapacity(additionalCapacity);

			
			getHandle(handle)->SetNextFree(m_firstFreeHandle);
		}
		

		return handle;
	}


	void freeHandle(int handle)
	{
		btAssert(handle >= 0);

		getHandle(handle)->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		m_numUsedHandles--;
	}

	///end handle management
	
	
	CommandLogger* m_commandLogger;
	CommandLogPlayback* m_logPlayback;

	
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
	SharedMemoryDebugDrawer*		m_remoteDebugDrawer;
	
    

    
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

	PhysicsServerCommandProcessorInternalData()
		:
		m_commandLogger(0),
		m_logPlayback(0),
		m_physicsDeltaTime(1./240.),
		m_dynamicsWorld(0),
		m_remoteDebugDrawer(0),
		m_guiHelper(0),
		m_sharedMemoryKey(SHARED_MEMORY_KEY),
		m_verboseOutput(false),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_pickingMultiBodyPoint2Point(0)
	{
        
		initHandles();
#if 0
		btAlignedObjectArray<int> bla;

		for (int i=0;i<1024;i++)
		{
			int handle = allocHandle();
			bla.push_back(handle);
			InternalBodyHandle* body = getHandle(handle);
			InteralBodyData* body2 = body;
		}
		for (int i=0;i<bla.size();i++)
		{
			freeHandle(bla[i]);
		}

		bla.resize(0);

		for (int i=0;i<1024;i++)
		{
			int handle = allocHandle();
			bla.push_back(handle);
			InternalBodyHandle* body = getHandle(handle);
			InteralBodyData* body2 = body;
		}
		for (int i=0;i<bla.size();i++)
		{
			freeHandle(bla[i]);
		}
		bla.resize(0);

		for (int i=0;i<1024;i++)
		{
			int handle = allocHandle();
			bla.push_back(handle);
			InternalBodyHandle* body = getHandle(handle);
			InteralBodyData* body2 = body;
		}
		for (int i=0;i<bla.size();i++)
		{
			freeHandle(bla[i]);
		}
#endif

	}


};

void PhysicsServerCommandProcessor::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
	
	if (guiHelper)
	{
		
		guiHelper->createPhysicsDebugDrawer(m_data->m_dynamicsWorld);
	} else
	{
		if (m_data->m_guiHelper && m_data->m_dynamicsWorld && m_data->m_dynamicsWorld->getDebugDrawer())
		{
			
			m_data->m_dynamicsWorld->setDebugDrawer(0);
		}
	}
	m_data->m_guiHelper = guiHelper;
}


PhysicsServerCommandProcessor::PhysicsServerCommandProcessor()
{
	m_data = new PhysicsServerCommandProcessorInternalData();

	createEmptyDynamicsWorld();
}

PhysicsServerCommandProcessor::~PhysicsServerCommandProcessor()
{
	deleteDynamicsWorld();
	if (m_data->m_commandLogger)
	{
		delete m_data->m_commandLogger;
		m_data->m_commandLogger = 0;
	}

	delete m_data;
}



void PhysicsServerCommandProcessor::createEmptyDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_data->m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();
	
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_data->m_dispatcher = new	btCollisionDispatcher(m_data->m_collisionConfiguration);
	
	m_data->m_broadphase = new btDbvtBroadphase();
	
	m_data->m_solver = new btMultiBodyConstraintSolver;
	
	m_data->m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);

	m_data->m_remoteDebugDrawer = new SharedMemoryDebugDrawer();
	
	
	m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
}

void PhysicsServerCommandProcessor::deleteDynamicsWorld()
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

	delete m_data->m_remoteDebugDrawer;
	m_data->m_remoteDebugDrawer =0;

	delete m_data->m_solver;
	m_data->m_solver=0;

	delete m_data->m_broadphase;
	m_data->m_broadphase=0;

	delete m_data->m_dispatcher;
	m_data->m_dispatcher=0;

	delete m_data->m_collisionConfiguration;
	m_data->m_collisionConfiguration=0;

}




bool PhysicsServerCommandProcessor::supportsJointMotor(btMultiBody* mb, int mbLinkIndex)
{
	bool canHaveMotor = (mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::eRevolute
			||mb->getLink(mbLinkIndex).m_jointType==btMultibodyLink::ePrismatic);
	return canHaveMotor;

}

//for testing, create joint motors for revolute and prismatic joints
void	PhysicsServerCommandProcessor::createJointMotors(btMultiBody* mb)
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



bool PhysicsServerCommandProcessor::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes)
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
		//get a body index
		int bodyUniqueId = m_data->allocHandle();
		if (bodyUniqueIdPtr)
			*bodyUniqueIdPtr= bodyUniqueId;

		InternalBodyHandle* bodyHandle = m_data->getHandle(bodyUniqueId);

        {
            btScalar mass = 0;
            bodyHandle->m_rootLocalInertialFrame.setIdentity();
            btVector3 localInertiaDiagonal(0,0,0);
            int urdfLinkIndex = u2b.getRootLinkIndex();
            u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,bodyHandle->m_rootLocalInertialFrame);
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
				bodyHandle->m_multiBody = mb;
				createJointMotors(mb);


				//serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire
			    UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
			    m_data->m_urdfLinkNameMapper.push_back(util);
			    util->m_mb = mb;
			    util->m_memSerializer = new btDefaultSerializer(bufferSizeInBytes ,(unsigned char*)bufferServerToClient);
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





bool PhysicsServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes )
{

	bool hasStatus = false;

    {
#if 0
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
        
        {
           
            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
            
            
			//const SharedMemoryCommand& clientCmd =m_data->m_testBlock1->m_clientCommands[0];
#if 0
			if (m_data->m_commandLogger)
			{
				m_data->m_commandLogger->logCommand(m_data->m_testBlock1);
			}
#endif

			//m_data->m_testBlock1->m_numProcessedClientCommands++;
			
			//no timestamp yet
            int timeStamp = 0;

            //consume the command
			switch (clientCmd.m_type)
            {
#if 0
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
#endif

				case CMD_REQUEST_DEBUG_LINES:
					{
						int curFlags =m_data->m_remoteDebugDrawer->getDebugMode();
                        
                        int debugMode = clientCmd.m_requestDebugLinesArguments.m_debugMode;//clientCmd.btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb;
                        int startingLineIndex = clientCmd.m_requestDebugLinesArguments.m_startingLineIndex;
                        if (startingLineIndex<0)
                        {
                            b3Warning("startingLineIndex should be non-negative");
                            startingLineIndex = 0;
                        }
                        
                        if (clientCmd.m_requestDebugLinesArguments.m_startingLineIndex==0)
                        {
                            m_data->m_remoteDebugDrawer->m_lines2.resize(0);
                            //|btIDebugDraw::DBG_DrawAabb|
                            //	btIDebugDraw::DBG_DrawConstraints |btIDebugDraw::DBG_DrawConstraintLimits ;
                            m_data->m_remoteDebugDrawer->setDebugMode(debugMode);
							btIDebugDraw* oldDebugDrawer = m_data->m_dynamicsWorld->getDebugDrawer();
							m_data->m_dynamicsWorld->setDebugDrawer(m_data->m_remoteDebugDrawer);
                            m_data->m_dynamicsWorld->debugDrawWorld();
							m_data->m_dynamicsWorld->setDebugDrawer(oldDebugDrawer);
                            m_data->m_remoteDebugDrawer->setDebugMode(curFlags);
                        }

                        //9 floats per line: 3 floats for 'from', 3 floats for 'to' and 3 floats for 'color'
                        int maxNumLines = bufferSizeInBytes/(sizeof(float)*9)-1;
                        if (startingLineIndex >m_data->m_remoteDebugDrawer->m_lines2.size())
                        {
                            b3Warning("m_startingLineIndex exceeds total number of debug lines");
                            startingLineIndex =m_data->m_remoteDebugDrawer->m_lines2.size();
                        }
                        
                        int numLines = btMin(maxNumLines,m_data->m_remoteDebugDrawer->m_lines2.size()-startingLineIndex);
                        
                        if (numLines)
                        {

							float* linesFrom = (float*)bufferServerToClient;
							float* linesTo = (float*)(bufferServerToClient+numLines*3*sizeof(float));
							float* linesColor = (float*)(bufferServerToClient+2*numLines*3*sizeof(float));

							for (int i=0;i<numLines;i++)
							{
								linesFrom[i*3] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_from.x();
								linesTo[i*3] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_to.x();
								linesColor[i*3] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_color.x();

								linesFrom[i*3+1] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_from.y();
								linesTo[i*3+1] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_to.y();
								linesColor[i*3+1] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_color.y();

								linesFrom[i*3+2] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_from.z();
								linesTo[i*3+2] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_to.z();
								linesColor[i*3+2] = m_data->m_remoteDebugDrawer->m_lines2[i+startingLineIndex].m_color.z();
                            }
						}
                        
						serverStatusOut.m_type = CMD_DEBUG_LINES_COMPLETED;

                        serverStatusOut.m_sendDebugLinesArgs.m_numDebugLines = numLines;
                        serverStatusOut.m_sendDebugLinesArgs.m_startingLineIndex = startingLineIndex;
                        serverStatusOut.m_sendDebugLinesArgs.m_numRemainingDebugLines = m_data->m_remoteDebugDrawer->m_lines2.size()-(startingLineIndex+numLines);
						hasStatus = true;
						
						break;
					}

                case CMD_LOAD_URDF:
                {
					
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
					int bodyUniqueId;
                    //load the actual URDF and send a report: completed or failed
                    bool completedOk = loadUrdf(urdfArgs.m_urdfFileName,
                                               initialPos,initialOrn,
                                               useMultiBody, useFixedBase,&bodyUniqueId, bufferServerToClient, bufferSizeInBytes);
                    
 
                    if (completedOk)
                    {
						
						m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
						
						serverStatusOut.m_type = CMD_URDF_LOADING_COMPLETED;
						if (m_data->m_urdfLinkNameMapper.size())
						{
							serverStatusOut.m_dataStreamArguments.m_streamChunkLength = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
						}
						serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
						hasStatus = true;
                        
                    } else
                    {
						serverStatusOut.m_type = CMD_URDF_LOADING_FAILED;
						hasStatus = true;
                    }
                    
                    

                    
                    break;
                }
                case CMD_CREATE_SENSOR:
                {
					if (m_data->m_verboseOutput)
					{
						b3Printf("Processed CMD_CREATE_SENSOR");
					}
					int bodyUniqueId = clientCmd.m_createSensorArguments.m_bodyUniqueId;
					InteralBodyData* body = m_data->getHandle(bodyUniqueId);
                    if (body && body->m_multiBody)
                    {
                        btMultiBody* mb = body->m_multiBody;
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
                    
					serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					
                    break;
                }
                case CMD_SEND_DESIRED_STATE:
                    {
						if (m_data->m_verboseOutput)
						{
                            b3Printf("Processed CMD_SEND_DESIRED_STATE");
						}
						
							int bodyUniqueId = clientCmd.m_sendDesiredStateCommandArgument.m_bodyUniqueId;
							InteralBodyData* body = m_data->getHandle(bodyUniqueId);

                            if (body && body->m_multiBody)
                            {
                                btMultiBody* mb = body->m_multiBody;
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
										{
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
										{
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
                            
							serverStatusOut.m_type = CMD_DESIRED_STATE_RECEIVED_COMPLETED;
							hasStatus = true;
                        break;
                    }
				case CMD_REQUEST_ACTUAL_STATE:
					{
						if (m_data->m_verboseOutput)
						{
							b3Printf("Sending the actual state (Q,U)");
						}
						int bodyUniqueId = clientCmd.m_requestActualStateInformationCommandArgument.m_bodyUniqueId;
						InteralBodyData* body = m_data->getHandle(bodyUniqueId);
						
						if (body && body->m_multiBody)
						{
							btMultiBody* mb = body->m_multiBody;
							SharedMemoryStatus& serverCmd = serverStatusOut;
							serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

							serverCmd.m_sendActualStateArgs.m_bodyUniqueId = bodyUniqueId;
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
                                    body->m_rootLocalInertialFrame.getOrigin()[0];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[1] =
                                    body->m_rootLocalInertialFrame.getOrigin()[1];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[2] =
                                    body->m_rootLocalInertialFrame.getOrigin()[2];

                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[3] =
                                    body->m_rootLocalInertialFrame.getRotation()[0];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[4] =
                                    body->m_rootLocalInertialFrame.getRotation()[1];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[5] =
                                    body->m_rootLocalInertialFrame.getRotation()[2];
                                serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[6] =
                                    body->m_rootLocalInertialFrame.getRotation()[3];


                                
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
							
							hasStatus = true;
							
						} else
						{
							if (body && body->m_rigidBody)
							{
								btRigidBody* rb = body->m_rigidBody;
								SharedMemoryStatus& serverCmd = serverStatusOut;
								serverCmd.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

								serverCmd.m_sendActualStateArgs.m_bodyUniqueId = bodyUniqueId;
								int totalDegreeOfFreedomQ = 0;
								int totalDegreeOfFreedomU = 0;

								btTransform tr = rb->getWorldTransform();
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
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = rb->getLinearVelocity()[0];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = rb->getLinearVelocity()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = rb->getLinearVelocity()[2];
	
								//base angular velocity (in world space, carthesian)
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = rb->getAngularVelocity()[0];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = rb->getAngularVelocity()[1];
								serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = rb->getAngularVelocity()[2];
								totalDegreeOfFreedomU += 6;//3 linear and 3 angular DOF

								serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
								serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;

								hasStatus = true;
							} else
							{
								b3Warning("Request state but no multibody or rigid body available");
								SharedMemoryStatus& serverCmd = serverStatusOut;
								serverCmd.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
								hasStatus = true;
							}
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
                    
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;
					hasStatus = true;
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
					
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					break;
					
				};
				case CMD_INIT_POSE:
				{
					if (m_data->m_verboseOutput)
					{
						b3Printf("Server Init Pose not implemented yet");
					}
					int bodyUniqueId = clientCmd.m_initPoseArgs.m_bodyUniqueId;
					InteralBodyData* body = m_data->getHandle(bodyUniqueId);

					if (body && body->m_multiBody)
					{
						btMultiBody* mb = body->m_multiBody;
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_POSITION)
						{
							btVector3 zero(0,0,0);
							mb->setBaseVel(zero);
							mb->setBasePos(btVector3(
									clientCmd.m_initPoseArgs.m_initialStateQ[0],
									clientCmd.m_initPoseArgs.m_initialStateQ[1],
									clientCmd.m_initPoseArgs.m_initialStateQ[2]));
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_ORIENTATION)
						{
							mb->setBaseOmega(btVector3(0,0,0));
							mb->setWorldToBaseRot(btQuaternion(
									clientCmd.m_initPoseArgs.m_initialStateQ[3],
									clientCmd.m_initPoseArgs.m_initialStateQ[4],
									clientCmd.m_initPoseArgs.m_initialStateQ[5],
									clientCmd.m_initPoseArgs.m_initialStateQ[6]));
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_JOINT_STATE)
						{
							int dofIndex = 7;
							for (int i=0;i<mb->getNumLinks();i++)
							{
								if (mb->getLink(i).m_dofCount==1)
								{
									
									mb->setJointPos(i,clientCmd.m_initPoseArgs.m_initialStateQ[dofIndex]);
									mb->setJointVel(i,0);
								}
								dofIndex += mb->getLink(i).m_dofCount;
							}
						}
					}
					
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;

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
					m_data->exitHandles();
					m_data->initHandles();
                   
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
					hasStatus = true;

                    break;
                }
				case CMD_CREATE_RIGID_BODY:
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

						btScalar mass = 0.f;
						if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_MASS)
						{
							mass = clientCmd.m_createBoxShapeArguments.m_mass;
						}

						int shapeType = COLLISION_SHAPE_TYPE_BOX;

						if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_COLLISION_SHAPE_TYPE)
						{
							shapeType = clientCmd.m_createBoxShapeArguments.m_collisionShapeType;
						}

						btBulletWorldImporter* worldImporter = new btBulletWorldImporter(m_data->m_dynamicsWorld);
						m_data->m_worldImporters.push_back(worldImporter);

						btCollisionShape* shape = 0;
						
						switch (shapeType)
						{
							case COLLISION_SHAPE_TYPE_CYLINDER_X:
							{
								btScalar radius = halfExtents[1];
								btScalar height = halfExtents[0];
								shape = worldImporter->createCylinderShapeX(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_CYLINDER_Y:
							{
								btScalar radius = halfExtents[0];
								btScalar height = halfExtents[1];
								shape = worldImporter->createCylinderShapeY(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_CYLINDER_Z:
							{
								btScalar radius = halfExtents[1];
								btScalar height = halfExtents[2];
								shape = worldImporter->createCylinderShapeZ(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_CAPSULE_X:
							{
								btScalar radius = halfExtents[1];
								btScalar height = halfExtents[0];
								shape = worldImporter->createCapsuleShapeX(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_CAPSULE_Y:
							{
								btScalar radius = halfExtents[0];
								btScalar height = halfExtents[1];
								shape = worldImporter->createCapsuleShapeY(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_CAPSULE_Z:
							{
								btScalar radius = halfExtents[1];
								btScalar height = halfExtents[2];
								shape = worldImporter->createCapsuleShapeZ(radius,height);
								break;
							}
							case COLLISION_SHAPE_TYPE_SPHERE:
							{
								btScalar radius = halfExtents[0];
								shape = worldImporter->createSphereShape(radius);
								break;
							}
							case COLLISION_SHAPE_TYPE_BOX:
							default:
							{
								shape = worldImporter->createBoxShape(halfExtents);
							}
						}
						
						
						bool isDynamic = (mass>0);
						btRigidBody* rb = worldImporter->createRigidBody(isDynamic,mass,startTrans,shape,0);
						rb->setRollingFriction(0.2);
						//m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
						btVector4 colorRGBA(1,0,0,1);
						if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_COLOR)
						{
							colorRGBA[0] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[0];
							colorRGBA[1] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[1];
							colorRGBA[2] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[2];
							colorRGBA[3] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[3];
						}
						m_data->m_guiHelper->createCollisionShapeGraphicsObject(rb->getCollisionShape());
						m_data->m_guiHelper->createCollisionObjectGraphicsObject(rb,colorRGBA);
						
						
						SharedMemoryStatus& serverCmd =serverStatusOut;
						serverCmd.m_type = CMD_RIGID_BODY_CREATION_COMPLETED;
						

						int bodyUniqueId = m_data->allocHandle();
						InternalBodyHandle* bodyHandle = m_data->getHandle(bodyUniqueId);
						serverCmd.m_rigidBodyCreateArgs.m_bodyUniqueId = bodyUniqueId;
						bodyHandle->m_rootLocalInertialFrame.setIdentity();
						bodyHandle->m_rigidBody = rb;
						hasStatus = true;
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

						
						SharedMemoryStatus& serverCmd =serverStatusOut;
						serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
						hasStatus = true;
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

                     	SharedMemoryStatus& serverCmd =serverStatusOut;
						serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
						hasStatus = true;
                        break;
                    }
                case CMD_REMOVE_PICKING_CONSTRAINT_BODY:
                    {
                        removePickingConstraint();

						SharedMemoryStatus& serverCmd =serverStatusOut;
						serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
						hasStatus = true;
                        break;
                    }
                default:
                {
                    b3Error("Unknown command encountered");
					
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_UNKNOWN_COMMAND_FLUSHED;
					hasStatus = true;


                }
            };
            
        }
    }
	return hasStatus;
}


void PhysicsServerCommandProcessor::renderScene()
{
	if (m_data->m_guiHelper)
	{
		m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
		
		m_data->m_guiHelper->render(m_data->m_dynamicsWorld);
	}
	
}

void    PhysicsServerCommandProcessor::physicsDebugDraw(int debugDrawFlags)
{
	if (m_data->m_dynamicsWorld)
	{
		if (m_data->m_dynamicsWorld->getDebugDrawer())
		{
			m_data->m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
			m_data->m_dynamicsWorld->debugDrawWorld();
		}
	}
}


bool PhysicsServerCommandProcessor::pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
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


bool PhysicsServerCommandProcessor::movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
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

void PhysicsServerCommandProcessor::removePickingConstraint()
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


void PhysicsServerCommandProcessor::enableCommandLogging(bool enable, const char* fileName)
{
	if (enable)
	{
		if (0==m_data->m_commandLogger)
		{
			m_data->m_commandLogger = new CommandLogger(fileName);
		}
	} else
	{
		if (0!=m_data->m_commandLogger)
		{
			delete m_data->m_commandLogger;
			m_data->m_commandLogger = 0;
		}
	}
}


void PhysicsServerCommandProcessor::replayFromLogFile(const char* fileName)
{
	CommandLogPlayback* pb = new CommandLogPlayback(fileName);
	m_data->m_logPlayback = pb;
}

