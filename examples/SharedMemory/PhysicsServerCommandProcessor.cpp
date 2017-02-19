#include "PhysicsServerCommandProcessor.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"
#include "TinyRendererVisualShapeConverter.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "LinearMath/btHashMap.h"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include "IKTrajectoryHelper.h"
#include "btBulletDynamicsCommon.h"
#include "../Utils/RobotLoggingUtil.h"
#include "LinearMath/btTransform.h"
#include "../Importers/ImportMJCFDemo/BulletMJCFImporter.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "LinearMath/btSerializer.h"
#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryCommands.h"

#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h"
#include "../SoftDemo/BunnyMesh.h"
#else
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#endif


//@todo(erwincoumans) those globals are hacks for a VR demo, move this to Python/pybullet!
btVector3 gLastPickPos(0, 0, 0);
bool gCloseToKuka=false;
bool gEnableRealTimeSimVR=false;
bool gCreateDefaultRobotAssets = false;
int gInternalSimFlags = 0;
bool gResetSimulation = 0;
int gVRTrackingObjectUniqueId = -1;
btTransform gVRTrackingObjectTr = btTransform::getIdentity();

int gCreateObjectSimVR = -1;
int gEnableKukaControl = 0;
btVector3 gVRTeleportPos1(0,0,0);
btQuaternion gVRTeleportOrn(0, 0, 0,1);


btScalar simTimeScalingFactor = 1;
btScalar gRhsClamp = 1.f;

struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btDefaultSerializer* m_memSerializer;

	UrdfLinkNameMapUtil():m_mb(0),m_memSerializer(0)
	{
	}
	virtual ~UrdfLinkNameMapUtil()
	{
		delete m_memSerializer;
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
	btAlignedObjectArray<btTransform> m_linkLocalInertialFrames;

	InteralBodyData()
		:m_multiBody(0),
		m_rigidBody(0),
		m_testData(0)
	{
		m_rootLocalInertialFrame.setIdentity();
	}
};

struct InteralUserConstraintData
{
	btTypedConstraint* m_rbConstraint;
	btMultiBodyConstraint* m_mbConstraint;

	b3UserConstraint m_userConstraintData;

	InteralUserConstraintData()
		:m_rbConstraint(0),
		m_mbConstraint(0)
	{
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
			size_t bytesRead;
			bytesRead = fread(m_header,12,1,m_file);
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

struct SaveWorldObjectData
{
	b3AlignedObjectArray<int> m_bodyUniqueIds;
	std::string	m_fileName;
};

struct MyBroadphaseCallback : public btBroadphaseAabbCallback
{
	b3AlignedObjectArray<int> m_bodyUniqueIds;
	b3AlignedObjectArray<int> m_links;


	MyBroadphaseCallback()
	{
	}
	virtual ~MyBroadphaseCallback()
	{
	}
	void clear()
	{
		m_bodyUniqueIds.clear();
		m_links.clear();
	}
	virtual bool	process(const btBroadphaseProxy* proxy)
	{
		btCollisionObject* colObj = (btCollisionObject*)proxy->m_clientObject;
		btMultiBodyLinkCollider* mbl = btMultiBodyLinkCollider::upcast(colObj);
		if (mbl)
		{
			int bodyUniqueId = mbl->m_multiBody->getUserIndex2();
			m_bodyUniqueIds.push_back(bodyUniqueId);
			m_links.push_back(mbl->m_link);
			return true;
		}
		int bodyUniqueId = colObj->getUserIndex2();
		if (bodyUniqueId >= 0)
		{
			m_bodyUniqueIds.push_back(bodyUniqueId);
			m_links.push_back(mbl->m_link);
		}
		return true;
	}
};



enum MyFilterModes
{
	FILTER_GROUPAMASKB_AND_GROUPBMASKA=0,
	FILTER_GROUPAMASKB_OR_GROUPBMASKA
};

struct MyOverlapFilterCallback : public btOverlapFilterCallback
{
	int m_filterMode;
	
	MyOverlapFilterCallback()
	:m_filterMode(FILTER_GROUPAMASKB_AND_GROUPBMASKA)
	{
	}
	
	virtual ~MyOverlapFilterCallback()
	{}
	// return true when pairs need collision
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
	{
		if (m_filterMode==FILTER_GROUPAMASKB_AND_GROUPBMASKA)
		{
			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}
		
		if (m_filterMode==FILTER_GROUPAMASKB_OR_GROUPBMASKA)
		{
			bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
			collides = collides || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}
		return false;
	}
};


struct InternalStateLogger
{
	int m_loggingUniqueId;
	int m_loggingType;

	InternalStateLogger()
		:m_loggingUniqueId(0),
		m_loggingType(0)
	{
	}
	virtual ~InternalStateLogger() {}

	virtual void stop() = 0;
	virtual void logState(btScalar timeStamp)=0;

};

struct MinitaurStateLogger : public InternalStateLogger
{
	int m_loggingTimeStamp;
	std::string m_fileName;
	int m_minitaurBodyUniqueId;
	FILE* m_logFileHandle;

	std::string m_structTypes;
	btMultiBody* m_minitaurMultiBody;
	btAlignedObjectArray<int> m_motorIdList;

	MinitaurStateLogger(int loggingUniqueId, std::string fileName, btMultiBody* minitaurMultiBody, btAlignedObjectArray<int>& motorIdList)
		:m_loggingTimeStamp(0),
		m_logFileHandle(0),
		m_minitaurMultiBody(minitaurMultiBody)
	{
		m_loggingType = STATE_LOGGING_MINITAUR;
		m_motorIdList.resize(motorIdList.size());
		for (int m=0;m<motorIdList.size();m++)
		{
			m_motorIdList[m] = motorIdList[m];
		}

		btAlignedObjectArray<std::string> structNames;
		//'t', 'r', 'p', 'y', 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7', 'xd', 'mo'
		structNames.push_back("t");
		structNames.push_back("r");
		structNames.push_back("p");
		structNames.push_back("y");
	
		structNames.push_back("q0");
		structNames.push_back("q1");
		structNames.push_back("q2");
		structNames.push_back("q3");
		structNames.push_back("q4");
		structNames.push_back("q5");
		structNames.push_back("q6");
		structNames.push_back("q7");

		structNames.push_back("u0");
		structNames.push_back("u1");
		structNames.push_back("u2");
		structNames.push_back("u3");
		structNames.push_back("u4");
		structNames.push_back("u5");
		structNames.push_back("u6");
		structNames.push_back("u7");

		structNames.push_back("dx");
		structNames.push_back("mo");

		m_structTypes = "IffffffffffffffffffffB";
		const char* fileNameC = fileName.c_str();

		m_logFileHandle = createMinitaurLogFile(fileNameC, structNames, m_structTypes);
	}
	virtual void stop()
	{
		if (m_logFileHandle)
		{
			closeMinitaurLogFile(m_logFileHandle);
			m_logFileHandle = 0;
		}
	}

	virtual void logState(btScalar timeStep)
	{
		if (m_logFileHandle)
		{
			
			btVector3 pos = m_minitaurMultiBody->getBasePos();

			MinitaurLogRecord logData;
			//'t', 'r', 'p', 'y', 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7', 'xd', 'mo'
			btScalar motorDir[8] = {1, -1, 1, -1, -1, 1, -1, 1};

			btQuaternion orn = m_minitaurMultiBody->getBaseWorldTransform().getRotation();
			btMatrix3x3 mat(orn);
			btScalar roll=0;
			btScalar pitch=0;
			btScalar yaw = 0;

			mat.getEulerZYX(yaw,pitch,roll);
			
			logData.m_values.push_back(m_loggingTimeStamp);
			logData.m_values.push_back((float)roll);
			logData.m_values.push_back((float)pitch);
			logData.m_values.push_back((float)yaw);

			for (int i=0;i<8;i++)
			{
				float jointAngle = (float)motorDir[i]*m_minitaurMultiBody->getJointPos(m_motorIdList[i]);
				logData.m_values.push_back(jointAngle);
			}
			for (int i=0;i<8;i++)
			{
                btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)m_minitaurMultiBody->getLink(m_motorIdList[i]).m_userPtr;

                if (motor && timeStep>btScalar(0))
                {
					btScalar force = motor->getAppliedImpulse(0)/timeStep;
					logData.m_values.push_back((float)force);
				}
			}
			//x is forward component, estimated speed forward
			float xd_speed = m_minitaurMultiBody->getBaseVel()[0];
			logData.m_values.push_back(xd_speed);
			char mode = 6;
			logData.m_values.push_back(mode);

			//at the moment, appendMinitaurLogData will directly write to disk (potential delay)
			//better to fill a huge memory buffer and once in a while write it to disk
			appendMinitaurLogData(m_logFileHandle, m_structTypes, logData);

			fflush(m_logFileHandle);
		
			m_loggingTimeStamp++;
		}
	}
};


struct PhysicsServerCommandProcessorInternalData
{
	///handle management
	btAlignedObjectArray<InternalBodyHandle>	m_bodyHandles;
	int m_numUsedHandles;						// number of active handles
	int	m_firstFreeHandle;		// free handles list

	int getNumHandles() const
	{
		return m_bodyHandles.size();
	}

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
			//int curCapacity = m_bodyHandles.size();
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

	bool m_allowRealTimeSimulation;
	bool m_hasGround;

	b3VRControllerEvent m_vrEvents[MAX_VR_CONTROLLERS];
	
	btMultiBodyFixedConstraint* m_gripperRigidbodyFixed;
	btMultiBody* m_gripperMultiBody;
	btMultiBodyFixedConstraint* m_kukaGripperFixed;
	btMultiBody* m_kukaGripperMultiBody;
	btMultiBodyPoint2Point* m_kukaGripperRevolute1;
	btMultiBodyPoint2Point* m_kukaGripperRevolute2;
	

	int m_huskyId;
	int m_KukaId;
	int m_sphereId;
	int m_gripperId;
	CommandLogger* m_commandLogger;
	CommandLogPlayback* m_logPlayback;


	btScalar m_physicsDeltaTime;
    btScalar m_numSimulationSubSteps;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_multiBodyJointFeedbacks;
	btHashMap<btHashPtr, btInverseDynamics::MultiBodyTree*> m_inverseDynamicsBodies;
	btHashMap<btHashPtr, IKTrajectoryHelper*> m_inverseKinematicsHelpers;
	
	int m_userConstraintUIDGenerator;
	btHashMap<btHashInt, InteralUserConstraintData> m_userConstraints;

	b3AlignedObjectArray<SaveWorldObjectData> m_saveWorldBodyData;


	btAlignedObjectArray<btBulletWorldImporter*> m_worldImporters;
	btAlignedObjectArray<UrdfLinkNameMapUtil*> m_urdfLinkNameMapper;
	btAlignedObjectArray<std::string*> m_strings;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	MyOverlapFilterCallback* m_broadphaseCollisionFilterCallback;
	btHashedOverlappingPairCache* m_pairCache;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btMultiBodyConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
    
#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	btSoftMultiBodyDynamicsWorld* m_dynamicsWorld;
    btSoftBodySolver* m_softbodySolver;
    btSoftBodyWorldInfo	m_softBodyWorldInfo;
#else
    btMultiBodyDynamicsWorld* m_dynamicsWorld;
#endif
    
	SharedMemoryDebugDrawer*		m_remoteDebugDrawer;
    
	btAlignedObjectArray<b3ContactPointData> m_cachedContactPoints;
	MyBroadphaseCallback m_cachedOverlappingObjects;


	btAlignedObjectArray<int> m_sdfRecentLoadedBodies;

	btAlignedObjectArray<InternalStateLogger*>	m_stateLoggers;
	int m_stateLoggersUniqueId;

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
	TinyRendererVisualShapeConverter  m_visualConverter;

	PhysicsServerCommandProcessorInternalData()
		:
		m_allowRealTimeSimulation(false),
	m_hasGround(false),	
	m_gripperRigidbodyFixed(0),
		m_gripperMultiBody(0),
		m_kukaGripperFixed(0),
		m_kukaGripperMultiBody(0),
		m_kukaGripperRevolute1(0),
		m_kukaGripperRevolute2(0),
		m_huskyId(-1),
		m_KukaId(-1),
		m_sphereId(-1),
		m_gripperId(-1),
		m_commandLogger(0),
		m_logPlayback(0),
		m_physicsDeltaTime(1./240.),
        m_numSimulationSubSteps(0),
		m_userConstraintUIDGenerator(1),
		m_broadphaseCollisionFilterCallback(0),
		m_pairCache(0),
		m_broadphase(0),
		m_dispatcher(0),
		m_solver(0),
		m_collisionConfiguration(0),
		m_dynamicsWorld(0),
		m_remoteDebugDrawer(0),
		m_stateLoggersUniqueId(0),
		m_guiHelper(0),
		m_sharedMemoryKey(SHARED_MEMORY_KEY),
		m_verboseOutput(false),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_pickingMultiBodyPoint2Point(0)
	{
		for (int i=0;i<MAX_VR_CONTROLLERS;i++)
		{
			m_vrEvents[i].m_numButtonEvents = 0;
			m_vrEvents[i].m_numMoveEvents = 0;
			for (int b=0;b<MAX_VR_BUTTONS;b++)
			{
				m_vrEvents[i].m_buttons[b] = 0;
			}
		}

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

    btInverseDynamics::MultiBodyTree* findOrCreateTree(btMultiBody* multiBody)
    {
        btInverseDynamics::MultiBodyTree* tree = 0;
        
        btInverseDynamics::MultiBodyTree** treePtrPtr =
        m_inverseDynamicsBodies.find(multiBody);
        
        if (treePtrPtr)
        {
            tree = *treePtrPtr;
        }
        else
        {
            btInverseDynamics::btMultiBodyTreeCreator id_creator;
            if (-1 == id_creator.createFromBtMultiBody(multiBody, false))
            {
                
            }
            else
            {
                tree = btInverseDynamics::CreateMultiBodyTree(id_creator);
                m_inverseDynamicsBodies.insert(multiBody, tree);
            }
        }
        
        return tree;
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

void logCallback(btDynamicsWorld *world, btScalar timeStep)
{
	PhysicsServerCommandProcessor* proc = (PhysicsServerCommandProcessor*) world->getWorldUserInfo();
	proc->logObjectStates(timeStep);

}


void PhysicsServerCommandProcessor::logObjectStates(btScalar timeStep)
{
	for (int i=0;i<m_data->m_stateLoggers.size();i++)
	{
		m_data->m_stateLoggers[i]->logState(timeStep);
	}

}

void PhysicsServerCommandProcessor::createEmptyDynamicsWorld()
{
    ///collision configuration contains default setup for memory, collision setup
    //m_collisionConfiguration->setConvexConvexMultipointIterations();
#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
    m_data->m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
#else
    m_data->m_collisionConfiguration = new btDefaultCollisionConfiguration();
#endif
    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_data->m_dispatcher = new	btCollisionDispatcher(m_data->m_collisionConfiguration);
    

	m_data->m_broadphaseCollisionFilterCallback = new MyOverlapFilterCallback();
	m_data->m_broadphaseCollisionFilterCallback->m_filterMode = FILTER_GROUPAMASKB_OR_GROUPBMASKA;
	
	m_data->m_pairCache = new btHashedOverlappingPairCache();
	
	m_data->m_pairCache->setOverlapFilterCallback(m_data->m_broadphaseCollisionFilterCallback);
	
    m_data->m_broadphase = new btDbvtBroadphase(m_data->m_pairCache);
    
    m_data->m_solver = new btMultiBodyConstraintSolver;
    
#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
    m_data->m_dynamicsWorld = new btSoftMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);
#else
    m_data->m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);
#endif
    
    //Workaround: in a VR application, where we avoid synchronizaing between GFX/Physics threads, we don't want to resize this array, so pre-allocate it
    m_data->m_dynamicsWorld->getCollisionObjectArray().reserve(32768);
    
    m_data->m_remoteDebugDrawer = new SharedMemoryDebugDrawer();
    
    
    m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
    m_data->m_dynamicsWorld->getSolverInfo().m_erp2 = 0.08;
    
	m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = 0.00001;
	m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
	m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = 1e-7;
//	m_data->m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 2;
	//todo: islands/constraints are buggy in btMultiBodyDynamicsWorld! (performance + see slipping grasp)


	m_data->m_dynamicsWorld->setInternalTickCallback(logCallback,this);
}

void PhysicsServerCommandProcessor::deleteStateLoggers()
{
	for (int i=0;i<m_data->m_stateLoggers.size();i++)
	{
		m_data->m_stateLoggers[i]->stop();
		delete m_data->m_stateLoggers[i];
	}
	m_data->m_stateLoggers.clear();

}

void PhysicsServerCommandProcessor::deleteCachedInverseKinematicsBodies()
{
	for (int i = 0; i < m_data->m_inverseKinematicsHelpers.size(); i++)
	{
		IKTrajectoryHelper** ikHelperPtr = m_data->m_inverseKinematicsHelpers.getAtIndex(i);
		if (ikHelperPtr)
		{
			IKTrajectoryHelper* ikHelper = *ikHelperPtr;
			delete ikHelper;
		}
	}
	m_data->m_inverseKinematicsHelpers.clear();
}
void PhysicsServerCommandProcessor::deleteCachedInverseDynamicsBodies()
{
	for (int i = 0; i < m_data->m_inverseDynamicsBodies.size(); i++)
	{
		btInverseDynamics::MultiBodyTree** treePtrPtr = m_data->m_inverseDynamicsBodies.getAtIndex(i);
		if (treePtrPtr)
		{
			btInverseDynamics::MultiBodyTree* tree = *treePtrPtr;
			delete tree;
		}

	}
	m_data->m_inverseDynamicsBodies.clear();
}

void PhysicsServerCommandProcessor::deleteDynamicsWorld()
{
	deleteCachedInverseDynamicsBodies();
	deleteCachedInverseKinematicsBodies();
	deleteStateLoggers();

	m_data->m_userConstraints.clear();
	m_data->m_saveWorldBodyData.clear();

	for (int i=0;i<m_data->m_multiBodyJointFeedbacks.size();i++)
	{
		delete m_data->m_multiBodyJointFeedbacks[i];
	}
	m_data->m_multiBodyJointFeedbacks.clear();


	for (int i=0;i<m_data->m_worldImporters.size();i++)
	{
		m_data->m_worldImporters[i]->deleteAllData();
		delete m_data->m_worldImporters[i];
	}
	m_data->m_worldImporters.clear();

	for (int i=0;i<m_data->m_urdfLinkNameMapper.size();i++)
	{
		delete m_data->m_urdfLinkNameMapper[i];
	}
	m_data->m_urdfLinkNameMapper.clear();


	for (int i=0;i<m_data->m_strings.size();i++)
	{
		delete m_data->m_strings[i];
	}
	m_data->m_strings.clear();

    btAlignedObjectArray<btTypedConstraint*> constraints;
    btAlignedObjectArray<btMultiBodyConstraint*> mbconstraints;


	if (m_data->m_dynamicsWorld)
	{

		int i;
		for (i = m_data->m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
		{
            btTypedConstraint* constraint =m_data->m_dynamicsWorld->getConstraint(i);
            constraints.push_back(constraint);
			m_data->m_dynamicsWorld->removeConstraint(constraint);
		}
        for (i=m_data->m_dynamicsWorld->getNumMultiBodyConstraints()-1;i>=0;i--)
        {
            btMultiBodyConstraint* mbconstraint = m_data->m_dynamicsWorld->getMultiBodyConstraint(i);
            mbconstraints.push_back(mbconstraint);
            m_data->m_dynamicsWorld->removeMultiBodyConstraint(mbconstraint);
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
        for (i=m_data->m_dynamicsWorld->getNumMultibodies()-1;i>=0;i--)
        {
            btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(i);
            m_data->m_dynamicsWorld->removeMultiBody(mb);
            delete mb;
        }
	}

    for (int i=0;i<constraints.size();i++)
    {
        delete constraints[i];
    }
    constraints.clear();
    for (int i=0;i<mbconstraints.size();i++)
    {
        delete mbconstraints[i];
    }
    mbconstraints.clear();
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

	delete m_data->m_pairCache;
	m_data->m_pairCache= 0;
	
	delete m_data->m_broadphaseCollisionFilterCallback;
	m_data->m_broadphaseCollisionFilterCallback= 0;
	
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
			float maxMotorImpulse = 1.f;
			int dof = 0;
			btScalar desiredVelocity = 0.f;
			btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb,mbLinkIndex,dof,desiredVelocity,maxMotorImpulse);
			motor->setPositionTarget(0, 0);
			motor->setVelocityTarget(0, 1);
			//motor->setRhsClamp(gRhsClamp);
			//motor->setMaxAppliedImpulse(0);
            mb->getLink(mbLinkIndex).m_userPtr = motor;
			m_data->m_dynamicsWorld->addMultiBodyConstraint(motor);
            motor->finalizeMultiDof();

		}

	}
}

bool PhysicsServerCommandProcessor::processImportedObjects(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags, URDFImporterInterface& u2b)
{
	bool loadOk = true;
        

    btTransform rootTrans;
    rootTrans.setIdentity();
    if (m_data->m_verboseOutput)
    {
        b3Printf("loaded %s OK!", fileName);
    }
	SaveWorldObjectData sd;
	sd.m_fileName = fileName;


    for (int m =0; m<u2b.getNumModels();m++)
    {

        u2b.activateModel(m);
        btMultiBody* mb = 0;
        btRigidBody* rb = 0;

        //get a body index
        int bodyUniqueId = m_data->allocHandle();

        InternalBodyHandle* bodyHandle = m_data->getHandle(bodyUniqueId);
			
		sd.m_bodyUniqueIds.push_back(bodyUniqueId);

        u2b.setBodyUniqueId(bodyUniqueId);
        {
            btScalar mass = 0;
            bodyHandle->m_rootLocalInertialFrame.setIdentity();
            btVector3 localInertiaDiagonal(0,0,0);
            int urdfLinkIndex = u2b.getRootLinkIndex();
            u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,bodyHandle->m_rootLocalInertialFrame);
        }



        //todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
        int rootLinkIndex = u2b.getRootLinkIndex();
        b3Printf("urdf root link index = %d\n",rootLinkIndex);
        MyMultiBodyCreator creation(m_data->m_guiHelper);

        u2b.getRootTransformInWorld(rootTrans);
        ConvertURDF2Bullet(u2b,creation, rootTrans,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix(),flags);



        mb = creation.getBulletMultiBody();
        rb = creation.getRigidBody();
		if (rb)
			rb->setUserIndex2(bodyUniqueId);

		if (mb)
			mb->setUserIndex2(bodyUniqueId);

			
        if (mb)
        {
            bodyHandle->m_multiBody = mb;
				

			m_data->m_sdfRecentLoadedBodies.push_back(bodyUniqueId);

			createJointMotors(mb);


			//disable serialization of the collision objects (they are too big, and the client likely doesn't need them);

            bodyHandle->m_linkLocalInertialFrames.reserve(mb->getNumLinks());
			for (int i=0;i<mb->getNumLinks();i++)
            {
				//disable serialization of the collision objects

				int urdfLinkIndex = creation.m_mb2urdfLink[i];
				btScalar mass;
                btVector3 localInertiaDiagonal(0,0,0);
                btTransform localInertialFrame;
				u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,localInertialFrame);
				bodyHandle->m_linkLocalInertialFrames.push_back(localInertialFrame);

				std::string* linkName = new std::string(u2b.getLinkName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(linkName);

				mb->getLink(i).m_linkName = linkName->c_str();

				std::string* jointName = new std::string(u2b.getJointName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(jointName);

				mb->getLink(i).m_jointName = jointName->c_str();
            }
			std::string* baseName = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
			m_data->m_strings.push_back(baseName);
			mb->setBaseName(baseName->c_str());
		} else
		{
			b3Warning("No multibody loaded from URDF. Could add btRigidBody+btTypedConstraint solution later.");
            bodyHandle->m_rigidBody = rb;
		}

    }
	
	for (int i=0;i<u2b.getNumAllocatedCollisionShapes();i++)
    {
        btCollisionShape* shape =u2b.getAllocatedCollisionShape(i);
        m_data->m_collisionShapes.push_back(shape);
    }

	m_data->m_saveWorldBodyData.push_back(sd);

	return loadOk;
}

struct MyMJCFLogger2 : public MJCFErrorLogger
{
	virtual void reportError(const char* error)
	{
		b3Error(error);
	}
	virtual void reportWarning(const char* warning)
	{
		b3Warning(warning);
	}
	virtual void printMessage(const char* msg)
	{
		b3Printf(msg);
	}
};

bool PhysicsServerCommandProcessor::loadMjcf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags)
{
	btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadSdf: No valid m_dynamicsWorld");
		return false;
	}

	m_data->m_sdfRecentLoadedBodies.clear();

    BulletMJCFImporter u2b(m_data->m_guiHelper);	//, &m_data->m_visualConverter

	bool useFixedBase = false;
	MyMJCFLogger2 logger;
    bool loadOk =  u2b.loadMJCF(fileName, &logger, useFixedBase);
    if (loadOk)
	{
		
		processImportedObjects(fileName,bufferServerToClient,bufferSizeInBytes,useMultiBody,flags, u2b);
	}
	return loadOk;
}

bool PhysicsServerCommandProcessor::loadSdf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags)
{
    btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadSdf: No valid m_dynamicsWorld");
		return false;
	}

	m_data->m_sdfRecentLoadedBodies.clear();

    BulletURDFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter);

	bool forceFixedBase = false;
	bool loadOk =u2b.loadSDF(fileName,forceFixedBase);
	
	if (loadOk)
	{
		processImportedObjects(fileName,bufferServerToClient,bufferSizeInBytes,useMultiBody,flags, u2b);
	}
    return loadOk;
}




bool PhysicsServerCommandProcessor::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("loadURDF");
	btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadUrdf: No valid m_dynamicsWorld");
		return false;
	}



    BulletURDFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter);


    bool loadOk =  u2b.loadURDF(fileName, useFixedBase);


    if (loadOk)
    {
		//get a body index
		int bodyUniqueId = m_data->allocHandle();
		if (bodyUniqueIdPtr)
			*bodyUniqueIdPtr= bodyUniqueId;

		//quick prototype of 'save world' for crude world editing
		{
			SaveWorldObjectData sd;
			sd.m_fileName = fileName;
			sd.m_bodyUniqueIds.push_back(bodyUniqueId);
			m_data->m_saveWorldBodyData.push_back(sd);
		}

        u2b.setBodyUniqueId(bodyUniqueId);
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

        for (int i=0;i<u2b.getNumAllocatedCollisionShapes();i++)
        {
            btCollisionShape* shape =u2b.getAllocatedCollisionShape(i);
            m_data->m_collisionShapes.push_back(shape);
        }

        btMultiBody* mb = creation.getBulletMultiBody();
        btRigidBody* rb = creation.getRigidBody();

		if (useMultiBody)
		{


			if (mb)
			{
				mb->setUserIndex2(bodyUniqueId);
				bodyHandle->m_multiBody = mb;

				createJointMotors(mb);


				//serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire
			    UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
			    m_data->m_urdfLinkNameMapper.push_back(util);
			    util->m_mb = mb;
				for (int i = 0; i < bufferSizeInBytes; i++)
				{
					bufferServerToClient[i] = 0;//0xcc;
				}
			    util->m_memSerializer = new btDefaultSerializer(bufferSizeInBytes ,(unsigned char*)bufferServerToClient);
			    //disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
				util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(),0);

				util->m_memSerializer->startSerialization();


                bodyHandle->m_linkLocalInertialFrames.reserve(mb->getNumLinks());
			    for (int i=0;i<mb->getNumLinks();i++)
                {
					//disable serialization of the collision objects
                   util->m_memSerializer->m_skipPointers.insert(mb->getLink(i).m_collider,0);
				   int urdfLinkIndex = creation.m_mb2urdfLink[i];
				   btScalar mass;
                   btVector3 localInertiaDiagonal(0,0,0);
                   btTransform localInertialFrame;
				   u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,localInertialFrame);
				   bodyHandle->m_linkLocalInertialFrames.push_back(localInertialFrame);

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

				mb->setBaseName(baseName->c_str());

				util->m_memSerializer->registerNameForPointer(baseName->c_str(),baseName->c_str());

				

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
            if (rb)
            {
                bodyHandle->m_rigidBody = rb;
				rb->setUserIndex2(bodyUniqueId);
                return true;
            }
		}

    }

    return false;
}

void PhysicsServerCommandProcessor::replayLogCommand(char* bufferServerToClient, int bufferSizeInBytes)
{
        if (m_data->m_logPlayback)
        {

            SharedMemoryCommand clientCmd;
            SharedMemoryStatus serverStatus;

            bool hasCommand = m_data->m_logPlayback->processNextCommand(&clientCmd);
            if (hasCommand)
            {
                processCommand(clientCmd,serverStatus,bufferServerToClient,bufferSizeInBytes);
            }
        }
}

int PhysicsServerCommandProcessor::createBodyInfoStream(int bodyUniqueId, char* bufferServerToClient, int bufferSizeInBytes)
{
    int streamSizeInBytes = 0;
    //serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire

    InternalBodyHandle* bodyHandle = m_data->getHandle(bodyUniqueId);
    btMultiBody* mb = bodyHandle->m_multiBody;
    if (mb)
    {
        UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
        m_data->m_urdfLinkNameMapper.push_back(util);
        util->m_mb = mb;
        util->m_memSerializer = new btDefaultSerializer(bufferSizeInBytes ,(unsigned char*)bufferServerToClient);
		util->m_memSerializer->startSerialization();

        //disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
        util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(),0);
		if (mb->getBaseName())
		{
			util->m_memSerializer->registerNameForPointer(mb->getBaseName(),mb->getBaseName());
		}

        bodyHandle->m_linkLocalInertialFrames.reserve(mb->getNumLinks());
        for (int i=0;i<mb->getNumLinks();i++)
        {
            //disable serialization of the collision objects
           util->m_memSerializer->m_skipPointers.insert(mb->getLink(i).m_collider,0);
           util->m_memSerializer->registerNameForPointer(mb->getLink(i).m_linkName,mb->getLink(i).m_linkName);
           util->m_memSerializer->registerNameForPointer(mb->getLink(i).m_jointName,mb->getLink(i).m_jointName);
        }

        util->m_memSerializer->registerNameForPointer(mb->getBaseName(),mb->getBaseName());


        int len = mb->calculateSerializeBufferSize();
        btChunk* chunk = util->m_memSerializer->allocate(len,1);
        const char* structType = mb->serialize(chunk->m_oldPtr, util->m_memSerializer);
        util->m_memSerializer->finalizeChunk(chunk,structType,BT_MULTIBODY_CODE,mb);
        streamSizeInBytes = util->m_memSerializer->getCurrentBufferSize();

    }
    return streamSizeInBytes;
}

bool PhysicsServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes )
{

	bool hasStatus = false;

    {
        ///we ignore overflow of integer for now

        {

            //until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands


			//const SharedMemoryCommand& clientCmd =m_data->m_testBlock1->m_clientCommands[0];
#if 1
			if (m_data->m_commandLogger)
			{
                m_data->m_commandLogger->logCommand(clientCmd);
			}
#endif

			//m_data->m_testBlock1->m_numProcessedClientCommands++;

			//no timestamp yet
            //int timeStamp = 0;
			
			//catch uninitialized cases
			serverStatusOut.m_type = CMD_INVALID_STATUS;
			serverStatusOut.m_numDataStreamBytes = 0;
			serverStatusOut.m_dataStream = 0;

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
				case CMD_STATE_LOGGING:
				{
					serverStatusOut.m_type = CMD_STATE_LOGGING_FAILED;
                    hasStatus = true;

					if (clientCmd.m_updateFlags & STATE_LOGGING_START_LOG)
					{

						
						if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_MINITAUR)
						{
							
							std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
							//either provide the minitaur by object unique Id, or search for first multibody with 8 motors...

							
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID)&& (clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds>0))
							{
								int bodyUniqueId = clientCmd.m_stateLoggingArguments.m_bodyUniqueIds[0];
								InteralBodyData* body = m_data->getHandle(bodyUniqueId);
								if (body)
								{
									if (body->m_multiBody)
									{
										btAlignedObjectArray<std::string> motorNames;
										motorNames.push_back("motor_front_leftR_joint");
										motorNames.push_back("motor_front_leftL_joint");
										motorNames.push_back("motor_back_leftR_joint");
										motorNames.push_back("motor_back_leftL_joint");
										motorNames.push_back("motor_front_rightL_joint");
										motorNames.push_back("motor_front_rightR_joint");
										motorNames.push_back("motor_back_rightL_joint");
										motorNames.push_back("motor_back_rightR_joint");
										
										btAlignedObjectArray<int> motorIdList;
										for (int m=0;m<motorNames.size();m++)
										{
											for (int i=0;i<body->m_multiBody->getNumLinks();i++)
											{
												std::string jointName;
												if (body->m_multiBody->getLink(i).m_jointName)
												{
													jointName = body->m_multiBody->getLink(i).m_jointName;
												}
												if (motorNames[m]==jointName)
												{
													motorIdList.push_back(i);
												}
											}
										}

										if (motorIdList.size()==8)
										{
											int loggerUid = m_data->m_stateLoggersUniqueId++;
											MinitaurStateLogger* logger = new MinitaurStateLogger(loggerUid,fileName,body->m_multiBody, motorIdList);
											m_data->m_stateLoggers.push_back(logger);
											serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
											serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
										}
									}
								}
							}
						}
					}
					if ((clientCmd.m_updateFlags & STATE_LOGGING_STOP_LOG) && clientCmd.m_stateLoggingArguments.m_loggingUniqueId>=0)
					{
						serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
						for (int i=0;i<m_data->m_stateLoggers.size();i++)
						{
							if (m_data->m_stateLoggers[i]->m_loggingUniqueId==clientCmd.m_stateLoggingArguments.m_loggingUniqueId)
							{
								m_data->m_stateLoggers[i]->stop();
								delete m_data->m_stateLoggers[i];
								m_data->m_stateLoggers.removeAtIndex(i);
							}
						}
					}
					break;
				}
				case CMD_SET_VR_CAMERA_STATE:
				{

					if (clientCmd.m_updateFlags & 	VR_CAMERA_ROOT_POSITION)
					{
						gVRTeleportPos1[0] = clientCmd.m_vrCameraStateArguments.m_rootPosition[0];
						gVRTeleportPos1[1] = clientCmd.m_vrCameraStateArguments.m_rootPosition[1];
						gVRTeleportPos1[2] = clientCmd.m_vrCameraStateArguments.m_rootPosition[2];
					}
					if (clientCmd.m_updateFlags & VR_CAMERA_ROOT_ORIENTATION)
					{
						gVRTeleportOrn[0] = clientCmd.m_vrCameraStateArguments.m_rootOrientation[0];
						gVRTeleportOrn[1] = clientCmd.m_vrCameraStateArguments.m_rootOrientation[1];
						gVRTeleportOrn[2] = clientCmd.m_vrCameraStateArguments.m_rootOrientation[2];
						gVRTeleportOrn[3] = clientCmd.m_vrCameraStateArguments.m_rootOrientation[3];
					}

					if (clientCmd.m_updateFlags & VR_CAMERA_ROOT_TRACKING_OBJECT)
					{
						gVRTrackingObjectUniqueId = clientCmd.m_vrCameraStateArguments.m_trackingObjectUniqueId;
					}

					serverStatusOut.m_type  = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					break;
				}
				case CMD_REQUEST_VR_EVENTS_DATA:
				{
					serverStatusOut.m_sendVREvents.m_numVRControllerEvents = 0;
					for (int i=0;i<MAX_VR_CONTROLLERS;i++)
					{
						if (m_data->m_vrEvents[i].m_numButtonEvents + m_data->m_vrEvents[i].m_numMoveEvents)
						{
							serverStatusOut.m_sendVREvents.m_controllerEvents[serverStatusOut.m_sendVREvents.m_numVRControllerEvents++] = m_data->m_vrEvents[i];
							m_data->m_vrEvents[i].m_numButtonEvents = 0;
							m_data->m_vrEvents[i].m_numMoveEvents = 0;
							for (int b=0;b<MAX_VR_BUTTONS;b++)
							{
								m_data->m_vrEvents[i].m_buttons[b] = 0;
							}
						}
					}
					serverStatusOut.m_type = CMD_REQUEST_VR_EVENTS_DATA_COMPLETED;
					hasStatus = true;
					break;
				};
				case CMD_REQUEST_RAY_CAST_INTERSECTIONS:
				{
					btVector3 rayFromWorld(clientCmd.m_requestRaycastIntersections.m_rayFromPosition[0],
						clientCmd.m_requestRaycastIntersections.m_rayFromPosition[1],
						clientCmd.m_requestRaycastIntersections.m_rayFromPosition[2]);
					btVector3 rayToWorld(clientCmd.m_requestRaycastIntersections.m_rayToPosition[0],
						clientCmd.m_requestRaycastIntersections.m_rayToPosition[1],
						clientCmd.m_requestRaycastIntersections.m_rayToPosition[2]);
					btCollisionWorld::ClosestRayResultCallback rayResultCallback(rayFromWorld,rayToWorld);
					m_data->m_dynamicsWorld->rayTest(rayFromWorld,rayToWorld,rayResultCallback);
					serverStatusOut.m_raycastHits.m_numRaycastHits = 0;

					if (rayResultCallback.hasHit())
					{
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitFraction 
							= rayResultCallback.m_closestHitFraction;

						int objectUniqueId = -1;
						int linkIndex = -1;

						const btRigidBody* body = btRigidBody::upcast(rayResultCallback.m_collisionObject);
						if (body)
						{
							objectUniqueId = rayResultCallback.m_collisionObject->getUserIndex2();
						} else
						{
							const btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(rayResultCallback.m_collisionObject);
							if (mblB && mblB->m_multiBody)
							{
								linkIndex = mblB->m_link;
								objectUniqueId = mblB->m_multiBody->getUserIndex2();
							}
						}

						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitObjectUniqueId 
							= objectUniqueId;
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitObjectLinkIndex
							= linkIndex;

						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitPositionWorld[0] 
							= rayResultCallback.m_hitPointWorld[0];
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitPositionWorld[1] 
							= rayResultCallback.m_hitPointWorld[1];
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitPositionWorld[2] 
							= rayResultCallback.m_hitPointWorld[2];
						
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitNormalWorld[0] 
							= rayResultCallback.m_hitNormalWorld[0]; 
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitNormalWorld[1] 
							= rayResultCallback.m_hitNormalWorld[1]; 
						serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitNormalWorld[2] 
							= rayResultCallback.m_hitNormalWorld[2]; 

						serverStatusOut.m_raycastHits.m_numRaycastHits++;
					}
					serverStatusOut.m_type = CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED;
					hasStatus = true;
					break;
				};
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
						int bytesPerLine = (sizeof(float) * 9);
                        int maxNumLines = bufferSizeInBytes/bytesPerLine-1;
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
						serverStatusOut.m_numDataStreamBytes = numLines * bytesPerLine;
                        serverStatusOut.m_sendDebugLinesArgs.m_numDebugLines = numLines;
                        serverStatusOut.m_sendDebugLinesArgs.m_startingLineIndex = startingLineIndex;
                        serverStatusOut.m_sendDebugLinesArgs.m_numRemainingDebugLines = m_data->m_remoteDebugDrawer->m_lines2.size()-(startingLineIndex+numLines);
						hasStatus = true;

						break;
					}

				case CMD_REQUEST_CAMERA_IMAGE_DATA:
				{

					int startPixelIndex = clientCmd.m_requestPixelDataArguments.m_startPixelIndex;
                    int width = clientCmd.m_requestPixelDataArguments.m_pixelWidth;
                    int height = clientCmd.m_requestPixelDataArguments.m_pixelHeight;
                    int numPixelsCopied = 0;



					if ((clientCmd.m_updateFlags & ER_BULLET_HARDWARE_OPENGL)!=0)
					{
						//m_data->m_guiHelper->copyCameraImageData(clientCmd.m_requestPixelDataArguments.m_viewMatrix,clientCmd.m_requestPixelDataArguments.m_projectionMatrix,0,0,0,0,0,width,height,0);
					}
					else
					{
					    if ((clientCmd.m_requestPixelDataArguments.m_startPixelIndex==0) &&
                            (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT)!=0)
                        {
                            m_data->m_visualConverter.setWidthAndHeight(clientCmd.m_requestPixelDataArguments.m_pixelWidth,
                                                                        clientCmd.m_requestPixelDataArguments.m_pixelHeight);
                        }
                        m_data->m_visualConverter.getWidthAndHeight(width,height);
					}



                    int numTotalPixels = width*height;
                    int numRemainingPixels = numTotalPixels - startPixelIndex;


                    if (numRemainingPixels>0)
                    {
                        int totalBytesPerPixel = 4+4+4;//4 for rgb, 4 for depth, 4 for segmentation mask
                        int maxNumPixels = bufferSizeInBytes/totalBytesPerPixel-1;
                        unsigned char* pixelRGBA = (unsigned char*)bufferServerToClient;
                        int numRequestedPixels = btMin(maxNumPixels,numRemainingPixels);

                        float* depthBuffer = (float*)(bufferServerToClient+numRequestedPixels*4);
                        int* segmentationMaskBuffer = (int*)(bufferServerToClient+numRequestedPixels*8);

						serverStatusOut.m_numDataStreamBytes = numRequestedPixels * totalBytesPerPixel;

                        if ((clientCmd.m_updateFlags & ER_BULLET_HARDWARE_OPENGL)!=0)
						{
							m_data->m_guiHelper->copyCameraImageData(clientCmd.m_requestPixelDataArguments.m_viewMatrix,
                                                clientCmd.m_requestPixelDataArguments.m_projectionMatrix,pixelRGBA,numRequestedPixels,
                                                depthBuffer,numRequestedPixels,
                                                segmentationMaskBuffer, numRequestedPixels,
                                                startPixelIndex,width,height,&numPixelsCopied);
						} else
						{

                            if (clientCmd.m_requestPixelDataArguments.m_startPixelIndex==0)
                            {
                             //   printf("-------------------------------\nRendering\n");

								if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DIRECTION) != 0)
								{
									m_data->m_visualConverter.setLightDirection(clientCmd.m_requestPixelDataArguments.m_lightDirection[0], clientCmd.m_requestPixelDataArguments.m_lightDirection[1], clientCmd.m_requestPixelDataArguments.m_lightDirection[2]);
								}
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_COLOR) != 0)
                                {
                                    m_data->m_visualConverter.setLightColor(clientCmd.m_requestPixelDataArguments.m_lightColor[0], clientCmd.m_requestPixelDataArguments.m_lightColor[1], clientCmd.m_requestPixelDataArguments.m_lightColor[2]);
                                }
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DISTANCE) != 0)
                                {
                                    m_data->m_visualConverter.setLightDistance(clientCmd.m_requestPixelDataArguments.m_lightDistance);
                                }
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SHADOW) != 0)
                                {
                                    m_data->m_visualConverter.setShadow((clientCmd.m_requestPixelDataArguments.m_hasShadow!=0));
                                }
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_AMBIENT_COEFF) != 0)
                                {
                                    m_data->m_visualConverter.setLightAmbientCoeff(clientCmd.m_requestPixelDataArguments.m_lightAmbientCoeff);
                                }
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_DIFFUSE_COEFF) != 0)
                                {
                                    m_data->m_visualConverter.setLightDiffuseCoeff(clientCmd.m_requestPixelDataArguments.m_lightDiffuseCoeff);
                                }
                                
                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SPECULAR_COEFF) != 0)
                                {
                                    m_data->m_visualConverter.setLightSpecularCoeff(clientCmd.m_requestPixelDataArguments.m_lightSpecularCoeff);
                                }

                                if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES)!=0)
                                {
                                    m_data->m_visualConverter.render(
                                                                     clientCmd.m_requestPixelDataArguments.m_viewMatrix,
                                                                     clientCmd.m_requestPixelDataArguments.m_projectionMatrix);
                                } else
                                {
                                    m_data->m_visualConverter.render();
                                }

                            }

							m_data->m_visualConverter.copyCameraImageData(pixelRGBA,numRequestedPixels,
                                                     depthBuffer,numRequestedPixels,
                                                     segmentationMaskBuffer, numRequestedPixels,
                                                     startPixelIndex,&width,&height,&numPixelsCopied);
						}

                        //each pixel takes 4 RGBA values and 1 float = 8 bytes

                    } else
                    {

                    }

                    serverStatusOut.m_type = CMD_CAMERA_IMAGE_COMPLETED;
					
                    serverStatusOut.m_sendPixelDataArguments.m_numPixelsCopied = numPixelsCopied;
					serverStatusOut.m_sendPixelDataArguments.m_numRemainingPixels = numRemainingPixels - numPixelsCopied;
					serverStatusOut.m_sendPixelDataArguments.m_startingPixelIndex = startPixelIndex;
					serverStatusOut.m_sendPixelDataArguments.m_imageWidth = width;
					serverStatusOut.m_sendPixelDataArguments.m_imageHeight= height;
					hasStatus = true;

					break;
				}

				case CMD_SYNC_BODY_INFO:
				{
					int numHandles = m_data->getNumHandles();
					int actualNumBodies = 0;
					for (int i=0;i<numHandles;i++)
					{
						InteralBodyData* body = m_data->getHandle(i);
						if (body && (body->m_multiBody || body->m_rigidBody))
						{
							serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[actualNumBodies++] = i;
						}
					}
					serverStatusOut.m_sdfLoadedArgs.m_numBodies = actualNumBodies;

					int usz = m_data->m_userConstraints.size();
					serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = usz;
					for (int i=0;i<usz;i++)
					{

						int key = m_data->m_userConstraints.getKeyAtIndex(i).getUid1();
//						int uid = m_data->m_userConstraints.getAtIndex(i)->m_userConstraintData.m_userConstraintUniqueId;
						serverStatusOut.m_sdfLoadedArgs.m_userConstraintUniqueIds[i] = key;
					}

					serverStatusOut.m_type = CMD_SYNC_BODY_INFO_COMPLETED;
					hasStatus = true;
					break;
				}
                case CMD_REQUEST_BODY_INFO:
                    {
                        const SdfRequestInfoArgs& sdfInfoArgs = clientCmd.m_sdfRequestInfoArgs;
                        //stream info into memory
                        int streamSizeInBytes = createBodyInfoStream(sdfInfoArgs.m_bodyUniqueId, bufferServerToClient, bufferSizeInBytes);

                        serverStatusOut.m_type = CMD_BODY_INFO_COMPLETED;
                        serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = sdfInfoArgs.m_bodyUniqueId;
                        serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;
						
                        hasStatus = true;
                        break;
                    }
				case CMD_SAVE_WORLD:
				{
					///this is a very rudimentary way to save the state of the world, for scene authoring
					///many todo's, for example save the state of motor controllers etc.

					
					{
						//saveWorld(clientCmd.m_sdfArguments.m_sdfFileName);
						int constraintCount = 0;
						FILE* f = fopen(clientCmd.m_sdfArguments.m_sdfFileName,"w");
						if (f)
						{
							char line[1024];
							{
								sprintf(line,"import pybullet as p\n");
								int len = strlen(line);
								fwrite(line,len,1,f);
							}
							{
								sprintf(line,"p.connect(p.SHARED_MEMORY)\n");
								int len = strlen(line);
								fwrite(line,len,1,f);
							}
							//for each objects ...
							for (int i=0;i<m_data->m_saveWorldBodyData.size();i++)
							{
								SaveWorldObjectData& sd = m_data->m_saveWorldBodyData[i];

								for (int i=0;i<sd.m_bodyUniqueIds.size();i++)
								{
									{
										int bodyUniqueId = sd.m_bodyUniqueIds[i];
										InteralBodyData* body = m_data->getHandle(bodyUniqueId);
										if (body)
										{
											 if (body->m_multiBody)
											 {
												btMultiBody* mb = body->m_multiBody;
												
												btTransform comTr = mb->getBaseWorldTransform();
												btTransform tr = comTr * body->m_rootLocalInertialFrame.inverse();
												
												if (strstr(sd.m_fileName.c_str(),".urdf"))
												{
													sprintf(line,"objects = [p.loadURDF(\"%s\", %f,%f,%f,%f,%f,%f,%f)]\n",sd.m_fileName.c_str(),
														tr.getOrigin()[0],tr.getOrigin()[1],tr.getOrigin()[2],
														tr.getRotation()[0],tr.getRotation()[1],tr.getRotation()[2],tr.getRotation()[3]);
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (strstr(sd.m_fileName.c_str(),".sdf") && i==0)
												{
													sprintf(line,"objects = p.loadSDF(\"%s\")\n",sd.m_fileName.c_str());
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (strstr(sd.m_fileName.c_str(),".sdf") || ((strstr(sd.m_fileName.c_str(),".urdf")) && mb->getNumLinks()) )
												{
													sprintf(line,"ob = objects[%d]\n",i);
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (strstr(sd.m_fileName.c_str(),".sdf"))
												{
													sprintf(line,"p.resetBasePositionAndOrientation(ob,[%f,%f,%f],[%f,%f,%f,%f])\n",
														comTr.getOrigin()[0],comTr.getOrigin()[1],comTr.getOrigin()[2],
														comTr.getRotation()[0],comTr.getRotation()[1],comTr.getRotation()[2],comTr.getRotation()[3]);
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (mb->getNumLinks())
												{
													{
														sprintf(line,"jointPositions=[");
														int len = strlen(line);
														fwrite(line,len,1,f);
													}

													for (int i=0;i<mb->getNumLinks();i++)
													{
														btScalar jointPos = mb->getJointPosMultiDof(i)[0];
														if (i<mb->getNumLinks()-1)
														{
															sprintf(line," %f,",jointPos);
															int len = strlen(line);
															fwrite(line,len,1,f);
														} else
														{
															sprintf(line," %f ",jointPos);
															int len = strlen(line);
															fwrite(line,len,1,f);
														}
													}

													{
														sprintf(line,"]\nfor jointIndex in range (p.getNumJoints(ob)):\n\tp.resetJointState(ob,jointIndex,jointPositions[jointIndex])\n\n");
														int len = strlen(line);
														fwrite(line,len,1,f);
													}
												}
											 } else
											 {
												 //todo: btRigidBody/btSoftBody etc case
											 }
										}
									}
									
								}
								
								//for URDF, load at origin, then reposition...
								

								struct SaveWorldObjectData
								{
									b3AlignedObjectArray<int> m_bodyUniqueIds;
									std::string	m_fileName;
								};
							}

							//user constraints
							{
								for (int i=0;i<m_data->m_userConstraints.size();i++)
								{
									InteralUserConstraintData* ucptr = m_data->m_userConstraints.getAtIndex(i);
									b3UserConstraint& uc = ucptr->m_userConstraintData;

									int parentBodyIndex=uc.m_parentBodyIndex;
									int parentJointIndex=uc.m_parentJointIndex;
									int childBodyIndex=uc.m_childBodyIndex;
									int childJointIndex=uc.m_childJointIndex;
									btVector3 jointAxis(uc.m_jointAxis[0],uc.m_jointAxis[1],uc.m_jointAxis[2]);
									btVector3 pivotParent(uc.m_parentFrame[0],uc.m_parentFrame[1],uc.m_parentFrame[2]);
									btVector3 pivotChild(uc.m_childFrame[0],uc.m_childFrame[1],uc.m_childFrame[2]);
									btQuaternion ornFrameParent(uc.m_parentFrame[3],uc.m_parentFrame[4],uc.m_parentFrame[5],uc.m_parentFrame[6]);
									btQuaternion ornFrameChild(uc.m_childFrame[3],uc.m_childFrame[4],uc.m_childFrame[5],uc.m_childFrame[6]);
									{
										char jointTypeStr[1024]="FIXED";
										bool hasKnownJointType = true;

										switch (uc.m_jointType)
										{
											case eRevoluteType:
											{
												sprintf(jointTypeStr,"p.JOINT_REVOLUTE");
												break;
											}
											case ePrismaticType:
											{
												sprintf(jointTypeStr,"p.JOINT_PRISMATIC");
												break;
											}
											case eSphericalType:
											{
												sprintf(jointTypeStr,"p.JOINT_SPHERICAL");
												break;
											}
											case ePlanarType:
											{
												sprintf(jointTypeStr,"p.JOINT_PLANAR");
												break;
											}
											case eFixedType :
											{
												sprintf(jointTypeStr,"p.JOINT_FIXED");
												break;
											}
											case ePoint2PointType:
											{
												sprintf(jointTypeStr,"p.JOINT_POINT2POINT");
												break;											}
											default:
											{
												hasKnownJointType = false;
												b3Warning("unknown constraint type in SAVE_WORLD");
											}
										};
										if (hasKnownJointType)
										{
											{
												sprintf(line,"cid%d = p.createConstraint(%d,%d,%d,%d,%s,[%f,%f,%f],[%f,%f,%f],[%f,%f,%f],[%f,%f,%f,%f],[%f,%f,%f,%f])\n",
													constraintCount,
													parentBodyIndex,
													parentJointIndex,
													childBodyIndex,
													childJointIndex,
													jointTypeStr,
													jointAxis[0],jointAxis[1],jointAxis[2],
													pivotParent[0],pivotParent[1],pivotParent[2],
													pivotChild[0],pivotChild[1],pivotChild[2],
													ornFrameParent[0],ornFrameParent[1],ornFrameParent[2],ornFrameParent[3],
													ornFrameChild[0],ornFrameChild[1],ornFrameChild[2],ornFrameChild[3]
													);
												int len = strlen(line);
												fwrite(line,len,1,f);
											}
											{
												sprintf(line,"p.changeConstraint(cid%d,maxForce=%f)\n",constraintCount,uc.m_maxAppliedForce);
												int len = strlen(line);
												fwrite(line,len,1,f);
												constraintCount++;
											}
										}
									}
								}
							}

							{
								btVector3 grav=this->m_data->m_dynamicsWorld->getGravity();
								sprintf(line,"p.setGravity(%f,%f,%f)\n",grav[0],grav[1],grav[2]);
								int len = strlen(line);
								fwrite(line,len,1,f);
							}
														

							{
									sprintf(line,"p.stepSimulation()\np.disconnect()\n");
									int len = strlen(line);
									fwrite(line,len,1,f);
							}
							fclose(f);
						}


						serverStatusOut.m_type = CMD_SAVE_WORLD_COMPLETED;
						hasStatus = true;
						break;
					}
					serverStatusOut.m_type = CMD_SAVE_WORLD_FAILED;
					hasStatus = true;
					break;
				}
                case CMD_LOAD_SDF:
                    {
                        const SdfArgs& sdfArgs = clientCmd.m_sdfArguments;
                        if (m_data->m_verboseOutput)
                        {
                            b3Printf("Processed CMD_LOAD_SDF:%s", sdfArgs.m_sdfFileName);
                        }
                        bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (sdfArgs.m_useMultiBody!=0) : true;

						int flags = CUF_USE_SDF; //CUF_USE_URDF_INERTIA
                        bool completedOk = loadSdf(sdfArgs.m_sdfFileName,bufferServerToClient, bufferSizeInBytes, useMultiBody, flags);
                        if (completedOk)
                        {
							m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

                            //serverStatusOut.m_type = CMD_SDF_LOADING_FAILED;
                            serverStatusOut.m_sdfLoadedArgs.m_numBodies = m_data->m_sdfRecentLoadedBodies.size();
							serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
                            int maxBodies = btMin(MAX_SDF_BODIES, serverStatusOut.m_sdfLoadedArgs.m_numBodies);
                            for (int i=0;i<maxBodies;i++)
                            {
                                serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = m_data->m_sdfRecentLoadedBodies[i];
                            }

                            serverStatusOut.m_type = CMD_SDF_LOADING_COMPLETED;
                        } else
                        {
                            serverStatusOut.m_type = CMD_SDF_LOADING_FAILED;
                        }
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
					bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (urdfArgs.m_useMultiBody!=0) : true;
					bool useFixedBase = (clientCmd.m_updateFlags & URDF_ARGS_USE_FIXED_BASE) ? (urdfArgs.m_useFixedBase!=0): false;
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
							serverStatusOut.m_numDataStreamBytes = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
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
                case CMD_LOAD_BUNNY:
                {
#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
                    double scale = 0.1;
                    double mass = 0.1;
                    double collisionMargin = 0.02;
                    if (clientCmd.m_updateFlags & LOAD_BUNNY_UPDATE_SCALE)
                    {
                        scale = clientCmd.m_loadBunnyArguments.m_scale;
                    }
                    if (clientCmd.m_updateFlags & LOAD_BUNNY_UPDATE_MASS)
                    {
                        mass = clientCmd.m_loadBunnyArguments.m_mass;
                    }
                    if (clientCmd.m_updateFlags & LOAD_BUNNY_UPDATE_COLLISION_MARGIN)
                    {
                        collisionMargin = clientCmd.m_loadBunnyArguments.m_collisionMargin;
                    }
                    m_data->m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
                    m_data->m_softBodyWorldInfo.water_density	=	0;
                    m_data->m_softBodyWorldInfo.water_offset	=	0;
                    m_data->m_softBodyWorldInfo.water_normal	=	btVector3(0,0,0);
                    m_data->m_softBodyWorldInfo.m_gravity.setValue(0,0,-10);
                    m_data->m_softBodyWorldInfo.m_broadphase = m_data->m_broadphase;
                    m_data->m_softBodyWorldInfo.m_sparsesdf.Initialize();
                    
                    btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(m_data->m_softBodyWorldInfo,gVerticesBunny,                                                       &gIndicesBunny[0][0],                                                         BUNNY_NUM_TRIANGLES);
                    
                    btSoftBody::Material*	pm=psb->appendMaterial();
                    pm->m_kLST				=	1.0;
                    pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
                    psb->generateBendingConstraints(2,pm);
                    psb->m_cfg.piterations	=	50;
                    psb->m_cfg.kDF			=	0.5;
                    psb->randomizeConstraints();
                    psb->rotate(btQuaternion(0.70711,0,0,0.70711));
                    psb->translate(btVector3(0,0,1.0));
                    psb->scale(btVector3(scale,scale,scale));
                    psb->setTotalMass(mass,true);
                    psb->getCollisionShape()->setMargin(collisionMargin);
                    
                    m_data->m_dynamicsWorld->addSoftBody(psb);
#endif
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
                                      //  mb->clearForcesAndTorques();
                                        int torqueIndex = 6;
                                        if ((clientCmd.m_updateFlags&SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
                                        {
                                            for (int link=0;link<mb->getNumLinks();link++)
                                            {

                                                for (int dof=0;dof<mb->getLink(link).m_dofCount;dof++)
                                                {
                                                    double torque = 0.f;
                                                    if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[torqueIndex]&SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
                                                    {
                                                        torque = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[torqueIndex];
                                                        mb->addJointTorqueMultiDof(link,dof,torque);
                                                    }
                                                    torqueIndex++;
                                                }
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

                                                    btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;


                                                    if (motor)
													{
														btScalar desiredVelocity = 0.f;
                                                        bool hasDesiredVelocity = false;


														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex]&SIM_DESIRED_STATE_HAS_QDOT)!=0)
                                                        {
															desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex];
                                                            btScalar kd = 0.1f;
                                                            if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] & SIM_DESIRED_STATE_HAS_KD)!=0)
                                                            {
                                                                kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[dofIndex];
                                                            }

                                                            motor->setVelocityTarget(desiredVelocity,kd);

                                                            btScalar kp = 0.f;
                                                            motor->setPositionTarget(0,kp);
                                                            hasDesiredVelocity = true;
                                                        }
                                                        if (hasDesiredVelocity)
                                                        {
                                                            btScalar maxImp = 1000000.f*m_data->m_physicsDeltaTime;
                                                            if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex]&SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
                                                            {
                                                                maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex]*m_data->m_physicsDeltaTime;
                                                            }
                                                            motor->setMaxAppliedImpulse(maxImp);
                                                        }
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

										int numMotors = 0;
										//find the joint motors and apply the desired velocity and maximum force/torque
										{
											int velIndex = 6;//skip the 3 linear + 3 angular degree of freedom velocity entries of the base
											int posIndex = 7;//skip 3 positional and 4 orientation (quaternion) positional degrees of freedom of the base
											for (int link=0;link<mb->getNumLinks();link++)
											{
												if (supportsJointMotor(mb,link))
												{


                                                    btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;

                                                	if (motor)
													{

                                                        bool hasDesiredPosOrVel = false;
                                                        btScalar kp = 0.f;
                                                        btScalar kd = 0.f;
                                                        btScalar desiredVelocity = 0.f;
														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT)!=0)
                                                        {
                                                            hasDesiredPosOrVel = true;
															desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
                                                            kd = 0.1;
                                                        }
														btScalar desiredPosition = 0.f;
														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q)!=0)
                                                        {
                                                            hasDesiredPosOrVel = true;
															desiredPosition = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex];
                                                            kp = 0.1;
                                                        }

                                                        if (hasDesiredPosOrVel)
                                                        {

                                                            if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KP)!=0)
                                                            {
                                                                kp = clientCmd.m_sendDesiredStateCommandArgument.m_Kp[velIndex];
                                                            }

                                                            if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KD)!=0)
                                                            {
                                                                kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[velIndex];
                                                            }

                                                            motor->setVelocityTarget(desiredVelocity,kd);
                                                            motor->setPositionTarget(desiredPosition,kp);

                                                            btScalar maxImp = 1000000.f*m_data->m_physicsDeltaTime;

                                                            if ((clientCmd.m_updateFlags & SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
                                                                maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex]*m_data->m_physicsDeltaTime;

                                                            motor->setMaxAppliedImpulse(maxImp);
                                                        }
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

							if (mb->getNumLinks()>= MAX_DEGREE_OF_FREEDOM)
							{
								serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
								hasStatus = true;
								break;
							}

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

                                serverCmd.m_sendActualStateArgs.m_jointMotorForce[l] = 0;

                                if (supportsJointMotor(mb,l))
                                {

                                    btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)body->m_multiBody->getLink(l).m_userPtr;

                                    if (motor && m_data->m_physicsDeltaTime>btScalar(0))
                                    {
                                        btScalar force =motor->getAppliedImpulse(0)/m_data->m_physicsDeltaTime;
                                        serverCmd.m_sendActualStateArgs.m_jointMotorForce[l] =
                                        force;
                                        //if (force>0)
                                        //{
                                        //   b3Printf("force = %f\n", force);
                                        //}
                                    }
                                }
                btVector3 linkLocalInertialOrigin = body->m_linkLocalInertialFrames[l].getOrigin();
                btQuaternion linkLocalInertialRotation = body->m_linkLocalInertialFrames[l].getRotation();

                btVector3 linkCOMOrigin =  mb->getLink(l).m_cachedWorldTransform.getOrigin();
                btQuaternion linkCOMRotation =  mb->getLink(l).m_cachedWorldTransform.getRotation();

                serverCmd.m_sendActualStateArgs.m_linkState[l*7+0] = linkCOMOrigin.getX();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+1] = linkCOMOrigin.getY();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+2] = linkCOMOrigin.getZ();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+3] = linkCOMRotation.x();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+4] = linkCOMRotation.y();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+5] = linkCOMRotation.z();
                serverCmd.m_sendActualStateArgs.m_linkState[l*7+6] = linkCOMRotation.w();

                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+0] = linkLocalInertialOrigin.getX();
                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+1] = linkLocalInertialOrigin.getY();
                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+2] = linkLocalInertialOrigin.getZ();

                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+3] = linkLocalInertialRotation.x();
                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+4] = linkLocalInertialRotation.y();
                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+5] = linkLocalInertialRotation.z();
                serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+6] = linkLocalInertialRotation.w();

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
						b3Printf("CMD_STEP_FORWARD_SIMULATION clientCmd = %d\n", clientCmd.m_sequenceNumber);
					}
                    ///todo(erwincoumans) move this damping inside Bullet
                    for (int i=0;i<m_data->m_bodyHandles.size();i++)
                    {
                        applyJointDamping(i);
                    }
					
					
					

					btScalar deltaTimeScaled = m_data->m_physicsDeltaTime*simTimeScalingFactor;

					if (m_data->m_numSimulationSubSteps > 0)
					{
						m_data->m_dynamicsWorld->stepSimulation(deltaTimeScaled, m_data->m_numSimulationSubSteps, m_data->m_physicsDeltaTime / m_data->m_numSimulationSubSteps);
					}
					else
					{
						m_data->m_dynamicsWorld->stepSimulation(deltaTimeScaled, 0);
					}

					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;
					hasStatus = true;
                    break;
                }

				case CMD_REQUEST_INTERNAL_DATA:
				{
					//todo: also check version etc?

					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverCmd.m_type = CMD_REQUEST_INTERNAL_DATA_FAILED;
					hasStatus = true;
					
					int sz = btDefaultSerializer::getMemoryDnaSizeInBytes();
					const char* memDna = btDefaultSerializer::getMemoryDna();
					if (sz < bufferSizeInBytes)
					{
						for (int i = 0; i < sz; i++)
						{
							bufferServerToClient[i] = memDna[i];
						}
						serverCmd.m_type = CMD_REQUEST_INTERNAL_DATA_COMPLETED;
						serverCmd.m_numDataStreamBytes = sz;
					}

					break;
				};
				case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				{
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DELTA_TIME)
					{
						m_data->m_physicsDeltaTime = clientCmd.m_physSimParamArgs.m_deltaTime;
					}
					if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_REAL_TIME_SIMULATION)
					{
						m_data->m_allowRealTimeSimulation = clientCmd.m_physSimParamArgs.m_allowRealTimeSimulation;
					}
					
					//see 
					if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS)
					{
						//these flags are for internal/temporary/easter-egg/experimental demo purposes, use at own risk
						gCreateDefaultRobotAssets = (clientCmd.m_physSimParamArgs.m_internalSimFlags & SIM_PARAM_INTERNAL_CREATE_ROBOT_ASSETS);
						gInternalSimFlags = clientCmd.m_physSimParamArgs.m_internalSimFlags;
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
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS)
					{
						m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = clientCmd.m_physSimParamArgs.m_numSolverIterations;
					}
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD)
					{
						gContactBreakingThreshold = clientCmd.m_physSimParamArgs.m_contactBreakingThreshold;
					}
					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_COLLISION_FILTER_MODE)
					{
						m_data->m_broadphaseCollisionFilterCallback->m_filterMode = clientCmd.m_physSimParamArgs.m_collisionFilterMode;
					}

					if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_USE_SPLIT_IMPULSE)
					{
						m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulse = clientCmd.m_physSimParamArgs.m_useSplitImpulse;
					}
					if (clientCmd.m_updateFlags &SIM_PARAM_UPDATE_SPLIT_IMPULSE_PENETRATION_THRESHOLD)
					{
						m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = clientCmd.m_physSimParamArgs.m_splitImpulsePenetrationThreshold;
					}
					

                    if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS)
                    {
                        m_data->m_numSimulationSubSteps = clientCmd.m_physSimParamArgs.m_numSimulationSubSteps;
                    }

					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DEFAULT_CONTACT_ERP)
                    {
                        m_data->m_dynamicsWorld->getSolverInfo().m_erp2 = clientCmd.m_physSimParamArgs.m_defaultContactERP;
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

					btVector3 baseLinVel(0, 0, 0);
					btVector3 baseAngVel(0, 0, 0);

					if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_LINEAR_VELOCITY)
					{
						baseLinVel.setValue(clientCmd.m_initPoseArgs.m_initialStateQdot[0],
							clientCmd.m_initPoseArgs.m_initialStateQdot[1],
							clientCmd.m_initPoseArgs.m_initialStateQdot[2]);
					}
					if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_ANGULAR_VELOCITY)
					{
						baseAngVel.setValue(clientCmd.m_initPoseArgs.m_initialStateQdot[3],
							clientCmd.m_initPoseArgs.m_initialStateQdot[4],
							clientCmd.m_initPoseArgs.m_initialStateQdot[5]);
					}
					btVector3 basePos(0, 0, 0);
					if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_POSITION)
					{
						basePos = btVector3(
							clientCmd.m_initPoseArgs.m_initialStateQ[0],
							clientCmd.m_initPoseArgs.m_initialStateQ[1],
							clientCmd.m_initPoseArgs.m_initialStateQ[2]);
					}
					btQuaternion baseOrn(0, 0, 0, 1);
					if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_ORIENTATION)
					{
						baseOrn.setValue(clientCmd.m_initPoseArgs.m_initialStateQ[3],
							clientCmd.m_initPoseArgs.m_initialStateQ[4],
							clientCmd.m_initPoseArgs.m_initialStateQ[5],
							clientCmd.m_initPoseArgs.m_initialStateQ[6]);
					}
					if (body && body->m_multiBody)
					{
						btMultiBody* mb = body->m_multiBody;
						


						if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_LINEAR_VELOCITY)
						{
							mb->setBaseVel(baseLinVel);
						}

						if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_ANGULAR_VELOCITY)
						{
							mb->setBaseOmega(baseAngVel);
						}


						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_POSITION)
						{
							btVector3 zero(0,0,0);
							btAssert(clientCmd.m_initPoseArgs.m_hasInitialStateQ[0] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[1] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[2]);

							mb->setBaseVel(baseLinVel);
							mb->setBasePos(basePos);
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_ORIENTATION)
						{
						    btAssert(clientCmd.m_initPoseArgs.m_hasInitialStateQ[3] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[4] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[5] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[6]);

							mb->setBaseOmega(baseAngVel);
							btQuaternion invOrn(baseOrn);

							mb->setWorldToBaseRot(invOrn.inverse());
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_JOINT_STATE)
						{
							int dofIndex = 7;
							for (int i=0;i<mb->getNumLinks();i++)
							{
							    if ( (clientCmd.m_initPoseArgs.m_hasInitialStateQ[dofIndex]) && (mb->getLink(i).m_dofCount==1))
								{
									mb->setJointPos(i,clientCmd.m_initPoseArgs.m_initialStateQ[dofIndex]);
									mb->setJointVel(i,0);
								}
								dofIndex += mb->getLink(i).m_dofCount;
							}
						}
                        
                        btAlignedObjectArray<btQuaternion> scratch_q;
                        btAlignedObjectArray<btVector3> scratch_m;
                        
                        mb->forwardKinematics(scratch_q,scratch_m);
                        int nLinks = mb->getNumLinks();
                        scratch_q.resize(nLinks+1);
                        scratch_m.resize(nLinks+1);
                        
                        mb->updateCollisionObjectWorldTransforms(scratch_q,scratch_m);

                        
					}

					if (body && body->m_rigidBody)
					{
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_LINEAR_VELOCITY)
						{
							body->m_rigidBody->setLinearVelocity(baseLinVel);
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_BASE_ANGULAR_VELOCITY)
						{
							body->m_rigidBody->setAngularVelocity(baseAngVel);
						}
						
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_POSITION)
						{
							body->m_rigidBody->getWorldTransform().setOrigin(basePos);
							body->m_rigidBody->setLinearVelocity(baseLinVel);
						}

						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_ORIENTATION)
						{
							body->m_rigidBody->getWorldTransform().setRotation(baseOrn);
							body->m_rigidBody->setAngularVelocity(baseAngVel);
						}

					}

					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;

					break;
				}


                case CMD_RESET_SIMULATION:
                {
					
					resetSimulation();

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
						rb->setUserIndex2(bodyUniqueId);
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
				case CMD_REQUEST_AABB_OVERLAP:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					int curObjectIndex = clientCmd.m_requestOverlappingObjectsArgs.m_startingOverlappingObjectIndex;

					if (0== curObjectIndex)
					{
						//clientCmd.m_requestContactPointArguments.m_aabbQueryMin
						btVector3 aabbMin, aabbMax;
						aabbMin.setValue(clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMin[0],
							clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMin[1],
							clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMin[2]);
						aabbMax.setValue(clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMax[0],
							clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMax[1],
							clientCmd.m_requestOverlappingObjectsArgs.m_aabbQueryMax[2]);

						m_data->m_cachedOverlappingObjects.clear();

						m_data->m_dynamicsWorld->getBroadphase()->aabbTest(aabbMin, aabbMax, m_data->m_cachedOverlappingObjects);
					}
					

					int totalBytesPerObject = sizeof(b3OverlappingObject);
					int overlapCapacity = bufferSizeInBytes / totalBytesPerObject - 1;
					int numOverlap =  m_data->m_cachedOverlappingObjects.m_bodyUniqueIds.size();
					int remainingObjects = numOverlap - curObjectIndex;

					int curNumObjects = btMin(overlapCapacity, remainingObjects);

					if (numOverlap < overlapCapacity)
					{
						
						b3OverlappingObject* overlapStorage = (b3OverlappingObject*)bufferServerToClient;
						for (int i = 0; i < m_data->m_cachedOverlappingObjects.m_bodyUniqueIds.size(); i++)
						{
							overlapStorage[i].m_objectUniqueId = m_data->m_cachedOverlappingObjects.m_bodyUniqueIds[i];
							overlapStorage[i].m_linkIndex = m_data->m_cachedOverlappingObjects.m_links[i];
						}

						serverCmd.m_type = CMD_REQUEST_AABB_OVERLAP_COMPLETED;

						//int m_startingOverlappingObjectIndex;
						//int m_numOverlappingObjectsCopied;
						//int m_numRemainingOverlappingObjects;
						serverCmd.m_sendOverlappingObjectsArgs.m_startingOverlappingObjectIndex = clientCmd.m_requestOverlappingObjectsArgs.m_startingOverlappingObjectIndex;
						serverCmd.m_sendOverlappingObjectsArgs.m_numOverlappingObjectsCopied = m_data->m_cachedOverlappingObjects.m_bodyUniqueIds.size();
						serverCmd.m_sendOverlappingObjectsArgs.m_numRemainingOverlappingObjects = remainingObjects - curNumObjects;
					}
					else
					{
						serverCmd.m_type = CMD_REQUEST_AABB_OVERLAP_FAILED;
					}

					hasStatus = true;
					break;
				}
                case CMD_REQUEST_CONTACT_POINT_INFORMATION:
                    {
                        SharedMemoryStatus& serverCmd =serverStatusOut;
                        serverCmd.m_sendContactPointArgs.m_numContactPointsCopied = 0;
                        
                        //make a snapshot of the contact manifolds into individual contact points
						if (clientCmd.m_requestContactPointArguments.m_startingContactPointIndex == 0)
						{
							m_data->m_cachedContactPoints.resize(0);

							int mode = CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS;

							if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_QUERY_MODE)
							{
								mode = clientCmd.m_requestContactPointArguments.m_mode;
							}

							switch (mode)
							{
							case CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS:
							{
								int numContactManifolds = m_data->m_dynamicsWorld->getDispatcher()->getNumManifolds();
								m_data->m_cachedContactPoints.reserve(numContactManifolds * 4);
								for (int i = 0; i < numContactManifolds; i++)
								{
									const btPersistentManifold* manifold = m_data->m_dynamicsWorld->getDispatcher()->getInternalManifoldPointer()[i];
									int linkIndexA = -1;
									int linkIndexB = -1;

									int objectIndexB = -1;

									const btRigidBody* bodyB = btRigidBody::upcast(manifold->getBody1());
									if (bodyB)
									{
										objectIndexB = bodyB->getUserIndex2();
									}
									const btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(manifold->getBody1());
									if (mblB && mblB->m_multiBody)
									{
										linkIndexB = mblB->m_link;
										objectIndexB = mblB->m_multiBody->getUserIndex2();
									}

									int objectIndexA = -1;
									const btRigidBody* bodyA = btRigidBody::upcast(manifold->getBody0());
									if (bodyA)
									{
										objectIndexA = bodyA->getUserIndex2();
									}
									const btMultiBodyLinkCollider* mblA = btMultiBodyLinkCollider::upcast(manifold->getBody0());
									if (mblA && mblA->m_multiBody)
									{
										linkIndexA = mblA->m_link;

										objectIndexA = mblA->m_multiBody->getUserIndex2();
									}

									btAssert(bodyA || mblA);

									//apply the filter, if the user provides it
									if (clientCmd.m_requestContactPointArguments.m_objectAIndexFilter >= 0)
									{
										if ((clientCmd.m_requestContactPointArguments.m_objectAIndexFilter != objectIndexA) &&
											(clientCmd.m_requestContactPointArguments.m_objectAIndexFilter != objectIndexB))
											continue;
									}

									//apply the second object filter, if the user provides it
									if (clientCmd.m_requestContactPointArguments.m_objectBIndexFilter >= 0)
									{
										if ((clientCmd.m_requestContactPointArguments.m_objectBIndexFilter != objectIndexA) &&
											(clientCmd.m_requestContactPointArguments.m_objectBIndexFilter != objectIndexB))
											continue;
									}

									for (int p = 0; p < manifold->getNumContacts(); p++)
									{

										b3ContactPointData pt;
										pt.m_bodyUniqueIdA = objectIndexA;
										pt.m_bodyUniqueIdB = objectIndexB;
										const btManifoldPoint& srcPt = manifold->getContactPoint(p);
										pt.m_contactDistance = srcPt.getDistance();
										pt.m_contactFlags = 0;
										pt.m_linkIndexA = linkIndexA;
										pt.m_linkIndexB = linkIndexB;
										for (int j = 0; j < 3; j++)
										{
											pt.m_contactNormalOnBInWS[j] = srcPt.m_normalWorldOnB[j];
											pt.m_positionOnAInWS[j] = srcPt.getPositionWorldOnA()[j];
											pt.m_positionOnBInWS[j] = srcPt.getPositionWorldOnB()[j];
										}
										pt.m_normalForce = srcPt.getAppliedImpulse() / m_data->m_physicsDeltaTime;
										//                                    pt.m_linearFrictionForce = srcPt.m_appliedImpulseLateral1;
										m_data->m_cachedContactPoints.push_back(pt);
									}
								}
								break;
							}
						
							case CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS:
							{
								//todo(erwincoumans) compute closest points between all, and vs all, pair
								btScalar closestDistanceThreshold = 0.f;

								if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_CLOSEST_DISTANCE_THRESHOLD)
								{
									closestDistanceThreshold = clientCmd.m_requestContactPointArguments.m_closestDistanceThreshold;
								}
								
								int bodyUniqueIdA = clientCmd.m_requestContactPointArguments.m_objectAIndexFilter;
								int bodyUniqueIdB = clientCmd.m_requestContactPointArguments.m_objectBIndexFilter;

								bool hasLinkIndexAFilter = (0!=(clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_A_FILTER));
								bool hasLinkIndexBFilter = (0!=(clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_B_FILTER));

								int linkIndexA  = clientCmd.m_requestContactPointArguments.m_linkIndexAIndexFilter;
								int linkIndexB = clientCmd.m_requestContactPointArguments.m_linkIndexBIndexFilter;

								btAlignedObjectArray<btCollisionObject*> setA;
								btAlignedObjectArray<btCollisionObject*> setB;
								btAlignedObjectArray<int> setALinkIndex;
								btAlignedObjectArray<int> setBLinkIndex;
								
								if (bodyUniqueIdA >= 0)
								{
									InteralBodyData* bodyA = m_data->getHandle(bodyUniqueIdA);
									if (bodyA)
									{
										if (bodyA->m_multiBody)
										{
											if (bodyA->m_multiBody->getBaseCollider())
											{
												if (!hasLinkIndexAFilter || (linkIndexA == -1))
												{
													setA.push_back(bodyA->m_multiBody->getBaseCollider());
													setALinkIndex.push_back(-1);
												}
											}
											for (int i = 0; i < bodyA->m_multiBody->getNumLinks(); i++)
											{
												if (bodyA->m_multiBody->getLink(i).m_collider)
												{
													if (!hasLinkIndexAFilter || (linkIndexA == i))
													{
														setA.push_back(bodyA->m_multiBody->getLink(i).m_collider);
														setALinkIndex.push_back(i);
													}
												}
											}
										}
										if (bodyA->m_rigidBody)
										{
											setA.push_back(bodyA->m_rigidBody);
											setALinkIndex.push_back(-1);
										}
									}
								}
								if (bodyUniqueIdB>=0)
								{
									InteralBodyData* bodyB = m_data->getHandle(bodyUniqueIdB);
									if (bodyB)
									{
										if (bodyB->m_multiBody)
										{
											if (bodyB->m_multiBody->getBaseCollider())
											{
												if (!hasLinkIndexBFilter || (linkIndexB == -1))
												{
													setB.push_back(bodyB->m_multiBody->getBaseCollider());
													setBLinkIndex.push_back(-1);
												}
											}
											for (int i = 0; i < bodyB->m_multiBody->getNumLinks(); i++)
											{
												if (bodyB->m_multiBody->getLink(i).m_collider)
												{
													if (!hasLinkIndexBFilter || (linkIndexB ==i))
													{
														setB.push_back(bodyB->m_multiBody->getLink(i).m_collider);
														setBLinkIndex.push_back(i);
													}
												}
											}
										}
										if (bodyB->m_rigidBody)
										{
											setB.push_back(bodyB->m_rigidBody);
											setBLinkIndex.push_back(-1);

										}
									}
								}

								{
									///ContactResultCallback is used to report contact points
									struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
									{
										int m_bodyUniqueIdA;
										int m_bodyUniqueIdB;
										int m_linkIndexA;
										int m_linkIndexB;
										btScalar m_deltaTime;

										btAlignedObjectArray<b3ContactPointData>& m_cachedContactPoints;

										MyContactResultCallback(btAlignedObjectArray<b3ContactPointData>& pointCache)
										:m_cachedContactPoints(pointCache)
										{
										}

										virtual ~MyContactResultCallback()
										{
										}

										virtual bool needsCollision(btBroadphaseProxy* proxy0) const
										{
											//bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
											//collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
											//return collides;
											return true;
										}

										virtual	btScalar	addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
										{
											if (cp.m_distance1<=m_closestDistanceThreshold)
											{
												b3ContactPointData pt;
												pt.m_bodyUniqueIdA = m_bodyUniqueIdA;
												pt.m_bodyUniqueIdB = m_bodyUniqueIdB;
												const btManifoldPoint& srcPt = cp;
												pt.m_contactDistance = srcPt.getDistance();
												pt.m_contactFlags = 0;
												pt.m_linkIndexA = m_linkIndexA;
												pt.m_linkIndexB = m_linkIndexB;
												for (int j = 0; j < 3; j++)
												{
													pt.m_contactNormalOnBInWS[j] = srcPt.m_normalWorldOnB[j];
													pt.m_positionOnAInWS[j] = srcPt.getPositionWorldOnA()[j];
													pt.m_positionOnBInWS[j] = srcPt.getPositionWorldOnB()[j];
												}
												pt.m_normalForce = srcPt.getAppliedImpulse() / m_deltaTime;
												//                                    pt.m_linearFrictionForce = srcPt.m_appliedImpulseLateral1;
												m_cachedContactPoints.push_back(pt);
											}
											return 1;

										}
									};


									MyContactResultCallback cb(m_data->m_cachedContactPoints);

									cb.m_bodyUniqueIdA = bodyUniqueIdA;
									cb.m_bodyUniqueIdB = bodyUniqueIdB;
									cb.m_deltaTime = m_data->m_physicsDeltaTime;

									for (int i = 0; i < setA.size(); i++)
									{
										cb.m_linkIndexA = setALinkIndex[i];
										for (int j = 0; j < setB.size(); j++)
										{
											cb.m_linkIndexB = setBLinkIndex[j];
											cb.m_closestDistanceThreshold = closestDistanceThreshold;
											this->m_data->m_dynamicsWorld->contactPairTest(setA[i], setB[j], cb);
										}
									}
								}
									
									break;
								}
								default:
								{
									b3Warning("Unknown contact query mode: %d", mode);
								}

							}
						}
                        
						int numContactPoints = m_data->m_cachedContactPoints.size();
						

						//b3ContactPoint
						//struct b3ContactPointDynamics

						int totalBytesPerContact = sizeof(b3ContactPointData);
						int contactPointStorage = bufferSizeInBytes/totalBytesPerContact-1;

						b3ContactPointData* contactData = (b3ContactPointData*)bufferServerToClient;

						int startContactPointIndex = clientCmd.m_requestContactPointArguments.m_startingContactPointIndex;
						int numContactPointBatch = btMin(numContactPoints,contactPointStorage);

						int endContactPointIndex = startContactPointIndex+numContactPointBatch;

						for (int i=startContactPointIndex;i<endContactPointIndex ;i++)
						{
							const b3ContactPointData& srcPt = m_data->m_cachedContactPoints[i];
							b3ContactPointData& destPt = contactData[serverCmd.m_sendContactPointArgs.m_numContactPointsCopied];
							destPt = srcPt;
							serverCmd.m_sendContactPointArgs.m_numContactPointsCopied++;
						}
						
						serverCmd.m_sendContactPointArgs.m_startingContactPointIndex = clientCmd.m_requestContactPointArguments.m_startingContactPointIndex;
						serverCmd.m_sendContactPointArgs.m_numRemainingContactPoints = numContactPoints - clientCmd.m_requestContactPointArguments.m_startingContactPointIndex - serverCmd.m_sendContactPointArgs.m_numContactPointsCopied;
						serverCmd.m_numDataStreamBytes = totalBytesPerContact * serverCmd.m_sendContactPointArgs.m_numContactPointsCopied;
						serverCmd.m_type = CMD_CONTACT_POINT_INFORMATION_COMPLETED; //CMD_CONTACT_POINT_INFORMATION_FAILED,
						hasStatus = true;
                        break;
                    }
				case CMD_CALCULATE_INVERSE_DYNAMICS:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					InternalBodyHandle* bodyHandle = m_data->getHandle(clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId);
					if (bodyHandle && bodyHandle->m_multiBody)
					{
                        serverCmd.m_type = CMD_CALCULATED_INVERSE_DYNAMICS_FAILED;
                        
						btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);

						if (tree)
						{
							int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
							const int num_dofs = bodyHandle->m_multiBody->getNumDofs();
							btInverseDynamics::vecx nu(num_dofs+baseDofs), qdot(num_dofs + baseDofs), q(num_dofs + baseDofs), joint_force(num_dofs + baseDofs);
							for (int i = 0; i < num_dofs; i++)
							{
								q[i + baseDofs] = clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[i];
								qdot[i + baseDofs] = clientCmd.m_calculateInverseDynamicsArguments.m_jointVelocitiesQdot[i];
								nu[i+baseDofs] = clientCmd.m_calculateInverseDynamicsArguments.m_jointAccelerations[i];
							}
							// Set the gravity to correspond to the world gravity
							btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
                            
							if (-1 != tree->setGravityInWorldFrame(id_grav) &&
                                -1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
							{
								serverCmd.m_inverseDynamicsResultArgs.m_bodyUniqueId = clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId;
								serverCmd.m_inverseDynamicsResultArgs.m_dofCount = num_dofs;
								for (int i = 0; i < num_dofs; i++)
								{
									serverCmd.m_inverseDynamicsResultArgs.m_jointForces[i] = joint_force[i+baseDofs];
								}
								serverCmd.m_type = CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED;
							}
							else
							{
								serverCmd.m_type = CMD_CALCULATED_INVERSE_DYNAMICS_FAILED;
							}
						}

					}
					else
					{
						serverCmd.m_type = CMD_CALCULATED_INVERSE_DYNAMICS_FAILED;
					}

					hasStatus = true;
					break;
				}
                case CMD_CALCULATE_JACOBIAN:
                {
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    InternalBodyHandle* bodyHandle = m_data->getHandle(clientCmd.m_calculateJacobianArguments.m_bodyUniqueId);
                    if (bodyHandle && bodyHandle->m_multiBody)
                    {
                        serverCmd.m_type = CMD_CALCULATED_JACOBIAN_FAILED;
                        
                        btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);
                        
                        if (tree)
                        {
                            int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
                            const int num_dofs = bodyHandle->m_multiBody->getNumDofs();
                            btInverseDynamics::vecx nu(num_dofs+baseDofs), qdot(num_dofs + baseDofs), q(num_dofs + baseDofs), joint_force(num_dofs + baseDofs);
                            for (int i = 0; i < num_dofs; i++)
                            {
                                q[i + baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointPositionsQ[i];
                                qdot[i + baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointVelocitiesQdot[i];
                                nu[i+baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointAccelerations[i];
                            }
                            // Set the gravity to correspond to the world gravity
                            btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
                            
                            if (-1 != tree->setGravityInWorldFrame(id_grav) &&
                                -1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
                            {
                                serverCmd.m_jacobianResultArgs.m_dofCount = num_dofs;
                                // Set jacobian value
                                tree->calculateJacobians(q);
                                btInverseDynamics::mat3x jac_t(3, num_dofs);
                                tree->getBodyJacobianTrans(clientCmd.m_calculateJacobianArguments.m_linkIndex, &jac_t);
                                for (int i = 0; i < 3; ++i)
                                {
                                    for (int j = 0; j < num_dofs; ++j)
                                    {
                                        serverCmd.m_jacobianResultArgs.m_linearJacobian[i*num_dofs+j] = jac_t(i,j);
                                    }
                                }
                                serverCmd.m_type = CMD_CALCULATED_JACOBIAN_COMPLETED;
                            }
                            else
                            {
                                serverCmd.m_type = CMD_CALCULATED_JACOBIAN_FAILED;
                            }
                        }
                        
                    }
                    else
                    {
                        serverCmd.m_type = CMD_CALCULATED_JACOBIAN_FAILED;
                    }
                    
                    hasStatus = true;
                    break;
                }
                case CMD_APPLY_EXTERNAL_FORCE:
                {
                	if (m_data->m_verboseOutput)
                    {
                        b3Printf("CMD_APPLY_EXTERNAL_FORCE clientCmd = %d\n", clientCmd.m_sequenceNumber);
                    }
                    for (int i = 0; i < clientCmd.m_externalForceArguments.m_numForcesAndTorques; ++i)
                    {
                        InteralBodyData* body = m_data->getHandle(clientCmd.m_externalForceArguments.m_bodyUniqueIds[i]);
						bool isLinkFrame = ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_LINK_FRAME) != 0);

                        if (body && body->m_multiBody)
                        {
                            btMultiBody* mb = body->m_multiBody;
                         
                            if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_FORCE)!=0)
                            {
                                btVector3 forceLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+0],
                                                clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+1],
                                                clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+2]);
                                btVector3 positionLocal(
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+0],
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+1],
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+2]);

                                if (clientCmd.m_externalForceArguments.m_linkIds[i] == -1)
                                {
                                    btVector3 forceWorld = isLinkFrame ? forceLocal : mb->getBaseWorldTransform().getBasis()*forceLocal;
                                    btVector3 relPosWorld = isLinkFrame ? positionLocal : mb->getBaseWorldTransform().getBasis()*positionLocal;
                                    mb->addBaseForce(forceWorld);
                                    mb->addBaseTorque(relPosWorld.cross(forceWorld));
                                    //b3Printf("apply base force of %f,%f,%f at %f,%f,%f\n", forceWorld[0],forceWorld[1],forceWorld[2],positionLocal[0],positionLocal[1],positionLocal[2]);
                                } else
                                {
                                    int link = clientCmd.m_externalForceArguments.m_linkIds[i];
                                    btVector3 forceWorld = mb->getLink(link).m_cachedWorldTransform.getBasis()*forceLocal;
                                    btVector3 relPosWorld = mb->getLink(link).m_cachedWorldTransform.getBasis()*positionLocal;
                                    mb->addLinkForce(link, forceWorld);
                                    mb->addLinkTorque(link,relPosWorld.cross(forceWorld));
                                    //b3Printf("apply link force of %f,%f,%f at %f,%f,%f\n", forceWorld[0],forceWorld[1],forceWorld[2], positionLocal[0],positionLocal[1],positionLocal[2]);
                                }
                            }
                            if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_TORQUE)!=0)
                            {
                                btVector3 torqueLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+0],
                                                      clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+1],
                                                      clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+2]);

                                if (clientCmd.m_externalForceArguments.m_linkIds[i] == -1)
                                {
                                    btVector3 torqueWorld = isLinkFrame ? torqueLocal : mb->getBaseWorldTransform().getBasis()*torqueLocal;
                                    mb->addBaseTorque(torqueWorld);
                                    //b3Printf("apply base torque of %f,%f,%f\n", torqueWorld[0],torqueWorld[1],torqueWorld[2]);
                                } else
                                {
                                    int link = clientCmd.m_externalForceArguments.m_linkIds[i];
                                    btVector3 torqueWorld = mb->getLink(link).m_cachedWorldTransform.getBasis()*torqueLocal;
                                    mb->addLinkTorque(link, torqueWorld);
                                    //b3Printf("apply link torque of %f,%f,%f\n", torqueWorld[0],torqueWorld[1],torqueWorld[2]);
                                }
                            }
                        }

						if (body && body->m_rigidBody)
						{
							btRigidBody* rb = body->m_rigidBody;
							if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_FORCE) != 0)
							{
								btVector3 forceLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 0],
									clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 1],
									clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 2]);
								btVector3 positionLocal(
									clientCmd.m_externalForceArguments.m_positions[i * 3 + 0],
									clientCmd.m_externalForceArguments.m_positions[i * 3 + 1],
									clientCmd.m_externalForceArguments.m_positions[i * 3 + 2]);

								btVector3 forceWorld = isLinkFrame ? forceLocal : rb->getWorldTransform().getBasis()*forceLocal;
								btVector3 relPosWorld = isLinkFrame ? positionLocal : rb->getWorldTransform().getBasis()*positionLocal;
								rb->applyForce(forceWorld, relPosWorld);

							}

							if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_TORQUE) != 0)
							{
								btVector3 torqueLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 0],
									clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 1],
									clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 2]);

								btVector3 torqueWorld = isLinkFrame ? torqueLocal : rb->getWorldTransform().getBasis()*torqueLocal;
								rb->applyTorque(torqueWorld);
							}
						}
                    }

                    SharedMemoryStatus& serverCmd =serverStatusOut;
                    serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
                    hasStatus = true;
                    break;
                }
                case CMD_USER_CONSTRAINT:
                {
					SharedMemoryStatus& serverCmd =serverStatusOut;
                    serverCmd.m_type = CMD_USER_CONSTRAINT_FAILED;
                    hasStatus = true;
					if (clientCmd.m_updateFlags & USER_CONSTRAINT_REQUEST_INFO)
					{
						int userConstraintUidChange = clientCmd.m_userConstraintArguments.m_userConstraintUniqueId;
						InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.find(userConstraintUidChange);
						if (userConstraintPtr)
						{
							serverCmd.m_userConstraintResultArgs = userConstraintPtr->m_userConstraintData;
							serverCmd.m_type = CMD_USER_CONSTRAINT_INFO_COMPLETED;
						}
					}
					if (clientCmd.m_updateFlags & USER_CONSTRAINT_ADD_CONSTRAINT)
					{
						btScalar defaultMaxForce = 500.0;
						InteralBodyData* parentBody = m_data->getHandle(clientCmd.m_userConstraintArguments.m_parentBodyIndex);
						if (parentBody && parentBody->m_multiBody)
						{
							if ((clientCmd.m_userConstraintArguments.m_parentJointIndex>=-1) && clientCmd.m_userConstraintArguments.m_parentJointIndex < parentBody->m_multiBody->getNumLinks())
							{
								InteralBodyData* childBody = clientCmd.m_userConstraintArguments.m_childBodyIndex>=0 ? m_data->getHandle(clientCmd.m_userConstraintArguments.m_childBodyIndex):0;
								//also create a constraint with just a single multibody/rigid body without child
								//if (childBody)
								{
									btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
									btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);
									btMatrix3x3 frameInParent(btQuaternion(clientCmd.m_userConstraintArguments.m_parentFrame[3], clientCmd.m_userConstraintArguments.m_parentFrame[4], clientCmd.m_userConstraintArguments.m_parentFrame[5], clientCmd.m_userConstraintArguments.m_parentFrame[6]));
									btMatrix3x3 frameInChild(btQuaternion(clientCmd.m_userConstraintArguments.m_childFrame[3], clientCmd.m_userConstraintArguments.m_childFrame[4], clientCmd.m_userConstraintArguments.m_childFrame[5], clientCmd.m_userConstraintArguments.m_childFrame[6]));
									btVector3 jointAxis(clientCmd.m_userConstraintArguments.m_jointAxis[0], clientCmd.m_userConstraintArguments.m_jointAxis[1], clientCmd.m_userConstraintArguments.m_jointAxis[2]);
									if (clientCmd.m_userConstraintArguments.m_jointType == eFixedType)
									{
										if (childBody && childBody->m_multiBody)
										{
											if ((clientCmd.m_userConstraintArguments.m_childJointIndex>=-1) && (clientCmd.m_userConstraintArguments.m_childJointIndex <childBody->m_multiBody->getNumLinks()))
											{
												btMultiBodyFixedConstraint* multibodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_userConstraintArguments.m_childJointIndex,pivotInParent,pivotInChild,frameInParent,frameInChild);
												multibodyFixed->setMaxAppliedImpulse(defaultMaxForce);
												m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodyFixed);
												InteralUserConstraintData userConstraintData;
												userConstraintData.m_mbConstraint = multibodyFixed;
												int uid = m_data->m_userConstraintUIDGenerator++;
												serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
												serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
												userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
												m_data->m_userConstraints.insert(uid,userConstraintData);
												serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
											}
									
										}
										else
										{
											btRigidBody* rb = childBody? childBody->m_rigidBody : 0;
											btMultiBodyFixedConstraint* rigidbodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,rb,pivotInParent,pivotInChild,frameInParent,frameInChild);
											rigidbodyFixed->setMaxAppliedImpulse(defaultMaxForce);
											btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
											world->addMultiBodyConstraint(rigidbodyFixed);
											InteralUserConstraintData userConstraintData;
											userConstraintData.m_mbConstraint = rigidbodyFixed;
											int uid = m_data->m_userConstraintUIDGenerator++;
											serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
											serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
											serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
											userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
											m_data->m_userConstraints.insert(uid,userConstraintData);
											serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
										}
								
									}
									else if (clientCmd.m_userConstraintArguments.m_jointType == ePrismaticType)
									{
										if (childBody &&  childBody->m_multiBody)
										{
											btMultiBodySliderConstraint* multibodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_userConstraintArguments.m_childJointIndex,pivotInParent,pivotInChild,frameInParent,frameInChild,jointAxis);
											multibodySlider->setMaxAppliedImpulse(defaultMaxForce);
											m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodySlider);
											InteralUserConstraintData userConstraintData;
											userConstraintData.m_mbConstraint = multibodySlider;
											int uid = m_data->m_userConstraintUIDGenerator++;
											serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
											serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
											serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
											userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
											m_data->m_userConstraints.insert(uid,userConstraintData);
											serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
										}
										else
										{
											btRigidBody* rb = childBody? childBody->m_rigidBody : 0;
									
											btMultiBodySliderConstraint* rigidbodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,rb,pivotInParent,pivotInChild,frameInParent,frameInChild,jointAxis);
											rigidbodySlider->setMaxAppliedImpulse(defaultMaxForce);
											btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
											world->addMultiBodyConstraint(rigidbodySlider);
											InteralUserConstraintData userConstraintData;
											userConstraintData.m_mbConstraint = rigidbodySlider;
											int uid = m_data->m_userConstraintUIDGenerator++;
											serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
											serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
											serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
											userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
											m_data->m_userConstraints.insert(uid,userConstraintData);
											serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;										}
								
									} else if (clientCmd.m_userConstraintArguments.m_jointType == ePoint2PointType)
									{
										if (childBody && childBody->m_multiBody)
										{
											btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_userConstraintArguments.m_childJointIndex,pivotInParent,pivotInChild);
											p2p->setMaxAppliedImpulse(defaultMaxForce);
											m_data->m_dynamicsWorld->addMultiBodyConstraint(p2p);
											InteralUserConstraintData userConstraintData;
											userConstraintData.m_mbConstraint = p2p;
											int uid = m_data->m_userConstraintUIDGenerator++;
											serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
											serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
											serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
											userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
											m_data->m_userConstraints.insert(uid,userConstraintData);
											serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
										}
										else
										{
											btRigidBody* rb = childBody? childBody->m_rigidBody : 0;
									
											btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,rb,pivotInParent,pivotInChild);
											p2p->setMaxAppliedImpulse(defaultMaxForce);
											btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
											world->addMultiBodyConstraint(p2p);
											InteralUserConstraintData userConstraintData;
											userConstraintData.m_mbConstraint = p2p;
											int uid = m_data->m_userConstraintUIDGenerator++;
											serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
											serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
											serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
											userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
											m_data->m_userConstraints.insert(uid,userConstraintData);
											serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
										}
								
									} else
									{
										b3Warning("unknown constraint type");
									}

							
								}
							}
							}
					}

					if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_CONSTRAINT)
					{
						serverCmd.m_type = CMD_CHANGE_USER_CONSTRAINT_FAILED;
						int userConstraintUidChange = clientCmd.m_userConstraintArguments.m_userConstraintUniqueId;
						InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.find(userConstraintUidChange);
						if (userConstraintPtr)
						{
							if (userConstraintPtr->m_mbConstraint)
							{
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_PIVOT_IN_B)
								{
									btVector3 pivotInB(clientCmd.m_userConstraintArguments.m_childFrame[0],
										clientCmd.m_userConstraintArguments.m_childFrame[1],
										clientCmd.m_userConstraintArguments.m_childFrame[2]);
									userConstraintPtr->m_userConstraintData.m_childFrame[0] = clientCmd.m_userConstraintArguments.m_childFrame[0];
									userConstraintPtr->m_userConstraintData.m_childFrame[1] = clientCmd.m_userConstraintArguments.m_childFrame[1];
									userConstraintPtr->m_userConstraintData.m_childFrame[2] = clientCmd.m_userConstraintArguments.m_childFrame[2];
									userConstraintPtr->m_mbConstraint->setPivotInB(pivotInB);
								}
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B)
								{
									btQuaternion childFrameOrn(clientCmd.m_userConstraintArguments.m_childFrame[3],
										clientCmd.m_userConstraintArguments.m_childFrame[4],
										clientCmd.m_userConstraintArguments.m_childFrame[5],
										clientCmd.m_userConstraintArguments.m_childFrame[6]);
									userConstraintPtr->m_userConstraintData.m_childFrame[3] = clientCmd.m_userConstraintArguments.m_childFrame[3];
									userConstraintPtr->m_userConstraintData.m_childFrame[4] = clientCmd.m_userConstraintArguments.m_childFrame[4];
									userConstraintPtr->m_userConstraintData.m_childFrame[5] = clientCmd.m_userConstraintArguments.m_childFrame[5];
									userConstraintPtr->m_userConstraintData.m_childFrame[6] = clientCmd.m_userConstraintArguments.m_childFrame[6];
									btMatrix3x3 childFrameBasis(childFrameOrn);
									userConstraintPtr->m_mbConstraint->setFrameInB(childFrameBasis);
								}
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
								{
									btScalar maxImp = clientCmd.m_userConstraintArguments.m_maxAppliedForce*m_data->m_physicsDeltaTime;
									userConstraintPtr->m_userConstraintData.m_maxAppliedForce = clientCmd.m_userConstraintArguments.m_maxAppliedForce;
									userConstraintPtr->m_mbConstraint->setMaxAppliedImpulse(maxImp);
								}
							}
							if (userConstraintPtr->m_rbConstraint)
							{
								//todo
							}
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = userConstraintUidChange;
							serverCmd.m_updateFlags = clientCmd.m_updateFlags;
							serverCmd.m_type = CMD_CHANGE_USER_CONSTRAINT_COMPLETED;
						}
					}
					if (clientCmd.m_updateFlags & USER_CONSTRAINT_REMOVE_CONSTRAINT)
					{
						serverCmd.m_type = CMD_REMOVE_USER_CONSTRAINT_FAILED;
						int userConstraintUidRemove = clientCmd.m_userConstraintArguments.m_userConstraintUniqueId;
						InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.find(userConstraintUidRemove);
						if (userConstraintPtr)
						{
							if (userConstraintPtr->m_mbConstraint)
							{
								m_data->m_dynamicsWorld->removeMultiBodyConstraint(userConstraintPtr->m_mbConstraint);
								delete userConstraintPtr->m_mbConstraint;
								m_data->m_userConstraints.remove(userConstraintUidRemove);
							}
							if (userConstraintPtr->m_rbConstraint)
							{

							}
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = userConstraintUidRemove;
							serverCmd.m_type = CMD_REMOVE_USER_CONSTRAINT_COMPLETED;

                            
						}

						
					}
					
                    break;
                }
				case CMD_CALCULATE_INVERSE_KINEMATICS:
					{
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_CALCULATE_INVERSE_KINEMATICS_FAILED;

						InternalBodyHandle* bodyHandle = m_data->getHandle(clientCmd.m_calculateInverseKinematicsArguments.m_bodyUniqueId);
						if (bodyHandle && bodyHandle->m_multiBody)
						{
							IKTrajectoryHelper** ikHelperPtrPtr = m_data->m_inverseKinematicsHelpers.find(bodyHandle->m_multiBody);
							IKTrajectoryHelper* ikHelperPtr = 0;
							

							if (ikHelperPtrPtr)
							{
								ikHelperPtr = *ikHelperPtrPtr;
							}
							else
							{
								IKTrajectoryHelper* tmpHelper = new IKTrajectoryHelper;
								m_data->m_inverseKinematicsHelpers.insert(bodyHandle->m_multiBody, tmpHelper);
								ikHelperPtr = tmpHelper;
							}

                            int endEffectorLinkIndex = clientCmd.m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex;
                            
							
							if (ikHelperPtr && (endEffectorLinkIndex<bodyHandle->m_multiBody->getNumLinks()))
							{
								const int numDofs = bodyHandle->m_multiBody->getNumDofs();

                                b3AlignedObjectArray<double> jacobian_linear;
                                jacobian_linear.resize(3*numDofs);
                                b3AlignedObjectArray<double> jacobian_angular;
                                jacobian_angular.resize(3*numDofs);
                                int jacSize = 0;
                                
                                btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);
                      
							

                                btAlignedObjectArray<double> q_current;
								q_current.resize(numDofs);
                                
                                if (tree)
                                {
                                    jacSize = jacobian_linear.size();
                                    // Set jacobian value
                                    int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
                                    
                                    
                                    btInverseDynamics::vecx nu(numDofs+baseDofs), qdot(numDofs + baseDofs), q(numDofs + baseDofs), joint_force(numDofs + baseDofs);
                                    for (int i = 0; i < numDofs; i++)
                                    {
                                        q_current[i] = bodyHandle->m_multiBody->getJointPos(i);
                                        q[i+baseDofs] = bodyHandle->m_multiBody->getJointPos(i);
                                        qdot[i + baseDofs] = 0;
                                        nu[i+baseDofs] = 0;
                                    }
                                    // Set the gravity to correspond to the world gravity
                                    btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
                                    
                                    if (-1 != tree->setGravityInWorldFrame(id_grav) &&
                                        -1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
                                    {
                                        tree->calculateJacobians(q);
                                        btInverseDynamics::mat3x jac_t(3, numDofs);
                                        btInverseDynamics::mat3x jac_r(3,numDofs);
                                        tree->getBodyJacobianTrans(endEffectorLinkIndex, &jac_t);
                                        tree->getBodyJacobianRot(endEffectorLinkIndex, &jac_r);
                                        for (int i = 0; i < 3; ++i)
                                        {
                                            for (int j = 0; j < numDofs; ++j)
                                            {
                                                jacobian_linear[i*numDofs+j] = jac_t(i,j);
                                                jacobian_angular[i*numDofs+j] = jac_r(i,j);
                                            }
                                        }
                                    }
                                }
                                
                                
                                btAlignedObjectArray<double> q_new;
								q_new.resize(numDofs);
                                int ikMethod = 0;
                                if ((clientCmd.m_updateFlags& IK_HAS_TARGET_ORIENTATION)&&(clientCmd.m_updateFlags&IK_HAS_NULL_SPACE_VELOCITY))
                                {
                                    ikMethod = IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE;
                                }
                                else if (clientCmd.m_updateFlags& IK_HAS_TARGET_ORIENTATION)
                                {
                                    ikMethod = IK2_VEL_DLS_WITH_ORIENTATION;
                                }
                                else if (clientCmd.m_updateFlags& IK_HAS_NULL_SPACE_VELOCITY)
                                {
                                    ikMethod = IK2_VEL_DLS_WITH_NULLSPACE;
                                }
                                else
                                {
                                    ikMethod = IK2_VEL_DLS;
                                }
                                
                                if (clientCmd.m_updateFlags& IK_HAS_NULL_SPACE_VELOCITY)
                                {
                                    btAlignedObjectArray<double> lower_limit;
                                    btAlignedObjectArray<double> upper_limit;
                                    btAlignedObjectArray<double> joint_range;
                                    btAlignedObjectArray<double> rest_pose;
                                    lower_limit.resize(numDofs);
                                    upper_limit.resize(numDofs);
                                    joint_range.resize(numDofs);
                                    rest_pose.resize(numDofs);
                                    for (int i = 0; i < numDofs; ++i)
                                    {
                                        lower_limit[i] = clientCmd.m_calculateInverseKinematicsArguments.m_lowerLimit[i];
                                        upper_limit[i] = clientCmd.m_calculateInverseKinematicsArguments.m_upperLimit[i];
                                        joint_range[i] = clientCmd.m_calculateInverseKinematicsArguments.m_jointRange[i];
                                        rest_pose[i] = clientCmd.m_calculateInverseKinematicsArguments.m_restPose[i];
                                    }
									ikHelperPtr->computeNullspaceVel(numDofs, &q_current[0], &lower_limit[0], &upper_limit[0], &joint_range[0], &rest_pose[0]);
                                }
                                
                                btTransform endEffectorTransformWorld = bodyHandle->m_multiBody->getLink(endEffectorLinkIndex).m_cachedWorldTransform * bodyHandle->m_linkLocalInertialFrames[endEffectorLinkIndex].inverse();
                               
                                btVector3DoubleData endEffectorWorldPosition;
                                btVector3DoubleData endEffectorWorldOrientation;
                                
                                btVector3 endEffectorPosWorld =  endEffectorTransformWorld.getOrigin();
                                btQuaternion endEffectorOriWorld = endEffectorTransformWorld.getRotation();
                                btVector4 endEffectorOri(endEffectorOriWorld.x(),endEffectorOriWorld.y(),endEffectorOriWorld.z(),endEffectorOriWorld.w());
                                
                                endEffectorPosWorld.serializeDouble(endEffectorWorldPosition);
                                endEffectorOri.serializeDouble(endEffectorWorldOrientation);
                                
                                // Set joint damping coefficents. A small default
                                // damping constant is added to prevent singularity
                                // with pseudo inverse. The user can set joint damping
                                // coefficients differently for each joint. The larger
                                // the damping coefficient is, the less we rely on
                                // this joint to achieve the IK target.
                                btAlignedObjectArray<double> joint_damping;
                                joint_damping.resize(numDofs,0.5);
                                if (clientCmd.m_updateFlags& IK_HAS_JOINT_DAMPING)
                                {
                                    for (int i = 0; i < numDofs; ++i)
                                    {
                                        joint_damping[i] = clientCmd.m_calculateInverseKinematicsArguments.m_jointDamping[i];
                                    }
                                }
                                ikHelperPtr->setDampingCoeff(numDofs, &joint_damping[0]);
                                
								double targetDampCoeff[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
                                ikHelperPtr->computeIK(clientCmd.m_calculateInverseKinematicsArguments.m_targetPosition, clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation,
                                                       endEffectorWorldPosition.m_floats, endEffectorWorldOrientation.m_floats,
                                                       &q_current[0],
                                                       numDofs, clientCmd.m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex,
									&q_new[0], ikMethod, &jacobian_linear[0], &jacobian_angular[0], jacSize*2, targetDampCoeff);
                                
                                serverCmd.m_inverseKinematicsResultArgs.m_bodyUniqueId =clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId;
                                for (int i=0;i<numDofs;i++)
                                {
                                    serverCmd.m_inverseKinematicsResultArgs.m_jointPositions[i] = q_new[i];
                                }
                                serverCmd.m_inverseKinematicsResultArgs.m_dofCount = numDofs;
                                serverCmd.m_type = CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED;
							}
						}
						hasStatus = true;
						break;
					}
                case CMD_REQUEST_VISUAL_SHAPE_INFO:
                {
                    
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type = CMD_VISUAL_SHAPE_INFO_FAILED;
                    //retrieve the visual shape information for a specific body
                    
					int totalNumVisualShapes = m_data->m_visualConverter.getNumVisualShapes(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId);
					//int totalBytesPerVisualShape = sizeof (b3VisualShapeData);
					//int visualShapeStorage = bufferSizeInBytes / totalBytesPerVisualShape - 1;
					b3VisualShapeData* visualShapeStoragePtr = (b3VisualShapeData*)bufferServerToClient;

					int remain = totalNumVisualShapes - clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
					int shapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;

					m_data->m_visualConverter.getVisualShapesData(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId,
						shapeIndex,
						visualShapeStoragePtr);


                    //m_visualConverter
					serverCmd.m_sendVisualShapeArgs.m_numRemainingVisualShapes = remain-1;
					serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied = 1;
					serverCmd.m_sendVisualShapeArgs.m_startingVisualShapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
					serverCmd.m_sendVisualShapeArgs.m_bodyUniqueId = clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId;
					serverCmd.m_numDataStreamBytes = sizeof(b3VisualShapeData)*serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied;
                    serverCmd.m_type =CMD_VISUAL_SHAPE_INFO_COMPLETED;
                    hasStatus = true;
                    break;
                }
                case CMD_UPDATE_VISUAL_SHAPE:
                {
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_FAILED;
                    
                    m_data->m_visualConverter.activateShapeTexture(clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId, clientCmd.m_updateVisualShapeDataArguments.m_jointIndex, clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex, clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId);
                    
                    serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_COMPLETED;
                    hasStatus = true;

                    break;
                }
                case CMD_LOAD_TEXTURE:
                {
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type = CMD_LOAD_TEXTURE_FAILED;
                    
                    int uid = m_data->m_visualConverter.loadTextureFile(clientCmd.m_loadTextureArguments.m_textureFileName);
                    
                    if (uid>=0)
                    {
                        serverCmd.m_type = CMD_LOAD_TEXTURE_COMPLETED;
                    } else
                    {
                        serverCmd.m_type = CMD_LOAD_TEXTURE_FAILED;
                    }
                    hasStatus = true;
                    
                    break;
                }

				case CMD_LOAD_BULLET:
				{
					
					SharedMemoryStatus& serverCmd = serverStatusOut;
					btBulletWorldImporter* importer = new btBulletWorldImporter(m_data->m_dynamicsWorld);

					const char* prefix[] = { "", "./", "./data/", "../data/", "../../data/", "../../../data/", "../../../../data/" };
					int numPrefixes = sizeof(prefix) / sizeof(const char*);
					char relativeFileName[1024];
					FILE* f = 0;
					bool found = false;

					for (int i = 0; !f && i<numPrefixes; i++)
					{
						sprintf(relativeFileName, "%s%s", prefix[i], clientCmd.m_fileArguments.m_fileName);
						f = fopen(relativeFileName, "rb");
						if (f)
						{
							found = true;
							break;
						}
					}
					if (f)
					{
						fclose(f);
					}

					if (found)
					{
						bool ok = importer->loadFile(relativeFileName);
						if (ok)
						{
							
							
							int numRb = importer->getNumRigidBodies();
							serverStatusOut.m_sdfLoadedArgs.m_numBodies = 0;
							serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
                            
							for( int i=0;i<numRb;i++)
							{
								btCollisionObject* colObj = importer->getRigidBodyByIndex(i);
								if (colObj)
								{
									btRigidBody* rb = btRigidBody::upcast(colObj);
									if (rb)
									{
										int bodyUniqueId = m_data->allocHandle();
										InternalBodyHandle* bodyHandle = m_data->getHandle(bodyUniqueId);
										colObj->setUserIndex2(bodyUniqueId);
										bodyHandle->m_rigidBody = rb;

										if (serverStatusOut.m_sdfLoadedArgs.m_numBodies<MAX_SDF_BODIES)
										{
											serverStatusOut.m_sdfLoadedArgs.m_numBodies++;
											serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = bodyUniqueId;
										}
									}
								}
							}

							serverCmd.m_type = CMD_BULLET_LOADING_COMPLETED;
							m_data->m_guiHelper->autogenerateGraphicsObjects(m_data->m_dynamicsWorld);
							hasStatus = true;
							break;
						}
					}
					serverCmd.m_type = CMD_BULLET_LOADING_FAILED;
					hasStatus = true;
					break;
				}

				case CMD_SAVE_BULLET:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					
					FILE* f = fopen(clientCmd.m_fileArguments.m_fileName, "wb");
					if (f)
					{
						btDefaultSerializer* ser = new btDefaultSerializer();
						m_data->m_dynamicsWorld->serialize(ser);
						fwrite(ser->getBufferPointer(), ser->getCurrentBufferSize(), 1, f);
						fclose(f);
						serverCmd.m_type = CMD_BULLET_SAVING_COMPLETED;
						delete ser;
					}
					serverCmd.m_type = CMD_BULLET_SAVING_FAILED;
					hasStatus = true;
					break;
				}

				case CMD_LOAD_MJCF:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverCmd.m_type = CMD_MJCF_LOADING_FAILED;
					  const MjcfArgs& mjcfArgs = clientCmd.m_mjcfArguments;
                        if (m_data->m_verboseOutput)
                        {
                            b3Printf("Processed CMD_LOAD_MJCF:%s", mjcfArgs.m_mjcfFileName);
                        }
                        bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (mjcfArgs.m_useMultiBody!=0) : true;
						int flags = CUF_USE_MJCF;//CUF_USE_URDF_INERTIA
                        bool completedOk = loadMjcf(mjcfArgs.m_mjcfFileName,bufferServerToClient, bufferSizeInBytes, useMultiBody, flags);
                        if (completedOk)
                        {
							m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

                            serverStatusOut.m_sdfLoadedArgs.m_numBodies = m_data->m_sdfRecentLoadedBodies.size();
							serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
                            int maxBodies = btMin(MAX_SDF_BODIES, serverStatusOut.m_sdfLoadedArgs.m_numBodies);
                            for (int i=0;i<maxBodies;i++)
                            {
                                serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = m_data->m_sdfRecentLoadedBodies[i];
                            }

                            serverStatusOut.m_type = CMD_MJCF_LOADING_COMPLETED;
                        } else
                        {
                            serverStatusOut.m_type = CMD_MJCF_LOADING_FAILED;
                        }
						hasStatus = true;
                        break;

				}
				
				case CMD_USER_DEBUG_DRAW:
					{
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_USER_DEBUG_DRAW_FAILED;
						hasStatus = true;
						if (clientCmd.m_updateFlags & USER_DEBUG_ADD_PARAMETER)
						{
							int uid = m_data->m_guiHelper->addUserDebugParameter(
								clientCmd.m_userDebugDrawArgs.m_text,
								clientCmd.m_userDebugDrawArgs.m_rangeMin,
								clientCmd.m_userDebugDrawArgs.m_rangeMax,
								clientCmd.m_userDebugDrawArgs.m_startValue
							);
							serverCmd.m_userDebugDrawArgs.m_debugItemUniqueId = uid;
							serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
						}
						if (clientCmd.m_updateFlags &USER_DEBUG_READ_PARAMETER)
						{
						
							int ok = m_data->m_guiHelper->readUserDebugParameter(
								clientCmd.m_userDebugDrawArgs.m_itemUniqueId,
								&serverCmd.m_userDebugDrawArgs.m_parameterValue);
							if (ok)
							{
								serverCmd.m_type = CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED;
							} 
						}
						if ((clientCmd.m_updateFlags & USER_DEBUG_SET_CUSTOM_OBJECT_COLOR) || (clientCmd.m_updateFlags & USER_DEBUG_REMOVE_CUSTOM_OBJECT_COLOR))
						{
							int bodyUniqueId = clientCmd.m_userDebugDrawArgs.m_objectUniqueId;
							InteralBodyData* body = m_data->getHandle(bodyUniqueId);
							if (body)
							{
								btCollisionObject* destColObj = 0;

								if (body->m_multiBody)
								{
									if (clientCmd.m_userDebugDrawArgs.m_linkIndex == -1)
									{
										destColObj = body->m_multiBody->getBaseCollider();
									}
									else
									{
										if (clientCmd.m_userDebugDrawArgs.m_linkIndex >= 0 && clientCmd.m_userDebugDrawArgs.m_linkIndex < body->m_multiBody->getNumLinks())
										{
											destColObj = body->m_multiBody->getLink(clientCmd.m_userDebugDrawArgs.m_linkIndex).m_collider;
										}
									}

								}
								if (body->m_rigidBody)
								{
									destColObj = body->m_rigidBody;
								}

								if (destColObj)
								{
									if (clientCmd.m_updateFlags & USER_DEBUG_REMOVE_CUSTOM_OBJECT_COLOR)
									{
										destColObj->removeCustomDebugColor();
										serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
									}
									if (clientCmd.m_updateFlags & USER_DEBUG_SET_CUSTOM_OBJECT_COLOR)
									{
										btVector3 objectColorRGB;
										objectColorRGB.setValue(clientCmd.m_userDebugDrawArgs.m_objectDebugColorRGB[0],
											clientCmd.m_userDebugDrawArgs.m_objectDebugColorRGB[1],
											clientCmd.m_userDebugDrawArgs.m_objectDebugColorRGB[2]);
										destColObj->setCustomDebugColor(objectColorRGB);
										serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
									}
								}
							}
						}

						if (clientCmd.m_updateFlags & USER_DEBUG_HAS_TEXT)
						{
							int uid = m_data->m_guiHelper->addUserDebugText3D(clientCmd.m_userDebugDrawArgs.m_text,
								clientCmd.m_userDebugDrawArgs.m_textPositionXYZ,
								clientCmd.m_userDebugDrawArgs.m_textColorRGB,
								clientCmd.m_userDebugDrawArgs.m_textSize,
								clientCmd.m_userDebugDrawArgs.m_lifeTime);

							if (uid>=0)
							{
								serverCmd.m_userDebugDrawArgs.m_debugItemUniqueId = uid;
								serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
							}
						}

						if (clientCmd.m_updateFlags & USER_DEBUG_HAS_LINE)
						{
							int uid = m_data->m_guiHelper->addUserDebugLine(
								clientCmd.m_userDebugDrawArgs.m_debugLineFromXYZ,
								clientCmd.m_userDebugDrawArgs.m_debugLineToXYZ,
								clientCmd.m_userDebugDrawArgs.m_debugLineColorRGB,
								clientCmd.m_userDebugDrawArgs.m_lineWidth,
								clientCmd.m_userDebugDrawArgs.m_lifeTime);

							if (uid>=0)
							{
								serverCmd.m_userDebugDrawArgs.m_debugItemUniqueId = uid;
								serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
							}
						}
						

						if (clientCmd.m_updateFlags & USER_DEBUG_REMOVE_ALL)
						{
							m_data->m_guiHelper->removeAllUserDebugItems();
							serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;

						}
						if (clientCmd.m_updateFlags & USER_DEBUG_REMOVE_ONE_ITEM)
						{
							m_data->m_guiHelper->removeUserDebugItem(clientCmd.m_userDebugDrawArgs.m_itemUniqueId);
							serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;

						}

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

//static int skip=1;

void PhysicsServerCommandProcessor::renderScene()
{
	if (m_data->m_guiHelper)
	{
		m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
		m_data->m_guiHelper->render(m_data->m_dynamicsWorld);
	}
#ifdef USE_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
    for (  int i=0;i<m_data->m_dynamicsWorld->getSoftBodyArray().size();i++)
    {
        btSoftBody*	psb=(btSoftBody*)m_data->m_dynamicsWorld->getSoftBodyArray()[i];
        if (m_data->m_dynamicsWorld->getDebugDrawer() && !(m_data->m_dynamicsWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
        {
            //btSoftBodyHelpers::DrawFrame(psb,m_data->m_dynamicsWorld->getDebugDrawer());
            btSoftBodyHelpers::Draw(psb,m_data->m_dynamicsWorld->getDebugDrawer(),m_data->m_dynamicsWorld->getDrawFlags());
        }
    }
#endif
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
		gLastPickPos = pickPos;
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
		m_data->m_pickedBody->forceActivationState(ACTIVE_TAG);
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


btVector3 gVRGripperPos(0.7, 0.3, 0.7);
btQuaternion gVRGripperOrn(0,0,0,1);
btVector3 gVRController2Pos(0,0,0.2);
btQuaternion gVRController2Orn(0,0,0,1);
btScalar gVRGripper2Analog = 0;
btScalar gVRGripperAnalog = 0;

bool gVRGripperClosed = false;


int gDroppedSimulationSteps = 0;
int gNumSteps = 0;
double gDtInSec = 0.f;
double gSubStep = 0.f;

void PhysicsServerCommandProcessor::enableRealTimeSimulation(bool enableRealTimeSim)
{
	m_data->m_allowRealTimeSimulation = enableRealTimeSim;
}

void PhysicsServerCommandProcessor::stepSimulationRealTime(double dtInSec,	const struct b3VRControllerEvent* vrEvents, int numVREvents)
{
	//update m_vrEvents
	for (int i=0;i<numVREvents;i++)
	{
		int controlledId = vrEvents[i].m_controllerId;
		if (vrEvents[i].m_numMoveEvents)
		{
			m_data->m_vrEvents[controlledId].m_analogAxis = vrEvents[i].m_analogAxis;
		}

		if (vrEvents[i].m_numMoveEvents+vrEvents[i].m_numButtonEvents)
		{
			m_data->m_vrEvents[controlledId].m_controllerId = vrEvents[i].m_controllerId;

			m_data->m_vrEvents[controlledId].m_pos[0] = vrEvents[i].m_pos[0];
			m_data->m_vrEvents[controlledId].m_pos[1] = vrEvents[i].m_pos[1];
			m_data->m_vrEvents[controlledId].m_pos[2] = vrEvents[i].m_pos[2];
			
			m_data->m_vrEvents[controlledId].m_orn[0] = vrEvents[i].m_orn[0];
			m_data->m_vrEvents[controlledId].m_orn[1] = vrEvents[i].m_orn[1];
			m_data->m_vrEvents[controlledId].m_orn[2] = vrEvents[i].m_orn[2];
			m_data->m_vrEvents[controlledId].m_orn[3] = vrEvents[i].m_orn[3];
		}

		m_data->m_vrEvents[controlledId].m_numButtonEvents += vrEvents[i].m_numButtonEvents;
		m_data->m_vrEvents[controlledId].m_numMoveEvents += vrEvents[i].m_numMoveEvents;
		for (int b=0;b<MAX_VR_BUTTONS;b++)
		{
			m_data->m_vrEvents[controlledId].m_buttons[b] |= vrEvents[i].m_buttons[b];
			if (vrEvents[i].m_buttons[b] & eButtonIsDown)
			{
				m_data->m_vrEvents[controlledId].m_buttons[b] |= eButtonIsDown;
			} else
			{
				m_data->m_vrEvents[controlledId].m_buttons[b] &= ~eButtonIsDown;
			}
		}
	}

	if (gResetSimulation)
	{
		resetSimulation();
		gResetSimulation = false;
	}

	if ((m_data->m_allowRealTimeSimulation) && m_data->m_guiHelper)
	{
		
		///this hardcoded C++ scene creation is temporary for demo purposes. It will be done in Python later...
		if (gCreateDefaultRobotAssets)
		{
			createDefaultRobotAssets();
		}

		int maxSteps = m_data->m_numSimulationSubSteps+3;
		if (m_data->m_numSimulationSubSteps)
		{
			gSubStep = m_data->m_physicsDeltaTime / m_data->m_numSimulationSubSteps;
		}
		else
		{
			gSubStep = m_data->m_physicsDeltaTime;
		}
		
		if (gVRTrackingObjectUniqueId >= 0)
		{
			InternalBodyHandle* bodyHandle = m_data->getHandle(gVRTrackingObjectUniqueId);
			if (bodyHandle && bodyHandle->m_multiBody)
			{
				gVRTrackingObjectTr  = bodyHandle->m_multiBody->getBaseWorldTransform();
			}
		}


		int numSteps = m_data->m_dynamicsWorld->stepSimulation(dtInSec*simTimeScalingFactor,maxSteps, gSubStep);
		gDroppedSimulationSteps += numSteps > maxSteps ? numSteps - maxSteps : 0;

		if (numSteps)
		{
			gNumSteps = numSteps;
			gDtInSec = dtInSec;
		}
	}
}

void PhysicsServerCommandProcessor::applyJointDamping(int bodyUniqueId)
{
    InteralBodyData* body = m_data->getHandle(bodyUniqueId);
    if (body) {
        btMultiBody* mb = body->m_multiBody;
        if (mb) {
            for (int l=0;l<mb->getNumLinks();l++) {
                for (int d=0;d<mb->getLink(l).m_dofCount;d++) {
                    double damping_coefficient = mb->getLink(l).m_jointDamping;
                    double damping = -damping_coefficient*mb->getJointVelMultiDof(l)[d];
                    mb->addJointTorqueMultiDof(l, d, damping);
                }
            }
        }
    }
}

void PhysicsServerCommandProcessor::resetSimulation()
{
	//clean up all data

	if (m_data && m_data->m_guiHelper)
	{
		m_data->m_guiHelper->removeAllGraphicsInstances();
	}
	if (m_data)
	{
		m_data->m_visualConverter.resetAll();
	}

	removePickingConstraint();

	deleteDynamicsWorld();
	createEmptyDynamicsWorld();

	m_data->exitHandles();
	m_data->initHandles();

	m_data->m_hasGround = false;
	m_data->m_gripperRigidbodyFixed = 0;

}

//todo: move this to Python/scripting (it is almost ready to be removed!)
void PhysicsServerCommandProcessor::createDefaultRobotAssets()
{
	static btAlignedObjectArray<char> gBufferServerToClient;
	gBufferServerToClient.resize(SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
	int bodyId = 0;

	if (gCreateObjectSimVR >= 0)
	{
		gCreateObjectSimVR = -1;
		btMatrix3x3 mat(gVRGripperOrn);
		btScalar spawnDistance = 0.1;
		btVector3 spawnDir = mat.getColumn(0);
		btVector3 shiftPos = spawnDir*spawnDistance;
		btVector3 spawnPos = gVRGripperPos + shiftPos;
		loadUrdf("sphere_small.urdf", spawnPos, gVRGripperOrn, true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("lego/lego.urdf", spawnPos, gVRGripperOrn, true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		m_data->m_sphereId = bodyId;
		InteralBodyData* parentBody = m_data->getHandle(bodyId);
		if (parentBody->m_multiBody)
		{
			parentBody->m_multiBody->setBaseVel(spawnDir * 5);
		}
	}

	if (!m_data->m_hasGround)
	{
		m_data->m_hasGround = true;

		loadUrdf("plane.urdf", btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1), true, true, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("samurai.urdf", btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
//		loadUrdf("quadruped/quadruped.urdf", btVector3(2, 2, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

		if (m_data->m_gripperRigidbodyFixed == 0)
		{
			int bodyId = 0;

			if (loadUrdf("pr2_gripper.urdf", btVector3(-0.2, 0.15, 0.9), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size()))
			{
				InteralBodyData* parentBody = m_data->getHandle(bodyId);
				if (parentBody->m_multiBody)
				{
					parentBody->m_multiBody->setHasSelfCollision(0);
					btVector3 pivotInParent(0.2, 0, 0);
					btMatrix3x3 frameInParent;
					//frameInParent.setRotation(btQuaternion(0, 0, 0, 1));
					frameInParent.setIdentity();
					btVector3 pivotInChild(0, 0, 0);
					btMatrix3x3 frameInChild;
					frameInChild.setIdentity();

					m_data->m_gripperRigidbodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody, -1, 0, pivotInParent, pivotInChild, frameInParent, frameInChild);
					m_data->m_gripperMultiBody = parentBody->m_multiBody;
					if (m_data->m_gripperMultiBody->getNumLinks() > 2)
					{
						m_data->m_gripperMultiBody->setJointPos(0, 0);
						m_data->m_gripperMultiBody->setJointPos(2, 0);
					}
					m_data->m_gripperRigidbodyFixed->setMaxAppliedImpulse(500);
					btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
					world->addMultiBodyConstraint(m_data->m_gripperRigidbodyFixed);
				}
			}	
		}

		loadUrdf("kuka_iiwa/model_vr_limits.urdf", btVector3(1.4, -0.2, 0.6), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		m_data->m_KukaId = bodyId;
		if (m_data->m_KukaId>=0)
		{
			InteralBodyData* kukaBody = m_data->getHandle(m_data->m_KukaId);
			if (kukaBody->m_multiBody && kukaBody->m_multiBody->getNumDofs() == 7)
			{
				btScalar q[7];
				q[0] = 0;// -SIMD_HALF_PI;
				q[1] = 0;
				q[2] = 0;
				q[3] = SIMD_HALF_PI;
				q[4] = 0;
				q[5] = -SIMD_HALF_PI*0.66;
				q[6] = 0;

				for (int i = 0; i < 7; i++)
				{
					kukaBody->m_multiBody->setJointPos(i, q[i]);
				}
				btAlignedObjectArray<btQuaternion> scratch_q;
				btAlignedObjectArray<btVector3> scratch_m;
				kukaBody->m_multiBody->forwardKinematics(scratch_q, scratch_m);
				int nLinks = kukaBody->m_multiBody->getNumLinks();
				scratch_q.resize(nLinks + 1);
				scratch_m.resize(nLinks + 1);
				kukaBody->m_multiBody->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
			}
		}
#if 1
		loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .7), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .8), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .9), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
#endif

		//		loadUrdf("r2d2.urdf", btVector3(-2, -4, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

#if 1
		// Load one motor gripper for kuka
		loadSdf("gripper/wsg50_one_motor_gripper_new_free_base.sdf", &gBufferServerToClient[0], gBufferServerToClient.size(), true,CUF_USE_SDF);
		m_data->m_gripperId = bodyId + 1;
		{
		InteralBodyData* gripperBody = m_data->getHandle(m_data->m_gripperId);

		// Reset the default gripper motor maximum torque for damping to 0
		for (int i = 0; i < gripperBody->m_multiBody->getNumLinks(); i++)
		{
			if (supportsJointMotor(gripperBody->m_multiBody, i))
			{
				btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)gripperBody->m_multiBody->getLink(i).m_userPtr;
				if (motor)
				{
					motor->setMaxAppliedImpulse(0);
				}
			}
		}
		}
#endif
#if 1
		for (int i = 0; i < 6; i++)
		{
			loadUrdf("jenga/jenga.urdf", btVector3(1.3-0.1*i,-0.7,  .75), btQuaternion(btVector3(0,1,0),SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		}
#endif
		//loadUrdf("nao/nao.urdf", btVector3(2,5, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

		// Add slider joint for fingers
		btVector3 pivotInParent1(-0.055, 0, 0.02);
		btVector3 pivotInChild1(0, 0, 0);
		btMatrix3x3 frameInParent1(btQuaternion(0, 0, 0, 1.0));
		btMatrix3x3 frameInChild1(btQuaternion(0, 0, 0, 1.0));
		btVector3 jointAxis1(1.0, 0, 0);
		btVector3 pivotInParent2(0.055, 0, 0.02);
		btVector3 pivotInChild2(0, 0, 0);
		btMatrix3x3 frameInParent2(btQuaternion(0, 0, 0, 1.0));
		btMatrix3x3 frameInChild2(btQuaternion(0, 0, 1.0, 0));
		btVector3 jointAxis2(1.0, 0, 0);

		if (m_data->m_gripperId>=0)
		{
		InteralBodyData* gripperBody = m_data->getHandle(m_data->m_gripperId);
		m_data->m_kukaGripperRevolute1 = new btMultiBodyPoint2Point(gripperBody->m_multiBody, 2, gripperBody->m_multiBody, 4, pivotInParent1, pivotInChild1);
		m_data->m_kukaGripperRevolute1->setMaxAppliedImpulse(5.0);
		m_data->m_kukaGripperRevolute2 = new btMultiBodyPoint2Point(gripperBody->m_multiBody, 3, gripperBody->m_multiBody, 6, pivotInParent2, pivotInChild2);
		m_data->m_kukaGripperRevolute2->setMaxAppliedImpulse(5.0);

		m_data->m_dynamicsWorld->addMultiBodyConstraint(m_data->m_kukaGripperRevolute1);
		m_data->m_dynamicsWorld->addMultiBodyConstraint(m_data->m_kukaGripperRevolute2);

		}

		if (m_data->m_KukaId>=0)
		{
		InteralBodyData* kukaBody = m_data->getHandle(m_data->m_KukaId);
		if (kukaBody->m_multiBody && kukaBody->m_multiBody->getNumDofs()==7)
		{
			if (m_data->m_gripperId>=0)
			{
			InteralBodyData* gripperBody = m_data->getHandle(m_data->m_gripperId);

			gripperBody->m_multiBody->setHasSelfCollision(0);
			btVector3 pivotInParent(0, 0, 0.05);
			btMatrix3x3 frameInParent;
			frameInParent.setIdentity();
			btVector3 pivotInChild(0, 0, 0);
			btMatrix3x3 frameInChild;
			frameInChild.setIdentity();

			m_data->m_kukaGripperFixed = new btMultiBodyFixedConstraint(kukaBody->m_multiBody, 6, gripperBody->m_multiBody, 0, pivotInParent, pivotInChild, frameInParent, frameInChild);
			m_data->m_kukaGripperMultiBody = gripperBody->m_multiBody;
			m_data->m_kukaGripperFixed->setMaxAppliedImpulse(500);
			m_data->m_dynamicsWorld->addMultiBodyConstraint(m_data->m_kukaGripperFixed);
			}
		}
		}
#if 0

		for (int i = 0; i < 10; i++)
		{
			loadUrdf("cube.urdf", btVector3(-4, -2, 0.5 + i), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		}

		loadUrdf("sphere2.urdf", btVector3(-5, 0, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("sphere2.urdf", btVector3(-5, 0, 2), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("sphere2.urdf", btVector3(-5, 0, 3), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
#endif		
		btTransform objectLocalTr[] = {
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.0, 0.0)),
			btTransform(btQuaternion(btVector3(0,0,1),-SIMD_HALF_PI), btVector3(0.0, 0.15, 0.64)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.1, 0.15, 0.85)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.4, 0.05, 0.85)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.3, -0.05, 0.7)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.1, 0.05, 0.7)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.2, 0.15, 0.7)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.2, 0.15, 0.9)),
			btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.2, 0.05, 0.8))
		};


		btAlignedObjectArray<btTransform> objectWorldTr;
		int numOb = sizeof(objectLocalTr) / sizeof(btTransform);
		objectWorldTr.resize(numOb);

		btTransform tr;
		tr.setIdentity();
		tr.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI));
		tr.setOrigin(btVector3(1.0, -0.2, 0));

		for (int i = 0; i < numOb; i++)
		{
			objectWorldTr[i] = tr*objectLocalTr[i];
		}

		// Table area
		loadUrdf("table/table.urdf", objectWorldTr[0].getOrigin(), objectWorldTr[0].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("tray/tray_textured.urdf", objectWorldTr[1].getOrigin(), objectWorldTr[1].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("cup_small.urdf", objectWorldTr[2].getOrigin(), objectWorldTr[2].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("pitcher_small.urdf", objectWorldTr[3].getOrigin(), objectWorldTr[3].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("teddy_vhacd.urdf", objectWorldTr[4].getOrigin(), objectWorldTr[4].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

		loadUrdf("cube_small.urdf", objectWorldTr[5].getOrigin(), objectWorldTr[5].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("sphere_small.urdf", objectWorldTr[6].getOrigin(), objectWorldTr[6].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("duck_vhacd.urdf", objectWorldTr[7].getOrigin(), objectWorldTr[7].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("Apple/apple.urdf", objectWorldTr[8].getOrigin(), objectWorldTr[8].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

		// Shelf area
		loadSdf("kiva_shelf/model.sdf", &gBufferServerToClient[0], gBufferServerToClient.size(), true, CUF_USE_SDF);
		loadUrdf("teddy_vhacd.urdf", btVector3(-0.1, 0.6, 0.85), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());			
		loadUrdf("sphere_small.urdf", btVector3(-0.1, 0.6, 1.25), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		loadUrdf("cube_small.urdf", btVector3(0.3, 0.6, 0.85), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
			
		// Chess area
		loadUrdf("table_square/table_square.urdf", btVector3(-1.0, 0, 0.0), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("pawn.urdf", btVector3(-0.8, -0.1, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("queen.urdf", btVector3(-0.9, -0.2, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("king.urdf", btVector3(-1.0, 0, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("bishop.urdf", btVector3(-1.1, 0.1, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("rook.urdf", btVector3(-1.2, 0, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		//loadUrdf("knight.urdf", btVector3(-1.2, 0.2, 0.7), btQuaternion(btVector3(1, 0, 0), SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

		loadUrdf("husky/husky.urdf", btVector3(2, -5, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
		m_data->m_huskyId = bodyId;

		m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, -10));

	}

	if (m_data->m_kukaGripperFixed && m_data->m_kukaGripperMultiBody)
	{
		InteralBodyData* childBody = m_data->getHandle(m_data->m_gripperId);
		// Add gripper controller
		btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)childBody->m_multiBody->getLink(1).m_userPtr;
		if (motor)
		{
			btScalar posTarget = (-0.048)*btMin(btScalar(0.75), gVRGripper2Analog) / 0.75;
			motor->setPositionTarget(posTarget, .8);
			motor->setVelocityTarget(0.0, .5);
			motor->setMaxAppliedImpulse(1.0);

		}
	}

	if (m_data->m_gripperRigidbodyFixed && m_data->m_gripperMultiBody)
	{
		m_data->m_gripperRigidbodyFixed->setFrameInB(btMatrix3x3(gVRGripperOrn));
		m_data->m_gripperRigidbodyFixed->setPivotInB(gVRGripperPos);
		btScalar avg = 0.f;

		for (int i = 0; i < m_data->m_gripperMultiBody->getNumLinks(); i++)
		{
			if (supportsJointMotor(m_data->m_gripperMultiBody, i))
			{
				btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)m_data->m_gripperMultiBody->getLink(i ).m_userPtr;
				if (motor)
				{
					motor->setErp(0.2);
					btScalar posTarget = 0.1 + (1 - btMin(btScalar(0.75),gVRGripperAnalog)*btScalar(1.5))*SIMD_HALF_PI*0.29;
					btScalar maxPosTarget = 0.55;
					
					btScalar correction = 0.f;

					if (avg)
					{
						correction = m_data->m_gripperMultiBody->getJointPos(i) - avg;
					}
					
					if (m_data->m_gripperMultiBody->getJointPos(i) < 0)
					{
						m_data->m_gripperMultiBody->setJointPos(i,0);
					}
					if (m_data->m_gripperMultiBody->getJointPos(i) > maxPosTarget)
					{
						m_data->m_gripperMultiBody->setJointPos(i, maxPosTarget);
					}

					if (avg)
					{
						motor->setPositionTarget(avg, 1);
					}
					else
					{
						motor->setPositionTarget(posTarget, 1);
					}
					motor->setVelocityTarget(0, 0.5);
					btScalar maxImp = (1+0.1*i)*m_data->m_physicsDeltaTime;
					motor->setMaxAppliedImpulse(maxImp);
					avg = m_data->m_gripperMultiBody->getJointPos(i);

					//motor->setRhsClamp(gRhsClamp);
				}
			}
		}
	}

	// Inverse kinematics for KUKA
	if (m_data->m_KukaId>=0)
	{
		InternalBodyHandle* bodyHandle = m_data->getHandle(m_data->m_KukaId);
		if (bodyHandle && bodyHandle->m_multiBody && bodyHandle->m_multiBody->getNumDofs()==7)
		{
			btMultiBody* mb = bodyHandle->m_multiBody;				
			btScalar sqLen = (mb->getBaseWorldTransform().getOrigin() - gVRController2Pos).length2();
			btScalar distanceThreshold = 1.3;
			gCloseToKuka=(sqLen<(distanceThreshold*distanceThreshold));

			int numDofs = bodyHandle->m_multiBody->getNumDofs();
			btAlignedObjectArray<double> q_new;
			btAlignedObjectArray<double> q_current;
			q_current.resize(numDofs);
			for (int i = 0; i < numDofs; i++)
			{
				q_current[i] = bodyHandle->m_multiBody->getJointPos(i);
			}
                       
			q_new.resize(numDofs);
			//sensible rest-pose
			q_new[0] = 0;// -SIMD_HALF_PI;
			q_new[1] = 0;
			q_new[2] = 0;
			q_new[3] = SIMD_HALF_PI;
			q_new[4] = 0;
			q_new[5] = -SIMD_HALF_PI*0.66;
			q_new[6] = 0;

			if (gCloseToKuka && gEnableKukaControl)
			{
				double dampIk[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 0.0};

				IKTrajectoryHelper** ikHelperPtrPtr = m_data->m_inverseKinematicsHelpers.find(bodyHandle->m_multiBody);
				IKTrajectoryHelper* ikHelperPtr = 0;

				if (ikHelperPtrPtr)
				{
					ikHelperPtr = *ikHelperPtrPtr;
				}
				else
				{
					IKTrajectoryHelper* tmpHelper = new IKTrajectoryHelper;
					m_data->m_inverseKinematicsHelpers.insert(bodyHandle->m_multiBody, tmpHelper);
					ikHelperPtr = tmpHelper;
				}

				int endEffectorLinkIndex = 6;

				if (ikHelperPtr && (endEffectorLinkIndex<bodyHandle->m_multiBody->getNumLinks()))
				{					
					b3AlignedObjectArray<double> jacobian_linear;
					jacobian_linear.resize(3*numDofs);
					b3AlignedObjectArray<double> jacobian_angular;
					jacobian_angular.resize(3*numDofs);
					int jacSize = 0;
                                
					btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);
                               
					if (tree)
					{
						jacSize = jacobian_linear.size();
						// Set jacobian value
						int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
                                                                      
						btInverseDynamics::vecx nu(numDofs+baseDofs), qdot(numDofs + baseDofs), q(numDofs + baseDofs), joint_force(numDofs + baseDofs);
						for (int i = 0; i < numDofs; i++)
						{
							q_current[i] = bodyHandle->m_multiBody->getJointPos(i);
							q[i+baseDofs] = bodyHandle->m_multiBody->getJointPos(i);
							qdot[i + baseDofs] = 0;
							nu[i+baseDofs] = 0;
						}
						// Set the gravity to correspond to the world gravity
						btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
                                    
						if (-1 != tree->setGravityInWorldFrame(id_grav) &&
							-1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
						{
							tree->calculateJacobians(q);
							btInverseDynamics::mat3x jac_t(3,numDofs);
							btInverseDynamics::mat3x jac_r(3,numDofs);
							tree->getBodyJacobianTrans(endEffectorLinkIndex, &jac_t);
							tree->getBodyJacobianRot(endEffectorLinkIndex, &jac_r);
							for (int i = 0; i < 3; ++i)
							{
								for (int j = 0; j < numDofs; ++j)
								{
									jacobian_linear[i*numDofs+j] = jac_t(i,j);
									jacobian_angular[i*numDofs+j] = jac_r(i,j);
								}
							}
						}
					}
                       
					int ikMethod= IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE;//IK2_VEL_DLS_WITH_ORIENTATION; //IK2_VEL_DLS;
                               
					btVector3DoubleData endEffectorWorldPosition;
					btVector3DoubleData endEffectorWorldOrientation;
					btVector3DoubleData targetWorldPosition;
					btVector3DoubleData targetWorldOrientation;
                                
					btVector3 endEffectorPosWorld =  bodyHandle->m_multiBody->getLink(endEffectorLinkIndex).m_cachedWorldTransform.getOrigin();
					btQuaternion endEffectorOriWorld = bodyHandle->m_multiBody->getLink(endEffectorLinkIndex).m_cachedWorldTransform.getRotation();
					btVector4 endEffectorOri(endEffectorOriWorld.x(),endEffectorOriWorld.y(),endEffectorOriWorld.z(),endEffectorOriWorld.w());
                        
					// Prescribed position and orientation
					static btScalar time=0.f;
					time+=0.01;
					btVector3 targetPos(0.4-0.4*b3Cos( time), 0, 0.8+0.4*b3Cos( time));
					targetPos +=mb->getBasePos();
					btVector4 downOrn(0,1,0,0);

					// Controller orientation
					btVector4 controllerOrn(gVRController2Orn.x(), gVRController2Orn.y(), gVRController2Orn.z(), gVRController2Orn.w());
						
					// Set position and orientation
					endEffectorPosWorld.serializeDouble(endEffectorWorldPosition);
					endEffectorOri.serializeDouble(endEffectorWorldOrientation);
					downOrn.serializeDouble(targetWorldOrientation);
					//targetPos.serializeDouble(targetWorldPosition);
						
					gVRController2Pos.serializeDouble(targetWorldPosition);
						
					//controllerOrn.serializeDouble(targetWorldOrientation);

					if (ikMethod == IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE)
					{
						btAlignedObjectArray<double> lower_limit;
						btAlignedObjectArray<double> upper_limit;
						btAlignedObjectArray<double> joint_range;
						btAlignedObjectArray<double> rest_pose;
						lower_limit.resize(numDofs);
						upper_limit.resize(numDofs);
						joint_range.resize(numDofs);
						rest_pose.resize(numDofs);
						lower_limit[0] = -.967;
						lower_limit[1] = -2.0;
						lower_limit[2] = -2.96;
						lower_limit[3] = 0.19;
						lower_limit[4] = -2.96;
						lower_limit[5] = -2.09;
						lower_limit[6] = -3.05;
						upper_limit[0] = .96;
						upper_limit[1] = 2.0;
						upper_limit[2] = 2.96;
						upper_limit[3] = 2.29;
						upper_limit[4] = 2.96;
						upper_limit[5] = 2.09;
						upper_limit[6] = 3.05;
						joint_range[0] = 5.8;
						joint_range[1] = 4;
						joint_range[2] = 5.8;
						joint_range[3] = 4;
						joint_range[4] = 5.8;
						joint_range[5] = 4;
						joint_range[6] = 6;
						rest_pose[0] = 0;
						rest_pose[1] = 0;
						rest_pose[2] = 0;
						rest_pose[3] = SIMD_HALF_PI;
						rest_pose[4] = 0;
						rest_pose[5] = -SIMD_HALF_PI*0.66;
						rest_pose[6] = 0;
						ikHelperPtr->computeNullspaceVel(numDofs, &q_current[0], &lower_limit[0], &upper_limit[0], &joint_range[0], &rest_pose[0]);
					}

					ikHelperPtr->computeIK(targetWorldPosition.m_floats, targetWorldOrientation.m_floats,
													endEffectorWorldPosition.m_floats, endEffectorWorldOrientation.m_floats,
													&q_current[0],
													numDofs, endEffectorLinkIndex,
								&q_new[0], ikMethod, &jacobian_linear[0], &jacobian_angular[0], jacSize*2, dampIk);
				}
			}

			//directly set the position of the links, only for debugging IK, don't use this method!
#if 0			
			if (0)
			{
				for (int i=0;i<mb->getNumLinks();i++)
				{
					btScalar desiredPosition = q_new[i];
					mb->setJointPosMultiDof(i,&desiredPosition);
				}
			} else
#endif
			{
				int numMotors = 0;
				//find the joint motors and apply the desired velocity and maximum force/torque
				{
					int velIndex = 6;//skip the 3 linear + 3 angular degree of freedom velocity entries of the base
					int posIndex = 7;//skip 3 positional and 4 orientation (quaternion) positional degrees of freedom of the base
					for (int link=0;link<mb->getNumLinks();link++)
					{
						if (supportsJointMotor(mb,link))
						{
							btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;

							if (motor)
							{
								btScalar desiredVelocity = 0.f;
								btScalar desiredPosition = q_new[link];
								motor->setRhsClamp(gRhsClamp);
								//printf("link %d: %f", link, q_new[link]);
								motor->setVelocityTarget(desiredVelocity,1.0);
								motor->setPositionTarget(desiredPosition,0.6);
								btScalar maxImp = 1.0;
									
								motor->setMaxAppliedImpulse(maxImp);
								numMotors++;
							}
						}
						velIndex += mb->getLink(link).m_dofCount;
						posIndex += mb->getLink(link).m_posVarCount;
					}
				}
			}
		} 
			
	}
}
