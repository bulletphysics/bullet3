#include "PhysicsServerCommandProcessor.h"

#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"
#include "TinyRendererVisualShapeConverter.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "LinearMath/btHashMap.h"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include "IKTrajectoryHelper.h"
#include "btBulletDynamicsCommon.h"

#include "LinearMath/btTransform.h"

#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "LinearMath/btSerializer.h"
#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryCommands.h"


//@todo(erwincoumans) those globals are hacks for a VR demo, move this to Python/pybullet!
btVector3 gLastPickPos(0, 0, 0);
bool gCloseToKuka=false;
bool gEnableRealTimeSimVR=false;
bool gCreateSamuraiRobotAssets = true;

int gCreateObjectSimVR = -1;
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

struct SaveWorldObjectData
{
	b3AlignedObjectArray<int> m_bodyUniqueIds;
	std::string	m_fileName;
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

	bool m_allowRealTimeSimulation;
	bool m_hasGround;

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
	b3AlignedObjectArray<SaveWorldObjectData> m_saveWorldBodyData;


	btAlignedObjectArray<btBulletWorldImporter*> m_worldImporters;
	btAlignedObjectArray<UrdfLinkNameMapUtil*> m_urdfLinkNameMapper;
	btAlignedObjectArray<std::string*> m_strings;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btMultiBodyConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	SharedMemoryDebugDrawer*		m_remoteDebugDrawer;
	
	btAlignedObjectArray<b3ContactPointData> m_cachedContactPoints;

	btAlignedObjectArray<int> m_sdfRecentLoadedBodies;



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
		:m_hasGround(false),
		m_gripperRigidbodyFixed(0),
		m_gripperMultiBody(0),
		m_kukaGripperFixed(0),
		m_kukaGripperMultiBody(0),
		m_kukaGripperRevolute1(0),
		m_kukaGripperRevolute2(0),
		m_allowRealTimeSimulation(false),
		m_huskyId(-1),
		m_KukaId(-1),
		m_sphereId(-1),
		m_gripperId(-1),
		m_commandLogger(0),
		m_logPlayback(0),
		m_physicsDeltaTime(1./240.),
        m_numSimulationSubSteps(0),
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
	m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = 0.00001;
	m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = 100;

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

	//Workaround: in a VR application, where we avoid synchronizaing between GFX/Physics threads, we don't want to resize this array, so pre-allocate it
	m_data->m_dynamicsWorld->getCollisionObjectArray().reserve(8192);

	m_data->m_remoteDebugDrawer = new SharedMemoryDebugDrawer();


	m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_data->m_dynamicsWorld->getSolverInfo().m_erp2 = 0.08;

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


bool PhysicsServerCommandProcessor::loadSdf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody)
{
    btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadSdf: No valid m_dynamicsWorld");
		return false;
	}

	m_data->m_sdfRecentLoadedBodies.clear();

    BulletURDFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter);

    bool useFixedBase = false;
    bool loadOk =  u2b.loadSDF(fileName, useFixedBase);
    if (loadOk)
    {
        for (int i=0;i<u2b.getNumAllocatedCollisionShapes();i++)
        {
            btCollisionShape* shape =u2b.getAllocatedCollisionShape(i);
            m_data->m_collisionShapes.push_back(shape);
        }

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
            ConvertURDF2Bullet(u2b,creation, rootTrans,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix(),CUF_USE_SDF);



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
		
		m_data->m_saveWorldBodyData.push_back(sd);

    }
    return loadOk;
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
			    util->m_memSerializer = new btDefaultSerializer(bufferSizeInBytes ,(unsigned char*)bufferServerToClient);
			    //disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
				util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(),0);

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

        util->m_memSerializer->insertHeader();

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
            int timeStamp = 0;
			
			//catch uninitialized cases
			serverStatusOut.m_type = CMD_INVALID_STATUS;

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

                case CMD_REQUEST_BODY_INFO:
                    {
                        const SdfRequestInfoArgs& sdfInfoArgs = clientCmd.m_sdfRequestInfoArgs;
                        //stream info into memory
                        int streamSizeInBytes = createBodyInfoStream(sdfInfoArgs.m_bodyUniqueId, bufferServerToClient, bufferSizeInBytes);

                        serverStatusOut.m_type = CMD_BODY_INFO_COMPLETED;
                        serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = sdfInfoArgs.m_bodyUniqueId;
                        serverStatusOut.m_dataStreamArguments.m_streamChunkLength = streamSizeInBytes;
                        hasStatus = true;
                        break;
                    }
				case CMD_SAVE_WORLD:
				{
					///this is a very rudimentary way to save the state of the world, for scene authoring
					///many todo's, for example save the state of motor controllers etc.

					
					{
						//saveWorld(clientCmd.m_sdfArguments.m_sdfFileName);

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
                        bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? sdfArgs.m_useMultiBody : true;

                        bool completedOk = loadSdf(sdfArgs.m_sdfFileName,bufferServerToClient, bufferSizeInBytes, useMultiBody);
                        if (completedOk)
                        {
                            //serverStatusOut.m_type = CMD_SDF_LOADING_FAILED;
                            serverStatusOut.m_sdfLoadedArgs.m_numBodies = m_data->m_sdfRecentLoadedBodies.size();
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
                        serverStatusOut.m_dataStreamArguments.m_streamChunkLength = 0;

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

					if (body && body->m_multiBody)
					{
						btMultiBody* mb = body->m_multiBody;
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_POSITION)
						{
							btVector3 zero(0,0,0);
							btAssert(clientCmd.m_initPoseArgs.m_hasInitialStateQ[0] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[1] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[2]);

							mb->setBaseVel(zero);
							mb->setBasePos(btVector3(
									clientCmd.m_initPoseArgs.m_initialStateQ[0],
									clientCmd.m_initPoseArgs.m_initialStateQ[1],
									clientCmd.m_initPoseArgs.m_initialStateQ[2]));
						}
						if (clientCmd.m_updateFlags & INIT_POSE_HAS_INITIAL_ORIENTATION)
						{
						    btAssert(clientCmd.m_initPoseArgs.m_hasInitialStateQ[3] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[4] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[5] &&
                                    clientCmd.m_initPoseArgs.m_hasInitialStateQ[6]);

							mb->setBaseOmega(btVector3(0,0,0));
							btQuaternion invOrn(clientCmd.m_initPoseArgs.m_initialStateQ[3],
									clientCmd.m_initPoseArgs.m_initialStateQ[4],
									clientCmd.m_initPoseArgs.m_initialStateQ[5],
									clientCmd.m_initPoseArgs.m_initialStateQ[6]);

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
					}

					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;

					break;
				}


                case CMD_RESET_SIMULATION:
                {
					//clean up all data
					deleteCachedInverseDynamicsBodies();

					if (m_data && m_data->m_guiHelper)
					{
						m_data->m_guiHelper->removeAllGraphicsInstances();
					}
					if (m_data)
					{
						m_data->m_visualConverter.resetAll();
					}

					deleteDynamicsWorld();
					createEmptyDynamicsWorld();

					m_data->exitHandles();
					m_data->initHandles();

					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
					hasStatus = true;
					m_data->m_hasGround = false;
					m_data->m_gripperRigidbodyFixed = 0;
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
                case CMD_REQUEST_CONTACT_POINT_INFORMATION:
                    {
                        SharedMemoryStatus& serverCmd =serverStatusOut;
                        serverCmd.m_sendContactPointArgs.m_numContactPointsCopied = 0;
                        
                        //make a snapshot of the contact manifolds into individual contact points
                        if (clientCmd.m_requestContactPointArguments.m_startingContactPointIndex==0)
                        {
                            int numContactManifolds = m_data->m_dynamicsWorld->getDispatcher()->getNumManifolds();
							m_data->m_cachedContactPoints.resize(0);
							m_data->m_cachedContactPoints.reserve(numContactManifolds*4);
                            for (int i=0;i<numContactManifolds;i++)
                            {
								const btPersistentManifold* manifold =  m_data->m_dynamicsWorld->getDispatcher()->getInternalManifoldPointer()[i];
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
									objectIndexA  = bodyA->getUserIndex2();
								}
								const btMultiBodyLinkCollider* mblA = btMultiBodyLinkCollider::upcast(manifold->getBody0());
								if (mblA && mblA->m_multiBody)
								{
                                    linkIndexA = mblA->m_link;

									objectIndexA = mblA->m_multiBody->getUserIndex2();
								}

								btAssert(bodyA || mblA);

								//apply the filter, if the user provides it
								if (clientCmd.m_requestContactPointArguments.m_objectAIndexFilter>=0)
								{
									if ((clientCmd.m_requestContactPointArguments.m_objectAIndexFilter != objectIndexA) &&
										(clientCmd.m_requestContactPointArguments.m_objectAIndexFilter != objectIndexB))
									continue;
								}

                                //apply the second object filter, if the user provides it
								if (clientCmd.m_requestContactPointArguments.m_objectBIndexFilter>=0)
								{
									if ((clientCmd.m_requestContactPointArguments.m_objectBIndexFilter != objectIndexA) &&
										(clientCmd.m_requestContactPointArguments.m_objectBIndexFilter != objectIndexB))
									continue;
								}

								for (int p=0;p<manifold->getNumContacts();p++)
								{

									b3ContactPointData pt;
									pt.m_bodyUniqueIdA = objectIndexA;
									pt.m_bodyUniqueIdB = objectIndexB;
									const btManifoldPoint& srcPt = manifold->getContactPoint(p);
									pt.m_contactDistance = srcPt.getDistance();
									pt.m_contactFlags = 0;
									pt.m_linkIndexA = linkIndexA;
									pt.m_linkIndexB = linkIndexB;
									for (int j=0;j<3;j++)
									{
										pt.m_contactNormalOnBInWS[j] = srcPt.m_normalWorldOnB[j];
										pt.m_positionOnAInWS[j] = srcPt.getPositionWorldOnA()[j];
										pt.m_positionOnBInWS[j] = srcPt.getPositionWorldOnB()[j];
									}
                                    pt.m_normalForce = srcPt.getAppliedImpulse()/m_data->m_physicsDeltaTime;
//                                    pt.m_linearFrictionForce = srcPt.m_appliedImpulseLateral1;
									m_data->m_cachedContactPoints.push_back (pt);
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
                        if (body && body->m_multiBody)
                        {
                            btMultiBody* mb = body->m_multiBody;
                            bool isLinkFrame = ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_LINK_FRAME)!=0);

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
                    }

                    SharedMemoryStatus& serverCmd =serverStatusOut;
                    serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
                    hasStatus = true;
                    break;
                }
                case CMD_CREATE_JOINT:
                {
                    InteralBodyData* parentBody = m_data->getHandle(clientCmd.m_createJointArguments.m_parentBodyIndex);
                    if (parentBody && parentBody->m_multiBody)
                    {
                        InteralBodyData* childBody = m_data->getHandle(clientCmd.m_createJointArguments.m_childBodyIndex);
                        if (childBody)
                        {
                            btVector3 pivotInParent(clientCmd.m_createJointArguments.m_parentFrame[0], clientCmd.m_createJointArguments.m_parentFrame[1], clientCmd.m_createJointArguments.m_parentFrame[2]);
                            btVector3 pivotInChild(clientCmd.m_createJointArguments.m_childFrame[0], clientCmd.m_createJointArguments.m_childFrame[1], clientCmd.m_createJointArguments.m_childFrame[2]);
                            btMatrix3x3 frameInParent(btQuaternion(clientCmd.m_createJointArguments.m_parentFrame[3], clientCmd.m_createJointArguments.m_parentFrame[4], clientCmd.m_createJointArguments.m_parentFrame[5], clientCmd.m_createJointArguments.m_parentFrame[6]));
                            btMatrix3x3 frameInChild(btQuaternion(clientCmd.m_createJointArguments.m_childFrame[3], clientCmd.m_createJointArguments.m_childFrame[4], clientCmd.m_createJointArguments.m_childFrame[5], clientCmd.m_createJointArguments.m_childFrame[6]));
                            btVector3 jointAxis(clientCmd.m_createJointArguments.m_jointAxis[0], clientCmd.m_createJointArguments.m_jointAxis[1], clientCmd.m_createJointArguments.m_jointAxis[2]);
                            if (clientCmd.m_createJointArguments.m_jointType == eFixedType)
                            {
                                if (childBody->m_multiBody)
                                {
                                    btMultiBodyFixedConstraint* multibodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_createJointArguments.m_childJointIndex,pivotInParent,pivotInChild,frameInParent,frameInChild);
                                    multibodyFixed->setMaxAppliedImpulse(500.0);
                                    m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodyFixed);
                                }
                                else
                                {
                                    btMultiBodyFixedConstraint* rigidbodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_rigidBody,pivotInParent,pivotInChild,frameInParent,frameInChild);
                                    rigidbodyFixed->setMaxAppliedImpulse(500.0);
                                    btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
                                    world->addMultiBodyConstraint(rigidbodyFixed);
                                }
                            }
                            else if (clientCmd.m_createJointArguments.m_jointType == ePrismaticType)
                            {
                                if (childBody->m_multiBody)
                                {
                                    btMultiBodySliderConstraint* multibodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_createJointArguments.m_childJointIndex,pivotInParent,pivotInChild,frameInParent,frameInChild,jointAxis);
                                    multibodySlider->setMaxAppliedImpulse(500.0);
                                    m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodySlider);
                                }
                                else
                                {
                                    btMultiBodySliderConstraint* rigidbodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_rigidBody,pivotInParent,pivotInChild,frameInParent,frameInChild,jointAxis);
                                    rigidbodySlider->setMaxAppliedImpulse(500.0);
                                    btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
                                    world->addMultiBodyConstraint(rigidbodySlider);
                                }
                            } else if (clientCmd.m_createJointArguments.m_jointType == ePoint2PointType)
                            {
                                if (childBody->m_multiBody)
                                {
									btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_createJointArguments.m_childJointIndex,pivotInParent,pivotInChild);
                                    p2p->setMaxAppliedImpulse(500);
                                    m_data->m_dynamicsWorld->addMultiBodyConstraint(p2p);
                                }
                                else
                                {
                                    btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody,clientCmd.m_createJointArguments.m_parentJointIndex,childBody->m_rigidBody,pivotInParent,pivotInChild);
                                    p2p->setMaxAppliedImpulse(500);
                                    btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_data->m_dynamicsWorld;
                                    world->addMultiBodyConstraint(p2p);
                                }
                            }

                        }
                    }
					SharedMemoryStatus& serverCmd =serverStatusOut;
                    serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
                    hasStatus = true;
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
								double dampIK[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
                                ikHelperPtr->computeIK(clientCmd.m_calculateInverseKinematicsArguments.m_targetPosition, clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation,
                                                       endEffectorWorldPosition.m_floats, endEffectorWorldOrientation.m_floats,
                                                       &q_current[0],
                                                       numDofs, clientCmd.m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex,
									&q_new[0], ikMethod, &jacobian_linear[0], &jacobian_angular[0], jacSize*2, dampIK);
                                
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

static int skip=1;

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


btVector3 gVRGripperPos(0,0,0.2);
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
void PhysicsServerCommandProcessor::stepSimulationRealTime(double dtInSec)
{
	if ((gEnableRealTimeSimVR || m_data->m_allowRealTimeSimulation) && m_data->m_guiHelper)
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

		///this hardcoded C++ scene creation is temporary for demo purposes. It will be done in Python later...
		if (gCreateSamuraiRobotAssets)
		{
			if (!m_data->m_hasGround)
			{
				m_data->m_hasGround = true;

				loadUrdf("plane.urdf", btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1), true, true, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("samurai.urdf", btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

				if (m_data->m_gripperRigidbodyFixed == 0)
				{
					int bodyId = 0;

					if (loadUrdf("pr2_gripper.urdf", btVector3(0, 0, 0.1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size()))
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
				loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .7), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .8), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("lego/lego.urdf", btVector3(1.0, -0.2, .9), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("r2d2.urdf", btVector3(-2, -4, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

				// Load one motor gripper for kuka
				loadSdf("gripper/wsg50_one_motor_gripper_new_free_base.sdf", &gBufferServerToClient[0], gBufferServerToClient.size(), true);
				m_data->m_gripperId = bodyId + 1;
				InteralBodyData* kukaBody = m_data->getHandle(m_data->m_KukaId);
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
				
				for (int i = 0; i < 6; i++)
				{
					loadUrdf("jenga/jenga.urdf", btVector3(1.3-0.1*i,-0.7,  .75), btQuaternion(btVector3(0,1,0),SIMD_HALF_PI), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				}

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
				m_data->m_kukaGripperRevolute1 = new btMultiBodyPoint2Point(gripperBody->m_multiBody, 2, gripperBody->m_multiBody, 4, pivotInParent1, pivotInChild1);
				m_data->m_kukaGripperRevolute1->setMaxAppliedImpulse(5.0);
				m_data->m_kukaGripperRevolute2 = new btMultiBodyPoint2Point(gripperBody->m_multiBody, 3, gripperBody->m_multiBody, 6, pivotInParent2, pivotInChild2);
				m_data->m_kukaGripperRevolute2->setMaxAppliedImpulse(5.0);

				m_data->m_dynamicsWorld->addMultiBodyConstraint(m_data->m_kukaGripperRevolute1);
				m_data->m_dynamicsWorld->addMultiBodyConstraint(m_data->m_kukaGripperRevolute2);

				if (kukaBody->m_multiBody && kukaBody->m_multiBody->getNumDofs()==7)
				{
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

				for (int i = 0; i < 10; i++)
				{
					loadUrdf("cube.urdf", btVector3(-4, -2, 0.5 + i), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				}
				loadUrdf("sphere2.urdf", btVector3(-5, 0, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("sphere2.urdf", btVector3(-5, 0, 2), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("sphere2.urdf", btVector3(-5, 0, 3), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
			
				btTransform objectLocalTr[] = {
					btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.0, 0.0)),
					btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.15, 0.64)),
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
				loadUrdf("tray.urdf", objectWorldTr[1].getOrigin(), objectWorldTr[1].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				//loadUrdf("cup_small.urdf", objectWorldTr[2].getOrigin(), objectWorldTr[2].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				//loadUrdf("pitcher_small.urdf", objectWorldTr[3].getOrigin(), objectWorldTr[3].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("teddy_vhacd.urdf", objectWorldTr[4].getOrigin(), objectWorldTr[4].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("cube_small.urdf", objectWorldTr[5].getOrigin(), objectWorldTr[5].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("sphere_small.urdf", objectWorldTr[6].getOrigin(), objectWorldTr[6].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				loadUrdf("duck_vhacd.urdf", objectWorldTr[7].getOrigin(), objectWorldTr[7].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
				//loadUrdf("Apple/apple.urdf", objectWorldTr[8].getOrigin(), objectWorldTr[8].getRotation(), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());

				// Shelf area
				loadSdf("kiva_shelf/model.sdf", &gBufferServerToClient[0], gBufferServerToClient.size(), true);
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

				//loadUrdf("husky/husky.urdf", btVector3(2, -5, 1), btQuaternion(0, 0, 0, 1), true, false, &bodyId, &gBufferServerToClient[0], gBufferServerToClient.size());
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
					motor->setPositionTarget(posTarget, .2);
					motor->setVelocityTarget(0.0, .5);
					motor->setMaxAppliedImpulse(5.0);
				}
			}

			if (m_data->m_gripperRigidbodyFixed && m_data->m_gripperMultiBody)
			{
				m_data->m_gripperRigidbodyFixed->setFrameInB(btMatrix3x3(gVRGripperOrn));
				m_data->m_gripperRigidbodyFixed->setPivotInB(gVRGripperPos);
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
						
							if (m_data->m_gripperMultiBody->getJointPos(i) < 0)
							{
								m_data->m_gripperMultiBody->setJointPos(i,0);
							}
							if (m_data->m_gripperMultiBody->getJointPos(i) > maxPosTarget)
							{
								m_data->m_gripperMultiBody->setJointPos(i, maxPosTarget);
							}

							motor->setPositionTarget(posTarget, 1);
							motor->setVelocityTarget(0, 0.5);
							btScalar maxImp = 1*m_data->m_physicsDeltaTime;
							motor->setMaxAppliedImpulse(maxImp);
							//motor->setRhsClamp(gRhsClamp);
						}
					}
				}
			}

			// Inverse kinematics for KUKA
			//if (0)
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

					if (gCloseToKuka)
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
					if (0)
					{
						for (int i=0;i<mb->getNumLinks();i++)
						{
							btScalar desiredPosition = q_new[i];
							mb->setJointPosMultiDof(i,&desiredPosition);
						}
					} else
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

		int maxSteps = m_data->m_numSimulationSubSteps+3;
		if (m_data->m_numSimulationSubSteps)
		{
			gSubStep = m_data->m_physicsDeltaTime / m_data->m_numSimulationSubSteps;
		}
		else
		{
			gSubStep = m_data->m_physicsDeltaTime;
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

