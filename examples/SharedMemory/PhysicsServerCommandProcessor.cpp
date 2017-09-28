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
#include "BulletDynamics/Featherstone/btMultiBodyGearConstraint.h"
#include "../Importers/ImportURDFDemo/UrdfParser.h"
#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "Bullet3Common/b3HashMap.h"
#include "../Utils/ChromeTraceUtil.h"
#include "stb_image/stb_image.h"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include "IKTrajectoryHelper.h"
#include "btBulletDynamicsCommon.h"
#include "../Utils/RobotLoggingUtil.h"
#include "LinearMath/btTransform.h"
#include "../Importers/ImportMJCFDemo/BulletMJCFImporter.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "LinearMath/btSerializer.h"
#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryCommands.h"
#include "LinearMath/btRandom.h"
#include "Bullet3Common/b3ResizablePool.h"
#include "../Utils/b3Clock.h"
#include "b3PluginManager.h"

#ifdef B3_ENABLE_TINY_AUDIO
#include "../TinyAudio/b3SoundEngine.h"
#endif

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


int gInternalSimFlags = 0;
bool gResetSimulation = 0;
int gVRTrackingObjectUniqueId = -1;
int gVRTrackingObjectFlag = VR_CAMERA_TRACK_OBJECT_ORIENTATION;

btTransform gVRTrackingObjectTr = btTransform::getIdentity();




btVector3 gVRTeleportPos1(0,0,0);
btQuaternion gVRTeleportOrn(0, 0, 0,1);


btScalar simTimeScalingFactor = 1;
btScalar gRhsClamp = 1.f;

struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_rigidBodyJoints;

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

struct InternalCollisionShapeData
{
	btCollisionShape* m_collisionShape;
	b3AlignedObjectArray<UrdfCollision> m_urdfCollisionObjects;
	void clear()
	{
		m_collisionShape=0;
	}
};

struct InternalBodyData
{
	btMultiBody* m_multiBody;
	btRigidBody* m_rigidBody;
	int m_testData;
	std::string m_bodyName;

	btTransform m_rootLocalInertialFrame;
	btAlignedObjectArray<btTransform> m_linkLocalInertialFrames;
	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_rigidBodyJoints;
	btAlignedObjectArray<std::string> m_rigidBodyJointNames;
	btAlignedObjectArray<std::string> m_rigidBodyLinkNames;
	
#ifdef B3_ENABLE_TINY_AUDIO
	b3HashMap<btHashInt, SDFAudioSource> m_audioSources;
#endif //B3_ENABLE_TINY_AUDIO

	InternalBodyData()		
	{
		clear();
	}

	void clear()
	{
		m_multiBody=0;
		m_rigidBody=0;
		m_testData=0;
		m_bodyName="";
		m_rootLocalInertialFrame.setIdentity();
		m_linkLocalInertialFrames.clear();
		m_rigidBodyJoints.clear();
		m_rigidBodyJointNames.clear();
		m_rigidBodyLinkNames.clear();
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

struct InternalTextureData
{
	int m_tinyRendererTextureId;
	int m_openglTextureId;
	void clear()
	{
		m_tinyRendererTextureId = -1;
		m_openglTextureId = -1;
	}
};

typedef b3PoolBodyHandle<InternalTextureData> InternalTextureHandle;
typedef b3PoolBodyHandle<InternalBodyData> InternalBodyHandle;
typedef b3PoolBodyHandle<InternalCollisionShapeData> InternalCollisionShapeHandle;

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
		if (m_file)
		{
			btCommandChunk chunk;
			chunk.m_chunkCode = command.m_type;
			chunk.m_oldPtr = 0;
			chunk.m_dna_nr = 0;
			chunk.m_length = sizeof(SharedMemoryCommand);
			chunk.m_number = 1;
			fwrite((const char*)&chunk,sizeof(btCommandChunk), 1,m_file);
			
			switch (command.m_type)
			{
				
				case CMD_LOAD_MJCF:
				{
					fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					fwrite((const char*)&command.m_mjcfArguments , sizeof(MjcfArgs),1,m_file);
					break;
				}
				case CMD_REQUEST_BODY_INFO:
                {
					fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					fwrite((const char*)&command.m_sdfRequestInfoArgs, sizeof(SdfRequestInfoArgs),1,m_file);
					break;
				}
				case CMD_REQUEST_VISUAL_SHAPE_INFO:
				{
					fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					fwrite((const char*)&command.m_requestVisualShapeDataArguments, sizeof(RequestVisualShapeDataArgs),1,m_file);
					break;
				}
				case CMD_LOAD_URDF:
                {
					fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					fwrite((const char*)&command.m_urdfArguments, sizeof(UrdfArgs),1,m_file);
					break;
				 }
				 case CMD_INIT_POSE:
				 {
					 fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					 fwrite((const char*)&command.m_initPoseArgs,sizeof(InitPoseArgs),1,m_file);
					break;
				 };
				 case CMD_REQUEST_ACTUAL_STATE:
				 {
					 fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					 fwrite((const char*)&command.m_requestActualStateInformationCommandArgument,
						 sizeof(RequestActualStateArgs),1,m_file);
					break;
				 };
				 case CMD_SEND_DESIRED_STATE:
				 {
					 fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					 fwrite((const char*)&command.m_sendDesiredStateCommandArgument,sizeof(SendDesiredStateArgs),1,m_file);
					 break;
				 }
				 case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				 {
					 fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					 fwrite((const char*)&command.m_physSimParamArgs, sizeof(SendPhysicsSimulationParameters), 1,m_file);
					 break;
				 }
				 case CMD_REQUEST_CONTACT_POINT_INFORMATION:
				 {
					 fwrite((const char*)&command.m_updateFlags,sizeof(int), 1,m_file);
					 fwrite((const char*)&command.m_requestContactPointArguments,sizeof(RequestContactDataArgs),1,m_file);
					 break;
				 }
				case CMD_STEP_FORWARD_SIMULATION:
				case CMD_RESET_SIMULATION:
				case CMD_REQUEST_INTERNAL_DATA:
				{
					break;
				};
				default:
				{
					fwrite((const char*)&command,sizeof(SharedMemoryCommand),1,m_file);
				}

			};
		}
	}

	CommandLogger(const char* fileName)
	{
		m_file = fopen(fileName,"wb");
		if (m_file)
		{
			unsigned char buf[15];
			buf[12] = 12;
			buf[13] = 13;
			buf[14] = 14;
			writeHeader(buf);
			fwrite(buf,12,1,m_file);
		}
	}
	virtual ~CommandLogger()
	{
		if (m_file)
		{
			fclose(m_file);
		}
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
//for a little while, keep this flag to be able to read 'old' log files
//#define BACKWARD_COMPAT
#if BACKWARD_COMPAT
		SharedMemoryCommand unused;
#endif//BACKWARD_COMPAT
		bool result = false;

		if (m_file)
		{
			size_t s = 0;
			int commandType = -1;

			if (m_fileIs64bit)
			{
				bCommandChunkPtr8 chunk8;
				s = fread((void*)&chunk8,sizeof(bCommandChunkPtr8),1,m_file);
				commandType = chunk8.code;
			} else
			{
				bCommandChunkPtr4 chunk4;
				s = fread((void*)&chunk4,sizeof(bCommandChunkPtr4),1,m_file);
				commandType = chunk4.code;
			}

			if (s==1)
			{
				memset(cmd,0,sizeof(SharedMemoryCommand));
				cmd->m_type = commandType;				

#ifdef BACKWARD_COMPAT
				s = fread(&unused,sizeof(SharedMemoryCommand),1,m_file);
				cmd->m_updateFlags = unused.m_updateFlags;
#endif


				switch (commandType)
				{
				case CMD_LOAD_MJCF:
				{
#ifdef BACKWARD_COMPAT
					cmd->m_mjcfArguments = unused.m_mjcfArguments;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_mjcfArguments,sizeof(MjcfArgs),1,m_file);
#endif
					result=true;
					break;
				}
				case CMD_REQUEST_BODY_INFO:
                {
#ifdef BACKWARD_COMPAT
					cmd->m_sdfRequestInfoArgs = unused.m_sdfRequestInfoArgs;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_sdfRequestInfoArgs,sizeof(SdfRequestInfoArgs),1,m_file);					
#endif
					result=true;
					break;
				}
				case CMD_REQUEST_VISUAL_SHAPE_INFO:
				{
#ifdef BACKWARD_COMPAT
					cmd->m_requestVisualShapeDataArguments = unused.m_requestVisualShapeDataArguments;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_requestVisualShapeDataArguments,sizeof(RequestVisualShapeDataArgs),1,m_file);					
#endif
					result=true;
					break;
				}
				 case CMD_LOAD_URDF:
                {
#ifdef BACKWARD_COMPAT
					 cmd->m_urdfArguments = unused.m_urdfArguments;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_urdfArguments,sizeof(UrdfArgs),1,m_file);					
#endif
					result=true;
					break;
				 }
				 case CMD_INIT_POSE:
				 {
#ifdef BACKWARD_COMPAT
					 cmd->m_initPoseArgs = unused.m_initPoseArgs;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_initPoseArgs,sizeof(InitPoseArgs),1,m_file);					

#endif
					 result=true;
					break;
				 };
				 case CMD_REQUEST_ACTUAL_STATE:
				 {
#ifdef BACKWARD_COMPAT					 
					cmd->m_requestActualStateInformationCommandArgument = unused.m_requestActualStateInformationCommandArgument;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_requestActualStateInformationCommandArgument,sizeof(RequestActualStateArgs),1,m_file);					
#endif
					 result=true;
					break;
				 };
				 case CMD_SEND_DESIRED_STATE:
				 {
#ifdef BACKWARD_COMPAT	
					 cmd->m_sendDesiredStateCommandArgument = unused.m_sendDesiredStateCommandArgument;
#else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_sendDesiredStateCommandArgument ,sizeof(SendDesiredStateArgs),1,m_file);					

#endif
					 result = true;
					 break;
				 }
				 case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				 {
#ifdef BACKWARD_COMPAT	
					 cmd->m_physSimParamArgs = unused.m_physSimParamArgs;
					 #else
					fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_physSimParamArgs ,sizeof(SendPhysicsSimulationParameters),1,m_file);					

					 #endif
					 result = true;
					 break;
				 }
				 case CMD_REQUEST_CONTACT_POINT_INFORMATION:
				 {
#ifdef BACKWARD_COMPAT	
					 cmd->m_requestContactPointArguments = unused.m_requestContactPointArguments;
					 #else
					 fread(&cmd->m_updateFlags,sizeof(int),1,m_file);
					fread(&cmd->m_requestContactPointArguments ,sizeof(RequestContactDataArgs),1,m_file);					

					 #endif
					 result = true;
					 break;
				 }
				 case CMD_STEP_FORWARD_SIMULATION:
				case CMD_RESET_SIMULATION:
				case CMD_REQUEST_INTERNAL_DATA:
				{
					result=true;
					break;
				}
				default:
				{
					s = fread(cmd,sizeof(SharedMemoryCommand),1,m_file);
					result=(s==1);
				}
				};
			}
		}
		return result;

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
			//it is not a multibody, so use -1 otherwise
			m_links.push_back(-1);
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
	virtual void logState(btScalar timeStep)=0;

};

struct VideoMP4Loggger : public InternalStateLogger
{

	struct GUIHelperInterface* m_guiHelper;
	std::string m_fileName;
	VideoMP4Loggger(int loggerUid,const char* fileName,GUIHelperInterface* guiHelper)
		:m_guiHelper(guiHelper)
	{
		m_fileName = fileName;
		m_loggingUniqueId = loggerUid;
		m_loggingType = STATE_LOGGING_VIDEO_MP4;
		m_guiHelper->dumpFramesToVideo(fileName);
	}

	virtual void stop()
	{
		m_guiHelper->dumpFramesToVideo(0);
	}
	virtual void logState(btScalar timeStep)
	{
		//dumping video frames happens in another thread
		//we could add some overlay of timestamp here, if needed/wanted
	}
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

	MinitaurStateLogger(int loggingUniqueId, const std::string& fileName, btMultiBody* minitaurMultiBody, btAlignedObjectArray<int>& motorIdList)
		:m_loggingTimeStamp(0),
		m_logFileHandle(0),
		m_minitaurMultiBody(minitaurMultiBody)
	{
        m_loggingUniqueId = loggingUniqueId;
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
			
			//btVector3 pos = m_minitaurMultiBody->getBasePos();

			MinitaurLogRecord logData;
			//'t', 'r', 'p', 'y', 'q0', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7', 'xd', 'mo'
			btScalar motorDir[8] = {1, 1, 1, 1, 1, 1, 1, 1};


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


struct b3VRControllerEvents
{
	b3VRControllerEvent m_vrEvents[MAX_VR_CONTROLLERS];
	
	b3VRControllerEvents()
	{
		init();
	}

	virtual ~b3VRControllerEvents()
	{
	}

	void init()
	{
		for (int i=0;i<MAX_VR_CONTROLLERS;i++)
		{
			m_vrEvents[i].m_deviceType = 0;
			m_vrEvents[i].m_numButtonEvents = 0;
			m_vrEvents[i].m_numMoveEvents = 0;
			for (int b=0;b<MAX_VR_BUTTONS;b++)
			{
				m_vrEvents[i].m_buttons[b] = 0;
			}
		}
	}

	void addNewVREvents(const struct b3VRControllerEvent* vrEvents, int numVREvents)
	{
	//update m_vrEvents
		for (int i=0;i<numVREvents;i++)
		{
			int controlledId = vrEvents[i].m_controllerId;
			if (vrEvents[i].m_numMoveEvents)
			{
				m_vrEvents[controlledId].m_analogAxis = vrEvents[i].m_analogAxis;
			}

			if (vrEvents[i].m_numMoveEvents+vrEvents[i].m_numButtonEvents)
			{
				m_vrEvents[controlledId].m_controllerId = vrEvents[i].m_controllerId;
				m_vrEvents[controlledId].m_deviceType = vrEvents[i].m_deviceType;

				m_vrEvents[controlledId].m_pos[0] = vrEvents[i].m_pos[0];
				m_vrEvents[controlledId].m_pos[1] = vrEvents[i].m_pos[1];
				m_vrEvents[controlledId].m_pos[2] = vrEvents[i].m_pos[2];
				m_vrEvents[controlledId].m_orn[0] = vrEvents[i].m_orn[0];
				m_vrEvents[controlledId].m_orn[1] = vrEvents[i].m_orn[1];
				m_vrEvents[controlledId].m_orn[2] = vrEvents[i].m_orn[2];
				m_vrEvents[controlledId].m_orn[3] = vrEvents[i].m_orn[3];
			}

			m_vrEvents[controlledId].m_numButtonEvents += vrEvents[i].m_numButtonEvents;
			m_vrEvents[controlledId].m_numMoveEvents += vrEvents[i].m_numMoveEvents;
			for (int b=0;b<MAX_VR_BUTTONS;b++)
			{
				m_vrEvents[controlledId].m_buttons[b] |= vrEvents[i].m_buttons[b];
				if (vrEvents[i].m_buttons[b] & eButtonIsDown)
				{
					m_vrEvents[controlledId].m_buttons[b] |= eButtonIsDown;
				} else
				{
					m_vrEvents[controlledId].m_buttons[b] &= ~eButtonIsDown;
				}
			}
		}
	};
};

struct VRControllerStateLogger : public InternalStateLogger
{
	b3VRControllerEvents m_vrEvents;
	int m_loggingTimeStamp;
	int m_deviceTypeFilter;
	std::string m_fileName;
	FILE* m_logFileHandle;
	std::string m_structTypes;

	VRControllerStateLogger(int loggingUniqueId, int deviceTypeFilter, const std::string& fileName)
		:m_loggingTimeStamp(0),
		m_deviceTypeFilter(deviceTypeFilter),
		m_fileName(fileName),
		m_logFileHandle(0)
	{
		m_loggingUniqueId = loggingUniqueId;
		m_loggingType = STATE_LOGGING_VR_CONTROLLERS;

		btAlignedObjectArray<std::string> structNames;
		structNames.push_back("stepCount");
		structNames.push_back("timeStamp");
		structNames.push_back("controllerId");
		structNames.push_back("numMoveEvents");
		structNames.push_back("m_numButtonEvents");
		structNames.push_back("posX");
        structNames.push_back("posY");
        structNames.push_back("posZ");
        structNames.push_back("oriX");
        structNames.push_back("oriY");
        structNames.push_back("oriZ");
        structNames.push_back("oriW");
		structNames.push_back("analogAxis");
		structNames.push_back("buttons0");
		structNames.push_back("buttons1");
		structNames.push_back("buttons2");
		structNames.push_back("buttons3");
		structNames.push_back("buttons4");
		structNames.push_back("buttons5");
		structNames.push_back("buttons6");
		structNames.push_back("deviceType");
		m_structTypes = "IfIIIffffffffIIIIIIII";

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
               
                int stepCount = m_loggingTimeStamp;
                float timeStamp = m_loggingTimeStamp*timeStep;

				for (int i=0;i<MAX_VR_CONTROLLERS;i++)
				{
					b3VRControllerEvent& event = m_vrEvents.m_vrEvents[i];
					if (m_deviceTypeFilter & event.m_deviceType)
					{
						if (event.m_numButtonEvents + event.m_numMoveEvents)
						{
							MinitaurLogRecord logData;

							//serverStatusOut.m_sendVREvents.m_controllerEvents[serverStatusOut.m_sendVREvents.m_numVRControllerEvents++] = event;
							//log the event
							logData.m_values.push_back(stepCount);
							logData.m_values.push_back(timeStamp);
							logData.m_values.push_back(event.m_controllerId);
							logData.m_values.push_back(event.m_numMoveEvents);
							logData.m_values.push_back(event.m_numButtonEvents);
							logData.m_values.push_back(event.m_pos[0]);
							logData.m_values.push_back(event.m_pos[1]);
							logData.m_values.push_back(event.m_pos[2]);
							logData.m_values.push_back(event.m_orn[0]);
							logData.m_values.push_back(event.m_orn[1]);
							logData.m_values.push_back(event.m_orn[2]);
							logData.m_values.push_back(event.m_orn[3]);
							logData.m_values.push_back(event.m_analogAxis);
							int packedButtons[7]={0,0,0,0,0,0,0};

							int packedButtonIndex = 0;
							int packedButtonShift = 0;
							//encode the 64 buttons into 7 int (3 bits each), each int stores 10 buttons
							for (int b=0;b<MAX_VR_BUTTONS;b++)
							{
								int buttonMask = event.m_buttons[b];
								buttonMask = buttonMask << (packedButtonShift*3);
								packedButtons[packedButtonIndex] |= buttonMask;
								packedButtonShift++;

								if (packedButtonShift>=10)
								{
									packedButtonShift=0;
									packedButtonIndex++;
									if (packedButtonIndex>=7)
									{
										btAssert(0);
										break;
									}
								}
							}

							for (int b=0;b<7;b++)
							{
								logData.m_values.push_back(packedButtons[b]);
							}
							logData.m_values.push_back(event.m_deviceType);
							appendMinitaurLogData(m_logFileHandle, m_structTypes, logData);

							event.m_numButtonEvents = 0;
							event.m_numMoveEvents = 0;
							for (int b=0;b<MAX_VR_BUTTONS;b++)
							{
								event.m_buttons[b] = 0;
							}
						}
					}
				}

				fflush(m_logFileHandle);
				m_loggingTimeStamp++;
		}
	}
};

struct GenericRobotStateLogger : public InternalStateLogger
{
    float m_loggingTimeStamp;
    std::string m_fileName;
    FILE* m_logFileHandle;
    std::string m_structTypes;
    const btMultiBodyDynamicsWorld* m_dynamicsWorld;
    btAlignedObjectArray<int> m_bodyIdList;
    bool m_filterObjectUniqueId;
    int m_maxLogDof;
	int m_logFlags;

    GenericRobotStateLogger(int loggingUniqueId, const std::string& fileName, const btMultiBodyDynamicsWorld* dynamicsWorld, int maxLogDof, int logFlags)
    :m_loggingTimeStamp(0),
    m_logFileHandle(0),
    m_dynamicsWorld(dynamicsWorld),
    m_filterObjectUniqueId(false),
	m_maxLogDof(maxLogDof),
	m_logFlags(logFlags)
    {
        m_loggingUniqueId = loggingUniqueId;
        m_loggingType = STATE_LOGGING_GENERIC_ROBOT;
        
        btAlignedObjectArray<std::string> structNames;
        structNames.push_back("stepCount");
        structNames.push_back("timeStamp");
        structNames.push_back("objectId");
        structNames.push_back("posX");
        structNames.push_back("posY");
        structNames.push_back("posZ");
        structNames.push_back("oriX");
        structNames.push_back("oriY");
        structNames.push_back("oriZ");
        structNames.push_back("oriW");
        structNames.push_back("velX");
        structNames.push_back("velY");
        structNames.push_back("velZ");
        structNames.push_back("omegaX");
        structNames.push_back("omegaY");
        structNames.push_back("omegaZ");
        structNames.push_back("qNum");

		m_structTypes = "IfifffffffffffffI";

		for (int i=0;i<m_maxLogDof;i++)
		{
			m_structTypes.append("f");
			char jointName[256];
			sprintf(jointName,"q%d",i);
			structNames.push_back(jointName);
		}

		for (int i=0;i<m_maxLogDof;i++)
		{
			m_structTypes.append("f");
			char jointName[256];
			sprintf(jointName,"u%d",i);
			structNames.push_back(jointName);
		}

		if (m_logFlags & STATE_LOG_JOINT_TORQUES)
		{
			for (int i=0;i<m_maxLogDof;i++)
			{
				m_structTypes.append("f");
				char jointName[256];
				sprintf(jointName,"t%d",i);
				structNames.push_back(jointName);
			}
		}

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
            for (int i=0;i<m_dynamicsWorld->getNumMultibodies();i++)
            {
                const btMultiBody* mb = m_dynamicsWorld->getMultiBody(i);
                int objectUniqueId = mb->getUserIndex2();
                if (m_filterObjectUniqueId && m_bodyIdList.findLinearSearch2(objectUniqueId) < 0)
                {
                    continue;
                }
                
                MinitaurLogRecord logData;
                int stepCount = m_loggingTimeStamp;
                float timeStamp = m_loggingTimeStamp*m_dynamicsWorld->getSolverInfo().m_timeStep;
                logData.m_values.push_back(stepCount);
                logData.m_values.push_back(timeStamp);
                
                btVector3 pos = mb->getBasePos();
                btQuaternion ori = mb->getWorldToBaseRot().inverse();
                btVector3 vel = mb->getBaseVel();
                btVector3 omega = mb->getBaseOmega();
                
                float posX = pos[0];
                float posY = pos[1];
                float posZ = pos[2];
                float oriX = ori.x();
                float oriY = ori.y();
                float oriZ = ori.z();
                float oriW = ori.w();
                float velX = vel[0];
                float velY = vel[1];
                float velZ = vel[2];
                float omegaX = omega[0];
                float omegaY = omega[1];
                float omegaZ = omega[2];
                
                logData.m_values.push_back(objectUniqueId);
                logData.m_values.push_back(posX);
                logData.m_values.push_back(posY);
                logData.m_values.push_back(posZ);
                logData.m_values.push_back(oriX);
                logData.m_values.push_back(oriY);
                logData.m_values.push_back(oriZ);
                logData.m_values.push_back(oriW);
                logData.m_values.push_back(velX);
                logData.m_values.push_back(velY);
                logData.m_values.push_back(velZ);
                logData.m_values.push_back(omegaX);
                logData.m_values.push_back(omegaY);
                logData.m_values.push_back(omegaZ);
                
                int numDofs = mb->getNumDofs();
                logData.m_values.push_back(numDofs);
                int numJoints = mb->getNumLinks();
                
                for (int j = 0; j < numJoints; ++j)
                {
                    if (mb->getLink(j).m_jointType == 0 || mb->getLink(j).m_jointType == 1)
                    {
                        float q = mb->getJointPos(j);
                        logData.m_values.push_back(q);
                    }
                }
                for (int j = numDofs; j < m_maxLogDof; ++j)
                {
                    float q = 0.0;
                    logData.m_values.push_back(q);
                }
                
                for (int j = 0; j < numJoints; ++j)
                {
                    if (mb->getLink(j).m_jointType == 0 || mb->getLink(j).m_jointType == 1)
                    {
                        float v = mb->getJointVel(j);
                        logData.m_values.push_back(v);
                    }
                }
                for (int j = numDofs; j < m_maxLogDof; ++j)
                {
                    float v = 0.0;
                    logData.m_values.push_back(v);
                }

				
				if (m_logFlags & STATE_LOG_JOINT_TORQUES)
				{
					for (int j = 0; j < numJoints; ++j)
					{
						if (mb->getLink(j).m_jointType == 0 || mb->getLink(j).m_jointType == 1)
						{
							float jointTorque = 0;
							if (m_logFlags & STATE_LOG_JOINT_MOTOR_TORQUES)
							{
								btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(j).m_userPtr;
								if (motor)
								{
									jointTorque += motor->getAppliedImpulse(0)/timeStep;
								}
							}
							if (m_logFlags & STATE_LOG_JOINT_USER_TORQUES)
							{
								if (mb->getLink(j).m_jointType == 0 || mb->getLink(j).m_jointType == 1)
								{
									jointTorque += mb->getJointTorque(j);//these are the 'user' applied external torques
								}
							}
							logData.m_values.push_back(jointTorque);
						}

					}
					for (int j = numDofs; j < m_maxLogDof; ++j)
					{
						float u = 0.0;
						logData.m_values.push_back(u);
					}
				}
                                
                //at the moment, appendMinitaurLogData will directly write to disk (potential delay)
                //better to fill a huge memory buffer and once in a while write it to disk
                appendMinitaurLogData(m_logFileHandle, m_structTypes, logData);
                fflush(m_logFileHandle);
            }
            
            m_loggingTimeStamp++;
        }
    }
};
struct ContactPointsStateLogger : public InternalStateLogger
{
	int m_loggingTimeStamp;
	
	std::string m_fileName;
	FILE* m_logFileHandle;
	std::string m_structTypes;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	bool m_filterLinkA;
	bool m_filterLinkB;
	int m_linkIndexA;
	int m_linkIndexB;
	int m_bodyUniqueIdA;
	int m_bodyUniqueIdB;
	
	ContactPointsStateLogger(int loggingUniqueId, const std::string& fileName, btMultiBodyDynamicsWorld* dynamicsWorld)
	:m_loggingTimeStamp(0),
	m_fileName(fileName),
	m_logFileHandle(0),
	m_dynamicsWorld(dynamicsWorld),
	m_filterLinkA(false),
	m_filterLinkB(false),
	m_linkIndexA(-2),
	m_linkIndexB(-2),
	m_bodyUniqueIdA(-1),
	m_bodyUniqueIdB(-1)
	{
		m_loggingUniqueId = loggingUniqueId;
		m_loggingType = STATE_LOGGING_CONTACT_POINTS;
		
		btAlignedObjectArray<std::string> structNames;
		structNames.push_back("stepCount");
		structNames.push_back("timeStamp");
		structNames.push_back("contactFlag");
		structNames.push_back("bodyUniqueIdA");
		structNames.push_back("bodyUniqueIdB");
		structNames.push_back("linkIndexA");
		structNames.push_back("linkIndexB");
		structNames.push_back("positionOnAX");
		structNames.push_back("positionOnAY");
		structNames.push_back("positionOnAZ");
		structNames.push_back("positionOnBX");
		structNames.push_back("positionOnBY");
		structNames.push_back("positionOnBZ");
		structNames.push_back("contactNormalOnBX");
		structNames.push_back("contactNormalOnBY");
		structNames.push_back("contactNormalOnBZ");
		structNames.push_back("contactDistance");
		structNames.push_back("normalForce");
		m_structTypes = "IfIiiiifffffffffff";
		
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
			int numContactManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
			for (int i = 0; i < numContactManifolds; i++)
			{
				const btPersistentManifold* manifold = m_dynamicsWorld->getDispatcher()->getInternalManifoldPointer()[i];
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
					if (m_filterLinkB && (m_linkIndexB != linkIndexB))
					{
						continue;
					}
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
					if (m_filterLinkA && (m_linkIndexA != linkIndexA))
					{
						continue;
					}
				}
				
				btAssert(bodyA || mblA);
				
				//apply the filter, if the user provides it
				if (m_bodyUniqueIdA >= 0)
				{
					if ((m_bodyUniqueIdA != objectIndexA) &&
						(m_bodyUniqueIdA != objectIndexB))
						continue;
				}
				
				//apply the second object filter, if the user provides it
				if (m_bodyUniqueIdB >= 0)
				{
					if ((m_bodyUniqueIdB != objectIndexA) &&
						(m_bodyUniqueIdB != objectIndexB))
						continue;
				}
				
				for (int p = 0; p < manifold->getNumContacts(); p++)
				{
					MinitaurLogRecord logData;
					int stepCount = m_loggingTimeStamp;
					float timeStamp = m_loggingTimeStamp*timeStep;
					logData.m_values.push_back(stepCount);
					logData.m_values.push_back(timeStamp);
					
					const btManifoldPoint& srcPt = manifold->getContactPoint(p);
					
					logData.m_values.push_back(0); // reserved contact flag
					logData.m_values.push_back(objectIndexA);
					logData.m_values.push_back(objectIndexB);
					logData.m_values.push_back(linkIndexA);
					logData.m_values.push_back(linkIndexB);
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnA()[0]));
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnA()[1]));
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnA()[2]));
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnB()[0]));
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnB()[1]));
					logData.m_values.push_back((float)(srcPt.getPositionWorldOnB()[2]));
					logData.m_values.push_back((float)(srcPt.m_normalWorldOnB[0]));
					logData.m_values.push_back((float)(srcPt.m_normalWorldOnB[1]));
					logData.m_values.push_back((float)(srcPt.m_normalWorldOnB[2]));
					logData.m_values.push_back((float)(srcPt.getDistance()));
					logData.m_values.push_back((float)(srcPt.getAppliedImpulse() / timeStep));
					
					appendMinitaurLogData(m_logFileHandle, m_structTypes, logData);
					fflush(m_logFileHandle);
				}
			}
			m_loggingTimeStamp++;
		}
	}
};

struct PhysicsServerCommandProcessorInternalData
{
	///handle management
	b3ResizablePool< InternalTextureHandle > m_textureHandles;
	b3ResizablePool< InternalBodyHandle > m_bodyHandles;
	b3ResizablePool<InternalCollisionShapeHandle> m_userCollisionShapeHandles;

	b3PluginManager m_pluginManager;

	bool m_allowRealTimeSimulation;
	

	b3VRControllerEvents m_vrControllerEvents;


	btAlignedObjectArray<b3KeyboardEvent> m_keyboardEvents;
	btAlignedObjectArray<b3MouseEvent> m_mouseEvents;

	CommandLogger* m_commandLogger;
	CommandLogPlayback* m_logPlayback;


	btScalar m_physicsDeltaTime;
    btScalar m_numSimulationSubSteps;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_multiBodyJointFeedbacks;
	b3HashMap<btHashPtr, btInverseDynamics::MultiBodyTree*> m_inverseDynamicsBodies;
	b3HashMap<btHashPtr, IKTrajectoryHelper*> m_inverseKinematicsHelpers;
	
	int m_userConstraintUIDGenerator;
	b3HashMap<btHashInt, InteralUserConstraintData> m_userConstraints;

	b3AlignedObjectArray<SaveWorldObjectData> m_saveWorldBodyData;


	btAlignedObjectArray<btBulletWorldImporter*> m_worldImporters;
	btAlignedObjectArray<UrdfLinkNameMapUtil*> m_urdfLinkNameMapper;
	btAlignedObjectArray<std::string*> m_strings;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btAlignedObjectArray<btStridingMeshInterface*> m_meshInterfaces;

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
	int m_profileTimingLoggingUid;
	std::string m_profileTimingFileName;

	struct GUIHelperInterface* m_guiHelper;
	int m_sharedMemoryKey;

	bool m_verboseOutput;


	//data for picking objects
	class btRigidBody*	m_pickedBody;
    int m_savedActivationState;
	class btTypedConstraint* m_pickedConstraint;
	class btMultiBodyPoint2Point*		m_pickingMultiBodyPoint2Point;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	bool m_prevCanSleep;
	TinyRendererVisualShapeConverter  m_visualConverter;
#ifdef B3_ENABLE_TINY_AUDIO
	b3SoundEngine m_soundEngine;
#endif

	b3HashMap<b3HashString,  char*> m_profileEvents;

	PhysicsServerCommandProcessorInternalData(PhysicsCommandProcessorInterface* proc)
		:m_pluginManager(proc),
		m_allowRealTimeSimulation(false),
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
		m_profileTimingLoggingUid(-1),
		m_guiHelper(0),
		m_sharedMemoryKey(SHARED_MEMORY_KEY),
		m_verboseOutput(false),
		m_pickedBody(0),
		m_pickedConstraint(0),
		m_pickingMultiBodyPoint2Point(0)
	{

		

		{
			//test to statically link a plugin
			//#include "plugins/testPlugin/testplugin.h"
			//register static plugins:
			//m_pluginManager.registerStaticLinkedPlugin("path", initPlugin, exitPlugin, executePluginCommand);
		}

		m_vrControllerEvents.init();

		m_bodyHandles.exitHandles();
		m_bodyHandles.initHandles();
		m_userCollisionShapeHandles.exitHandles();
		m_userCollisionShapeHandles.initHandles();

#if 0
		btAlignedObjectArray<int> bla;

		for (int i=0;i<1024;i++)
		{
			int handle = allocHandle();
			bla.push_back(handle);
			InternalBodyHandle* body = getHandle(handle);
			InternalBodyData* body2 = body;
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
			InternalBodyData* body2 = body;
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
			InternalBodyData* body2 = body;
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
	:m_data(0)
{
	m_data = new PhysicsServerCommandProcessorInternalData(this);

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
	for (int i=0;i<m_data->m_profileEvents.size();i++)
	{
		char* event = *m_data->m_profileEvents.getAtIndex(i);
		delete[] event;
	}
	delete m_data;
}


void preTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
	PhysicsServerCommandProcessor* proc = (PhysicsServerCommandProcessor*) world->getWorldUserInfo();
	bool isPreTick = true;
	proc->tickPlugins(timeStep, isPreTick);
}

void logCallback(btDynamicsWorld *world, btScalar timeStep)
{
	//handle the logging and playing sounds
	PhysicsServerCommandProcessor* proc = (PhysicsServerCommandProcessor*) world->getWorldUserInfo();
	proc->processCollisionForces(timeStep);
	proc->logObjectStates(timeStep);
	
	bool isPreTick = false;
	proc->tickPlugins(timeStep, isPreTick);

}

bool MyContactAddedCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
{
	return true;
}




bool MyContactDestroyedCallback(void* userPersistentData)
{
	//printf("destroyed\n");
	return false;
}

bool MyContactProcessedCallback(btManifoldPoint& cp,void* body0,void* body1)
{
	//printf("processed\n");
	return false;

}
void MyContactStartedCallback(btPersistentManifold* const &manifold)
{
	//printf("started\n");
}
void MyContactEndedCallback(btPersistentManifold* const &manifold)
{
//	printf("ended\n");
}



void PhysicsServerCommandProcessor::processCollisionForces(btScalar timeStep)
{
#ifdef B3_ENABLE_TINY_AUDIO
	//this is experimental at the moment: impulse thresholds, sound parameters will be exposed in C-API/pybullet.
	//audio will go into a wav file, as well as real-time output to speakers/headphones using RtAudio/DAC.

	int numContactManifolds =  m_data->m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numContactManifolds; i++)
	{
		const btPersistentManifold* manifold = m_data->m_dynamicsWorld->getDispatcher()->getInternalManifoldPointer()[i];

		bool objHasSound[2];
		objHasSound[0] = (0!=(manifold->getBody0()->getCollisionFlags() & btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER));
		objHasSound[1] = (0!=(manifold->getBody1()->getCollisionFlags() & btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER));
		const btCollisionObject* colObjs[2] = {manifold->getBody0(),manifold->getBody1()};

		for (int ob = 0;ob<2;ob++)
		{
			if (objHasSound[ob])
			{
				int uid0 = -1;
				int linkIndex = -2;

				const btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(colObjs[ob]);
				if (mblB && mblB->m_multiBody)
				{
					linkIndex = mblB->m_link;
					uid0 = mblB->m_multiBody->getUserIndex2();
				}
				const btRigidBody* bodyB = btRigidBody::upcast(colObjs[ob]);
				if (bodyB)
				{
					uid0 = bodyB->getUserIndex2();
					linkIndex = -1;
				}

				if ((uid0<0)||(linkIndex<-1))
					continue;

				InternalBodyHandle* bodyHandle0 = m_data->m_bodyHandles.getHandle(uid0);
				SDFAudioSource* audioSrc = bodyHandle0->m_audioSources[linkIndex];
				if (audioSrc==0)
					continue;

				for (int p=0;p<manifold->getNumContacts();p++)
				{
					double imp = manifold->getContactPoint(p).getAppliedImpulse();
						//printf ("manifold %d, contact %d, lifeTime:%d, appliedImpulse:%f\n",i,p, manifold->getContactPoint(p).getLifeTime(),imp);

					if (imp>audioSrc->m_collisionForceThreshold && manifold->getContactPoint(p).getLifeTime()==1)
					{
						int soundSourceIndex = m_data->m_soundEngine.getAvailableSoundSource();
						if (soundSourceIndex>=0)
						{
							b3SoundMessage msg;
							msg.m_attackRate = audioSrc->m_attackRate;
							msg.m_decayRate = audioSrc->m_decayRate;
							msg.m_sustainLevel = audioSrc->m_sustainLevel;
							msg.m_releaseRate = audioSrc->m_releaseRate;
							msg.m_amplitude = audioSrc->m_gain;
							msg.m_frequency = audioSrc->m_pitch;
							msg.m_type = B3_SOUND_SOURCE_WAV_FILE;
							msg.m_wavId = audioSrc->m_userIndex;
							msg.m_autoKeyOff = true;
							m_data->m_soundEngine.startSound(soundSourceIndex,msg);
						}
					}
				}
			}
		}
	}
#endif//B3_ENABLE_TINY_AUDIO
}

void PhysicsServerCommandProcessor::tickPlugins(btScalar timeStep, bool isPreTick)
{
	m_data->m_pluginManager.tickPlugins(timeStep, isPreTick);
}


void PhysicsServerCommandProcessor::logObjectStates(btScalar timeStep)
{
	for (int i=0;i<m_data->m_stateLoggers.size();i++)
	{
		m_data->m_stateLoggers[i]->logState(timeStep);
	}

}

struct ProgrammaticUrdfInterface : public URDFImporterInterface
{
	int m_bodyUniqueId;

	const b3CreateMultiBodyArgs& m_createBodyArgs;
	mutable b3AlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
	PhysicsServerCommandProcessorInternalData* m_data;

	ProgrammaticUrdfInterface(const b3CreateMultiBodyArgs& bodyArgs, PhysicsServerCommandProcessorInternalData* data)
		:m_bodyUniqueId(-1),
		m_createBodyArgs(bodyArgs),
		m_data(data)
	{

	}

	virtual ~ProgrammaticUrdfInterface()
	{

	}

	 virtual bool loadURDF(const char* fileName, bool forceFixedBase = false)
	 {
		 b3Assert(0);
		 return false;
	 }

    virtual const char* getPathPrefix()
	{
		return "";
	}
    
    ///return >=0 for the root link index, -1 if there is no root link
    virtual int getRootLinkIndex() const
	{
		return m_createBodyArgs.m_baseLinkIndex;
	}
    
    ///pure virtual interfaces, precondition is a valid linkIndex (you can assert/terminate if the linkIndex is out of range)
    virtual std::string getLinkName(int linkIndex) const
	{
		std::string linkName = "link";
		char numstr[21]; // enough to hold all numbers up to 64-bits
		sprintf(numstr, "%d", linkIndex);
		linkName = linkName + numstr;
		return linkName;
	}

	//various derived class in internal source code break with new pure virtual methods, so provide some default implementation
	virtual std::string getBodyName() const
	{
		return m_createBodyArgs.m_bodyName;
	}
    
	/// optional method to provide the link color. return true if the color is available and copied into colorRGBA, return false otherwise
	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const 
	{ 
		b3Assert(0);
		return false;
	}

	virtual bool getLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const 
	{ 
		return false;
	}

	virtual int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const 
	{ 
		return 0;
	}
	///this API will likely change, don't override it!
	virtual bool getLinkContactInfo(int linkIndex, URDFLinkContactInfo& contactInfo ) const  
	{ 

		return false;
	}
    
	virtual bool getLinkAudioSource(int linkIndex, SDFAudioSource& audioSource) const 
	{
		b3Assert(0);
		return false;
	}

    virtual std::string getJointName(int linkIndex) const
	{
		std::string jointName = "joint";
		char numstr[21]; // enough to hold all numbers up to 64-bits
		sprintf(numstr, "%d", linkIndex);
		jointName = jointName + numstr;
		return jointName;
	}
	
    //fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
    virtual void  getMassAndInertia(int urdfLinkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
	{
		if (urdfLinkIndex>=0 && urdfLinkIndex < m_createBodyArgs.m_numLinks)
		{
			mass = m_createBodyArgs.m_linkMasses[urdfLinkIndex];
			localInertiaDiagonal.setValue(
				m_createBodyArgs.m_linkInertias[urdfLinkIndex*3+0],
				m_createBodyArgs.m_linkInertias[urdfLinkIndex*3+1],
				m_createBodyArgs.m_linkInertias[urdfLinkIndex*3+2]);
			inertialFrame.setOrigin(btVector3(
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex*3+0],
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex*3+1],
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex*3+2]));
			inertialFrame.setRotation(btQuaternion(
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex*4+0],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex*4+1],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex*4+2],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex*4+3]));
		} else
		{		
			mass = 0;
			localInertiaDiagonal.setValue(0,0,0);
			inertialFrame.setIdentity();
		}
	}
    
    ///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
    virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const
	{
		for (int i=0;i<m_createBodyArgs.m_numLinks;i++)
		{
			if (m_createBodyArgs.m_linkParentIndices[i] == urdfLinkIndex)
			{
				childLinkIndices.push_back(i);
			}
		}
		
	}
    
    virtual bool getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
	{
		return false;
	};

	virtual bool getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const 
	{
		bool isValid = false;

		int jointTypeOrg = m_createBodyArgs.m_linkJointTypes[urdfLinkIndex];

		switch (jointTypeOrg)
		{
		case eRevoluteType:
		{
			isValid = true;
			jointType = URDFRevoluteJoint;
			break;
		}
		case	ePrismaticType:
		{
			isValid = true;
			jointType = URDFPrismaticJoint;
			break;
		}
		case	eFixedType:
		{
			isValid = true;
			jointType = URDFFixedJoint;
			break;
		}
		//case	eSphericalType:
		//case	ePlanarType:
		//case	eFixedType:
		//case ePoint2PointType:
		//case eGearType:
		default:
		{
		}
		};

		if (isValid)
		{
			//backwards compatibility for custom file importers
			jointMaxForce = 0;
			jointMaxVelocity = 0;
			jointFriction = 0;
			jointDamping = 0;
			jointLowerLimit = 1;
			jointUpperLimit = -1;

			parent2joint.setOrigin(btVector3(
				m_createBodyArgs.m_linkPositions[urdfLinkIndex*3+0],
				m_createBodyArgs.m_linkPositions[urdfLinkIndex*3+1],
				m_createBodyArgs.m_linkPositions[urdfLinkIndex*3+2]));
			parent2joint.setRotation(btQuaternion(
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex*4+0],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex*4+1],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex*4+2],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex*4+3]
			));

		
			linkTransformInWorld.setIdentity();

			jointAxisInJointSpace.setValue(
				m_createBodyArgs.m_linkJointAxis[3*urdfLinkIndex+0],
				m_createBodyArgs.m_linkJointAxis[3*urdfLinkIndex+1],
				m_createBodyArgs.m_linkJointAxis[3*urdfLinkIndex+2]);

		
		}
		return isValid;
	};
    
    virtual bool getRootTransformInWorld(btTransform& rootTransformInWorld) const
	{
		int baseLinkIndex = m_createBodyArgs.m_baseLinkIndex;

		rootTransformInWorld.setOrigin(btVector3(
			m_createBodyArgs.m_linkPositions[baseLinkIndex*3+0],
			m_createBodyArgs.m_linkPositions[baseLinkIndex*3+1],
			m_createBodyArgs.m_linkPositions[baseLinkIndex*3+2]));
		rootTransformInWorld.setRotation(btQuaternion(
			m_createBodyArgs.m_linkOrientations[baseLinkIndex*4+0],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex*4+1],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex*4+2],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex*4+3]));
		return true;
	}
	virtual void setRootTransformInWorld(const btTransform& rootTransformInWorld)
	{
		b3Assert(0);
	}

	///quick hack: need to rethink the API/dependencies of this
    virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& inertialFrame) const
	{
		return -1;
	}
    
    virtual void convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& localInertiaFrame, class btCollisionObject* colObj, int bodyUniqueId) const  
	{
		//if there is a visual, use it, otherwise convert collision shape back into UrdfCollision...

		UrdfModel model;// = m_data->m_urdfParser.getModel();
		UrdfLink link;
		int colShapeUniqueId = m_createBodyArgs.m_linkCollisionShapeUniqueIds[urdfIndex];
		if (colShapeUniqueId>=0)
		{
			InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(colShapeUniqueId);
			if (handle)
			{
				for (int i=0;i<handle->m_urdfCollisionObjects.size();i++)
				{
					link.m_collisionArray.push_back(handle->m_urdfCollisionObjects[i]);
				}
			}
		}
		//UrdfVisual vis;
		//link.m_visualArray.push_back(vis);
		//UrdfLink*const* linkPtr = model.m_links.getAtIndex(urdfIndex);
		m_data->m_visualConverter.convertVisualShapes(linkIndex,pathPrefix,localInertiaFrame, &link, &model, colObj, bodyUniqueId);
	}
    virtual void setBodyUniqueId(int bodyId) 
	{
		m_bodyUniqueId = bodyId;
	}
    virtual int getBodyUniqueId() const 
	{
		return m_bodyUniqueId;
	}

   //default implementation for backward compatibility 
	virtual class btCompoundShape* convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
	{
		btCompoundShape* compound = new btCompoundShape();

		int colShapeUniqueId = m_createBodyArgs.m_linkCollisionShapeUniqueIds[linkIndex];
		if (colShapeUniqueId>=0)
		{
			InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(colShapeUniqueId);
			if (handle)
			{
				btTransform childTrans;
				childTrans.setIdentity();				
				compound->addChildShape(localInertiaFrame.inverse()*childTrans,handle->m_collisionShape);
			}
		}
		m_allocatedCollisionShapes.push_back(compound);
		return compound;
	}
	
	virtual int getNumAllocatedCollisionShapes() const 
	{ 
		return m_allocatedCollisionShapes.size();
	}
    
	virtual class btCollisionShape* getAllocatedCollisionShape(int index) 
	{
		return m_allocatedCollisionShapes[index];
	}
	virtual int getNumModels() const 
	{
		return 1;
	}
    virtual void activateModel(int /*modelIndex*/) 
	{
	}
};


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

	m_data->m_dynamicsWorld->getSolverInfo().m_frictionERP = 0.2;//need to check if there are artifacts with frictionERP
	m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = 0.00001;
	m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
	m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = 1e-7;
//	m_data->m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 2;
	//todo: islands/constraints are buggy in btMultiBodyDynamicsWorld! (performance + see slipping grasp)

	if (m_data->m_guiHelper)
	{
		m_data->m_guiHelper->createPhysicsDebugDrawer(m_data->m_dynamicsWorld);
	}
	bool isPreTick=false;
	m_data->m_dynamicsWorld->setInternalTickCallback(logCallback,this,isPreTick);
	isPreTick = true;
	m_data->m_dynamicsWorld->setInternalTickCallback(preTickCallback,this,isPreTick);


#ifdef B3_ENABLE_TINY_AUDIO
	m_data->m_soundEngine.init(16,true);

//we don't use those callbacks (yet), experimental
//	gContactAddedCallback = MyContactAddedCallback;
//	gContactDestroyedCallback = MyContactDestroyedCallback;
//	gContactProcessedCallback = MyContactProcessedCallback;
//	gContactStartedCallback = MyContactStartedCallback;
//	gContactEndedCallback = MyContactEndedCallback;
#endif
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
#ifdef B3_ENABLE_TINY_AUDIO
	m_data->m_soundEngine.exit();
	//gContactDestroyedCallback = 0;
	//gContactProcessedCallback = 0;
	//gContactStartedCallback = 0;
	//gContactEndedCallback = 0;
#endif
	
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
	for (int j=0;j<m_data->m_meshInterfaces.size();j++)
	{
		delete m_data->m_meshInterfaces[j];
	}
	m_data->m_meshInterfaces.clear();
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
	m_data->m_userConstraintUIDGenerator = 1;
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
        int bodyUniqueId = m_data->m_bodyHandles.allocHandle();

        InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
			
		sd.m_bodyUniqueIds.push_back(bodyUniqueId);

        u2b.setBodyUniqueId(bodyUniqueId);
        {
            btScalar mass = 0;
            bodyHandle->m_rootLocalInertialFrame.setIdentity();
			bodyHandle->m_bodyName = u2b.getBodyName();
            btVector3 localInertiaDiagonal(0,0,0);
            int urdfLinkIndex = u2b.getRootLinkIndex();
            u2b.getMassAndInertia(urdfLinkIndex, mass,localInertiaDiagonal,bodyHandle->m_rootLocalInertialFrame);
        }



        //todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
        //int rootLinkIndex = u2b.getRootLinkIndex();
        //b3Printf("urdf root link index = %d\n",rootLinkIndex);
        MyMultiBodyCreator creation(m_data->m_guiHelper);

        u2b.getRootTransformInWorld(rootTrans);
		//CUF_RESERVED is a temporary flag, for backward compatibility purposes
		flags |= CUF_RESERVED;
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

#ifdef B3_ENABLE_TINY_AUDIO
			{
				SDFAudioSource audioSource;
				int urdfRootLink = u2b.getRootLinkIndex();//LinkIndex = creation.m_mb2urdfLink[-1];
				if (u2b.getLinkAudioSource(urdfRootLink,audioSource))
				{
					int flags = mb->getBaseCollider()->getCollisionFlags();
					mb->getBaseCollider()->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
					audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
					if (audioSource.m_userIndex>=0)
					{
						bodyHandle->m_audioSources.insert(-1, audioSource);
					}
				}
			}
#endif
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

#ifdef B3_ENABLE_TINY_AUDIO
				   {
						SDFAudioSource audioSource;
						int urdfLinkIndex = creation.m_mb2urdfLink[link];
						if (u2b.getLinkAudioSource(urdfLinkIndex,audioSource))
						{
							int flags = mb->getLink(link).m_collider->getCollisionFlags();
							mb->getLink(i).m_collider->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
							audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
							if (audioSource.m_userIndex>=0)
							{
								bodyHandle->m_audioSources.insert(link, audioSource);
							}
						}
					}
#endif
            }
			std::string* baseName = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
			m_data->m_strings.push_back(baseName);
			mb->setBaseName(baseName->c_str());
		} else
		{
			//b3Warning("No multibody loaded from URDF. Could add btRigidBody+btTypedConstraint solution later.");
            bodyHandle->m_rigidBody = rb;
			rb->setUserIndex2(bodyUniqueId);
			m_data->m_sdfRecentLoadedBodies.push_back(bodyUniqueId);

			std::string* baseName = new std::string(u2b.getLinkName(u2b.getRootLinkIndex()));
			m_data->m_strings.push_back(baseName);
			bodyHandle->m_bodyName = *baseName;

			int numJoints = creation.getNum6DofConstraints();
			bodyHandle->m_rigidBodyJoints.reserve(numJoints);
			bodyHandle->m_rigidBodyJointNames.reserve(numJoints);
			bodyHandle->m_rigidBodyLinkNames.reserve(numJoints);
			for (int i=0;i<numJoints;i++)
			{
				int urdfLinkIndex = creation.m_mb2urdfLink[i];

				btGeneric6DofSpring2Constraint* con = creation.get6DofConstraint(i);
				
				std::string* linkName = new std::string(u2b.getLinkName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(linkName);

				std::string* jointName = new std::string(u2b.getJointName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(jointName);

				bodyHandle->m_rigidBodyJointNames.push_back(*jointName);
				bodyHandle->m_rigidBodyLinkNames.push_back(*linkName);

				bodyHandle->m_rigidBodyJoints.push_back(con);
			}
		}

    }

	for (int i=0;i<u2b.getNumAllocatedMeshInterfaces();i++)
	{
		m_data->m_meshInterfaces.push_back(u2b.getAllocatedMeshInterface(i));
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

    BulletMJCFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter);

	bool useFixedBase = false;
	MyMJCFLogger2 logger;
    bool loadOk =  u2b.loadMJCF(fileName, &logger, useFixedBase);
    if (loadOk)
	{
		processImportedObjects(fileName,bufferServerToClient,bufferSizeInBytes,useMultiBody,flags, u2b);
	}
	return loadOk;
}

bool PhysicsServerCommandProcessor::loadSdf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags, btScalar globalScaling)
{
    btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadSdf: No valid m_dynamicsWorld");
		return false;
	}

	m_data->m_sdfRecentLoadedBodies.clear();

    BulletURDFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter, globalScaling);

	bool forceFixedBase = false;
	bool loadOk =u2b.loadSDF(fileName,forceFixedBase);
	
	if (loadOk)
	{
		processImportedObjects(fileName,bufferServerToClient,bufferSizeInBytes,useMultiBody,flags, u2b);
	}
    return loadOk;
}




bool PhysicsServerCommandProcessor::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes, int flags, btScalar globalScaling)
{
	m_data->m_sdfRecentLoadedBodies.clear();
	*bodyUniqueIdPtr = -1;

	BT_PROFILE("loadURDF");
	btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadUrdf: No valid m_dynamicsWorld");
		return false;
	}



    BulletURDFImporter u2b(m_data->m_guiHelper, &m_data->m_visualConverter, globalScaling);

    bool loadOk =  u2b.loadURDF(fileName, useFixedBase);


    if (loadOk)
    {

#if 1
		btTransform rootTrans;
		rootTrans.setOrigin(pos);
		rootTrans.setRotation(orn);
		u2b.setRootTransformInWorld(rootTrans);
		bool ok = processImportedObjects(fileName, bufferServerToClient, bufferSizeInBytes,  useMultiBody, flags, u2b);
		if (ok)
		{
			if (m_data->m_sdfRecentLoadedBodies.size()==1)
			{
				*bodyUniqueIdPtr = m_data->m_sdfRecentLoadedBodies[0];

			}
			m_data->m_sdfRecentLoadedBodies.clear();
		}
		return ok;
#else

		//get a body index
		int bodyUniqueId = m_data->m_bodyHandles.allocHandle();
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
		InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
		


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

		flags |= URDF_ORDER_TYPED_CONSTRAINT;

        ConvertURDF2Bullet(u2b,creation, tr,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix(),flags);

        for (int i=0;i<u2b.getNumAllocatedCollisionShapes();i++)
        {
            btCollisionShape* shape =u2b.getAllocatedCollisionShape(i);
            m_data->m_collisionShapes.push_back(shape);
        }

        btMultiBody* mb = creation.getBulletMultiBody();
        btRigidBody* rb = creation.getRigidBody();
		
		bodyHandle->m_bodyName = u2b.getBodyName();

		if (useMultiBody)
		{


			if (mb)
			{
				mb->setUserIndex2(bodyUniqueId);
				bodyHandle->m_multiBody = mb;

				if (flags & URDF_USE_SELF_COLLISION)
				{
					mb->setHasSelfCollision(true);
				}
				createJointMotors(mb);

#ifdef B3_ENABLE_TINY_AUDIO
				{
					SDFAudioSource audioSource;
					int urdfRootLink = u2b.getRootLinkIndex();//LinkIndex = creation.m_mb2urdfLink[-1];
					if (u2b.getLinkAudioSource(urdfRootLink,audioSource))
					{
						int flags = mb->getBaseCollider()->getCollisionFlags();
						mb->getBaseCollider()->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
						audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
						if (audioSource.m_userIndex>=0)
						{
							bodyHandle->m_audioSources.insert(-1, audioSource);
						}
					}
				}
#endif

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
					int link=i;
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
#ifdef B3_ENABLE_TINY_AUDIO
				   {
						SDFAudioSource audioSource;
						int urdfLinkIndex = creation.m_mb2urdfLink[link];
						if (u2b.getLinkAudioSource(urdfLinkIndex,audioSource))
						{
							int flags = mb->getLink(link).m_collider->getCollisionFlags();
							mb->getLink(i).m_collider->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
							audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
							if (audioSource.m_userIndex>=0)
							{
								bodyHandle->m_audioSources.insert(link, audioSource);
							}
						}
					}
#endif
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
		#endif
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

    InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
	btMultiBody* mb = bodyHandle? bodyHandle->m_multiBody:0;   
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

    } else
	{
		btRigidBody* rb = bodyHandle? bodyHandle->m_rigidBody :0;   
		if (rb)
		{
			UrdfLinkNameMapUtil* util = new UrdfLinkNameMapUtil;
			m_data->m_urdfLinkNameMapper.push_back(util);
			util->m_memSerializer = new btDefaultSerializer(bufferSizeInBytes ,(unsigned char*)bufferServerToClient);
			util->m_memSerializer->startSerialization();
			util->m_memSerializer->registerNameForPointer(bodyHandle->m_rigidBody,bodyHandle->m_bodyName.c_str());
			//disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
			for (int i=0;i<bodyHandle->m_rigidBodyJoints.size();i++)
			{
				const btGeneric6DofSpring2Constraint* con = bodyHandle->m_rigidBodyJoints.at(i);
#if 0
				const btRigidBody& bodyA = con->getRigidBodyA();
				const btRigidBody& bodyB = con->getRigidBodyB();
				int len = bodyA.calculateSerializeBufferSize();
				btChunk* chunk = util->m_memSerializer->allocate(len,1);
				const char* structType = bodyA.serialize(chunk->m_oldPtr, util->m_memSerializer);
				util->m_memSerializer->finalizeChunk(chunk,structType,BT_RIGIDBODY_CODE,(void*)&bodyA);
#endif
				util->m_memSerializer->registerNameForPointer(con,bodyHandle->m_rigidBodyJointNames[i].c_str());
				util->m_memSerializer->registerNameForPointer(&con->getRigidBodyB(),bodyHandle->m_rigidBodyLinkNames[i].c_str());

				const btRigidBody& bodyA = con->getRigidBodyA();

				int len = con->calculateSerializeBufferSize();
				btChunk* chunk = util->m_memSerializer->allocate(len,1);
				const char* structType = con->serialize(chunk->m_oldPtr, util->m_memSerializer);
				util->m_memSerializer->finalizeChunk(chunk,structType,BT_CONSTRAINT_CODE,(void*)con);
			}

			streamSizeInBytes = util->m_memSerializer->getCurrentBufferSize();
#if 0
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
#endif
		}
	}

    return streamSizeInBytes;
}

bool PhysicsServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes )
{
//	BT_PROFILE("processCommand");

	bool hasStatus = false;

    {
        {
			if (m_data->m_commandLogger)
			{
                m_data->m_commandLogger->logCommand(clientCmd);
			}
			serverStatusOut.m_type = CMD_INVALID_STATUS;
			serverStatusOut.m_numDataStreamBytes = 0;
			serverStatusOut.m_dataStream = 0;

            //consume the command
			switch (clientCmd.m_type)
            {

				case CMD_STATE_LOGGING:
				{
					BT_PROFILE("CMD_STATE_LOGGING");

					serverStatusOut.m_type = CMD_STATE_LOGGING_FAILED;
                    hasStatus = true;

					if (clientCmd.m_updateFlags & STATE_LOGGING_START_LOG)
					{
						if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_PROFILE_TIMINGS)
						{
							if (m_data->m_profileTimingLoggingUid<0)
							{
								b3ChromeUtilsStartTimings();
								m_data->m_profileTimingFileName = clientCmd.m_stateLoggingArguments.m_fileName;
								int loggerUid = m_data->m_stateLoggersUniqueId++;
								serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
								serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
								m_data->m_profileTimingLoggingUid = loggerUid;
							}
						}
						if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_VIDEO_MP4)
						{
							//if (clientCmd.m_stateLoggingArguments.m_fileName)
							{
								int loggerUid = m_data->m_stateLoggersUniqueId++;
								VideoMP4Loggger* logger = new VideoMP4Loggger(loggerUid,clientCmd.m_stateLoggingArguments.m_fileName,this->m_data->m_guiHelper);
								m_data->m_stateLoggers.push_back(logger);
								serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
								serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
							}
						}
						
						if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_MINITAUR)
						{
							
							std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
							//either provide the minitaur by object unique Id, or search for first multibody with 8 motors...

							
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID)&& (clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds>0))
							{
								int bodyUniqueId = clientCmd.m_stateLoggingArguments.m_bodyUniqueIds[0];
								InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
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
                        if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_GENERIC_ROBOT)
                        {
                            std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
                            
                            int loggerUid = m_data->m_stateLoggersUniqueId++;
							int maxLogDof = 12;
							if ((clientCmd.m_updateFlags & STATE_LOGGING_MAX_LOG_DOF))
							{
								maxLogDof = clientCmd.m_stateLoggingArguments.m_maxLogDof;
							}
							
							int logFlags = 0;
							if (clientCmd.m_updateFlags & STATE_LOGGING_LOG_FLAGS)
							{
								logFlags = clientCmd.m_stateLoggingArguments.m_logFlags;
							}
                            GenericRobotStateLogger* logger = new GenericRobotStateLogger(loggerUid,fileName,m_data->m_dynamicsWorld,maxLogDof, logFlags);
                            
                            if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID) && (clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds>0))
                            {
                                logger->m_filterObjectUniqueId = true;
                                for (int i = 0; i < clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds; ++i)
                                {
									int objectUniqueId  = clientCmd.m_stateLoggingArguments.m_bodyUniqueIds[i];
                                    logger->m_bodyIdList.push_back(objectUniqueId);
                                }
                            }
                            
                            m_data->m_stateLoggers.push_back(logger);
                            serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
                            serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
                        }
						if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_CONTACT_POINTS)
						{
							std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
							int loggerUid = m_data->m_stateLoggersUniqueId++;
							ContactPointsStateLogger* logger = new ContactPointsStateLogger(loggerUid,fileName,m_data->m_dynamicsWorld);
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_LINK_INDEX_A) && clientCmd.m_stateLoggingArguments.m_linkIndexA >= -1)
							{
								logger->m_filterLinkA = true;
								logger->m_linkIndexA = clientCmd.m_stateLoggingArguments.m_linkIndexA;
							}
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_LINK_INDEX_B) && clientCmd.m_stateLoggingArguments.m_linkIndexB >= -1)
							{
								logger->m_filterLinkB = true;
								logger->m_linkIndexB = clientCmd.m_stateLoggingArguments.m_linkIndexB;
							}
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_BODY_UNIQUE_ID_A) && clientCmd.m_stateLoggingArguments.m_bodyUniqueIdA > -1)
							{
								logger->m_bodyUniqueIdA = clientCmd.m_stateLoggingArguments.m_bodyUniqueIdA;
							}
							if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_BODY_UNIQUE_ID_B) && clientCmd.m_stateLoggingArguments.m_bodyUniqueIdB > -1)
							{
								logger->m_bodyUniqueIdB = clientCmd.m_stateLoggingArguments.m_bodyUniqueIdB;
							}
							m_data->m_stateLoggers.push_back(logger);
							serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
							serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
						}
						if (clientCmd.m_stateLoggingArguments.m_logType ==STATE_LOGGING_VR_CONTROLLERS)
						{
							std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
                            int loggerUid = m_data->m_stateLoggersUniqueId++;
							int deviceFilterType = VR_DEVICE_CONTROLLER;
							if (clientCmd.m_updateFlags & STATE_LOGGING_FILTER_DEVICE_TYPE)
							{
								deviceFilterType = clientCmd.m_stateLoggingArguments.m_deviceFilterType;
							}
                            VRControllerStateLogger* logger = new VRControllerStateLogger(loggerUid,deviceFilterType, fileName);
                            m_data->m_stateLoggers.push_back(logger);
                            serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
                            serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
							
						}
					}
					if ((clientCmd.m_updateFlags & STATE_LOGGING_STOP_LOG) && clientCmd.m_stateLoggingArguments.m_loggingUniqueId>=0)
					{
						if (clientCmd.m_stateLoggingArguments.m_loggingUniqueId == m_data->m_profileTimingLoggingUid)
						{
							serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
							b3ChromeUtilsStopTimingsAndWriteJsonFile(m_data->m_profileTimingFileName.c_str());
							m_data->m_profileTimingLoggingUid = -1;
						}
						else
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
					}
					break;
				}
				case CMD_SET_VR_CAMERA_STATE:
				{
					BT_PROFILE("CMD_SET_VR_CAMERA_STATE");

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

					if (clientCmd.m_updateFlags & VR_CAMERA_FLAG)
					{
						gVRTrackingObjectFlag = clientCmd.m_vrCameraStateArguments.m_trackingObjectFlag;
					}

					serverStatusOut.m_type  = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					break;
				}
				case CMD_REQUEST_VR_EVENTS_DATA:
				{
					//BT_PROFILE("CMD_REQUEST_VR_EVENTS_DATA");
					serverStatusOut.m_sendVREvents.m_numVRControllerEvents = 0;

					for (int i=0;i<MAX_VR_CONTROLLERS;i++)
					{
						b3VRControllerEvent& event = m_data->m_vrControllerEvents.m_vrEvents[i];

						if (clientCmd.m_updateFlags&event.m_deviceType)
						{
							if (event.m_numButtonEvents + event.m_numMoveEvents)
							{
								serverStatusOut.m_sendVREvents.m_controllerEvents[serverStatusOut.m_sendVREvents.m_numVRControllerEvents++] = event;
								event.m_numButtonEvents = 0;
								event.m_numMoveEvents = 0;
								for (int b=0;b<MAX_VR_BUTTONS;b++)
								{
									event.m_buttons[b] = 0;
								}
							}
						}
					}
					serverStatusOut.m_type = CMD_REQUEST_VR_EVENTS_DATA_COMPLETED;
					hasStatus = true;
					break;
				};

				case CMD_REQUEST_MOUSE_EVENTS_DATA:
				{

					serverStatusOut.m_sendMouseEvents.m_numMouseEvents = m_data->m_mouseEvents.size();
					if (serverStatusOut.m_sendMouseEvents.m_numMouseEvents>MAX_MOUSE_EVENTS)
					{
						serverStatusOut.m_sendMouseEvents.m_numMouseEvents = MAX_MOUSE_EVENTS;
					}
					for (int i=0;i<serverStatusOut.m_sendMouseEvents.m_numMouseEvents;i++)
					{
						serverStatusOut.m_sendMouseEvents.m_mouseEvents[i] = m_data->m_mouseEvents[i];
					}

					m_data->m_mouseEvents.resize(0);
					serverStatusOut.m_type = CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED;
					hasStatus = true;
					break;
				};

				

				case CMD_REQUEST_KEYBOARD_EVENTS_DATA:
				{
					//BT_PROFILE("CMD_REQUEST_KEYBOARD_EVENTS_DATA");

					serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents = m_data->m_keyboardEvents.size();
					if (serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents>MAX_KEYBOARD_EVENTS)
					{
						serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents = MAX_KEYBOARD_EVENTS;
					}
					for (int i=0;i<serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents;i++)
					{
						serverStatusOut.m_sendKeyboardEvents.m_keyboardEvents[i] = m_data->m_keyboardEvents[i];
					}

					btAlignedObjectArray<b3KeyboardEvent> events;

					//remove out-of-date events
					for (int i=0;i<m_data->m_keyboardEvents.size();i++)
					{
						b3KeyboardEvent event = m_data->m_keyboardEvents[i];
						if (event.m_keyState & eButtonIsDown)
						{
							event.m_keyState = eButtonIsDown;
							events.push_back(event);
						}
					}
					m_data->m_keyboardEvents.resize(events.size());
					for (int i=0;i<events.size();i++)
					{
						m_data->m_keyboardEvents[i] = events[i];
					}

					serverStatusOut.m_type = CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED;
					hasStatus = true;
					break;
				};

				case CMD_REQUEST_RAY_CAST_INTERSECTIONS:
				{
					BT_PROFILE("CMD_REQUEST_RAY_CAST_INTERSECTIONS");
					serverStatusOut.m_raycastHits.m_numRaycastHits = 0;

					for (int ray=0;ray<clientCmd.m_requestRaycastIntersections.m_numRays;ray++)
					{
						btVector3 rayFromWorld(clientCmd.m_requestRaycastIntersections.m_rayFromPositions[ray][0],
							clientCmd.m_requestRaycastIntersections.m_rayFromPositions[ray][1],
							clientCmd.m_requestRaycastIntersections.m_rayFromPositions[ray][2]);
						btVector3 rayToWorld(clientCmd.m_requestRaycastIntersections.m_rayToPositions[ray][0],
							clientCmd.m_requestRaycastIntersections.m_rayToPositions[ray][1],
							clientCmd.m_requestRaycastIntersections.m_rayToPositions[ray][2]);

						btCollisionWorld::ClosestRayResultCallback rayResultCallback(rayFromWorld,rayToWorld);
						m_data->m_dynamicsWorld->rayTest(rayFromWorld,rayToWorld,rayResultCallback);
						int rayHits = serverStatusOut.m_raycastHits.m_numRaycastHits;

						if (rayResultCallback.hasHit())
						{
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitFraction 
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

							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitObjectUniqueId 
								= objectUniqueId;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitObjectLinkIndex
								= linkIndex;

							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[0] 
								= rayResultCallback.m_hitPointWorld[0];
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[1] 
								= rayResultCallback.m_hitPointWorld[1];
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[2] 
								= rayResultCallback.m_hitPointWorld[2];
						
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[0] 
								= rayResultCallback.m_hitNormalWorld[0]; 
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[1] 
								= rayResultCallback.m_hitNormalWorld[1]; 
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[2] 
								= rayResultCallback.m_hitNormalWorld[2]; 

						} else
						{
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitFraction = 1;
							serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitObjectUniqueId = -1;
							serverStatusOut.m_raycastHits.m_rayHits[serverStatusOut.m_raycastHits.m_numRaycastHits].m_hitObjectLinkIndex = -1;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[0] = 0;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[1] = 0;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitPositionWorld[2] = 0;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[0] = 0;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[1] = 0;
							serverStatusOut.m_raycastHits.m_rayHits[rayHits].m_hitNormalWorld[2] = 0;
						}
						serverStatusOut.m_raycastHits.m_numRaycastHits++;
					}
					serverStatusOut.m_type = CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED;
					hasStatus = true;
					break;
				};
				case CMD_REQUEST_DEBUG_LINES:
					{
						BT_PROFILE("CMD_REQUEST_DEBUG_LINES");

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
					BT_PROFILE("CMD_REQUEST_CAMERA_IMAGE_DATA");
					int startPixelIndex = clientCmd.m_requestPixelDataArguments.m_startPixelIndex;
                    int width = clientCmd.m_requestPixelDataArguments.m_pixelWidth;
                    int height = clientCmd.m_requestPixelDataArguments.m_pixelHeight;
                    int numPixelsCopied = 0;



		    if ((clientCmd.m_requestPixelDataArguments.m_startPixelIndex==0) &&
                            (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT)!=0)
                    {
                            m_data->m_visualConverter.setWidthAndHeight(clientCmd.m_requestPixelDataArguments.m_pixelWidth,
                                                                        clientCmd.m_requestPixelDataArguments.m_pixelHeight);
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
						float viewMat[16];
						float projMat[16];
						for (int i=0;i<16;i++)
						{
							viewMat[i] = clientCmd.m_requestPixelDataArguments.m_viewMatrix[i];
							projMat[i] = clientCmd.m_requestPixelDataArguments.m_projectionMatrix[i];
						}
						if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES)==0)
                        {
							b3OpenGLVisualizerCameraInfo tmpCamResult;
							bool result = this->m_data->m_guiHelper->getCameraInfo(
								&tmpCamResult.m_width,
								&tmpCamResult.m_height,
								tmpCamResult.m_viewMatrix,
								tmpCamResult.m_projectionMatrix,
								tmpCamResult.m_camUp,
								tmpCamResult.m_camForward,
								tmpCamResult.m_horizontal,
								tmpCamResult.m_vertical,
								&tmpCamResult.m_yaw,
								&tmpCamResult.m_pitch,
								&tmpCamResult.m_dist,
								tmpCamResult.m_target);
							if (result)
							{
								for (int i=0;i<16;i++)
								{
									viewMat[i] = tmpCamResult.m_viewMatrix[i];
									projMat[i] = tmpCamResult.m_projectionMatrix[i];
								}
							}
						 }
                        bool handled = false;
                        
                        if ((clientCmd.m_updateFlags & ER_BULLET_HARDWARE_OPENGL)!=0)
						{

							m_data->m_guiHelper->copyCameraImageData(viewMat,
                                                projMat,pixelRGBA,numRequestedPixels,
                                                depthBuffer,numRequestedPixels,
                                                segmentationMaskBuffer, numRequestedPixels,
                                                startPixelIndex,width,height,&numPixelsCopied);

                            if (numPixelsCopied>0)
                            {
                                handled = true;
                                m_data->m_guiHelper->debugDisplayCameraImageData(viewMat,
                                                projMat,pixelRGBA,numRequestedPixels,
                                                depthBuffer,numRequestedPixels,
                                                0, numRequestedPixels,
                                                startPixelIndex,width,height,&numPixelsCopied);
                            }

							
						}
                        if (!handled)
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
									b3OpenGLVisualizerCameraInfo tmpCamResult;
									bool result = this->m_data->m_guiHelper->getCameraInfo(
										&tmpCamResult.m_width,
										&tmpCamResult.m_height,
										tmpCamResult.m_viewMatrix,
										tmpCamResult.m_projectionMatrix,
										tmpCamResult.m_camUp,
										tmpCamResult.m_camForward,
										tmpCamResult.m_horizontal,
										tmpCamResult.m_vertical,
										&tmpCamResult.m_yaw,
										&tmpCamResult.m_pitch,
										&tmpCamResult.m_dist,
										tmpCamResult.m_target);
									if (result)
									{
	                                    m_data->m_visualConverter.render(tmpCamResult.m_viewMatrix,
										tmpCamResult.m_projectionMatrix);
									} else
									{
										m_data->m_visualConverter.render();
									}
                                }
                            }

							m_data->m_visualConverter.copyCameraImageData(pixelRGBA,numRequestedPixels,
                                                     depthBuffer,numRequestedPixels,
                                                     segmentationMaskBuffer, numRequestedPixels,
                                                     startPixelIndex,&width,&height,&numPixelsCopied);

							m_data->m_guiHelper->debugDisplayCameraImageData(clientCmd.m_requestPixelDataArguments.m_viewMatrix,
                                                clientCmd.m_requestPixelDataArguments.m_projectionMatrix,pixelRGBA,numRequestedPixels,
                                                depthBuffer,numRequestedPixels,
                                                segmentationMaskBuffer, numRequestedPixels,
                                                startPixelIndex,width,height,&numPixelsCopied);

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
					BT_PROFILE("CMD_SYNC_BODY_INFO");

					b3AlignedObjectArray<int> usedHandles;
					m_data->m_bodyHandles.getUsedHandles(usedHandles);
					int actualNumBodies = 0;
					for (int i=0;i<usedHandles.size();i++)
					{
						int usedHandle = usedHandles[i];
						InternalBodyData* body = m_data->m_bodyHandles.getHandle(usedHandle);
						if (body && (body->m_multiBody || body->m_rigidBody))
						{
							serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[actualNumBodies++] = usedHandle;
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
						BT_PROFILE("CMD_REQUEST_BODY_INFO");

                        const SdfRequestInfoArgs& sdfInfoArgs = clientCmd.m_sdfRequestInfoArgs;
                        //stream info into memory
                        int streamSizeInBytes = createBodyInfoStream(sdfInfoArgs.m_bodyUniqueId, bufferServerToClient, bufferSizeInBytes);

                        serverStatusOut.m_type = CMD_BODY_INFO_COMPLETED;
                        serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = sdfInfoArgs.m_bodyUniqueId;
                        serverStatusOut.m_dataStreamArguments.m_bodyName[0] = 0;
						
						InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(sdfInfoArgs.m_bodyUniqueId);
						if (bodyHandle)
						{
							strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName,bodyHandle->m_bodyName.c_str());
						}

						serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;
						
                        hasStatus = true;
                        break;
                    }
				case CMD_SAVE_WORLD:
				{
					BT_PROFILE("CMD_SAVE_WORLD");

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
								sprintf(line,"cin = p.connect(p.SHARED_MEMORY)\n");
								int len = strlen(line);
								fwrite(line,len,1,f);
							}
							{
								sprintf(line,"if (cin < 0):\n");
								int len = strlen(line);
								fwrite(line,len,1,f);
							}
							{
								sprintf(line,"    cin = p.connect(p.GUI)\n");
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
										InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
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
												if (strstr(sd.m_fileName.c_str(),".xml") && i==0)
												{
													sprintf(line,"objects = p.loadMJCF(\"%s\")\n",sd.m_fileName.c_str());
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (strstr(sd.m_fileName.c_str(),".sdf") || strstr(sd.m_fileName.c_str(),".xml") || ((strstr(sd.m_fileName.c_str(),".urdf")) && mb->getNumLinks()) )
												{
													sprintf(line,"ob = objects[%d]\n",i);
													int len = strlen(line);
													fwrite(line,len,1,f);
												}

												if (strstr(sd.m_fileName.c_str(),".sdf")||strstr(sd.m_fileName.c_str(),".xml"))
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
						BT_PROFILE("CMD_LOAD_SDF");

                        const SdfArgs& sdfArgs = clientCmd.m_sdfArguments;
                        if (m_data->m_verboseOutput)
                        {
                            b3Printf("Processed CMD_LOAD_SDF:%s", sdfArgs.m_sdfFileName);
                        }
                        bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (sdfArgs.m_useMultiBody!=0) : true;

						int flags = CUF_USE_SDF; //CUF_USE_URDF_INERTIA
						btScalar globalScaling = 1.f;
						if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
						{
							globalScaling = sdfArgs.m_globalScaling;
						}
                        bool completedOk = loadSdf(sdfArgs.m_sdfFileName,bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, globalScaling);
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
				case CMD_CREATE_COLLISION_SHAPE:
				{
					hasStatus = true;
					serverStatusOut.m_type = CMD_CREATE_COLLISION_SHAPE_FAILED;
					
					btBulletWorldImporter* worldImporter = new btBulletWorldImporter(m_data->m_dynamicsWorld);

					btCollisionShape* shape = 0;
					b3AlignedObjectArray<UrdfCollision> urdfCollisionObjects;

					btCompoundShape* compound = 0;

					if (clientCmd.m_createCollisionShapeArgs.m_numCollisionShapes>1)
					{
						compound = worldImporter->createCompoundShape();
					}
					for (int i=0;i<clientCmd.m_createCollisionShapeArgs.m_numCollisionShapes;i++)
					{
						UrdfCollision urdfColObj;

						btTransform childTransform;
						childTransform.setIdentity();
						if (clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_hasChildTransform)
						{
							childTransform.setOrigin(btVector3(clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childPosition[0],
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childPosition[1],
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childPosition[2]));
							childTransform.setRotation(btQuaternion(
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childOrientation[0],
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childOrientation[1],
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childOrientation[2],
								clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_childOrientation[3]
							));
							if (compound==0)
							{
								compound = worldImporter->createCompoundShape();
							}
						}

						urdfColObj.m_linkLocalFrame = childTransform;
						urdfColObj.m_sourceFileLocation = "memory";
						urdfColObj.m_name = "memory";
						urdfColObj.m_geometry.m_type = URDF_GEOM_UNKNOWN;

						switch (clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_type)
						{
							case GEOM_SPHERE:
							{
								double radius = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_sphereRadius;
								shape = worldImporter->createSphereShape(radius);
								if (compound)
								{
									compound->addChildShape(childTransform,shape);
								}
								urdfColObj.m_geometry.m_type = URDF_GEOM_SPHERE;
								urdfColObj.m_geometry.m_sphereRadius = radius;
								break;
							}
							case GEOM_BOX:
							{
								//double halfExtents[3] = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_sphereRadius;
								btVector3 halfExtents(
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_boxHalfExtents[0],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_boxHalfExtents[1],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_boxHalfExtents[2]);
								shape = worldImporter->createBoxShape(halfExtents);
								if (compound)
								{
									compound->addChildShape(childTransform,shape);
								}
								urdfColObj.m_geometry.m_type = URDF_GEOM_BOX;
								urdfColObj.m_geometry.m_boxSize = 2.*halfExtents;
								break;
							}
							case GEOM_CAPSULE:
							{
								shape = worldImporter->createCapsuleShapeZ(clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleRadius,
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleHeight);
								if (compound)
								{
									compound->addChildShape(childTransform,shape);
								}
								urdfColObj.m_geometry.m_type = URDF_GEOM_CAPSULE;
								urdfColObj.m_geometry.m_capsuleRadius = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleRadius;
								urdfColObj.m_geometry.m_capsuleHeight = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleHeight;

								break;
							}
							case GEOM_CYLINDER:
							{
								shape = worldImporter->createCylinderShapeZ(clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleRadius,
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleHeight);
								if (compound)
								{
									compound->addChildShape(childTransform,shape);
								}
								urdfColObj.m_geometry.m_type = URDF_GEOM_CYLINDER;
								urdfColObj.m_geometry.m_capsuleRadius = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleRadius;
								urdfColObj.m_geometry.m_capsuleHeight = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_capsuleHeight;

								break;
							}
							case GEOM_PLANE:
							{
								btVector3 planeNormal(clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[0],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[1],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[2]);

								shape = worldImporter->createPlaneShape(planeNormal,0);
								if (compound)
								{
									compound->addChildShape(childTransform,shape);
								}
								urdfColObj.m_geometry.m_type = URDF_GEOM_PLANE;
								urdfColObj.m_geometry.m_planeNormal.setValue(
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[0],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[1],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_planeNormal[2]);
									
								break;
							}
							case GEOM_MESH:
							{
								btScalar defaultCollisionMargin = 0.001;

								btVector3 meshScale(clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_meshScale[0],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_meshScale[1],
									clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_meshScale[2]);

								const std::string& urdf_path="";

								std::string fileName = clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_meshFileName;
								urdfColObj.m_geometry.m_type = URDF_GEOM_MESH;
								urdfColObj.m_geometry.m_meshFileName = fileName;
							
								urdfColObj.m_geometry.m_meshScale = meshScale;
								char relativeFileName[1024];
								char pathPrefix[1024];
								pathPrefix[0] = 0;
								if (b3ResourcePath::findResourcePath(fileName.c_str(), relativeFileName, 1024))
								{

									b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
								}
								
								const std::string& error_message_prefix="";
								std::string out_found_filename; 
								int out_type;

								bool foundFile = findExistingMeshFile(pathPrefix, relativeFileName,error_message_prefix,&out_found_filename, &out_type); 
								if (foundFile)
								{
									urdfColObj.m_geometry.m_meshFileType = out_type;

									if (out_type==UrdfGeometry::FILE_OBJ)
									{
										//create a convex hull for each shape, and store it in a btCompoundShape

										if (clientCmd.m_createCollisionShapeArgs.m_shapes[i].m_collisionFlags&GEOM_FORCE_CONCAVE_TRIMESH)
										{
											GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, pathPrefix);
											
											if (!glmesh || glmesh->m_numvertices<=0)
											{
												b3Warning("%s: cannot extract mesh from '%s'\n", pathPrefix, relativeFileName);
												delete glmesh;
												break;
											}
											btAlignedObjectArray<btVector3> convertedVerts;
											convertedVerts.reserve(glmesh->m_numvertices);
											
											for (int i=0; i<glmesh->m_numvertices; i++)
											{
												convertedVerts.push_back(btVector3(
													glmesh->m_vertices->at(i).xyzw[0]*meshScale[0],
													glmesh->m_vertices->at(i).xyzw[1]*meshScale[1],
													glmesh->m_vertices->at(i).xyzw[2]*meshScale[2]));
											}

											BT_PROFILE("convert trimesh");
											btTriangleMesh* meshInterface = new btTriangleMesh();
											this->m_data->m_meshInterfaces.push_back(meshInterface);
											{
												BT_PROFILE("convert vertices");

												for (int i=0; i<glmesh->m_numIndices/3; i++)
												{
													const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i*3)];
													const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i*3+1)];
													const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i*3+2)];
													meshInterface->addTriangle(v0,v1,v2);
												}
											}
											{
												BT_PROFILE("create btBvhTriangleMeshShape");
												btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface,true,true);
												m_data->m_collisionShapes.push_back(trimesh);
												//trimesh->setLocalScaling(collision->m_geometry.m_meshScale);
												shape = trimesh;
												if (compound)
												{
													compound->addChildShape(childTransform,shape);
												}
											}
											delete glmesh;
										} else
										{

											std::vector<tinyobj::shape_t> shapes;
											std::string err = tinyobj::LoadObj(shapes,out_found_filename.c_str());

											//shape = createConvexHullFromShapes(shapes, collision->m_geometry.m_meshScale);
											//static btCollisionShape* createConvexHullFromShapes(std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale)
											B3_PROFILE("createConvexHullFromShapes");
											if (compound==0)
											{
												compound = worldImporter->createCompoundShape();
											}
											compound->setMargin(defaultCollisionMargin);

											for (int s = 0; s<(int)shapes.size(); s++)
											{
												btConvexHullShape* convexHull = worldImporter->createConvexHullShape();
												convexHull->setMargin(defaultCollisionMargin);
												tinyobj::shape_t& shape = shapes[s];
												int faceCount = shape.mesh.indices.size();

												for (int f = 0; f<faceCount; f += 3)
												{

													btVector3 pt;
													pt.setValue(shape.mesh.positions[shape.mesh.indices[f] * 3 + 0],
														shape.mesh.positions[shape.mesh.indices[f] * 3 + 1],
														shape.mesh.positions[shape.mesh.indices[f] * 3 + 2]);
			
													convexHull->addPoint(pt*meshScale,false);

													pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 0],
																shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 1],
																shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 2]);
													convexHull->addPoint(pt*meshScale, false);

													pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 0],
																shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 1],
																shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 2]);
													convexHull->addPoint(pt*meshScale, false);
												}

												convexHull->recalcLocalAabb();
												convexHull->optimizeConvexHull();
												compound->addChildShape(childTransform,convexHull);
											}
										}
									}
								}
								break;
							}
							default:
							{
							}
						}
						if (urdfColObj.m_geometry.m_type != URDF_GEOM_UNKNOWN)
						{
							urdfCollisionObjects.push_back(urdfColObj);
						}
					}

					
					#if 0
					shape = worldImporter->createCylinderShapeX(radius,height);
					shape = worldImporter->createCylinderShapeY(radius,height);
					shape = worldImporter->createCylinderShapeZ(radius,height);
					shape = worldImporter->createCapsuleShapeX(radius,height);
					shape = worldImporter->createCapsuleShapeY(radius,height);
					shape = worldImporter->createCapsuleShapeZ(radius,height);
					shape = worldImporter->createBoxShape(halfExtents);
					#endif
					if (compound && compound->getNumChildShapes())
					{
						shape = compound;
					}

					if (shape)
					{
						int collisionShapeUid = m_data->m_userCollisionShapeHandles.allocHandle();
						InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(collisionShapeUid);
						handle->m_collisionShape = shape;
						for (int i=0;i<urdfCollisionObjects.size();i++)
						{
							handle->m_urdfCollisionObjects.push_back(urdfCollisionObjects[i]);
						}
						serverStatusOut.m_createCollisionShapeResultArgs.m_collisionShapeUniqueId = collisionShapeUid;
						m_data->m_worldImporters.push_back(worldImporter);
						serverStatusOut.m_type = CMD_CREATE_COLLISION_SHAPE_COMPLETED;
					} else
					{
						delete worldImporter;
					}

				
					break;
				}
				case CMD_CREATE_VISUAL_SHAPE:
				{
					hasStatus = true;
					serverStatusOut.m_type = CMD_CREATE_VISUAL_SHAPE_FAILED;
					break;
				}
				case CMD_CREATE_MULTI_BODY:
				{
					hasStatus = true;
					serverStatusOut.m_type = CMD_CREATE_MULTI_BODY_FAILED;
					if (clientCmd.m_createMultiBodyArgs.m_baseLinkIndex>=0)
					{
						m_data->m_sdfRecentLoadedBodies.clear();

						#if 0
						struct UrdfModel
						{
							std::string m_name;
							std::string m_sourceFile;
							btTransform m_rootTransformInWorld;
							btHashMap<btHashString, UrdfMaterial*> m_materials;
							btHashMap<btHashString, UrdfLink*> m_links;
							btHashMap<btHashString, UrdfJoint*> m_joints;
	
							btArray<UrdfLink*> m_rootLinks;
							bool m_overrideFixedBase;

							UrdfModel()
						
						clientCmd.m_createMultiBodyArgs.

						char m_bodyName[1024];
						int m_baseLinkIndex;

						double m_baseWorldPosition[3];
						double m_baseWorldOrientation[4];

						UrdfModel tmpModel;
						tmpModel.m_bodyName = 
							#endif

						ProgrammaticUrdfInterface u2b(clientCmd.m_createMultiBodyArgs, m_data);
						
						bool useMultiBody = true;
						if (clientCmd.m_updateFlags & MULT_BODY_USE_MAXIMAL_COORDINATES)
						{
							useMultiBody = false;
						}						
					
						int flags = 0;
						bool ok = processImportedObjects("memory", bufferServerToClient, bufferSizeInBytes, useMultiBody,  flags, u2b);

						if (ok)
						{
							int bodyUniqueId = -1;

							if (m_data->m_sdfRecentLoadedBodies.size()==1)
							{
								bodyUniqueId = m_data->m_sdfRecentLoadedBodies[0];
							}
							m_data->m_sdfRecentLoadedBodies.clear();
							if (bodyUniqueId>=0)
							{
								m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
								serverStatusOut.m_type = CMD_CREATE_MULTI_BODY_COMPLETED;
								
								int streamSizeInBytes = createBodyInfoStream(bodyUniqueId, bufferServerToClient, bufferSizeInBytes);
								if (m_data->m_urdfLinkNameMapper.size())
								{
									serverStatusOut.m_numDataStreamBytes = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
								}
								serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
								InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
								strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName, body->m_bodyName.c_str());
							}
						}

						//ConvertURDF2Bullet(u2b,creation, rootTrans,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix(),flags);

						
					}
					break;
				}
				case CMD_SET_ADDITIONAL_SEARCH_PATH:
				{
					BT_PROFILE("CMD_SET_ADDITIONAL_SEARCH_PATH");
					b3ResourcePath::setAdditionalSearchPath(clientCmd.m_searchPathArgs.m_path);
					serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
                    hasStatus = true;
					break;
				}
                case CMD_LOAD_URDF:
                {
					BT_PROFILE("CMD_LOAD_URDF");
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
					int urdfFlags = 0;
					if (clientCmd.m_updateFlags & URDF_ARGS_HAS_CUSTOM_URDF_FLAGS)
					{
						urdfFlags = urdfArgs.m_urdfFlags;
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
					btScalar globalScaling = 1.f;
					if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
					{
						globalScaling = urdfArgs.m_globalScaling;
					}
                    //load the actual URDF and send a report: completed or failed
                    bool completedOk = loadUrdf(urdfArgs.m_urdfFileName,
                                               initialPos,initialOrn,
                                               useMultiBody, useFixedBase,&bodyUniqueId, bufferServerToClient, bufferSizeInBytes, urdfFlags, globalScaling);

                    if (completedOk && bodyUniqueId>=0)
                    {


						m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

						serverStatusOut.m_type = CMD_URDF_LOADING_COMPLETED;
                       
						int streamSizeInBytes = createBodyInfoStream(bodyUniqueId, bufferServerToClient, bufferSizeInBytes);


						if (m_data->m_urdfLinkNameMapper.size())
						{
							serverStatusOut.m_numDataStreamBytes = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size()-1)->m_memSerializer->getCurrentBufferSize();
						}
						serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
						InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName, body->m_bodyName.c_str());
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
					serverStatusOut.m_type = CMD_UNKNOWN_COMMAND_FLUSHED;
					hasStatus = true;
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
					serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
#endif
                    break;
                }
                case CMD_CREATE_SENSOR:
                {
					BT_PROFILE("CMD_CREATE_SENSOR");

					if (m_data->m_verboseOutput)
					{
						b3Printf("Processed CMD_CREATE_SENSOR");
					}
					int bodyUniqueId = clientCmd.m_createSensorArguments.m_bodyUniqueId;
					InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
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
				case CMD_PROFILE_TIMING:
				{
					{
						B3_PROFILE("custom");//clientCmd.m_profile.m_name);
						{
							B3_PROFILE("event");//clientCmd.m_profile.m_name);
							char** eventNamePtr = m_data->m_profileEvents[clientCmd.m_profile.m_name];
							char* eventName = 0;
							if (eventNamePtr)
							{
								B3_PROFILE("reuse");
								eventName = *eventNamePtr;
								
							} else
							{
								B3_PROFILE("alloc");
								int len = strlen(clientCmd.m_profile.m_name);
								eventName = new char[len+1];
								strcpy(eventName,clientCmd.m_profile.m_name);
								eventName[len] = 0;
								m_data->m_profileEvents.insert(eventName,eventName);
								
							}

							
							{
								{
									B3_PROFILE("with");//clientCmd.m_profile.m_name);
									{
										B3_PROFILE("some");//clientCmd.m_profile.m_name);
										{
											B3_PROFILE("deep");//clientCmd.m_profile.m_name);
											{
												B3_PROFILE("level");//clientCmd.m_profile.m_name);
												{
													B3_PROFILE(eventName);
													b3Clock::usleep(clientCmd.m_profile.m_durationInMicroSeconds);
												}
											}
										}
									}
								}
							}
						}
					}
					serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					break;
				}

				case CMD_SEND_DESIRED_STATE:
                    {
						BT_PROFILE("CMD_SEND_DESIRED_STATE");
						if (m_data->m_verboseOutput)
						{
                            b3Printf("Processed CMD_SEND_DESIRED_STATE");
						}

							int bodyUniqueId = clientCmd.m_sendDesiredStateCommandArgument.m_bodyUniqueId;
							InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);

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
					} else
					{
						//support for non-btMultiBody, such as btRigidBody

						if (body && body->m_rigidBody)
                        {
                                btRigidBody* rb = body->m_rigidBody;
                                btAssert(rb);

                                //switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
                                {
								 //case CONTROL_MODE_TORQUE:
                                    {
										if (m_data->m_verboseOutput)
										{
											b3Printf("Using CONTROL_MODE_TORQUE");
										}
                                      //  mb->clearForcesAndTorques();
                                        ///see addJointInfoFromConstraint
										int velIndex = 6;
										int posIndex = 7;
                                        //if ((clientCmd.m_updateFlags&SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
                                        {
											for (int link=0;link<body->m_rigidBodyJoints.size();link++)
                                            {
												btGeneric6DofSpring2Constraint* con = body->m_rigidBodyJoints[link];
												
												btVector3 linearLowerLimit;
												btVector3 linearUpperLimit;
												btVector3 angularLowerLimit;
												btVector3 angularUpperLimit;


                                                //for (int dof=0;dof<mb->getLink(link).m_dofCount;dof++)
                                                {
                                                    

                                                    {

														int torqueIndex = velIndex;
														double torque = 100;
														bool hasDesiredTorque = false;
														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_MAX_FORCE)!=0)
														{
															torque = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex];
															hasDesiredTorque = true;
														}
														
														bool hasDesiredPosOrVel = false;
                                                        btScalar qdotTarget = 0.f;
														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT)!=0)
                                                        {
                                                            hasDesiredPosOrVel = true;
															qdotTarget = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
                                                        }
														btScalar qTarget = 0.f;
														if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q)!=0)
                                                        {
                                                            hasDesiredPosOrVel = true;
															qTarget = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex];
                                                        }
                                                        
														con->getLinearLowerLimit(linearLowerLimit);
														con->getLinearUpperLimit(linearUpperLimit);
														con->getAngularLowerLimit(angularLowerLimit);
														con->getAngularUpperLimit(angularUpperLimit);

														if (linearLowerLimit.isZero() && linearUpperLimit.isZero() && angularLowerLimit.isZero() && angularUpperLimit.isZero())
														{
															//fixed, don't do anything
														} else
														{
															con->calculateTransforms();

															if (linearLowerLimit.isZero() && linearUpperLimit.isZero())
															{
																//eRevoluteType;
																btVector3 limitRange = angularLowerLimit.absolute()+angularUpperLimit.absolute();
																int limitAxis = limitRange.maxAxis();
																const btTransform& transA = con->getCalculatedTransformA();
																const btTransform& transB = con->getCalculatedTransformB();
																btVector3 axisA = transA.getBasis().getColumn(limitAxis);
																btVector3 axisB = transB.getBasis().getColumn(limitAxis);

																switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
																{
																	case CONTROL_MODE_TORQUE:
																		{
																			if (hasDesiredTorque)
																			{
																				con->getRigidBodyA().applyTorque(torque*axisA);
																				con->getRigidBodyB().applyTorque(-torque*axisB);
																			}
																			break;
																		}
																		case CONTROL_MODE_VELOCITY:
																		{
																			if (hasDesiredPosOrVel)
																			{
																				con->enableMotor(3+limitAxis,true);
																				con->setTargetVelocity(3+limitAxis, qdotTarget);
																				//this is max motor force impulse
																				btScalar torqueImpulse = torque*m_data->m_dynamicsWorld->getSolverInfo().m_timeStep;
																				con->setMaxMotorForce(3+limitAxis,torqueImpulse);
																			}
																			break;
																		}
																		case CONTROL_MODE_POSITION_VELOCITY_PD:
																		{
																			if (hasDesiredPosOrVel)
																			{
																				con->setServo(3+limitAxis,true);
																				con->setServoTarget(3+limitAxis,-qTarget);
																				//next one is the maximum velocity to reach target position.
																				//the maximum velocity is limited by maxMotorForce
																				con->setTargetVelocity(3+limitAxis, 100);
																				//this is max motor force impulse
																				btScalar torqueImpulse = torque*m_data->m_dynamicsWorld->getSolverInfo().m_timeStep;
																				con->setMaxMotorForce(3+limitAxis,torqueImpulse);
																				con->enableMotor(3+limitAxis,true);
																			}
																			break;
																		}
																	default:
																		{
																		}
																};


																
																
															} else
															{
																//ePrismaticType; @todo
																btVector3 limitRange = linearLowerLimit.absolute()+linearUpperLimit.absolute();
																int limitAxis = limitRange.maxAxis();

																const btTransform& transA = con->getCalculatedTransformA();
																const btTransform& transB = con->getCalculatedTransformB();
																btVector3 axisA = transA.getBasis().getColumn(limitAxis);
																btVector3 axisB = transB.getBasis().getColumn(limitAxis);

																switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
																{
																	case CONTROL_MODE_TORQUE:
																		{
																			con->getRigidBodyA().applyForce(-torque*axisA,btVector3(0,0,0));
																			con->getRigidBodyB().applyForce(torque*axisB,btVector3(0,0,0));
																			break;
																		}
																		case CONTROL_MODE_VELOCITY:
																		{
																			con->enableMotor(limitAxis,true);
																			con->setTargetVelocity(limitAxis, -qdotTarget);
																			//this is max motor force impulse
																			btScalar torqueImpulse = torque*m_data->m_dynamicsWorld->getSolverInfo().m_timeStep;
																			con->setMaxMotorForce(limitAxis,torqueImpulse);
																			break;
																		}
																		case CONTROL_MODE_POSITION_VELOCITY_PD:
																		{
																			con->setServo(limitAxis,true);
																			con->setServoTarget(limitAxis,qTarget);
																			//next one is the maximum velocity to reach target position.
																			//the maximum velocity is limited by maxMotorForce
																			con->setTargetVelocity(limitAxis, 100);
																			//this is max motor force impulse
																			btScalar torqueImpulse = torque*m_data->m_dynamicsWorld->getSolverInfo().m_timeStep;
																			con->setMaxMotorForce(limitAxis,torqueImpulse);
																			con->enableMotor(limitAxis,true);
																			break;
																		}
																	default:
																		{
																		}
																};

															}
														}
                                                    }//fi
													///see addJointInfoFromConstraint
													velIndex ++;//info.m_uIndex
													posIndex ++;//info.m_qIndex
                                                    
                                                }
                                            }
                                        }//fi
                                        //break;
                                    }
								
								}
						} //if (body && body->m_rigidBody)
					}

					serverStatusOut.m_type = CMD_DESIRED_STATE_RECEIVED_COMPLETED;
					hasStatus = true;
					break;
				}
				case 		CMD_REQUEST_COLLISION_INFO:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverStatusOut.m_type = CMD_REQUEST_COLLISION_INFO_FAILED;
					hasStatus=true;
					int bodyUniqueId = clientCmd.m_requestCollisionInfoArgs.m_bodyUniqueId;
					InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						

					if (body && body->m_multiBody)
					{
						btMultiBody* mb = body->m_multiBody;
						serverStatusOut.m_type = CMD_REQUEST_COLLISION_INFO_COMPLETED;
						serverCmd.m_sendCollisionInfoArgs.m_numLinks = body->m_multiBody->getNumLinks();
						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = 0;
						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = 0;
						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = 0;

						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = -1;
						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = -1;
						serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = -1;

						if (body->m_multiBody->getBaseCollider())
						{
							btTransform tr;
							tr.setOrigin(mb->getBasePos());
							tr.setRotation(mb->getWorldToBaseRot().inverse());

							btVector3 aabbMin,aabbMax;
							body->m_multiBody->getBaseCollider()->getCollisionShape()->getAabb(tr,aabbMin,aabbMax);
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = aabbMin[0];
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = aabbMin[1];
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = aabbMin[2];

							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = aabbMax[0];
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = aabbMax[1];
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = aabbMax[2];
						}
						for (int l=0;l<mb->getNumLinks();l++)
						{
							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+0] = 0;
							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+1] = 0;
							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+2] = 0;

							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+0] = -1;
							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+1] = -1;
							serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+2] = -1;

								

							if (body->m_multiBody->getLink(l).m_collider)
							{
								btVector3 aabbMin,aabbMax;
								body->m_multiBody->getLinkCollider(l)->getCollisionShape()->getAabb(mb->getLink(l).m_cachedWorldTransform,aabbMin,aabbMax);

								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+0] = aabbMin[0];
								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+1] = aabbMin[1];
								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3*l+2] = aabbMin[2];
								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+0] = aabbMax[0];
								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+1] = aabbMax[1];
								serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3*l+2] = aabbMax[2];
							}
						
						}
					}
					else
					{
						if (body && body->m_rigidBody)
						{
							btRigidBody* rb = body->m_rigidBody;
							SharedMemoryStatus& serverCmd = serverStatusOut;
							serverStatusOut.m_type = CMD_REQUEST_COLLISION_INFO_COMPLETED;
							serverCmd.m_sendCollisionInfoArgs.m_numLinks = 0;
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = 0;
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = 0;
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = 0;

							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = -1;
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = -1;
							serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = -1;
							if (rb->getCollisionShape())
							{
								btTransform tr = rb->getWorldTransform();
								
								btVector3 aabbMin,aabbMax;
								rb->getCollisionShape()->getAabb(tr,aabbMin,aabbMax);
								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = aabbMin[0];
								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = aabbMin[1];
								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = aabbMin[2];

								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = aabbMax[0];
								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = aabbMax[1];
								serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = aabbMax[2];
							}
						}
					}
					break;
				}

				case CMD_REQUEST_ACTUAL_STATE:
					{
					BT_PROFILE("CMD_REQUEST_ACTUAL_STATE");
						if (m_data->m_verboseOutput)
						{
							b3Printf("Sending the actual state (Q,U)");
						}
						int bodyUniqueId = clientCmd.m_requestActualStateInformationCommandArgument.m_bodyUniqueId;
						InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						

						if (body && body->m_multiBody)
						{
							btMultiBody* mb = body->m_multiBody;
							SharedMemoryStatus& serverCmd = serverStatusOut;
							serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

							serverCmd.m_sendActualStateArgs.m_bodyUniqueId = bodyUniqueId;
							serverCmd.m_sendActualStateArgs.m_numLinks = body->m_multiBody->getNumLinks();

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

							btAlignedObjectArray<btVector3> omega;
							btAlignedObjectArray<btVector3> linVel;
							
							bool computeForwardKinematics = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS)!=0);
							if (computeForwardKinematics)
							{
								B3_PROFILE("compForwardKinematics");
								btAlignedObjectArray<btQuaternion> world_to_local;
								btAlignedObjectArray<btVector3> local_origin;
								world_to_local.resize(mb->getNumLinks()+1);
								local_origin.resize(mb->getNumLinks()+1);
								mb->forwardKinematics(world_to_local,local_origin);
							}

							bool computeLinkVelocities = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_LINKVELOCITY)!=0);
							if (computeLinkVelocities)
							{
								omega.resize(mb->getNumLinks()+1);
								linVel.resize(mb->getNumLinks()+1);
								{
									B3_PROFILE("compTreeLinkVelocities");
									mb->compTreeLinkVelocities(&omega[0], &linVel[0]);
								}
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

							

								btVector3 worldLinVel(0,0,0);
								btVector3 worldAngVel(0,0,0);
								
								if (computeLinkVelocities)
								{
									const btMatrix3x3& linkRotMat = mb->getLink(l).m_cachedWorldTransform.getBasis();
									worldLinVel = linkRotMat * linVel[l+1];
									worldAngVel = linkRotMat * omega[l+1];
								}

								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+0] = worldLinVel[0];
								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+1] = worldLinVel[1];
								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+2] = worldLinVel[2];
								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+3] = worldAngVel[0];
								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+4] = worldAngVel[1];
								serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+5] = worldAngVel[2];
								
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
								serverCmd.m_sendActualStateArgs.m_numLinks = 0;

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
					BT_PROFILE("CMD_STEP_FORWARD_SIMULATION");


					if (m_data->m_verboseOutput)
					{
						b3Printf("Step simulation request");
						b3Printf("CMD_STEP_FORWARD_SIMULATION clientCmd = %d\n", clientCmd.m_sequenceNumber);
					}
                    ///todo(erwincoumans) move this damping inside Bullet
                    for (int i=0;i<m_data->m_dynamicsWorld->getNumMultibodies();i++)
					{
						btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(i);
						for (int l=0;l<mb->getNumLinks();l++) 
						{
							for (int d=0;d<mb->getLink(l).m_dofCount;d++) 
							{
								double damping_coefficient = mb->getLink(l).m_jointDamping;
								double damping = -damping_coefficient*mb->getJointVelMultiDof(l)[d];
								mb->addJointTorqueMultiDof(l, d, damping);
							}
						}
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
					BT_PROFILE("CMD_REQUEST_INTERNAL_DATA");

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
				case CMD_CHANGE_DYNAMICS_INFO:
				{
					BT_PROFILE("CMD_CHANGE_DYNAMICS_INFO");
					
					int bodyUniqueId = clientCmd.m_changeDynamicsInfoArgs.m_bodyUniqueId;
					int linkIndex = clientCmd.m_changeDynamicsInfoArgs.m_linkIndex;
					double mass = clientCmd.m_changeDynamicsInfoArgs.m_mass;
					double lateralFriction = clientCmd.m_changeDynamicsInfoArgs.m_lateralFriction;
					double spinningFriction = clientCmd.m_changeDynamicsInfoArgs.m_spinningFriction;
					double rollingFriction = clientCmd.m_changeDynamicsInfoArgs.m_rollingFriction;
					double restitution = clientCmd.m_changeDynamicsInfoArgs.m_restitution;
					btAssert(bodyUniqueId >= 0);
					btAssert(linkIndex >= -1);
						
					InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);

					if (body && body->m_multiBody)
					{
						btMultiBody* mb = body->m_multiBody;
						if (linkIndex == -1)
						{
							if (mb->getBaseCollider())
							{
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_RESTITUTION)
								{
									mb->getBaseCollider()->setRestitution(restitution);
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LINEAR_DAMPING)
								{
									mb->setLinearDamping(clientCmd.m_changeDynamicsInfoArgs.m_linearDamping);
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ANGULAR_DAMPING)
								{
									mb->setLinearDamping(clientCmd.m_changeDynamicsInfoArgs.m_angularDamping);
								}

								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_CONTACT_STIFFNESS_AND_DAMPING)
								{
									mb->getBaseCollider()->setContactStiffnessAndDamping(clientCmd.m_changeDynamicsInfoArgs.m_contactStiffness, clientCmd.m_changeDynamicsInfoArgs.m_contactDamping);
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LATERAL_FRICTION)
								{
									mb->getBaseCollider()->setFriction(lateralFriction);
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_SPINNING_FRICTION)
								{
									mb->getBaseCollider()->setSpinningFriction(spinningFriction);
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ROLLING_FRICTION)
								{
									mb->getBaseCollider()->setRollingFriction(rollingFriction);
								}		

								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_FRICTION_ANCHOR)
								{
									if (clientCmd.m_changeDynamicsInfoArgs.m_frictionAnchor)
									{
										mb->getBaseCollider()->setCollisionFlags(mb->getBaseCollider()->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
									} else
									{
										mb->getBaseCollider()->setCollisionFlags(mb->getBaseCollider()->getCollisionFlags() & ~btCollisionObject::CF_HAS_FRICTION_ANCHOR);
									}
								}		
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_MASS)
							{
								mb->setBaseMass(mass);	
								if (mb->getBaseCollider() && mb->getBaseCollider()->getCollisionShape())
								{
									btVector3 localInertia;
									mb->getBaseCollider()->getCollisionShape()->calculateLocalInertia(mass,localInertia);
									mb->setBaseInertia(localInertia);
								}
							}
						}
						else
						{
							if (linkIndex >= 0 && linkIndex < mb->getNumLinks())
							{
								if (mb->getLinkCollider(linkIndex))
								{
									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_RESTITUTION)
									{
										mb->getLinkCollider(linkIndex)->setRestitution(restitution);
									}
									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_SPINNING_FRICTION)
									{
										mb->getLinkCollider(linkIndex)->setSpinningFriction(spinningFriction);
									}
									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ROLLING_FRICTION)
									{
										mb->getLinkCollider(linkIndex)->setRollingFriction(rollingFriction);
									}

									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_FRICTION_ANCHOR)
									{
										if (clientCmd.m_changeDynamicsInfoArgs.m_frictionAnchor)
										{
											mb->getLinkCollider(linkIndex)->setCollisionFlags(mb->getLinkCollider(linkIndex)->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
										} else
										{
											mb->getLinkCollider(linkIndex)->setCollisionFlags(mb->getLinkCollider(linkIndex)->getCollisionFlags() & ~btCollisionObject::CF_HAS_FRICTION_ANCHOR);
										}
									}		


									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LATERAL_FRICTION)
									{
										mb->getLinkCollider(linkIndex)->setFriction(lateralFriction);
									}

									if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_CONTACT_STIFFNESS_AND_DAMPING)
									{
										mb->getLinkCollider(linkIndex)->setContactStiffnessAndDamping(clientCmd.m_changeDynamicsInfoArgs.m_contactStiffness, clientCmd.m_changeDynamicsInfoArgs.m_contactDamping);
									}
								
			
								}
								if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_MASS)
								{
									mb->getLink(linkIndex).m_mass = mass;
									if (mb->getLinkCollider(linkIndex) && mb->getLinkCollider(linkIndex)->getCollisionShape())
									{
										btVector3 localInertia;
										mb->getLinkCollider(linkIndex)->getCollisionShape()->calculateLocalInertia(mass,localInertia);
										mb->getLink(linkIndex).m_inertiaLocal = localInertia;
									}
								}
							}
						}
					} else
					{
						if (body && body->m_rigidBody)
						{
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LINEAR_DAMPING)
							{
								btScalar angDamping = body->m_rigidBody->getAngularDamping();
								body->m_rigidBody->setDamping(clientCmd.m_changeDynamicsInfoArgs.m_linearDamping,angDamping);
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ANGULAR_DAMPING)
							{
								btScalar linDamping = body->m_rigidBody->getLinearDamping();
								body->m_rigidBody->setDamping(linDamping, clientCmd.m_changeDynamicsInfoArgs.m_angularDamping);
							}

							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_CONTACT_STIFFNESS_AND_DAMPING)
							{
								body->m_rigidBody->setContactStiffnessAndDamping(clientCmd.m_changeDynamicsInfoArgs.m_contactStiffness, clientCmd.m_changeDynamicsInfoArgs.m_contactDamping);
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_RESTITUTION)
							{
								body->m_rigidBody->setRestitution(restitution);
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LATERAL_FRICTION)
							{
								body->m_rigidBody->setFriction(lateralFriction);
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_SPINNING_FRICTION)
							{
								body->m_rigidBody->setSpinningFriction(spinningFriction);
							}
							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ROLLING_FRICTION)
							{
								body->m_rigidBody->setRollingFriction(rollingFriction);
							}

							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_FRICTION_ANCHOR)
							{
								if (clientCmd.m_changeDynamicsInfoArgs.m_frictionAnchor)
								{
									body->m_rigidBody->setCollisionFlags(body->m_rigidBody->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
								} else
								{
									body->m_rigidBody->setCollisionFlags(body->m_rigidBody->getCollisionFlags() & ~btCollisionObject::CF_HAS_FRICTION_ANCHOR);
								}
							}	

							if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_MASS)
							{
								btVector3 localInertia;
								if (body->m_rigidBody->getCollisionShape())
								{
									body->m_rigidBody->getCollisionShape()->calculateLocalInertia(mass,localInertia);
								}
								body->m_rigidBody->setMassProps(mass,localInertia);
							}
						}
					}
					
					
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					
					break;
				};
				case CMD_GET_DYNAMICS_INFO:
				{
					int bodyUniqueId = clientCmd.m_getDynamicsInfoArgs.m_bodyUniqueId;
					int linkIndex = clientCmd.m_getDynamicsInfoArgs.m_linkIndex;
					InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
					if (body && body->m_multiBody)
					{
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_GET_DYNAMICS_INFO_COMPLETED;
						
						btMultiBody* mb = body->m_multiBody;
						if (linkIndex == -1)
						{
							serverCmd.m_dynamicsInfo.m_mass = mb->getBaseMass();
							serverCmd.m_dynamicsInfo.m_lateralFrictionCoeff = mb->getBaseCollider()->getFriction();
						}
						else
						{
							serverCmd.m_dynamicsInfo.m_mass = mb->getLinkMass(linkIndex);
							if (mb->getLinkCollider(linkIndex))
							{
								serverCmd.m_dynamicsInfo.m_lateralFrictionCoeff = mb->getLinkCollider(linkIndex)->getFriction();
							}
							else
							{
								b3Warning("The dynamic info requested is not available");
								serverCmd.m_type = CMD_GET_DYNAMICS_INFO_FAILED;
							}
						}
						hasStatus = true;
					}
					else
					{
						b3Warning("The dynamic info requested is not available");
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_GET_DYNAMICS_INFO_FAILED;
						hasStatus = true;
					}
					break;
				}
				case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				{
					BT_PROFILE("CMD_SEND_PHYSICS_SIMULATION_PARAMETERS");

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

					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DEFAULT_NON_CONTACT_ERP)
                    {
                        m_data->m_dynamicsWorld->getSolverInfo().m_erp = clientCmd.m_physSimParamArgs.m_defaultNonContactERP;
                    }

					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DEFAULT_FRICTION_ERP)
                    {
                        m_data->m_dynamicsWorld->getSolverInfo().m_frictionERP = clientCmd.m_physSimParamArgs.m_frictionERP;
                    }
					

					if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD)
                    {
                        m_data->m_dynamicsWorld->getSolverInfo().m_restitutionVelocityThreshold = clientCmd.m_physSimParamArgs.m_restitutionVelocityThreshold;
                    }

					

					if (clientCmd.m_updateFlags&SIM_PARAM_ENABLE_FILE_CACHING)
                    {
						b3EnableFileCaching(clientCmd.m_physSimParamArgs.m_enableFileCaching);
                    }


					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					hasStatus = true;
					break;

				};
				case CMD_INIT_POSE:
				{
					BT_PROFILE("CMD_INIT_POSE");

					if (m_data->m_verboseOutput)
					{
						b3Printf("Server Init Pose not implemented yet");
					}
					int bodyUniqueId = clientCmd.m_initPoseArgs.m_bodyUniqueId;
					InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);

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
							int uDofIndex = 6;
							int posVarCountIndex = 7;
							for (int i=0;i<mb->getNumLinks();i++)
							{
							    if ( (clientCmd.m_initPoseArgs.m_hasInitialStateQ[posVarCountIndex]) && (mb->getLink(i).m_dofCount==1))
								{
									mb->setJointPos(i,clientCmd.m_initPoseArgs.m_initialStateQ[posVarCountIndex]);
									mb->setJointVel(i,0);//backwards compatibility
								}
							    if ((clientCmd.m_initPoseArgs.m_hasInitialStateQdot[uDofIndex]) && (mb->getLink(i).m_dofCount==1))
								{
									btScalar vel = clientCmd.m_initPoseArgs.m_initialStateQdot[uDofIndex];
									mb->setJointVel(i,vel);
								}

								posVarCountIndex += mb->getLink(i).m_posVarCount;
								uDofIndex += mb->getLink(i).m_dofCount;

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

					BT_PROFILE("CMD_RESET_SIMULATION");
					m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL,0);
					resetSimulation();
					m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL,1);
					
					SharedMemoryStatus& serverCmd =serverStatusOut;
					serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
					hasStatus = true;
                    break;
                }
				case CMD_CREATE_RIGID_BODY:
				case CMD_CREATE_BOX_COLLISION_SHAPE:
					{
						BT_PROFILE("CMD_CREATE_RIGID_BODY");

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


						int bodyUniqueId = m_data->m_bodyHandles.allocHandle();
						InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						serverCmd.m_rigidBodyCreateArgs.m_bodyUniqueId = bodyUniqueId;
						rb->setUserIndex2(bodyUniqueId);
						bodyHandle->m_rootLocalInertialFrame.setIdentity();
						bodyHandle->m_rigidBody = rb;
						hasStatus = true;
						break;
					}
                case CMD_PICK_BODY:
                    {
						BT_PROFILE("CMD_PICK_BODY");

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
						BT_PROFILE("CMD_MOVE_PICKED_BODY");

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
						BT_PROFILE("CMD_REMOVE_PICKING_CONSTRAINT_BODY");
                        removePickingConstraint();

						SharedMemoryStatus& serverCmd =serverStatusOut;
						serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
						hasStatus = true;
                        break;
                    }
				case CMD_REQUEST_AABB_OVERLAP:
				{
					BT_PROFILE("CMD_REQUEST_AABB_OVERLAP");
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
                    
				case CMD_REQUEST_OPENGL_VISUALIZER_CAMERA:
				{
					BT_PROFILE("CMD_REQUEST_OPENGL_VISUALIZER_CAMERA");
                    SharedMemoryStatus& serverCmd = serverStatusOut;
					bool result = this->m_data->m_guiHelper->getCameraInfo(
						&serverCmd.m_visualizerCameraResultArgs.m_width,
						&serverCmd.m_visualizerCameraResultArgs.m_height,
						serverCmd.m_visualizerCameraResultArgs.m_viewMatrix,
						serverCmd.m_visualizerCameraResultArgs.m_projectionMatrix,
						serverCmd.m_visualizerCameraResultArgs.m_camUp,
						serverCmd.m_visualizerCameraResultArgs.m_camForward,
						serverCmd.m_visualizerCameraResultArgs.m_horizontal,
						serverCmd.m_visualizerCameraResultArgs.m_vertical,
						&serverCmd.m_visualizerCameraResultArgs.m_yaw,
						&serverCmd.m_visualizerCameraResultArgs.m_pitch,
						&serverCmd.m_visualizerCameraResultArgs.m_dist,
						serverCmd.m_visualizerCameraResultArgs.m_target);
                    serverCmd.m_type = result ? CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED: CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED;
					hasStatus = true;
					break;
				}

                case CMD_CONFIGURE_OPENGL_VISUALIZER:
                {
					BT_PROFILE("CMD_CONFIGURE_OPENGL_VISUALIZER");
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type =CMD_CLIENT_COMMAND_COMPLETED;
                    
                    hasStatus = true;
                    if (clientCmd.m_updateFlags&COV_SET_FLAGS)
                    {
                        m_data->m_guiHelper->setVisualizerFlag(clientCmd.m_configureOpenGLVisualizerArguments.m_setFlag,
                                                           clientCmd.m_configureOpenGLVisualizerArguments.m_setEnabled);
                    }
                    if (clientCmd.m_updateFlags&COV_SET_CAMERA_VIEW_MATRIX)
                    {
                        m_data->m_guiHelper->resetCamera( clientCmd.m_configureOpenGLVisualizerArguments.m_cameraDistance,
                                                          clientCmd.m_configureOpenGLVisualizerArguments.m_cameraYaw,
                                                          clientCmd.m_configureOpenGLVisualizerArguments.m_cameraPitch,
                                                          clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[0],
                                                          clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[1],
                                                          clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[2]);
                    }
                    break;
                }
                                       
                case CMD_REQUEST_CONTACT_POINT_INFORMATION:
                    {
						BT_PROFILE("CMD_REQUEST_CONTACT_POINT_INFORMATION");
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
									bool swap = false;
									if (clientCmd.m_requestContactPointArguments.m_objectAIndexFilter >= 0)
									{
										if (clientCmd.m_requestContactPointArguments.m_objectAIndexFilter == objectIndexA)
										{
											swap = false;
										}
										else if (clientCmd.m_requestContactPointArguments.m_objectAIndexFilter == objectIndexB)
										{
											swap = true;
										}
										else
										{
											continue;
										}
									}

									if (swap)
									{
										std::swap(objectIndexA, objectIndexB);
										std::swap(linkIndexA, linkIndexB);
										std::swap(bodyA, bodyB);
									}

									//apply the second object filter, if the user provides it
									if (clientCmd.m_requestContactPointArguments.m_objectBIndexFilter >= 0)
									{
										if (clientCmd.m_requestContactPointArguments.m_objectBIndexFilter != objectIndexB)
										{
											continue;
										}
									}

									if (
										(clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_A_FILTER) &&
										clientCmd.m_requestContactPointArguments.m_linkIndexAIndexFilter != linkIndexA)
									{
										continue;
									}

									if (
										(clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_B_FILTER) &&
										clientCmd.m_requestContactPointArguments.m_linkIndexBIndexFilter != linkIndexB)
									{
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
									InternalBodyData* bodyA = m_data->m_bodyHandles.getHandle(bodyUniqueIdA);
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
									InternalBodyData* bodyB = m_data->m_bodyHandles.getHandle(bodyUniqueIdB);
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
					BT_PROFILE("CMD_CALCULATE_INVERSE_DYNAMICS");
					SharedMemoryStatus& serverCmd = serverStatusOut;
					InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId);
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
					BT_PROFILE("CMD_CALCULATE_JACOBIAN");

                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_calculateJacobianArguments.m_bodyUniqueId);
                    if (bodyHandle && bodyHandle->m_multiBody)
                    {
                        serverCmd.m_type = CMD_CALCULATED_JACOBIAN_FAILED;
                        
                        btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);
                        
                        if (tree)
                        {
                            int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
                            const int numDofs = bodyHandle->m_multiBody->getNumDofs();
                            btInverseDynamics::vecx q(numDofs + baseDofs);
                            btInverseDynamics::vecx qdot(numDofs + baseDofs);
                            btInverseDynamics::vecx nu(numDofs + baseDofs);
                            btInverseDynamics::vecx joint_force(numDofs + baseDofs);
                            for (int i = 0; i < numDofs; i++)
                            {
                                q[i + baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointPositionsQ[i];
                                qdot[i + baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointVelocitiesQdot[i];
                                nu[i + baseDofs] = clientCmd.m_calculateJacobianArguments.m_jointAccelerations[i];
                            }
                            // Set the gravity to correspond to the world gravity
                            btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
                            if (-1 != tree->setGravityInWorldFrame(id_grav) &&
                                -1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
                            {
                                serverCmd.m_jacobianResultArgs.m_dofCount = numDofs + baseDofs;
                                // Set jacobian value
                                tree->calculateJacobians(q);
                                btInverseDynamics::mat3x jac_t(3, numDofs + baseDofs);
                                btInverseDynamics::mat3x jac_r(3, numDofs + baseDofs);
                                
                                // Note that inverse dynamics uses zero-based indexing of bodies, not starting from -1 for the base link.
                                tree->getBodyJacobianTrans(clientCmd.m_calculateJacobianArguments.m_linkIndex + 1, &jac_t);
                                tree->getBodyJacobianRot(clientCmd.m_calculateJacobianArguments.m_linkIndex + 1, &jac_r);
                                // Update the translational jacobian based on the desired local point.
                                // v_pt = v_frame + w x pt
                                // v_pt = J_t * qd + (J_r * qd) x pt
                                // v_pt = J_t * qd - pt x (J_r * qd)
                                // v_pt = J_t * qd - pt_x * J_r * qd)
                                // v_pt = (J_t - pt_x * J_r) * qd
                                // J_t_new = J_t - pt_x * J_r
                                btInverseDynamics::vec3 localPosition;
                                for (int i = 0; i < 3; ++i) {
                                    localPosition(i) = clientCmd.m_calculateJacobianArguments.m_localPosition[i];
                                }
                                // Only calculate if the localPosition is non-zero.
                                if (btInverseDynamics::maxAbs(localPosition) > 0.0) {
                                    // Write the localPosition into world coordinates.
                                    btInverseDynamics::mat33 world_rotation_body;
                                    tree->getBodyTransform(clientCmd.m_calculateJacobianArguments.m_linkIndex + 1, &world_rotation_body);
                                    localPosition = world_rotation_body * localPosition;
                                    // Correct the translational jacobian.
                                    btInverseDynamics::mat33 skewCrossProduct;
                                    btInverseDynamics::skew(localPosition, &skewCrossProduct);
                                    btInverseDynamics::mat3x jac_l(3, numDofs + baseDofs);
                                    btInverseDynamics::mul(skewCrossProduct, jac_r, &jac_l);
                                    btInverseDynamics::mat3x jac_t_new(3, numDofs + baseDofs);
                                    btInverseDynamics::sub(jac_t, jac_l, &jac_t_new);
                                    jac_t = jac_t_new;
                                }
                                // Fill in the result into the shared memory.
                                for (int i = 0; i < 3; ++i)
                                {
                                    for (int j = 0; j < (numDofs + baseDofs); ++j)
                                    {
                                        int element = (numDofs + baseDofs) * i + j;
                                        serverCmd.m_jacobianResultArgs.m_linearJacobian[element] = jac_t(i,j);
                                        serverCmd.m_jacobianResultArgs.m_angularJacobian[element] = jac_r(i,j);
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
					BT_PROFILE("CMD_APPLY_EXTERNAL_FORCE");

                	if (m_data->m_verboseOutput)
                    {
                        b3Printf("CMD_APPLY_EXTERNAL_FORCE clientCmd = %d\n", clientCmd.m_sequenceNumber);
                    }
                    for (int i = 0; i < clientCmd.m_externalForceArguments.m_numForcesAndTorques; ++i)
                    {
                        InternalBodyData* body = m_data->m_bodyHandles.getHandle(clientCmd.m_externalForceArguments.m_bodyUniqueIds[i]);
						bool isLinkFrame = ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_LINK_FRAME) != 0);

                        if (body && body->m_multiBody)
                        {
                            btMultiBody* mb = body->m_multiBody;
                         
                            if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_FORCE)!=0)
                            {
                                btVector3 tmpForce(clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+0],
                                                clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+1],
                                                clientCmd.m_externalForceArguments.m_forcesAndTorques[i*3+2]);
                                btVector3 tmpPosition(
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+0],
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+1],
                                                        clientCmd.m_externalForceArguments.m_positions[i*3+2]);

								
                                if (clientCmd.m_externalForceArguments.m_linkIds[i] == -1)
                                {
                                    btVector3 forceWorld = isLinkFrame ? mb->getBaseWorldTransform().getBasis()*tmpForce : tmpForce;
									btVector3 relPosWorld = isLinkFrame ? mb->getBaseWorldTransform().getBasis()*tmpPosition : tmpPosition - mb->getBaseWorldTransform().getOrigin();
                                    mb->addBaseForce(forceWorld);
                                    mb->addBaseTorque(relPosWorld.cross(forceWorld));
                                    //b3Printf("apply base force of %f,%f,%f at %f,%f,%f\n", forceWorld[0],forceWorld[1],forceWorld[2],positionLocal[0],positionLocal[1],positionLocal[2]);
                                } else
                                {
                                    int link = clientCmd.m_externalForceArguments.m_linkIds[i];

									btVector3 forceWorld = isLinkFrame ? mb->getLink(link).m_cachedWorldTransform.getBasis()*tmpForce : tmpForce;
									btVector3 relPosWorld = isLinkFrame ? mb->getLink(link).m_cachedWorldTransform.getBasis()*tmpPosition : tmpPosition - mb->getBaseWorldTransform().getOrigin();
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
				case CMD_REMOVE_BODY:
				{
					SharedMemoryStatus& serverCmd =serverStatusOut;
                    serverCmd.m_type = CMD_REMOVE_BODY_FAILED;
					serverCmd.m_removeObjectArgs.m_numBodies = 0;
					serverCmd.m_removeObjectArgs.m_numUserConstraints = 0;

					m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL,0);

					for (int i=0;i<clientCmd.m_removeObjectArgs.m_numBodies;i++)
					{
						int bodyUniqueId = clientCmd.m_removeObjectArgs.m_bodyUniqueIds[i];
						InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						if (bodyHandle)
						{
							if (bodyHandle->m_multiBody)
							{
								serverCmd.m_removeObjectArgs.m_bodyUniqueIds[serverCmd.m_removeObjectArgs.m_numBodies++] = bodyUniqueId;

								//also remove user constraints...
								for (int i=m_data->m_dynamicsWorld->getNumMultiBodyConstraints()-1;i>=0;i--)
								{
									btMultiBodyConstraint* mbc = m_data->m_dynamicsWorld->getMultiBodyConstraint(i);
									if ((mbc->getMultiBodyA() == bodyHandle->m_multiBody)||(mbc->getMultiBodyB()==bodyHandle->m_multiBody))
									{
										m_data->m_dynamicsWorld->removeMultiBodyConstraint(mbc);

										//also remove user constraint and submit it as removed
										for (int c=m_data->m_userConstraints.size()-1;c>=0;c--)
										{
											InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.getAtIndex(c);
											int userConstraintKey = m_data->m_userConstraints.getKeyAtIndex(c).getUid1();

											if (userConstraintPtr->m_mbConstraint == mbc)
											{
												m_data->m_userConstraints.remove(userConstraintKey);
												serverCmd.m_removeObjectArgs.m_userConstraintUniqueIds[serverCmd.m_removeObjectArgs.m_numUserConstraints++]=userConstraintKey;
											}
										}

										delete mbc;
										

									}
								}
								
								if (bodyHandle->m_multiBody->getBaseCollider())
								{
									m_data->m_visualConverter.removeVisualShape(bodyHandle->m_multiBody->getBaseCollider());
									m_data->m_dynamicsWorld->removeCollisionObject(bodyHandle->m_multiBody->getBaseCollider());
									int graphicsIndex = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
									m_data->m_guiHelper->removeGraphicsInstance(graphicsIndex);
								}
								for (int link=0;link<bodyHandle->m_multiBody->getNumLinks();link++)
								{
									
									if (bodyHandle->m_multiBody->getLink(link).m_collider)
									{
										m_data->m_visualConverter.removeVisualShape(bodyHandle->m_multiBody->getLink(link).m_collider);
										m_data->m_dynamicsWorld->removeCollisionObject(bodyHandle->m_multiBody->getLink(link).m_collider);
										int graphicsIndex = bodyHandle->m_multiBody->getLink(link).m_collider->getUserIndex();
										m_data->m_guiHelper->removeGraphicsInstance(graphicsIndex);
									}
								}
								int numCollisionObjects = m_data->m_dynamicsWorld->getNumCollisionObjects();
								m_data->m_dynamicsWorld->removeMultiBody(bodyHandle->m_multiBody);
								numCollisionObjects =  m_data->m_dynamicsWorld->getNumCollisionObjects();
								//todo: clear all other remaining data, release memory etc

								delete bodyHandle->m_multiBody;
								bodyHandle->m_multiBody=0;
								serverCmd.m_type = CMD_REMOVE_BODY_COMPLETED;
							}
							if (bodyHandle->m_rigidBody)
							{
								m_data->m_visualConverter.removeVisualShape(bodyHandle->m_rigidBody);
								serverCmd.m_removeObjectArgs.m_bodyUniqueIds[serverCmd.m_removeObjectArgs.m_numBodies++] = bodyUniqueId;
								//todo: clear all other remaining data...
								m_data->m_dynamicsWorld->removeRigidBody(bodyHandle->m_rigidBody);
								int graphicsInstance = bodyHandle->m_rigidBody->getUserIndex2();
								m_data->m_guiHelper->removeGraphicsInstance(graphicsInstance);
								delete bodyHandle->m_rigidBody;
								bodyHandle->m_rigidBody=0;
								serverCmd.m_type = CMD_REMOVE_BODY_COMPLETED;
							}
						}

						m_data->m_bodyHandles.freeHandle(bodyUniqueId);
					}
					m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL,1);


                    hasStatus = true;
					break;
				}
                case CMD_USER_CONSTRAINT:
                {
					BT_PROFILE("CMD_USER_CONSTRAINT");

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
						InternalBodyData* parentBody = m_data->m_bodyHandles.getHandle(clientCmd.m_userConstraintArguments.m_parentBodyIndex);
						if (parentBody && parentBody->m_multiBody)
						{
							if ((clientCmd.m_userConstraintArguments.m_parentJointIndex>=-1) && clientCmd.m_userConstraintArguments.m_parentJointIndex < parentBody->m_multiBody->getNumLinks())
							{
								InternalBodyData* childBody = clientCmd.m_userConstraintArguments.m_childBodyIndex>=0 ? m_data->m_bodyHandles.getHandle(clientCmd.m_userConstraintArguments.m_childBodyIndex):0;
								//also create a constraint with just a single multibody/rigid body without child
								//if (childBody)
								{
									btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
									btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);
									btMatrix3x3 frameInParent(btQuaternion(clientCmd.m_userConstraintArguments.m_parentFrame[3], clientCmd.m_userConstraintArguments.m_parentFrame[4], clientCmd.m_userConstraintArguments.m_parentFrame[5], clientCmd.m_userConstraintArguments.m_parentFrame[6]));
									btMatrix3x3 frameInChild(btQuaternion(clientCmd.m_userConstraintArguments.m_childFrame[3], clientCmd.m_userConstraintArguments.m_childFrame[4], clientCmd.m_userConstraintArguments.m_childFrame[5], clientCmd.m_userConstraintArguments.m_childFrame[6]));
									btVector3 jointAxis(clientCmd.m_userConstraintArguments.m_jointAxis[0], clientCmd.m_userConstraintArguments.m_jointAxis[1], clientCmd.m_userConstraintArguments.m_jointAxis[2]);
									

									
									if (clientCmd.m_userConstraintArguments.m_jointType == eGearType)
									{
										if (childBody && childBody->m_multiBody)
										{
											if ((clientCmd.m_userConstraintArguments.m_childJointIndex>=-1) && (clientCmd.m_userConstraintArguments.m_childJointIndex <childBody->m_multiBody->getNumLinks()))
											{
												btMultiBodyGearConstraint* multibodyGear = new btMultiBodyGearConstraint(parentBody->m_multiBody,clientCmd.m_userConstraintArguments.m_parentJointIndex,childBody->m_multiBody,clientCmd.m_userConstraintArguments.m_childJointIndex,pivotInParent,pivotInChild,frameInParent,frameInChild);
												multibodyGear->setMaxAppliedImpulse(defaultMaxForce);
												m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodyGear);
												InteralUserConstraintData userConstraintData;
												userConstraintData.m_mbConstraint = multibodyGear;
												int uid = m_data->m_userConstraintUIDGenerator++;
												serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
												serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
												userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
												m_data->m_userConstraints.insert(uid,userConstraintData);
												serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
											}
									
										}
									}
									else if (clientCmd.m_userConstraintArguments.m_jointType == eFixedType)
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
						else
						{
							InternalBodyData* childBody = clientCmd.m_userConstraintArguments.m_childBodyIndex>=0 ? m_data->m_bodyHandles.getHandle(clientCmd.m_userConstraintArguments.m_childBodyIndex):0;
						
							if (parentBody && childBody)
							{
								if (parentBody->m_rigidBody)
								{

									btRigidBody* parentRb = 0;
									if (clientCmd.m_userConstraintArguments.m_parentJointIndex==-1)
									{
										parentRb = parentBody->m_rigidBody;
									} else
									{
										if ((clientCmd.m_userConstraintArguments.m_parentJointIndex>=0) &&
											(clientCmd.m_userConstraintArguments.m_parentJointIndex<parentBody->m_rigidBodyJoints.size()))
										{
											parentRb = &parentBody->m_rigidBodyJoints[clientCmd.m_userConstraintArguments.m_parentJointIndex]->getRigidBodyB();
										}
									}
												

									btRigidBody* childRb = 0;
									if (childBody->m_rigidBody)
									{
											
										if (clientCmd.m_userConstraintArguments.m_childJointIndex==-1)
										{
											childRb = childBody->m_rigidBody;
										}
										else
										{
											if ((clientCmd.m_userConstraintArguments.m_childJointIndex>=0)
												&& (clientCmd.m_userConstraintArguments.m_childJointIndex<childBody->m_rigidBodyJoints.size()))
											{
												childRb = &childBody->m_rigidBodyJoints[clientCmd.m_userConstraintArguments.m_childJointIndex]->getRigidBodyB();
											}
													
										}
									}

									switch (clientCmd.m_userConstraintArguments.m_jointType)
									{
										case eRevoluteType:
										{
											break;
										}
										case ePrismaticType:
										{
											break;
										}
										
										case eFixedType:
										{
											if (childRb && parentRb && (childRb!=parentRb))
											{
												btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
												btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);

												btTransform offsetTrA,offsetTrB;
												offsetTrA.setIdentity();
												offsetTrA.setOrigin(pivotInParent);
												offsetTrB.setIdentity();
												offsetTrB.setOrigin(pivotInChild);

												btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRb, *childRb,  offsetTrA, offsetTrB);
												
												dof6->setLinearLowerLimit(btVector3(0,0,0));
												dof6->setLinearUpperLimit(btVector3(0,0,0));

												dof6->setAngularLowerLimit(btVector3(0,0,0));
												dof6->setAngularUpperLimit(btVector3(0,0,0));
												m_data->m_dynamicsWorld->addConstraint(dof6);
												InteralUserConstraintData userConstraintData;
												userConstraintData.m_rbConstraint = dof6;
												int uid = m_data->m_userConstraintUIDGenerator++;
												serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
												serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
												serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
												userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
												m_data->m_userConstraints.insert(uid,userConstraintData);
												serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
											}

											break;
										}
											
										case ePoint2PointType:
										{
											if (childRb && parentRb && (childRb!=parentRb))
											{
												btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
												btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);

												btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*parentRb,*childRb,pivotInParent,pivotInChild);
												p2p->m_setting.m_impulseClamp = defaultMaxForce;
												m_data->m_dynamicsWorld->addConstraint(p2p);
												InteralUserConstraintData userConstraintData;
												userConstraintData.m_rbConstraint = p2p;
												int uid = m_data->m_userConstraintUIDGenerator++;
												serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
												serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
												serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
												userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
												m_data->m_userConstraints.insert(uid,userConstraintData);
												serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
											}
											break;
										}
										
										case eGearType:
										{
											
											if (childRb && parentRb && (childRb!=parentRb))
											{
												btVector3 axisA(clientCmd.m_userConstraintArguments.m_jointAxis[0],
													clientCmd.m_userConstraintArguments.m_jointAxis[1],
													clientCmd.m_userConstraintArguments.m_jointAxis[2]);
												//for now we use the same local axis for both objects
												btVector3 axisB(clientCmd.m_userConstraintArguments.m_jointAxis[0],
													clientCmd.m_userConstraintArguments.m_jointAxis[1],
													clientCmd.m_userConstraintArguments.m_jointAxis[2]);
												btScalar ratio=1;
												btGearConstraint* gear = new btGearConstraint(*parentRb,*childRb, axisA,axisB,ratio);
												m_data->m_dynamicsWorld->addConstraint(gear,true);
												InteralUserConstraintData userConstraintData;
												userConstraintData.m_rbConstraint = gear;
												int uid = m_data->m_userConstraintUIDGenerator++;
												serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
												serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
												serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
												userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
												m_data->m_userConstraints.insert(uid,userConstraintData);
												serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
											}
											break;
										}
										case eSphericalType:
										{
											b3Warning("constraint type not handled yet");
											break;
										}
										case ePlanarType:
										{
											b3Warning("constraint type not handled yet");
											break;
										}
									default:
										{
											b3Warning("unknown constraint type");
										}
									};
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

								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
								{
									userConstraintPtr->m_mbConstraint->setGearRatio(clientCmd.m_userConstraintArguments.m_gearRatio);
								}
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET)
								{
									userConstraintPtr->m_mbConstraint->setRelativePositionTarget(clientCmd.m_userConstraintArguments.m_relativePositionTarget);
								}
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_ERP)
								{
									userConstraintPtr->m_mbConstraint->setErp(clientCmd.m_userConstraintArguments.m_erp);
								}

								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK)
								{
									userConstraintPtr->m_mbConstraint->setGearAuxLink(clientCmd.m_userConstraintArguments.m_gearAuxLink);
								}

							}
							if (userConstraintPtr->m_rbConstraint)
							{
								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
								{
									btScalar maxImp = clientCmd.m_userConstraintArguments.m_maxAppliedForce*m_data->m_physicsDeltaTime;
									userConstraintPtr->m_userConstraintData.m_maxAppliedForce = clientCmd.m_userConstraintArguments.m_maxAppliedForce;
									//userConstraintPtr->m_rbConstraint->setMaxAppliedImpulse(maxImp);
								}

								if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
								{
									if (userConstraintPtr->m_rbConstraint->getObjectType()==GEAR_CONSTRAINT_TYPE)
									{
										btGearConstraint* gear = (btGearConstraint*) userConstraintPtr->m_rbConstraint;
										gear->setRatio(clientCmd.m_userConstraintArguments.m_gearRatio);
									}
								}
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
								m_data->m_dynamicsWorld->removeConstraint(userConstraintPtr->m_rbConstraint);
								delete userConstraintPtr->m_rbConstraint;
								m_data->m_userConstraints.remove(userConstraintUidRemove);	
							}
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = userConstraintUidRemove;
							serverCmd.m_type = CMD_REMOVE_USER_CONSTRAINT_COMPLETED;

                            
						}

						
					}
					
                    break;
                }
				case CMD_CALCULATE_INVERSE_KINEMATICS:
					{
						BT_PROFILE("CMD_CALCULATE_INVERSE_KINEMATICS");
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_CALCULATE_INVERSE_KINEMATICS_FAILED;

						InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_calculateInverseKinematicsArguments.m_bodyUniqueId);
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
	                                    // Note that inverse dynamics uses zero-based indexing of bodies, not starting from -1 for the base link.
                                        tree->getBodyJacobianTrans(endEffectorLinkIndex+1, &jac_t);
                                        tree->getBodyJacobianRot(endEffectorLinkIndex+1, &jac_r);
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
                    BT_PROFILE("CMD_REQUEST_VISUAL_SHAPE_INFO");
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type = CMD_VISUAL_SHAPE_INFO_FAILED;
                    //retrieve the visual shape information for a specific body
                    
					int totalNumVisualShapes = m_data->m_visualConverter.getNumVisualShapes(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId);
					//int totalBytesPerVisualShape = sizeof (b3VisualShapeData);
					//int visualShapeStorage = bufferSizeInBytes / totalBytesPerVisualShape - 1;
					b3VisualShapeData* visualShapeStoragePtr = (b3VisualShapeData*)bufferServerToClient;

					int remain = totalNumVisualShapes - clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
					int shapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;

					int success = m_data->m_visualConverter.getVisualShapesData(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId,
						shapeIndex,
						visualShapeStoragePtr);
					if (success) {
						serverCmd.m_sendVisualShapeArgs.m_numRemainingVisualShapes = remain-1;
						serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied = 1;
						serverCmd.m_sendVisualShapeArgs.m_startingVisualShapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
						serverCmd.m_sendVisualShapeArgs.m_bodyUniqueId = clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId;
						serverCmd.m_numDataStreamBytes = sizeof(b3VisualShapeData)*serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied;
						serverCmd.m_type = CMD_VISUAL_SHAPE_INFO_COMPLETED;
					}
					hasStatus = true;
					break;
                }
                case CMD_UPDATE_VISUAL_SHAPE:
                {
					BT_PROFILE("CMD_UPDATE_VISUAL_SHAPE");
                    SharedMemoryStatus& serverCmd = serverStatusOut;
                    serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_FAILED;
                    InternalTextureHandle* texHandle = 0;
					
					if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE) 
					{
						texHandle = m_data->m_textureHandles.getHandle(clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId);

						if (clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId>=0)
						{
							if (texHandle)
							{
			                    m_data->m_visualConverter.activateShapeTexture(clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId, clientCmd.m_updateVisualShapeDataArguments.m_jointIndex, clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex, texHandle->m_tinyRendererTextureId);
							}
						}
					}                    
   
					{
						int bodyUniqueId = clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId;
						int linkIndex = clientCmd.m_updateVisualShapeDataArguments.m_jointIndex;

						InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
						if (bodyHandle)
						{
							if (bodyHandle->m_multiBody)
							{
								if (linkIndex==-1)
								{
									if (bodyHandle->m_multiBody->getBaseCollider())
									{
										int graphicsIndex = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
										if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE) 
										{
											if (texHandle)
											{
												int shapeIndex = m_data->m_guiHelper->getShapeIndexFromInstance(graphicsIndex);
												m_data->m_guiHelper->replaceTexture(shapeIndex,texHandle->m_openglTextureId);
											}
										}
										if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR) 
										{
											m_data->m_visualConverter.changeRGBAColor(bodyUniqueId,linkIndex,clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
											m_data->m_guiHelper->changeRGBAColor(graphicsIndex,clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
										}
										if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR) 
										{
											m_data->m_guiHelper->changeSpecularColor(graphicsIndex,clientCmd.m_updateVisualShapeDataArguments.m_specularColor);
										}
										
									}
								} else
								{
									if (linkIndex<bodyHandle->m_multiBody->getNumLinks())
									{
										if (bodyHandle->m_multiBody->getLink(linkIndex).m_collider)
										{
											int graphicsIndex = bodyHandle->m_multiBody->getLink(linkIndex).m_collider->getUserIndex();
											if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE) 
											{
												if (texHandle)
												{
													int shapeIndex = m_data->m_guiHelper->getShapeIndexFromInstance(graphicsIndex);
													m_data->m_guiHelper->replaceTexture(shapeIndex,texHandle->m_openglTextureId);
												}
											}
											if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR) 
											{
												m_data->m_visualConverter.changeRGBAColor(bodyUniqueId,linkIndex,clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);										
												m_data->m_guiHelper->changeRGBAColor(graphicsIndex,clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
											}
											if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR) 
											{
												m_data->m_guiHelper->changeSpecularColor(graphicsIndex,clientCmd.m_updateVisualShapeDataArguments.m_specularColor);
											}

										}
									}
								}
							} else
							{
								//todo: change color for rigid body
							}
						}
					}
	
					serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_COMPLETED;
                    hasStatus = true;

                    break;
                }

				case CMD_CHANGE_TEXTURE:
				{
					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverCmd.m_type = CMD_CHANGE_TEXTURE_COMMAND_FAILED;
					
					InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(clientCmd.m_changeTextureArgs.m_textureUniqueId);
					if(texH)
					{
						int gltex = texH->m_openglTextureId;
						m_data->m_guiHelper->changeTexture(gltex,
							(const unsigned char*)bufferServerToClient, clientCmd.m_changeTextureArgs.m_width,clientCmd.m_changeTextureArgs.m_height);

						serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
					}
                    hasStatus = true;
                    break;
				}
				case CMD_LOAD_TEXTURE:
				{
					BT_PROFILE("CMD_LOAD_TEXTURE");
					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverCmd.m_type = CMD_LOAD_TEXTURE_FAILED;

					char relativeFileName[1024];
					char pathPrefix[1024];

					if(b3ResourcePath::findResourcePath(clientCmd.m_loadTextureArguments.m_textureFileName,relativeFileName,1024))
					{
						b3FileUtils::extractPath(relativeFileName,pathPrefix,1024);

						int texHandle = m_data->m_textureHandles.allocHandle();
						InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(texHandle);
						if(texH)
						{
							texH->m_tinyRendererTextureId = -1;
							texH->m_openglTextureId = -1;

							int uid = m_data->m_visualConverter.loadTextureFile(relativeFileName);
							if(uid>=0)
							{
								texH->m_tinyRendererTextureId = uid;
							}

							{
								int width,height,n;
								unsigned char* imageData= stbi_load(relativeFileName,&width,&height,&n,3);

								if(imageData)
								{
									texH->m_openglTextureId = m_data->m_guiHelper->registerTexture(imageData,width,height);
									free(imageData);
								}
								else
								{
									b3Warning("Unsupported texture image format [%s]\n",relativeFileName);
								}
							}
							serverCmd.m_loadTextureResultArguments.m_textureUniqueId = texHandle;
							serverCmd.m_type = CMD_LOAD_TEXTURE_COMPLETED;
						}
					}
					else
					{
						serverCmd.m_type = CMD_LOAD_TEXTURE_FAILED;
					}
					hasStatus = true;

					break;
				}

				case CMD_LOAD_BULLET:
				{
					BT_PROFILE("CMD_LOAD_BULLET");
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
										int bodyUniqueId = m_data->m_bodyHandles.allocHandle();
										InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
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
					BT_PROFILE("CMD_SAVE_BULLET");
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
					BT_PROFILE("CMD_LOAD_MJCF");
					SharedMemoryStatus& serverCmd = serverStatusOut;
					serverCmd.m_type = CMD_MJCF_LOADING_FAILED;
					  const MjcfArgs& mjcfArgs = clientCmd.m_mjcfArguments;
                        if (m_data->m_verboseOutput)
                        {
                            b3Printf("Processed CMD_LOAD_MJCF:%s", mjcfArgs.m_mjcfFileName);
                        }
                        bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (mjcfArgs.m_useMultiBody!=0) : true;
						int flags = CUF_USE_MJCF;
						if (clientCmd.m_updateFlags&URDF_ARGS_HAS_CUSTOM_URDF_FLAGS)
						{
							flags |= clientCmd.m_mjcfArguments.m_flags;
						}

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
					BT_PROFILE("CMD_USER_DEBUG_DRAW");
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_USER_DEBUG_DRAW_FAILED;
						hasStatus = true;


						int trackingVisualShapeIndex = -1;

						if (clientCmd.m_userDebugDrawArgs.m_parentObjectUniqueId>=0)
						{
							InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_userDebugDrawArgs.m_parentObjectUniqueId);
							if (bodyHandle && bodyHandle->m_multiBody)
							{
								int linkIndex = clientCmd.m_userDebugDrawArgs.m_parentLinkIndex;
								if (linkIndex ==-1)
								{
									if (bodyHandle->m_multiBody->getBaseCollider())
									{
										trackingVisualShapeIndex  = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
									}
								} else
								{
									if (linkIndex >=0 && linkIndex < bodyHandle->m_multiBody->getNumLinks())
									{
										if (bodyHandle->m_multiBody->getLink(linkIndex).m_collider)
										{
											trackingVisualShapeIndex  = bodyHandle->m_multiBody->getLink(linkIndex).m_collider->getUserIndex();
										}
									}

								}

							}
						}

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
							InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
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
								//addUserDebugText3D( const double orientation[4], const double	textColorRGB[3], double size, double lifeTime, int trackingObjectUniqueId, int optionFlags){return -1;}

							int optionFlags = clientCmd.m_userDebugDrawArgs.m_optionFlags;

							if (clientCmd.m_updateFlags & USER_DEBUG_HAS_TEXT_ORIENTATION)
							{
								optionFlags |= DEB_DEBUG_TEXT_USE_ORIENTATION;
							}


							


							int uid = m_data->m_guiHelper->addUserDebugText3D(clientCmd.m_userDebugDrawArgs.m_text,
								clientCmd.m_userDebugDrawArgs.m_textPositionXYZ,
								clientCmd.m_userDebugDrawArgs.m_textOrientation,
								clientCmd.m_userDebugDrawArgs.m_textColorRGB,
								clientCmd.m_userDebugDrawArgs.m_textSize,
								clientCmd.m_userDebugDrawArgs.m_lifeTime,
								trackingVisualShapeIndex,
								optionFlags);

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
								clientCmd.m_userDebugDrawArgs.m_lifeTime,
								trackingVisualShapeIndex);

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
					case CMD_CUSTOM_COMMAND:
					{
						SharedMemoryStatus& serverCmd = serverStatusOut;
						serverCmd.m_type = CMD_CUSTOM_COMMAND_FAILED;
						serverCmd.m_customCommandResultArgs.m_pluginUniqueId = -1;

						if (clientCmd.m_updateFlags & CMD_CUSTOM_COMMAND_LOAD_PLUGIN)
						{
							//pluginPath could be registered or load from disk
							int pluginUniqueId = m_data->m_pluginManager.loadPlugin(clientCmd.m_customCommandArgs.m_pluginPath);
							if (pluginUniqueId>=0)
							{
								serverCmd.m_customCommandResultArgs.m_pluginUniqueId = pluginUniqueId;
								serverCmd.m_type = CMD_CUSTOM_COMMAND_COMPLETED;
							}
						}
						if (clientCmd.m_updateFlags & CMD_CUSTOM_COMMAND_UNLOAD_PLUGIN)
						{
							m_data->m_pluginManager.unloadPlugin(clientCmd.m_customCommandArgs.m_pluginUniqueId);
							serverCmd.m_type = CMD_CUSTOM_COMMAND_COMPLETED;
						}
						if (clientCmd.m_updateFlags & CMD_CUSTOM_COMMAND_EXECUTE_PLUGIN_COMMAND)
						{
							
							int result = m_data->m_pluginManager.executePluginCommand(clientCmd.m_customCommandArgs.m_pluginUniqueId, &clientCmd.m_customCommandArgs.m_arguments);
							serverCmd.m_customCommandResultArgs.m_executeCommandResult = result;
							serverCmd.m_type = CMD_CUSTOM_COMMAND_COMPLETED;

						}

						hasStatus = true;
						break;
					}
                default:
                {
					BT_PROFILE("CMD_UNKNOWN");
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

void PhysicsServerCommandProcessor::syncPhysicsToGraphics()
{
	m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
}


void PhysicsServerCommandProcessor::renderScene(int renderFlags)
{
	if (m_data->m_guiHelper)
	{
		if (0==(renderFlags&COV_DISABLE_SYNC_RENDERING))		
 		{		
 			m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);		
 		}

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
                m_data->m_savedActivationState = body->getActivationState();
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
		m_data->m_pickedBody->forceActivationState(m_data->m_savedActivationState);
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






int gDroppedSimulationSteps = 0;
int gNumSteps = 0;
double gDtInSec = 0.f;
double gSubStep = 0.f;

void PhysicsServerCommandProcessor::enableRealTimeSimulation(bool enableRealTimeSim)
{
	m_data->m_allowRealTimeSimulation = enableRealTimeSim;
}

bool PhysicsServerCommandProcessor::isRealTimeSimulationEnabled() const
{
	return 	m_data->m_allowRealTimeSimulation;
}

void PhysicsServerCommandProcessor::stepSimulationRealTime(double dtInSec,const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents)
{
	m_data->m_vrControllerEvents.addNewVREvents(vrControllerEvents,numVRControllerEvents);


	for (int i=0;i<m_data->m_stateLoggers.size();i++)
	{
		if (m_data->m_stateLoggers[i]->m_loggingType==STATE_LOGGING_VR_CONTROLLERS)
		{
			VRControllerStateLogger* vrLogger = (VRControllerStateLogger*) m_data->m_stateLoggers[i];
			vrLogger->m_vrEvents.addNewVREvents(vrControllerEvents,numVRControllerEvents);
		}
	}

	for (int ii=0;ii<numMouseEvents;ii++)
	{
		const b3MouseEvent& event = mouseEvents[ii];
		bool found = false;
		//search a matching one first, otherwise add new event
		for (int e=0;e<m_data->m_mouseEvents.size();e++)
		{
			if (event.m_eventType == m_data->m_mouseEvents[e].m_eventType)
			{
				if (event.m_eventType == MOUSE_MOVE_EVENT)
				{
					m_data->m_mouseEvents[e].m_mousePosX = event.m_mousePosX;
					m_data->m_mouseEvents[e].m_mousePosY = event.m_mousePosY;
					found = true;
				} else
				if ((event.m_eventType == MOUSE_BUTTON_EVENT) && event.m_buttonIndex == m_data->m_mouseEvents[e].m_buttonIndex)
				{
					m_data->m_mouseEvents[e].m_buttonState |= event.m_buttonState;
					if (event.m_buttonState & eButtonIsDown)
					{
						m_data->m_mouseEvents[e].m_buttonState |= eButtonIsDown;
					} else
					{
						m_data->m_mouseEvents[e].m_buttonState &= ~eButtonIsDown;
					}
					found = true;
				}
			}
		}	
		if (!found)
		{
			m_data->m_mouseEvents.push_back(event);
		}
	}

	for (int i=0;i<numKeyEvents;i++)
	{
		const b3KeyboardEvent& event = keyEvents[i];
		bool found = false;
		//search a matching one first, otherwise add new event
		for (int e=0;e<m_data->m_keyboardEvents.size();e++)
		{
			if (event.m_keyCode == m_data->m_keyboardEvents[e].m_keyCode)
			{
				m_data->m_keyboardEvents[e].m_keyState |= event.m_keyState;
				if (event.m_keyState & eButtonIsDown)
				{
					m_data->m_keyboardEvents[e].m_keyState |= eButtonIsDown;
				} else
				{
					m_data->m_keyboardEvents[e].m_keyState &= ~eButtonIsDown;
				}
				found=true;
			}
		}
		if (!found)
		{
			m_data->m_keyboardEvents.push_back(event);
		}
	}
	if (gResetSimulation)
	{
		resetSimulation();
		gResetSimulation = false;
	}

	if (gVRTrackingObjectUniqueId >= 0)
	{
		InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(gVRTrackingObjectUniqueId);
		if (bodyHandle && bodyHandle->m_multiBody)
		{
//			gVRTrackingObjectTr  = bodyHandle->m_multiBody->getBaseWorldTransform();

			if (gVRTrackingObjectUniqueId>=0)
			{
				gVRTrackingObjectTr.setOrigin(bodyHandle->m_multiBody->getBaseWorldTransform().getOrigin());
				gVRTeleportPos1 = gVRTrackingObjectTr.getOrigin();
			}
			if (gVRTrackingObjectFlag&VR_CAMERA_TRACK_OBJECT_ORIENTATION)
			{
				gVRTrackingObjectTr.setBasis(bodyHandle->m_multiBody->getBaseWorldTransform().getBasis());
				gVRTeleportOrn = gVRTrackingObjectTr.getRotation();

			}	

		}
	}

	if ((m_data->m_allowRealTimeSimulation) && m_data->m_guiHelper)
	{
		
		
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



void PhysicsServerCommandProcessor::resetSimulation()
{
	//clean up all data

	if (m_data && m_data->m_guiHelper)
	{
		m_data->m_guiHelper->removeAllGraphicsInstances();
		m_data->m_guiHelper->removeAllUserDebugItems();
	}
	if (m_data)
	{
		m_data->m_visualConverter.resetAll();
	}

	removePickingConstraint();

	deleteDynamicsWorld();
	createEmptyDynamicsWorld();

	m_data->m_bodyHandles.exitHandles();
	m_data->m_bodyHandles.initHandles();

	m_data->m_userCollisionShapeHandles.exitHandles();
	m_data->m_userCollisionShapeHandles.initHandles();

}


void PhysicsServerCommandProcessor::setTimeOut(double /*timeOutInSeconds*/)
{
}

const btVector3& PhysicsServerCommandProcessor::getVRTeleportPosition() const
{
	return gVRTeleportPos1;
}
void PhysicsServerCommandProcessor::setVRTeleportPosition(const btVector3& vrTeleportPos)
{
	gVRTeleportPos1 = vrTeleportPos;
}
const btQuaternion& PhysicsServerCommandProcessor::getVRTeleportOrientation() const
{
	return gVRTeleportOrn;
}
void PhysicsServerCommandProcessor::setVRTeleportOrientation(const btQuaternion& vrTeleportOrn)
{
	gVRTeleportOrn = vrTeleportOrn;
}
