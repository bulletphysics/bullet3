#include "PhysicsServerCommandProcessor.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "plugins/b3PluginCollisionInterface.h"
#include "../Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/UrdfFindMeshFile.h"

#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"

#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "../Importers/ImportMeshUtility/b3ImportMeshUtility.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h"


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
#include "../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "LinearMath/btSerializer.h"
#include "Bullet3Common/b3Logging.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryCommands.h"
#include "LinearMath/btRandom.h"
#include "Bullet3Common/b3ResizablePool.h"
#include "../Utils/b3Clock.h"
#include "b3PluginManager.h"
#include "../Extras/Serialize/BulletFileLoader/btBulletFile.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "LinearMath/TaskScheduler/btThreadSupportInterface.h"

#ifndef SKIP_COLLISION_FILTER_PLUGIN
#include "plugins/collisionFilterPlugin/collisionFilterPlugin.h"
#endif


#ifdef ENABLE_STATIC_GRPC_PLUGIN
#include "plugins/grpcPlugin/grpcPlugin.h"
#endif  //ENABLE_STATIC_GRPC_PLUGIN

#ifndef SKIP_STATIC_PD_CONTROL_PLUGIN
#include "plugins/pdControlPlugin/pdControlPlugin.h"
#endif  //SKIP_STATIC_PD_CONTROL_PLUGIN

#ifdef STATIC_LINK_VR_PLUGIN
#include "plugins/vrSyncPlugin/vrSyncPlugin.h"
#endif

#ifdef STATIC_EGLRENDERER_PLUGIN
#include "plugins/eglPlugin/eglRendererPlugin.h"
#endif  //STATIC_EGLRENDERER_PLUGIN

#ifndef SKIP_STATIC_TINYRENDERER_PLUGIN
#include "plugins/tinyRendererPlugin/tinyRendererPlugin.h"
#endif


#ifdef B3_ENABLE_FILEIO_PLUGIN
#include "plugins/fileIOPlugin/fileIOPlugin.h"
#endif//B3_DISABLE_FILEIO_PLUGIN

#ifdef B3_ENABLE_TINY_AUDIO
#include "../TinyAudio/b3SoundEngine.h"
#endif

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftMultiBodyDynamicsWorld.h"
#include "../SoftDemo/BunnyMesh.h"
#else
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#endif


int gInternalSimFlags = 0;
bool gResetSimulation = 0;
int gVRTrackingObjectUniqueId = -1;
int gVRTrackingObjectFlag = VR_CAMERA_TRACK_OBJECT_ORIENTATION;

btTransform gVRTrackingObjectTr = btTransform::getIdentity();

btVector3 gVRTeleportPos1(0, 0, 0);
btQuaternion gVRTeleportOrn(0, 0, 0, 1);

btScalar simTimeScalingFactor = 1;
btScalar gRhsClamp = 1.f;

#include "../CommonInterfaces/CommonFileIOInterface.h"




struct UrdfLinkNameMapUtil
{
	btMultiBody* m_mb;
	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_rigidBodyJoints;

	btDefaultSerializer* m_memSerializer;

	UrdfLinkNameMapUtil() : m_mb(0), m_memSerializer(0)
	{
	}
	virtual ~UrdfLinkNameMapUtil()
	{
	}
};

class b3ThreadPool
{
public:
	b3ThreadPool(const char* name = "b3ThreadPool")
	{
		btThreadSupportInterface::ConstructionInfo info(name, threadFunction);
		m_threadSupportInterface = btThreadSupportInterface::create(info);
	}

	~b3ThreadPool()
	{
		delete m_threadSupportInterface;
	}

	const int numWorkers() const { return m_threadSupportInterface->getNumWorkerThreads(); }

	void runTask(int threadIdx, btThreadSupportInterface::ThreadFunc func, void* arg)
	{
		FunctionContext& ctx = m_functionContexts[threadIdx];
		ctx.func = func;
		ctx.arg = arg;
		m_threadSupportInterface->runTask(threadIdx, (void*)&ctx);
	}

	void waitForAllTasks()
	{
		BT_PROFILE("b3ThreadPool_waitForAllTasks");
		m_threadSupportInterface->waitForAllTasks();
	}

private:
	struct FunctionContext
	{
		btThreadSupportInterface::ThreadFunc func;
		void* arg;
	};

	static void threadFunction(void* userPtr)
	{
		BT_PROFILE("b3ThreadPool_threadFunction");
		FunctionContext* ctx = (FunctionContext*)userPtr;
		ctx->func(ctx->arg);
	}

	btThreadSupportInterface* m_threadSupportInterface;
	FunctionContext m_functionContexts[BT_MAX_THREAD_COUNT];
};

struct SharedMemoryDebugDrawer : public btIDebugDraw
{
	int m_debugMode;
	btAlignedObjectArray<SharedMemLines> m_lines2;

	SharedMemoryDebugDrawer()
		: m_debugMode(0)
	{
	}
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
	{
	}

	virtual void reportErrorWarning(const char* warningString)
	{
	}

	virtual void draw3dText(const btVector3& location, const char* textString)
	{
	}

	virtual void setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}

	virtual int getDebugMode() const
	{
		return m_debugMode;
	}
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		SharedMemLines line;
		line.m_from = from;
		line.m_to = to;
		line.m_color = color;
		m_lines2.push_back(line);
	}
};

struct InternalVisualShapeData
{
	int m_tinyRendererVisualShapeIndex;
	int m_OpenGLGraphicsIndex;

	b3AlignedObjectArray<UrdfVisual> m_visualShapes;

	b3AlignedObjectArray<std::string> m_pathPrefixes;

	void clear()
	{
		m_tinyRendererVisualShapeIndex = -1;
		m_OpenGLGraphicsIndex = -1;
		m_visualShapes.clear();
		m_pathPrefixes.clear();
	}
};

struct InternalCollisionShapeData
{
	btCollisionShape* m_collisionShape;
	b3AlignedObjectArray<UrdfCollision> m_urdfCollisionObjects;
	int m_used;
	InternalCollisionShapeData()
		: m_collisionShape(0),
		  m_used(0)
	{
	}
	void clear()
	{
		m_collisionShape = 0;
		m_used = 0;
	}
};

#include "SharedMemoryUserData.h"

struct InternalBodyData
{
	btMultiBody* m_multiBody;
	btRigidBody* m_rigidBody;
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	btSoftBody* m_softBody;
#endif
	int m_testData;
	std::string m_bodyName;

	btTransform m_rootLocalInertialFrame;
	btAlignedObjectArray<btTransform> m_linkLocalInertialFrames;
	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_rigidBodyJoints;
	btAlignedObjectArray<std::string> m_rigidBodyJointNames;
	btAlignedObjectArray<std::string> m_rigidBodyLinkNames;
	btAlignedObjectArray<int> m_userDataHandles;

#ifdef B3_ENABLE_TINY_AUDIO
	b3HashMap<btHashInt, SDFAudioSource> m_audioSources;
#endif  //B3_ENABLE_TINY_AUDIO

	InternalBodyData()
	{
		clear();
	}

	void clear()
	{
		m_multiBody = 0;
		m_rigidBody = 0;
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
		m_softBody = 0;
#endif
		m_testData = 0;
		m_bodyName = "";
		m_rootLocalInertialFrame.setIdentity();
		m_linkLocalInertialFrames.clear();
		m_rigidBodyJoints.clear();
		m_rigidBodyJointNames.clear();
		m_rigidBodyLinkNames.clear();
		m_userDataHandles.clear();
	}
};

struct InteralUserConstraintData
{
	btTypedConstraint* m_rbConstraint;
	btMultiBodyConstraint* m_mbConstraint;

	b3UserConstraint m_userConstraintData;

	InteralUserConstraintData()
		: m_rbConstraint(0),
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
typedef b3PoolBodyHandle<InternalVisualShapeData> InternalVisualShapeHandle;

class btCommandChunk
{
public:
	int m_chunkCode;
	int m_length;
	void* m_oldPtr;
	int m_dna_nr;
	int m_number;
};

class bCommandChunkPtr4
{
public:
	bCommandChunkPtr4() {}
	int code;
	int len;
	union {
		int m_uniqueInt;
	};
	int dna_nr;
	int nr;
};

// ----------------------------------------------------- //
class bCommandChunkPtr8
{
public:
	bCommandChunkPtr8() {}
	int code, len;
	union {
		int m_uniqueInts[2];
	};
	int dna_nr, nr;
};

struct CommandLogger
{
	FILE* m_file;

	void writeHeader(unsigned char* buffer) const
	{
#ifdef BT_USE_DOUBLE_PRECISION
		memcpy(buffer, "BT3CMDd", 7);
#else
		memcpy(buffer, "BT3CMDf", 7);
#endif  //BT_USE_DOUBLE_PRECISION

		int littleEndian = 1;
		littleEndian = ((char*)&littleEndian)[0];

		if (sizeof(void*) == 8)
		{
			buffer[7] = '-';
		}
		else
		{
			buffer[7] = '_';
		}

		if (littleEndian)
		{
			buffer[8] = 'v';
		}
		else
		{
			buffer[8] = 'V';
		}

		buffer[9] = 0;
		buffer[10] = 0;
		buffer[11] = 0;

		int ver = btGetVersion();
		if (ver >= 0 && ver < 999)
		{
			sprintf((char*)&buffer[9], "%d", ver);
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
			fwrite((const char*)&chunk, sizeof(btCommandChunk), 1, m_file);

			switch (command.m_type)
			{
				case CMD_LOAD_MJCF:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_mjcfArguments, sizeof(MjcfArgs), 1, m_file);
					break;
				}
				case CMD_REQUEST_BODY_INFO:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_sdfRequestInfoArgs, sizeof(SdfRequestInfoArgs), 1, m_file);
					break;
				}
				case CMD_REQUEST_VISUAL_SHAPE_INFO:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_requestVisualShapeDataArguments, sizeof(RequestVisualShapeDataArgs), 1, m_file);
					break;
				}
				case CMD_LOAD_URDF:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_urdfArguments, sizeof(UrdfArgs), 1, m_file);
					break;
				}
				case CMD_INIT_POSE:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_initPoseArgs, sizeof(InitPoseArgs), 1, m_file);
					break;
				};
				case CMD_REQUEST_ACTUAL_STATE:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_requestActualStateInformationCommandArgument,
						   sizeof(RequestActualStateArgs), 1, m_file);
					break;
				};
				case CMD_SEND_DESIRED_STATE:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_sendDesiredStateCommandArgument, sizeof(SendDesiredStateArgs), 1, m_file);
					break;
				}
				case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_physSimParamArgs, sizeof(b3PhysicsSimulationParameters), 1, m_file);
					break;
				}
				case CMD_REQUEST_CONTACT_POINT_INFORMATION:
				{
					fwrite((const char*)&command.m_updateFlags, sizeof(int), 1, m_file);
					fwrite((const char*)&command.m_requestContactPointArguments, sizeof(RequestContactDataArgs), 1, m_file);
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
					fwrite((const char*)&command, sizeof(SharedMemoryCommand), 1, m_file);
				}
			};
		}
	}

	CommandLogger(const char* fileName)
	{
		m_file = fopen(fileName, "wb");
		if (m_file)
		{
			unsigned char buf[15];
			buf[12] = 12;
			buf[13] = 13;
			buf[14] = 14;
			writeHeader(buf);
			fwrite(buf, 12, 1, m_file);
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
		m_file = fopen(fileName, "rb");
		if (m_file)
		{
			size_t bytesRead;
			bytesRead = fread(m_header, 12, 1, m_file);
		}
		unsigned char c = m_header[7];
		m_fileIs64bit = (c == '-');

		const bool VOID_IS_8 = ((sizeof(void*) == 8));
		m_bitsVary = (VOID_IS_8 != m_fileIs64bit);
	}
	virtual ~CommandLogPlayback()
	{
		if (m_file)
		{
			fclose(m_file);
			m_file = 0;
		}
	}
	bool processNextCommand(SharedMemoryCommand* cmd)
	{
//for a little while, keep this flag to be able to read 'old' log files
//#define BACKWARD_COMPAT
#if BACKWARD_COMPAT
		SharedMemoryCommand unused;
#endif  //BACKWARD_COMPAT
		bool result = false;
		size_t s = 0;
		if (m_file)
		{
			int commandType = -1;

			if (m_fileIs64bit)
			{
				bCommandChunkPtr8 chunk8;
				s = fread((void*)&chunk8, sizeof(bCommandChunkPtr8), 1, m_file);
				commandType = chunk8.code;
			}
			else
			{
				bCommandChunkPtr4 chunk4;
				s = fread((void*)&chunk4, sizeof(bCommandChunkPtr4), 1, m_file);
				commandType = chunk4.code;
			}

			if (s == 1)
			{
				memset(cmd, 0, sizeof(SharedMemoryCommand));
				cmd->m_type = commandType;

#ifdef BACKWARD_COMPAT
				s = fread(&unused, sizeof(SharedMemoryCommand), 1, m_file);
				cmd->m_updateFlags = unused.m_updateFlags;
#endif

				switch (commandType)
				{
					case CMD_LOAD_MJCF:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_mjcfArguments = unused.m_mjcfArguments;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_mjcfArguments, sizeof(MjcfArgs), 1, m_file);
#endif
						result = true;
						break;
					}
					case CMD_REQUEST_BODY_INFO:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_sdfRequestInfoArgs = unused.m_sdfRequestInfoArgs;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_sdfRequestInfoArgs, sizeof(SdfRequestInfoArgs), 1, m_file);
#endif
						result = true;
						break;
					}
					case CMD_REQUEST_VISUAL_SHAPE_INFO:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_requestVisualShapeDataArguments = unused.m_requestVisualShapeDataArguments;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_requestVisualShapeDataArguments, sizeof(RequestVisualShapeDataArgs), 1, m_file);
#endif
						result = true;
						break;
					}
					case CMD_LOAD_URDF:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_urdfArguments = unused.m_urdfArguments;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_urdfArguments, sizeof(UrdfArgs), 1, m_file);
#endif
						result = true;
						break;
					}
					case CMD_INIT_POSE:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_initPoseArgs = unused.m_initPoseArgs;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_initPoseArgs, sizeof(InitPoseArgs), 1, m_file);

#endif
						result = true;
						break;
					};
					case CMD_REQUEST_ACTUAL_STATE:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_requestActualStateInformationCommandArgument = unused.m_requestActualStateInformationCommandArgument;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_requestActualStateInformationCommandArgument, sizeof(RequestActualStateArgs), 1, m_file);
#endif
						result = true;
						break;
					};
					case CMD_SEND_DESIRED_STATE:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_sendDesiredStateCommandArgument = unused.m_sendDesiredStateCommandArgument;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_sendDesiredStateCommandArgument, sizeof(SendDesiredStateArgs), 1, m_file);

#endif
						result = true;
						break;
					}
					case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_physSimParamArgs = unused.m_physSimParamArgs;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_physSimParamArgs, sizeof(b3PhysicsSimulationParameters), 1, m_file);

#endif
						result = true;
						break;
					}
					case CMD_REQUEST_CONTACT_POINT_INFORMATION:
					{
#ifdef BACKWARD_COMPAT
						cmd->m_requestContactPointArguments = unused.m_requestContactPointArguments;
#else
						s = fread(&cmd->m_updateFlags, sizeof(int), 1, m_file);
						s = fread(&cmd->m_requestContactPointArguments, sizeof(RequestContactDataArgs), 1, m_file);

#endif
						result = true;
						break;
					}
					case CMD_STEP_FORWARD_SIMULATION:
					case CMD_RESET_SIMULATION:
					case CMD_REQUEST_INTERNAL_DATA:
					{
						result = true;
						break;
					}
					default:
					{
						s = fread(cmd, sizeof(SharedMemoryCommand), 1, m_file);
						result = (s == 1);
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
	std::string m_fileName;
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
	virtual bool process(const btBroadphaseProxy* proxy)
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

struct MyOverlapFilterCallback : public btOverlapFilterCallback
{
	int m_filterMode;
	b3PluginManager* m_pluginManager;

	MyOverlapFilterCallback(b3PluginManager* pluginManager)
		: m_filterMode(B3_FILTER_GROUPAMASKB_AND_GROUPBMASKA),
		  m_pluginManager(pluginManager)
	{
	}

	virtual ~MyOverlapFilterCallback()
	{
	}
	// return true when pairs need collision
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	{
		b3PluginCollisionInterface* collisionInterface = m_pluginManager->getCollisionInterface();

		if (collisionInterface && collisionInterface->getNumRules())
		{
			int objectUniqueIdB = -1, linkIndexB = -1;
			btCollisionObject* colObjB = (btCollisionObject*)proxy1->m_clientObject;
			btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(colObjB);
			if (mblB)
			{
				objectUniqueIdB = mblB->m_multiBody->getUserIndex2();
				linkIndexB = mblB->m_link;
			}
			else
			{
				objectUniqueIdB = colObjB->getUserIndex2();
				linkIndexB = -1;
			}
			int objectUniqueIdA = -1, linkIndexA = -1;
			btCollisionObject* colObjA = (btCollisionObject*)proxy0->m_clientObject;
			btMultiBodyLinkCollider* mblA = btMultiBodyLinkCollider::upcast(colObjA);
			if (mblA)
			{
				objectUniqueIdA = mblA->m_multiBody->getUserIndex2();
				linkIndexA = mblA->m_link;
			}
			else
			{
				objectUniqueIdA = colObjA->getUserIndex2();
				linkIndexA = -1;
			}
			int collisionFilterGroupA = proxy0->m_collisionFilterGroup;
			int collisionFilterMaskA = proxy0->m_collisionFilterMask;
			int collisionFilterGroupB = proxy1->m_collisionFilterGroup;
			int collisionFilterMaskB = proxy1->m_collisionFilterMask;

			return collisionInterface->needsBroadphaseCollision(objectUniqueIdA, linkIndexA,
																collisionFilterGroupA, collisionFilterMaskA,
																objectUniqueIdB, linkIndexB, collisionFilterGroupB, collisionFilterMaskB, m_filterMode);
		}
		else
		{
			if (m_filterMode == B3_FILTER_GROUPAMASKB_AND_GROUPBMASKA)
			{
				bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
				collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
				return collides;
			}

			if (m_filterMode == B3_FILTER_GROUPAMASKB_OR_GROUPBMASKA)
			{
				bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
				collides = collides || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
				return collides;
			}
			return false;
		}
	}
};

struct InternalStateLogger
{
	int m_loggingUniqueId;
	int m_loggingType;

	InternalStateLogger()
		: m_loggingUniqueId(0),
		  m_loggingType(0)
	{
	}
	virtual ~InternalStateLogger() {}

	virtual void stop() = 0;
	virtual void logState(btScalar timeStep) = 0;
};

struct VideoMP4Loggger : public InternalStateLogger
{
	struct GUIHelperInterface* m_guiHelper;
	std::string m_fileName;
	VideoMP4Loggger(int loggerUid, const char* fileName, GUIHelperInterface* guiHelper)
		: m_guiHelper(guiHelper)
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
		: m_loggingTimeStamp(0),
		  m_logFileHandle(0),
		  m_minitaurMultiBody(minitaurMultiBody)
	{
		m_loggingUniqueId = loggingUniqueId;
		m_loggingType = STATE_LOGGING_MINITAUR;
		m_motorIdList.resize(motorIdList.size());
		for (int m = 0; m < motorIdList.size(); m++)
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
			btScalar roll = 0;
			btScalar pitch = 0;
			btScalar yaw = 0;

			mat.getEulerZYX(yaw, pitch, roll);

			logData.m_values.push_back(m_loggingTimeStamp);
			logData.m_values.push_back((float)roll);
			logData.m_values.push_back((float)pitch);
			logData.m_values.push_back((float)yaw);

			for (int i = 0; i < 8; i++)
			{
				float jointAngle = (float)motorDir[i] * m_minitaurMultiBody->getJointPos(m_motorIdList[i]);
				logData.m_values.push_back(jointAngle);
			}
			for (int i = 0; i < 8; i++)
			{
				btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)m_minitaurMultiBody->getLink(m_motorIdList[i]).m_userPtr;

				if (motor && timeStep > btScalar(0))
				{
					btScalar force = motor->getAppliedImpulse(0) / timeStep;
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
		for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
		{
			m_vrEvents[i].m_deviceType = 0;
			m_vrEvents[i].m_numButtonEvents = 0;
			m_vrEvents[i].m_numMoveEvents = 0;
			for (int b = 0; b < MAX_VR_BUTTONS; b++)
			{
				m_vrEvents[i].m_buttons[b] = 0;
			}
		}
	}

	void addNewVREvents(const struct b3VRControllerEvent* vrEvents, int numVREvents)
	{
		//update m_vrEvents
		for (int i = 0; i < numVREvents; i++)
		{
			int controlledId = vrEvents[i].m_controllerId;
			if (vrEvents[i].m_numMoveEvents)
			{
				m_vrEvents[controlledId].m_analogAxis = vrEvents[i].m_analogAxis;
				for (int a = 0; a < 10; a++)
				{
					m_vrEvents[controlledId].m_auxAnalogAxis[a] = vrEvents[i].m_auxAnalogAxis[a];
				}
			}
			else
			{
				m_vrEvents[controlledId].m_analogAxis = 0;
				for (int a = 0; a < 10; a++)
				{
					m_vrEvents[controlledId].m_auxAnalogAxis[a] = 0;
				}
			}
			if (vrEvents[i].m_numMoveEvents + vrEvents[i].m_numButtonEvents)
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
			for (int b = 0; b < MAX_VR_BUTTONS; b++)
			{
				m_vrEvents[controlledId].m_buttons[b] |= vrEvents[i].m_buttons[b];
				if (vrEvents[i].m_buttons[b] & eButtonIsDown)
				{
					m_vrEvents[controlledId].m_buttons[b] |= eButtonIsDown;
				}
				else
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
		: m_loggingTimeStamp(0),
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
			float timeStamp = m_loggingTimeStamp * timeStep;

			for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
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
						int packedButtons[7] = {0, 0, 0, 0, 0, 0, 0};

						int packedButtonIndex = 0;
						int packedButtonShift = 0;
						//encode the 64 buttons into 7 int (3 bits each), each int stores 10 buttons
						for (int b = 0; b < MAX_VR_BUTTONS; b++)
						{
							int buttonMask = event.m_buttons[b];
							buttonMask = buttonMask << (packedButtonShift * 3);
							packedButtons[packedButtonIndex] |= buttonMask;
							packedButtonShift++;

							if (packedButtonShift >= 10)
							{
								packedButtonShift = 0;
								packedButtonIndex++;
								if (packedButtonIndex >= 7)
								{
									btAssert(0);
									break;
								}
							}
						}

						for (int b = 0; b < 7; b++)
						{
							logData.m_values.push_back(packedButtons[b]);
						}
						logData.m_values.push_back(event.m_deviceType);
						appendMinitaurLogData(m_logFileHandle, m_structTypes, logData);

						event.m_numButtonEvents = 0;
						event.m_numMoveEvents = 0;
						for (int b = 0; b < MAX_VR_BUTTONS; b++)
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
		: m_loggingTimeStamp(0),
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

		for (int i = 0; i < m_maxLogDof; i++)
		{
			m_structTypes.append("f");
			char jointName[256];
			sprintf(jointName, "q%d", i);
			structNames.push_back(jointName);
		}

		for (int i = 0; i < m_maxLogDof; i++)
		{
			m_structTypes.append("f");
			char jointName[256];
			sprintf(jointName, "u%d", i);
			structNames.push_back(jointName);
		}

		if (m_logFlags & STATE_LOG_JOINT_TORQUES)
		{
			for (int i = 0; i < m_maxLogDof; i++)
			{
				m_structTypes.append("f");
				char jointName[256];
				sprintf(jointName, "t%d", i);
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
			for (int i = 0; i < m_dynamicsWorld->getNumMultibodies(); i++)
			{
				const btMultiBody* mb = m_dynamicsWorld->getMultiBody(i);
				int objectUniqueId = mb->getUserIndex2();
				if (m_filterObjectUniqueId && m_bodyIdList.findLinearSearch2(objectUniqueId) < 0)
				{
					continue;
				}

				MinitaurLogRecord logData;
				int stepCount = m_loggingTimeStamp;
				float timeStamp = m_loggingTimeStamp * m_dynamicsWorld->getSolverInfo().m_timeStep;
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
									jointTorque += motor->getAppliedImpulse(0) / timeStep;
								}
							}
							if (m_logFlags & STATE_LOG_JOINT_USER_TORQUES)
							{
								if (mb->getLink(j).m_jointType == 0 || mb->getLink(j).m_jointType == 1)
								{
									jointTorque += mb->getJointTorque(j);  //these are the 'user' applied external torques
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
		: m_loggingTimeStamp(0),
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
					float timeStamp = m_loggingTimeStamp * timeStep;
					logData.m_values.push_back(stepCount);
					logData.m_values.push_back(timeStamp);

					const btManifoldPoint& srcPt = manifold->getContactPoint(p);

					logData.m_values.push_back(0);  // reserved contact flag
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

struct SaveStateData
{
	bParse::btBulletFile* m_bulletFile;
	btSerializer* m_serializer;
};

struct PhysicsServerCommandProcessorInternalData
{
	///handle management
	b3ResizablePool<InternalTextureHandle> m_textureHandles;
	b3ResizablePool<InternalBodyHandle> m_bodyHandles;
	b3ResizablePool<InternalCollisionShapeHandle> m_userCollisionShapeHandles;
	b3ResizablePool<InternalVisualShapeHandle> m_userVisualShapeHandles;
	b3ResizablePool<b3PoolBodyHandle<SharedMemoryUserData> > m_userDataHandles;
	btHashMap<SharedMemoryUserDataHashKey, int> m_userDataHandleLookup;

	b3PluginManager m_pluginManager;

	bool m_useRealTimeSimulation;

	b3VRControllerEvents m_vrControllerEvents;

	btAlignedObjectArray<SaveStateData> m_savedStates;

	btAlignedObjectArray<b3KeyboardEvent> m_keyboardEvents;
	btAlignedObjectArray<b3MouseEvent> m_mouseEvents;

	CommandLogger* m_commandLogger;
	int m_commandLoggingUid;

	CommandLogPlayback* m_logPlayback;
	int m_logPlaybackUid;

	btScalar m_physicsDeltaTime;
	btScalar m_numSimulationSubSteps;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_multiBodyJointFeedbacks;
	b3HashMap<btHashPtr, btInverseDynamics::MultiBodyTree*> m_inverseDynamicsBodies;
	b3HashMap<btHashPtr, IKTrajectoryHelper*> m_inverseKinematicsHelpers;

	int m_userConstraintUIDGenerator;
	b3HashMap<btHashInt, InteralUserConstraintData> m_userConstraints;

	b3AlignedObjectArray<SaveWorldObjectData> m_saveWorldBodyData;

	btAlignedObjectArray<btMultiBodyWorldImporter*> m_worldImporters;

	btAlignedObjectArray<std::string*> m_strings;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btAlignedObjectArray<int> m_allocatedTextures;
	btHashMap<btHashPtr, UrdfCollision> m_bulletCollisionShape2UrdfCollision;
	btAlignedObjectArray<btStridingMeshInterface*> m_meshInterfaces;

	MyOverlapFilterCallback* m_broadphaseCollisionFilterCallback;
	btHashedOverlappingPairCache* m_pairCache;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btMultiBodyConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	btSoftMultiBodyDynamicsWorld* m_dynamicsWorld;
	btSoftBodySolver* m_softbodySolver;
#else
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
#endif

	int m_constraintSolverType;
	SharedMemoryDebugDrawer* m_remoteDebugDrawer;

	btAlignedObjectArray<b3ContactPointData> m_cachedContactPoints;
	MyBroadphaseCallback m_cachedOverlappingObjects;

	btAlignedObjectArray<int> m_sdfRecentLoadedBodies;
	btAlignedObjectArray<int> m_graphicsIndexToSegmentationMask;

	btAlignedObjectArray<InternalStateLogger*> m_stateLoggers;
	int m_stateLoggersUniqueId;
	int m_profileTimingLoggingUid;
	std::string m_profileTimingFileName;

	struct GUIHelperInterface* m_guiHelper;

	int m_sharedMemoryKey;
	bool m_enableTinyRenderer;

	bool m_verboseOutput;

	//data for picking objects
	class btRigidBody* m_pickedBody;
	int m_savedActivationState;
	class btTypedConstraint* m_pickedConstraint;
	class btMultiBodyPoint2Point* m_pickingMultiBodyPoint2Point;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	bool m_prevCanSleep;
	int m_pdControlPlugin;
	int m_collisionFilterPlugin;
	int m_grpcPlugin;

#ifdef B3_ENABLE_TINY_AUDIO
	b3SoundEngine m_soundEngine;
#endif

	b3HashMap<b3HashString, char*> m_profileEvents;
	b3HashMap<b3HashString, UrdfVisualShapeCache> m_cachedVUrdfisualShapes;

	b3ThreadPool* m_threadPool;
	btScalar m_defaultCollisionMargin;

	PhysicsServerCommandProcessorInternalData(PhysicsCommandProcessorInterface* proc)
		: m_pluginManager(proc),
		  m_useRealTimeSimulation(false),
		  m_commandLogger(0),
		  m_commandLoggingUid(-1),
		  m_logPlayback(0),
		  m_logPlaybackUid(-1),
		  m_physicsDeltaTime(1. / 240.),
		  m_numSimulationSubSteps(0),
		  m_userConstraintUIDGenerator(1),
		  m_broadphaseCollisionFilterCallback(0),
		  m_pairCache(0),
		  m_broadphase(0),
		  m_dispatcher(0),
		  m_solver(0),
		  m_collisionConfiguration(0),
		  m_dynamicsWorld(0),
		  m_constraintSolverType(-1),
		  m_remoteDebugDrawer(0),
		  m_stateLoggersUniqueId(0),
		  m_profileTimingLoggingUid(-1),
		  m_guiHelper(0),
		  m_sharedMemoryKey(SHARED_MEMORY_KEY),
		  m_enableTinyRenderer(true),
		  m_verboseOutput(false),
		  m_pickedBody(0),
		  m_pickedConstraint(0),
		  m_pickingMultiBodyPoint2Point(0),
		  m_pdControlPlugin(-1),
		  m_collisionFilterPlugin(-1),
		  m_grpcPlugin(-1),
		  m_threadPool(0),
		  m_defaultCollisionMargin(0.001)
	{
		{
			//register static plugins:
#ifdef STATIC_LINK_VR_PLUGIN
			b3PluginFunctions funcs(initPlugin_vrSyncPlugin,exitPlugin_vrSyncPlugin, executePluginCommand_vrSyncPlugin);
			funcs.m_preTickFunc = preTickPluginCallback_vrSyncPlugin;
			m_pluginManager.registerStaticLinkedPlugin("vrSyncPlugin", funcs);
#endif  //STATIC_LINK_VR_PLUGIN
		}
#ifndef SKIP_STATIC_PD_CONTROL_PLUGIN
			{
				//int b3PluginManager::registerStaticLinkedPlugin(const char* pluginPath, PFN_INIT initFunc, PFN_EXIT exitFunc, PFN_EXECUTE executeCommandFunc, PFN_TICK preTickFunc, PFN_TICK postTickFunc, PFN_GET_RENDER_INTERFACE getRendererFunc, PFN_TICK processClientCommandsFunc, PFN_GET_COLLISION_INTERFACE getCollisionFunc, bool initPlugin)
				b3PluginFunctions funcs(initPlugin_pdControlPlugin,exitPlugin_pdControlPlugin,executePluginCommand_pdControlPlugin);
				funcs.m_preTickFunc = preTickPluginCallback_pdControlPlugin;
				m_pdControlPlugin = m_pluginManager.registerStaticLinkedPlugin("pdControlPlugin", funcs);
			}
#endif  //SKIP_STATIC_PD_CONTROL_PLUGIN

#ifndef SKIP_COLLISION_FILTER_PLUGIN
			{
				b3PluginFunctions funcs(initPlugin_collisionFilterPlugin,exitPlugin_collisionFilterPlugin, executePluginCommand_collisionFilterPlugin);
				funcs.m_getCollisionFunc = getCollisionInterface_collisionFilterPlugin;
				m_collisionFilterPlugin = m_pluginManager.registerStaticLinkedPlugin("collisionFilterPlugin", funcs );
				m_pluginManager.selectCollisionPlugin(m_collisionFilterPlugin);
			}
#endif

#ifdef ENABLE_STATIC_GRPC_PLUGIN
			{
				b3PluginFunctions funcs(initPlugin_grpcPlugin, exitPlugin_grpcPlugin, executePluginCommand_grpcPlugin);
				funcs.m_processClientCommandsFunc = processClientCommands_grpcPlugin;
				m_grpcPlugin = m_pluginManager.registerStaticLinkedPlugin("grpcPlugin", funcs);
			}
#endif  //ENABLE_STATIC_GRPC_PLUGIN

#ifdef STATIC_EGLRENDERER_PLUGIN
			{
				bool initPlugin = false;
				b3PluginFunctions funcs(initPlugin_eglRendererPlugin, exitPlugin_eglRendererPlugin, executePluginCommand_eglRendererPlugin);
				funcs.m_getRendererFunc = getRenderInterface_eglRendererPlugin;
				int renderPluginId = m_pluginManager.registerStaticLinkedPlugin("eglRendererPlugin", funcs, initPlugin);
				m_pluginManager.selectPluginRenderer(renderPluginId);
			}
#endif  //STATIC_EGLRENDERER_PLUGIN

#ifndef SKIP_STATIC_TINYRENDERER_PLUGIN
			{
				b3PluginFunctions funcs(initPlugin_tinyRendererPlugin, exitPlugin_tinyRendererPlugin, executePluginCommand_tinyRendererPlugin);
				funcs.m_getRendererFunc=getRenderInterface_tinyRendererPlugin;
				int renderPluginId = m_pluginManager.registerStaticLinkedPlugin("tinyRendererPlugin", funcs);
				m_pluginManager.selectPluginRenderer(renderPluginId);
			}
#endif

#ifdef B3_ENABLE_FILEIO_PLUGIN
			{
				b3PluginFunctions funcs(initPlugin_fileIOPlugin, exitPlugin_fileIOPlugin, executePluginCommand_fileIOPlugin);
				funcs.m_fileIoFunc = getFileIOFunc_fileIOPlugin;
				int renderPluginId = m_pluginManager.registerStaticLinkedPlugin("fileIOPlugin", funcs);
				m_pluginManager.selectFileIOPlugin(renderPluginId);
			}
#endif



		m_vrControllerEvents.init();

		m_bodyHandles.exitHandles();
		m_bodyHandles.initHandles();
		m_userCollisionShapeHandles.exitHandles();
		m_userCollisionShapeHandles.initHandles();

		m_userVisualShapeHandles.exitHandles();
		m_userVisualShapeHandles.initHandles();
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
	}
	else
	{
		//state loggers use guiHelper, so remove them before the guiHelper is deleted
		deleteStateLoggers();
		if (m_data->m_guiHelper && m_data->m_dynamicsWorld && m_data->m_dynamicsWorld->getDebugDrawer())
		{
			m_data->m_dynamicsWorld->setDebugDrawer(0);
		}
	}
	m_data->m_guiHelper = guiHelper;
}

PhysicsServerCommandProcessor::PhysicsServerCommandProcessor()
	: m_data(0)
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
	for (int i = 0; i < m_data->m_profileEvents.size(); i++)
	{
		char* event = *m_data->m_profileEvents.getAtIndex(i);
		delete[] event;
	}
	if (m_data->m_threadPool)
		delete m_data->m_threadPool;

	delete m_data;
}

void preTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
	PhysicsServerCommandProcessor* proc = (PhysicsServerCommandProcessor*)world->getWorldUserInfo();

	proc->tickPlugins(timeStep, true);
}

void logCallback(btDynamicsWorld* world, btScalar timeStep)
{
	//handle the logging and playing sounds
	PhysicsServerCommandProcessor* proc = (PhysicsServerCommandProcessor*)world->getWorldUserInfo();
	proc->processCollisionForces(timeStep);
	proc->logObjectStates(timeStep);

	proc->tickPlugins(timeStep, false);
}

bool MyContactAddedCallback(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
{
	btAdjustInternalEdgeContacts(cp, colObj1Wrap, colObj0Wrap, partId1, index1);
	return true;
}

bool MyContactDestroyedCallback(void* userPersistentData)
{
	//printf("destroyed\n");
	return false;
}

bool MyContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
	//printf("processed\n");
	return false;
}
void MyContactStartedCallback(btPersistentManifold* const& manifold)
{
	//printf("started\n");
}
void MyContactEndedCallback(btPersistentManifold* const& manifold)
{
	//	printf("ended\n");
}

void PhysicsServerCommandProcessor::processCollisionForces(btScalar timeStep)
{
#ifdef B3_ENABLE_TINY_AUDIO
	//this is experimental at the moment: impulse thresholds, sound parameters will be exposed in C-API/pybullet.
	//audio will go into a wav file, as well as real-time output to speakers/headphones using RtAudio/DAC.

	int numContactManifolds = m_data->m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numContactManifolds; i++)
	{
		const btPersistentManifold* manifold = m_data->m_dynamicsWorld->getDispatcher()->getInternalManifoldPointer()[i];

		bool objHasSound[2];
		objHasSound[0] = (0 != (manifold->getBody0()->getCollisionFlags() & btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER));
		objHasSound[1] = (0 != (manifold->getBody1()->getCollisionFlags() & btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER));
		const btCollisionObject* colObjs[2] = {manifold->getBody0(), manifold->getBody1()};

		for (int ob = 0; ob < 2; ob++)
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

				if ((uid0 < 0) || (linkIndex < -1))
					continue;

				InternalBodyHandle* bodyHandle0 = m_data->m_bodyHandles.getHandle(uid0);
				SDFAudioSource* audioSrc = bodyHandle0->m_audioSources[linkIndex];
				if (audioSrc == 0)
					continue;

				for (int p = 0; p < manifold->getNumContacts(); p++)
				{
					double imp = manifold->getContactPoint(p).getAppliedImpulse();
					//printf ("manifold %d, contact %d, lifeTime:%d, appliedImpulse:%f\n",i,p, manifold->getContactPoint(p).getLifeTime(),imp);

					if (imp > audioSrc->m_collisionForceThreshold && manifold->getContactPoint(p).getLifeTime() == 1)
					{
						int soundSourceIndex = m_data->m_soundEngine.getAvailableSoundSource();
						if (soundSourceIndex >= 0)
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
							m_data->m_soundEngine.startSound(soundSourceIndex, msg);
						}
					}
				}
			}
		}
	}
#endif  //B3_ENABLE_TINY_AUDIO
}

void PhysicsServerCommandProcessor::processClientCommands()
{
	m_data->m_pluginManager.tickPlugins(0, B3_PROCESS_CLIENT_COMMANDS_TICK);
}

void PhysicsServerCommandProcessor::reportNotifications()
{
	m_data->m_pluginManager.reportNotifications();
}

void PhysicsServerCommandProcessor::tickPlugins(btScalar timeStep, bool isPreTick)
{
	b3PluginManagerTickMode tickMode = isPreTick ? B3_PRE_TICK_MODE : B3_POST_TICK_MODE;
	m_data->m_pluginManager.tickPlugins(timeStep, tickMode);
	if (!isPreTick)
	{
		//clear events after each postTick, so we don't receive events multiple ticks
		m_data->m_pluginManager.clearEvents();
	}
}

void PhysicsServerCommandProcessor::logObjectStates(btScalar timeStep)
{
	for (int i = 0; i < m_data->m_stateLoggers.size(); i++)
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
	int m_flags;

	ProgrammaticUrdfInterface(const b3CreateMultiBodyArgs& bodyArgs, PhysicsServerCommandProcessorInternalData* data, int flags)
		: m_bodyUniqueId(-1),
		  m_createBodyArgs(bodyArgs),
		  m_data(data),
		  m_flags(flags)
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
		char numstr[21];  // enough to hold all numbers up to 64-bits
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

	mutable btHashMap<btHashInt, UrdfMaterialColor> m_linkColors;

	virtual bool getLinkColor2(int linkIndex, struct UrdfMaterialColor& matCol) const
	{

		if (m_flags & URDF_USE_MATERIAL_COLORS_FROM_MTL)
		{
			const UrdfMaterialColor* matColPtr = m_linkColors[linkIndex];
			if (matColPtr)
			{
				matCol = *matColPtr;
				if ((m_flags&CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL)==0)
				{
					matCol.m_rgbaColor[3] = 1;
				}

				return true;
			}
		} else
		{
			if (m_createBodyArgs.m_linkVisualShapeUniqueIds[linkIndex] >= 0)
			{
				const InternalVisualShapeHandle* visHandle = m_data->m_userVisualShapeHandles.getHandle(m_createBodyArgs.m_linkVisualShapeUniqueIds[linkIndex]);
				if (visHandle)
				{
					for (int i = 0; i < visHandle->m_visualShapes.size(); i++)
					{
						if (visHandle->m_visualShapes[i].m_geometry.m_hasLocalMaterial)
						{
							matCol = visHandle->m_visualShapes[i].m_geometry.m_localMaterial.m_matColor;
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	virtual int getCollisionGroupAndMask(int linkIndex, int& colGroup, int& colMask) const
	{
		return 0;
	}
	///this API will likely change, don't override it!
	virtual bool getLinkContactInfo(int linkIndex, URDFLinkContactInfo& contactInfo) const
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
		char numstr[21];  // enough to hold all numbers up to 64-bits
		sprintf(numstr, "%d", linkIndex);
		jointName = jointName + numstr;
		return jointName;
	}

	//fill mass and inertial data. If inertial data is missing, please initialize mass, inertia to sensitive values, and inertialFrame to identity.
	virtual void getMassAndInertia(int urdfLinkIndex, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
	{
		if (urdfLinkIndex >= 0 && urdfLinkIndex < m_createBodyArgs.m_numLinks)
		{
			mass = m_createBodyArgs.m_linkMasses[urdfLinkIndex];
			localInertiaDiagonal.setValue(
				m_createBodyArgs.m_linkInertias[urdfLinkIndex * 3 + 0],
				m_createBodyArgs.m_linkInertias[urdfLinkIndex * 3 + 1],
				m_createBodyArgs.m_linkInertias[urdfLinkIndex * 3 + 2]);
			inertialFrame.setOrigin(btVector3(
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex * 3 + 0],
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex * 3 + 1],
				m_createBodyArgs.m_linkInertialFramePositions[urdfLinkIndex * 3 + 2]));
			inertialFrame.setRotation(btQuaternion(
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex * 4 + 0],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex * 4 + 1],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex * 4 + 2],
				m_createBodyArgs.m_linkInertialFrameOrientations[urdfLinkIndex * 4 + 3]));
		}
		else
		{
			mass = 0;
			localInertiaDiagonal.setValue(0, 0, 0);
			inertialFrame.setIdentity();
		}
	}

	///fill an array of child link indices for this link, btAlignedObjectArray behaves like a std::vector so just use push_back and resize(0) if needed
	virtual void getLinkChildIndices(int urdfLinkIndex, btAlignedObjectArray<int>& childLinkIndices) const
	{
		for (int i = 0; i < m_createBodyArgs.m_numLinks; i++)
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
			case ePrismaticType:
			{
				isValid = true;
				jointType = URDFPrismaticJoint;
				break;
			}
			case eFixedType:
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
				m_createBodyArgs.m_linkPositions[urdfLinkIndex * 3 + 0],
				m_createBodyArgs.m_linkPositions[urdfLinkIndex * 3 + 1],
				m_createBodyArgs.m_linkPositions[urdfLinkIndex * 3 + 2]));
			parent2joint.setRotation(btQuaternion(
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex * 4 + 0],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex * 4 + 1],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex * 4 + 2],
				m_createBodyArgs.m_linkOrientations[urdfLinkIndex * 4 + 3]));

			linkTransformInWorld.setIdentity();

			jointAxisInJointSpace.setValue(
				m_createBodyArgs.m_linkJointAxis[3 * urdfLinkIndex + 0],
				m_createBodyArgs.m_linkJointAxis[3 * urdfLinkIndex + 1],
				m_createBodyArgs.m_linkJointAxis[3 * urdfLinkIndex + 2]);
		}
		return isValid;
	};

	virtual bool getRootTransformInWorld(btTransform& rootTransformInWorld) const
	{
		int baseLinkIndex = m_createBodyArgs.m_baseLinkIndex;

		rootTransformInWorld.setOrigin(btVector3(
			m_createBodyArgs.m_linkPositions[baseLinkIndex * 3 + 0],
			m_createBodyArgs.m_linkPositions[baseLinkIndex * 3 + 1],
			m_createBodyArgs.m_linkPositions[baseLinkIndex * 3 + 2]));
		rootTransformInWorld.setRotation(btQuaternion(
			m_createBodyArgs.m_linkOrientations[baseLinkIndex * 4 + 0],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex * 4 + 1],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex * 4 + 2],
			m_createBodyArgs.m_linkOrientations[baseLinkIndex * 4 + 3]));
		return true;
	}
	virtual void setRootTransformInWorld(const btTransform& rootTransformInWorld)
	{
		b3Assert(0);
	}

	virtual int convertLinkVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
	{
		int graphicsIndex = -1;
		double globalScaling = 1.f;  //todo!
		int flags = 0;
		CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();

		BulletURDFImporter u2b(m_data->m_guiHelper, m_data->m_pluginManager.getRenderInterface(),fileIO, globalScaling, flags);
		u2b.setEnableTinyRenderer(m_data->m_enableTinyRenderer);

		btAlignedObjectArray<GLInstanceVertex> vertices;
		btAlignedObjectArray<int> indices;
		btTransform startTrans;
		startTrans.setIdentity();
		btAlignedObjectArray<BulletURDFTexture> textures;

		if (m_createBodyArgs.m_linkVisualShapeUniqueIds[linkIndex] >= 0)
		{
			InternalVisualShapeHandle* visHandle = m_data->m_userVisualShapeHandles.getHandle(m_createBodyArgs.m_linkVisualShapeUniqueIds[linkIndex]);
			if (visHandle)
			{
				if (visHandle->m_OpenGLGraphicsIndex >= 0)
				{
					//instancing. assume the inertial frame is identical
					graphicsIndex = visHandle->m_OpenGLGraphicsIndex;
				}
				else
				{
					for (int v = 0; v < visHandle->m_visualShapes.size(); v++)
					{
						b3ImportMeshData meshData;
						u2b.convertURDFToVisualShapeInternal(&visHandle->m_visualShapes[v], pathPrefix, localInertiaFrame.inverse() * visHandle->m_visualShapes[v].m_linkLocalFrame, vertices, indices, textures, meshData);
						if ((meshData.m_flags & B3_IMPORT_MESH_HAS_RGBA_COLOR) &&
						(meshData.m_flags & B3_IMPORT_MESH_HAS_SPECULAR_COLOR))
						{
							UrdfMaterialColor matCol;
							matCol.m_rgbaColor.setValue(meshData.m_rgbaColor[0],
								meshData.m_rgbaColor[1],
								meshData.m_rgbaColor[2],
								meshData.m_rgbaColor[3]);
							matCol.m_specularColor.setValue(meshData.m_specularColor[0],
								meshData.m_specularColor[1],
								meshData.m_specularColor[2]);
							m_linkColors.insert(linkIndex, matCol);
						}
					}

					if (vertices.size() && indices.size())
					{
						if (1)
						{
							int textureIndex = -1;
							if (textures.size())
							{
								textureIndex = m_data->m_guiHelper->registerTexture(textures[0].textureData1, textures[0].m_width, textures[0].m_height);
							}

							{
								B3_PROFILE("registerGraphicsShape");
								graphicsIndex = m_data->m_guiHelper->registerGraphicsShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
								visHandle->m_OpenGLGraphicsIndex = graphicsIndex;
							}
						}
					}
				}
			}
		}
		return graphicsIndex;
	}

	virtual void convertLinkVisualShapes2(int linkIndex, int urdfIndex, const char* pathPrefix, const btTransform& localInertiaFrame, class btCollisionObject* colObj, int bodyUniqueId) const
	{
		//if there is a visual, use it, otherwise convert collision shape back into UrdfCollision...

		UrdfModel model;  // = m_data->m_urdfParser.getModel();
		UrdfLink link;

		if (m_createBodyArgs.m_linkVisualShapeUniqueIds[urdfIndex] >= 0)
		{
			const InternalVisualShapeHandle* visHandle = m_data->m_userVisualShapeHandles.getHandle(m_createBodyArgs.m_linkVisualShapeUniqueIds[urdfIndex]);
			if (visHandle)
			{
				for (int i = 0; i < visHandle->m_visualShapes.size(); i++)
				{
					link.m_visualArray.push_back(visHandle->m_visualShapes[i]);
				}
			}
		}

		if (link.m_visualArray.size() == 0)
		{
			int colShapeUniqueId = m_createBodyArgs.m_linkCollisionShapeUniqueIds[urdfIndex];
			if (colShapeUniqueId >= 0)
			{
				InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(colShapeUniqueId);
				if (handle)
				{
					for (int i = 0; i < handle->m_urdfCollisionObjects.size(); i++)
					{
						link.m_collisionArray.push_back(handle->m_urdfCollisionObjects[i]);
					}
				}
			}
		}
		//UrdfVisual vis;
		//link.m_visualArray.push_back(vis);
		//UrdfLink*const* linkPtr = model.m_links.getAtIndex(urdfIndex);
		if (m_data->m_pluginManager.getRenderInterface())
		{
			CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
			m_data->m_pluginManager.getRenderInterface()->convertVisualShapes(linkIndex, pathPrefix, localInertiaFrame, &link, &model, colObj->getBroadphaseHandle()->getUid(), bodyUniqueId, fileIO);
		}
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
		if (colShapeUniqueId >= 0)
		{
			InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(colShapeUniqueId);
			if (handle && handle->m_collisionShape)
			{
				handle->m_used++;
				if (handle->m_collisionShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
				{
					btCompoundShape* childCompound = (btCompoundShape*)handle->m_collisionShape;
					for (int c = 0; c < childCompound->getNumChildShapes(); c++)
					{
						btTransform childTrans = childCompound->getChildTransform(c);
						btCollisionShape* childShape = childCompound->getChildShape(c);
						btTransform tr = localInertiaFrame.inverse() * childTrans;
						compound->addChildShape(tr, childShape);
					}
				}
				else
				{
					btTransform childTrans;
					childTrans.setIdentity();
					compound->addChildShape(localInertiaFrame.inverse() * childTrans, handle->m_collisionShape);
				}
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
	m_data->m_constraintSolverType = eConstraintSolverLCP_SI;
	///collision configuration contains default setup for memory, collision setup
	//m_collisionConfiguration->setConvexConvexMultipointIterations();
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	m_data->m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
#else
	m_data->m_collisionConfiguration = new btDefaultCollisionConfiguration();
#endif
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_data->m_dispatcher = new btCollisionDispatcher(m_data->m_collisionConfiguration);

	m_data->m_broadphaseCollisionFilterCallback = new MyOverlapFilterCallback(&m_data->m_pluginManager);
	m_data->m_broadphaseCollisionFilterCallback->m_filterMode = B3_FILTER_GROUPAMASKB_OR_GROUPBMASKA;

	m_data->m_pairCache = new btHashedOverlappingPairCache();

	m_data->m_pairCache->setOverlapFilterCallback(m_data->m_broadphaseCollisionFilterCallback);

	//int maxProxies = 32768;
	//m_data->m_broadphase = new btSimpleBroadphase(maxProxies, m_data->m_pairCache);
	btDbvtBroadphase* bv = new btDbvtBroadphase(m_data->m_pairCache);
	bv->setVelocityPrediction(0);
	m_data->m_broadphase = bv;

	m_data->m_solver = new btMultiBodyConstraintSolver;

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	m_data->m_dynamicsWorld = new btSoftMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);
#else
	m_data->m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_data->m_dispatcher, m_data->m_broadphase, m_data->m_solver, m_data->m_collisionConfiguration);
#endif

	//Workaround: in a VR application, where we avoid synchronizing between GFX/Physics threads, we don't want to resize this array, so pre-allocate it
	m_data->m_dynamicsWorld->getCollisionObjectArray().reserve(32768);

	m_data->m_remoteDebugDrawer = new SharedMemoryDebugDrawer();

	m_data->m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_data->m_dynamicsWorld->getSolverInfo().m_erp2 = 0.08;

	m_data->m_dynamicsWorld->getSolverInfo().m_frictionERP = 0.2;  //need to check if there are artifacts with frictionERP
	m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = 0.00001;
	m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = 50;
	m_data->m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 0;
	m_data->m_dynamicsWorld->getSolverInfo().m_warmstartingFactor = 0.1;
	gDbvtMargin = btScalar(0);
	m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = 1e-7;

	if (m_data->m_guiHelper)
	{
		m_data->m_guiHelper->createPhysicsDebugDrawer(m_data->m_dynamicsWorld);
	}
	bool isPreTick = false;
	m_data->m_dynamicsWorld->setInternalTickCallback(logCallback, this, isPreTick);
	isPreTick = true;
	m_data->m_dynamicsWorld->setInternalTickCallback(preTickCallback, this, isPreTick);

	gContactAddedCallback = MyContactAddedCallback;

#ifdef B3_ENABLE_TINY_AUDIO
	m_data->m_soundEngine.init(16, true);

//we don't use those callbacks (yet), experimental

//	gContactDestroyedCallback = MyContactDestroyedCallback;
//	gContactProcessedCallback = MyContactProcessedCallback;
//	gContactStartedCallback = MyContactStartedCallback;
//	gContactEndedCallback = MyContactEndedCallback;
#endif
}

void PhysicsServerCommandProcessor::deleteStateLoggers()
{
	for (int i = 0; i < m_data->m_stateLoggers.size(); i++)
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

	for (int i = 0; i < m_data->m_multiBodyJointFeedbacks.size(); i++)
	{
		delete m_data->m_multiBodyJointFeedbacks[i];
	}
	m_data->m_multiBodyJointFeedbacks.clear();

	for (int i = 0; i < m_data->m_worldImporters.size(); i++)
	{
		m_data->m_worldImporters[i]->deleteAllData();
		delete m_data->m_worldImporters[i];
	}
	m_data->m_worldImporters.clear();

#ifdef ENABLE_LINK_MAPPER
	for (int i = 0; i < m_data->m_urdfLinkNameMapper.size(); i++)
	{
		delete m_data->m_urdfLinkNameMapper[i];
	}
	m_data->m_urdfLinkNameMapper.clear();
#endif  //ENABLE_LINK_MAPPER

	for (int i = 0; i < m_data->m_strings.size(); i++)
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
			btTypedConstraint* constraint = m_data->m_dynamicsWorld->getConstraint(i);
			constraints.push_back(constraint);
			m_data->m_dynamicsWorld->removeConstraint(constraint);
		}
		for (i = m_data->m_dynamicsWorld->getNumMultiBodyConstraints() - 1; i >= 0; i--)
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
		for (i = m_data->m_dynamicsWorld->getNumMultibodies() - 1; i >= 0; i--)
		{
			btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(i);
			m_data->m_dynamicsWorld->removeMultiBody(mb);
			delete mb;
		}
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
		for (i = m_data->m_dynamicsWorld->getSoftBodyArray().size() - 1; i >= 0; i--)
		{
			btSoftBody* sb = m_data->m_dynamicsWorld->getSoftBodyArray()[i];
			m_data->m_dynamicsWorld->removeSoftBody(sb);
			delete sb;
		}
#endif
	}

	for (int i = 0; i < constraints.size(); i++)
	{
		delete constraints[i];
	}
	constraints.clear();
	for (int i = 0; i < mbconstraints.size(); i++)
	{
		delete mbconstraints[i];
	}
	mbconstraints.clear();
	//delete collision shapes
	for (int j = 0; j < m_data->m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_data->m_collisionShapes[j];

		//check for internal edge utility, delete memory
		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		{
			btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
			if (trimesh->getTriangleInfoMap())
			{
				delete trimesh->getTriangleInfoMap();
			}
		}
		delete shape;
	}
	for (int j = 0; j < m_data->m_meshInterfaces.size(); j++)
	{
		delete m_data->m_meshInterfaces[j];
	}

	if (m_data->m_guiHelper)
	{
		for (int j = 0; j < m_data->m_allocatedTextures.size(); j++)
		{
			int texId = m_data->m_allocatedTextures[j];
			m_data->m_guiHelper->removeTexture(texId);
		}
	}
	m_data->m_allocatedTextures.clear();
	m_data->m_meshInterfaces.clear();
	m_data->m_collisionShapes.clear();
	m_data->m_bulletCollisionShape2UrdfCollision.clear();
	m_data->m_graphicsIndexToSegmentationMask.clear();

	delete m_data->m_dynamicsWorld;
	m_data->m_dynamicsWorld = 0;

	delete m_data->m_remoteDebugDrawer;
	m_data->m_remoteDebugDrawer = 0;

	delete m_data->m_solver;
	m_data->m_solver = 0;

	delete m_data->m_broadphase;
	m_data->m_broadphase = 0;

	delete m_data->m_pairCache;
	m_data->m_pairCache = 0;

	delete m_data->m_broadphaseCollisionFilterCallback;
	m_data->m_broadphaseCollisionFilterCallback = 0;

	delete m_data->m_dispatcher;
	m_data->m_dispatcher = 0;

	delete m_data->m_collisionConfiguration;
	m_data->m_collisionConfiguration = 0;
	m_data->m_userConstraintUIDGenerator = 1;
}

bool PhysicsServerCommandProcessor::supportsJointMotor(btMultiBody* mb, int mbLinkIndex)
{
	bool canHaveMotor = (mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::eRevolute 
		|| mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::ePrismatic);
	return canHaveMotor;
}

//for testing, create joint motors for revolute and prismatic joints
void PhysicsServerCommandProcessor::createJointMotors(btMultiBody* mb)
{
	int numLinks = mb->getNumLinks();
	for (int i = 0; i < numLinks; i++)
	{
		int mbLinkIndex = i;
		float maxMotorImpulse = 1.f;

		if (supportsJointMotor(mb, mbLinkIndex))
		{
			int dof = 0;
			btScalar desiredVelocity = 0.f;
			btMultiBodyJointMotor* motor = new btMultiBodyJointMotor(mb, mbLinkIndex, dof, desiredVelocity, maxMotorImpulse);
			motor->setPositionTarget(0, 0);
			motor->setVelocityTarget(0, 1);
			//motor->setRhsClamp(gRhsClamp);
			//motor->setMaxAppliedImpulse(0);
			mb->getLink(mbLinkIndex).m_userPtr = motor;
			m_data->m_dynamicsWorld->addMultiBodyConstraint(motor);
			motor->finalizeMultiDof();
		}
		if (mb->getLink(mbLinkIndex).m_jointType == btMultibodyLink::eSpherical)
		{
			btMultiBodySphericalJointMotor* motor = new btMultiBodySphericalJointMotor(mb, mbLinkIndex, 1000*maxMotorImpulse);
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



	for (int m = 0; m < u2b.getNumModels(); m++)
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
			btVector3 localInertiaDiagonal(0, 0, 0);
			int urdfLinkIndex = u2b.getRootLinkIndex();
			u2b.getMassAndInertia2(urdfLinkIndex, mass, localInertiaDiagonal, bodyHandle->m_rootLocalInertialFrame, flags);
		}

		//todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
		//int rootLinkIndex = u2b.getRootLinkIndex();
		//b3Printf("urdf root link index = %d\n",rootLinkIndex);
		MyMultiBodyCreator creation(m_data->m_guiHelper);

		u2b.getRootTransformInWorld(rootTrans);
		//CUF_RESERVED is a temporary flag, for backward compatibility purposes
		flags |= CUF_RESERVED;

		if (flags & CUF_ENABLE_CACHED_GRAPHICS_SHAPES)
		{
			{
				UrdfVisualShapeCache* tmpPtr = m_data->m_cachedVUrdfisualShapes[fileName];
				if (tmpPtr == 0)
				{
					m_data->m_cachedVUrdfisualShapes.insert(fileName, UrdfVisualShapeCache());
				}
			}
			UrdfVisualShapeCache* cachedVisualShapesPtr = m_data->m_cachedVUrdfisualShapes[fileName];
			ConvertURDF2Bullet(u2b, creation, rootTrans, m_data->m_dynamicsWorld, useMultiBody, u2b.getPathPrefix(), flags, cachedVisualShapesPtr);
		}
		else
		{
			ConvertURDF2Bullet(u2b, creation, rootTrans, m_data->m_dynamicsWorld, useMultiBody, u2b.getPathPrefix(), flags);
		}

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

			int segmentationMask = bodyUniqueId;

			{
				int graphicsIndex = -1;
				if (mb->getBaseCollider())
				{
					graphicsIndex = mb->getBaseCollider()->getUserIndex();
				}
				if (graphicsIndex >= 0)
				{
					if (m_data->m_graphicsIndexToSegmentationMask.size() < (graphicsIndex + 1))
					{
						m_data->m_graphicsIndexToSegmentationMask.resize(graphicsIndex + 1);
					}
					m_data->m_graphicsIndexToSegmentationMask[graphicsIndex] = segmentationMask;
				}
			}

			createJointMotors(mb);

#ifdef B3_ENABLE_TINY_AUDIO
			{
				SDFAudioSource audioSource;
				int urdfRootLink = u2b.getRootLinkIndex();  //LinkIndex = creation.m_mb2urdfLink[-1];
				if (u2b.getLinkAudioSource(urdfRootLink, audioSource))
				{
					int flags = mb->getBaseCollider()->getCollisionFlags();
					mb->getBaseCollider()->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
					audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
					if (audioSource.m_userIndex >= 0)
					{
						bodyHandle->m_audioSources.insert(-1, audioSource);
					}
				}
			}
#endif
			//disable serialization of the collision objects (they are too big, and the client likely doesn't need them);

			bodyHandle->m_linkLocalInertialFrames.reserve(mb->getNumLinks());
			for (int i = 0; i < mb->getNumLinks(); i++)
			{
				//disable serialization of the collision objects

				int urdfLinkIndex = creation.m_mb2urdfLink[i];
				btScalar mass;
				btVector3 localInertiaDiagonal(0, 0, 0);
				btTransform localInertialFrame;
				u2b.getMassAndInertia2(urdfLinkIndex, mass, localInertiaDiagonal, localInertialFrame, flags);
				bodyHandle->m_linkLocalInertialFrames.push_back(localInertialFrame);

				std::string* linkName = new std::string(u2b.getLinkName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(linkName);

				mb->getLink(i).m_linkName = linkName->c_str();

				{
					int graphicsIndex = -1;
					if (mb->getLinkCollider(i))
					{
						graphicsIndex = mb->getLinkCollider(i)->getUserIndex();
					}
					if (graphicsIndex >= 0)
					{
						int linkIndex = i;
						if (m_data->m_graphicsIndexToSegmentationMask.size() < (graphicsIndex + 1))
						{
							m_data->m_graphicsIndexToSegmentationMask.resize(graphicsIndex + 1);
						}
						int segmentationMask = bodyUniqueId + ((linkIndex + 1) << 24);
						m_data->m_graphicsIndexToSegmentationMask[graphicsIndex] = segmentationMask;
					}
				}

				std::string* jointName = new std::string(u2b.getJointName(urdfLinkIndex).c_str());
				m_data->m_strings.push_back(jointName);

				mb->getLink(i).m_jointName = jointName->c_str();

#ifdef B3_ENABLE_TINY_AUDIO
				{
					SDFAudioSource audioSource;
					int urdfLinkIndex = creation.m_mb2urdfLink[link];
					if (u2b.getLinkAudioSource(urdfLinkIndex, audioSource))
					{
						int flags = mb->getLink(link).m_collider->getCollisionFlags();
						mb->getLink(i).m_collider->setCollisionFlags(flags | btCollisionObject::CF_HAS_COLLISION_SOUND_TRIGGER);
						audioSource.m_userIndex = m_data->m_soundEngine.loadWavFile(audioSource.m_uri.c_str());
						if (audioSource.m_userIndex >= 0)
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
			
#if 0
			btAlignedObjectArray<char> urdf;
			mb->dumpUrdf(urdf);
			FILE* f = fopen("e:/pybullet.urdf", "w");
			if (f)
			{
				fwrite(&urdf[0], urdf.size(), 1, f);
				fclose(f);
			}
#endif
			
		}
		else
		{
			int segmentationMask = bodyUniqueId;
			if (rb)
			{
				int graphicsIndex = -1;
				{
					graphicsIndex = rb->getUserIndex();
				}
				if (graphicsIndex >= 0)
				{
					if (m_data->m_graphicsIndexToSegmentationMask.size() < (graphicsIndex + 1))
					{
						m_data->m_graphicsIndexToSegmentationMask.resize(graphicsIndex + 1);
					}
					m_data->m_graphicsIndexToSegmentationMask[graphicsIndex] = segmentationMask;
				}
			}

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
			for (int i = 0; i < numJoints; i++)
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

		{


			if (m_data->m_pluginManager.getRenderInterface())
			{
				int currentOpenGLTextureIndex=0;
				int totalNumVisualShapes = m_data->m_pluginManager.getRenderInterface()->getNumVisualShapes(bodyUniqueId);

				for (int shapeIndex=0;shapeIndex<totalNumVisualShapes;shapeIndex++)
				{
					b3VisualShapeData tmpShape;
					int success = m_data->m_pluginManager.getRenderInterface()->getVisualShapesData(bodyUniqueId, shapeIndex, &tmpShape);
					if (success)
					{
						if (tmpShape.m_tinyRendererTextureId >= 0)
						{
							int openglTextureUniqueId = -1;

							//find companion opengl texture unique id and create a 'textureUid'
							if (currentOpenGLTextureIndex<u2b.getNumAllocatedTextures())
							{
								openglTextureUniqueId = u2b.getAllocatedTexture(currentOpenGLTextureIndex++);
							}
							//if (openglTextureUniqueId>=0)
							{
								int texHandle = m_data->m_textureHandles.allocHandle();
								InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(texHandle);
								if (texH)
								{
									texH->m_tinyRendererTextureId = tmpShape.m_tinyRendererTextureId;
									texH->m_openglTextureId = openglTextureUniqueId;
								}
							}
						}
					}
				}
			}
		}

		b3Notification notification;
		notification.m_notificationType = BODY_ADDED;
		notification.m_bodyArgs.m_bodyUniqueId = bodyUniqueId;
		m_data->m_pluginManager.addNotification(notification);
	}

	for (int i = 0; i < u2b.getNumAllocatedTextures(); i++)
	{
		int texId = u2b.getAllocatedTexture(i);
		m_data->m_allocatedTextures.push_back(texId);
	}

	/*
 int texHandle = m_data->m_textureHandles.allocHandle();
 InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(texHandle);
 if(texH)
 {
 texH->m_tinyRendererTextureId = -1;
 texH->m_openglTextureId = -1;
 */

	for (int i = 0; i < u2b.getNumAllocatedMeshInterfaces(); i++)
	{
		m_data->m_meshInterfaces.push_back(u2b.getAllocatedMeshInterface(i));
	}

	for (int i = 0; i < u2b.getNumAllocatedCollisionShapes(); i++)
	{
		btCollisionShape* shape = u2b.getAllocatedCollisionShape(i);
		m_data->m_collisionShapes.push_back(shape);
		UrdfCollision urdfCollision;
		if (u2b.getUrdfFromCollisionShape(shape, urdfCollision))
		{
			m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, urdfCollision);
		}
		if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
		{
			btCompoundShape* compound = (btCompoundShape*)shape;
			for (int c = 0; c < compound->getNumChildShapes(); c++)
			{
				btCollisionShape* childShape = compound->getChildShape(c);
				if (u2b.getUrdfFromCollisionShape(childShape, urdfCollision))
				{
					m_data->m_bulletCollisionShape2UrdfCollision.insert(childShape, urdfCollision);
				}
			}
		}
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

	CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
	BulletMJCFImporter u2b(m_data->m_guiHelper, m_data->m_pluginManager.getRenderInterface(), fileIO, flags);

	bool useFixedBase = false;
	MyMJCFLogger2 logger;
	bool loadOk = u2b.loadMJCF(fileName, &logger, useFixedBase);
	if (loadOk)
	{
		processImportedObjects(fileName, bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, u2b);
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
	CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
	BulletURDFImporter u2b(m_data->m_guiHelper, m_data->m_pluginManager.getRenderInterface(), fileIO, globalScaling, flags);
	u2b.setEnableTinyRenderer(m_data->m_enableTinyRenderer);

	bool forceFixedBase = false;
	bool loadOk = u2b.loadSDF(fileName, forceFixedBase);

	if (loadOk)
	{
		processImportedObjects(fileName, bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, u2b);
	}
	return loadOk;
}

bool PhysicsServerCommandProcessor::loadUrdf(const char* fileName, const btVector3& pos, const btQuaternion& orn,
											 bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes, int orgFlags, btScalar globalScaling)
{
	//clear the LOAD_SDF_FILE=1 bit, which is reserved for internal use of loadSDF command.
	int flags = orgFlags & ~1;
	m_data->m_sdfRecentLoadedBodies.clear();
	*bodyUniqueIdPtr = -1;

	BT_PROFILE("loadURDF");
	btAssert(m_data->m_dynamicsWorld);
	if (!m_data->m_dynamicsWorld)
	{
		b3Error("loadUrdf: No valid m_dynamicsWorld");
		return false;
	}

	CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
	BulletURDFImporter u2b(m_data->m_guiHelper, m_data->m_pluginManager.getRenderInterface(), fileIO, globalScaling, flags);
	u2b.setEnableTinyRenderer(m_data->m_enableTinyRenderer);
	bool loadOk = u2b.loadURDF(fileName, useFixedBase);

	if (loadOk)
	{
		btTransform rootTrans;
		rootTrans.setOrigin(pos);
		rootTrans.setRotation(orn);
		u2b.setRootTransformInWorld(rootTrans);
		bool ok = processImportedObjects(fileName, bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, u2b);
		if (ok)
		{
			if (m_data->m_sdfRecentLoadedBodies.size() == 1)
			{
				*bodyUniqueIdPtr = m_data->m_sdfRecentLoadedBodies[0];
			}
			m_data->m_sdfRecentLoadedBodies.clear();
		}
		return ok;
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
			processCommand(clientCmd, serverStatus, bufferServerToClient, bufferSizeInBytes);
		}
	}
}

int PhysicsServerCommandProcessor::createBodyInfoStream(int bodyUniqueId, char* bufferServerToClient, int bufferSizeInBytes)
{
	int streamSizeInBytes = 0;
	//serialize the btMultiBody and send the data to the client. This is one way to get the link/joint names across the (shared memory) wire

	InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
	btMultiBody* mb = bodyHandle ? bodyHandle->m_multiBody : 0;
	if (mb)
	{
		UrdfLinkNameMapUtil utilBla;
		UrdfLinkNameMapUtil* util = &utilBla;
		btDefaultSerializer ser(bufferSizeInBytes, (unsigned char*)bufferServerToClient);

		util->m_mb = mb;
		util->m_memSerializer = &ser;
		util->m_memSerializer->startSerialization();

		//disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
		util->m_memSerializer->m_skipPointers.insert(mb->getBaseCollider(), 0);
		if (mb->getBaseName())
		{
			util->m_memSerializer->registerNameForPointer(mb->getBaseName(), mb->getBaseName());
		}

		bodyHandle->m_linkLocalInertialFrames.reserve(mb->getNumLinks());
		for (int i = 0; i < mb->getNumLinks(); i++)
		{
			//disable serialization of the collision objects
			util->m_memSerializer->m_skipPointers.insert(mb->getLink(i).m_collider, 0);
			util->m_memSerializer->registerNameForPointer(mb->getLink(i).m_linkName, mb->getLink(i).m_linkName);
			util->m_memSerializer->registerNameForPointer(mb->getLink(i).m_jointName, mb->getLink(i).m_jointName);
		}

		util->m_memSerializer->registerNameForPointer(mb->getBaseName(), mb->getBaseName());

		int len = mb->calculateSerializeBufferSize();
		btChunk* chunk = util->m_memSerializer->allocate(len, 1);
		const char* structType = mb->serialize(chunk->m_oldPtr, util->m_memSerializer);
		util->m_memSerializer->finalizeChunk(chunk, structType, BT_MULTIBODY_CODE, mb);
		streamSizeInBytes = util->m_memSerializer->getCurrentBufferSize();
	}
	else
	{
		btRigidBody* rb = bodyHandle ? bodyHandle->m_rigidBody : 0;
		if (rb)
		{
			UrdfLinkNameMapUtil utilBla;
			UrdfLinkNameMapUtil* util = &utilBla;
			btDefaultSerializer ser(bufferSizeInBytes, (unsigned char*)bufferServerToClient);

			util->m_memSerializer = &ser;
			util->m_memSerializer->startSerialization();
			util->m_memSerializer->registerNameForPointer(bodyHandle->m_rigidBody, bodyHandle->m_bodyName.c_str());
			//disable serialization of the collision objects (they are too big, and the client likely doesn't need them);
			for (int i = 0; i < bodyHandle->m_rigidBodyJoints.size(); i++)
			{
				const btGeneric6DofSpring2Constraint* con = bodyHandle->m_rigidBodyJoints.at(i);

				util->m_memSerializer->registerNameForPointer(con, bodyHandle->m_rigidBodyJointNames[i].c_str());
				util->m_memSerializer->registerNameForPointer(&con->getRigidBodyB(), bodyHandle->m_rigidBodyLinkNames[i].c_str());

				const btRigidBody& bodyA = con->getRigidBodyA();

				int len = con->calculateSerializeBufferSize();
				btChunk* chunk = util->m_memSerializer->allocate(len, 1);
				const char* structType = con->serialize(chunk->m_oldPtr, util->m_memSerializer);
				util->m_memSerializer->finalizeChunk(chunk, structType, BT_CONSTRAINT_CODE, (void*)con);
			}

			streamSizeInBytes = util->m_memSerializer->getCurrentBufferSize();
		}
	}

	return streamSizeInBytes;
}

bool PhysicsServerCommandProcessor::processStateLoggingCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_STATE_LOGGING");

	serverStatusOut.m_type = CMD_STATE_LOGGING_FAILED;
	bool hasStatus = true;

	if (clientCmd.m_updateFlags & STATE_LOGGING_START_LOG)
	{
		if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_ALL_COMMANDS)
		{
			if (m_data->m_commandLogger == 0)
			{
				enableCommandLogging(true, clientCmd.m_stateLoggingArguments.m_fileName);
				serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
				int loggerUid = m_data->m_stateLoggersUniqueId++;
				m_data->m_commandLoggingUid = loggerUid;
				serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
			}
		}

		if (clientCmd.m_stateLoggingArguments.m_logType == STATE_REPLAY_ALL_COMMANDS)
		{
			if (m_data->m_logPlayback == 0)
			{
				replayFromLogFile(clientCmd.m_stateLoggingArguments.m_fileName);
				serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
				int loggerUid = m_data->m_stateLoggersUniqueId++;
				m_data->m_logPlaybackUid = loggerUid;
				serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
			}
		}

		if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_PROFILE_TIMINGS)
		{
			if (m_data->m_profileTimingLoggingUid < 0)
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
				VideoMP4Loggger* logger = new VideoMP4Loggger(loggerUid, clientCmd.m_stateLoggingArguments.m_fileName, this->m_data->m_guiHelper);
				m_data->m_stateLoggers.push_back(logger);
				serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
				serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
			}
		}

		if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_MINITAUR)
		{
			std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
			//either provide the minitaur by object unique Id, or search for first multibody with 8 motors...

			if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID) && (clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds > 0))
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
						for (int m = 0; m < motorNames.size(); m++)
						{
							for (int i = 0; i < body->m_multiBody->getNumLinks(); i++)
							{
								std::string jointName;
								if (body->m_multiBody->getLink(i).m_jointName)
								{
									jointName = body->m_multiBody->getLink(i).m_jointName;
								}
								if (motorNames[m] == jointName)
								{
									motorIdList.push_back(i);
								}
							}
						}

						if (motorIdList.size() == 8)
						{
							int loggerUid = m_data->m_stateLoggersUniqueId++;
							MinitaurStateLogger* logger = new MinitaurStateLogger(loggerUid, fileName, body->m_multiBody, motorIdList);
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
			GenericRobotStateLogger* logger = new GenericRobotStateLogger(loggerUid, fileName, m_data->m_dynamicsWorld, maxLogDof, logFlags);

			if ((clientCmd.m_updateFlags & STATE_LOGGING_FILTER_OBJECT_UNIQUE_ID) && (clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds > 0))
			{
				logger->m_filterObjectUniqueId = true;
				for (int i = 0; i < clientCmd.m_stateLoggingArguments.m_numBodyUniqueIds; ++i)
				{
					int objectUniqueId = clientCmd.m_stateLoggingArguments.m_bodyUniqueIds[i];
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
			ContactPointsStateLogger* logger = new ContactPointsStateLogger(loggerUid, fileName, m_data->m_dynamicsWorld);
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
		if (clientCmd.m_stateLoggingArguments.m_logType == STATE_LOGGING_VR_CONTROLLERS)
		{
			std::string fileName = clientCmd.m_stateLoggingArguments.m_fileName;
			int loggerUid = m_data->m_stateLoggersUniqueId++;
			int deviceFilterType = VR_DEVICE_CONTROLLER;
			if (clientCmd.m_updateFlags & STATE_LOGGING_FILTER_DEVICE_TYPE)
			{
				deviceFilterType = clientCmd.m_stateLoggingArguments.m_deviceFilterType;
			}
			VRControllerStateLogger* logger = new VRControllerStateLogger(loggerUid, deviceFilterType, fileName);
			m_data->m_stateLoggers.push_back(logger);
			serverStatusOut.m_type = CMD_STATE_LOGGING_START_COMPLETED;
			serverStatusOut.m_stateLoggingResultArgs.m_loggingUniqueId = loggerUid;
		}
	}
	if ((clientCmd.m_updateFlags & STATE_LOGGING_STOP_LOG) && clientCmd.m_stateLoggingArguments.m_loggingUniqueId >= 0)
	{
		if (clientCmd.m_stateLoggingArguments.m_loggingUniqueId == m_data->m_logPlaybackUid)
		{
			if (m_data->m_logPlayback)
			{
				delete m_data->m_logPlayback;
				m_data->m_logPlayback = 0;
				m_data->m_logPlaybackUid = -1;
			}
		}

		if (clientCmd.m_stateLoggingArguments.m_loggingUniqueId == m_data->m_commandLoggingUid)
		{
			if (m_data->m_commandLogger)
			{
				enableCommandLogging(false, 0);
				serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
				m_data->m_commandLoggingUid = -1;
			}
		}

		if (clientCmd.m_stateLoggingArguments.m_loggingUniqueId == m_data->m_profileTimingLoggingUid)
		{
			serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
			b3ChromeUtilsStopTimingsAndWriteJsonFile(m_data->m_profileTimingFileName.c_str());
			m_data->m_profileTimingLoggingUid = -1;
		}
		else
		{
			serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
			for (int i = 0; i < m_data->m_stateLoggers.size(); i++)
			{
				if (m_data->m_stateLoggers[i]->m_loggingUniqueId == clientCmd.m_stateLoggingArguments.m_loggingUniqueId)
				{
					m_data->m_stateLoggers[i]->stop();
					delete m_data->m_stateLoggers[i];
					m_data->m_stateLoggers.removeAtIndex(i);
				}
			}
		}
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestCameraImageCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_CAMERA_IMAGE_DATA");
	int startPixelIndex = clientCmd.m_requestPixelDataArguments.m_startPixelIndex;
	int width = clientCmd.m_requestPixelDataArguments.m_pixelWidth;
	int height = clientCmd.m_requestPixelDataArguments.m_pixelHeight;
	int numPixelsCopied = 0;

	if ((clientCmd.m_requestPixelDataArguments.m_startPixelIndex == 0) &&
		(clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT) != 0)
	{
		if (m_data->m_pluginManager.getRenderInterface())
		{
			m_data->m_pluginManager.getRenderInterface()->setWidthAndHeight(clientCmd.m_requestPixelDataArguments.m_pixelWidth,
																			clientCmd.m_requestPixelDataArguments.m_pixelHeight);
		}
	}
	int flags = 0;
	if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_FLAGS)
	{
		flags = clientCmd.m_requestPixelDataArguments.m_flags;
	}
	if (m_data->m_pluginManager.getRenderInterface())
	{
		m_data->m_pluginManager.getRenderInterface()->setFlags(flags);
	}

	int numTotalPixels = width * height;
	int numRemainingPixels = numTotalPixels - startPixelIndex;

	if (numRemainingPixels > 0)
	{
		int totalBytesPerPixel = 4 + 4 + 4;  //4 for rgb, 4 for depth, 4 for segmentation mask
		int maxNumPixels = bufferSizeInBytes / totalBytesPerPixel - 1;
		unsigned char* pixelRGBA = (unsigned char*)bufferServerToClient;
		int numRequestedPixels = btMin(maxNumPixels, numRemainingPixels);

		float* depthBuffer = (float*)(bufferServerToClient + numRequestedPixels * 4);
		int* segmentationMaskBuffer = (int*)(bufferServerToClient + numRequestedPixels * 8);

		serverStatusOut.m_numDataStreamBytes = numRequestedPixels * totalBytesPerPixel;
		float viewMat[16];
		float projMat[16];
		float projTextureViewMat[16];
		float projTextureProjMat[16];
		for (int i = 0; i < 16; i++)
		{
			viewMat[i] = clientCmd.m_requestPixelDataArguments.m_viewMatrix[i];
			projMat[i] = clientCmd.m_requestPixelDataArguments.m_projectionMatrix[i];
		}
		if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES) == 0)
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
				for (int i = 0; i < 16; i++)
				{
					viewMat[i] = tmpCamResult.m_viewMatrix[i];
					projMat[i] = tmpCamResult.m_projectionMatrix[i];
				}
			}
		}
		bool handled = false;

		if ((clientCmd.m_updateFlags & ER_BULLET_HARDWARE_OPENGL) != 0)
		{
			if ((flags & ER_USE_PROJECTIVE_TEXTURE) != 0)
			{
				this->m_data->m_guiHelper->setProjectiveTexture(true);
				if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_PROJECTIVE_TEXTURE_MATRICES) != 0)
				{
					for (int i = 0; i < 16; i++)
					{
						projTextureViewMat[i] = clientCmd.m_requestPixelDataArguments.m_projectiveTextureViewMatrix[i];
						projTextureProjMat[i] = clientCmd.m_requestPixelDataArguments.m_projectiveTextureProjectionMatrix[i];
					}
				}
				else  // If no specified matrices for projective texture, then use the camera matrices.
				{
					for (int i = 0; i < 16; i++)
					{
						projTextureViewMat[i] = viewMat[i];
						projTextureProjMat[i] = projMat[i];
					}
				}
				this->m_data->m_guiHelper->setProjectiveTextureMatrices(projTextureViewMat, projTextureProjMat);
			}
			else
			{
				this->m_data->m_guiHelper->setProjectiveTexture(false);
			}

			if ((flags & ER_NO_SEGMENTATION_MASK) != 0)
			{
				segmentationMaskBuffer = 0;
			}
			m_data->m_guiHelper->copyCameraImageData(viewMat,
													 projMat, pixelRGBA, numRequestedPixels,
													 depthBuffer, numRequestedPixels,
													 segmentationMaskBuffer, numRequestedPixels,
													 startPixelIndex, width, height, &numPixelsCopied);

			if (numPixelsCopied > 0)
			{
				//convert segmentation mask

				if (segmentationMaskBuffer)
				{
					for (int i = 0; i < numPixelsCopied; i++)
					{
						int graphicsSegMask = segmentationMaskBuffer[i];
						int segMask = -1;
						if ((graphicsSegMask >= 0) && (graphicsSegMask < m_data->m_graphicsIndexToSegmentationMask.size()))
						{
							segMask = m_data->m_graphicsIndexToSegmentationMask[graphicsSegMask];
						}
						if ((flags & ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX) == 0)
						{
							if (segMask >= 0)
							{
								segMask &= ((1 << 24) - 1);
							}
						}
						segmentationMaskBuffer[i] = segMask;
					}
				}

				handled = true;
				m_data->m_guiHelper->debugDisplayCameraImageData(viewMat,
																 projMat, pixelRGBA, numRequestedPixels,
																 depthBuffer, numRequestedPixels,
																 segmentationMaskBuffer, numRequestedPixels,
																 startPixelIndex, width, height, &numPixelsCopied);
			}
		}
		if (!handled)
		{
			if (m_data->m_pluginManager.getRenderInterface())
			{
				if (clientCmd.m_requestPixelDataArguments.m_startPixelIndex == 0)
				{
					//   printf("-------------------------------\nRendering\n");

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DIRECTION) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightDirection(clientCmd.m_requestPixelDataArguments.m_lightDirection[0], clientCmd.m_requestPixelDataArguments.m_lightDirection[1], clientCmd.m_requestPixelDataArguments.m_lightDirection[2]);
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_COLOR) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightColor(clientCmd.m_requestPixelDataArguments.m_lightColor[0], clientCmd.m_requestPixelDataArguments.m_lightColor[1], clientCmd.m_requestPixelDataArguments.m_lightColor[2]);
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DISTANCE) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightDistance(clientCmd.m_requestPixelDataArguments.m_lightDistance);
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SHADOW) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setShadow((clientCmd.m_requestPixelDataArguments.m_hasShadow != 0));
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_AMBIENT_COEFF) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightAmbientCoeff(clientCmd.m_requestPixelDataArguments.m_lightAmbientCoeff);
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_DIFFUSE_COEFF) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightDiffuseCoeff(clientCmd.m_requestPixelDataArguments.m_lightDiffuseCoeff);
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SPECULAR_COEFF) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->setLightSpecularCoeff(clientCmd.m_requestPixelDataArguments.m_lightSpecularCoeff);
					}

					for (int i = 0; i < m_data->m_dynamicsWorld->getNumCollisionObjects(); i++)
					{
						const btCollisionObject* colObj = m_data->m_dynamicsWorld->getCollisionObjectArray()[i];
						m_data->m_pluginManager.getRenderInterface()->syncTransform(colObj->getBroadphaseHandle()->getUid(), colObj->getWorldTransform(), colObj->getCollisionShape()->getLocalScaling());
					}

					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES) != 0)
					{
						m_data->m_pluginManager.getRenderInterface()->render(
							clientCmd.m_requestPixelDataArguments.m_viewMatrix,
							clientCmd.m_requestPixelDataArguments.m_projectionMatrix);
					}
					else
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
							m_data->m_pluginManager.getRenderInterface()->render(tmpCamResult.m_viewMatrix, tmpCamResult.m_projectionMatrix);
						}
						else
						{
							m_data->m_pluginManager.getRenderInterface()->render();
						}
					}
				}
			}

			if (m_data->m_pluginManager.getRenderInterface())
			{
				if ((flags & ER_USE_PROJECTIVE_TEXTURE) != 0)
				{
					m_data->m_pluginManager.getRenderInterface()->setProjectiveTexture(true);
					if ((clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_PROJECTIVE_TEXTURE_MATRICES) != 0)
					{
						for (int i = 0; i < 16; i++)
						{
							projTextureViewMat[i] = clientCmd.m_requestPixelDataArguments.m_projectiveTextureViewMatrix[i];
							projTextureProjMat[i] = clientCmd.m_requestPixelDataArguments.m_projectiveTextureProjectionMatrix[i];
						}
					}
					else  // If no specified matrices for projective texture, then use the camera matrices.
					{
						for (int i = 0; i < 16; i++)
						{
							projTextureViewMat[i] = viewMat[i];
							projTextureProjMat[i] = projMat[i];
						}
					}
					m_data->m_pluginManager.getRenderInterface()->setProjectiveTextureMatrices(projTextureViewMat, projTextureProjMat);
				}
				else
				{
					m_data->m_pluginManager.getRenderInterface()->setProjectiveTexture(false);
				}

				if ((flags & ER_NO_SEGMENTATION_MASK) != 0)
				{
					segmentationMaskBuffer = 0;
				}

				m_data->m_pluginManager.getRenderInterface()->copyCameraImageData(pixelRGBA, numRequestedPixels,
																				  depthBuffer, numRequestedPixels,
																				  segmentationMaskBuffer, numRequestedPixels,
																				  startPixelIndex, &width, &height, &numPixelsCopied);
				m_data->m_pluginManager.getRenderInterface()->setProjectiveTexture(false);
			}

			m_data->m_guiHelper->debugDisplayCameraImageData(clientCmd.m_requestPixelDataArguments.m_viewMatrix,
															 clientCmd.m_requestPixelDataArguments.m_projectionMatrix, pixelRGBA, numRequestedPixels,
															 depthBuffer, numRequestedPixels,
															 segmentationMaskBuffer, numRequestedPixels,
															 startPixelIndex, width, height, &numPixelsCopied);
		}

		//each pixel takes 4 RGBA values and 1 float = 8 bytes
	}
	else
	{
	}

	serverStatusOut.m_type = CMD_CAMERA_IMAGE_COMPLETED;

	serverStatusOut.m_sendPixelDataArguments.m_numPixelsCopied = numPixelsCopied;
	serverStatusOut.m_sendPixelDataArguments.m_numRemainingPixels = numRemainingPixels - numPixelsCopied;
	serverStatusOut.m_sendPixelDataArguments.m_startingPixelIndex = startPixelIndex;
	serverStatusOut.m_sendPixelDataArguments.m_imageWidth = width;
	serverStatusOut.m_sendPixelDataArguments.m_imageHeight = height;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSaveWorldCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = false;
	BT_PROFILE("CMD_SAVE_WORLD");
	serverStatusOut.m_type = CMD_SAVE_WORLD_FAILED;

	///this is a very rudimentary way to save the state of the world, for scene authoring
	///many todo's, for example save the state of motor controllers etc.

	{
		//saveWorld(clientCmd.m_sdfArguments.m_sdfFileName);
		int constraintCount = 0;
		FILE* f = fopen(clientCmd.m_sdfArguments.m_sdfFileName, "w");
		if (f)
		{
			char line[1024];
			{
				sprintf(line, "import pybullet as p\n");
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}
			{
				sprintf(line, "cin = p.connect(p.SHARED_MEMORY)\n");
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}
			{
				sprintf(line, "if (cin < 0):\n");
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}
			{
				sprintf(line, "    cin = p.connect(p.GUI)\n");
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}

			//for each objects ...
			for (int i = 0; i < m_data->m_saveWorldBodyData.size(); i++)
			{
				SaveWorldObjectData& sd = m_data->m_saveWorldBodyData[i];

				for (int i = 0; i < sd.m_bodyUniqueIds.size(); i++)
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

								if (strstr(sd.m_fileName.c_str(), ".urdf"))
								{
									sprintf(line, "objects = [p.loadURDF(\"%s\", %f,%f,%f,%f,%f,%f,%f)]\n", sd.m_fileName.c_str(),
											tr.getOrigin()[0], tr.getOrigin()[1], tr.getOrigin()[2],
											tr.getRotation()[0], tr.getRotation()[1], tr.getRotation()[2], tr.getRotation()[3]);
									int len = strlen(line);
									fwrite(line, len, 1, f);
								}

								if (strstr(sd.m_fileName.c_str(), ".sdf") && i == 0)
								{
									sprintf(line, "objects = p.loadSDF(\"%s\")\n", sd.m_fileName.c_str());
									int len = strlen(line);
									fwrite(line, len, 1, f);
								}
								if (strstr(sd.m_fileName.c_str(), ".xml") && i == 0)
								{
									sprintf(line, "objects = p.loadMJCF(\"%s\")\n", sd.m_fileName.c_str());
									int len = strlen(line);
									fwrite(line, len, 1, f);
								}

								if (strstr(sd.m_fileName.c_str(), ".sdf") || strstr(sd.m_fileName.c_str(), ".xml") || ((strstr(sd.m_fileName.c_str(), ".urdf")) && mb->getNumLinks()))
								{
									sprintf(line, "ob = objects[%d]\n", i);
									int len = strlen(line);
									fwrite(line, len, 1, f);
								}

								if (strstr(sd.m_fileName.c_str(), ".sdf") || strstr(sd.m_fileName.c_str(), ".xml"))
								{
									sprintf(line, "p.resetBasePositionAndOrientation(ob,[%f,%f,%f],[%f,%f,%f,%f])\n",
											comTr.getOrigin()[0], comTr.getOrigin()[1], comTr.getOrigin()[2],
											comTr.getRotation()[0], comTr.getRotation()[1], comTr.getRotation()[2], comTr.getRotation()[3]);
									int len = strlen(line);
									fwrite(line, len, 1, f);
								}

								if (mb->getNumLinks())
								{
									{
										sprintf(line, "jointPositions=[");
										int len = strlen(line);
										fwrite(line, len, 1, f);
									}

									for (int i = 0; i < mb->getNumLinks(); i++)
									{
										btScalar jointPos = mb->getJointPosMultiDof(i)[0];
										if (i < mb->getNumLinks() - 1)
										{
											sprintf(line, " %f,", jointPos);
											int len = strlen(line);
											fwrite(line, len, 1, f);
										}
										else
										{
											sprintf(line, " %f ", jointPos);
											int len = strlen(line);
											fwrite(line, len, 1, f);
										}
									}

									{
										sprintf(line, "]\nfor jointIndex in range (p.getNumJoints(ob)):\n\tp.resetJointState(ob,jointIndex,jointPositions[jointIndex])\n\n");
										int len = strlen(line);
										fwrite(line, len, 1, f);
									}
								}
							}
							else
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
					std::string m_fileName;
				};
			}

			//user constraints
			{
				for (int i = 0; i < m_data->m_userConstraints.size(); i++)
				{
					InteralUserConstraintData* ucptr = m_data->m_userConstraints.getAtIndex(i);
					b3UserConstraint& uc = ucptr->m_userConstraintData;

					int parentBodyIndex = uc.m_parentBodyIndex;
					int parentJointIndex = uc.m_parentJointIndex;
					int childBodyIndex = uc.m_childBodyIndex;
					int childJointIndex = uc.m_childJointIndex;
					btVector3 jointAxis(uc.m_jointAxis[0], uc.m_jointAxis[1], uc.m_jointAxis[2]);
					btVector3 pivotParent(uc.m_parentFrame[0], uc.m_parentFrame[1], uc.m_parentFrame[2]);
					btVector3 pivotChild(uc.m_childFrame[0], uc.m_childFrame[1], uc.m_childFrame[2]);
					btQuaternion ornFrameParent(uc.m_parentFrame[3], uc.m_parentFrame[4], uc.m_parentFrame[5], uc.m_parentFrame[6]);
					btQuaternion ornFrameChild(uc.m_childFrame[3], uc.m_childFrame[4], uc.m_childFrame[5], uc.m_childFrame[6]);
					{
						char jointTypeStr[1024] = "FIXED";
						bool hasKnownJointType = true;

						switch (uc.m_jointType)
						{
							case eRevoluteType:
							{
								sprintf(jointTypeStr, "p.JOINT_REVOLUTE");
								break;
							}
							case ePrismaticType:
							{
								sprintf(jointTypeStr, "p.JOINT_PRISMATIC");
								break;
							}
							case eSphericalType:
							{
								sprintf(jointTypeStr, "p.JOINT_SPHERICAL");
								break;
							}
							case ePlanarType:
							{
								sprintf(jointTypeStr, "p.JOINT_PLANAR");
								break;
							}
							case eFixedType:
							{
								sprintf(jointTypeStr, "p.JOINT_FIXED");
								break;
							}
							case ePoint2PointType:
							{
								sprintf(jointTypeStr, "p.JOINT_POINT2POINT");
								break;
							}
							case eGearType:
							{
								sprintf(jointTypeStr, "p.JOINT_GEAR");
								break;
							}
							default:
							{
								hasKnownJointType = false;
								b3Warning("unknown constraint type in SAVE_WORLD");
							}
						};
						if (hasKnownJointType)
						{
							{
								sprintf(line, "cid%d = p.createConstraint(%d,%d,%d,%d,%s,[%f,%f,%f],[%f,%f,%f],[%f,%f,%f],[%f,%f,%f,%f],[%f,%f,%f,%f])\n",
										constraintCount,
										parentBodyIndex,
										parentJointIndex,
										childBodyIndex,
										childJointIndex,
										jointTypeStr,
										jointAxis[0], jointAxis[1], jointAxis[2],
										pivotParent[0], pivotParent[1], pivotParent[2],
										pivotChild[0], pivotChild[1], pivotChild[2],
										ornFrameParent[0], ornFrameParent[1], ornFrameParent[2], ornFrameParent[3],
										ornFrameChild[0], ornFrameChild[1], ornFrameChild[2], ornFrameChild[3]);
								int len = strlen(line);
								fwrite(line, len, 1, f);
							}
							{
								sprintf(line, "p.changeConstraint(cid%d,maxForce=%f)\n", constraintCount, uc.m_maxAppliedForce);
								int len = strlen(line);
								fwrite(line, len, 1, f);
								constraintCount++;
							}
						}
					}
				}
			}

			{
				btVector3 grav = this->m_data->m_dynamicsWorld->getGravity();
				sprintf(line, "p.setGravity(%f,%f,%f)\n", grav[0], grav[1], grav[2]);
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}

			{
				sprintf(line, "p.stepSimulation()\np.disconnect()\n");
				int len = strlen(line);
				fwrite(line, len, 1, f);
			}
			fclose(f);
		}

		serverStatusOut.m_type = CMD_SAVE_WORLD_COMPLETED;
		hasStatus = true;
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateCollisionShapeCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_CREATE_COLLISION_SHAPE_FAILED;

	btMultiBodyWorldImporter* worldImporter = new btMultiBodyWorldImporter(m_data->m_dynamicsWorld);

	btCollisionShape* shape = 0;
	b3AlignedObjectArray<UrdfCollision> urdfCollisionObjects;

	btCompoundShape* compound = 0;

	if (clientCmd.m_createUserShapeArgs.m_numUserShapes > 1)
	{
		compound = worldImporter->createCompoundShape();
		compound->setMargin(m_data->m_defaultCollisionMargin);
	}
	for (int i = 0; i < clientCmd.m_createUserShapeArgs.m_numUserShapes; i++)
	{
		GLInstanceGraphicsShape* glmesh = 0;
		char pathPrefix[1024] = "";
		char relativeFileName[1024] = "";
		UrdfCollision urdfColObj;

		btTransform childTransform;
		childTransform.setIdentity();
		if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_hasChildTransform)
		{
			childTransform.setOrigin(btVector3(clientCmd.m_createUserShapeArgs.m_shapes[i].m_childPosition[0],
											   clientCmd.m_createUserShapeArgs.m_shapes[i].m_childPosition[1],
											   clientCmd.m_createUserShapeArgs.m_shapes[i].m_childPosition[2]));
			childTransform.setRotation(btQuaternion(
				clientCmd.m_createUserShapeArgs.m_shapes[i].m_childOrientation[0],
				clientCmd.m_createUserShapeArgs.m_shapes[i].m_childOrientation[1],
				clientCmd.m_createUserShapeArgs.m_shapes[i].m_childOrientation[2],
				clientCmd.m_createUserShapeArgs.m_shapes[i].m_childOrientation[3]));
			if (compound == 0)
			{
				compound = worldImporter->createCompoundShape();
			}
		}

		urdfColObj.m_linkLocalFrame = childTransform;
		urdfColObj.m_sourceFileLocation = "memory";
		urdfColObj.m_name = "memory";
		urdfColObj.m_geometry.m_type = URDF_GEOM_UNKNOWN;

		switch (clientCmd.m_createUserShapeArgs.m_shapes[i].m_type)
		{
			case GEOM_SPHERE:
			{
				double radius = clientCmd.m_createUserShapeArgs.m_shapes[i].m_sphereRadius;
				shape = worldImporter->createSphereShape(radius);
				shape->setMargin(m_data->m_defaultCollisionMargin);
				if (compound)
				{
					compound->addChildShape(childTransform, shape);
				}
				urdfColObj.m_geometry.m_type = URDF_GEOM_SPHERE;
				urdfColObj.m_geometry.m_sphereRadius = radius;
				break;
			}
			case GEOM_BOX:
			{
				//double halfExtents[3] = clientCmd.m_createUserShapeArgs.m_shapes[i].m_sphereRadius;
				btVector3 halfExtents(
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_boxHalfExtents[0],
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_boxHalfExtents[1],
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_boxHalfExtents[2]);
				shape = worldImporter->createBoxShape(halfExtents);
				shape->setMargin(m_data->m_defaultCollisionMargin);
				if (compound)
				{
					compound->addChildShape(childTransform, shape);
				}
				urdfColObj.m_geometry.m_type = URDF_GEOM_BOX;
				urdfColObj.m_geometry.m_boxSize = 2. * halfExtents;
				break;
			}
			case GEOM_CAPSULE:
			{
				shape = worldImporter->createCapsuleShapeZ(clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleRadius,
														   clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleHeight);
				shape->setMargin(m_data->m_defaultCollisionMargin);
				if (compound)
				{
					compound->addChildShape(childTransform, shape);
				}
				urdfColObj.m_geometry.m_type = URDF_GEOM_CAPSULE;
				urdfColObj.m_geometry.m_capsuleRadius = clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleRadius;
				urdfColObj.m_geometry.m_capsuleHeight = clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleHeight;

				break;
			}
			case GEOM_CYLINDER:
			{
				shape = worldImporter->createCylinderShapeZ(clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleRadius,
															0.5 * clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleHeight);
				shape->setMargin(m_data->m_defaultCollisionMargin);
				if (compound)
				{
					compound->addChildShape(childTransform, shape);
				}
				urdfColObj.m_geometry.m_type = URDF_GEOM_CYLINDER;
				urdfColObj.m_geometry.m_capsuleRadius = clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleRadius;
				urdfColObj.m_geometry.m_capsuleHeight = clientCmd.m_createUserShapeArgs.m_shapes[i].m_capsuleHeight;

				break;
			}
			case GEOM_PLANE:
			{
				btVector3 planeNormal(clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[0],
									  clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[1],
									  clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[2]);

				shape = worldImporter->createPlaneShape(planeNormal, clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeConstant);
				shape->setMargin(m_data->m_defaultCollisionMargin);
				if (compound)
				{
					compound->addChildShape(childTransform, shape);
				}
				urdfColObj.m_geometry.m_type = URDF_GEOM_PLANE;
				urdfColObj.m_geometry.m_planeNormal.setValue(
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[0],
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[1],
					clientCmd.m_createUserShapeArgs.m_shapes[i].m_planeNormal[2]);

				break;
			}
			case GEOM_MESH:
			{
				btVector3 meshScale(clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[0],
									clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[1],
									clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[2]);

				const std::string& urdf_path = "";

				std::string fileName = clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshFileName;
				urdfColObj.m_geometry.m_type = URDF_GEOM_MESH;
				urdfColObj.m_geometry.m_meshFileName = fileName;

				urdfColObj.m_geometry.m_meshScale = meshScale;
				CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
				pathPrefix[0] = 0;
				if (fileIO->findResourcePath(fileName.c_str(), relativeFileName, 1024))
				{
					b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
				}

				const std::string& error_message_prefix = "";
				std::string out_found_filename;
				int out_type;

				if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_numVertices)
				{
					if (compound == 0)
					{
						compound = worldImporter->createCompoundShape();
					}
					compound->setMargin(m_data->m_defaultCollisionMargin);

					if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_numIndices)
					{
						BT_PROFILE("convert trimesh2");
						btTriangleMesh* meshInterface = new btTriangleMesh();
						this->m_data->m_meshInterfaces.push_back(meshInterface);
						{
							BT_PROFILE("convert vertices2");

							for (int j = 0; j < clientCmd.m_createUserShapeArgs.m_shapes[i].m_numIndices / 3; j++)
							{
								int i0 = clientCmd.m_createUserShapeArgs.m_shapes[i].m_indices[j*3+0];
								int i1 = clientCmd.m_createUserShapeArgs.m_shapes[i].m_indices[j*3+1];
								int i2 = clientCmd.m_createUserShapeArgs.m_shapes[i].m_indices[j*3+2];

								btVector3 v0(	clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i0*3+0],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i0*3+1],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i0*3+2]);
								btVector3 v1(	clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i1*3+0],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i1*3+1],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i1*3+2]);
								btVector3 v2(	clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i2*3+0],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i2*3+1],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[i2*3+2]);
								meshInterface->addTriangle(v0, v1, v2);
							}
						}

						{
							BT_PROFILE("create btBvhTriangleMeshShape");
							btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface, true, true);
							m_data->m_collisionShapes.push_back(trimesh);

							if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_collisionFlags & GEOM_CONCAVE_INTERNAL_EDGE)
							{
								btTriangleInfoMap* triangleInfoMap = new btTriangleInfoMap();
								btGenerateInternalEdgeInfo(trimesh, triangleInfoMap);
							}
							shape = trimesh;
							if (compound)
							{
								compound->addChildShape(childTransform, shape);
								shape->setMargin(m_data->m_defaultCollisionMargin);
							}
						}
					}
					else
					{
						btConvexHullShape* convexHull = worldImporter->createConvexHullShape();
						convexHull->setMargin(m_data->m_defaultCollisionMargin);

						for (int v = 0; v < clientCmd.m_createUserShapeArgs.m_shapes[i].m_numVertices; v++)
						{

							btVector3 pt(	clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[v*3+0],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[v*3+1],
											clientCmd.m_createUserShapeArgs.m_shapes[i].m_vertices[v*3+2]);
							convexHull->addPoint(pt, false);
						}

						convexHull->recalcLocalAabb();
						convexHull->optimizeConvexHull();
						compound->addChildShape(childTransform, convexHull);
					}
					urdfColObj.m_geometry.m_meshFileType = UrdfGeometry::MEMORY_VERTICES;
					break;
				}

				bool foundFile = UrdfFindMeshFile(fileIO, pathPrefix, relativeFileName, error_message_prefix, &out_found_filename, &out_type);
				if (foundFile)
				{
					urdfColObj.m_geometry.m_meshFileType = out_type;

					if (out_type == UrdfGeometry::FILE_STL)
					{
						CommonFileIOInterface* fileIO(m_data->m_pluginManager.getFileIOInterface());
						glmesh = LoadMeshFromSTL(relativeFileName,fileIO);
					}
					if (out_type == UrdfGeometry::FILE_OBJ)
					{
						//create a convex hull for each shape, and store it in a btCompoundShape

						if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_collisionFlags & GEOM_FORCE_CONCAVE_TRIMESH)
						{
							CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
							glmesh = LoadMeshFromObj(relativeFileName, pathPrefix,fileIO);
						}
						else
						{
							std::vector<tinyobj::shape_t> shapes;
							std::string err = tinyobj::LoadObj(shapes, out_found_filename.c_str(),"",fileIO);

							//shape = createConvexHullFromShapes(shapes, collision->m_geometry.m_meshScale);
							//static btCollisionShape* createConvexHullFromShapes(std::vector<tinyobj::shape_t>& shapes, const btVector3& geomScale)
							B3_PROFILE("createConvexHullFromShapes");
							if (compound == 0)
							{
								compound = worldImporter->createCompoundShape();
							}
							compound->setMargin(m_data->m_defaultCollisionMargin);

							for (int s = 0; s < (int)shapes.size(); s++)
							{
								btConvexHullShape* convexHull = worldImporter->createConvexHullShape();
								convexHull->setMargin(m_data->m_defaultCollisionMargin);
								tinyobj::shape_t& shape = shapes[s];
								int faceCount = shape.mesh.indices.size();

								for (int f = 0; f < faceCount; f += 3)
								{
									btVector3 pt;
									pt.setValue(shape.mesh.positions[shape.mesh.indices[f] * 3 + 0],
												shape.mesh.positions[shape.mesh.indices[f] * 3 + 1],
												shape.mesh.positions[shape.mesh.indices[f] * 3 + 2]);

									convexHull->addPoint(pt * meshScale, false);

									pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 0],
												shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 1],
												shape.mesh.positions[shape.mesh.indices[f + 1] * 3 + 2]);
									convexHull->addPoint(pt * meshScale, false);

									pt.setValue(shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 0],
												shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 1],
												shape.mesh.positions[shape.mesh.indices[f + 2] * 3 + 2]);
									convexHull->addPoint(pt * meshScale, false);
								}

								convexHull->recalcLocalAabb();
								convexHull->optimizeConvexHull();
								compound->addChildShape(childTransform, convexHull);
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

		if (glmesh)
		{
			btVector3 meshScale(clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[0],
								clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[1],
								clientCmd.m_createUserShapeArgs.m_shapes[i].m_meshScale[2]);

			if (!glmesh || glmesh->m_numvertices <= 0)
			{
				b3Warning("%s: cannot extract mesh from '%s'\n", pathPrefix, relativeFileName);
				delete glmesh;
			}
			else
			{
				btAlignedObjectArray<btVector3> convertedVerts;
				convertedVerts.reserve(glmesh->m_numvertices);

				for (int v = 0; v < glmesh->m_numvertices; v++)
				{
					convertedVerts.push_back(btVector3(
						glmesh->m_vertices->at(v).xyzw[0] * meshScale[0],
						glmesh->m_vertices->at(v).xyzw[1] * meshScale[1],
						glmesh->m_vertices->at(v).xyzw[2] * meshScale[2]));
				}

				if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_collisionFlags & GEOM_FORCE_CONCAVE_TRIMESH)
				{
					BT_PROFILE("convert trimesh");
					btTriangleMesh* meshInterface = new btTriangleMesh();
					this->m_data->m_meshInterfaces.push_back(meshInterface);
					{
						BT_PROFILE("convert vertices");

						for (int i = 0; i < glmesh->m_numIndices / 3; i++)
						{
							const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i * 3)];
							const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i * 3 + 1)];
							const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i * 3 + 2)];
							meshInterface->addTriangle(v0, v1, v2);
						}
					}

					{
						BT_PROFILE("create btBvhTriangleMeshShape");
						btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface, true, true);
						m_data->m_collisionShapes.push_back(trimesh);

						if (clientCmd.m_createUserShapeArgs.m_shapes[i].m_collisionFlags & GEOM_CONCAVE_INTERNAL_EDGE)
						{
							btTriangleInfoMap* triangleInfoMap = new btTriangleInfoMap();
							btGenerateInternalEdgeInfo(trimesh, triangleInfoMap);
						}
						//trimesh->setLocalScaling(collision->m_geometry.m_meshScale);
						shape = trimesh;
						if (compound)
						{
							compound->addChildShape(childTransform, shape);
							shape->setMargin(m_data->m_defaultCollisionMargin);
						}
					}
					delete glmesh;
				}
				else
				{
					//convex mesh

					if (compound == 0)
					{
						compound = worldImporter->createCompoundShape();
					}
					compound->setMargin(m_data->m_defaultCollisionMargin);

					{
						btConvexHullShape* convexHull = worldImporter->createConvexHullShape();
						convexHull->setMargin(m_data->m_defaultCollisionMargin);

						for (int v = 0; v < convertedVerts.size(); v++)
						{
							btVector3 pt = convertedVerts[v];
							convexHull->addPoint(pt, false);
						}

						convexHull->recalcLocalAabb();
						convexHull->optimizeConvexHull();
						compound->addChildShape(childTransform, convexHull);
					}
				}
			}
		}
	}
	if (compound && compound->getNumChildShapes())
	{
		shape = compound;
	}

	if (shape)
	{
		int collisionShapeUid = m_data->m_userCollisionShapeHandles.allocHandle();
		InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(collisionShapeUid);
		handle->m_collisionShape = shape;
		for (int i = 0; i < urdfCollisionObjects.size(); i++)
		{
			handle->m_urdfCollisionObjects.push_back(urdfCollisionObjects[i]);
		}
		serverStatusOut.m_createUserShapeResultArgs.m_userShapeUniqueId = collisionShapeUid;
		m_data->m_worldImporters.push_back(worldImporter);
		serverStatusOut.m_type = CMD_CREATE_COLLISION_SHAPE_COMPLETED;
	}
	else
	{
		delete worldImporter;
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateVisualShapeCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_CREATE_VISUAL_SHAPE_FAILED;
	double globalScaling = 1.f;
	int flags = 0;
	CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
	BulletURDFImporter u2b(m_data->m_guiHelper, m_data->m_pluginManager.getRenderInterface(), fileIO, globalScaling, flags);
	u2b.setEnableTinyRenderer(m_data->m_enableTinyRenderer);
	btTransform localInertiaFrame;
	localInertiaFrame.setIdentity();

	const char* pathPrefix = "";
	int visualShapeUniqueId = -1;

	UrdfVisual visualShape;
	for (int userShapeIndex = 0; userShapeIndex < clientCmd.m_createUserShapeArgs.m_numUserShapes; userShapeIndex++)
	{
		btTransform childTrans;
		childTrans.setIdentity();
		visualShape.m_geometry.m_type = (UrdfGeomTypes)clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_type;
		char relativeFileName[1024];
		char pathPrefix[1024];
		pathPrefix[0] = 0;

		const b3CreateUserShapeData& visShape = clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex];

		switch (visualShape.m_geometry.m_type)
		{
			case URDF_GEOM_CYLINDER:
			{
				visualShape.m_geometry.m_capsuleHeight = visShape.m_capsuleHeight;
				visualShape.m_geometry.m_capsuleRadius = visShape.m_capsuleRadius;
				break;
			}
			case URDF_GEOM_BOX:
			{
				visualShape.m_geometry.m_boxSize.setValue(2. * visShape.m_boxHalfExtents[0],
														  2. * visShape.m_boxHalfExtents[1],
														  2. * visShape.m_boxHalfExtents[2]);
				break;
			}
			case URDF_GEOM_SPHERE:
			{
				visualShape.m_geometry.m_sphereRadius = visShape.m_sphereRadius;
				break;
			}
			case URDF_GEOM_CAPSULE:
			{
				visualShape.m_geometry.m_hasFromTo = visShape.m_hasFromTo;
				if (visualShape.m_geometry.m_hasFromTo)
				{
					visualShape.m_geometry.m_capsuleFrom.setValue(visShape.m_capsuleFrom[0],
																  visShape.m_capsuleFrom[1],
																  visShape.m_capsuleFrom[2]);
					visualShape.m_geometry.m_capsuleTo.setValue(visShape.m_capsuleTo[0],
																visShape.m_capsuleTo[1],
																visShape.m_capsuleTo[2]);
				}
				else
				{
					visualShape.m_geometry.m_capsuleHeight = visShape.m_capsuleHeight;
					visualShape.m_geometry.m_capsuleRadius = visShape.m_capsuleRadius;
				}
				break;
			}
			case URDF_GEOM_MESH:
			{
				std::string fileName = clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_meshFileName;
				const std::string& error_message_prefix = "";
				std::string out_found_filename;
				int out_type;
				if (fileIO->findResourcePath(fileName.c_str(), relativeFileName, 1024))
				{
					b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
				}

				bool foundFile = UrdfFindMeshFile(fileIO, pathPrefix, relativeFileName, error_message_prefix, &out_found_filename, &out_type);
				visualShape.m_geometry.m_meshFileType = out_type;
				visualShape.m_geometry.m_meshFileName = fileName;

				visualShape.m_geometry.m_meshScale.setValue(clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_meshScale[0],
															clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_meshScale[1],
															clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_meshScale[2]);
				break;
			}

			default:
			{
			}
		};
		visualShape.m_name = "in_memory";
		visualShape.m_materialName = "";
		visualShape.m_sourceFileLocation = "in_memory_unknown_line";
		visualShape.m_linkLocalFrame.setIdentity();
		visualShape.m_geometry.m_hasLocalMaterial = false;

		bool hasRGBA = (clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_visualFlags & GEOM_VISUAL_HAS_RGBA_COLOR) != 0;
		;
		bool hasSpecular = (clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_visualFlags & GEOM_VISUAL_HAS_SPECULAR_COLOR) != 0;
		;
		visualShape.m_geometry.m_hasLocalMaterial = hasRGBA | hasSpecular;
		if (visualShape.m_geometry.m_hasLocalMaterial)
		{
			if (hasRGBA)
			{
				visualShape.m_geometry.m_localMaterial.m_matColor.m_rgbaColor.setValue(
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_rgbaColor[0],
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_rgbaColor[1],
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_rgbaColor[2],
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_rgbaColor[3]);
			}
			else
			{
				visualShape.m_geometry.m_localMaterial.m_matColor.m_rgbaColor.setValue(1,1,1,1);
			}
			if (hasSpecular)
			{
				visualShape.m_geometry.m_localMaterial.m_matColor.m_specularColor.setValue(
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_specularColor[0],
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_specularColor[1],
					clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_specularColor[2]);
			}
			else
			{
				visualShape.m_geometry.m_localMaterial.m_matColor.m_specularColor.setValue(0.4, 0.4, 0.4);
			}
		}

		if (clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_hasChildTransform != 0)
		{
			childTrans.setOrigin(btVector3(clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childPosition[0],
										   clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childPosition[1],
										   clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childPosition[2]));
			childTrans.setRotation(btQuaternion(
				clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childOrientation[0],
				clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childOrientation[1],
				clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childOrientation[2],
				clientCmd.m_createUserShapeArgs.m_shapes[userShapeIndex].m_childOrientation[3]));
		}

		if (visualShapeUniqueId < 0)
		{
			visualShapeUniqueId = m_data->m_userVisualShapeHandles.allocHandle();
		}
		InternalVisualShapeHandle* visualHandle = m_data->m_userVisualShapeHandles.getHandle(visualShapeUniqueId);
		visualHandle->m_OpenGLGraphicsIndex = -1;
		visualHandle->m_tinyRendererVisualShapeIndex = -1;
		//tinyrenderer doesn't separate shape versus instance, so create it when creating the multibody instance
		//store needed info for tinyrenderer

		visualShape.m_linkLocalFrame = childTrans;
		visualHandle->m_visualShapes.push_back(visualShape);
		visualHandle->m_pathPrefixes.push_back(pathPrefix[0] ? pathPrefix : "");

		serverStatusOut.m_createUserShapeResultArgs.m_userShapeUniqueId = visualShapeUniqueId;
		serverStatusOut.m_type = CMD_CREATE_VISUAL_SHAPE_COMPLETED;
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCustomCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CUSTOM_COMMAND_FAILED;
	serverCmd.m_customCommandResultArgs.m_pluginUniqueId = -1;

	if (clientCmd.m_updateFlags & CMD_CUSTOM_COMMAND_LOAD_PLUGIN)
	{
		//pluginPath could be registered or load from disk
		const char* postFix = "";
		if (clientCmd.m_updateFlags & CMD_CUSTOM_COMMAND_LOAD_PLUGIN_POSTFIX)
		{
			postFix = clientCmd.m_customCommandArgs.m_postFix;
		}

		int pluginUniqueId = m_data->m_pluginManager.loadPlugin(clientCmd.m_customCommandArgs.m_pluginPath, postFix);
		if (pluginUniqueId >= 0)
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processUserDebugDrawCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_USER_DEBUG_DRAW");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_USER_DEBUG_DRAW_FAILED;

	int trackingVisualShapeIndex = -1;

	if (clientCmd.m_userDebugDrawArgs.m_parentObjectUniqueId >= 0)
	{
		InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_userDebugDrawArgs.m_parentObjectUniqueId);
		if (bodyHandle)
		{
			int linkIndex = -1;

			if (bodyHandle->m_multiBody)
			{
				int linkIndex = clientCmd.m_userDebugDrawArgs.m_parentLinkIndex;
				if (linkIndex == -1)
				{
					if (bodyHandle->m_multiBody->getBaseCollider())
					{
						trackingVisualShapeIndex = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
					}
				}
				else
				{
					if (linkIndex >= 0 && linkIndex < bodyHandle->m_multiBody->getNumLinks())
					{
						if (bodyHandle->m_multiBody->getLink(linkIndex).m_collider)
						{
							trackingVisualShapeIndex = bodyHandle->m_multiBody->getLink(linkIndex).m_collider->getUserIndex();
						}
					}
				}
			}
			if (bodyHandle->m_rigidBody)
			{
				trackingVisualShapeIndex = bodyHandle->m_rigidBody->getUserIndex();
			}
		}
	}

	if (clientCmd.m_updateFlags & USER_DEBUG_ADD_PARAMETER)
	{
		int uid = m_data->m_guiHelper->addUserDebugParameter(
			clientCmd.m_userDebugDrawArgs.m_text,
			clientCmd.m_userDebugDrawArgs.m_rangeMin,
			clientCmd.m_userDebugDrawArgs.m_rangeMax,
			clientCmd.m_userDebugDrawArgs.m_startValue);
		serverCmd.m_userDebugDrawArgs.m_debugItemUniqueId = uid;
		serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
	}
	if (clientCmd.m_updateFlags & USER_DEBUG_READ_PARAMETER)
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

		if ((clientCmd.m_updateFlags & USER_DEBUG_HAS_TEXT_ORIENTATION) == 0)
		{
			optionFlags |= DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA;
		}

		int replaceItemUniqueId = -1;
		if ((clientCmd.m_updateFlags & USER_DEBUG_HAS_REPLACE_ITEM_UNIQUE_ID) != 0)
		{
			replaceItemUniqueId = clientCmd.m_userDebugDrawArgs.m_replaceItemUniqueId;
		}

		int uid = m_data->m_guiHelper->addUserDebugText3D(clientCmd.m_userDebugDrawArgs.m_text,
														  clientCmd.m_userDebugDrawArgs.m_textPositionXYZ,
														  clientCmd.m_userDebugDrawArgs.m_textOrientation,
														  clientCmd.m_userDebugDrawArgs.m_textColorRGB,
														  clientCmd.m_userDebugDrawArgs.m_textSize,
														  clientCmd.m_userDebugDrawArgs.m_lifeTime,
														  trackingVisualShapeIndex,
														  optionFlags,
														  replaceItemUniqueId);

		if (uid >= 0)
		{
			serverCmd.m_userDebugDrawArgs.m_debugItemUniqueId = uid;
			serverCmd.m_type = CMD_USER_DEBUG_DRAW_COMPLETED;
		}
	}

	if (clientCmd.m_updateFlags & USER_DEBUG_HAS_LINE)
	{
		int replaceItemUid = -1;
		if (clientCmd.m_updateFlags & USER_DEBUG_HAS_REPLACE_ITEM_UNIQUE_ID)
		{
			replaceItemUid = clientCmd.m_userDebugDrawArgs.m_replaceItemUniqueId;
		}

		int uid = m_data->m_guiHelper->addUserDebugLine(
			clientCmd.m_userDebugDrawArgs.m_debugLineFromXYZ,
			clientCmd.m_userDebugDrawArgs.m_debugLineToXYZ,
			clientCmd.m_userDebugDrawArgs.m_debugLineColorRGB,
			clientCmd.m_userDebugDrawArgs.m_lineWidth,
			clientCmd.m_userDebugDrawArgs.m_lifeTime,
			trackingVisualShapeIndex,
			replaceItemUid);

		if (uid >= 0)
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSetVRCameraStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SET_VR_CAMERA_STATE");

	if (clientCmd.m_updateFlags & VR_CAMERA_ROOT_POSITION)
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

	serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestVREventsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	//BT_PROFILE("CMD_REQUEST_VR_EVENTS_DATA");
	serverStatusOut.m_sendVREvents.m_numVRControllerEvents = 0;

	for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
	{
		b3VRControllerEvent& event = m_data->m_vrControllerEvents.m_vrEvents[i];

		if (clientCmd.m_updateFlags & event.m_deviceType)
		{
			if (event.m_numButtonEvents + event.m_numMoveEvents)
			{
				serverStatusOut.m_sendVREvents.m_controllerEvents[serverStatusOut.m_sendVREvents.m_numVRControllerEvents++] = event;
				event.m_numButtonEvents = 0;
				event.m_numMoveEvents = 0;
				for (int b = 0; b < MAX_VR_BUTTONS; b++)
				{
					event.m_buttons[b] = 0;
				}
			}
		}
	}
	serverStatusOut.m_type = CMD_REQUEST_VR_EVENTS_DATA_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestMouseEventsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_sendMouseEvents.m_numMouseEvents = m_data->m_mouseEvents.size();
	if (serverStatusOut.m_sendMouseEvents.m_numMouseEvents > MAX_MOUSE_EVENTS)
	{
		serverStatusOut.m_sendMouseEvents.m_numMouseEvents = MAX_MOUSE_EVENTS;
	}
	for (int i = 0; i < serverStatusOut.m_sendMouseEvents.m_numMouseEvents; i++)
	{
		serverStatusOut.m_sendMouseEvents.m_mouseEvents[i] = m_data->m_mouseEvents[i];
	}

	m_data->m_mouseEvents.resize(0);
	serverStatusOut.m_type = CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestKeyboardEventsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	//BT_PROFILE("CMD_REQUEST_KEYBOARD_EVENTS_DATA");
	bool hasStatus = true;
	serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents = m_data->m_keyboardEvents.size();
	if (serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents > MAX_KEYBOARD_EVENTS)
	{
		serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents = MAX_KEYBOARD_EVENTS;
	}
	for (int i = 0; i < serverStatusOut.m_sendKeyboardEvents.m_numKeyboardEvents; i++)
	{
		serverStatusOut.m_sendKeyboardEvents.m_keyboardEvents[i] = m_data->m_keyboardEvents[i];
	}

	btAlignedObjectArray<b3KeyboardEvent> events;

	//remove out-of-date events
	for (int i = 0; i < m_data->m_keyboardEvents.size(); i++)
	{
		b3KeyboardEvent event = m_data->m_keyboardEvents[i];
		if (event.m_keyState & eButtonIsDown)
		{
			event.m_keyState = eButtonIsDown;
			events.push_back(event);
		}
	}
	m_data->m_keyboardEvents.resize(events.size());
	for (int i = 0; i < events.size(); i++)
	{
		m_data->m_keyboardEvents[i] = events[i];
	}

	serverStatusOut.m_type = CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED;
	return hasStatus;
}

#if __cplusplus >= 201103L
#include <atomic>

struct CastSyncInfo
{
	std::atomic<int> m_nextTaskNumber;

	CastSyncInfo() : m_nextTaskNumber(0) {}

	inline int getNextTask()
	{
		return m_nextTaskNumber++;
	}
};
#else   // __cplusplus >= 201103L
struct CastSyncInfo
{
	volatile int m_nextTaskNumber;
	btSpinMutex m_taskLock;

	CastSyncInfo() : m_nextTaskNumber(0) {}

	inline int getNextTask()
	{
		m_taskLock.lock();
		const int taskNr = m_nextTaskNumber++;
		m_taskLock.unlock();
		return taskNr;
	}
};
#endif  // __cplusplus >= 201103L

struct BatchRayCaster
{
	b3ThreadPool* m_threadPool;
	CastSyncInfo* m_syncInfo;
	const btCollisionWorld* m_world;
	const b3RayData* m_rayInputBuffer;
	b3RayHitInfo* m_hitInfoOutputBuffer;
	int m_numRays;

	BatchRayCaster(b3ThreadPool* threadPool, const btCollisionWorld* world, const b3RayData* rayInputBuffer, b3RayHitInfo* hitInfoOutputBuffer, int numRays)
		: m_threadPool(threadPool), m_world(world), m_rayInputBuffer(rayInputBuffer), m_hitInfoOutputBuffer(hitInfoOutputBuffer), m_numRays(numRays)
	{
		m_syncInfo = new CastSyncInfo;
	}

	~BatchRayCaster()
	{
		delete m_syncInfo;
	}

	void castRays(int numWorkers)
	{
#if BT_THREADSAFE
		if (numWorkers <= 1)
		{
			castSequentially();
		}
		else
		{
			{
				BT_PROFILE("BatchRayCaster_startingWorkerThreads");
				int numTasks = btMin(m_threadPool->numWorkers(), numWorkers - 1);
				for (int i = 0; i < numTasks; i++)
				{
					m_threadPool->runTask(i, BatchRayCaster::rayCastWorker, this);
				}
			}
			rayCastWorker(this);
			m_threadPool->waitForAllTasks();
		}
#else   // BT_THREADSAFE
		castSequentially();
#endif  // BT_THREADSAFE
	}

	static void rayCastWorker(void* arg)
	{
		BT_PROFILE("BatchRayCaster_raycastWorker");
		BatchRayCaster* const obj = (BatchRayCaster*)arg;
		const int numRays = obj->m_numRays;
		int taskNr;
		while (true)
		{
			{
				BT_PROFILE("CastSyncInfo_getNextTask");
				taskNr = obj->m_syncInfo->getNextTask();
			}
			if (taskNr >= numRays)
				return;
			obj->processRay(taskNr);
		}
	}

	void castSequentially()
	{
		for (int i = 0; i < m_numRays; i++)
		{
			processRay(i);
		}
	}

	void processRay(int ray)
	{
		BT_PROFILE("BatchRayCaster_processRay");
		const double* from = m_rayInputBuffer[ray].m_rayFromPosition;
		const double* to = m_rayInputBuffer[ray].m_rayToPosition;
		btVector3 rayFromWorld(from[0], from[1], from[2]);
		btVector3 rayToWorld(to[0], to[1], to[2]);

		btCollisionWorld::ClosestRayResultCallback rayResultCallback(rayFromWorld, rayToWorld);
		rayResultCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;

		m_world->rayTest(rayFromWorld, rayToWorld, rayResultCallback);

		b3RayHitInfo& hit = m_hitInfoOutputBuffer[ray];
		if (rayResultCallback.hasHit())
		{
			hit.m_hitFraction = rayResultCallback.m_closestHitFraction;

			int objectUniqueId = -1;
			int linkIndex = -1;

			const btRigidBody* body = btRigidBody::upcast(rayResultCallback.m_collisionObject);
			if (body)
			{
				objectUniqueId = rayResultCallback.m_collisionObject->getUserIndex2();
			}
			else
			{
				const btMultiBodyLinkCollider* mblB = btMultiBodyLinkCollider::upcast(rayResultCallback.m_collisionObject);
				if (mblB && mblB->m_multiBody)
				{
					linkIndex = mblB->m_link;
					objectUniqueId = mblB->m_multiBody->getUserIndex2();
				}
			}

			hit.m_hitObjectUniqueId = objectUniqueId;
			hit.m_hitObjectLinkIndex = linkIndex;

			hit.m_hitPositionWorld[0] = rayResultCallback.m_hitPointWorld[0];
			hit.m_hitPositionWorld[1] = rayResultCallback.m_hitPointWorld[1];
			hit.m_hitPositionWorld[2] = rayResultCallback.m_hitPointWorld[2];
			hit.m_hitNormalWorld[0] = rayResultCallback.m_hitNormalWorld[0];
			hit.m_hitNormalWorld[1] = rayResultCallback.m_hitNormalWorld[1];
			hit.m_hitNormalWorld[2] = rayResultCallback.m_hitNormalWorld[2];
		}
		else
		{
			hit.m_hitFraction = 1;
			hit.m_hitObjectUniqueId = -1;
			hit.m_hitObjectLinkIndex = -1;
			hit.m_hitPositionWorld[0] = 0;
			hit.m_hitPositionWorld[1] = 0;
			hit.m_hitPositionWorld[2] = 0;
			hit.m_hitNormalWorld[0] = 0;
			hit.m_hitNormalWorld[1] = 0;
			hit.m_hitNormalWorld[2] = 0;
		}
	}
};

void PhysicsServerCommandProcessor::createThreadPool()
{
#ifdef BT_THREADSAFE
	if (m_data->m_threadPool == 0)
	{
		m_data->m_threadPool = new b3ThreadPool("PhysicsServerCommandProcessorThreadPool");
	}
#endif  //BT_THREADSAFE
}

bool PhysicsServerCommandProcessor::processRequestRaycastIntersectionsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_RAY_CAST_INTERSECTIONS");
	serverStatusOut.m_raycastHits.m_numRaycastHits = 0;

	const int numCommandRays = clientCmd.m_requestRaycastIntersections.m_numCommandRays;
	const int numStreamingRays = clientCmd.m_requestRaycastIntersections.m_numStreamingRays;
	const int totalRays = numCommandRays + numStreamingRays;
	int numThreads = clientCmd.m_requestRaycastIntersections.m_numThreads;
	if (numThreads == 0)
	{
		// When 0 is specified, Bullet can decide how many threads to use.
		// About 16 rays per thread seems to work reasonably well.
		numThreads = btMax(1, totalRays / 16);
	}
	if (numThreads > 1)
	{
		createThreadPool();
	}

	btAlignedObjectArray<b3RayData> rays;
	rays.resize(totalRays);
	if (numCommandRays)
	{
		memcpy(&rays[0], &clientCmd.m_requestRaycastIntersections.m_fromToRays[0], numCommandRays * sizeof(b3RayData));
	}
	if (numStreamingRays)
	{
		memcpy(&rays[numCommandRays], bufferServerToClient, numStreamingRays * sizeof(b3RayData));
	}

	if (clientCmd.m_requestRaycastIntersections.m_parentObjectUniqueId>=0)
	{
		btTransform tr;
		tr.setIdentity();

		InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_requestRaycastIntersections.m_parentObjectUniqueId);
		if (bodyHandle)
		{
			int linkIndex = -1;
			if (bodyHandle->m_multiBody)
			{
				int linkIndex = clientCmd.m_userDebugDrawArgs.m_parentLinkIndex;
				if (linkIndex == -1)
				{
					tr = bodyHandle->m_multiBody->getBaseWorldTransform();
				}
				else
				{
					if (linkIndex >= 0 && linkIndex < bodyHandle->m_multiBody->getNumLinks())
					{
						tr = bodyHandle->m_multiBody->getLink(linkIndex).m_cachedWorldTransform;
					}
				}
			}
			if (bodyHandle->m_rigidBody)
			{
				tr = bodyHandle->m_rigidBody->getWorldTransform();
			}
			//convert all rays into world space
			for (int i=0;i<totalRays;i++)
			{
				btVector3 localPosTo(rays[i].m_rayToPosition[0],rays[i].m_rayToPosition[1],rays[i].m_rayToPosition[2]);
				btVector3 worldPosTo = tr*localPosTo;

				btVector3 localPosFrom(rays[i].m_rayFromPosition[0],rays[i].m_rayFromPosition[1],rays[i].m_rayFromPosition[2]);
				btVector3 worldPosFrom = tr*localPosFrom;
				rays[i].m_rayFromPosition[0] = worldPosFrom[0];
				rays[i].m_rayFromPosition[1] = worldPosFrom[1];
				rays[i].m_rayFromPosition[2] = worldPosFrom[2];
				rays[i].m_rayToPosition[0] = worldPosTo[0];
				rays[i].m_rayToPosition[1] = worldPosTo[1];
				rays[i].m_rayToPosition[2] = worldPosTo[2];
			}
		}
	}


	BatchRayCaster batchRayCaster(m_data->m_threadPool, m_data->m_dynamicsWorld, &rays[0], (b3RayHitInfo*)bufferServerToClient, totalRays);
	batchRayCaster.castRays(numThreads);

	serverStatusOut.m_raycastHits.m_numRaycastHits = totalRays;
	serverStatusOut.m_type = CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestDebugLinesCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_DEBUG_LINES");
	int curFlags = m_data->m_remoteDebugDrawer->getDebugMode();

	int debugMode = clientCmd.m_requestDebugLinesArguments.m_debugMode;  //clientCmd.btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb;
	int startingLineIndex = clientCmd.m_requestDebugLinesArguments.m_startingLineIndex;
	if (startingLineIndex < 0)
	{
		b3Warning("startingLineIndex should be non-negative");
		startingLineIndex = 0;
	}

	if (clientCmd.m_requestDebugLinesArguments.m_startingLineIndex == 0)
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
	int maxNumLines = bufferSizeInBytes / bytesPerLine - 1;
	if (startingLineIndex > m_data->m_remoteDebugDrawer->m_lines2.size())
	{
		b3Warning("m_startingLineIndex exceeds total number of debug lines");
		startingLineIndex = m_data->m_remoteDebugDrawer->m_lines2.size();
	}

	int numLines = btMin(maxNumLines, m_data->m_remoteDebugDrawer->m_lines2.size() - startingLineIndex);

	if (numLines)
	{
		float* linesFrom = (float*)bufferServerToClient;
		float* linesTo = (float*)(bufferServerToClient + numLines * 3 * sizeof(float));
		float* linesColor = (float*)(bufferServerToClient + 2 * numLines * 3 * sizeof(float));

		for (int i = 0; i < numLines; i++)
		{
			linesFrom[i * 3] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_from.x();
			linesTo[i * 3] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_to.x();
			linesColor[i * 3] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_color.x();

			linesFrom[i * 3 + 1] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_from.y();
			linesTo[i * 3 + 1] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_to.y();
			linesColor[i * 3 + 1] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_color.y();

			linesFrom[i * 3 + 2] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_from.z();
			linesTo[i * 3 + 2] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_to.z();
			linesColor[i * 3 + 2] = m_data->m_remoteDebugDrawer->m_lines2[i + startingLineIndex].m_color.z();
		}
	}

	serverStatusOut.m_type = CMD_DEBUG_LINES_COMPLETED;
	serverStatusOut.m_numDataStreamBytes = numLines * bytesPerLine;
	serverStatusOut.m_sendDebugLinesArgs.m_numDebugLines = numLines;
	serverStatusOut.m_sendDebugLinesArgs.m_startingLineIndex = startingLineIndex;
	serverStatusOut.m_sendDebugLinesArgs.m_numRemainingDebugLines = m_data->m_remoteDebugDrawer->m_lines2.size() - (startingLineIndex + numLines);

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSyncBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_BODY_INFO");

	b3AlignedObjectArray<int> usedHandles;
	m_data->m_bodyHandles.getUsedHandles(usedHandles);
	int actualNumBodies = 0;
	for (int i = 0; i < usedHandles.size(); i++)
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
	for (int i = 0; i < usz; i++)
	{
		int key = m_data->m_userConstraints.getKeyAtIndex(i).getUid1();
		//						int uid = m_data->m_userConstraints.getAtIndex(i)->m_userConstraintData.m_userConstraintUniqueId;
		serverStatusOut.m_sdfLoadedArgs.m_userConstraintUniqueIds[i] = key;
	}

	serverStatusOut.m_type = CMD_SYNC_BODY_INFO_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSyncUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_USER_DATA");

	b3AlignedObjectArray<int> userDataHandles;
	m_data->m_userDataHandles.getUsedHandles(userDataHandles);
	if (userDataHandles.size())
	{
		memcpy(bufferServerToClient, &userDataHandles[0], sizeof(int) * userDataHandles.size());
	}
	serverStatusOut.m_syncUserDataArgs.m_numUserDataIdentifiers = userDataHandles.size();
	serverStatusOut.m_type = CMD_SYNC_USER_DATA_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_USER_DATA");
	serverStatusOut.m_type = CMD_REQUEST_USER_DATA_FAILED;

	SharedMemoryUserData* userData = m_data->m_userDataHandles.getHandle(clientCmd.m_userDataRequestArgs.m_userDataId);
	if (!userData)
	{
		return hasStatus;
	}

	btAssert(bufferSizeInBytes >= userData->m_bytes.size());
	serverStatusOut.m_userDataResponseArgs.m_userDataId = clientCmd.m_userDataRequestArgs.m_userDataId;
	serverStatusOut.m_userDataResponseArgs.m_bodyUniqueId = userData->m_bodyUniqueId;
	serverStatusOut.m_userDataResponseArgs.m_linkIndex = userData->m_linkIndex;
	serverStatusOut.m_userDataResponseArgs.m_visualShapeIndex = userData->m_visualShapeIndex;
	serverStatusOut.m_userDataResponseArgs.m_valueType = userData->m_type;
	serverStatusOut.m_userDataResponseArgs.m_valueLength = userData->m_bytes.size();
	serverStatusOut.m_type = CMD_REQUEST_USER_DATA_COMPLETED;

	strcpy(serverStatusOut.m_userDataResponseArgs.m_key, userData->m_key.c_str());
	if (userData->m_bytes.size())
	{
		memcpy(bufferServerToClient, &userData->m_bytes[0], userData->m_bytes.size());
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processAddUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_ADD_USER_DATA");
	serverStatusOut.m_type = CMD_ADD_USER_DATA_FAILED;

	if (clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId < 0 || clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId >= m_data->m_bodyHandles.getNumHandles())
	{
		return hasStatus;
	}

	InternalBodyData* body = m_data->m_bodyHandles.getHandle(clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId);
	if (!body)
	{
		return hasStatus;
	}

	SharedMemoryUserDataHashKey userDataIdentifier(
		clientCmd.m_addUserDataRequestArgs.m_key,
		clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId,
		clientCmd.m_addUserDataRequestArgs.m_linkIndex,
		clientCmd.m_addUserDataRequestArgs.m_visualShapeIndex);

	int* userDataHandlePtr = m_data->m_userDataHandleLookup.find(userDataIdentifier);
	int userDataHandle = userDataHandlePtr ? *userDataHandlePtr : m_data->m_userDataHandles.allocHandle();

	SharedMemoryUserData* userData = m_data->m_userDataHandles.getHandle(userDataHandle);
	if (!userData)
	{
		return hasStatus;
	}

	if (!userDataHandlePtr)
	{
		userData->m_key = clientCmd.m_addUserDataRequestArgs.m_key;
		userData->m_bodyUniqueId = clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId;
		userData->m_linkIndex = clientCmd.m_addUserDataRequestArgs.m_linkIndex;
		userData->m_visualShapeIndex = clientCmd.m_addUserDataRequestArgs.m_visualShapeIndex;
		m_data->m_userDataHandleLookup.insert(userDataIdentifier, userDataHandle);
		body->m_userDataHandles.push_back(userDataHandle);
	}
	userData->replaceValue(bufferServerToClient,
						   clientCmd.m_addUserDataRequestArgs.m_valueLength,
						   clientCmd.m_addUserDataRequestArgs.m_valueType);

	serverStatusOut.m_type = CMD_ADD_USER_DATA_COMPLETED;
	serverStatusOut.m_userDataResponseArgs.m_userDataId = userDataHandle;
	serverStatusOut.m_userDataResponseArgs.m_bodyUniqueId = clientCmd.m_addUserDataRequestArgs.m_bodyUniqueId;
	serverStatusOut.m_userDataResponseArgs.m_linkIndex = clientCmd.m_addUserDataRequestArgs.m_linkIndex;
	serverStatusOut.m_userDataResponseArgs.m_visualShapeIndex = clientCmd.m_addUserDataRequestArgs.m_visualShapeIndex;
	serverStatusOut.m_userDataResponseArgs.m_valueLength = clientCmd.m_addUserDataRequestArgs.m_valueLength;
	serverStatusOut.m_userDataResponseArgs.m_valueType = clientCmd.m_addUserDataRequestArgs.m_valueType;
	strcpy(serverStatusOut.m_userDataResponseArgs.m_key, clientCmd.m_addUserDataRequestArgs.m_key);

	b3Notification notification;
	notification.m_notificationType = USER_DATA_ADDED;
	notification.m_userDataArgs.m_userDataId = userDataHandle;
	m_data->m_pluginManager.addNotification(notification);

	// Keep bufferServerToClient as-is.
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCollisionFilterCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	b3PluginCollisionInterface* collisionInterface = m_data->m_pluginManager.getCollisionInterface();
	if (collisionInterface)
	{
		if (clientCmd.m_updateFlags & B3_COLLISION_FILTER_PAIR)
		{
			collisionInterface->setBroadphaseCollisionFilter(clientCmd.m_collisionFilterArgs.m_bodyUniqueIdA,
															 clientCmd.m_collisionFilterArgs.m_bodyUniqueIdB,
															 clientCmd.m_collisionFilterArgs.m_linkIndexA,
															 clientCmd.m_collisionFilterArgs.m_linkIndexB,
															 clientCmd.m_collisionFilterArgs.m_enableCollision);

			btAlignedObjectArray<InternalBodyData*> bodies;

			//now also 'refresh' the broadphase collision pairs involved
			if (clientCmd.m_collisionFilterArgs.m_bodyUniqueIdA >= 0)
			{
				bodies.push_back(m_data->m_bodyHandles.getHandle(clientCmd.m_collisionFilterArgs.m_bodyUniqueIdA));
			}
			if (clientCmd.m_collisionFilterArgs.m_bodyUniqueIdB >= 0)
			{
				bodies.push_back(m_data->m_bodyHandles.getHandle(clientCmd.m_collisionFilterArgs.m_bodyUniqueIdB));
			}
			for (int i = 0; i < bodies.size(); i++)
			{
				InternalBodyData* body = bodies[i];
				if (body)
				{
					if (body->m_multiBody)
					{
						if (body->m_multiBody->getBaseCollider())
						{
							m_data->m_dynamicsWorld->refreshBroadphaseProxy(body->m_multiBody->getBaseCollider());
						}
						for (int i = 0; i < body->m_multiBody->getNumLinks(); i++)
						{
							if (body->m_multiBody->getLinkCollider(i))
							{
								m_data->m_dynamicsWorld->refreshBroadphaseProxy(body->m_multiBody->getLinkCollider(i));
							}
						}
					}
					else
					{
						//btRigidBody case
						if (body->m_rigidBody)
						{
							m_data->m_dynamicsWorld->refreshBroadphaseProxy(body->m_rigidBody);
						}
					}
				}
			}
		}
		if (clientCmd.m_updateFlags & B3_COLLISION_FILTER_GROUP_MASK)
		{
			InternalBodyData* body = m_data->m_bodyHandles.getHandle(clientCmd.m_collisionFilterArgs.m_bodyUniqueIdA);
			btCollisionObject* colObj = 0;
			if (body->m_multiBody)
			{
				if (clientCmd.m_collisionFilterArgs.m_linkIndexA)
				{
					colObj = body->m_multiBody->getBaseCollider();
				}
				else
				{
					if (clientCmd.m_collisionFilterArgs.m_linkIndexA >= 0 && clientCmd.m_collisionFilterArgs.m_linkIndexA < body->m_multiBody->getNumLinks())
					{
						colObj = body->m_multiBody->getLinkCollider(clientCmd.m_collisionFilterArgs.m_linkIndexA);
					}
				}
			}
			else
			{
				if (body->m_rigidBody)
				{
					colObj = body->m_rigidBody;
				}
			}
			if (colObj)
			{
				colObj->getBroadphaseHandle()->m_collisionFilterGroup = clientCmd.m_collisionFilterArgs.m_collisionFilterGroup;
				colObj->getBroadphaseHandle()->m_collisionFilterMask = clientCmd.m_collisionFilterArgs.m_collisionFilterMask;
				m_data->m_dynamicsWorld->refreshBroadphaseProxy(colObj);
			}
		}
	}
	return true;
}

bool PhysicsServerCommandProcessor::processRemoveUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REMOVE_USER_DATA");
	serverStatusOut.m_type = CMD_REMOVE_USER_DATA_FAILED;

	SharedMemoryUserData* userData = m_data->m_userDataHandles.getHandle(clientCmd.m_removeUserDataRequestArgs.m_userDataId);
	if (!userData)
	{
		return hasStatus;
	}

	InternalBodyData* body = m_data->m_bodyHandles.getHandle(userData->m_bodyUniqueId);
	if (!body)
	{
		return hasStatus;
	}
	body->m_userDataHandles.remove(clientCmd.m_removeUserDataRequestArgs.m_userDataId);

	m_data->m_userDataHandleLookup.remove(SharedMemoryUserDataHashKey(userData));
	m_data->m_userDataHandles.freeHandle(clientCmd.m_removeUserDataRequestArgs.m_userDataId);

	serverStatusOut.m_removeUserDataResponseArgs = clientCmd.m_removeUserDataRequestArgs;
	serverStatusOut.m_type = CMD_REMOVE_USER_DATA_COMPLETED;

	b3Notification notification;
	notification.m_notificationType = USER_DATA_REMOVED;
	notification.m_userDataArgs.m_userDataId = clientCmd.m_removeUserDataRequestArgs.m_userDataId;
	m_data->m_pluginManager.addNotification(notification);

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSendDesiredStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
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
			case CONTROL_MODE_PD:
			{
				if (m_data->m_verboseOutput)
				{
					b3Printf("Using CONTROL_MODE_PD");
				}

				b3PluginArguments args;
				args.m_ints[1] = bodyUniqueId;
				//find the joint motors and apply the desired velocity and maximum force/torque
				{
					args.m_numInts = 0;
					args.m_numFloats = 0;
					//syncBodies is expensive/slow, use it only once
					m_data->m_pluginManager.executePluginCommand(m_data->m_pdControlPlugin, &args);

					int velIndex = 6;  //skip the 3 linear + 3 angular degree of freedom velocity entries of the base
					int posIndex = 7;  //skip 3 positional and 4 orientation (quaternion) positional degrees of freedom of the base
					for (int link = 0; link < mb->getNumLinks(); link++)
					{
						if (supportsJointMotor(mb, link))
						{
							bool hasDesiredPosOrVel = false;
							btScalar desiredVelocity = 0.f;
							if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT) != 0)
							{
								hasDesiredPosOrVel = true;
								desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
								args.m_floats[2] = 0.1;  // kd
							}
							btScalar desiredPosition = 0.f;
							if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q) != 0)
							{
								hasDesiredPosOrVel = true;
								desiredPosition = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex];
								args.m_floats[3] = 0.1;  // kp
							}

							if (hasDesiredPosOrVel)
							{
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KP) != 0)
								{
									args.m_floats[3] = clientCmd.m_sendDesiredStateCommandArgument.m_Kp[velIndex];
								}
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KD) != 0)
								{
									args.m_floats[2] = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[velIndex];
								}

								args.m_floats[1] = desiredVelocity;
								//clamp position
								if (mb->getLink(link).m_jointLowerLimit <= mb->getLink(link).m_jointUpperLimit)
								{
									btClamp(desiredPosition, mb->getLink(link).m_jointLowerLimit, mb->getLink(link).m_jointUpperLimit);
								}
								args.m_floats[0] = desiredPosition;

								btScalar maxImp = 1000000.f;
								if ((clientCmd.m_updateFlags & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
									maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex];
								args.m_floats[4] = maxImp;

								args.m_ints[2] = link;
								args.m_numInts = 3;
								args.m_numFloats = 5;

								args.m_ints[0] = eSetPDControl;
								if (args.m_floats[4] < B3_EPSILON)
								{
									args.m_ints[0] = eRemovePDControl;
								}
								m_data->m_pluginManager.executePluginCommand(m_data->m_pdControlPlugin, &args);
							}
						}
						velIndex += mb->getLink(link).m_dofCount;
						posIndex += mb->getLink(link).m_posVarCount;
					}
				}
				break;
			}
			case CONTROL_MODE_TORQUE:
			{
				if (m_data->m_verboseOutput)
				{
					b3Printf("Using CONTROL_MODE_TORQUE");
				}
				//  mb->clearForcesAndTorques();
				int torqueIndex = 6;
				if ((clientCmd.m_updateFlags & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
				{
					for (int link = 0; link < mb->getNumLinks(); link++)
					{
						for (int dof = 0; dof < mb->getLink(link).m_dofCount; dof++)
						{
							double torque = 0.f;
							if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[torqueIndex] & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
							{
								torque = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[torqueIndex];
								mb->addJointTorqueMultiDof(link, dof, torque);
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
					int dofIndex = 6;  //skip the 3 linear + 3 angular degree of freedom entries of the base
					for (int link = 0; link < mb->getNumLinks(); link++)
					{
						if (supportsJointMotor(mb, link))
						{
							btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;

							if (motor)
							{
								btScalar desiredVelocity = 0.f;
								bool hasDesiredVelocity = false;

								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] & SIM_DESIRED_STATE_HAS_QDOT) != 0)
								{
									desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[dofIndex];
									btScalar kd = 0.1f;
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] & SIM_DESIRED_STATE_HAS_KD) != 0)
									{
										kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[dofIndex];
									}

									motor->setVelocityTarget(desiredVelocity, kd);

									btScalar kp = 0.f;
									motor->setPositionTarget(0, kp);
									hasDesiredVelocity = true;
								}
								if (hasDesiredVelocity)
								{
									//disable velocity clamp in velocity mode
									motor->setRhsClamp(SIMD_INFINITY);

									btScalar maxImp = 1000000.f * m_data->m_physicsDeltaTime;
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[dofIndex] & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
									{
										maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[dofIndex] * m_data->m_physicsDeltaTime;
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
					int velIndex = 6;  //skip the 3 linear + 3 angular degree of freedom velocity entries of the base
					int posIndex = 7;  //skip 3 positional and 4 orientation (quaternion) positional degrees of freedom of the base
					for (int link = 0; link < mb->getNumLinks(); link++)
					{
						if (supportsJointMotor(mb, link))
						{
							btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)mb->getLink(link).m_userPtr;

							if (motor)
							{
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_RHS_CLAMP) != 0)
								{
									motor->setRhsClamp(clientCmd.m_sendDesiredStateCommandArgument.m_rhsClamp[velIndex]);
								}

								bool hasDesiredPosOrVel = false;
								btScalar kp = 0.f;
								btScalar kd = 0.f;
								btScalar desiredVelocity = 0.f;
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT) != 0)
								{
									hasDesiredPosOrVel = true;
									desiredVelocity = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
									kd = 0.1;
								}
								btScalar desiredPosition = 0.f;
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q) != 0)
								{
									hasDesiredPosOrVel = true;
									desiredPosition = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex];
									kp = 0.1;
								}

								if (hasDesiredPosOrVel)
								{
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KP) != 0)
									{
										kp = clientCmd.m_sendDesiredStateCommandArgument.m_Kp[velIndex];
									}

									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KD) != 0)
									{
										kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[velIndex];
									}

									motor->setVelocityTarget(desiredVelocity, kd);
									//todo: instead of clamping, combine the motor and limit
									//and combine handling of limit force and motor force.

									//clamp position
									if (mb->getLink(link).m_jointLowerLimit <= mb->getLink(link).m_jointUpperLimit)
									{
										btClamp(desiredPosition, mb->getLink(link).m_jointLowerLimit, mb->getLink(link).m_jointUpperLimit);
									}
									motor->setPositionTarget(desiredPosition, kp);

									btScalar maxImp = 1000000.f * m_data->m_physicsDeltaTime;

									if ((clientCmd.m_updateFlags & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
										maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex] * m_data->m_physicsDeltaTime;

									motor->setMaxAppliedImpulse(maxImp);
								}
								numMotors++;
							}
						}

						if (mb->getLink(link).m_jointType == btMultibodyLink::eSpherical)
						{
							btMultiBodySphericalJointMotor* motor = (btMultiBodySphericalJointMotor*)mb->getLink(link).m_userPtr;
							if (motor)
							{
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_RHS_CLAMP) != 0)
								{
									motor->setRhsClamp(clientCmd.m_sendDesiredStateCommandArgument.m_rhsClamp[velIndex]);
								}
								bool hasDesiredPosOrVel = false;
								btScalar kp = 0.f;
								btScalar kd = 0.f;
								btVector3 desiredVelocity(0, 0, 0);
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT) != 0)
								{
									hasDesiredPosOrVel = true;
									desiredVelocity.setValue(
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex+0],
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex+1],
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex+2]);
									kd = 0.1;
								}
								btQuaternion desiredPosition(0, 0, 0, 1);
								if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q) != 0)
								{
									hasDesiredPosOrVel = true;
									desiredPosition.setValue(
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex + 0],
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex + 1],
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex + 2],
										clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[posIndex + 3]);
									kp = 0.1;
								}

								if (hasDesiredPosOrVel)
								{
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KP) != 0)
									{
										kp = clientCmd.m_sendDesiredStateCommandArgument.m_Kp[velIndex];
									}

									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_KD) != 0)
									{
										kd = clientCmd.m_sendDesiredStateCommandArgument.m_Kd[velIndex];
									}

									motor->setVelocityTarget(desiredVelocity, kd);
									//todo: instead of clamping, combine the motor and limit
									//and combine handling of limit force and motor force.

									//clamp position
									//if (mb->getLink(link).m_jointLowerLimit <= mb->getLink(link).m_jointUpperLimit)
									//{
									//	btClamp(desiredPosition, mb->getLink(link).m_jointLowerLimit, mb->getLink(link).m_jointUpperLimit);
									//}
									motor->setPositionTarget(desiredPosition, kp);

									btScalar maxImp = 1000000.f * m_data->m_physicsDeltaTime;

									if ((clientCmd.m_updateFlags & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
										maxImp = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex] * m_data->m_physicsDeltaTime;

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
	else
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
						for (int link = 0; link < body->m_rigidBodyJoints.size(); link++)
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
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_MAX_FORCE) != 0)
									{
										torque = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[velIndex];
										hasDesiredTorque = true;
									}

									bool hasDesiredPosOrVel = false;
									btScalar qdotTarget = 0.f;
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[velIndex] & SIM_DESIRED_STATE_HAS_QDOT) != 0)
									{
										hasDesiredPosOrVel = true;
										qdotTarget = clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[velIndex];
									}
									btScalar qTarget = 0.f;
									if ((clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[posIndex] & SIM_DESIRED_STATE_HAS_Q) != 0)
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
									}
									else
									{
										con->calculateTransforms();

										if (linearLowerLimit.isZero() && linearUpperLimit.isZero())
										{
											//eRevoluteType;
											btVector3 limitRange = angularLowerLimit.absolute() + angularUpperLimit.absolute();
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
														con->getRigidBodyA().applyTorque(torque * axisA);
														con->getRigidBodyB().applyTorque(-torque * axisB);
													}
													break;
												}
												case CONTROL_MODE_VELOCITY:
												{
													if (hasDesiredPosOrVel)
													{
														con->enableMotor(3 + limitAxis, true);
														con->setTargetVelocity(3 + limitAxis, qdotTarget);
														con->setMaxMotorForce(3 + limitAxis, torque);
													}
													break;
												}
												case CONTROL_MODE_POSITION_VELOCITY_PD:
												{
													if (hasDesiredPosOrVel)
													{
														con->setServo(3 + limitAxis, true);
														con->setServoTarget(3 + limitAxis, -qTarget);
														//next one is the maximum velocity to reach target position.
														//the maximum velocity is limited by maxMotorForce
														con->setTargetVelocity(3 + limitAxis, 100);
														con->setMaxMotorForce(3 + limitAxis, torque);
														con->enableMotor(3 + limitAxis, true);
													}
													break;
												}
												default:
												{
												}
											};
										}
										else
										{
											//ePrismaticType; @todo
											btVector3 limitRange = linearLowerLimit.absolute() + linearUpperLimit.absolute();
											int limitAxis = limitRange.maxAxis();

											const btTransform& transA = con->getCalculatedTransformA();
											const btTransform& transB = con->getCalculatedTransformB();
											btVector3 axisA = transA.getBasis().getColumn(limitAxis);
											btVector3 axisB = transB.getBasis().getColumn(limitAxis);

											switch (clientCmd.m_sendDesiredStateCommandArgument.m_controlMode)
											{
												case CONTROL_MODE_TORQUE:
												{
													con->getRigidBodyA().applyForce(-torque * axisA, btVector3(0, 0, 0));
													con->getRigidBodyB().applyForce(torque * axisB, btVector3(0, 0, 0));
													break;
												}
												case CONTROL_MODE_VELOCITY:
												{
													con->enableMotor(limitAxis, true);
													con->setTargetVelocity(limitAxis, -qdotTarget);
													con->setMaxMotorForce(limitAxis, torque);
													break;
												}
												case CONTROL_MODE_POSITION_VELOCITY_PD:
												{
													con->setServo(limitAxis, true);
													con->setServoTarget(limitAxis, qTarget);
													//next one is the maximum velocity to reach target position.
													//the maximum velocity is limited by maxMotorForce
													con->setTargetVelocity(limitAxis, 100);
													con->setMaxMotorForce(limitAxis, torque);
													con->enableMotor(limitAxis, true);
													break;
												}
												default:
												{
												}
											};
										}
									}
								}  //fi
								///see addJointInfoFromConstraint
								velIndex++;  //info.m_uIndex
								posIndex++;  //info.m_qIndex
							}
						}
					}  //fi
					//break;
				}
			}
		}  //if (body && body->m_rigidBody)
	}

	serverStatusOut.m_type = CMD_DESIRED_STATE_RECEIVED_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestActualStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;

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

		if (mb->getNumLinks() >= MAX_DEGREE_OF_FREEDOM)
		{
			serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
			hasStatus = true;
			return hasStatus;
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
			totalDegreeOfFreedomQ += 7;  //pos + quaternion

			//base linear velocity (in world space, carthesian)
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = mb->getBaseVel()[0];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = mb->getBaseVel()[1];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = mb->getBaseVel()[2];

			//base angular velocity (in world space, carthesian)
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = mb->getBaseOmega()[0];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = mb->getBaseOmega()[1];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = mb->getBaseOmega()[2];
			totalDegreeOfFreedomU += 6;  //3 linear and 3 angular DOF
		}

		btAlignedObjectArray<btVector3> omega;
		btAlignedObjectArray<btVector3> linVel;

		bool computeForwardKinematics = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS) != 0);
		if (computeForwardKinematics)
		{
			B3_PROFILE("compForwardKinematics");
			btAlignedObjectArray<btQuaternion> world_to_local;
			btAlignedObjectArray<btVector3> local_origin;
			world_to_local.resize(mb->getNumLinks() + 1);
			local_origin.resize(mb->getNumLinks() + 1);
			mb->forwardKinematics(world_to_local, local_origin);
		}

		bool computeLinkVelocities = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_LINKVELOCITY) != 0);
		if (computeLinkVelocities)
		{
			omega.resize(mb->getNumLinks() + 1);
			linVel.resize(mb->getNumLinks() + 1);
			{
				B3_PROFILE("compTreeLinkVelocities");
				mb->compTreeLinkVelocities(&omega[0], &linVel[0]);
			}
		}
		for (int l = 0; l < mb->getNumLinks(); l++)
		{
			for (int d = 0; d < mb->getLink(l).m_posVarCount; d++)
			{
				serverCmd.m_sendActualStateArgs.m_actualStateQ[totalDegreeOfFreedomQ++] = mb->getJointPosMultiDof(l)[d];
			}
			for (int d = 0; d < mb->getLink(l).m_dofCount; d++)
			{
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[totalDegreeOfFreedomU++] = mb->getJointVelMultiDof(l)[d];
			}

			if (0 == mb->getLink(l).m_jointFeedback)
			{
				for (int d = 0; d < 6; d++)
				{
					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + d] = 0;
				}
			}
			else
			{
				btVector3 sensedForce = mb->getLink(l).m_jointFeedback->m_reactionForces.getLinear();
				btVector3 sensedTorque = mb->getLink(l).m_jointFeedback->m_reactionForces.getAngular();

				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 0] = sensedForce[0];
				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 1] = sensedForce[1];
				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 2] = sensedForce[2];

				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 3] = sensedTorque[0];
				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 4] = sensedTorque[1];
				serverCmd.m_sendActualStateArgs.m_jointReactionForces[l * 6 + 5] = sensedTorque[2];
			}

			serverCmd.m_sendActualStateArgs.m_jointMotorForce[l] = 0;

			if (supportsJointMotor(mb, l))
			{
				btMultiBodyJointMotor* motor = (btMultiBodyJointMotor*)body->m_multiBody->getLink(l).m_userPtr;

				if (motor && m_data->m_physicsDeltaTime > btScalar(0))
				{
					btScalar force = motor->getAppliedImpulse(0) / m_data->m_physicsDeltaTime;
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

			btVector3 linkCOMOrigin = mb->getLink(l).m_cachedWorldTransform.getOrigin();
			btQuaternion linkCOMRotation = mb->getLink(l).m_cachedWorldTransform.getRotation();

			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 0] = linkCOMOrigin.getX();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 1] = linkCOMOrigin.getY();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 2] = linkCOMOrigin.getZ();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 3] = linkCOMRotation.x();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 4] = linkCOMRotation.y();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 5] = linkCOMRotation.z();
			serverCmd.m_sendActualStateArgs.m_linkState[l * 7 + 6] = linkCOMRotation.w();

			btVector3 worldLinVel(0, 0, 0);
			btVector3 worldAngVel(0, 0, 0);

			if (computeLinkVelocities)
			{
				const btMatrix3x3& linkRotMat = mb->getLink(l).m_cachedWorldTransform.getBasis();
				worldLinVel = linkRotMat * linVel[l + 1];
				worldAngVel = linkRotMat * omega[l + 1];
			}

			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 0] = worldLinVel[0];
			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 1] = worldLinVel[1];
			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 2] = worldLinVel[2];
			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 3] = worldAngVel[0];
			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 4] = worldAngVel[1];
			serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l * 6 + 5] = worldAngVel[2];

			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 0] = linkLocalInertialOrigin.getX();
			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 1] = linkLocalInertialOrigin.getY();
			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 2] = linkLocalInertialOrigin.getZ();

			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 3] = linkLocalInertialRotation.x();
			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 4] = linkLocalInertialRotation.y();
			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 5] = linkLocalInertialRotation.z();
			serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l * 7 + 6] = linkLocalInertialRotation.w();
		}

		serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
		serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;

		hasStatus = true;
	}
	else
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
			totalDegreeOfFreedomQ += 7;  //pos + quaternion

			//base linear velocity (in world space, carthesian)
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = rb->getLinearVelocity()[0];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = rb->getLinearVelocity()[1];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = rb->getLinearVelocity()[2];

			//base angular velocity (in world space, carthesian)
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = rb->getAngularVelocity()[0];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = rb->getAngularVelocity()[1];
			serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = rb->getAngularVelocity()[2];
			totalDegreeOfFreedomU += 6;  //3 linear and 3 angular DOF

			serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
			serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;

			hasStatus = true;
		}
		else
		{
			//b3Warning("Request state but no multibody or rigid body available");
			SharedMemoryStatus& serverCmd = serverStatusOut;
			serverCmd.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
			hasStatus = true;
		}
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestContactpointInformationCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_CONTACT_POINT_INFORMATION");
	SharedMemoryStatus& serverCmd = serverStatusOut;
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
						pt.m_linearFrictionForce1 = srcPt.m_appliedImpulseLateral1 / m_data->m_physicsDeltaTime;
						pt.m_linearFrictionForce2 = srcPt.m_appliedImpulseLateral2 / m_data->m_physicsDeltaTime;
						for (int j = 0; j < 3; j++)
						{
							pt.m_linearFrictionDirection1[j] = srcPt.m_lateralFrictionDir1[j];
							pt.m_linearFrictionDirection2[j] = srcPt.m_lateralFrictionDir2[j];
						}
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

				bool hasLinkIndexAFilter = (0 != (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_A_FILTER));
				bool hasLinkIndexBFilter = (0 != (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_LINK_INDEX_B_FILTER));

				int linkIndexA = clientCmd.m_requestContactPointArguments.m_linkIndexAIndexFilter;
				int linkIndexB = clientCmd.m_requestContactPointArguments.m_linkIndexBIndexFilter;

				btAlignedObjectArray<btCollisionObject*> setA;
				btAlignedObjectArray<btCollisionObject*> setB;
				btAlignedObjectArray<int> setALinkIndex;
				btAlignedObjectArray<int> setBLinkIndex;

				btCollisionObject colObA;
				btCollisionObject colObB;

				int collisionShapeA = (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_A) ? clientCmd.m_requestContactPointArguments.m_collisionShapeA : -1;
				int collisionShapeB = (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_B) ? clientCmd.m_requestContactPointArguments.m_collisionShapeB : -1;

				if (collisionShapeA >= 0)
				{
					btVector3 posA(0, 0, 0);
					if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_POSITION_A)
					{
						posA.setValue(clientCmd.m_requestContactPointArguments.m_collisionShapePositionA[0],
									  clientCmd.m_requestContactPointArguments.m_collisionShapePositionA[1],
									  clientCmd.m_requestContactPointArguments.m_collisionShapePositionA[2]);
					}
					btQuaternion ornA(0, 0, 0, 1);
					if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_ORIENTATION_A)
					{
						ornA.setValue(clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationA[0],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationA[1],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationA[2],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationA[3]);
					}
					InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(collisionShapeA);

					if (handle && handle->m_collisionShape)
					{
						colObA.setCollisionShape(handle->m_collisionShape);
						btTransform tr;
						tr.setIdentity();
						tr.setOrigin(posA);
						tr.setRotation(ornA);
						colObA.setWorldTransform(tr);
						setA.push_back(&colObA);
						setALinkIndex.push_back(-2);
					}
					else
					{
						b3Warning("collisionShapeA provided is not valid.");
					}
				}
				if (collisionShapeB >= 0)
				{
					btVector3 posB(0, 0, 0);
					if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_POSITION_B)
					{
						posB.setValue(clientCmd.m_requestContactPointArguments.m_collisionShapePositionB[0],
									  clientCmd.m_requestContactPointArguments.m_collisionShapePositionB[1],
									  clientCmd.m_requestContactPointArguments.m_collisionShapePositionB[2]);
					}
					btQuaternion ornB(0, 0, 0, 1);
					if (clientCmd.m_updateFlags & CMD_REQUEST_CONTACT_POINT_HAS_COLLISION_SHAPE_ORIENTATION_B)
					{
						ornB.setValue(clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationB[0],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationB[1],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationB[2],
									  clientCmd.m_requestContactPointArguments.m_collisionShapeOrientationB[3]);
					}

					InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(collisionShapeB);
					if (handle && handle->m_collisionShape)
					{
						colObB.setCollisionShape(handle->m_collisionShape);
						btTransform tr;
						tr.setIdentity();
						tr.setOrigin(posB);
						tr.setRotation(ornB);
						colObB.setWorldTransform(tr);
						setB.push_back(&colObB);
						setBLinkIndex.push_back(-2);
					}
					else
					{
						b3Warning("collisionShapeB provided is not valid.");
					}
				}

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
				if (bodyUniqueIdB >= 0)
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
									if (!hasLinkIndexBFilter || (linkIndexB == i))
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
							: m_cachedContactPoints(pointCache)
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

						virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
						{
							if (cp.m_distance1 <= m_closestDistanceThreshold)
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
								pt.m_linearFrictionForce1 = srcPt.m_appliedImpulseLateral1 / m_deltaTime;
								pt.m_linearFrictionForce2 = srcPt.m_appliedImpulseLateral2 / m_deltaTime;
								for (int j = 0; j < 3; j++)
								{
									pt.m_linearFrictionDirection1[j] = srcPt.m_lateralFrictionDir1[j];
									pt.m_linearFrictionDirection2[j] = srcPt.m_lateralFrictionDir2[j];
								}
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
	int contactPointStorage = bufferSizeInBytes / totalBytesPerContact - 1;

	b3ContactPointData* contactData = (b3ContactPointData*)bufferServerToClient;

	int startContactPointIndex = clientCmd.m_requestContactPointArguments.m_startingContactPointIndex;
	int numContactPointBatch = btMin(numContactPoints, contactPointStorage);

	int endContactPointIndex = startContactPointIndex + numContactPointBatch;

	for (int i = startContactPointIndex; i < endContactPointIndex; i++)
	{
		const b3ContactPointData& srcPt = m_data->m_cachedContactPoints[i];
		b3ContactPointData& destPt = contactData[serverCmd.m_sendContactPointArgs.m_numContactPointsCopied];
		destPt = srcPt;
		serverCmd.m_sendContactPointArgs.m_numContactPointsCopied++;
	}

	serverCmd.m_sendContactPointArgs.m_startingContactPointIndex = clientCmd.m_requestContactPointArguments.m_startingContactPointIndex;
	serverCmd.m_sendContactPointArgs.m_numRemainingContactPoints = numContactPoints - clientCmd.m_requestContactPointArguments.m_startingContactPointIndex - serverCmd.m_sendContactPointArgs.m_numContactPointsCopied;
	serverCmd.m_numDataStreamBytes = totalBytesPerContact * serverCmd.m_sendContactPointArgs.m_numContactPointsCopied;
	serverCmd.m_type = CMD_CONTACT_POINT_INFORMATION_COMPLETED;  //CMD_CONTACT_POINT_INFORMATION_FAILED,

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
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
		strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName, bodyHandle->m_bodyName.c_str());
	}

	serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadSDFCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_LOAD_SDF");

	const SdfArgs& sdfArgs = clientCmd.m_sdfArguments;
	if (m_data->m_verboseOutput)
	{
		b3Printf("Processed CMD_LOAD_SDF:%s", sdfArgs.m_sdfFileName);
	}
	bool useMultiBody = (clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (sdfArgs.m_useMultiBody != 0) : true;

	int flags = CUF_USE_SDF;  //CUF_USE_URDF_INERTIA
	btScalar globalScaling = 1.f;
	if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
	{
		globalScaling = sdfArgs.m_globalScaling;
	}
	bool completedOk = loadSdf(sdfArgs.m_sdfFileName, bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, globalScaling);
	if (completedOk)
	{
		m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

		//serverStatusOut.m_type = CMD_SDF_LOADING_FAILED;
		serverStatusOut.m_sdfLoadedArgs.m_numBodies = m_data->m_sdfRecentLoadedBodies.size();
		serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
		int maxBodies = btMin(MAX_SDF_BODIES, serverStatusOut.m_sdfLoadedArgs.m_numBodies);
		for (int i = 0; i < maxBodies; i++)
		{
			serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = m_data->m_sdfRecentLoadedBodies[i];
		}

		serverStatusOut.m_type = CMD_SDF_LOADING_COMPLETED;
	}
	else
	{
		serverStatusOut.m_type = CMD_SDF_LOADING_FAILED;
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateMultiBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	serverStatusOut.m_type = CMD_CREATE_MULTI_BODY_FAILED;
	if (clientCmd.m_createMultiBodyArgs.m_baseLinkIndex >= 0)
	{
		m_data->m_sdfRecentLoadedBodies.clear();

		int flags = 0;

		if (clientCmd.m_updateFlags & MULT_BODY_HAS_FLAGS)
		{
			flags = clientCmd.m_createMultiBodyArgs.m_flags;
		}

		ProgrammaticUrdfInterface u2b(clientCmd.m_createMultiBodyArgs, m_data, flags);

		bool useMultiBody = true;
		if (clientCmd.m_updateFlags & MULT_BODY_USE_MAXIMAL_COORDINATES)
		{
			useMultiBody = false;
		}



		bool ok = processImportedObjects("memory", bufferServerToClient, bufferSizeInBytes, useMultiBody, flags, u2b);

		if (ok)
		{
			int bodyUniqueId = -1;

			if (m_data->m_sdfRecentLoadedBodies.size() == 1)
			{
				bodyUniqueId = m_data->m_sdfRecentLoadedBodies[0];
			}
			m_data->m_sdfRecentLoadedBodies.clear();
			if (bodyUniqueId >= 0)
			{
				m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
				serverStatusOut.m_type = CMD_CREATE_MULTI_BODY_COMPLETED;

				int streamSizeInBytes = createBodyInfoStream(bodyUniqueId, bufferServerToClient, bufferSizeInBytes);
				serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;

				serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
				InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
				strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName, body->m_bodyName.c_str());
			}
		}

		//ConvertURDF2Bullet(u2b,creation, rootTrans,m_data->m_dynamicsWorld,useMultiBody,u2b.getPathPrefix(),flags);
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadURDFCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_URDF_LOADING_FAILED;

	BT_PROFILE("CMD_LOAD_URDF");
	const UrdfArgs& urdfArgs = clientCmd.m_urdfArguments;
	if (m_data->m_verboseOutput)
	{
		b3Printf("Processed CMD_LOAD_URDF:%s", urdfArgs.m_urdfFileName);
	}
	btAssert((clientCmd.m_updateFlags & URDF_ARGS_FILE_NAME) != 0);
	btAssert(urdfArgs.m_urdfFileName);
	btVector3 initialPos(0, 0, 0);
	btQuaternion initialOrn(0, 0, 0, 1);
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
	bool useMultiBody = (clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (urdfArgs.m_useMultiBody != 0) : true;
	bool useFixedBase = (clientCmd.m_updateFlags & URDF_ARGS_USE_FIXED_BASE) ? (urdfArgs.m_useFixedBase != 0) : false;
	int bodyUniqueId;
	btScalar globalScaling = 1.f;
	if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
	{
		globalScaling = urdfArgs.m_globalScaling;
	}
	//load the actual URDF and send a report: completed or failed
	bool completedOk = loadUrdf(urdfArgs.m_urdfFileName,
								initialPos, initialOrn,
								useMultiBody, useFixedBase, &bodyUniqueId, bufferServerToClient, bufferSizeInBytes, urdfFlags, globalScaling);

	if (completedOk && bodyUniqueId >= 0)
	{
		m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

		serverStatusOut.m_type = CMD_URDF_LOADING_COMPLETED;

		int streamSizeInBytes = createBodyInfoStream(bodyUniqueId, bufferServerToClient, bufferSizeInBytes);
		serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;

#ifdef ENABLE_LINK_MAPPER
		if (m_data->m_urdfLinkNameMapper.size())
		{
			serverStatusOut.m_numDataStreamBytes = m_data->m_urdfLinkNameMapper.at(m_data->m_urdfLinkNameMapper.size() - 1)->m_memSerializer->getCurrentBufferSize();
		}
#endif
		serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
		InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);
		strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName, body->m_bodyName.c_str());
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadSoftBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	serverStatusOut.m_type = CMD_LOAD_SOFT_BODY_FAILED;
	bool hasStatus = true;
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	double scale = 0.1;
	double mass = 0.1;
	double collisionMargin = 0.02;
	const LoadSoftBodyArgs& loadSoftBodyArgs = clientCmd.m_loadSoftBodyArguments;
	if (m_data->m_verboseOutput)
	{
		b3Printf("Processed CMD_LOAD_SOFT_BODY:%s", loadSoftBodyArgs.m_fileName);
	}
	btAssert((clientCmd.m_updateFlags & LOAD_SOFT_BODY_FILE_NAME) != 0);
	btAssert(loadSoftBodyArgs.m_fileName);

	if (clientCmd.m_updateFlags & LOAD_SOFT_BODY_UPDATE_SCALE)
	{
		scale = clientCmd.m_loadSoftBodyArguments.m_scale;
	}
	if (clientCmd.m_updateFlags & LOAD_SOFT_BODY_UPDATE_MASS)
	{
		mass = clientCmd.m_loadSoftBodyArguments.m_mass;
	}
	if (clientCmd.m_updateFlags & LOAD_SOFT_BODY_UPDATE_COLLISION_MARGIN)
	{
		collisionMargin = clientCmd.m_loadSoftBodyArguments.m_collisionMargin;
	}

	{
		CommonFileIOInterface* fileIO(m_data->m_pluginManager.getFileIOInterface());
		char relativeFileName[1024];
		char pathPrefix[1024];
		pathPrefix[0] = 0;
		if (fileIO->findResourcePath(loadSoftBodyArgs.m_fileName, relativeFileName, 1024))
		{
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}
		const std::string& error_message_prefix = "";
		std::string out_found_filename;
		int out_type;

		bool foundFile = UrdfFindMeshFile(fileIO,pathPrefix, relativeFileName, error_message_prefix, &out_found_filename, &out_type);
		std::vector<tinyobj::shape_t> shapes;
		std::string err = tinyobj::LoadObj(shapes, out_found_filename.c_str(),"",fileIO);
		if (shapes.size() > 0)
		{
			const tinyobj::shape_t& shape = shapes[0];
			btAlignedObjectArray<btScalar> vertices;
			btAlignedObjectArray<int> indices;
			for (int i = 0; i < shape.mesh.positions.size(); i++)
			{
				vertices.push_back(shape.mesh.positions[i]);
			}
			for (int i = 0; i < shape.mesh.indices.size(); i++)
			{
				indices.push_back(shape.mesh.indices[i]);
			}
			int numTris = indices.size() / 3;
			if (numTris > 0)
			{
				btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(m_data->m_dynamicsWorld->getWorldInfo(), &vertices[0], &indices[0], numTris);
				btSoftBody::Material* pm = psb->appendMaterial();
				pm->m_kLST = 0.5;
				pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
				psb->generateBendingConstraints(2, pm);
				psb->m_cfg.piterations = 20;
				psb->m_cfg.kDF = 0.5;
				psb->randomizeConstraints();
				psb->rotate(btQuaternion(0.70711, 0, 0, 0.70711));
				psb->translate(btVector3(-0.05, 0, 1.0));
				psb->scale(btVector3(scale, scale, scale));

				psb->setTotalMass(mass, true);
				psb->getCollisionShape()->setMargin(collisionMargin);
				psb->getCollisionShape()->setUserPointer(psb);
				m_data->m_dynamicsWorld->addSoftBody(psb);
				m_data->m_guiHelper->createCollisionShapeGraphicsObject(psb->getCollisionShape());
				m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
				int bodyUniqueId = m_data->m_bodyHandles.allocHandle();
				InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
				bodyHandle->m_softBody = psb;
				serverStatusOut.m_loadSoftBodyResultArguments.m_objectUniqueId = bodyUniqueId;
				serverStatusOut.m_type = CMD_LOAD_SOFT_BODY_COMPLETED;

				b3Notification notification;
				notification.m_notificationType = BODY_ADDED;
				notification.m_bodyArgs.m_bodyUniqueId = bodyUniqueId;
				m_data->m_pluginManager.addNotification(notification);
			}
		}
	}

#endif
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateSensorCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
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
		for (int i = 0; i < clientCmd.m_createSensorArguments.m_numJointSensorChanges; i++)
		{
			int jointIndex = clientCmd.m_createSensorArguments.m_jointIndex[i];
			if (clientCmd.m_createSensorArguments.m_enableJointForceSensor[i])
			{
				if (mb->getLink(jointIndex).m_jointFeedback)
				{
					b3Warning("CMD_CREATE_SENSOR: sensor for joint [%d] already enabled", jointIndex);
				}
				else
				{
					btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
					fb->m_reactionForces.setZero();
					mb->getLink(jointIndex).m_jointFeedback = fb;
					m_data->m_multiBodyJointFeedbacks.push_back(fb);
				};
			}
			else
			{
				if (mb->getLink(jointIndex).m_jointFeedback)
				{
					m_data->m_multiBodyJointFeedbacks.remove(mb->getLink(jointIndex).m_jointFeedback);
					delete mb->getLink(jointIndex).m_jointFeedback;
					mb->getLink(jointIndex).m_jointFeedback = 0;
				}
				else
				{
					b3Warning("CMD_CREATE_SENSOR: cannot perform sensor removal request, no sensor on joint [%d]", jointIndex);
				};
			}
		}
	}
	else
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processProfileTimingCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	B3_PROFILE("custom");  //clientCmd.m_profile.m_name);
	{
		B3_PROFILE("event");  //clientCmd.m_profile.m_name);
		char** eventNamePtr = m_data->m_profileEvents[clientCmd.m_profile.m_name];
		char* eventName = 0;
		if (eventNamePtr)
		{
			B3_PROFILE("reuse");
			eventName = *eventNamePtr;
		}
		else
		{
			B3_PROFILE("alloc");
			int len = strlen(clientCmd.m_profile.m_name);
			eventName = new char[len + 1];
			strcpy(eventName, clientCmd.m_profile.m_name);
			eventName[len] = 0;
			m_data->m_profileEvents.insert(eventName, eventName);
		}

		{
			{
				B3_PROFILE("with");  //clientCmd.m_profile.m_name);
				{
					B3_PROFILE("some");  //clientCmd.m_profile.m_name);
					{
						B3_PROFILE("deep");  //clientCmd.m_profile.m_name);
						{
							B3_PROFILE("level");  //clientCmd.m_profile.m_name);
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

	serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	hasStatus = true;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestCollisionInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverStatusOut.m_type = CMD_REQUEST_COLLISION_INFO_FAILED;
	hasStatus = true;
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

			btVector3 aabbMin, aabbMax;
			body->m_multiBody->getBaseCollider()->getCollisionShape()->getAabb(tr, aabbMin, aabbMax);
			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = aabbMin[0];
			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = aabbMin[1];
			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = aabbMin[2];

			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = aabbMax[0];
			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = aabbMax[1];
			serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = aabbMax[2];
		}
		for (int l = 0; l < mb->getNumLinks(); l++)
		{
			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 0] = 0;
			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 1] = 0;
			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 2] = 0;

			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 0] = -1;
			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 1] = -1;
			serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 2] = -1;

			if (body->m_multiBody->getLink(l).m_collider)
			{
				btVector3 aabbMin, aabbMax;
				body->m_multiBody->getLinkCollider(l)->getCollisionShape()->getAabb(mb->getLink(l).m_cachedWorldTransform, aabbMin, aabbMax);

				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 0] = aabbMin[0];
				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 1] = aabbMin[1];
				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMin[3 * l + 2] = aabbMin[2];
				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 0] = aabbMax[0];
				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 1] = aabbMax[1];
				serverCmd.m_sendCollisionInfoArgs.m_linkWorldAABBsMax[3 * l + 2] = aabbMax[2];
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

				btVector3 aabbMin, aabbMax;
				rb->getCollisionShape()->getAabb(tr, aabbMin, aabbMax);
				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[0] = aabbMin[0];
				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[1] = aabbMin[1];
				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMin[2] = aabbMin[2];

				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[0] = aabbMax[0];
				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[1] = aabbMax[1];
				serverCmd.m_sendCollisionInfoArgs.m_rootWorldAABBMax[2] = aabbMax[2];
			}
		}
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processForwardDynamicsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_STEP_FORWARD_SIMULATION");

	if (m_data->m_verboseOutput)
	{
		b3Printf("Step simulation request");
		b3Printf("CMD_STEP_FORWARD_SIMULATION clientCmd = %d\n", clientCmd.m_sequenceNumber);
	}
	///todo(erwincoumans) move this damping inside Bullet
	for (int i = 0; i < m_data->m_dynamicsWorld->getNumMultibodies(); i++)
	{
		btMultiBody* mb = m_data->m_dynamicsWorld->getMultiBody(i);
		for (int l = 0; l < mb->getNumLinks(); l++)
		{
			for (int d = 0; d < mb->getLink(l).m_dofCount; d++)
			{
				double damping_coefficient = mb->getLink(l).m_jointDamping;
				double damping = -damping_coefficient * mb->getJointVelMultiDof(l)[d];
				mb->addJointTorqueMultiDof(l, d, damping);
			}
		}
	}

	btScalar deltaTimeScaled = m_data->m_physicsDeltaTime * simTimeScalingFactor;

	int numSteps = 0;
	if (m_data->m_numSimulationSubSteps > 0)
	{
		numSteps = m_data->m_dynamicsWorld->stepSimulation(deltaTimeScaled, m_data->m_numSimulationSubSteps, m_data->m_physicsDeltaTime / m_data->m_numSimulationSubSteps);
	}
	else
	{
		numSteps = m_data->m_dynamicsWorld->stepSimulation(deltaTimeScaled, 0);
	}

	if (numSteps > 0)
	{
		addTransformChangedNotifications();
	}

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestInternalDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_INTERNAL_DATA");

	//todo: also check version etc?

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_REQUEST_INTERNAL_DATA_FAILED;

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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processChangeDynamicsInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_CHANGE_DYNAMICS_INFO");

	int bodyUniqueId = clientCmd.m_changeDynamicsInfoArgs.m_bodyUniqueId;
	int linkIndex = clientCmd.m_changeDynamicsInfoArgs.m_linkIndex;
	double mass = clientCmd.m_changeDynamicsInfoArgs.m_mass;
	double lateralFriction = clientCmd.m_changeDynamicsInfoArgs.m_lateralFriction;
	double spinningFriction = clientCmd.m_changeDynamicsInfoArgs.m_spinningFriction;
	double rollingFriction = clientCmd.m_changeDynamicsInfoArgs.m_rollingFriction;
	double restitution = clientCmd.m_changeDynamicsInfoArgs.m_restitution;
	btVector3 newLocalInertiaDiagonal(clientCmd.m_changeDynamicsInfoArgs.m_localInertiaDiagonal[0],
									  clientCmd.m_changeDynamicsInfoArgs.m_localInertiaDiagonal[1],
									  clientCmd.m_changeDynamicsInfoArgs.m_localInertiaDiagonal[2]);

	btAssert(bodyUniqueId >= 0);

	InternalBodyData* body = m_data->m_bodyHandles.getHandle(bodyUniqueId);

	if (body && body->m_multiBody)
	{
		btMultiBody* mb = body->m_multiBody;

		if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ACTIVATION_STATE)
		{
			if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateWakeUp)
			{
				mb->wakeUp();
			}
			if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateSleep)
			{
				mb->goToSleep();
			}
			if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateEnableSleeping)
			{
				mb->setCanSleep(true);
			}
			if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateDisableSleeping)
			{
				mb->setCanSleep(false);
			}
		}

		if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LINEAR_DAMPING)
		{
			mb->setLinearDamping(clientCmd.m_changeDynamicsInfoArgs.m_linearDamping);
		}
		if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ANGULAR_DAMPING)
		{
			mb->setAngularDamping(clientCmd.m_changeDynamicsInfoArgs.m_angularDamping);
		}

		if (linkIndex == -1)
		{
			if (mb->getBaseCollider())
			{
				if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_RESTITUTION)
				{
					mb->getBaseCollider()->setRestitution(restitution);
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
					}
					else
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
					mb->getBaseCollider()->getCollisionShape()->calculateLocalInertia(mass, localInertia);
					mb->setBaseInertia(localInertia);
				}
			}
			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LOCAL_INERTIA_DIAGONAL)
			{
				mb->setBaseInertia(newLocalInertiaDiagonal);
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
						}
						else
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
						mb->getLinkCollider(linkIndex)->getCollisionShape()->calculateLocalInertia(mass, localInertia);
						mb->getLink(linkIndex).m_inertiaLocal = localInertia;
					}
				}
				if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LOCAL_INERTIA_DIAGONAL)
				{
					mb->getLink(linkIndex).m_inertiaLocal = newLocalInertiaDiagonal;
				}
			}
		}
	}
	else
	{
		if (body && body->m_rigidBody)
		{
			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_ACTIVATION_STATE)
			{
				if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateEnableSleeping)
				{
					body->m_rigidBody->forceActivationState(ACTIVE_TAG);
				}
				if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateDisableSleeping)
				{
					body->m_rigidBody->forceActivationState(DISABLE_DEACTIVATION);
				}
				if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateWakeUp)
				{
					body->m_rigidBody->forceActivationState(ACTIVE_TAG);
				}
				if (clientCmd.m_changeDynamicsInfoArgs.m_activationState & eActivationStateSleep)
				{
					body->m_rigidBody->forceActivationState(ISLAND_SLEEPING);
				}
			}

			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LINEAR_DAMPING)
			{
				btScalar angDamping = body->m_rigidBody->getAngularDamping();
				body->m_rigidBody->setDamping(clientCmd.m_changeDynamicsInfoArgs.m_linearDamping, angDamping);
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
				}
				else
				{
					body->m_rigidBody->setCollisionFlags(body->m_rigidBody->getCollisionFlags() & ~btCollisionObject::CF_HAS_FRICTION_ANCHOR);
				}
			}

			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_MASS)
			{
				btVector3 localInertia;
				if (body->m_rigidBody->getCollisionShape())
				{
					body->m_rigidBody->getCollisionShape()->calculateLocalInertia(mass, localInertia);
				}
				body->m_rigidBody->setMassProps(mass, localInertia);
			}
			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_LOCAL_INERTIA_DIAGONAL)
			{
				btScalar orgMass = body->m_rigidBody->getInvMass();
				if (orgMass > 0)
				{
					body->m_rigidBody->setMassProps(mass, newLocalInertiaDiagonal);
				}
			}

			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_CONTACT_PROCESSING_THRESHOLD)
			{
				body->m_rigidBody->setContactProcessingThreshold(clientCmd.m_changeDynamicsInfoArgs.m_contactProcessingThreshold);
			}

			if (clientCmd.m_updateFlags & CHANGE_DYNAMICS_INFO_SET_CCD_SWEPT_SPHERE_RADIUS)
			{
				body->m_rigidBody->setCcdSweptSphereRadius(clientCmd.m_changeDynamicsInfoArgs.m_ccdSweptSphereRadius);
				//for a given sphere radius, use a motion threshold of half the radius, before the ccd algorithm is enabled
				body->m_rigidBody->setCcdMotionThreshold(clientCmd.m_changeDynamicsInfoArgs.m_ccdSweptSphereRadius / 2.);
			}
		}
	}

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;

	b3Notification notification;
	notification.m_notificationType = LINK_DYNAMICS_CHANGED;
	notification.m_linkArgs.m_bodyUniqueId = bodyUniqueId;
	notification.m_linkArgs.m_linkIndex = linkIndex;
	m_data->m_pluginManager.addNotification(notification);

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSetAdditionalSearchPathCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_SET_ADDITIONAL_SEARCH_PATH");
	b3ResourcePath::setAdditionalSearchPath(clientCmd.m_searchPathArgs.m_path);
	serverStatusOut.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processGetDynamicsInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_GET_DYNAMICS_INFO_FAILED;

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
			if (mb->getBaseCollider())
			{
				serverCmd.m_dynamicsInfo.m_activationState = mb->getBaseCollider()->getActivationState();
				serverCmd.m_dynamicsInfo.m_contactProcessingThreshold = mb->getBaseCollider()->getContactProcessingThreshold();
				serverCmd.m_dynamicsInfo.m_ccdSweptSphereRadius = mb->getBaseCollider()->getCcdSweptSphereRadius();
				serverCmd.m_dynamicsInfo.m_frictionAnchor = mb->getBaseCollider()->getCollisionFlags() & btCollisionObject::CF_HAS_FRICTION_ANCHOR;
			}
			else
			{
				serverCmd.m_dynamicsInfo.m_activationState = 0;
				serverCmd.m_dynamicsInfo.m_contactProcessingThreshold = 0;
				serverCmd.m_dynamicsInfo.m_ccdSweptSphereRadius = 0;
				serverCmd.m_dynamicsInfo.m_frictionAnchor = 0;
			}
			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[0] = mb->getBaseInertia()[0];
			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[1] = mb->getBaseInertia()[1];
			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[2] = mb->getBaseInertia()[2];
			serverCmd.m_dynamicsInfo.m_lateralFrictionCoeff = mb->getBaseCollider()->getFriction();

			serverCmd.m_dynamicsInfo.m_localInertialFrame[0] = body->m_rootLocalInertialFrame.getOrigin()[0];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[1] = body->m_rootLocalInertialFrame.getOrigin()[1];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[2] = body->m_rootLocalInertialFrame.getOrigin()[2];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[3] = body->m_rootLocalInertialFrame.getRotation()[0];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[4] = body->m_rootLocalInertialFrame.getRotation()[1];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[5] = body->m_rootLocalInertialFrame.getRotation()[2];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[6] = body->m_rootLocalInertialFrame.getRotation()[3];

			serverCmd.m_dynamicsInfo.m_angularDamping = body->m_multiBody->getAngularDamping();
			serverCmd.m_dynamicsInfo.m_linearDamping = body->m_multiBody->getLinearDamping();

			serverCmd.m_dynamicsInfo.m_restitution = mb->getBaseCollider()->getRestitution();
			serverCmd.m_dynamicsInfo.m_rollingFrictionCoeff = mb->getBaseCollider()->getRollingFriction();
			serverCmd.m_dynamicsInfo.m_spinningFrictionCoeff = mb->getBaseCollider()->getSpinningFriction();

			if (mb->getBaseCollider()->getCollisionFlags() & btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING)
			{
				serverCmd.m_dynamicsInfo.m_contactStiffness = mb->getBaseCollider()->getContactStiffness();
				serverCmd.m_dynamicsInfo.m_contactDamping = mb->getBaseCollider()->getContactDamping();
			}
			else
			{
				serverCmd.m_dynamicsInfo.m_contactStiffness = -1;
				serverCmd.m_dynamicsInfo.m_contactDamping = -1;
			}
		}
		else
		{
			serverCmd.m_dynamicsInfo.m_mass = mb->getLinkMass(linkIndex);

			if (mb->getLinkCollider(linkIndex))
			{
				serverCmd.m_dynamicsInfo.m_activationState = mb->getLinkCollider(linkIndex)->getActivationState();
				serverCmd.m_dynamicsInfo.m_contactProcessingThreshold = mb->getLinkCollider(linkIndex)->getContactProcessingThreshold();
				serverCmd.m_dynamicsInfo.m_ccdSweptSphereRadius = mb->getLinkCollider(linkIndex)->getCcdSweptSphereRadius();
				serverCmd.m_dynamicsInfo.m_frictionAnchor = mb->getLinkCollider(linkIndex)->getCollisionFlags() & btCollisionObject::CF_HAS_FRICTION_ANCHOR;
			}
			else
			{
				serverCmd.m_dynamicsInfo.m_activationState = 0;
				serverCmd.m_dynamicsInfo.m_contactProcessingThreshold = 0;
				serverCmd.m_dynamicsInfo.m_ccdSweptSphereRadius = 0;
				serverCmd.m_dynamicsInfo.m_frictionAnchor = 0;
			}

			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[0] = mb->getLinkInertia(linkIndex)[0];
			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[1] = mb->getLinkInertia(linkIndex)[1];
			serverCmd.m_dynamicsInfo.m_localInertialDiagonal[2] = mb->getLinkInertia(linkIndex)[2];

			serverCmd.m_dynamicsInfo.m_localInertialFrame[0] = body->m_linkLocalInertialFrames[linkIndex].getOrigin()[0];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[1] = body->m_linkLocalInertialFrames[linkIndex].getOrigin()[1];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[2] = body->m_linkLocalInertialFrames[linkIndex].getOrigin()[2];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[3] = body->m_linkLocalInertialFrames[linkIndex].getRotation()[0];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[4] = body->m_linkLocalInertialFrames[linkIndex].getRotation()[1];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[5] = body->m_linkLocalInertialFrames[linkIndex].getRotation()[2];
			serverCmd.m_dynamicsInfo.m_localInertialFrame[6] = body->m_linkLocalInertialFrames[linkIndex].getRotation()[3];

			serverCmd.m_dynamicsInfo.m_angularDamping = body->m_multiBody->getAngularDamping();
			serverCmd.m_dynamicsInfo.m_linearDamping = body->m_multiBody->getLinearDamping();

			if (mb->getLinkCollider(linkIndex))
			{
				serverCmd.m_dynamicsInfo.m_lateralFrictionCoeff = mb->getLinkCollider(linkIndex)->getFriction();
				serverCmd.m_dynamicsInfo.m_restitution = mb->getLinkCollider(linkIndex)->getRestitution();
				serverCmd.m_dynamicsInfo.m_rollingFrictionCoeff = mb->getLinkCollider(linkIndex)->getRollingFriction();
				serverCmd.m_dynamicsInfo.m_spinningFrictionCoeff = mb->getLinkCollider(linkIndex)->getSpinningFriction();

				if (mb->getLinkCollider(linkIndex)->getCollisionFlags() & btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING)
				{
					serverCmd.m_dynamicsInfo.m_contactStiffness = mb->getLinkCollider(linkIndex)->getContactStiffness();
					serverCmd.m_dynamicsInfo.m_contactDamping = mb->getLinkCollider(linkIndex)->getContactDamping();
				}
				else
				{
					serverCmd.m_dynamicsInfo.m_contactStiffness = -1;
					serverCmd.m_dynamicsInfo.m_contactDamping = -1;
				}
			}
			else
			{
				b3Warning("The dynamic info requested is not available");
				serverCmd.m_type = CMD_GET_DYNAMICS_INFO_FAILED;
			}
		}
		hasStatus = true;
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestPhysicsSimulationParametersCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED;

	serverCmd.m_simulationParameterResultArgs.m_allowedCcdPenetration = m_data->m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration;
	serverCmd.m_simulationParameterResultArgs.m_collisionFilterMode = m_data->m_broadphaseCollisionFilterCallback->m_filterMode;
	serverCmd.m_simulationParameterResultArgs.m_deltaTime = m_data->m_physicsDeltaTime;
	serverCmd.m_simulationParameterResultArgs.m_contactBreakingThreshold = gContactBreakingThreshold;
	serverCmd.m_simulationParameterResultArgs.m_contactSlop = m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop;
	serverCmd.m_simulationParameterResultArgs.m_enableSAT = m_data->m_dynamicsWorld->getDispatchInfo().m_enableSatConvex;

	serverCmd.m_simulationParameterResultArgs.m_defaultGlobalCFM = m_data->m_dynamicsWorld->getSolverInfo().m_globalCfm;
	serverCmd.m_simulationParameterResultArgs.m_defaultContactERP = m_data->m_dynamicsWorld->getSolverInfo().m_erp2;
	serverCmd.m_simulationParameterResultArgs.m_defaultNonContactERP = m_data->m_dynamicsWorld->getSolverInfo().m_erp;
	serverCmd.m_simulationParameterResultArgs.m_deltaTime = m_data->m_physicsDeltaTime;
	serverCmd.m_simulationParameterResultArgs.m_deterministicOverlappingPairs = m_data->m_dynamicsWorld->getDispatchInfo().m_deterministicOverlappingPairs;
	serverCmd.m_simulationParameterResultArgs.m_enableConeFriction = (m_data->m_dynamicsWorld->getSolverInfo().m_solverMode & SOLVER_DISABLE_IMPLICIT_CONE_FRICTION) ? 0 : 1;
	serverCmd.m_simulationParameterResultArgs.m_enableFileCaching = b3IsFileCachingEnabled();
	serverCmd.m_simulationParameterResultArgs.m_frictionCFM = m_data->m_dynamicsWorld->getSolverInfo().m_frictionCFM;
	serverCmd.m_simulationParameterResultArgs.m_frictionERP = m_data->m_dynamicsWorld->getSolverInfo().m_frictionERP;
	btVector3 grav = m_data->m_dynamicsWorld->getGravity();
	serverCmd.m_simulationParameterResultArgs.m_gravityAcceleration[0] = grav[0];
	serverCmd.m_simulationParameterResultArgs.m_gravityAcceleration[1] = grav[1];
	serverCmd.m_simulationParameterResultArgs.m_gravityAcceleration[2] = grav[2];
	serverCmd.m_simulationParameterResultArgs.m_internalSimFlags = gInternalSimFlags;
	serverCmd.m_simulationParameterResultArgs.m_jointFeedbackMode = 0;
	if (m_data->m_dynamicsWorld->getSolverInfo().m_jointFeedbackInWorldSpace)
	{
		serverCmd.m_simulationParameterResultArgs.m_jointFeedbackMode |= JOINT_FEEDBACK_IN_WORLD_SPACE;
	}
	if (m_data->m_dynamicsWorld->getSolverInfo().m_jointFeedbackInJointFrame)
	{
		serverCmd.m_simulationParameterResultArgs.m_jointFeedbackMode |= JOINT_FEEDBACK_IN_JOINT_FRAME;
	}

	serverCmd.m_simulationParameterResultArgs.m_numSimulationSubSteps = m_data->m_numSimulationSubSteps;
	serverCmd.m_simulationParameterResultArgs.m_numSolverIterations = m_data->m_dynamicsWorld->getSolverInfo().m_numIterations;
	serverCmd.m_simulationParameterResultArgs.m_restitutionVelocityThreshold = m_data->m_dynamicsWorld->getSolverInfo().m_restitutionVelocityThreshold;

	serverCmd.m_simulationParameterResultArgs.m_solverResidualThreshold = m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold;
	serverCmd.m_simulationParameterResultArgs.m_splitImpulsePenetrationThreshold = m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold;
	serverCmd.m_simulationParameterResultArgs.m_useRealTimeSimulation = m_data->m_useRealTimeSimulation;
	serverCmd.m_simulationParameterResultArgs.m_useSplitImpulse = m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulse;

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSendPhysicsParametersCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_SEND_PHYSICS_SIMULATION_PARAMETERS");

	if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_CONE_FRICTION)
	{
		if (clientCmd.m_physSimParamArgs.m_enableConeFriction)
		{
			m_data->m_dynamicsWorld->getSolverInfo().m_solverMode &= ~SOLVER_DISABLE_IMPLICIT_CONE_FRICTION;
		}
		else
		{
			m_data->m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_DISABLE_IMPLICIT_CONE_FRICTION;
		}
	}
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DETERMINISTIC_OVERLAPPING_PAIRS)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_deterministicOverlappingPairs = (clientCmd.m_physSimParamArgs.m_deterministicOverlappingPairs != 0);
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CCD_ALLOWED_PENETRATION)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = clientCmd.m_physSimParamArgs.m_allowedCcdPenetration;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_JOINT_FEEDBACK_MODE)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_jointFeedbackInWorldSpace = (clientCmd.m_physSimParamArgs.m_jointFeedbackMode & JOINT_FEEDBACK_IN_WORLD_SPACE) != 0;
		m_data->m_dynamicsWorld->getSolverInfo().m_jointFeedbackInJointFrame = (clientCmd.m_physSimParamArgs.m_jointFeedbackMode & JOINT_FEEDBACK_IN_JOINT_FRAME) != 0;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DELTA_TIME)
	{
		m_data->m_physicsDeltaTime = clientCmd.m_physSimParamArgs.m_deltaTime;
	}
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_REAL_TIME_SIMULATION)
	{
		m_data->m_useRealTimeSimulation = (clientCmd.m_physSimParamArgs.m_useRealTimeSimulation != 0);
	}

	//see
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS)
	{
		//these flags are for internal/temporary/easter-egg/experimental demo purposes, use at own risk
		gInternalSimFlags = clientCmd.m_physSimParamArgs.m_internalSimFlags;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_GRAVITY)
	{
		btVector3 grav(clientCmd.m_physSimParamArgs.m_gravityAcceleration[0],
					   clientCmd.m_physSimParamArgs.m_gravityAcceleration[1],
					   clientCmd.m_physSimParamArgs.m_gravityAcceleration[2]);
		this->m_data->m_dynamicsWorld->setGravity(grav);
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
		m_data->m_dynamicsWorld->getWorldInfo().m_gravity = grav;

#endif
		if (m_data->m_verboseOutput)
		{
			b3Printf("Updated Gravity: %f,%f,%f", grav[0], grav[1], grav[2]);
		}
	}
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = clientCmd.m_physSimParamArgs.m_numSolverIterations;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_SOLVER_RESIDULAL_THRESHOLD)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = clientCmd.m_physSimParamArgs.m_solverResidualThreshold;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD)
	{
		gContactBreakingThreshold = clientCmd.m_physSimParamArgs.m_contactBreakingThreshold;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CONTACT_SLOP)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = clientCmd.m_physSimParamArgs.m_contactSlop;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_SAT)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_enableSatConvex = clientCmd.m_physSimParamArgs.m_enableSAT != 0;
	}
	if (clientCmd.m_updateFlags & SIM_PARAM_CONSTRAINT_SOLVER_TYPE)
	{
		//check if the current type is different from requested one
		if (m_data->m_constraintSolverType != clientCmd.m_physSimParamArgs.m_constraintSolverType)
		{
			m_data->m_constraintSolverType = clientCmd.m_physSimParamArgs.m_constraintSolverType;

			btConstraintSolver* oldSolver = m_data->m_dynamicsWorld->getConstraintSolver();

			btMultiBodyConstraintSolver* newSolver = 0;

			switch (clientCmd.m_physSimParamArgs.m_constraintSolverType)
			{
				case eConstraintSolverLCP_SI:
				{
					newSolver = new btMultiBodyConstraintSolver;
					b3Printf("PyBullet: Constraint Solver: btMultiBodyConstraintSolver\n");
					break;
				}
				case eConstraintSolverLCP_PGS:
				{
					btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel();
					newSolver = new btMultiBodyMLCPConstraintSolver(mlcp);
					b3Printf("PyBullet: Constraint Solver: MLCP + PGS\n");
					break;
				}
				case eConstraintSolverLCP_DANTZIG:
				{
					btDantzigSolver* mlcp = new btDantzigSolver();
					newSolver = new btMultiBodyMLCPConstraintSolver(mlcp);
					b3Printf("PyBullet: Constraint Solver: MLCP + Dantzig\n");
					break;
				}
				case eConstraintSolverLCP_BLOCK_PGS:
				{
					break;
				}
				default:
				{
				}
			};

			if (newSolver)
			{
				delete oldSolver;

				m_data->m_dynamicsWorld->setMultiBodyConstraintSolver(newSolver);
				m_data->m_solver = newSolver;
				printf("switched solver\n");
			}
		}
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_COLLISION_FILTER_MODE)
	{
		m_data->m_broadphaseCollisionFilterCallback->m_filterMode = clientCmd.m_physSimParamArgs.m_collisionFilterMode;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_USE_SPLIT_IMPULSE)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulse = clientCmd.m_physSimParamArgs.m_useSplitImpulse;
	}
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_SPLIT_IMPULSE_PENETRATION_THRESHOLD)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = clientCmd.m_physSimParamArgs.m_splitImpulsePenetrationThreshold;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS)
	{
		m_data->m_numSimulationSubSteps = clientCmd.m_physSimParamArgs.m_numSimulationSubSteps;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_CONTACT_ERP)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_erp2 = clientCmd.m_physSimParamArgs.m_defaultContactERP;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_NON_CONTACT_ERP)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_erp = clientCmd.m_physSimParamArgs.m_defaultNonContactERP;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_ERP)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_frictionERP = clientCmd.m_physSimParamArgs.m_frictionERP;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_GLOBAL_CFM)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_globalCfm = clientCmd.m_physSimParamArgs.m_defaultGlobalCFM;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_CFM)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_frictionCFM = clientCmd.m_physSimParamArgs.m_frictionCFM;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_restitutionVelocityThreshold = clientCmd.m_physSimParamArgs.m_restitutionVelocityThreshold;
	}

	if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_FILE_CACHING)
	{
		b3EnableFileCaching(clientCmd.m_physSimParamArgs.m_enableFileCaching);
		m_data->m_pluginManager.getFileIOInterface()->enableFileCaching(clientCmd.m_physSimParamArgs.m_enableFileCaching!=0);
	}

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processInitPoseCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

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
			btVector3 zero(0, 0, 0);
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
			for (int i = 0; i < mb->getNumLinks(); i++)
			{
				bool hasPosVar = true;

				for (int j = 0; j < mb->getLink(i).m_posVarCount; j++)
				{
					if (clientCmd.m_initPoseArgs.m_hasInitialStateQ[posVarCountIndex+j] == 0)
					{
						hasPosVar = false;
						break;
					}
				}
				if (hasPosVar)
				{
					mb->setJointPosMultiDof(i, &clientCmd.m_initPoseArgs.m_initialStateQ[posVarCountIndex]);
					double vel[6] = { 0, 0, 0, 0, 0, 0 };
					mb->setJointVelMultiDof(i, vel);
				}
				
				bool hasVel = true;
				for (int j = 0; j < mb->getLink(i).m_dofCount; j++)
				{
					if (clientCmd.m_initPoseArgs.m_hasInitialStateQdot[uDofIndex + j] == 0)
					{
						hasVel = false;
						break;
					}
				}

				if (hasVel)
				{
					mb->setJointVelMultiDof(i, &clientCmd.m_initPoseArgs.m_initialStateQdot[uDofIndex]);
				}
								
				posVarCountIndex += mb->getLink(i).m_posVarCount;
				uDofIndex += mb->getLink(i).m_dofCount;
			}
		}

		btAlignedObjectArray<btQuaternion> scratch_q;
		btAlignedObjectArray<btVector3> scratch_m;

		mb->forwardKinematics(scratch_q, scratch_m);
		int nLinks = mb->getNumLinks();
		scratch_q.resize(nLinks + 1);
		scratch_m.resize(nLinks + 1);

		mb->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);
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

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processResetSimulationCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_RESET_SIMULATION");
	m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL, 0);
	resetSimulation();
	m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL, 1);

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateRigidBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_RIGID_BODY_CREATION_COMPLETED;

	BT_PROFILE("CMD_CREATE_RIGID_BODY");

	btVector3 halfExtents(1, 1, 1);
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

	btMultiBodyWorldImporter* worldImporter = new btMultiBodyWorldImporter(m_data->m_dynamicsWorld);
	m_data->m_worldImporters.push_back(worldImporter);

	btCollisionShape* shape = 0;

	switch (shapeType)
	{
		case COLLISION_SHAPE_TYPE_CYLINDER_X:
		{
			btScalar radius = halfExtents[1];
			btScalar height = halfExtents[0];
			shape = worldImporter->createCylinderShapeX(radius, height);
			break;
		}
		case COLLISION_SHAPE_TYPE_CYLINDER_Y:
		{
			btScalar radius = halfExtents[0];
			btScalar height = halfExtents[1];
			shape = worldImporter->createCylinderShapeY(radius, height);
			break;
		}
		case COLLISION_SHAPE_TYPE_CYLINDER_Z:
		{
			btScalar radius = halfExtents[1];
			btScalar height = halfExtents[2];
			shape = worldImporter->createCylinderShapeZ(radius, height);
			break;
		}
		case COLLISION_SHAPE_TYPE_CAPSULE_X:
		{
			btScalar radius = halfExtents[1];
			btScalar height = halfExtents[0];
			shape = worldImporter->createCapsuleShapeX(radius, height);
			break;
		}
		case COLLISION_SHAPE_TYPE_CAPSULE_Y:
		{
			btScalar radius = halfExtents[0];
			btScalar height = halfExtents[1];
			shape = worldImporter->createCapsuleShapeY(radius, height);
			break;
		}
		case COLLISION_SHAPE_TYPE_CAPSULE_Z:
		{
			btScalar radius = halfExtents[1];
			btScalar height = halfExtents[2];
			shape = worldImporter->createCapsuleShapeZ(radius, height);
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

	bool isDynamic = (mass > 0);
	btRigidBody* rb = worldImporter->createRigidBody(isDynamic, mass, startTrans, shape, 0);
	//m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);
	btVector4 colorRGBA(1, 0, 0, 1);
	if (clientCmd.m_updateFlags & BOX_SHAPE_HAS_COLOR)
	{
		colorRGBA[0] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[0];
		colorRGBA[1] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[1];
		colorRGBA[2] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[2];
		colorRGBA[3] = clientCmd.m_createBoxShapeArguments.m_colorRGBA[3];
	}
	m_data->m_guiHelper->createCollisionShapeGraphicsObject(rb->getCollisionShape());
	m_data->m_guiHelper->createCollisionObjectGraphicsObject(rb, colorRGBA);

	int bodyUniqueId = m_data->m_bodyHandles.allocHandle();
	InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
	serverCmd.m_rigidBodyCreateArgs.m_bodyUniqueId = bodyUniqueId;
	rb->setUserIndex2(bodyUniqueId);
	bodyHandle->m_rootLocalInertialFrame.setIdentity();
	bodyHandle->m_rigidBody = rb;

	b3Notification notification;
	notification.m_notificationType = BODY_ADDED;
	notification.m_bodyArgs.m_bodyUniqueId = bodyUniqueId;
	m_data->m_pluginManager.addNotification(notification);

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processPickBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_PICK_BODY");

	pickBody(btVector3(clientCmd.m_pickBodyArguments.m_rayFromWorld[0],
					   clientCmd.m_pickBodyArguments.m_rayFromWorld[1],
					   clientCmd.m_pickBodyArguments.m_rayFromWorld[2]),
			 btVector3(clientCmd.m_pickBodyArguments.m_rayToWorld[0],
					   clientCmd.m_pickBodyArguments.m_rayToWorld[1],
					   clientCmd.m_pickBodyArguments.m_rayToWorld[2]));

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processMovePickedBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_MOVE_PICKED_BODY");

	movePickedBody(btVector3(clientCmd.m_pickBodyArguments.m_rayFromWorld[0],
							 clientCmd.m_pickBodyArguments.m_rayFromWorld[1],
							 clientCmd.m_pickBodyArguments.m_rayFromWorld[2]),
				   btVector3(clientCmd.m_pickBodyArguments.m_rayToWorld[0],
							 clientCmd.m_pickBodyArguments.m_rayToWorld[1],
							 clientCmd.m_pickBodyArguments.m_rayToWorld[2]));

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRemovePickingConstraintCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_REMOVE_PICKING_CONSTRAINT_BODY");
	removePickingConstraint();

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestAabbOverlapCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_REQUEST_AABB_OVERLAP");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	int curObjectIndex = clientCmd.m_requestOverlappingObjectsArgs.m_startingOverlappingObjectIndex;

	if (0 == curObjectIndex)
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
	int numOverlap = m_data->m_cachedOverlappingObjects.m_bodyUniqueIds.size();
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRequestOpenGLVisualizeCameraCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

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
	serverCmd.m_type = result ? CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED : CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processConfigureOpenGLVisualizerCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_CONFIGURE_OPENGL_VISUALIZER");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;

	hasStatus = true;

	if (clientCmd.m_updateFlags & COV_SET_FLAGS)
	{
		if (clientCmd.m_configureOpenGLVisualizerArguments.m_setFlag == COV_ENABLE_TINY_RENDERER)
		{
			m_data->m_enableTinyRenderer = clientCmd.m_configureOpenGLVisualizerArguments.m_setEnabled != 0;
		}
		m_data->m_guiHelper->setVisualizerFlag(clientCmd.m_configureOpenGLVisualizerArguments.m_setFlag,
											   clientCmd.m_configureOpenGLVisualizerArguments.m_setEnabled);
	}
	if (clientCmd.m_updateFlags & COV_SET_CAMERA_VIEW_MATRIX)
	{
		m_data->m_guiHelper->resetCamera(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraDistance,
										 clientCmd.m_configureOpenGLVisualizerArguments.m_cameraYaw,
										 clientCmd.m_configureOpenGLVisualizerArguments.m_cameraPitch,
										 clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[0],
										 clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[1],
										 clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[2]);
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processInverseDynamicsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_CALCULATE_INVERSE_DYNAMICS");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId);
	serverCmd.m_type = CMD_CALCULATED_INVERSE_DYNAMICS_FAILED;
	if (bodyHandle && bodyHandle->m_multiBody)
	{

		btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);

		int baseDofQ = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 7;
		int baseDofQdot = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
		const int num_dofs = bodyHandle->m_multiBody->getNumDofs();

		if (tree && clientCmd.m_calculateInverseDynamicsArguments.m_dofCountQ == (baseDofQ+ num_dofs) &&
			clientCmd.m_calculateInverseDynamicsArguments.m_dofCountQdot == (baseDofQdot+ num_dofs))
		{
			
			btInverseDynamics::vecx nu(num_dofs + baseDofQdot), qdot(num_dofs + baseDofQdot), q(num_dofs + baseDofQdot), joint_force(num_dofs + baseDofQdot);

			//for floating base, inverse dynamics expects euler angle x,y,z and position x,y,z in that order
			//PyBullet expects quaternion, so convert and swap to have a more consistent PyBullet API
			if (baseDofQ)
			{
				btVector3 pos(clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[0],
					clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[1],
					clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[2]);

				btQuaternion orn(clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[0],
					clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[1],
					clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[2],
					clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[3]);
				btScalar yawZ, pitchY, rollX;
				orn.getEulerZYX(yawZ, pitchY, rollX);
				q[0] = rollX;
				q[1] = pitchY;
				q[2] = yawZ;
				q[3] = pos[0];
				q[4] = pos[1];
				q[5] = pos[2];
			}
			else
			{
				for (int i = 0; i < num_dofs; i++)
				{
					q[i] = clientCmd.m_calculateInverseDynamicsArguments.m_jointPositionsQ[i];
				}
			}
			for (int i = 0; i < num_dofs + baseDofQdot; i++)
			{
				qdot[i] = clientCmd.m_calculateInverseDynamicsArguments.m_jointVelocitiesQdot[i];
				nu[i] = clientCmd.m_calculateInverseDynamicsArguments.m_jointAccelerations[i];
			}

			// Set the gravity to correspond to the world gravity
			btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());

			if (-1 != tree->setGravityInWorldFrame(id_grav) &&
				-1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
			{
				serverCmd.m_inverseDynamicsResultArgs.m_bodyUniqueId = clientCmd.m_calculateInverseDynamicsArguments.m_bodyUniqueId;
				serverCmd.m_inverseDynamicsResultArgs.m_dofCount = num_dofs+ baseDofQdot;

				//inverse dynamics stores angular before linear, swap it to have a consistent PyBullet API.
				if (baseDofQdot)
				{
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[0] = joint_force[3];
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[1] = joint_force[4];
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[2] = joint_force[5];
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[3] = joint_force[0];
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[4] = joint_force[1];
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[5] = joint_force[2];
				}
				
				for (int i = baseDofQdot; i < num_dofs+ baseDofQdot; i++)
				{
					serverCmd.m_inverseDynamicsResultArgs.m_jointForces[i] = joint_force[i];
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

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCalculateJacobianCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
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
				for (int i = 0; i < 3; ++i)
				{
					localPosition(i) = clientCmd.m_calculateJacobianArguments.m_localPosition[i];
				}
				// Only calculate if the localPosition is non-zero.
				if (btInverseDynamics::maxAbs(localPosition) > 0.0)
				{
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
						serverCmd.m_jacobianResultArgs.m_linearJacobian[element] = jac_t(i, j);
						serverCmd.m_jacobianResultArgs.m_angularJacobian[element] = jac_r(i, j);
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

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCalculateMassMatrixCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_CALCULATE_MASS_MATRIX");

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CALCULATED_MASS_MATRIX_FAILED;
	InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(clientCmd.m_calculateMassMatrixArguments.m_bodyUniqueId);
	if (bodyHandle && bodyHandle->m_multiBody)
	{
		btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);

		if (tree)
		{
			int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
			const int numDofs = bodyHandle->m_multiBody->getNumDofs();
			const int totDofs = numDofs + baseDofs;
			btInverseDynamics::vecx q(totDofs);
			btInverseDynamics::matxx massMatrix(totDofs, totDofs);
			for (int i = 0; i < numDofs; i++)
			{
				q[i + baseDofs] = clientCmd.m_calculateMassMatrixArguments.m_jointPositionsQ[i];
			}
			if (-1 != tree->calculateMassMatrix(q, &massMatrix))
			{
				serverCmd.m_massMatrixResultArgs.m_dofCount = totDofs;
				// Fill in the result into the shared memory.
				double* sharedBuf = (double*)bufferServerToClient;
				int sizeInBytes = totDofs * totDofs * sizeof(double);
				if (sizeInBytes < bufferSizeInBytes)
				{
					for (int i = 0; i < (totDofs); ++i)
					{
						for (int j = 0; j < (totDofs); ++j)
						{
							int element = (totDofs)*i + j;

							sharedBuf[element] = massMatrix(i, j);
						}
					}
					serverCmd.m_type = CMD_CALCULATED_MASS_MATRIX_COMPLETED;
				}
			}
		}
	}
	else
	{
		serverCmd.m_type = CMD_CALCULATED_MASS_MATRIX_FAILED;
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processApplyExternalForceCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

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

			if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_FORCE) != 0)
			{
				btVector3 tmpForce(clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 0],
								   clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 1],
								   clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 2]);
				btVector3 tmpPosition(
					clientCmd.m_externalForceArguments.m_positions[i * 3 + 0],
					clientCmd.m_externalForceArguments.m_positions[i * 3 + 1],
					clientCmd.m_externalForceArguments.m_positions[i * 3 + 2]);

				if (clientCmd.m_externalForceArguments.m_linkIds[i] == -1)
				{
					btVector3 forceWorld = isLinkFrame ? mb->getBaseWorldTransform().getBasis() * tmpForce : tmpForce;
					btVector3 relPosWorld = isLinkFrame ? mb->getBaseWorldTransform().getBasis() * tmpPosition : tmpPosition - mb->getBaseWorldTransform().getOrigin();
					mb->addBaseForce(forceWorld);
					mb->addBaseTorque(relPosWorld.cross(forceWorld));
					//b3Printf("apply base force of %f,%f,%f at %f,%f,%f\n", forceWorld[0],forceWorld[1],forceWorld[2],positionLocal[0],positionLocal[1],positionLocal[2]);
				}
				else
				{
					int link = clientCmd.m_externalForceArguments.m_linkIds[i];

					btVector3 forceWorld = isLinkFrame ? mb->getLink(link).m_cachedWorldTransform.getBasis() * tmpForce : tmpForce;
					btVector3 relPosWorld = isLinkFrame ? mb->getLink(link).m_cachedWorldTransform.getBasis() * tmpPosition : tmpPosition - mb->getBaseWorldTransform().getOrigin();
					mb->addLinkForce(link, forceWorld);
					mb->addLinkTorque(link, relPosWorld.cross(forceWorld));
					//b3Printf("apply link force of %f,%f,%f at %f,%f,%f\n", forceWorld[0],forceWorld[1],forceWorld[2], positionLocal[0],positionLocal[1],positionLocal[2]);
				}
			}
			if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_TORQUE) != 0)
			{
				btVector3 torqueLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 0],
									  clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 1],
									  clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 2]);

				if (clientCmd.m_externalForceArguments.m_linkIds[i] == -1)
				{
					btVector3 torqueWorld = isLinkFrame ? torqueLocal : mb->getBaseWorldTransform().getBasis() * torqueLocal;
					mb->addBaseTorque(torqueWorld);
					//b3Printf("apply base torque of %f,%f,%f\n", torqueWorld[0],torqueWorld[1],torqueWorld[2]);
				}
				else
				{
					int link = clientCmd.m_externalForceArguments.m_linkIds[i];
					btVector3 torqueWorld = mb->getLink(link).m_cachedWorldTransform.getBasis() * torqueLocal;
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

				btVector3 forceWorld = isLinkFrame ? forceLocal : rb->getWorldTransform().getBasis() * forceLocal;
				btVector3 relPosWorld = isLinkFrame ? positionLocal : rb->getWorldTransform().getBasis() * positionLocal;
				rb->applyForce(forceWorld, relPosWorld);
			}

			if ((clientCmd.m_externalForceArguments.m_forceFlags[i] & EF_TORQUE) != 0)
			{
				btVector3 torqueLocal(clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 0],
									  clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 1],
									  clientCmd.m_externalForceArguments.m_forcesAndTorques[i * 3 + 2]);

				btVector3 torqueWorld = isLinkFrame ? torqueLocal : rb->getWorldTransform().getBasis() * torqueLocal;
				rb->applyTorque(torqueWorld);
			}
		}
	}

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRemoveBodyCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_REMOVE_BODY_FAILED;
	serverCmd.m_removeObjectArgs.m_numBodies = 0;
	serverCmd.m_removeObjectArgs.m_numUserConstraints = 0;

	m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL, 0);

	for (int i = 0; i < clientCmd.m_removeObjectArgs.m_numBodies; i++)
	{
		int bodyUniqueId = clientCmd.m_removeObjectArgs.m_bodyUniqueIds[i];
		InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
		if (bodyHandle)
		{
			if (bodyHandle->m_multiBody)
			{
				serverCmd.m_removeObjectArgs.m_bodyUniqueIds[serverCmd.m_removeObjectArgs.m_numBodies++] = bodyUniqueId;

				if (m_data->m_pickingMultiBodyPoint2Point && m_data->m_pickingMultiBodyPoint2Point->getMultiBodyA() == bodyHandle->m_multiBody)
				{
					//memory will be deleted in the code that follows
					m_data->m_pickingMultiBodyPoint2Point = 0;
				}

				//also remove user constraints...
				for (int i = m_data->m_dynamicsWorld->getNumMultiBodyConstraints() - 1; i >= 0; i--)
				{
					btMultiBodyConstraint* mbc = m_data->m_dynamicsWorld->getMultiBodyConstraint(i);
					if ((mbc->getMultiBodyA() == bodyHandle->m_multiBody) || (mbc->getMultiBodyB() == bodyHandle->m_multiBody))
					{
						m_data->m_dynamicsWorld->removeMultiBodyConstraint(mbc);

						//also remove user constraint and submit it as removed
						for (int c = m_data->m_userConstraints.size() - 1; c >= 0; c--)
						{
							InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.getAtIndex(c);
							int userConstraintKey = m_data->m_userConstraints.getKeyAtIndex(c).getUid1();

							if (userConstraintPtr->m_mbConstraint == mbc)
							{
								m_data->m_userConstraints.remove(userConstraintKey);
								serverCmd.m_removeObjectArgs.m_userConstraintUniqueIds[serverCmd.m_removeObjectArgs.m_numUserConstraints++] = userConstraintKey;
							}
						}

						delete mbc;
					}
				}

				if (bodyHandle->m_multiBody->getBaseCollider())
				{
					if (m_data->m_pluginManager.getRenderInterface())
					{
						m_data->m_pluginManager.getRenderInterface()->removeVisualShape(bodyHandle->m_multiBody->getBaseCollider()->getBroadphaseHandle()->getUid());
					}
					m_data->m_dynamicsWorld->removeCollisionObject(bodyHandle->m_multiBody->getBaseCollider());
					int graphicsIndex = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
					m_data->m_guiHelper->removeGraphicsInstance(graphicsIndex);
				}
				for (int link = 0; link < bodyHandle->m_multiBody->getNumLinks(); link++)
				{
					if (bodyHandle->m_multiBody->getLink(link).m_collider)
					{
						if (m_data->m_pluginManager.getRenderInterface())
						{
							m_data->m_pluginManager.getRenderInterface()->removeVisualShape(bodyHandle->m_multiBody->getLink(link).m_collider->getBroadphaseHandle()->getUid());
						}
						m_data->m_dynamicsWorld->removeCollisionObject(bodyHandle->m_multiBody->getLink(link).m_collider);
						int graphicsIndex = bodyHandle->m_multiBody->getLink(link).m_collider->getUserIndex();
						m_data->m_guiHelper->removeGraphicsInstance(graphicsIndex);
					}
				}
				int numCollisionObjects = m_data->m_dynamicsWorld->getNumCollisionObjects();
				m_data->m_dynamicsWorld->removeMultiBody(bodyHandle->m_multiBody);
				numCollisionObjects = m_data->m_dynamicsWorld->getNumCollisionObjects();
				//todo: clear all other remaining data, release memory etc

				delete bodyHandle->m_multiBody;
				bodyHandle->m_multiBody = 0;
				serverCmd.m_type = CMD_REMOVE_BODY_COMPLETED;
			}
			if (bodyHandle->m_rigidBody)
			{
				if (m_data->m_pluginManager.getRenderInterface())
				{
					m_data->m_pluginManager.getRenderInterface()->removeVisualShape(bodyHandle->m_rigidBody->getBroadphaseHandle()->getUid());
				}
				serverCmd.m_removeObjectArgs.m_bodyUniqueIds[serverCmd.m_removeObjectArgs.m_numBodies++] = bodyUniqueId;

				if (m_data->m_pickedConstraint && m_data->m_pickedBody == bodyHandle->m_rigidBody)
				{
					m_data->m_pickedConstraint = 0;
					m_data->m_pickedBody = 0;
				}

				//todo: clear all other remaining data...
				m_data->m_dynamicsWorld->removeRigidBody(bodyHandle->m_rigidBody);
				int graphicsInstance = bodyHandle->m_rigidBody->getUserIndex2();
				m_data->m_guiHelper->removeGraphicsInstance(graphicsInstance);
				delete bodyHandle->m_rigidBody;
				bodyHandle->m_rigidBody = 0;
				serverCmd.m_type = CMD_REMOVE_BODY_COMPLETED;
			}
			for (int i = 0; i < bodyHandle->m_userDataHandles.size(); i++)
			{
				int userDataHandle = bodyHandle->m_userDataHandles[i];
				SharedMemoryUserData* userData = m_data->m_userDataHandles.getHandle(userDataHandle);
				m_data->m_userDataHandleLookup.remove(SharedMemoryUserDataHashKey(userData));
				m_data->m_userDataHandles.freeHandle(userDataHandle);
			}
			m_data->m_bodyHandles.freeHandle(bodyUniqueId);
		}
	}

	for (int i = 0; i < clientCmd.m_removeObjectArgs.m_numUserCollisionShapes; i++)
	{
		int removeCollisionShapeId = clientCmd.m_removeObjectArgs.m_userCollisionShapes[i];
		InternalCollisionShapeHandle* handle = m_data->m_userCollisionShapeHandles.getHandle(removeCollisionShapeId);
		if (handle && handle->m_collisionShape)
		{
			if (handle->m_used)
			{
				b3Warning("Don't remove collision shape: it is used.");
			}
			else
			{
				b3Warning("TODO: dealloc");
				int foundIndex = -1;

				for (int i = 0; i < m_data->m_worldImporters.size(); i++)
				{
					btMultiBodyWorldImporter* importer = m_data->m_worldImporters[i];
					for (int c = 0; c < importer->getNumCollisionShapes(); c++)
					{
						if (importer->getCollisionShapeByIndex(c) == handle->m_collisionShape)
						{
							if ((importer->getNumRigidBodies() == 0) &&
								(importer->getNumConstraints() == 0))
							{
								foundIndex = i;
								break;
							}
						}
					}
				}
				if (foundIndex >= 0)
				{
					btMultiBodyWorldImporter* importer = m_data->m_worldImporters[foundIndex];
					m_data->m_worldImporters.removeAtIndex(foundIndex);
					importer->deleteAllData();
					delete importer;
					m_data->m_userCollisionShapeHandles.freeHandle(removeCollisionShapeId);
					serverCmd.m_type = CMD_REMOVE_BODY_COMPLETED;
				}
			}
		}
	}

	m_data->m_guiHelper->setVisualizerFlag(COV_ENABLE_SYNC_RENDERING_INTERNAL, 1);

	for (int i = 0; i < serverCmd.m_removeObjectArgs.m_numBodies; i++)
	{
		b3Notification notification;
		notification.m_notificationType = BODY_REMOVED;
		notification.m_bodyArgs.m_bodyUniqueId = serverCmd.m_removeObjectArgs.m_bodyUniqueIds[i];
		m_data->m_pluginManager.addNotification(notification);
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCreateUserConstraintCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_USER_CONSTRAINT");

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_USER_CONSTRAINT_FAILED;
	hasStatus = true;
	if (clientCmd.m_updateFlags & USER_CONSTRAINT_REQUEST_STATE)
	{
		int constraintUid = clientCmd.m_userConstraintArguments.m_userConstraintUniqueId;
		InteralUserConstraintData* userConstraintPtr = m_data->m_userConstraints.find(constraintUid);
		if (userConstraintPtr)
		{
			serverCmd.m_userConstraintStateResultArgs.m_numDofs = 0;
			for (int i = 0; i < 6; i++)
			{
				serverCmd.m_userConstraintStateResultArgs.m_appliedConstraintForces[i] = 0;
			}
			if (userConstraintPtr->m_mbConstraint)
			{
				serverCmd.m_userConstraintStateResultArgs.m_numDofs = userConstraintPtr->m_mbConstraint->getNumRows();
				for (int i = 0; i < userConstraintPtr->m_mbConstraint->getNumRows(); i++)
				{
					serverCmd.m_userConstraintStateResultArgs.m_appliedConstraintForces[i] = userConstraintPtr->m_mbConstraint->getAppliedImpulse(i) / m_data->m_dynamicsWorld->getSolverInfo().m_timeStep;
				}
				serverCmd.m_type = CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED;
			}
		}
	};
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
			if ((clientCmd.m_userConstraintArguments.m_parentJointIndex >= -1) && clientCmd.m_userConstraintArguments.m_parentJointIndex < parentBody->m_multiBody->getNumLinks())
			{
				InternalBodyData* childBody = clientCmd.m_userConstraintArguments.m_childBodyIndex >= 0 ? m_data->m_bodyHandles.getHandle(clientCmd.m_userConstraintArguments.m_childBodyIndex) : 0;
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
							if ((clientCmd.m_userConstraintArguments.m_childJointIndex >= -1) && (clientCmd.m_userConstraintArguments.m_childJointIndex < childBody->m_multiBody->getNumLinks()))
							{
								btMultiBodyGearConstraint* multibodyGear = new btMultiBodyGearConstraint(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, childBody->m_multiBody, clientCmd.m_userConstraintArguments.m_childJointIndex, pivotInParent, pivotInChild, frameInParent, frameInChild);
								multibodyGear->setMaxAppliedImpulse(defaultMaxForce);
								m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodyGear);
								InteralUserConstraintData userConstraintData;
								userConstraintData.m_mbConstraint = multibodyGear;
								int uid = m_data->m_userConstraintUIDGenerator++;
								serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
								serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
								serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
								userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
								m_data->m_userConstraints.insert(uid, userConstraintData);
								serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
							}
						}
					}
					else if (clientCmd.m_userConstraintArguments.m_jointType == eFixedType)
					{
						if (childBody && childBody->m_multiBody)
						{
							if ((clientCmd.m_userConstraintArguments.m_childJointIndex >= -1) && (clientCmd.m_userConstraintArguments.m_childJointIndex < childBody->m_multiBody->getNumLinks()))
							{
								btMultiBodyFixedConstraint* multibodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, childBody->m_multiBody, clientCmd.m_userConstraintArguments.m_childJointIndex, pivotInParent, pivotInChild, frameInParent, frameInChild);
								multibodyFixed->setMaxAppliedImpulse(defaultMaxForce);
								m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodyFixed);
								InteralUserConstraintData userConstraintData;
								userConstraintData.m_mbConstraint = multibodyFixed;
								int uid = m_data->m_userConstraintUIDGenerator++;
								serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
								serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
								serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
								userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
								m_data->m_userConstraints.insert(uid, userConstraintData);
								serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
							}
						}
						else
						{
							btRigidBody* rb = childBody ? childBody->m_rigidBody : 0;
							btMultiBodyFixedConstraint* rigidbodyFixed = new btMultiBodyFixedConstraint(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, rb, pivotInParent, pivotInChild, frameInParent, frameInChild);
							rigidbodyFixed->setMaxAppliedImpulse(defaultMaxForce);
							btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
							world->addMultiBodyConstraint(rigidbodyFixed);
							InteralUserConstraintData userConstraintData;
							userConstraintData.m_mbConstraint = rigidbodyFixed;
							int uid = m_data->m_userConstraintUIDGenerator++;
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
							serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
							userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
							m_data->m_userConstraints.insert(uid, userConstraintData);
							serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
						}
					}
					else if (clientCmd.m_userConstraintArguments.m_jointType == ePrismaticType)
					{
						if (childBody && childBody->m_multiBody)
						{
							btMultiBodySliderConstraint* multibodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, childBody->m_multiBody, clientCmd.m_userConstraintArguments.m_childJointIndex, pivotInParent, pivotInChild, frameInParent, frameInChild, jointAxis);
							multibodySlider->setMaxAppliedImpulse(defaultMaxForce);
							m_data->m_dynamicsWorld->addMultiBodyConstraint(multibodySlider);
							InteralUserConstraintData userConstraintData;
							userConstraintData.m_mbConstraint = multibodySlider;
							int uid = m_data->m_userConstraintUIDGenerator++;
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
							serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
							userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
							m_data->m_userConstraints.insert(uid, userConstraintData);
							serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
						}
						else
						{
							btRigidBody* rb = childBody ? childBody->m_rigidBody : 0;

							btMultiBodySliderConstraint* rigidbodySlider = new btMultiBodySliderConstraint(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, rb, pivotInParent, pivotInChild, frameInParent, frameInChild, jointAxis);
							rigidbodySlider->setMaxAppliedImpulse(defaultMaxForce);
							btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
							world->addMultiBodyConstraint(rigidbodySlider);
							InteralUserConstraintData userConstraintData;
							userConstraintData.m_mbConstraint = rigidbodySlider;
							int uid = m_data->m_userConstraintUIDGenerator++;
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
							serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
							userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
							m_data->m_userConstraints.insert(uid, userConstraintData);
							serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
						}
					}
					else if (clientCmd.m_userConstraintArguments.m_jointType == ePoint2PointType)
					{
						if (childBody && childBody->m_multiBody)
						{
							btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, childBody->m_multiBody, clientCmd.m_userConstraintArguments.m_childJointIndex, pivotInParent, pivotInChild);
							p2p->setMaxAppliedImpulse(defaultMaxForce);
							m_data->m_dynamicsWorld->addMultiBodyConstraint(p2p);
							InteralUserConstraintData userConstraintData;
							userConstraintData.m_mbConstraint = p2p;
							int uid = m_data->m_userConstraintUIDGenerator++;
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
							serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
							userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
							m_data->m_userConstraints.insert(uid, userConstraintData);
							serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
						}
						else
						{
							btRigidBody* rb = childBody ? childBody->m_rigidBody : 0;

							btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(parentBody->m_multiBody, clientCmd.m_userConstraintArguments.m_parentJointIndex, rb, pivotInParent, pivotInChild);
							p2p->setMaxAppliedImpulse(defaultMaxForce);
							btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
							world->addMultiBodyConstraint(p2p);
							InteralUserConstraintData userConstraintData;
							userConstraintData.m_mbConstraint = p2p;
							int uid = m_data->m_userConstraintUIDGenerator++;
							serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
							serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
							serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
							userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
							m_data->m_userConstraints.insert(uid, userConstraintData);
							serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
						}
					}
					else
					{
						b3Warning("unknown constraint type");
					}
				}
			}
		}
		else
		{
			InternalBodyData* childBody = clientCmd.m_userConstraintArguments.m_childBodyIndex >= 0 ? m_data->m_bodyHandles.getHandle(clientCmd.m_userConstraintArguments.m_childBodyIndex) : 0;

			if (parentBody && childBody)
			{
				if (parentBody->m_rigidBody)
				{
					btRigidBody* parentRb = 0;
					if (clientCmd.m_userConstraintArguments.m_parentJointIndex == -1)
					{
						parentRb = parentBody->m_rigidBody;
					}
					else
					{
						if ((clientCmd.m_userConstraintArguments.m_parentJointIndex >= 0) &&
							(clientCmd.m_userConstraintArguments.m_parentJointIndex < parentBody->m_rigidBodyJoints.size()))
						{
							parentRb = &parentBody->m_rigidBodyJoints[clientCmd.m_userConstraintArguments.m_parentJointIndex]->getRigidBodyB();
						}
					}

					btRigidBody* childRb = 0;
					if (childBody->m_rigidBody)
					{
						if (clientCmd.m_userConstraintArguments.m_childJointIndex == -1)
						{
							childRb = childBody->m_rigidBody;
						}
						else
						{
							if ((clientCmd.m_userConstraintArguments.m_childJointIndex >= 0) && (clientCmd.m_userConstraintArguments.m_childJointIndex < childBody->m_rigidBodyJoints.size()))
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
							if (childRb && parentRb && (childRb != parentRb))
							{
								btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
								btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);

								btTransform offsetTrA, offsetTrB;
								offsetTrA.setIdentity();
								offsetTrA.setOrigin(pivotInParent);
								offsetTrB.setIdentity();
								offsetTrB.setOrigin(pivotInChild);

								btGeneric6DofSpring2Constraint* dof6 = new btGeneric6DofSpring2Constraint(*parentRb, *childRb, offsetTrA, offsetTrB);

								dof6->setLinearLowerLimit(btVector3(0, 0, 0));
								dof6->setLinearUpperLimit(btVector3(0, 0, 0));

								dof6->setAngularLowerLimit(btVector3(0, 0, 0));
								dof6->setAngularUpperLimit(btVector3(0, 0, 0));
								m_data->m_dynamicsWorld->addConstraint(dof6);
								InteralUserConstraintData userConstraintData;
								userConstraintData.m_rbConstraint = dof6;
								int uid = m_data->m_userConstraintUIDGenerator++;
								serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
								serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
								serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
								userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
								m_data->m_userConstraints.insert(uid, userConstraintData);
								serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
							}

							break;
						}

						case ePoint2PointType:
						{
							if (childRb && parentRb && (childRb != parentRb))
							{
								btVector3 pivotInParent(clientCmd.m_userConstraintArguments.m_parentFrame[0], clientCmd.m_userConstraintArguments.m_parentFrame[1], clientCmd.m_userConstraintArguments.m_parentFrame[2]);
								btVector3 pivotInChild(clientCmd.m_userConstraintArguments.m_childFrame[0], clientCmd.m_userConstraintArguments.m_childFrame[1], clientCmd.m_userConstraintArguments.m_childFrame[2]);

								btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*parentRb, *childRb, pivotInParent, pivotInChild);
								p2p->m_setting.m_impulseClamp = defaultMaxForce;
								m_data->m_dynamicsWorld->addConstraint(p2p);
								InteralUserConstraintData userConstraintData;
								userConstraintData.m_rbConstraint = p2p;
								int uid = m_data->m_userConstraintUIDGenerator++;
								serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
								serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
								serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
								userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
								m_data->m_userConstraints.insert(uid, userConstraintData);
								serverCmd.m_type = CMD_USER_CONSTRAINT_COMPLETED;
							}
							break;
						}

						case eGearType:
						{
							if (childRb && parentRb && (childRb != parentRb))
							{
								btVector3 axisA(clientCmd.m_userConstraintArguments.m_jointAxis[0],
												clientCmd.m_userConstraintArguments.m_jointAxis[1],
												clientCmd.m_userConstraintArguments.m_jointAxis[2]);
								//for now we use the same local axis for both objects
								btVector3 axisB(clientCmd.m_userConstraintArguments.m_jointAxis[0],
												clientCmd.m_userConstraintArguments.m_jointAxis[1],
												clientCmd.m_userConstraintArguments.m_jointAxis[2]);
								btScalar ratio = 1;
								btGearConstraint* gear = new btGearConstraint(*parentRb, *childRb, axisA, axisB, ratio);
								m_data->m_dynamicsWorld->addConstraint(gear, true);
								InteralUserConstraintData userConstraintData;
								userConstraintData.m_rbConstraint = gear;
								int uid = m_data->m_userConstraintUIDGenerator++;
								serverCmd.m_userConstraintResultArgs = clientCmd.m_userConstraintArguments;
								serverCmd.m_userConstraintResultArgs.m_userConstraintUniqueId = uid;
								serverCmd.m_userConstraintResultArgs.m_maxAppliedForce = defaultMaxForce;
								userConstraintData.m_userConstraintData = serverCmd.m_userConstraintResultArgs;
								m_data->m_userConstraints.insert(uid, userConstraintData);
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
					btScalar maxImp = clientCmd.m_userConstraintArguments.m_maxAppliedForce * m_data->m_physicsDeltaTime;
					userConstraintPtr->m_userConstraintData.m_maxAppliedForce = clientCmd.m_userConstraintArguments.m_maxAppliedForce;
					userConstraintPtr->m_mbConstraint->setMaxAppliedImpulse(maxImp);
				}

				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
				{
					userConstraintPtr->m_mbConstraint->setGearRatio(clientCmd.m_userConstraintArguments.m_gearRatio);
					userConstraintPtr->m_userConstraintData.m_gearRatio = clientCmd.m_userConstraintArguments.m_gearRatio;
				}
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET)
				{
					userConstraintPtr->m_mbConstraint->setRelativePositionTarget(clientCmd.m_userConstraintArguments.m_relativePositionTarget);
					userConstraintPtr->m_userConstraintData.m_relativePositionTarget = clientCmd.m_userConstraintArguments.m_relativePositionTarget;
				}
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_ERP)
				{
					userConstraintPtr->m_mbConstraint->setErp(clientCmd.m_userConstraintArguments.m_erp);
					userConstraintPtr->m_userConstraintData.m_erp = clientCmd.m_userConstraintArguments.m_erp;
				}

				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK)
				{
					userConstraintPtr->m_mbConstraint->setGearAuxLink(clientCmd.m_userConstraintArguments.m_gearAuxLink);
					userConstraintPtr->m_userConstraintData.m_gearAuxLink = clientCmd.m_userConstraintArguments.m_gearAuxLink;
				}
			}
			if (userConstraintPtr->m_rbConstraint)
			{
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
				{
					btScalar maxImp = clientCmd.m_userConstraintArguments.m_maxAppliedForce * m_data->m_physicsDeltaTime;
					userConstraintPtr->m_userConstraintData.m_maxAppliedForce = clientCmd.m_userConstraintArguments.m_maxAppliedForce;
					//userConstraintPtr->m_rbConstraint->setMaxAppliedImpulse(maxImp);
				}

				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
				{
					if (userConstraintPtr->m_rbConstraint->getObjectType() == GEAR_CONSTRAINT_TYPE)
					{
						btGearConstraint* gear = (btGearConstraint*)userConstraintPtr->m_rbConstraint;
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCalculateInverseKinematicsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

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

		btAlignedObjectArray<double> startingPositions;
		startingPositions.reserve(bodyHandle->m_multiBody->getNumLinks());

		btVector3 targetPosWorld(clientCmd.m_calculateInverseKinematicsArguments.m_targetPosition[0],
								 clientCmd.m_calculateInverseKinematicsArguments.m_targetPosition[1],
								 clientCmd.m_calculateInverseKinematicsArguments.m_targetPosition[2]);

		btQuaternion targetOrnWorld(clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation[0],
									clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation[1],
									clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation[2],
									clientCmd.m_calculateInverseKinematicsArguments.m_targetOrientation[3]);

		btTransform targetBaseCoord;
		if (clientCmd.m_updateFlags & IK_HAS_CURRENT_JOINT_POSITIONS)
		{
			targetBaseCoord.setOrigin(targetPosWorld);
			targetBaseCoord.setRotation(targetOrnWorld);
		}
		else
		{
			btTransform targetWorld;
			targetWorld.setOrigin(targetPosWorld);
			targetWorld.setRotation(targetOrnWorld);
			btTransform tr = bodyHandle->m_multiBody->getBaseWorldTransform();
			targetBaseCoord = tr.inverse() * targetWorld;
		}

		{
			int DofIndex = 0;
			for (int i = 0; i < bodyHandle->m_multiBody->getNumLinks(); ++i)
			{
				if (bodyHandle->m_multiBody->getLink(i).m_jointType >= 0 && bodyHandle->m_multiBody->getLink(i).m_jointType <= 2)
				{
					// 0, 1, 2 represent revolute, prismatic, and spherical joint types respectively. Skip the fixed joints.
					double curPos = 0;
					if (clientCmd.m_updateFlags & IK_HAS_CURRENT_JOINT_POSITIONS)
					{
						curPos = clientCmd.m_calculateInverseKinematicsArguments.m_currentPositions[DofIndex];
					}
					else
					{
						curPos = bodyHandle->m_multiBody->getJointPos(i);
					}
					startingPositions.push_back(curPos);
					DofIndex++;
				}
			}
		}

		int numIterations = 20;
		if (clientCmd.m_updateFlags & IK_HAS_MAX_ITERATIONS)
		{
			numIterations = clientCmd.m_calculateInverseKinematicsArguments.m_maxNumIterations;
		}
		double residualThreshold = 1e-4;
		if (clientCmd.m_updateFlags & IK_HAS_RESIDUAL_THRESHOLD)
		{
			residualThreshold = clientCmd.m_calculateInverseKinematicsArguments.m_residualThreshold;
		}

		btScalar currentDiff = 1e30f;
		b3AlignedObjectArray<double> jacobian_linear;
		b3AlignedObjectArray<double> jacobian_angular;
		btAlignedObjectArray<double> q_current;
		btAlignedObjectArray<double> q_new;
		btAlignedObjectArray<double> lower_limit;
		btAlignedObjectArray<double> upper_limit;
		btAlignedObjectArray<double> joint_range;
		btAlignedObjectArray<double> rest_pose;
		const int numDofs = bodyHandle->m_multiBody->getNumDofs();
		int baseDofs = bodyHandle->m_multiBody->hasFixedBase() ? 0 : 6;
		btInverseDynamics::vecx nu(numDofs + baseDofs), qdot(numDofs + baseDofs), q(numDofs + baseDofs), joint_force(numDofs + baseDofs);

		for (int i = 0; i < numIterations && currentDiff > residualThreshold; i++)
		{
			BT_PROFILE("InverseKinematics1Step");
			if (ikHelperPtr && (endEffectorLinkIndex < bodyHandle->m_multiBody->getNumLinks()))
			{
				jacobian_linear.resize(3 * numDofs);
				jacobian_angular.resize(3 * numDofs);
				int jacSize = 0;

				btInverseDynamics::MultiBodyTree* tree = m_data->findOrCreateTree(bodyHandle->m_multiBody);

				q_current.resize(numDofs);

				if (tree && ((numDofs + baseDofs) == tree->numDoFs()))
				{
					btInverseDynamics::vec3 world_origin;
					btInverseDynamics::mat33 world_rot;

					jacSize = jacobian_linear.size();
					// Set jacobian value

					int DofIndex = 0;
					for (int i = 0; i < bodyHandle->m_multiBody->getNumLinks(); ++i)
					{
						if (bodyHandle->m_multiBody->getLink(i).m_jointType >= 0 && bodyHandle->m_multiBody->getLink(i).m_jointType <= 2)
						{
							// 0, 1, 2 represent revolute, prismatic, and spherical joint types respectively. Skip the fixed joints.

							double curPos = startingPositions[DofIndex];
							q_current[DofIndex] = curPos;
							q[DofIndex + baseDofs] = curPos;
							qdot[DofIndex + baseDofs] = 0;
							nu[DofIndex + baseDofs] = 0;
							DofIndex++;
						}
					}  // Set the gravity to correspond to the world gravity
					btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());

					{
						BT_PROFILE("calculateInverseDynamics");
						if (-1 != tree->setGravityInWorldFrame(id_grav) &&
							-1 != tree->calculateInverseDynamics(q, qdot, nu, &joint_force))
						{
							tree->calculateJacobians(q);
							btInverseDynamics::mat3x jac_t(3, numDofs + baseDofs);
							btInverseDynamics::mat3x jac_r(3, numDofs + baseDofs);
							// Note that inverse dynamics uses zero-based indexing of bodies, not starting from -1 for the base link.
							tree->getBodyJacobianTrans(endEffectorLinkIndex + 1, &jac_t);
							tree->getBodyJacobianRot(endEffectorLinkIndex + 1, &jac_r);

							//calculatePositionKinematics is already done inside calculateInverseDynamics
							tree->getBodyOrigin(endEffectorLinkIndex + 1, &world_origin);
							tree->getBodyTransform(endEffectorLinkIndex + 1, &world_rot);

							for (int i = 0; i < 3; ++i)
							{
								for (int j = 0; j < numDofs; ++j)
								{
									jacobian_linear[i * numDofs + j] = jac_t(i, (baseDofs + j));
									jacobian_angular[i * numDofs + j] = jac_r(i, (baseDofs + j));
								}
							}
						}
					}

					q_new.resize(numDofs);
					int ikMethod = 0;
					if ((clientCmd.m_updateFlags & IK_HAS_TARGET_ORIENTATION) && (clientCmd.m_updateFlags & IK_HAS_NULL_SPACE_VELOCITY))
					{
						//Nullspace task only works with DLS now. TODO: add nullspace task to SDLS.
						ikMethod = IK2_VEL_DLS_WITH_ORIENTATION_NULLSPACE;
					}
					else if (clientCmd.m_updateFlags & IK_HAS_TARGET_ORIENTATION)
					{
						if (clientCmd.m_updateFlags & IK_SDLS)
						{
							ikMethod = IK2_VEL_SDLS_WITH_ORIENTATION;
						}
						else
						{
							ikMethod = IK2_VEL_DLS_WITH_ORIENTATION;
						}
					}
					else if (clientCmd.m_updateFlags & IK_HAS_NULL_SPACE_VELOCITY)
					{
						//Nullspace task only works with DLS now. TODO: add nullspace task to SDLS.
						ikMethod = IK2_VEL_DLS_WITH_NULLSPACE;
					}
					else
					{
						if (clientCmd.m_updateFlags & IK_SDLS)
						{
							ikMethod = IK2_VEL_SDLS;
						}
						else
						{
							ikMethod = IK2_VEL_DLS;
							;
						}
					}

					if (clientCmd.m_updateFlags & IK_HAS_NULL_SPACE_VELOCITY)
					{
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
						{
							BT_PROFILE("computeNullspaceVel");
							ikHelperPtr->computeNullspaceVel(numDofs, &q_current[0], &lower_limit[0], &upper_limit[0], &joint_range[0], &rest_pose[0]);
						}
					}

					//btTransform endEffectorTransformWorld = bodyHandle->m_multiBody->getLink(endEffectorLinkIndex).m_cachedWorldTransform * bodyHandle->m_linkLocalInertialFrames[endEffectorLinkIndex].inverse();

					btVector3DoubleData endEffectorWorldPosition;
					btQuaternionDoubleData endEffectorWorldOrientation;

					//get the position from the inverse dynamics (based on q) instead of endEffectorTransformWorld
					btVector3 endEffectorPosWorldOrg = world_origin;
					btQuaternion endEffectorOriWorldOrg;
					world_rot.getRotation(endEffectorOriWorldOrg);

					btTransform endEffectorBaseCoord;
					endEffectorBaseCoord.setOrigin(endEffectorPosWorldOrg);
					endEffectorBaseCoord.setRotation(endEffectorOriWorldOrg);

					//don't need the next two lines
					//btTransform linkInertiaInv = bodyHandle->m_linkLocalInertialFrames[endEffectorLinkIndex].inverse();
					//endEffectorBaseCoord = endEffectorBaseCoord * linkInertiaInv;

					//btTransform tr = bodyHandle->m_multiBody->getBaseWorldTransform();
					//endEffectorBaseCoord = tr.inverse()*endEffectorTransformWorld;
					//endEffectorBaseCoord = tr.inverse()*endEffectorTransformWorld;

					btQuaternion endEffectorOriBaseCoord = endEffectorBaseCoord.getRotation();

					//btVector4 endEffectorOri(endEffectorOriBaseCoord.x(), endEffectorOriBaseCoord.y(), endEffectorOriBaseCoord.z(), endEffectorOriBaseCoord.w());

					endEffectorBaseCoord.getOrigin().serializeDouble(endEffectorWorldPosition);
					endEffectorBaseCoord.getRotation().serializeDouble(endEffectorWorldOrientation);

					//diff
					currentDiff = (endEffectorBaseCoord.getOrigin() - targetBaseCoord.getOrigin()).length();

					btVector3DoubleData targetPosBaseCoord;
					btQuaternionDoubleData targetOrnBaseCoord;
					targetBaseCoord.getOrigin().serializeDouble(targetPosBaseCoord);
					targetBaseCoord.getRotation().serializeDouble(targetOrnBaseCoord);

					// Set joint damping coefficents. A small default
					// damping constant is added to prevent singularity
					// with pseudo inverse. The user can set joint damping
					// coefficients differently for each joint. The larger
					// the damping coefficient is, the less we rely on
					// this joint to achieve the IK target.
					btAlignedObjectArray<double> joint_damping;
					joint_damping.resize(numDofs, 0.5);
					if (clientCmd.m_updateFlags & IK_HAS_JOINT_DAMPING)
					{
						for (int i = 0; i < numDofs; ++i)
						{
							joint_damping[i] = clientCmd.m_calculateInverseKinematicsArguments.m_jointDamping[i];
						}
					}
					ikHelperPtr->setDampingCoeff(numDofs, &joint_damping[0]);

					double targetDampCoeff[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

					{
						BT_PROFILE("computeIK");
						ikHelperPtr->computeIK(targetPosBaseCoord.m_floats, targetOrnBaseCoord.m_floats,
											   endEffectorWorldPosition.m_floats, endEffectorWorldOrientation.m_floats,
											   &q_current[0],
											   numDofs, clientCmd.m_calculateInverseKinematicsArguments.m_endEffectorLinkIndex,
											   &q_new[0], ikMethod, &jacobian_linear[0], &jacobian_angular[0], jacSize * 2, targetDampCoeff);
					}
					serverCmd.m_inverseKinematicsResultArgs.m_bodyUniqueId = clientCmd.m_calculateInverseKinematicsArguments.m_bodyUniqueId;
					for (int i = 0; i < numDofs; i++)
					{
						serverCmd.m_inverseKinematicsResultArgs.m_jointPositions[i] = q_new[i];
					}
					serverCmd.m_inverseKinematicsResultArgs.m_dofCount = numDofs;
					serverCmd.m_type = CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED;
					for (int i = 0; i < numDofs; i++)
					{
						startingPositions[i] = q_new[i];
					}
				}
			}
		}
	}
	return hasStatus;
}

//		PyModule_AddIntConstant(m, "GEOM_SPHERE", GEOM_SPHERE);
//		PyModule_AddIntConstant(m, "GEOM_BOX", GEOM_BOX);
//		PyModule_AddIntConstant(m, "GEOM_CYLINDER", GEOM_CYLINDER);
//		PyModule_AddIntConstant(m, "GEOM_MESH", GEOM_MESH);
//		PyModule_AddIntConstant(m, "GEOM_PLANE", GEOM_PLANE);
//		PyModule_AddIntConstant(m, "GEOM_CAPSULE", GEOM_CAPSULE);

int PhysicsServerCommandProcessor::extractCollisionShapes(const btCollisionShape* colShape, const btTransform& transform, b3CollisionShapeData* collisionShapeBuffer, int maxCollisionShapes)
{
	if (maxCollisionShapes <= 0)
	{
		b3Warning("No space in buffer");
		return 0;
	}

	int numConverted = 0;

	collisionShapeBuffer[0].m_localCollisionFrame[0] = transform.getOrigin()[0];
	collisionShapeBuffer[0].m_localCollisionFrame[1] = transform.getOrigin()[1];
	collisionShapeBuffer[0].m_localCollisionFrame[2] = transform.getOrigin()[2];
	collisionShapeBuffer[0].m_localCollisionFrame[3] = transform.getRotation()[0];
	collisionShapeBuffer[0].m_localCollisionFrame[4] = transform.getRotation()[1];
	collisionShapeBuffer[0].m_localCollisionFrame[5] = transform.getRotation()[2];
	collisionShapeBuffer[0].m_localCollisionFrame[6] = transform.getRotation()[3];
	collisionShapeBuffer[0].m_meshAssetFileName[0] = 0;

	switch (colShape->getShapeType())
	{
        case STATIC_PLANE_PROXYTYPE:
        {
            btStaticPlaneShape* plane = (btStaticPlaneShape*) colShape;
            collisionShapeBuffer[0].m_collisionGeometryType = GEOM_PLANE;
            collisionShapeBuffer[0].m_dimensions[0] = plane->getPlaneNormal()[0];
            collisionShapeBuffer[0].m_dimensions[1] = plane->getPlaneNormal()[1];
            collisionShapeBuffer[0].m_dimensions[2] = plane->getPlaneNormal()[2];
            numConverted += 1;
            break;
        }
		case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			UrdfCollision* urdfCol = m_data->m_bulletCollisionShape2UrdfCollision.find(colShape);
			if (urdfCol && (urdfCol->m_geometry.m_type == URDF_GEOM_MESH))
			{
				collisionShapeBuffer[0].m_collisionGeometryType = GEOM_MESH;
				collisionShapeBuffer[0].m_dimensions[0] = urdfCol->m_geometry.m_meshScale[0];
				collisionShapeBuffer[0].m_dimensions[1] = urdfCol->m_geometry.m_meshScale[1];
				collisionShapeBuffer[0].m_dimensions[2] = urdfCol->m_geometry.m_meshScale[2];
				strcpy(collisionShapeBuffer[0].m_meshAssetFileName, urdfCol->m_geometry.m_meshFileName.c_str());
				numConverted += 1;
			}
			else
			{
				collisionShapeBuffer[0].m_collisionGeometryType = GEOM_MESH;
				sprintf(collisionShapeBuffer[0].m_meshAssetFileName, "unknown_file");
				collisionShapeBuffer[0].m_dimensions[0] = 1;
				collisionShapeBuffer[0].m_dimensions[1] = 1;
				collisionShapeBuffer[0].m_dimensions[2] = 1;
				numConverted++;
			}

			break;
		}
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			btCapsuleShapeZ* capsule = (btCapsuleShapeZ*)colShape;
			collisionShapeBuffer[0].m_collisionGeometryType = GEOM_CAPSULE;
			collisionShapeBuffer[0].m_dimensions[0] = 2. * capsule->getHalfHeight();
			collisionShapeBuffer[0].m_dimensions[1] = capsule->getRadius();
			collisionShapeBuffer[0].m_dimensions[2] = 0;
			numConverted++;
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShapeZ* cyl = (btCylinderShapeZ*)colShape;
			collisionShapeBuffer[0].m_collisionGeometryType = GEOM_CYLINDER;
			collisionShapeBuffer[0].m_dimensions[0] = 2. * cyl->getHalfExtentsWithMargin().getZ();
			collisionShapeBuffer[0].m_dimensions[1] = cyl->getHalfExtentsWithMargin().getX();
			collisionShapeBuffer[0].m_dimensions[2] = 0;
			numConverted++;
			break;
		}
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* box = (btBoxShape*)colShape;
			btVector3 halfExtents = box->getHalfExtentsWithMargin();
			collisionShapeBuffer[0].m_collisionGeometryType = GEOM_BOX;
			collisionShapeBuffer[0].m_dimensions[0] = 2. * halfExtents[0];
			collisionShapeBuffer[0].m_dimensions[1] = 2. * halfExtents[1];
			collisionShapeBuffer[0].m_dimensions[2] = 2. * halfExtents[2];
			numConverted++;
			break;
		}
		case SPHERE_SHAPE_PROXYTYPE:
		{
			btSphereShape* sphere = (btSphereShape*)colShape;
			collisionShapeBuffer[0].m_collisionGeometryType = GEOM_SPHERE;
			collisionShapeBuffer[0].m_dimensions[0] = sphere->getRadius();
			collisionShapeBuffer[0].m_dimensions[1] = sphere->getRadius();
			collisionShapeBuffer[0].m_dimensions[2] = sphere->getRadius();
			numConverted++;
			break;
		}
		case COMPOUND_SHAPE_PROXYTYPE:
		{
			//it could be a compound mesh from a wavefront OBJ, check it
			UrdfCollision* urdfCol = m_data->m_bulletCollisionShape2UrdfCollision.find(colShape);
			if (urdfCol && (urdfCol->m_geometry.m_type == URDF_GEOM_MESH))
			{
				collisionShapeBuffer[0].m_collisionGeometryType = GEOM_MESH;
				collisionShapeBuffer[0].m_dimensions[0] = urdfCol->m_geometry.m_meshScale[0];
				collisionShapeBuffer[0].m_dimensions[1] = urdfCol->m_geometry.m_meshScale[1];
				collisionShapeBuffer[0].m_dimensions[2] = urdfCol->m_geometry.m_meshScale[2];
				strcpy(collisionShapeBuffer[0].m_meshAssetFileName, urdfCol->m_geometry.m_meshFileName.c_str());
				numConverted += 1;
			}
			else
			{
				//recurse, accumulate childTransform
				btCompoundShape* compound = (btCompoundShape*)colShape;
				for (int i = 0; i < compound->getNumChildShapes(); i++)
				{
					btTransform childTrans = transform * compound->getChildTransform(i);
					int remain = maxCollisionShapes - numConverted;
					int converted = extractCollisionShapes(compound->getChildShape(i), childTrans, &collisionShapeBuffer[numConverted], remain);
					numConverted += converted;
				}
			}
			break;
		}
		default:
		{
			b3Warning("Unexpected collision shape type in PhysicsServerCommandProcessor::extractCollisionShapes");
		}
	};

	return numConverted;
}

bool PhysicsServerCommandProcessor::processRequestCollisionShapeInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_REQUEST_COLLISION_SHAPE_INFO");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_COLLISION_SHAPE_INFO_FAILED;
	int bodyUniqueId = clientCmd.m_requestCollisionShapeDataArguments.m_bodyUniqueId;
	int linkIndex = clientCmd.m_requestCollisionShapeDataArguments.m_linkIndex;
	InternalBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);
	if (bodyHandle)
	{
		if (bodyHandle->m_multiBody)
		{
			b3CollisionShapeData* collisionShapeStoragePtr = (b3CollisionShapeData*)bufferServerToClient;
			int totalBytesPerObject = sizeof(b3CollisionShapeData);
			int maxNumColObjects = bufferSizeInBytes / totalBytesPerObject - 1;
			btTransform childTrans;
			childTrans.setIdentity();
			serverCmd.m_sendCollisionShapeArgs.m_bodyUniqueId = bodyUniqueId;
			serverCmd.m_sendCollisionShapeArgs.m_linkIndex = linkIndex;

			if (linkIndex == -1)
			{
				if (bodyHandle->m_multiBody->getBaseCollider())
				{
					//extract shape info from base collider
					int numConvertedCollisionShapes = extractCollisionShapes(bodyHandle->m_multiBody->getBaseCollider()->getCollisionShape(), childTrans, collisionShapeStoragePtr, maxNumColObjects);
					serverCmd.m_sendCollisionShapeArgs.m_numCollisionShapes = numConvertedCollisionShapes;
					serverCmd.m_type = CMD_COLLISION_SHAPE_INFO_COMPLETED;
				}
			}
			else
			{
				if (linkIndex >= 0 && linkIndex < bodyHandle->m_multiBody->getNumLinks() && bodyHandle->m_multiBody->getLinkCollider(linkIndex))
				{
					int numConvertedCollisionShapes = extractCollisionShapes(bodyHandle->m_multiBody->getLinkCollider(linkIndex)->getCollisionShape(), childTrans, collisionShapeStoragePtr, maxNumColObjects);
					serverCmd.m_sendCollisionShapeArgs.m_numCollisionShapes = numConvertedCollisionShapes;
					serverCmd.m_type = CMD_COLLISION_SHAPE_INFO_COMPLETED;
				}
			}
		}
	}

	return hasStatus;
}
bool PhysicsServerCommandProcessor::processRequestVisualShapeInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_REQUEST_VISUAL_SHAPE_INFO");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_VISUAL_SHAPE_INFO_FAILED;
	//retrieve the visual shape information for a specific body

	if (m_data->m_pluginManager.getRenderInterface())
	{
		int totalNumVisualShapes = m_data->m_pluginManager.getRenderInterface()->getNumVisualShapes(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId);
		//int totalBytesPerVisualShape = sizeof (b3VisualShapeData);
		//int visualShapeStorage = bufferSizeInBytes / totalBytesPerVisualShape - 1;
		b3VisualShapeData* visualShapeStoragePtr = (b3VisualShapeData*)bufferServerToClient;

		int remain = totalNumVisualShapes - clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
		int shapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;

		int success = m_data->m_pluginManager.getRenderInterface()->getVisualShapesData(clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId,
																						shapeIndex,
																						visualShapeStoragePtr);
		if (success)
		{
			//find the matching texture unique ids.
			if (visualShapeStoragePtr->m_tinyRendererTextureId >= 0)
			{
				b3AlignedObjectArray<int> usedHandles;
				m_data->m_textureHandles.getUsedHandles(usedHandles);

				for (int i = 0; i < usedHandles.size(); i++)
				{
					int texHandle = usedHandles[i];
					InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(texHandle);
					if (texH && (texH->m_tinyRendererTextureId == visualShapeStoragePtr->m_tinyRendererTextureId))
					{
						visualShapeStoragePtr->m_openglTextureId = texH->m_openglTextureId;
						visualShapeStoragePtr->m_textureUniqueId = texHandle;
					}
				}
			}

			serverCmd.m_sendVisualShapeArgs.m_numRemainingVisualShapes = remain - 1;
			serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied = 1;
			serverCmd.m_sendVisualShapeArgs.m_startingVisualShapeIndex = clientCmd.m_requestVisualShapeDataArguments.m_startingVisualShapeIndex;
			serverCmd.m_sendVisualShapeArgs.m_bodyUniqueId = clientCmd.m_requestVisualShapeDataArguments.m_bodyUniqueId;
			serverCmd.m_numDataStreamBytes = sizeof(b3VisualShapeData) * serverCmd.m_sendVisualShapeArgs.m_numVisualShapesCopied;
			serverCmd.m_type = CMD_VISUAL_SHAPE_INFO_COMPLETED;
		}
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processUpdateVisualShapeCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_UPDATE_VISUAL_SHAPE");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_FAILED;
	InternalTextureHandle* texHandle = 0;

	if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE)
	{
		if (clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId>=0)
		{
			texHandle = m_data->m_textureHandles.getHandle(clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId);
		}

		if (clientCmd.m_updateVisualShapeDataArguments.m_textureUniqueId >= -1)
		{
			if (texHandle)
			{
				if (m_data->m_pluginManager.getRenderInterface())
				{
					m_data->m_pluginManager.getRenderInterface()->changeShapeTexture(clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId,
																					 clientCmd.m_updateVisualShapeDataArguments.m_jointIndex,
																					 clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex,
																					 texHandle->m_tinyRendererTextureId);
				}
			} else
			{
				m_data->m_pluginManager.getRenderInterface()->changeShapeTexture(clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId,
																					 clientCmd.m_updateVisualShapeDataArguments.m_jointIndex,
																					 clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex,
																					 -1);
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
				if (linkIndex == -1)
				{
					if (bodyHandle->m_multiBody->getBaseCollider())
					{
						int graphicsIndex = bodyHandle->m_multiBody->getBaseCollider()->getUserIndex();
						if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE)
						{
							int shapeIndex = m_data->m_guiHelper->getShapeIndexFromInstance(graphicsIndex);
							if (texHandle)
							{
								m_data->m_guiHelper->replaceTexture(shapeIndex, texHandle->m_openglTextureId);
							} else
							{
								m_data->m_guiHelper->replaceTexture(shapeIndex, -1);
							}
						}
						if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR)
						{
							if (m_data->m_pluginManager.getRenderInterface())
							{
								m_data->m_pluginManager.getRenderInterface()->changeRGBAColor(bodyUniqueId, linkIndex,
																							  clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex,
																							  clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
							}
							m_data->m_guiHelper->changeRGBAColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
						}
						if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR)
						{
							m_data->m_guiHelper->changeSpecularColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_specularColor);
						}
					}
				}
				else
				{
					if (linkIndex < bodyHandle->m_multiBody->getNumLinks())
					{
						if (bodyHandle->m_multiBody->getLink(linkIndex).m_collider)
						{
							int graphicsIndex = bodyHandle->m_multiBody->getLink(linkIndex).m_collider->getUserIndex();
							if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE)
							{
								int shapeIndex = m_data->m_guiHelper->getShapeIndexFromInstance(graphicsIndex);
								if (texHandle)
								{
									m_data->m_guiHelper->replaceTexture(shapeIndex, texHandle->m_openglTextureId);
								}
								else
								{
									m_data->m_guiHelper->replaceTexture(shapeIndex, -1);
								}
							}
							if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR)
							{
								if (m_data->m_pluginManager.getRenderInterface())
								{
									m_data->m_pluginManager.getRenderInterface()->changeRGBAColor(bodyUniqueId, linkIndex,
																								  clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex, clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
								}
								m_data->m_guiHelper->changeRGBAColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
							}
							if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR)
							{
								m_data->m_guiHelper->changeSpecularColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_specularColor);
							}
						}
					}
				}
			}
			else
			{
				if (bodyHandle->m_rigidBody)
				{
					int graphicsIndex = bodyHandle->m_rigidBody->getUserIndex();
					if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_TEXTURE)
					{
						if (texHandle)
						{
							int shapeIndex = m_data->m_guiHelper->getShapeIndexFromInstance(graphicsIndex);
							m_data->m_guiHelper->replaceTexture(shapeIndex, texHandle->m_openglTextureId);
						}
					}
					if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_RGBA_COLOR)
					{
						if (m_data->m_pluginManager.getRenderInterface())
						{
							m_data->m_pluginManager.getRenderInterface()->changeRGBAColor(bodyUniqueId, linkIndex,
																						  clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex, clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
						}
						m_data->m_guiHelper->changeRGBAColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_rgbaColor);
					}
					if (clientCmd.m_updateFlags & CMD_UPDATE_VISUAL_SHAPE_SPECULAR_COLOR)
					{
						m_data->m_guiHelper->changeSpecularColor(graphicsIndex, clientCmd.m_updateVisualShapeDataArguments.m_specularColor);
					}
				}
			}
		}
	}

	serverCmd.m_type = CMD_VISUAL_SHAPE_UPDATE_COMPLETED;

	b3Notification notification;
	notification.m_notificationType = VISUAL_SHAPE_CHANGED;
	notification.m_visualShapeArgs.m_bodyUniqueId = clientCmd.m_updateVisualShapeDataArguments.m_bodyUniqueId;
	notification.m_visualShapeArgs.m_linkIndex = clientCmd.m_updateVisualShapeDataArguments.m_jointIndex;
	notification.m_visualShapeArgs.m_visualShapeIndex = clientCmd.m_updateVisualShapeDataArguments.m_shapeIndex;
	m_data->m_pluginManager.addNotification(notification);

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processChangeTextureCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CHANGE_TEXTURE_COMMAND_FAILED;

	InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(clientCmd.m_changeTextureArgs.m_textureUniqueId);
	if (texH)
	{
		int gltex = texH->m_openglTextureId;
		m_data->m_guiHelper->changeTexture(gltex,
										   (const unsigned char*)bufferServerToClient, clientCmd.m_changeTextureArgs.m_width, clientCmd.m_changeTextureArgs.m_height);

		serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadTextureCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_LOAD_TEXTURE");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_LOAD_TEXTURE_FAILED;

	char relativeFileName[1024];
	char pathPrefix[1024];

	CommonFileIOInterface* fileIO(m_data->m_pluginManager.getFileIOInterface());
	if (fileIO->findResourcePath(clientCmd.m_loadTextureArguments.m_textureFileName, relativeFileName, 1024))
	{
		b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);

		int texHandle = m_data->m_textureHandles.allocHandle();
		InternalTextureHandle* texH = m_data->m_textureHandles.getHandle(texHandle);
		if (texH)
		{
			texH->m_tinyRendererTextureId = -1;
			texH->m_openglTextureId = -1;

			int uid = -1;
			if (m_data->m_pluginManager.getRenderInterface())
			{
				uid = m_data->m_pluginManager.getRenderInterface()->loadTextureFile(relativeFileName, fileIO);
			}
			if (uid >= 0)
			{
				texH->m_tinyRendererTextureId = uid;
			}

			{
				int width, height, n;
				unsigned char* imageData = 0;

				CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
				if (fileIO)
				{
					b3AlignedObjectArray<char> buffer;
					buffer.reserve(1024);
					int fileId = fileIO->fileOpen(relativeFileName,"rb");
					if (fileId>=0)
					{
						int size = fileIO->getFileSize(fileId);
						if (size>0)
						{
							buffer.resize(size);
							int actual = fileIO->fileRead(fileId,&buffer[0],size);
							if (actual != size)
							{
								b3Warning("image filesize mismatch!\n");
								buffer.resize(0);
							}
						}
						fileIO->fileClose(fileId);
					}
					if (buffer.size())
					{
						imageData = stbi_load_from_memory((const unsigned char*)&buffer[0], buffer.size(), &width, &height, &n, 3);
					}
				} else
				{
					imageData = stbi_load(relativeFileName, &width, &height, &n, 3);
				}

				if (imageData)
				{
					texH->m_openglTextureId = m_data->m_guiHelper->registerTexture(imageData, width, height);
					free(imageData);
				}
				else
				{
					b3Warning("Unsupported texture image format [%s]\n", relativeFileName);
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
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSaveStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_RESTORE_STATE");
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_SAVE_STATE_FAILED;

	btDefaultSerializer* ser = new btDefaultSerializer();
	int currentFlags = ser->getSerializationFlags();
	ser->setSerializationFlags(currentFlags | BT_SERIALIZE_CONTACT_MANIFOLDS);
	m_data->m_dynamicsWorld->serialize(ser);
	bParse::btBulletFile* bulletFile = new bParse::btBulletFile((char*)ser->getBufferPointer(), ser->getCurrentBufferSize());
	bulletFile->parse(false);
	if (bulletFile->ok())
	{
		serverCmd.m_type = CMD_SAVE_STATE_COMPLETED;
		serverCmd.m_saveStateResultArgs.m_stateId = m_data->m_savedStates.size();
		SaveStateData sd;
		sd.m_bulletFile = bulletFile;
		sd.m_serializer = ser;
		m_data->m_savedStates.push_back(sd);
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processRestoreStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_RESTORE_STATE");
	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_RESTORE_STATE_FAILED;

	btMultiBodyWorldImporter* importer = new btMultiBodyWorldImporter(m_data->m_dynamicsWorld);
	importer->setImporterFlags(eRESTORE_EXISTING_OBJECTS);

	bool ok = false;

	if (clientCmd.m_loadStateArguments.m_stateId >= 0)
	{
		if (clientCmd.m_loadStateArguments.m_stateId < m_data->m_savedStates.size())
		{
			bParse::btBulletFile* bulletFile = m_data->m_savedStates[clientCmd.m_loadStateArguments.m_stateId].m_bulletFile;
			ok = importer->convertAllObjects(bulletFile);
		}
	}
	else
	{
		bool found = false;
		char fileName[1024];
		fileName[0] = 0;

		CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
		b3AlignedObjectArray<char> buffer;
		buffer.reserve(1024);
		if (fileIO)
		{

			int fileId = -1;
			found = fileIO->findResourcePath(clientCmd.m_fileArguments.m_fileName,  fileName, 1024);
			if (found)
			{
				fileId = fileIO->fileOpen(fileName,"rb");
			}
			if (fileId>=0)
			{
				int size = fileIO->getFileSize(fileId);
				if (size>0)
				{
					buffer.resize(size);
					int actual = fileIO->fileRead(fileId,&buffer[0],size);
					if (actual != size)
					{
						b3Warning("image filesize mismatch!\n");
						buffer.resize(0);
					} else
					{
						found=true;
					}
				}
				fileIO->fileClose(fileId);
			}
		}

		if (found && buffer.size())
		{
			ok = importer->loadFileFromMemory(&buffer[0], buffer.size());
		} else
		{
			b3Error("Error in restoreState: cannot load file %s\n", clientCmd.m_fileArguments.m_fileName);
		}
	}
	if (ok)
	{
		serverCmd.m_type = CMD_RESTORE_STATE_COMPLETED;
	}

	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadBulletCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_LOAD_BULLET");

	bool hasStatus = true;
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_BULLET_LOADING_FAILED;

	//btBulletWorldImporter* importer = new btBulletWorldImporter(m_data->m_dynamicsWorld);
	btMultiBodyWorldImporter* importer = new btMultiBodyWorldImporter(m_data->m_dynamicsWorld);

	bool found = false;

	CommonFileIOInterface* fileIO = m_data->m_pluginManager.getFileIOInterface();
	b3AlignedObjectArray<char> buffer;
	buffer.reserve(1024);
	if (fileIO)
	{
		char fileName[1024];
		int fileId = -1;
		found = fileIO->findResourcePath(clientCmd.m_fileArguments.m_fileName,  fileName, 1024);
		if (found)
		{
			fileId = fileIO->fileOpen(fileName,"rb");
		}
		if (fileId>=0)
		{
			int size = fileIO->getFileSize(fileId);
			if (size>0)
			{
				buffer.resize(size);
				int actual = fileIO->fileRead(fileId,&buffer[0],size);
				if (actual != size)
				{
					b3Warning("image filesize mismatch!\n");
					buffer.resize(0);
				} else
				{
					found=true;
				}
			}
			fileIO->fileClose(fileId);
		}
	}

	if (found && buffer.size())
	{
		bool ok = importer->loadFileFromMemory(&buffer[0], buffer.size());
		if (ok)
		{
			int numRb = importer->getNumRigidBodies();
			serverStatusOut.m_sdfLoadedArgs.m_numBodies = 0;
			serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;

			for (int i = 0; i < numRb; i++)
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

						if (serverStatusOut.m_sdfLoadedArgs.m_numBodies < MAX_SDF_BODIES)
						{
							serverStatusOut.m_sdfLoadedArgs.m_numBodies++;
							serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = bodyUniqueId;
						}

						b3Notification notification;
						notification.m_notificationType = BODY_ADDED;
						notification.m_bodyArgs.m_bodyUniqueId = bodyUniqueId;
						m_data->m_pluginManager.addNotification(notification);
					}
				}
			}

			serverCmd.m_type = CMD_BULLET_LOADING_COMPLETED;
			m_data->m_guiHelper->autogenerateGraphicsObjects(m_data->m_dynamicsWorld);
		}
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processLoadMJCFCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_LOAD_MJCF");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_MJCF_LOADING_FAILED;
	const MjcfArgs& mjcfArgs = clientCmd.m_mjcfArguments;
	if (m_data->m_verboseOutput)
	{
		b3Printf("Processed CMD_LOAD_MJCF:%s", mjcfArgs.m_mjcfFileName);
	}
	bool useMultiBody = (clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (mjcfArgs.m_useMultiBody != 0) : true;
	int flags = CUF_USE_MJCF;
	if (clientCmd.m_updateFlags & URDF_ARGS_HAS_CUSTOM_URDF_FLAGS)
	{
		flags |= clientCmd.m_mjcfArguments.m_flags;
	}

	bool completedOk = loadMjcf(mjcfArgs.m_mjcfFileName, bufferServerToClient, bufferSizeInBytes, useMultiBody, flags);
	if (completedOk)
	{
		m_data->m_guiHelper->autogenerateGraphicsObjects(this->m_data->m_dynamicsWorld);

		serverStatusOut.m_sdfLoadedArgs.m_numBodies = m_data->m_sdfRecentLoadedBodies.size();
		serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
		int maxBodies = btMin(MAX_SDF_BODIES, serverStatusOut.m_sdfLoadedArgs.m_numBodies);
		for (int i = 0; i < maxBodies; i++)
		{
			serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i] = m_data->m_sdfRecentLoadedBodies[i];
		}

		serverStatusOut.m_type = CMD_MJCF_LOADING_COMPLETED;
	}
	else
	{
		serverStatusOut.m_type = CMD_MJCF_LOADING_FAILED;
	}
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processSaveBulletCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_SAVE_BULLET");
	SharedMemoryStatus& serverCmd = serverStatusOut;

	FILE* f = fopen(clientCmd.m_fileArguments.m_fileName, "wb");
	if (f)
	{
		btDefaultSerializer* ser = new btDefaultSerializer();
		int currentFlags = ser->getSerializationFlags();
		ser->setSerializationFlags(currentFlags | BT_SERIALIZE_CONTACT_MANIFOLDS);

		m_data->m_dynamicsWorld->serialize(ser);
		fwrite(ser->getBufferPointer(), ser->getCurrentBufferSize(), 1, f);
		fclose(f);
		serverCmd.m_type = CMD_BULLET_SAVING_COMPLETED;
		delete ser;
		return hasStatus;
	}
	serverCmd.m_type = CMD_BULLET_SAVING_FAILED;
	return hasStatus;
}

bool PhysicsServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	//	BT_PROFILE("processCommand");

	int sz = sizeof(SharedMemoryStatus);
	int sz2 = sizeof(SharedMemoryCommand);

	bool hasStatus = false;

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
			hasStatus = processStateLoggingCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SET_VR_CAMERA_STATE:
		{
			hasStatus = processSetVRCameraStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_VR_EVENTS_DATA:
		{
			hasStatus = processRequestVREventsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_REQUEST_MOUSE_EVENTS_DATA:
		{
			hasStatus = processRequestMouseEventsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_REQUEST_KEYBOARD_EVENTS_DATA:
		{
			hasStatus = processRequestKeyboardEventsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};

		case CMD_REQUEST_RAY_CAST_INTERSECTIONS:
		{
			hasStatus = processRequestRaycastIntersectionsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_REQUEST_DEBUG_LINES:
		{
			hasStatus = processRequestDebugLinesCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_REQUEST_CAMERA_IMAGE_DATA:
		{
			hasStatus = processRequestCameraImageCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SYNC_BODY_INFO:
		{
			hasStatus = processSyncBodyInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_BODY_INFO:
		{
			hasStatus = processRequestBodyInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SAVE_WORLD:
		{
			hasStatus = processSaveWorldCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_LOAD_SDF:
		{
			hasStatus = processLoadSDFCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_COLLISION_SHAPE:
		{
			hasStatus = processCreateCollisionShapeCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_VISUAL_SHAPE:
		{
			hasStatus = processCreateVisualShapeCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_MULTI_BODY:
		{
			hasStatus = processCreateMultiBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SET_ADDITIONAL_SEARCH_PATH:
		{
			hasStatus = processSetAdditionalSearchPathCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_LOAD_URDF:
		{
			hasStatus = processLoadURDFCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_LOAD_SOFT_BODY:
		{
			hasStatus = processLoadSoftBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_SENSOR:
		{
			hasStatus = processCreateSensorCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_PROFILE_TIMING:
		{
			hasStatus = processProfileTimingCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_SEND_DESIRED_STATE:
		{
			hasStatus = processSendDesiredStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_COLLISION_INFO:
		{
			hasStatus = processRequestCollisionInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_REQUEST_ACTUAL_STATE:
		{
			hasStatus = processRequestActualStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_STEP_FORWARD_SIMULATION:
		{
			hasStatus = processForwardDynamicsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_REQUEST_INTERNAL_DATA:
		{
			hasStatus = processRequestInternalDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_CHANGE_DYNAMICS_INFO:
		{
			hasStatus = processChangeDynamicsInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_GET_DYNAMICS_INFO:
		{
			hasStatus = processGetDynamicsInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS:
		{
			hasStatus = processRequestPhysicsSimulationParametersCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
		{
			hasStatus = processSendPhysicsParametersCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};
		case CMD_INIT_POSE:
		{
			hasStatus = processInitPoseCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_RESET_SIMULATION:
		{
			hasStatus = processResetSimulationCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_RIGID_BODY:
		{
			hasStatus = processCreateRigidBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CREATE_BOX_COLLISION_SHAPE:
		{
			//for backward compatibility, CMD_CREATE_BOX_COLLISION_SHAPE is the same as CMD_CREATE_RIGID_BODY
			hasStatus = processCreateRigidBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_PICK_BODY:
		{
			hasStatus = processPickBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_MOVE_PICKED_BODY:
		{
			hasStatus = processMovePickedBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REMOVE_PICKING_CONSTRAINT_BODY:
		{
			hasStatus = processRemovePickingConstraintCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_AABB_OVERLAP:
		{
			hasStatus = processRequestAabbOverlapCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_OPENGL_VISUALIZER_CAMERA:
		{
			hasStatus = processRequestOpenGLVisualizeCameraCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CONFIGURE_OPENGL_VISUALIZER:
		{
			hasStatus = processConfigureOpenGLVisualizerCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_CONTACT_POINT_INFORMATION:
		{
			hasStatus = processRequestContactpointInformationCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CALCULATE_INVERSE_DYNAMICS:
		{
			hasStatus = processInverseDynamicsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CALCULATE_JACOBIAN:
		{
			hasStatus = processCalculateJacobianCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CALCULATE_MASS_MATRIX:
		{
			hasStatus = processCalculateMassMatrixCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_APPLY_EXTERNAL_FORCE:
		{
			hasStatus = processApplyExternalForceCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REMOVE_BODY:
		{
			hasStatus = processRemoveBodyCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_USER_CONSTRAINT:
		{
			hasStatus = processCreateUserConstraintCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CALCULATE_INVERSE_KINEMATICS:
		{
			hasStatus = processCalculateInverseKinematicsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_VISUAL_SHAPE_INFO:
		{
			hasStatus = processRequestVisualShapeInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_COLLISION_SHAPE_INFO:
		{
			hasStatus = processRequestCollisionShapeInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_UPDATE_VISUAL_SHAPE:
		{
			hasStatus = processUpdateVisualShapeCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CHANGE_TEXTURE:
		{
			hasStatus = processChangeTextureCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_LOAD_TEXTURE:
		{
			hasStatus = processLoadTextureCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_RESTORE_STATE:
		{
			hasStatus = processRestoreStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_SAVE_STATE:
		{
			hasStatus = processSaveStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_LOAD_BULLET:
		{
			hasStatus = processLoadBulletCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SAVE_BULLET:
		{
			hasStatus = processSaveBulletCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_LOAD_MJCF:
		{
			hasStatus = processLoadMJCFCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_USER_DEBUG_DRAW:
		{
			hasStatus = processUserDebugDrawCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CUSTOM_COMMAND:
		{
			hasStatus = processCustomCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SYNC_USER_DATA:
		{
			hasStatus = processSyncUserDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_USER_DATA:
		{
			hasStatus = processRequestUserDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_ADD_USER_DATA:
		{
			hasStatus = processAddUserDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REMOVE_USER_DATA:
		{
			hasStatus = processRemoveUserDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_COLLISION_FILTER:
		{
			hasStatus = processCollisionFilterCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		default:
		{
			BT_PROFILE("CMD_UNKNOWN");
			b3Error("Unknown command encountered");
			SharedMemoryStatus& serverCmd = serverStatusOut;
			serverCmd.m_type = CMD_UNKNOWN_COMMAND_FLUSHED;
			hasStatus = true;
		}
	};

	return hasStatus;
}

void PhysicsServerCommandProcessor::syncPhysicsToGraphics()
{
	m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
}

void PhysicsServerCommandProcessor::renderScene(int renderFlags)
{
	if (m_data->m_guiHelper)
	{
		if (0 == (renderFlags & COV_DISABLE_SYNC_RENDERING))
		{
			m_data->m_guiHelper->syncPhysicsToGraphics(m_data->m_dynamicsWorld);
		}

		m_data->m_guiHelper->render(m_data->m_dynamicsWorld);
	}
}

void PhysicsServerCommandProcessor::physicsDebugDraw(int debugDrawFlags)
{
	if (m_data->m_dynamicsWorld)
	{
		if (m_data->m_dynamicsWorld->getDebugDrawer())
		{
			m_data->m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
			m_data->m_dynamicsWorld->debugDrawWorld();

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
			for (int i = 0; i < m_data->m_dynamicsWorld->getSoftBodyArray().size(); i++)
			{
				btSoftBody* psb = (btSoftBody*)m_data->m_dynamicsWorld->getSoftBodyArray()[i];
				if (m_data->m_dynamicsWorld->getDebugDrawer() && !(m_data->m_dynamicsWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
				{
					//btSoftBodyHelpers::DrawFrame(psb,m_data->m_dynamicsWorld->getDebugDrawer());
					btSoftBodyHelpers::Draw(psb, m_data->m_dynamicsWorld->getDebugDrawer(), m_data->m_dynamicsWorld->getDrawFlags());
				}
			}
#endif
		}
	}
}

struct MyResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
	MyResultCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld)
		: btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld)
	{
	}

	virtual bool needsCollision(btBroadphaseProxy* proxy0) const
	{
		return true;
	}
};

bool PhysicsServerCommandProcessor::pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
{
	if (m_data->m_dynamicsWorld == 0)
		return false;

	//btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);
	MyResultCallback rayCallback(rayFromWorld, rayToWorld);
	rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
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
		}
		else
		{
			btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
			if (multiCol && multiCol->m_multiBody)
			{
				m_data->m_prevCanSleep = multiCol->m_multiBody->getCanSleep();
				multiCol->m_multiBody->setCanSleep(false);

				btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

				btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody, multiCol->m_link, 0, pivotInA, pickPos);
				//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
				//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
				//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
				//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
				btScalar scaling = 10;
				p2p->setMaxAppliedImpulse(2 * scaling);

				btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
				world->addMultiBodyConstraint(p2p);
				m_data->m_pickingMultiBodyPoint2Point = p2p;
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
	if (m_data->m_pickedBody && m_data->m_pickedConstraint)
	{
		btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_data->m_pickedConstraint);
		if (pickCon)
		{
			//keep it at the same picking distance

			btVector3 dir = rayToWorld - rayFromWorld;
			dir.normalize();
			dir *= m_data->m_oldPickingDist;

			btVector3 newPivotB = rayFromWorld + dir;
			pickCon->setPivotB(newPivotB);
		}
	}

	if (m_data->m_pickingMultiBodyPoint2Point)
	{
		//keep it at the same picking distance

		btVector3 dir = rayToWorld - rayFromWorld;
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
		btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_data->m_dynamicsWorld;
		world->removeMultiBodyConstraint(m_data->m_pickingMultiBodyPoint2Point);
		delete m_data->m_pickingMultiBodyPoint2Point;
		m_data->m_pickingMultiBodyPoint2Point = 0;
	}
}

void PhysicsServerCommandProcessor::enableCommandLogging(bool enable, const char* fileName)
{
	if (enable)
	{
		if (0 == m_data->m_commandLogger)
		{
			m_data->m_commandLogger = new CommandLogger(fileName);
		}
	}
	else
	{
		if (0 != m_data->m_commandLogger)
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
	m_data->m_useRealTimeSimulation = enableRealTimeSim;
}

bool PhysicsServerCommandProcessor::isRealTimeSimulationEnabled() const
{
	return m_data->m_useRealTimeSimulation;
}

void PhysicsServerCommandProcessor::stepSimulationRealTime(double dtInSec, const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents)
{
	m_data->m_vrControllerEvents.addNewVREvents(vrControllerEvents, numVRControllerEvents);
	m_data->m_pluginManager.addEvents(vrControllerEvents, numVRControllerEvents, keyEvents, numKeyEvents, mouseEvents, numMouseEvents);

	for (int i = 0; i < m_data->m_stateLoggers.size(); i++)
	{
		if (m_data->m_stateLoggers[i]->m_loggingType == STATE_LOGGING_VR_CONTROLLERS)
		{
			VRControllerStateLogger* vrLogger = (VRControllerStateLogger*)m_data->m_stateLoggers[i];
			vrLogger->m_vrEvents.addNewVREvents(vrControllerEvents, numVRControllerEvents);
		}
	}

	for (int ii = 0; ii < numMouseEvents; ii++)
	{
		const b3MouseEvent& event = mouseEvents[ii];
		bool found = false;
		//search a matching one first, otherwise add new event
		for (int e = 0; e < m_data->m_mouseEvents.size(); e++)
		{
			if (event.m_eventType == m_data->m_mouseEvents[e].m_eventType)
			{
				if (event.m_eventType == MOUSE_MOVE_EVENT)
				{
					m_data->m_mouseEvents[e].m_mousePosX = event.m_mousePosX;
					m_data->m_mouseEvents[e].m_mousePosY = event.m_mousePosY;
					found = true;
				}
				else if ((event.m_eventType == MOUSE_BUTTON_EVENT) && event.m_buttonIndex == m_data->m_mouseEvents[e].m_buttonIndex)
				{
					m_data->m_mouseEvents[e].m_buttonState |= event.m_buttonState;
					if (event.m_buttonState & eButtonIsDown)
					{
						m_data->m_mouseEvents[e].m_buttonState |= eButtonIsDown;
					}
					else
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

	for (int i = 0; i < numKeyEvents; i++)
	{
		const b3KeyboardEvent& event = keyEvents[i];
		bool found = false;
		//search a matching one first, otherwise add new event
		for (int e = 0; e < m_data->m_keyboardEvents.size(); e++)
		{
			if (event.m_keyCode == m_data->m_keyboardEvents[e].m_keyCode)
			{
				m_data->m_keyboardEvents[e].m_keyState |= event.m_keyState;
				if (event.m_keyState & eButtonIsDown)
				{
					m_data->m_keyboardEvents[e].m_keyState |= eButtonIsDown;
				}
				else
				{
					m_data->m_keyboardEvents[e].m_keyState &= ~eButtonIsDown;
				}
				found = true;
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

			if (gVRTrackingObjectUniqueId >= 0)
			{
				gVRTrackingObjectTr.setOrigin(bodyHandle->m_multiBody->getBaseWorldTransform().getOrigin());
				gVRTeleportPos1 = gVRTrackingObjectTr.getOrigin();
			}
			if (gVRTrackingObjectFlag & VR_CAMERA_TRACK_OBJECT_ORIENTATION)
			{
				gVRTrackingObjectTr.setBasis(bodyHandle->m_multiBody->getBaseWorldTransform().getBasis());
				gVRTeleportOrn = gVRTrackingObjectTr.getRotation();
			}
		}
	}

	if ((m_data->m_useRealTimeSimulation) && m_data->m_guiHelper)
	{
		int maxSteps = m_data->m_numSimulationSubSteps + 3;
		if (m_data->m_numSimulationSubSteps)
		{
			gSubStep = m_data->m_physicsDeltaTime / m_data->m_numSimulationSubSteps;
		}
		else
		{
			gSubStep = m_data->m_physicsDeltaTime;
		}

		int numSteps = m_data->m_dynamicsWorld->stepSimulation(dtInSec * simTimeScalingFactor, maxSteps, gSubStep);
		gDroppedSimulationSteps += numSteps > maxSteps ? numSteps - maxSteps : 0;

		if (numSteps)
		{
			gNumSteps = numSteps;
			gDtInSec = dtInSec;

			addTransformChangedNotifications();
		}
	}
}

b3Notification createTransformChangedNotification(int bodyUniqueId, int linkIndex, const btCollisionObject* colObj)
{
	b3Notification notification;
	notification.m_notificationType = TRANSFORM_CHANGED;
	notification.m_transformChangeArgs.m_bodyUniqueId = bodyUniqueId;
	notification.m_transformChangeArgs.m_linkIndex = linkIndex;

	const btTransform& tr = colObj->getWorldTransform();
	notification.m_transformChangeArgs.m_worldPosition[0] = tr.getOrigin()[0];
	notification.m_transformChangeArgs.m_worldPosition[1] = tr.getOrigin()[1];
	notification.m_transformChangeArgs.m_worldPosition[2] = tr.getOrigin()[2];

	notification.m_transformChangeArgs.m_worldRotation[0] = tr.getRotation()[0];
	notification.m_transformChangeArgs.m_worldRotation[1] = tr.getRotation()[1];
	notification.m_transformChangeArgs.m_worldRotation[2] = tr.getRotation()[2];
	notification.m_transformChangeArgs.m_worldRotation[3] = tr.getRotation()[3];

	const btVector3& scaling = colObj->getCollisionShape()->getLocalScaling();
	notification.m_transformChangeArgs.m_localScaling[0] = scaling[0];
	notification.m_transformChangeArgs.m_localScaling[1] = scaling[1];
	notification.m_transformChangeArgs.m_localScaling[2] = scaling[2];
	return notification;
}

void PhysicsServerCommandProcessor::addTransformChangedNotifications()
{
	b3Notification notification;
	notification.m_notificationType = SIMULATION_STEPPED;
	m_data->m_pluginManager.addNotification(notification);

	b3AlignedObjectArray<int> usedHandles;
	m_data->m_bodyHandles.getUsedHandles(usedHandles);
	for (int i = 0; i < usedHandles.size(); i++)
	{
		int bodyUniqueId = usedHandles[i];
		InternalBodyData* bodyData = m_data->m_bodyHandles.getHandle(bodyUniqueId);
		if (!bodyData)
		{
			continue;
		}
		if (bodyData->m_multiBody)
		{
			btMultiBody* mb = bodyData->m_multiBody;
			if (mb->getBaseCollider()->isActive())
			{
				m_data->m_pluginManager.addNotification(createTransformChangedNotification(bodyUniqueId, -1, mb->getBaseCollider()));
			}
			for (int linkIndex = 0; linkIndex < mb->getNumLinks(); linkIndex++)
			{
				if (mb->getLinkCollider(linkIndex)->isActive())
				{
					m_data->m_pluginManager.addNotification(createTransformChangedNotification(bodyUniqueId, linkIndex, mb->getLinkCollider(linkIndex)));
				}
			}
		}
		else if (bodyData->m_rigidBody && bodyData->m_rigidBody->isActive())
		{
			m_data->m_pluginManager.addNotification(createTransformChangedNotification(bodyUniqueId, -1, bodyData->m_rigidBody));
		}
	}
}

void PhysicsServerCommandProcessor::resetSimulation()
{
	//clean up all data

	m_data->m_cachedVUrdfisualShapes.clear();

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	if (m_data && m_data->m_dynamicsWorld)
	{
		m_data->m_dynamicsWorld->getWorldInfo().m_sparsesdf.Reset();
	}
#endif
	if (m_data && m_data->m_guiHelper)
	{
		m_data->m_guiHelper->removeAllGraphicsInstances();
		m_data->m_guiHelper->removeAllUserDebugItems();
	}
	if (m_data)
	{
		if (m_data->m_pluginManager.getRenderInterface())
		{
			m_data->m_pluginManager.getRenderInterface()->resetAll();
		}

		if (m_data->m_pluginManager.getCollisionInterface())
		{
			m_data->m_pluginManager.getCollisionInterface()->resetAll();
		}

		for (int i = 0; i < m_data->m_savedStates.size(); i++)
		{
			delete m_data->m_savedStates[i].m_bulletFile;
			delete m_data->m_savedStates[i].m_serializer;
		}
		m_data->m_savedStates.clear();
	}

	removePickingConstraint();

	deleteDynamicsWorld();
	createEmptyDynamicsWorld();

	m_data->m_bodyHandles.exitHandles();
	m_data->m_bodyHandles.initHandles();

	m_data->m_userCollisionShapeHandles.exitHandles();
	m_data->m_userCollisionShapeHandles.initHandles();

	m_data->m_userDataHandles.exitHandles();
	m_data->m_userDataHandles.initHandles();
	m_data->m_userDataHandleLookup.clear();

	b3Notification notification;
	notification.m_notificationType = SIMULATION_RESET;
	m_data->m_pluginManager.addNotification(notification);
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
