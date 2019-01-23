#ifdef BT_ENABLE_PHYSX
#include "PhysXServerCommandProcessor.h"

#include "../../Utils/ChromeTraceUtil.h"

#include <stdio.h>
#include "../SharedMemoryCommands.h"
#include "LinearMath/btQuickprof.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "LinearMath/btMinMax.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3ResizablePool.h"
#include "PxPhysicsAPI.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include "PhysXUrdfImporter.h"
#include "PxTolerancesScale.h"
#include "PxDefaultCpuDispatcher.h"
#include "PxDefaultSimulationFilterShader.h"
#include "URDF2PhysX.h"
#include "../b3PluginManager.h"

#define STATIC_EGLRENDERER_PLUGIN
#ifdef STATIC_EGLRENDERER_PLUGIN
#include "../plugins/eglPlugin/eglRendererPlugin.h"
#endif  //STATIC_EGLRENDERER_PLUGIN

//for serialization of data to client
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "../Extras/Serialize/BulletFileLoader/btBulletFile.h"
#include "LinearMath/btSerializer.h"
#include "PhysXUserData.h"

class MyPhysXErrorCallback : public physx::PxErrorCallback
{
public:
	MyPhysXErrorCallback()
	{
	}
	~MyPhysXErrorCallback()
	{
	}

	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line)
	{
		b3Printf("%s in file:%s line:%d\n", message, file, line);
	}
};

struct InternalPhysXBodyData
{
	physx::PxArticulationReducedCoordinate*		mArticulation;
	std::string m_bodyName;
	//physx::PxArticulationJointReducedCoordinate*	gDriveJoint;
	void clear()
	{
		m_bodyName = "";
	}
};



typedef b3PoolBodyHandle<InternalPhysXBodyData> InternalPhysXBodyHandle;


struct PhysXServerCommandProcessorInternalData
{
	bool m_isConnected;
	bool m_verboseOutput;
	double m_physicsDeltaTime;
	int m_numSimulationSubSteps;

	

	b3PluginManager m_pluginManager;


	physx::PxDefaultAllocator		m_allocator;
	MyPhysXErrorCallback	m_errorCallback;
	physx::PxFoundation*			m_foundation;
	physx::PxPhysics*				m_physics;
	physx::PxCooking*		m_cooking;
	physx::PxDefaultCpuDispatcher*	m_dispatcher;
	physx::PxScene*				m_scene;
	physx::PxMaterial*				m_material;
	//physx::PxPvd*                  m_pvd;

	b3ResizablePool<InternalPhysXBodyHandle> m_bodyHandles;
	



	b3AlignedObjectArray<int> m_mjcfRecentLoadedBodies;

	int m_profileTimingLoggingUid;
	int m_stateLoggersUniqueId;
	std::string m_profileTimingFileName;

	PhysXServerCommandProcessorInternalData(PhysXServerCommandProcessor* sdk)
		: m_isConnected(false),
		  m_verboseOutput(false),
		  m_physicsDeltaTime(1. / 240.),
		  m_numSimulationSubSteps(0),
		m_pluginManager(sdk),
		m_profileTimingLoggingUid(-1),
		m_stateLoggersUniqueId(1)
	{
		m_foundation = NULL;
		m_physics = NULL;
		m_cooking = NULL;
		m_dispatcher = NULL;
		m_scene = NULL;
		m_material = NULL;
		//m_pvd = NULL;

#ifdef STATIC_EGLRENDERER_PLUGIN
		{
			bool initPlugin = false;
			b3PluginFunctions funcs(initPlugin_eglRendererPlugin, exitPlugin_eglRendererPlugin, executePluginCommand_eglRendererPlugin);
			funcs.m_getRendererFunc = getRenderInterface_eglRendererPlugin;
			int renderPluginId = m_pluginManager.registerStaticLinkedPlugin("eglRendererPlugin", funcs, initPlugin);
			m_pluginManager.selectPluginRenderer(renderPluginId);
		}
#endif  //STATIC_EGLRENDERER_PLUGIN
	}
};

PhysXServerCommandProcessor::PhysXServerCommandProcessor()
{
	m_data = new PhysXServerCommandProcessorInternalData(this);
}

PhysXServerCommandProcessor::~PhysXServerCommandProcessor()
{
	delete m_data;
}



physx::PxFilterFlags MyPhysXFilter(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
	physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
	physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);
	if (filterData0.word2 != 0 && filterData0.word2 == filterData1.word2)
		return physx::PxFilterFlag::eKILL;
	pairFlags |= physx::PxPairFlag::eCONTACT_DEFAULT;
	return physx::PxFilterFlag::eDEFAULT;
}



bool PhysXServerCommandProcessor::connect()
{
	if (m_data->m_isConnected)
	{
		printf("already connected\n");
		return true;
	}

	int result = 0;
	{
		
		m_data->m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_data->m_allocator, m_data->m_errorCallback);
		m_data->m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_data->m_foundation, physx::PxTolerancesScale(), true, 0);
		m_data->m_cooking = PxCreateCooking(PX_PHYSICS_VERSION, *m_data->m_foundation, physx::PxCookingParams(physx::PxTolerancesScale()));

		physx::PxU32 numCores = 1;//
		m_data->m_dispatcher = physx::PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
		
		physx::PxSceneDesc sceneDesc(m_data->m_physics->getTolerancesScale());
		sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
		sceneDesc.solverType = physx::PxSolverType::eTGS;
		//sceneDesc.solverType = physx::PxSolverType::ePGS;
		sceneDesc.cpuDispatcher = m_data->m_dispatcher;
		//sceneDesc.filterShader = MyPhysXFilter;
		sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;


		m_data->m_scene = m_data->m_physics->createScene(sceneDesc);

		m_data->m_material = m_data->m_physics->createMaterial(0.5f, 0.5f, 0.f);

		PxInitExtensions(*m_data->m_physics, 0);


		//PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
		//gScene->addActor(*groundPlane);

		result = 1;
	}
	if (result == 1)
	{
		m_data->m_isConnected = true;
		return true;
	}

	return false;
}

void PhysXServerCommandProcessor::resetSimulation()
{
	//gArticulation->release();
	m_data->m_scene->release();
	m_data->m_dispatcher->release();
	m_data->m_cooking->release();
	m_data->m_physics->release();
	//PxPvdTransport* transport = gPvd->getTransport();
	//gPvd->release();
	//transport->release();
	PxCloseExtensions();
	
	m_data->m_foundation->release();
}

void PhysXServerCommandProcessor::disconnect()
{
	resetSimulation();

	m_data->m_isConnected = false;
}

bool PhysXServerCommandProcessor::isConnected() const
{
	return m_data->m_isConnected;
}


bool PhysXServerCommandProcessor::processCustomCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
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


bool PhysXServerCommandProcessor::processStateLoggingCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_STATE_LOGGING");

	serverStatusOut.m_type = CMD_STATE_LOGGING_FAILED;
	bool hasStatus = true;

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

	if ((clientCmd.m_updateFlags & STATE_LOGGING_STOP_LOG) && clientCmd.m_stateLoggingArguments.m_loggingUniqueId >= 0)
	{
		if (clientCmd.m_stateLoggingArguments.m_loggingUniqueId == m_data->m_profileTimingLoggingUid)
		{
			serverStatusOut.m_type = CMD_STATE_LOGGING_COMPLETED;
			b3ChromeUtilsStopTimingsAndWriteJsonFile(m_data->m_profileTimingFileName.c_str());
			m_data->m_profileTimingLoggingUid = -1;
		}
	}

#if 0
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
#endif
	return hasStatus;
}


bool PhysXServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	//	BT_PROFILE("processCommand");

	int sz = sizeof(SharedMemoryStatus);
	int sz2 = sizeof(SharedMemoryCommand);

	bool hasStatus = false;

	serverStatusOut.m_type = CMD_INVALID_STATUS;
	serverStatusOut.m_numDataStreamBytes = 0;
	serverStatusOut.m_dataStream = 0;

	//consume the command
	switch (clientCmd.m_type)
	{
		case CMD_REQUEST_INTERNAL_DATA:
		{
			hasStatus = processRequestInternalDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};

		case CMD_SYNC_BODY_INFO:
		{
			hasStatus = processSyncBodyInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_SYNC_USER_DATA:
		{
			hasStatus = processSyncUserDataCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_REQUEST_BODY_INFO:
		{
			hasStatus = processRequestBodyInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_STEP_FORWARD_SIMULATION:
		{
			hasStatus = processForwardDynamicsCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
		{
			hasStatus = processSendPhysicsParametersCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		};

		case CMD_REQUEST_ACTUAL_STATE:
		{
			hasStatus = processRequestActualStateCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_RESET_SIMULATION:
		{
			hasStatus = processResetSimulationCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}

		default:
		{
			BT_PROFILE("CMD_UNKNOWN");
			printf("Unknown command encountered: %d", clientCmd.m_type);
			SharedMemoryStatus& serverCmd = serverStatusOut;
			serverCmd.m_type = CMD_UNKNOWN_COMMAND_FLUSHED;
			hasStatus = true;
		}

		case CMD_LOAD_URDF:
		{
			hasStatus = processLoadURDFCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_CUSTOM_COMMAND:
		{
			hasStatus = processCustomCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}
		case CMD_STATE_LOGGING:
		{
			hasStatus = processStateLoggingCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
			break;
		}


#if 0
	case CMD_SET_VR_CAMERA_STATE:
		{
			hasStatus = processSetVRCameraStateCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_VR_EVENTS_DATA:
		{
			hasStatus = processRequestVREventsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		};
	case CMD_REQUEST_MOUSE_EVENTS_DATA:
		{
			hasStatus = processRequestMouseEventsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		};
	case CMD_REQUEST_KEYBOARD_EVENTS_DATA:
		{
			hasStatus = processRequestKeyboardEventsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		};

	case CMD_REQUEST_RAY_CAST_INTERSECTIONS:
		{

			hasStatus = processRequestRaycastIntersectionsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		};
	case CMD_REQUEST_DEBUG_LINES:
		{
			hasStatus = processRequestDebugLinesCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	case CMD_REQUEST_CAMERA_IMAGE_DATA:
		{
			hasStatus = processRequestCameraImageCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	
	case CMD_REQUEST_BODY_INFO:
		{
			hasStatus = processRequestBodyInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_SAVE_WORLD:
		{
			hasStatus = processSaveWorldCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_LOAD_SDF:
		{
			hasStatus = processLoadSDFCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CREATE_COLLISION_SHAPE:
		{
			hasStatus = processCreateCollisionShapeCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CREATE_VISUAL_SHAPE:
		{
			hasStatus = processCreateVisualShapeCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CREATE_MULTI_BODY:
		{
			hasStatus = processCreateMultiBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_SET_ADDITIONAL_SEARCH_PATH:
		{
			hasStatus = processSetAdditionalSearchPathCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	
	case CMD_LOAD_MJCF:
	{
		hasStatus = processLoadMJCFCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
		break;
	}

	case CMD_LOAD_SOFT_BODY:
		{
			hasStatus = processLoadSoftBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CREATE_SENSOR:
		{
			hasStatus = processCreateSensorCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_PROFILE_TIMING:
		{
			hasStatus = processProfileTimingCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	case CMD_SEND_DESIRED_STATE:
		{
			hasStatus = processSendDesiredStateCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_COLLISION_INFO:
		{
			hasStatus = processRequestCollisionInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	
	
	case CMD_CHANGE_DYNAMICS_INFO:
		{
			hasStatus = processChangeDynamicsInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		};
	case CMD_GET_DYNAMICS_INFO:
		{
			hasStatus = processGetDynamicsInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS:
		{
			hasStatus = processRequestPhysicsSimulationParametersCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	
	case CMD_INIT_POSE:
		{
			hasStatus = processInitPoseCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	
	case CMD_CREATE_RIGID_BODY:
		{
			hasStatus = processCreateRigidBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CREATE_BOX_COLLISION_SHAPE:
		{
			//for backward compatibility, CMD_CREATE_BOX_COLLISION_SHAPE is the same as CMD_CREATE_RIGID_BODY
			hasStatus = processCreateRigidBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_PICK_BODY:
		{
			hasStatus = processPickBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_MOVE_PICKED_BODY:
		{
			hasStatus = processMovePickedBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REMOVE_PICKING_CONSTRAINT_BODY:
		{
			hasStatus = processRemovePickingConstraintCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_AABB_OVERLAP:
		{
			hasStatus = processRequestAabbOverlapCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_OPENGL_VISUALIZER_CAMERA:
		{
			hasStatus = processRequestOpenGLVisualizeCameraCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CONFIGURE_OPENGL_VISUALIZER:
		{
			hasStatus = processConfigureOpenGLVisualizerCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_CONTACT_POINT_INFORMATION:
		{
			hasStatus = processRequestContactpointInformationCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CALCULATE_INVERSE_DYNAMICS:
		{
			hasStatus = processInverseDynamicsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CALCULATE_JACOBIAN:
		{
			hasStatus = processCalculateJacobianCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CALCULATE_MASS_MATRIX:
		{
			hasStatus = processCalculateMassMatrixCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_APPLY_EXTERNAL_FORCE:
		{
			hasStatus = processApplyExternalForceCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REMOVE_BODY:
		{
			hasStatus = processRemoveBodyCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_USER_CONSTRAINT:
		{
			hasStatus = processCreateUserConstraintCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CALCULATE_INVERSE_KINEMATICS:
		{
			hasStatus = processCalculateInverseKinematicsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_VISUAL_SHAPE_INFO:
		{
			hasStatus = processRequestVisualShapeInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REQUEST_COLLISION_SHAPE_INFO:
	{
		hasStatus = processRequestCollisionShapeInfoCommand(clientCmd, serverStatusOut, bufferServerToClient, bufferSizeInBytes);
		break;
	}
	case CMD_UPDATE_VISUAL_SHAPE:
		{
			hasStatus = processUpdateVisualShapeCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_CHANGE_TEXTURE:
		{
			hasStatus = processChangeTextureCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_LOAD_TEXTURE:
		{
			hasStatus = processLoadTextureCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
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
			hasStatus = processLoadBulletCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_SAVE_BULLET:
		{
			hasStatus = processSaveBulletCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_LOAD_MJCF:
		{
			hasStatus = processLoadMJCFCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_USER_DEBUG_DRAW:
		{
			hasStatus = processUserDebugDrawCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;        
		}
	
	case CMD_REQUEST_USER_DATA:
		{
			hasStatus = processRequestUserDataCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_ADD_USER_DATA:
		{
			hasStatus = processAddUserDataCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	case CMD_REMOVE_USER_DATA:
		{
			hasStatus = processRemoveUserDataCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
#endif
	};

	return hasStatus;
}

bool PhysXServerCommandProcessor::processRequestInternalDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_INTERNAL_DATA");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_REQUEST_INTERNAL_DATA_COMPLETED;
	serverCmd.m_numDataStreamBytes = 0;
	return hasStatus;
}

bool PhysXServerCommandProcessor::processSyncBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_BODY_INFO");
	int actualNumBodies = 0;
	serverStatusOut.m_sdfLoadedArgs.m_numBodies = 0;
	serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
	serverStatusOut.m_type = CMD_SYNC_BODY_INFO_COMPLETED;
	return hasStatus;
}

bool PhysXServerCommandProcessor::processSyncUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_USER_DATA");
	int numIdentifiers = 0;
	serverStatusOut.m_syncUserDataArgs.m_numUserDataIdentifiers = numIdentifiers;
	serverStatusOut.m_type = CMD_SYNC_USER_DATA_COMPLETED;
	return hasStatus;
}

struct MyPhysXURDFImporter : public PhysXURDFImporter
{
	b3PluginManager& m_pluginManager;
	
	MyPhysXURDFImporter(struct CommonFileIOInterface* fileIO, double globalScaling, int flags, b3PluginManager& pluginManager)
		:PhysXURDFImporter(fileIO, globalScaling, flags),
		m_pluginManager(pluginManager)
	{

	}

	int convertLinkVisualShapes3(
			int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame,
			const UrdfLink* linkPtr, const UrdfModel* model,
			int collisionObjectUniqueId, int bodyUniqueId, struct  CommonFileIOInterface* fileIO) const
	{

		if (m_pluginManager.getRenderInterface())
		{
			int graphicsUniqueId = m_pluginManager.getRenderInterface()->convertVisualShapes(linkIndex, pathPrefix, localInertiaFrame, linkPtr, model, collisionObjectUniqueId, bodyUniqueId, fileIO);
			return graphicsUniqueId;
		}
		return 0;
	}

};


bool PhysXServerCommandProcessor::processLoadURDFCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	BT_PROFILE("CMD_LOAD_URDF");
	serverStatusOut.m_type = CMD_URDF_LOADING_FAILED;
	serverStatusOut.m_numDataStreamBytes = 0;

	const UrdfArgs& urdfArgs = clientCmd.m_urdfArguments;

	btAssert(m_data->m_foundation);
	if (!m_data->m_foundation)
	{
		b3Error("loadUrdf: No valid m_dynamicsWorld");
		return false;
	}

	bool useMultiBody = (clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (urdfArgs.m_useMultiBody != 0) : true;
	bool useFixedBase = (clientCmd.m_updateFlags & URDF_ARGS_USE_FIXED_BASE) ? (urdfArgs.m_useFixedBase != 0) : false;

	btScalar globalScaling = 1.f;
	if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
	{
		globalScaling = urdfArgs.m_globalScaling;
	}

	b3BulletDefaultFileIO fileIO;
	
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


	MyPhysXURDFImporter u2p(&fileIO, globalScaling, urdfArgs.m_urdfFlags, m_data->m_pluginManager);
	
	

	bool loadOk = u2p.loadURDF(urdfArgs.m_urdfFileName, useFixedBase);


	if (loadOk)
	{
		

		for (int m = 0; m < u2p.getNumModels(); m++)
		{
			u2p.activateModel(m);

			btTransform rootTrans;
			rootTrans.setOrigin(initialPos);
			rootTrans.setRotation(initialOrn);
			u2p.setRootTransformInWorld(rootTrans);

			//get a body index
			int bodyUniqueId = m_data->m_bodyHandles.allocHandle();

			InternalPhysXBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);

			//sd.m_bodyUniqueIds.push_back(bodyUniqueId);

			
			
			u2p.setBodyUniqueId(bodyUniqueId);
			{
				btScalar mass = 0;
				//bodyHandle->m_rootLocalInertialFrame.setIdentity();
				bodyHandle->m_bodyName = u2p.getBodyName();
				btVector3 localInertiaDiagonal(0, 0, 0);
				int urdfLinkIndex = u2p.getRootLinkIndex();
				//u2p.getMassAndInertia2((urdfLinkIndex, mass, localInertiaDiagonal, bodyHandle->m_rootLocalInertialFrame, flags);
			}
			
			physx::PxArticulationReducedCoordinate* articulation = URDF2PhysX(m_data->m_foundation,m_data->m_physics, m_data->m_cooking, m_data->m_scene, u2p, urdfArgs.m_urdfFlags, u2p.getPathPrefix(), rootTrans, &fileIO);
			
			if (articulation)
			{
				bodyHandle->mArticulation = articulation;
				
				serverStatusOut.m_type = CMD_URDF_LOADING_COMPLETED;
				serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = bodyUniqueId;
				sprintf(serverStatusOut.m_dataStreamArguments.m_bodyName, "%s", bodyHandle->m_bodyName.c_str());

				btDefaultSerializer ser(bufferSizeInBytes, (unsigned char*)bufferServerToClient);

				ser.startSerialization();

				int len = sizeof(btMultiBodyData); 
				btChunk* chunk = ser.allocate(len, 1);


				btMultiBodyData *mbd = (btMultiBodyData *)chunk->m_oldPtr;
				btTransform rootTrans;
				u2p.getRootTransformInWorld(rootTrans);
				rootTrans.getOrigin().serialize(mbd->m_baseWorldPosition);
				rootTrans.getRotation().serialize(mbd->m_baseWorldOrientation);
				btVector3 zero(0, 0, 0);

				zero.serialize(mbd->m_baseLinearVelocity);
				zero.serialize(mbd->m_baseAngularVelocity);
				
				ser.registerNameForPointer(bodyHandle->m_bodyName.c_str(), bodyHandle->m_bodyName.c_str());
				{
					char *name = (char *)ser.findNameForPointer(bodyHandle->m_bodyName.c_str());
					mbd->m_baseName = (char *)ser.getUniquePointer(name);
					if (mbd->m_baseName)
					{
						ser.serializeName(name);
					}
				}
				mbd->m_numLinks = articulation->getNbLinks()-1;

				if (mbd->m_numLinks)
				{
					int sz = sizeof(btMultiBodyLinkData);
					int numElem = mbd->m_numLinks;
					btChunk *chunk = ser.allocate(sz, numElem);

					physx::PxArticulationLink* physxLinks[64];
					physx::PxU32 bufferSize = 64;
					physx::PxU32 startIndex = 0;
					int numLinks2 = articulation->getLinks(physxLinks, bufferSize, startIndex);
					
					btMultiBodyLinkData *memPtr = (btMultiBodyLinkData *)chunk->m_oldPtr;
					for (int j = 0; j < numElem; j++, memPtr++)
					{
						int i = j + 1;
						
						
						memPtr->m_jointType = 0;//todo 
						memPtr->m_dofCount = physxLinks[i]->getInboundJointDof(); 
						memPtr->m_posVarCount = physxLinks[i]->getInboundJointDof(); //??

						physx::PxVec3 li = physxLinks[i]->getMassSpaceInertiaTensor();
						btVector3 localInertia(li[0], li[1], li[2]);
						localInertia.serialize(memPtr->m_linkInertia);

						memPtr->m_linkMass = physxLinks[i]->getMass();
						memPtr->m_parentIndex = i>0? physxLinks[i]->getInboundJoint()->getParentArticulationLink().getLinkIndex(): -1;
						memPtr->m_jointDamping = 0;//todophysxLinks[i]->getLinearDamping();//??
						memPtr->m_jointFriction = 0;//todo
						memPtr->m_jointLowerLimit = 0;//todogetLink(i).m_jointLowerLimit;
						memPtr->m_jointUpperLimit = 0;//todogetLink(i).m_jointUpperLimit;
						memPtr->m_jointMaxForce = 0;//todogetLink(i).m_jointMaxForce;
						memPtr->m_jointMaxVelocity = 0;//todogetLink(i).m_jointMaxVelocity;

						//getLink(i).m_eVector.serialize(memPtr->m_parentComToThisPivotOffset);
						//getLink(i).m_dVector.serialize(memPtr->m_thisPivotToThisComOffset);
						//getLink(i).m_zeroRotParentToThis.serialize(memPtr->m_zeroRotParentToThis);
						
						
						{
							char *name = (char *)ser.findNameForPointer(physxLinks[i]->getName());
							memPtr->m_linkName = (char *)ser.getUniquePointer(name);
							if (memPtr->m_linkName)
							{
								ser.serializeName(name);
							}
						}
						{
							char *name = (char *)ser.findNameForPointer(physxLinks[i]->getName());
							memPtr->m_jointName = (char *)ser.getUniquePointer(name);
							if (memPtr->m_jointName)
							{
								ser.serializeName(name);
							}
						}
						memPtr->m_linkCollider = (btCollisionObjectData *)ser.getUniquePointer(0);
					}
					ser.finalizeChunk(chunk, btMultiBodyLinkDataName, BT_ARRAY_CODE, (void *)articulation);
				}
				mbd->m_links = mbd->m_numLinks ? (btMultiBodyLinkData *)ser.getUniquePointer((void *)articulation) : 0;

				// Fill padding with zeros to appease msan.
#ifdef BT_USE_DOUBLE_PRECISION
				memset(mbd->m_padding, 0, sizeof(mbd->m_padding));
#endif

				const char* structType = btMultiBodyDataName;
				ser.finalizeChunk(chunk, structType, BT_MULTIBODY_CODE,0);
				int streamSizeInBytes = ser.getCurrentBufferSize();
				serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;
				
			}
		}
#if 0
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
#endif
		return true;
	}
	return false;
}

bool PhysXServerCommandProcessor::processRequestBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_BODY_INFO");

	const SdfRequestInfoArgs& sdfInfoArgs = clientCmd.m_sdfRequestInfoArgs;
	//stream info into memory
	int streamSizeInBytes = 0;  //createBodyInfoStream(sdfInfoArgs.m_bodyUniqueId, bufferServerToClient, bufferSizeInBytes);

	serverStatusOut.m_type = CMD_BODY_INFO_COMPLETED;

	
	serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;

	return hasStatus;
}

bool PhysXServerCommandProcessor::processForwardDynamicsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_STEP_FORWARD_SIMULATION");
	
	int numArt = m_data->m_scene->getNbArticulations();
	
	{
		B3_PROFILE("PhysX_simulate_fetchResults");
		m_data->m_scene->simulate(m_data->m_physicsDeltaTime);
		m_data->m_scene->fetchResults(true);
	}
	{
		B3_PROFILE("syncTransform");
		if (m_data->m_pluginManager.getRenderInterface())
		{

			//sync transforms...

			b3AlignedObjectArray<int> usedHandles;
			m_data->m_bodyHandles.getUsedHandles(usedHandles);

			for (int i = 0; i < usedHandles.size(); i++)
			{
				InternalPhysXBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(usedHandles[i]);
				physx::PxArticulationLink* physxLinks[64];
				physx::PxU32 bufferSize = 64;
				physx::PxU32 startIndex = 0;
				int numLinks2 = bodyHandle->mArticulation->getLinks(physxLinks, bufferSize, startIndex);
				
				for (int l = 0; l < numLinks2; l++)
				{
					MyPhysXUserData* ud = (MyPhysXUserData*)physxLinks[l]->userData;
					if (ud)
					{
						btTransform tr;
						tr.setIdentity();
						physx::PxTransform pt = physxLinks[l]->getGlobalPose();
						tr.setOrigin(btVector3(pt.p[0], pt.p[1], pt.p[2]));
						tr.setRotation(btQuaternion(pt.q.x, pt.q.y, pt.q.z, pt.q.w));
						btVector3 localScaling(1, 1, 1);//??
						m_data->m_pluginManager.getRenderInterface()->syncTransform(ud->m_graphicsUniqueId, tr, localScaling);
					}
				}
			}
		}
		
		{
			B3_PROFILE("render");
			//m_data->m_pluginManager.getRenderInterface()->render();
			unsigned char* pixelRGBA = 0;
			int numRequestedPixels = 0;
			float* depthBuffer = 0;
			int* segmentationMaskBuffer = 0;
			int startPixelIndex = 0;
			int width = 1024;
			int height = 768;
			int numPixelsCopied = 0;

			m_data->m_pluginManager.getRenderInterface()->copyCameraImageData(pixelRGBA, numRequestedPixels,
				depthBuffer, numRequestedPixels,
				segmentationMaskBuffer, numRequestedPixels,
				startPixelIndex, &width, &height, &numPixelsCopied);
		}
	}

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;
	return hasStatus;
}

bool PhysXServerCommandProcessor::processSendPhysicsParametersCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_SEND_PHYSICS_SIMULATION_PARAMETERS");

	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DELTA_TIME)
	{
		m_data->m_physicsDeltaTime = clientCmd.m_physSimParamArgs.m_deltaTime;
	}

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_GRAVITY)
	{
		btVector3 grav(clientCmd.m_physSimParamArgs.m_gravityAcceleration[0],
			clientCmd.m_physSimParamArgs.m_gravityAcceleration[1],
			clientCmd.m_physSimParamArgs.m_gravityAcceleration[2]);
		
		m_data->m_scene->setGravity(physx::PxVec3(grav[0], grav[1], grav[2]));

		if (m_data->m_verboseOutput)
		{
			b3Printf("Updated Gravity: %f,%f,%f", grav[0], grav[1], grav[2]);
		}

	}

#if 0
	if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_CONE_FRICTION)
	{
		if (clientCmd.m_physSimParamArgs.m_enableConeFriction)
		{
			m_data->m_dynamicsWorld->getSolverInfo().m_solverMode &=~SOLVER_DISABLE_IMPLICIT_CONE_FRICTION;
		} else
		{
			m_data->m_dynamicsWorld->getSolverInfo().m_solverMode |=SOLVER_DISABLE_IMPLICIT_CONE_FRICTION;
		}
	}
	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DETERMINISTIC_OVERLAPPING_PAIRS)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_deterministicOverlappingPairs = (clientCmd.m_physSimParamArgs.m_deterministicOverlappingPairs!=0);
	}

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_CCD_ALLOWED_PENETRATION)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = clientCmd.m_physSimParamArgs.m_allowedCcdPenetration;
	}
	
	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_JOINT_FEEDBACK_MODE)
	{
		gJointFeedbackInWorldSpace = (clientCmd.m_physSimParamArgs.m_jointFeedbackMode&JOINT_FEEDBACK_IN_WORLD_SPACE)!=0;
		gJointFeedbackInJointFrame = (clientCmd.m_physSimParamArgs.m_jointFeedbackMode&JOINT_FEEDBACK_IN_JOINT_FRAME)!=0;
	}

	
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_REAL_TIME_SIMULATION)
	{
		m_data->m_useRealTimeSimulation = (clientCmd.m_physSimParamArgs.m_useRealTimeSimulation!=0);
	}
					
	//see 
	if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS)
	{
		//these flags are for internal/temporary/easter-egg/experimental demo purposes, use at own risk
		gInternalSimFlags = clientCmd.m_physSimParamArgs.m_internalSimFlags;
	}

	
	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_numIterations = clientCmd.m_physSimParamArgs.m_numSolverIterations;
	}

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_SOLVER_RESIDULAL_THRESHOLD)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = clientCmd.m_physSimParamArgs.m_solverResidualThreshold;
	}
	

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD)
	{
		gContactBreakingThreshold = clientCmd.m_physSimParamArgs.m_contactBreakingThreshold;
	}
	
	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_CONTACT_SLOP)
	{
		m_data->m_dynamicsWorld->getSolverInfo().m_linearSlop = clientCmd.m_physSimParamArgs.m_contactSlop;
	}

	if (clientCmd.m_updateFlags&SIM_PARAM_ENABLE_SAT)
	{
		m_data->m_dynamicsWorld->getDispatchInfo().m_enableSatConvex = clientCmd.m_physSimParamArgs.m_enableSAT!=0;
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

    if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DEFAULT_GLOBAL_CFM)
    {
        m_data->m_dynamicsWorld->getSolverInfo().m_globalCfm = clientCmd.m_physSimParamArgs.m_defaultGlobalCFM;
    }

    if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DEFAULT_FRICTION_CFM)
    {
		m_data->m_dynamicsWorld->getSolverInfo().m_frictionCFM = clientCmd.m_physSimParamArgs.m_frictionCFM;
    }

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD)
    {
        m_data->m_dynamicsWorld->getSolverInfo().m_restitutionVelocityThreshold = clientCmd.m_physSimParamArgs.m_restitutionVelocityThreshold;
    }

					

	if (clientCmd.m_updateFlags&SIM_PARAM_ENABLE_FILE_CACHING)
    {
		b3EnableFileCaching(clientCmd.m_physSimParamArgs.m_enableFileCaching);
    }

#endif

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool PhysXServerCommandProcessor::processRequestActualStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;

	int bodyUniqueId = clientCmd.m_requestActualStateInformationCommandArgument.m_bodyUniqueId;
	InternalPhysXBodyHandle* bodyHandle = m_data->m_bodyHandles.getHandle(bodyUniqueId);

	if (bodyHandle->mArticulation)
	{
		BT_PROFILE("CMD_REQUEST_ACTUAL_STATE");
		if (m_data->m_verboseOutput)
		{
			b3Printf("Sending the actual state (Q,U)");
		}

		{
			SharedMemoryStatus& serverCmd = serverStatusOut;
			serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

			serverCmd.m_sendActualStateArgs.m_bodyUniqueId = bodyUniqueId;
			serverCmd.m_sendActualStateArgs.m_numLinks = bodyHandle->mArticulation->getNbLinks()-1; //skip base!

			int totalDegreeOfFreedomQ = 0;
			int totalDegreeOfFreedomU = 0;

			if (serverCmd.m_sendActualStateArgs.m_numLinks >= MAX_DEGREE_OF_FREEDOM)
			{
				serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
				hasStatus = true;
				return hasStatus;
			}

			bool computeForwardKinematics = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS) != 0);
			bool computeLinkVelocities = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_LINKVELOCITY) != 0);

			if (computeForwardKinematics || computeLinkVelocities)
			{
				//todo:check this
			}

			physx::PxArticulationLink* physxLinks[64];
			physx::PxU32 bufferSize = 64;
			physx::PxU32 startIndex = 0;
			int numLinks2 = bodyHandle->mArticulation->getLinks(physxLinks, bufferSize, startIndex);


			//always add the base, even for static (non-moving objects)
			//so that we can easily move the 'fixed' base when needed
			//do we don't use this conditional "if (!mb->hasFixedBase())"
			{
				int rootLink = 0;  //todo check
				
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[0] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[1] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[2] = 0;

				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[3] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[4] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[5] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[6] = 1;

				physx::PxArticulationLink* l = physxLinks[0];
				physx::PxVec3 pos = l->getGlobalPose().p;
				physx::PxQuat orn = l->getGlobalPose().q;
				//base position in world space, carthesian
				serverCmd.m_sendActualStateArgs.m_actualStateQ[0] = pos[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[1] = pos[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[2] = pos[2];

				//base orientation, quaternion x,y,z,w, in world space, carthesian
				serverCmd.m_sendActualStateArgs.m_actualStateQ[3] = orn.x;
				serverCmd.m_sendActualStateArgs.m_actualStateQ[4] = orn.y;
				serverCmd.m_sendActualStateArgs.m_actualStateQ[5] = orn.z;
				serverCmd.m_sendActualStateArgs.m_actualStateQ[6] = orn.w;
				totalDegreeOfFreedomQ += 7;  //pos + quaternion

				//base linear velocity (in world space, carthesian)
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = 0;//cvel[3];  //mb->getBaseVel()[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = 0;//cvel[4];  //mb->getBaseVel()[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = 0;//cvel[5];  //mb->getBaseVel()[2];

				//base angular velocity (in world space, carthesian)
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = 0;//cvel[0];  //mb->getBaseOmega()[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = 0;//cvel[1];  //mb->getBaseOmega()[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = 0;//cvel[2];  //mb->getBaseOmega()[2];
				totalDegreeOfFreedomU += 6;                                      //3 linear and 3 angular DOF
			}

			//btAlignedObjectArray<btVector3> omega;
			//btAlignedObjectArray<btVector3> linVel;

			int numLinks = 0;// m_data->m_mujocoModel->body_jntnum[bodyUniqueId];
			for (int l = 0; l < numLinks; l++)
			{
				//int type = (m_data->m_mujocoModel->jnt_type + m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[l];
				//int type=(m_data->m_mujocoModel->jnt_type+m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[l];

#if 0
				physx::PxArticulationCache* c = bodyHandle->mArticulation->createCache();
				if (c)
				{
					c->jointVelocity[0] = 1;
					bodyHandle->mArticulation->applyCache(*c, physx::PxArticulationCache::eVELOCITY);
					bodyHandle->mArticulation->releaseCache(*c);
				}
#endif
#if 0
				mjtNum* xpos = 
				for (int d=0;d<mb->getLink(l).m_posVarCount;d++)
				{
					serverCmd.m_sendActualStateArgs.m_actualStateQ[totalDegreeOfFreedomQ++] = 0;
				}
				for (int d=0;d<mb->getLink(l).m_dofCount;d++)
				{
					serverCmd.m_sendActualStateArgs.m_actualStateQdot[totalDegreeOfFreedomU++] = 0;
				}

				if (0 == mb->getLink(l).m_jointFeedback)
				{
					for (int d=0;d<6;d++)
					{
						serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+d]=0;
					}
				} else
				{

					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+0] = 0;
					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+1] = 0;
					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+2] = 0;

					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+3] = 0;
					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+4] = 0;
					serverCmd.m_sendActualStateArgs.m_jointReactionForces[l*6+5] = 0;
				}

				serverCmd.m_sendActualStateArgs.m_jointMotorForce[l] = 0;
#if 0
				if (supportsJointMotor(mb,l))
				{
					if (motor && m_data->m_physicsDeltaTime>btScalar(0))
					{
						serverCmd.m_sendActualStateArgs.m_jointMotorForce[l] = 0;
					}
				}
#endif
				//btVector3 linkLocalInertialOrigin = body->m_linkLocalInertialFrames[l].getOrigin();
				//btQuaternion linkLocalInertialRotation = body->m_linkLocalInertialFrames[l].getRotation();

				//btVector3 linkCOMOrigin =  mb->getLink(l).m_cachedWorldTransform.getOrigin();
				//btQuaternion linkCOMRotation =  mb->getLink(l).m_cachedWorldTransform.getRotation();

				serverCmd.m_sendActualStateArgs.m_linkState[l*7+0] = 0;//linkCOMOrigin.getX();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+1] = 0;//linkCOMOrigin.getY();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+2] = 0;//linkCOMOrigin.getZ();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+3] = 0;//linkCOMRotation.x();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+4] = 0;//linkCOMRotation.y();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+5] = 0;//linkCOMRotation.z();
				serverCmd.m_sendActualStateArgs.m_linkState[l*7+6] = 1;//linkCOMRotation.w();

#if 0
				btVector3 worldLinVel(0,0,0);
				btVector3 worldAngVel(0,0,0);
								
				if (computeLinkVelocities)
				{
					const btMatrix3x3& linkRotMat = mb->getLink(l).m_cachedWorldTransform.getBasis();
					worldLinVel = linkRotMat * linVel[l+1];
					worldAngVel = linkRotMat * omega[l+1];
				}
#endif
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+0] = 0;//worldLinVel[0];
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+1] = 0;//worldLinVel[1];
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+2] = 0;//worldLinVel[2];
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+3] = 0;//worldAngVel[0];
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+4] = 0;//worldAngVel[1];
				serverCmd.m_sendActualStateArgs.m_linkWorldVelocities[l*6+5] = 0;//worldAngVel[2];
								
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+0] = 0;//linkLocalInertialOrigin.getX();
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+1] = 0;//linkLocalInertialOrigin.getY();
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+2] = 0;//linkLocalInertialOrigin.getZ();

				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+3] = 0;//linkLocalInertialRotation.x();
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+4] = 0;//linkLocalInertialRotation.y();
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+5] = 0;//linkLocalInertialRotation.z();
				serverCmd.m_sendActualStateArgs.m_linkLocalInertialFrames[l*7+6] = 1;//linkLocalInertialRotation.w();
#endif
			}

			serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomQ = totalDegreeOfFreedomQ;
			serverCmd.m_sendActualStateArgs.m_numDegreeOfFreedomU = totalDegreeOfFreedomU;

			hasStatus = true;
		}
	}


	return hasStatus;
}

bool PhysXServerCommandProcessor::processResetSimulationCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_RESET_SIMULATION");

	resetSimulation();

	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
	return hasStatus;
}

bool PhysXServerCommandProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	return false;
}

#endif  //BT_ENABLE_PHYSX