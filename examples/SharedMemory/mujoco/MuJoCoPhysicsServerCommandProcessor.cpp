#ifdef BT_ENABLE_MUJOCO
#include "MuJoCoPhysicsServerCommandProcessor.h"
#include "mujoco.h"
#include <stdio.h>
#include "../SharedMemoryCommands.h"
#include "LinearMath/btQuickprof.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "LinearMath/btMinMax.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../../Utils/b3ResourcePath.h"


struct MuJoCoPhysicsServerCommandProcessorInternalData
{
	bool m_isConnected;
	bool m_verboseOutput;
	double m_physicsDeltaTime;
	int m_numSimulationSubSteps;

	mjModel* m_mujocoModel;
	mjData* m_mujocoData;
	
	b3AlignedObjectArray<int> m_mjcfRecentLoadedBodies;
	MuJoCoPhysicsServerCommandProcessorInternalData()
		:m_isConnected(false),
		m_verboseOutput(false),
		m_mujocoModel(0),
		m_mujocoData(0),
		m_physicsDeltaTime(1./240.),
		m_numSimulationSubSteps(0)
	{
	}
};

MuJoCoPhysicsServerCommandProcessor::MuJoCoPhysicsServerCommandProcessor()
{
	m_data = new MuJoCoPhysicsServerCommandProcessorInternalData;
}
	
MuJoCoPhysicsServerCommandProcessor::~MuJoCoPhysicsServerCommandProcessor()
{
	delete m_data;

}

bool MuJoCoPhysicsServerCommandProcessor::connect()
{
	if (m_data->m_isConnected)
	{
		printf("already connected\n");
		return true;
	}

	printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    int result = mj_activate("mjkey.txt");
	if (result==1)
	{
		m_data->m_isConnected = true;
		return true;
	}

	return false;
}

void MuJoCoPhysicsServerCommandProcessor::resetSimulation()
{
	if (m_data->m_mujocoModel)
	{
		mj_deleteModel(m_data->m_mujocoModel);
		m_data->m_mujocoModel=0;
	}
	if (m_data->m_mujocoData)
	{
		mj_deleteData(m_data->m_mujocoData);
		m_data->m_mujocoData = 0;
	}
}

void MuJoCoPhysicsServerCommandProcessor::disconnect()
{
	resetSimulation();
	
	m_data->m_isConnected = false;
}

bool MuJoCoPhysicsServerCommandProcessor::isConnected() const
{
	return m_data->m_isConnected;
}

bool MuJoCoPhysicsServerCommandProcessor::processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
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
		hasStatus = processRequestInternalDataCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
		break;
	};

	case CMD_SYNC_BODY_INFO:
	{
		hasStatus = processSyncBodyInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
		break;
	}
	case CMD_SYNC_USER_DATA:
	{
		hasStatus = processSyncUserDataCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
		break;
	}
	case CMD_LOAD_MJCF:
	{
		hasStatus = processLoadMJCFCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
		break;
	}
	case CMD_REQUEST_BODY_INFO:
	{
		hasStatus = processRequestBodyInfoCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
		break;
	}
	case CMD_STEP_FORWARD_SIMULATION:
		{
			hasStatus = processForwardDynamicsCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

		case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
		{
			hasStatus = processSendPhysicsParametersCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;

		};

		case CMD_REQUEST_ACTUAL_STATE:
		{
			hasStatus = processRequestActualStateCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
	
		case CMD_RESET_SIMULATION:
		{
			hasStatus = processResetSimulationCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}

	default:
		{
			BT_PROFILE("CMD_UNKNOWN");
			printf("Unknown command encountered: %d",clientCmd.m_type);
			SharedMemoryStatus& serverCmd =serverStatusOut;
			serverCmd.m_type = CMD_UNKNOWN_COMMAND_FLUSHED;
			hasStatus = true;
		}

#if 0
	case CMD_STATE_LOGGING:
		{
			hasStatus = processStateLoggingCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
			break;
		}
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
	case CMD_LOAD_URDF:
		{
			hasStatus = processLoadURDFCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
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
	case CMD_CUSTOM_COMMAND:
		{
			hasStatus = processCustomCommand(clientCmd,serverStatusOut,bufferServerToClient, bufferSizeInBytes);
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

bool MuJoCoPhysicsServerCommandProcessor::processRequestInternalDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_INTERNAL_DATA");
	SharedMemoryStatus& serverCmd = serverStatusOut;
	serverCmd.m_type = CMD_REQUEST_INTERNAL_DATA_COMPLETED;
	serverCmd.m_numDataStreamBytes = 0;
	return hasStatus;
}



bool MuJoCoPhysicsServerCommandProcessor::processSyncBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_BODY_INFO");
	int actualNumBodies = 0;
	serverStatusOut.m_sdfLoadedArgs.m_numBodies = 0;
	serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
	serverStatusOut.m_type = CMD_SYNC_BODY_INFO_COMPLETED;
	return hasStatus;
}

bool MuJoCoPhysicsServerCommandProcessor::processSyncUserDataCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_SYNC_USER_DATA");
	int numIdentifiers = 0;
	serverStatusOut.m_syncUserDataArgs.m_numUserDataIdentifiers = numIdentifiers;
	serverStatusOut.m_type = CMD_SYNC_USER_DATA_COMPLETED;
	return hasStatus;
}

bool MuJoCoPhysicsServerCommandProcessor::processLoadMJCFCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_LOAD_MJCF");
	serverStatusOut.m_type = CMD_MJCF_LOADING_FAILED;
	const MjcfArgs& mjcfArgs = clientCmd.m_mjcfArguments;
    if (m_data->m_verboseOutput)
    {
        printf("Processed CMD_LOAD_MJCF:%s", mjcfArgs.m_mjcfFileName);
    }
    bool useMultiBody=(clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY) ? (mjcfArgs.m_useMultiBody!=0) : true;
	int flags = 0;
	if (clientCmd.m_updateFlags&URDF_ARGS_HAS_CUSTOM_URDF_FLAGS)
	{
		flags |= clientCmd.m_mjcfArguments.m_flags;
	}

	const char* fileName = mjcfArgs.m_mjcfFileName;

	if (strlen(fileName)>0)
	{
		char relativeFileName[1024];
		b3FileUtils fu;
		//bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
		bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024)>0);
		if (!fileFound){
			printf("MJCF file not found: %s\n", fileName);
		} else
		{
			int maxPathLen = 1024;
			char pathPrefix[1024];
			fu.extractPath(relativeFileName,pathPrefix,maxPathLen);
			
			{
			   char error[1000] = "could not load binary model";
				mjModel* mnew = 0;                 
				if( strlen(relativeFileName)>4 && !strcmp(relativeFileName+strlen(relativeFileName)-4, ".mjb") )
				{
					mnew = mj_loadModel(relativeFileName, 0);
				}
				else
				{
					mnew = mj_loadXML(relativeFileName, 0, error, 1000);
					if (mnew)
					{
						//replace old one for now
						if (m_data->m_mujocoModel)
						{
							mj_deleteModel(m_data->m_mujocoModel);
						}
						if (m_data->m_mujocoData)
						{
							mj_deleteData(m_data->m_mujocoData);
						}
						m_data->m_mujocoModel = mnew;
						m_data->m_mujocoData = mj_makeData(m_data->m_mujocoModel);
						mj_forward(m_data->m_mujocoModel, m_data->m_mujocoData);
					}
				}
				if( !mnew )
				{
					printf("%s\n", error);
				} else
				{
					int maxBodies = btMin(MAX_SDF_BODIES, mnew->nbody);

					serverStatusOut.m_sdfLoadedArgs.m_numBodies = maxBodies;
					for (int i=0;i<maxBodies;i++)
					{
						serverStatusOut.m_sdfLoadedArgs.m_bodyUniqueIds[i]=i;
					}
					serverStatusOut.m_sdfLoadedArgs.m_numUserConstraints = 0;
					serverStatusOut.m_type = CMD_MJCF_LOADING_COMPLETED;
				}
			}
	

		
		}

	}

	return hasStatus;
}



bool MuJoCoPhysicsServerCommandProcessor::processRequestBodyInfoCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_REQUEST_BODY_INFO");

	const SdfRequestInfoArgs& sdfInfoArgs = clientCmd.m_sdfRequestInfoArgs;
	//stream info into memory
	int streamSizeInBytes = 0;//createBodyInfoStream(sdfInfoArgs.m_bodyUniqueId, bufferServerToClient, bufferSizeInBytes);

	serverStatusOut.m_type = CMD_BODY_INFO_COMPLETED;
	serverStatusOut.m_dataStreamArguments.m_bodyUniqueId = sdfInfoArgs.m_bodyUniqueId;
	serverStatusOut.m_dataStreamArguments.m_bodyName[0] = 0;
						
	if (m_data->m_mujocoModel && sdfInfoArgs.m_bodyUniqueId>=0 && sdfInfoArgs.m_bodyUniqueId<m_data->m_mujocoModel->nbody)
	{
		const char* name = m_data->m_mujocoModel->names+m_data->m_mujocoModel->name_bodyadr[sdfInfoArgs.m_bodyUniqueId];
		strcpy(serverStatusOut.m_dataStreamArguments.m_bodyName,name);
	}

	serverStatusOut.m_numDataStreamBytes = streamSizeInBytes;
						
	return hasStatus;

}


bool MuJoCoPhysicsServerCommandProcessor::processForwardDynamicsCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_STEP_FORWARD_SIMULATION");
	
	if (m_data->m_mujocoModel)
	{
		if (m_data->m_verboseOutput)
		{
			b3Printf("Step simulation request");
			b3Printf("CMD_STEP_FORWARD_SIMULATION clientCmd = %d\n", clientCmd.m_sequenceNumber);
		}
    
		btScalar deltaTimeScaled = m_data->m_physicsDeltaTime;

		if (m_data->m_numSimulationSubSteps > 0)
		{
			for (int i=0;i<m_data->m_numSimulationSubSteps;i++)
			{
				m_data->m_mujocoModel->opt.timestep = m_data->m_physicsDeltaTime/m_data->m_numSimulationSubSteps;
				mj_step(m_data->m_mujocoModel,m_data->m_mujocoData);
				mj_forward(m_data->m_mujocoModel,m_data->m_mujocoData);
			}
		}
		else
		{
			m_data->m_mujocoModel->opt.timestep = m_data->m_physicsDeltaTime;
			mj_step(m_data->m_mujocoModel,m_data->m_mujocoData);
			mj_forward(m_data->m_mujocoModel,m_data->m_mujocoData);
		}
	}
	SharedMemoryStatus& serverCmd =serverStatusOut;
	serverCmd.m_type = CMD_STEP_FORWARD_SIMULATION_COMPLETED;
	return hasStatus;

}


bool MuJoCoPhysicsServerCommandProcessor::processSendPhysicsParametersCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;

	BT_PROFILE("CMD_SEND_PHYSICS_SIMULATION_PARAMETERS");

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_DELTA_TIME)
	{
		m_data->m_physicsDeltaTime = clientCmd.m_physSimParamArgs.m_deltaTime;
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

	if (clientCmd.m_updateFlags&SIM_PARAM_UPDATE_GRAVITY)
	{
		btVector3 grav(clientCmd.m_physSimParamArgs.m_gravityAcceleration[0],
						clientCmd.m_physSimParamArgs.m_gravityAcceleration[1],
						clientCmd.m_physSimParamArgs.m_gravityAcceleration[2]);
		this->m_data->m_dynamicsWorld->setGravity(grav);
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
		m_data->m_dynamicsWorld->getWorldInfo().m_gravity=grav;
		
#endif
		if (m_data->m_verboseOutput)
		{
			b3Printf("Updated Gravity: %f,%f,%f",grav[0],grav[1],grav[2]);
		}

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

	SharedMemoryStatus& serverCmd =serverStatusOut;
	serverCmd.m_type = CMD_CLIENT_COMMAND_COMPLETED;
	return hasStatus;
}

bool MuJoCoPhysicsServerCommandProcessor::processRequestActualStateCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
	
	if (m_data->m_mujocoModel)
	{
		BT_PROFILE("CMD_REQUEST_ACTUAL_STATE");
		if (m_data->m_verboseOutput)
		{
			b3Printf("Sending the actual state (Q,U)");
		}
		int bodyUniqueId = clientCmd.m_requestActualStateInformationCommandArgument.m_bodyUniqueId;

		if (bodyUniqueId>=0 && bodyUniqueId<m_data->m_mujocoModel->nbody)
		{
			SharedMemoryStatus& serverCmd = serverStatusOut;
			serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_COMPLETED;

			serverCmd.m_sendActualStateArgs.m_bodyUniqueId = bodyUniqueId;
			serverCmd.m_sendActualStateArgs.m_numLinks = 0;//todo body->m_multiBody->getNumLinks();

			int totalDegreeOfFreedomQ = 0;
			int totalDegreeOfFreedomU = 0;

			if (serverCmd.m_sendActualStateArgs.m_numLinks>= MAX_DEGREE_OF_FREEDOM)
			{
				serverStatusOut.m_type = CMD_ACTUAL_STATE_UPDATE_FAILED;
				hasStatus = true;
				return hasStatus;
			}

			bool computeForwardKinematics = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS)!=0);
			bool computeLinkVelocities = ((clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_LINKVELOCITY)!=0);

			if (computeForwardKinematics || computeLinkVelocities)
			{
				//todo:check this
				mj_forward(m_data->m_mujocoModel, m_data->m_mujocoData);
			}

			//always add the base, even for static (non-moving objects)
			//so that we can easily move the 'fixed' base when needed
			//do we don't use this conditional "if (!mb->hasFixedBase())"
			{
				int rootLink = 0;//todo check
				int type=(m_data->m_mujocoModel->jnt_type+m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[rootLink];
				//assume mjJNT_FREE?
				int qposAdr = (m_data->m_mujocoModel->jnt_qposadr+m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[rootLink];
				mjtNum* pos = m_data->m_mujocoData->xipos+bodyUniqueId*3;

				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[0] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[1] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[2] = 0;

				mjtNum* orn= m_data->m_mujocoData->xquat+bodyUniqueId*4;
				mjtNum* cvel=m_data->m_mujocoData->cvel+bodyUniqueId*6;

				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[3] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[4] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[5] = 0;
				serverCmd.m_sendActualStateArgs.m_rootLocalInertialFrame[6] = 1;

				//base position in world space, carthesian
				serverCmd.m_sendActualStateArgs.m_actualStateQ[0] = pos[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[1] = pos[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[2] = pos[2];

				//base orientation, quaternion x,y,z,w, in world space, carthesian
				serverCmd.m_sendActualStateArgs.m_actualStateQ[3] = orn[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[4] = orn[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[5] = orn[2];
				serverCmd.m_sendActualStateArgs.m_actualStateQ[6] = orn[3];
				totalDegreeOfFreedomQ +=7;//pos + quaternion

				//base linear velocity (in world space, carthesian)
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[0] = cvel[3];//mb->getBaseVel()[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[1] = cvel[4];//mb->getBaseVel()[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[2] = cvel[5];//mb->getBaseVel()[2];

				//base angular velocity (in world space, carthesian)
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[3] = cvel[0];//mb->getBaseOmega()[0];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[4] = cvel[1];//mb->getBaseOmega()[1];
				serverCmd.m_sendActualStateArgs.m_actualStateQdot[5] = cvel[2];//mb->getBaseOmega()[2];
				totalDegreeOfFreedomU += 6;//3 linear and 3 angular DOF
			}

			//btAlignedObjectArray<btVector3> omega;
			//btAlignedObjectArray<btVector3> linVel;
							
		
			int numLinks = m_data->m_mujocoModel->body_jntnum[bodyUniqueId];
			for (int l=0;l<numLinks ;l++)
			{
				int type=(m_data->m_mujocoModel->jnt_type+m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[l];
				//int type=(m_data->m_mujocoModel->jnt_type+m_data->m_mujocoModel->body_jntnum[bodyUniqueId])[l];

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


bool MuJoCoPhysicsServerCommandProcessor::processResetSimulationCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool hasStatus = true;
	BT_PROFILE("CMD_RESET_SIMULATION");
	
	resetSimulation();
	
					
	SharedMemoryStatus& serverCmd =serverStatusOut;
	serverCmd.m_type = CMD_RESET_SIMULATION_COMPLETED;
	return hasStatus;
}



bool MuJoCoPhysicsServerCommandProcessor::receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
{
	return false;
}

#endif //BT_ENABLE_MUJOCO