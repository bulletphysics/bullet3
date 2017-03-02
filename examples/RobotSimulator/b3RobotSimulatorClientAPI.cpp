#include "b3RobotSimulatorClientAPI.h"


//#include "SharedMemoryCommands.h"

#include "SharedMemory/PhysicsClientC_API.h"

#ifdef BT_ENABLE_ENET
#include "SharedMemory/PhysicsClientUDP_C_API.h"
#endif//PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "SharedMemory/PhysicsClientTCP_C_API.h"
#endif//PHYSICS_TCP

#include "SharedMemory/PhysicsDirectC_API.h"

#include "SharedMemory/SharedMemoryInProcessPhysicsC_API.h"

#include "SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

struct b3RobotSimulatorClientAPI_InternalData
{
	b3PhysicsClientHandle m_physicsClientHandle;

	b3RobotSimulatorClientAPI_InternalData()
		:m_physicsClientHandle(0)
	{
	}
};

b3RobotSimulatorClientAPI::b3RobotSimulatorClientAPI()
{
	m_data = new b3RobotSimulatorClientAPI_InternalData();
}

 b3RobotSimulatorClientAPI::~b3RobotSimulatorClientAPI()
{
	 delete m_data;
}


bool b3RobotSimulatorClientAPI::connect(int mode, const std::string& hostName, int portOrKey)
{
	if (m_data->m_physicsClientHandle)
	{
		b3Warning ("Already connected, disconnect first.");
		return false;
	}
	b3PhysicsClientHandle sm = 0;

	int udpPort = 1234;
	int tcpPort = 6667;
	int key = SHARED_MEMORY_KEY;
	bool connected = false;
	

	switch (mode) 
	{
		case eCONNECT_GUI: 
		{
		int argc = 0;
		char* argv[1] = {0};
	#ifdef __APPLE__
		sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
	#else
		sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
	#endif
		break;
		}
		case eCONNECT_DIRECT: {
		sm = b3ConnectPhysicsDirect();
		break;
		}
		case eCONNECT_SHARED_MEMORY: {
			if (portOrKey>=0)
			{
				key = portOrKey;
			}
		sm = b3ConnectSharedMemory(key);
		break;
		}
		case eCONNECT_UDP: 
	{
			if (portOrKey>=0)
			{
				udpPort = portOrKey;
			}
	#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName.c_str(), udpPort);
	#else
 		b3Warning("UDP is not enabled in this build");
	#endif //BT_ENABLE_ENET

			break;
	}
	case eCONNECT_TCP:
		{
			if (portOrKey>=0)
			{
				tcpPort = portOrKey;
			}
	#ifdef BT_ENABLE_CLSOCKET
            
			sm = b3ConnectPhysicsTCP(hostName.c_str(), tcpPort);
	#else
			b3Warning("TCP is not enabled in this pybullet build");
	#endif //BT_ENABLE_CLSOCKET
			break;
		}
            
            

		default: {
		b3Warning("connectPhysicsServer unexpected argument");
		}
	};

  if (sm)
  {
	  m_data->m_physicsClientHandle = sm;
	  if (!b3CanSubmitCommand(m_data->m_physicsClientHandle))
	  {
		  disconnect();
		  return false;
	  }
	  return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::isConnected() const
{
	return  (m_data->m_physicsClientHandle!=0);
}


void b3RobotSimulatorClientAPI::disconnect()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3DisconnectSharedMemory(m_data->m_physicsClientHandle);
	m_data->m_physicsClientHandle = 0;
}

void b3RobotSimulatorClientAPI::syncBodies()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}


	b3SharedMemoryCommandHandle	command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3InitSyncBodyInfoCommand(m_data->m_physicsClientHandle);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);

}

void b3RobotSimulatorClientAPI::resetSimulation()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
    b3SharedMemoryStatusHandle statusHandle;
    statusHandle = b3SubmitClientCommandAndWaitStatus(
    m_data->m_physicsClientHandle, b3InitResetSimulationCommand(m_data->m_physicsClientHandle));
}

bool b3RobotSimulatorClientAPI::canSubmitCommand() const
{
	if (!isConnected())
	{
		return false;
	}
	return (b3CanSubmitCommand(m_data->m_physicsClientHandle));    
}

void b3RobotSimulatorClientAPI::stepSimulation()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
    {
        statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, b3InitStepSimulationCommand(m_data->m_physicsClientHandle));
        statusType = b3GetStatusType(statusHandle);
        b3Assert(statusType==CMD_STEP_FORWARD_SIMULATION_COMPLETED);
    }
}

void b3RobotSimulatorClientAPI::setGravity(const b3Vector3& gravityAcceleration)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

   b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
    b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetGravity(command,  gravityAcceleration[0],gravityAcceleration[1],gravityAcceleration[2]);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);
}

b3Quaternion b3RobotSimulatorClientAPI::getQuaternionFromEuler(const b3Vector3& rollPitchYaw)
{
    double phi, the, psi;
    phi = rollPitchYaw[0] / 2.0;
    the = rollPitchYaw[1] / 2.0;
    psi = rollPitchYaw[2] / 2.0;
    double quat[4] = {
        sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
        cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
        cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
        cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi)};

    // normalize the quaternion
    double len = sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                    quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] /= len;
    quat[1] /= len;
    quat[2] /= len;
    quat[3] /= len;
	b3Quaternion q(quat[0],quat[1],quat[2],quat[3]);
	return q;
}

b3Vector3 b3RobotSimulatorClientAPI::getEulerFromQuaternion(const b3Quaternion& quat)
{
	 double squ;
  double sqx;
  double sqy;
  double sqz;
    double sarg;
  
    b3Vector3 rpy;
    sqx = quat[0] * quat[0];
    sqy = quat[1] * quat[1];
    sqz = quat[2] * quat[2];
    squ = quat[3] * quat[3];
    rpy[0] = atan2(2 * (quat[1] * quat[2] + quat[3] * quat[0]),
                   squ - sqx - sqy + sqz);
    sarg = -2 * (quat[0] * quat[2] - quat[3] * quat[1]);
    rpy[1] = sarg <= -1.0 ? -0.5 * 3.141592538
                          : (sarg >= 1.0 ? 0.5 * 3.141592538 : asin(sarg));
    rpy[2] = atan2(2 * (quat[0] * quat[1] + quat[3] * quat[2]),
                   squ + sqx - sqy - sqz);
	return rpy;
}

int b3RobotSimulatorClientAPI::loadURDF(const std::string& fileName, const struct b3RobotSimulatorLoadUrdfFileArgs& args)
{
	int robotUniqueId = -1;

	if (!isConnected())
	{
		b3Warning("Not connected");
		return robotUniqueId;
	}

	

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
			
	//setting the initial position, orientation and other arguments are optional
            
	b3LoadUrdfCommandSetStartPosition(command, args.m_startPosition[0],
													args.m_startPosition[1],
													args.m_startPosition[2]);
	b3LoadUrdfCommandSetStartOrientation(command,args.m_startOrientation[0]
											,args.m_startOrientation[1]
											,args.m_startOrientation[2]
											,args.m_startOrientation[3]);
	if (args.m_forceOverrideFixedBase)
	{
		b3LoadUrdfCommandSetUseFixedBase(command,true);
	}
	b3LoadUrdfCommandSetUseMultiBody(command, args.m_useMultiBody);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);

	b3Assert(statusType==CMD_URDF_LOADING_COMPLETED);
	if (statusType==CMD_URDF_LOADING_COMPLETED)
	{
		robotUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return robotUniqueId;
}

bool b3RobotSimulatorClientAPI::loadMJCF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;

	command = b3LoadMJCFCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_MJCF_LOADING_COMPLETED)
	{
		b3Warning("Couldn't load .mjcf file.");
		return false;
	}
	int numBodies = b3GetStatusBodyIndices(statusHandle, 0,0);
	if (numBodies)
	{
		results.m_uniqueObjectIds.resize(numBodies);
		int numBodies;
		numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0],results.m_uniqueObjectIds.size());
	}

	return true;
}

bool b3RobotSimulatorClientAPI::loadBullet(const std::string& fileName, b3RobotSimulatorLoadFileResults& results)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;

	command = b3LoadBulletCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_LOADING_COMPLETED)
	{
		return false;
	}
	int numBodies = b3GetStatusBodyIndices(statusHandle, 0,0);
	if (numBodies)
	{
		results.m_uniqueObjectIds.resize(numBodies);
		int numBodies;
		numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0],results.m_uniqueObjectIds.size());
	}

	return true;
}

bool b3RobotSimulatorClientAPI::loadSDF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results, const struct b3RobotSimulatorLoadSdfFileArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	bool statusOk = false;

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command = b3LoadSdfCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
    b3LoadSdfCommandSetUseMultiBody(command, args.m_useMultiBody);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	b3Assert(statusType == CMD_SDF_LOADING_COMPLETED);
	if (statusType == CMD_SDF_LOADING_COMPLETED)
	{
		int numBodies = b3GetStatusBodyIndices(statusHandle, 0,0);
		if (numBodies)
		{
			results.m_uniqueObjectIds.resize(numBodies);
			int numBodies;
			numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0],results.m_uniqueObjectIds.size());

		}
		statusOk = true;
	}

	return statusOk;
}


bool b3RobotSimulatorClientAPI::getBasePositionAndOrientation(int bodyUniqueId, b3Vector3& basePosition, b3Quaternion& baseOrientation) const
{
	b3SharedMemoryCommandHandle cmd_handle =
		b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
	b3SharedMemoryStatusHandle status_handle =
		b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, cmd_handle);

	const int status_type = b3GetStatusType(status_handle);
	const double* actualStateQ;

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
		return false;
	}

	b3GetStatusActualState(
		status_handle, 0 /* body_unique_id */,
		0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
		0 /*root_local_inertial_frame*/, &actualStateQ,
		0 /* actual_state_q_dot */, 0 /* joint_reaction_forces */);

	basePosition[0] = actualStateQ[0];
	basePosition[1] = actualStateQ[1];
	basePosition[2] = actualStateQ[2];

	baseOrientation[0] = actualStateQ[3];
	baseOrientation[1] = actualStateQ[4];
	baseOrientation[2] = actualStateQ[5];
	baseOrientation[3] = actualStateQ[6];
	return true;
}

bool b3RobotSimulatorClientAPI::resetBasePositionAndOrientation(int bodyUniqueId, b3Vector3& basePosition, b3Quaternion& baseOrientation)
{
	b3SharedMemoryCommandHandle commandHandle;

	commandHandle = b3CreatePoseCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

	b3CreatePoseCommandSetBasePosition(commandHandle, basePosition[0], basePosition[1], basePosition[2]);
	b3CreatePoseCommandSetBaseOrientation(commandHandle, baseOrientation[0], baseOrientation[1],
										baseOrientation[2], baseOrientation[3]);

	b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
  
	return true;
}


void b3RobotSimulatorClientAPI::setRealTimeSimulation(bool enableRealTimeSimulation)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;

	int ret = b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

int b3RobotSimulatorClientAPI::getNumJoints(int bodyUniqueId) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return b3GetNumJoints(m_data->m_physicsClientHandle,bodyUniqueId);
}


bool b3RobotSimulatorClientAPI::getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return (b3GetJointInfo(m_data->m_physicsClientHandle,bodyUniqueId, jointIndex,jointInfo)!=0);
}

void b3RobotSimulatorClientAPI::createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
    b3SharedMemoryStatusHandle statusHandle;
    b3Assert(b3CanSubmitCommand(m_data->m_physicsClientHandle));
    if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
    {
        statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, b3InitCreateUserConstraintCommand(m_data->m_physicsClientHandle, parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo));
    }
}

bool b3RobotSimulatorClientAPI::getJointState(int bodyUniqueId, int jointIndex, struct b3JointSensorState *state)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
    b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle,bodyUniqueId);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    int statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_ACTUAL_STATE_UPDATE_COMPLETED) 
	{
	  if (b3GetJointState(m_data->m_physicsClientHandle, statusHandle, jointIndex, state))
	  {
		  return true;
	  }
	}
    return false;
}

bool b3RobotSimulatorClientAPI::resetJointState(int bodyUniqueId, int jointIndex, double targetValue)
{
    b3SharedMemoryCommandHandle commandHandle;
    b3SharedMemoryStatusHandle statusHandle;
    int numJoints;

    numJoints = b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
    if ((jointIndex >= numJoints) || (jointIndex < 0)) {
    return false;
    }

    commandHandle = b3CreatePoseCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

    b3CreatePoseCommandSetJointPosition(m_data->m_physicsClientHandle, commandHandle, jointIndex,
                                        targetValue);

    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	return false;
}


void b3RobotSimulatorClientAPI::setJointMotorControl(int bodyUniqueId, int jointIndex, const b3RobotSimulatorJointMotorArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryStatusHandle statusHandle;
	switch (args.m_controlMode)
	{
		case CONTROL_MODE_VELOCITY:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_VELOCITY);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
            b3JointControlSetKd(command,uIndex,args.m_kd);
			b3JointControlSetDesiredVelocity(command,uIndex,args.m_targetVelocity);
			b3JointControlSetMaximumForce(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			break;
		}
		case CONTROL_MODE_POSITION_VELOCITY_PD:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_POSITION_VELOCITY_PD);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			int qIndex = jointInfo.m_qIndex;

			b3JointControlSetDesiredPosition(command,qIndex,args.m_targetPosition);
			b3JointControlSetKp(command,uIndex,args.m_kp);
			b3JointControlSetDesiredVelocity(command,uIndex,args.m_targetVelocity);
			b3JointControlSetKd(command,uIndex,args.m_kd);
			b3JointControlSetMaximumForce(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			break;
		}
		case CONTROL_MODE_TORQUE:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_TORQUE);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			b3JointControlSetDesiredForceTorque(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			break;
		}
		default:
		{
			b3Error("Unknown control command in b3RobotSimulationClientAPI::setJointMotorControl");
		}
	}
}


void b3RobotSimulatorClientAPI::setNumSolverIterations(int numIterations)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
    b3SharedMemoryStatusHandle statusHandle;
    b3PhysicsParamSetNumSolverIterations(command, numIterations);
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);

}

void b3RobotSimulatorClientAPI::setTimeStep(double timeStepInSeconds)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;
	int ret;
	ret = b3PhysicsParamSetTimeStep(command, timeStepInSeconds);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	
}


void b3RobotSimulatorClientAPI::setNumSimulationSubSteps(int numSubSteps)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
    b3SharedMemoryStatusHandle statusHandle;
    b3PhysicsParamSetNumSubSteps(command, numSubSteps);
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);
}


bool b3RobotSimulatorClientAPI::calculateInverseKinematics(const struct b3RobotSimulatorInverseKinematicArgs& args, struct b3RobotSimulatorInverseKinematicsResults& results)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	b3Assert(args.m_endEffectorLinkIndex>=0);
	b3Assert(args.m_bodyUniqueId>=0);
	

	b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(m_data->m_physicsClientHandle,args.m_bodyUniqueId);
	if ((args.m_flags & B3_HAS_IK_TARGET_ORIENTATION) && (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY))
    {
        b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
    } else if (args.m_flags & B3_HAS_IK_TARGET_ORIENTATION)
	{
		b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command,args.m_endEffectorLinkIndex,args.m_endEffectorTargetPosition,args.m_endEffectorTargetOrientation);
	} else if (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY)
    {
        b3CalculateInverseKinematicsPosWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
    } else
	{
		b3CalculateInverseKinematicsAddTargetPurePosition(command,args.m_endEffectorLinkIndex,args.m_endEffectorTargetPosition);
	}
    
    if (args.m_flags & B3_HAS_JOINT_DAMPING)
    {
        b3CalculateInverseKinematicsSetJointDamping(command, args.m_numDegreeOfFreedom, &args.m_jointDamping[0]);
    }

    b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	

    int numPos=0;
    
	bool result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
		&results.m_bodyUniqueId,
		&numPos,
		0)!=0;
    if (result && numPos)
    {
        results.m_calculatedJointPositions.resize(numPos);
        result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
                                                                 &results.m_bodyUniqueId,
                                                                 &numPos,
                                &results.m_calculatedJointPositions[0])!=0;
        
    }
	return result;
}



bool b3RobotSimulatorClientAPI::getBodyJacobian(int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositions, const double* jointVelocities, const double* jointAccelerations, double* linearJacobian, double* angularJacobian)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
    b3SharedMemoryCommandHandle command = b3CalculateJacobianCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, linkIndex, localPosition, jointPositions, jointVelocities, jointAccelerations);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    
    if (b3GetStatusType(statusHandle) == CMD_CALCULATED_JACOBIAN_COMPLETED)
    {
        b3GetStatusJacobian(statusHandle, linearJacobian, angularJacobian);
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI::getLinkState(int bodyUniqueId, int linkIndex, b3LinkState* linkState)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
    b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle,bodyUniqueId);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    
    if (b3GetStatusType(statusHandle) == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
    {
        b3GetLinkState(m_data->m_physicsClientHandle, statusHandle, linkIndex, linkState);
		return true;
    }
	return false;
}

void b3RobotSimulatorClientAPI::configureDebugVisualizer(b3ConfigureDebugVisualizerEnum flag, int enable)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryCommandHandle commandHandle = b3InitConfigureOpenGLVisualizer(m_data->m_physicsClientHandle);
	b3ConfigureOpenGLVisualizerSetVisualizationFlags(commandHandle, flag, enable);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
}

void b3RobotSimulatorClientAPI::getVREvents(b3VREventsData* vrEventsData)
{
	vrEventsData->m_numControllerEvents = 0;
	vrEventsData->m_controllerEvents = 0;
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle	commandHandle = b3RequestVREventsCommandInit(m_data->m_physicsClientHandle);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	b3GetVREventsData(m_data->m_physicsClientHandle, vrEventsData);	
}

void b3RobotSimulatorClientAPI::getKeyboardEvents(b3KeyboardEventsData* keyboardEventsData)
{
	keyboardEventsData->m_numKeyboardEvents = 0;
	keyboardEventsData->m_keyboardEvents = 0;
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle	commandHandle = b3RequestKeyboardEventsCommandInit(m_data->m_physicsClientHandle);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	b3GetKeyboardEventsData(m_data->m_physicsClientHandle,keyboardEventsData);

}


int b3RobotSimulatorClientAPI::startStateLogging(b3StateLoggingType loggingType, const std::string& fileName, const b3AlignedObjectArray<int>& objectUniqueIds)
{
	int loggingUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	commandHandle = b3StateLoggingCommandInit(m_data->m_physicsClientHandle);
		
	b3StateLoggingStart(commandHandle,loggingType,fileName.c_str());

	for ( int i=0;i<objectUniqueIds.size();i++)
	{
		int objectUid = objectUniqueIds[i];
		b3StateLoggingAddLoggingObjectUniqueId(commandHandle,objectUid);
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType==CMD_STATE_LOGGING_START_COMPLETED)
	{
		loggingUniqueId = b3GetStatusLoggingUniqueId(statusHandle);
	}
	return loggingUniqueId;
}

void b3RobotSimulatorClientAPI::stopStateLogging(int stateLoggerUniqueId)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	commandHandle = b3StateLoggingCommandInit(m_data->m_physicsClientHandle);
	b3StateLoggingStop(commandHandle, stateLoggerUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
}

