#include "b3RobotSimulatorClientAPI.h"

//#include "SharedMemoryCommands.h"

#include "../SharedMemory/PhysicsClientC_API.h"

#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "../SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //PHYSICS_TCP

#include "../SharedMemory/PhysicsDirectC_API.h"

#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"


#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

struct b3RobotSimulatorClientAPI_InternalData
{
	b3PhysicsClientHandle m_physicsClientHandle;
	struct GUIHelperInterface* m_guiHelper;

	b3RobotSimulatorClientAPI_InternalData()
		: m_physicsClientHandle(0),
		m_guiHelper(0)
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

void b3RobotSimulatorClientAPI::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
	m_data->m_guiHelper = guiHelper;
}

void b3RobotSimulatorClientAPI::renderScene()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessRenderSceneInternal(m_data->m_physicsClientHandle);
	}
}

void b3RobotSimulatorClientAPI::debugDraw(int debugDrawMode)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessDebugDrawInternal(m_data->m_physicsClientHandle,debugDrawMode);
	}
}

bool	b3RobotSimulatorClientAPI::mouseMoveCallback(float x,float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseMoveCallback(m_data->m_physicsClientHandle, x,y);
	}
	return false;
}
bool	b3RobotSimulatorClientAPI::mouseButtonCallback(int button, int state, float x, float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseButtonCallback(m_data->m_physicsClientHandle, button,state,x,y);
	}
	return false;
}



bool b3RobotSimulatorClientAPI::connect(int mode, const std::string& hostName, int portOrKey)
{
	if (m_data->m_physicsClientHandle)
	{
		b3Warning("Already connected, disconnect first.");
		return false;
	}
	b3PhysicsClientHandle sm = 0;

	int udpPort = 1234;
	int tcpPort = 6667;
	int key = SHARED_MEMORY_KEY;
	bool connected = false;

	switch (mode)
	{
		case eCONNECT_EXISTING_EXAMPLE_BROWSER:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(m_data->m_guiHelper);
			break;
		}

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
		case eCONNECT_GUI_SERVER:
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
		case eCONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
			break;
		}
		case eCONNECT_SHARED_MEMORY:
		{
			if (portOrKey >= 0)
			{
				key = portOrKey;
			}
			sm = b3ConnectSharedMemory(key);
			break;
		}
		case eCONNECT_UDP:
		{
			if (portOrKey >= 0)
			{
				udpPort = portOrKey;
			}
#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName.c_str(), udpPort);
#else
			b3Warning("UDP is not enabled in this build");
#endif  //BT_ENABLE_ENET

			break;
		}
		case eCONNECT_TCP:
		{
			if (portOrKey >= 0)
			{
				tcpPort = portOrKey;
			}
#ifdef BT_ENABLE_CLSOCKET

			sm = b3ConnectPhysicsTCP(hostName.c_str(), tcpPort);
#else
			b3Warning("TCP is not enabled in this pybullet build");
#endif  //BT_ENABLE_CLSOCKET
			break;
		}

		default:
		{
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
	return (m_data->m_physicsClientHandle != 0);
}

void b3RobotSimulatorClientAPI::setTimeOut(double timeOutInSec)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SetTimeOut(m_data->m_physicsClientHandle,timeOutInSec);

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

	b3SharedMemoryCommandHandle command;
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
	return (b3CanSubmitCommand(m_data->m_physicsClientHandle) != 0);
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
		//b3Assert(statusType == CMD_STEP_FORWARD_SIMULATION_COMPLETED);
	}
}

void b3RobotSimulatorClientAPI::setGravity(const b3Vector3& gravityAcceleration)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3Assert(b3CanSubmitCommand(m_data->m_physicsClientHandle));

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetGravity(command, gravityAcceleration[0], gravityAcceleration[1], gravityAcceleration[2]);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
//	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

b3Quaternion b3RobotSimulatorClientAPI::getQuaternionFromEuler(const b3Vector3& rollPitchYaw)
{
	b3Quaternion q;
	q.setEulerZYX(rollPitchYaw[2],rollPitchYaw[1],rollPitchYaw[0]);
	return q;
}

b3Vector3 b3RobotSimulatorClientAPI::getEulerFromQuaternion(const b3Quaternion& quat)
{
	b3Scalar roll,pitch,yaw;
	quat.getEulerZYX(yaw,pitch,roll);
	b3Vector3 rpy2 = b3MakeVector3(roll,pitch,yaw);
	return rpy2;
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

	b3LoadUrdfCommandSetFlags(command,args.m_flags);

	b3LoadUrdfCommandSetStartPosition(command, args.m_startPosition[0],
									  args.m_startPosition[1],
									  args.m_startPosition[2]);
	b3LoadUrdfCommandSetStartOrientation(command, args.m_startOrientation[0], args.m_startOrientation[1], args.m_startOrientation[2], args.m_startOrientation[3]);
	if (args.m_forceOverrideFixedBase)
	{
		b3LoadUrdfCommandSetUseFixedBase(command, true);
	}
	b3LoadUrdfCommandSetUseMultiBody(command, args.m_useMultiBody);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);

	b3Assert(statusType == CMD_URDF_LOADING_COMPLETED);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
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
	int numBodies = b3GetStatusBodyIndices(statusHandle, 0, 0);
	if (numBodies)
	{
		results.m_uniqueObjectIds.resize(numBodies);
		int numBodies;
		numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0], results.m_uniqueObjectIds.size());
	}

	return true;
}

bool b3RobotSimulatorClientAPI::loadBullet(const std::string& fileName, b3RobotSimulatorLoadFileResults& results)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

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
	int numBodies = b3GetStatusBodyIndices(statusHandle, 0, 0);
	if (numBodies)
	{
		results.m_uniqueObjectIds.resize(numBodies);
		int numBodies;
		numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0], results.m_uniqueObjectIds.size());
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
		int numBodies = b3GetStatusBodyIndices(statusHandle, 0, 0);
		if (numBodies)
		{
			results.m_uniqueObjectIds.resize(numBodies);
			int numBodies;
			numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0], results.m_uniqueObjectIds.size());
		}
		statusOk = true;
	}

	return statusOk;
}

bool b3RobotSimulatorClientAPI::getBodyInfo(int bodyUniqueId, struct b3BodyInfo* bodyInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	int result = b3GetBodyInfo(m_data->m_physicsClientHandle, bodyUniqueId, bodyInfo);
	return (result != 0);
}

bool b3RobotSimulatorClientAPI::getBasePositionAndOrientation(int bodyUniqueId, b3Vector3& basePosition, b3Quaternion& baseOrientation) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle cmd_handle =
		b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
	b3SharedMemoryStatusHandle status_handle =
		b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, cmd_handle);

	const int status_type = b3GetStatusType(status_handle);
	const double* actualStateQ;

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
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
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle commandHandle;

	commandHandle = b3CreatePoseCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

	b3CreatePoseCommandSetBasePosition(commandHandle, basePosition[0], basePosition[1], basePosition[2]);
	b3CreatePoseCommandSetBaseOrientation(commandHandle, baseOrientation[0], baseOrientation[1],
										  baseOrientation[2], baseOrientation[3]);

	b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);

	return true;
}

bool b3RobotSimulatorClientAPI::getBaseVelocity(int bodyUniqueId, b3Vector3& baseLinearVelocity, b3Vector3& baseAngularVelocity) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	const int status_type = b3GetStatusType(statusHandle);
	const double* actualStateQdot;

	if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		return false;
	}

	b3GetStatusActualState(statusHandle, 0 /* body_unique_id */,
						   0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
						   0 /*root_local_inertial_frame*/, 0,
						   &actualStateQdot, 0 /* joint_reaction_forces */);

	baseLinearVelocity[0] = actualStateQdot[0];
	baseLinearVelocity[1] = actualStateQdot[1];
	baseLinearVelocity[2] = actualStateQdot[2];

	baseAngularVelocity[0] = actualStateQdot[3];
	baseAngularVelocity[1] = actualStateQdot[4];
	baseAngularVelocity[2] = actualStateQdot[5];
	return true;
}

bool b3RobotSimulatorClientAPI::resetBaseVelocity(int bodyUniqueId, const b3Vector3& linearVelocity, const b3Vector3& angularVelocity) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;

	commandHandle = b3CreatePoseCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

	b3Vector3DoubleData linVelDouble;
	linearVelocity.serializeDouble(linVelDouble);
	b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, linVelDouble.m_floats);

	b3Vector3DoubleData angVelDouble;
	angularVelocity.serializeDouble(angVelDouble);
	b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, angVelDouble.m_floats);

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	return true;
}

void b3RobotSimulatorClientAPI::setInternalSimFlags(int flags)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
		b3SharedMemoryStatusHandle statusHandle;
		b3PhysicsParamSetInternalSimFlags(command, flags);
		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	}
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
	return b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
}

bool b3RobotSimulatorClientAPI::getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return (b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, jointInfo) != 0);
}

int b3RobotSimulatorClientAPI::createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	b3SharedMemoryStatusHandle statusHandle;
	b3Assert(b3CanSubmitCommand(m_data->m_physicsClientHandle));
	if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
	{
		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, b3InitCreateUserConstraintCommand(m_data->m_physicsClientHandle, parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo));
		int statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_USER_CONSTRAINT_COMPLETED)
		{
			int userConstraintUid = b3GetStatusUserConstraintUniqueId(statusHandle);
			return userConstraintUid;
		}
	}
	return -1;
}

int b3RobotSimulatorClientAPI::changeConstraint(int constraintId, b3JointInfo* jointInfo)
{

	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	b3SharedMemoryCommandHandle commandHandle = b3InitChangeUserConstraintCommand(m_data->m_physicsClientHandle, constraintId);

	if (jointInfo->m_flags & eJointChangeMaxForce)
	{
		b3InitChangeUserConstraintSetMaxForce(commandHandle, jointInfo->m_jointMaxForce);
	}

	if (jointInfo->m_flags & eJointChangeChildFramePosition)
	{
		b3InitChangeUserConstraintSetPivotInB(commandHandle, &jointInfo->m_childFrame[0]);
	}
	if (jointInfo->m_flags & eJointChangeChildFrameOrientation)
	{
		b3InitChangeUserConstraintSetFrameInB(commandHandle, &jointInfo->m_childFrame[3]);
	}

	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	int statusType = b3GetStatusType(statusHandle);
	return statusType;
}

void b3RobotSimulatorClientAPI::removeConstraint(int constraintId)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryCommandHandle commandHandle = b3InitRemoveUserConstraintCommand(m_data->m_physicsClientHandle, constraintId);
	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	int statusType = b3GetStatusType(statusHandle);
}


bool b3RobotSimulatorClientAPI::getJointState(int bodyUniqueId, int jointIndex, struct b3JointSensorState* state)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	int statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		if (b3GetJointState(m_data->m_physicsClientHandle, statusHandle, jointIndex, state))
		{
			return true;
		}
	}
	return false;
}

bool b3RobotSimulatorClientAPI::getJointStates(int bodyUniqueId, b3JointStates2& state)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle,bodyUniqueId);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    
    if (statusHandle)
    {
     //   double rootInertialFrame[7];
        const double* rootLocalInertialFrame;
        const double* actualStateQ;
        const double* actualStateQdot;
        const double* jointReactionForces;
        
        int stat = b3GetStatusActualState(statusHandle,
                        &state.m_bodyUniqueId,
                        &state.m_numDegreeOfFreedomQ,
                        &state.m_numDegreeOfFreedomU,
                        &rootLocalInertialFrame,
                        &actualStateQ,
                        &actualStateQdot,
                        &jointReactionForces);
        if (stat)
        {
            state.m_actualStateQ.resize(state.m_numDegreeOfFreedomQ);
            state.m_actualStateQdot.resize(state.m_numDegreeOfFreedomU);

            for (int i=0;i<state.m_numDegreeOfFreedomQ;i++)
            {
                state.m_actualStateQ[i] = actualStateQ[i];
            }
            for (int i=0;i<state.m_numDegreeOfFreedomU;i++)
            {
                state.m_actualStateQdot[i] = actualStateQdot[i];
            }
            int numJoints = getNumJoints(bodyUniqueId);
            state.m_jointReactionForces.resize(6*numJoints);
            for (int i=0;i<numJoints*6;i++)
            {
                state.m_jointReactionForces[i] = jointReactionForces[i];
            }
            
            return true;
        }
        //rootInertialFrame,
          //              &state.m_actualStateQ[0],
            //            &state.m_actualStateQdot[0],
              //          &state.m_jointReactionForces[0]);
        
     
    }
    return false;
}

bool b3RobotSimulatorClientAPI::resetJointState(int bodyUniqueId, int jointIndex, double targetValue)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int numJoints;

	numJoints = b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
	if ((jointIndex >= numJoints) || (jointIndex < 0))
	{
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
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2(m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_VELOCITY);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			b3JointControlSetKd(command, uIndex, args.m_kd);
			b3JointControlSetDesiredVelocity(command, uIndex, args.m_targetVelocity);
			b3JointControlSetMaximumForce(command, uIndex, args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			break;
		}
		case CONTROL_MODE_POSITION_VELOCITY_PD:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2(m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_POSITION_VELOCITY_PD);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			int qIndex = jointInfo.m_qIndex;

			b3JointControlSetDesiredPosition(command, qIndex, args.m_targetPosition);
			b3JointControlSetKp(command, uIndex, args.m_kp);
			b3JointControlSetDesiredVelocity(command, uIndex, args.m_targetVelocity);
			b3JointControlSetKd(command, uIndex, args.m_kd);
			b3JointControlSetMaximumForce(command, uIndex, args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			break;
		}
		case CONTROL_MODE_TORQUE:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2(m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_TORQUE);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			b3JointControlSetDesiredForceTorque(command, uIndex, args.m_maxTorqueValue);
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
	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

void b3RobotSimulatorClientAPI::setContactBreakingThreshold(double threshold)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetContactBreakingThreshold(command,threshold);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
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
	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

bool b3RobotSimulatorClientAPI::calculateInverseKinematics(const struct b3RobotSimulatorInverseKinematicArgs& args, struct b3RobotSimulatorInverseKinematicsResults& results)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	b3Assert(args.m_endEffectorLinkIndex >= 0);
	b3Assert(args.m_bodyUniqueId >= 0);

	b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(m_data->m_physicsClientHandle, args.m_bodyUniqueId);
	if ((args.m_flags & B3_HAS_IK_TARGET_ORIENTATION) && (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY))
	{
		b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
	}
	else if (args.m_flags & B3_HAS_IK_TARGET_ORIENTATION)
	{
		b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation);
	}
	else if (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY)
	{
		b3CalculateInverseKinematicsPosWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
	}
	else
	{
		b3CalculateInverseKinematicsAddTargetPurePosition(command, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition);
	}

	if (args.m_flags & B3_HAS_JOINT_DAMPING)
	{
		b3CalculateInverseKinematicsSetJointDamping(command, args.m_numDegreeOfFreedom, &args.m_jointDamping[0]);
	}

	b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	int numPos = 0;

	bool result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
															 &results.m_bodyUniqueId,
															 &numPos,
															 0) != 0;
	if (result && numPos)
	{
		results.m_calculatedJointPositions.resize(numPos);
		result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
															&results.m_bodyUniqueId,
															&numPos,
															&results.m_calculatedJointPositions[0]) != 0;
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
		int dofCount;
		b3GetStatusJacobian(statusHandle, &dofCount, linearJacobian, angularJacobian);
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
	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
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

void b3RobotSimulatorClientAPI::getVREvents(b3VREventsData* vrEventsData,int deviceTypeFilter)
{
	vrEventsData->m_numControllerEvents = 0;
	vrEventsData->m_controllerEvents = 0;
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle commandHandle = b3RequestVREventsCommandInit(m_data->m_physicsClientHandle);
	b3VREventsSetDeviceTypeFilter(commandHandle, deviceTypeFilter);
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

	b3SharedMemoryCommandHandle commandHandle = b3RequestKeyboardEventsCommandInit(m_data->m_physicsClientHandle);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	b3GetKeyboardEventsData(m_data->m_physicsClientHandle, keyboardEventsData);
}

int b3RobotSimulatorClientAPI::startStateLogging(b3StateLoggingType loggingType, const std::string& fileName, const b3AlignedObjectArray<int>& objectUniqueIds, int maxLogDof)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	int loggingUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	commandHandle = b3StateLoggingCommandInit(m_data->m_physicsClientHandle);

	b3StateLoggingStart(commandHandle, loggingType, fileName.c_str());

	for (int i = 0; i < objectUniqueIds.size(); i++)
	{
		int objectUid = objectUniqueIds[i];
		b3StateLoggingAddLoggingObjectUniqueId(commandHandle, objectUid);
	}

	if (maxLogDof > 0)
	{
		b3StateLoggingSetMaxLogDof(commandHandle, maxLogDof);
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_STATE_LOGGING_START_COMPLETED)
	{
		loggingUniqueId = b3GetStatusLoggingUniqueId(statusHandle);
	}
	return loggingUniqueId;
}

void b3RobotSimulatorClientAPI::stopStateLogging(int stateLoggerUniqueId)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	commandHandle = b3StateLoggingCommandInit(m_data->m_physicsClientHandle);
	b3StateLoggingStop(commandHandle, stateLoggerUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
}

void b3RobotSimulatorClientAPI::resetDebugVisualizerCamera(double cameraDistance, double cameraPitch, double cameraYaw, const b3Vector3& targetPos)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle commandHandle = b3InitConfigureOpenGLVisualizer(m_data->m_physicsClientHandle);
	if (commandHandle)
	{
	if ((cameraDistance >= 0))
	{
		b3Vector3FloatData camTargetPos;
		targetPos.serializeFloat(camTargetPos);
		b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, camTargetPos.m_floats);			
	}
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	}
}

void b3RobotSimulatorClientAPI::submitProfileTiming(const std::string&  profileName, int durationInMicroSeconds)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle	commandHandle = b3ProfileTimingCommandInit(m_data->m_physicsClientHandle, profileName.c_str());
	if (durationInMicroSeconds>=0)
	{
		b3SetProfileTimingDuractionInMicroSeconds(commandHandle, durationInMicroSeconds);
	}
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
}

void b3RobotSimulatorClientAPI::loadBunny(double scale, double mass, double collisionMargin)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

    b3SharedMemoryCommandHandle command = b3LoadBunnyCommandInit(m_data->m_physicsClientHandle);
    b3LoadBunnySetScale(command, scale);
    b3LoadBunnySetMass(command, mass);
    b3LoadBunnySetCollisionMargin(command, collisionMargin);
    b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
}