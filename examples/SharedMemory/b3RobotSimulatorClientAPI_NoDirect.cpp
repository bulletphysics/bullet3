#include "b3RobotSimulatorClientAPI_NoDirect.h"

#include "PhysicsClientC_API.h"
#include "b3RobotSimulatorClientAPI_InternalData.h"

#include "SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

static void scalarToDouble3(btScalar a[3], double b[3])
{
	for (int i = 0; i < 3; i++)
	{
		b[i] = a[i];
	}
}

static void scalarToDouble4(btScalar a[4], double b[4])
{
	for (int i = 0; i < 4; i++)
	{
		b[i] = a[i];
	}
}

b3RobotSimulatorClientAPI_NoDirect::b3RobotSimulatorClientAPI_NoDirect()
{
	m_data = new b3RobotSimulatorClientAPI_InternalData();
}

b3RobotSimulatorClientAPI_NoDirect::~b3RobotSimulatorClientAPI_NoDirect()
{
	delete m_data;
}

bool b3RobotSimulatorClientAPI_NoDirect::isConnected() const
{
	return (m_data->m_physicsClientHandle != 0);
}

void b3RobotSimulatorClientAPI_NoDirect::setTimeOut(double timeOutInSec)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SetTimeOut(m_data->m_physicsClientHandle, timeOutInSec);
}

void b3RobotSimulatorClientAPI_NoDirect::disconnect()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
#ifndef BT_DISABLE_PHYSICS_DIRECT
	b3DisconnectSharedMemory(m_data->m_physicsClientHandle);
	m_data->m_physicsClientHandle = 0;
#endif
}

void b3RobotSimulatorClientAPI_NoDirect::syncBodies()
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

void b3RobotSimulatorClientAPI_NoDirect::resetSimulation()
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

void b3RobotSimulatorClientAPI_NoDirect::resetSimulation(int flag)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryStatusHandle statusHandle;
	b3SharedMemoryCommandHandle command = b3InitResetSimulationCommand(m_data->m_physicsClientHandle);
	b3InitResetSimulationSetFlags(command, flag);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
}

bool b3RobotSimulatorClientAPI_NoDirect::canSubmitCommand() const
{
	if (!isConnected())
	{
		return false;
	}
	return (b3CanSubmitCommand(m_data->m_physicsClientHandle) != 0);
}

void b3RobotSimulatorClientAPI_NoDirect::stepSimulation()
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
		//btAssert(statusType == CMD_STEP_FORWARD_SIMULATION_COMPLETED);
	}
}

void b3RobotSimulatorClientAPI_NoDirect::setGravity(const btVector3& gravityAcceleration)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	btAssert(b3CanSubmitCommand(m_data->m_physicsClientHandle));

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetGravity(command, gravityAcceleration[0], gravityAcceleration[1], gravityAcceleration[2]);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	//	btAssert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

btQuaternion b3RobotSimulatorClientAPI_NoDirect::getQuaternionFromEuler(const btVector3& rollPitchYaw)
{
	btQuaternion q;
	q.setEulerZYX(rollPitchYaw[2], rollPitchYaw[1], rollPitchYaw[0]);
	return q;
}

btVector3 b3RobotSimulatorClientAPI_NoDirect::getEulerFromQuaternion(const btQuaternion& quat)
{
	btScalar roll, pitch, yaw;
	quat.getEulerZYX(yaw, pitch, roll);
	btVector3 rpy2 = btVector3(roll, pitch, yaw);
	return rpy2;
}

int b3RobotSimulatorClientAPI_NoDirect::loadTexture(const std::string& fileName)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	btAssert(b3CanSubmitCommand(m_data->m_physicsClientHandle));

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	{
		commandHandle = b3InitLoadTexture(m_data->m_physicsClientHandle, fileName.c_str());

		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_LOAD_TEXTURE_COMPLETED)
		{
			return b3GetStatusTextureUniqueId(statusHandle);
		}
	}
	return -1;
}

bool b3RobotSimulatorClientAPI_NoDirect::changeVisualShape(const struct b3RobotSimulatorChangeVisualShapeArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	int objectUniqueId = args.m_objectUniqueId;
	int jointIndex = args.m_linkIndex;
	int shapeIndex = args.m_shapeIndex;
	int textureUniqueId = args.m_textureUniqueId;

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUpdateVisualShape2(m_data->m_physicsClientHandle, objectUniqueId, jointIndex, shapeIndex);

	if (textureUniqueId>=-1)
	{
		b3UpdateVisualShapeTexture(commandHandle, textureUniqueId);
	}

	if (args.m_hasSpecularColor)
	{
		double specularColor[3] = {args.m_specularColor[0], args.m_specularColor[1], args.m_specularColor[2]};
		b3UpdateVisualShapeSpecularColor(commandHandle, specularColor);
	}
	if (args.m_hasRgbaColor)
	{
		double rgbaColor[4] = {args.m_rgbaColor[0], args.m_rgbaColor[1], args.m_rgbaColor[2], args.m_rgbaColor[3]};
		b3UpdateVisualShapeRGBAColor(commandHandle, rgbaColor);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	return (statusType == CMD_VISUAL_SHAPE_UPDATE_COMPLETED);
}

int b3RobotSimulatorClientAPI_NoDirect::loadURDF(const std::string& fileName, const struct b3RobotSimulatorLoadUrdfFileArgs& args)
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

	b3LoadUrdfCommandSetFlags(command, args.m_flags);

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

	btAssert(statusType == CMD_URDF_LOADING_COMPLETED);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		robotUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return robotUniqueId;
}

bool b3RobotSimulatorClientAPI_NoDirect::loadMJCF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results)
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
	b3LoadMJCFCommandSetFlags(command, URDF_USE_IMPLICIT_CYLINDER);
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

bool b3RobotSimulatorClientAPI_NoDirect::savePythonWorld(const std::string& fileName)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;

	if (fileName.length() == 0)
	{
		return false;
	}

	command = b3SaveWorldCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_SAVE_WORLD_COMPLETED)
	{
		return false;
	}
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::saveBullet(const std::string& fileName)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;

	if (fileName.length() == 0)
	{
		return false;
	}

	command = b3SaveBulletCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_SAVING_COMPLETED)
	{
		return false;
	}
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::loadBullet(const std::string& fileName, b3RobotSimulatorLoadFileResults& results)
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

bool b3RobotSimulatorClientAPI_NoDirect::loadSDF(const std::string& fileName, b3RobotSimulatorLoadFileResults& results, const struct b3RobotSimulatorLoadSdfFileArgs& args)
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
	btAssert(statusType == CMD_SDF_LOADING_COMPLETED);
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

bool b3RobotSimulatorClientAPI_NoDirect::getBodyInfo(int bodyUniqueId, struct b3BodyInfo* bodyInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	int result = b3GetBodyInfo(m_data->m_physicsClientHandle, bodyUniqueId, bodyInfo);
	return (result != 0);
}

bool b3RobotSimulatorClientAPI_NoDirect::getBasePositionAndOrientation(int bodyUniqueId, btVector3& basePosition, btQuaternion& baseOrientation) const
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

bool b3RobotSimulatorClientAPI_NoDirect::resetBasePositionAndOrientation(int bodyUniqueId, const btVector3& basePosition, const btQuaternion& baseOrientation)
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

bool b3RobotSimulatorClientAPI_NoDirect::getBaseVelocity(int bodyUniqueId, btVector3& baseLinearVelocity, btVector3& baseAngularVelocity) const
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

bool b3RobotSimulatorClientAPI_NoDirect::resetBaseVelocity(int bodyUniqueId, const btVector3& linearVelocity, const btVector3& angularVelocity) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;

	commandHandle = b3CreatePoseCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

	btVector3DoubleData linVelDouble;
	linearVelocity.serializeDouble(linVelDouble);
	b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, linVelDouble.m_floats);

	btVector3DoubleData angVelDouble;
	angularVelocity.serializeDouble(angVelDouble);
	b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, angVelDouble.m_floats);

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	return true;
}

void b3RobotSimulatorClientAPI_NoDirect::setInternalSimFlags(int flags)
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

void b3RobotSimulatorClientAPI_NoDirect::setRealTimeSimulation(bool enableRealTimeSimulation)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;

	b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	btAssert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

int b3RobotSimulatorClientAPI_NoDirect::getNumJoints(int bodyUniqueId) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
}

bool b3RobotSimulatorClientAPI_NoDirect::getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return (b3GetJointInfo(m_data->m_physicsClientHandle, bodyUniqueId, jointIndex, jointInfo) != 0);
}

int b3RobotSimulatorClientAPI_NoDirect::createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	b3SharedMemoryStatusHandle statusHandle;
	btAssert(b3CanSubmitCommand(m_data->m_physicsClientHandle));
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

int b3RobotSimulatorClientAPI_NoDirect::changeConstraint(int constraintId, b3RobotUserConstraint* jointInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	b3SharedMemoryCommandHandle commandHandle = b3InitChangeUserConstraintCommand(m_data->m_physicsClientHandle, constraintId);

	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
	{
		b3InitChangeUserConstraintSetMaxForce(commandHandle, jointInfo->m_maxAppliedForce);
	}

	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
	{
		b3InitChangeUserConstraintSetGearRatio(commandHandle, jointInfo->m_gearRatio);
	}

	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_ERP)
	{
		b3InitChangeUserConstraintSetERP(commandHandle, jointInfo->m_erp);
	}
	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK)
	{
		b3InitChangeUserConstraintSetGearAuxLink(commandHandle, jointInfo->m_gearAuxLink);
	}

	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET)
	{
		b3InitChangeUserConstraintSetRelativePositionTarget(commandHandle, jointInfo->m_relativePositionTarget);
	}
	
	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_PIVOT_IN_B)
	{
		b3InitChangeUserConstraintSetPivotInB(commandHandle, &jointInfo->m_childFrame[0]);
	}
	if (jointInfo->m_userUpdateFlags & USER_CONSTRAINT_CHANGE_FRAME_ORN_IN_B)
	{
		b3InitChangeUserConstraintSetFrameInB(commandHandle, &jointInfo->m_childFrame[3]);
	}

	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	int statusType = b3GetStatusType(statusHandle);
	return statusType;
}

void b3RobotSimulatorClientAPI_NoDirect::removeConstraint(int constraintId)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryCommandHandle commandHandle = b3InitRemoveUserConstraintCommand(m_data->m_physicsClientHandle, constraintId);
	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	b3GetStatusType(statusHandle);
}

bool b3RobotSimulatorClientAPI_NoDirect::getConstraintInfo(int constraintUniqueId, struct b3UserConstraint& constraintInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (b3GetUserConstraintInfo(m_data->m_physicsClientHandle, constraintUniqueId, &constraintInfo))
	{
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getJointState(int bodyUniqueId, int jointIndex, struct b3JointSensorState* state)
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

bool b3RobotSimulatorClientAPI_NoDirect::getJointStates(int bodyUniqueId, b3JointStates2& state)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);
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

			for (int i = 0; i < state.m_numDegreeOfFreedomQ; i++)
			{
				state.m_actualStateQ[i] = actualStateQ[i];
			}
			for (int i = 0; i < state.m_numDegreeOfFreedomU; i++)
			{
				state.m_actualStateQdot[i] = actualStateQdot[i];
			}
			int numJoints = getNumJoints(bodyUniqueId);
			state.m_jointReactionForces.resize(6 * numJoints);
			for (int i = 0; i < numJoints * 6; i++)
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

bool b3RobotSimulatorClientAPI_NoDirect::resetJointState(int bodyUniqueId, int jointIndex, double targetValue)
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

void b3RobotSimulatorClientAPI_NoDirect::setJointMotorControl(int bodyUniqueId, int jointIndex, const b3RobotSimulatorJointMotorArgs& args)
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
			if (uIndex >= 0)
			{
				b3JointControlSetKd(command, uIndex, args.m_kd);
				b3JointControlSetDesiredVelocity(command, uIndex, args.m_targetVelocity);
				b3JointControlSetMaximumForce(command, uIndex, args.m_maxTorqueValue);
				statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			}
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
			if (uIndex >= 0)
			{
				b3JointControlSetDesiredForceTorque(command, uIndex, args.m_maxTorqueValue);
				statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
			}
			break;
		}
		case CONTROL_MODE_PD:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2(m_data->m_physicsClientHandle, bodyUniqueId, CONTROL_MODE_PD);
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
		default:
		{
			b3Error("Unknown control command in b3RobotSimulationClientAPI::setJointMotorControl");
		}
	}
}

void b3RobotSimulatorClientAPI_NoDirect::setNumSolverIterations(int numIterations)
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
	btAssert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

void b3RobotSimulatorClientAPI_NoDirect::setContactBreakingThreshold(double threshold)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClientHandle);
	b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetContactBreakingThreshold(command, threshold);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	btAssert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

void b3RobotSimulatorClientAPI_NoDirect::setTimeStep(double timeStepInSeconds)
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

void b3RobotSimulatorClientAPI_NoDirect::setNumSimulationSubSteps(int numSubSteps)
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
	btAssert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);
}

bool b3RobotSimulatorClientAPI_NoDirect::calculateInverseKinematics(const struct b3RobotSimulatorInverseKinematicArgs& args, struct b3RobotSimulatorInverseKinematicsResults& results)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	btAssert(args.m_endEffectorLinkIndex >= 0);
	btAssert(args.m_bodyUniqueId >= 0);

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

	if (args.m_flags & B3_HAS_CURRENT_POSITIONS)
	{
		b3CalculateInverseKinematicsSetCurrentPositions(command, args.m_numDegreeOfFreedom, &args.m_currentJointPositions[0]);
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

int b3RobotSimulatorClientAPI_NoDirect::computeDofCount(int bodyUniqueId) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return 0;
	}
	return b3ComputeDofCount(m_data->m_physicsClientHandle, bodyUniqueId);
}

int b3RobotSimulatorClientAPI_NoDirect::calculateMassMatrix(int bodyUniqueId, const double* jointPositions, int numJointPositions, double* massMatrix, int flags)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return 0;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle commandHandle =
		b3CalculateMassMatrixCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, jointPositions, numJointPositions);
	b3CalculateMassMatrixSetFlags(commandHandle, flags);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CALCULATED_MASS_MATRIX_COMPLETED)
	{
		int dofCount;
		b3GetStatusMassMatrix(m_data->m_physicsClientHandle, statusHandle, &dofCount, NULL);
		if (dofCount)
		{
			b3GetStatusMassMatrix(m_data->m_physicsClientHandle, statusHandle, NULL, massMatrix);
			return dofCount;
		}
	}
	
	return 0;
}

bool b3RobotSimulatorClientAPI_NoDirect::getBodyJacobian(int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositions, const double* jointVelocities, const double* jointAccelerations, double* linearJacobian, double* angularJacobian)
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

bool b3RobotSimulatorClientAPI_NoDirect::getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeForwardKinematics, b3LinkState* linkState)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

	if (computeLinkVelocity)
	{
		b3RequestActualStateCommandComputeLinkVelocity(command, computeLinkVelocity);
	}

	if (computeForwardKinematics)
	{
		b3RequestActualStateCommandComputeForwardKinematics(command, computeForwardKinematics);
	}

	b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);

	if (b3GetStatusType(statusHandle) == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
	{
		b3GetLinkState(m_data->m_physicsClientHandle, statusHandle, linkIndex, linkState);
		return true;
	}
	return false;
}

void b3RobotSimulatorClientAPI_NoDirect::configureDebugVisualizer(b3ConfigureDebugVisualizerEnum flag, int enable)
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

void b3RobotSimulatorClientAPI_NoDirect::getVREvents(b3VREventsData* vrEventsData, int deviceTypeFilter)
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

void b3RobotSimulatorClientAPI_NoDirect::getKeyboardEvents(b3KeyboardEventsData* keyboardEventsData)
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

int b3RobotSimulatorClientAPI_NoDirect::startStateLogging(b3StateLoggingType loggingType, const std::string& fileName, const btAlignedObjectArray<int>& objectUniqueIds, int maxLogDof)
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

void b3RobotSimulatorClientAPI_NoDirect::stopStateLogging(int stateLoggerUniqueId)
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

void b3RobotSimulatorClientAPI_NoDirect::resetDebugVisualizerCamera(double cameraDistance, double cameraPitch, double cameraYaw, const btVector3& targetPos)
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
			btVector3FloatData camTargetPos;
			targetPos.serializeFloat(camTargetPos);
			b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, camTargetPos.m_floats);
		}
		b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
	}
}

void b3RobotSimulatorClientAPI_NoDirect::submitProfileTiming(const std::string& profileName)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle commandHandle = b3ProfileTimingCommandInit(m_data->m_physicsClientHandle, profileName.c_str());

	if (profileName.length())
	{
		b3SetProfileTimingType(commandHandle, 0);
	}
	else
	{
		b3SetProfileTimingType(commandHandle, 1);
	}

	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);
}

void b3RobotSimulatorClientAPI_NoDirect::loadSoftBody(const std::string& fileName, const struct b3RobotSimulatorLoadSoftBodyArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3LoadSoftBodyCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	b3LoadSoftBodySetStartPosition(command, args.m_startPosition[0], args.m_startPosition[1], args.m_startPosition[2]);
	b3LoadSoftBodySetStartOrientation(command, args.m_startOrientation[0], args.m_startOrientation[1], args.m_startOrientation[2], args.m_startOrientation[3]);
	b3LoadSoftBodySetScale(command, args.m_scale);
	b3LoadSoftBodySetMass(command, args.m_mass);
	b3LoadSoftBodySetCollisionMargin(command, args.m_collisionMargin);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
}

void b3RobotSimulatorClientAPI_NoDirect::loadDeformableBody(const std::string& fileName, const struct b3RobotSimulatorLoadDeformableBodyArgs& args)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3LoadSoftBodyCommandInit(m_data->m_physicsClientHandle, fileName.c_str());
	b3LoadSoftBodySetStartPosition(command, args.m_startPosition[0], args.m_startPosition[1], args.m_startPosition[2]);
	b3LoadSoftBodySetStartOrientation(command, args.m_startOrientation[0], args.m_startOrientation[1], args.m_startOrientation[2], args.m_startOrientation[3]);
	b3LoadSoftBodySetScale(command, args.m_scale);
	b3LoadSoftBodySetMass(command, args.m_mass);
	b3LoadSoftBodySetCollisionMargin(command, args.m_collisionMargin);
	if (args.m_NeoHookeanMu > 0)
	{
		b3LoadSoftBodyAddNeoHookeanForce(command, args.m_NeoHookeanMu, args.m_NeoHookeanLambda, args.m_NeoHookeanDamping);
	}
	if (args.m_springElasticStiffness > 0)
	{
		b3LoadSoftBodyAddMassSpringForce(command, args.m_springElasticStiffness, args.m_springDampingStiffness);
	}
	b3LoadSoftBodySetSelfCollision(command, args.m_useSelfCollision);
	b3LoadSoftBodyUseFaceContact(command, args.m_useFaceContact);
	b3LoadSoftBodySetFrictionCoefficient(command, args.m_frictionCoeff);
	b3LoadSoftBodyUseBendingSprings(command, args.m_useBendingSprings, args.m_springBendingStiffness);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
}

void b3RobotSimulatorClientAPI_NoDirect::getMouseEvents(b3MouseEventsData* mouseEventsData)
{
	mouseEventsData->m_numMouseEvents = 0;
	mouseEventsData->m_mouseEvents = 0;
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}

	b3SharedMemoryCommandHandle command = b3RequestMouseEventsCommandInit(m_data->m_physicsClientHandle);
	b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
	b3GetMouseEventsData(m_data->m_physicsClientHandle, mouseEventsData);
}

bool b3RobotSimulatorClientAPI_NoDirect::getCameraImage(int width, int height, struct b3RobotSimulatorGetCameraImageArgs args, struct b3CameraImageData& imageData)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryCommandHandle command;

	command = b3InitRequestCameraImage(m_data->m_physicsClientHandle);

	b3RequestCameraImageSetPixelResolution(command, width, height);

	// Check and apply optional arguments
	if (args.m_viewMatrix && args.m_projectionMatrix)
	{
		b3RequestCameraImageSetCameraMatrices(command, args.m_viewMatrix, args.m_projectionMatrix);
	}

	if (args.m_lightDirection != NULL)
	{
		b3RequestCameraImageSetLightDirection(command, args.m_lightDirection);
	}

	if (args.m_lightColor != NULL)
	{
		b3RequestCameraImageSetLightColor(command, args.m_lightColor);
	}

	if (args.m_lightDistance >= 0)
	{
		b3RequestCameraImageSetLightDistance(command, args.m_lightDistance);
	}

	if (args.m_hasShadow >= 0)
	{
		b3RequestCameraImageSetShadow(command, args.m_hasShadow);
	}

	if (args.m_lightAmbientCoeff >= 0)
	{
		b3RequestCameraImageSetLightAmbientCoeff(command, args.m_lightAmbientCoeff);
	}

	if (args.m_lightDiffuseCoeff >= 0)
	{
		b3RequestCameraImageSetLightDiffuseCoeff(command, args.m_lightDiffuseCoeff);
	}

	if (args.m_lightSpecularCoeff >= 0)
	{
		b3RequestCameraImageSetLightSpecularCoeff(command, args.m_lightSpecularCoeff);
	}

	if (args.m_renderer >= 0)
	{
		b3RequestCameraImageSelectRenderer(command, args.m_renderer);
	}

	// Actually retrieve the image
	if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CAMERA_IMAGE_COMPLETED)
		{
			b3GetCameraImageData(m_data->m_physicsClientHandle, &imageData);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::calculateInverseDynamics(int bodyUniqueId, double* jointPositions, double* jointVelocities,
																  double* jointAccelerations, double* jointForcesOutput)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle commandHandle = b3CalculateInverseDynamicsCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, jointPositions,
																					  jointVelocities, jointAccelerations);

	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);

	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED)
	{
		int bodyUniqueId;
		int dofCount;

		b3GetStatusInverseDynamicsJointForces(statusHandle, &bodyUniqueId, &dofCount, 0);

		if (dofCount)
		{
			b3GetStatusInverseDynamicsJointForces(statusHandle, 0, 0, jointForcesOutput);
			return true;
		}
	}
	return false;
}

int b3RobotSimulatorClientAPI_NoDirect::getNumBodies() const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return b3GetNumBodies(m_data->m_physicsClientHandle);
}

int b3RobotSimulatorClientAPI_NoDirect::getBodyUniqueId(int bodyId) const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	return b3GetBodyUniqueId(m_data->m_physicsClientHandle, bodyId);
}

bool b3RobotSimulatorClientAPI_NoDirect::removeBody(int bodyUniqueId)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
	{
		statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, b3InitRemoveBodyCommand(m_data->m_physicsClientHandle, bodyUniqueId));
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_REMOVE_BODY_COMPLETED)
		{
			return true;
		}
		else
		{
			b3Warning("getDynamicsInfo did not complete");
			return false;
		}
	}
	b3Warning("removeBody could not submit command");
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getDynamicsInfo(int bodyUniqueId, int linkIndex, b3DynamicsInfo* dynamicsInfo)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;
	//  struct b3DynamicsInfo info;

	if (bodyUniqueId < 0)
	{
		b3Warning("getDynamicsInfo failed; invalid bodyUniqueId");
		return false;
	}
	if (linkIndex < -1)
	{
		b3Warning("getDynamicsInfo failed; invalid linkIndex");
		return false;
	}

	if (b3CanSubmitCommand(m_data->m_physicsClientHandle))
	{
		cmd_handle = b3GetDynamicsInfoCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, linkIndex);
		status_handle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, cmd_handle);
		status_type = b3GetStatusType(status_handle);
		if (status_type == CMD_GET_DYNAMICS_INFO_COMPLETED)
		{
			b3GetDynamicsInfo(status_handle, dynamicsInfo);
			return true;
		}
		else
		{
			b3Warning("getDynamicsInfo did not complete");
			return false;
		}
	}
	b3Warning("getDynamicsInfo could not submit command");
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::changeDynamics(int bodyUniqueId, int linkIndex, struct b3RobotSimulatorChangeDynamicsArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return false;
	}
	b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm);
	b3SharedMemoryStatusHandle statusHandle;

	if (args.m_activationState >= 0)
	{
		b3ChangeDynamicsInfoSetActivationState(command, bodyUniqueId, args.m_activationState);
	}
	if (args.m_mass >= 0)
	{
		b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, args.m_mass);
	}

	if (args.m_lateralFriction >= 0)
	{
		b3ChangeDynamicsInfoSetLateralFriction(command, bodyUniqueId, linkIndex, args.m_lateralFriction);
	}

	if (args.m_spinningFriction >= 0)
	{
		b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex, args.m_spinningFriction);
	}

	if (args.m_rollingFriction >= 0)
	{
		b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex, args.m_rollingFriction);
	}

	if (args.m_linearDamping >= 0)
	{
		b3ChangeDynamicsInfoSetLinearDamping(command, bodyUniqueId, args.m_linearDamping);
	}

	if (args.m_angularDamping >= 0)
	{
		b3ChangeDynamicsInfoSetAngularDamping(command, bodyUniqueId, args.m_angularDamping);
	}

	if (args.m_restitution >= 0)
	{
		b3ChangeDynamicsInfoSetRestitution(command, bodyUniqueId, linkIndex, args.m_restitution);
	}

	if (args.m_contactStiffness >= 0 && args.m_contactDamping >= 0)
	{
		b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command, bodyUniqueId, linkIndex, args.m_contactStiffness, args.m_contactDamping);
	}

	if (args.m_frictionAnchor >= 0)
	{
		b3ChangeDynamicsInfoSetFrictionAnchor(command, bodyUniqueId, linkIndex, args.m_frictionAnchor);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	return true;
}

int b3RobotSimulatorClientAPI_NoDirect::addUserDebugParameter(const char* paramName, double rangeMin, double rangeMax, double startValue)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return -1;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUserDebugAddParameter(sm, paramName, rangeMin, rangeMax, startValue);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
		return debugItemUniqueId;
	}
	b3Warning("addUserDebugParameter failed.");
	return -1;
}

double b3RobotSimulatorClientAPI_NoDirect::readUserDebugParameter(int itemUniqueId)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return 0;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUserDebugReadParameter(sm, itemUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED)
	{
		double paramValue = 0.f;
		int ok = b3GetStatusDebugParameterValue(statusHandle, &paramValue);
		if (ok)
		{
			return paramValue;
		}
	}
	b3Warning("readUserDebugParameter failed.");
	return 0;
}

bool b3RobotSimulatorClientAPI_NoDirect::removeUserDebugItem(int itemUniqueId)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return false;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUserDebugDrawRemove(sm, itemUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	return true;
}

int b3RobotSimulatorClientAPI_NoDirect::addUserDebugText(const char* text, double* textPosition, struct b3RobotSimulatorAddUserDebugTextArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return -1;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUserDebugDrawAddText3D(sm, text, textPosition, &args.m_colorRGB[0], args.m_size, args.m_lifeTime);

	if (args.m_parentObjectUniqueId >= 0)
	{
		b3UserDebugItemSetParentObject(commandHandle, args.m_parentObjectUniqueId, args.m_parentLinkIndex);
	}

	if (args.m_flags & DEBUG_TEXT_HAS_ORIENTATION)
	{
		b3UserDebugTextSetOrientation(commandHandle, &args.m_textOrientation[0]);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
		return debugItemUniqueId;
	}
	b3Warning("addUserDebugText3D failed.");
	return -1;
}

int b3RobotSimulatorClientAPI_NoDirect::addUserDebugText(const char* text, btVector3& textPosition, struct b3RobotSimulatorAddUserDebugTextArgs& args)
{
	double dposXYZ[3];
	dposXYZ[0] = textPosition.x();
	dposXYZ[1] = textPosition.y();
	dposXYZ[2] = textPosition.z();

	return addUserDebugText(text, &dposXYZ[0], args);
}

int b3RobotSimulatorClientAPI_NoDirect::addUserDebugLine(double* fromXYZ, double* toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return -1;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitUserDebugDrawAddLine3D(sm, fromXYZ, toXYZ, &args.m_colorRGB[0], args.m_lineWidth, args.m_lifeTime);

	if (args.m_parentObjectUniqueId >= 0)
	{
		b3UserDebugItemSetParentObject(commandHandle, args.m_parentObjectUniqueId, args.m_parentLinkIndex);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
		return debugItemUniqueId;
	}
	b3Warning("addUserDebugLine failed.");
	return -1;
}

int b3RobotSimulatorClientAPI_NoDirect::addUserDebugLine(btVector3& fromXYZ, btVector3& toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs& args)
{
	double dfromXYZ[3];
	double dtoXYZ[3];
	dfromXYZ[0] = fromXYZ.x();
	dfromXYZ[1] = fromXYZ.y();
	dfromXYZ[2] = fromXYZ.z();

	dtoXYZ[0] = toXYZ.x();
	dtoXYZ[1] = toXYZ.y();
	dtoXYZ[2] = toXYZ.z();
	return addUserDebugLine(&dfromXYZ[0], &dtoXYZ[0], args);
}

bool b3RobotSimulatorClientAPI_NoDirect::setJointMotorControlArray(int bodyUniqueId, struct b3RobotSimulatorJointMotorArrayArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected to physics server.");
		return false;
	}
	b3GetNumJoints(sm, bodyUniqueId);

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	//  int statusType;
	struct b3JointInfo info;

	commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, args.m_controlMode);

	for (int i = 0; i < args.m_numControlledDofs; i++)
	{
		double targetVelocity = 0.0;
		double targetPosition = 0.0;
		double force = 100000.0;
		double kp = 0.1;
		double kd = 1.0;
		int jointIndex;

		if (args.m_jointIndices)
		{
			jointIndex = args.m_jointIndices[i];
		}
		else
		{
			jointIndex = i;
		}

		if (args.m_targetVelocities)
		{
			targetVelocity = args.m_targetVelocities[i];
		}

		if (args.m_targetPositions)
		{
			targetPosition = args.m_targetPositions[i];
		}

		if (args.m_forces)
		{
			force = args.m_forces[i];
		}

		if (args.m_kps)
		{
			kp = args.m_kps[i];
		}

		if (args.m_kds)
		{
			kd = args.m_kds[i];
		}

		b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

		switch (args.m_controlMode)
		{
			case CONTROL_MODE_VELOCITY:
			{
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
				break;
			}

			case CONTROL_MODE_TORQUE:
			{
				b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex, force);
				break;
			}

			case CONTROL_MODE_POSITION_VELOCITY_PD:
			{
				b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex, targetPosition);
				b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
				break;
			}

			default:
			{
			}
		};
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::getPhysicsEngineParameters(struct b3RobotSimulatorSetPhysicsEngineParameters& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitRequestPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED)
		{
			return false;
		}
		b3GetStatusPhysicsSimulationParameters(statusHandle, &args);
	}
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::setPhysicsEngineParameter(const struct b3RobotSimulatorSetPhysicsEngineParameters& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;

	if (args.m_numSolverIterations >= 0)
	{
		b3PhysicsParamSetNumSolverIterations(command, args.m_numSolverIterations);
	}

	if (args.m_collisionFilterMode >= 0)
	{
		b3PhysicsParamSetCollisionFilterMode(command, args.m_collisionFilterMode);
	}

	if (args.m_numSimulationSubSteps >= 0)
	{
		b3PhysicsParamSetNumSubSteps(command, args.m_numSimulationSubSteps);
	}

	if (args.m_deltaTime >= 0)
	{
		b3PhysicsParamSetTimeStep(command, args.m_deltaTime);
	}

	if (args.m_useSplitImpulse >= 0)
	{
		b3PhysicsParamSetUseSplitImpulse(command, args.m_useSplitImpulse);
	}

	if (args.m_splitImpulsePenetrationThreshold >= 0)
	{
		b3PhysicsParamSetSplitImpulsePenetrationThreshold(command, args.m_splitImpulsePenetrationThreshold);
	}

	if (args.m_contactBreakingThreshold >= 0)
	{
		b3PhysicsParamSetContactBreakingThreshold(command, args.m_contactBreakingThreshold);
	}

	if (args.m_restitutionVelocityThreshold >= 0)
	{
		b3PhysicsParamSetRestitutionVelocityThreshold(command, args.m_restitutionVelocityThreshold);
	}

	if (args.m_enableFileCaching >= 0)
	{
		b3PhysicsParamSetEnableFileCaching(command, args.m_enableFileCaching);
	}

	if (args.m_defaultNonContactERP >= 0)
	{
		b3PhysicsParamSetDefaultNonContactERP(command, args.m_defaultNonContactERP);
	}

	if (args.m_defaultContactERP >= 0)
	{
		b3PhysicsParamSetDefaultContactERP(command, args.m_defaultContactERP);
	}

	if (args.m_frictionERP >= 0)
	{
		b3PhysicsParamSetDefaultFrictionERP(command, args.m_frictionERP);
	}

	if (args.m_solverResidualThreshold >= 0)
	{
		b3PhysicsParamSetSolverResidualThreshold(command, args.m_solverResidualThreshold);
	}

	if (args.m_constraintSolverType >= 0)
	{
		b3PhysicsParameterSetConstraintSolverType(command, args.m_constraintSolverType);
	}
	if (args.m_minimumSolverIslandSize >= 0)
	{
		b3PhysicsParameterSetMinimumSolverIslandSize(command, args.m_minimumSolverIslandSize);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::applyExternalForce(int objectUniqueId, int linkIndex, double* force, double* position, int flags)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;

	command = b3ApplyExternalForceCommandInit(sm);
	b3ApplyExternalForce(command, objectUniqueId, linkIndex, force, position, flags);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::applyExternalForce(int objectUniqueId, int linkIndex, btVector3& force, btVector3& position, int flags)
{
	double dforce[3];
	double dposition[3];

	dforce[0] = force.x();
	dforce[1] = force.y();
	dforce[2] = force.z();

	dposition[0] = position.x();
	dposition[1] = position.y();
	dposition[2] = position.z();

	return applyExternalForce(objectUniqueId, linkIndex, &dforce[0], &dposition[0], flags);
}

bool b3RobotSimulatorClientAPI_NoDirect::applyExternalTorque(int objectUniqueId, int linkIndex, double* torque, int flags)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;

	command = b3ApplyExternalForceCommandInit(sm);
	b3ApplyExternalTorque(command, objectUniqueId, linkIndex, torque, flags);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::applyExternalTorque(int objectUniqueId, int linkIndex, btVector3& torque, int flags)
{
	double dtorque[3];

	dtorque[0] = torque.x();
	dtorque[1] = torque.y();
	dtorque[2] = torque.z();

	return applyExternalTorque(objectUniqueId, linkIndex, &dtorque[0], flags);
}

bool b3RobotSimulatorClientAPI_NoDirect::enableJointForceTorqueSensor(int bodyUniqueId, int jointIndex, bool enable)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	int numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex < 0) || (jointIndex >= numJoints))
	{
		b3Warning("Error: invalid jointIndex.");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3CreateSensorCommandInit(sm, bodyUniqueId);
	b3CreateSensorEnable6DofJointForceTorqueSensor(command, jointIndex, enable);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CLIENT_COMMAND_COMPLETED)
	{
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getDebugVisualizerCamera(struct b3OpenGLVisualizerCameraInfo* cameraInfo)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3InitRequestOpenGLVisualizerCameraCommand(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusOpenGLVisualizerCamera(statusHandle, cameraInfo);

	if (statusType)
	{
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getContactPoints(struct b3RobotSimulatorGetContactPointsArgs& args, struct b3ContactInformation* contactInfo)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3InitRequestContactPointInformation(sm);

	if (args.m_bodyUniqueIdA >= 0)
	{
		b3SetContactFilterBodyA(command, args.m_bodyUniqueIdA);
	}
	if (args.m_bodyUniqueIdB >= 0)
	{
		b3SetContactFilterBodyB(command, args.m_bodyUniqueIdB);
	}
	if (args.m_linkIndexA >= -1)
	{
		b3SetContactFilterLinkA(command, args.m_linkIndexA);
	}
	if (args.m_linkIndexB >= -1)
	{
		b3SetContactFilterLinkB(command, args.m_linkIndexB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, contactInfo);
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getClosestPoints(struct b3RobotSimulatorGetContactPointsArgs& args, double distance, struct b3ContactInformation* contactInfo)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	command = b3InitClosestDistanceQuery(sm);

	b3SetClosestDistanceFilterBodyA(command, args.m_bodyUniqueIdA);
	b3SetClosestDistanceFilterBodyB(command, args.m_bodyUniqueIdB);
	b3SetClosestDistanceThreshold(command, distance);

	if (args.m_linkIndexA >= -1)
	{
		b3SetClosestDistanceFilterLinkA(command, args.m_linkIndexA);
	}
	if (args.m_linkIndexB >= -1)
	{
		b3SetClosestDistanceFilterLinkB(command, args.m_linkIndexB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, contactInfo);
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getOverlappingObjects(double* aabbMin, double* aabbMax, struct b3AABBOverlapData* overlapData)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	//  int statusType;

	command = b3InitAABBOverlapQuery(sm, aabbMin, aabbMax);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	b3GetAABBOverlapResults(sm, overlapData);

	return true;
}

bool b3RobotSimulatorClientAPI_NoDirect::getOverlappingObjects(btVector3& aabbMin, btVector3& aabbMax, struct b3AABBOverlapData* overlapData)
{
	double daabbMin[3];
	double daabbMax[3];

	daabbMin[0] = aabbMin.x();
	daabbMin[1] = aabbMin.y();
	daabbMin[2] = aabbMin.z();

	daabbMax[0] = aabbMax.x();
	daabbMax[1] = aabbMax.y();
	daabbMax[2] = aabbMax.z();

	return getOverlappingObjects(&daabbMin[0], &daabbMax[0], overlapData);
}

bool b3RobotSimulatorClientAPI_NoDirect::getAABB(int bodyUniqueId, int linkIndex, double* aabbMin, double* aabbMax)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	if (bodyUniqueId < 0)
	{
		b3Warning("Invalid bodyUniqueId");
		return false;
	}

	if (linkIndex < -1)
	{
		b3Warning("Invalid linkIndex");
		return false;
	}

	if (aabbMin == NULL || aabbMax == NULL)
	{
		b3Warning("Output AABB matrix is NULL");
		return false;
	}

	command = b3RequestCollisionInfoCommandInit(sm, bodyUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_REQUEST_COLLISION_INFO_COMPLETED)
	{
		return false;
	}
	if (b3GetStatusAABB(statusHandle, linkIndex, aabbMin, aabbMax))
	{
		return true;
	}
	return false;
}

bool b3RobotSimulatorClientAPI_NoDirect::getAABB(int bodyUniqueId, int linkIndex, btVector3& aabbMin, btVector3& aabbMax)
{
	double daabbMin[3];
	double daabbMax[3];

	bool status = getAABB(bodyUniqueId, linkIndex, &daabbMin[0], &daabbMax[0]);

	aabbMin[0] = (float)daabbMin[0];
	aabbMin[1] = (float)daabbMin[1];
	aabbMin[2] = (float)daabbMin[2];

	aabbMax[0] = (float)daabbMax[0];
	aabbMax[1] = (float)daabbMax[1];
	aabbMax[2] = (float)daabbMax[2];

	return status;
}


int b3RobotSimulatorClientAPI_NoDirect::createVisualShape(int shapeType, struct b3RobotSimulatorCreateVisualShapeArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int shapeIndex = -1;

	command = b3CreateVisualShapeCommandInit(sm);

	if (shapeType == GEOM_SPHERE && args.m_radius > 0)
	{
		shapeIndex = b3CreateVisualShapeAddSphere(command, args.m_radius);
	}
	if (shapeType == GEOM_BOX)
	{
		double halfExtents[3];
		scalarToDouble3(args.m_halfExtents, halfExtents);
		shapeIndex = b3CreateVisualShapeAddBox(command, halfExtents);
	}
	if (shapeType == GEOM_CAPSULE && args.m_radius > 0 && args.m_height >= 0)
	{
		shapeIndex = b3CreateVisualShapeAddCapsule(command, args.m_radius, args.m_height);
	}
	if (shapeType == GEOM_CYLINDER && args.m_radius > 0 && args.m_height >= 0)
	{
		shapeIndex = b3CreateVisualShapeAddCylinder(command, args.m_radius, args.m_height);
	}
	if (shapeType == GEOM_MESH && args.m_fileName)
	{
		double meshScale[3];
		scalarToDouble3(args.m_meshScale, meshScale);
		shapeIndex = b3CreateVisualShapeAddMesh(command, args.m_fileName, meshScale);
	}
	if (shapeType == GEOM_PLANE)
	{
		double planeConstant = 0;
		double planeNormal[3];
		scalarToDouble3(args.m_planeNormal, planeNormal);
		shapeIndex = b3CreateVisualShapeAddPlane(command, planeNormal, planeConstant);
	}
	if (shapeIndex >= 0 && args.m_flags)
	{
		b3CreateVisualSetFlag(command, shapeIndex, args.m_flags);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CREATE_VISUAL_SHAPE_COMPLETED)
	{
		int uid = b3GetStatusVisualShapeUniqueId(statusHandle);
		return uid;
	}
	return -1;
}

int b3RobotSimulatorClientAPI_NoDirect::createCollisionShape(int shapeType, struct b3RobotSimulatorCreateCollisionShapeArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int shapeIndex = -1;

	command = b3CreateCollisionShapeCommandInit(sm);

	if (shapeType == GEOM_SPHERE && args.m_radius > 0)
	{
		shapeIndex = b3CreateCollisionShapeAddSphere(command, args.m_radius);
	}
	if (shapeType == GEOM_BOX)
	{
		double halfExtents[3];
		scalarToDouble3(args.m_halfExtents, halfExtents);
		shapeIndex = b3CreateCollisionShapeAddBox(command, halfExtents);
	}
	if (shapeType == GEOM_CAPSULE && args.m_radius > 0 && args.m_height >= 0)
	{
		shapeIndex = b3CreateCollisionShapeAddCapsule(command, args.m_radius, args.m_height);
	}
	if (shapeType == GEOM_CYLINDER && args.m_radius > 0 && args.m_height >= 0)
	{
		shapeIndex = b3CreateCollisionShapeAddCylinder(command, args.m_radius, args.m_height);
	}
	if (shapeType == GEOM_MESH && args.m_fileName)
	{
		double meshScale[3];
		scalarToDouble3(args.m_meshScale, meshScale);
		shapeIndex = b3CreateCollisionShapeAddMesh(command, args.m_fileName, meshScale);
	}
	if (shapeType == GEOM_HEIGHTFIELD)
	{
		double meshScale[3];
		scalarToDouble3(args.m_meshScale, meshScale);
		if (args.m_fileName)
		{
			shapeIndex = b3CreateCollisionShapeAddHeightfield(command, args.m_fileName, meshScale, args.m_heightfieldTextureScaling);
		}
		else
		{
			if (args.m_heightfieldData.size() && args.m_numHeightfieldRows>0 && args.m_numHeightfieldColumns>0)
			{
				shapeIndex = b3CreateCollisionShapeAddHeightfield2(sm, command, meshScale, args.m_heightfieldTextureScaling,
					&args.m_heightfieldData[0],
					args.m_numHeightfieldRows,
					args.m_numHeightfieldColumns,
					args.m_replaceHeightfieldIndex);
			}
		}
		

	}

	if (shapeType == GEOM_PLANE)
	{
		double planeConstant = 0;
		double planeNormal[3];
		scalarToDouble3(args.m_planeNormal, planeNormal);
		shapeIndex = b3CreateCollisionShapeAddPlane(command, planeNormal, planeConstant);
	}
	if (shapeIndex >= 0 && args.m_flags)
	{
		b3CreateCollisionSetFlag(command, shapeIndex, args.m_flags);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
	{
		int uid = b3GetStatusCollisionShapeUniqueId(statusHandle);
		return uid;
	}
	return -1;
}

int b3RobotSimulatorClientAPI_NoDirect::createMultiBody(struct b3RobotSimulatorCreateMultiBodyArgs& args)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType, baseIndex;

	double doubleBasePosition[3];
	double doubleBaseInertialFramePosition[3];
	scalarToDouble3(args.m_basePosition.m_floats, doubleBasePosition);
	scalarToDouble3(args.m_baseInertialFramePosition.m_floats, doubleBaseInertialFramePosition);

	double doubleBaseOrientation[4];
	double doubleBaseInertialFrameOrientation[4];
	scalarToDouble4(args.m_baseOrientation, doubleBaseOrientation);
	scalarToDouble4(args.m_baseInertialFrameOrientation, doubleBaseInertialFrameOrientation);

	command = b3CreateMultiBodyCommandInit(sm);

	if (args.m_useMaximalCoordinates)
	{
		b3CreateMultiBodyUseMaximalCoordinates(command);
	}
	if (args.m_batchPositions.size())
	{
		btAlignedObjectArray<double> positionArray;
		for (int i = 0; i < args.m_batchPositions.size(); i++)
		{
			positionArray.push_back(args.m_batchPositions[i][0]);
			positionArray.push_back(args.m_batchPositions[i][1]);
			positionArray.push_back(args.m_batchPositions[i][2]);
		}
		b3CreateMultiBodySetBatchPositions(sm, command, &positionArray[0], args.m_batchPositions.size());
	}
	baseIndex = b3CreateMultiBodyBase(command, args.m_baseMass, args.m_baseCollisionShapeIndex, args.m_baseVisualShapeIndex,
									  doubleBasePosition, doubleBaseOrientation, doubleBaseInertialFramePosition, doubleBaseInertialFrameOrientation);

	for (int i = 0; i < args.m_numLinks; i++)
	{
		double linkMass = args.m_linkMasses[i];
		int linkCollisionShapeIndex = args.m_linkCollisionShapeIndices[i];
		int linkVisualShapeIndex = args.m_linkVisualShapeIndices[i];
		btVector3 linkPosition = args.m_linkPositions[i];
		btQuaternion linkOrientation = args.m_linkOrientations[i];
		btVector3 linkInertialFramePosition = args.m_linkInertialFramePositions[i];
		btQuaternion linkInertialFrameOrientation = args.m_linkInertialFrameOrientations[i];
		int linkParentIndex = args.m_linkParentIndices[i];
		int linkJointType = args.m_linkJointTypes[i];
		btVector3 linkJointAxis = args.m_linkJointAxes[i];

		double doubleLinkPosition[3];
		double doubleLinkInertialFramePosition[3];
		double doubleLinkJointAxis[3];
		scalarToDouble3(linkPosition.m_floats, doubleLinkPosition);
		scalarToDouble3(linkInertialFramePosition.m_floats, doubleLinkInertialFramePosition);
		scalarToDouble3(linkJointAxis.m_floats, doubleLinkJointAxis);

		double doubleLinkOrientation[4];
		double doubleLinkInertialFrameOrientation[4];
		scalarToDouble4(linkOrientation, doubleLinkOrientation);
		scalarToDouble4(linkInertialFrameOrientation, doubleLinkInertialFrameOrientation);

		b3CreateMultiBodyLink(command,
							  linkMass,
							  linkCollisionShapeIndex,
							  linkVisualShapeIndex,
							  doubleLinkPosition,
							  doubleLinkOrientation,
							  doubleLinkInertialFramePosition,
							  doubleLinkInertialFrameOrientation,
							  linkParentIndex,
							  linkJointType,
							  doubleLinkJointAxis);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CREATE_MULTI_BODY_COMPLETED)
	{
		int uid = b3GetStatusBodyIndex(statusHandle);
		return uid;
	}
	return -1;
}

int b3RobotSimulatorClientAPI_NoDirect::getNumConstraints() const
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	return b3GetNumUserConstraints(m_data->m_physicsClientHandle);
}

int b3RobotSimulatorClientAPI_NoDirect::getConstraintUniqueId(int serialIndex)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return -1;
	}
	int userConstraintId = -1;
	userConstraintId = b3GetUserConstraintId(m_data->m_physicsClientHandle, serialIndex);
	return userConstraintId;
}

void b3RobotSimulatorClientAPI_NoDirect::setGuiHelper(struct GUIHelperInterface* guiHelper)
{
	m_data->m_guiHelper = guiHelper;
}

struct GUIHelperInterface* b3RobotSimulatorClientAPI_NoDirect::getGuiHelper()
{
	return m_data->m_guiHelper;
}

void b3RobotSimulatorClientAPI_NoDirect::setInternalData(struct b3RobotSimulatorClientAPI_InternalData* data)
{
	*m_data = *data;
}

bool b3RobotSimulatorClientAPI_NoDirect::getCollisionShapeData(int bodyUniqueId, int linkIndex,
															   b3CollisionShapeInformation& collisionShapeInfo)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	{
		command = b3InitRequestCollisionShapeInformation(sm, bodyUniqueId, linkIndex);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
	}

	btAssert(statusType == CMD_COLLISION_SHAPE_INFO_COMPLETED);
	if (statusType == CMD_COLLISION_SHAPE_INFO_COMPLETED)
	{
		b3GetCollisionShapeInformation(sm, &collisionShapeInfo);
	}
	return true;
}

int b3RobotSimulatorClientAPI_NoDirect::saveStateToMemory()
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;

	int stateId = -1;

	if (sm == 0)
	{
		return -1;
	}

	command = b3SaveStateCommandInit(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SAVE_STATE_COMPLETED)
	{
		return -1;
	}

	stateId = b3GetStatusGetStateId(statusHandle);
	return stateId;
}

void b3RobotSimulatorClientAPI_NoDirect::restoreStateFromMemory(int stateId)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return;
	}

	int statusType;
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int physicsClientId = 0;

	command = b3LoadStateCommandInit(sm);
	if (stateId >= 0)
	{
		b3LoadStateSetStateId(command, stateId);
	}
	//	if (fileName)
	//	{
	//		b3LoadStateSetFileName(command, fileName);
	//	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
}

void b3RobotSimulatorClientAPI_NoDirect::removeState(int stateUniqueId)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return;
	}

	if (stateUniqueId >= 0)
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		if (b3CanSubmitCommand(sm))
		{
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitRemoveStateCommand(sm, stateUniqueId));
			statusType = b3GetStatusType(statusHandle);
		}
	}
}

bool b3RobotSimulatorClientAPI_NoDirect::getVisualShapeData(int bodyUniqueId, b3VisualShapeInformation& visualShapeInfo)
{
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return false;
	}
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	commandHandle = b3InitRequestVisualShapeInformation(sm, bodyUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	btAssert(statusType == CMD_VISUAL_SHAPE_INFO_COMPLETED);
	if (statusType == CMD_VISUAL_SHAPE_INFO_COMPLETED)
	{
		b3GetVisualShapeInformation(sm, &visualShapeInfo);
		return true;
	}
	return false;
}

void b3RobotSimulatorClientAPI_NoDirect::setAdditionalSearchPath(const std::string& path)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
	if (sm == 0)
	{
		b3Warning("Not connected");
		return;
	}
	if (path.length())
	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		commandHandle = b3SetAdditionalSearchPath(sm, path.c_str());
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
}

void b3RobotSimulatorClientAPI_NoDirect::setCollisionFilterGroupMask(int bodyUniqueIdA, int linkIndexA, int collisionFilterGroup, int collisionFilterMask)
{
    int physicsClientId = 0;
    b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
    if (sm == 0)
    {
        b3Warning("Not connected");
        return;
    }
    
    b3SharedMemoryCommandHandle commandHandle;
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;
    
    commandHandle = b3CollisionFilterCommandInit(sm);
    b3SetCollisionFilterGroupMask(commandHandle, bodyUniqueIdA, linkIndexA, collisionFilterGroup, collisionFilterMask);
    
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
    statusType = b3GetStatusType(statusHandle);
}

