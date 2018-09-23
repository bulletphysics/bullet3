/**
 * License: Bullet3 license
 * Author: Avik De <avikde@gmail.com>
 */
#include <map>
#include <string>
#include <stdio.h>
#include "../Utils/b3Clock.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "SharedMemory/SharedMemoryInProcessPhysicsC_API.h"

extern const int CONTROL_RATE;
const int CONTROL_RATE = 500;

// Bullet globals
b3PhysicsClientHandle kPhysClient = 0;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
// temp vars used a lot
b3SharedMemoryCommandHandle command;
b3SharedMemoryStatusHandle statusHandle;
int statusType, ret;
b3JointInfo jointInfo;
b3JointSensorState state;
// test
int twojoint;
std::map<std::string, int> jointNameToId;

int main(int argc, char* argv[])
{
	kPhysClient = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
	if (!kPhysClient)
		return -1;
	// visualizer
	command = b3InitConfigureOpenGLVisualizer(kPhysClient);
	b3ConfigureOpenGLVisualizerSetVisualizationFlags(command, COV_ENABLE_GUI, 0);
	b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	b3ConfigureOpenGLVisualizerSetVisualizationFlags(command, COV_ENABLE_SHADOWS, 0);
	b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	b3SetTimeOut(kPhysClient, 10);

	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	command = b3InitSyncBodyInfoCommand(kPhysClient);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	statusType = b3GetStatusType(statusHandle);

	// set fixed time step
	command = b3InitPhysicsParamCommand(kPhysClient);
	ret = b3PhysicsParamSetTimeStep(command, FIXED_TIMESTEP);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	ret = b3PhysicsParamSetRealTimeSimulation(command, false);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	b3Assert(b3GetStatusType(statusHandle) == CMD_CLIENT_COMMAND_COMPLETED);

	// load test
	command = b3LoadUrdfCommandInit(kPhysClient, "TwoJointRobot_wo_fixedJoints.urdf");
	int flags = URDF_USE_INERTIA_FROM_FILE;
	b3LoadUrdfCommandSetFlags(command, flags);
	b3LoadUrdfCommandSetUseFixedBase(command, true);
	// q.setEulerZYX(0, 0, 0);
	// b3LoadUrdfCommandSetStartOrientation(command, q[0], q[1], q[2], q[3]);
	b3LoadUrdfCommandSetUseMultiBody(command, true);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	statusType = b3GetStatusType(statusHandle);
	b3Assert(statusType == CMD_URDF_LOADING_COMPLETED);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		twojoint = b3GetStatusBodyIndex(statusHandle);
	}

	//disable default linear/angular damping
	b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(kPhysClient);
	double linearDamping = 0;
	double angularDamping = 0;
	b3ChangeDynamicsInfoSetLinearDamping(command, twojoint, linearDamping);
	b3ChangeDynamicsInfoSetAngularDamping(command, twojoint, angularDamping);
	statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

	int numJoints = b3GetNumJoints(kPhysClient, twojoint);
	printf("twojoint numjoints = %d\n", numJoints);
	// Loop through all joints
	for (int i = 0; i < numJoints; ++i)
	{
		b3GetJointInfo(kPhysClient, twojoint, i, &jointInfo);
		if (jointInfo.m_jointName[0])
		{
			jointNameToId[std::string(jointInfo.m_jointName)] = i;
		}
		else
		{
			continue;
		}
		// Reset before torque control - see #1459
		command = b3JointControlCommandInit2(kPhysClient, twojoint, CONTROL_MODE_VELOCITY);
		b3JointControlSetDesiredVelocity(command, jointInfo.m_uIndex, 0);
		b3JointControlSetMaximumForce(command, jointInfo.m_uIndex, 0);
		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
	}

	// loop
	unsigned long dtus1 = (unsigned long)1000000.0 * FIXED_TIMESTEP;
	double simTimeS = 0;
	double q[2], v[2];
	while (b3CanSubmitCommand(kPhysClient))
	{
		simTimeS += 0.000001 * dtus1;
		// apply some torque
		b3GetJointInfo(kPhysClient, twojoint, jointNameToId["joint_2"], &jointInfo);
		command = b3JointControlCommandInit2(kPhysClient, twojoint, CONTROL_MODE_TORQUE);
		b3JointControlSetDesiredForceTorque(command, jointInfo.m_uIndex, 0.5 * sin(10 * simTimeS));
		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);

		// get joint values
		command = b3RequestActualStateCommandInit(kPhysClient, twojoint);
		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, command);
		b3GetJointState(kPhysClient, statusHandle, jointNameToId["joint_1"], &state);
		q[0] = state.m_jointPosition;
		v[0] = state.m_jointVelocity;
		b3GetJointState(kPhysClient, statusHandle, jointNameToId["joint_2"], &state);
		q[1] = state.m_jointPosition;
		v[1] = state.m_jointVelocity;

		statusHandle = b3SubmitClientCommandAndWaitStatus(kPhysClient, b3InitStepSimulationCommand(kPhysClient));

		// debugging output
		printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", simTimeS, q[0], q[1], v[0], v[1]);
		b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
	}
	b3DisconnectSharedMemory(kPhysClient);
	return 0;
}