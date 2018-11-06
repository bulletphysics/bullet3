
//#include "SharedMemoryCommands.h"
#ifdef PHYSICS_SHARED_MEMORY
#include "SharedMemory/PhysicsClientC_API.h"
#endif  //PHYSICS_SHARED_MEMORY

#ifdef PHYSICS_UDP
#include "SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //PHYSICS_UDP

#ifdef PHYSICS_TCP
#include "SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //PHYSICS_TCP

#ifdef PHYSICS_LOOP_BACK
#include "SharedMemory/PhysicsLoopBackC_API.h"
#endif  //PHYSICS_LOOP_BACK

#ifdef PHYSICS_SERVER_DIRECT
#include "SharedMemory/PhysicsDirectC_API.h"
#endif  //PHYSICS_SERVER_DIRECT

#ifdef PHYSICS_IN_PROCESS_EXAMPLE_BROWSER
#include "SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#endif  //PHYSICS_IN_PROCESS_EXAMPLE_BROWSER

#include "SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"
#include <string.h>

#include <stdio.h>

#ifndef ENABLE_GTEST
#include <assert.h>
#define ASSERT_EQ(a, b) assert((a) == (b));
#else
#define printf
#endif

void testSharedMemory(b3PhysicsClientHandle sm)
{
	int i, dofCount, posVarCount, ret, numJoints;
	int sensorJointIndexLeft = -1;
	int sensorJointIndexRight = -1;
	const char* urdfFileName = "r2d2.urdf";
	const char* sdfFileName = "kuka_iiwa/model.sdf";
	double gravx = 0, gravy = 0, gravz = -9.8;
	double timeStep = 1. / 60.;
	double startPosX, startPosY, startPosZ;
	int imuLinkIndex = -1;
	int bodyIndex = -1;

	if (b3CanSubmitCommand(sm))
	{
		{
			b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
			b3SharedMemoryStatusHandle statusHandle;
			ret = b3PhysicsParamSetGravity(command, gravx, gravy, gravz);
			ret = b3PhysicsParamSetTimeStep(command, timeStep);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
		}

		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			int bodyIndicesOut[10];  //MAX_SDF_BODIES = 10
			int numJoints, numBodies;
			int bodyUniqueId;
			b3SharedMemoryCommandHandle command = b3LoadSdfCommandInit(sm, sdfFileName);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			statusType = b3GetStatusType(statusHandle);
			ASSERT_EQ(statusType, CMD_SDF_LOADING_COMPLETED);

			numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, 10);
			ASSERT_EQ(numBodies, 1);
			bodyUniqueId = bodyIndicesOut[0];
			{
				{
					b3SharedMemoryStatusHandle statusHandle;
					int statusType;
					b3SharedMemoryCommandHandle command = b3InitRequestVisualShapeInformation(sm, bodyUniqueId);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
					statusType = b3GetStatusType(statusHandle);
					if (statusType == CMD_VISUAL_SHAPE_INFO_COMPLETED)
					{
						struct b3VisualShapeInformation vi;
						b3GetVisualShapeInformation(sm, &vi);
					}
				}
			}

			numJoints = b3GetNumJoints(sm, bodyUniqueId);
			ASSERT_EQ(numJoints, 7);

#if 0
            b3Printf("numJoints: %d\n", numJoints);
            for (i=0;i<numJoints;i++)
            {
                struct b3JointInfo jointInfo;
                if (b3GetJointInfo(sm,bodyUniqueId, i,&jointInfo))
                {
                    b3Printf("jointInfo[%d].m_jointName=%s\n",i,jointInfo.m_jointName);
                }
            }
#endif
			{
				b3SharedMemoryStatusHandle statusHandle;
				b3SharedMemoryCommandHandle commandHandle;
				double jointAngle = 0.f;
				int jointIndex;
				commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);
				for (jointIndex = 0; jointIndex < numJoints; jointIndex++)
				{
					b3CreatePoseCommandSetJointPosition(sm, commandHandle, jointIndex, jointAngle);
				}

				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

				ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
			}
		}

		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);

			//setting the initial position, orientation and other arguments are optional
			startPosX = 2;
			startPosY = 0;
			startPosZ = 1;
			ret = b3LoadUrdfCommandSetStartPosition(command, startPosX, startPosY, startPosZ);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			statusType = b3GetStatusType(statusHandle);
			ASSERT_EQ(statusType, CMD_URDF_LOADING_COMPLETED);
			bodyIndex = b3GetStatusBodyIndex(statusHandle);
		}

		if (bodyIndex >= 0)
		{
			numJoints = b3GetNumJoints(sm, bodyIndex);
			for (i = 0; i < numJoints; i++)
			{
				struct b3JointInfo jointInfo;
				b3GetJointInfo(sm, bodyIndex, i, &jointInfo);

				//	printf("jointInfo[%d].m_jointName=%s\n",i,jointInfo.m_jointName);
				//pick the IMU link index based on torso name
				if (strstr(jointInfo.m_linkName, "base_link"))
				{
					imuLinkIndex = i;
				}

				//pick the joint index based on joint name
				if (strstr(jointInfo.m_jointName, "base_to_left_leg"))
				{
					sensorJointIndexLeft = i;
				}
				if (strstr(jointInfo.m_jointName, "base_to_right_leg"))
				{
					sensorJointIndexRight = i;
				}
			}

			if ((sensorJointIndexLeft >= 0) || (sensorJointIndexRight >= 0))
			{
				b3SharedMemoryCommandHandle command = b3CreateSensorCommandInit(sm, bodyIndex);
				b3SharedMemoryStatusHandle statusHandle;
				if (imuLinkIndex >= 0)
				{
					ret = b3CreateSensorEnableIMUForLink(command, imuLinkIndex, 1);
				}

				if (sensorJointIndexLeft >= 0)
				{
					ret = b3CreateSensorEnable6DofJointForceTorqueSensor(command, sensorJointIndexLeft, 1);
				}
				if (sensorJointIndexRight >= 0)
				{
					ret = b3CreateSensorEnable6DofJointForceTorqueSensor(command, sensorJointIndexRight, 1);
				}
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
				ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
			}
		}

		{
			b3SharedMemoryStatusHandle statusHandle;
			b3SharedMemoryCommandHandle command = b3CreateBoxShapeCommandInit(sm);
			ret = b3CreateBoxCommandSetStartPosition(command, 0, 0, -1);
			ret = b3CreateBoxCommandSetStartOrientation(command, 0, 0, 0, 1);
			ret = b3CreateBoxCommandSetHalfExtents(command, 10, 10, 1);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			ASSERT_EQ(b3GetStatusType(statusHandle), CMD_RIGID_BODY_CREATION_COMPLETED);
		}

		{
			int statusType;
			b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(sm, bodyIndex);
			b3SharedMemoryStatusHandle statusHandle;
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			statusType = b3GetStatusType(statusHandle);
			ASSERT_EQ(statusType, CMD_ACTUAL_STATE_UPDATE_COMPLETED);

			if (statusType == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				b3GetStatusActualState(statusHandle,
									   0, &posVarCount, &dofCount,
									   0, 0, 0, 0);
				ASSERT_EQ(posVarCount, 15);
				ASSERT_EQ(dofCount, 14);
			}
		}

		{
#if 0
            b3SharedMemoryStatusHandle statusHandle;
             b3SharedMemoryCommandHandle command = b3JointControlCommandInit( sm, CONTROL_MODE_VELOCITY);
            for ( dofIndex=0;dofIndex<dofCount;dofIndex++)
            {
                b3JointControlSetDesiredVelocity(command,dofIndex,1);
                b3JointControlSetMaximumForce(command,dofIndex,100);
            }
             statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
#endif
		}
		///perform some simulation steps for testing
		for (i = 0; i < 1000; i++)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;

			if (b3CanSubmitCommand(sm))
			{
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitStepSimulationCommand(sm));
				statusType = b3GetStatusType(statusHandle);
				ASSERT_EQ(statusType, CMD_STEP_FORWARD_SIMULATION_COMPLETED);
			}
			else
			{
				break;
			}
		}

		{
			b3SharedMemoryCommandHandle command;
			b3SharedMemoryStatusHandle statusHandle;
			int width = 1024;
			int height = 1024;

			command = b3InitRequestCameraImage(sm);

			b3RequestCameraImageSetPixelResolution(command, width, height);
			b3RequestCameraImageSelectRenderer(command, ER_BULLET_HARDWARE_OPENGL);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		}

		if (b3CanSubmitCommand(sm))
		{
			b3SharedMemoryStatusHandle state = b3SubmitClientCommandAndWaitStatus(sm, b3RequestActualStateCommandInit(sm, bodyIndex));

			if (sensorJointIndexLeft >= 0)
			{
				struct b3JointSensorState sensorState;
				b3GetJointState(sm, state, sensorJointIndexLeft, &sensorState);

				b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexLeft,
						 sensorState.m_jointForceTorque[0],
						 sensorState.m_jointForceTorque[1],
						 sensorState.m_jointForceTorque[2]);
			}

			if (sensorJointIndexRight >= 0)
			{
				struct b3JointSensorState sensorState;
				b3GetJointState(sm, state, sensorJointIndexRight, &sensorState);

				b3Printf("Sensor for joint [%d] = %f,%f,%f\n", sensorJointIndexRight,
						 sensorState.m_jointForceTorque[0],
						 sensorState.m_jointForceTorque[1],
						 sensorState.m_jointForceTorque[2]);
			}

			{
				b3SharedMemoryStatusHandle statusHandle;
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitResetSimulationCommand(sm));
				ASSERT_EQ(b3GetStatusType(statusHandle), CMD_RESET_SIMULATION_COMPLETED);
			}
		}
	}
	else
	{
		b3Warning("Cannot submit commands.\n");
	}

	b3DisconnectSharedMemory(sm);
}

#ifdef ENABLE_GTEST

TEST(BulletPhysicsClientServerTest, DirectConnection)
{
	b3PhysicsClientHandle sm = b3ConnectPhysicsDirect();
	testSharedMemory(sm);
}

TEST(BulletPhysicsClientServerTest, LoopBackSharedMemory)
{
	b3PhysicsClientHandle sm = b3ConnectPhysicsLoopback(SHARED_MEMORY_KEY);
	testSharedMemory(sm);
}

#else

int main(int argc, char* argv[])
{
#ifdef PHYSICS_LOOP_BACK
	b3PhysicsClientHandle sm = b3ConnectPhysicsLoopback(SHARED_MEMORY_KEY);
#endif

#ifdef PHYSICS_SERVER_DIRECT
	b3PhysicsClientHandle sm = b3ConnectPhysicsDirect();
#endif

#ifdef PHYSICS_IN_PROCESS_EXAMPLE_BROWSER

#ifdef __APPLE__
	b3PhysicsClientHandle sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
	b3PhysicsClientHandle sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif  //__APPLE__
#endif

#ifdef PHYSICS_SHARED_MEMORY
	b3PhysicsClientHandle sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
#endif  //PHYSICS_SHARED_MEMORY

#ifdef PHYSICS_UDP
	b3PhysicsClientHandle sm = b3ConnectPhysicsUDP("localhost", 1234);
#endif  //PHYSICS_UDP

#ifdef PHYSICS_TCP
	b3PhysicsClientHandle sm = b3ConnectPhysicsTCP("localhost", 6667);
#endif  //PHYSICS_UDP

	testSharedMemory(sm);
}
#endif
