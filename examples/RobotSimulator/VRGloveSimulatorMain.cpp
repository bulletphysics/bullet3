//VR Glove hand simulator is a C++ conversion from the Python pybullet vrhand_vive_tracker.py
//For more details about the VR glove, see also https://docs.google.com/document/d/1_qwXJRBTGKmhktdBtVQ6zdOdxwud1K30jt0G5dkAr10/edit

#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "serial/serial.h"
#include "../Importers/ImportURDFDemo/urdfStringSplit.h"

double convertSensor(int inputV, int minV, int maxV)
{
	b3Clamp(inputV, minV, maxV);
	double outVal = (double)inputV;
	double b = (outVal - (double)minV) / float(maxV - minV);
	return (b);
}

void setJointMotorPositionControl(b3RobotSimulatorClientAPI* sim, int obUid, int linkIndex, double targetPosition)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = 50.;
	controlArgs.m_targetPosition = targetPosition;
	controlArgs.m_targetVelocity = 0;
	sim->setJointMotorControl(obUid, linkIndex, controlArgs);
}

int main(int argc, char* argv[])
{
	b3CommandLineArgs args(argc, argv);
	std::string port = "COM9";
	args.GetCmdLineArgument("port", port);
	int baud = 115200;
	args.GetCmdLineArgument("speed", baud);
	std::string mode = "SHARED_MEMORY";
	args.GetCmdLineArgument("mode", mode);
	int disableGui = 0;
	args.GetCmdLineArgument("disableGui", disableGui);
	int disableShadows = 0;
	args.GetCmdLineArgument("disableShadows", disableShadows);
	int useKitchen = 0;
	args.GetCmdLineArgument("useKitchen", useKitchen);

	int deviceTypeFilter = VR_DEVICE_GENERIC_TRACKER;
	args.GetCmdLineArgument("deviceTypeFilter", deviceTypeFilter);

	printf("port=%s, speed=%d, connection mode=%s\n", port.c_str(), baud, mode.c_str());

	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

	//Can also use eCONNECT_UDP,eCONNECT_TCP, for example: sim->connect(eCONNECT_UDP, "localhost", 1234);
	if (mode == "GUI")
	{
		sim->connect(eCONNECT_GUI);
	}
	else
	{
		if (mode == "DIRECT")
		{
			sim->connect(eCONNECT_DIRECT);
		}
		else
		{
			sim->connect(eCONNECT_SHARED_MEMORY);
		}
	}

	sim->setRealTimeSimulation(true);
	sim->setInternalSimFlags(0);
	sim->resetSimulation();

	if (disableGui)
	{
		sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	}

	if (disableShadows)
	{
		sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 0);
	}

	sim->setTimeOut(12345);
	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();
	b3Scalar fixedTimeStep = 1. / 240.;

	sim->setTimeStep(fixedTimeStep);

	b3Quaternion q = sim->getQuaternionFromEuler(b3MakeVector3(0.1, 0.2, 0.3));
	b3Vector3 rpy;
	rpy = sim->getEulerFromQuaternion(q);

	sim->setGravity(b3MakeVector3(0, 0, -9.8));
	sim->setContactBreakingThreshold(0.001);

	if (useKitchen)
	{
		b3RobotSimulatorLoadFileResults res;
		sim->loadSDF("kitchens/1.sdf", res);
	}
	else
	{
		sim->loadURDF("plane_with_collision_audio.urdf");
	}

	int handUid = -1;

	b3RobotSimulatorLoadFileResults mjcfResults;
	const char* mjcfFileName = "MPL/mpl2.xml";
	if (sim->loadMJCF(mjcfFileName, mjcfResults))
	{
		printf("mjcfResults = %d\n", mjcfResults.m_uniqueObjectIds.size());
		if (mjcfResults.m_uniqueObjectIds.size() == 1)
		{
			handUid = mjcfResults.m_uniqueObjectIds[0];
		}
	}
	if (handUid < 0)
	{
		printf("Cannot load MJCF file %s\n", mjcfFileName);
	}

#ifdef TOUCH
	b3Vector3 handPos = b3MakeVector3(-0.10, -0.03, 0.02);
	b3Vector3 rollPitchYaw = b3MakeVector3(0.5 * B3_PI, 0, 1.25 * B3_PI);  //-B3_PI/2,0,B3_PI/2);
	handOrn = sim->getQuaternionFromEuler(rollPitchYaw);

#else
	b3Quaternion handOrn = sim->getQuaternionFromEuler(b3MakeVector3(3.14, -3.14 / 2, 0));
	b3Vector3 handPos = b3MakeVector3(-0.05, 0, 0.02);
#endif

	b3Vector3 handStartPosWorld = b3MakeVector3(0.500000, 0.300006, 0.900000);
	b3Quaternion handStartOrnWorld = b3Quaternion ::getIdentity();

	b3JointInfo jointInfo;
	jointInfo.m_jointType = eFixedType;

	printf("handStartOrnWorld=%f,%f,%f,%f\n", handStartOrnWorld[0], handStartOrnWorld[1], handStartOrnWorld[2], handStartOrnWorld[3]);
	jointInfo.m_childFrame[0] = handStartPosWorld[0];
	jointInfo.m_childFrame[1] = handStartPosWorld[1];
	jointInfo.m_childFrame[2] = handStartPosWorld[2];
	jointInfo.m_childFrame[3] = handStartOrnWorld[0];
	jointInfo.m_childFrame[4] = handStartOrnWorld[1];
	jointInfo.m_childFrame[5] = handStartOrnWorld[2];
	jointInfo.m_childFrame[6] = handStartOrnWorld[3];

	jointInfo.m_parentFrame[0] = handPos[0];
	jointInfo.m_parentFrame[1] = handPos[1];
	jointInfo.m_parentFrame[2] = handPos[2];
	jointInfo.m_parentFrame[3] = handOrn[0];
	jointInfo.m_parentFrame[4] = handOrn[1];
	jointInfo.m_parentFrame[5] = handOrn[2];
	jointInfo.m_parentFrame[6] = handOrn[3];

	sim->resetBasePositionAndOrientation(handUid, handStartPosWorld, handStartOrnWorld);
	int handConstraintId = sim->createConstraint(handUid, -1, -1, -1, &jointInfo);
	double maxFingerForce = 10;
	double maxArmForce = 1000;
	for (int j = 0; j < sim->getNumJoints(handUid); j++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
		controlArgs.m_maxTorqueValue = maxFingerForce;
		controlArgs.m_kp = 0.1;
		controlArgs.m_kd = 1;
		controlArgs.m_targetPosition = 0;
		controlArgs.m_targetVelocity = 0;
		sim->setJointMotorControl(handUid, j, controlArgs);
	}

	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(1.300000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(1.200000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(1.100000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(1.000000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.900000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.800000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.700000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("jenga/jenga.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.600000, -0.700000, 0.750000), b3Quaternion(0.000000, 0.707107, 0.000000, 0.707107)));
	sim->loadURDF("table/table.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(1.000000, -0.200000, 0.000000), b3Quaternion(0.000000, 0.000000, 0.707107, 0.707107)));
	sim->loadURDF("cube_small.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.950000, -0.100000, 0.700000), b3Quaternion(0.000000, 0.000000, 0.707107, 0.707107)));
	sim->loadURDF("sphere_small.urdf", b3RobotSimulatorLoadUrdfFileArgs(b3MakeVector3(0.850000, -0.400000, 0.700000), b3Quaternion(0.000000, 0.000000, 0.707107, 0.707107)));

	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double simWallClockSeconds = 20.;

	int vidLogId = -1;
	int minitaurLogId = -1;
	int rotateCamera = 0;
	serial::Serial my_serial;

	try
	{
		// port, baudrate, timeout in milliseconds
		my_serial.setBaudrate(baud);
		my_serial.setPort(port);
		my_serial.setBytesize(serial::sevenbits);
		my_serial.setParity(serial::parity_odd);
		my_serial.setStopbits(serial::stopbits_two);
		my_serial.setTimeout(serial::Timeout::simpleTimeout(0.01));
		my_serial.open();
	}
	catch (...)
	{
		printf("Cannot open port, use --port=PORTNAME\n");
		exit(0);
	}

	if (!my_serial.isOpen())
	{
		printf("Cannot open serial port\n");
		return -1;
	}

	my_serial.flush();

	while (sim->canSubmitCommand())
	{
		clock.usleep(1);

		b3VREventsData vrEvents;

		sim->getVREvents(&vrEvents, deviceTypeFilter);
		//instead of iterating over all vr events, we just take the most up-to-date one
		if (vrEvents.m_numControllerEvents)
		{
			int i = vrEvents.m_numControllerEvents - 1;
			b3VRControllerEvent& e = vrEvents.m_controllerEvents[i];
			//			printf("e.pos=%f,%f,%f\n",e.m_pos[0],e.m_pos[1],e.m_pos[2]);
			b3JointInfo changeConstraintInfo;
			changeConstraintInfo.m_flags = 0;
			changeConstraintInfo.m_jointMaxForce = maxArmForce;
			changeConstraintInfo.m_flags |= eJointChangeMaxForce;

			changeConstraintInfo.m_childFrame[0] = e.m_pos[0];
			changeConstraintInfo.m_childFrame[1] = e.m_pos[1];
			changeConstraintInfo.m_childFrame[2] = e.m_pos[2];
			changeConstraintInfo.m_flags |= eJointChangeChildFramePosition;

			changeConstraintInfo.m_childFrame[3] = e.m_orn[0];
			changeConstraintInfo.m_childFrame[4] = e.m_orn[1];
			changeConstraintInfo.m_childFrame[5] = e.m_orn[2];
			changeConstraintInfo.m_childFrame[6] = e.m_orn[3];
			changeConstraintInfo.m_flags |= eJointChangeChildFrameOrientation;

			sim->changeConstraint(handConstraintId, &changeConstraintInfo);
		}

		//read the serial output from the hand, extract into parts
		std::string result;
		try
		{
			result = my_serial.readline();
		}
		catch (...)
		{
		}
		if (result.length())
		{
			my_serial.flush();
			int res = result.find("\n");
			while (res < 0)
			{
				result += my_serial.readline();
				res = result.find("\n");
			}
			btAlignedObjectArray<std::string> pieces;
			btAlignedObjectArray<std::string> separators;
			separators.push_back(",");
			urdfStringSplit(pieces, result, separators);

			//printf("serial: %s\n", result.c_str());
			if (pieces.size() == 6)
			{
				double pinkTarget = 0;
				double middleTarget = 0;
				double indexTarget = 0;
				double thumbTarget = 0;
				{
					int pink = atoi(pieces[1].c_str());
					int middle = atoi(pieces[2].c_str());
					int index = atoi(pieces[3].c_str());
					int thumb = atoi(pieces[4].c_str());

					pinkTarget = convertSensor(pink, 250, 400);
					middleTarget = convertSensor(middle, 250, 400);
					indexTarget = convertSensor(index, 250, 400);
					thumbTarget = convertSensor(thumb, 250, 400);
				}

				//printf("pink = %d, middle=%d, index=%d, thumb=%d\n", pink,middle,index,thumb);

				setJointMotorPositionControl(sim, handUid, 5, 1.3);
				setJointMotorPositionControl(sim, handUid, 7, thumbTarget);
				setJointMotorPositionControl(sim, handUid, 9, thumbTarget);
				setJointMotorPositionControl(sim, handUid, 11, thumbTarget);

				setJointMotorPositionControl(sim, handUid, 15, indexTarget);
				setJointMotorPositionControl(sim, handUid, 17, indexTarget);
				setJointMotorPositionControl(sim, handUid, 19, indexTarget);

				setJointMotorPositionControl(sim, handUid, 22, middleTarget);
				setJointMotorPositionControl(sim, handUid, 24, middleTarget);
				setJointMotorPositionControl(sim, handUid, 27, middleTarget);

				double ringTarget = 0.5f * (pinkTarget + middleTarget);
				setJointMotorPositionControl(sim, handUid, 30, ringTarget);
				setJointMotorPositionControl(sim, handUid, 32, ringTarget);
				setJointMotorPositionControl(sim, handUid, 34, ringTarget);

				setJointMotorPositionControl(sim, handUid, 38, pinkTarget);
				setJointMotorPositionControl(sim, handUid, 40, pinkTarget);
				setJointMotorPositionControl(sim, handUid, 42, pinkTarget);
			}
		}

		b3KeyboardEventsData keyEvents;
		sim->getKeyboardEvents(&keyEvents);

		//sim->stepSimulation();

		if (rotateCamera)
		{
			static double yaw = 0;
			double distance = 1;
			yaw += 0.1;
			b3Vector3 basePos;
			b3Quaternion baseOrn;
			//			sim->getBasePositionAndOrientation(minitaurUid,basePos,baseOrn);
			//			sim->resetDebugVisualizerCamera(distance,yaw,20,basePos);
		}
		//b3Clock::usleep(1000.*1000.*fixedTimeStep);
	}

	printf("serial.close()\n");
	my_serial.close();

	printf("sim->disconnect\n");
	sim->disconnect();

	printf("delete sim\n");
	delete sim;

	printf("exit\n");
}
