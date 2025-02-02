
#ifdef B3_USE_ROBOTSIM_GUI
#include "b3RobotSimulatorClientAPI.h"
#else
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#endif

#include "../Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#define ASSERT_EQ(a, b) assert((a) == (b));
#include "MinitaurSetup.h"

int main(int /*argc*/, char* /*argv*/[])
{
#ifdef B3_USE_ROBOTSIM_GUI
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);
#else
	b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();
	bool isConnected = sim->connect(eCONNECT_DIRECT);
#endif
	if (!isConnected)
	{
		printf("Cannot connect\n");
		return -1;
	}
	//Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
	//sim->connect(eCONNECT_UDP, "localhost", 1234);
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	//	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME
	sim->setTimeOut(10);
	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();
	btScalar fixedTimeStep = btScalar(1. / 240.);

	sim->setTimeStep(fixedTimeStep);

	btQuaternion q = sim->getQuaternionFromEuler(btVector3(btScalar(0.1), btScalar(0.2), btScalar(0.3)));
	btVector3 rpy;
	rpy = sim->getEulerFromQuaternion(q);

	sim->setGravity(btVector3(0, 0, btScalar(-9.8)));

	//int blockId = sim->loadURDF("cube.urdf");
	//b3BodyInfo bodyInfo;
	//sim->getBodyInfo(blockId,&bodyInfo);

	sim->loadURDF("plane.urdf");

	MinitaurSetup minitaur;
	int minitaurUid = minitaur.setupMinitaur(sim, btVector3(0, 0, btScalar(.3)));

	//b3RobotSimulatorLoadUrdfFileArgs args;
	//args.m_startPosition.setValue(2,0,1);
	//int r2d2 = sim->loadURDF("r2d2.urdf",args);

	//b3RobotSimulatorLoadFileResults sdfResults;
	//if (!sim->loadSDF("two_cubes.sdf",sdfResults))
	//{
	//		b3Warning("Can't load SDF!\n");
	//}

#if 0
	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double simWallClockSeconds = 20.;
	while (clock.getTimeInSeconds()-startTime < simWallClockSeconds)
	{
		sim->stepSimulation();
	}
#endif
	sim->setRealTimeSimulation(false);
	int vidLogId = -1;
	int minitaurLogId = -1;
	int rotateCamera = 0;

	while (sim->canSubmitCommand())
	{
		b3KeyboardEventsData keyEvents;
		sim->getKeyboardEvents(&keyEvents);
		if (keyEvents.m_numKeyboardEvents)
		{
			//printf("num key events = %d]\n", keyEvents.m_numKeyboardEvents);
			//m_keyState is a flag combination of eButtonIsDown,eButtonTriggered, eButtonReleased
			for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++)
			{
				b3KeyboardEvent& e = keyEvents.m_keyboardEvents[i];

				if (e.m_keyCode == '0')
				{
					if (e.m_keyState & eButtonTriggered)
					{
						if (vidLogId < 0)
						{
							vidLogId = sim->startStateLogging(STATE_LOGGING_VIDEO_MP4, "video.mp4");
						}
						else
						{
							sim->stopStateLogging(vidLogId);
							vidLogId = -1;
						}
					}
				}

				if (e.m_keyCode == 'm')
				{
					if (minitaurLogId < 0 && e.m_keyState & eButtonTriggered)
					{
						minitaurLogId = sim->startStateLogging(STATE_LOGGING_MINITAUR, "simlog.bin");
					}
					if (minitaurLogId >= 0 && e.m_keyState & eButtonReleased)
					{
						sim->stopStateLogging(minitaurLogId);
						minitaurLogId = -1;
					}
				}

				if (e.m_keyCode == 'r' && e.m_keyState & eButtonTriggered)
				{
					rotateCamera = 1 - rotateCamera;
				}

				//printf("keyEvent[%d].m_keyCode = %d, state = %d\n", i,keyEvents.m_keyboardEvents[i].m_keyCode,keyEvents.m_keyboardEvents[i].m_keyState);
			}
		}
		sim->stepSimulation();

		if (rotateCamera)
		{
			static double yaw = 0;
			double distance = 1;
			yaw += 0.1;
			btVector3 basePos;
			btQuaternion baseOrn;
			sim->getBasePositionAndOrientation(minitaurUid, basePos, baseOrn);
			sim->resetDebugVisualizerCamera(distance, -20, yaw, basePos);
		}
		b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	}

	printf("sim->disconnect\n");

	sim->disconnect();

	printf("delete sim\n");
	delete sim;

	printf("exit\n");
}
