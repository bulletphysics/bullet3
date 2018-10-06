//todo: turn this into a gtest, comparing maximal and reduced coordinates contact behavior etc

#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#define ASSERT_EQ(a, b) assert((a) == (b));
#include "MinitaurSetup.h"
#define NUM_SIM 1
int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sims[2];
	b3Scalar fixedTimeStep = 1. / 240.;
	for (int i = 0; i < NUM_SIM; i++)
	{
		b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
		sims[i] = sim;
		sim->connect(eCONNECT_GUI);  //eCONNECT_GUI);//DIRECT);
		//Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
		//sim->connect(eCONNECT_UDP, "localhost", 1234);
		sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
		//	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME
		//sim->setTimeOut(3);
		//syncBodies is only needed when connecting to an existing physics server that has already some bodies
		sim->syncBodies();

		sim->setTimeStep(fixedTimeStep);

		b3Quaternion q = sim->getQuaternionFromEuler(b3MakeVector3(0.1, 0.2, 0.3));
		b3Vector3 rpy;
		rpy = sim->getEulerFromQuaternion(q);

		sim->setGravity(b3MakeVector3(0, 0, -9.8));

		//int blockId = sim->loadURDF("cube.urdf");
		//b3BodyInfo bodyInfo;
		//sim->getBodyInfo(blockId,&bodyInfo);

		b3Quaternion orn;
		orn.setEulerZYX(0, B3_PI * 0.23, 0);

		b3RobotSimulatorLoadUrdfFileArgs args;
		args.m_startPosition.setValue(0, 0, 0);
		args.m_startOrientation = orn;
		args.m_useMultiBody = i == 0 ? false : true;  //true : false;//false : true;

		sim->loadURDF("plane.urdf", args);

		args.m_startPosition.setValue(0, 0, 0.66);
		args.m_startOrientation.setEulerZYX(0, B3_PI * 0.23, 0);
		sim->loadURDF("cube_soft.urdf", args);

		double distance = 1.5;
		double yaw = 50;
		sim->resetDebugVisualizerCamera(distance, yaw, 20, b3MakeVector3(0, 0, 0.1));
		sim->setRealTimeSimulation(false);
	}
	int enableSim = 1;
	while (sims[0]->canSubmitCommand())
	{
#if 0
		b3KeyboardEventsData keyEvents;
		sim->getKeyboardEvents(&keyEvents);
		if (keyEvents.m_numKeyboardEvents)
		{
		
			//printf("num key events = %d]\n", keyEvents.m_numKeyboardEvents);
			//m_keyState is a flag combination of eButtonIsDown,eButtonTriggered, eButtonReleased
			for (int i=0;i<keyEvents.m_numKeyboardEvents;i++)
			{
				if (keyEvents.m_keyboardEvents[i].m_keyCode=='i' && keyEvents.m_keyboardEvents[i].m_keyState & eButtonTriggered)
				{
					enableSim = !enableSim;
				}
			}
		}
#endif
		for (int i = 0; i < NUM_SIM; i++)
		{
			sims[i]->setGravity(b3MakeVector3(0, 0, -10));
			//printf(".");
			if (enableSim)
			{
				sims[i]->stepSimulation();
			}
		}
		b3Clock::usleep(1000. * 1000. * fixedTimeStep);
	}

	printf("sim->disconnect\n");

	for (int i = 0; i < NUM_SIM; i++)
	{
		sims[i]->disconnect();
		printf("delete sim\n");
		delete sims[i];
	}

	printf("exit\n");
}
