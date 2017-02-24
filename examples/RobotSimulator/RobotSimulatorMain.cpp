
#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#define ASSERT_EQ(a,b) assert((a)==(b));
#include "MinitaurSetup.h"
int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

	sim->connect(eCONNECT_GUI);
	//Can also use eCONNECT_DIRECT,eCONNECT_SHARED_MEMORY,eCONNECT_UDP,eCONNECT_TCP, for example:
	//sim->connect(eCONNECT_UDP, "localhost", 1234);
	sim->configureDebugVisualizer( COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);//COV_ENABLE_WIREFRAME

	//syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();
	
	sim->setTimeStep(1./240.);

	sim->setGravity(b3MakeVector3(0,0,-10));

	sim->loadURDF("plane.urdf");

	MinitaurSetup minitaur;
	int minitaurUid = minitaur.setupMinitaur(sim, b3MakeVector3(0,0,1));

	
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition.setValue(2,0,1);
	int r2d2 = sim->loadURDF("r2d2.urdf",args);

	b3RobotSimulatorLoadFileResults sdfResults;
	if (!sim->loadSDF("two_cubes.sdf",sdfResults))
	{
		b3Warning("Can't load SDF!\n");
	}

	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double simWallClockSeconds = 20.;

	while (clock.getTimeInSeconds()-startTime < simWallClockSeconds)
	{
		sim->stepSimulation();
	}

	sim->setRealTimeSimulation(true);
	
	startTime = clock.getTimeInSeconds();
	while (clock.getTimeInSeconds()-startTime < simWallClockSeconds)
	{
		b3Clock::usleep(1000);
	}

	sim->disconnect();
	delete sim;


}


