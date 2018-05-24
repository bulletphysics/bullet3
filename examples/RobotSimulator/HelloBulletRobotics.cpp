
#include "b3RobotSimulatorClientAPI_NoGUI.h"

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();

	bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);
	if (!isConnected)
	{
		printf("Using Direct mode\n");
		isConnected = sim->connect(eCONNECT_DIRECT);
	} else
	{
		printf("Using shared memory\n");
	}
	//remove all existing objects (if any)
	sim->resetSimulation();
	
	int planeUid = sim->loadURDF("plane.urdf");
	printf("planeUid = %d\n", planeUid);
	
	int r2d2Uid = sim->loadURDF("r2d2.urdf");
	printf("r2d2 #joints = %d\n", sim->getNumJoints(r2d2Uid));

	b3Vector3 basePosition = b3MakeVector3(0,0,1);
	b3Quaternion baseOrientation = b3Quaternion(0,0,0,1);

	sim->resetBasePositionAndOrientation(r2d2Uid, basePosition, baseOrientation);	
	sim->setGravity(b3MakeVector3(0,0,-10));
	
	while (sim->isConnected())
	{
		sim->stepSimulation();
	}
	delete sim;
}
