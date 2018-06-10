
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
	sim->setGravity(btVector3(0,0,-9.8));
	sim->setNumSolverIterations(100);
	b3RobotSimulatorSetPhysicsEngineParameters args;
	sim->getPhysicsEngineParameters(args);
	

	int planeUid = sim->loadURDF("plane.urdf");
	printf("planeUid = %d\n", planeUid);
	
	int r2d2Uid = sim->loadURDF("r2d2.urdf");
	printf("r2d2 #joints = %d\n", sim->getNumJoints(r2d2Uid));
	
	
	btVector3 basePosition = btVector3(0,0,0.5);
	btQuaternion baseOrientation = btQuaternion(0,0,0,1);

	sim->resetBasePositionAndOrientation(r2d2Uid, basePosition, baseOrientation);
	
	
	while (sim->isConnected())
	{
		btVector3 basePos;
		btQuaternion baseOrn;
		sim->getBasePositionAndOrientation(r2d2Uid,basePos,baseOrn);
		printf("r2d2 basePosition = [%f,%f,%f]\n", basePos[0],basePos[1],basePos[2]);
		
		sim->stepSimulation();
	}
	delete sim;
}
