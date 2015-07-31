#include "SharedMemoryBlock.h"
#include "PhysicsClientC_API.h"
#include "SharedMemoryCommands.h"

struct test
{
	int unused;
};

#include <stdio.h>
#define MAX_TIMEOUT 1024

int main(int argc, char* argv[])
{
	int ret ,allowSharedMemoryInitialization=0;
	int timeout = MAX_TIMEOUT;
	const char* urdfFileName = "r2d2.urdf";
	double gravx=0, gravy=0, gravz=0;
	double timeStep = 1./60.;
	double startPosX, startPosY,startPosZ;

	SharedMemoryCommand_t command;
	SharedMemoryStatus_t status;

	b3PhysicsClientHandle sm;
	printf("hello world\n");

	sm = b3ConnectSharedMemory( allowSharedMemoryInitialization);
	if (b3CanSubmitCommand(sm))
	{
		ret = b3InitPhysicsParamCommand(&command);
		ret = b3PhysicsParamSetGravity(&command,  gravx,gravy, gravz);
		ret = b3PhysicsParamSetTimeStep(&command,  timeStep);
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
		

		ret = b3LoadUrdfCommandInit(&command, urdfFileName);
		//setting the initial position, orientation and other arguments are optional
		startPosX =2;
		startPosY =3;
		startPosZ = 1;
		ret = b3LoadUrdfCommandSetStartPosition(&command, startPosX,startPosY,startPosZ);
		
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
		
		


		ret = b3InitStepSimulationCommand(&command);
		ret = b3SubmitClientCommand(sm, &command);
		timeout = MAX_TIMEOUT;
		while ((timeout-- > 0) && b3ProcessServerStatus(sm, &status)==0)	{}
	}
	b3DisconnectSharedMemory(sm);
}
