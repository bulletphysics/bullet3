#include "PhysicsClientC_API.h"
#include "PhysicsClient.h"
#include "Bullet3Common/b3Scalar.h"
#include <string.h>

int	b3LoadUrdfCommandInit(struct SharedMemoryCommand* command, const char* urdfFileName)
{
	b3Assert(command);
	command->m_type = CMD_LOAD_URDF;
	int len = strlen(urdfFileName);
	if (len<MAX_URDF_FILENAME_LENGTH)
	{
		strcpy(command->m_urdfArguments.m_urdfFileName,urdfFileName);
	} else
	{
		command->m_urdfArguments.m_urdfFileName[0] = 0;
	}
	command->m_updateFlags = URDF_ARGS_FILE_NAME;
	
	return 0;
}




int	b3LoadUrdfCommandSetStartPosition(struct SharedMemoryCommand* command, double startPosX,double startPosY,double startPosZ)
{
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialPosition[0] = startPosX;
	command->m_urdfArguments.m_initialPosition[1] = startPosY;
	command->m_urdfArguments.m_initialPosition[2] = startPosZ;
	command->m_updateFlags|=URDF_ARGS_INITIAL_POSITION;
	return 0;
}
int	b3LoadUrdfCommandSetStartOrientation(struct SharedMemoryCommand* command, double startOrnX,double startOrnY,double startOrnZ, double startOrnW)
{
	b3Assert(command);
	b3Assert(command->m_type == CMD_LOAD_URDF);
	command->m_urdfArguments.m_initialOrientation[0] = startOrnX;
	command->m_urdfArguments.m_initialOrientation[1] = startOrnY;
	command->m_urdfArguments.m_initialOrientation[2] = startOrnZ;
	command->m_urdfArguments.m_initialOrientation[3] = startOrnW;
	command->m_updateFlags|=URDF_ARGS_INITIAL_ORIENTATION;
	return 0;
}

int     b3InitPhysicsParamCommand(struct SharedMemoryCommand* command)
{
	b3Assert(command);
	command->m_type = CMD_SEND_PHYSICS_SIMULATION_PARAMETERS;
	command->m_updateFlags = 0;

	return 0;
}
int     b3PhysicsParamSetGravity(struct SharedMemoryCommand* command, double gravx,double gravy, double gravz)
{
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_gravityAcceleration[0] = gravx;
	command->m_physSimParamArgs.m_gravityAcceleration[1] = gravy;
	command->m_physSimParamArgs.m_gravityAcceleration[2] = gravz; 
	command->m_updateFlags |= SIM_PARAM_UPDATE_GRAVITY;
	return 0;
}

int	b3PhysicsParamSetTimeStep(struct SharedMemoryCommand* command, double timeStep)
{
	b3Assert(command->m_type == CMD_SEND_PHYSICS_SIMULATION_PARAMETERS);
	command->m_physSimParamArgs.m_deltaTime = timeStep;
	return 0;
}

int	b3InitStepSimulationCommand(struct SharedMemoryCommand* command)
{
	b3Assert(command);
	command->m_type = CMD_STEP_FORWARD_SIMULATION;
	command->m_updateFlags = 0;

	return 0;

}

b3PhysicsClientHandle b3ConnectSharedMemory( int allowSharedMemoryInitialization)
{
	PhysicsClientSharedMemory* cl = new PhysicsClientSharedMemory();
	cl->connect(allowSharedMemoryInitialization);
	return (b3PhysicsClientHandle ) cl;
}

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	delete cl;
}

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return (int)cl->processServerStatus(*status);
}

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return (int)cl->canSubmitCommand();
}

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command)
{
		PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
		return (int)cl->submitClientCommand(*command);
}

int	b3GetNumJoints(b3PhysicsClientHandle physClient)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	return cl->getNumPoweredJoints();
}

void	b3GetPoweredJointInfo(b3PhysicsClientHandle physClient, int linkIndex, struct PoweredJointInfo* info)
{
	PhysicsClientSharedMemory* cl = (PhysicsClientSharedMemory* ) physClient;
	cl->getPoweredJointInfo(linkIndex,*info);
}

#if 0

#include "SharedMemoryBlock.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);

b3PhysicsClientHandle b3ConnectSharedMemory(int memKey, int allowSharedMemoryInitialization);

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status);

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command);

int	b3GetNumPoweredJoints(b3PhysicsClientHandle physClient);

void	b3GetPoweredJointInfo(int linkIndex, struct PoweredJointInfo* info);

int	b3InitPhysicsParamCommand(struct SharedMemoryCommand* command);
int	b3PhysicsParamSetGravity(struct SharedMemoryCommand* command, double gravx,double gravy, double gravz);

#endif
