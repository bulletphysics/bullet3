#include "PhysicsClientC_API.h"
#include "PhysicsClient.h"
#include "Bullet3Common/b3Scalar.h"

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
