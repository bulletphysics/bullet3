#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

#include "SharedMemoryBlock.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);

#ifdef __cplusplus
extern "C" { 
#endif

b3PhysicsClientHandle b3ConnectSharedMemory(int memKey, int allowSharedMemoryInitialization);

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status);

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command);

int	b3GetNumPoweredJoints(b3PhysicsClientHandle physClient);

void	b3GetPoweredJointInfo(int linkIndex, struct PoweredJointInfo* info);

int	b3InitPhysicsParamCommand(struct SharedMemoryCommand* command);
int	b3PhysicsParamSetGravity(struct SharedMemoryCommand* command, double gravx,double gravy, double gravz);

#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
