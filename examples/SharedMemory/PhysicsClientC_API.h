#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

#include "SharedMemoryBlock.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);
B3_DECLARE_HANDLE(b3PhysicsRobotHandle);

#ifdef __cplusplus
extern "C" { 
#endif

b3PhysicsClientHandle b3ConnectSharedMemory(int allowSharedMemoryInitialization);

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status);

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command);

int	b3GetNumPoweredJoints(b3PhysicsClientHandle physClient);

void	b3GetPoweredJointInfo(b3PhysicsClientHandle physClient, int linkIndex, struct PoweredJointInfo* info);

int	b3InitPhysicsParamCommand(struct SharedMemoryCommand* command);
int	b3PhysicsParamSetGravity(struct SharedMemoryCommand* command, double gravx,double gravy, double gravz);
int	b3PhysicsParamSetTimeStep(struct SharedMemoryCommand* command, double timeStep);

int	b3InitStepSimulationCommand(struct SharedMemoryCommand* command);

int	b3LoadUrdfCommandInit(struct SharedMemoryCommand* command, const char* urdfFileName);
///all those commands are optional, except for the *Init
int	b3LoadUrdfCommandSetStartPosition(struct SharedMemoryCommand* command, double startPosX,double startPosY,double startPosZ);
int	b3LoadUrdfCommandSetStartOrientation(struct SharedMemoryCommand* command, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3LoadUrdfCommandSetUseMultiBody(struct SharedMemoryCommand* command, int useMultiBody);
int	b3LoadUrdfCommandSetUseFixedBase(struct SharedMemoryCommand* command, int useFixedBase);


#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
