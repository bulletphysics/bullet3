#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

#include "SharedMemoryBlock.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);
B3_DECLARE_HANDLE(b3PhysicsRobotHandle);

#ifdef __cplusplus
extern "C" { 
#endif

///make sure to start the server first!
b3PhysicsClientHandle b3ConnectSharedMemory();

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

int	b3ProcessServerStatus(b3PhysicsClientHandle physClient, struct SharedMemoryStatus* status);

int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, struct SharedMemoryCommand* command);

int	b3GetNumJoints(b3PhysicsClientHandle physClient);

void	b3GetJointInfo(b3PhysicsClientHandle physClient, int linkIndex, struct b3JointInfo* info);

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

///Set joint control variables such as desired position/angle, desired velocity,
///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
int b3JointControlCommandInit(struct SharedMemoryCommand* command, int controlMode);
//Only use when controlMode is CONTROL_MODE_VELOCITY
int b3JointControlSetDesiredVelocity(struct SharedMemoryCommand* command, int dofIndex, double value);
int b3JointControlSetMaximumForce(struct SharedMemoryCommand* command, int dofIndex,  double value);
///Only use if when controlMode is CONTROL_MODE_TORQUE,
int b3JointControlSetDesiredForceTorque(struct SharedMemoryCommand* command, int  dofIndex, double value);
    

///the creation of collision shapes and rigid bodies etc is likely going to change,
///but good to have a b3CreateBoxShapeCommandInit for now

//create a box of size (1,1,1) at world origin (0,0,0) at orientation quat (0,0,0,1)
//after that, you can optionally adjust the initial position, orientation and size
int b3CreateBoxShapeCommandInit(struct SharedMemoryCommand* command);
int	b3CreateBoxCommandSetStartPosition(struct SharedMemoryCommand* command, double startPosX,double startPosY,double startPosZ);
int	b3CreateBoxCommandSetStartOrientation(struct SharedMemoryCommand* command, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3CreateBoxCommandSetHalfExtents(struct SharedMemoryCommand* command, double halfExtentsX,double halfExtentsY,double halfExtentsZ);
    

    int b3CreateSensorCommandInit(struct SharedMemoryCommand* command);
int b3CreateSensorEnable6DofJointForceTorqueSensor(struct SharedMemoryCommand* command, int dofIndex, int enable);


int b3RequestActualStateCommandInit(struct SharedMemoryCommand* command);


#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
