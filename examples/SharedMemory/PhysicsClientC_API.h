#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

//#include "SharedMemoryBlock.h"
#include "SharedMemoryPublic.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);
B3_DECLARE_HANDLE(b3SharedMemoryCommandHandle);
B3_DECLARE_HANDLE(b3SharedMemoryStatusHandle);


#ifdef __cplusplus
extern "C" { 
#endif

///make sure to start the server first, before connecting client to a physics server over shared memory or UDP
b3PhysicsClientHandle b3ConnectSharedMemory(int key);

void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

///check if a command can be send
int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

//blocking submit command and wait for status
b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///non-blocking submit command
int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///non-blocking check status
b3SharedMemoryStatusHandle	b3ProcessServerStatus(b3PhysicsClientHandle physClient);

int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle);

int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle);

int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
                           int* bodyUniqueId,
                           int* numDegreeOfFreedomQ,
                           int* numDegreeOfFreedomU,
                           const double* rootLocalInertialFrame[],
                           const double* actualStateQ[],
                           const double* actualStateQdot[],
                           const double* jointReactionForces[]);

int	b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyIndex);

void	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, struct b3JointInfo* info);

b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);
    
void    b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);
    

b3SharedMemoryCommandHandle	b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
int	b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz);
int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);

b3SharedMemoryCommandHandle	b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle	b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle	b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);
///all those commands are optional, except for the *Init
int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);

///Set joint control variables such as desired position/angle, desired velocity,
///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
b3SharedMemoryCommandHandle  b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);
///Only use when controlMode is CONTROL_MODE_POSITION_VELOCITY_PD
int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
//Only use when controlMode is CONTROL_MODE_VELOCITY
int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value); /* find a better name for dof/q/u indices, point to b3JointInfo */
int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value);
///Only use if when controlMode is CONTROL_MODE_TORQUE,
int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value);
    

///the creation of collision shapes and rigid bodies etc is likely going to change,
///but good to have a b3CreateBoxShapeCommandInit for now

//create a box of size (1,1,1) at world origin (0,0,0) at orientation quat (0,0,0,1)
//after that, you can optionally adjust the initial position, orientation and size
b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient);
int	b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX,double halfExtentsY,double halfExtentsZ);
int	b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
int	b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType);
int	b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red,double green,double blue, double alpha);


///Initialize (teleport) the pose of a body/robot. You can individually set the base position, base orientation and joint angles.
///This will set all velocities of base and joints to zero.
b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
int	b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
int	b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointPosition);

b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient);
int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);

b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient,int bodyUniqueId);
void b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState *state);

b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ);
b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);

#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
