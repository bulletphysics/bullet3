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

///b3ConnectSharedMemory will connect to a physics server over shared memory, so
///make sure to start the server first.
///and a way to spawn an OpenGL 3D GUI physics server and connect (b3CreateInProcessPhysicsServerAndConnect)
b3PhysicsClientHandle b3ConnectSharedMemory(int key);

///b3DisconnectSharedMemory will disconnect the client from the server and cleanup memory.
void	b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

///There can only be 1 outstanding command. Check if a command can be send.
int	b3CanSubmitCommand(b3PhysicsClientHandle physClient);

///blocking submit command and wait for status
b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///In general it is better to use b3SubmitClientCommandAndWaitStatus. b3SubmitClientCommand is a non-blocking submit
///command, which requires checking for the status manually, using b3ProcessServerStatus. Also, before sending the
///next command, make sure to check if you can send a command using 'b3CanSubmitCommand'.
int	b3SubmitClientCommand(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

///non-blocking check status
b3SharedMemoryStatusHandle	b3ProcessServerStatus(b3PhysicsClientHandle physClient);

/// Get the physics server return status type. See EnumSharedMemoryServerStatus in SharedMemoryPublic.h for error codes.
int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle);

int b3GetStatusBodyIndices(b3SharedMemoryStatusHandle statusHandle, int* bodyIndicesOut, int bodyIndicesCapacity);

int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle);

int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
                           int* bodyUniqueId,
                           int* numDegreeOfFreedomQ,
                           int* numDegreeOfFreedomU,
                           const double* rootLocalInertialFrame[],
                           const double* actualStateQ[],
                           const double* actualStateQdot[],
                           const double* jointReactionForces[]);

///return the total number of bodies in the simulation
int	b3GetNumBodies(b3PhysicsClientHandle physClient);

/// return the body unique id, given the index in range [0 , b3GetNumBodies() )
int b3GetBodyUniqueId(b3PhysicsClientHandle physClient, int serialIndex);

///given a body unique id, return the body information. See b3BodyInfo in SharedMemoryPublic.h
int b3GetBodyInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, struct b3BodyInfo* info);

///give a unique body index (after loading the body) return the number of joints.
int	b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyIndex);

///given a body and joint index, return the joint information. See b3JointInfo in SharedMemoryPublic.h
int	b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyIndex, int jointIndex, struct b3JointInfo* info);
    
b3SharedMemoryCommandHandle b3CreateJoint(b3PhysicsClientHandle physClient, int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, struct b3JointInfo* info);

///Request debug lines for debug visualization. The flags in debugMode are the same as used in Bullet
///See btIDebugDraw::DebugDrawModes in Bullet/src/LinearMath/btIDebugDraw.h
b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);

///Get the pointers to the debug line information, after b3InitRequestDebugLinesCommand returns
///status CMD_DEBUG_LINES_COMPLETED
void    b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);

///request an image from a simulated camera, using a software renderer.
b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient);
void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle command, float viewMatrix[16], float projectionMatrix[16]);
void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle command, const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3]);
void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis);
void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle command, float left, float right, float bottom, float top, float nearVal, float farVal);
void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle command, float fov, float aspect, float nearVal, float farVal);
void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle command, int width, int height );
void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer);
void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData);

///request an contact point information
b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient);
void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointData);

b3SharedMemoryCommandHandle	b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
int	b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz);
int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
int	b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
int	b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);



b3SharedMemoryCommandHandle	b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle	b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);

///Load a robot from a URDF file. Status type will CMD_URDF_LOADING_COMPLETED.
///Access the robot from the unique body index, through b3GetStatusBodyIndex(statusHandle);
b3SharedMemoryCommandHandle	b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);

int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);


///compute the forces to achieve an acceleration, given a state q and qdot using inverse dynamics
b3SharedMemoryCommandHandle	b3CalculateInverseDynamicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex,
	const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);

int b3GetStatusInverseDynamicsJointForces(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointForces);

b3SharedMemoryCommandHandle	b3CalculateJacobianCommandInit(b3PhysicsClientHandle physClient, int bodyIndex, int linkIndex, const double* localPosition, const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);

int b3GetStatusJacobian(b3SharedMemoryStatusHandle statusHandle, double* linearJacobian, double* angularJacobian);

///compute the joint positions to move the end effector to a desired target using inverse kinematics
b3SharedMemoryCommandHandle	b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
void b3CalculateInverseKinematicsAddTargetPurePosition(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3]);
void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4]);
void b3CalculateInverseKinematicsPosWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[3], const double targetOrientation[4], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointPositions);

b3SharedMemoryCommandHandle	b3LoadSdfCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);
int	b3LoadSdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);

b3SharedMemoryCommandHandle	b3SaveWorldCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);


///The b3JointControlCommandInit method is obsolete, use b3JointControlCommandInit2 instead
b3SharedMemoryCommandHandle  b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);

    
///Set joint motor control variables such as desired position/angle, desired velocity,
///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
b3SharedMemoryCommandHandle  b3JointControlCommandInit2(b3PhysicsClientHandle physClient, int bodyUniqueId, int controlMode);

///Only use when controlMode is CONTROL_MODE_POSITION_VELOCITY_PD
int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);

///Only use when controlMode is CONTROL_MODE_VELOCITY
int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value); /* find a better name for dof/q/u indices, point to b3JointInfo */
int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value);
///Only use if when controlMode is CONTROL_MODE_TORQUE,
int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value);
    

///the creation of collision shapes and rigid bodies etc is likely going to change,
///but good to have a b3CreateBoxShapeCommandInit for now

///create a box of size (1,1,1) at world origin (0,0,0) at orientation quat (0,0,0,1)
///after that, you can optionally adjust the initial position, orientation and size
b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient);
int	b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX,double halfExtentsY,double halfExtentsZ);
int	b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
int	b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType);
int	b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red,double green,double blue, double alpha);


///b3CreatePoseCommandInit will initialize (teleport) the pose of a body/robot. You can individually set the base position,
///base orientation and joint angles. This will set all velocities of base and joints to zero.
///This is not a robot control command using actuators/joint motors, but manual repositioning the robot.
b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
int	b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
int	b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointPosition);

///We are currently not reading the sensor information from the URDF file, and programmatically assign sensors.
///This is rather inconsistent, to mix programmatical creation with loading from file.
b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
///b3CreateSensorEnableIMUForLink is not implemented yet.
///For now, if the IMU is located in the root link, use the root world transform to mimic an IMU.
int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);

b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient,int bodyUniqueId);
void b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState *state);
void b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, struct b3LinkState *state);

b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ);
b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);

/// Apply external force at the body (or link) center of mass, in world space/Cartesian coordinates.
b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient);
void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[3], const double position[3], int flags);
void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[3], int flags);

#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
