#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

//#include "SharedMemoryBlock.h"
#include "SharedMemoryPublic.h"

#define B3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);
B3_DECLARE_HANDLE(b3SharedMemoryCommandHandle);
B3_DECLARE_HANDLE(b3SharedMemoryStatusHandle);


///There are several connection methods, see following header files:
#include "PhysicsClientSharedMemory_C_API.h"
#include "PhysicsClientSharedMemory2_C_API.h"
#include "PhysicsDirectC_API.h"
#include "PhysicsClientUDP_C_API.h"
#include "SharedMemoryInProcessPhysicsC_API.h"

#ifdef __cplusplus
extern "C" { 
#endif



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


b3SharedMemoryCommandHandle b3RequestCollisionInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3GetStatusAABB(b3SharedMemoryStatusHandle statusHandle, int linkIndex, double aabbMin[3], double aabbMax[3]);

///If you re-connected to an existing server, or server changed otherwise, sync the body info and user constraints etc.
b3SharedMemoryCommandHandle b3InitSyncBodyInfoCommand(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle b3InitRemoveBodyCommand(b3PhysicsClientHandle physClient, int bodyUniqueId);

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

b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
///given a body unique id and link index, return the dynamics information. See b3DynamicsInfo in SharedMemoryPublic.h
int b3GetDynamicsInfo(b3SharedMemoryStatusHandle statusHandle, struct b3DynamicsInfo* info);
	
b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo(b3PhysicsClientHandle physClient);
int b3ChangeDynamicsInfoSetMass(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double mass);
int b3ChangeDynamicsInfoSetLateralFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction);
int b3ChangeDynamicsInfoSetSpinningFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
int b3ChangeDynamicsInfoSetRollingFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
int b3ChangeDynamicsInfoSetRestitution(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double restitution);
int b3ChangeDynamicsInfoSetLinearDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double linearDamping);
int b3ChangeDynamicsInfoSetAngularDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId,double angularDamping);
int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex,double contactStiffness, double contactDamping);
int b3ChangeDynamicsInfoSetFrictionAnchor(b3SharedMemoryCommandHandle commandHandle,int bodyUniqueId,int linkIndex, int frictionAnchor);

b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand(b3PhysicsClientHandle physClient, int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, struct b3JointInfo* info);

///return a unique id for the user constraint, after successful creation, or -1 for an invalid constraint id
int b3GetStatusUserConstraintUniqueId(b3SharedMemoryStatusHandle statusHandle);

///change parameters of an existing user constraint
b3SharedMemoryCommandHandle  b3InitChangeUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);
int b3InitChangeUserConstraintSetPivotInB(b3SharedMemoryCommandHandle commandHandle, double jointChildPivot[3]);
int b3InitChangeUserConstraintSetFrameInB(b3SharedMemoryCommandHandle commandHandle, double jointChildFrameOrn[4]);
int b3InitChangeUserConstraintSetMaxForce(b3SharedMemoryCommandHandle commandHandle, double maxAppliedForce);
int b3InitChangeUserConstraintSetGearRatio(b3SharedMemoryCommandHandle commandHandle, double gearRatio);


b3SharedMemoryCommandHandle  b3InitRemoveUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);

int b3GetNumUserConstraints(b3PhysicsClientHandle physClient);
int b3GetUserConstraintInfo(b3PhysicsClientHandle physClient, int constraintUniqueId, struct b3UserConstraint* info);
/// return the user constraint id, given the index in range [0 , b3GetNumUserConstraints() )
int b3GetUserConstraintId(b3PhysicsClientHandle physClient, int serialIndex);
    
///Request physics debug lines for debug visualization. The flags in debugMode are the same as used in Bullet
///See btIDebugDraw::DebugDrawModes in Bullet/src/LinearMath/btIDebugDraw.h
b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);

///Get the pointers to the physics debug line information, after b3InitRequestDebugLinesCommand returns
///status CMD_DEBUG_LINES_COMPLETED
void    b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);
    
///configure the 3D OpenGL debug visualizer (enable/disable GUI widgets, shadows, position camera etc)
b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer(b3PhysicsClientHandle physClient);
void b3ConfigureOpenGLVisualizerSetVisualizationFlags(b3SharedMemoryCommandHandle commandHandle, int flag, int enabled);
void b3ConfigureOpenGLVisualizerSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, const float cameraTargetPosition[3]);

b3SharedMemoryCommandHandle b3InitRequestOpenGLVisualizerCameraCommand(b3PhysicsClientHandle physClient);
int b3GetStatusOpenGLVisualizerCamera(b3SharedMemoryStatusHandle statusHandle, struct b3OpenGLVisualizerCameraInfo* camera);

    
/// Add/remove user-specific debug lines and debug text messages
b3SharedMemoryCommandHandle b3InitUserDebugDrawAddLine3D(b3PhysicsClientHandle physClient, double fromXYZ[3], double toXYZ[3], double colorRGB[3], double lineWidth, double lifeTime);

b3SharedMemoryCommandHandle b3InitUserDebugDrawAddText3D(b3PhysicsClientHandle physClient, const char* txt, double positionXYZ[3], double colorRGB[3], double textSize, double lifeTime);
void b3UserDebugTextSetOptionFlags(b3SharedMemoryCommandHandle commandHandle, int optionFlags);
void b3UserDebugTextSetOrientation(b3SharedMemoryCommandHandle commandHandle, double orientation[4]);

void b3UserDebugItemSetParentObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

b3SharedMemoryCommandHandle b3InitUserDebugAddParameter(b3PhysicsClientHandle physClient, const char* txt, double rangeMin, double rangeMax, double startValue);
b3SharedMemoryCommandHandle b3InitUserDebugReadParameter(b3PhysicsClientHandle physClient, int debugItemUniqueId);
int b3GetStatusDebugParameterValue(b3SharedMemoryStatusHandle statusHandle, double* paramValue);

b3SharedMemoryCommandHandle b3InitUserDebugDrawRemove(b3PhysicsClientHandle physClient, int debugItemUniqueId);
b3SharedMemoryCommandHandle b3InitUserDebugDrawRemoveAll(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle b3InitDebugDrawingCommand(b3PhysicsClientHandle physClient);
void b3SetDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex, double objectColorRGB[3]);
void b3RemoveDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

///All debug items unique Ids are positive: a negative unique Id means failure.
int b3GetDebugItemUniqueId(b3SharedMemoryStatusHandle statusHandle);


///request an image from a simulated camera, using a software renderer.
b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient);
void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle command, float viewMatrix[16], float projectionMatrix[16]);
void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle command, int width, int height );
void b3RequestCameraImageSetLightDirection(b3SharedMemoryCommandHandle commandHandle, const float lightDirection[3]);
void b3RequestCameraImageSetLightColor(b3SharedMemoryCommandHandle commandHandle, const float lightColor[3]);
void b3RequestCameraImageSetLightDistance(b3SharedMemoryCommandHandle commandHandle, float lightDistance);
void b3RequestCameraImageSetLightAmbientCoeff(b3SharedMemoryCommandHandle commandHandle, float lightAmbientCoeff);
void b3RequestCameraImageSetLightDiffuseCoeff(b3SharedMemoryCommandHandle commandHandle, float lightDiffuseCoeff);
void b3RequestCameraImageSetLightSpecularCoeff(b3SharedMemoryCommandHandle commandHandle, float lightSpecularCoeff);
void b3RequestCameraImageSetShadow(b3SharedMemoryCommandHandle commandHandle, int hasShadow);
void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer);
void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData);

///compute a view matrix, helper function for b3RequestCameraImageSetCameraMatrices
void b3ComputeViewMatrixFromPositions(const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3], float viewMatrix[16]);
void b3ComputeViewMatrixFromYawPitchRoll(const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis, float viewMatrix[16]);
void b3ComputePositionFromViewMatrix(const float viewMatrix[16], float cameraPosition[3], float cameraTargetPosition[3], float cameraUp[3]);

///compute a projection matrix, helper function for b3RequestCameraImageSetCameraMatrices
void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, float projectionMatrix[16]);
void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float projectionMatrix[16]);


/* obsolete, please use b3ComputeViewProjectionMatrices */
void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle command, const float cameraPosition[3], const float cameraTargetPosition[3], const float cameraUp[3]);
/* obsolete, please use b3ComputeViewProjectionMatrices */
void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[3], float distance, float yaw, float pitch, float roll, int upAxis);
/* obsolete, please use b3ComputeViewProjectionMatrices */
void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle command, float left, float right, float bottom, float top, float nearVal, float farVal);
/* obsolete, please use b3ComputeViewProjectionMatrices */
void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle command, float fov, float aspect, float nearVal, float farVal);


///request an contact point information
b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient);
void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
void b3SetContactFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
void b3SetContactFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);

///compute the closest points between two bodies
b3SharedMemoryCommandHandle b3InitClosestDistanceQuery(b3PhysicsClientHandle physClient);
void b3SetClosestDistanceFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
void b3SetClosestDistanceFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
void b3SetClosestDistanceFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
void b3SetClosestDistanceFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
void b3SetClosestDistanceThreshold(b3SharedMemoryCommandHandle commandHandle, double distance);
void b3GetClosestPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);

///get all the bodies that touch a given axis aligned bounding box specified in world space (min and max coordinates)
b3SharedMemoryCommandHandle b3InitAABBOverlapQuery(b3PhysicsClientHandle physClient, const double aabbMin[3],const double aabbMax[3]);
void b3GetAABBOverlapResults(b3PhysicsClientHandle physClient, struct b3AABBOverlapData* data);

//request visual shape information
b3SharedMemoryCommandHandle b3InitRequestVisualShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueIdA);
void b3GetVisualShapeInformation(b3PhysicsClientHandle physClient, struct b3VisualShapeInformation* visualShapeInfo);

b3SharedMemoryCommandHandle b3InitLoadTexture(b3PhysicsClientHandle physClient, const char* filename);
b3SharedMemoryCommandHandle b3InitUpdateVisualShape(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);
void b3UpdateVisualShapeRGBAColor(b3SharedMemoryCommandHandle commandHandle, double rgbaColor[4]);
void b3UpdateVisualShapeSpecularColor(b3SharedMemoryCommandHandle commandHandle, double specularColor[3]);


b3SharedMemoryCommandHandle	b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
int	b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx,double gravy, double gravz);
int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
int	b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
int	b3PhysicsParamSetDefaultNonContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultNonContactERP);
int	b3PhysicsParamSetDefaultFrictionERP(b3SharedMemoryCommandHandle commandHandle, double frictionERP);

int	b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);
int b3PhysicsParamSetCollisionFilterMode(b3SharedMemoryCommandHandle commandHandle, int filterMode);


	
int b3PhysicsParamSetUseSplitImpulse(b3SharedMemoryCommandHandle commandHandle, int useSplitImpulse);
int b3PhysicsParamSetSplitImpulsePenetrationThreshold(b3SharedMemoryCommandHandle commandHandle, double splitImpulsePenetrationThreshold);
int b3PhysicsParamSetContactBreakingThreshold(b3SharedMemoryCommandHandle commandHandle, double contactBreakingThreshold);
int b3PhysicsParamSetMaxNumCommandsPer1ms(b3SharedMemoryCommandHandle commandHandle, int maxNumCmdPer1ms);
int b3PhysicsParamSetEnableFileCaching(b3SharedMemoryCommandHandle commandHandle, int enableFileCaching);
int b3PhysicsParamSetRestitutionVelocityThreshold(b3SharedMemoryCommandHandle commandHandle, double restitutionVelocityThreshold);



//b3PhysicsParamSetInternalSimFlags is for internal/temporary/easter-egg/experimental demo purposes
//Use at own risk: magic things may or my not happen when calling this API
int b3PhysicsParamSetInternalSimFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


b3SharedMemoryCommandHandle	b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle	b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);

///Load a robot from a URDF file. Status type will CMD_URDF_LOADING_COMPLETED.
///Access the robot from the unique body index, through b3GetStatusBodyIndex(statusHandle);
b3SharedMemoryCommandHandle	b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);

int	b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
int	b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
int	b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
int	b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);
int	b3LoadUrdfCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);

b3SharedMemoryCommandHandle	b3LoadBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
b3SharedMemoryCommandHandle	b3SaveBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
b3SharedMemoryCommandHandle	b3LoadMJCFCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
void b3LoadMJCFCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);


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
void b3CalculateInverseKinematicsSetJointDamping(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* jointDampingCoeff);
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


b3SharedMemoryCommandHandle b3CreateCollisionShapeCommandInit(b3PhysicsClientHandle physClient);
int b3CreateCollisionShapeAddSphere(b3SharedMemoryCommandHandle commandHandle,double radius);
int b3CreateCollisionShapeAddBox(b3SharedMemoryCommandHandle commandHandle,double halfExtents[3]);
int b3CreateCollisionShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
int b3CreateCollisionShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle,double radius, double height);
int b3CreateCollisionShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, double planeNormal[3], double planeConstant);
int b3CreateCollisionShapeAddMesh(b3SharedMemoryCommandHandle commandHandle,const char* fileName, double meshScale[3]);
void b3CreateCollisionSetFlag(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, int flags);

void b3CreateCollisionShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle,int shapeIndex, double childPosition[3], double childOrientation[4]);

int b3GetStatusCollisionShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

b3SharedMemoryCommandHandle b3CreateVisualShapeCommandInit(b3PhysicsClientHandle physClient);
int b3GetStatusVisualShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

b3SharedMemoryCommandHandle b3CreateMultiBodyCommandInit(b3PhysicsClientHandle physClient);
int b3CreateMultiBodyBase(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, double basePosition[3], double baseOrientation[4]);

int b3CreateMultiBodyLink(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex, 
									double linkVisualShapeIndex, 
									double linkPosition[3], 
									double linkOrientation[4],
									double linkInertialFramePosition[3],
									double linkInertialFrameOrientation[4],
									int linkParentIndex,
									int linkJointType,
									double linkJointAxis[3]);


//useMaximalCoordinates are disabled by default, enabling them is experimental and not fully supported yet
void b3CreateMultiBodyUseMaximalCoordinates(b3SharedMemoryCommandHandle commandHandle);

//int b3CreateMultiBodyAddLink(b3SharedMemoryCommandHandle commandHandle, int jointType, int parentLinkIndex, double linkMass, int linkCollisionShapeUnique, int linkVisualShapeUniqueId);



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
int	b3CreatePoseCommandSetBaseLinearVelocity(b3SharedMemoryCommandHandle commandHandle, double linVel[3]);
int	b3CreatePoseCommandSetBaseAngularVelocity(b3SharedMemoryCommandHandle commandHandle, double angVel[3]);

int	b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
int	b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointPosition);

int	b3CreatePoseCommandSetJointVelocities(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int numJointVelocities, const double* jointVelocities);
int	b3CreatePoseCommandSetJointVelocity(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointVelocity);

///We are currently not reading the sensor information from the URDF file, and programmatically assign sensors.
///This is rather inconsistent, to mix programmatical creation with loading from file.
b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
///b3CreateSensorEnableIMUForLink is not implemented yet.
///For now, if the IMU is located in the root link, use the root world transform to mimic an IMU.
int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);

b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient,int bodyUniqueId);
int b3RequestActualStateCommandComputeLinkVelocity(b3SharedMemoryCommandHandle commandHandle, int computeLinkVelocity);

int b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState *state);
int b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, struct b3LinkState *state);

b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);
b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                             double rayFromWorldY, double rayFromWorldZ,
                                             double rayToWorldX, double rayToWorldY,
                                             double rayToWorldZ);
b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);

b3SharedMemoryCommandHandle b3CreateRaycastCommandInit(b3PhysicsClientHandle physClient, double rayFromWorldX,
                                       double rayFromWorldY, double rayFromWorldZ,
                                       double rayToWorldX, double rayToWorldY, double rayToWorldZ);

b3SharedMemoryCommandHandle b3CreateRaycastBatchCommandInit(b3PhysicsClientHandle physClient);
void b3RaycastBatchAddRay(b3SharedMemoryCommandHandle commandHandle, const double rayFromWorld[3], const double rayToWorld[3]);

void b3GetRaycastInformation(b3PhysicsClientHandle physClient, struct b3RaycastInformation* raycastInfo);



/// Apply external force at the body (or link) center of mass, in world space/Cartesian coordinates.
b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient);
void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[3], const double position[3], int flags);
void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[3], int flags);

///experiments of robots interacting with non-rigid objects (such as btSoftBody)
b3SharedMemoryCommandHandle	b3LoadBunnyCommandInit(b3PhysicsClientHandle physClient);
int b3LoadBunnySetScale(b3SharedMemoryCommandHandle commandHandle, double scale);
int b3LoadBunnySetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
int b3LoadBunnySetCollisionMargin(b3SharedMemoryCommandHandle commandHandle, double collisionMargin);


b3SharedMemoryCommandHandle	b3RequestVREventsCommandInit(b3PhysicsClientHandle physClient);
void b3VREventsSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);

void b3GetVREventsData(b3PhysicsClientHandle physClient, struct b3VREventsData* vrEventsData);

b3SharedMemoryCommandHandle	b3SetVRCameraStateCommandInit(b3PhysicsClientHandle physClient);
int b3SetVRCameraRootPosition(b3SharedMemoryCommandHandle commandHandle, double rootPos[3]);
int b3SetVRCameraRootOrientation(b3SharedMemoryCommandHandle commandHandle, double rootOrn[4]);
int b3SetVRCameraTrackingObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
int b3SetVRCameraTrackingObjectFlag(b3SharedMemoryCommandHandle commandHandle, int flag);

b3SharedMemoryCommandHandle	b3RequestKeyboardEventsCommandInit(b3PhysicsClientHandle physClient);
void b3GetKeyboardEventsData(b3PhysicsClientHandle physClient, struct b3KeyboardEventsData* keyboardEventsData);

b3SharedMemoryCommandHandle	b3RequestMouseEventsCommandInit(b3PhysicsClientHandle physClient);
void b3GetMouseEventsData(b3PhysicsClientHandle physClient, struct b3MouseEventsData* mouseEventsData);


b3SharedMemoryCommandHandle	b3StateLoggingCommandInit(b3PhysicsClientHandle physClient);
int b3StateLoggingStart(b3SharedMemoryCommandHandle commandHandle, int loggingType, const char* fileName);
int b3StateLoggingAddLoggingObjectUniqueId(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
int b3StateLoggingSetMaxLogDof(b3SharedMemoryCommandHandle commandHandle, int maxLogDof);
int b3StateLoggingSetLinkIndexA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
int b3StateLoggingSetLinkIndexB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
int b3StateLoggingSetBodyAUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyAUniqueId);
int b3StateLoggingSetBodyBUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyBUniqueId);
int b3StateLoggingSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);

int b3GetStatusLoggingUniqueId(b3SharedMemoryStatusHandle statusHandle);
int b3StateLoggingStop(b3SharedMemoryCommandHandle commandHandle, int loggingUniqueId);


b3SharedMemoryCommandHandle	b3ProfileTimingCommandInit(b3PhysicsClientHandle physClient, const char* name);
void b3SetProfileTimingDuractionInMicroSeconds(b3SharedMemoryCommandHandle commandHandle, int duration);

void b3SetTimeOut(b3PhysicsClientHandle physClient, double timeOutInSeconds);
double b3GetTimeOut(b3PhysicsClientHandle physClient);

void b3MultiplyTransforms(const double posA[3], const double ornA[4], const double posB[3], const double ornB[4], double outPos[3], double outOrn[4]);
void b3InvertTransform(const double pos[3], const double orn[4], double outPos[3], double outOrn[4]);

#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_C_API_H
