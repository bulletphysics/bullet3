#ifndef PHYSICS_CLIENT_C_API_H
#define PHYSICS_CLIENT_C_API_H

//#include "SharedMemoryBlock.h"
#include "SharedMemoryPublic.h"

#define B3_DECLARE_HANDLE(name) \
	typedef struct name##__     \
	{                           \
		int unused;             \
	} * name

B3_DECLARE_HANDLE(b3PhysicsClientHandle);
B3_DECLARE_HANDLE(b3SharedMemoryCommandHandle);
B3_DECLARE_HANDLE(b3SharedMemoryStatusHandle);

#ifdef _WIN32
#define B3_SHARED_API __declspec(dllexport)
#elif defined(__GNUC__)
#define B3_SHARED_API __attribute__((visibility("default")))
#else
#define B3_SHARED_API
#endif

///There are several connection methods, see following header files:
#include "PhysicsClientSharedMemory_C_API.h"
#include "PhysicsClientSharedMemory2_C_API.h"
#include "PhysicsDirectC_API.h"

#ifdef BT_ENABLE_ENET
#include "PhysicsClientUDP_C_API.h"
#endif

#ifdef BT_ENABLE_CLSOCKET
#include "PhysicsClientTCP_C_API.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

	///b3DisconnectSharedMemory will disconnect the client from the server and cleanup memory.
	B3_SHARED_API void b3DisconnectSharedMemory(b3PhysicsClientHandle physClient);

	///There can only be 1 outstanding command. Check if a command can be send.
	B3_SHARED_API int b3CanSubmitCommand(b3PhysicsClientHandle physClient);

	///blocking submit command and wait for status
	B3_SHARED_API b3SharedMemoryStatusHandle b3SubmitClientCommandAndWaitStatus(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

	///In general it is better to use b3SubmitClientCommandAndWaitStatus. b3SubmitClientCommand is a non-blocking submit
	///command, which requires checking for the status manually, using b3ProcessServerStatus. Also, before sending the
	///next command, make sure to check if you can send a command using 'b3CanSubmitCommand'.
	B3_SHARED_API int b3SubmitClientCommand(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle);

	///non-blocking check status
	B3_SHARED_API b3SharedMemoryStatusHandle b3ProcessServerStatus(b3PhysicsClientHandle physClient);

	/// Get the physics server return status type. See EnumSharedMemoryServerStatus in SharedMemoryPublic.h for error codes.
	B3_SHARED_API int b3GetStatusType(b3SharedMemoryStatusHandle statusHandle);

	///Plugin system, load and unload a plugin, execute a command
	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateCustomCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3CustomCommandLoadPlugin(b3SharedMemoryCommandHandle commandHandle, const char* pluginPath);
	B3_SHARED_API void b3CustomCommandLoadPluginSetPostFix(b3SharedMemoryCommandHandle commandHandle, const char* postFix);
	B3_SHARED_API int b3GetStatusPluginUniqueId(b3SharedMemoryStatusHandle statusHandle);
	B3_SHARED_API int b3GetStatusPluginCommandResult(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API void b3CustomCommandUnloadPlugin(b3SharedMemoryCommandHandle commandHandle, int pluginUniqueId);
	B3_SHARED_API void b3CustomCommandExecutePluginCommand(b3SharedMemoryCommandHandle commandHandle, int pluginUniqueId, const char* textArguments);
	B3_SHARED_API void b3CustomCommandExecuteAddIntArgument(b3SharedMemoryCommandHandle commandHandle, int intVal);
	B3_SHARED_API void b3CustomCommandExecuteAddFloatArgument(b3SharedMemoryCommandHandle commandHandle, float floatVal);

	B3_SHARED_API int b3GetStatusBodyIndices(b3SharedMemoryStatusHandle statusHandle, int* bodyIndicesOut, int bodyIndicesCapacity);

	B3_SHARED_API int b3GetStatusBodyIndex(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API int b3GetStatusActualState(b3SharedMemoryStatusHandle statusHandle,
											 int* bodyUniqueId,
											 int* numDegreeOfFreedomQ,
											 int* numDegreeOfFreedomU,
											 const double* rootLocalInertialFrame[],
											 const double* actualStateQ[],
											 const double* actualStateQdot[],
											 const double* jointReactionForces[]);

	B3_SHARED_API int b3GetStatusActualState2(b3SharedMemoryStatusHandle statusHandle,
											  int* bodyUniqueId,
											  int* numLinks,
											  int* numDegreeOfFreedomQ,
											  int* numDegreeOfFreedomU,
											  const double* rootLocalInertialFrame[],
											  const double* actualStateQ[],
											  const double* actualStateQdot[],
											  const double* jointReactionForces[],
											  const double* linkLocalInertialFrames[],
											  const double* jointMotorForces[],
											  const double* linkStates[],
											  const double* linkWorldVelocities[]);

	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestCollisionInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API int b3GetStatusAABB(b3SharedMemoryStatusHandle statusHandle, int linkIndex, double aabbMin[/*3*/], double aabbMax[/*3*/]);

	///If you re-connected to an existing server, or server changed otherwise, sync the body info and user constraints etc.
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitSyncBodyInfoCommand(b3PhysicsClientHandle physClient);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRemoveBodyCommand(b3PhysicsClientHandle physClient, int bodyUniqueId);

	///return the total number of bodies in the simulation
	B3_SHARED_API int b3GetNumBodies(b3PhysicsClientHandle physClient);

	/// return the body unique id, given the index in range [0 , b3GetNumBodies() )
	B3_SHARED_API int b3GetBodyUniqueId(b3PhysicsClientHandle physClient, int serialIndex);

	///given a body unique id, return the body information. See b3BodyInfo in SharedMemoryPublic.h
	B3_SHARED_API int b3GetBodyInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, struct b3BodyInfo* info);

	///give a unique body index (after loading the body) return the number of joints.
	B3_SHARED_API int b3GetNumJoints(b3PhysicsClientHandle physClient, int bodyUniqueId);

	///compute the number of degrees of freedom for this body.
	///Return -1 for unsupported spherical joint, -2 for unsupported planar joint.
	B3_SHARED_API int b3ComputeDofCount(b3PhysicsClientHandle physClient, int bodyUniqueId);

	///given a body and joint index, return the joint information. See b3JointInfo in SharedMemoryPublic.h
	B3_SHARED_API int b3GetJointInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, struct b3JointInfo* info);

	///user data handling
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitSyncUserDataCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitAddUserDataCommand(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, const char* key, enum UserDataValueType valueType, int valueLength, const void* valueData);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRemoveUserDataCommand(b3PhysicsClientHandle physClient, int userDataId);

	B3_SHARED_API int b3GetUserData(b3PhysicsClientHandle physClient, int userDataId, struct b3UserDataValue* valueOut);
	B3_SHARED_API int b3GetUserDataId(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, int visualShapeIndex, const char* key);
	B3_SHARED_API int b3GetUserDataIdFromStatus(b3SharedMemoryStatusHandle statusHandle);
	B3_SHARED_API int b3GetNumUserData(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API void b3GetUserDataInfo(b3PhysicsClientHandle physClient, int bodyUniqueId, int userDataIndex, const char** keyOut, int* userDataIdOut, int* linkIndexOut, int* visualShapeIndexOut);

	B3_SHARED_API b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
	B3_SHARED_API b3SharedMemoryCommandHandle b3GetDynamicsInfoCommandInit2(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex);

	///given a body unique id and link index, return the dynamics information. See b3DynamicsInfo in SharedMemoryPublic.h
	B3_SHARED_API int b3GetDynamicsInfo(b3SharedMemoryStatusHandle statusHandle, struct b3DynamicsInfo* info);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitChangeDynamicsInfo2(b3SharedMemoryCommandHandle commandHandle);

	B3_SHARED_API int b3ChangeDynamicsInfoSetMass(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double mass);
	B3_SHARED_API int b3ChangeDynamicsInfoSetLocalInertiaDiagonal(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, const double localInertiaDiagonal[]);

	B3_SHARED_API int b3ChangeDynamicsInfoSetLateralFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction);
	B3_SHARED_API int b3ChangeDynamicsInfoSetSpinningFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
	B3_SHARED_API int b3ChangeDynamicsInfoSetRollingFriction(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double friction);
	B3_SHARED_API int b3ChangeDynamicsInfoSetRestitution(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double restitution);
	B3_SHARED_API int b3ChangeDynamicsInfoSetLinearDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, double linearDamping);
	B3_SHARED_API int b3ChangeDynamicsInfoSetAngularDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, double angularDamping);
	B3_SHARED_API int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double contactStiffness, double contactDamping);
	B3_SHARED_API int b3ChangeDynamicsInfoSetFrictionAnchor(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, int frictionAnchor);
	B3_SHARED_API int b3ChangeDynamicsInfoSetCcdSweptSphereRadius(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double ccdSweptSphereRadius);
	B3_SHARED_API int b3ChangeDynamicsInfoSetContactProcessingThreshold(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkIndex, double contactProcessingThreshold);
	B3_SHARED_API int b3ChangeDynamicsInfoSetActivationState(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int activationState);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand(b3PhysicsClientHandle physClient, int parentBodyUniqueId, int parentJointIndex, int childBodyUniqueId, int childJointIndex, struct b3JointInfo* info);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitCreateUserConstraintCommand2(b3SharedMemoryCommandHandle commandHandle, int parentBodyUniqueId, int parentJointIndex, int childBodyUniqueId, int childJointIndex, struct b3JointInfo* info);

	///return a unique id for the user constraint, after successful creation, or -1 for an invalid constraint id
	B3_SHARED_API int b3GetStatusUserConstraintUniqueId(b3SharedMemoryStatusHandle statusHandle);

	///change parameters of an existing user constraint
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitChangeUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);
	B3_SHARED_API int b3InitChangeUserConstraintSetPivotInB(b3SharedMemoryCommandHandle commandHandle, const double jointChildPivot[/*3*/]);
	B3_SHARED_API int b3InitChangeUserConstraintSetFrameInB(b3SharedMemoryCommandHandle commandHandle, const double jointChildFrameOrn[/*4*/]);
	B3_SHARED_API int b3InitChangeUserConstraintSetMaxForce(b3SharedMemoryCommandHandle commandHandle, double maxAppliedForce);
	B3_SHARED_API int b3InitChangeUserConstraintSetGearRatio(b3SharedMemoryCommandHandle commandHandle, double gearRatio);
	B3_SHARED_API int b3InitChangeUserConstraintSetGearAuxLink(b3SharedMemoryCommandHandle commandHandle, int gearAuxLink);
	B3_SHARED_API int b3InitChangeUserConstraintSetRelativePositionTarget(b3SharedMemoryCommandHandle commandHandle, double relativePositionTarget);
	B3_SHARED_API int b3InitChangeUserConstraintSetERP(b3SharedMemoryCommandHandle commandHandle, double erp);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRemoveUserConstraintCommand(b3PhysicsClientHandle physClient, int userConstraintUniqueId);

	B3_SHARED_API int b3GetNumUserConstraints(b3PhysicsClientHandle physClient);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitGetUserConstraintStateCommand(b3PhysicsClientHandle physClient, int constraintUniqueId);
	B3_SHARED_API int b3GetStatusUserConstraintState(b3SharedMemoryStatusHandle statusHandle, struct b3UserConstraintState* constraintState);

	B3_SHARED_API int b3GetUserConstraintInfo(b3PhysicsClientHandle physClient, int constraintUniqueId, struct b3UserConstraint* info);
	/// return the user constraint id, given the index in range [0 , b3GetNumUserConstraints() )
	B3_SHARED_API int b3GetUserConstraintId(b3PhysicsClientHandle physClient, int serialIndex);

	///Request physics debug lines for debug visualization. The flags in debugMode are the same as used in Bullet
	///See btIDebugDraw::DebugDrawModes in Bullet/src/LinearMath/btIDebugDraw.h
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestDebugLinesCommand(b3PhysicsClientHandle physClient, int debugMode);

	///Get the pointers to the physics debug line information, after b3InitRequestDebugLinesCommand returns
	///status CMD_DEBUG_LINES_COMPLETED
	B3_SHARED_API void b3GetDebugLines(b3PhysicsClientHandle physClient, struct b3DebugLines* lines);

	///configure the 3D OpenGL debug visualizer (enable/disable GUI widgets, shadows, position camera etc)
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitConfigureOpenGLVisualizer2(b3SharedMemoryCommandHandle commandHandle);
	B3_SHARED_API void b3ConfigureOpenGLVisualizerSetVisualizationFlags(b3SharedMemoryCommandHandle commandHandle, int flag, int enabled);
	B3_SHARED_API void b3ConfigureOpenGLVisualizerSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, const float cameraTargetPosition[/*3*/]);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestOpenGLVisualizerCameraCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3GetStatusOpenGLVisualizerCamera(b3SharedMemoryStatusHandle statusHandle, struct b3OpenGLVisualizerCameraInfo* camera);

	/// Add/remove user-specific debug lines and debug text messages
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugDrawAddLine3D(b3PhysicsClientHandle physClient, const double fromXYZ[/*3*/], const double toXYZ[/*3*/], const double colorRGB[/*3*/], double lineWidth, double lifeTime);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugDrawAddText3D(b3PhysicsClientHandle physClient, const char* txt, const double positionXYZ[/*3*/], const double colorRGB[/*3*/], double textSize, double lifeTime);
	B3_SHARED_API void b3UserDebugTextSetOptionFlags(b3SharedMemoryCommandHandle commandHandle, int optionFlags);
	B3_SHARED_API void b3UserDebugTextSetOrientation(b3SharedMemoryCommandHandle commandHandle, const double orientation[/*4*/]);
	B3_SHARED_API void b3UserDebugItemSetReplaceItemUniqueId(b3SharedMemoryCommandHandle commandHandle, int replaceItem);

	B3_SHARED_API void b3UserDebugItemSetParentObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugAddParameter(b3PhysicsClientHandle physClient, const char* txt, double rangeMin, double rangeMax, double startValue);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugReadParameter(b3PhysicsClientHandle physClient, int debugItemUniqueId);
	B3_SHARED_API int b3GetStatusDebugParameterValue(b3SharedMemoryStatusHandle statusHandle, double* paramValue);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugDrawRemove(b3PhysicsClientHandle physClient, int debugItemUniqueId);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUserDebugDrawRemoveAll(b3PhysicsClientHandle physClient);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitDebugDrawingCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3SetDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex, const double objectColorRGB[/*3*/]);
	B3_SHARED_API void b3RemoveDebugObjectColor(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId, int linkIndex);

	///All debug items unique Ids are positive: a negative unique Id means failure.
	B3_SHARED_API int b3GetDebugItemUniqueId(b3SharedMemoryStatusHandle statusHandle);

	///request an image from a simulated camera, using a software renderer.
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestCameraImage(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestCameraImage2(b3SharedMemoryCommandHandle commandHandle);
	B3_SHARED_API void b3RequestCameraImageSetCameraMatrices(b3SharedMemoryCommandHandle commandHandle, float viewMatrix[/*16*/], float projectionMatrix[/*16*/]);
	B3_SHARED_API void b3RequestCameraImageSetPixelResolution(b3SharedMemoryCommandHandle commandHandle, int width, int height);
	B3_SHARED_API void b3RequestCameraImageSetLightDirection(b3SharedMemoryCommandHandle commandHandle, const float lightDirection[/*3*/]);
	B3_SHARED_API void b3RequestCameraImageSetLightColor(b3SharedMemoryCommandHandle commandHandle, const float lightColor[/*3*/]);
	B3_SHARED_API void b3RequestCameraImageSetLightDistance(b3SharedMemoryCommandHandle commandHandle, float lightDistance);
	B3_SHARED_API void b3RequestCameraImageSetLightAmbientCoeff(b3SharedMemoryCommandHandle commandHandle, float lightAmbientCoeff);
	B3_SHARED_API void b3RequestCameraImageSetLightDiffuseCoeff(b3SharedMemoryCommandHandle commandHandle, float lightDiffuseCoeff);
	B3_SHARED_API void b3RequestCameraImageSetLightSpecularCoeff(b3SharedMemoryCommandHandle commandHandle, float lightSpecularCoeff);
	B3_SHARED_API void b3RequestCameraImageSetShadow(b3SharedMemoryCommandHandle commandHandle, int hasShadow);
	B3_SHARED_API void b3RequestCameraImageSelectRenderer(b3SharedMemoryCommandHandle commandHandle, int renderer);
	B3_SHARED_API void b3RequestCameraImageSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);

	B3_SHARED_API void b3GetCameraImageData(b3PhysicsClientHandle physClient, struct b3CameraImageData* imageData);

	///set projective texture camera matrices.
	B3_SHARED_API void b3RequestCameraImageSetProjectiveTextureMatrices(b3SharedMemoryCommandHandle commandHandle, float viewMatrix[/*16*/], float projectionMatrix[/*16*/]);

	///compute a view matrix, helper function for b3RequestCameraImageSetCameraMatrices
	B3_SHARED_API void b3ComputeViewMatrixFromPositions(const float cameraPosition[/*3*/], const float cameraTargetPosition[/*3*/], const float cameraUp[/*3*/], float viewMatrix[/*16*/]);
	B3_SHARED_API void b3ComputeViewMatrixFromYawPitchRoll(const float cameraTargetPosition[/*3*/], float distance, float yaw, float pitch, float roll, int upAxis, float viewMatrix[/*16*/]);
	B3_SHARED_API void b3ComputePositionFromViewMatrix(const float viewMatrix[/*16*/], float cameraPosition[/*3*/], float cameraTargetPosition[/*3*/], float cameraUp[/*3*/]);

	///compute a projection matrix, helper function for b3RequestCameraImageSetCameraMatrices
	B3_SHARED_API void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, float projectionMatrix[/*16*/]);
	B3_SHARED_API void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float projectionMatrix[/*16*/]);

	/* obsolete, please use b3ComputeViewProjectionMatrices */
	B3_SHARED_API void b3RequestCameraImageSetViewMatrix(b3SharedMemoryCommandHandle commandHandle, const float cameraPosition[/*3*/], const float cameraTargetPosition[/*3*/], const float cameraUp[/*3*/]);
	/* obsolete, please use b3ComputeViewProjectionMatrices */
	B3_SHARED_API void b3RequestCameraImageSetViewMatrix2(b3SharedMemoryCommandHandle commandHandle, const float cameraTargetPosition[/*3*/], float distance, float yaw, float pitch, float roll, int upAxis);
	/* obsolete, please use b3ComputeViewProjectionMatrices */
	B3_SHARED_API void b3RequestCameraImageSetProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float left, float right, float bottom, float top, float nearVal, float farVal);
	/* obsolete, please use b3ComputeViewProjectionMatrices */
	B3_SHARED_API void b3RequestCameraImageSetFOVProjectionMatrix(b3SharedMemoryCommandHandle commandHandle, float fov, float aspect, float nearVal, float farVal);

	///request an contact point information
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestContactPointInformation(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3SetContactFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
	B3_SHARED_API void b3SetContactFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
	B3_SHARED_API void b3SetContactFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	B3_SHARED_API void b3SetContactFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	B3_SHARED_API void b3GetContactPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointData);

	///compute the closest points between two bodies
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitClosestDistanceQuery(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3SetClosestDistanceFilterBodyA(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA);
	B3_SHARED_API void b3SetClosestDistanceFilterLinkA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	B3_SHARED_API void b3SetClosestDistanceFilterBodyB(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdB);
	B3_SHARED_API void b3SetClosestDistanceFilterLinkB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	B3_SHARED_API void b3SetClosestDistanceThreshold(b3SharedMemoryCommandHandle commandHandle, double distance);

	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapeA(b3SharedMemoryCommandHandle commandHandle, int collisionShapeA);
	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapeB(b3SharedMemoryCommandHandle commandHandle, int collisionShapeB);
	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapePositionA(b3SharedMemoryCommandHandle commandHandle, const double collisionShapePositionA[/*3*/]);
	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapePositionB(b3SharedMemoryCommandHandle commandHandle, const double collisionShapePositionB[/*3*/]);
	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapeOrientationA(b3SharedMemoryCommandHandle commandHandle, const double collisionShapeOrientationA[/*4*/]);
	B3_SHARED_API void b3SetClosestDistanceFilterCollisionShapeOrientationB(b3SharedMemoryCommandHandle commandHandle, const double collisionShapeOrientationB[/*4*/]);

	B3_SHARED_API void b3GetClosestPointInformation(b3PhysicsClientHandle physClient, struct b3ContactInformation* contactPointInfo);

	///get all the bodies that touch a given axis aligned bounding box specified in world space (min and max coordinates)
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitAABBOverlapQuery(b3PhysicsClientHandle physClient, const double aabbMin[/*3*/], const double aabbMax[/*3*/]);
	B3_SHARED_API void b3GetAABBOverlapResults(b3PhysicsClientHandle physClient, struct b3AABBOverlapData* data);

	//request visual shape information
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestVisualShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueIdA);
	B3_SHARED_API void b3GetVisualShapeInformation(b3PhysicsClientHandle physClient, struct b3VisualShapeInformation* visualShapeInfo);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestCollisionShapeInformation(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex);
	B3_SHARED_API void b3GetCollisionShapeInformation(b3PhysicsClientHandle physClient, struct b3CollisionShapeInformation* collisionShapeInfo);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitLoadTexture(b3PhysicsClientHandle physClient, const char* filename);
	B3_SHARED_API int b3GetStatusTextureUniqueId(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateChangeTextureCommandInit(b3PhysicsClientHandle physClient, int textureUniqueId, int width, int height, const char* rgbPixels);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUpdateVisualShape(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitUpdateVisualShape2(b3PhysicsClientHandle physClient, int bodyUniqueId, int jointIndex, int shapeIndex);
	B3_SHARED_API void b3UpdateVisualShapeTexture(b3SharedMemoryCommandHandle commandHandle, int textureUniqueId);
	B3_SHARED_API void b3UpdateVisualShapeRGBAColor(b3SharedMemoryCommandHandle commandHandle, const double rgbaColor[/*4*/]);
	B3_SHARED_API void b3UpdateVisualShapeSpecularColor(b3SharedMemoryCommandHandle commandHandle, const double specularColor[/*3*/]);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitPhysicsParamCommand2(b3SharedMemoryCommandHandle commandHandle);
	B3_SHARED_API int b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx, double gravy, double gravz);
	B3_SHARED_API int b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
	B3_SHARED_API int b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
	B3_SHARED_API int b3PhysicsParamSetDefaultNonContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultNonContactERP);
	B3_SHARED_API int b3PhysicsParamSetDefaultFrictionERP(b3SharedMemoryCommandHandle commandHandle, double frictionERP);
	B3_SHARED_API int b3PhysicsParamSetDefaultGlobalCFM(b3SharedMemoryCommandHandle commandHandle, double defaultGlobalCFM);
	B3_SHARED_API int b3PhysicsParamSetDefaultFrictionCFM(b3SharedMemoryCommandHandle commandHandle, double frictionCFM);
	B3_SHARED_API int b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
	B3_SHARED_API int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
	B3_SHARED_API int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);
	B3_SHARED_API int b3PhysicsParamSetCollisionFilterMode(b3SharedMemoryCommandHandle commandHandle, int filterMode);
	B3_SHARED_API int b3PhysicsParamSetUseSplitImpulse(b3SharedMemoryCommandHandle commandHandle, int useSplitImpulse);
	B3_SHARED_API int b3PhysicsParamSetSplitImpulsePenetrationThreshold(b3SharedMemoryCommandHandle commandHandle, double splitImpulsePenetrationThreshold);
	B3_SHARED_API int b3PhysicsParamSetContactBreakingThreshold(b3SharedMemoryCommandHandle commandHandle, double contactBreakingThreshold);
	B3_SHARED_API int b3PhysicsParamSetMaxNumCommandsPer1ms(b3SharedMemoryCommandHandle commandHandle, int maxNumCmdPer1ms);
	B3_SHARED_API int b3PhysicsParamSetEnableFileCaching(b3SharedMemoryCommandHandle commandHandle, int enableFileCaching);
	B3_SHARED_API int b3PhysicsParamSetRestitutionVelocityThreshold(b3SharedMemoryCommandHandle commandHandle, double restitutionVelocityThreshold);
	B3_SHARED_API int b3PhysicsParamSetEnableConeFriction(b3SharedMemoryCommandHandle commandHandle, int enableConeFriction);
	B3_SHARED_API int b3PhysicsParameterSetDeterministicOverlappingPairs(b3SharedMemoryCommandHandle commandHandle, int deterministicOverlappingPairs);
	B3_SHARED_API int b3PhysicsParameterSetAllowedCcdPenetration(b3SharedMemoryCommandHandle commandHandle, double allowedCcdPenetration);
	B3_SHARED_API int b3PhysicsParameterSetJointFeedbackMode(b3SharedMemoryCommandHandle commandHandle, int jointFeedbackMode);
	B3_SHARED_API int b3PhysicsParamSetSolverResidualThreshold(b3SharedMemoryCommandHandle commandHandle, double solverResidualThreshold);
	B3_SHARED_API int b3PhysicsParamSetContactSlop(b3SharedMemoryCommandHandle commandHandle, double contactSlop);
	B3_SHARED_API int b3PhysicsParameterSetEnableSAT(b3SharedMemoryCommandHandle commandHandle, int enableSAT);
	B3_SHARED_API int b3PhysicsParameterSetConstraintSolverType(b3SharedMemoryCommandHandle commandHandle, int constraintSolverType);
	B3_SHARED_API int b3PhysicsParameterSetMinimumSolverIslandSize(b3SharedMemoryCommandHandle commandHandle, int minimumSolverIslandSize);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRequestPhysicsParamCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3GetStatusPhysicsSimulationParameters(b3SharedMemoryStatusHandle statusHandle, struct b3PhysicsSimulationParameters* params);

	//b3PhysicsParamSetInternalSimFlags is for internal/temporary/easter-egg/experimental demo purposes
	//Use at own risk: magic things may or my not happen when calling this API
	B3_SHARED_API int b3PhysicsParamSetInternalSimFlags(b3SharedMemoryCommandHandle commandHandle, int flags);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitStepSimulationCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitStepSimulationCommand2(b3SharedMemoryCommandHandle commandHandle);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitResetSimulationCommand(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3InitResetSimulationCommand2(b3SharedMemoryCommandHandle commandHandle);

	///Load a robot from a URDF file. Status type will CMD_URDF_LOADING_COMPLETED.
	///Access the robot from the unique body index, through b3GetStatusBodyIndex(statusHandle);
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadUrdfCommandInit(b3PhysicsClientHandle physClient, const char* urdfFileName);
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadUrdfCommandInit2(b3SharedMemoryCommandHandle commandHandle, const char* urdfFileName);
	B3_SHARED_API int b3LoadUrdfCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ);
	B3_SHARED_API int b3LoadUrdfCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);
	B3_SHARED_API int b3LoadUrdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
	B3_SHARED_API int b3LoadUrdfCommandSetUseFixedBase(b3SharedMemoryCommandHandle commandHandle, int useFixedBase);
	B3_SHARED_API int b3LoadUrdfCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);
	B3_SHARED_API int b3LoadUrdfCommandSetGlobalScaling(b3SharedMemoryCommandHandle commandHandle, double globalScaling);

	B3_SHARED_API b3SharedMemoryCommandHandle b3SaveStateCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3GetStatusGetStateId(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadStateCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3LoadStateSetStateId(b3SharedMemoryCommandHandle commandHandle, int stateId);
	B3_SHARED_API int b3LoadStateSetFileName(b3SharedMemoryCommandHandle commandHandle, const char* fileName);

	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);

	B3_SHARED_API b3SharedMemoryCommandHandle b3SaveBulletCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadMJCFCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadMJCFCommandInit2(b3SharedMemoryCommandHandle commandHandle, const char* fileName);
	B3_SHARED_API void b3LoadMJCFCommandSetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);

	///compute the forces to achieve an acceleration, given a state q and qdot using inverse dynamics
	B3_SHARED_API b3SharedMemoryCommandHandle b3CalculateInverseDynamicsCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId,
																					const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
	B3_SHARED_API b3SharedMemoryCommandHandle b3CalculateInverseDynamicsCommandInit2(b3PhysicsClientHandle physClient, int bodyUniqueId,
		const double* jointPositionsQ, int dofCountQ, const double* jointVelocitiesQdot, const double* jointAccelerations, int dofCountQdot);

	B3_SHARED_API int b3GetStatusInverseDynamicsJointForces(b3SharedMemoryStatusHandle statusHandle,
															int* bodyUniqueId,
															int* dofCount,
															double* jointForces);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CalculateJacobianCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositionsQ, const double* jointVelocitiesQdot, const double* jointAccelerations);
	B3_SHARED_API int b3GetStatusJacobian(b3SharedMemoryStatusHandle statusHandle,
										  int* dofCount,
										  double* linearJacobian,
										  double* angularJacobian);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CalculateMassMatrixCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId, const double* jointPositionsQ);
	///the mass matrix is stored in column-major layout of size dofCount*dofCount
	B3_SHARED_API int b3GetStatusMassMatrix(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int* dofCount, double* massMatrix);

	///compute the joint positions to move the end effector to a desired target using inverse kinematics
	B3_SHARED_API b3SharedMemoryCommandHandle b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API void b3CalculateInverseKinematicsAddTargetPurePosition(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[/*3*/]);
	B3_SHARED_API void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(b3SharedMemoryCommandHandle commandHandle, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double targetOrientation[/*4*/]);
	B3_SHARED_API void b3CalculateInverseKinematicsPosWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
	B3_SHARED_API void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(b3SharedMemoryCommandHandle commandHandle, int numDof, int endEffectorLinkIndex, const double targetPosition[/*3*/], const double targetOrientation[/*4*/], const double* lowerLimit, const double* upperLimit, const double* jointRange, const double* restPose);
	B3_SHARED_API void b3CalculateInverseKinematicsSetJointDamping(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* jointDampingCoeff);
	B3_SHARED_API void b3CalculateInverseKinematicsSelectSolver(b3SharedMemoryCommandHandle commandHandle, int solver);
	B3_SHARED_API int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
																 int* bodyUniqueId,
																 int* dofCount,
																 double* jointPositions);

	B3_SHARED_API void b3CalculateInverseKinematicsSetCurrentPositions(b3SharedMemoryCommandHandle commandHandle, int numDof, const double* currentJointPositions);
	B3_SHARED_API void b3CalculateInverseKinematicsSetMaxNumIterations(b3SharedMemoryCommandHandle commandHandle, int maxNumIterations);
	B3_SHARED_API void b3CalculateInverseKinematicsSetResidualThreshold(b3SharedMemoryCommandHandle commandHandle, double residualThreshold);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CollisionFilterCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3SetCollisionFilterPair(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA,
												int bodyUniqueIdB, int linkIndexA, int linkIndexB, int enableCollision);
	B3_SHARED_API void b3SetCollisionFilterGroupMask(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueIdA,
													 int linkIndexA, int collisionFilterGroup, int collisionFilterMask);

	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadSdfCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadSdfCommandInit2(b3SharedMemoryCommandHandle commandHandle, const char* sdfFileName);

	B3_SHARED_API int b3LoadSdfCommandSetUseMultiBody(b3SharedMemoryCommandHandle commandHandle, int useMultiBody);
	B3_SHARED_API int b3LoadSdfCommandSetUseGlobalScaling(b3SharedMemoryCommandHandle commandHandle, double globalScaling);

	B3_SHARED_API b3SharedMemoryCommandHandle b3SaveWorldCommandInit(b3PhysicsClientHandle physClient, const char* sdfFileName);

	///The b3JointControlCommandInit method is obsolete, use b3JointControlCommandInit2 instead
	B3_SHARED_API b3SharedMemoryCommandHandle b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);

	///Set joint motor control variables such as desired position/angle, desired velocity,
	///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
	B3_SHARED_API b3SharedMemoryCommandHandle b3JointControlCommandInit2(b3PhysicsClientHandle physClient, int bodyUniqueId, int controlMode);
	B3_SHARED_API b3SharedMemoryCommandHandle b3JointControlCommandInit2Internal(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int controlMode);

	///Only use when controlMode is CONTROL_MODE_POSITION_VELOCITY_PD
	B3_SHARED_API int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
	B3_SHARED_API int b3JointControlSetDesiredPositionMultiDof(b3SharedMemoryCommandHandle commandHandle, int qIndex, const double* position, int dofCount);

	B3_SHARED_API int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
	B3_SHARED_API int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
	B3_SHARED_API int b3JointControlSetMaximumVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double maximumVelocity);

	///Only use when controlMode is CONTROL_MODE_VELOCITY
	B3_SHARED_API int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value); /* find a better name for dof/q/u indices, point to b3JointInfo */
	B3_SHARED_API int b3JointControlSetDesiredVelocityMultiDof(b3SharedMemoryCommandHandle commandHandle, int dofIndex, const double* velocity, int dofCount);
	B3_SHARED_API int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
	///Only use if when controlMode is CONTROL_MODE_TORQUE,
	B3_SHARED_API int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);

	///the creation of collision shapes and rigid bodies etc is likely going to change,
	///but good to have a b3CreateBoxShapeCommandInit for now

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateCollisionShapeCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3CreateCollisionShapeAddSphere(b3SharedMemoryCommandHandle commandHandle, double radius);
	B3_SHARED_API int b3CreateCollisionShapeAddBox(b3SharedMemoryCommandHandle commandHandle, const double halfExtents[/*3*/]);
	B3_SHARED_API int b3CreateCollisionShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
	B3_SHARED_API int b3CreateCollisionShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
	B3_SHARED_API int b3CreateCollisionShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, const double planeNormal[/*3*/], double planeConstant);
	B3_SHARED_API int b3CreateCollisionShapeAddMesh(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/]);
	B3_SHARED_API int b3CreateCollisionShapeAddConvexMesh(b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices);
	B3_SHARED_API int b3CreateCollisionShapeAddConcaveMesh(b3SharedMemoryCommandHandle commandHandle, const double meshScale[/*3*/], const double* vertices, int numVertices, const int* indices, int numIndices);
	B3_SHARED_API void b3CreateCollisionSetFlag(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, int flags);
	B3_SHARED_API void b3CreateCollisionShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double childPosition[/*3*/], const double childOrientation[/*4*/]);
	B3_SHARED_API int b3GetStatusCollisionShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API b3SharedMemoryCommandHandle b3InitRemoveCollisionShapeCommand(b3PhysicsClientHandle physClient, int collisionShapeId);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateVisualShapeCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3CreateVisualShapeAddSphere(b3SharedMemoryCommandHandle commandHandle, double radius);
	B3_SHARED_API int b3CreateVisualShapeAddBox(b3SharedMemoryCommandHandle commandHandle, const double halfExtents[/*3*/]);
	B3_SHARED_API int b3CreateVisualShapeAddCapsule(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
	B3_SHARED_API int b3CreateVisualShapeAddCylinder(b3SharedMemoryCommandHandle commandHandle, double radius, double height);
	B3_SHARED_API int b3CreateVisualShapeAddPlane(b3SharedMemoryCommandHandle commandHandle, const double planeNormal[/*3*/], double planeConstant);
	B3_SHARED_API int b3CreateVisualShapeAddMesh(b3SharedMemoryCommandHandle commandHandle, const char* fileName, const double meshScale[/*3*/]);
	B3_SHARED_API void b3CreateVisualSetFlag(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, int flags);
	B3_SHARED_API void b3CreateVisualShapeSetChildTransform(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double childPosition[/*3*/], const double childOrientation[/*4*/]);
	B3_SHARED_API void b3CreateVisualShapeSetSpecularColor(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double specularColor[/*3*/]);
	B3_SHARED_API void b3CreateVisualShapeSetRGBAColor(b3SharedMemoryCommandHandle commandHandle, int shapeIndex, const double rgbaColor[/*4*/]);

	B3_SHARED_API int b3GetStatusVisualShapeUniqueId(b3SharedMemoryStatusHandle statusHandle);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateMultiBodyCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3CreateMultiBodyBase(b3SharedMemoryCommandHandle commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, const double basePosition[/*3*/], const double baseOrientation[/*4*/], const double baseInertialFramePosition[/*3*/], const double baseInertialFrameOrientation[/*4*/]);

	B3_SHARED_API int b3CreateMultiBodyLink(b3SharedMemoryCommandHandle commandHandle, double linkMass, double linkCollisionShapeIndex,
											double linkVisualShapeIndex,
											const double linkPosition[/*3*/],
											const double linkOrientation[/*4*/],
											const double linkInertialFramePosition[/*3*/],
											const double linkInertialFrameOrientation[/*4*/],
											int linkParentIndex,
											int linkJointType,
											const double linkJointAxis[/*3*/]);

	//useMaximalCoordinates are disabled by default, enabling them is experimental and not fully supported yet
	B3_SHARED_API void b3CreateMultiBodyUseMaximalCoordinates(b3SharedMemoryCommandHandle commandHandle);
	B3_SHARED_API void b3CreateMultiBodySetFlags(b3SharedMemoryCommandHandle commandHandle, int flags);

	//int b3CreateMultiBodyAddLink(b3SharedMemoryCommandHandle commandHandle, int jointType, int parentLinkIndex, double linkMass, int linkCollisionShapeUnique, int linkVisualShapeUniqueId);

	///create a box of size (1,1,1) at world origin (0,0,0) at orientation quat (0,0,0,1)
	///after that, you can optionally adjust the initial position, orientation and size
	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateBoxShapeCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3CreateBoxCommandSetStartPosition(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ);
	B3_SHARED_API int b3CreateBoxCommandSetStartOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);
	B3_SHARED_API int b3CreateBoxCommandSetHalfExtents(b3SharedMemoryCommandHandle commandHandle, double halfExtentsX, double halfExtentsY, double halfExtentsZ);
	B3_SHARED_API int b3CreateBoxCommandSetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
	B3_SHARED_API int b3CreateBoxCommandSetCollisionShapeType(b3SharedMemoryCommandHandle commandHandle, int collisionShapeType);
	B3_SHARED_API int b3CreateBoxCommandSetColorRGBA(b3SharedMemoryCommandHandle commandHandle, double red, double green, double blue, double alpha);

	///b3CreatePoseCommandInit will initialize (teleport) the pose of a body/robot. You can individually set the base position,
	///base orientation and joint angles. This will set all velocities of base and joints to zero.
	///This is not a robot control command using actuators/joint motors, but manual repositioning the robot.
	B3_SHARED_API b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API b3SharedMemoryCommandHandle b3CreatePoseCommandInit2(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId);

	B3_SHARED_API int b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX, double startPosY, double startPosZ);
	B3_SHARED_API int b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW);
	B3_SHARED_API int b3CreatePoseCommandSetBaseLinearVelocity(b3SharedMemoryCommandHandle commandHandle, const double linVel[/*3*/]);
	B3_SHARED_API int b3CreatePoseCommandSetBaseAngularVelocity(b3SharedMemoryCommandHandle commandHandle, const double angVel[/*3*/]);

	B3_SHARED_API int b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
	B3_SHARED_API int b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, double jointPosition);
	B3_SHARED_API int b3CreatePoseCommandSetJointPositionMultiDof(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, const double* jointPosition, int posSize);

	B3_SHARED_API int b3CreatePoseCommandSetQ(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* q, const int* hasQ);
	B3_SHARED_API int b3CreatePoseCommandSetQdots(b3SharedMemoryCommandHandle commandHandle, int numJointVelocities, const double* qDots, const int* hasQdots);

	B3_SHARED_API int b3CreatePoseCommandSetJointVelocities(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int numJointVelocities, const double* jointVelocities);
	B3_SHARED_API int b3CreatePoseCommandSetJointVelocity(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, double jointVelocity);
	B3_SHARED_API int b3CreatePoseCommandSetJointVelocityMultiDof(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, int jointIndex, const double* jointVelocity, int velSize);
	
	///We are currently not reading the sensor information from the URDF file, and programmatically assign sensors.
	///This is rather inconsistent, to mix programmatical creation with loading from file.
	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateSensorCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API int b3CreateSensorEnable6DofJointForceTorqueSensor(b3SharedMemoryCommandHandle commandHandle, int jointIndex, int enable);
	///b3CreateSensorEnableIMUForLink is not implemented yet.
	///For now, if the IMU is located in the root link, use the root world transform to mimic an IMU.
	B3_SHARED_API int b3CreateSensorEnableIMUForLink(b3SharedMemoryCommandHandle commandHandle, int linkIndex, int enable);

	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestActualStateCommandInit(b3PhysicsClientHandle physClient, int bodyUniqueId);
	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestActualStateCommandInit2(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId);

	B3_SHARED_API int b3RequestActualStateCommandComputeLinkVelocity(b3SharedMemoryCommandHandle commandHandle, int computeLinkVelocity);
	B3_SHARED_API int b3RequestActualStateCommandComputeForwardKinematics(b3SharedMemoryCommandHandle commandHandle, int computeForwardKinematics);

	B3_SHARED_API int b3GetJointState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState* state);
	B3_SHARED_API int b3GetJointStateMultiDof(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int jointIndex, struct b3JointSensorState2* state);
	B3_SHARED_API int b3GetLinkState(b3PhysicsClientHandle physClient, b3SharedMemoryStatusHandle statusHandle, int linkIndex, struct b3LinkState* state);

	B3_SHARED_API b3SharedMemoryCommandHandle b3PickBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
														 double rayFromWorldY, double rayFromWorldZ,
														 double rayToWorldX, double rayToWorldY, double rayToWorldZ);
	B3_SHARED_API b3SharedMemoryCommandHandle b3MovePickedBody(b3PhysicsClientHandle physClient, double rayFromWorldX,
															   double rayFromWorldY, double rayFromWorldZ,
															   double rayToWorldX, double rayToWorldY,
															   double rayToWorldZ);
	B3_SHARED_API b3SharedMemoryCommandHandle b3RemovePickingConstraint(b3PhysicsClientHandle physClient);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateRaycastCommandInit(b3PhysicsClientHandle physClient, double rayFromWorldX,
																		 double rayFromWorldY, double rayFromWorldZ,
																		 double rayToWorldX, double rayToWorldY, double rayToWorldZ);

	B3_SHARED_API b3SharedMemoryCommandHandle b3CreateRaycastBatchCommandInit(b3PhysicsClientHandle physClient);
	// Sets the number of threads to use to compute the ray intersections for the batch. Specify 0 to let Bullet decide, 1 (default) for single core execution, 2 or more to select the number of threads to use.
	B3_SHARED_API void b3RaycastBatchSetNumThreads(b3SharedMemoryCommandHandle commandHandle, int numThreads);
	//max num rays for b3RaycastBatchAddRay is MAX_RAY_INTERSECTION_BATCH_SIZE
	B3_SHARED_API void b3RaycastBatchAddRay(b3SharedMemoryCommandHandle commandHandle, const double rayFromWorld[/*3*/], const double rayToWorld[/*3*/]);
	//max num rays for b3RaycastBatchAddRays is MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING
	B3_SHARED_API void b3RaycastBatchAddRays(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle, const double* rayFromWorld, const double* rayToWorld, int numRays);
	B3_SHARED_API void b3RaycastBatchSetParentObject(b3SharedMemoryCommandHandle commandHandle, int parentObjectUniqueId, int parentLinkIndex);

	B3_SHARED_API void b3GetRaycastInformation(b3PhysicsClientHandle physClient, struct b3RaycastInformation* raycastInfo);

	/// Apply external force at the body (or link) center of mass, in world space/Cartesian coordinates.
	B3_SHARED_API b3SharedMemoryCommandHandle b3ApplyExternalForceCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3ApplyExternalForce(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double force[/*3*/], const double position[/*3*/], int flag);
	B3_SHARED_API void b3ApplyExternalTorque(b3SharedMemoryCommandHandle commandHandle, int bodyUniqueId, int linkId, const double torque[/*3*/], int flag);

	///experiments of robots interacting with non-rigid objects (such as btSoftBody)
	B3_SHARED_API b3SharedMemoryCommandHandle b3LoadSoftBodyCommandInit(b3PhysicsClientHandle physClient, const char* fileName);
	B3_SHARED_API int b3LoadSoftBodySetScale(b3SharedMemoryCommandHandle commandHandle, double scale);
	B3_SHARED_API int b3LoadSoftBodySetMass(b3SharedMemoryCommandHandle commandHandle, double mass);
	B3_SHARED_API int b3LoadSoftBodySetCollisionMargin(b3SharedMemoryCommandHandle commandHandle, double collisionMargin);

	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestVREventsCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3VREventsSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);

	B3_SHARED_API void b3GetVREventsData(b3PhysicsClientHandle physClient, struct b3VREventsData* vrEventsData);

	B3_SHARED_API b3SharedMemoryCommandHandle b3SetVRCameraStateCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3SetVRCameraRootPosition(b3SharedMemoryCommandHandle commandHandle, const double rootPos[/*3*/]);
	B3_SHARED_API int b3SetVRCameraRootOrientation(b3SharedMemoryCommandHandle commandHandle, const double rootOrn[/*4*/]);
	B3_SHARED_API int b3SetVRCameraTrackingObject(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
	B3_SHARED_API int b3SetVRCameraTrackingObjectFlag(b3SharedMemoryCommandHandle commandHandle, int flag);

	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestKeyboardEventsCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestKeyboardEventsCommandInit2(b3SharedMemoryCommandHandle commandHandle);
	B3_SHARED_API void b3GetKeyboardEventsData(b3PhysicsClientHandle physClient, struct b3KeyboardEventsData* keyboardEventsData);

	B3_SHARED_API b3SharedMemoryCommandHandle b3RequestMouseEventsCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API void b3GetMouseEventsData(b3PhysicsClientHandle physClient, struct b3MouseEventsData* mouseEventsData);

	B3_SHARED_API b3SharedMemoryCommandHandle b3StateLoggingCommandInit(b3PhysicsClientHandle physClient);
	B3_SHARED_API int b3StateLoggingStart(b3SharedMemoryCommandHandle commandHandle, int loggingType, const char* fileName);
	B3_SHARED_API int b3StateLoggingAddLoggingObjectUniqueId(b3SharedMemoryCommandHandle commandHandle, int objectUniqueId);
	B3_SHARED_API int b3StateLoggingSetMaxLogDof(b3SharedMemoryCommandHandle commandHandle, int maxLogDof);
	B3_SHARED_API int b3StateLoggingSetLinkIndexA(b3SharedMemoryCommandHandle commandHandle, int linkIndexA);
	B3_SHARED_API int b3StateLoggingSetLinkIndexB(b3SharedMemoryCommandHandle commandHandle, int linkIndexB);
	B3_SHARED_API int b3StateLoggingSetBodyAUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyAUniqueId);
	B3_SHARED_API int b3StateLoggingSetBodyBUniqueId(b3SharedMemoryCommandHandle commandHandle, int bodyBUniqueId);
	B3_SHARED_API int b3StateLoggingSetDeviceTypeFilter(b3SharedMemoryCommandHandle commandHandle, int deviceTypeFilter);
	B3_SHARED_API int b3StateLoggingSetLogFlags(b3SharedMemoryCommandHandle commandHandle, int logFlags);

	B3_SHARED_API int b3GetStatusLoggingUniqueId(b3SharedMemoryStatusHandle statusHandle);
	B3_SHARED_API int b3StateLoggingStop(b3SharedMemoryCommandHandle commandHandle, int loggingUid);

	B3_SHARED_API b3SharedMemoryCommandHandle b3ProfileTimingCommandInit(b3PhysicsClientHandle physClient, const char* name);
	B3_SHARED_API void b3SetProfileTimingDuractionInMicroSeconds(b3SharedMemoryCommandHandle commandHandle, int duration);

	B3_SHARED_API void b3PushProfileTiming(b3PhysicsClientHandle physClient, const char* timingName);
	B3_SHARED_API void b3PopProfileTiming(b3PhysicsClientHandle physClient);

	B3_SHARED_API void b3SetTimeOut(b3PhysicsClientHandle physClient, double timeOutInSeconds);
	B3_SHARED_API double b3GetTimeOut(b3PhysicsClientHandle physClient);

	B3_SHARED_API b3SharedMemoryCommandHandle b3SetAdditionalSearchPath(b3PhysicsClientHandle physClient, const char* path);

	B3_SHARED_API void b3MultiplyTransforms(const double posA[/*3*/], const double ornA[/*4*/], const double posB[/*3*/], const double ornB[/*4*/], double outPos[/*3*/], double outOrn[/*4*/]);
	B3_SHARED_API void b3InvertTransform(const double pos[/*3*/], const double orn[/*4*/], double outPos[/*3*/], double outOrn[/*4*/]);
	B3_SHARED_API void b3QuaternionSlerp(const double startQuat[/*4*/], const double endQuat[/*4*/], double interpolationFraction, double outOrn[/*4*/]);
	B3_SHARED_API void b3GetQuaternionFromAxisAngle(const double axis[/*3*/], double angle, double outQuat[/*4*/]);
	B3_SHARED_API void b3GetAxisAngleFromQuaternion(const double quat[/*4*/], double axis[/*3*/], double* angle);
	B3_SHARED_API void b3GetQuaternionDifference(const double startQuat[/*4*/], const double endQuat[/*4*/], double outOrn[/*4*/]);
	B3_SHARED_API void b3CalculateVelocityQuaternion(const double startQuat[/*4*/], const double endQuat[/*4*/], double deltaTime, double angVelOut[/*3*/]);
	B3_SHARED_API void b3RotateVector(const double quat[/*4*/], const double vec[/*3*/], double vecOut[/*3*/]);

	

#ifdef __cplusplus
}
#endif

#endif  //PHYSICS_CLIENT_C_API_H
