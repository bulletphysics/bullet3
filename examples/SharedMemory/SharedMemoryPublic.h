#ifndef SHARED_MEMORY_PUBLIC_H
#define SHARED_MEMORY_PUBLIC_H

#define SHARED_MEMORY_KEY 12347
///increase the SHARED_MEMORY_MAGIC_NUMBER whenever incompatible changes are made in the structures
///my convention is year/month/day/rev
#define SHARED_MEMORY_MAGIC_NUMBER 201706015
//#define SHARED_MEMORY_MAGIC_NUMBER 201706001
//#define SHARED_MEMORY_MAGIC_NUMBER 201703024


enum EnumSharedMemoryClientCommand
{
    CMD_LOAD_SDF,
	CMD_LOAD_URDF,
	CMD_LOAD_BULLET,
	CMD_SAVE_BULLET,
	CMD_LOAD_MJCF,
    CMD_LOAD_BUNNY,
	CMD_SEND_BULLET_DATA_STREAM,
	CMD_CREATE_BOX_COLLISION_SHAPE,
	CMD_CREATE_RIGID_BODY,
	CMD_DELETE_RIGID_BODY,
	CMD_CREATE_SENSOR,///enable or disable joint feedback for force/torque sensors
	CMD_INIT_POSE,
	CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
	CMD_SEND_DESIRED_STATE,//todo: reconsider naming, for example SET_JOINT_CONTROL_VARIABLE?
	CMD_REQUEST_ACTUAL_STATE,
	CMD_REQUEST_DEBUG_LINES,
    CMD_REQUEST_BODY_INFO,
	CMD_REQUEST_INTERNAL_DATA,
    CMD_STEP_FORWARD_SIMULATION,
    CMD_RESET_SIMULATION,
    CMD_PICK_BODY,
    CMD_MOVE_PICKED_BODY,
    CMD_REMOVE_PICKING_CONSTRAINT_BODY,
    CMD_REQUEST_CAMERA_IMAGE_DATA,
    CMD_APPLY_EXTERNAL_FORCE,
	CMD_CALCULATE_INVERSE_DYNAMICS,
    CMD_CALCULATE_INVERSE_KINEMATICS,
    CMD_CALCULATE_JACOBIAN,
    CMD_USER_CONSTRAINT,
    CMD_REQUEST_CONTACT_POINT_INFORMATION,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS,

	CMD_REQUEST_AABB_OVERLAP,

	CMD_SAVE_WORLD,
	CMD_REQUEST_VISUAL_SHAPE_INFO,
    CMD_UPDATE_VISUAL_SHAPE,
    CMD_LOAD_TEXTURE,
    CMD_SET_SHADOW,
	CMD_USER_DEBUG_DRAW,
	CMD_REQUEST_VR_EVENTS_DATA,
	CMD_SET_VR_CAMERA_STATE,
	CMD_SYNC_BODY_INFO,
	CMD_STATE_LOGGING,
    CMD_CONFIGURE_OPENGL_VISUALIZER,
	CMD_REQUEST_KEYBOARD_EVENTS_DATA,
	CMD_REQUEST_OPENGL_VISUALIZER_CAMERA,
	CMD_REMOVE_BODY,
	CMD_CHANGE_DYNAMICS_INFO,
	CMD_GET_DYNAMICS_INFO,
	CMD_PROFILE_TIMING,
	CMD_CREATE_COLLISION_SHAPE,
	CMD_CREATE_VISUAL_SHAPE,
	CMD_CREATE_MULTI_BODY,
	CMD_REQUEST_COLLISION_INFO,
	CMD_REQUEST_MOUSE_EVENTS_DATA,
    //don't go beyond this command!
    CMD_MAX_CLIENT_COMMANDS,
    
};

enum EnumSharedMemoryServerStatus
{
        CMD_SHARED_MEMORY_NOT_INITIALIZED=0,
        CMD_WAITING_FOR_CLIENT_COMMAND,
        //CMD_CLIENT_COMMAND_COMPLETED is a generic 'completed' status that doesn't need special handling on the client
        CMD_CLIENT_COMMAND_COMPLETED,
        //the server will skip unknown command and report a status 'CMD_UNKNOWN_COMMAND_FLUSHED'
        CMD_UNKNOWN_COMMAND_FLUSHED,
		CMD_SDF_LOADING_COMPLETED,
        CMD_SDF_LOADING_FAILED,
        CMD_URDF_LOADING_COMPLETED,
        CMD_URDF_LOADING_FAILED,
		CMD_BULLET_LOADING_COMPLETED,
		CMD_BULLET_LOADING_FAILED,
		CMD_BULLET_SAVING_COMPLETED,
		CMD_BULLET_SAVING_FAILED,
		CMD_MJCF_LOADING_COMPLETED,
		CMD_MJCF_LOADING_FAILED,
		CMD_REQUEST_INTERNAL_DATA_COMPLETED,
		CMD_REQUEST_INTERNAL_DATA_FAILED,
        CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,
        CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,
        CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,
        CMD_RIGID_BODY_CREATION_COMPLETED,
        CMD_SET_JOINT_FEEDBACK_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_COMPLETED,
        CMD_ACTUAL_STATE_UPDATE_FAILED,
        CMD_DEBUG_LINES_COMPLETED,
        CMD_DEBUG_LINES_OVERFLOW_FAILED,
        CMD_DESIRED_STATE_RECEIVED_COMPLETED,
        CMD_STEP_FORWARD_SIMULATION_COMPLETED,
        CMD_RESET_SIMULATION_COMPLETED,
        CMD_CAMERA_IMAGE_COMPLETED,
        CMD_CAMERA_IMAGE_FAILED,
        CMD_BODY_INFO_COMPLETED,
        CMD_BODY_INFO_FAILED,
		CMD_INVALID_STATUS,
		CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,
		CMD_CALCULATED_INVERSE_DYNAMICS_FAILED,
        CMD_CALCULATED_JACOBIAN_COMPLETED,
        CMD_CALCULATED_JACOBIAN_FAILED,
		CMD_CONTACT_POINT_INFORMATION_COMPLETED,
		CMD_CONTACT_POINT_INFORMATION_FAILED,
		CMD_REQUEST_AABB_OVERLAP_COMPLETED,
		CMD_REQUEST_AABB_OVERLAP_FAILED,
		CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED,
		CMD_CALCULATE_INVERSE_KINEMATICS_FAILED,
		CMD_SAVE_WORLD_COMPLETED,
		CMD_SAVE_WORLD_FAILED,
        CMD_VISUAL_SHAPE_INFO_COMPLETED,
        CMD_VISUAL_SHAPE_INFO_FAILED,
        CMD_VISUAL_SHAPE_UPDATE_COMPLETED,
        CMD_VISUAL_SHAPE_UPDATE_FAILED,
        CMD_LOAD_TEXTURE_COMPLETED,
        CMD_LOAD_TEXTURE_FAILED,
		CMD_USER_DEBUG_DRAW_COMPLETED,
		CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,
		CMD_USER_DEBUG_DRAW_FAILED,
		CMD_USER_CONSTRAINT_COMPLETED,
		CMD_USER_CONSTRAINT_INFO_COMPLETED,
        CMD_REMOVE_USER_CONSTRAINT_COMPLETED,
        CMD_CHANGE_USER_CONSTRAINT_COMPLETED,
		CMD_REMOVE_USER_CONSTRAINT_FAILED,
        CMD_CHANGE_USER_CONSTRAINT_FAILED,
		CMD_USER_CONSTRAINT_FAILED,
		CMD_REQUEST_VR_EVENTS_DATA_COMPLETED,
		CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED,
		CMD_SYNC_BODY_INFO_COMPLETED,
		CMD_SYNC_BODY_INFO_FAILED,
		CMD_STATE_LOGGING_COMPLETED,
		CMD_STATE_LOGGING_START_COMPLETED,
		CMD_STATE_LOGGING_FAILED,
		CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED,
		CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED,
		CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED,
		CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED,
		CMD_REMOVE_BODY_COMPLETED,
		CMD_REMOVE_BODY_FAILED,
		CMD_GET_DYNAMICS_INFO_COMPLETED,
		CMD_GET_DYNAMICS_INFO_FAILED,
		CMD_CREATE_COLLISION_SHAPE_FAILED,
		CMD_CREATE_COLLISION_SHAPE_COMPLETED,
		CMD_CREATE_VISUAL_SHAPE_FAILED,
		CMD_CREATE_VISUAL_SHAPE_COMPLETED,
		CMD_CREATE_MULTI_BODY_FAILED,
		CMD_CREATE_MULTI_BODY_COMPLETED,
		CMD_REQUEST_COLLISION_INFO_COMPLETED,
		CMD_REQUEST_COLLISION_INFO_FAILED,
		CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED,
        //don't go beyond 'CMD_MAX_SERVER_COMMANDS!
        CMD_MAX_SERVER_COMMANDS
};

enum JointInfoFlags
{
    JOINT_HAS_MOTORIZED_POWER=1,
};

enum 
{
	COLLISION_SHAPE_TYPE_BOX=1,
	COLLISION_SHAPE_TYPE_CYLINDER_X,
	COLLISION_SHAPE_TYPE_CYLINDER_Y,
	COLLISION_SHAPE_TYPE_CYLINDER_Z,
	COLLISION_SHAPE_TYPE_CAPSULE_X,
	COLLISION_SHAPE_TYPE_CAPSULE_Y,
	COLLISION_SHAPE_TYPE_CAPSULE_Z,
	COLLISION_SHAPE_TYPE_SPHERE
};

// copied from btMultiBodyLink.h
enum JointType {
	eRevoluteType = 0,
	ePrismaticType = 1,
	eSphericalType = 2,
	ePlanarType = 3,
	eFixedType = 4,
	ePoint2PointType = 5,
	eGearType=6
};


enum b3JointInfoFlags
{
	eJointChangeMaxForce = 1,
	eJointChangeChildFramePosition = 2,
	eJointChangeChildFrameOrientation = 4,
};

struct b3JointInfo
{
        char* m_linkName;
        char* m_jointName;
        int m_jointType;
        int m_qIndex;
        int m_uIndex;
        int m_jointIndex;
        int m_flags;
		double m_jointDamping;
		double m_jointFriction;
		double m_jointLowerLimit;
		double m_jointUpperLimit;
		double m_jointMaxForce;
		double m_jointMaxVelocity;
		double m_parentFrame[7]; // position and orientation (quaternion)
		double m_childFrame[7]; // ^^^
		double m_jointAxis[3]; // joint axis in parent local frame
};


struct b3UserConstraint
{
    int m_parentBodyIndex;
    int m_parentJointIndex;
    int m_childBodyIndex;
    int m_childJointIndex;
    double m_parentFrame[7];
    double m_childFrame[7];
    double m_jointAxis[3];
    int m_jointType;
    double m_maxAppliedForce;
    int m_userConstraintUniqueId;
	double m_gearRatio;

};

struct b3BodyInfo
{
	const char* m_baseName;
	const char* m_bodyName; // for btRigidBody, it does not have a base, but can still have a body name from urdf
};

struct b3DynamicsInfo
{
	double m_mass;
	double m_localInertialPosition[3];
	double m_lateralFrictionCoeff;
};

// copied from btMultiBodyLink.h
enum SensorType {
	eSensorForceTorqueType = 1,
};


struct b3JointSensorState
{
  double m_jointPosition;
  double m_jointVelocity;
  double m_jointForceTorque[6];  /* note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint */
  double m_jointMotorTorque;
};

struct b3DebugLines
{
    int m_numDebugLines;
    const float*  m_linesFrom;//float x,y,z times 'm_numDebugLines'.
    const float*  m_linesTo;//float x,y,z times 'm_numDebugLines'.
    const float*  m_linesColor;//float red,green,blue times 'm_numDebugLines'.
};

struct b3OverlappingObject
{
	int m_objectUniqueId;
	int m_linkIndex;
};

struct b3AABBOverlapData
{
    int m_numOverlappingObjects;
	struct b3OverlappingObject* m_overlappingObjects;
};

struct b3CameraImageData
{
	int m_pixelWidth;
	int m_pixelHeight;
	const unsigned char* m_rgbColorData;//3*m_pixelWidth*m_pixelHeight bytes
	const float* m_depthValues;//m_pixelWidth*m_pixelHeight floats
	const int* m_segmentationMaskValues;//m_pixelWidth*m_pixelHeight ints
};

struct b3OpenGLVisualizerCameraInfo
{
    int m_width;
    int m_height;
	float m_viewMatrix[16];
	float m_projectionMatrix[16];
	
	float m_camUp[3];
	float m_camForward[3];

	float m_horizontal[3];
	float m_vertical[3];
	
	float m_yaw;
	float m_pitch;
	float m_dist;
	float m_target[3];
};

enum b3VREventType
{
	VR_CONTROLLER_MOVE_EVENT=1,
	VR_CONTROLLER_BUTTON_EVENT=2,
	VR_HMD_MOVE_EVENT=4,
	VR_GENERIC_TRACKER_MOVE_EVENT=8,
};

#define MAX_VR_BUTTONS 64
#define MAX_VR_CONTROLLERS 8

#define MAX_RAY_INTERSECTION_BATCH_SIZE 256
#define MAX_RAY_HITS MAX_RAY_INTERSECTION_BATCH_SIZE
#define MAX_KEYBOARD_EVENTS 256
#define MAX_MOUSE_EVENTS 256


enum b3VRButtonInfo
{
	eButtonIsDown = 1,
	eButtonTriggered = 2,
	eButtonReleased = 4,
};



enum eVRDeviceTypeEnums
{
	VR_DEVICE_CONTROLLER=1,
	VR_DEVICE_HMD=2,
	VR_DEVICE_GENERIC_TRACKER=4,
};

enum EVRCameraFlags
{
	VR_CAMERA_TRACK_OBJECT_ORIENTATION=1,
};

struct b3VRControllerEvent
{
	int m_controllerId;//valid for VR_CONTROLLER_MOVE_EVENT and VR_CONTROLLER_BUTTON_EVENT
	int m_deviceType;
	int m_numMoveEvents;
	int m_numButtonEvents;
	
	float m_pos[4];//valid for VR_CONTROLLER_MOVE_EVENT and VR_CONTROLLER_BUTTON_EVENT
	float m_orn[4];//valid for VR_CONTROLLER_MOVE_EVENT and VR_CONTROLLER_BUTTON_EVENT

	float m_analogAxis;//valid if VR_CONTROLLER_MOVE_EVENT

	int m_buttons[MAX_VR_BUTTONS];//valid if VR_CONTROLLER_BUTTON_EVENT, see b3VRButtonInfo
};

struct b3VREventsData
{
	int m_numControllerEvents;
	struct b3VRControllerEvent* m_controllerEvents;
	
};


struct b3KeyboardEvent
{
	int m_keyCode;//ascii
	int m_keyState;// see b3VRButtonInfo
};

struct b3KeyboardEventsData
{
	int m_numKeyboardEvents;
	struct b3KeyboardEvent* m_keyboardEvents;
};


enum eMouseEventTypeEnums
{
	MOUSE_MOVE_EVENT=1,
	MOUSE_BUTTON_EVENT=2,
};

struct b3MouseEvent
{
	int m_eventType;
	float m_mousePosX;
	float m_mousePosY;
	int m_buttonIndex;
	int m_buttonState;
};

struct b3MouseEventsData
{
	int m_numMouseEvents;
	struct b3MouseEvent* m_mouseEvents;
};

struct b3ContactPointData
{
//todo: expose some contact flags, such as telling which fields below are valid
    int m_contactFlags;
    int m_bodyUniqueIdA;
    int m_bodyUniqueIdB;
    int m_linkIndexA;
    int m_linkIndexB;
    double m_positionOnAInWS[3];//contact point location on object A, in world space coordinates
    double m_positionOnBInWS[3];//contact point location on object A, in world space coordinates
    double m_contactNormalOnBInWS[3];//the separating contact normal, pointing from object B towards object A
    double m_contactDistance;//negative number is penetration, positive is distance.
    
    double m_normalForce;

//todo: expose the friction forces as well
//    double m_linearFrictionDirection0[3];
//    double m_linearFrictionForce0;
//    double m_linearFrictionDirection1[3];
//    double m_linearFrictionForce1;
//    double m_angularFrictionDirection[3];
//    double m_angularFrictionForce;
};

enum 
{
	CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS = 0,
	CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS = 1,
};

enum  b3StateLoggingType
{
	STATE_LOGGING_MINITAUR = 0,
	STATE_LOGGING_GENERIC_ROBOT = 1,
	STATE_LOGGING_VR_CONTROLLERS = 2,
	STATE_LOGGING_VIDEO_MP4 = 3,
	STATE_LOGGING_COMMANDS = 4,
	STATE_LOGGING_CONTACT_POINTS = 5,
	STATE_LOGGING_PROFILE_TIMINGS = 6,
};


struct b3ContactInformation
{
	int m_numContactPoints;
	struct b3ContactPointData* m_contactPointData;
};

struct b3RayHitInfo
{
	double m_hitFraction;
	int m_hitObjectUniqueId;
	int m_hitObjectLinkIndex;
	double m_hitPositionWorld[3];
	double m_hitNormalWorld[3];
};

struct b3RaycastInformation
{
	int m_numRayHits;
	struct b3RayHitInfo* m_rayHits;
};


#define VISUAL_SHAPE_MAX_PATH_LEN 1024

struct b3VisualShapeData
{
	int m_objectUniqueId;
	int m_linkIndex;
	int m_visualGeometryType;//box primitive, sphere primitive, triangle mesh
	double m_dimensions[3];//meaning depends on m_visualGeometryType
	char m_meshAssetFileName[VISUAL_SHAPE_MAX_PATH_LEN];
    double m_localVisualFrame[7];//pos[3], orn[4]
	//todo: add more data if necessary (material color etc, although material can be in asset file .obj file)
    double m_rgbaColor[4];
};

struct b3VisualShapeInformation
{
	int m_numVisualShapes;
	struct b3VisualShapeData* m_visualShapeData;
};

enum eLinkStateFlags
{
	ACTUAL_STATE_COMPUTE_LINKVELOCITY=1
};

///b3LinkState provides extra information such as the Cartesian world coordinates
///center of mass (COM) of the link, relative to the world reference frame.
///Orientation is a quaternion x,y,z,w
///Note: to compute the URDF link frame (which equals the joint frame at joint position 0)
///use URDF link frame = link COM frame * inertiaFrame.inverse()
struct b3LinkState
{
	//m_worldPosition and m_worldOrientation of the Center Of Mass (COM)
    double m_worldPosition[3];
    double m_worldOrientation[4];

    double m_localInertialPosition[3];
    double m_localInertialOrientation[4];

	///world position and orientation of the (URDF) link frame
	double m_worldLinkFramePosition[3];
    double m_worldLinkFrameOrientation[4];

	double m_worldLinearVelocity[3]; //only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)
	double m_worldAngularVelocity[3]; //only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)

	double m_worldAABBMin[3];//world space bounding minium and maximum box corners.
	double m_worldAABBMax[3];
};

//todo: discuss and decide about control mode and combinations
enum {
    //    POSITION_CONTROL=0,
    CONTROL_MODE_VELOCITY=0,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_POSITION_VELOCITY_PD,
};

///flags for b3ApplyExternalTorque and b3ApplyExternalForce
enum EnumExternalForceFlags
{
    EF_LINK_FRAME=1,
    EF_WORLD_FRAME=2,
};

///flags to pick the renderer for synthetic camera
enum EnumRenderer
{
    ER_TINY_RENDERER=(1<<16),
    ER_BULLET_HARDWARE_OPENGL=(1<<17),
    //ER_FIRE_RAYS=(1<<18),
};

enum b3ConfigureDebugVisualizerEnum
{
    COV_ENABLE_GUI=1,
    COV_ENABLE_SHADOWS,
    COV_ENABLE_WIREFRAME,
	COV_ENABLE_VR_TELEPORTING,
	COV_ENABLE_VR_PICKING,
	COV_ENABLE_VR_RENDER_CONTROLLERS,
	COV_ENABLE_RENDERING,
	COV_ENABLE_SYNC_RENDERING_INTERNAL,
	COV_ENABLE_KEYBOARD_SHORTCUTS,
	COV_ENABLE_MOUSE_PICKING,
};

enum b3AddUserDebugItemEnum
{
	DEB_DEBUG_TEXT_USE_ORIENTATION=1,
	DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2,
	DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4,
};

enum eCONNECT_METHOD {
  eCONNECT_GUI = 1,
  eCONNECT_DIRECT = 2,
  eCONNECT_SHARED_MEMORY = 3,
  eCONNECT_UDP = 4,
  eCONNECT_TCP = 5,
  eCONNECT_EXISTING_EXAMPLE_BROWSER=6,
};

enum eURDF_Flags
{
	URDF_USE_INERTIA_FROM_FILE=2,//sync with URDF2Bullet.h 'ConvertURDFFlags'
	URDF_USE_SELF_COLLISION=8,//see CUF_USE_SELF_COLLISION
	URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16,
	URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32,
};

enum eUrdfGeomTypes //sync with UrdfParser UrdfGeomTypes
{
	GEOM_SPHERE=2,
	GEOM_BOX,
	GEOM_CYLINDER,
	GEOM_MESH,
	GEOM_PLANE,
	GEOM_CAPSULE, //non-standard URDF?
	GEOM_UNKNOWN, 
};

enum eUrdfCollisionFlags
{
	GEOM_FORCE_CONCAVE_TRIMESH=1,
};




#endif//SHARED_MEMORY_PUBLIC_H
