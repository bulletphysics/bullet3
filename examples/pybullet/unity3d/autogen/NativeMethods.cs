
public partial class NativeConstants {
    
    /// PHYSICS_CLIENT_C_API_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_C_API_H = "";
    
    /// SHARED_MEMORY_PUBLIC_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string SHARED_MEMORY_PUBLIC_H = "";
    
    /// SHARED_MEMORY_KEY -> 12347
    public const int SHARED_MEMORY_KEY = 12347;
    
    /// SHARED_MEMORY_MAGIC_NUMBER -> 201708270
    public const int SHARED_MEMORY_MAGIC_NUMBER = 201708270;
    
    /// MAX_VR_BUTTONS -> 64
    public const int MAX_VR_BUTTONS = 64;
    
    /// MAX_VR_CONTROLLERS -> 8
    public const int MAX_VR_CONTROLLERS = 8;
    
    /// MAX_RAY_INTERSECTION_BATCH_SIZE -> 256
    public const int MAX_RAY_INTERSECTION_BATCH_SIZE = 256;
    
    /// MAX_RAY_HITS -> MAX_RAY_INTERSECTION_BATCH_SIZE
    public const int MAX_RAY_HITS = NativeConstants.MAX_RAY_INTERSECTION_BATCH_SIZE;
    
    /// MAX_KEYBOARD_EVENTS -> 256
    public const int MAX_KEYBOARD_EVENTS = 256;
    
    /// MAX_MOUSE_EVENTS -> 256
    public const int MAX_MOUSE_EVENTS = 256;
    
    /// VISUAL_SHAPE_MAX_PATH_LEN -> 1024
    public const int VISUAL_SHAPE_MAX_PATH_LEN = 1024;
    
    /// Warning: Generation of Method Macros is not supported at this time
    /// B3_DECLARE_HANDLE -> "(name) typedef struct name##__ { int unused; } *name"
    public const string B3_DECLARE_HANDLE = "(name) typedef struct name##__ { int unused; } *name";
    
    /// PHYSICS_CLIENT_SHARED_MEMORY_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_SHARED_MEMORY_H = "";
    
    /// PHYSICS_CLIENT_SHARED_MEMORY2_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_CLIENT_SHARED_MEMORY2_H = "";
    
    /// PHYSICS_DIRECT_C_API_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string PHYSICS_DIRECT_C_API_H = "";
    
    /// IN_PROCESS_PHYSICS_C_API_H -> 
    /// Error generating expression: Value cannot be null.
    ///Parameter name: node
    public const string IN_PROCESS_PHYSICS_C_API_H = "";
}

public enum EnumSharedMemoryClientCommand {
    
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
    
    CMD_CREATE_SENSOR,
    
    CMD_INIT_POSE,
    
    CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
    
    CMD_SEND_DESIRED_STATE,
    
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
    
    CMD_CHANGE_TEXTURE,
    
    CMD_SET_ADDITIONAL_SEARCH_PATH,
    
    CMD_MAX_CLIENT_COMMANDS,
}

public enum EnumSharedMemoryServerStatus {
    
    /// CMD_SHARED_MEMORY_NOT_INITIALIZED -> 0
    CMD_SHARED_MEMORY_NOT_INITIALIZED = 0,
    
    CMD_WAITING_FOR_CLIENT_COMMAND,
    
    CMD_CLIENT_COMMAND_COMPLETED,
    
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
    
    CMD_CHANGE_TEXTURE_COMMAND_FAILED,
    
    CMD_MAX_SERVER_COMMANDS,
}

public enum JointInfoFlags {
    
    /// JOINT_HAS_MOTORIZED_POWER -> 1
    JOINT_HAS_MOTORIZED_POWER = 1,
}

public enum JointType {
    
    /// eRevoluteType -> 0
    eRevoluteType = 0,
    
    /// ePrismaticType -> 1
    ePrismaticType = 1,
    
    /// eSphericalType -> 2
    eSphericalType = 2,
    
    /// ePlanarType -> 3
    ePlanarType = 3,
    
    /// eFixedType -> 4
    eFixedType = 4,
    
    /// ePoint2PointType -> 5
    ePoint2PointType = 5,
    
    /// eGearType -> 6
    eGearType = 6,
}

public enum b3JointInfoFlags {
    
    /// eJointChangeMaxForce -> 1
    eJointChangeMaxForce = 1,
    
    /// eJointChangeChildFramePosition -> 2
    eJointChangeChildFramePosition = 2,
    
    /// eJointChangeChildFrameOrientation -> 4
    eJointChangeChildFrameOrientation = 4,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3JointInfo {
    
    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_linkName;
    
    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_jointName;
    
    /// int
    public int m_jointType;
    
    /// int
    public int m_qIndex;
    
    /// int
    public int m_uIndex;
    
    /// int
    public int m_jointIndex;
    
    /// int
    public int m_flags;
    
    /// double
    public double m_jointDamping;
    
    /// double
    public double m_jointFriction;
    
    /// double
    public double m_jointLowerLimit;
    
    /// double
    public double m_jointUpperLimit;
    
    /// double
    public double m_jointMaxForce;
    
    /// double
    public double m_jointMaxVelocity;
    
    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=7, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_parentFrame;
    
    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=7, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_childFrame;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointAxis;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3UserConstraint {
    
    /// int
    public int m_parentBodyIndex;
    
    /// int
    public int m_parentJointIndex;
    
    /// int
    public int m_childBodyIndex;
    
    /// int
    public int m_childJointIndex;
    
    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=7, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_parentFrame;
    
    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=7, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_childFrame;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointAxis;
    
    /// int
    public int m_jointType;
    
    /// double
    public double m_maxAppliedForce;
    
    /// int
    public int m_userConstraintUniqueId;
    
    /// double
    public double m_gearRatio;
    
    /// int
    public int m_gearAuxLink;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3BodyInfo {
    
    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_baseName;
    
    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_bodyName;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3DynamicsInfo {
    
    /// double
    public double m_mass;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialPosition;
    
    /// double
    public double m_lateralFrictionCoeff;
}

public enum SensorType {
    
    /// eSensorForceTorqueType -> 1
    eSensorForceTorqueType = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3JointSensorState {
    
    /// double
    public double m_jointPosition;
    
    /// double
    public double m_jointVelocity;
    
    /// double[6]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=6, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_jointForceTorque;
    
    /// double
    public double m_jointMotorTorque;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3DebugLines {
    
    /// int
    public int m_numDebugLines;
    
    /// float*
    public System.IntPtr m_linesFrom;
    
    /// float*
    public System.IntPtr m_linesTo;
    
    /// float*
    public System.IntPtr m_linesColor;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3OverlappingObject {
    
    /// int
    public int m_objectUniqueId;
    
    /// int
    public int m_linkIndex;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3AABBOverlapData {
    
    /// int
    public int m_numOverlappingObjects;
    
    /// b3OverlappingObject*
    public System.IntPtr m_overlappingObjects;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3CameraImageData {
    
    /// int
    public int m_pixelWidth;
    
    /// int
    public int m_pixelHeight;
    
    /// char*
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)]
    public string m_rgbColorData;
    
    /// float*
    public System.IntPtr m_depthValues;
    
    /// int*
    public System.IntPtr m_segmentationMaskValues;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3OpenGLVisualizerCameraInfo {
    
    /// int
    public int m_width;
    
    /// int
    public int m_height;
    
    /// float[16]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=16, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_viewMatrix;
    
    /// float[16]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=16, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_projectionMatrix;
    
    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_camUp;
    
    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_camForward;
    
    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_horizontal;
    
    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_vertical;
    
    /// float
    public float m_yaw;
    
    /// float
    public float m_pitch;
    
    /// float
    public float m_dist;
    
    /// float[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_target;
}

public enum b3VREventType {
    
    /// VR_CONTROLLER_MOVE_EVENT -> 1
    VR_CONTROLLER_MOVE_EVENT = 1,
    
    /// VR_CONTROLLER_BUTTON_EVENT -> 2
    VR_CONTROLLER_BUTTON_EVENT = 2,
    
    /// VR_HMD_MOVE_EVENT -> 4
    VR_HMD_MOVE_EVENT = 4,
    
    /// VR_GENERIC_TRACKER_MOVE_EVENT -> 8
    VR_GENERIC_TRACKER_MOVE_EVENT = 8,
}

public enum b3VRButtonInfo {
    
    /// eButtonIsDown -> 1
    eButtonIsDown = 1,
    
    /// eButtonTriggered -> 2
    eButtonTriggered = 2,
    
    /// eButtonReleased -> 4
    eButtonReleased = 4,
}

public enum eVRDeviceTypeEnums {
    
    /// VR_DEVICE_CONTROLLER -> 1
    VR_DEVICE_CONTROLLER = 1,
    
    /// VR_DEVICE_HMD -> 2
    VR_DEVICE_HMD = 2,
    
    /// VR_DEVICE_GENERIC_TRACKER -> 4
    VR_DEVICE_GENERIC_TRACKER = 4,
}

public enum EVRCameraFlags {
    
    /// VR_CAMERA_TRACK_OBJECT_ORIENTATION -> 1
    VR_CAMERA_TRACK_OBJECT_ORIENTATION = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VRControllerEvent {
    
    /// int
    public int m_controllerId;
    
    /// int
    public int m_deviceType;
    
    /// int
    public int m_numMoveEvents;
    
    /// int
    public int m_numButtonEvents;
    
    /// float[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_pos;
    
    /// float[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R4)]
    public float[] m_orn;
    
    /// float
    public float m_analogAxis;
    
    /// int[64]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=64, ArraySubType=System.Runtime.InteropServices.UnmanagedType.I4)]
    public int[] m_buttons;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VREventsData {
    
    /// int
    public int m_numControllerEvents;
    
    /// b3VRControllerEvent*
    public System.IntPtr m_controllerEvents;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3KeyboardEvent {
    
    /// int
    public int m_keyCode;
    
    /// int
    public int m_keyState;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3KeyboardEventsData {
    
    /// int
    public int m_numKeyboardEvents;
    
    /// b3KeyboardEvent*
    public System.IntPtr m_keyboardEvents;
}

public enum eMouseEventTypeEnums {
    
    /// MOUSE_MOVE_EVENT -> 1
    MOUSE_MOVE_EVENT = 1,
    
    /// MOUSE_BUTTON_EVENT -> 2
    MOUSE_BUTTON_EVENT = 2,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MouseEvent {
    
    /// int
    public int m_eventType;
    
    /// float
    public float m_mousePosX;
    
    /// float
    public float m_mousePosY;
    
    /// int
    public int m_buttonIndex;
    
    /// int
    public int m_buttonState;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3MouseEventsData {
    
    /// int
    public int m_numMouseEvents;
    
    /// b3MouseEvent*
    public System.IntPtr m_mouseEvents;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ContactPointData {
    
    /// int
    public int m_contactFlags;
    
    /// int
    public int m_bodyUniqueIdA;
    
    /// int
    public int m_bodyUniqueIdB;
    
    /// int
    public int m_linkIndexA;
    
    /// int
    public int m_linkIndexB;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_positionOnAInWS;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_positionOnBInWS;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_contactNormalOnBInWS;
    
    /// double
    public double m_contactDistance;
    
    /// double
    public double m_normalForce;
}

public enum b3StateLoggingType {
    
    /// STATE_LOGGING_MINITAUR -> 0
    STATE_LOGGING_MINITAUR = 0,
    
    /// STATE_LOGGING_GENERIC_ROBOT -> 1
    STATE_LOGGING_GENERIC_ROBOT = 1,
    
    /// STATE_LOGGING_VR_CONTROLLERS -> 2
    STATE_LOGGING_VR_CONTROLLERS = 2,
    
    /// STATE_LOGGING_VIDEO_MP4 -> 3
    STATE_LOGGING_VIDEO_MP4 = 3,
    
    /// STATE_LOGGING_COMMANDS -> 4
    STATE_LOGGING_COMMANDS = 4,
    
    /// STATE_LOGGING_CONTACT_POINTS -> 5
    STATE_LOGGING_CONTACT_POINTS = 5,
    
    /// STATE_LOGGING_PROFILE_TIMINGS -> 6
    STATE_LOGGING_PROFILE_TIMINGS = 6,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3ContactInformation {
    
    /// int
    public int m_numContactPoints;
    
    /// b3ContactPointData*
    public System.IntPtr m_contactPointData;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3RayHitInfo {
    
    /// double
    public double m_hitFraction;
    
    /// int
    public int m_hitObjectUniqueId;
    
    /// int
    public int m_hitObjectLinkIndex;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_hitPositionWorld;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_hitNormalWorld;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3RaycastInformation {
    
    /// int
    public int m_numRayHits;
    
    /// b3RayHitInfo*
    public System.IntPtr m_rayHits;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential, CharSet=System.Runtime.InteropServices.CharSet.Ansi)]
public struct b3VisualShapeData {
    
    /// int
    public int m_objectUniqueId;
    
    /// int
    public int m_linkIndex;
    
    /// int
    public int m_visualGeometryType;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_dimensions;
    
    /// char[1024]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValTStr, SizeConst=1024)]
    public string m_meshAssetFileName;
    
    /// double[7]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=7, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localVisualFrame;
    
    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_rgbaColor;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3VisualShapeInformation {
    
    /// int
    public int m_numVisualShapes;
    
    /// b3VisualShapeData*
    public System.IntPtr m_visualShapeData;
}

public enum eLinkStateFlags {
    
    /// ACTUAL_STATE_COMPUTE_LINKVELOCITY -> 1
    ACTUAL_STATE_COMPUTE_LINKVELOCITY = 1,
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3LinkState {
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldPosition;
    
    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldOrientation;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialPosition;
    
    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_localInertialOrientation;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinkFramePosition;
    
    /// double[4]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=4, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinkFrameOrientation;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldLinearVelocity;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAngularVelocity;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAABBMin;
    
    /// double[3]
    [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.ByValArray, SizeConst=3, ArraySubType=System.Runtime.InteropServices.UnmanagedType.R8)]
    public double[] m_worldAABBMax;
}

public enum EnumExternalForceFlags {
    
    /// EF_LINK_FRAME -> 1
    EF_LINK_FRAME = 1,
    
    /// EF_WORLD_FRAME -> 2
    EF_WORLD_FRAME = 2,
}

public enum EnumRenderer {
    
    /// ER_TINY_RENDERER -> (1<<16)
    ER_TINY_RENDERER = (1) << (16),
    
    /// ER_BULLET_HARDWARE_OPENGL -> (1<<17)
    ER_BULLET_HARDWARE_OPENGL = (1) << (17),
}

public enum b3ConfigureDebugVisualizerEnum {
    
    /// COV_ENABLE_GUI -> 1
    COV_ENABLE_GUI = 1,
    
    COV_ENABLE_SHADOWS,
    
    COV_ENABLE_WIREFRAME,
    
    COV_ENABLE_VR_TELEPORTING,
    
    COV_ENABLE_VR_PICKING,
    
    COV_ENABLE_VR_RENDER_CONTROLLERS,
    
    COV_ENABLE_RENDERING,
    
    COV_ENABLE_SYNC_RENDERING_INTERNAL,
    
    COV_ENABLE_KEYBOARD_SHORTCUTS,
    
    COV_ENABLE_MOUSE_PICKING,
}

public enum b3AddUserDebugItemEnum {
    
    /// DEB_DEBUG_TEXT_USE_ORIENTATION -> 1
    DEB_DEBUG_TEXT_USE_ORIENTATION = 1,
    
    /// DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS -> 2
    DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS = 2,
    
    /// DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT -> 4
    DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT = 4,
}

public enum eCONNECT_METHOD {
    
    /// eCONNECT_GUI -> 1
    eCONNECT_GUI = 1,
    
    /// eCONNECT_DIRECT -> 2
    eCONNECT_DIRECT = 2,
    
    /// eCONNECT_SHARED_MEMORY -> 3
    eCONNECT_SHARED_MEMORY = 3,
    
    /// eCONNECT_UDP -> 4
    eCONNECT_UDP = 4,
    
    /// eCONNECT_TCP -> 5
    eCONNECT_TCP = 5,
    
    /// eCONNECT_EXISTING_EXAMPLE_BROWSER -> 6
    eCONNECT_EXISTING_EXAMPLE_BROWSER = 6,
    
    /// eCONNECT_GUI_SERVER -> 7
    eCONNECT_GUI_SERVER = 7,
}

public enum eURDF_Flags {
    
    /// URDF_USE_INERTIA_FROM_FILE -> 2
    URDF_USE_INERTIA_FROM_FILE = 2,
    
    /// URDF_USE_SELF_COLLISION -> 8
    URDF_USE_SELF_COLLISION = 8,
    
    /// URDF_USE_SELF_COLLISION_EXCLUDE_PARENT -> 16
    URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
    
    /// URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS -> 32
    URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
    
    /// URDF_RESERVED -> 64
    URDF_RESERVED = 64,
}

public enum eUrdfGeomTypes {
    
    /// GEOM_SPHERE -> 2
    GEOM_SPHERE = 2,
    
    GEOM_BOX,
    
    GEOM_CYLINDER,
    
    GEOM_MESH,
    
    GEOM_PLANE,
    
    GEOM_CAPSULE,
    
    GEOM_UNKNOWN,
}

public enum eUrdfCollisionFlags {
    
    /// GEOM_FORCE_CONCAVE_TRIMESH -> 1
    GEOM_FORCE_CONCAVE_TRIMESH = 1,
}

public enum eStateLoggingFlags {
    
    /// STATE_LOG_JOINT_MOTOR_TORQUES -> 1
    STATE_LOG_JOINT_MOTOR_TORQUES = 1,
    
    /// STATE_LOG_JOINT_USER_TORQUES -> 2
    STATE_LOG_JOINT_USER_TORQUES = 2,
    
    /// STATE_LOG_JOINT_TORQUES -> STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES
    STATE_LOG_JOINT_TORQUES = (eStateLoggingFlags.STATE_LOG_JOINT_MOTOR_TORQUES + eStateLoggingFlags.STATE_LOG_JOINT_USER_TORQUES),
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3PhysicsClientHandle__ {
    
    /// int
    public int unused;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3SharedMemoryCommandHandle__ {
    
    /// int
    public int unused;
}

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public struct b3SharedMemoryStatusHandle__ {
    
    /// int
    public int unused;
}

public partial class NativeMethods {
    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///key: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ConnectSharedMemory")]
public static extern  System.IntPtr b3ConnectSharedMemory(int key) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///key: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ConnectSharedMemory2")]
public static extern  System.IntPtr b3ConnectSharedMemory2(int key) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ConnectPhysicsDirect")]
public static extern  System.IntPtr b3ConnectPhysicsDirect() ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///argc: int
    ///argv: char**
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateInProcessPhysicsServerAndConnect")]
public static extern  System.IntPtr b3CreateInProcessPhysicsServerAndConnect(int argc, ref System.IntPtr argv) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///argc: int
    ///argv: char**
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateInProcessPhysicsServerAndConnectSharedMemory")]
public static extern  System.IntPtr b3CreateInProcessPhysicsServerAndConnectSharedMemory(int argc, ref System.IntPtr argv) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///argc: int
    ///argv: char**
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateInProcessPhysicsServerAndConnectMainThread")]
public static extern  System.IntPtr b3CreateInProcessPhysicsServerAndConnectMainThread(int argc, ref System.IntPtr argv) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///argc: int
    ///argv: char**
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory")]
public static extern  System.IntPtr b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(int argc, ref System.IntPtr argv) ;

    
    /// Return Type: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///guiHelperPtr: void*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect")]
public static extern  System.IntPtr b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(System.IntPtr guiHelperPtr) ;

    
    /// Return Type: void
    ///clientHandle: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InProcessRenderSceneInternal")]
public static extern  void b3InProcessRenderSceneInternal(ref b3PhysicsClientHandle__ clientHandle) ;

    
    /// Return Type: void
    ///clientHandle: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugDrawMode: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InProcessDebugDrawInternal")]
public static extern  void b3InProcessDebugDrawInternal(ref b3PhysicsClientHandle__ clientHandle, int debugDrawMode) ;

    
    /// Return Type: int
    ///clientHandle: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///x: float
    ///y: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InProcessMouseMoveCallback")]
public static extern  int b3InProcessMouseMoveCallback(ref b3PhysicsClientHandle__ clientHandle, float x, float y) ;

    
    /// Return Type: int
    ///clientHandle: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///button: int
    ///state: int
    ///x: float
    ///y: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InProcessMouseButtonCallback")]
public static extern  int b3InProcessMouseButtonCallback(ref b3PhysicsClientHandle__ clientHandle, int button, int state, float x, float y) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3DisconnectSharedMemory")]
public static extern  void b3DisconnectSharedMemory(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CanSubmitCommand")]
public static extern  int b3CanSubmitCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SubmitClientCommandAndWaitStatus")]
public static extern  System.IntPtr b3SubmitClientCommandAndWaitStatus(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryCommandHandle__ commandHandle) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SubmitClientCommand")]
public static extern  int b3SubmitClientCommand(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryCommandHandle__ commandHandle) ;

    
    /// Return Type: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ProcessServerStatus")]
public static extern  System.IntPtr b3ProcessServerStatus(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusType")]
public static extern  int b3GetStatusType(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyIndicesOut: int*
    ///bodyIndicesCapacity: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusBodyIndices")]
public static extern  int b3GetStatusBodyIndices(ref b3SharedMemoryStatusHandle__ statusHandle, ref int bodyIndicesOut, int bodyIndicesCapacity) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusBodyIndex")]
public static extern  int b3GetStatusBodyIndex(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///numDegreeOfFreedomQ: int*
    ///numDegreeOfFreedomU: int*
    ///rootLocalInertialFrame: double**
    ///actualStateQ: double**
    ///actualStateQdot: double**
    ///jointReactionForces: double**
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusActualState")]
public static extern  int b3GetStatusActualState(ref b3SharedMemoryStatusHandle__ statusHandle, ref int bodyUniqueId, ref int numDegreeOfFreedomQ, ref int numDegreeOfFreedomU, ref System.IntPtr rootLocalInertialFrame, ref System.IntPtr actualStateQ, ref System.IntPtr actualStateQdot, ref System.IntPtr jointReactionForces) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCollisionInfoCommandInit")]
public static extern  System.IntPtr b3RequestCollisionInfoCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///linkIndex: int
    ///aabbMin: double*
    ///aabbMax: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusAABB")]
public static extern  int b3GetStatusAABB(ref b3SharedMemoryStatusHandle__ statusHandle, int linkIndex, ref double aabbMin, ref double aabbMax) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitSyncBodyInfoCommand")]
public static extern  System.IntPtr b3InitSyncBodyInfoCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRemoveBodyCommand")]
public static extern  System.IntPtr b3InitRemoveBodyCommand(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetNumBodies")]
public static extern  int b3GetNumBodies(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///serialIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetBodyUniqueId")]
public static extern  int b3GetBodyUniqueId(ref b3PhysicsClientHandle__ physClient, int serialIndex) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///info: b3BodyInfo*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetBodyInfo")]
public static extern  int b3GetBodyInfo(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId, ref b3BodyInfo info) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetNumJoints")]
public static extern  int b3GetNumJoints(ref b3PhysicsClientHandle__ physClient, int bodyIndex) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    ///jointIndex: int
    ///info: b3JointInfo*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetJointInfo")]
public static extern  int b3GetJointInfo(ref b3PhysicsClientHandle__ physClient, int bodyIndex, int jointIndex, ref b3JointInfo info) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetDynamicsInfoCommandInit")]
public static extern  System.IntPtr b3GetDynamicsInfoCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId, int linkIndex) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///info: b3DynamicsInfo*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetDynamicsInfo")]
public static extern  int b3GetDynamicsInfo(ref b3SharedMemoryStatusHandle__ statusHandle, ref b3DynamicsInfo info) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeDynamicsInfo")]
public static extern  System.IntPtr b3InitChangeDynamicsInfo(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetMass")]
public static extern  int b3ChangeDynamicsInfoSetMass(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double mass) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///lateralFriction: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetLateralFriction")]
public static extern  int b3ChangeDynamicsInfoSetLateralFriction(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double lateralFriction) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///friction: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetSpinningFriction")]
public static extern  int b3ChangeDynamicsInfoSetSpinningFriction(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double friction) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///friction: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetRollingFriction")]
public static extern  int b3ChangeDynamicsInfoSetRollingFriction(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double friction) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///restitution: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetRestitution")]
public static extern  int b3ChangeDynamicsInfoSetRestitution(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double restitution) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linearDamping: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetLinearDamping")]
public static extern  int b3ChangeDynamicsInfoSetLinearDamping(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, double linearDamping) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///angularDamping: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetAngularDamping")]
public static extern  int b3ChangeDynamicsInfoSetAngularDamping(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, double angularDamping) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///contactStiffness: double
    ///contactDamping: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetContactStiffnessAndDamping")]
public static extern  int b3ChangeDynamicsInfoSetContactStiffnessAndDamping(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, double contactStiffness, double contactDamping) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkIndex: int
    ///frictionAnchor: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ChangeDynamicsInfoSetFrictionAnchor")]
public static extern  int b3ChangeDynamicsInfoSetFrictionAnchor(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkIndex, int frictionAnchor) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///parentBodyIndex: int
    ///parentJointIndex: int
    ///childBodyIndex: int
    ///childJointIndex: int
    ///info: b3JointInfo*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitCreateUserConstraintCommand")]
public static extern  System.IntPtr b3InitCreateUserConstraintCommand(ref b3PhysicsClientHandle__ physClient, int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, ref b3JointInfo info) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusUserConstraintUniqueId")]
public static extern  int b3GetStatusUserConstraintUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userConstraintUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintCommand")]
public static extern  System.IntPtr b3InitChangeUserConstraintCommand(ref b3PhysicsClientHandle__ physClient, int userConstraintUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointChildPivot: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintSetPivotInB")]
public static extern  int b3InitChangeUserConstraintSetPivotInB(ref b3SharedMemoryCommandHandle__ commandHandle, ref double jointChildPivot) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointChildFrameOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintSetFrameInB")]
public static extern  int b3InitChangeUserConstraintSetFrameInB(ref b3SharedMemoryCommandHandle__ commandHandle, ref double jointChildFrameOrn) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxAppliedForce: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintSetMaxForce")]
public static extern  int b3InitChangeUserConstraintSetMaxForce(ref b3SharedMemoryCommandHandle__ commandHandle, double maxAppliedForce) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gearRatio: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintSetGearRatio")]
public static extern  int b3InitChangeUserConstraintSetGearRatio(ref b3SharedMemoryCommandHandle__ commandHandle, double gearRatio) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gearAuxLink: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitChangeUserConstraintSetGearAuxLink")]
public static extern  int b3InitChangeUserConstraintSetGearAuxLink(ref b3SharedMemoryCommandHandle__ commandHandle, int gearAuxLink) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///userConstraintUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRemoveUserConstraintCommand")]
public static extern  System.IntPtr b3InitRemoveUserConstraintCommand(ref b3PhysicsClientHandle__ physClient, int userConstraintUniqueId) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetNumUserConstraints")]
public static extern  int b3GetNumUserConstraints(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///constraintUniqueId: int
    ///info: b3UserConstraint*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetUserConstraintInfo")]
public static extern  int b3GetUserConstraintInfo(ref b3PhysicsClientHandle__ physClient, int constraintUniqueId, ref b3UserConstraint info) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///serialIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetUserConstraintId")]
public static extern  int b3GetUserConstraintId(ref b3PhysicsClientHandle__ physClient, int serialIndex) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugMode: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRequestDebugLinesCommand")]
public static extern  System.IntPtr b3InitRequestDebugLinesCommand(ref b3PhysicsClientHandle__ physClient, int debugMode) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///lines: b3DebugLines*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetDebugLines")]
public static extern  void b3GetDebugLines(ref b3PhysicsClientHandle__ physClient, ref b3DebugLines lines) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitConfigureOpenGLVisualizer")]
public static extern  System.IntPtr b3InitConfigureOpenGLVisualizer(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flag: int
    ///enabled: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ConfigureOpenGLVisualizerSetVisualizationFlags")]
public static extern  void b3ConfigureOpenGLVisualizerSetVisualizationFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int flag, int enabled) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraDistance: float
    ///cameraPitch: float
    ///cameraYaw: float
    ///cameraTargetPosition: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ConfigureOpenGLVisualizerSetViewMatrix")]
public static extern  void b3ConfigureOpenGLVisualizerSetViewMatrix(ref b3SharedMemoryCommandHandle__ commandHandle, float cameraDistance, float cameraPitch, float cameraYaw, ref float cameraTargetPosition) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRequestOpenGLVisualizerCameraCommand")]
public static extern  System.IntPtr b3InitRequestOpenGLVisualizerCameraCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///camera: b3OpenGLVisualizerCameraInfo*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusOpenGLVisualizerCamera")]
public static extern  int b3GetStatusOpenGLVisualizerCamera(ref b3SharedMemoryStatusHandle__ statusHandle, ref b3OpenGLVisualizerCameraInfo camera) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fromXYZ: double*
    ///toXYZ: double*
    ///colorRGB: double*
    ///lineWidth: double
    ///lifeTime: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugDrawAddLine3D")]
public static extern  System.IntPtr b3InitUserDebugDrawAddLine3D(ref b3PhysicsClientHandle__ physClient, ref double fromXYZ, ref double toXYZ, ref double colorRGB, double lineWidth, double lifeTime) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///txt: char*
    ///positionXYZ: double*
    ///colorRGB: double*
    ///textSize: double
    ///lifeTime: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugDrawAddText3D")]
public static extern  System.IntPtr b3InitUserDebugDrawAddText3D(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string txt, ref double positionXYZ, ref double colorRGB, double textSize, double lifeTime) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///optionFlags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3UserDebugTextSetOptionFlags")]
public static extern  void b3UserDebugTextSetOptionFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int optionFlags) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///orientation: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3UserDebugTextSetOrientation")]
public static extern  void b3UserDebugTextSetOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, ref double orientation) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3UserDebugItemSetParentObject")]
public static extern  void b3UserDebugItemSetParentObject(ref b3SharedMemoryCommandHandle__ commandHandle, int objectUniqueId, int linkIndex) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///txt: char*
    ///rangeMin: double
    ///rangeMax: double
    ///startValue: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugAddParameter")]
public static extern  System.IntPtr b3InitUserDebugAddParameter(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string txt, double rangeMin, double rangeMax, double startValue) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugItemUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugReadParameter")]
public static extern  System.IntPtr b3InitUserDebugReadParameter(ref b3PhysicsClientHandle__ physClient, int debugItemUniqueId) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///paramValue: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusDebugParameterValue")]
public static extern  int b3GetStatusDebugParameterValue(ref b3SharedMemoryStatusHandle__ statusHandle, ref double paramValue) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///debugItemUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugDrawRemove")]
public static extern  System.IntPtr b3InitUserDebugDrawRemove(ref b3PhysicsClientHandle__ physClient, int debugItemUniqueId) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUserDebugDrawRemoveAll")]
public static extern  System.IntPtr b3InitUserDebugDrawRemoveAll(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitDebugDrawingCommand")]
public static extern  System.IntPtr b3InitDebugDrawingCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    ///objectColorRGB: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetDebugObjectColor")]
public static extern  void b3SetDebugObjectColor(ref b3SharedMemoryCommandHandle__ commandHandle, int objectUniqueId, int linkIndex, ref double objectColorRGB) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    ///linkIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RemoveDebugObjectColor")]
public static extern  void b3RemoveDebugObjectColor(ref b3SharedMemoryCommandHandle__ commandHandle, int objectUniqueId, int linkIndex) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetDebugItemUniqueId")]
public static extern  int b3GetDebugItemUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRequestCameraImage")]
public static extern  System.IntPtr b3InitRequestCameraImage(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///command: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///viewMatrix: float*
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetCameraMatrices")]
public static extern  void b3RequestCameraImageSetCameraMatrices(ref b3SharedMemoryCommandHandle__ command, ref float viewMatrix, ref float projectionMatrix) ;

    
    /// Return Type: void
    ///command: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///width: int
    ///height: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetPixelResolution")]
public static extern  void b3RequestCameraImageSetPixelResolution(ref b3SharedMemoryCommandHandle__ command, int width, int height) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDirection: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightDirection")]
public static extern  void b3RequestCameraImageSetLightDirection(ref b3SharedMemoryCommandHandle__ commandHandle, ref float lightDirection) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightColor: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightColor")]
public static extern  void b3RequestCameraImageSetLightColor(ref b3SharedMemoryCommandHandle__ commandHandle, ref float lightColor) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDistance: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightDistance")]
public static extern  void b3RequestCameraImageSetLightDistance(ref b3SharedMemoryCommandHandle__ commandHandle, float lightDistance) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightAmbientCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightAmbientCoeff")]
public static extern  void b3RequestCameraImageSetLightAmbientCoeff(ref b3SharedMemoryCommandHandle__ commandHandle, float lightAmbientCoeff) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightDiffuseCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightDiffuseCoeff")]
public static extern  void b3RequestCameraImageSetLightDiffuseCoeff(ref b3SharedMemoryCommandHandle__ commandHandle, float lightDiffuseCoeff) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///lightSpecularCoeff: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetLightSpecularCoeff")]
public static extern  void b3RequestCameraImageSetLightSpecularCoeff(ref b3SharedMemoryCommandHandle__ commandHandle, float lightSpecularCoeff) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///hasShadow: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetShadow")]
public static extern  void b3RequestCameraImageSetShadow(ref b3SharedMemoryCommandHandle__ commandHandle, int hasShadow) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///renderer: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSelectRenderer")]
public static extern  void b3RequestCameraImageSelectRenderer(ref b3SharedMemoryCommandHandle__ commandHandle, int renderer) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///imageData: b3CameraImageData*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetCameraImageData")]
public static extern  void b3GetCameraImageData(ref b3PhysicsClientHandle__ physClient, ref b3CameraImageData imageData) ;

    
    /// Return Type: void
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    ///viewMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ComputeViewMatrixFromPositions")]
public static extern  void b3ComputeViewMatrixFromPositions(ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp, ref float viewMatrix) ;

    
    /// Return Type: void
    ///cameraTargetPosition: float*
    ///distance: float
    ///yaw: float
    ///pitch: float
    ///roll: float
    ///upAxis: int
    ///viewMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ComputeViewMatrixFromYawPitchRoll")]
public static extern  void b3ComputeViewMatrixFromYawPitchRoll(ref float cameraTargetPosition, float distance, float yaw, float pitch, float roll, int upAxis, ref float viewMatrix) ;

    
    /// Return Type: void
    ///viewMatrix: float*
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ComputePositionFromViewMatrix")]
public static extern  void b3ComputePositionFromViewMatrix(ref float viewMatrix, ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp) ;

    
    /// Return Type: void
    ///left: float
    ///right: float
    ///bottom: float
    ///top: float
    ///nearVal: float
    ///farVal: float
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ComputeProjectionMatrix")]
public static extern  void b3ComputeProjectionMatrix(float left, float right, float bottom, float top, float nearVal, float farVal, ref float projectionMatrix) ;

    
    /// Return Type: void
    ///fov: float
    ///aspect: float
    ///nearVal: float
    ///farVal: float
    ///projectionMatrix: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ComputeProjectionMatrixFOV")]
public static extern  void b3ComputeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, ref float projectionMatrix) ;

    
    /// Return Type: void
    ///command: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraPosition: float*
    ///cameraTargetPosition: float*
    ///cameraUp: float*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetViewMatrix")]
public static extern  void b3RequestCameraImageSetViewMatrix(ref b3SharedMemoryCommandHandle__ command, ref float cameraPosition, ref float cameraTargetPosition, ref float cameraUp) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///cameraTargetPosition: float*
    ///distance: float
    ///yaw: float
    ///pitch: float
    ///roll: float
    ///upAxis: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetViewMatrix2")]
public static extern  void b3RequestCameraImageSetViewMatrix2(ref b3SharedMemoryCommandHandle__ commandHandle, ref float cameraTargetPosition, float distance, float yaw, float pitch, float roll, int upAxis) ;

    
    /// Return Type: void
    ///command: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///left: float
    ///right: float
    ///bottom: float
    ///top: float
    ///nearVal: float
    ///farVal: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetProjectionMatrix")]
public static extern  void b3RequestCameraImageSetProjectionMatrix(ref b3SharedMemoryCommandHandle__ command, float left, float right, float bottom, float top, float nearVal, float farVal) ;

    
    /// Return Type: void
    ///command: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fov: float
    ///aspect: float
    ///nearVal: float
    ///farVal: float
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestCameraImageSetFOVProjectionMatrix")]
public static extern  void b3RequestCameraImageSetFOVProjectionMatrix(ref b3SharedMemoryCommandHandle__ command, float fov, float aspect, float nearVal, float farVal) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRequestContactPointInformation")]
public static extern  System.IntPtr b3InitRequestContactPointInformation(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetContactFilterBodyA")]
public static extern  void b3SetContactFilterBodyA(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueIdA) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdB: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetContactFilterBodyB")]
public static extern  void b3SetContactFilterBodyB(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueIdB) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetContactFilterLinkA")]
public static extern  void b3SetContactFilterLinkA(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexA) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetContactFilterLinkB")]
public static extern  void b3SetContactFilterLinkB(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexB) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///contactPointInfo: b3ContactInformation*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetContactPointInformation")]
public static extern  void b3GetContactPointInformation(ref b3PhysicsClientHandle__ physClient, ref b3ContactInformation contactPointInfo) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitClosestDistanceQuery")]
public static extern  System.IntPtr b3InitClosestDistanceQuery(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetClosestDistanceFilterBodyA")]
public static extern  void b3SetClosestDistanceFilterBodyA(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueIdA) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetClosestDistanceFilterLinkA")]
public static extern  void b3SetClosestDistanceFilterLinkA(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexA) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueIdB: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetClosestDistanceFilterBodyB")]
public static extern  void b3SetClosestDistanceFilterBodyB(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueIdB) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetClosestDistanceFilterLinkB")]
public static extern  void b3SetClosestDistanceFilterLinkB(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexB) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///distance: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetClosestDistanceThreshold")]
public static extern  void b3SetClosestDistanceThreshold(ref b3SharedMemoryCommandHandle__ commandHandle, double distance) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///contactPointInfo: b3ContactInformation*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetClosestPointInformation")]
public static extern  void b3GetClosestPointInformation(ref b3PhysicsClientHandle__ physClient, ref b3ContactInformation contactPointInfo) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///aabbMin: double*
    ///aabbMax: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitAABBOverlapQuery")]
public static extern  System.IntPtr b3InitAABBOverlapQuery(ref b3PhysicsClientHandle__ physClient, ref double aabbMin, ref double aabbMax) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///data: b3AABBOverlapData*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetAABBOverlapResults")]
public static extern  void b3GetAABBOverlapResults(ref b3PhysicsClientHandle__ physClient, ref b3AABBOverlapData data) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueIdA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitRequestVisualShapeInformation")]
public static extern  System.IntPtr b3InitRequestVisualShapeInformation(ref b3PhysicsClientHandle__ physClient, int bodyUniqueIdA) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///visualShapeInfo: b3VisualShapeInformation*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetVisualShapeInformation")]
public static extern  void b3GetVisualShapeInformation(ref b3PhysicsClientHandle__ physClient, ref b3VisualShapeInformation visualShapeInfo) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///filename: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitLoadTexture")]
public static extern  System.IntPtr b3InitLoadTexture(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string filename) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusTextureUniqueId")]
public static extern  int b3GetStatusTextureUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///textureUniqueId: int
    ///width: int
    ///height: int
    ///rgbPixels: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateChangeTextureCommandInit")]
public static extern  System.IntPtr b3CreateChangeTextureCommandInit(ref b3PhysicsClientHandle__ physClient, int textureUniqueId, int width, int height, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string rgbPixels) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///jointIndex: int
    ///shapeIndex: int
    ///textureUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitUpdateVisualShape")]
public static extern  System.IntPtr b3InitUpdateVisualShape(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId, int jointIndex, int shapeIndex, int textureUniqueId) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rgbaColor: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3UpdateVisualShapeRGBAColor")]
public static extern  void b3UpdateVisualShapeRGBAColor(ref b3SharedMemoryCommandHandle__ commandHandle, ref double rgbaColor) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///specularColor: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3UpdateVisualShapeSpecularColor")]
public static extern  void b3UpdateVisualShapeSpecularColor(ref b3SharedMemoryCommandHandle__ commandHandle, ref double specularColor) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitPhysicsParamCommand")]
public static extern  System.IntPtr b3InitPhysicsParamCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///gravx: double
    ///gravy: double
    ///gravz: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetGravity")]
public static extern  int b3PhysicsParamSetGravity(ref b3SharedMemoryCommandHandle__ commandHandle, double gravx, double gravy, double gravz) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///timeStep: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetTimeStep")]
public static extern  int b3PhysicsParamSetTimeStep(ref b3SharedMemoryCommandHandle__ commandHandle, double timeStep) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///defaultContactERP: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetDefaultContactERP")]
public static extern  int b3PhysicsParamSetDefaultContactERP(ref b3SharedMemoryCommandHandle__ commandHandle, double defaultContactERP) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///defaultNonContactERP: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetDefaultNonContactERP")]
public static extern  int b3PhysicsParamSetDefaultNonContactERP(ref b3SharedMemoryCommandHandle__ commandHandle, double defaultNonContactERP) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///frictionERP: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetDefaultFrictionERP")]
public static extern  int b3PhysicsParamSetDefaultFrictionERP(ref b3SharedMemoryCommandHandle__ commandHandle, double frictionERP) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numSubSteps: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetNumSubSteps")]
public static extern  int b3PhysicsParamSetNumSubSteps(ref b3SharedMemoryCommandHandle__ commandHandle, int numSubSteps) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableRealTimeSimulation: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetRealTimeSimulation")]
public static extern  int b3PhysicsParamSetRealTimeSimulation(ref b3SharedMemoryCommandHandle__ commandHandle, int enableRealTimeSimulation) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numSolverIterations: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetNumSolverIterations")]
public static extern  int b3PhysicsParamSetNumSolverIterations(ref b3SharedMemoryCommandHandle__ commandHandle, int numSolverIterations) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///filterMode: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetCollisionFilterMode")]
public static extern  int b3PhysicsParamSetCollisionFilterMode(ref b3SharedMemoryCommandHandle__ commandHandle, int filterMode) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useSplitImpulse: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetUseSplitImpulse")]
public static extern  int b3PhysicsParamSetUseSplitImpulse(ref b3SharedMemoryCommandHandle__ commandHandle, int useSplitImpulse) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///splitImpulsePenetrationThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetSplitImpulsePenetrationThreshold")]
public static extern  int b3PhysicsParamSetSplitImpulsePenetrationThreshold(ref b3SharedMemoryCommandHandle__ commandHandle, double splitImpulsePenetrationThreshold) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///contactBreakingThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetContactBreakingThreshold")]
public static extern  int b3PhysicsParamSetContactBreakingThreshold(ref b3SharedMemoryCommandHandle__ commandHandle, double contactBreakingThreshold) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxNumCmdPer1ms: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetMaxNumCommandsPer1ms")]
public static extern  int b3PhysicsParamSetMaxNumCommandsPer1ms(ref b3SharedMemoryCommandHandle__ commandHandle, int maxNumCmdPer1ms) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///enableFileCaching: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetEnableFileCaching")]
public static extern  int b3PhysicsParamSetEnableFileCaching(ref b3SharedMemoryCommandHandle__ commandHandle, int enableFileCaching) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///restitutionVelocityThreshold: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetRestitutionVelocityThreshold")]
public static extern  int b3PhysicsParamSetRestitutionVelocityThreshold(ref b3SharedMemoryCommandHandle__ commandHandle, double restitutionVelocityThreshold) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PhysicsParamSetInternalSimFlags")]
public static extern  int b3PhysicsParamSetInternalSimFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int flags) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitStepSimulationCommand")]
public static extern  System.IntPtr b3InitStepSimulationCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InitResetSimulationCommand")]
public static extern  System.IntPtr b3InitResetSimulationCommand(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///urdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandInit")]
public static extern  System.IntPtr b3LoadUrdfCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string urdfFileName) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetStartPosition")]
public static extern  int b3LoadUrdfCommandSetStartPosition(ref b3SharedMemoryCommandHandle__ commandHandle, double startPosX, double startPosY, double startPosZ) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetStartOrientation")]
public static extern  int b3LoadUrdfCommandSetStartOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useMultiBody: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetUseMultiBody")]
public static extern  int b3LoadUrdfCommandSetUseMultiBody(ref b3SharedMemoryCommandHandle__ commandHandle, int useMultiBody) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useFixedBase: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetUseFixedBase")]
public static extern  int b3LoadUrdfCommandSetUseFixedBase(ref b3SharedMemoryCommandHandle__ commandHandle, int useFixedBase) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetFlags")]
public static extern  int b3LoadUrdfCommandSetFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int flags) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///globalScaling: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadUrdfCommandSetGlobalScaling")]
public static extern  int b3LoadUrdfCommandSetGlobalScaling(ref b3SharedMemoryCommandHandle__ commandHandle, double globalScaling) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadBulletCommandInit")]
public static extern  System.IntPtr b3LoadBulletCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SaveBulletCommandInit")]
public static extern  System.IntPtr b3SaveBulletCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadMJCFCommandInit")]
public static extern  System.IntPtr b3LoadMJCFCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadMJCFCommandSetFlags")]
public static extern  void b3LoadMJCFCommandSetFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int flags) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    ///jointPositionsQ: double*
    ///jointVelocitiesQdot: double*
    ///jointAccelerations: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseDynamicsCommandInit")]
public static extern  System.IntPtr b3CalculateInverseDynamicsCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyIndex, ref double jointPositionsQ, ref double jointVelocitiesQdot, ref double jointAccelerations) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///dofCount: int*
    ///jointForces: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusInverseDynamicsJointForces")]
public static extern  int b3GetStatusInverseDynamicsJointForces(ref b3SharedMemoryStatusHandle__ statusHandle, ref int bodyUniqueId, ref int dofCount, ref double jointForces) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    ///linkIndex: int
    ///localPosition: double*
    ///jointPositionsQ: double*
    ///jointVelocitiesQdot: double*
    ///jointAccelerations: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateJacobianCommandInit")]
public static extern  System.IntPtr b3CalculateJacobianCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyIndex, int linkIndex, ref double localPosition, ref double jointPositionsQ, ref double jointVelocitiesQdot, ref double jointAccelerations) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///linearJacobian: double*
    ///angularJacobian: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusJacobian")]
public static extern  int b3GetStatusJacobian(ref b3SharedMemoryStatusHandle__ statusHandle, ref double linearJacobian, ref double angularJacobian) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsCommandInit")]
public static extern  System.IntPtr b3CalculateInverseKinematicsCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyIndex) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsAddTargetPurePosition")]
public static extern  void b3CalculateInverseKinematicsAddTargetPurePosition(ref b3SharedMemoryCommandHandle__ commandHandle, int endEffectorLinkIndex, ref double targetPosition) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///targetOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsAddTargetPositionWithOrientation")]
public static extern  void b3CalculateInverseKinematicsAddTargetPositionWithOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, int endEffectorLinkIndex, ref double targetPosition, ref double targetOrientation) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///lowerLimit: double*
    ///upperLimit: double*
    ///jointRange: double*
    ///restPose: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsPosWithNullSpaceVel")]
public static extern  void b3CalculateInverseKinematicsPosWithNullSpaceVel(ref b3SharedMemoryCommandHandle__ commandHandle, int numDof, int endEffectorLinkIndex, ref double targetPosition, ref double lowerLimit, ref double upperLimit, ref double jointRange, ref double restPose) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///endEffectorLinkIndex: int
    ///targetPosition: double*
    ///targetOrientation: double*
    ///lowerLimit: double*
    ///upperLimit: double*
    ///jointRange: double*
    ///restPose: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsPosOrnWithNullSpaceVel")]
public static extern  void b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(ref b3SharedMemoryCommandHandle__ commandHandle, int numDof, int endEffectorLinkIndex, ref double targetPosition, ref double targetOrientation, ref double lowerLimit, ref double upperLimit, ref double jointRange, ref double restPose) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numDof: int
    ///jointDampingCoeff: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CalculateInverseKinematicsSetJointDamping")]
public static extern  void b3CalculateInverseKinematicsSetJointDamping(ref b3SharedMemoryCommandHandle__ commandHandle, int numDof, ref double jointDampingCoeff) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///bodyUniqueId: int*
    ///dofCount: int*
    ///jointPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusInverseKinematicsJointPositions")]
public static extern  int b3GetStatusInverseKinematicsJointPositions(ref b3SharedMemoryStatusHandle__ statusHandle, ref int bodyUniqueId, ref int dofCount, ref double jointPositions) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///sdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadSdfCommandInit")]
public static extern  System.IntPtr b3LoadSdfCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string sdfFileName) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///useMultiBody: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadSdfCommandSetUseMultiBody")]
public static extern  int b3LoadSdfCommandSetUseMultiBody(ref b3SharedMemoryCommandHandle__ commandHandle, int useMultiBody) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///globalScaling: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadSdfCommandSetUseGlobalScaling")]
public static extern  int b3LoadSdfCommandSetUseGlobalScaling(ref b3SharedMemoryCommandHandle__ commandHandle, double globalScaling) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///sdfFileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SaveWorldCommandInit")]
public static extern  System.IntPtr b3SaveWorldCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string sdfFileName) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///controlMode: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlCommandInit")]
public static extern  System.IntPtr b3JointControlCommandInit(ref b3PhysicsClientHandle__ physClient, int controlMode) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    ///controlMode: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlCommandInit2")]
public static extern  System.IntPtr b3JointControlCommandInit2(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId, int controlMode) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///qIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetDesiredPosition")]
public static extern  int b3JointControlSetDesiredPosition(ref b3SharedMemoryCommandHandle__ commandHandle, int qIndex, double value) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetKp")]
public static extern  int b3JointControlSetKp(ref b3SharedMemoryCommandHandle__ commandHandle, int dofIndex, double value) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetKd")]
public static extern  int b3JointControlSetKd(ref b3SharedMemoryCommandHandle__ commandHandle, int dofIndex, double value) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetDesiredVelocity")]
public static extern  int b3JointControlSetDesiredVelocity(ref b3SharedMemoryCommandHandle__ commandHandle, int dofIndex, double value) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetMaximumForce")]
public static extern  int b3JointControlSetMaximumForce(ref b3SharedMemoryCommandHandle__ commandHandle, int dofIndex, double value) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///dofIndex: int
    ///value: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3JointControlSetDesiredForceTorque")]
public static extern  int b3JointControlSetDesiredForceTorque(ref b3SharedMemoryCommandHandle__ commandHandle, int dofIndex, double value) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeCommandInit")]
public static extern  System.IntPtr b3CreateCollisionShapeCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddSphere")]
public static extern  int b3CreateCollisionShapeAddSphere(ref b3SharedMemoryCommandHandle__ commandHandle, double radius) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///halfExtents: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddBox")]
public static extern  int b3CreateCollisionShapeAddBox(ref b3SharedMemoryCommandHandle__ commandHandle, ref double halfExtents) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddCapsule")]
public static extern  int b3CreateCollisionShapeAddCapsule(ref b3SharedMemoryCommandHandle__ commandHandle, double radius, double height) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///radius: double
    ///height: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddCylinder")]
public static extern  int b3CreateCollisionShapeAddCylinder(ref b3SharedMemoryCommandHandle__ commandHandle, double radius, double height) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///planeNormal: double*
    ///planeConstant: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddPlane")]
public static extern  int b3CreateCollisionShapeAddPlane(ref b3SharedMemoryCommandHandle__ commandHandle, ref double planeNormal, double planeConstant) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///fileName: char*
    ///meshScale: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeAddMesh")]
public static extern  int b3CreateCollisionShapeAddMesh(ref b3SharedMemoryCommandHandle__ commandHandle, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName, ref double meshScale) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionSetFlag")]
public static extern  void b3CreateCollisionSetFlag(ref b3SharedMemoryCommandHandle__ commandHandle, int shapeIndex, int flags) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///shapeIndex: int
    ///childPosition: double*
    ///childOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateCollisionShapeSetChildTransform")]
public static extern  void b3CreateCollisionShapeSetChildTransform(ref b3SharedMemoryCommandHandle__ commandHandle, int shapeIndex, ref double childPosition, ref double childOrientation) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusCollisionShapeUniqueId")]
public static extern  int b3GetStatusCollisionShapeUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateVisualShapeCommandInit")]
public static extern  System.IntPtr b3CreateVisualShapeCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusVisualShapeUniqueId")]
public static extern  int b3GetStatusVisualShapeUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateMultiBodyCommandInit")]
public static extern  System.IntPtr b3CreateMultiBodyCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    ///collisionShapeUnique: int
    ///visualShapeUniqueId: int
    ///basePosition: double*
    ///baseOrientation: double*
    ///baseInertialFramePosition: double*
    ///baseInertialFrameOrientation: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateMultiBodyBase")]
public static extern  int b3CreateMultiBodyBase(ref b3SharedMemoryCommandHandle__ commandHandle, double mass, int collisionShapeUnique, int visualShapeUniqueId, ref double basePosition, ref double baseOrientation, ref double baseInertialFramePosition, ref double baseInertialFrameOrientation) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkMass: double
    ///linkCollisionShapeIndex: double
    ///linkVisualShapeIndex: double
    ///linkPosition: double*
    ///linkOrientation: double*
    ///linkInertialFramePosition: double*
    ///linkInertialFrameOrientation: double*
    ///linkParentIndex: int
    ///linkJointType: int
    ///linkJointAxis: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateMultiBodyLink")]
public static extern  int b3CreateMultiBodyLink(ref b3SharedMemoryCommandHandle__ commandHandle, double linkMass, double linkCollisionShapeIndex, double linkVisualShapeIndex, ref double linkPosition, ref double linkOrientation, ref double linkInertialFramePosition, ref double linkInertialFrameOrientation, int linkParentIndex, int linkJointType, ref double linkJointAxis) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateMultiBodyUseMaximalCoordinates")]
public static extern  void b3CreateMultiBodyUseMaximalCoordinates(ref b3SharedMemoryCommandHandle__ commandHandle) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxShapeCommandInit")]
public static extern  System.IntPtr b3CreateBoxShapeCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetStartPosition")]
public static extern  int b3CreateBoxCommandSetStartPosition(ref b3SharedMemoryCommandHandle__ commandHandle, double startPosX, double startPosY, double startPosZ) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetStartOrientation")]
public static extern  int b3CreateBoxCommandSetStartOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///halfExtentsX: double
    ///halfExtentsY: double
    ///halfExtentsZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetHalfExtents")]
public static extern  int b3CreateBoxCommandSetHalfExtents(ref b3SharedMemoryCommandHandle__ commandHandle, double halfExtentsX, double halfExtentsY, double halfExtentsZ) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetMass")]
public static extern  int b3CreateBoxCommandSetMass(ref b3SharedMemoryCommandHandle__ commandHandle, double mass) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionShapeType: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetCollisionShapeType")]
public static extern  int b3CreateBoxCommandSetCollisionShapeType(ref b3SharedMemoryCommandHandle__ commandHandle, int collisionShapeType) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///red: double
    ///green: double
    ///blue: double
    ///alpha: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateBoxCommandSetColorRGBA")]
public static extern  int b3CreateBoxCommandSetColorRGBA(ref b3SharedMemoryCommandHandle__ commandHandle, double red, double green, double blue, double alpha) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyIndex: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandInit")]
public static extern  System.IntPtr b3CreatePoseCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyIndex) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startPosX: double
    ///startPosY: double
    ///startPosZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetBasePosition")]
public static extern  int b3CreatePoseCommandSetBasePosition(ref b3SharedMemoryCommandHandle__ commandHandle, double startPosX, double startPosY, double startPosZ) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///startOrnX: double
    ///startOrnY: double
    ///startOrnZ: double
    ///startOrnW: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetBaseOrientation")]
public static extern  int b3CreatePoseCommandSetBaseOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, double startOrnX, double startOrnY, double startOrnZ, double startOrnW) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linVel: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetBaseLinearVelocity")]
public static extern  int b3CreatePoseCommandSetBaseLinearVelocity(ref b3SharedMemoryCommandHandle__ commandHandle, ref double linVel) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///angVel: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetBaseAngularVelocity")]
public static extern  int b3CreatePoseCommandSetBaseAngularVelocity(ref b3SharedMemoryCommandHandle__ commandHandle, ref double angVel) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointPositions: int
    ///jointPositions: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetJointPositions")]
public static extern  int b3CreatePoseCommandSetJointPositions(ref b3SharedMemoryCommandHandle__ commandHandle, int numJointPositions, ref double jointPositions) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointPosition: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetJointPosition")]
public static extern  int b3CreatePoseCommandSetJointPosition(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryCommandHandle__ commandHandle, int jointIndex, double jointPosition) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///numJointVelocities: int
    ///jointVelocities: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetJointVelocities")]
public static extern  int b3CreatePoseCommandSetJointVelocities(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryCommandHandle__ commandHandle, int numJointVelocities, ref double jointVelocities) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///jointVelocity: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreatePoseCommandSetJointVelocity")]
public static extern  int b3CreatePoseCommandSetJointVelocity(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryCommandHandle__ commandHandle, int jointIndex, double jointVelocity) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateSensorCommandInit")]
public static extern  System.IntPtr b3CreateSensorCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///jointIndex: int
    ///enable: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateSensorEnable6DofJointForceTorqueSensor")]
public static extern  int b3CreateSensorEnable6DofJointForceTorqueSensor(ref b3SharedMemoryCommandHandle__ commandHandle, int jointIndex, int enable) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndex: int
    ///enable: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateSensorEnableIMUForLink")]
public static extern  int b3CreateSensorEnableIMUForLink(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndex, int enable) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///bodyUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestActualStateCommandInit")]
public static extern  System.IntPtr b3RequestActualStateCommandInit(ref b3PhysicsClientHandle__ physClient, int bodyUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///computeLinkVelocity: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestActualStateCommandComputeLinkVelocity")]
public static extern  int b3RequestActualStateCommandComputeLinkVelocity(ref b3SharedMemoryCommandHandle__ commandHandle, int computeLinkVelocity) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///jointIndex: int
    ///state: b3JointSensorState*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetJointState")]
public static extern  int b3GetJointState(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryStatusHandle__ statusHandle, int jointIndex, ref b3JointSensorState state) ;

    
    /// Return Type: int
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    ///linkIndex: int
    ///state: b3LinkState*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetLinkState")]
public static extern  int b3GetLinkState(ref b3PhysicsClientHandle__ physClient, ref b3SharedMemoryStatusHandle__ statusHandle, int linkIndex, ref b3LinkState state) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3PickBody")]
public static extern  System.IntPtr b3PickBody(ref b3PhysicsClientHandle__ physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3MovePickedBody")]
public static extern  System.IntPtr b3MovePickedBody(ref b3PhysicsClientHandle__ physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RemovePickingConstraint")]
public static extern  System.IntPtr b3RemovePickingConstraint(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///rayFromWorldX: double
    ///rayFromWorldY: double
    ///rayFromWorldZ: double
    ///rayToWorldX: double
    ///rayToWorldY: double
    ///rayToWorldZ: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateRaycastCommandInit")]
public static extern  System.IntPtr b3CreateRaycastCommandInit(ref b3PhysicsClientHandle__ physClient, double rayFromWorldX, double rayFromWorldY, double rayFromWorldZ, double rayToWorldX, double rayToWorldY, double rayToWorldZ) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3CreateRaycastBatchCommandInit")]
public static extern  System.IntPtr b3CreateRaycastBatchCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rayFromWorld: double*
    ///rayToWorld: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RaycastBatchAddRay")]
public static extern  void b3RaycastBatchAddRay(ref b3SharedMemoryCommandHandle__ commandHandle, ref double rayFromWorld, ref double rayToWorld) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///raycastInfo: b3RaycastInformation*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetRaycastInformation")]
public static extern  void b3GetRaycastInformation(ref b3PhysicsClientHandle__ physClient, ref b3RaycastInformation raycastInfo) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ApplyExternalForceCommandInit")]
public static extern  System.IntPtr b3ApplyExternalForceCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkId: int
    ///force: double*
    ///position: double*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ApplyExternalForce")]
public static extern  void b3ApplyExternalForce(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkId, ref double force, ref double position, int flags) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyUniqueId: int
    ///linkId: int
    ///torque: double*
    ///flags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ApplyExternalTorque")]
public static extern  void b3ApplyExternalTorque(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyUniqueId, int linkId, ref double torque, int flags) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadBunnyCommandInit")]
public static extern  System.IntPtr b3LoadBunnyCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///scale: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadBunnySetScale")]
public static extern  int b3LoadBunnySetScale(ref b3SharedMemoryCommandHandle__ commandHandle, double scale) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///mass: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadBunnySetMass")]
public static extern  int b3LoadBunnySetMass(ref b3SharedMemoryCommandHandle__ commandHandle, double mass) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///collisionMargin: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3LoadBunnySetCollisionMargin")]
public static extern  int b3LoadBunnySetCollisionMargin(ref b3SharedMemoryCommandHandle__ commandHandle, double collisionMargin) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestVREventsCommandInit")]
public static extern  System.IntPtr b3RequestVREventsCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///deviceTypeFilter: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3VREventsSetDeviceTypeFilter")]
public static extern  void b3VREventsSetDeviceTypeFilter(ref b3SharedMemoryCommandHandle__ commandHandle, int deviceTypeFilter) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///vrEventsData: b3VREventsData*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetVREventsData")]
public static extern  void b3GetVREventsData(ref b3PhysicsClientHandle__ physClient, ref b3VREventsData vrEventsData) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetVRCameraStateCommandInit")]
public static extern  System.IntPtr b3SetVRCameraStateCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rootPos: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetVRCameraRootPosition")]
public static extern  int b3SetVRCameraRootPosition(ref b3SharedMemoryCommandHandle__ commandHandle, ref double rootPos) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///rootOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetVRCameraRootOrientation")]
public static extern  int b3SetVRCameraRootOrientation(ref b3SharedMemoryCommandHandle__ commandHandle, ref double rootOrn) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetVRCameraTrackingObject")]
public static extern  int b3SetVRCameraTrackingObject(ref b3SharedMemoryCommandHandle__ commandHandle, int objectUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///flag: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetVRCameraTrackingObjectFlag")]
public static extern  int b3SetVRCameraTrackingObjectFlag(ref b3SharedMemoryCommandHandle__ commandHandle, int flag) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestKeyboardEventsCommandInit")]
public static extern  System.IntPtr b3RequestKeyboardEventsCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///keyboardEventsData: b3KeyboardEventsData*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetKeyboardEventsData")]
public static extern  void b3GetKeyboardEventsData(ref b3PhysicsClientHandle__ physClient, ref b3KeyboardEventsData keyboardEventsData) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3RequestMouseEventsCommandInit")]
public static extern  System.IntPtr b3RequestMouseEventsCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///mouseEventsData: b3MouseEventsData*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetMouseEventsData")]
public static extern  void b3GetMouseEventsData(ref b3PhysicsClientHandle__ physClient, ref b3MouseEventsData mouseEventsData) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingCommandInit")]
public static extern  System.IntPtr b3StateLoggingCommandInit(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///loggingType: int
    ///fileName: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingStart")]
public static extern  int b3StateLoggingStart(ref b3SharedMemoryCommandHandle__ commandHandle, int loggingType, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string fileName) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///objectUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingAddLoggingObjectUniqueId")]
public static extern  int b3StateLoggingAddLoggingObjectUniqueId(ref b3SharedMemoryCommandHandle__ commandHandle, int objectUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///maxLogDof: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetMaxLogDof")]
public static extern  int b3StateLoggingSetMaxLogDof(ref b3SharedMemoryCommandHandle__ commandHandle, int maxLogDof) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexA: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetLinkIndexA")]
public static extern  int b3StateLoggingSetLinkIndexA(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexA) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///linkIndexB: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetLinkIndexB")]
public static extern  int b3StateLoggingSetLinkIndexB(ref b3SharedMemoryCommandHandle__ commandHandle, int linkIndexB) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyAUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetBodyAUniqueId")]
public static extern  int b3StateLoggingSetBodyAUniqueId(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyAUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///bodyBUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetBodyBUniqueId")]
public static extern  int b3StateLoggingSetBodyBUniqueId(ref b3SharedMemoryCommandHandle__ commandHandle, int bodyBUniqueId) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///deviceTypeFilter: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetDeviceTypeFilter")]
public static extern  int b3StateLoggingSetDeviceTypeFilter(ref b3SharedMemoryCommandHandle__ commandHandle, int deviceTypeFilter) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///logFlags: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingSetLogFlags")]
public static extern  int b3StateLoggingSetLogFlags(ref b3SharedMemoryCommandHandle__ commandHandle, int logFlags) ;

    
    /// Return Type: int
    ///statusHandle: b3SharedMemoryStatusHandle->b3SharedMemoryStatusHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetStatusLoggingUniqueId")]
public static extern  int b3GetStatusLoggingUniqueId(ref b3SharedMemoryStatusHandle__ statusHandle) ;

    
    /// Return Type: int
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///loggingUniqueId: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3StateLoggingStop")]
public static extern  int b3StateLoggingStop(ref b3SharedMemoryCommandHandle__ commandHandle, int loggingUniqueId) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///name: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3ProfileTimingCommandInit")]
public static extern  System.IntPtr b3ProfileTimingCommandInit(ref b3PhysicsClientHandle__ physClient, [System.Runtime.InteropServices.InAttribute()] [System.Runtime.InteropServices.MarshalAsAttribute(System.Runtime.InteropServices.UnmanagedType.LPStr)] string name) ;

    
    /// Return Type: void
    ///commandHandle: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///duration: int
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetProfileTimingDuractionInMicroSeconds")]
public static extern  void b3SetProfileTimingDuractionInMicroSeconds(ref b3SharedMemoryCommandHandle__ commandHandle, int duration) ;

    
    /// Return Type: void
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///timeOutInSeconds: double
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetTimeOut")]
public static extern  void b3SetTimeOut(ref b3PhysicsClientHandle__ physClient, double timeOutInSeconds) ;

    
    /// Return Type: double
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3GetTimeOut")]
public static extern  double b3GetTimeOut(ref b3PhysicsClientHandle__ physClient) ;

    
    /// Return Type: b3SharedMemoryCommandHandle->b3SharedMemoryCommandHandle__*
    ///physClient: b3PhysicsClientHandle->b3PhysicsClientHandle__*
    ///path: char*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3SetAdditionalSearchPath")]
public static extern  System.IntPtr b3SetAdditionalSearchPath(ref b3PhysicsClientHandle__ physClient, System.IntPtr path) ;

    
    /// Return Type: void
    ///posA: double*
    ///ornA: double*
    ///posB: double*
    ///ornB: double*
    ///outPos: double*
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3MultiplyTransforms")]
public static extern  void b3MultiplyTransforms(ref double posA, ref double ornA, ref double posB, ref double ornB, ref double outPos, ref double outOrn) ;

    
    /// Return Type: void
    ///pos: double*
    ///orn: double*
    ///outPos: double*
    ///outOrn: double*
    [System.Runtime.InteropServices.DllImportAttribute("<Unknown>", EntryPoint="b3InvertTransform")]
public static extern  void b3InvertTransform(ref double pos, ref double orn, ref double outPos, ref double outOrn) ;

}
