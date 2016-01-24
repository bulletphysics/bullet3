#ifndef SHARED_MEMORY_PUBLIC_H
#define SHARED_MEMORY_PUBLIC_H

#define SHARED_MEMORY_KEY 12347

enum EnumSharedMemoryClientCommand
{
    CMD_LOAD_URDF,
        CMD_SEND_BULLET_DATA_STREAM,
        CMD_CREATE_BOX_COLLISION_SHAPE,
//      CMD_DELETE_BOX_COLLISION_SHAPE,
      CMD_CREATE_RIGID_BODY,
      CMD_DELETE_RIGID_BODY,
    CMD_CREATE_SENSOR,///enable or disable joint feedback for force/torque sensors
//    CMD_REQUEST_SENSOR_MEASUREMENTS,//see CMD_REQUEST_ACTUAL_STATE/CMD_ACTUAL_STATE_UPDATE_COMPLETED
        CMD_INIT_POSE,
        CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
        CMD_SEND_DESIRED_STATE,//todo: reconsider naming, for example SET_JOINT_CONTROL_VARIABLE?
        CMD_REQUEST_ACTUAL_STATE,
        CMD_REQUEST_DEBUG_LINES,
    CMD_STEP_FORWARD_SIMULATION,
    CMD_RESET_SIMULATION,
    CMD_PICK_BODY,
    CMD_MOVE_PICKED_BODY,
    CMD_REMOVE_PICKING_CONSTRAINT_BODY,
    CMD_MAX_CLIENT_COMMANDS
};

enum EnumSharedMemoryServerStatus
{
        CMD_SHARED_MEMORY_NOT_INITIALIZED=0,
        CMD_WAITING_FOR_CLIENT_COMMAND,

        //CMD_CLIENT_COMMAND_COMPLETED is a generic 'completed' status that doesn't need special handling on the client
        CMD_CLIENT_COMMAND_COMPLETED,
        //the server will skip unknown command and report a status 'CMD_UNKNOWN_COMMAND_FLUSHED'
        CMD_UNKNOWN_COMMAND_FLUSHED,

        CMD_URDF_LOADING_COMPLETED,
        CMD_URDF_LOADING_FAILED,
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
};

struct b3JointSensorState
{
  double m_jointPosition;
  double m_jointVelocity;
  double m_jointForceTorque[6];  /* note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint */
};

struct b3DebugLines
{
    int m_numDebugLines;
    const float*  m_linesFrom;//float x,y,z times 'm_numDebugLines'.
    const float*  m_linesTo;//float x,y,z times 'm_numDebugLines'.
    const float*  m_linesColor;//float red,green,blue times 'm_numDebugLines'.
};

//todo: discuss and decide about control mode and combinations
enum {
    //    POSITION_CONTROL=0,
    CONTROL_MODE_VELOCITY=0,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_POSITION_VELOCITY_PD,
};

#endif//SHARED_MEMORY_PUBLIC_H
