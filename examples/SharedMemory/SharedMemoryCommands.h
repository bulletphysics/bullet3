
#ifndef SHARED_MEMORY_COMMANDS_H
#define SHARED_MEMORY_COMMANDS_H

//this is a very experimental draft of commands. We will iterate on this API (commands, arguments etc)

#ifdef __GNUC__
	#include <stdint.h>
	typedef int32_t smInt32_t;
	typedef int64_t smInt64_t;
	typedef uint32_t smUint32_t;
	typedef uint64_t smUint64_t;
#elif defined(_MSC_VER)
	typedef __int32 smInt32_t;
	typedef __int64 smInt64_t;
	typedef unsigned __int32 smUint32_t;
	typedef unsigned __int64 smUint64_t;
#else
	typedef int smInt32_t;
	typedef long long int smInt64_t;
	typedef unsigned int smUint32_t;
	typedef unsigned long long int smUint64_t;
#endif

enum EnumSharedMemoryClientCommand
{
    CMD_LOAD_URDF,
	CMD_SEND_BULLET_DATA_STREAM,
	CMD_CREATE_BOX_COLLISION_SHAPE,
//	CMD_DELETE_BOX_COLLISION_SHAPE,
//	CMD_CREATE_RIGID_BODY,
//	CMD_DELETE_RIGID_BODY,
    CMD_CREATE_SENSOR,///enable or disable joint feedback for force/torque sensors
//    CMD_REQUEST_SENSOR_MEASUREMENTS,//see CMD_REQUEST_ACTUAL_STATE/CMD_ACTUAL_STATE_UPDATE_COMPLETED
	CMD_INIT_POSE,
	CMD_SEND_PHYSICS_SIMULATION_PARAMETERS,
	CMD_SEND_DESIRED_STATE,//todo: reconsider naming, for example SET_JOINT_CONTROL_VARIABLE?
	CMD_REQUEST_ACTUAL_STATE,
    CMD_STEP_FORWARD_SIMULATION,
    CMD_RESET_SIMULATION,
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
	CMD_DESIRED_STATE_RECEIVED_COMPLETED,
	CMD_STEP_FORWARD_SIMULATION_COMPLETED,
	CMD_MAX_SERVER_COMMANDS
};

#define SHARED_MEMORY_SERVER_TEST_C
#define MAX_DEGREE_OF_FREEDOM 256
#define MAX_NUM_SENSORS 256
#define MAX_URDF_FILENAME_LENGTH 1024

enum EnumUrdfArgsUpdateFlags
{
	URDF_ARGS_FILE_NAME=1,
	URDF_ARGS_INITIAL_POSITION=2,
	URDF_ARGS_INITIAL_ORIENTATION=4,
	URDF_ARGS_USE_MULTIBODY=8,
	URDF_ARGS_USE_FIXED_BASE=16,
};


struct UrdfArgs
{
	char m_urdfFileName[MAX_URDF_FILENAME_LENGTH];
	double m_initialPosition[3];
	double m_initialOrientation[4];
	int m_useMultiBody;
	int m_useFixedBase;
};


struct BulletDataStreamArgs
{
	int m_streamChunkLength;
	int m_bodyUniqueId;
};

struct SetJointFeedbackArgs
{
	int m_bodyUniqueId;
	int m_linkId;
	int m_isEnabled;
};

//todo: discuss and decide about control mode and combinations
enum {
//    POSITION_CONTROL=0,
    CONTROL_MODE_VELOCITY=0,
    CONTROL_MODE_TORQUE,
};

///InitPoseArgs is mainly to initialize (teleport) the robot in a particular position
///No motors or controls are needed to initialize the pose. It is similar to
///moving a robot to a starting place, while it is switched off. It is only called
///at the start of a robot control session. All velocities and control forces are cleared to zero.
struct InitPoseArgs
{
	int m_bodyUniqueId;
	double m_initialStateQ[MAX_DEGREE_OF_FREEDOM];
};


///Controlling a robot involves sending the desired state to its joint motor controllers.
///The control mode determines the state variables used for motor control.
struct SendDesiredStateArgs
{
	int m_bodyUniqueId;
	int m_controlMode;

	//desired state is only written by the client, read-only access by server is expected
    double m_desiredStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_desiredStateQdot[MAX_DEGREE_OF_FREEDOM];
	
	//m_desiredStateForceTorque is either the actual applied force/torque (in CONTROL_MODE_TORQUE) or
	//or m_desiredStateForceTorque is the maximum applied force/torque for the motor/constraint to reach the desired velocity in CONTROL_MODE_VELOCITY mode
    double m_desiredStateForceTorque[MAX_DEGREE_OF_FREEDOM];
 
};


enum EnumSimParamUpdateFlags
{
	SIM_PARAM_UPDATE_DELTA_TIME=1,
	SIM_PARAM_UPDATE_GRAVITY=2,
	SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS=4,	
	SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS=8,
};

///Controlling a robot involves sending the desired state to its joint motor controllers.
///The control mode determines the state variables used for motor control.
struct SendPhysicsSimulationParameters
{
	double m_deltaTime;
	double m_gravityAcceleration[3];
	int m_numSimulationSubSteps;
	int m_numSolverIterations;
};

struct RequestActualStateArgs
{
	int m_bodyUniqueId;
};




struct SendActualStateArgs
{
	int m_bodyUniqueId;
	int m_numDegreeOfFreedomQ;
	int m_numDegreeOfFreedomU;

	  //actual state is only written by the server, read-only access by client is expected
    double m_actualStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateQdot[MAX_DEGREE_OF_FREEDOM];

    //measured 6DOF force/torque sensors: force[x,y,z] and torque[x,y,z]
    double m_jointReactionForces[6*MAX_DEGREE_OF_FREEDOM];
  
};

struct CreateSensorArgs
{
    int m_bodyUniqueId;
    int m_numJointSensorChanges;
    int m_jointIndex[MAX_DEGREE_OF_FREEDOM];
    int m_enableJointForceSensor[MAX_DEGREE_OF_FREEDOM];
};

typedef  struct SharedMemoryCommand SharedMemoryCommand_t;

enum EnumBoxShapeFlags
{
    BOX_SHAPE_HAS_INITIAL_POSITION=1,
    BOX_SHAPE_HAS_INITIAL_ORIENTATION=2,
    BOX_SHAPE_HAS_HALF_EXTENTS=4
};
///This command will be replaced to allow arbitrary collision shape types
struct CreateBoxShapeArgs
{
    double m_halfExtentsX;
    double m_halfExtentsY;
    double m_halfExtentsZ;

    double m_initialPosition[3];
	double m_initialOrientation[4];
};

struct SharedMemoryCommand
{
	int m_type;
	smUint64_t	m_timeStamp;
	int	m_sequenceNumber;
	
	//m_updateFlags is a bit fields to tell which parameters need updating
    //for example m_updateFlags = SIM_PARAM_UPDATE_DELTA_TIME | SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS;
    int m_updateFlags;

    union
    {
        struct UrdfArgs m_urdfArguments;
		struct InitPoseArgs m_initPoseArgs;
		struct SendPhysicsSimulationParameters m_physSimParamArgs;
		struct BulletDataStreamArgs	m_dataStreamArguments;
		struct SendDesiredStateArgs m_sendDesiredStateCommandArgument;
		struct RequestActualStateArgs m_requestActualStateInformationCommandArgument;
        struct CreateSensorArgs m_createSensorArguments;
        struct CreateBoxShapeArgs m_createBoxShapeArguments;
    };
};


struct SharedMemoryStatus
{
	int m_type;
	
	smUint64_t	m_timeStamp;
	int	m_sequenceNumber;
	
	union
	{
		struct BulletDataStreamArgs	m_dataStreamArguments;
		struct SendActualStateArgs m_sendActualStateArgs;
	};
};

typedef  struct SharedMemoryStatus SharedMemoryStatus_t;

enum JointInfoFlags
{
    JOINT_HAS_MOTORIZED_POWER=1,
};
struct b3JointInfo
{
        char* m_linkName;
        char* m_jointName;
        int m_jointType;
        int m_qIndex;
        int m_uIndex;
    ///
        int m_flags;
};




#endif //SHARED_MEMORY_COMMANDS_H
