
#ifndef SHARED_MEMORY_COMMANDS_H
#define SHARED_MEMORY_COMMANDS_H

//this is a very experimental draft of commands. We will iterate on this API (commands, arguments etc)

#include "SharedMemoryPublic.h"

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

#define SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE (256*1024)

#define SHARED_MEMORY_SERVER_TEST_C
#define MAX_DEGREE_OF_FREEDOM 64
#define MAX_NUM_SENSORS 256
#define MAX_URDF_FILENAME_LENGTH 1024
#define MAX_FILENAME_LENGTH MAX_URDF_FILENAME_LENGTH

struct TmpFloat3 
{
    float m_x;
    float m_y;
    float m_z;
};

#ifdef _WIN32
__inline
#else
inline
#endif
TmpFloat3 CreateTmpFloat3(float x, float y, float z) 
{
    TmpFloat3 tmp;
    tmp.m_x = x;
    tmp.m_y = y;
    tmp.m_z = z;
    return tmp;
}

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
	char m_bulletFileName[MAX_FILENAME_LENGTH];
	int m_streamChunkLength;
	int m_bodyUniqueId;
};

struct SetJointFeedbackArgs
{
	int m_bodyUniqueId;
	int m_linkId;
	int m_isEnabled;
};

enum EnumInitPoseFlags
{
    INIT_POSE_HAS_INITIAL_POSITION=1,
    INIT_POSE_HAS_INITIAL_ORIENTATION=2,
    INIT_POSE_HAS_JOINT_STATE=4
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


struct RequestDebugLinesArgs
{
	int m_debugMode;
    int m_startingLineIndex;
};

struct SendDebugLinesArgs
{
    int m_startingLineIndex;
	int m_numDebugLines;
    int m_numRemainingDebugLines;
};

struct PickBodyArgs
{
    double m_rayFromWorld[3];
    double m_rayToWorld[3];
};


///Controlling a robot involves sending the desired state to its joint motor controllers.
///The control mode determines the state variables used for motor control.
struct SendDesiredStateArgs
{
	int m_bodyUniqueId;
	int m_controlMode;

	//PD parameters in case m_controlMode == CONTROL_MODE_POSITION_VELOCITY_PD
	double m_Kp[MAX_DEGREE_OF_FREEDOM];//indexed by degree of freedom, 6 for base, and then the dofs for each link
	double m_Kd[MAX_DEGREE_OF_FREEDOM];//indexed by degree of freedom, 6 for base, and then the dofs for each link

	//desired state is only written by the client, read-only access by server is expected

	//m_desiredStateQ is indexed by position variables, 
	//starting with 3 base position variables, 4 base orientation variables (quaternion), then link position variables
    double m_desiredStateQ[MAX_DEGREE_OF_FREEDOM];

	//m_desiredStateQdot is index by velocity degrees of freedom, 3 linear and 3 angular variables for the base and then link velocity variables
    double m_desiredStateQdot[MAX_DEGREE_OF_FREEDOM];
	
	//m_desiredStateForceTorque is either the actual applied force/torque (in CONTROL_MODE_TORQUE) or
	//or the maximum applied force/torque for the PD/motor/constraint to reach the desired velocity in CONTROL_MODE_VELOCITY and CONTROL_MODE_POSITION_VELOCITY_PD mode
	//indexed by degree of freedom, 6 dof base, and then dofs for each link
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

    double m_rootLocalInertialFrame[7];
    
	  //actual state is only written by the server, read-only access by client is expected
    double m_actualStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateQdot[MAX_DEGREE_OF_FREEDOM];

    //measured 6DOF force/torque sensors: force[x,y,z] and torque[x,y,z]
    double m_jointReactionForces[6*MAX_DEGREE_OF_FREEDOM];
  
};

enum EnumSensorTypes
{
    SENSOR_FORCE_TORQUE=1,
    SENSOR_IMU=2,
};

struct CreateSensorArgs
{
    int m_bodyUniqueId;
    int m_numJointSensorChanges;
    int m_sensorType[MAX_DEGREE_OF_FREEDOM];
    
///todo: clean up the duplication, make sure no-one else is using those members directly (use C-API header instead)
    int m_jointIndex[MAX_DEGREE_OF_FREEDOM];
    int m_enableJointForceSensor[MAX_DEGREE_OF_FREEDOM];

    int m_linkIndex[MAX_DEGREE_OF_FREEDOM];
    int m_enableSensor[MAX_DEGREE_OF_FREEDOM];
    
};

typedef  struct SharedMemoryCommand SharedMemoryCommand_t;

enum EnumBoxShapeFlags
{
    BOX_SHAPE_HAS_INITIAL_POSITION=1,
    BOX_SHAPE_HAS_INITIAL_ORIENTATION=2,
    BOX_SHAPE_HAS_HALF_EXTENTS=4,
	BOX_SHAPE_HAS_MASS=8,
	BOX_SHAPE_HAS_COLLISION_SHAPE_TYPE=16,
	BOX_SHAPE_HAS_COLOR=32,
};
///This command will be replaced to allow arbitrary collision shape types
struct CreateBoxShapeArgs
{
    double m_halfExtentsX;
    double m_halfExtentsY;
    double m_halfExtentsZ;

	double m_mass;
	int m_collisionShapeType;//see SharedMemoryPublic.h

    double m_initialPosition[3];
	double m_initialOrientation[4];
	double m_colorRGBA[4];
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
		struct RequestDebugLinesArgs m_requestDebugLinesArguments;
		struct PickBodyArgs m_pickBodyArguments;
    };
};

struct RigidBodyCreateArgs
{
	int m_bodyUniqueId; 
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
		struct SendDebugLinesArgs m_sendDebugLinesArgs;
		struct RigidBodyCreateArgs m_rigidBodyCreateArgs;
	};
};

typedef  struct SharedMemoryStatus SharedMemoryStatus_t;



#endif //SHARED_MEMORY_COMMANDS_H
