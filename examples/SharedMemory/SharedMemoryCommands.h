
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

enum SharedMemoryServerCommand
{
    CMD_URDF_LOADING_COMPLETED,
    CMD_URDF_LOADING_FAILED,
	CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,
	CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,
	CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,
	CMD_RIGID_BODY_CREATION_COMPLETED,
	CMD_SET_JOINT_FEEDBACK_COMPLETED,
	CMD_ACTUAL_STATE_UPDATE_COMPLETED,
	CMD_DESIRED_STATE_RECEIVED_COMPLETED,
    CMD_STEP_FORWARD_SIMULATION_COMPLETED,
    CMD_MAX_SERVER_COMMANDS
};

enum SharedMemoryClientCommand
{
    CMD_LOAD_URDF,
	CMD_SEND_BULLET_DATA_STREAM,
	CMD_CREATE_BOX_COLLISION_SHAPE,
	CMD_DELETE_BOX_COLLISION_SHAPE,
	CMD_CREATE_RIGID_BODY,
	CMD_DELETE_RIGID_BODY,
	CMD_SET_JOINT_FEEDBACK,///enable or disable joint feedback
	CMD_SEND_DESIRED_STATE,
	CMD_REQUEST_ACTUAL_STATE,
    CMD_STEP_FORWARD_SIMULATION, //includes CMD_REQUEST_STATE
    CMD_SHUTDOWN,
    CMD_MAX_CLIENT_COMMANDS
};

#define SHARED_MEMORY_SERVER_TEST_C
#define MAX_DEGREE_OF_FREEDOM 1024
#define MAX_NUM_SENSORS 1024
#define MAX_URDF_FILENAME_LENGTH 1024


struct UrdfArgs
{
    char m_urdfFileName[MAX_URDF_FILENAME_LENGTH];
    double m_initialPosition[3];
    double m_initialOrientation[4];
	bool m_useMultiBody;
	bool m_useFixedBase;
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
	bool m_isEnabled;
};

//todo: discuss and decide about control mode and combinations
enum {
//    POSITION_CONTROL=0,
    CONTROL_MODE_VELOCITY,
    CONTROL_MODE_TORQUE,
};

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
    double m_actualStateSensors[MAX_NUM_SENSORS];//these are force sensors and IMU information
  
};




struct SharedMemoryCommand
{
    int m_type;
	
    smUint64_t	m_timeStamp;
	int	m_sequenceNumber;

    union
    {
        UrdfArgs m_urdfArguments;
		BulletDataStreamArgs	m_dataStreamArguments;
  		SendDesiredStateArgs m_sendDesiredStateCommandArgument;
		RequestActualStateArgs m_requestActualStateInformationCommandArgument;
		SendActualStateArgs m_sendActualStateArgs;
    };
};

typedef SharedMemoryCommand ServerStatus;

#endif //SHARED_MEMORY_COMMANDS_H
