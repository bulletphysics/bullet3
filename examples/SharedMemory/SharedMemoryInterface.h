#ifndef SHARED_MEMORY_INTERFACE_H
#define SHARED_MEMORY_INTERFACE_H

#define SHARED_MEMORY_KEY 12347
#define SHARED_MEMORY_MAGIC_NUMBER 64738
#define SHARED_MEMORY_MAX_COMMANDS 64

enum SharedMemoryServerCommand{
    CMD_URDF_LOADING_COMPLETED,
    CMD_URDF_LOADING_FAILED,
    
    CMD_SERVER_STATE_UPDATE_COMPLETED,
    CMD_STEP_FORWARD_SIMULATION_COMPLETED,
    CMD_MAX_SERVER_COMMANDS
};

enum SharedMemoryClientCommand{
    CMD_LOAD_URDF,
    CMD_STATE_UPDATED,
    CMD_STEP_FORWARD_SIMULATION, //includes CMD_REQUEST_STATE
    CMD_SHUTDOWN,
    CMD_MAX_CLIENT_COMMANDS
};

#define SHARED_MEMORY_SERVER_TEST_C
#define MAX_DEGREE_OF_FREEDOM 1024
#define MAX_NUM_SENSORS 1024
#define MAX_URDF_FILENAME_LENGTH 1024


struct UrdfCommandArgument
{
    char m_urdfFileName[MAX_URDF_FILENAME_LENGTH];
};

struct StepSimulationCommandArgument
{
    double m_deltaTimeInSeconds;
};

struct SharedMemoryCommand
{
    int m_type;
    
    union
    {
        UrdfCommandArgument m_urdfArguments;
        StepSimulationCommandArgument m_stepSimulationArguments;
    };
};

struct SharedMemoryExampleData
{
    int m_magicId;
    SharedMemoryCommand m_clientCommands[SHARED_MEMORY_MAX_COMMANDS];
    SharedMemoryCommand m_serverCommands[SHARED_MEMORY_MAX_COMMANDS];
    
    int m_numClientCommands;
    int m_numProcessedClientCommands;
    
    int m_numServerCommands;
    int m_numProcessedServerCommands;
    
   
    //desired state is only written by the client, read-only access by server is expected
    double m_desiredStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_desiredStateQdot[MAX_DEGREE_OF_FREEDOM];
    double m_desiredStateSensors[MAX_NUM_SENSORS];//these are force sensors and IMU information
    
    //actual state is only written by the server, read-only access by client is expected
    double m_actualStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateQdot[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateSensors[MAX_NUM_SENSORS];//these are force sensors and IMU information
    
    
};

#define SHARED_MEMORY_SIZE sizeof(SharedMemoryExampleData)




class SharedMemoryInterface
{
	public:
	virtual ~SharedMemoryInterface()
	{
	}
	
	virtual void*	allocateSharedMemory(int key, int size) =0;
	virtual void releaseSharedMemory(int key, int size) =0;
};
#endif

