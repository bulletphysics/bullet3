#ifndef SHARED_MEMORY_INTERFACE_H
#define SHARED_MEMORY_INTERFACE_H

#define SHARED_MEMORY_KEY 12345
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
    CMD_MAX_CLIENT_COMMANDS
};

#define SHARED_MEMORY_SERVER_TEST_C
#define MAX_DEGREE_OF_FREEDOM 1024
#define MAX_NUM_SENSORS 1024
#define MAX_URDF_FILENAME_LENGTH 1024

struct CommandArguments
{
    double m_deltaTimeInSeconds;
    char m_urdfFileName[MAX_URDF_FILENAME_LENGTH];
};

struct SharedMemoryExampleData
{
    int m_magicId;
    int m_clientCommands[SHARED_MEMORY_MAX_COMMANDS];
    int m_serverCommands[SHARED_MEMORY_MAX_COMMANDS];
    
    int m_numClientCommands;
    int m_numProcessedClientCommands;
    
    int m_numServerCommands;
    int m_numProcessedServerCommands;
    
    
    double m_stateQ[MAX_DEGREE_OF_FREEDOM];
    double m_stateQdot[MAX_DEGREE_OF_FREEDOM];
    double m_stateSensors[MAX_NUM_SENSORS];//these are force sensors and IMU information
    
    CommandArguments m_clientCommandArguments[SHARED_MEMORY_MAX_COMMANDS];
    CommandArguments m_serverCommandArguments[SHARED_MEMORY_MAX_COMMANDS];
    
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

