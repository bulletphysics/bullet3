#ifndef SHARED_MEMORY_INTERFACE_H
#define SHARED_MEMORY_INTERFACE_H

#define SHARED_MEMORY_KEY 12347
#define SHARED_MEMORY_MAGIC_NUMBER 64738
#define SHARED_MEMORY_MAX_COMMANDS 64
#define SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE (256*1024)

#include "SharedMemoryCommands.h"

struct SharedMemoryExampleData
{
    int m_magicId;
    SharedMemoryCommand m_clientCommands[SHARED_MEMORY_MAX_COMMANDS];
    SharedMemoryCommand m_serverCommands[SHARED_MEMORY_MAX_COMMANDS];
    
    int m_numClientCommands;
    int m_numProcessedClientCommands;
    
    int m_numServerCommands;
    int m_numProcessedServerCommands;
  
	//TODO: it is better to move this single desired/actual state (m_desiredStateQ, m_actualStateQ etc) into the command,
	//so we can deal with multiple bodies/robots
	
    //desired state is only written by the client, read-only access by server is expected
    double m_desiredStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_desiredStateQdot[MAX_DEGREE_OF_FREEDOM];
    
    //actual state is only written by the server, read-only access by client is expected
    double m_actualStateQ[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateQdot[MAX_DEGREE_OF_FREEDOM];
    double m_actualStateSensors[MAX_NUM_SENSORS];//these are force sensors and IMU information
        
	char	m_bulletStreamDataClientToServer[SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE];
	char	m_bulletStreamDataServerToClient[SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE];
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

