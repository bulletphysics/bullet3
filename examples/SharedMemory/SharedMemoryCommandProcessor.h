#ifndef SHARED_MEMORY_COMMAND_PROCESSOR_H
#define SHARED_MEMORY_COMMAND_PROCESSOR_H

#include "PhysicsCommandProcessorInterface.h"

class SharedMemoryCommandProcessor : public PhysicsCommandProcessorInterface
{

	struct SharedMemoryCommandProcessorInternalData* m_data;

public:
	SharedMemoryCommandProcessor();

	virtual ~SharedMemoryCommandProcessor();

	virtual bool connect();

	virtual void disconnect();

	virtual bool isConnected() const;

	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual void renderScene();
	virtual void   physicsDebugDraw(int debugDrawFlags);
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);

	void setSharedMemoryInterface(class SharedMemoryInterface* sharedMem);
	void setSharedMemoryKey(int key);


};

#endif //SHARED_MEMORY_COMMAND_PROCESSOR_H

