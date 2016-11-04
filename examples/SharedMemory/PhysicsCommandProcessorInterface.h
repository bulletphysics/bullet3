#ifndef PHYSICS_COMMAND_PROCESSOR_INTERFACE_H
#define PHYSICS_COMMAND_PROCESSOR_INTERFACE_H

class PhysicsCommandProcessorInterface
{

public:
	virtual ~PhysicsCommandProcessorInterface() {}

	virtual bool connect()=0;

	virtual void disconnect() = 0;

	virtual bool isConnected() const = 0;

	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes) = 0;

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes) = 0;

	virtual void renderScene() = 0;
	virtual void   physicsDebugDraw(int debugDrawFlags) = 0;
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper) = 0;


};

#endif //PHYSICS_COMMAND_PROCESSOR_INTERFACE_H
