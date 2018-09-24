#ifndef PHYSICS_CLIENT_GRPC_H
#define PHYSICS_CLIENT_GRPC_H

#include "PhysicsDirect.h"
#include "PhysicsCommandProcessorInterface.h"

class GRPCNetworkedPhysicsProcessor : public PhysicsCommandProcessorInterface
{
	struct GRPCNetworkedInternalData* m_data;

public:
	GRPCNetworkedPhysicsProcessor(const char* hostName, int port);

	virtual ~GRPCNetworkedPhysicsProcessor();

	virtual bool connect();

	virtual void disconnect();

	virtual bool isConnected() const;

	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual void renderScene(int renderFlags);

	virtual void physicsDebugDraw(int debugDrawFlags);

	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);

	virtual void setTimeOut(double timeOutInSeconds);

	virtual void reportNotifications() {}
};

#endif  //PHYSICS_CLIENT_GRPC_H
