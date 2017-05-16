#ifndef PHYSICS_CLIENT_UDP_H
#define PHYSICS_CLIENT_UDP_H

#include "PhysicsDirect.h"
#include "PhysicsServerCommandProcessor.h"

class UdpNetworkedPhysicsProcessor : public PhysicsCommandProcessorInterface
{

	struct UdpNetworkedInternalData* m_data;

public:
	UdpNetworkedPhysicsProcessor(const char* hostName, int port);

	virtual ~UdpNetworkedPhysicsProcessor();

	virtual bool connect();

	virtual void disconnect();

	virtual bool isConnected() const;
	
	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual void renderScene(int renderFlags);

	virtual void   physicsDebugDraw(int debugDrawFlags);

	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);

	virtual void setTimeOut(double timeOutInSeconds);
};


#endif //PHYSICS_CLIENT_UDP_H

