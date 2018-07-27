#ifndef PHYSICS_COMMAND_PROCESSOR_INTERFACE_H
#define PHYSICS_COMMAND_PROCESSOR_INTERFACE_H

enum PhysicsCommandRenderFlags
{
	COV_DISABLE_SYNC_RENDERING=1
};

class PhysicsCommandProcessorInterface
{

public:
	virtual ~PhysicsCommandProcessorInterface() {}

	virtual bool connect()=0;

	virtual void disconnect() = 0;

	virtual bool isConnected() const = 0;

	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes) = 0;

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes) = 0;

	virtual void renderScene(int renderFlags) = 0;
	virtual void   physicsDebugDraw(int debugDrawFlags) = 0;
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper) = 0;
	virtual void setTimeOut(double timeOutInSeconds) = 0;

};


class btVector3;
class btQuaternion;

class CommandProcessorInterface : public PhysicsCommandProcessorInterface
{

public:
	virtual ~CommandProcessorInterface(){}

	virtual void syncPhysicsToGraphics()=0;
	virtual void stepSimulationRealTime(double dtInSec,const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents)=0;
	virtual void enableRealTimeSimulation(bool enableRealTimeSim)=0;
	virtual bool isRealTimeSimulationEnabled() const=0;

	virtual void reportNotifications() = 0;

	virtual void enableCommandLogging(bool enable, const char* fileName)=0;
	virtual void replayFromLogFile(const char* fileName)=0;
	virtual void replayLogCommand(char* bufferServerToClient, int bufferSizeInBytes )=0;

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)=0;
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)=0;
	virtual void removePickingConstraint()=0;

	virtual const btVector3& getVRTeleportPosition() const=0;
	virtual void setVRTeleportPosition(const btVector3& vrReleportPos)=0;

	virtual const btQuaternion& getVRTeleportOrientation() const=0;
	virtual void setVRTeleportOrientation(const btQuaternion& vrReleportOrn)=0;
};

#endif //PHYSICS_COMMAND_PROCESSOR_INTERFACE_H
