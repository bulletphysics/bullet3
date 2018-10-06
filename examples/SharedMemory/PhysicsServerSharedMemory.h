#ifndef PHYSICS_SERVER_SHARED_MEMORY_H
#define PHYSICS_SERVER_SHARED_MEMORY_H

#include "PhysicsServer.h"
#include "LinearMath/btQuaternion.h"

class PhysicsServerSharedMemory : public PhysicsServer
{
	struct PhysicsServerSharedMemoryInternalData* m_data;

protected:
	void releaseSharedMemory();

public:
	PhysicsServerSharedMemory(struct CommandProcessorCreationInterface* commandProcessorCreator, class SharedMemoryInterface* sharedMem, int bla);
	virtual ~PhysicsServerSharedMemory();

	virtual void setSharedMemoryKey(int key);

	//todo: implement option to allocated shared memory from client
	virtual bool connectSharedMemory(struct GUIHelperInterface* guiHelper);

	virtual void disconnectSharedMemory(bool deInitializeSharedMemory);

	virtual void processClientCommands();

	virtual void stepSimulationRealTime(double dtInSec, const struct b3VRControllerEvent* vrEvents, int numVREvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents);

	virtual void enableRealTimeSimulation(bool enableRealTimeSim);
	virtual bool isRealTimeSimulationEnabled() const;

	virtual void reportNotifications();

	//bool	supportsJointMotor(class btMultiBody* body, int linkIndex);

	///The pickBody method will try to pick the first body along a ray, return true if succeeds, false otherwise
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();

	virtual const btVector3& getVRTeleportPosition() const;
	virtual void setVRTeleportPosition(const btVector3& vrTeleportPos);

	virtual const btQuaternion& getVRTeleportOrientation() const;
	virtual void setVRTeleportOrientation(const btQuaternion& vrTeleportOrn);

	//for physicsDebugDraw and renderScene are mainly for debugging purposes
	//and for physics visualization. The idea is that physicsDebugDraw can also send wireframe
	//to a physics client, over shared memory
	void physicsDebugDraw(int debugDrawFlags);
	void renderScene(int renderFlags);
	void syncPhysicsToGraphics();

	void enableCommandLogging(bool enable, const char* fileName);
	void replayFromLogFile(const char* fileName);
};

#endif  //PHYSICS_SERVER_EXAMPLESHARED_MEMORY_H
