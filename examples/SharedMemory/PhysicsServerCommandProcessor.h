#ifndef PHYSICS_SERVER_COMMAND_PROCESSOR_H
#define PHYSICS_SERVER_COMMAND_PROCESSOR_H

#include "LinearMath/btVector3.h"

#include "PhysicsCommandProcessorInterface.h"

struct SharedMemLines
{
	btVector3 m_from;
	btVector3 m_to;
	btVector3 m_color;
};



///todo: naming. Perhaps PhysicsSdkCommandprocessor?
class PhysicsServerCommandProcessor : public CommandProcessorInterface
{

	struct PhysicsServerCommandProcessorInternalData* m_data;

	void resetSimulation();

protected:




	bool loadSdf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags);

	bool loadUrdf(const char* fileName, const class btVector3& pos, const class btQuaternion& orn,
		bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes, int flags=0);

	bool loadMjcf(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags);

	bool processImportedObjects(const char* fileName, char* bufferServerToClient, int bufferSizeInBytes, bool useMultiBody, int flags, class URDFImporterInterface& u2b);

	bool	supportsJointMotor(class btMultiBody* body, int linkIndex);

	int createBodyInfoStream(int bodyUniqueId, char* bufferServerToClient, int bufferSizeInBytes);
	void deleteCachedInverseDynamicsBodies();
	void deleteCachedInverseKinematicsBodies();
	void deleteStateLoggers();

public:
	PhysicsServerCommandProcessor();
	virtual ~PhysicsServerCommandProcessor();

	void	createJointMotors(class btMultiBody* body);

	virtual void createEmptyDynamicsWorld();
	virtual void deleteDynamicsWorld();

	virtual bool connect()
	{
		return true;
	};

	virtual void disconnect() {}

	virtual bool isConnected() const
	{
		return true;
	}



	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes);

	virtual bool receiveStatus(struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
	{
		return false;
	};

	virtual void renderScene(int renderFlags);
	virtual void   physicsDebugDraw(int debugDrawFlags);
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);
	virtual void syncPhysicsToGraphics();


	//@todo(erwincoumans) Should we have shared memory commands for picking objects?
	///The pickBody method will try to pick the first body along a ray, return true if succeeds, false otherwise
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();

	//logging /playback the shared memory commands
	virtual void enableCommandLogging(bool enable, const char* fileName);
	virtual void replayFromLogFile(const char* fileName);
	virtual void replayLogCommand(char* bufferServerToClient, int bufferSizeInBytes );

	//logging of object states (position etc)
	void logObjectStates(btScalar timeStep);
	void processCollisionForces(btScalar timeStep);

	virtual void stepSimulationRealTime(double dtInSec,	const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents);
	virtual void enableRealTimeSimulation(bool enableRealTimeSim);
	virtual bool isRealTimeSimulationEnabled() const;

	void applyJointDamping(int bodyUniqueId);

	virtual void setTimeOut(double timeOutInSeconds);

	virtual const btVector3& getVRTeleportPosition() const;
	virtual void setVRTeleportPosition(const btVector3& vrTeleportPos);

	virtual const btQuaternion& getVRTeleportOrientation() const;
	virtual void setVRTeleportOrientation(const btQuaternion& vrTeleportOrn);
};

#endif //PHYSICS_SERVER_COMMAND_PROCESSOR_H
