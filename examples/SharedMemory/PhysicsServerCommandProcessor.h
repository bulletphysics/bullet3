#ifndef PHYSICS_SERVER_COMMAND_PROCESSOR_H
#define PHYSICS_SERVER_COMMAND_PROCESSOR_H

#include "LinearMath/btVector3.h"

struct SharedMemLines
{
	btVector3 m_from;
	btVector3 m_to;
	btVector3 m_color;
};

///todo: naming. Perhaps PhysicsSdkCommandprocessor?
class PhysicsServerCommandProcessor
{

	struct PhysicsServerCommandProcessorInternalData* m_data;

protected:


	bool loadUrdf(const char* fileName, const class btVector3& pos, const class btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase, int* bodyUniqueIdPtr, char* bufferServerToClient, int bufferSizeInBytes);

	bool	supportsJointMotor(class btMultiBody* body, int linkIndex);

public:
	PhysicsServerCommandProcessor();
	virtual ~PhysicsServerCommandProcessor();

	void	createJointMotors(class btMultiBody* body);
	
	virtual void createEmptyDynamicsWorld();
	virtual void deleteDynamicsWorld();

	virtual bool processCommand(const struct SharedMemoryCommand& clientCmd, struct SharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes );

	virtual void renderScene();
	virtual void   physicsDebugDraw(int debugDrawFlags);
	virtual void setGuiHelper(struct GUIHelperInterface* guiHelper);
	
	//@todo(erwincoumans) Should we have shared memory commands for picking objects?
	///The pickBody method will try to pick the first body along a ray, return true if succeeds, false otherwise
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();
	
	void enableCommandLogging(bool enable, const char* fileName);
	void replayFromLogFile(const char* fileName);

};

#endif //PHYSICS_SERVER_COMMAND_PROCESSOR_H
