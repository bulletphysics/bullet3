#ifndef PHYSICS_SERVER_SHARED_MEMORY_H
#define PHYSICS_SERVER_SHARED_MEMORY_H

#include "PhysicsServer.h"
 
class PhysicsServerSharedMemory : public PhysicsServer
{
	struct PhysicsServerSharedMemoryInternalData* m_data;

protected:

	
	void	releaseSharedMemory();
	
	
public:
	PhysicsServerSharedMemory(class SharedMemoryInterface* sharedMem=0);
	virtual ~PhysicsServerSharedMemory();

	virtual void setSharedMemoryKey(int key);
	
	//todo: implement option to allocated shared memory from client 
	virtual bool connectSharedMemory( struct GUIHelperInterface* guiHelper);

	virtual void disconnectSharedMemory (bool deInitializeSharedMemory);

	virtual void processClientCommands();

	virtual void stepSimulationRealTime(double dtInSec,const struct b3VRControllerEvent* vrEvents, int numVREvents);

	virtual void enableRealTimeSimulation(bool enableRealTimeSim);

	//bool	supportsJointMotor(class btMultiBody* body, int linkIndex);

	
	///The pickBody method will try to pick the first body along a ray, return true if succeeds, false otherwise
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();

	//for physicsDebugDraw and renderScene are mainly for debugging purposes
	//and for physics visualization. The idea is that physicsDebugDraw can also send wireframe
	//to a physics client, over shared memory
	void    physicsDebugDraw(int debugDrawFlags);
	void    renderScene();

	void enableCommandLogging(bool enable, const char* fileName);
	void replayFromLogFile(const char* fileName);
	
	void resetDynamicsWorld();

};


#endif //PHYSICS_SERVER_EXAMPLESHARED_MEMORY_H


