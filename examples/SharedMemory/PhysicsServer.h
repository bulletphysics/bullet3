#ifndef PHYSICS_SERVER_SHARED_MEMORY_H
#define PHYSICS_SERVER_SHARED_MEMORY_H

class PhysicsServerSharedMemory
{
	struct PhysicsServerInternalData* m_data;

protected:

	void	createJointMotors(class btMultiBody* body);
	
	virtual void createEmptyDynamicsWorld();
	virtual void deleteDynamicsWorld();
	
	void	releaseSharedMemory();
	
	bool loadUrdf(const char* fileName, const class btVector3& pos, const class btQuaternion& orn,
                             bool useMultiBody, bool useFixedBase);

public:
	PhysicsServerSharedMemory();
	virtual ~PhysicsServerSharedMemory();

	virtual void setSharedMemoryKey(int key);
	
	//todo: implement option to allocated shared memory from client 
	virtual bool connectSharedMemory( struct GUIHelperInterface* guiHelper);

	virtual void disconnectSharedMemory (bool deInitializeSharedMemory);

	virtual void processClientCommands();

	bool	supportsJointMotor(class btMultiBody* body, int linkIndex);

	//for physicsDebugDraw and renderScene are mainly for debugging purposes
	//and for physics visualization. The idea is that physicsDebugDraw can also send wireframe
	//to a physics client, over shared memory
	void    physicsDebugDraw(int debugDrawFlags);
	void    renderScene();
	
};


#endif //PHYSICS_SERVER_EXAMPLESHARED_MEMORY_H


