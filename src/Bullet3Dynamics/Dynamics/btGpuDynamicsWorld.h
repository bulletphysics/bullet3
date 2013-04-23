#ifndef BT_GPU_DYNAMICS_WORLD_H
#define BT_GPU_DYNAMICS_WORLD_H

class btVector3;
class btRigidBody;
class btCollisionObject;
struct btGpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base

#include "BulletCommon/btAlignedObjectArray.h"
//#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCommon/btTransform.h"

class btGpuDynamicsWorld 
{
	
	btAlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	btAlignedObjectArray<int> m_uniqueShapeMapping;

	btAlignedObjectArray<btCollisionObject*> m_collisionObjects;

	class btGpuSapBroadphase* m_bp;
	class btGpuNarrowPhase* m_np;
	class btGpuRigidBodyPipeline* m_rigidBodyPipeline;


	btVector3			m_gravity;
	bool	m_once;
	
	bool initOpenCL(int preferredDeviceIndex, int preferredPlatformIndex, bool useInterop);
	void exitOpenCL();
	
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);

	
public:
	btGpuDynamicsWorld(btGpuSapBroadphase*bp, btGpuNarrowPhase* np, btGpuRigidBodyPipeline* rb);

	virtual ~btGpuDynamicsWorld();

	virtual int		stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	virtual void	synchronizeMotionStates()
	{
		btAssert(0);
	}

	void	debugDrawWorld() {}

	void	setGravity(const btVector3& gravity);

	void	addRigidBody(btRigidBody* body);

	void	removeCollisionObject(btCollisionObject* colObj);

	

	btAlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray();

	const btAlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray() const;



	btVector3 getGravity () const
	{
		return m_gravity;
	}

		virtual void	addRigidBody(btRigidBody* body, short group, short mask)
		{
			addRigidBody(body);
		}

		virtual void	removeRigidBody(btRigidBody* body)
		{
			btAssert(0);
		}


		virtual void	clearForces()
		{
			btAssert(0);
		}

		

};


#endif //BT_GPU_DYNAMICS_WORLD_H
