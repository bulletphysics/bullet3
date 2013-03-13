#ifndef BT_GPU_DYNAMICS_WORLD_H
#define BT_GPU_DYNAMICS_WORLD_H

class btVector3;
class btRigidBody;
class btCollisionObject;
struct btGpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
class CLPhysicsDemo;

#include "BulletCommon/btAlignedObjectArray.h"
//#include "BulletDynamics/Dynamics/btDynamicsWorld.h"


class btGpuDynamicsWorld //: public btDynamicsWorld
{
	
	btAlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	btAlignedObjectArray<int> m_uniqueShapeMapping;


	CLPhysicsDemo*		m_gpuPhysics;
	btVector3			m_gravity;
	bool	m_once;
	
	bool initOpenCL(int preferredDeviceIndex, int preferredPlatformIndex, bool useInterop);
	void exitOpenCL();
	
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);

	
public:
	btGpuDynamicsWorld(int preferredOpenCLPlatformIndex,int preferredOpenCLDeviceIndex);

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

	virtual void	addAction(btActionInterface* action)
	{
		btAssert(0);
	}

	virtual void	removeAction(btActionInterface* action)
	{
		btAssert(0);
	}


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

		virtual void	setConstraintSolver(btConstraintSolver* solver)
		{
			btAssert(0);
		}

		virtual btConstraintSolver* getConstraintSolver()
		{
			btAssert(0);
			return 0;
		}

		virtual btDynamicsWorldType	getWorldType() const
		{
			return BT_GPU_PHYSICS_WORLD;
		}

		virtual void	clearForces()
		{
			btAssert(0);
		}

		

};


#endif //BT_GPU_DYNAMICS_WORLD_H
