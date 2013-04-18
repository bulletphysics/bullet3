#ifndef BT_GPU_DYNAMICS_WORLD_H
#define BT_GPU_DYNAMICS_WORLD_H

class b3Vector3;
class btRigidBody;
class btCollisionObject;
struct btGpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
class CLPhysicsDemo;

#include "Bullet3Common/b3AlignedObjectArray.h"
//#include "BulletDynamics/Dynamics/btDynamicsWorld.h"


class btGpuDynamicsWorld //: public btDynamicsWorld
{
	
	b3AlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	b3AlignedObjectArray<int> m_uniqueShapeMapping;


	CLPhysicsDemo*		m_gpuPhysics;
	b3Vector3			m_gravity;
	bool	m_once;
	
	bool initOpenCL(int preferredDeviceIndex, int preferredPlatformIndex, bool useInterop);
	void exitOpenCL();
	
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);

	
public:
	btGpuDynamicsWorld(int preferredOpenCLPlatformIndex,int preferredOpenCLDeviceIndex);

	virtual ~btGpuDynamicsWorld();

	virtual int		stepSimulation( b3Scalar timeStep,int maxSubSteps=1, b3Scalar fixedTimeStep=b3Scalar(1.)/b3Scalar(60.));

	virtual void	synchronizeMotionStates()
	{
		btAssert(0);
	}

	void	debugDrawWorld() {}

	void	setGravity(const b3Vector3& gravity);

	void	addRigidBody(btRigidBody* body);

	void	removeCollisionObject(btCollisionObject* colObj);

	

	b3AlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray();

	const b3AlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray() const;

	virtual void	addAction(btActionInterface* action)
	{
		btAssert(0);
	}

	virtual void	removeAction(btActionInterface* action)
	{
		btAssert(0);
	}


	b3Vector3 getGravity () const
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
