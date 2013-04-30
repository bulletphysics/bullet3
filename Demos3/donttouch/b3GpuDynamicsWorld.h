#ifndef B3_GPU_DYNAMICS_WORLD_H
#define B3_GPU_DYNAMICS_WORLD_H

class b3Vector3;
class b3RigidBody;
class b3CollisionObject;
struct b3GpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
class CLPhysicsDemo;

#include "Bullet3Common/b3AlignedObjectArray.h"
//#include "BulletDynamics/Dynamics/b3DynamicsWorld.h"


class b3GpuDynamicsWorld //: public b3DynamicsWorld
{
	
	b3AlignedObjectArray<const class  b3CollisionShape*> m_uniqueShapes;
	b3AlignedObjectArray<int> m_uniqueShapeMapping;


	CLPhysicsDemo*		m_gpuPhysics;
	b3Vector3			m_gravity;
	bool	m_once;
	
	bool initOpenCL(int preferredDeviceIndex, int preferredPlatformIndex, bool useInterop);
	void exitOpenCL();
	
	int findOrRegisterCollisionShape(const b3CollisionShape* colShape);

	
public:
	b3GpuDynamicsWorld(int preferredOpenCLPlatformIndex,int preferredOpenCLDeviceIndex);

	virtual ~b3GpuDynamicsWorld();

	virtual int		stepSimulation( b3Scalar timeStep,int maxSubSteps=1, b3Scalar fixedTimeStep=b3Scalar(1.)/b3Scalar(60.));

	virtual void	synchronizeMotionStates()
	{
		b3Assert(0);
	}

	void	debugDrawWorld() {}

	void	setGravity(const b3Vector3& gravity);

	void	addRigidBody(b3RigidBody* body);

	void	removeCollisionObject(b3CollisionObject* colObj);

	

	b3AlignedObjectArray<class b3CollisionObject*>& getCollisionObjectArray();

	const b3AlignedObjectArray<class b3CollisionObject*>& getCollisionObjectArray() const;

	virtual void	addAction(b3ActionInterface* action)
	{
		b3Assert(0);
	}

	virtual void	removeAction(b3ActionInterface* action)
	{
		b3Assert(0);
	}


	b3Vector3 getGravity () const
	{
		return m_gravity;
	}

		virtual void	addRigidBody(b3RigidBody* body, short group, short mask)
		{
			addRigidBody(body);
		}

		virtual void	removeRigidBody(b3RigidBody* body)
		{
			b3Assert(0);
		}

		virtual void	setConstraintSolver(b3ConstraintSolver* solver)
		{
			b3Assert(0);
		}

		virtual b3ConstraintSolver* getConstraintSolver()
		{
			b3Assert(0);
			return 0;
		}

		virtual b3DynamicsWorldType	getWorldType() const
		{
			return B3_GPU_PHYSICS_WORLD;
		}

		virtual void	clearForces()
		{
			b3Assert(0);
		}

		

};


#endif //B3_GPU_DYNAMICS_WORLD_H
