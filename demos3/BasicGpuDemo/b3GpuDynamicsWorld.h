#ifndef B3_GPU_DYNAMICS_WORLD_H
#define B3_GPU_DYNAMICS_WORLD_H

#include "LinearMath/btVector3.h"

class btRigidBody;
class btCollisionObject;
struct b3GpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
class CLPhysicsDemo;
class btActionInterface;

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"


class b3GpuDynamicsWorld : public btDynamicsWorld
{
	
	btAlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	btAlignedObjectArray<int> m_uniqueShapeMapping;


	class b3GpuRigidBodyPipeline* m_rigidBodyPipeline;
	class b3GpuNarrowPhase* m_np;
	class b3GpuSapBroadphase* m_bp;

	
	btVector3			m_gravity;
	bool	m_cpuGpuSync;
	
		
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);

	
public:
	b3GpuDynamicsWorld(class b3GpuSapBroadphase* bp,class b3GpuNarrowPhase* np, class b3GpuRigidBodyPipeline* rigidBodyPipeline);

	virtual ~b3GpuDynamicsWorld();

	virtual int		stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	virtual void	synchronizeMotionStates()
	{
		btAssert(0);
	}

	void	debugDrawWorld() {}

	void	setGravity(const btVector3& gravity);

	void	addRigidBody(btRigidBody* body);

	void	removeCollisionObject(btCollisionObject* colObj);

	void	rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const;

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


		virtual void	addAction(btActionInterface* action) 
		{
			btAssert(0);
		}

		virtual void	removeAction(btActionInterface* action)
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
		

		virtual void	clearForces()
		{
			btAssert(0);
		}

		virtual btDynamicsWorldType	getWorldType() const
		{
			return BT_GPU_DYNAMICS_WORLD;
		}

};


#endif //B3_GPU_DYNAMICS_WORLD_H
