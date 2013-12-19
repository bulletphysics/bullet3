#ifndef B3_GPU_DYNAMICS_WORLD_H
#define B3_GPU_DYNAMICS_WORLD_H

#include "LinearMath/btVector3.h"

class btRigidBody;
class btCollisionObject;
struct b3GpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
class CLPhysicsDemo;
class btActionInterface;
class btTypedConstraint;

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "Bullet3Common/b3Logging.h"


class b3GpuDynamicsWorld : public btDynamicsWorld
{
	
	btAlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	btAlignedObjectArray<int> m_uniqueShapeMapping;
	btAlignedObjectArray<btTypedConstraint*>	m_constraints;
	btAlignedObjectArray<int> m_bodyUpdateRevisions;
	class b3GpuRigidBodyPipeline* m_rigidBodyPipeline;
	class b3GpuNarrowPhase* m_np;
	class b3GpuSapBroadphase* m_bp;

	
	btVector3			m_gravity;
	bool	m_cpuGpuSync;
	float	m_localTime;
	class btRigidBody*		m_staticBody;//used for picking and Bullet 2.x compatibility. In Bullet 3.x all constraints have explicitly 2 bodies.
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);
	
	
	
public:
	b3GpuDynamicsWorld(class b3GpuSapBroadphase* bp,class b3GpuNarrowPhase* np, class b3GpuRigidBodyPipeline* rigidBodyPipeline);

	virtual ~b3GpuDynamicsWorld();

	virtual int		stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	virtual void	synchronizeMotionStates();
	

	void	debugDrawWorld();

	void	setGravity(const btVector3& gravity);

	void	addRigidBody(btRigidBody* body);

	void	removeCollisionObject(btCollisionObject* colObj);

	virtual void	removeRigidBody(btRigidBody* body);

	virtual void	addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false);

	virtual void	removeConstraint(btTypedConstraint* constraint);
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
		

	virtual void	clearForces();

	virtual btDynamicsWorldType	getWorldType() const
	{
		return BT_GPU_DYNAMICS_WORLD;
	}

	///this can be useful to synchronize a single rigid body -> graphics object
	void	synchronizeSingleMotionState(btRigidBody* body);

	void reset();
};

#endif //B3_GPU_DYNAMICS_WORLD_H
