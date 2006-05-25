#ifndef BULLET_ODE_COLLISION_PAIR_H
#define BULLET_ODE_COLLISION_PAIR_H

#include <BroadphaseCollision/CollisionAlgorithm.h>
#include <ode/common.h>
class PersistentManifold;

/// BulletOdeCollisionPair provides Bullet convex collision detection for Open Dynamics Engine
class BulletOdeCollisionPair : public CollisionAlgorithm
{
public:
	BulletOdeCollisionPair(dGeomID o1,dGeomID o2);
	virtual ~BulletOdeCollisionPair();

	PersistentManifold*	m_manifold;
	dGeomID m_o1;
	dGeomID m_o2;

	class CollisionShape*	m_shape1;

	class CollisionShape*	m_shape2;


	void	CalculateContacts();

	CollisionShape*	CreateShapeFromGeom(dGeomID);

	///future support for Bullet broadphase + framework?
	virtual void ProcessCollision (BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const struct DispatcherInfo& dispatchInfo);

	///future support for Bullet broadphase + framework?
	virtual float CalculateTimeOfImpact(BroadphaseProxy* proxy0,BroadphaseProxy* proxy1,const struct DispatcherInfo& dispatchInfo);
};

#endif //BULLET_ODE_COLLISION_PAIR_H