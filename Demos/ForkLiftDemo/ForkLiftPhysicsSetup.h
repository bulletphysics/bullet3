#ifndef FORK_LIFT_PHYSICS_SETUP_H
#define FORK_LIFT_PHYSICS_SETUP_H

class btRigidBody;
class btCollisionShape;
class btBroadphaseInterface;
class btConstraintSolver;
class btCollisionDispatcher;
class btDefaultCollisionConfiguration;
class btDiscreteDynamicsWorld;
class btTransform;
class btVector3;
class btBoxShape;

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "Bullet3AppSupport/CommonRigidBodySetup.h"

class ForkLiftPhysicsSetup : public CommonPhysicsSetup
{

protected:

	struct ForkLiftInternalData* m_data;

public:

	ForkLiftPhysicsSetup();
	virtual ~ForkLiftPhysicsSetup();
	
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
	
	
	virtual void exitPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual void    debugDraw(int debugDrawFlags);
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();
	virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge);
	virtual void renderScene(GraphicsPhysicsBridge& gfxBridge);

	void resetForklift();
	void lockLiftHinge();
	void lockForkSlider();
	class btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTrans, btCollisionShape* shape);

};

#endif //FORK_LIFT_PHYSICS_SETUP_H
