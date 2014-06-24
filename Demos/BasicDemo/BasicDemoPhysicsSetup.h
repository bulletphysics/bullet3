#ifndef BASIC_DEMO_PHYSICS_SETUP_H
#define BASIC_DEMO_PHYSICS_SETUP_H

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


#include "../CommonRigidBodySetup.h"

struct BasicDemoPhysicsSetup : public CommonRigidBodySetup
{
	
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

};

#endif //BASIC_DEMO_PHYSICS_SETUP_H
