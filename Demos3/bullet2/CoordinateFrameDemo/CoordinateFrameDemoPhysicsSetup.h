#ifndef COORDINATE_FRAME_DEMO_PHYSICS_SETUP_H
#define COORDINATE_FRAME_DEMO_PHYSICS_SETUP_H

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

struct CoordinateFrameDemoPhysicsSetup : public CommonRigidBodySetup
{
	
	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);
	virtual void    debugDraw();
};

#endif //COORDINATE_FRAME_DEMO_PHYSICS_SETUP_H
