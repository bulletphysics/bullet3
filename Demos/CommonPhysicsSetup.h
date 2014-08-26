
#ifndef COMMON_PHYSICS_SETUP_H
#define COMMON_PHYSICS_SETUP_H

class btRigidBody;
class btCollisionObject;
class btBoxShape;
class btTransform;
class btCollisionShape;
#include "LinearMath/btVector3.h"
#include "CommonParameterInterface.h"

class btDiscreteDynamicsWorld;

///The GraphicsPhysicsBridge let's the graphics engine create graphics representation and synchronize
struct GraphicsPhysicsBridge
{

	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color)
	{
	}

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
	}
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
	}
	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
	}
	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld)
	{
	}

	virtual CommonParameterInterface* getParameterInterface()
	{
		return 0;
	}

	virtual void setUpAxis(int axis)
	{
	}

};

struct CommonPhysicsSetup
{
public:

	virtual ~CommonPhysicsSetup() {}

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge) = 0;

	virtual void exitPhysics()=0;

	virtual void stepSimulation(float deltaTime)=0;

    virtual void    debugDraw()=0;

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld) = 0;
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)=0;
	virtual void removePickingConstraint() = 0;

	virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge) = 0;

	virtual btRigidBody*	createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, const btVector4& color=btVector4(1,0,0,1))=0;

	virtual btBoxShape* createBoxShape(const btVector3& halfExtents)=0;
};



#endif //COMMON_PHYSICS_SETUP_H


