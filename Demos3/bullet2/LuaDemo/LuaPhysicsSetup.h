#ifndef _LUA_PHYSICS_SETUP_H
#define _LUA_PHYSICS_SETUP_H

#include "Bullet3AppSupport/CommonPhysicsSetup.h"

//we don't derive from CommonRigidBodySetup because we
//create and own our own dynamics world (one or more)
//at run-time
struct LuaPhysicsSetup : public CommonPhysicsSetup
{

    LuaPhysicsSetup(struct CommonGraphicsApp* app);
    virtual ~LuaPhysicsSetup();

	class btDefaultCollisionConfiguration* m_config;
	class btCollisionDispatcher* m_dispatcher;
	class btDbvtBroadphase* m_bp;
	class btNNCGConstraintSolver* m_solver;
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
    struct CommonGraphicsApp* m_glApp;

	virtual void initPhysics(GraphicsPhysicsBridge& gfxBridge);

	virtual void exitPhysics();

	virtual void stepSimulation(float deltaTime);

    virtual void    debugDraw(int debugDrawFlags);

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld);
	virtual void removePickingConstraint();

	virtual void syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge);

	virtual btRigidBody*	createRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, const btVector4& color=btVector4(1,0,0,1));

	virtual btBoxShape* createBoxShape(const btVector3& halfExtents);

};


#endif //_LUA_PHYSICS_SETUP_H
