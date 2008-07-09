#ifndef CHARACTER_CONTROLLER_INTERFACE_H
#define CHARACTER_CONTROLLER_INTERFACE_H

#include "LinearMath/btVector3.h"

class btCollisionShape;
class btRigidBody;
class btDynamicsWorld;

class CharacterControllerInterface
{
public:
	CharacterControllerInterface () {};
	virtual ~CharacterControllerInterface () {};
	virtual void setup (btScalar height = 2.0, btScalar width = 0.25, btScalar stepHeight = 0.25) = 0;
	virtual void destroy () = 0;

	virtual btCollisionObject* getCollisionObject () = 0;

	virtual void reset () = 0;
	virtual void warp (const btVector3& origin) = 0;
	virtual void registerPairCacheAndDispatcher (btOverlappingPairCache* pairCache, btCollisionDispatcher* dispatcher)=0;
	virtual void preStep (const btDynamicsWorld* dynamicsWorld) = 0;
	virtual void playerStep (const btDynamicsWorld* dynamicsWorld, btScalar dt,
					         int forward,
							 int backward,
							 int left,
							 int right,
							 int jump) = 0;
	virtual bool canJump () const = 0;
	virtual void jump () = 0;

	virtual bool onGround () const = 0;
};

#endif
