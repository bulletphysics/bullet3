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
	virtual void setup (btDynamicsWorld* dynamicsWorld, btScalar height = 2.0, btScalar width = 0.25, btScalar stepHeight = 0.25) = 0;
	virtual void destroy (btDynamicsWorld* dynamicsWorld) = 0;

	virtual btRigidBody* getRigidBody () = 0;

	virtual void preStep (btDynamicsWorld* dynamicsWorld) = 0;
	virtual void playerStep (btDynamicsWorld* dynamicsWorld, btScalar dt,
					         int forward,
							 int backward,
							 int left,
							 int right) = 0;
	virtual bool canJump () const = 0;
	virtual void jump () = 0;

	virtual bool onGround () const = 0;
};

#endif
