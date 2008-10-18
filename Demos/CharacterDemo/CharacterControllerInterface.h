#ifndef CHARACTER_CONTROLLER_INTERFACE_H
#define CHARACTER_CONTROLLER_INTERFACE_H

#include "LinearMath/btVector3.h"

class btCollisionShape;
class btRigidBody;
class btCollisionWorld;

class CharacterControllerInterface
{
public:
	CharacterControllerInterface () {};
	virtual ~CharacterControllerInterface () {};
	
	virtual void reset () = 0;
	virtual void warp (const btVector3& origin) = 0;

	virtual void preStep ( btCollisionWorld* collisionWorld) = 0;
	virtual void playerStep (btCollisionWorld* collisionWorld, btScalar dt,
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
