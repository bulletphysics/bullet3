#ifndef CHARACTER_CONTROLLER_H
#define CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "CharacterControllerInterface.h"

class btCollisionShape;
class btRigidBody;
class btDynamicsWorld;

class DynamicCharacterController : public CharacterControllerInterface
{
protected:
	btScalar m_halfHeight;
	btCollisionShape* m_shape;
	btRigidBody* m_rigidBody;

	btVector3 m_raySource[2];
	btVector3 m_rayTarget[2];
	btScalar m_rayLambda[2];
	btVector3 m_rayNormal[2];

	btScalar m_turnAngle;

	btScalar m_maxLinearVelocity;
	btScalar m_walkVelocity;
	btScalar m_turnVelocity;
public:
	DynamicCharacterController ();
	~DynamicCharacterController ();
	void setup (btDynamicsWorld* dynamicsWorld, btScalar height = 2.0, btScalar width = 0.25, btScalar stepHeight = 0.25);
	void destroy (btDynamicsWorld* dynamicsWorld);

	btRigidBody* getRigidBody ();

	void preStep (btDynamicsWorld* dynamicsWorld);
	void playerStep (btScalar dt,
					 int forward,
					 int backward,
					 int left,
					 int right);
	bool canJump () const;
	void jump ();

	bool onGround () const;
};

#endif
