#ifndef KINEMATIC_CHARACTER_CONTROLLER_H
#define KINEMATIC_CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "CharacterControllerInterface.h"

class btCollisionShape;
class btRigidBody;
class btDynamicsWorld;

class KinematicCharacterController : public CharacterControllerInterface
{
protected:
	btScalar m_halfHeight;
	btConvexShape* m_shape;
	btRigidBody* m_rigidBody;
	btOverlappingPairCache* m_pairCache;

	btScalar m_turnAngle;
	btScalar m_walkVelocity;

	btScalar m_height;
	btScalar m_width;
	btScalar m_stepHeight;

	btVector3 m_upDirection;
	btVector3 m_forwardDirection;
	btVector3 m_strafeDirection;

	btVector3 m_currentPosition;
	btScalar  m_currentStepOffset;
	btVector3 m_targetPosition;
	
	void recoverFromPenetration (btDynamicsWorld* dynamicsWorld);
	void stepUp (btDynamicsWorld* dynamicsWorld);
	void updateTargetPositionBasedOnCollision (const btVector3& hit_normal, btScalar tangentMag = btScalar(1.0), btScalar normalMag = btScalar(0.0));
	void stepForwardAndStrafe (btDynamicsWorld* dynamicsWorld, const btVector3& walkMove);
	void stepDown (btDynamicsWorld* dynamicsWorld, btScalar dt);
public:
	KinematicCharacterController ();
	~KinematicCharacterController ();
	void setup (btDynamicsWorld* dynamicsWorld, btScalar height = btScalar(1.75), btScalar width = btScalar(0.4), btScalar stepHeight = btScalar(0.35));
	void destroy (btDynamicsWorld* dynamicsWorld);

	btRigidBody* getRigidBody ();

	void registerPairCache (btOverlappingPairCache* pairCache);
	void preStep (btDynamicsWorld* dynamicsWorld);
	void playerStep (btDynamicsWorld* dynamicsWorld, btScalar dt,
					 int forward,
					 int backward,
					 int left,
					 int right);
	bool canJump () const;
	void jump ();

	bool onGround () const;
};

#endif // KINEMATIC_CHARACTER_CONTROLLER_H
