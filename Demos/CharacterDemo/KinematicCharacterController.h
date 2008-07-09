#ifndef KINEMATIC_CHARACTER_CONTROLLER_H
#define KINEMATIC_CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "CharacterControllerInterface.h"

class btCollisionShape;
class btRigidBody;
class btCollisionWorld;
class btCollisionDispatcher;


///KinematicCharacterController is a collision object with support for sliding motion in a world.
///It uses the convex sweep test to test for upcoming collisions. This is combined with discrete collision detection to recover from penetrations.
///Interaction between KinematicCharacterController and dynamic rigid bodies needs to be explicity implemented by the user.
class KinematicCharacterController : public CharacterControllerInterface
{
protected:
	btScalar m_halfHeight;
	btConvexShape* m_shape;
	btCollisionObject* m_collisionObject;
	btOverlappingPairCache* m_pairCache;
	btCollisionDispatcher*	m_dispatcher;

	btScalar m_fallSpeed;
	btScalar m_jumpSpeed;
	btScalar m_maxJumpHeight;

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

	btManifoldArray	m_manifoldArray;

	bool m_touchingContact;
	btVector3 m_touchingNormal;
	
	bool recoverFromPenetration (const btCollisionWorld* collisionWorld);
	void stepUp (const btCollisionWorld* collisionWorld);
	void updateTargetPositionBasedOnCollision (const btVector3& hit_normal, btScalar tangentMag = btScalar(0.0), btScalar normalMag = btScalar(1.0));
	void stepForwardAndStrafe (const btCollisionWorld* collisionWorld, const btVector3& walkMove);
	void stepDown (const btCollisionWorld* collisionWorld, btScalar dt);
public:
	KinematicCharacterController ();
	~KinematicCharacterController ();
	void setup (btScalar height = btScalar(1.75), btScalar width = btScalar(0.4), btScalar stepHeight = btScalar(0.35));
	void destroy ();

	btCollisionObject* getCollisionObject ();

	void reset ();
	void warp (const btVector3& origin);

	virtual void registerPairCacheAndDispatcher (btOverlappingPairCache* pairCache, btCollisionDispatcher* dispatcher);
	void preStep (const btCollisionWorld* collisionWorld);
	void playerStep (const btCollisionWorld* collisionWorld, btScalar dt,
					 int forward,
					 int backward,
					 int left,
					 int right,
					 int jump);

	void setFallSpeed (btScalar fallSpeed);
	void setJumpSpeed (btScalar jumpSpeed);
	void setMaxJumpHeight (btScalar maxJumpHeight);
	bool canJump () const;
	void jump ();

	bool onGround () const;
};

#endif // KINEMATIC_CHARACTER_CONTROLLER_H
