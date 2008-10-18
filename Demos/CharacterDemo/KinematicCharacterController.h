#ifndef KINEMATIC_CHARACTER_CONTROLLER_H
#define KINEMATIC_CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "CharacterControllerInterface.h"

class btCollisionShape;
class btRigidBody;
class btCollisionWorld;
class btCollisionDispatcher;
class btPairCachingGhostObject;

///KinematicCharacterController is a collision object with support for sliding motion in a world.
///It uses the convex sweep test to test for upcoming collisions. This is combined with discrete collision detection to recover from penetrations.
///Interaction between KinematicCharacterController and dynamic rigid bodies needs to be explicity implemented by the user.
class KinematicCharacterController : public CharacterControllerInterface
{
protected:
	btScalar m_halfHeight;
	
	btPairCachingGhostObject* m_ghostObject;
	btConvexShape*	m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast
	
	btScalar m_fallSpeed;
	btScalar m_jumpSpeed;
	btScalar m_maxJumpHeight;

	btScalar m_turnAngle;
	btScalar m_walkVelocity;

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

	bool	m_useGhostObjectSweepTest;
	
	bool recoverFromPenetration (btCollisionWorld* collisionWorld);
	void stepUp (btCollisionWorld* collisionWorld);
	void updateTargetPositionBasedOnCollision (const btVector3& hit_normal, btScalar tangentMag = btScalar(0.0), btScalar normalMag = btScalar(1.0));
	void stepForwardAndStrafe (btCollisionWorld* collisionWorld, const btVector3& walkMove);
	void stepDown (btCollisionWorld* collisionWorld, btScalar dt);
public:
	KinematicCharacterController (btPairCachingGhostObject* ghostObject,btConvexShape* convexShape,btScalar stepHeight);
	~KinematicCharacterController ();
	

	btPairCachingGhostObject* getGhostObject();

	void reset ();
	void warp (const btVector3& origin);

	void preStep ( btCollisionWorld* collisionWorld);
	void playerStep (btCollisionWorld* collisionWorld, btScalar dt,
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
	void	setUseGhostSweepTest(bool useGhostObjectSweepTest)
	{
		m_useGhostObjectSweepTest = useGhostObjectSweepTest;
	}

	bool onGround () const;
};

#endif // KINEMATIC_CHARACTER_CONTROLLER_H
