
#include "GLDebugDrawer.h"

#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "KinematicCharacterController.h"

/* TODO:
 * Interact with dynamic objects
 * Ride kinematicly animated platforms properly
 * More realistic (or maybe just a config option) falling
 *   -> Should integrate falling velocity manually and use that in stepDown()
 * Support jumping
 * Support ducking
 */
class ClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
public:
	ClosestNotMeRayResultCallback (btCollisionObject* me) : btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me)
			return 1.0;

		return ClosestRayResultCallback::addSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	btCollisionObject* m_me;
};

class ClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
	ClosestNotMeConvexResultCallback (btCollisionObject* me) : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0;

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	btCollisionObject* m_me;
};

/*
 * Returns the reflection direction of a ray going 'direction' hitting a surface with normal 'normal'
 *
 * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
 */
btVector3 computeReflectionDirection (const btVector3& direction, const btVector3& normal)
{
	return direction - (btScalar(2.0) * direction.dot(normal)) * normal;
}

/*
 * Returns the portion of 'direction' that is parallel to 'normal'
 */
btVector3 parallelComponent (const btVector3& direction, const btVector3& normal)
{
	btScalar magnitude = direction.dot(normal);
	return normal * magnitude;
}

/*
 * Returns the portion of 'direction' that is perpindicular to 'normal'
 */
btVector3 perpindicularComponent (const btVector3& direction, const btVector3& normal)
{
	return direction - parallelComponent(direction, normal);
}

KinematicCharacterController::KinematicCharacterController ()
{
	m_turnAngle = btScalar(0.0);
	m_walkVelocity = btScalar(1.1) * 4.0; // 4 km/h -> 1.1 m/s
	m_shape = NULL;
	m_pairCache = NULL;
	m_collisionObject = NULL;
}

KinematicCharacterController::~KinematicCharacterController ()
{
}

void KinematicCharacterController::setup (btScalar height, btScalar width, btScalar stepHeight)
{
	btVector3 spherePositions[2];
	btScalar sphereRadii[2];
	
	sphereRadii[0] = width;
	sphereRadii[1] = width;
	spherePositions[0] = btVector3 (0.0, (height/btScalar(2.0) ), 0.0);
	spherePositions[1] = btVector3 (0.0, (-height/btScalar(2.0)), 0.0);

	m_halfHeight = height/btScalar(2.0);

	m_shape = new btMultiSphereShape (btVector3(width/btScalar(2.0), height/btScalar(2.0), width/btScalar(2.0)), &spherePositions[0], &sphereRadii[0], 2);
	m_stepHeight = stepHeight;
	m_height = height;
	m_width = width;
	btTransform startTransform;
	startTransform.setIdentity ();
	startTransform.setOrigin (btVector3(0.0, 4.0, 0.0));
	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	
	m_collisionObject = new btCollisionObject ();

	m_collisionObject->setWorldTransform(startTransform);
	m_collisionObject->setCollisionShape (m_shape);
	m_collisionObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
}

void KinematicCharacterController::destroy ()
{
	if (m_collisionObject)
	{
		delete m_collisionObject;
	}

	if (m_shape)
	{
		delete m_shape;
	}
}

btCollisionObject* KinematicCharacterController::getCollisionObject ()
{
	return m_collisionObject;
}

bool KinematicCharacterController::recoverFromPenetration (const btCollisionWorld* collisionWorld)
{

	if (m_pairCache == NULL)
		return false;
	
	bool penetration = false;

	
	m_dispatcher->dispatchAllCollisionPairs (m_pairCache, collisionWorld->getDispatchInfo(), m_dispatcher);

	m_currentPosition = m_collisionObject->getWorldTransform().getOrigin();
	
	btScalar maxPen = btScalar(0.0);
	for (int i = 0; i < m_pairCache->getNumOverlappingPairs(); i++)
	{
		m_manifoldArray.resize(0);

		btBroadphasePair* collisionPair = &m_pairCache->getOverlappingPairArray()[i];
		
		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

		
		for (int j=0;j<m_manifoldArray.size();j++)
		{
			btPersistentManifold* manifold = m_manifoldArray[j];
			btScalar directionSign = manifold->getBody0() == m_collisionObject ? btScalar(-1.0) : btScalar(1.0);
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint&pt = manifold->getContactPoint(p);

				if (pt.getDistance() < 0.0)
				{
					if (pt.getDistance() < maxPen)
					{
						maxPen = pt.getDistance();
						m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

					}
					m_currentPosition += pt.m_normalWorldOnB * directionSign * pt.getDistance() * btScalar(0.2);
					penetration = true;
				} else {
					//printf("touching %f\n", pt.getDistance());
				}
			}
			
			//manifold->clearManifold();
		}
	}
	btTransform newTrans = m_collisionObject->getWorldTransform();
	newTrans.setOrigin(m_currentPosition);
	m_collisionObject->setWorldTransform(newTrans);
//	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
	return penetration;
}

void KinematicCharacterController::stepUp (const btCollisionWorld* world)
{
	// phase 1: up
	btTransform start, end;
	m_targetPosition = m_currentPosition + btVector3 (btScalar(0.0), m_stepHeight, btScalar(0.0));

	start.setIdentity ();
	end.setIdentity ();

	/* FIXME: Handle penetration properly */
	start.setOrigin (m_currentPosition + btVector3(btScalar(0.0), btScalar(0.1), btScalar(0.0)));
	end.setOrigin (m_targetPosition);

	ClosestNotMeConvexResultCallback callback (m_collisionObject);
	callback.m_collisionFilterGroup = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterGroup;
	callback.m_collisionFilterMask = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask;
	
	world->convexSweepTest (m_shape, start, end, callback);

	if (callback.hasHit())
	{
		// we moved up only a fraction of the step height
		m_currentStepOffset = m_stepHeight * callback.m_closestHitFraction;
		m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
	} else {
		m_currentStepOffset = m_stepHeight;
		m_currentPosition = m_targetPosition;
	}
}

void KinematicCharacterController::updateTargetPositionBasedOnCollision (const btVector3& hitNormal, btScalar tangentMag, btScalar normalMag)
{
	btVector3 movementDirection = m_targetPosition - m_currentPosition;
	btScalar movementLength = movementDirection.length();
	if (movementLength>SIMD_EPSILON)
	{
		movementDirection.normalize();

		btVector3 reflectDir = computeReflectionDirection (movementDirection, hitNormal);
		reflectDir.normalize();

		btVector3 parallelDir, perpindicularDir;

		parallelDir = parallelComponent (reflectDir, hitNormal);
		perpindicularDir = perpindicularComponent (reflectDir, hitNormal);

		m_targetPosition = m_currentPosition;
		if (0)//tangentMag != 0.0)
		{
			btVector3 parComponent = parallelDir * btScalar (tangentMag*movementLength);
//			printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
			m_targetPosition +=  parComponent;
		}

		if (normalMag != 0.0)
		{
			btVector3 perpComponent = perpindicularDir * btScalar (normalMag*movementLength);
//			printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
			m_targetPosition += perpComponent;
		}
	} else
	{
//		printf("movementLength don't normalize a zero vector\n");
	}
}

void KinematicCharacterController::stepForwardAndStrafe (const btCollisionWorld* collisionWorld, const btVector3& walkMove)
{

	btVector3 originalDir = walkMove.normalized();
	if (walkMove.length() < SIMD_EPSILON)
	{
		originalDir.setValue(0.f,0.f,0.f);
	}
//	printf("originalDir=%f,%f,%f\n",originalDir[0],originalDir[1],originalDir[2]);
	// phase 2: forward and strafe
	btTransform start, end;
	m_targetPosition = m_currentPosition + walkMove;
	start.setIdentity ();
	end.setIdentity ();
	
	btScalar fraction = 1.0;
	btScalar distance2 = (m_currentPosition-m_targetPosition).length2();
//	printf("distance2=%f\n",distance2);

	if (m_touchingContact)
	{
		if (originalDir.dot(m_touchingNormal) > btScalar(0.0))
			updateTargetPositionBasedOnCollision (m_touchingNormal);
	}

	int maxIter = 10;

	while (fraction > btScalar(0.01) && maxIter-- > 0)
	{
		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);

		ClosestNotMeConvexResultCallback callback (m_collisionObject);
		callback.m_collisionFilterGroup = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterGroup;
		callback.m_collisionFilterMask = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask;


		//btScalar margin = m_shape->getMargin();
		//m_shape->setMargin(margin - 0.06f);
		collisionWorld->convexSweepTest (m_shape, start, end, callback);
		//m_shape->setMargin(margin);

		
		fraction -= callback.m_closestHitFraction;

		if (callback.hasHit())
		{	
			// we moved only a fraction
			btScalar hitDistance = (callback.m_hitPointWorld - m_currentPosition).length();
			if (hitDistance<0.f)
			{
//				printf("neg dist?\n");
			}

			/* If the distance is farther than the collision margin, move */
			if (hitDistance > 0.05)
			{
//				printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
				m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
			}

			updateTargetPositionBasedOnCollision (callback.m_hitNormalWorld);
			btVector3 currentDir = m_targetPosition - m_currentPosition;
			distance2 = currentDir.length2();
			if (distance2 > SIMD_EPSILON)
			{
				currentDir.normalize();
				/* See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners." */
				if (currentDir.dot(originalDir) <= btScalar(0.0))
				{
					break;
				}
			} else
			{
//				printf("currentDir: don't normalize a zero vector\n");
				break;
			}
		} else {
			// we moved whole way
			m_currentPosition = m_targetPosition;
		}

	//	if (callback.m_closestHitFraction == 0.f)
	//		break;

	}
}

void KinematicCharacterController::stepDown (const btCollisionWorld* collisionWorld, btScalar dt)
{
	btTransform start, end;

	// phase 3: down
	btVector3 step_drop = btVector3(btScalar(0.0), m_currentStepOffset, btScalar(0.0));
	btVector3 gravity_drop = btVector3(btScalar(0.0), m_stepHeight, btScalar(0.0));
	m_targetPosition -= (step_drop + gravity_drop);

	start.setIdentity ();
	end.setIdentity ();

	start.setOrigin (m_currentPosition);
	end.setOrigin (m_targetPosition);

	ClosestNotMeConvexResultCallback callback (m_collisionObject);
	callback.m_collisionFilterGroup = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterGroup;
	callback.m_collisionFilterMask = getCollisionObject()->getBroadphaseHandle()->m_collisionFilterMask;
	
	collisionWorld->convexSweepTest (m_shape, start, end, callback);

	if (callback.hasHit())
	{
		// we dropped a fraction of the height -> hit floor
		m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
	} else {
		// we dropped the full height
		
		m_currentPosition = m_targetPosition;
	}
}

void KinematicCharacterController::reset ()
{
}

void KinematicCharacterController::warp (const btVector3& origin)
{
	btTransform xform;
	xform.setIdentity();
	xform.setOrigin (origin);
	m_collisionObject->setWorldTransform (xform);
}

void KinematicCharacterController::registerPairCacheAndDispatcher (btOverlappingPairCache* pairCache, btCollisionDispatcher* dispatcher)
{
	m_pairCache = pairCache;
	m_dispatcher = dispatcher;
}

void KinematicCharacterController::preStep (const btCollisionWorld* collisionWorld)
{
	
	int numPenetrationLoops = 0;
	m_touchingContact = false;
	while (recoverFromPenetration (collisionWorld))
	{
		numPenetrationLoops++;
		m_touchingContact = true;
		if (numPenetrationLoops > 4)
		{
//			printf("character could not recover from penetration = %d\n", numPenetrationLoops);
			break;
		}
	}
	btTransform xform;
	xform = m_collisionObject->getWorldTransform ();

	btVector3 forwardDir = xform.getBasis()[2];
//	printf("forwardDir=%f,%f,%f\n",forwardDir[0],forwardDir[1],forwardDir[2]);
	btVector3 upDir = xform.getBasis()[1];
	btVector3 strafeDir = xform.getBasis()[0];
	forwardDir.normalize ();
	upDir.normalize ();
	strafeDir.normalize ();
	
	m_upDirection = upDir;
	m_forwardDirection = forwardDir;
	m_strafeDirection = strafeDir;

	m_currentPosition = xform.getOrigin();
	m_targetPosition = m_currentPosition;
//	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);

	
}

void KinematicCharacterController::playerStep (const btCollisionWorld* collisionWorld,
					 btScalar dt,
					 int forward,
					 int backward,
					 int left,
					 int right,
					 int jump)
{
	btVector3 walkDirection = btVector3(0.0, 0.0, 0.0);
	btScalar walkSpeed = m_walkVelocity * dt;

	if (left)
		walkDirection += m_strafeDirection;

	if (right)
		walkDirection -= m_strafeDirection;

	if (forward)
		walkDirection += m_forwardDirection;

	if (backward)
		walkDirection -= m_forwardDirection;	

	btTransform xform;
	xform = m_collisionObject->getWorldTransform ();

//	printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
//	printf("walkSpeed=%f\n",walkSpeed);

	stepUp (collisionWorld);
	stepForwardAndStrafe (collisionWorld, walkDirection * walkSpeed);
	stepDown (collisionWorld, dt);

	xform.setOrigin (m_currentPosition);
	m_collisionObject->setWorldTransform (xform);
}

void KinematicCharacterController::setFallSpeed (btScalar fallSpeed)
{
	m_fallSpeed = fallSpeed;
}

void KinematicCharacterController::setJumpSpeed (btScalar jumpSpeed)
{
	m_jumpSpeed = jumpSpeed;
}

void KinematicCharacterController::setMaxJumpHeight (btScalar maxJumpHeight)
{
	m_maxJumpHeight = maxJumpHeight;
}

bool KinematicCharacterController::canJump () const
{
	return onGround();
}

void KinematicCharacterController::jump ()
{
	if (!canJump())
		return;

#if 0
	currently no jumping.
	btTransform xform;
	m_rigidBody->getMotionState()->getWorldTransform (xform);
	btVector3 up = xform.getBasis()[1];
	up.normalize ();
	btScalar magnitude = (btScalar(1.0)/m_rigidBody->getInvMass()) * btScalar(8.0);
	m_rigidBody->applyCentralImpulse (up * magnitude);
#endif
}

bool KinematicCharacterController::onGround () const
{
	return true;
}
