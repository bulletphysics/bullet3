#include <stdio.h>

#include "GLDebugDrawer.h"

#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "KinematicCharacterController.h"

/* TODO:
 * Handle projecting/slide along surfaces
 * Deal with starting in penetration
 * Interact with dynamic objects
 * Ride kinematicly animated platforms properly
 * Step climbing
 */
class ClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
public:
	ClosestNotMeRayResultCallback (btRigidBody* me) : btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar AddSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me)
			return 1.0;

		return ClosestRayResultCallback::AddSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	btRigidBody* m_me;
};

class ClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
	ClosestNotMeConvexResultCallback (btRigidBody* me) : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btScalar AddSingleResult(btCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0;

		return ClosestConvexResultCallback::AddSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	btRigidBody* m_me;
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
	m_rigidBody = NULL;
}

KinematicCharacterController::~KinematicCharacterController ()
{
}

void KinematicCharacterController::setup (btDynamicsWorld* dynamicsWorld, btScalar height, btScalar width, btScalar stepHeight)
{
	btVector3 spherePositions[2];
	btScalar sphereRadii[2];
	
	sphereRadii[0] = width;
	sphereRadii[1] = width;
	spherePositions[0] = btVector3 (0.0, (height/btScalar(2.0) - width), 0.0);
	spherePositions[1] = btVector3 (0.0, (-height/btScalar(2.0) + width), 0.0);

	m_halfHeight = height/btScalar(2.0);

	m_shape = new btMultiSphereShape (btVector3(width/btScalar(2.0), height/btScalar(2.0), width/btScalar(2.0)), &spherePositions[0], &sphereRadii[0], 2);
	m_stepHeight = stepHeight;
	m_height = height;
	m_width = width;
	btTransform startTransform;
	startTransform.setIdentity ();
	startTransform.setOrigin (btVector3(0.0, 4.0, 0.0));
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(1.0, myMotionState, m_shape);
	m_rigidBody = new btRigidBody(cInfo);
	m_rigidBody->setCollisionFlags( m_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	m_rigidBody->setSleepingThresholds (0.0, 0.0);
	m_rigidBody->setAngularFactor (0.0);
	dynamicsWorld->addRigidBody (m_rigidBody);
}

void KinematicCharacterController::destroy (btDynamicsWorld* dynamicsWorld)
{
	if (m_rigidBody)
	{
		dynamicsWorld->removeRigidBody (m_rigidBody);
		delete m_rigidBody;
	}

	if (m_shape)
	{
		delete m_shape;
	}
}

btRigidBody* KinematicCharacterController::getRigidBody ()
{
	return m_rigidBody;
}

void KinematicCharacterController::recoverFromPenetration (btDynamicsWorld* dynamicsWorld)
{
	if (m_pairCache == NULL)
		return;
	
	printf("%d\n", m_pairCache->getNumOverlappingPairs());
	dynamicsWorld->getDispatcher()->dispatchAllCollisionPairs (m_pairCache, dynamicsWorld->getDispatchInfo(), dynamicsWorld->getDispatcher());

	btManifoldArray	manifoldArray;
	for (int i = 0; i < m_pairCache->getNumOverlappingPairs(); i++)
	{
		printf("%d\n",i);
		manifoldArray.clear();

		btBroadphasePair* collisionPair = &m_pairCache->getOverlappingPairArray()[i];
		
		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		for (int j=0;j<manifoldArray.size();j++)
		{
			btPersistentManifold* manifold = manifoldArray[j];
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint&pt = manifold->getContactPoint(p);

				if (pt.getDistance() < 0.0)
				{
					printf("penetration %f\n", pt.getDistance());
				} else {
					printf("touching %f\n", pt.getDistance());
				}
			}
		}
	}
}

void KinematicCharacterController::stepUp (btDynamicsWorld* dynamicsWorld)
{
	// phase 1: up
	btTransform start, end;
	m_targetPosition = m_currentPosition + btVector3 (btScalar(0.0), m_stepHeight, btScalar(0.0));

	start.setIdentity ();
	end.setIdentity ();

	/* FIXME: Handle penetration properly */
	start.setOrigin (m_currentPosition + btVector3(btScalar(0.0), btScalar(0.1), btScalar(0.0)));
	end.setOrigin (m_targetPosition);

	ClosestNotMeConvexResultCallback callback (m_rigidBody);
	
	dynamicsWorld->convexSweepTest (m_shape, start, end, callback);

	if (callback.HasHit())
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
	movementDirection.normalize();

	btVector3 reflectDir = computeReflectionDirection (movementDirection, hitNormal);
	reflectDir.normalize();

	btVector3 parallelDir, perpindicularDir;

	parallelDir = parallelComponent (reflectDir, hitNormal);
	perpindicularDir = perpindicularComponent (reflectDir, hitNormal);

	m_targetPosition = m_currentPosition;
	if (tangentMag != 0.0)
	{
		m_targetPosition += parallelDir * btScalar (tangentMag*movementLength);
	}

	if (normalMag != 0.0)
	{
		m_targetPosition += perpindicularDir * btScalar (normalMag*movementLength);
	}
}

void KinematicCharacterController::stepForwardAndStrafe (btDynamicsWorld* dynamicsWorld, const btVector3& walkMove)
{
	// phase 2: forward and strafe
	btTransform start, end;
	m_targetPosition = m_currentPosition + walkMove;
	start.setIdentity ();
	end.setIdentity ();
	
	btScalar fraction = 1.0;
	btScalar distance2 = (m_currentPosition-m_targetPosition).length2();

	while (fraction > btScalar(0.01))
	{
		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);

		ClosestNotMeConvexResultCallback callback (m_rigidBody);
		dynamicsWorld->convexSweepTest (m_shape, start, end, callback);

		fraction -= callback.m_closestHitFraction;

		if (callback.HasHit())
		{	
			// we moved only a fraction
			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
			updateTargetPositionBasedOnCollision (callback.m_hitNormalWorld);
			distance2 = (m_currentPosition-m_targetPosition).length2();
		} else {
			// we moved whole way
			m_currentPosition = m_targetPosition;
		}
	}
}

void KinematicCharacterController::stepDown (btDynamicsWorld* dynamicsWorld, btScalar dt)
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

	ClosestNotMeConvexResultCallback callback (m_rigidBody);
	
	dynamicsWorld->convexSweepTest (m_shape, start, end, callback);

	if (callback.HasHit())
	{
		// we dropped a fraction of the height -> hit floor
		m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
	} else {
		// we dropped the full height
		
		m_currentPosition = m_targetPosition;
	}
}

void KinematicCharacterController::registerPairCache (btOverlappingPairCache* pairCache)
{
	m_pairCache = pairCache;
}

void KinematicCharacterController::preStep (btDynamicsWorld* dynamicsWorld)
{
	btTransform xform;
	m_rigidBody->getMotionState()->getWorldTransform (xform);

	btVector3 forwardDir = xform.getBasis()[2];
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
	recoverFromPenetration (dynamicsWorld);
}

void KinematicCharacterController::playerStep (btDynamicsWorld* dynamicsWorld,
					 btScalar dt,
					 int forward,
					 int backward,
					 int left,
					 int right)
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
	m_rigidBody->getMotionState()->getWorldTransform (xform);

	stepUp (dynamicsWorld);
	stepForwardAndStrafe (dynamicsWorld, walkDirection * walkSpeed);
	stepDown (dynamicsWorld, dt);

	xform.setOrigin (m_currentPosition);
	m_rigidBody->getMotionState()->setWorldTransform (xform);
	m_rigidBody->setCenterOfMassTransform (xform);
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
