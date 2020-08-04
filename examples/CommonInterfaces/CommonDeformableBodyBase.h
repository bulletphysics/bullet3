
#ifndef COMMON_DEFORMABLE_BODY_SETUP_H
#define COMMON_DEFORMABLE_BODY_SETUP_H
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "btBulletDynamicsCommon.h"
#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include "CommonRenderInterface.h"
#include "CommonGraphicsAppInterface.h"
#include "CommonWindowInterface.h"
#include "CommonCameraInterface.h"
#include "CommonMultiBodyBase.h"
#include "BulletSoftBody/btSoftBody.h"

struct CommonDeformableBodyBase : public CommonMultiBodyBase
{
	btAlignedObjectArray<btDeformableLagrangianForce*> m_forces;
	btSoftBody* m_pickedSoftBody;
	btDeformableMousePickingForce* m_mouseForce;
	btScalar m_pickingForceElasticStiffness, m_pickingForceDampingStiffness, m_maxPickingForce;
	CommonDeformableBodyBase(GUIHelperInterface* helper)
	: CommonMultiBodyBase(helper),
	m_pickedSoftBody(0),
	m_mouseForce(0),
	m_pickingForceElasticStiffness(100),
	m_pickingForceDampingStiffness(0.0),
	m_maxPickingForce(0.3)
	{
	}

	virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
	{
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}

	virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
	{
		return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
	}
	
	struct ClosestRayResultCallbackWithInfo : public btCollisionWorld::ClosestRayResultCallback
	{
		ClosestRayResultCallbackWithInfo(const btVector3& rayFromWorld, const btVector3& rayToWorld)
		: ClosestRayResultCallback(rayFromWorld, rayToWorld)
		{
		}
		int m_faceId;
		
		virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
			
			m_closestHitFraction = rayResult.m_hitFraction;
			m_collisionObject = rayResult.m_collisionObject;
			if (rayResult.m_localShapeInfo)
			{
				m_faceId = rayResult.m_localShapeInfo->m_triangleIndex;
			}
			else
			{
				m_faceId = -1;
			}
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			}
			else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
			}
			m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
			return rayResult.m_hitFraction;
		}
	};
	
	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (getDeformableDynamicsWorld() == 0)
			return false;
		ClosestRayResultCallbackWithInfo rayCallback(rayFromWorld, rayToWorld);
		getDeformableDynamicsWorld()->rayTest(rayFromWorld, rayToWorld, rayCallback);
		if (rayCallback.hasHit())
		{
			btVector3 pickPos = rayCallback.m_hitPointWorld;
			btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
			btSoftBody* psb = (btSoftBody*)btSoftBody::upcast(rayCallback.m_collisionObject);
            m_oldPickingPos = rayToWorld;
            m_hitPos = pickPos;
            m_oldPickingDist = (pickPos - rayFromWorld).length();
			if (body)
			{
				if (!(body->isStaticObject() || body->isKinematicObject()))
				{
					m_pickedBody = body;
					m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
					btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
					btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
					m_dynamicsWorld->addConstraint(p2p, true);
					m_pickedConstraint = p2p;
					btScalar mousePickClamping = 30.f;
					p2p->m_setting.m_impulseClamp = mousePickClamping;
					//very weak constraint for picking
					p2p->m_setting.m_tau = 0.001f;
				}
			}
			else if (psb)
			{
				int face_id = rayCallback.m_faceId;
				if (face_id >= 0 && face_id < psb->m_faces.size())
				{
					m_pickedSoftBody = psb;
					psb->setActivationState(DISABLE_DEACTIVATION);
					const btSoftBody::Face& f = psb->m_faces[face_id];
					btDeformableMousePickingForce* mouse_force = new btDeformableMousePickingForce(m_pickingForceElasticStiffness, m_pickingForceDampingStiffness, f, m_hitPos, m_maxPickingForce);
					m_mouseForce = mouse_force;
					getDeformableDynamicsWorld()->addForce(psb, mouse_force);
				}
			}
			else
			{
				btMultiBodyLinkCollider* multiCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(rayCallback.m_collisionObject);
				if (multiCol && multiCol->m_multiBody)
				{
					m_prevCanSleep = multiCol->m_multiBody->getCanSleep();
					multiCol->m_multiBody->setCanSleep(false);

					btVector3 pivotInA = multiCol->m_multiBody->worldPosToLocal(multiCol->m_link, pickPos);

					btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody, multiCol->m_link, 0, pivotInA, pickPos);
					//if you add too much energy to the system, causing high angular velocities, simulation 'explodes'
					//see also http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=949
					//so we try to avoid it by clamping the maximum impulse (force) that the mouse pick can apply
					//it is not satisfying, hopefully we find a better solution (higher order integrator, using joint friction using a zero-velocity target motor with limited force etc?)
					btScalar scaling = 1;
					p2p->setMaxAppliedImpulse(2 * scaling);
					btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_dynamicsWorld;
					world->addMultiBodyConstraint(p2p);
					m_pickingMultiBodyPoint2Point = p2p;
				}
			}
		}
		return false;
	}

	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (m_pickedBody && m_pickedConstraint)
		{
			btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance
				btVector3 newPivotB;
				btVector3 dir = rayToWorld - rayFromWorld;
				dir.normalize();
				dir *= m_oldPickingDist;
				newPivotB = rayFromWorld + dir;
				pickCon->setPivotB(newPivotB);
				return true;
			}
		}
		if (m_pickingMultiBodyPoint2Point)
		{
			//keep it at the same picking distance
			btVector3 dir = rayToWorld - rayFromWorld;
			dir.normalize();
			dir *= m_oldPickingDist;
			btVector3 newPivotB = rayFromWorld + dir;
			m_pickingMultiBodyPoint2Point->setPivotInB(newPivotB);
		}
		if (m_pickedSoftBody && m_mouseForce)
		{
			btVector3 newPivot;
			btVector3 dir = rayToWorld - rayFromWorld;
			dir.normalize();
			dir *= m_oldPickingDist;
			newPivot = rayFromWorld + dir;
			m_mouseForce->setMousePos(newPivot);
		}
		return false;
	}

	virtual void removePickingConstraint()
	{
		if (m_pickedConstraint)
		{
			m_dynamicsWorld->removeConstraint(m_pickedConstraint);

			if (m_pickedBody)
			{
				m_pickedBody->forceActivationState(ACTIVE_TAG);
				m_pickedBody->activate(true);
			}
			delete m_pickedConstraint;
			m_pickedConstraint = 0;
			m_pickedBody = 0;
		}
		if (m_pickingMultiBodyPoint2Point)
		{
			m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(m_prevCanSleep);
			btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*)m_dynamicsWorld;
			world->removeMultiBodyConstraint(m_pickingMultiBodyPoint2Point);
			delete m_pickingMultiBodyPoint2Point;
			m_pickingMultiBodyPoint2Point = 0;
		}
		if (m_pickedSoftBody)
		{
			getDeformableDynamicsWorld()->removeForce(m_pickedSoftBody, m_mouseForce);
			delete m_mouseForce;
			m_mouseForce = 0;
			m_pickedSoftBody = 0;
		}
	}
};
#endif  //COMMON_MULTI_BODY_SETUP_H
