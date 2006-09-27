/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>
#include <LinearMath/btPoint3.h>
#include <LinearMath/btTransform.h>
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class btCollisionShape;
struct btMassProps;
typedef btScalar dMatrix3[4*3];

extern float gLinearAirDamping;
extern bool gUseEpa;

extern float gDeactivationTime;
extern bool gDisableDeactivation;
extern float gLinearSleepingTreshold;
extern float gAngularSleepingTreshold;


/// btRigidBody class for btRigidBody Dynamics
/// 
class btRigidBody  : public btCollisionObject
{

	btMatrix3x3	m_invInertiaTensorWorld;
	btVector3		m_linearVelocity;
	btVector3		m_angularVelocity;
	btScalar		m_inverseMass;

	btVector3		m_gravity;	
	btVector3		m_invInertiaLocal;
	btVector3		m_totalForce;
	btVector3		m_totalTorque;
	
	btScalar		m_linearDamping;
	btScalar		m_angularDamping;
	
	btScalar		m_kinematicTimeStep;

	btBroadphaseProxy*	m_broadphaseProxy;

public:

	btRigidBody(const btMassProps& massProps,btScalar linearDamping=0.f,btScalar angularDamping=0.f,btScalar friction=0.5f,btScalar restitution=0.f);

	void			proceedToTransform(const btTransform& newTrans); 
	
	
	/// continuous collision detection needs prediction
	void			predictIntegratedTransform(btScalar step, btTransform& predictedTransform) const;
	
	void			saveKinematicState(btScalar step);
	

	void			applyForces(btScalar step);
	
	void			setGravity(const btVector3& acceleration);  
	
	void			setDamping(btScalar lin_damping, btScalar ang_damping);
	
	inline const btCollisionShape*	GetCollisionShape() const {
		return m_collisionShape;
	}

	inline btCollisionShape*	GetCollisionShape() {
			return m_collisionShape;
	}
	
	void			setMassProps(btScalar mass, const btVector3& inertia);
	
	btScalar		getInvMass() const { return m_inverseMass; }
	const btMatrix3x3& getInvInertiaTensorWorld() const { 
		return m_invInertiaTensorWorld; 
	}
		
	void			integrateVelocities(btScalar step);

	void			setCenterOfMassTransform(const btTransform& xform);

	void			applyCentralForce(const btVector3& force)
	{
		m_totalForce += force;
	}
    
	const btVector3& getInvInertiaDiagLocal()
	{
		return m_invInertiaLocal;
	};

	void	setInvInertiaDiagLocal(const btVector3& diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	void	applyTorque(const btVector3& torque)
	{
		m_totalTorque += torque;
	}
	
	void	applyForce(const btVector3& force, const btVector3& rel_pos) 
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force));
	}
	
	void applyCentralImpulse(const btVector3& impulse)
	{
		m_linearVelocity += impulse * m_inverseMass;
	}
	
  	void applyTorqueImpulse(const btVector3& torque)
	{
		if (!IsStatic())
			m_angularVelocity += m_invInertiaTensorWorld * torque;

	}
	
	void applyImpulse(const btVector3& impulse, const btVector3& rel_pos) 
	{
		if (m_inverseMass != 0.f)
		{
			applyCentralImpulse(impulse);
			applyTorqueImpulse(rel_pos.cross(impulse));
		}
	}
	
	void clearForces() 
	{
		m_totalForce.setValue(0.0f, 0.0f, 0.0f);
		m_totalTorque.setValue(0.0f, 0.0f, 0.0f);
	}
	
	void updateInertiaTensor();    
	
	const btPoint3&     getCenterOfMassPosition() const { 
		return m_worldTransform.getOrigin(); 
	}
	btQuaternion getOrientation() const;
	
	const btTransform&  getCenterOfMassTransform() const { 
		return m_worldTransform; 
	}
	const btVector3&   getLinearVelocity() const { 
		return m_linearVelocity; 
	}
	const btVector3&    getAngularVelocity() const { 
		return m_angularVelocity; 
	}
	

	void setLinearVelocity(const btVector3& lin_vel);
	void setAngularVelocity(const btVector3& ang_vel) { 
		if (!IsStatic())
		{
			m_angularVelocity = ang_vel; 
		}
	}

	btVector3 getVelocityInLocalPoint(const btVector3& rel_pos) const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	void translate(const btVector3& v) 
	{
		m_worldTransform.getOrigin() += v; 
	}

	
	void	getAabb(btVector3& aabbMin,btVector3& aabbMax) const;




	
	inline float ComputeImpulseDenominator(const btPoint3& pos, const btVector3& normal) const
	{
		btVector3 r0 = pos - getCenterOfMassPosition();

		btVector3 c0 = (r0).cross(normal);

		btVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

		return m_inverseMass + normal.dot(vec);

	}

	inline float ComputeAngularImpulseDenominator(const btVector3& axis) const
	{
		btVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}

	inline void	updateDeactivation(float timeStep)
	{
		if ( (GetActivationState() == ISLAND_SLEEPING) || (GetActivationState() == DISABLE_DEACTIVATION))
			return;

		if ((getLinearVelocity().length2() < gLinearSleepingTreshold*gLinearSleepingTreshold) &&
			(getAngularVelocity().length2() < gAngularSleepingTreshold*gAngularSleepingTreshold))
		{
			m_deactivationTime += timeStep;
		} else
		{
			m_deactivationTime=0.f;
			SetActivationState(0);
		}

	}

	inline bool	wantsSleeping()
	{

		if (GetActivationState() == DISABLE_DEACTIVATION)
			return false;

		//disable deactivation
		if (gDisableDeactivation || (gDeactivationTime == 0.f))
			return false;

		if ( (GetActivationState() == ISLAND_SLEEPING) || (GetActivationState() == WANTS_DEACTIVATION))
			return true;

		if (m_deactivationTime> gDeactivationTime)
		{
			return true;
		}
		return false;
	}


	
	const btBroadphaseProxy*	GetBroadphaseProxy() const
	{
		return m_broadphaseProxy;
	}
	btBroadphaseProxy*	GetBroadphaseProxy() 
	{
		return m_broadphaseProxy;
	}
	void	SetBroadphaseProxy(btBroadphaseProxy* broadphaseProxy)
	{
		m_broadphaseProxy = broadphaseProxy;
	}
	
	//for experimental overriding of friction/contact solver func
	int	m_contactSolverType;
	int	m_frictionSolverType;

	

	int	m_debugBodyId;
};



#endif
