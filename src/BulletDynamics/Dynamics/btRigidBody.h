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

#ifndef BT_RIGIDBODY_H
#define BT_RIGIDBODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class btCollisionShape;
class btMotionState;
class btTypedConstraint;


extern btScalar gDeactivationTime;
extern bool gDisableDeactivation;

#ifdef BT_USE_DOUBLE_PRECISION
#define btRigidBodyData	btRigidBodyDoubleData
#define btRigidBodyDataName	"btRigidBodyDoubleData"
#else
#define btRigidBodyData	btRigidBodyFloatData
#define btRigidBodyDataName	"btRigidBodyFloatData"
#endif //BT_USE_DOUBLE_PRECISION


/// @brief Specifies flags for a btRigidBody.
enum	btRigidBodyFlags
{
    /// @brief Disables the world gravity for the rigid body.
	BT_DISABLE_WORLD_GRAVITY = 1,

	/// @brief The BT_ENABLE_GYROPSCOPIC_FORCE can easily introduce instability, so generally it is best to not enable it.
	///        If really needed, run at a high frequency like 1000 Hertz.
	/// @note See Demos/GyroscopicDemo for an example use.
	BT_ENABLE_GYROPSCOPIC_FORCE = 2
};


///The btRigidBody is the main class for rigid body objects. It is
///


/// @brief The main class for rigid body objects.
/// btRigidBody is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape. It is recommended for performance and memory
/// use to share btCollisionShape objects whenever possible.
///
/// There are 3 types of rigid bodies:
///     -# Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
///     -# Fixed objects with zero mass. They are not moving (basically collision objects)
///     -# Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
/// Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
/// Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)
class btRigidBody  : public btCollisionObject
{

	btMatrix3x3	m_invInertiaTensorWorld;
	btVector3		m_linearVelocity;
	btVector3		m_angularVelocity;
	btScalar		m_inverseMass;
	btVector3		m_linearFactor;

	btVector3		m_gravity;
	btVector3		m_gravity_acceleration;
	btVector3		m_invInertiaLocal;
	btVector3		m_totalForce;
	btVector3		m_totalTorque;

	btScalar		m_linearDamping;
	btScalar		m_angularDamping;

	bool			m_additionalDamping;
	btScalar		m_additionalDampingFactor;
	btScalar		m_additionalLinearDampingThresholdSqr;
	btScalar		m_additionalAngularDampingThresholdSqr;
	btScalar		m_additionalAngularDampingFactor;


	btScalar		m_linearSleepingThreshold;
	btScalar		m_angularSleepingThreshold;

	//m_optionalMotionState allows to automatic synchronize the world transform for active objects
	btMotionState*	m_optionalMotionState;

	//keep track of typed constraints referencing this rigid body, to disable collision between linked bodies
	btAlignedObjectArray<btTypedConstraint*> m_constraintRefs;

	int				m_rigidbodyFlags;

	int				m_debugBodyId;


protected:

    /// @brief The delta linear velocity.
	ATTRIBUTE_ALIGNED16(btVector3		m_deltaLinearVelocity);
	/// @brief The delta angular velocity.
	btVector3		m_deltaAngularVelocity;
	/// @brief The angular factor.
	btVector3		m_angularFactor;
	/// @brief The inverse mass (1/mass).
	btVector3		m_invMass;
	/// @brief The push velocity.
	btVector3		m_pushVelocity;
	/// @brief The turn velocity.
	btVector3		m_turnVelocity;


public:

	/// @brief Provides information to create a rigid body.
	/// Setting mass to zero creates a fixed (non-dynamic) rigid body.
	/// For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
	/// You can use the motion state to synchronize the world transform between physics and graphics objects.
	/// And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state.
	/// m_startWorldTransform is only used when you don't provide a motion state.
	struct	btRigidBodyConstructionInfo
	{
        /// @brief The mass of the rigid body.
		btScalar			m_mass;

		/// @brief The motion state of the rigid body.
		btMotionState*		m_motionState;

		/// @brief The world transform of the rigid body.
		/// If m_motionState is null, then this value is used, otherwise, this value is ignored and the motion state is used in its place.
		btTransform	m_startWorldTransform;

		/// @brief The shape of the rigid body.
		btCollisionShape*	m_collisionShape;
		/// @brief The local inertia of the rigid body.
		btVector3			m_localInertia;
		/// @brief The linear damping of the rigid body.
		btScalar			m_linearDamping;
		/// @brief The angular dampting of the rigid body.
		btScalar			m_angularDamping;

		/// @brief The friction acting on the rigid body.
		btScalar			m_friction;

		/// @brief The rolling friction of the rigid body.
		/// This is the friction encountered when a rigid body is rolling along a surface.
		/// Rolling friction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
		/// @note See Bullet/Demos/RollingFrictionDemo for usage.
		btScalar			m_rollingFriction;

		/// @brief The restitution value of the rigid body.
		btScalar			m_restitution;

		/// @brief The linear velocity sleeping threshold of the rigid body.
		btScalar			m_linearSleepingThreshold;
		/// @brief The angular velocity sleeping threshold of the rigid body.
		btScalar			m_angularSleepingThreshold;

		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		bool				m_additionalDamping;
		btScalar			m_additionalDampingFactor;
		btScalar			m_additionalLinearDampingThresholdSqr;
		btScalar			m_additionalAngularDampingThresholdSqr;
		btScalar			m_additionalAngularDampingFactor;

		/// @brief Creates a new btRigidBodyConstructionInfo object.
		/// @param mass The mass.
		/// @param motionState The motion state.
		/// @param collisionShape The shape of the rigid body.
		/// @param localInertia The inertia of the rigid body.
		btRigidBodyConstructionInfo(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0)):
		m_mass(mass),
			m_motionState(motionState),
			m_collisionShape(collisionShape),
			m_localInertia(localInertia),
			m_linearDamping(btScalar(0.)),
			m_angularDamping(btScalar(0.)),
			m_friction(btScalar(0.5)),
			m_rollingFriction(btScalar(0)),
			m_restitution(btScalar(0.)),
			m_linearSleepingThreshold(btScalar(0.8)),
			m_angularSleepingThreshold(btScalar(1.f)),
			m_additionalDamping(false),
			m_additionalDampingFactor(btScalar(0.005)),
			m_additionalLinearDampingThresholdSqr(btScalar(0.01)),
			m_additionalAngularDampingThresholdSqr(btScalar(0.01)),
			m_additionalAngularDampingFactor(btScalar(0.01))
		{
			m_startWorldTransform.setIdentity();
		}
	};

	/// @brief Creates a rigid body from a btRigidBodyConstructionInfo object.
	btRigidBody(	const btRigidBodyConstructionInfo& constructionInfo);

	/// @brief Creates a rigid body.
	/// @note To specify friction, restitution (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo).
	btRigidBody(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia=btVector3(0,0,0));

	/// @brief Destroys the rigid body.
	virtual ~btRigidBody()
	{
        // No constraints should point to this rigidbody
		// Remove constraints from the dynamics world before you delete the related rigidbodies.
        btAssert(m_constraintRefs.size()==0);
    }

protected:

	/// @brief Sets up the rigid body from a btRigidBodyConstructionInfo object.
	/// @note This is intended to only be used internally by the constructor
	void	setupRigidBody(const btRigidBodyConstructionInfo& constructionInfo);

public:

    /// @brief Sets the center of mass transform.
	void			proceedToTransform(const btTransform& newTrans);


	/// @brief Upcasts a btCollisionObject pointer to a btRigidBody.
	/// @param colObj A pointer to a btRigidBody.
	/// @return The casted rigid body pointer, or null if colObj is not a btRigidBody.
	static const btRigidBody*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()&btCollisionObject::CO_RIGID_BODY)
			return (const btRigidBody*)colObj;
		return 0;
	}

	/// @brief Upcasts a btCollisionObject pointer to a btRigidBody.
	/// @param colObj A pointer to a btRigidBody.
	/// @return The casted rigid body pointer, or null if colObj is not a btRigidBody.
	static btRigidBody*	upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()&btCollisionObject::CO_RIGID_BODY)
			return (btRigidBody*)colObj;
		return 0;
	}

	/// @brief Predicts the integrated transform of the rigid body at some point in the future.
	/// @param step The future elapsed time at which the transform will be predicted.
	/// @param predictedTransform The transform object which will receive the result.
	void			predictIntegratedTransform(btScalar step, btTransform& predictedTransform) ;

	/// @brief Saves the kinematic state.
	void			saveKinematicState(btScalar step);

	/// @brief Applies the force of gravity to the rigid body.
	void			applyGravity();

	/// @brief Sets the acceleration due to gravity that acts on the rigid body.
	void			setGravity(const btVector3& acceleration);

	/// @brief Gets the acceleration due to gravity which is acting on the rigid body.
	const btVector3&	getGravity() const
	{
		return m_gravity_acceleration;
	}

	/// @brief Sets the damping on the rigid body.
	/// @param lin_damping The linear damping.
	/// @param ang_damping The angular damping.
	void			setDamping(btScalar lin_damping, btScalar ang_damping);

	/// @brief Gets the linear damping.
	btScalar getLinearDamping() const
	{
		return m_linearDamping;
	}

	/// @brief Gets the angular damping.
	btScalar getAngularDamping() const
	{
		return m_angularDamping;
	}

	/// @brief Gets the linear velocity sleeping threshold.
	btScalar getLinearSleepingThreshold() const
	{
		return m_linearSleepingThreshold;
	}

	/// @brief Gets the angular velocity sleeping threshold.
	btScalar getAngularSleepingThreshold() const
	{
		return m_angularSleepingThreshold;
	}

	/// @brief Applies linear and angular damping to the velocity.
	void			applyDamping(btScalar timeStep);

	/// @brief Gets the collision shape of the rigid body.
	SIMD_FORCE_INLINE const btCollisionShape*	getCollisionShape() const {
		return m_collisionShape;
	}

	/// @brief Gets the collision shape of the rigid body.
	SIMD_FORCE_INLINE btCollisionShape*	getCollisionShape() {
			return m_collisionShape;
	}

	/// @brief Sets the mass and inertia of the rigid body.
	/// @param mass The mass.
	/// @param inertia The inertia.
	void			setMassProps(btScalar mass, const btVector3& inertia);

	/// @brief Gets the linear factor.
	const btVector3& getLinearFactor() const
	{
		return m_linearFactor;
	}

	/// @brief Sets the linear factor.
	void setLinearFactor(const btVector3& linearFactor)
	{
		m_linearFactor = linearFactor;
		m_invMass = m_linearFactor*m_inverseMass;
	}

	/// @brief Gets the inverse mass (1/mass).
	btScalar		getInvMass() const { return m_inverseMass; }

	/// @brief Gets the inverse of the inertia tensor.
	const btMatrix3x3& getInvInertiaTensorWorld() const {
		return m_invInertiaTensorWorld;
	}

	/// @brief Integrates the linear and angular velocities of the rigid body.
	/// @param step The time step.
	void			integrateVelocities(btScalar step);

	/// @brief Sets the rigid body transfom, relative to the center of mass.
	void			setCenterOfMassTransform(const btTransform& xform);

	/// @brief Applies a force, acting on the center of mass, to the rigid body.
	void			applyCentralForce(const btVector3& force)
	{
		m_totalForce += force*m_linearFactor;
	}

	/// @brief Gets the total net force acting on the rigid body.
	const btVector3& getTotalForce() const
	{
		return m_totalForce;
	};

	/// @brief Gets the total net torque acting on the rigid body.
	const btVector3& getTotalTorque() const
	{
		return m_totalTorque;
	};

	const btVector3& getInvInertiaDiagLocal() const
	{
		return m_invInertiaLocal;
	};

	void	setInvInertiaDiagLocal(const btVector3& diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	/// @brief Sets the velocity sleeping thresholds.
	/// @param linear The linear velocity sleeping threshold.
	/// @param angular The angular velocity sleeping threshold.
	void	setSleepingThresholds(btScalar linear,btScalar angular)
	{
		m_linearSleepingThreshold = linear;
		m_angularSleepingThreshold = angular;
	}

	/// @brief Applies a torque to the rigid body.
	/// @param torque The torque.
	void	applyTorque(const btVector3& torque)
	{
		m_totalTorque += torque*m_angularFactor;
	}

	/// @brief Applies a force to the rigid body at an arbitrary point.
	/// @param force The force to apply.
	/// @param rel_pos The point (on or inside) the rigid body where the force is to be applied.
	/// @note If you simply want to apply a force to the entire object, use applyCentralForce().
	void	applyForce(const btVector3& force, const btVector3& rel_pos)
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force*m_linearFactor));
	}

	/// @brief Applies an impulse to the rigid body at the center of mass.
	/// @param impulse The impulse.
	void applyCentralImpulse(const btVector3& impulse)
	{
		m_linearVelocity += impulse *m_linearFactor * m_inverseMass;
	}

	/// @brief Applies a torque impulse to the rigid body.
  	void applyTorqueImpulse(const btVector3& torque)
	{
			m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
	}

	/// @brief Applies an impulse to the rigid body at an arbitrary point.
	/// @param impulse The impulse.
	/// @param rel_pos The point (on or inside) the rigid body where the impulse is to be applied.
	void applyImpulse(const btVector3& impulse, const btVector3& rel_pos)
	{
		if (m_inverseMass != btScalar(0.))
		{
			applyCentralImpulse(impulse);
			if (m_angularFactor)
			{
				applyTorqueImpulse(rel_pos.cross(impulse*m_linearFactor));
			}
		}
	}

	/// @brief Clears every single force (including gravity) that is acting on the rigid body.
	void clearForces()
	{
		m_totalForce.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
		m_totalTorque.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	}

	/// @brief Updates the inertia tensor.
	void updateInertiaTensor();

	/// @brief Gets the position of the center of mass.
	const btVector3&     getCenterOfMassPosition() const {
		return m_worldTransform.getOrigin();
	}

	/// @brief Gets the orientation of the rigid body.
	btQuaternion getOrientation() const;

	/// @brief Gets the transform of the center of mass.
	const btTransform&  getCenterOfMassTransform() const {
		return m_worldTransform;
	}

	/// @brief Gets the linear velocity.
	const btVector3&   getLinearVelocity() const {
		return m_linearVelocity;
	}

	/// @brief Gets the angular velocity.
	const btVector3&    getAngularVelocity() const {
		return m_angularVelocity;
	}


	/// @brief Sets the linear velocity.
	/// @param lin_vel The linear velocity.
	inline void setLinearVelocity(const btVector3& lin_vel)
	{
		m_updateRevision++;
		m_linearVelocity = lin_vel;
	}

	/// @brief Sets the angular velocity.
	/// @param ang_vel The angular velocity.
	inline void setAngularVelocity(const btVector3& ang_vel)
	{
		m_updateRevision++;
		m_angularVelocity = ang_vel;
	}

	/// @brief Gets the velocity of an arbitrary point on or inside the rigid body.
	/// @param rel_pos The point.
	btVector3 getVelocityInLocalPoint(const btVector3& rel_pos) const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	/// @brief Translates the rigid body.
	/// @param v The vector to translate by.
	void translate(const btVector3& v)
	{
		m_worldTransform.getOrigin() += v;
	}


	/// @brief Gets the axis aligned bounding box of the rigid body.
	void	getAabb(btVector3& aabbMin, btVector3& aabbMax) const;




	SIMD_FORCE_INLINE btScalar computeImpulseDenominator(const btVector3& pos, const btVector3& normal) const
	{
		btVector3 r0 = pos - getCenterOfMassPosition();

		btVector3 c0 = (r0).cross(normal);

		btVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

		return m_inverseMass + normal.dot(vec);

	}

	SIMD_FORCE_INLINE btScalar computeAngularImpulseDenominator(const btVector3& axis) const
	{
		btVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}

	SIMD_FORCE_INLINE void	updateDeactivation(btScalar timeStep)
	{
		if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION))
			return;

		if ((getLinearVelocity().length2() < m_linearSleepingThreshold*m_linearSleepingThreshold) &&
			(getAngularVelocity().length2() < m_angularSleepingThreshold*m_angularSleepingThreshold))
		{
			m_deactivationTime += timeStep;
		} else
		{
			m_deactivationTime=btScalar(0.);
			setActivationState(0);
		}

	}

	SIMD_FORCE_INLINE bool	wantsSleeping()
	{

		if (getActivationState() == DISABLE_DEACTIVATION)
			return false;

		//disable deactivation
		if (gDisableDeactivation || (gDeactivationTime == btScalar(0.)))
			return false;

		if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION))
			return true;

		if (m_deactivationTime> gDeactivationTime)
		{
			return true;
		}
		return false;
	}



	const btBroadphaseProxy*	getBroadphaseProxy() const
	{
		return m_broadphaseHandle;
	}
	btBroadphaseProxy*	getBroadphaseProxy()
	{
		return m_broadphaseHandle;
	}
	void	setNewBroadphaseProxy(btBroadphaseProxy* broadphaseProxy)
	{
		m_broadphaseHandle = broadphaseProxy;
	}

	/// @brief Gets the motion state of the rigid body.
	btMotionState*	getMotionState()
	{
		return m_optionalMotionState;
	}

	/// @brief Gets the motion state of the rigid body.
	const btMotionState*	getMotionState() const
	{
		return m_optionalMotionState;
	}

	/// @brief Sets the motion state of the rigid body.
	void	setMotionState(btMotionState* motionState)
	{
		m_optionalMotionState = motionState;
		if (m_optionalMotionState)
			motionState->getWorldTransform(m_worldTransform);
	}

	//for experimental overriding of friction/contact solver func
	int	m_contactSolverType;
	int	m_frictionSolverType;

	/// @brief Sets the angular factor.
	/// @param angFac The angular factor.
	void	setAngularFactor(const btVector3& angFac)
	{
		m_updateRevision++;
		m_angularFactor = angFac;
	}

	/// @brief Sets the angular factor.
	/// @param angFac The angular factor.
	/// @note This is equivalent to calling setAngularFactor(btVector3(angFac,angFac,angFac)).
	void	setAngularFactor(btScalar angFac)
	{
		m_updateRevision++;
		m_angularFactor.setValue(angFac,angFac,angFac);
	}

	/// @brief Gets the angular factor.
	const btVector3&	getAngularFactor() const
	{
		return m_angularFactor;
	}

	/// @brief Checks if the rigid body is inside a world.
	bool	isInWorld() const
	{
		return (getBroadphaseProxy() != 0);
	}

	void addConstraintRef(btTypedConstraint* c);
	void removeConstraintRef(btTypedConstraint* c);

	btTypedConstraint* getConstraintRef(int index)
	{
		return m_constraintRefs[index];
	}

	int getNumConstraintRefs() const
	{
		return m_constraintRefs.size();
	}

	/// @brief Sets the rigid body flags.
	/// @param flags The rigid body flags - a combination of btRigidBodyFlags.
	void	setFlags(int flags)
	{
		m_rigidbodyFlags = flags;
	}

	/// @brief Gets the rigid body flags.
	/// @return The rigid body flags - a combination of btRigidBodyFlags.
	int getFlags() const
	{
		return m_rigidbodyFlags;
	}

	btVector3 computeGyroscopicForce(btScalar maxGyroscopicForce) const;

	///////////////////////////////////////////////

	virtual	int	calculateSerializeBufferSize()	const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer,  class btSerializer* serializer) const;

	virtual void serializeSingleObject(class btSerializer* serializer) const;

};

/// @todo add m_optionalMotionState and m_constraintRefs to btRigidBodyData.
/// @note Do not change this serialization structure, it requires an updated sBulletDNAstr/sBulletDNAstr64.
struct	btRigidBodyFloatData
{
	btCollisionObjectFloatData	m_collisionObjectData;
	btMatrix3x3FloatData		m_invInertiaTensorWorld;
	btVector3FloatData		m_linearVelocity;
	btVector3FloatData		m_angularVelocity;
	btVector3FloatData		m_angularFactor;
	btVector3FloatData		m_linearFactor;
	btVector3FloatData		m_gravity;
	btVector3FloatData		m_gravity_acceleration;
	btVector3FloatData		m_invInertiaLocal;
	btVector3FloatData		m_totalForce;
	btVector3FloatData		m_totalTorque;
	float					m_inverseMass;
	float					m_linearDamping;
	float					m_angularDamping;
	float					m_additionalDampingFactor;
	float					m_additionalLinearDampingThresholdSqr;
	float					m_additionalAngularDampingThresholdSqr;
	float					m_additionalAngularDampingFactor;
	float					m_linearSleepingThreshold;
	float					m_angularSleepingThreshold;
	int						m_additionalDamping;
};

/// @note Do not change this serialization structure, it requires an updated sBulletDNAstr/sBulletDNAstr64.
struct	btRigidBodyDoubleData
{
	btCollisionObjectDoubleData	m_collisionObjectData;
	btMatrix3x3DoubleData		m_invInertiaTensorWorld;
	btVector3DoubleData		m_linearVelocity;
	btVector3DoubleData		m_angularVelocity;
	btVector3DoubleData		m_angularFactor;
	btVector3DoubleData		m_linearFactor;
	btVector3DoubleData		m_gravity;
	btVector3DoubleData		m_gravity_acceleration;
	btVector3DoubleData		m_invInertiaLocal;
	btVector3DoubleData		m_totalForce;
	btVector3DoubleData		m_totalTorque;
	double					m_inverseMass;
	double					m_linearDamping;
	double					m_angularDamping;
	double					m_additionalDampingFactor;
	double					m_additionalLinearDampingThresholdSqr;
	double					m_additionalAngularDampingThresholdSqr;
	double					m_additionalAngularDampingFactor;
	double					m_linearSleepingThreshold;
	double					m_angularSleepingThreshold;
	int						m_additionalDamping;
	char	m_padding[4];
};



#endif //BT_RIGIDBODY_H

