//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_PHYSICS_NX_RIGIDBODY
#define PX_PHYSICS_NX_RIGIDBODY
/** \addtogroup physics
@{
*/

#include "PxRigidActor.h"
#include "PxForceMode.h"

#if !PX_DOXYGEN
namespace physx
{
#endif


/**
\brief Collection of flags describing the behavior of a rigid body.

@see PxRigidBody.setRigidBodyFlag(), PxRigidBody.getRigidBodyFlags()
*/

struct PxRigidBodyFlag
{
	enum Enum
	{

		/**
		\brief Enables kinematic mode for the actor.

		Kinematic actors are special dynamic actors that are not 
		influenced by forces (such as gravity), and have no momentum. They are considered to have infinite
		mass and can be moved around the world using the setKinematicTarget() method. They will push 
		regular dynamic actors out of the way. Kinematics will not collide with static or other kinematic objects.

		Kinematic actors are great for moving platforms or characters, where direct motion control is desired.

		You can not connect Reduced joints to kinematic actors. Lagrange joints work ok if the platform
		is moving with a relatively low, uniform velocity.

		<b>Sleeping:</b>
		\li Setting this flag on a dynamic actor will put the actor to sleep and set the velocities to 0.
		\li If this flag gets cleared, the current sleep state of the actor will be kept.

		\note kinematic actors are incompatible with CCD so raising this flag will automatically clear eENABLE_CCD

		@see PxRigidDynamic.setKinematicTarget()
		*/
		eKINEMATIC									= (1<<0),		//!< Enable kinematic mode for the body.

		/**
		\brief Use the kinematic target transform for scene queries.

		If this flag is raised, then scene queries will treat the kinematic target transform as the current pose
		of the body (instead of using the actual pose). Without this flag, the kinematic target will only take 
		effect with respect to scene queries after a simulation step.

		@see PxRigidDynamic.setKinematicTarget()
		*/
		eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES		= (1<<1),

		/**
		\brief Enables swept integration for the actor.

		If this flag is raised and CCD is enabled on the scene, then this body will be simulated by the CCD system to ensure that collisions are not missed due to 
		high-speed motion. Note individual shape pairs still need to enable PxPairFlag::eDETECT_CCD_CONTACT in the collision filtering to enable the CCD to respond to 
		individual interactions. 

		\note kinematic actors are incompatible with CCD so this flag will be cleared automatically when raised on a kinematic actor

		*/
		eENABLE_CCD					= (1<<2),		//!< Enable CCD for the body.

		/**
		\brief Enabled CCD in swept integration for the actor.

		If this flag is raised and CCD is enabled, CCD interactions will simulate friction. By default, friction is disabled in CCD interactions because 
		CCD friction has been observed to introduce some simulation artifacts. CCD friction was enabled in previous versions of the SDK. Raising this flag will result in behavior 
		that is a closer match for previous versions of the SDK.

		\note This flag requires PxRigidBodyFlag::eENABLE_CCD to be raised to have any effect.
		*/
		eENABLE_CCD_FRICTION			= (1<<3),

		/**
		\brief Register a rigid body for reporting pose changes by the simulation at an early stage.

		Sometimes it might be advantageous to get access to the new pose of a rigid body as early as possible and
		not wait until the call to fetchResults() returns. Setting this flag will schedule the rigid body to get reported
		in #PxSimulationEventCallback::onAdvance(). Please refer to the documentation of that callback to understand
		the behavior and limitations of this functionality.

		@see PxSimulationEventCallback::onAdvance()
		*/
		eENABLE_POSE_INTEGRATION_PREVIEW 	= (1 << 4),

		/**
		\brief Register a rigid body to dynamicly adjust contact offset based on velocity. This can be used to achieve a CCD effect.
		*/
		eENABLE_SPECULATIVE_CCD 			= (1 << 5),

		/**
		\brief Permit CCD to limit maxContactImpulse. This is useful for use-cases like a destruction system but can cause visual artefacts so is not enabled by default.
		*/
		eENABLE_CCD_MAX_CONTACT_IMPULSE = (1 << 6),

		/**
		\brief Carries over forces/accelerations between frames, rather than clearning them
		*/
		eRETAIN_ACCELERATIONS = (1<<7)
	};
};

/**
\brief collection of set bits defined in PxRigidBodyFlag.

@see PxRigidBodyFlag
*/
typedef PxFlags<PxRigidBodyFlag::Enum,PxU8> PxRigidBodyFlags;
PX_FLAGS_OPERATORS(PxRigidBodyFlag::Enum,PxU8)

/**
\brief PxRigidBody is a base class shared between dynamic rigid body objects.

@see PxRigidActor
*/

class PxRigidBody : public PxRigidActor
{
public:
	// Runtime modifications

/************************************************************************************************/
/** @name Mass Manipulation
*/

	/**
	\brief Sets the pose of the center of mass relative to the actor.	
	
	\note Changing this transform will not move the actor in the world!

	\note Setting an unrealistic center of mass which is a long way from the body can make it difficult for
	the SDK to solve constraints. Perhaps leading to instability and jittering bodies.

	<b>Default:</b> the identity transform

	\param[in] pose Mass frame offset transform relative to the actor frame. <b>Range:</b> rigid body transform.

	@see getCMassLocalPose() PxRigidBodyDesc.massLocalPose
	*/
	virtual		void			setCMassLocalPose(const PxTransform& pose) = 0;


	/**
	\brief Retrieves the center of mass pose relative to the actor frame.

	\return The center of mass pose relative to the actor frame.

	@see setCMassLocalPose() PxRigidBodyDesc.massLocalPose
	*/
	virtual		PxTransform 	getCMassLocalPose() const = 0;


	/**
	\brief Sets the mass of a dynamic actor.
	
	The mass must be non-negative.
	
	setMass() does not update the inertial properties of the body, to change the inertia tensor
	use setMassSpaceInertiaTensor() or the PhysX extensions method #PxRigidBodyExt::updateMassAndInertia().

	\note A value of 0 is interpreted as infinite mass.
	\note Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic. 

	<b>Default:</b> 1.0

	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.

	\param[in] mass New mass value for the actor. <b>Range:</b> [0, PX_MAX_F32)

	@see getMass() PxRigidBodyDesc.mass setMassSpaceInertiaTensor()
	*/
	virtual		void			setMass(PxReal mass) = 0;

	/**
	\brief Retrieves the mass of the actor.

	\note A value of 0 is interpreted as infinite mass.

	\return The mass of this actor.

	@see setMass() PxRigidBodyDesc.mass setMassSpaceInertiaTensor()
	*/
	virtual		PxReal			getMass() const = 0;

	/**
	\brief Retrieves the inverse mass of the actor.

	\return The inverse mass of this actor.

	@see setMass() PxRigidBodyDesc.mass setMassSpaceInertiaTensor()
	*/
	virtual		PxReal			getInvMass() const = 0;

	/**
	\brief Sets the inertia tensor, using a parameter specified in mass space coordinates.
	
	Note that such matrices are diagonal -- the passed vector is the diagonal.

	If you have a non diagonal world/actor space inertia tensor(3x3 matrix). Then you need to
	diagonalize it and set an appropriate mass space transform. See #setCMassLocalPose().

	The inertia tensor elements must be non-negative.

	\note A value of 0 in an element is interpreted as infinite inertia along that axis.
	\note Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic. 

	<b>Default:</b> (1.0, 1.0, 1.0)

	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.

	\param[in] m New mass space inertia tensor for the actor.

	@see PxRigidBodyDesc.massSpaceInertia getMassSpaceInertia() setMass() setCMassLocalPose()
	*/
	virtual		void			setMassSpaceInertiaTensor(const PxVec3& m) = 0;

	/**
	\brief  Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.

	This method retrieves a mass frame inertia vector.

	\return The mass space inertia tensor of this actor.

	\note A value of 0 in an element is interpreted as infinite inertia along that axis.

	@see PxRigidBodyDesc.massSpaceInertia setMassSpaceInertiaTensor() setMass() setCMassLocalPose()
	*/
	virtual		PxVec3			getMassSpaceInertiaTensor()			const = 0;

	/**
	\brief  Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.

	This method retrieves a mass frame inverse inertia vector.

	\note A value of 0 in an element is interpreted as infinite inertia along that axis.

	\return The mass space inverse inertia tensor of this actor.

	@see PxRigidBodyDesc.massSpaceInertia setMassSpaceInertiaTensor() setMass() setCMassLocalPose()
	*/
	virtual		PxVec3			getMassSpaceInvInertiaTensor()			const = 0;

	/************************************************************************************************/
	/** @name Damping
	*/

	/**
	\brief Sets the linear damping coefficient.

	Zero represents no damping. The damping coefficient must be nonnegative.

	<b>Default:</b> 0.0

	\param[in] linDamp Linear damping coefficient. <b>Range:</b> [0, PX_MAX_F32)

	@see getLinearDamping() setAngularDamping()
	*/
	virtual		void				setLinearDamping(PxReal linDamp) = 0;

	/**
	\brief Retrieves the linear damping coefficient.

	\return The linear damping coefficient associated with this actor.

	@see setLinearDamping() getAngularDamping()
	*/
	virtual		PxReal				getLinearDamping() const = 0;

	/**
	\brief Sets the angular damping coefficient.

	Zero represents no damping.

	The angular damping coefficient must be nonnegative.

	<b>Default:</b> 0.05

	\param[in] angDamp Angular damping coefficient. <b>Range:</b> [0, PX_MAX_F32)

	@see getAngularDamping() setLinearDamping()
	*/
	virtual		void				setAngularDamping(PxReal angDamp) = 0;

	/**
	\brief Retrieves the angular damping coefficient.

	\return The angular damping coefficient associated with this actor.

	@see setAngularDamping() getLinearDamping()
	*/
	virtual		PxReal				getAngularDamping() const = 0;


/************************************************************************************************/
/** @name Velocity
*/


	/**
	\brief Retrieves the linear velocity of an actor.

	\return The linear velocity of the actor.

	@see PxRigidDynamic.setLinearVelocity() getAngularVelocity()
	*/
	virtual		PxVec3			getLinearVelocity()		const = 0;

	/**
	\brief Sets the linear velocity of the actor.
	
	Note that if you continuously set the velocity of an actor yourself, 
	forces such as gravity or friction will not be able to manifest themselves, because forces directly
	influence only the velocity/momentum of an actor.

	<b>Default:</b> (0.0, 0.0, 0.0)

	<b>Sleeping:</b> This call wakes the actor if it is sleeping, the autowake parameter is true (default) or the 
	new velocity is non-zero

	\note It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.

	\param[in] linVel New linear velocity of actor. <b>Range:</b> velocity vector
	\param[in] autowake Whether to wake the object up if it is asleep and the velocity is non-zero. If true and the current wake counter value is smaller than #PxSceneDesc::wakeCounterResetValue it will get increased to the reset value.

	@see getLinearVelocity() setAngularVelocity()
	*/
	virtual		void			setLinearVelocity(const PxVec3& linVel, bool autowake = true ) = 0;



	/**
	\brief Retrieves the angular velocity of the actor.

	\return The angular velocity of the actor.

	@see PxRigidDynamic.setAngularVelocity() getLinearVelocity() 
	*/
	virtual		PxVec3			getAngularVelocity()	const = 0;


	/**
	\brief Sets the angular velocity of the actor.
	
	Note that if you continuously set the angular velocity of an actor yourself, 
	forces such as friction will not be able to rotate the actor, because forces directly influence only the velocity/momentum.

	<b>Default:</b> (0.0, 0.0, 0.0)

	<b>Sleeping:</b> This call wakes the actor if it is sleeping, the autowake parameter is true (default) or the 
	new velocity is non-zero

	\note It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.

	\param[in] angVel New angular velocity of actor. <b>Range:</b> angular velocity vector
	\param[in] autowake Whether to wake the object up if it is asleep and the velocity is non-zero.  If true and the current wake 
	counter value is smaller than #PxSceneDesc::wakeCounterResetValue it will get increased to the reset value.

	@see getAngularVelocity() setLinearVelocity() 
	*/
	virtual		void			setAngularVelocity(const PxVec3& angVel, bool autowake = true ) = 0;

	/**
	\brief Lets you set the maximum angular velocity permitted for this actor.

	For various internal computations, very quickly rotating actors introduce error
	into the simulation, which leads to undesired results.

	With this function, you can set the  maximum angular velocity permitted for this rigid body.
	Higher angular velocities are clamped to this value.

	Note: The angular velocity is clamped to the set value <i>before</i> the solver, which means that
	the limit may still be momentarily exceeded.

	<b>Default:</b> 100.0

	\param[in] maxAngVel Max allowable angular velocity for actor. <b>Range:</b> [0, PX_MAX_F32)

	@see getMaxAngularVelocity()
	*/
	virtual		void				setMaxAngularVelocity(PxReal maxAngVel) = 0;

	/**
	\brief Retrieves the maximum angular velocity permitted for this actor.

	\return The maximum allowed angular velocity for this actor.

	@see setMaxAngularVelocity
	*/
	virtual		PxReal				getMaxAngularVelocity()	const = 0;


	/**
	\brief Lets you set the maximum linear velocity permitted for this actor.

	With this function, you can set the  maximum linear velocity permitted for this rigid body.
	Higher angular velocities are clamped to this value.

	Note: The angular velocity is clamped to the set value <i>before</i> the solver, which means that
	the limit may still be momentarily exceeded.

	<b>Default:</b> PX_MAX_F32

	\param[in] maxLinVel Max allowable linear velocity for actor. <b>Range:</b> [0, PX_MAX_F32)

	@see getMaxAngularVelocity()
	*/
	virtual		void				setMaxLinearVelocity(PxReal maxLinVel) = 0;

	/**
	\brief Retrieves the maximum angular velocity permitted for this actor.

	\return The maximum allowed angular velocity for this actor.

	@see setMaxLinearVelocity
	*/
	virtual		PxReal				getMaxLinearVelocity()	const = 0;


/************************************************************************************************/
/** @name Forces
*/

	/**
	\brief Applies a force (or impulse) defined in the global coordinate frame to the actor at its center of mass.
	
	<b>This will not induce a torque</b>.

	::PxForceMode determines if the force is to be conventional or impulsive.
	
	Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION 
	and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same 
	accumulators and are just short hand for multiplying the vector parameter by inverse mass and then using PxForceMode::eACCELERATION and 
	PxForceMode::eVELOCITY_CHANGE respectively.


	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\note The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.

	\note if this is called on an articulation link, only the link is updated, not the entire articulation.

	\note see #PxRigidBodyExt::computeVelocityDeltaFromImpulse for details of how to compute the change in linear velocity that 
	will arise from the application of an impulsive force, where an impulsive force is applied force multiplied by a timestep.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the autowake parameter is true (default) or the force is non-zero.

	\param[in] force Force/Impulse to apply defined in the global frame.
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode)
	\param[in] autowake Specify if the call should wake up the actor if it is currently asleep. If true and the current wake counter value is smaller than #PxSceneDesc::wakeCounterResetValue it will get increased to the reset value.

	@see PxForceMode addTorque
	*/
	virtual		void			addForce(const PxVec3& force, PxForceMode::Enum mode = PxForceMode::eFORCE, bool autowake = true) = 0;

	/**
	\brief Applies an impulsive torque defined in the global coordinate frame to the actor.

	::PxForceMode determines if the torque is to be conventional or impulsive.
	
	Each actor has an angular acceleration and an angular velocity change accumulator which are directly modified using the modes 
	PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE 
	also modify these same accumulators and are just short hand for multiplying the vector parameter by inverse inertia and then 
	using PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.
	

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\note The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
	
	\note if this called on an articulation link, only the link is updated, not the entire articulation.

	\note see #PxRigidBodyExt::computeVelocityDeltaFromImpulse for details of how to compute the change in angular velocity that 
	will arise from the application of an impulsive torque, where an impulsive torque is an applied torque multiplied by a timestep.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the autowake parameter is true (default) or the torque is non-zero.

	\param[in] torque Torque to apply defined in the global frame. <b>Range:</b> torque vector
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode).
	\param[in] autowake whether to wake up the object if it is asleep. If true and the current wake counter value is smaller than #PxSceneDesc::wakeCounterResetValue it will get increased to the reset value.

	@see PxForceMode addForce()
	*/
	virtual		void			addTorque(const PxVec3& torque, PxForceMode::Enum mode = PxForceMode::eFORCE, bool autowake = true) = 0;

	/**
	\brief Clears the accumulated forces (sets the accumulated force back to zero).
	
	Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION 
	and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same 
	accumulators (see PxRigidBody::addForce() for details); therefore the effect of calling clearForce(PxForceMode::eFORCE) is equivalent to calling 
	clearForce(PxForceMode::eACCELERATION), and the effect of calling clearForce(PxForceMode::eIMPULSE) is equivalent to calling 
	clearForce(PxForceMode::eVELOCITY_CHANGE).

	::PxForceMode determines if the cleared force is to be conventional or impulsive.

	\note The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\param[in] mode The mode to use when clearing the force/impulse(see #PxForceMode)

	@see PxForceMode addForce
	*/
	virtual		void			clearForce(PxForceMode::Enum mode = PxForceMode::eFORCE) = 0;

	/**
	\brief Clears the impulsive torque defined in the global coordinate frame to the actor.

	::PxForceMode determines if the cleared torque is to be conventional or impulsive.
	
	Each actor has an angular acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION 
	and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same 
	accumulators (see PxRigidBody::addTorque() for details); therefore the effect of calling clearTorque(PxForceMode::eFORCE) is equivalent to calling 
	clearTorque(PxForceMode::eACCELERATION), and the effect of calling clearTorque(PxForceMode::eIMPULSE) is equivalent to calling 
	clearTorque(PxForceMode::eVELOCITY_CHANGE).	

	\note The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\param[in] mode The mode to use when clearing the force/impulse(see #PxForceMode).

	@see PxForceMode addTorque
	*/
	virtual		void			clearTorque(PxForceMode::Enum mode = PxForceMode::eFORCE) = 0;


	/**
	\brief Sets the impulsive force and torque defined in the global coordinate frame to the actor.

	::PxForceMode determines if the cleared torque is to be conventional or impulsive.

	\note The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	@see PxForceMode addTorque
	*/
	virtual		void			setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode = PxForceMode::eFORCE) = 0;

	/**
	\brief Raises or clears a particular rigid body flag.
	
	See the list of flags #PxRigidBodyFlag

	<b>Default:</b> no flags are set

	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.

	\param[in] flag		The PxRigidBody flag to raise(set) or clear. See #PxRigidBodyFlag.
	\param[in] value	The new boolean value for the flag.

	@see PxRigidBodyFlag getRigidBodyFlags() 
	*/

	virtual		void				setRigidBodyFlag(PxRigidBodyFlag::Enum flag, bool value) = 0;
	virtual		void				setRigidBodyFlags(PxRigidBodyFlags inFlags) = 0;

	/**
	\brief Reads the PxRigidBody flags.
	
	See the list of flags #PxRigidBodyFlag

	\return The values of the PxRigidBody flags.

	@see PxRigidBodyFlag setRigidBodyFlag()
	*/
	virtual		PxRigidBodyFlags	getRigidBodyFlags()	const = 0;

	/**
	\brief Sets the CCD minimum advance coefficient.

	The CCD minimum advance coefficient is a value in the range [0, 1] that is used to control the minimum amount of time a body is integrated when
	it has a CCD contact. The actual minimum amount of time that is integrated depends on various properties, including the relative speed and collision shapes
	of the bodies involved in the contact. From these properties, a numeric value is calculated that determines the maximum distance (and therefore maximum time) 
	which these bodies could be integrated forwards that would ensure that these bodies did not pass through each-other. This value is then scaled by CCD minimum advance
	coefficient to determine the amount of time that will be consumed in the CCD pass.

	<b>Things to consider:</b> 
	A large value (approaching 1) ensures that the objects will always advance some time. However, larger values increase the chances of objects gently drifting through each-other in
	scenes which the constraint solver can't converge, e.g. scenes where an object is being dragged through a wall with a constraint.
	A value of 0 ensures that the pair of objects stop at the exact time-of-impact and will not gently drift through each-other. However, with very small/thin objects initially in 
	contact, this can lead to a large amount of time being dropped and increases the chances of jamming. Jamming occurs when the an object is persistently in contact with an object 
	such that the time-of-impact is	0, which results in no time being advanced for those objects in that CCD pass.

	The chances of jamming can be reduced by increasing the number of CCD mass @see PxSceneDesc.ccdMaxPasses. However, increasing this number increases the CCD overhead.

	\param[in] advanceCoefficient The CCD min advance coefficient. <b>Range:</b> [0, 1] <b>Default:</b> 0.15
	*/

	virtual void setMinCCDAdvanceCoefficient(PxReal advanceCoefficient) = 0;

	/**
	\brief Gets the CCD minimum advance coefficient.

	\return The value of the CCD min advance coefficient.

	@see setMinCCDAdvanceCoefficient

	*/

	virtual PxReal getMinCCDAdvanceCoefficient() const = 0;


	/**
	\brief Sets the maximum depenetration velocity permitted to be introduced by the solver.
	This value controls how much velocity the solver can introduce to correct for penetrations in contacts. 
	\param[in] biasClamp The maximum velocity to de-penetrate by <b>Range:</b> (0, PX_MAX_F32].
	*/
	virtual void setMaxDepenetrationVelocity(PxReal biasClamp) = 0;

	/**
	\brief Returns the maximum depenetration velocity the solver is permitted to introduced.
	This value controls how much velocity the solver can introduce to correct for penetrations in contacts. 
	\return The maximum penetration bias applied by the solver.
	*/
	virtual PxReal getMaxDepenetrationVelocity() const = 0;


	/**
	\brief Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
	bodies will be the minimum	of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
	by the value for the dynamic body.

	\param[in] maxImpulse the maximum contact impulse. <b>Range:</b> [0, PX_MAX_F32] <b>Default:</b> PX_MAX_F32

	@see getMaxContactImpulse
	*/
	virtual void setMaxContactImpulse(PxReal maxImpulse) = 0;

	/**
	\brief Returns the maximum impulse that may be applied at a contact.

	\return The maximum impulse that may be applied at a contact

	@see setMaxContactImpulse
	*/
	virtual PxReal getMaxContactImpulse() const = 0;

	/**
	\brief Returns the island node index that only for internal use only

	\return The island node index that only for internal use only
	*/
	virtual PxU32 getInternalIslandNodeIndex() const = 0;


protected:
	PX_INLINE					PxRigidBody(PxType concreteType, PxBaseFlags baseFlags) : PxRigidActor(concreteType, baseFlags) {}
	PX_INLINE					PxRigidBody(PxBaseFlags baseFlags) : PxRigidActor(baseFlags) {}
	virtual						~PxRigidBody()	{}
	virtual		bool			isKindOf(const char* name)const	{	return !::strcmp("PxRigidBody", name) || PxRigidActor::isKindOf(name); }
};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
