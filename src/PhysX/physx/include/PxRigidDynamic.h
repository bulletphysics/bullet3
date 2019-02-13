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


#ifndef PX_PHYSICS_NX_RIGIDDYNAMIC
#define PX_PHYSICS_NX_RIGIDDYNAMIC
/** \addtogroup physics
@{
*/

#include "PxRigidBody.h"

#if !PX_DOXYGEN
namespace physx
{
#endif


/**
\brief Collection of flags providing a mechanism to lock motion along/around a specific axis.

@see PxRigidDynamic.setRigidDynamicLockFlag(), PxRigidBody.getRigidDynamicLockFlags()
*/
struct PxRigidDynamicLockFlag
{
	enum Enum
	{
		eLOCK_LINEAR_X = (1 << 0),
		eLOCK_LINEAR_Y = (1 << 1),
		eLOCK_LINEAR_Z = (1 << 2),
		eLOCK_ANGULAR_X = (1 << 3),
		eLOCK_ANGULAR_Y = (1 << 4),
		eLOCK_ANGULAR_Z = (1 << 5)
	};
};

typedef PxFlags<PxRigidDynamicLockFlag::Enum, PxU16> PxRigidDynamicLockFlags;
PX_FLAGS_OPERATORS(PxRigidDynamicLockFlag::Enum, PxU16)

/**
\brief PxRigidDynamic represents a dynamic rigid simulation object in the physics SDK.

<h3>Creation</h3>
Instances of this class are created by calling #PxPhysics::createRigidDynamic() and deleted with #release().


<h3>Visualizations</h3>
\li #PxVisualizationParameter::eACTOR_AXES
\li #PxVisualizationParameter::eBODY_AXES
\li #PxVisualizationParameter::eBODY_MASS_AXES
\li #PxVisualizationParameter::eBODY_LIN_VELOCITY
\li #PxVisualizationParameter::eBODY_ANG_VELOCITY

@see PxRigidBody  PxPhysics.createRigidDynamic()  release()
*/

class PxRigidDynamic : public PxRigidBody
{
public:
	// Runtime modifications


/************************************************************************************************/
/** @name Kinematic Actors
*/

	/**
	\brief Moves kinematically controlled dynamic actors through the game world.

	You set a dynamic actor to be kinematic using the PxRigidBodyFlag::eKINEMATIC flag
	with setRigidBodyFlag().
	
	The move command will result in a velocity that will move the body into 
	the desired pose. After the move is carried out during a single time step, 
	the velocity is returned to zero. Thus, you must continuously call 
	this in every time step for kinematic actors so that they move along a path.
	
	This function simply stores the move destination until the next simulation
	step is processed, so consecutive calls will simply overwrite the stored target variable.

	The motion is always fully carried out.	

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and will set the wake counter to #PxSceneDesc::wakeCounterResetValue.

	\param[in] destination The desired pose for the kinematic actor, in the global frame. <b>Range:</b> rigid body transform.

	@see getKinematicTarget() PxRigidBodyFlag setRigidBodyFlag()
	*/
	virtual		void				setKinematicTarget(const PxTransform& destination) = 0;

	/**
	\brief Get target pose of a kinematically controlled dynamic actor.

	\param[out] target Transform to write the target pose to. Only valid if the method returns true.
	\return True if the actor is a kinematically controlled dynamic and the target has been set, else False.

	@see setKinematicTarget() PxRigidBodyFlag setRigidBodyFlag()
	*/
	virtual		bool				getKinematicTarget(PxTransform& target)	const	= 0;


/************************************************************************************************/
/** @name Sleeping
*/

	/**
	\brief Returns true if this body is sleeping.

	When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
	is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
	or one of its properties is changed by the user, the entire sleep mechanism should be transparent to the user.

	In general, a dynamic rigid actor is guaranteed to be awake if at least one of the following holds:

	\li The wake counter is positive (see #setWakeCounter()).
	\li The linear or angular velocity is non-zero.
	\li A non-zero force or torque has been applied.

	If a dynamic rigid actor is sleeping, the following state is guaranteed:

	\li The wake counter is zero.
	\li The linear and angular velocity is zero.
	\li There is no force update pending.

	When an actor gets inserted into a scene, it will be considered asleep if all the points above hold, else it will be treated as awake.
	
	If an actor is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the pose of the actor 
	was not changed. You can use this information to avoid updating the transforms of associated objects.

	\note A kinematic actor is asleep unless a target pose has been set (in which case it will stay awake until the end of the next 
	simulation step where no target pose has been set anymore). The wake counter will get set to zero or to the reset value 
	#PxSceneDesc::wakeCounterResetValue in the case where a target pose has been set to be consistent with the definitions above.

	\note It is invalid to use this method if the actor has not been added to a scene already.

	\return True if the actor is sleeping.

	@see isSleeping() wakeUp() putToSleep()  getSleepThreshold()
	*/
	virtual		bool				isSleeping() const = 0;


    /**
	\brief Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.

	Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.

	<b>Default:</b> 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed

	\param[in] threshold Energy below which an actor may go to sleep. <b>Range:</b> [0, PX_MAX_F32)

	@see isSleeping() getSleepThreshold() wakeUp() putToSleep() PxTolerancesScale
	*/
	virtual		void				setSleepThreshold(PxReal threshold) = 0;

	/**
	\brief Returns the mass-normalized kinetic energy below which an actor may go to sleep.

	\return The energy threshold for sleeping.

	@see isSleeping() wakeUp() putToSleep() setSleepThreshold()
	*/
	virtual		PxReal				getSleepThreshold() const = 0;

	 /**
	\brief Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.

	Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.

	This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.

	<b>Default:</b> 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed

	\param[in] threshold Energy below which an actor may participate in stabilization. <b>Range:</b> [0,inf)

	@see  getStabilizationThreshold() PxSceneFlag::eENABLE_STABILIZATION
	*/
	virtual		void				setStabilizationThreshold(PxReal threshold) = 0;

	/**
	\brief Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.

	Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization. 

	\return The energy threshold for participating in stabilization.

	@see setStabilizationThreshold() PxSceneFlag::eENABLE_STABILIZATION
	*/
	virtual		PxReal				getStabilizationThreshold() const = 0;


	/**
	\brief Reads the PxRigidDynamic lock flags.

	See the list of flags #PxRigidDynamicLockFlag

	\return The values of the PxRigidDynamicLock flags.

	@see PxRigidDynamicLockFlag setRigidDynamicLockFlag()
	*/
	virtual		PxRigidDynamicLockFlags getRigidDynamicLockFlags() const = 0;

	/**
	\brief Raises or clears a particular rigid dynamic lock flag.

	See the list of flags #PxRigidDynamicLockFlag

	<b>Default:</b> no flags are set


	\param[in] flag		The PxRigidDynamicLockBody flag to raise(set) or clear. See #PxRigidBodyFlag.
	\param[in] value	The new boolean value for the flag.

	@see PxRigidDynamicLockFlag getRigidDynamicLockFlags()
	*/
	virtual		void				setRigidDynamicLockFlag(PxRigidDynamicLockFlag::Enum flag, bool value) = 0;
	virtual		void				setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags) = 0;
	


	/**
	\brief Sets the wake counter for the actor.

	The wake counter value determines the minimum amount of time until the body can be put to sleep. Please note
	that a body will not be put to sleep if the energy is above the specified threshold (see #setSleepThreshold())
	or if other awake bodies are touching it.

	\note Passing in a positive value will wake the actor up automatically.

	\note It is invalid to use this method for kinematic actors since the wake counter for kinematics is defined
	based on whether a target pose has been set (see the comment in #isSleeping()).

	\note It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.

	<b>Default:</b> 0.4 (which corresponds to 20 frames for a time step of 0.02)

	\param[in] wakeCounterValue Wake counter value. <b>Range:</b> [0, PX_MAX_F32)

	@see isSleeping() getWakeCounter()
	*/
	virtual		void				setWakeCounter(PxReal wakeCounterValue) = 0;

	/**
	\brief Returns the wake counter of the actor.

	\return The wake counter of the actor.

	@see isSleeping() setWakeCounter()
	*/
	virtual		PxReal				getWakeCounter() const = 0;

	/**
	\brief Wakes up the actor if it is sleeping.

	The actor will get woken up and might cause other touching actors to wake up as well during the next simulation step.

	\note This will set the wake counter of the actor to the value specified in #PxSceneDesc::wakeCounterResetValue.

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\note It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
	based on whether a target pose has been set (see the comment in #isSleeping()).

	@see isSleeping() putToSleep()
	*/
	virtual		void				wakeUp() = 0;

	/**
	\brief Forces the actor to sleep. 
	
	The actor will stay asleep during the next simulation step if not touched by another non-sleeping actor.
	
	\note Any applied force will be cleared and the velocity and the wake counter of the actor will be set to 0.

	\note It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.

	\note It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
	based on whether a target pose has been set (see the comment in #isSleeping()).

	@see isSleeping() wakeUp()
	*/
	virtual		void				putToSleep() = 0;

/************************************************************************************************/

    /**
	\brief Sets the solver iteration counts for the body. 
	
	The solver iteration count determines how accurately joints and contacts are resolved. 
	If you are having trouble with jointed bodies oscillating and behaving erratically, then
	setting a higher position iteration count may improve their stability.

	If intersecting bodies are being depenetrated too violently, increase the number of velocity 
	iterations. More velocity iterations will drive the relative exit velocity of the intersecting 
	objects closer to the correct value given the restitution.

	<b>Default:</b> 4 position iterations, 1 velocity iteration

	\param[in] minPositionIters Number of position iterations the solver should perform for this body. <b>Range:</b> [1,255]
	\param[in] minVelocityIters Number of velocity iterations the solver should perform for this body. <b>Range:</b> [1,255]

	@see getSolverIterationCounts()
	*/
	virtual		void				setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1) = 0;

	/**
	\brief Retrieves the solver iteration counts.

	@see setSolverIterationCounts()
	*/
	virtual		void				getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const = 0;

	/**
	\brief Retrieves the force threshold for contact reports.

	The contact report threshold is a force threshold. If the force between 
	two actors exceeds this threshold for either of the two actors, a contact report 
	will be generated according to the contact report threshold flags provided by
	the filter shader/callback.
	See #PxPairFlag.

	The threshold used for a collision between a dynamic actor and the static environment is 
    the threshold of the dynamic actor, and all contacts with static actors are summed to find 
    the total normal force.

	<b>Default:</b> PX_MAX_F32

	\return Force threshold for contact reports.

	@see setContactReportThreshold PxPairFlag PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual     PxReal				getContactReportThreshold() const = 0;

	/**
	\brief Sets the force threshold for contact reports.

	See #getContactReportThreshold().

	\param[in] threshold Force threshold for contact reports. <b>Range:</b> [0, PX_MAX_F32)

	@see getContactReportThreshold PxPairFlag
	*/
	virtual     void				setContactReportThreshold(PxReal threshold) = 0;

	virtual		const char*			getConcreteTypeName() const { return "PxRigidDynamic"; }

protected:
	PX_INLINE						PxRigidDynamic(PxType concreteType, PxBaseFlags baseFlags) : PxRigidBody(concreteType, baseFlags) {}
	PX_INLINE						PxRigidDynamic(PxBaseFlags baseFlags) : PxRigidBody(baseFlags) {}
	virtual							~PxRigidDynamic() {}
	virtual		bool				isKindOf(const char* name) const { return !::strcmp("PxRigidDynamic", name) || PxRigidBody::isKindOf(name); }

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
