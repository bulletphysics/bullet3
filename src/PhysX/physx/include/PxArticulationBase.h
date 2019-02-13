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


#ifndef PX_PHYSICS_NX_ARTICULATION_BASE
#define PX_PHYSICS_NX_ARTICULATION_BASE
/** \addtogroup physics
@{ */

#include "PxPhysXConfig.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	
	class PxArticulationImpl;

	/**
	\brief a tree structure of bodies connected by joints that is treated as a unit by the dynamics solver

	Articulations are more expensive to simulate than the equivalent collection of
	PxRigidDynamic and PxJoint structures, but because the dynamics solver treats
	each articulation as a single object, they are much less prone to separation and
	have better support for actuation. An articulation may have at most 64 links.

	@see PxArticulationJoint PxArticulationLink PxPhysics.createArticulation
	*/
	class PxArticulationBase : public PxBase
	{
	public:

		enum Enum
		{
			eReducedCoordinate = 0,
			eMaximumCoordinate = 1
		};

		/**
		\brief Retrieves the scene which this articulation belongs to.

		\return Owner Scene. NULL if not part of a scene.

		@see PxScene
		*/
		virtual		PxScene*		getScene()	const = 0;

		/**
		\brief Sets the solver iteration counts for the articulation.

		The solver iteration count determines how accurately joints and contacts are resolved.
		If you are having trouble with jointed bodies oscillating and behaving erratically, then
		setting a higher position iteration count may improve their stability.

		If intersecting bodies are being depenetrated too violently, increase the number of velocity
		iterations. More velocity iterations will drive the relative exit velocity of the intersecting
		objects closer to the correct value given the restitution.

		\param[in] minPositionIters Number of position iterations the solver should perform for this articulation. <b>Range:</b> [1,255]
		\param[in] minVelocityIters Number of velocity iterations the solver should perform for this articulation. <b>Range:</b> [1,255]

		@see getSolverIterationCounts()
		*/
		virtual		void				setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1) = 0;

		/**
		\brief Retrieves the solver iteration counts.

		@see setSolverIterationCounts()
		*/
		virtual		void				getSolverIterationCounts(PxU32 & minPositionIters, PxU32 & minVelocityIters) const = 0;

		/**
		\brief Returns true if this articulation is sleeping.

		When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
		is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
		or a sleep-affecting property is changed by the user, the entire sleep mechanism should be transparent to the user.

		An articulation can only go to sleep if all links are ready for sleeping. An articulation is guaranteed to be awake
		if at least one of the following holds:

		\li The wake counter is positive (see #setWakeCounter()).
		\li The linear or angular velocity of any link is non-zero.
		\li A non-zero force or torque has been applied to the articulation or any of its links.

		If an articulation is sleeping, the following state is guaranteed:

		\li The wake counter is zero.
		\li The linear and angular velocity of all links is zero.
		\li There is no force update pending.

		When an articulation gets inserted into a scene, it will be considered asleep if all the points above hold, else it will
		be treated as awake.

		If an articulation is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the poses of the
		links were not changed. You can use this information to avoid updating the transforms of associated of dependent objects.

		\note It is invalid to use this method if the articulation has not been added to a scene already.

		\return True if the articulation is sleeping.

		@see isSleeping() wakeUp() putToSleep()  getSleepThreshold()
		*/
		virtual		bool				isSleeping() const = 0;

		/**
		\brief Sets the mass-normalized energy threshold below which an articulation may go to sleep.

		The articulation will sleep if the energy of each body is below this threshold.

		\param[in] threshold Energy below which an actor may go to sleep. <b>Range:</b> [0, PX_MAX_F32)

		@see isSleeping() getSleepThreshold() wakeUp() putToSleep()
		*/
		virtual		void				setSleepThreshold(PxReal threshold) = 0;

		/**
		\brief Returns the mass-normalized energy below which an articulation may go to sleep.

		\return The energy threshold for sleeping.

		@see isSleeping() wakeUp() putToSleep() setSleepThreshold()
		*/
		virtual		PxReal				getSleepThreshold() const = 0;

		/**
		\brief Sets the mass-normalized kinetic energy threshold below which an articulation may participate in stabilization.

		Articulation whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.

		This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.

		<b>Default:</b> 0.01 * PxTolerancesScale::speed * PxTolerancesScale::speed

		\param[in] threshold Energy below which an actor may participate in stabilization. <b>Range:</b> [0,inf)

		@see  getStabilizationThreshold() PxSceneFlag::eENABLE_STABILIZATION
		*/
		virtual		void				setStabilizationThreshold(PxReal threshold) = 0;

		/**
		\brief Returns the mass-normalized kinetic energy below which an articulation may participate in stabilization.

		Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.

		\return The energy threshold for participating in stabilization.

		@see setStabilizationThreshold() PxSceneFlag::eENABLE_STABILIZATION
		*/
		virtual		PxReal				getStabilizationThreshold() const = 0;

		/**
		\brief Sets the wake counter for the articulation.

		The wake counter value determines the minimum amount of time until the articulation can be put to sleep. Please note
		that an articulation will not be put to sleep if the energy is above the specified threshold (see #setSleepThreshold())
		or if other awake objects are touching it.

		\note Passing in a positive value will wake the articulation up automatically.

		<b>Default:</b> 0.4 (which corresponds to 20 frames for a time step of 0.02)

		\param[in] wakeCounterValue Wake counter value. <b>Range:</b> [0, PX_MAX_F32)

		@see isSleeping() getWakeCounter()
		*/
		virtual		void				setWakeCounter(PxReal wakeCounterValue) = 0;

		/**
		\brief Returns the wake counter of the articulation.

		\return The wake counter of the articulation.

		@see isSleeping() setWakeCounter()
		*/
		virtual		PxReal				getWakeCounter() const = 0;

		/**
		\brief Wakes up the articulation if it is sleeping.

		The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.

		\note This will set the wake counter of the articulation to the value specified in #PxSceneDesc::wakeCounterResetValue.

		\note It is invalid to use this method if the articulation has not been added to a scene already.

		@see isSleeping() putToSleep()
		*/
		virtual		void				wakeUp() = 0;

		/**
		\brief Forces the articulation to sleep.

		The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.

		\note This will set any applied force, the velocity and the wake counter of all bodies in the articulation to zero.

		\note It is invalid to use this method if the articulation has not been added to a scene already.

		@see isSleeping() wakeUp()
		*/
		virtual		void				putToSleep() = 0;

		/**
		\brief adds a link to the articulation with default attribute values.

		\param[in] parent the parent link of the articulation. Should be NULL if (and only if) this is the root link
		\param[in] pose the initial pose of the new link. Must be a valid transform

		\return the new link, or NULL if the link cannot be created because the articulation has reached
		its maximum link count (currently 64).

		@see PxArticulationLink
		*/

		virtual			PxArticulationLink*			createLink(PxArticulationLink* parent, const PxTransform& pose) = 0;


		/**
		\brief returns the number of links in the articulation
		*/

		virtual		PxU32			getNbLinks() const = 0;

		/**
		\brief returns the set of links in the articulation

		\param[in] userBuffer buffer into which to write an array of articulation link pointers
		\param[in] bufferSize the size of the buffer. If this is not large enough to contain all the pointers to links,
		only as many as will fit are written.
		\param[in] startIndex Index of first link pointer to be retrieved

		\return the number of links written into the buffer.

		@see ArticulationLink
		*/

		virtual		PxU32							getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;


		/**
		\brief Sets a name string for the object that can be retrieved with getName().

		This is for debugging and is not used by the SDK. The string is not copied by the SDK,
		only the pointer is stored.

		\param[in] name String to set the objects name to.

		@see getName()
		*/
		virtual		void			setName(const char* name) = 0;

		/**
		\brief Retrieves the name string set with setName().

		\return Name string associated with object.

		@see setName()
		*/
		virtual		const char*		getName()			const = 0;

		/**
		\brief Retrieves the axis aligned bounding box enclosing the articulation.

		\param[in] inflation  Scale factor for computed world bounds. Box extents are multiplied by this value.

		\return The articulation's bounding box.

		@see PxBounds3
		*/
		virtual		PxBounds3		getWorldBounds(float inflation = 1.01f) const = 0;

		/**
		\brief Retrieves the aggregate the articulation might be a part of.

		\return The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.

		@see PxAggregate
		*/
		virtual		PxAggregate*	getAggregate() const = 0;

		virtual		PxArticulationImpl* getImpl() = 0;

		virtual const PxArticulationImpl* getImpl() const = 0;

		virtual		PxArticulationBase::Enum getType() const = 0;

		void*						userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.

		virtual						~PxArticulationBase() {}

	protected:
		PX_INLINE					PxArticulationBase(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
		PX_INLINE					PxArticulationBase(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
public:
		virtual PxArticulationJointBase* createArticulationJoint(PxArticulationLink& parent,
			const PxTransform& parentFrame,
			PxArticulationLink& child,
			const PxTransform& childFrame) = 0;
		virtual void					 releaseArticulationJoint(PxArticulationJointBase* joint) = 0;
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

  /** @} */
#endif
