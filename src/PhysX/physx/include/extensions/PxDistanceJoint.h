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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_DISTANCEJOINT_H
#define PX_DISTANCEJOINT_H
/** \addtogroup extensions
  @{
*/

#include "extensions/PxJoint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxDistanceJoint;

/**
\brief Create a distance Joint.

 \param[in] physics		The physics SDK
 \param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame0	The position and orientation of the joint relative to actor0
 \param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame1	The position and orientation of the joint relative to actor1 

@see PxDistanceJoint
*/
PxDistanceJoint*	PxDistanceJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);


/** 
\brief flags for configuring the drive of a PxDistanceJoint

@see PxDistanceJoint
*/
struct PxDistanceJointFlag
{
	enum Enum
	{
		eMAX_DISTANCE_ENABLED	= 1<<1,
		eMIN_DISTANCE_ENABLED	= 1<<2,
		eSPRING_ENABLED			= 1<<3
	};
};

typedef PxFlags<PxDistanceJointFlag::Enum, PxU16> PxDistanceJointFlags;
PX_FLAGS_OPERATORS(PxDistanceJointFlag::Enum, PxU16)

/**
\brief a joint that maintains an upper or lower bound (or both) on the distance between two points on different objects

@see PxDistanceJointCreate PxJoint
*/
class PxDistanceJoint : public PxJoint
{
public:

	/**
	\brief Return the current distance of the joint
	*/
	virtual PxReal					getDistance()	const	= 0;
	
	/**
	\brief Set the allowed minimum distance for the joint.

	The minimum	distance must be no more than the maximum distance

	<b>Default</b> 0.0f
	<b>Range</b> [0, PX_MAX_F32)

	\param[in] distance the minimum distance

	@see PxDistanceJoint::minDistance, PxDistanceJointFlag::eMIN_DISTANCE_ENABLED getMinDistance()
	*/
	virtual void					setMinDistance(PxReal distance)	= 0;

	/**
	\brief Get the allowed minimum distance for the joint.

	\return the allowed minimum distance

	@see PxDistanceJoint::minDistance, PxDistanceJointFlag::eMIN_DISTANCE_ENABLED setMinDistance()
	*/
	virtual PxReal					getMinDistance()	const	= 0;

	/**
	\brief Set the allowed maximum distance for the joint.

	The maximum	distance must be no less than the minimum distance. 

	<b>Default</b> 0.0f
	<b>Range</b> [0, PX_MAX_F32)

	\param[in] distance the maximum distance

	@see PxDistanceJoint::maxDistance, PxDistanceJointFlag::eMAX_DISTANCE_ENABLED getMinDistance()
	*/
	virtual void					setMaxDistance(PxReal distance)	= 0;

	/**
	\brief Get the allowed maximum distance for the joint.

	\return the allowed maximum distance

	@see PxDistanceJoint::maxDistance, PxDistanceJointFlag::eMAX_DISTANCE_ENABLED setMaxDistance()
	*/
	virtual PxReal					getMaxDistance()	const	= 0;

	/**
	\brief Set the error tolerance of the joint.

	\param[in] tolerance the distance beyond the allowed range at which the joint becomes active

	@see PxDistanceJoint::tolerance, getTolerance()
	*/
	virtual void					setTolerance(PxReal tolerance)	= 0;

	/**
	\brief Get the error tolerance of the joint.

	the distance beyond the joint's [min, max] range before the joint becomes active.

	<b>Default</b> 0.25f * PxTolerancesScale::length
	<b>Range</b> (0, PX_MAX_F32)

	This value should be used to ensure that if the minimum distance is zero and the 
	spring function is in use, the rest length of the spring is non-zero. 

	@see PxDistanceJoint::tolerance, setTolerance()
	*/
	virtual PxReal					getTolerance()	const	= 0;

	/**
	\brief Set the strength of the joint spring.

	The spring is used if enabled, and the distance exceeds the range [min-error, max+error].

	<b>Default</b> 0.0f
	<b>Range</b> [0, PX_MAX_F32)

	\param[in] stiffness the spring strength of the joint

	@see PxDistanceJointFlag::eSPRING_ENABLED getStiffness()
	*/
	virtual void					setStiffness(PxReal stiffness)	= 0;

	/**
	\brief Get the strength of the joint spring.

	\return stiffness the spring strength of the joint

	@see PxDistanceJointFlag::eSPRING_ENABLED setStiffness()
	*/
	virtual PxReal					getStiffness()	const	= 0;

	/**
	\brief Set the damping of the joint spring.

	The spring is used if enabled, and the distance exceeds the range [min-error, max+error].

	<b>Default</b> 0.0f
	<b>Range</b> [0, PX_MAX_F32)

	\param[in] damping the degree of damping of the joint spring of the joint

	@see PxDistanceJointFlag::eSPRING_ENABLED setDamping()
	*/
	virtual void					setDamping(PxReal damping)	= 0;
	
	/**
	\brief Get the damping of the joint spring.

	\return the degree of damping of the joint spring of the joint

	@see PxDistanceJointFlag::eSPRING_ENABLED setDamping()
	*/
	virtual PxReal					getDamping()	const	= 0;

	/**
	\brief Set the flags specific to the Distance Joint.

	<b>Default</b> PxDistanceJointFlag::eMAX_DISTANCE_ENABLED

	\param[in] flags The joint flags.

	@see PxDistanceJointFlag setFlag() getFlags()
	*/
	virtual void					setDistanceJointFlags(PxDistanceJointFlags flags) = 0;

	/**
	\brief Set a single flag specific to a Distance Joint to true or false.

	\param[in] flag The flag to set or clear.
	\param[in] value the value to which to set the flag

	@see PxDistanceJointFlag, getFlags() setFlags()
	*/
	virtual void					setDistanceJointFlag(PxDistanceJointFlag::Enum flag, bool value) = 0;

	/**
	\brief Get the flags specific to the Distance Joint.

	\return the joint flags

	@see PxDistanceJoint::flags, PxDistanceJointFlag setFlag() setFlags()
	*/
	virtual PxDistanceJointFlags	getDistanceJointFlags()	const	= 0;

	/**
	\brief Returns string name of PxDistanceJoint, used for serialization
	*/
	virtual	const char*				getConcreteTypeName() const { return "PxDistanceJoint"; }

protected:

	//serialization

	/**
	\brief Constructor
	*/
	PX_INLINE						PxDistanceJoint(PxType concreteType, PxBaseFlags baseFlags) : PxJoint(concreteType, baseFlags) {}

	/**
	\brief Deserialization constructor
	*/
	PX_INLINE						PxDistanceJoint(PxBaseFlags baseFlags)	: PxJoint(baseFlags) {}

	/**
	\brief Returns whether a given type name matches with the type of this instance
	*/							
	virtual	bool					isKindOf(const char* name)	const { return !::strcmp("PxDistanceJoint", name) || PxJoint::isKindOf(name);	}

	//~serialization
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
