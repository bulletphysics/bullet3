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

#ifndef PX_SPHERICALJOINT_H
#define PX_SPHERICALJOINT_H
/** \addtogroup extensions
  @{
*/

#include "extensions/PxJoint.h"
#include "extensions/PxJointLimit.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxSphericalJoint;

/**
\brief Create a spherical joint.

 \param[in] physics		The physics SDK
 \param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame0	The position and orientation of the joint relative to actor0
 \param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame1	The position and orientation of the joint relative to actor1 

@see PxSphericalJoint
*/
PxSphericalJoint*	PxSphericalJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);


/**
\brief Flags specific to the spherical joint.

@see PxSphericalJoint
*/
struct PxSphericalJointFlag
{
	enum Enum
	{
		eLIMIT_ENABLED	= 1<<1	//!< the cone limit for the spherical joint is enabled
	};
};
typedef PxFlags<PxSphericalJointFlag::Enum, PxU16> PxSphericalJointFlags;
PX_FLAGS_OPERATORS(PxSphericalJointFlag::Enum, PxU16)

/**
\brief A joint which behaves in a similar way to a ball and socket.

 A spherical joint removes all linear degrees of freedom from two objects.

 The position of the joint on each actor is specified by the origin of the body's joint frame.
 
 A spherical joint may have a cone limit, to restrict the motion to within a certain range. In
 addition, the bodies may be projected together if the distance between them exceeds a given threshold.
 
 Projection, drive and limits are activated by setting the appropriate flags on the joint.

 @see PxRevoluteJointCreate() PxJoint
*/
class PxSphericalJoint : public PxJoint
{
public:
	
	/**
	\brief Set the limit cone.

	If enabled, the limit cone will constrain the angular movement of the joint to lie
	within an elliptical cone.

	\return the limit cone

	@see PxJointLimitCone setLimit() 
	*/
	virtual PxJointLimitCone	getLimitCone()	const	= 0;

	/**
	\brief Get the limit cone.

	\param[in] limit the limit cone

	@see PxJointLimitCone getLimit() 
	*/
	virtual void				setLimitCone(const PxJointLimitCone& limit)	= 0;

	/**
	\brief get the swing angle of the joint from the Y axis
	*/
	virtual PxReal				getSwingYAngle()	const	= 0;

	/**
	\brief get the swing angle of the joint from the Z axis
	*/
	virtual PxReal				getSwingZAngle()	const	= 0;

	/**
	\brief Set the flags specific to the Spherical Joint.

	<b>Default</b> PxSphericalJointFlags(0)

	\param[in] flags The joint flags.

	@see PxSphericalJointFlag setFlag() getFlags()
	*/
	virtual void				setSphericalJointFlags(PxSphericalJointFlags flags) = 0;

	/**
	\brief Set a single flag specific to a Spherical Joint to true or false.

	\param[in] flag The flag to set or clear.
	\param[in] value the value to which to set the flag

	@see PxSphericalJointFlag, getFlags() setFlags()
	*/
	virtual void				setSphericalJointFlag(PxSphericalJointFlag::Enum flag, bool value) = 0;

	/**
	\brief Get the flags specific to the Spherical Joint.

	\return the joint flags

	@see PxSphericalJoint::flags, PxSphericalJointFlag setFlag() setFlags()
	*/
	virtual PxSphericalJointFlags	getSphericalJointFlags()	const	= 0;

	/**
	\brief Set the linear tolerance threshold for projection. Projection is enabled if PxConstraintFlag::ePROJECTION
	is set for the joint.

	If the joint separates by more than this distance along its locked degrees of freedom, the solver 
	will move the bodies to close the distance.

	Setting a very small tolerance may result in simulation jitter or other artifacts.

	Sometimes it is not possible to project (for example when the joints form a cycle).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	<b>Default:</b> 1e10f

	\param[in] tolerance the linear tolerance threshold

	@see getProjectionLinearTolerance() PxJoint::setConstraintFlags() PxConstraintFlag::ePROJECTION
	*/
	virtual void				setProjectionLinearTolerance(PxReal tolerance)	= 0;

	/**
	\brief Get the linear tolerance threshold for projection.

	\return the linear tolerance threshold

	@see setProjectionLinearTolerance()
	*/
	virtual PxReal				getProjectionLinearTolerance()	const	= 0;

	/**
	\brief Returns string name of PxSphericalJoint, used for serialization
	*/
	virtual	const char*			getConcreteTypeName() const { return "PxSphericalJoint"; }

protected:

	//serialization

	/**
	\brief Constructor
	*/
	PX_INLINE					PxSphericalJoint(PxType concreteType, PxBaseFlags baseFlags) : PxJoint(concreteType, baseFlags) {}

	/**
	\brief Deserialization constructor
	*/
	PX_INLINE					PxSphericalJoint(PxBaseFlags baseFlags) : PxJoint(baseFlags)	{}

	/**
	\brief Returns whether a given type name matches with the type of this instance
	*/
	virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxSphericalJoint", name) || PxJoint::isKindOf(name); }

	//~serialization
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
