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

#ifndef PX_FIXEDJOINT_H
#define PX_FIXEDJOINT_H
/** \addtogroup extensions
  @{
*/

#include "extensions/PxJoint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxFixedJoint;

/**
\brief Create a fixed joint.

 \param[in] physics		The physics SDK
 \param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame0	The position and orientation of the joint relative to actor0
 \param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
 \param[in] localFrame1	The position and orientation of the joint relative to actor1 

@see PxFixedJoint
*/
PxFixedJoint*	PxFixedJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);

/**
 \brief A fixed joint permits no relative movement between two bodies. ie the bodies are glued together.

 \image html fixedJoint.png

 @see PxFixedJointCreate() PxJoint
*/
class PxFixedJoint : public PxJoint
{
public:
	
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

	@see setProjectionLinearTolerance() PxJoint::setConstraintFlag()
	*/
	virtual PxReal				getProjectionLinearTolerance()	const	= 0;

	/**
	\brief Set the angular tolerance threshold for projection. Projection is enabled if 
	PxConstraintFlag::ePROJECTION is set for the joint.

	If the joint deviates by more than this angle around its locked angular degrees of freedom, 
	the solver will move the bodies to close the angle.
	
	Setting a very small tolerance may result in simulation jitter or other artifacts.

	Sometimes it is not possible to project (for example when the joints form a cycle).

	<b>Range:</b> [0,Pi] <br>
	<b>Default:</b> Pi

	\param[in] tolerance the angular tolerance threshold in radians

	@see getProjectionAngularTolerance() PxJoint::setConstraintFlag() PxConstraintFlag::ePROJECTION
	*/
	virtual void				setProjectionAngularTolerance(PxReal tolerance)	= 0;

	/**
	\brief Get the angular tolerance threshold for projection.

	\return the angular tolerance threshold in radians

	@see setProjectionAngularTolerance() 
	*/
	virtual PxReal				getProjectionAngularTolerance()	const	= 0;
	
	/**
	\brief Returns string name of PxFixedJoint, used for serialization
	*/
	virtual	const char*			getConcreteTypeName() const { return "PxFixedJoint"; }

protected:

	//serialization

	/**
	\brief Constructor
	*/
	PX_INLINE					PxFixedJoint(PxType concreteType, PxBaseFlags baseFlags) : PxJoint(concreteType, baseFlags) {}

	/**
	\brief Deserialization constructor
	*/
	PX_INLINE					PxFixedJoint(PxBaseFlags baseFlags) : PxJoint(baseFlags)	{}

	/**
	\brief Returns whether a given type name matches with the type of this instance
	*/
	virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxFixedJoint", name) || PxJoint::isKindOf(name);	}

	//~serialization
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
