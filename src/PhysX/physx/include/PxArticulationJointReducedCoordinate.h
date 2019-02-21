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


#ifndef PX_PHYSICS_NX_ARTICULATION_JOINT_RC
#define PX_PHYSICS_NX_ARTICULATION_JOINT_RC
/** \addtogroup physics
@{ */

#if 1
#include "PxPhysXConfig.h"
#include "common/PxBase.h"
#include "PxArticulationJoint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief a joint between two links in an articulation.

	The joint model is very similar to a PxSphericalJoint with swing and twist limits,
	and an implicit drive model.

	@see PxArticulation PxArticulationLink
	*/

	class PxArticulationJointReducedCoordinate : public PxArticulationJointBase
	{
	public:

		virtual	void								setJointType(PxArticulationJointType::Enum jointType) = 0;
		virtual	PxArticulationJointType::Enum		getJointType() const = 0;

		virtual	void								setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion) = 0;
		virtual	PxArticulationMotion::Enum			getMotion(PxArticulationAxis::Enum axis) const = 0;

		virtual void setLimit(PxArticulationAxis::Enum axis, const PxReal lowLimit, const PxReal highLimit) = 0;
		virtual void getLimit(PxArticulationAxis::Enum axis, PxReal& lowLimit, PxReal& highLimit) = 0;
		virtual void setDrive(PxArticulationAxis::Enum axis, const PxReal stiffness, const PxReal damping, const PxReal maxForce, bool isAccelerationDrive = false) = 0;
		virtual void getDrive(PxArticulationAxis::Enum axis, PxReal& stiffness, PxReal& damping, PxReal& maxForce, bool& isAcceleration) = 0;
		virtual void setDriveTarget(PxArticulationAxis::Enum axis, const PxReal target) = 0;
		virtual void setDriveVelocity(PxArticulationAxis::Enum axis, const PxReal targetVel) = 0;
		virtual PxReal getDriveTarget(PxArticulationAxis::Enum axis) = 0;
		virtual PxReal getDriveVelocity(PxArticulationAxis::Enum axis) = 0;

		virtual	void			setFrictionCoefficient(const PxReal coefficient) = 0;
		virtual	PxReal			getFrictionCoefficient() const = 0;
		virtual	const char*		getConcreteTypeName() const { return "PxArticulationJointReducedCoordinate"; }

		virtual void	setMaxJointVelocity(const PxReal maxJointV) = 0;
		virtual PxReal	getMaxJointVelocity() const = 0;

	protected:
		PX_INLINE					PxArticulationJointReducedCoordinate(PxType concreteType, PxBaseFlags baseFlags) : PxArticulationJointBase(concreteType, baseFlags) {}
		PX_INLINE					PxArticulationJointReducedCoordinate(PxBaseFlags baseFlags) : PxArticulationJointBase(baseFlags) {}
		virtual						~PxArticulationJointReducedCoordinate() {}
		virtual		bool			isKindOf(const char* name)	const { return !::strcmp("PxArticulationJointReducedCoordinate", name) || PxBase::isKindOf(name); }
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

  /** @} */
#endif
