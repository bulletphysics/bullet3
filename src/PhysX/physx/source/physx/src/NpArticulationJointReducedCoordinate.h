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


#ifndef PX_PHYSICS_NP_ARTICULATION_JOINT_RC
#define PX_PHYSICS_NP_ARTICULATION_JOINT_RC

#include "PxArticulationJointReducedCoordinate.h"
#include "ScbArticulationJoint.h"
#include "NpArticulationTemplate.h"
#include "NpArticulationJoint.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

namespace physx
{

	class NpArticulationJointReducedCoordinate : public NpArticulationJointTemplate<PxArticulationJointReducedCoordinate>
	{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================
	public:
		// PX_SERIALIZATION
		NpArticulationJointReducedCoordinate(PxBaseFlags baseFlags) : NpArticulationJointTemplate(baseFlags) {}
		virtual		void				resolveReferences(PxDeserializationContext& context) { mImpl.resolveReferences(context); }
		static		NpArticulationJointReducedCoordinate* createObject(PxU8*& address, PxDeserializationContext& context);
		static		void				getBinaryMetaData(PxOutputStream& stream);
		void				exportExtraData(PxSerializationContext&) {}
		void				importExtraData(PxDeserializationContext&) {}
		virtual		void				requiresObjects(PxProcessPxBaseCallback&) {}
		virtual		bool			    isSubordinate()  const { return true; }
		//~PX_SERIALIZATION
		NpArticulationJointReducedCoordinate(NpArticulationLink& parent,
			const PxTransform& parentFrame,
			NpArticulationLink& child,
			const PxTransform& childFrame);

		virtual							~NpArticulationJointReducedCoordinate();

		//---------------------------------------------------------------------------------
		// PxArticulationJoint implementation
		//---------------------------------------------------------------------------------
		// Save


		virtual		void				setJointType(PxArticulationJointType::Enum jointType);
		virtual		PxArticulationJointType::Enum
			getJointType() const;

		virtual		void				setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
		virtual		PxArticulationMotion::Enum
			getMotion(PxArticulationAxis::Enum axis) const;

		virtual		void				setFrictionCoefficient(const PxReal coefficient);
		virtual		PxReal				getFrictionCoefficient() const;

		virtual		void				setMaxJointVelocity(const PxReal maxJointV);
		virtual		PxReal				getMaxJointVelocity() const;

		virtual void setLimit(PxArticulationAxis::Enum axis, const PxReal lowLimit, const PxReal highLimit);
		virtual void getLimit(PxArticulationAxis::Enum axis, PxReal& lowLimit, PxReal& highLimit);
		virtual void setDrive(PxArticulationAxis::Enum axis, const PxReal stiffness, const PxReal damping, const PxReal maxForce, bool isAccelerationDrive);
		virtual void getDrive(PxArticulationAxis::Enum axis, PxReal& stiffness, PxReal& damping, PxReal& maxForce, bool& isAcceleration);
		virtual void setDriveTarget(PxArticulationAxis::Enum axis, const PxReal target);
		virtual void setDriveVelocity(PxArticulationAxis::Enum axis, const PxReal targetVel);
		virtual PxReal getDriveTarget(PxArticulationAxis::Enum axis);
		virtual PxReal getDriveVelocity(PxArticulationAxis::Enum axis);
#if PX_CHECKED
	private:
		bool isValidMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
#endif
	};

}

#endif
