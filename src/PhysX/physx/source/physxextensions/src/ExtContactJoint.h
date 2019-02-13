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


#ifndef NP_CONTACTJOINTCONSTRAINT_H
#define NP_CONTACTJOINTCONSTRAINT_H

#include "PsUserAllocated.h"
#include "ExtJoint.h"
#include "PxContactJoint.h"
#include "PxTolerancesScale.h"
#include "CmUtils.h"

namespace physx
{
struct PxContactJointGeneratedValues;
namespace Ext
{
	struct ContactJointData : public JointData
	{
		PxVec3	contact;
		PxVec3	normal;
		PxReal	penetration;
		PxReal  restitution;
		PxReal	bounceThreshold;
	};

	typedef Joint<PxContactJoint, PxContactJointGeneratedValues> ContactJointT;
	class ContactJoint : public ContactJointT
	{
	public:
		// PX_SERIALIZATION
		ContactJoint(PxBaseFlags baseFlags) : ContactJointT(baseFlags) {}
		virtual		void			exportExtraData(PxSerializationContext& context);
		void						importExtraData(PxDeserializationContext& context);
		void						resolveReferences(PxDeserializationContext& context);
		static		ContactJoint*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void			getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

		ContactJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
			ContactJointT(PxJointConcreteType::eCONTACT, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, actor0, localFrame0, actor1, localFrame1, sizeof(ContactJointData), "ContactJointData")
		{
			PX_UNUSED(scale);

			ContactJointData* data = static_cast<ContactJointData*>(mData);

			data->contact = PxVec3(0.f);
			data->normal = PxVec3(0.f);
			data->penetration = 0.f;
			data->restitution = 0.f;
			data->bounceThreshold = 0.f;
		}

		// PxContactJoint
		virtual	PxVec3					getContact()	const;
		virtual	void					setContact(const PxVec3& contact);
		virtual	PxVec3					getContactNormal()	const;
		virtual	void					setContactNormal(const PxVec3& normal);
		virtual	PxReal					getPenetration()	const;
		virtual	void					setPenetration(const PxReal penetration);
		virtual	PxReal					getResititution()	const;
		virtual	void					setResititution(const PxReal resititution);
		virtual PxReal					getBounceThreshold() const;
		virtual void					setBounceThreshold(const PxReal bounceThreshold);
		virtual void					computeJacobians(PxJacobianRow* jacobian) const;
		virtual PxU32					getNbJacobianRows() const;

		//~PxContactJoint
		bool					attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1);

		static const PxConstraintShaderTable& getConstraintShaderTable() { return sShaders; }

		virtual PxConstraintSolverPrep getPrep()const { return sShaders.solverPrep; }

	private:

		static PxConstraintShaderTable sShaders;

		PX_FORCE_INLINE ContactJointData& data() const
		{
			return *static_cast<ContactJointData*>(mData);
		}
	};


}// namespace Ext

namespace Ext
{
	// global function to share the joint shaders with API capture	
	extern "C" const PxConstraintShaderTable* GetContactJointShaderTable();
}

}//physx
#endif
