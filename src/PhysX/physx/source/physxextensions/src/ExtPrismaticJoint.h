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

#ifndef NP_PRISMATICJOINTCONSTRAINT_H
#define NP_PRISMATICJOINTCONSTRAINT_H

#include "ExtJoint.h"
#include "PxPrismaticJoint.h"
#include "PxTolerancesScale.h"
#include "CmUtils.h"

namespace physx
{
struct PxPrismaticJointGeneratedValues;
namespace Ext
{
	struct PrismaticJointData : public JointData
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

		PxJointLinearLimitPair	limit;
		PxReal					projectionLinearTolerance;
		PxReal					projectionAngularTolerance;

		PxPrismaticJointFlags	jointFlags;
		// forestall compiler complaints about not being able to generate a constructor
	private:
		PrismaticJointData(const PxJointLinearLimitPair &pair):
			limit(pair) {}
	};

    typedef Joint<PxPrismaticJoint, PxPrismaticJointGeneratedValues> PrismaticJointT;
   
	class PrismaticJoint : public PrismaticJointT
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:
// PX_SERIALIZATION
									PrismaticJoint(PxBaseFlags baseFlags) : PrismaticJointT(baseFlags) {}
		virtual		void			exportExtraData(PxSerializationContext& context);
					void			importExtraData(PxDeserializationContext& context);
					void			resolveReferences(PxDeserializationContext& context);
		static		PrismaticJoint*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void			getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

		PrismaticJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
			PrismaticJointT(PxJointConcreteType::ePRISMATIC, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, actor0, localFrame0, actor1, localFrame1, sizeof(PrismaticJointData), "PrismaticJointData")
		{
			PrismaticJointData* data = static_cast<PrismaticJointData*>(mData);

			data->limit							= PxJointLinearLimitPair(scale);
			data->projectionLinearTolerance		= 1e10f;
			data->projectionAngularTolerance	= PxPi;
			data->jointFlags					= PxPrismaticJointFlags();
		}

		// PxPrismaticJoint
		virtual	PxReal					getPosition()	const	{	return getRelativeTransform().p.x;		}
		virtual	PxReal					getVelocity()	const 	{	return getRelativeLinearVelocity().x;	}
		virtual	void					setLimit(const PxJointLinearLimitPair& limit);
		virtual	PxJointLinearLimitPair	getLimit()	const;
		virtual	void					setPrismaticJointFlags(PxPrismaticJointFlags flags);
		virtual	void					setPrismaticJointFlag(PxPrismaticJointFlag::Enum flag, bool value);
		virtual	PxPrismaticJointFlags	getPrismaticJointFlags()	const;
		virtual	void					setProjectionLinearTolerance(PxReal tolerance);
		virtual	PxReal					getProjectionLinearTolerance()	const;
		virtual	void					setProjectionAngularTolerance(PxReal tolerance);
		virtual	PxReal					getProjectionAngularTolerance()	const;
		//~PxPrismaticJoint
		
		bool					attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1);
		
		static const PxConstraintShaderTable& getConstraintShaderTable() { return sShaders; }

		virtual PxConstraintSolverPrep getPrep() const { return sShaders.solverPrep; }

	private:
		PX_FORCE_INLINE PrismaticJointData& data() const				
		{	
			return *static_cast<PrismaticJointData*>(mData);
		}

		static PxConstraintShaderTable sShaders;
	};
} // namespace Ext

namespace Ext
{
	// global function to share the joint shaders with API capture	
	extern "C" const PxConstraintShaderTable* GetPrismaticJointShaderTable();
}

}

#endif
