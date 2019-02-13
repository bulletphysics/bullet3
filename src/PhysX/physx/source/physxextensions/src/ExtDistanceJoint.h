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


#ifndef NP_DISTANCEJOINTCONSTRAINT_H
#define NP_DISTANCEJOINTCONSTRAINT_H

#include "PsUserAllocated.h"
#include "ExtJoint.h"
#include "PxDistanceJoint.h"
#include "PxTolerancesScale.h"
#include "CmUtils.h"

namespace physx
{
struct PxDistanceJointGeneratedValues;
namespace Ext
{

	struct DistanceJointData : public JointData
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

							PxReal					minDistance;
							PxReal					maxDistance;
							PxReal					tolerance;
							PxReal					stiffness;
							PxReal					damping;

							PxDistanceJointFlags	jointFlags;
	};

    typedef Joint<PxDistanceJoint, PxDistanceJointGeneratedValues> DistanceJointT;
	class DistanceJoint : public DistanceJointT
	{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================
	public:
		// PX_SERIALIZATION
		DistanceJoint(PxBaseFlags baseFlags) : DistanceJointT(baseFlags) {}
		virtual		void			exportExtraData(PxSerializationContext& context);
					void			importExtraData(PxDeserializationContext& context);
					void			resolveReferences(PxDeserializationContext& context);
		static		DistanceJoint*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void			getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

		DistanceJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
			DistanceJointT(PxJointConcreteType::eDISTANCE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, actor0, localFrame0, actor1, localFrame1, sizeof(DistanceJointData), "DistanceJointData")
		{
			DistanceJointData* data = static_cast<DistanceJointData*>(mData);

			data->stiffness		= 0.0f;
			data->damping		= 0.0f;
			data->minDistance	= 0.0f;
			data->maxDistance	= 0.0f;
			data->tolerance		= 0.025f * scale.length;
			data->jointFlags	= PxDistanceJointFlag::eMAX_DISTANCE_ENABLED;
		}

		// PxDistanceJoint
		virtual	PxReal					getDistance()	const;
		virtual	void					setMinDistance(PxReal distance);
		virtual	PxReal					getMinDistance()	const;
		virtual	void					setMaxDistance(PxReal distance);
		virtual	PxReal					getMaxDistance()	const;
		virtual	void					setTolerance(PxReal tolerance);
		virtual	PxReal					getTolerance()	const;
		virtual	void					setStiffness(PxReal spring);
		virtual	PxReal					getStiffness()	const;
		virtual	void					setDamping(PxReal damping);
		virtual	PxReal					getDamping()	const;
		virtual	void					setDistanceJointFlags(PxDistanceJointFlags flags);
		virtual	void					setDistanceJointFlag(PxDistanceJointFlag::Enum flag, bool value);
		virtual	PxDistanceJointFlags	getDistanceJointFlags()	const;
		//~PxDistanceJoint

		bool					attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1);

		static const PxConstraintShaderTable& getConstraintShaderTable() { return sShaders; }

		virtual PxConstraintSolverPrep getPrep()const {return sShaders.solverPrep;}
		
	private:

		static PxConstraintShaderTable sShaders;

		PX_FORCE_INLINE DistanceJointData& data() const				
		{	
			return *static_cast<DistanceJointData*>(mData);
		}
	};

} // namespace Ext

namespace Ext
{
	// global function to share the joint shaders with API capture	
	extern "C" const PxConstraintShaderTable* GetDistanceJointShaderTable();
}

}

#endif
