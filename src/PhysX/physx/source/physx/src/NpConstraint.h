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


#ifndef PX_PHYSICS_NP_CONSTRAINT
#define PX_PHYSICS_NP_CONSTRAINT

#include "PsUserAllocated.h"
#include "PxConstraint.h"
#include "ScbConstraint.h"

namespace physx
{

class NpScene;
class NpRigidDynamic;

namespace Scb
{
	class RigidObject;
}

class NpConstraint : public PxConstraint, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
													NpConstraint(PxBaseFlags baseFlags) : PxConstraint(baseFlags), mConstraint(PxEmpty) {}
	virtual			void							setConstraintFunctions(PxConstraintConnector& n,
																		   const PxConstraintShaderTable &t);
	static			NpConstraint*					createObject(PxU8*& address, PxDeserializationContext& context);
	static			void							getBinaryMetaData(PxOutputStream& stream);
					void							exportExtraData(PxSerializationContext&)	{}
					void							importExtraData(PxDeserializationContext&)	{ }
					void							resolveReferences(PxDeserializationContext& context);
	virtual			void							requiresObjects(PxProcessPxBaseCallback&){}
	virtual		    bool			                isSubordinate()  const	 { return true; }  
//~PX_SERIALIZATION
													NpConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
													~NpConstraint();

	virtual			void							release();

	virtual			PxScene*						getScene() const;

	virtual			void							getActors(PxRigidActor*& actor0, PxRigidActor*& actor1)		const;
	virtual			void							setActors(PxRigidActor* actor0, PxRigidActor* actor1);

	virtual			PxConstraintFlags				getFlags()													const;
	virtual			void							setFlags(PxConstraintFlags flags);
	virtual			void							setFlag(PxConstraintFlag::Enum flag, bool value);

	virtual			void							getForce(PxVec3& linear, PxVec3& angular)					const;

	virtual			void							markDirty();
	
	virtual			void							setBreakForce(PxReal linear, PxReal angular);
	virtual			void							getBreakForce(PxReal& linear, PxReal& angular)				const;

	virtual			void							setMinResponseThreshold(PxReal threshold);
	virtual			PxReal							getMinResponseThreshold()									const;

	virtual			bool							isValid() const;

	virtual			void*							getExternalReference(PxU32& typeID);

					void							initialize(const PxConstraintDesc&, Scb::Constraint*);
					void							updateConstants();
					void							comShift(PxRigidActor*);
					void							actorDeleted(PxRigidActor*);



					NpScene*						getNpScene() const;

					NpScene*						getSceneFromActors() const;
	PX_FORCE_INLINE	Scb::Constraint&				getScbConstraint()				{ return mConstraint; }
	PX_FORCE_INLINE	const Scb::Constraint&			getScbConstraint() const		{ return mConstraint; }
	PX_FORCE_INLINE	bool							isDirty() const					{ return mIsDirty; }


					static Scb::RigidObject*		getScbRigidObject(PxRigidActor*);
private:
	PX_FORCE_INLINE	static NpScene*					getSceneFromActors(const PxRigidActor* actor0, const PxRigidActor* actor1);  // Returns the scene if both actors are in the scene, else NULL

					PxRigidActor*					mActor0;
					PxRigidActor*					mActor1;
					Scb::Constraint					mConstraint;

					// this used to be stored in Scb, but that doesn't really work since Scb::Constraint's 
					// flags all get cleared on fetchResults. In any case, in order to support O(1) 
					// insert/remove this really wants to be an index into NpScene's dirty joint array

					bool							mIsDirty;
					bool							mPaddingFromBool[3];	// PT: because of mIsDirty
};

}

#endif
