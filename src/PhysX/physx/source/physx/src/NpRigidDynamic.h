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


#ifndef PX_PHYSICS_NP_RIGIDDYNAMIC
#define PX_PHYSICS_NP_RIGIDDYNAMIC

#include "NpRigidBodyTemplate.h"
#include "PxRigidDynamic.h"
#include "ScbBody.h"
#include "PxMetaData.h"

namespace physx
{

class NpRigidDynamic;
typedef NpRigidBodyTemplate<PxRigidDynamic> NpRigidDynamicT;

class NpRigidDynamic : public NpRigidDynamicT
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
									NpRigidDynamic(PxBaseFlags baseFlags) : NpRigidDynamicT(baseFlags) {}

	virtual		void				exportData(PxSerializationContext& context) const;
	virtual		void				requiresObjects(PxProcessPxBaseCallback& c);

	static		NpRigidDynamic*		createObject(PxU8*& address, PxDeserializationContext& context);
	static		void				getBinaryMetaData(PxOutputStream& stream);	
//~PX_SERIALIZATION
	virtual							~NpRigidDynamic();

	//---------------------------------------------------------------------------------
	// PxActor implementation
	//---------------------------------------------------------------------------------

	virtual		void				release();

	//---------------------------------------------------------------------------------
	// PxRigidDynamic implementation
	//---------------------------------------------------------------------------------

	virtual		PxActorType::Enum	getType() const { return PxActorType::eRIGID_DYNAMIC; }

	// Pose
	virtual		void 				setGlobalPose(const PxTransform& pose, bool autowake);
	PX_FORCE_INLINE		PxTransform			getGlobalPoseFast() const
	{
		const Scb::Body& body=getScbBodyFast();
		return body.getBody2World() * body.getBody2Actor().getInverse();
	}
	virtual		PxTransform			getGlobalPose() const
	{
		NP_READ_CHECK(NpActor::getOwnerScene(*this));
		return getGlobalPoseFast();
	}

	virtual		void				setKinematicTarget(const PxTransform& destination);
	virtual		bool				getKinematicTarget(PxTransform& target)	const;

	// Center of mass pose
	virtual		void				setCMassLocalPose(const PxTransform&);

	// Damping
	virtual		void				setLinearDamping(PxReal);
	virtual		PxReal				getLinearDamping() const;
	virtual		void				setAngularDamping(PxReal);
	virtual		PxReal				getAngularDamping() const;

	// Velocity
	virtual		void				setLinearVelocity(const PxVec3&, bool autowake);
	virtual		void				setAngularVelocity(const PxVec3&, bool autowake);
	virtual		void				setMaxAngularVelocity(PxReal);
	virtual		PxReal				getMaxAngularVelocity() const;
	virtual		void				setMaxLinearVelocity(PxReal);
	virtual		PxReal				getMaxLinearVelocity() const;

	// Force/Torque modifiers
	virtual		void				addForce(const PxVec3&, PxForceMode::Enum mode, bool autowake);
	virtual		void				clearForce(PxForceMode::Enum mode);
	virtual		void				addTorque(const PxVec3&, PxForceMode::Enum mode, bool autowake);
	virtual		void				clearTorque(PxForceMode::Enum mode);
	virtual		void				setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode = PxForceMode::eFORCE);



	// Sleeping
	virtual		bool				isSleeping() const;
	virtual		PxReal				getSleepThreshold() const;
	virtual		void				setSleepThreshold(PxReal threshold);
	virtual		PxReal				getStabilizationThreshold() const;
	virtual		void				setStabilizationThreshold(PxReal threshold);
	virtual		void				setWakeCounter(PxReal wakeCounterValue);
	virtual		PxReal				getWakeCounter() const;
	virtual		void				wakeUp();
	virtual		void				putToSleep();

	virtual		void				setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters);
	virtual		void				getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const;

	virtual		void				setContactReportThreshold(PxReal threshold);
	virtual		PxReal				getContactReportThreshold() const;

	virtual		PxRigidDynamicLockFlags getRigidDynamicLockFlags() const;
	virtual		void				setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags);
	virtual		void				setRigidDynamicLockFlag(PxRigidDynamicLockFlag::Enum flag, bool value);

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
									NpRigidDynamic(const PxTransform& bodyPose);

	virtual		void				switchToNoSim();
	virtual		void				switchFromNoSim();

	PX_FORCE_INLINE void			wakeUpInternal();
					void			wakeUpInternalNoKinematicTest(Scb::Body& body, bool forceWakeUp, bool autowake);

private:
	PX_FORCE_INLINE	void			setKinematicTargetInternal(const PxTransform& destination);

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
				void				visualize(Cm::RenderOutput& out, NpScene* scene);
#endif
};




PX_FORCE_INLINE void NpRigidDynamic::wakeUpInternal()
{
	PX_ASSERT(NpActor::getOwnerScene(*this));

	Scb::Body& body = getScbBodyFast();
	const PxRigidBodyFlags currentFlags = body.getFlags();

	if (!(currentFlags & PxRigidBodyFlag::eKINEMATIC))  // kinematics are only awake when a target is set, else they are asleep
		wakeUpInternalNoKinematicTest(body, false, true);
}


}

#endif
