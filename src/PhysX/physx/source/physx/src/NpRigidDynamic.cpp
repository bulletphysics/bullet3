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


#include "NpRigidDynamic.h"
#include "NpRigidActorTemplateInternal.h"

using namespace physx;

NpRigidDynamic::NpRigidDynamic(const PxTransform& bodyPose)
:	NpRigidDynamicT(PxConcreteType::eRIGID_DYNAMIC, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE, PxActorType::eRIGID_DYNAMIC, bodyPose)
{}

NpRigidDynamic::~NpRigidDynamic()
{
}

// PX_SERIALIZATION
void NpRigidDynamic::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpRigidDynamicT::requiresObjects(c);
}

NpRigidDynamic* NpRigidDynamic::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpRigidDynamic* obj = new (address) NpRigidDynamic(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpRigidDynamic);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpRigidDynamic::release()
{
	releaseActorT(this, mBody);
}


void NpRigidDynamic::setGlobalPose(const PxTransform& pose, bool autowake)
{
	NpScene* scene = NpActor::getAPIScene(*this);

#if PX_CHECKED
	if(scene)
		scene->checkPositionSanity(*this, pose, "PxRigidDynamic::setGlobalPose");
#endif

	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidDynamic::setGlobalPose: pose is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

	const PxTransform newPose = pose.getNormalized();	//AM: added to fix 1461 where users read and write orientations for no reason.
	
	Scb::Body& b = getScbBodyFast();
	const PxTransform body2World = newPose * b.getBody2Actor();
	b.setBody2World(body2World, false);

	if(scene)
		updateDynamicSceneQueryShapes(mShapeManager, scene->getSceneQueryManagerFast(), *this);

	// invalidate the pruning structure if the actor bounds changed
	if(mShapeManager.getPruningStructure())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidDynamic::setGlobalPose: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	if(scene && autowake && !(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
		wakeUpInternal();
}


PX_FORCE_INLINE void NpRigidDynamic::setKinematicTargetInternal(const PxTransform& targetPose)
{
	Scb::Body& b = getScbBodyFast();

	// The target is actor related. Transform to body related target
	const PxTransform bodyTarget = targetPose * b.getBody2Actor();

	b.setKinematicTarget(bodyTarget);

	NpScene* scene = NpActor::getAPIScene(*this);
	if ((b.getFlags() & PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES) && scene)
	{
		updateDynamicSceneQueryShapes(mShapeManager, scene->getSceneQueryManagerFast(), *this);
	}
}


void NpRigidDynamic::setKinematicTarget(const PxTransform& destination)
{
	PX_CHECK_AND_RETURN(destination.isSane(), "PxRigidDynamic::setKinematicTarget: destination is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

#if PX_CHECKED
	NpScene* scene = NpActor::getAPIScene(*this);
	if(scene)
		scene->checkPositionSanity(*this, destination, "PxRigidDynamic::setKinematicTarget");

	Scb::Body& b = getScbBodyFast();
	PX_CHECK_AND_RETURN((b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setKinematicTarget: Body must be kinematic!");
	PX_CHECK_AND_RETURN(scene, "PxRigidDynamic::setKinematicTarget: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::setKinematicTarget: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");
#endif
	
	setKinematicTargetInternal(destination.getNormalized());
}


bool NpRigidDynamic::getKinematicTarget(PxTransform& target) const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	const Scb::Body& b = getScbBodyFast();
	if(b.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		PxTransform bodyTarget;
		if(b.getKinematicTarget(bodyTarget))
		{
			// The internal target is body related. Transform to actor related target
			target = bodyTarget * b.getBody2Actor().getInverse();
			return true;
		}
	}
	return false;
}

void NpRigidDynamic::setCMassLocalPose(const PxTransform& pose)
{
	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidDynamic::setCMassLocalPose pose is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

	const PxTransform p = pose.getNormalized();

	const PxTransform oldBody2Actor = getScbBodyFast().getBody2Actor();

	NpRigidDynamicT::setCMassLocalPoseInternal(p);

	Scb::Body& b = getScbBodyFast();
	if(b.getFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		PxTransform bodyTarget;
		if(b.getKinematicTarget(bodyTarget))
		{
			PxTransform actorTarget = bodyTarget * oldBody2Actor.getInverse();  // get old target pose for the actor from the body target
			setKinematicTargetInternal(actorTarget);
		}
	}
}


void NpRigidDynamic::setLinearDamping(PxReal linearDamping)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(linearDamping), "PxRigidDynamic::setLinearDamping: invalid float");
	PX_CHECK_AND_RETURN(linearDamping >=0, "PxRigidDynamic::setLinearDamping: The linear damping must be nonnegative!");

	getScbBodyFast().setLinearDamping(linearDamping);
}


PxReal NpRigidDynamic::getLinearDamping() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getLinearDamping();
}


void NpRigidDynamic::setAngularDamping(PxReal angularDamping)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(angularDamping), "PxRigidDynamic::setAngularDamping: invalid float");
	PX_CHECK_AND_RETURN(angularDamping>=0, "PxRigidDynamic::setAngularDamping: The angular damping must be nonnegative!")
	
	getScbBodyFast().setAngularDamping(angularDamping);
}


PxReal NpRigidDynamic::getAngularDamping() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getAngularDamping();
}


void NpRigidDynamic::setLinearVelocity(const PxVec3& velocity, bool autowake)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(velocity.isFinite(), "PxRigidDynamic::setLinearVelocity: velocity is not valid.");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setLinearVelocity: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::setLinearVelocity: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");
	
	Scb::Body& b = getScbBodyFast();
	b.setLinearVelocity(velocity);

	NpScene* scene = NpActor::getAPIScene(*this);
	if(scene)
		wakeUpInternalNoKinematicTest(b, (!velocity.isZero()), autowake);
}


void NpRigidDynamic::setAngularVelocity(const PxVec3& velocity, bool autowake)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(velocity.isFinite(), "PxRigidDynamic::setAngularVelocity: velocity is not valid.");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setAngularVelocity: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::setAngularVelocity: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	Scb::Body& b = getScbBodyFast();
	b.setAngularVelocity(velocity);

	NpScene* scene = NpActor::getAPIScene(*this);
	if(scene)
		wakeUpInternalNoKinematicTest(b, (!velocity.isZero()), autowake);
}


void NpRigidDynamic::setMaxAngularVelocity(PxReal maxAngularVelocity)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(maxAngularVelocity), "PxRigidDynamic::setMaxAngularVelocity: invalid float");
	PX_CHECK_AND_RETURN(maxAngularVelocity>=0.0f, "PxRigidDynamic::setMaxAngularVelocity: threshold must be non-negative!");

	getScbBodyFast().setMaxAngVelSq(maxAngularVelocity * maxAngularVelocity);		
}


PxReal NpRigidDynamic::getMaxAngularVelocity() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return PxSqrt(getScbBodyFast().getMaxAngVelSq());
}

void NpRigidDynamic::setMaxLinearVelocity(PxReal maxLinearVelocity)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(maxLinearVelocity), "PxRigidDynamic::setMaxAngularVelocity: invalid float");
	PX_CHECK_AND_RETURN(maxLinearVelocity >= 0.0f, "PxRigidDynamic::setMaxAngularVelocity: threshold must be non-negative!");

	getScbBodyFast().setMaxLinVelSq(maxLinearVelocity * maxLinearVelocity);
}

PxReal NpRigidDynamic::getMaxLinearVelocity() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return PxSqrt(getScbBodyFast().getMaxLinVelSq());
}


void NpRigidDynamic::addForce(const PxVec3& force, PxForceMode::Enum mode, bool autowake)
{
	Scb::Body& b = getScbBodyFast();

	PX_CHECK_AND_RETURN(force.isFinite(), "PxRigidDynamic::addForce: force is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::addForce: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::addForce: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::addForce: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	addSpatialForce(&force, NULL, mode);

	wakeUpInternalNoKinematicTest(b, (!force.isZero()), autowake);
}

void NpRigidDynamic::setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode)
{
	Scb::Body& b = getScbBodyFast();

	PX_CHECK_AND_RETURN(force.isFinite(), "PxRigidDynamic::setForce: force is not valid.");
	PX_CHECK_AND_RETURN(torque.isFinite(), "PxRigidDynamic::setForce: force is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::addForce: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::addForce: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::addForce: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	setSpatialForce(&force, &torque, mode);

	wakeUpInternalNoKinematicTest(b, (!force.isZero()), true);
}


void NpRigidDynamic::addTorque(const PxVec3& torque, PxForceMode::Enum mode, bool autowake)
{
	Scb::Body& b = getScbBodyFast();

	PX_CHECK_AND_RETURN(torque.isFinite(), "PxRigidDynamic::addTorque: torque is not valid.");
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::addTorque: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::addTorque: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::addTorque: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	addSpatialForce(NULL, &torque, mode);

	wakeUpInternalNoKinematicTest(b, (!torque.isZero()), autowake);
}

void NpRigidDynamic::clearForce(PxForceMode::Enum mode)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::clearForce: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::clearForce: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::clearForce: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	clearSpatialForce(mode, true, false);
}


void NpRigidDynamic::clearTorque(PxForceMode::Enum mode)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::clearTorque: Body must be in a scene!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::clearTorque: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(getScbBodyFast().getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::clearTorque: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	clearSpatialForce(mode, false, true);
}


bool NpRigidDynamic::isSleeping() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN_VAL(NpActor::getAPIScene(*this), "PxRigidDynamic::isSleeping: Body must be in a scene.", true);

	return getScbBodyFast().isSleeping();
}


void NpRigidDynamic::setSleepThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setSleepThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold>=0.0f, "PxRigidDynamic::setSleepThreshold: threshold must be non-negative!");

	getScbBodyFast().setSleepThreshold(threshold);
}


PxReal NpRigidDynamic::getSleepThreshold() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getSleepThreshold();
}

void NpRigidDynamic::setStabilizationThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setSleepThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold>=0.0f, "PxRigidDynamic::setSleepThreshold: threshold must be non-negative!");

	getScbBodyFast().setFreezeThreshold(threshold);
}


PxReal NpRigidDynamic::getStabilizationThreshold() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getFreezeThreshold();
}


void NpRigidDynamic::setWakeCounter(PxReal wakeCounterValue)
{
	Scb::Body& b = getScbBodyFast();

	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(wakeCounterValue), "PxRigidDynamic::setWakeCounter: invalid float.");
	PX_CHECK_AND_RETURN(wakeCounterValue>=0.0f, "PxRigidDynamic::setWakeCounter: wakeCounterValue must be non-negative!");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::setWakeCounter: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::setWakeCounter: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");

	b.setWakeCounter(wakeCounterValue);
}


PxReal NpRigidDynamic::getWakeCounter() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getWakeCounter();
}


void NpRigidDynamic::wakeUp()
{
	Scb::Body& b = getScbBodyFast();

	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::wakeUp: Body must be in a scene.");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::wakeUp: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::wakeUp: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");
	
	b.wakeUp();
}


void NpRigidDynamic::putToSleep()
{
	Scb::Body& b = getScbBodyFast();

	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(NpActor::getAPIScene(*this), "PxRigidDynamic::putToSleep: Body must be in a scene.");
	PX_CHECK_AND_RETURN(!(b.getFlags() & PxRigidBodyFlag::eKINEMATIC), "PxRigidDynamic::putToSleep: Body must be non-kinematic!");
	PX_CHECK_AND_RETURN(!(b.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION), "PxRigidDynamic::putToSleep: Not allowed if PxActorFlag::eDISABLE_SIMULATION is set!");
	
	b.putToSleep();
}


void NpRigidDynamic::setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(positionIters > 0, "PxRigidDynamic::setSolverIterationCount: positionIters must be more than zero!");
	PX_CHECK_AND_RETURN(positionIters <= 255, "PxRigidDynamic::setSolverIterationCount: positionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(velocityIters > 0, "PxRigidDynamic::setSolverIterationCount: velocityIters must be more than zero!");
	PX_CHECK_AND_RETURN(velocityIters <= 255, "PxRigidDynamic::setSolverIterationCount: velocityIters must be no greater than 255!");

	getScbBodyFast().setSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));
}


void NpRigidDynamic::getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	PxU16 x = getScbBodyFast().getSolverIterationCounts();
	velocityIters = PxU32(x >> 8);
	positionIters = PxU32(x & 0xff);
}


void NpRigidDynamic::setContactReportThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(threshold), "PxRigidDynamic::setContactReportThreshold: invalid float.");
	PX_CHECK_AND_RETURN(threshold >= 0.0f, "PxRigidDynamic::setContactReportThreshold: Force threshold must be greater than zero!");

	getScbBodyFast().setContactReportThreshold(threshold<0 ? 0 : threshold);
}


PxReal NpRigidDynamic::getContactReportThreshold() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getContactReportThreshold();
}


PxU32 physx::NpRigidDynamicGetShapes(Scb::Body& body, void* const*& shapes, bool* isCompound)
{
	NpRigidDynamic* a = static_cast<NpRigidDynamic*>(body.getScBody().getPxActor());
	NpShapeManager& sm = a->getShapeManager();
	shapes = reinterpret_cast<void *const *>(sm.getShapes());
	if(isCompound)
		*isCompound = sm.isSqCompound();
	return sm.getNbShapes();
}


void NpRigidDynamic::switchToNoSim()
{
	getScbBodyFast().switchBodyToNoSim();
}


void NpRigidDynamic::switchFromNoSim()
{
	getScbBodyFast().switchFromNoSim(true);
}


void NpRigidDynamic::wakeUpInternalNoKinematicTest(Scb::Body& body, bool forceWakeUp, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_ASSERT(scene);
	PxReal wakeCounterResetValue = scene->getWakeCounterResetValueInteral();

	PxReal wakeCounter = body.getWakeCounter();

	bool needsWakingUp = body.isSleeping() && (autowake || forceWakeUp);
	if (autowake && (wakeCounter < wakeCounterResetValue))
	{
		wakeCounter = wakeCounterResetValue;
		needsWakingUp = true;
	}

	if (needsWakingUp)
		body.wakeUpInternal(wakeCounter);
}

PxRigidDynamicLockFlags NpRigidDynamic::getRigidDynamicLockFlags() const
{
	return mBody.getLockFlags();
}
void NpRigidDynamic::setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags)
{
	mBody.setLockFlags(flags);
}
void NpRigidDynamic::setRigidDynamicLockFlag(PxRigidDynamicLockFlag::Enum flag, bool value)
{
	PxRigidDynamicLockFlags flags = mBody.getLockFlags();
	if (value)
		flags = flags | flag;
	else
		flags = flags & (~flag);

	mBody.setLockFlags(flags);
}


#if PX_ENABLE_DEBUG_VISUALIZATION
void NpRigidDynamic::visualize(Cm::RenderOutput& out, NpScene* npScene)
{
	NpRigidDynamicT::visualize(out, npScene);

	if (getScbBodyFast().getActorFlags() & PxActorFlag::eVISUALIZATION)
	{
		PX_ASSERT(npScene);
		const PxReal scale = npScene->getVisualizationParameter(PxVisualizationParameter::eSCALE);

		const PxReal massAxes = scale * npScene->getVisualizationParameter(PxVisualizationParameter::eBODY_MASS_AXES);
		if (massAxes != 0.0f)
		{
			PxReal sleepTime = getScbBodyFast().getWakeCounter() / npScene->getWakeCounterResetValueInteral();
			PxU32 color = PxU32(0xff * (sleepTime>1.0f ? 1.0f : sleepTime));
			color = getScbBodyFast().isSleeping() ? 0xff0000 : (color<<16 | color<<8 | color);
			PxVec3 dims = invertDiagInertia(getScbBodyFast().getInverseInertia());
			dims = getDimsFromBodyInertia(dims, 1.0f / getScbBodyFast().getInverseMass());

			out << color << getScbBodyFast().getBody2World() << Cm::DebugBox(dims * 0.5f);
		}
	}
}
#endif
