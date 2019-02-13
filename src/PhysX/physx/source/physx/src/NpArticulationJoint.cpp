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


#include "NpCast.h"
#include "NpWriteCheck.h"
#include "NpReadCheck.h"

namespace physx
{
// PX_SERIALIZATION
void NpArticulationJoint::resolveReferences(PxDeserializationContext& context)
{
	mImpl.resolveReferences(context);
}

NpArticulationJoint* NpArticulationJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationJoint* obj = new (address) NpArticulationJoint(PxBaseFlags(0));
	address += sizeof(NpArticulationJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
// ~PX_SERIALIZATION

NpArticulationJoint::NpArticulationJoint(NpArticulationLink& parent, 
										 const PxTransform& parentFrame,
										 NpArticulationLink& child, 
										 const PxTransform& childFrame) 
: NpArticulationJointTemplate(parent, parentFrame, child, childFrame)
{
}


NpArticulationJoint::~NpArticulationJoint()
{
}


void NpArticulationJoint::setTargetOrientation(const PxQuat& p)
{
	PX_CHECK_AND_RETURN(mImpl.getScbArticulationJoint().getDriveType() != PxArticulationJointDriveType::eTARGET || p.isUnit(), "NpArticulationJoint::setTargetOrientation, quat orientation is not valid.");
	PX_CHECK_AND_RETURN(mImpl.getScbArticulationJoint().getDriveType() != PxArticulationJointDriveType::eERROR || (p.getImaginaryPart().isFinite() && p.w == 0), "NpArticulationJoint::setTargetOrientation rotation vector orientation is not valid.");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTargetOrientation(p);
}


PxQuat NpArticulationJoint::getTargetOrientation() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTargetOrientation();
}


void NpArticulationJoint::setTargetVelocity(const PxVec3& v)
{
	PX_CHECK_AND_RETURN(v.isFinite(), "NpArticulationJoint::setTargetVelocity v is not valid.");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTargetVelocity(v);
}


PxArticulationJointDriveType::Enum	NpArticulationJoint::getDriveType() const
{
	NP_READ_CHECK(getOwnerScene());
	return mImpl.getScbArticulationJoint().getDriveType();
}

void  NpArticulationJoint::setDriveType(PxArticulationJointDriveType::Enum driveType)
{
	NP_WRITE_CHECK(getOwnerScene());
	mImpl.getScbArticulationJoint().setDriveType(driveType);
}


PxVec3 NpArticulationJoint::getTargetVelocity() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTargetVelocity();
}


void NpArticulationJoint::setStiffness(PxReal s)
{
	PX_CHECK_AND_RETURN(PxIsFinite(s) && s >= 0.0f, "PxArticulationJoint::setStiffness: spring coefficient must be >= 0!");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setStiffness(s);
}


PxReal NpArticulationJoint::getStiffness() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getStiffness();
}


void NpArticulationJoint::setDamping(PxReal d)
{
	PX_CHECK_AND_RETURN(PxIsFinite(d) && d >= 0.0f, "PxArticulationJoint::setDamping: damping coefficient must be >= 0!");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setDamping(d);
}


PxReal NpArticulationJoint::getDamping() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getDamping();
}


void NpArticulationJoint::setSwingLimitContactDistance(PxReal d)
{
	PX_CHECK_AND_RETURN(PxIsFinite(d) && d >= 0.0f, "PxArticulationJoint::setSwingLimitContactDistance: padding coefficient must be > 0!");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setSwingLimitContactDistance(d);
}


PxReal NpArticulationJoint::getSwingLimitContactDistance() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getSwingLimitContactDistance();
}


void NpArticulationJoint::setTwistLimitContactDistance(PxReal d)
{
	PX_CHECK_AND_RETURN(PxIsFinite(d) && d >= 0.0f, "PxArticulationJoint::setTwistLimitContactDistance: padding coefficient must be > 0!");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTwistLimitContactDistance(d);
}


PxReal NpArticulationJoint::getTwistLimitContactDistance() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTwistLimitContactDistance();
}

PxArticulationJointType::Enum NpArticulationJoint::getJointType() const
{
	NP_READ_CHECK(getOwnerScene());
	return mImpl.getScbArticulationJoint().getJointType();
}

void NpArticulationJoint::setJointType(PxArticulationJointType::Enum jointType)
{
	NP_WRITE_CHECK(getOwnerScene());
	mImpl.getScbArticulationJoint().setJointType(jointType);
}

void NpArticulationJoint::setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
{
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setMotion(axis, motion);

	reinterpret_cast<PxArticulationImpl*>(getChild().getArticulation().getImpl())->increaseCacheVersion();
}

PxArticulationMotion::Enum	NpArticulationJoint::getMotion(PxArticulationAxis::Enum axis) const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getMotion(axis);
}

void NpArticulationJoint::setFrictionCoefficient(const PxReal coefficient)
{
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setFrictionCoefficient(coefficient);

}

PxReal NpArticulationJoint::getFrictionCoefficient() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getFrictionCoefficient();
}

void NpArticulationJoint::setInternalCompliance(PxReal r)
{
	PX_CHECK_AND_RETURN(PxIsFinite(r) && r>0, "PxArticulationJoint::setInternalCompliance: compliance must be > 0");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setInternalCompliance(r);
}



PxReal NpArticulationJoint::getInternalCompliance() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getInternalCompliance();
}


void NpArticulationJoint::setExternalCompliance(PxReal r)
{
	PX_CHECK_AND_RETURN(PxIsFinite(r) && r>0, "PxArticulationJoint::setExternalCompliance: compliance must be > 0");
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setExternalCompliance(r);
}


PxReal NpArticulationJoint::getExternalCompliance() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getExternalCompliance();
}


void NpArticulationJoint::setSwingLimit(PxReal yLimit, PxReal zLimit)
{
	PX_CHECK_AND_RETURN(PxIsFinite(yLimit) && PxIsFinite(zLimit) && yLimit > 0 && zLimit > 0 && yLimit < PxPi && zLimit < PxPi, 
		"PxArticulationJoint::setSwingLimit: values must be >0 and < Pi");
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setSwingLimit(yLimit, zLimit);
}


void NpArticulationJoint::getSwingLimit(PxReal &yLimit, PxReal &zLimit) const
{
	NP_READ_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().getSwingLimit(yLimit, zLimit);
}


void NpArticulationJoint::setTangentialStiffness(PxReal stiffness)
{
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness) && stiffness >= 0, "PxArticulationJoint::setTangentialStiffness: stiffness must be > 0");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTangentialStiffness(stiffness);
}


PxReal NpArticulationJoint::getTangentialStiffness() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTangentialStiffness();
}


void NpArticulationJoint::setTangentialDamping(PxReal damping)
{
	PX_CHECK_AND_RETURN(PxIsFinite(damping) && damping >= 0, "PxArticulationJoint::setTangentialDamping: damping must be > 0");
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTangentialDamping(damping);
}


PxReal NpArticulationJoint::getTangentialDamping() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTangentialDamping();
}



void NpArticulationJoint::setSwingLimitEnabled(bool e)
{
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setSwingLimitEnabled(e);
}


bool NpArticulationJoint::getSwingLimitEnabled() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getSwingLimitEnabled();
}


void NpArticulationJoint::setTwistLimit(PxReal lower, PxReal upper)
{
	PX_CHECK_AND_RETURN(PxIsFinite(lower) && PxIsFinite(upper) && lower<upper && lower>-PxPi && upper < PxPi, "PxArticulationJoint::setTwistLimit: illegal parameters");

	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTwistLimit(lower, upper);
}


void NpArticulationJoint::getTwistLimit(PxReal &lower, PxReal &upper) const
{
	NP_READ_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().getTwistLimit(lower, upper);
}


void NpArticulationJoint::setTwistLimitEnabled(bool e)
{
	NP_WRITE_CHECK(getOwnerScene());

	mImpl.getScbArticulationJoint().setTwistLimitEnabled(e);
}


bool NpArticulationJoint::getTwistLimitEnabled() const
{
	NP_READ_CHECK(getOwnerScene());

	return mImpl.getScbArticulationJoint().getTwistLimitEnabled();
}


void NpArticulationJointGetBodiesFromScb(Scb::ArticulationJoint&c, Scb::Body*&b0, Scb::Body*&b1)
{
	NpArticulationJoint* np = const_cast<NpArticulationJoint*>(getNpArticulationJoint(&c));

	NpArticulationLink& a0 = np->getParent(), & a1 = np->getChild();
	b0 = &a0.getScbBodyFast();
	b1 = &a1.getScbBodyFast();
}


}
