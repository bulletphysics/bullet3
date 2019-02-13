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


#include "ScArticulationJointCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointSim.h"
#include "ScBodyCore.h"


using namespace physx;

Sc::ArticulationJointCore::ArticulationJointCore(const PxTransform& parentFrame,
												 const PxTransform& childFrame,
												 PxArticulationBase::Enum type) :
	mSim	(NULL)
{
	PX_ASSERT(parentFrame.isValid());
	PX_ASSERT(childFrame.isValid());

	mCore.parentPose = parentFrame;
	mCore.childPose = childFrame;

	mCore.targetPosition = PxQuat(PxIdentity);
	mCore.targetVelocity = PxVec3(0.f);

	mCore.driveType = PxArticulationJointDriveType::eTARGET;

	mCore.spring = 0.0f;
	mCore.damping = 0.0f;

	mCore.internalCompliance = 1.0f;
	mCore.externalCompliance = 1.0f;

	if (type == PxArticulationBase::eMaximumCoordinate)
	{
		PxReal swingYLimit = PxPi / 4;
		PxReal swingZLimit = PxPi / 4;
		mCore.limits[PxArticulationAxis::eSWING1].low = swingYLimit;
		mCore.limits[PxArticulationAxis::eSWING1].high = swingYLimit;
		mCore.limits[PxArticulationAxis::eSWING2].low = swingZLimit;
		mCore.limits[PxArticulationAxis::eSWING2].high = swingZLimit;
		mCore.swingLimitContactDistance = 0.05f;

		PxReal twistLimitLow = -PxPi / 4;
		PxReal twistLimitHigh = PxPi / 4;

		mCore.limits[PxArticulationAxis::eTWIST].low = twistLimitLow;
		mCore.limits[PxArticulationAxis::eTWIST].high = twistLimitHigh;
		mCore.twistLimitContactDistance = 0.05f;
		mCore.tanQSwingY = PxTan(swingYLimit / 4);
		mCore.tanQSwingZ = PxTan(swingZLimit / 4);
		mCore.tanQSwingPad = PxTan(mCore.swingLimitContactDistance / 4);

		mCore.tanQTwistHigh = PxTan(twistLimitHigh / 4);
		mCore.tanQTwistLow = PxTan(twistLimitLow / 4);
		mCore.tanQTwistPad = PxTan(mCore.twistLimitContactDistance / 4);
	}
	else
	{

		for (PxU32 i = 0; i < 6; ++i)
		{
			mCore.limits[i].low = 0.f;
			mCore.limits[i].high = 0.f;
			mCore.drives[i].stiffness = 0.f;
			mCore.drives[i].damping = 0.f;
			mCore.drives[i].maxForce = 0.f;
			mCore.targetP[i] = 0.f;
			mCore.targetV[i] = 0.f;
		}

		mCore.twistLimitContactDistance = 0.f;
		mCore.tanQSwingY = 0.f;
		mCore.tanQSwingZ = 0.f;
		mCore.tanQSwingPad = 0.f;

		mCore.tanQTwistHigh = 0.f;
		mCore.tanQTwistLow = 0.f;
		mCore.tanQTwistPad = 0.f;
	}

	mCore.swingLimited = false;
	mCore.twistLimited = false;
	mCore.prismaticLimited = false;
	mCore.tangentialStiffness = 0.0f;
	mCore.tangentialDamping = 0.0f;

	mCore.frictionCoefficient = 0.05f;

	mCore.jointType = PxArticulationJointType::eUNDEFINED;

	for(PxU32 i = 0; i < PxArticulationAxis::eCOUNT; ++i)
		mCore.motion[i] = PxArticulationMotion::eLOCKED;

}


Sc::ArticulationJointCore::~ArticulationJointCore()
{
	PX_ASSERT(getSim() == 0);
}


//--------------------------------------------------------------
//
// ArticulationJointCore interface implementation
//
//--------------------------------------------------------------


void Sc::ArticulationJointCore::setParentPose(const PxTransform& t)
{
	mCore.parentPose = t;
	mCore.dirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::ePOSE;
	mArticulation->setDirty(true);
}


void Sc::ArticulationJointCore::setChildPose(const PxTransform& t)
{
	mCore.childPose = t;
	mCore.dirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::ePOSE;
	mArticulation->setDirty(true);
}


void Sc::ArticulationJointCore::setTargetOrientation(const PxQuat& p)
{
	mCore.targetPosition = p;
}

void Sc::ArticulationJointCore::setTargetVelocity(const PxVec3& v)
{
	mCore.targetVelocity = v;
}

void Sc::ArticulationJointCore::setDriveType(PxArticulationJointDriveType::Enum type)
{
	mCore.driveType = PxU8(type);
}

void Sc::ArticulationJointCore::setJointType(PxArticulationJointType::Enum type)
{
	mCore.jointType = PxU8(type);
}

PxArticulationJointType::Enum Sc::ArticulationJointCore::getJointType() const 
{ 
	return PxArticulationJointType::Enum(mCore.jointType); 
}

void Sc::ArticulationJointCore::setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
{
	mCore.motion[axis] = motion;
	mCore.dirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eMOTION;
	mArticulation->setDirty(true);
}

PxArticulationMotion::Enum Sc::ArticulationJointCore::getMotion(PxArticulationAxis::Enum axis) const
{
	return PxArticulationMotion::Enum(PxU8(mCore.motion[axis]));
}

void Sc::ArticulationJointCore::setFrictionCoefficient(const PxReal coefficient)
{
	mCore.frictionCoefficient = coefficient;
}

PxReal Sc::ArticulationJointCore::getFrictionCoefficient() const
{
	return mCore.frictionCoefficient;
}

void Sc::ArticulationJointCore::setMaxJointVelocity(const PxReal maxJointV)
{
	mCore.maxJointVelocity = maxJointV;
}

PxReal Sc::ArticulationJointCore::getMaxJointVelocity() const
{
	return mCore.maxJointVelocity;
}

void Sc::ArticulationJointCore::setStiffness(PxReal s)
{
	mCore.spring = s;
}


void Sc::ArticulationJointCore::setDamping(PxReal d)
{
	mCore.damping = d;
}


void Sc::ArticulationJointCore::setInternalCompliance(PxReal r)
{
	mCore.internalCompliance = r;
}

void Sc::ArticulationJointCore::setExternalCompliance(PxReal r)
{
	mCore.externalCompliance = r;
}


void Sc::ArticulationJointCore::setSwingLimit(PxReal yLimit, PxReal zLimit)
{
	mCore.limits[PxArticulationAxis::eSWING1].low = yLimit;
	mCore.limits[PxArticulationAxis::eSWING2].low = zLimit;

	mCore.tanQSwingY			= PxTan(yLimit/4);
	mCore.tanQSwingZ			= PxTan(zLimit/4);
}


void Sc::ArticulationJointCore::setTangentialStiffness(PxReal s)
{
	mCore.tangentialStiffness = s;
}


void Sc::ArticulationJointCore::setTangentialDamping(PxReal d)
{
	mCore.tangentialDamping = d;
}


void Sc::ArticulationJointCore::setSwingLimitEnabled(bool e)
{
	mCore.swingLimited = e;
}

void Sc::ArticulationJointCore::setSwingLimitContactDistance(PxReal e)
{
	mCore.swingLimitContactDistance = e;
	mCore.tanQSwingPad = PxTan(e/4);
}


void Sc::ArticulationJointCore::setTwistLimit(PxReal lower, PxReal upper)
{
	mCore.limits[PxArticulationAxis::eTWIST].low = lower;
	mCore.limits[PxArticulationAxis::eTWIST].high = upper;

	mCore.tanQTwistHigh			= PxTan(upper/4);
	mCore.tanQTwistLow			= PxTan(lower/4);
}

void Sc::ArticulationJointCore::setTwistLimitEnabled(bool e)
{
	mCore.twistLimited = e;
}

void Sc::ArticulationJointCore::setTwistLimitContactDistance(PxReal e)
{
	mCore.twistLimitContactDistance = e;
	mCore.tanQTwistPad = PxTan(e/4);
}

void Sc::ArticulationJointCore::setTargetP(PxArticulationAxis::Enum axis, PxReal targetP)
{
	mCore.targetP[axis] = targetP;
	mCore.dirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eTARGETPOSE;
	mArticulation->setDirty(true);
}

void Sc::ArticulationJointCore::setTargetV(PxArticulationAxis::Enum axis, PxReal targetV)
{
	mCore.targetV[axis] = targetV;
	mCore.dirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eTARGETVELOCITY;
	mArticulation->setDirty(true);
}
