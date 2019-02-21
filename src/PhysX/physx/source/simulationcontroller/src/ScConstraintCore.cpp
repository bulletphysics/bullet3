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


#include "PsFoundation.h"

#include "ScPhysics.h"
#include "ScBodyCore.h"
#include "ScConstraintCore.h"
#include "ScConstraintSim.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ConstraintCore::ConstraintCore(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)
:	mFlags(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES)
,	mAppliedForce(PxVec3(0))
,	mAppliedTorque(PxVec3(0))
,	mConnector(&connector)
,	mProject(shaders.project)
,	mSolverPrep(shaders.solverPrep)
,	mVisualize(shaders.visualize)
,	mDataSize(dataSize)
,	mLinearBreakForce(PX_MAX_F32)
,	mAngularBreakForce(PX_MAX_F32)
,	mMinResponseThreshold(0)
,	mSim(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ConstraintCore::~ConstraintCore()
{
}

void Sc::ConstraintCore::setFlags(PxConstraintFlags flags)
{
	PxConstraintFlags old = mFlags;
	flags = flags | (old & PxConstraintFlag::eGPU_COMPATIBLE);
	if(flags != old)
	{		
		mFlags = flags;
		if(getSim())
			getSim()->postFlagChange(old, flags);
	}
}

void Sc::ConstraintCore::getForce(PxVec3& force, PxVec3& torque) const
{
	if(!mSim)
	{
		force = PxVec3(0,0,0);
		torque = PxVec3(0,0,0);
	}
	else
		mSim->getForce(force, torque);

}

void Sc::ConstraintCore::setBodies(RigidCore* r0v, RigidCore* r1v)
{
	if(mSim)
		mSim->postBodiesChange(r0v, r1v);
}

bool Sc::ConstraintCore::updateConstants(void* addr)
{
	if (getSim())
	{
		getSim()->setConstantsLL(addr);
		return true;
	}
	return false;
}

void Sc::ConstraintCore::setBreakForce(PxReal linear, PxReal angular)
{
	mLinearBreakForce = linear;
	mAngularBreakForce = angular;

	if (getSim())
		getSim()->setBreakForceLL(linear, angular);
}

void Sc::ConstraintCore::getBreakForce(PxReal& linear, PxReal& angular) const
{
	linear = mLinearBreakForce;
	angular = mAngularBreakForce;
}

void Sc::ConstraintCore::setMinResponseThreshold(PxReal threshold)
{
	mMinResponseThreshold = threshold;

	if (getSim())
		getSim()->setMinResponseThresholdLL(threshold);
}

PxConstraint* Sc::ConstraintCore::getPxConstraint()
{
	return gOffsetTable.convertScConstraint2Px(this);
}

const PxConstraint* Sc::ConstraintCore::getPxConstraint() const
{
	return gOffsetTable.convertScConstraint2Px(this);
}

void Sc::ConstraintCore::breakApart()
{
	// TODO: probably want to do something with the interaction here
	// as well as remove the constraint from LL.

	mFlags |= PxConstraintFlag::eBROKEN;
}

void Sc::ConstraintCore::prepareForSetBodies()
{
	if(mSim)
		mSim->preBodiesChange();
}
