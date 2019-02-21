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

#include "CmUtils.h"
#include "DySolverBody.h"
#include "PxsRigidBody.h"
#include "PxvDynamics.h"

using namespace physx;

// PT: TODO: SIMDify all this...
void Dy::copyToSolverBodyData(const PxVec3& linearVelocity, const PxVec3& angularVelocity, const PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
	const PxReal maxDepenetrationVelocity, const PxReal maxContactImpulse, const PxU32 nodeIndex, const PxReal reportThreshold, PxSolverBodyData& data, PxU32 lockFlags)
{
	data.nodeIndex = nodeIndex;

	const PxVec3 safeSqrtInvInertia = computeSafeSqrtInertia(invInertia);

	// PT: TODO: re-SIMDify this one
	const PxMat33 rotation(globalPose.q);

	Cm::transformInertiaTensor(safeSqrtInvInertia, rotation, data.sqrtInvInertia);

	// Copy simple properties
	data.linearVelocity = linearVelocity;
	data.angularVelocity = angularVelocity;

	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			data.linearVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			data.linearVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			data.linearVelocity.z = 0.f;

		//KS - technically, we can zero the inertia columns and produce stiffer constraints. However, this can cause numerical issues with the 
		//joint solver, which is fixed by disabling joint preprocessing and setting minResponseThreshold to some reasonable value > 0. However, until
		//this is handled automatically, it's probably better not to zero these inertia rows
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
		{
			data.angularVelocity.x = 0.f;
			//data.sqrtInvInertia.column0 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
		{
			data.angularVelocity.y = 0.f;
			//data.sqrtInvInertia.column1 = PxVec3(0.f);
		}
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
		{
			data.angularVelocity.z = 0.f;
			//data.sqrtInvInertia.column2 = PxVec3(0.f);
		}
	}

	PX_ASSERT(linearVelocity.isFinite());
	PX_ASSERT(angularVelocity.isFinite());

	data.invMass = invMass;
	data.penBiasClamp = maxDepenetrationVelocity;
	data.maxContactImpulse = maxContactImpulse;
	data.body2World = globalPose;

	data.lockFlags = PxU16(lockFlags);

	data.reportThreshold = reportThreshold;
}
