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


#ifndef DY_SOLVERBODY_H
#define DY_SOLVERBODY_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"
#include "CmPhysXCommon.h"
#include "CmSpatialVector.h"
#include "solver/PxSolverDefs.h"

namespace physx
{

class PxsRigidBody;
struct PxsBodyCore;

namespace Dy
{

// PT: TODO: make sure this is still needed / replace with V4sqrt
//This method returns values of 0 when the inertia is 0. This is a bit of a hack but allows us to 
//represent kinematic objects' velocities in our new format
PX_FORCE_INLINE PxVec3 computeSafeSqrtInertia(const PxVec3& v)
{
	return PxVec3(	v.x == 0.0f ? 0.0f : PxSqrt(v.x),
					v.y == 0.0f ? 0.0f : PxSqrt(v.y),
					v.z == 0.0f ? 0.0f : PxSqrt(v.z));
}

void copyToSolverBodyData(const PxVec3& linearVelocity, const PxVec3& angularVelocity, const PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
	const PxReal maxDepenetrationVelocity, const PxReal maxContactImpulse, const PxU32 nodeIndex, const PxReal reportThreshold, PxSolverBodyData& solverBodyData, PxU32 lockFlags);

// PT: TODO: using PxsBodyCore in the interface makes us write less data to the stack for passing arguments, and we can take advantage of the class layout
// (we know what is aligned or not, we know if it is safe to V4Load vectors, etc). Note that this is what we previously had, which is why PxsBodyCore was still
// forward-referenced above.
//void copyToSolverBodyData(PxSolverBodyData& solverBodyData, const PxsBodyCore& core, const PxU32 nodeIndex);

}

}

#endif //DY_SOLVERBODY_H
