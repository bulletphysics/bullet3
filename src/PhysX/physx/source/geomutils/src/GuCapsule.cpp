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

#include "PsIntrinsics.h"
#include "PsMathUtils.h"
#include "GuInternal.h"
#include "GuBox.h"
#include "GuCapsule.h"

using namespace physx;

/**
*	Computes an OBB surrounding the capsule.
*	\param		box		[out] the OBB
*/
void Gu::computeBoxAroundCapsule(const Gu::Capsule& capsule, Gu::Box& box)
{
	// Box center = center of the two capsule's endpoints
	box.center = capsule.computeCenter();

	// Box extents
	const PxF32 d = (capsule.p0 - capsule.p1).magnitude();
	box.extents.x = capsule.radius + (d * 0.5f);
	box.extents.y = capsule.radius;
	box.extents.z = capsule.radius;

	// Box orientation
	if(d==0.0f)
	{
		box.rot = PxMat33(PxIdentity);
	}
	else
	{
		PxVec3 dir, right, up;
		Ps::computeBasis(capsule.p0, capsule.p1, dir, right, up);
		box.setAxes(dir, right, up);
	}
}
