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

#include "GuIntersectionCapsuleTriangle.h"
#include "GuDistancePointSegment.h"

using namespace physx;
using namespace Gu;

bool Gu::intersectCapsuleTriangle(const PxVec3& N, const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const Gu::Capsule& capsule, const CapsuleTriangleOverlapData& params)
{
	PX_ASSERT(capsule.p0!=capsule.p1);

	{
		const PxReal d2 = distancePointSegmentSquaredInternal(capsule.p0, params.mCapsuleDir, p0);
		if(d2<=capsule.radius*capsule.radius)
			return true;
	}

//	const PxVec3 N = (p0 - p1).cross(p0 - p2);

	if(!testAxis(p0, p1, p2, capsule, N))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p0, p1 - p0, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p1, p2 - p1, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	if(!testAxis(p0, p1, p2, capsule, computeEdgeAxis(p2, p0 - p2, capsule.p0, params.mCapsuleDir, params.mBDotB, params.mOneOverBDotB)))
		return false;

	return true;
}
