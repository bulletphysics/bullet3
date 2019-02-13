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

#ifndef GU_DISTANCE_SEGMENT_SEGMENT_H
#define GU_DISTANCE_SEGMENT_SEGMENT_H

#include "common/PxPhysXCommonConfig.h"
#include "GuSegment.h"

namespace physx
{
namespace Gu
{
	// This version fixes accuracy issues (e.g.  TTP 4617), but needs to do 2 square roots in order
	// to find the normalized direction and length of the segments, and then
	// a division in order to renormalize the output

	PX_PHYSX_COMMON_API	PxReal	distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& dir0, PxReal extent0,
																const PxVec3& origin1, const PxVec3& dir1, PxReal extent1,
																PxReal* s=NULL, PxReal* t=NULL);

	PX_PHYSX_COMMON_API PxReal	distanceSegmentSegmentSquared(	const PxVec3& origin0, const PxVec3& extent0,
																const PxVec3& origin1, const PxVec3& extent1,
																PxReal* s=NULL, PxReal* t=NULL);

	PX_FORCE_INLINE	PxReal	distanceSegmentSegmentSquared(	const Gu::Segment& segment0,
															const Gu::Segment& segment1,
															PxReal* s=NULL, PxReal* t=NULL)
	{
		return distanceSegmentSegmentSquared(	segment0.p0, segment0.computeDirection(),
												segment1.p0, segment1.computeDirection(),
												s, t);
	}

} // namespace Gu

}

#endif
