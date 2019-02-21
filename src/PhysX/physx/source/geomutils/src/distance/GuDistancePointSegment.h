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

#ifndef GU_DISTANCE_POINT_SEGMENT_H
#define GU_DISTANCE_POINT_SEGMENT_H

#include "PxPhysXCommonConfig.h"
#include "GuSegment.h"

namespace physx
{
namespace Gu
{
	// dir = p1 - p0
	PX_FORCE_INLINE PxReal distancePointSegmentSquaredInternal(const PxVec3& p0, const PxVec3& dir, const PxVec3& point, PxReal* param=NULL)
	{
		PxVec3 diff = point - p0;
		PxReal fT = diff.dot(dir);

		if(fT<=0.0f)
		{
			fT = 0.0f;
		}
		else
		{
			const PxReal sqrLen = dir.magnitudeSquared();
			if(fT>=sqrLen)
			{
				fT = 1.0f;
				diff -= dir;
			}
			else
			{
				fT /= sqrLen;
				diff -= fT*dir;
			}
		}

		if(param)
			*param = fT;

		return diff.magnitudeSquared();
	}

	/**
	A segment is defined by S(t) = mP0 * (1 - t) + mP1 * t, with 0 <= t <= 1
	Alternatively, a segment is S(t) = Origin + t * Direction for 0 <= t <= 1.
	Direction is not necessarily unit length. The end points are Origin = mP0 and Origin + Direction = mP1.
	*/
	PX_FORCE_INLINE PxReal distancePointSegmentSquared(const PxVec3& p0, const PxVec3& p1, const PxVec3& point, PxReal* param=NULL)
	{
		return distancePointSegmentSquaredInternal(p0, p1 - p0, point, param);
	}

	PX_INLINE PxReal distancePointSegmentSquared(const Gu::Segment& segment, const PxVec3& point, PxReal* param=NULL)
	{
		return distancePointSegmentSquared(segment.p0, segment.p1, point, param);
	}

} // namespace Gu

}

#endif
