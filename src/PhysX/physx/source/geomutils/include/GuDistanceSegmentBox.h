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

#ifndef GU_DISTANCE_SEGMENT_BOX_H
#define GU_DISTANCE_SEGMENT_BOX_H

#include "foundation/PxMat33.h"
#include "GuSegment.h"
#include "GuBox.h"

namespace physx
{
namespace Gu
{

	//! Compute the smallest distance from the (finite) line segment to the box.
	PX_PHYSX_COMMON_API PxReal distanceSegmentBoxSquared(	const PxVec3& segmentPoint0, const PxVec3& segmentPoint1,
															const PxVec3& boxOrigin, const PxVec3& boxExtent, const PxMat33& boxBase,
															PxReal* segmentParam = NULL,
															PxVec3* boxParam = NULL);

	PX_FORCE_INLINE PxReal distanceSegmentBoxSquared(const Gu::Segment& segment, const Gu::Box& box, PxReal* t = NULL, PxVec3* p = NULL)
	{
		return distanceSegmentBoxSquared(segment.p0, segment.p1, box.center, box.extents, box.rot, t, p);
	}

} // namespace Gu

}

#endif
