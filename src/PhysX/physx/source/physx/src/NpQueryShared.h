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


#ifndef PX_PHYSICS_NP_QUERYSHARED
#define PX_PHYSICS_NP_QUERYSHARED

#include "foundation/PxMemory.h"

namespace physx
{

using namespace Cm;

PX_FORCE_INLINE bool applyFilterEquation(const Scb::Shape& scbShape, const PxFilterData& queryFd)
{
	// if the filterData field is non-zero, and the bitwise-AND value of filterData AND the shape's
	// queryFilterData is zero, the shape is skipped.
	if(queryFd.word0 | queryFd.word1 | queryFd.word2 | queryFd.word3)
	{
		const PxFilterData& objFd = scbShape.getScShape().getQueryFilterData();
		const PxU32 keep = (queryFd.word0 & objFd.word0) | (queryFd.word1 & objFd.word1) | (queryFd.word2 & objFd.word2) | (queryFd.word3 & objFd.word3);
		if(!keep)
			return false;
	}
	return true;
}

//========================================================================================================================
// these partial template specializations are used to generalize the query code to be reused for all permutations of
// hit type=(raycast, overlap, sweep) x query type=(ANY, SINGLE, MULTIPLE)
template <typename HitType> struct HitTypeSupport { enum { IsRaycast = 0, IsSweep = 0, IsOverlap = 0 }; };
template <> struct HitTypeSupport<PxRaycastHit>
{
	enum { IsRaycast = 1, IsSweep = 0, IsOverlap = 0 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit& hit) { return static_cast<const PxRaycastHit&>(hit).distance; }
};
template <> struct HitTypeSupport<PxSweepHit>
{
	enum { IsRaycast = 0, IsSweep = 1, IsOverlap = 0 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit& hit) { return static_cast<const PxSweepHit&>(hit).distance; }
};
template <> struct HitTypeSupport<PxOverlapHit>
{
	enum { IsRaycast = 0, IsSweep = 0, IsOverlap = 1 };
	static PX_FORCE_INLINE PxReal getDistance(const PxQueryHit&) { return -1.0f; }
};

#define HITDIST(hit) HitTypeSupport<HitType>::getDistance(hit)

template<typename HitType>
static PxU32 clipHitsToNewMaxDist(HitType* ppuHits, PxU32 count, PxReal newMaxDist)
{
	PxU32 i=0;
	while(i!=count)
	{
		if(HITDIST(ppuHits[i]) > newMaxDist)
			ppuHits[i] = ppuHits[--count];
		else
			i++;
	}
	return count;
}

} // namespace physx

#endif // PX_PHYSICS_NP_QUERYSHARED
