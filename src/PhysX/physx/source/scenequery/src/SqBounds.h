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

#ifndef SQ_BOUNDS_H
#define SQ_BOUNDS_H

#include "CmPhysXCommon.h"
#include "foundation/PxBounds3.h"
#include "PsVecMath.h"

namespace physx
{
	namespace Scb
	{
		class Shape;
		class Actor;
	}

namespace Sq
{
	void computeStaticWorldAABB(PxBounds3& bounds, const Scb::Shape& scbShape, const Scb::Actor& scbActor);
	void computeDynamicWorldAABB(PxBounds3& bounds, const Scb::Shape& scbShape, const Scb::Actor& scbActor);

	typedef void(*ComputeBoundsFunc)	(PxBounds3& bounds, const Scb::Shape& scbShape, const Scb::Actor& scbActor);

	extern const ComputeBoundsFunc gComputeBoundsTable[2];

	// PT: TODO: - check that this is compatible with Gu::computeBounds(..., SQ_PRUNER_INFLATION, ...)
	// PT: TODO: - refactor with "inflateBounds" in GuBounds.cpp if possible
	// PT: TODO: - use SQ_PRUNER_INFLATION instead of hardcoding "0.01f"
	PX_FORCE_INLINE void inflateBounds(PxBounds3& dst, const PxBounds3& src)
	{
		using namespace physx::shdfnd::aos;

		const Vec4V minV = V4LoadU(&src.minimum.x);
		const Vec4V maxV = V4LoadU(&src.maximum.x);
		const Vec4V eV = V4Scale(V4Sub(maxV, minV), FLoad(0.5f * 0.01f));

		V4StoreU(V4Sub(minV, eV), &dst.minimum.x);
		PX_ALIGN(16, PxVec4) max4;
		V4StoreA(V4Add(maxV, eV), &max4.x);
		dst.maximum = PxVec3(max4.x, max4.y, max4.z);
	}

	// PT: the PX_MAX_BOUNDS_EXTENTS value is too large and produces INF floats when the box values are squared in
	// some collision routines. Thus, for the SQ subsystem we use this alternative (smaller) value to mark empty bounds.
	// See PX-954 for details.
	#define SQ_EMPTY_BOUNDS_EXTENTS	PxSqrt(0.25f * 1e33f)
}
}

#endif // SQ_BOUNDS_H
