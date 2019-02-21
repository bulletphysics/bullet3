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

#ifndef GU_SWEEP_CAPSULE_TRIANGLE_H
#define GU_SWEEP_CAPSULE_TRIANGLE_H

#include "foundation/PxVec3.h"
#include "CmPhysXCommon.h"
#include "PxQueryReport.h"

namespace physx
{
	class PxTriangle;

namespace Gu
{
	class BoxPadded;
	class Capsule;

	/**
	Sweeps a capsule against a set of triangles.

	\param nbTris			[in] number of triangles in input array
	\param triangles		[in] array of input triangles
	\param capsule			[in] the capsule
	\param unitDir			[in] sweep's unit direcion
	\param distance			[in] sweep's length
	\param cachedIndex		[in] cached triangle index, or NULL. Cached triangle will be tested first.
	\param hit				[out] results
	\param triNormalOut		[out] triangle normal
	\param hitFlags			[in] query modifiers
	\param isDoubleSided	[in] true if input triangles are double-sided
	\param cullBox			[in] additional/optional culling box. Triangles not intersecting the box are quickly discarded.
	\warning	if using a cullbox, make sure all triangles can be safely V4Loaded (i.e. allocate 4 more bytes after last triangle)
	\return	true if an impact has been found
	*/
	bool sweepCapsuleTriangles_Precise(	PxU32 nbTris, const PxTriangle* PX_RESTRICT triangles,	// Triangle data
										const Capsule& capsule,									// Capsule data
										const PxVec3& unitDir, const PxReal distance,			// Ray data
										const PxU32* PX_RESTRICT cachedIndex,					// Cache data
										PxSweepHit& hit, PxVec3& triNormalOut,						// Results
										PxHitFlags hitFlags, bool isDoubleSided,				// Query modifiers
										const BoxPadded* cullBox=NULL);							// Cull data

} // namespace Gu

}

#endif
