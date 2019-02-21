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

#ifndef GU_RAYCAST_TESTS_H
#define GU_RAYCAST_TESTS_H

#include "CmPhysXCommon.h"
#include "foundation/PxSimpleTypes.h"
#include "PxQueryReport.h"
#include "PxGeometry.h"

namespace physx
{
	// PT: TODO: why is PxHitFlag::eMESH_MULTIPLE used in the ray-vs-hf function, but not in the ray-vs-mesh function?

	// PT: we use a define to be able to quickly change the signature of all raycast functions.
	// (this also ensures they all use consistent names for passed parameters).
	// \param[in]	geom		geometry object to raycast against
	// \param[in]	pose		pose of geometry object
	// \param[in]	rayOrigin	ray's origin
	// \param[in]	rayDir		ray's unit dir
	// \param[in]	maxDist		ray's length/max distance
	// \param[in]	hitFlags	query behavior flags
	// \param[in]	maxHits		max number of hits = size of 'hits' buffer
	// \param[out]	hits		result buffer where to write raycast hits
	#define GU_RAY_FUNC_PARAMS	const PxGeometry& geom, const PxTransform& pose,					\
								const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,		\
								PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits
	namespace Gu
	{
		// PT: function pointer for Geom-indexed raycast functions
		// See GU_RAY_FUNC_PARAMS for function parameters details.
		// \return		number of hits written to 'hits' result buffer
		// \note		there's no mechanism to report overflow. Returned number of hits is just clamped to maxHits.
		typedef PxU32	(*RaycastFunc)		(GU_RAY_FUNC_PARAMS);

		// PT: typedef for a bundle of all raycast functions, i.e. the function table itself (indexed by geom-type).
		typedef RaycastFunc GeomRaycastTable[PxGeometryType::eGEOMETRY_COUNT];

		// PT: retrieves the raycast function table (for access by external non-Gu modules)
		PX_PHYSX_COMMON_API const GeomRaycastTable& getRaycastFuncTable();

	}  // namespace Gu
}

#endif
