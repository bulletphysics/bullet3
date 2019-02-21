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

#ifndef GU_OVERLAP_TESTS_H
#define GU_OVERLAP_TESTS_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxAssert.h"
#include "CmPhysXCommon.h"
#include "PxGeometry.h"
#include "PsFoundation.h"

namespace physx
{
namespace Gu
{
	class Capsule;
	class Sphere;

	PX_PHYSX_COMMON_API bool checkOverlapAABB_triangleGeom		(const PxGeometry& triGeom,	const PxTransform& pose, const PxBounds3& box);
	PX_PHYSX_COMMON_API bool checkOverlapAABB_heightFieldGeom	(const PxGeometry& hfGeom,	const PxTransform& pose, const PxBounds3& box);

	// PT: this is just a shadow of what it used to be. We currently don't use TRIGGER_INSIDE anymore, but I leave it for now,
	// since I really want to put this back the way it was before.
	enum TriggerStatus
	{
		TRIGGER_DISJOINT,
		TRIGGER_INSIDE,
		TRIGGER_OVERLAP
	};

	// PT: currently only used for convex triggers
	struct TriggerCache
	{
		PxVec3	dir;
		PxU16	state;
		PxU16	gjkState; //gjk succeed or fail
	};

	// PT: we use a define to be able to quickly change the signature of all overlap functions.
	// (this also ensures they all use consistent names for passed parameters).
	// \param[in]	geom0	first geometry object
	// \param[in]	pose0	pose of first geometry object
	// \param[in]	geom1	second geometry object
	// \param[in]	pose1	pose of second geometry object
	// \param[in]	cache	optional cached data for triggers
	#define GU_OVERLAP_FUNC_PARAMS	const PxGeometry& geom0, const PxTransform& pose0,	\
									const PxGeometry& geom1, const PxTransform& pose1,	\
									Gu::TriggerCache* cache

	// PT: function pointer for Geom-indexed overlap functions
	// See GU_OVERLAP_FUNC_PARAMS for function parameters details.
	// \return		true if an overlap was found, false otherwise
	typedef bool (*GeomOverlapFunc)	(GU_OVERLAP_FUNC_PARAMS);

	// PT: typedef for a bundle of all overlap functions, i.e. the function table itself (indexed by geom-type).
	typedef GeomOverlapFunc GeomOverlapTable[PxGeometryType::eGEOMETRY_COUNT];

	// PT: retrieves the overlap function table (for access by external non-Gu modules)
	PX_PHYSX_COMMON_API const GeomOverlapTable* getOverlapFuncTable();

	// dynamic registration of height fields
	PX_PHYSX_COMMON_API void registerHeightFields();

	PX_FORCE_INLINE bool overlap(	const PxGeometry& geom0, const PxTransform& pose0,
									const PxGeometry& geom1, const PxTransform& pose1,
									const GeomOverlapTable* PX_RESTRICT overlapFuncs)
	{
		PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "Gu::overlap(): pose0 is not valid.", false);
		PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "Gu::overlap(): pose1 is not valid.", false);
		
		if(geom0.getType() > geom1.getType())
		{
			GeomOverlapFunc overlapFunc = overlapFuncs[geom1.getType()][geom0.getType()];
			PX_ASSERT(overlapFunc);
			return overlapFunc(geom1, pose1, geom0, pose0, NULL);
		}
		else
		{
			GeomOverlapFunc overlapFunc = overlapFuncs[geom0.getType()][geom1.getType()];
			PX_ASSERT(overlapFunc);
			return overlapFunc(geom0, pose0, geom1, pose1, NULL);
		}
	}

}  // namespace Gu

}

#endif
