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

#ifndef GU_SWEEP_TESTS_H
#define GU_SWEEP_TESTS_H

#include "CmPhysXCommon.h"
#include "PxQueryReport.h"
#include "PxGeometry.h"

namespace physx
{
	class PxConvexMeshGeometry;
	class PxCapsuleGeometry;
	class PxTriangle;
	class PxBoxGeometry;

	// PT: TODO: unify this with raycast calls (names and order of params)

	// PT: we use defines to be able to quickly change the signature of all sweep functions.
	// (this also ensures they all use consistent names for passed parameters).
	// \param[in]	geom		geometry object to sweep against
	// \param[in]	pose		pose of geometry object
	// \param[in]	unitDir		sweep's unit dir
	// \param[in]	distance	sweep's length/max distance
	// \param[out]	sweepHit	hit result
	// \param[in]	hitFlags	query behavior flags
	// \param[in]	inflation	optional inflation value for swept shape

	// PT: sweep parameters for capsule
	#define GU_CAPSULE_SWEEP_FUNC_PARAMS	const PxGeometry& geom, const PxTransform& pose,												\
											const PxCapsuleGeometry& capsuleGeom_, const PxTransform& capsulePose_, const Gu::Capsule& lss,	\
											const PxVec3& unitDir, PxReal distance,															\
											PxSweepHit& sweepHit, const PxHitFlags hitFlags, PxReal inflation 

	// PT: sweep parameters for box
	#define GU_BOX_SWEEP_FUNC_PARAMS	const PxGeometry& geom, const PxTransform& pose,								\
										const PxBoxGeometry& boxGeom_, const PxTransform& boxPose_, const Gu::Box& box,	\
										const PxVec3& unitDir, PxReal distance,											\
										PxSweepHit& sweepHit, const PxHitFlags hitFlags, PxReal inflation 

	// PT: sweep parameters for convex
	#define GU_CONVEX_SWEEP_FUNC_PARAMS		const PxGeometry& geom, const PxTransform& pose,						\
											const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,	\
											const PxVec3& unitDir, PxReal distance,									\
											PxSweepHit& sweepHit, const PxHitFlags hitFlags, PxReal inflation
	namespace Gu
	{
		class Capsule;
		class Box;

		// PT: function pointer for Geom-indexed capsule sweep functions
		// See GU_CAPSULE_SWEEP_FUNC_PARAMS for function parameters details.
		// \return		true if a hit was found, false otherwise
		typedef bool (*SweepCapsuleFunc)	(GU_CAPSULE_SWEEP_FUNC_PARAMS);

		// PT: function pointer for Geom-indexed box sweep functions
		// See GU_BOX_SWEEP_FUNC_PARAMS for function parameters details.
		// \return		true if a hit was found, false otherwise
		typedef bool (*SweepBoxFunc)		(GU_BOX_SWEEP_FUNC_PARAMS);

		// PT: function pointer for Geom-indexed box sweep functions
		// See GU_CONVEX_SWEEP_FUNC_PARAMS for function parameters details.
		// \return		true if a hit was found, false otherwise
		typedef bool (*SweepConvexFunc)		(GU_CONVEX_SWEEP_FUNC_PARAMS);

		// PT: typedef for bundles of all sweep functions, i.e. the function tables themselves (indexed by geom-type).
		typedef SweepCapsuleFunc	GeomSweepCapsuleTable	[PxGeometryType::eGEOMETRY_COUNT];
		typedef SweepBoxFunc		GeomSweepBoxTable		[PxGeometryType::eGEOMETRY_COUNT];
		typedef SweepConvexFunc		GeomSweepConvexTable	[PxGeometryType::eGEOMETRY_COUNT];

		struct GeomSweepFuncs
		{
			GeomSweepCapsuleTable	capsuleMap;
			GeomSweepCapsuleTable	preciseCapsuleMap;
			GeomSweepBoxTable		boxMap;
			GeomSweepBoxTable		preciseBoxMap;
			GeomSweepConvexTable	convexMap;
		};
		// PT: grabs all sweep function tables at once (for access by external non-Gu modules)
		PX_PHYSX_COMMON_API const GeomSweepFuncs& getSweepFuncTable();

		// PT: signature for sweep-vs-triangles functions.
		// We use defines to be able to quickly change the signature of all sweep functions.
		// (this also ensures they all use consistent names for passed parameters).
		// \param[in]	nbTris			number of triangles in input array
		// \param[in]	triangles		array of triangles to sweep the shape against
		// \param[in]	doubleSided		true if input triangles are double-sided
		// \param[in]	x				geom to sweep against input triangles
		// \param[in]	pose			pose of geom x
		// \param[in]	unitDir			sweep's unit dir
		// \param[in]	distance		sweep's length/max distance
		// \param[out]	hit				hit result
		// \param[in]	cachedIndex		optional initial triangle index (must be <nbTris)
		// \param[in]	inflation		optional inflation value for swept shape
		// \param[in]	hitFlags		query behavior flags
		#define GU_SWEEP_TRIANGLES_FUNC_PARAMS(x)	PxU32 nbTris, const PxTriangle* triangles, bool doubleSided,	\
													const x& geom, const PxTransform& pose,							\
													const PxVec3& unitDir, const PxReal distance,					\
													PxSweepHit& hit, const PxU32* cachedIndex,						\
													const PxReal inflation, PxHitFlags hitFlags

		bool sweepCapsuleTriangles		(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxCapsuleGeometry));
		bool sweepBoxTriangles			(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxBoxGeometry));
		bool sweepBoxTriangles_Precise	(GU_SWEEP_TRIANGLES_FUNC_PARAMS(PxBoxGeometry));

	}  // namespace Gu
}

#endif
