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

#ifndef GU_MIDPHASE_INTERFACE_H
#define GU_MIDPHASE_INTERFACE_H

#include "GuOverlapTests.h"
#include "GuRaycastTests.h"
#include "GuTriangleMesh.h"
#include "PsVecMath.h"

// PT: this file contains the common interface for all midphase implementations. Specifically the Midphase namespace contains the
// midphase-related entry points, dispatching calls to the proper implementations depending on the triangle mesh's type. The rest of it
// is simply classes & structs shared by all implementations.

namespace physx
{
	class PxMeshScale;
	class PxTriangleMeshGeometry;
namespace Cm
{
	class Matrix34;
	class FastVertex2ShapeScaling;
}

namespace Gu
{
	struct ConvexHullData;

	struct CallbackMode { enum Enum { eANY, eCLOSEST, eMULTIPLE }; };

	template<typename HitType>
	struct MeshHitCallback
	{
		CallbackMode::Enum mode;

		MeshHitCallback(CallbackMode::Enum aMode) : mode(aMode) {}

		PX_FORCE_INLINE	bool inAnyMode()		const { return mode == CallbackMode::eANY;		}
		PX_FORCE_INLINE	bool inClosestMode()	const { return mode == CallbackMode::eCLOSEST;	}
		PX_FORCE_INLINE	bool inMultipleMode()	const { return mode == CallbackMode::eMULTIPLE;	}

		virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
			const HitType& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal& shrunkMaxT, const PxU32* vIndices) = 0;

		virtual ~MeshHitCallback() {}
	};

	struct SweepConvexMeshHitCallback;

	struct LimitedResults
	{
		PxU32*	mResults;
		PxU32	mNbResults;
		PxU32	mMaxResults;
		PxU32	mStartIndex;
		PxU32	mNbSkipped;
		bool	mOverflow;

		PX_FORCE_INLINE LimitedResults(PxU32* results, PxU32 maxResults, PxU32 startIndex)
			: mResults(results), mMaxResults(maxResults), mStartIndex(startIndex)
		{
			reset();
		}

		PX_FORCE_INLINE	void reset()
		{
			mNbResults	= 0;
			mNbSkipped	= 0;
			mOverflow	= false;
		}

		PX_FORCE_INLINE	bool add(PxU32 index)
		{
			if(mNbResults>=mMaxResults)
			{
				mOverflow = true;
				return false;
			}

			if(mNbSkipped>=mStartIndex)
				mResults[mNbResults++] = index;
			else
				mNbSkipped++;

			return true;
		}
	};

	// RTree forward declarations
	PX_PHYSX_COMMON_API PxU32 raycast_triangleMesh_RTREE(const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
									const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
									PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits);
	PX_PHYSX_COMMON_API bool intersectSphereVsMesh_RTREE(const Sphere& sphere,		const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API bool intersectBoxVsMesh_RTREE	(const Box& box,			const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API bool intersectCapsuleVsMesh_RTREE(const Capsule& capsule,	const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API void intersectOBB_RTREE(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned);
	PX_PHYSX_COMMON_API bool sweepCapsule_MeshGeom_RTREE(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
										const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance,
										PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	PX_PHYSX_COMMON_API bool sweepBox_MeshGeom_RTREE(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
									const Gu::Box& box, const PxVec3& unitDir, const PxReal distance,
									PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	PX_PHYSX_COMMON_API void sweepConvex_MeshGeom_RTREE(const TriangleMesh* mesh, const Gu::Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool anyHit);

#if PX_INTEL_FAMILY
	// BV4 forward declarations
	PX_PHYSX_COMMON_API PxU32 raycast_triangleMesh_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
									const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
									PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits);
	PX_PHYSX_COMMON_API bool intersectSphereVsMesh_BV4	(const Sphere& sphere,		const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API bool intersectBoxVsMesh_BV4		(const Box& box,			const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API bool intersectCapsuleVsMesh_BV4	(const Capsule& capsule,	const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	PX_PHYSX_COMMON_API void intersectOBB_BV4(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned);
	PX_PHYSX_COMMON_API bool sweepCapsule_MeshGeom_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
									const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance,
									PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	PX_PHYSX_COMMON_API bool sweepBox_MeshGeom_BV4(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
								const Gu::Box& box, const PxVec3& unitDir, const PxReal distance,
								PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	PX_PHYSX_COMMON_API void sweepConvex_MeshGeom_BV4(const TriangleMesh* mesh, const Gu::Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool anyHit);
#endif

	typedef PxU32 (*MidphaseRaycastFunction)(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
												const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
												PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits);

	typedef bool (*MidphaseSphereOverlapFunction)	(const Sphere& sphere,		const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	typedef bool (*MidphaseBoxOverlapFunction)		(const Box& box,			const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	typedef bool (*MidphaseCapsuleOverlapFunction)	(const Capsule& capsule,	const TriangleMesh& triMesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results);
	typedef void (*MidphaseBoxCBOverlapFunction)	(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned);

	typedef bool (*MidphaseCapsuleSweepFunction)(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
													const Gu::Capsule& lss, const PxVec3& unitDir, const PxReal distance,
													PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	typedef bool (*MidphaseBoxSweepFunction)(		const TriangleMesh* mesh, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
													const Gu::Box& box, const PxVec3& unitDir, const PxReal distance,
													PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation);
	typedef void (*MidphaseConvexSweepFunction)(	const TriangleMesh* mesh, const Gu::Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool anyHit);

namespace Midphase
{
	PX_FORCE_INLINE bool outputError()
	{
		static bool reportOnlyOnce = false;
		if(!reportOnlyOnce)
		{
			reportOnlyOnce = true;
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "BV4 midphase only supported on Intel platforms.");
		}
		return false;
	}
}

	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
	#else
	static PxU32 unsupportedMidphase(	const TriangleMesh*, const PxTriangleMeshGeometry&, const PxTransform&,
										const PxVec3&, const PxVec3&, PxReal,
										PxHitFlags, PxU32, PxRaycastHit* PX_RESTRICT)
	{
		return PxU32(Midphase::outputError());
	}
	static bool unsupportedSphereOverlapMidphase(const Sphere&, const TriangleMesh&, const PxTransform&, const PxMeshScale&, LimitedResults*)
	{
		return Midphase::outputError();
	}
	static bool unsupportedBoxOverlapMidphase(const Box&, const TriangleMesh&, const PxTransform&, const PxMeshScale&, LimitedResults*)
	{
		return Midphase::outputError();
	}
	static bool unsupportedCapsuleOverlapMidphase(const Capsule&, const TriangleMesh&, const PxTransform&, const PxMeshScale&, LimitedResults*)
	{
		return Midphase::outputError();
	}
	static void unsupportedBoxCBOverlapMidphase(const TriangleMesh*, const Box&, MeshHitCallback<PxRaycastHit>&, bool, bool)
	{
		Midphase::outputError();
	}
	static bool unsupportedBoxSweepMidphase(const TriangleMesh*, const PxTriangleMeshGeometry&, const PxTransform&, const Gu::Box&, const PxVec3&, const PxReal, PxSweepHit&, PxHitFlags, const PxReal)
	{
		return Midphase::outputError();
	}
	static bool unsupportedCapsuleSweepMidphase(const TriangleMesh*, const PxTriangleMeshGeometry&, const PxTransform&, const Gu::Capsule&, const PxVec3&, const PxReal, PxSweepHit&, PxHitFlags, const PxReal)
	{
		return Midphase::outputError();
	}
	static void unsupportedConvexSweepMidphase(const TriangleMesh*, const Gu::Box&, const PxVec3&, const PxReal, SweepConvexMeshHitCallback&, bool)
	{
		Midphase::outputError();
	}
	#endif

	static const MidphaseRaycastFunction	gMidphaseRaycastTable[PxMeshMidPhase::eLAST] =
	{
		raycast_triangleMesh_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		raycast_triangleMesh_BV4,
	#else
		unsupportedMidphase,
	#endif
	};

	static const MidphaseSphereOverlapFunction gMidphaseSphereOverlapTable[PxMeshMidPhase::eLAST] =
	{
		intersectSphereVsMesh_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		intersectSphereVsMesh_BV4,
	#else
		unsupportedSphereOverlapMidphase,
	#endif
	};

	static const MidphaseBoxOverlapFunction gMidphaseBoxOverlapTable[PxMeshMidPhase::eLAST] =
	{
		intersectBoxVsMesh_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		intersectBoxVsMesh_BV4,
	#else
		unsupportedBoxOverlapMidphase,
	#endif
	};

	static const MidphaseCapsuleOverlapFunction gMidphaseCapsuleOverlapTable[PxMeshMidPhase::eLAST] =
	{
		intersectCapsuleVsMesh_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		intersectCapsuleVsMesh_BV4,
	#else
		unsupportedCapsuleOverlapMidphase,
	#endif
	};

	static const MidphaseBoxCBOverlapFunction gMidphaseBoxCBOverlapTable[PxMeshMidPhase::eLAST] =
	{
		intersectOBB_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		intersectOBB_BV4,
	#else
		unsupportedBoxCBOverlapMidphase,
	#endif
	};

	static const MidphaseBoxSweepFunction gMidphaseBoxSweepTable[PxMeshMidPhase::eLAST] =
	{
		sweepBox_MeshGeom_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		sweepBox_MeshGeom_BV4,
	#else
		unsupportedBoxSweepMidphase,
	#endif
	};

	static const MidphaseCapsuleSweepFunction gMidphaseCapsuleSweepTable[PxMeshMidPhase::eLAST] =
	{
		sweepCapsule_MeshGeom_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		sweepCapsule_MeshGeom_BV4,
	#else
		unsupportedCapsuleSweepMidphase,
	#endif
	};

	static const MidphaseConvexSweepFunction gMidphaseConvexSweepTable[PxMeshMidPhase::eLAST] =
	{
		sweepConvex_MeshGeom_RTREE,
	#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		sweepConvex_MeshGeom_BV4,
	#else
		unsupportedConvexSweepMidphase,
	#endif
	};

namespace Midphase
{
	// \param[in]	mesh			triangle mesh to raycast against
	// \param[in]	meshGeom		geometry object associated with the mesh
	// \param[in]	meshTransform	pose/transform of geometry object
	// \param[in]	rayOrigin		ray's origin
	// \param[in]	rayDir			ray's unit dir
	// \param[in]	maxDist			ray's length/max distance
	// \param[in]	hitFlags		query behavior flags
	// \param[in]	maxHits			max number of hits = size of 'hits' buffer
	// \param[out]	hits			result buffer where to write raycast hits
	// \return		number of hits written to 'hits' result buffer
	// \note		there's no mechanism to report overflow. Returned number of hits is just clamped to maxHits.
	PX_FORCE_INLINE PxU32 raycastTriangleMesh(	const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshTransform,
												const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,
												PxHitFlags hitFlags, PxU32 maxHits, PxRaycastHit* PX_RESTRICT hits)
	{
		const PxU32 index = PxU32(mesh->getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseRaycastTable[index](mesh, meshGeom, meshTransform, rayOrigin, rayDir, maxDist, hitFlags, maxHits, hits);
	}

	// \param[in]	sphere			sphere
	// \param[in]	mesh			triangle mesh
	// \param[in]	meshTransform	pose/transform of triangle mesh
	// \param[in]	meshScale		mesh scale
	// \param[out]	results			results object if multiple hits are needed, NULL if a simple boolean answer is enough
	// \return 		true if at least one overlap has been found
	PX_FORCE_INLINE bool intersectSphereVsMesh(const Sphere& sphere, const TriangleMesh& mesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
	{
		const PxU32 index = PxU32(mesh.getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseSphereOverlapTable[index](sphere, mesh, meshTransform, meshScale, results);
	}

	// \param[in]	box				box
	// \param[in]	mesh			triangle mesh
	// \param[in]	meshTransform	pose/transform of triangle mesh
	// \param[in]	meshScale		mesh scale
	// \param[out]	results			results object if multiple hits are needed, NULL if a simple boolean answer is enough
	// \return 		true if at least one overlap has been found
	PX_FORCE_INLINE bool intersectBoxVsMesh(const Box& box, const TriangleMesh& mesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
	{
		const PxU32 index = PxU32(mesh.getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseBoxOverlapTable[index](box, mesh, meshTransform, meshScale, results);
	}

	// \param[in]	capsule			capsule
	// \param[in]	mesh			triangle mesh
	// \param[in]	meshTransform	pose/transform of triangle mesh
	// \param[in]	meshScale		mesh scale
	// \param[out]	results			results object if multiple hits are needed, NULL if a simple boolean answer is enough
	// \return 		true if at least one overlap has been found
	PX_FORCE_INLINE bool intersectCapsuleVsMesh(const Capsule& capsule, const TriangleMesh& mesh, const PxTransform& meshTransform, const PxMeshScale& meshScale, LimitedResults* results)
	{
		const PxU32 index = PxU32(mesh.getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseCapsuleOverlapTable[index](capsule, mesh, meshTransform, meshScale, results);
	}

	// \param[in]	mesh						triangle mesh
	// \param[in]	box							box
	// \param[in]	callback					callback object, called each time a hit is found
	// \param[in]	bothTriangleSidesCollide	true for double-sided meshes
	// \param[in]	checkObbIsAligned			true to use a dedicated codepath for axis-aligned boxes
	PX_FORCE_INLINE void intersectOBB(const TriangleMesh* mesh, const Box& obb, MeshHitCallback<PxRaycastHit>& callback, bool bothTriangleSidesCollide, bool checkObbIsAligned = true)
	{
		const PxU32 index = PxU32(mesh->getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		gMidphaseBoxCBOverlapTable[index](mesh, obb, callback, bothTriangleSidesCollide, checkObbIsAligned);
	}

	// \param[in]	mesh			triangle mesh
	// \param[in]	meshGeom		geometry object associated with the mesh
	// \param[in]	meshTransform	pose/transform of geometry object
	// \param[in]	capsule			swept capsule
	// \param[in]	unitDir			sweep's unit dir
	// \param[in]	distance		sweep's length/max distance
	// \param[out]	sweepHit		hit result
	// \param[in]	hitFlags		query behavior flags
	// \param[in]	inflation		optional inflation value for swept shape
	// \return		true if a hit was found, false otherwise
	PX_FORCE_INLINE bool sweepCapsuleVsMesh(const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshTransform,
											const Gu::Capsule& capsule, const PxVec3& unitDir, const PxReal distance,
											PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
	{
		const PxU32 index = PxU32(mesh->getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseCapsuleSweepTable[index](mesh, meshGeom, meshTransform, capsule, unitDir, distance, sweepHit, hitFlags, inflation);
	}

	// \param[in]	mesh			triangle mesh
	// \param[in]	meshGeom		geometry object associated with the mesh
	// \param[in]	meshTransform	pose/transform of geometry object
	// \param[in]	box				swept box
	// \param[in]	unitDir			sweep's unit dir
	// \param[in]	distance		sweep's length/max distance
	// \param[out]	sweepHit		hit result
	// \param[in]	hitFlags		query behavior flags
	// \param[in]	inflation		optional inflation value for swept shape
	// \return		true if a hit was found, false otherwise
	PX_FORCE_INLINE bool sweepBoxVsMesh(const TriangleMesh* mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshTransform,
										const Gu::Box& box, const PxVec3& unitDir, const PxReal distance,
										PxSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation)
	{
		const PxU32 index = PxU32(mesh->getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		return gMidphaseBoxSweepTable[index](mesh, meshGeom, meshTransform, box, unitDir, distance, sweepHit, hitFlags, inflation);
	}

	// \param[in]	mesh			triangle mesh
	// \param[in]	hullBox			hull's bounding box
	// \param[in]	localDir		sweep's unit dir, in local/mesh space
	// \param[in]	distance		sweep's length/max distance
	// \param[in]	callback		callback object, called each time a hit is found
	// \param[in]	anyHit			true for PxHitFlag::eMESH_ANY queries
	PX_FORCE_INLINE void sweepConvexVsMesh(const TriangleMesh* mesh, const Gu::Box& hullBox, const PxVec3& localDir, const PxReal distance, SweepConvexMeshHitCallback& callback, bool anyHit)
	{
		const PxU32 index = PxU32(mesh->getConcreteType() - PxConcreteType::eTRIANGLE_MESH_BVH33);
		gMidphaseConvexSweepTable[index](mesh, hullBox, localDir, distance, callback, anyHit);
	}
}
}
}
#endif	// GU_MIDPHASE_INTERFACE_H
