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


#ifndef PX_PHYSICS_GEOMUTILS_PX_MESH_QUERY
#define PX_PHYSICS_GEOMUTILS_PX_MESH_QUERY

/** \addtogroup geomutils
  @{
*/

#include "common/PxPhysXCommonConfig.h"
#include "PxQueryReport.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxGeometry;
class PxConvexMeshGeometry;
class PxTriangleMeshGeometry;
class PxHeightFieldGeometry;

class PxTriangle;

class PxMeshQuery
{
public:

	/**
	\brief Retrieves triangle data from a triangle ID.

	This function can be used together with #findOverlapTriangleMesh() to retrieve triangle properties.

	\param[in] triGeom Geometry of the triangle mesh to extract the triangle from.
	\param[in] transform Transform for the triangle mesh
	\param[in] triangleIndex The index of the triangle to retrieve.
	\param[out] triangle Triangle points in world space.
	\param[out] vertexIndices Returned vertex indices for given triangle
	\param[out] adjacencyIndices Returned 3 triangle adjacency internal face indices (0xFFFFFFFF if no adjacency). The mesh must be cooked with cooking param buildTriangleAdjacencies enabled.

	\note This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.

	@see PxTriangle PxTriangleFlags PxTriangleID findOverlapTriangleMesh()
	*/
	PX_PHYSX_COMMON_API static void getTriangle(const PxTriangleMeshGeometry& triGeom, const PxTransform& transform, PxTriangleID triangleIndex, PxTriangle& triangle, PxU32* vertexIndices=NULL, PxU32* adjacencyIndices=NULL);


	/**
	\brief Retrieves triangle data from a triangle ID.

	This function can be used together with #findOverlapHeightField() to retrieve triangle properties.

	\param[in] hfGeom Geometry of the height field to extract the triangle from.
	\param[in] transform Transform for the height field.
	\param[in] triangleIndex The index of the triangle to retrieve.
	\param[out] triangle Triangle points in world space.
	\param[out] vertexIndices Returned vertex indices for given triangle
	\param[out] adjacencyIndices Returned 3 triangle adjacency triangle indices (0xFFFFFFFF if no adjacency).

	\note This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
	\note TriangleIndex is an index used in internal format, which does have an index out of the bounds in last row.
			To traverse all tri indices in the HF, the following code can be applied:
			for (PxU32 row = 0; row < (nbRows - 1); row++)
			{
				for (PxU32 col = 0; col < (nbCols - 1); col++)
				{
					for (PxU32 k = 0; k < 2; k++)
					{ 
						const PxU32 triIndex = 2 * (row*nbCols + col) + k; 
						....
					}
				}
			}
	@see PxTriangle PxTriangleFlags PxTriangleID findOverlapHeightField()
	*/
	PX_PHYSX_COMMON_API static void getTriangle(const PxHeightFieldGeometry& hfGeom, const PxTransform& transform, PxTriangleID triangleIndex, PxTriangle& triangle, PxU32* vertexIndices=NULL, PxU32* adjacencyIndices=NULL);


	/**
	\brief Find the mesh triangles which touch the specified geometry object.

	Returned triangle indices can be used with #getTriangle() to retrieve the triangle properties.

	\param[in] geom The geometry object to test for mesh triangle overlaps. Supported geometries are #PxSphereGeometry, #PxCapsuleGeometry and #PxBoxGeometry
	\param[in] geomPose Pose of the geometry object
	\param[in] meshGeom The triangle mesh geometry to check overlap against
	\param[in] meshPose Pose of the triangle mesh
	\param[out] results Indices of overlapping triangles
	\param[in] maxResults Size of 'results' buffer
	\param[in] startIndex Index of first result to be retrieved. Previous indices are skipped.
	\param[out] overflow True if a buffer overflow occurred
	\return Number of overlaps found, i.e. number of elements written to the results buffer

	@see PxTriangleMeshGeometry getTriangle()
	*/
	PX_PHYSX_COMMON_API static PxU32 findOverlapTriangleMesh(	const PxGeometry& geom, const PxTransform& geomPose,
																const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose,
																PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow);

	/**
	\brief Find the height field triangles which touch the specified geometry object.

	Returned triangle indices can be used with #getTriangle() to retrieve the triangle properties.

	\param[in] geom The geometry object to test for height field overlaps. Supported geometries are #PxSphereGeometry, #PxCapsuleGeometry and #PxBoxGeometry. The sphere and capsule queries are currently conservative estimates.
	\param[in] geomPose Pose of the geometry object
	\param[in] hfGeom The height field geometry to check overlap against
	\param[in] hfPose Pose of the height field
	\param[out] results Indices of overlapping triangles
	\param[in] maxResults Size of 'results' buffer
	\param[in] startIndex Index of first result to be retrieved. Previous indices are skipped.
	\param[out] overflow True if a buffer overflow occurred
	\return Number of overlaps found, i.e. number of elements written to the results buffer

	@see PxHeightFieldGeometry getTriangle()
	*/
	PX_PHYSX_COMMON_API static PxU32 findOverlapHeightField(const PxGeometry& geom, const PxTransform& geomPose,
															const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose,
															PxU32* results, PxU32 maxResults, PxU32 startIndex, bool& overflow);


	/**
	\brief Sweep a specified geometry object in space and test for collision with a set of given triangles.

	This function simply sweeps input geometry against each input triangle, in the order they are given.
	This is an O(N) operation with N = number of input triangles. It does not use any particular acceleration structure.

	\param[in] unitDir Normalized direction of the sweep.
	\param[in] distance Sweep distance. Needs to be larger than 0. Clamped to PX_MAX_SWEEP_DISTANCE.
	\param[in] geom The geometry object to sweep. Supported geometries are #PxSphereGeometry, #PxCapsuleGeometry and #PxBoxGeometry
	\param[in] pose Pose of the geometry object to sweep.
	\param[in] triangleCount Number of specified triangles
	\param[in] triangles Array of triangles to sweep against
	\param[out] sweepHit The sweep hit information. See the notes below for limitations about returned results.
	\param[in] hitFlags Specification of the kind of information to retrieve on hit. Combination of #PxHitFlag flags. See the notes below for limitations about supported flags.
	\param[in] cachedIndex Cached triangle index for subsequent calls. Cached triangle is tested first. Optional parameter.
	\param[in] inflation This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
	\param[in] doubleSided Counterpart of PxMeshGeometryFlag::eDOUBLE_SIDED for input triangles.
	\return True if the swept geometry object hits the specified triangles

	\note Only the following geometry types are currently supported: PxSphereGeometry, PxCapsuleGeometry, PxBoxGeometry
	\note If a shape from the scene is already overlapping with the query shape in its starting position, the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
	\note This function returns a single closest hit across all the input triangles. Multiple hits are not supported.
	\note Supported hitFlags are PxHitFlag::eDEFAULT, PxHitFlag::eASSUME_NO_INITIAL_OVERLAP, PxHitFlag::ePRECISE_SWEEP, PxHitFlag::eMESH_BOTH_SIDES, PxHitFlag::eMESH_ANY.
	\note ePOSITION is only defined when there is no initial overlap (sweepHit.hadInitialOverlap() == false)
	\note The returned normal for initially overlapping sweeps is set to -unitDir.
	\note Otherwise the returned normal is the front normal of the triangle even if PxHitFlag::eMESH_BOTH_SIDES is set.
	\note The returned PxSweepHit::faceIndex parameter will hold the index of the hit triangle in input array, i.e. the range is [0; triangleCount). For initially overlapping sweeps, this is the index of overlapping triangle.
	\note The returned PxSweepHit::actor and PxSweepHit::shape pointers are not filled.
	\note The inflation parameter is not compatible with PxHitFlag::ePRECISE_SWEEP.

	@see PxTriangle PxSweepHit PxGeometry PxTransform
	*/
	PX_PHYSX_COMMON_API static bool sweep(const PxVec3& unitDir,
							const PxReal distance,
							const PxGeometry& geom,
							const PxTransform& pose,
							PxU32 triangleCount,
							const PxTriangle* triangles,
							PxSweepHit& sweepHit,
							PxHitFlags hitFlags = PxHitFlag::eDEFAULT,
							const PxU32* cachedIndex = NULL,
							const PxReal inflation = 0.0f,
							bool doubleSided = false);
};


#if !PX_DOXYGEN
}
#endif

/** @} */
#endif
