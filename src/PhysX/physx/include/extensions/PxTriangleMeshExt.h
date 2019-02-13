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


#ifndef PX_PHYSICS_EXTENSIONS_TRIANGLE_MESH_H
#define PX_PHYSICS_EXTENSIONS_TRIANGLE_MESH_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"
#include "common/PxPhysXCommonConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxGeometry;
class PxTriangleMeshGeometry;
class PxHeightFieldGeometry;

	/**
	\brief Utility class to find mesh triangles touched by a specified geometry object.

	This class is a helper calling PxMeshQuery::findOverlapTriangleMesh or PxMeshQuery::findOverlapHeightField under the hood,
	while taking care of necessary memory management issues.

	PxMeshQuery::findOverlapTriangleMesh and PxMeshQuery::findOverlapHeightField are the "raw" functions operating on user-provided fixed-size
	buffers. These functions abort with an error code in case of buffer overflow. PxMeshOverlapUtil is a convenient helper function checking
	this error code, and resizing buffers appropriately, until the desired call succeeds.
	
	Returned triangle indices are stored within the class, and can be used with PxMeshQuery::getTriangle() to retrieve the triangle properties.
	*/
	class PxMeshOverlapUtil
	{
		public:
										PxMeshOverlapUtil();
										~PxMeshOverlapUtil();
	/**
	\brief Find the mesh triangles which touch the specified geometry object.

	\param[in] geom The geometry object to test for mesh triangle overlaps. Supported geometries are #PxSphereGeometry, #PxCapsuleGeometry and #PxBoxGeometry
	\param[in] geomPose Pose of the geometry object
	\param[in] meshGeom The triangle mesh geometry to check overlap against
	\param[in] meshPose Pose of the triangle mesh
	\return Number of overlaps found. Triangle indices can then be accessed through the #getResults() function.

	@see PxGeometry PxTransform PxTriangleMeshGeometry PxMeshQuery::findOverlapTriangleMesh
	*/
						PxU32			findOverlap(const PxGeometry& geom, const PxTransform& geomPose, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose);

	/**
	\brief Find the height field triangles which touch the specified geometry object.

	\param[in] geom The geometry object to test for height field overlaps. Supported geometries are #PxSphereGeometry, #PxCapsuleGeometry and #PxBoxGeometry. The sphere and capsule queries are currently conservative estimates.
	\param[in] geomPose Pose of the geometry object
	\param[in] hfGeom The height field geometry to check overlap against
	\param[in] hfPose Pose of the height field
	\return Number of overlaps found. Triangle indices can then be accessed through the #getResults() function.

	@see PxGeometry PxTransform PxHeightFieldGeometry PxMeshQuery::findOverlapHeightField
	*/
						PxU32			findOverlap(const PxGeometry& geom, const PxTransform& geomPose, const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose);

	/**
	\brief Retrieves array of triangle indices after a findOverlap call.
	\return Indices of touched triangles
	*/
		PX_FORCE_INLINE	const PxU32*	getResults()	const	{ return mResultsMemory;	}

	/**
	\brief Retrieves number of triangle indices after a findOverlap call.
	\return Number of touched triangles
	*/
		PX_FORCE_INLINE	PxU32			getNbResults()	const	{ return mNbResults;		}

		private:
						PxU32*			mResultsMemory;
						PxU32			mResults[256];
						PxU32			mNbResults;
						PxU32			mMaxNbResults;
	};

	/**
	\brief Computes an approximate minimum translational distance (MTD) between a geometry object and a mesh.

	This iterative function computes an approximate vector that can be used to depenetrate a geom object
	from a triangle mesh. Returned depenetration vector should be applied to 'geom', to get out of the mesh.

	The function works best when the amount of overlap between the geom object and the mesh is small. If the
	geom object's center goes inside the mesh, backface culling usually kicks in, no overlap is detected,
	and the function does not compute an MTD vector.

	The function early exits if no overlap is detected after a depenetration attempt. This means that if
	maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
	been successful. Usually N = 4 gives good results.

	\param[out] direction Computed MTD unit direction
	\param[out] depth Penetration depth. Always positive or zero.
	\param[in] geom The geometry object
	\param[in] geomPose Pose for the geometry object
	\param[in] meshGeom The mesh geometry
	\param[in] meshPose Pose for the mesh
	\param[in] maxIter Max number of iterations before returning.
	\param[out] usedIter Number of depenetrations attempts performed during the call. Will not be returned if the pointer is NULL.

	\return True if the MTD has successfully been computed, i.e. if objects do overlap.

	@see PxGeometry PxTransform PxTriangleMeshGeometry
	*/
	bool PxComputeTriangleMeshPenetration(PxVec3& direction, 
										  PxReal& depth,
										  const PxGeometry& geom, 
										  const PxTransform& geomPose, 
										  const PxTriangleMeshGeometry& meshGeom, 
										  const PxTransform& meshPose, 
										  PxU32 maxIter,
										  PxU32* usedIter = NULL);

	/**
	\brief Computes an approximate minimum translational distance (MTD) between a geometry object and a heightfield.

	This iterative function computes an approximate vector that can be used to depenetrate a geom object
	from a heightfield. Returned depenetration vector should be applied to 'geom', to get out of the heightfield.

	The function works best when the amount of overlap between the geom object and the mesh is small. If the
	geom object's center goes inside the heightfield, backface culling usually kicks in, no overlap is detected,
	and the function does not compute an MTD vector.

	The function early exits if no overlap is detected after a depenetration attempt. This means that if
	maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
	been successful. Usually N = 4 gives good results.

	\param[out] direction Computed MTD unit direction
	\param[out] depth Penetration depth. Always positive or zero.
	\param[in] geom The geometry object
	\param[in] geomPose Pose for the geometry object
	\param[in] heightFieldGeom The heightfield geometry
	\param[in] heightFieldPose Pose for the heightfield
	\param[in] maxIter Max number of iterations before returning.
	\param[out] usedIter Number of depenetrations attempts performed during the call. Will not be returned if the pointer is NULL.

	\return True if the MTD has successfully been computed, i.e. if objects do overlap.

	@see PxGeometry PxTransform PxHeightFieldGeometry
	*/
	bool PxComputeHeightFieldPenetration(PxVec3& direction, 
										 PxReal& depth,
										 const PxGeometry& geom, 
									     const PxTransform& geomPose, 
										 const PxHeightFieldGeometry& heightFieldGeom, 
										 const PxTransform& heightFieldPose,
										 PxU32 maxIter, 
										 PxU32* usedIter = NULL);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
