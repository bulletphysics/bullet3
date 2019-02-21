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


#ifndef PX_COOKING_H
#define PX_COOKING_H
/** \addtogroup cooking
@{
*/
#include "common/PxPhysXCommonConfig.h"
#include "common/PxTolerancesScale.h"
#include "cooking/Pxc.h"

#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "cooking/PxMidphaseDesc.h"
#include "cooking/PxBVHStructureDesc.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxBVHStructure.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPhysicsInsertionCallback;
class PxFoundation;

/**
\brief Result from convex cooking.
*/
struct PxConvexMeshCookingResult
{
	enum Enum
	{
		/**
		\brief Convex mesh cooking succeeded.
		*/
		eSUCCESS,

		/**
		\brief Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.

		@see PxCookingParams::areaTestEpsilon PxConvexFlag::eCHECK_ZERO_AREA_TRIANGLES
		*/
		eZERO_AREA_TEST_FAILED,

		/**
		\brief Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
			The produced hull does not contain all input vertices. Try to simplify the input vertices
			or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.

		@see PxConvexFlag::eINFLATE_CONVEX PxConvexFlag::eQUANTIZE_INPUT
		*/
		ePOLYGONS_LIMIT_REACHED,

		/**
		\brief Something unrecoverable happened. Check the error stream to find out what.
		*/
		eFAILURE
	};
};

/** \brief Enumeration for convex mesh cooking algorithms. */
struct PxConvexMeshCookingType
{
	enum Enum
	{
		/**
		\brief The Quickhull algorithm constructs the hull from the given input points. The resulting hull
		will only contain a subset of the input points. 
		
		*/
		eQUICKHULL
	};
};

/**
\brief Result from triangle mesh cooking
*/
struct PxTriangleMeshCookingResult
{
	enum Enum
	{
		/**
		\brief Everything is A-OK.
		*/
		eSUCCESS			= 0,

		/**
		\brief a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
		*/
		eLARGE_TRIANGLE,

		/**
		\brief Something unrecoverable happened. Check the error stream to find out what.
		*/
		eFAILURE
	};
};

/**

\brief Enum for the set of mesh pre-processing parameters.

*/

struct PxMeshPreprocessingFlag
{
	enum Enum
	{
		/**
		\brief When set, mesh welding is performed. See PxCookingParams::meshWeldTolerance. Clean mesh must be enabled.
		*/
		eWELD_VERTICES					=	1 << 0, 

		/**
		\brief When set, mesh cleaning is disabled. This makes cooking faster.

		When clean mesh is not performed, mesh welding is also not performed. 

		It is recommended to use only meshes that passed during validateTriangleMesh. 

		*/
		eDISABLE_CLEAN_MESH								=	1 << 1, 

		/**
		\brief When set, active edges are set for each triangle edge. This makes cooking faster but slow up contact generation.
		*/
		eDISABLE_ACTIVE_EDGES_PRECOMPUTE				=	1 << 2,

		/**
		\brief When set, 32-bit indices will always be created regardless of triangle count.

		\note By default mesh will be created with 16-bit indices for triangle count <= 0xFFFF and 32-bit otherwise.
		*/
		eFORCE_32BIT_INDICES							=	1 << 3
	};
};

typedef PxFlags<PxMeshPreprocessingFlag::Enum,PxU32> PxMeshPreprocessingFlags;

/**

\brief Structure describing parameters affecting mesh cooking.

@see PxSetCookingParams() PxGetCookingParams()
*/
struct PxCookingParams
{
	/**
	\brief Zero-size area epsilon used in convex hull computation.

	If the area of a triangle of the hull is below this value, the triangle will be rejected. This test
	is done only if PxConvexFlag::eCHECK_ZERO_AREA_TRIANGLES is used.

	@see PxConvexFlag::eCHECK_ZERO_AREA_TRIANGLES

	<b>Default value:</b> 0.06f*PxTolerancesScale.length*PxTolerancesScale.length

	<b>Range:</b> (0.0f, PX_MAX_F32)
	*/
	float		areaTestEpsilon;

	/**
	\brief Plane tolerance used in convex hull computation.

	The value is used during hull construction. When a new point is about to be added to the hull it
	gets dropped when the point is closer to the hull than the planeTolerance. The planeTolerance
	is increased according to the hull size. 

	If 0.0f is set all points are accepted when the convex hull is created. This may lead to edge cases
	where the new points may be merged into an existing polygon and the polygons plane equation might 
	slightly change therefore. This might lead to failures during polygon merging phase in the hull computation.

	It is recommended to use the default value, however if it is required that all points needs to be
	accepted or huge thin convexes are created, it might be required to lower the default value.

	\note The plane tolerance is used only within PxConvexMeshCookingType::eQUICKHULL algorithm.

	<b>Default value:</b> 0.0007f

	<b>Range:</b> <0.0f, PX_MAX_F32)
	*/
	float		planeTolerance;

	/**
	\brief Convex hull creation algorithm.

	<b>Default value:</b> PxConvexMeshCookingType::eQUICKHULL

	@see PxConvexMeshCookingType
	*/
	PxConvexMeshCookingType::Enum convexMeshCookingType;

	/**
	\brief When true, the face remap table is not created.  This saves a significant amount of memory, but the SDK will
		not be able to provide the remap information for internal mesh triangles returned by collisions, 
		sweeps or raycasts hits.

	<b>Default value:</b> false
	*/
	bool		suppressTriangleMeshRemapTable;

	/**
	\brief When true, the triangle adjacency information is created. You can get the adjacency triangles
	for a given triangle from getTriangle.

	<b>Default value:</b> false
	*/
	bool		buildTriangleAdjacencies;

	/**
	\brief When true, addigional information required for GPU-accelerated rigid body simulation is created. This can increase memory usage and cooking times for convex meshes and triangle meshes.

	<b>Default value:</b> false
	*/
	bool		buildGPUData;

	/**
	\brief Tolerance scale is used to check if cooked triangles are not too huge. This check will help with simulation stability.

	\note The PxTolerancesScale values have to match the values used when creating a PxPhysics or PxScene instance.

	@see PxTolerancesScale
	*/
	PxTolerancesScale scale;

	/**
	\brief Mesh pre-processing parameters. Used to control options like whether the mesh cooking performs vertex welding before cooking.

	<b>Default value:</b> 0
	*/
	PxMeshPreprocessingFlags	meshPreprocessParams;

	/**
	\brief Mesh weld tolerance. If mesh welding is enabled, this controls the distance at which vertices are welded.
	If mesh welding is not enabled, this value defines the acceptance distance for mesh validation. Provided no two vertices are within this distance, the mesh is considered to be
	clean. If not, a warning will be emitted. Having a clean, welded mesh is required to achieve the best possible performance.

	The default vertex welding uses a snap-to-grid approach. This approach effectively truncates each vertex to integer values using meshWeldTolerance.
	Once these snapped vertices are produced, all vertices that snap to a given vertex on the grid are remapped to reference a single vertex. Following this,
	all triangles' indices are remapped to reference this subset of clean vertices. It should be noted that	the vertices that we do not alter the
	position of the vertices; the snap-to-grid is only performed to identify nearby vertices.

	The mesh validation approach also uses the same snap-to-grid approach to identify nearby vertices. If more than one vertex snaps to a given grid coordinate,
	we ensure that the distance between the vertices is at least meshWeldTolerance. If this is not the case, a warning is emitted.

	<b>Default value:</b> 0.0
	*/
	PxReal		meshWeldTolerance;

	/**
	\brief Controls the desired midphase desc structure for triangle meshes.

	@see PxBVH33MidphaseDesc, PxBVH34MidphaseDesc, PxMidphaseDesc

	<b>Default value:</b> PxMeshMidPhase::eBVH33
	*/
	PxMidphaseDesc midphaseDesc;

	/**
	\brief Vertex limit beyond which additional acceleration structures are computed for each convex mesh. Increase that limit to reduce memory usage.
	Computing the extra structures all the time does not guarantee optimal performance. There is a per-platform break-even point below which the
	extra structures actually hurt performance.

	<b>Default value:</b> 32
	*/
	PxU32	gaussMapLimit;

	PxCookingParams(const PxTolerancesScale& sc):
		areaTestEpsilon					(0.06f*sc.length*sc.length),
		planeTolerance					(0.0007f),
		convexMeshCookingType			(PxConvexMeshCookingType::eQUICKHULL),
		suppressTriangleMeshRemapTable	(false),
		buildTriangleAdjacencies		(false),
		buildGPUData					(false),
		scale							(sc),
		meshPreprocessParams			(0),
		meshWeldTolerance				(0.f),
		gaussMapLimit					(32)
	{
	}
};

class PxCooking
{
public:
	/**
	\brief Closes this instance of the interface.

	This function should be called to cleanly shut down the Cooking library before application exit.

	\note This function is required to be called to release foundation usage.

	*/
	virtual void  release() = 0;

	/**
	\brief Sets cooking parameters

	\param[in] params Cooking parameters

	@see getParams()
	*/
	virtual void  setParams(const PxCookingParams& params) = 0;

	/**
	\brief Gets cooking parameters

	\return Current cooking parameters.

	@see PxCookingParams setParams()
	*/
	virtual const PxCookingParams& getParams() const = 0;

	/**
	\brief Checks endianness is the same between cooking & target platforms

	\return True if there is and endian mismatch.
	*/
	virtual bool  platformMismatch() const = 0;

	/**
	\brief Cooks a triangle mesh. The results are written to the stream.

	To create a triangle mesh object it is necessary to first 'cook' the mesh data into
	a form which allows the SDK to perform efficient collision detection.

	cookTriangleMesh() allows a mesh description to be cooked into a binary stream
	suitable for loading and performing collision detection at runtime.

	\param[in] desc The triangle mesh descriptor to read the mesh from.
	\param[in] stream User stream to output the cooked data.
	\param[out] condition Result from triangle mesh cooking.
	\return true on success

	@see cookConvexMesh() setParams() PxPhysics.createTriangleMesh() PxTriangleMeshCookingResult::Enum
	*/
	virtual bool  cookTriangleMesh(const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition = NULL) const = 0;


	/**
	\brief Cooks and creates a triangle mesh and inserts it into PxPhysics.

	\note PxPhysicsInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback().

	\param[in] desc The triangle mesh descriptor to read the mesh from.
	\param[in] insertionCallback The insertion interface from PxPhysics.
	\param[out] condition Result from triangle mesh cooking.
	\return PxTriangleMesh pointer on success.	

	@see cookTriangleMesh() setParams() PxPhysics.createTriangleMesh() PxPhysicsInsertionCallback
	*/
	virtual PxTriangleMesh*    createTriangleMesh(const PxTriangleMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition = NULL) const = 0;

	/**
	\brief Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.

	The following conditions are true for a valid triangle mesh:
	1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    2. There are no large triangles (within specified PxTolerancesScale.)

	\param[in] desc The triangle mesh descriptor to read the mesh from.

	\return true if all the validity conditions hold, false otherwise.

	@see cookTriangleMesh()
	*/
	virtual bool  validateTriangleMesh(const PxTriangleMeshDesc& desc) const = 0;


	/**
	\brief Cooks a convex mesh. The results are written to the stream.

	To create a triangle mesh object it is necessary to first 'cook' the mesh data into
	a form which allows the SDK to perform efficient collision detection.

	cookConvexMesh() allows a mesh description to be cooked into a binary stream
	suitable for loading and performing collision detection at runtime.

	\note The number of vertices and the number of convex polygons in a cooked convex mesh is limited to 255.
	\note If those limits are exceeded in either the user-provided data or the final cooked mesh, an error is reported.

	\param[in] desc The convex mesh descriptor to read the mesh from.
	\param[in] stream User stream to output the cooked data.
	\param[out] condition Result from convex mesh cooking.
	\return true on success.

	@see cookTriangleMesh() setParams() PxConvexMeshCookingResult::Enum
	*/
	virtual bool  cookConvexMesh(const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition = NULL) const = 0;

	/**
	\brief Cooks and creates a convex mesh and inserts it into PxPhysics.

	\note This method does the same as cookConvexMesh, but the produced convex mesh is not stored
	into a stream but is directly inserted in PxPhysics. Use this method if you are unable to cook offline.

	\note PxPhysicsInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback().

	\param[in] desc The convex mesh descriptor to read the mesh from.
	\param[in] insertionCallback The insertion interface from PxPhysics.
	\param[out] condition Result from convex mesh cooking.
	\return PxConvexMesh pointer on success	

	@see cookConvexMesh() setParams() PxPhysicsInsertionCallback
	*/
	virtual PxConvexMesh*    createConvexMesh(const PxConvexMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition = NULL) const = 0;

	/**
	\brief Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.

	The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.	

	\note This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.

	\param[in] desc The convex mesh descriptor to read the mesh from.

	\return true if all the validity conditions hold, false otherwise.

	@see cookConvexMesh()
	*/
	virtual bool  validateConvexMesh(const PxConvexMeshDesc& desc) const = 0;


	/**
	\brief Computed hull polygons from given vertices and triangles. Polygons are needed for PxConvexMeshDesc rather than triangles.

	Please note that the resulting polygons may have different number of vertices. Some vertices may be removed. 
	The output vertices, indices and polygons must be used to construct a hull.

	The provided PxAllocatorCallback does allocate the out array's. It is the user responsibility to deallocated those
	array's.

	\param[in] mesh Simple triangle mesh containing vertices and triangles used to compute polygons.
	\param[in] inCallback Memory allocator for out array allocations.
	\param[out] nbVerts Number of vertices used by polygons.
	\param[out] vertices Vertices array used by polygons.
	\param[out] nbIndices Number of indices used by polygons.
	\param[out] indices Indices array used by polygons.
	\param[out] nbPolygons Number of created polygons.
	\param[out] hullPolygons Polygons array.
	\return true on success

	@see cookConvexMesh() PxConvexFlags PxConvexMeshDesc PxSimpleTriangleMesh
	*/
	virtual bool  computeHullPolygons(const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback, PxU32& nbVerts, PxVec3*& vertices,
											PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons) const = 0;

	/**
	\brief Cooks a heightfield. The results are written to the stream.

	To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.

	cookHeightField() allows a heightfield description to be cooked into a binary stream
	suitable for loading and performing collision detection at runtime.

	\param[in] desc The heightfield descriptor to read the HF from.
	\param[in] stream User stream to output the cooked data.
	\return true on success

	@see PxPhysics.createHeightField()
	*/
	virtual bool  cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream) const = 0;

	/**
	\brief Cooks and creates a heightfield mesh and inserts it into PxPhysics.

	\param[in] desc The heightfield descriptor to read the HF from.
	\param[in] insertionCallback The insertion interface from PxPhysics.
	\return PxHeightField pointer on success

	@see cookConvexMesh() setParams() PxPhysics.createTriangleMesh() PxPhysicsInsertionCallback
	*/
	virtual PxHeightField*    createHeightField(const PxHeightFieldDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const = 0;

	/**
	\brief Cooks a bounding volume hierarchy structure. The results are written to the stream.

	cookBVHStructure() allows a BVH structure description to be cooked into a binary stream
	suitable for loading and performing BVH detection at runtime.

	\param[in] desc The BVH structure descriptor.
	\param[in] stream User stream to output the cooked data.
	\return true on success.

	@see PxBVHStructure PxRigidActorExt::getRigidActorShapeLocalBoundsList
	*/
	virtual bool  cookBVHStructure(const PxBVHStructureDesc& desc, PxOutputStream& stream) const = 0;

	/**
	\brief Cooks and creates a bounding volume hierarchy structure and inserts it into PxPhysics.

	\note This method does the same as cookBVHStructure, but the produced BVH structure is not stored
	into a stream but is directly inserted in PxPhysics. Use this method if you are unable to cook offline.

	\note PxPhysicsInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback().

	\param[in] desc The BVH structure descriptor.
	\param[in] insertionCallback The insertion interface from PxPhysics.
	\return PxBVHStructure pointer on success	

	@see cookBVHStructure() PxPhysicsInsertionCallback
	*/
	virtual PxBVHStructure*    createBVHStructure(const PxBVHStructureDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const = 0;
protected:
	virtual ~PxCooking(){}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Create an instance of the cooking interface.

Note that the foundation object is handled as an application-wide singleton in statically linked executables
and a DLL-wide singleton in dynamically linked executables. Therefore, if you are using the runtime SDK in the
same executable as cooking, you should pass the Physics's copy of foundation (acquired with
PxPhysics::getFoundation()) to the cooker. This will also ensure correct handling of memory for objects
passed from the cooker to the SDK.

To use cooking in standalone mode, create an instance of the Foundation object with PxCreateCookingFoundation.
You should pass the same foundation object to all instances of the cooking interface.

\param[in] version the SDK version number
\param[in] foundation the foundation object associated with this instance of the cooking interface.
\param[in] params the parameters for this instance of the cooking interface
\return true on success.
*/
PX_C_EXPORT PX_PHYSX_COOKING_API physx::PxCooking* PX_CALL_CONV PxCreateCooking(physx::PxU32 version,
																				physx::PxFoundation& foundation,
																				const physx::PxCookingParams& params);

/** @} */
#endif
