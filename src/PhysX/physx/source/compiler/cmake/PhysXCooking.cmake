##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2018-2019 NVIDIA Corporation. All rights reserved.

#
# Build PhysXCooking common
#
SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxcooking/src)

SET(PHYSXCOOKING_LINK_FLAGS_DEBUG " ")
SET(PHYSXCOOKING_LINK_FLAGS_CHECKED " ")
SET(PHYSXCOOKING_LINK_FLAGS_PROFILE " ")
SET(PHYSXCOOKING_LINK_FLAGS_RELEASE " ")

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXCooking.cmake)


SET(PHYSX_COOKING_HEADERS
	${PHYSX_ROOT_DIR}/include/cooking/PxBVH33MidphaseDesc.h
	${PHYSX_ROOT_DIR}/include/cooking/PxBVH34MidphaseDesc.h
	${PHYSX_ROOT_DIR}/include/cooking/Pxc.h
	${PHYSX_ROOT_DIR}/include/cooking/PxConvexMeshDesc.h
	${PHYSX_ROOT_DIR}/include/cooking/PxCooking.h
	${PHYSX_ROOT_DIR}/include/cooking/PxMidphaseDesc.h
	${PHYSX_ROOT_DIR}/include/cooking/PxTriangleMeshDesc.h
	${PHYSX_ROOT_DIR}/include/cooking/PxBVHStructureDesc.h
)
SOURCE_GROUP(include FILES ${PHYSX_COOKING_HEADERS})

SET(PHYSX_COOKING_SOURCE
	${LL_SOURCE_DIR}/Adjacencies.cpp
	${LL_SOURCE_DIR}/Cooking.cpp
	${LL_SOURCE_DIR}/CookingUtils.cpp
	${LL_SOURCE_DIR}/EdgeList.cpp
	${LL_SOURCE_DIR}/MeshCleaner.cpp
	${LL_SOURCE_DIR}/Quantizer.cpp	
	${LL_SOURCE_DIR}/Adjacencies.h
	${LL_SOURCE_DIR}/Cooking.h
	${LL_SOURCE_DIR}/CookingUtils.h
	${LL_SOURCE_DIR}/EdgeList.h
	${LL_SOURCE_DIR}/MeshCleaner.h
	${LL_SOURCE_DIR}/Quantizer.h
	${LL_SOURCE_DIR}/BVHStructureBuilder.cpp
	${LL_SOURCE_DIR}/BVHStructureBuilder.h
)
SOURCE_GROUP(src FILES ${PHYSX_COOKING_SOURCE})

SET(PHYSX_COOKING_MESH_SOURCE
	${LL_SOURCE_DIR}/mesh/HeightFieldCooking.cpp
	${LL_SOURCE_DIR}/mesh/RTreeCooking.cpp
	${LL_SOURCE_DIR}/mesh/MeshBuilder.cpp	
	${LL_SOURCE_DIR}/mesh/TriangleMeshBuilder.cpp
	${LL_SOURCE_DIR}/mesh/GrbTriangleMeshCooking.cpp
	${LL_SOURCE_DIR}/mesh/GrbTriangleMeshCooking.h
	${LL_SOURCE_DIR}/mesh/HeightFieldCooking.h
	${LL_SOURCE_DIR}/mesh/QuickSelect.h
	${LL_SOURCE_DIR}/mesh/RTreeCooking.h
	${LL_SOURCE_DIR}/mesh/MeshBuilder.h
	${LL_SOURCE_DIR}/mesh/TriangleMeshBuilder.h
)
SOURCE_GROUP(src\\mesh FILES ${PHYSX_COOKING_MESH_SOURCE})


SET(PHYSX_COOKING_CONVEX_SOURCE
	${LL_SOURCE_DIR}/convex/BigConvexDataBuilder.cpp
	${LL_SOURCE_DIR}/convex/ConvexHullBuilder.cpp
	${LL_SOURCE_DIR}/convex/ConvexHullLib.cpp
	${LL_SOURCE_DIR}/convex/ConvexHullUtils.cpp
	${LL_SOURCE_DIR}/convex/ConvexMeshBuilder.cpp
	${LL_SOURCE_DIR}/convex/ConvexPolygonsBuilder.cpp	
	${LL_SOURCE_DIR}/convex/QuickHullConvexHullLib.cpp
	${LL_SOURCE_DIR}/convex/VolumeIntegration.cpp
	${LL_SOURCE_DIR}/convex/BigConvexDataBuilder.h
	${LL_SOURCE_DIR}/convex/ConvexHullBuilder.h
	${LL_SOURCE_DIR}/convex/ConvexHullLib.h
	${LL_SOURCE_DIR}/convex/ConvexHullUtils.h
	${LL_SOURCE_DIR}/convex/ConvexMeshBuilder.h
	${LL_SOURCE_DIR}/convex/ConvexPolygonsBuilder.h	
	${LL_SOURCE_DIR}/convex/QuickHullConvexHullLib.h
	${LL_SOURCE_DIR}/convex/VolumeIntegration.h
)
SOURCE_GROUP(src\\convex FILES ${PHYSX_COOKING_CONVEX_SOURCE})

ADD_LIBRARY(PhysXCooking ${PHYSXCOOKING_LIBTYPE} 
	${PHYSX_COOKING_HEADERS}
	
	${PHYSX_COOKING_SOURCE}
	${PHYSX_COOKING_MESH_SOURCE}
	${PHYSX_COOKING_CONVEX_SOURCE}
	
	${PHYSXCOOKING_PLATFORM_SRC_FILES}
)

INSTALL(FILES ${PHYSX_COOKING_HEADERS} DESTINATION include/cooking)

# Target specific compile options

TARGET_INCLUDE_DIRECTORIES(PhysXCooking 
	PRIVATE ${PHYSXCOOKING_PLATFORM_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/cooking
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/include
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/common
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/convex
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/distance
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/sweep
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/gjk
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/hf
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/pcm
	PUBLIC ${PHYSX_SOURCE_DIR}/geomutils/src/ccd

	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src/convex

	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
	
)

TARGET_LINK_LIBRARIES(PhysXCooking PUBLIC PhysXCommon PhysXFoundation)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(PhysXCooking 
	PRIVATE ${PHYSXCOOKING_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXCooking PROPERTIES
	OUTPUT_NAME PhysXCooking
)


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSXCOOKING_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXCooking PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXCooking_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXCooking_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXCooking_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXCooking_static"
	)
ENDIF()

IF(PHYSXCOOKING_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXCooking PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXCOOKING_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXCOOKING_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXCOOKING_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXCOOKING_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

SET_TARGET_PROPERTIES(PhysXCooking PROPERTIES 
	LINK_FLAGS ${PHYSXCOOKING_LINK_FLAGS}
	LINK_FLAGS_DEBUG ${PHYSXCOOKING_LINK_FLAGS_DEBUG}
	LINK_FLAGS_CHECKED ${PHYSXCOOKING_LINK_FLAGS_CHECKED}
	LINK_FLAGS_PROFILE ${PHYSXCOOKING_LINK_FLAGS_PROFILE}
	LINK_FLAGS_RELEASE ${PHYSXCOOKING_LINK_FLAGS_RELEASE}
)

IF(PX_GENERATE_SOURCE_DISTRO)	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COOKING_HEADERS})	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COOKING_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COOKING_MESH_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COOKING_CONVEX_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXCOOKING_PLATFORM_SRC_FILES})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXCooking PROPERTIES POSITION_INDEPENDENT_CODE TRUE)


