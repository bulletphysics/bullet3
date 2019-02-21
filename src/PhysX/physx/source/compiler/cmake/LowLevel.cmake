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
# Build LowLevel common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/lowlevel)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/LowLevel.cmake)


SET(LL_API_DIR ${LL_SOURCE_DIR}/api/)
SET(LL_API_HEADERS	
	${LL_API_DIR}/include/PxsMaterialCore.h
	${LL_API_DIR}/include/PxsMaterialManager.h
	${LL_API_DIR}/include/PxvConfig.h
	${LL_API_DIR}/include/PxvDynamics.h
	${LL_API_DIR}/include/PxvGeometry.h
	${LL_API_DIR}/include/PxvGlobals.h
	${LL_API_DIR}/include/PxvManager.h
	${LL_API_DIR}/include/PxvSimStats.h
)
SOURCE_GROUP("API Includes" FILES ${LL_API_HEADERS})

SET(LL_API_SOURCE
	${LL_API_DIR}/src/px_globals.cpp
)
SOURCE_GROUP("API Source" FILES ${LL_API_SOURCE})

SET(LL_COMMON_DIR ${LL_SOURCE_DIR}/common/)
SET(LL_COMMON_COLLISION_HEADERS	
	${LL_COMMON_DIR}/include/collision/PxcContactMethodImpl.h
)
SOURCE_GROUP("Common Includes\\collision" FILES ${LL_COMMON_COLLISION_HEADERS})
SET(LL_COMMON_PIPELINE_HEADERS		
	${LL_COMMON_DIR}/include/pipeline/PxcConstraintBlockStream.h
	${LL_COMMON_DIR}/include/pipeline/PxcContactCache.h
	${LL_COMMON_DIR}/include/pipeline/PxcMaterialMethodImpl.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpBatch.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpCache.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpCacheStreamPair.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpContactPrepShared.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpMemBlockPool.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpThreadContext.h
	${LL_COMMON_DIR}/include/pipeline/PxcNpWorkUnit.h
	${LL_COMMON_DIR}/include/pipeline/PxcRigidBody.h
)
SOURCE_GROUP("Common Includes\\pipeline" FILES ${LL_COMMON_PIPELINE_HEADERS})
SET(LL_COMMON_UTILS_HEADERS	
	${LL_COMMON_DIR}/include/utils/PxcScratchAllocator.h
	${LL_COMMON_DIR}/include/utils/PxcThreadCoherentCache.h
)
SOURCE_GROUP("Common Includes\\utils" FILES ${LL_COMMON_UTILS_HEADERS})

SET(LL_COMMON_COLLISION_SOURCE
	${LL_COMMON_DIR}/src/collision/PxcContact.cpp	
)
SOURCE_GROUP("Common Source\\collision" FILES ${LL_COMMON_COLLISION_SOURCE})
SET(LL_COMMON_PIPELINE_SOURCE	
	${LL_COMMON_DIR}/src/pipeline/PxcContactCache.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcContactMethodImpl.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcMaterialHeightField.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcMaterialMesh.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcMaterialMethodImpl.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcMaterialShape.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcNpBatch.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcNpCacheStreamPair.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcNpContactPrepShared.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcNpMemBlockPool.cpp
	${LL_COMMON_DIR}/src/pipeline/PxcNpThreadContext.cpp
)
SOURCE_GROUP("Common Source\\pipeline" FILES ${LL_COMMON_PIPELINE_SOURCE})

SET(LL_SOFTWARE_DIR ${LL_SOURCE_DIR}/software/)
SET(LL_SOFTWARE_HEADERS		
	${LL_SOFTWARE_DIR}/include/PxsBodySim.h
	${LL_SOFTWARE_DIR}/include/PxsCCD.h
	${LL_SOFTWARE_DIR}/include/PxsContactManager.h
	${LL_SOFTWARE_DIR}/include/PxsContactManagerState.h
	${LL_SOFTWARE_DIR}/include/PxsContext.h
	${LL_SOFTWARE_DIR}/include/PxsDefaultMemoryManager.h
	${LL_SOFTWARE_DIR}/include/PxsHeapMemoryAllocator.h
	${LL_SOFTWARE_DIR}/include/PxsIncrementalConstraintPartitioning.h
	${LL_SOFTWARE_DIR}/include/PxsIslandManagerTypes.h
	${LL_SOFTWARE_DIR}/include/PxsIslandSim.h
	${LL_SOFTWARE_DIR}/include/PxsIslandNodeIndex.h
	${LL_SOFTWARE_DIR}/include/PxsKernelWrangler.h
	${LL_SOFTWARE_DIR}/include/PxsMaterialCombiner.h
	${LL_SOFTWARE_DIR}/include/PxsMemoryManager.h
	${LL_SOFTWARE_DIR}/include/PxsNphaseImplementationContext.h
	${LL_SOFTWARE_DIR}/include/PxsRigidBody.h
	${LL_SOFTWARE_DIR}/include/PxsShapeSim.h
	${LL_SOFTWARE_DIR}/include/PxsSimpleIslandManager.h
	${LL_SOFTWARE_DIR}/include/PxsSimulationController.h
	${LL_SOFTWARE_DIR}/include/PxsTransformCache.h
	${LL_SOFTWARE_DIR}/include/PxvNphaseImplementationContext.h
)
SOURCE_GROUP("Software Includes" FILES ${LL_SOFTWARE_HEADERS})
SET(LL_SOFTWARE_SOURCE			
	${LL_SOFTWARE_DIR}/src/PxsCCD.cpp
	${LL_SOFTWARE_DIR}/src/PxsContactManager.cpp
	${LL_SOFTWARE_DIR}/src/PxsContext.cpp
	${LL_SOFTWARE_DIR}/src/PxsDefaultMemoryManager.cpp
	${LL_SOFTWARE_DIR}/src/PxsIslandSim.cpp
	${LL_SOFTWARE_DIR}/src/PxsMaterialCombiner.cpp
	${LL_SOFTWARE_DIR}/src/PxsNphaseImplementationContext.cpp
	${LL_SOFTWARE_DIR}/src/PxsSimpleIslandManager.cpp
)
SOURCE_GROUP("Software Source" FILES ${LL_SOFTWARE_SOURCE})

ADD_LIBRARY(LowLevel ${LOWLEVEL_LIBTYPE}
	${LL_API_HEADERS}
	${LL_API_SOURCE}
	
	${LL_COMMON_COLLISION_HEADERS}
	${LL_COMMON_COLLISION_SOURCE}
	
	${LL_COMMON_PIPELINE_HEADERS}
	${LL_COMMON_PIPELINE_SOURCE}
	
	${LL_COMMON_UTILS_HEADERS}
	
	${LL_SOFTWARE_HEADERS}
	${LL_SOFTWARE_SOURCE}	
)

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(LowLevel 
	PRIVATE ${LOWLEVEL_PLATFORM_INCLUDES}

	PRIVATE ${PHYSXFOUNDATION_INCLUDES}
	
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/common
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/convex
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/distance
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/sweep
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/gjk
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/hf
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/pcm
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/ccd
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/collision
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include

)

TARGET_COMPILE_DEFINITIONS(LowLevel 
	PRIVATE ${LOWLEVEL_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(LowLevel PROPERTIES
	LINK_FLAGS ${LOWLEVEL_PLATFORM_LINK_FLAGS}
)


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS)	
	SET_TARGET_PROPERTIES(LowLevel PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "LowLevel_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "LowLevel_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "LowLevel_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "LowLevel_static"
	)
ENDIF()

IF(LOWLEVEL_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(LowLevel PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${LOWLEVEL_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${LOWLEVEL_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${LOWLEVEL_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${LOWLEVEL_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_SOFTWARE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_SOFTWARE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_API_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_API_SOURCE})		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_COMMON_COLLISION_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_COMMON_COLLISION_SOURCE})		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_COMMON_PIPELINE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_COMMON_PIPELINE_SOURCE})		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LL_COMMON_UTILS_HEADERS})
ENDIF()

SET_TARGET_PROPERTIES(LowLevel PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
