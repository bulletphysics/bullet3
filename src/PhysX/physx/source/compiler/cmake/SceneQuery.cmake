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
## Copyright (c) 2018 NVIDIA Corporation. All rights reserved.

#
# Build SceneQuery common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/scenequery/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/SceneQuery.cmake)


SET(SCENEQUERY_BASE_DIR ${PHYSX_ROOT_DIR}/source/scenequery)
SET(SCENEQUERY_HEADERS		
	${SCENEQUERY_BASE_DIR}/include/SqPruner.h
	${SCENEQUERY_BASE_DIR}/include/SqPrunerMergeData.h
	${SCENEQUERY_BASE_DIR}/include/SqPruningStructure.h
	${SCENEQUERY_BASE_DIR}/include/SqSceneQueryManager.h	
)
SOURCE_GROUP(include FILES ${SCENEQUERY_HEADERS})

SET(SCENEQUERY_SOURCE			
	${SCENEQUERY_BASE_DIR}/src/SqAABBPruner.cpp
	${SCENEQUERY_BASE_DIR}/src/SqAABBPruner.h
	${SCENEQUERY_BASE_DIR}/src/SqAABBTree.cpp
	${SCENEQUERY_BASE_DIR}/src/SqAABBTree.h
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBPruner.cpp
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBPruner.h
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBPrunerCore.cpp
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBPrunerCore.h
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBTree.cpp
	${SCENEQUERY_BASE_DIR}/src/SqIncrementalAABBTree.h
	${SCENEQUERY_BASE_DIR}/src/SqAABBTreeUpdateMap.cpp
	${SCENEQUERY_BASE_DIR}/src/SqAABBTreeUpdateMap.h
	${SCENEQUERY_BASE_DIR}/src/SqBounds.cpp
	${SCENEQUERY_BASE_DIR}/src/SqBounds.h
	${SCENEQUERY_BASE_DIR}/src/SqCompoundPruner.cpp
	${SCENEQUERY_BASE_DIR}/src/SqCompoundPruner.h	
	${SCENEQUERY_BASE_DIR}/src/SqCompoundPruningPool.cpp
	${SCENEQUERY_BASE_DIR}/src/SqCompoundPruningPool.h	
	${SCENEQUERY_BASE_DIR}/src/SqBucketPruner.cpp
	${SCENEQUERY_BASE_DIR}/src/SqBucketPruner.h
	${SCENEQUERY_BASE_DIR}/src/SqExtendedBucketPruner.cpp
	${SCENEQUERY_BASE_DIR}/src/SqExtendedBucketPruner.h
	${SCENEQUERY_BASE_DIR}/src/SqMetaData.cpp
	${SCENEQUERY_BASE_DIR}/src/SqPruningPool.cpp
	${SCENEQUERY_BASE_DIR}/src/SqPruningPool.h
	${SCENEQUERY_BASE_DIR}/src/SqPruningStructure.cpp
	${SCENEQUERY_BASE_DIR}/src/SqSceneQueryManager.cpp
	${SCENEQUERY_BASE_DIR}/src/SqTypedef.h
)
SOURCE_GROUP(src FILES ${SCENEQUERY_SOURCE})

ADD_LIBRARY(SceneQuery ${SCENEQUERY_LIBTYPE}
	${SCENEQUERY_HEADERS}
	${SCENEQUERY_SOURCE}
)

# Target specific compile options

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(SceneQuery 
	PRIVATE ${SCENEQUERY_PLATFORM_INCLUDES}

	PRIVATE ${PHYSXFOUNDATION_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/pvd
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
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

	PRIVATE ${PHYSX_SOURCE_DIR}/scenequery/include
	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/buffering
	
	PRIVATE ${PHYSX_SOURCE_DIR}/pvd/include
)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(SceneQuery 

	# Common to all configurations
	PRIVATE ${SCENEQUERY_COMPILE_DEFS}
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND SCENEQUERY_LIBTYPE STREQUAL "STATIC")	
	SET_TARGET_PROPERTIES(SceneQuery PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "SceneQuery_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "SceneQuery_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "SceneQuery_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "SceneQuery_static"
	)
ENDIF()

IF(SCENEQUERY_COMPILE_PDB_NAME_DEBUG)	
	SET_TARGET_PROPERTIES(SceneQuery PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${SCENEQUERY_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${SCENEQUERY_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${SCENEQUERY_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${SCENEQUERY_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)			
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SCENEQUERY_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SCENEQUERY_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(SceneQuery PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
