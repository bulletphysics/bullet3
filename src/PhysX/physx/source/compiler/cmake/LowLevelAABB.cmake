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
# Build LowLevelAABB common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LLAABB_DIR ${PHYSX_SOURCE_DIR}/lowlevelaabb)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/LowLevelAABB.cmake)


SET(LLAABB_HEADERS		
	${LLAABB_DIR}/include/BpAABBManager.h
	${LLAABB_DIR}/include/BpAABBManagerTasks.h
	${LLAABB_DIR}/include/BpBroadPhase.h
	${LLAABB_DIR}/include/BpBroadPhaseUpdate.h
)
SOURCE_GROUP("includes" FILES ${LLAABB_HEADERS})

SET(LLAABB_SOURCE	
	${LLAABB_DIR}/src/BpAABBManager.cpp
	${LLAABB_DIR}/src/BpBroadPhase.cpp
	${LLAABB_DIR}/src/BpBroadPhaseABP.cpp
	${LLAABB_DIR}/src/BpBroadPhaseABP.h
	${LLAABB_DIR}/src/BpBroadPhaseMBP.cpp
	${LLAABB_DIR}/src/BpBroadPhaseMBP.h
	${LLAABB_DIR}/src/BpBroadPhaseMBPCommon.h
	${LLAABB_DIR}/src/BpBroadPhaseSap.cpp
	${LLAABB_DIR}/src/BpBroadPhaseSap.h
	${LLAABB_DIR}/src/BpBroadPhaseSapAux.cpp
	${LLAABB_DIR}/src/BpBroadPhaseSapAux.h
	${LLAABB_DIR}/src/BpBroadPhaseShared.cpp
	${LLAABB_DIR}/src/BpBroadPhaseShared.h
	${LLAABB_DIR}/src/BpMBPTasks.cpp
	${LLAABB_DIR}/src/BpMBPTasks.h
	${LLAABB_DIR}/src/BpSAPTasks.cpp
	${LLAABB_DIR}/src/BpSAPTasks.h
)
SOURCE_GROUP("src" FILES ${LLAABB_SOURCE})

ADD_LIBRARY(LowLevelAABB ${LOWLEVELAABB_LIBTYPE}
	${LLAABB_HEADERS}
	${LLAABB_SOURCE}
)

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(LowLevelAABB 
	PRIVATE ${LOWLEVELAABB_PLATFORM_INCLUDES}

	PRIVATE ${PHYSXFOUNDATION_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/src
)

TARGET_COMPILE_DEFINITIONS(LowLevelAABB 
	PRIVATE ${LOWLEVELAABB_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(LowLevelAABB PROPERTIES
	LINK_FLAGS ${LOWLEVELAABB_PLATFORM_LINK_FLAGS}
)


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS)	
	SET_TARGET_PROPERTIES(LowLevelAABB PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "LowLevelAABB_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "LowLevelAABB_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "LowLevelAABB_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "LowLevelAABB_static"
	)
ENDIF()

IF(LOWLEVELAABB_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(LowLevelAABB PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${LOWLEVELAABB_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${LOWLEVELAABB_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${LOWLEVELAABB_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${LOWLEVELAABB_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LLAABB_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LLAABB_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(LowLevelAABB PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
