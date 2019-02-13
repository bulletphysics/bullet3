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
# Build PhysXTask common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/task)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXTask.cmake)

SET(PHYSXTASK_HEADERS
	${PHYSX_ROOT_DIR}/include/task/PxCpuDispatcher.h
	${PHYSX_ROOT_DIR}/include/task/PxGpuDispatcher.h
	${PHYSX_ROOT_DIR}/include/task/PxGpuTask.h
	${PHYSX_ROOT_DIR}/include/task/PxTask.h
	${PHYSX_ROOT_DIR}/include/task/PxTaskDefine.h
	${PHYSX_ROOT_DIR}/include/task/PxTaskManager.h
)
SOURCE_GROUP(include FILES ${PHYSXTASK_HEADERS})

SET(PHYSXTASK_SOURCE
	${LL_SOURCE_DIR}/src/TaskManager.cpp
)
SOURCE_GROUP(src FILES ${PHYSXTASK_SOURCE})

ADD_LIBRARY(PhysXTask ${PHYSXTASK_LIBTYPE}
	${PHYSXTASK_HEADERS}
	${PHYSXTASK_SOURCE}
)

INSTALL(FILES ${PHYSXTASK_HEADERS} DESTINATION include/task)

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(PhysXTask 
	PRIVATE ${PHYSXTASK_PLATFORM_INCLUDES}
	
	PRIVATE ${PHYSXFOUNDATION_INCLUDES}
	
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_SOURCE_DIR}/cudamanager/include
)

TARGET_COMPILE_DEFINITIONS(PhysXTask 
	PRIVATE ${PHYSXTASK_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXTask PROPERTIES
	OUTPUT_NAME PhysXTask
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSXTASK_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysXTask PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXTask_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXTask_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXTask_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXTask_static"	
	)
ENDIF()

IF(PHYSXTASK_COMPILE_PDB_NAME_DEBUG)	
	SET_TARGET_PROPERTIES(PhysXTask PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${PHYSXTASK_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSXTASK_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSXTASK_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSXTASK_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXTASK_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXTASK_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXTask PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
