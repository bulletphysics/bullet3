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
# Build PhysXFoundation common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/foundation)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysXFoundation.cmake)

SET(PHYSXFOUNDATION_HEADERS
	${PHYSX_ROOT_DIR}/include/PxFoundation.h
)
SOURCE_GROUP(include FILES ${PHYSXFOUNDATION_HEADERS})

SET(PHYSXFOUNDATION_HEADERS_2
	${PHYSX_ROOT_DIR}/include/foundation/PxAssert.h
	${PHYSX_ROOT_DIR}/include/foundation/PxFoundationConfig.h
	${PHYSX_ROOT_DIR}/include/foundation/PxMathUtils.h
)
SOURCE_GROUP(include\\foundation FILES ${PHYSXFOUNDATION_HEADERS_2})


SET(PXSHARED_HEADERS
	${PXSHARED_PATH}/include/foundation/Px.h
	${PXSHARED_PATH}/include/foundation/PxAllocatorCallback.h
	${PXSHARED_PATH}/include/foundation/PxProfiler.h
	${PXSHARED_PATH}/include/foundation/PxSharedAssert.h
	${PXSHARED_PATH}/include/foundation/PxBitAndData.h
	${PXSHARED_PATH}/include/foundation/PxBounds3.h
	${PXSHARED_PATH}/include/foundation/PxErrorCallback.h
	${PXSHARED_PATH}/include/foundation/PxErrors.h
	${PXSHARED_PATH}/include/foundation/PxFlags.h
	${PXSHARED_PATH}/include/foundation/PxIntrinsics.h
	${PXSHARED_PATH}/include/foundation/PxIO.h
	${PXSHARED_PATH}/include/foundation/PxMat33.h
	${PXSHARED_PATH}/include/foundation/PxMat44.h
	${PXSHARED_PATH}/include/foundation/PxMath.h	
	${PXSHARED_PATH}/include/foundation/PxMemory.h
	${PXSHARED_PATH}/include/foundation/PxPlane.h
	${PXSHARED_PATH}/include/foundation/PxPreprocessor.h
	${PXSHARED_PATH}/include/foundation/PxQuat.h
	${PXSHARED_PATH}/include/foundation/PxSimpleTypes.h
	${PXSHARED_PATH}/include/foundation/PxStrideIterator.h
	${PXSHARED_PATH}/include/foundation/PxTransform.h
	${PXSHARED_PATH}/include/foundation/PxUnionCast.h
	${PXSHARED_PATH}/include/foundation/PxVec2.h
	${PXSHARED_PATH}/include/foundation/PxVec3.h
	${PXSHARED_PATH}/include/foundation/PxVec4.h
)
SOURCE_GROUP(shared\\include FILES ${PXSHARED_HEADERS})

SET(PHYSXFOUNDATION_SOURCE
	${LL_SOURCE_DIR}/src/PsAllocator.cpp
	${LL_SOURCE_DIR}/src/PsAssert.cpp
	${LL_SOURCE_DIR}/src/PsFoundation.cpp
	${LL_SOURCE_DIR}/src/PsMathUtils.cpp
	${LL_SOURCE_DIR}/src/PsString.cpp
	${LL_SOURCE_DIR}/src/PsTempAllocator.cpp
	${LL_SOURCE_DIR}/src/PsUtilities.cpp
)
SOURCE_GROUP(src\\src FILES ${PHYSXFOUNDATION_SOURCE})

SET(PHYSXFOUNDATION_SOURCE_HEADERS
	${LL_SOURCE_DIR}/include/Ps.h
	${LL_SOURCE_DIR}/include/PsAlignedMalloc.h
	${LL_SOURCE_DIR}/include/PsAlloca.h
	${LL_SOURCE_DIR}/include/PsAllocator.h
	${LL_SOURCE_DIR}/include/PsAoS.h
	${LL_SOURCE_DIR}/include/PsArray.h
	${LL_SOURCE_DIR}/include/PsAtomic.h
	${LL_SOURCE_DIR}/include/PsBasicTemplates.h
	${LL_SOURCE_DIR}/include/PsBitUtils.h
	${LL_SOURCE_DIR}/include/PsBroadcast.h
	${LL_SOURCE_DIR}/include/PsCpu.h
	${LL_SOURCE_DIR}/include/PsFoundation.h
	${LL_SOURCE_DIR}/include/PsFPU.h
	${LL_SOURCE_DIR}/include/PsHash.h
	${LL_SOURCE_DIR}/include/PsHashInternals.h
	${LL_SOURCE_DIR}/include/PsHashMap.h
	${LL_SOURCE_DIR}/include/PsHashSet.h
	${LL_SOURCE_DIR}/include/PsInlineAllocator.h
	${LL_SOURCE_DIR}/include/PsInlineAoS.h
	${LL_SOURCE_DIR}/include/PsInlineArray.h
	${LL_SOURCE_DIR}/include/PsIntrinsics.h
	${LL_SOURCE_DIR}/include/PsMathUtils.h
	${LL_SOURCE_DIR}/include/PsMutex.h
	${LL_SOURCE_DIR}/include/PsPool.h
	${LL_SOURCE_DIR}/include/PsSList.h
	${LL_SOURCE_DIR}/include/PsSocket.h
	${LL_SOURCE_DIR}/include/PsSort.h
	${LL_SOURCE_DIR}/include/PsSortInternals.h
	${LL_SOURCE_DIR}/include/PsString.h
	${LL_SOURCE_DIR}/include/PsSync.h
	${LL_SOURCE_DIR}/include/PsTempAllocator.h
	${LL_SOURCE_DIR}/include/PsThread.h
	${LL_SOURCE_DIR}/include/PsTime.h
	${LL_SOURCE_DIR}/include/PsUserAllocated.h
	${LL_SOURCE_DIR}/include/PsUtilities.h
	${LL_SOURCE_DIR}/include/PsVecMath.h
	${LL_SOURCE_DIR}/include/PsVecMathAoSScalar.h
	${LL_SOURCE_DIR}/include/PsVecMathAoSScalarInline.h
	${LL_SOURCE_DIR}/include/PsVecMathSSE.h
	${LL_SOURCE_DIR}/include/PsVecMathUtilities.h
	${LL_SOURCE_DIR}/include/PsVecQuat.h
	${LL_SOURCE_DIR}/include/PsVecTransform.h
)
SOURCE_GROUP("src\\include" FILES ${PHYSXFOUNDATION_SOURCE_HEADERS})

ADD_LIBRARY(PhysXFoundation ${PHYSXFOUNDATION_LIBTYPE} 
	${PHYSXFOUNDATION_SOURCE}
	${PHYSXFOUNDATION_SOURCE_HEADERS}
	${PHYSXFOUNDATION_HEADERS}
	${PHYSXFOUNDATION_HEADERS_2}
	${PHYSXFOUNDATION_PLATFORM_FILES}
	${PXSHARED_HEADERS}
	${PXSHARED_PLATFORM_HEADERS}
)

# Add the headers to the install
INSTALL(FILES ${PHYSXFOUNDATION_HEADERS} DESTINATION include)
INSTALL(FILES ${PHYSXFOUNDATION_HEADERS_2} DESTINATION include/foundation)
INSTALL(FILES ${PHYSXFOUNDATION_SOURCE_HEADERS} DESTINATION source/foundation/include)
INSTALL(FILES ${PXSHARED_HEADERS} DESTINATION ${PXSHARED_INSTALL_PREFIX}/include/foundation)

TARGET_INCLUDE_DIRECTORIES(PhysXFoundation 
	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PXSHARED_PATH}/include
	PRIVATE ${LL_SOURCE_DIR}/include
	PRIVATE ${PHYSXFOUNDATION_PLATFORM_INCLUDES}
	
	INTERFACE $<INSTALL_INTERFACE:include>$<BUILD_INTERFACE:${PHYSX_ROOT_DIR}/include>
	INTERFACE $<INSTALL_INTERFACE:../PxShared/include>$<BUILD_INTERFACE:${PXSHARED_PATH}/include>

	# FIXME: This is really terrible! Don't export src directories
	INTERFACE $<INSTALL_INTERFACE:source/foundation/include>$<BUILD_INTERFACE:${LL_SOURCE_DIR}/include>
)

TARGET_COMPILE_DEFINITIONS(PhysXFoundation 
	PRIVATE ${PHYSXFOUNDATION_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES
	OUTPUT_NAME PhysXFoundation
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSXFOUNDATION_LIBTYPE STREQUAL "STATIC")	
	SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES 			
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysXFoundation_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysXFoundation_static"
	)	
ENDIF()

IF(PHYSXFOUNDATION_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES 
		COMPILE_PDB_NAME_DEBUG ${PHYSXFOUNDATION_COMPILE_PDB_NAME_DEBUG}
		COMPILE_PDB_NAME_CHECKED ${PHYSXFOUNDATION_COMPILE_PDB_NAME_CHECKED}
		COMPILE_PDB_NAME_PROFILE ${PHYSXFOUNDATION_COMPILE_PDB_NAME_PROFILE}
		COMPILE_PDB_NAME_RELEASE ${PHYSXFOUNDATION_COMPILE_PDB_NAME_RELEASE}
	)
ENDIF()

# Add linked libraries
TARGET_LINK_LIBRARIES(PhysXFoundation 
	PRIVATE ${PHYSXFOUNDATION_PLATFORM_LINKED_LIBS}
)

IF(PX_GENERATE_SOURCE_DISTRO)		
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_SOURCE_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_HEADERS_2})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSXFOUNDATION_PLATFORM_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PXSHARED_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PXSHARED_PLATFORM_HEADERS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysXFoundation PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
