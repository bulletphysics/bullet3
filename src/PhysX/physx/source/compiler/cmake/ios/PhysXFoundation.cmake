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
# Build PhysXFoundation
#

SET(PHYSXFOUNDATION_LIBTYPE STATIC)

SET(PXSHARED_PLATFORM_HEADERS
	${PXSHARED_PATH}/include/foundation/unix/PxUnixIntrinsics.h	
)
SOURCE_GROUP(shared\\include\\unix FILES ${PXSHARED_PLATFORM_HEADERS})

SET(PHYSXFOUNDATION_PLATFORM_INCLUDES
	${LL_SOURCE_DIR}/include/ios
)

SET(PHYSXFOUNDATION_PLATFORM_SOURCE
	${LL_SOURCE_DIR}/src/unix/PsUnixAtomic.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixCpu.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixFPU.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixMutex.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixPrintString.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixSList.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixSocket.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixSync.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixThread.cpp
	${LL_SOURCE_DIR}/src/unix/PsUnixTime.cpp
)

SET(PHYSXFOUNDATION_NEON_FILES
	${LL_SOURCE_DIR}/include/unix/neon/PsUnixNeonAoS.h
	${LL_SOURCE_DIR}/include/unix/neon/PsUnixNeonInlineAoS.h
)


SET(PHYSXFOUNDATION_PLATFORM_SOURCE_HEADERS
	${LL_SOURCE_DIR}/include/unix/PsUnixAoS.h
	${LL_SOURCE_DIR}/include/unix/PsUnixFPU.h
	${LL_SOURCE_DIR}/include/unix/PsUnixInlineAoS.h
	${LL_SOURCE_DIR}/include/unix/PsUnixIntrinsics.h
	${LL_SOURCE_DIR}/include/unix/PsUnixTrigConstants.h
)
SOURCE_GROUP("src\\include\\unix" FILES ${PHYSXFOUNDATION_PLATFORM_SOURCE_HEADERS})

INSTALL(FILES ${PHYSXFOUNDATION_PLATFORM_SOURCE_HEADERS} DESTINATION source/foundation/include/unix)
INSTALL(FILES ${PHYSXFOUNDATION_NEON_FILES} DESTINATION source/foundation/include/unix/neon)
INSTALL(FILES ${PXSHARED_PLATFORM_HEADERS} DESTINATION ${PXSHARED_INSTALL_PREFIX}/include/foundation/unix)

SET(PHYSXFOUNDATION_PLATFORM_FILES
	${PHYSXFOUNDATION_PLATFORM_SOURCE}
	${PHYSXFOUNDATION_PLATFORM_SOURCE_HEADERS}
	${PHYSXFOUNDATION_NEON_FILES}
	${PHYSXFOUNDATION_RESOURCE_FILE}
)

SET(PHYSXFOUNDATION_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_IOS_COMPILE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_IOS_DEBUG_COMPILE_DEFS}>
	$<$<CONFIG:checked>:${PHYSX_IOS_CHECKED_COMPILE_DEFS}>
	$<$<CONFIG:profile>:${PHYSX_IOS_PROFILE_COMPILE_DEFS}>
	$<$<CONFIG:release>:${PHYSX_IOS_RELEASE_COMPILE_DEFS}>
)
