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

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

#TODO: Fix warnings
SET(CLANG_WARNINGS "-ferror-limit=0 -Wall -Wextra -Werror -Wstrict-aliasing=2 -Weverything -Wno-documentation-deprecated-sync -Wno-documentation-unknown-command -Wno-gnu-anonymous-struct -Wno-undef -Wno-unused-function -Wno-nested-anon-types -Wno-float-equal -Wno-padded -Wno-weak-vtables -Wno-cast-align -Wno-conversion -Wno-missing-noreturn -Wno-missing-variable-declarations -Wno-shift-sign-overflow -Wno-covered-switch-default -Wno-exit-time-destructors -Wno-global-constructors -Wno-missing-prototypes -Wno-unreachable-code -Wno-unused-macros -Wno-unused-member-function -Wno-used-but-marked-unused -Wno-weak-template-vtables -Wno-deprecated -Wno-non-virtual-dtor -Wno-invalid-noreturn -Wno-return-type-c-linkage -Wno-reserved-id-macro -Wno-c++98-compat-pedantic -Wno-unused-local-typedef -Wno-old-style-cast -Wno-newline-eof -Wno-unused-private-field -Wno-format-nonliteral -Wno-implicit-fallthrough -Wno-undefined-reinterpret-cast -Wno-disabled-macro-expansion -Wno-zero-as-null-pointer-constant -Wno-shadow -Wno-unknown-warning-option")
SET(GCC_WARNINGS "-Wall -Werror -Wno-invalid-offsetof -Wno-uninitialized")

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# using Clang	  
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fstrict-aliasing ${CLANG_WARNINGS}" CACHE INTERAL "PhysX CXX")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fno-strict-aliasing ${GCC_WARNINGS}" CACHE INTERAL "PhysX CXX")
ENDIF()

# Build debug info for all configurations
SET(PHYSX_CXX_FLAGS_DEBUG   "-O0 -g3 -gdwarf-2" CACHE INTERNAL "PhysX Debug CXX Flags")
SET(PHYSX_CXX_FLAGS_CHECKED "-O3 -g3 -gdwarf-2" CACHE INTERNAL "PhysX Checked CXX Flags")
SET(PHYSX_CXX_FLAGS_PROFILE "-O3" CACHE INTERNAL "PhysX Profile CXX Flags")
SET(PHYSX_CXX_FLAGS_RELEASE "-O3" CACHE INTERNAL "PhysX Release CXX Flags")

# These flags are local to the directory the CMakeLists.txt is in, so don't get carried over to OTHER CMakeLists.txt (thus the CACHE variables above)
SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})


# Controls PX_NVTX for all projects
SET(NVTX_FLAG "PX_NVTX=0")

IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	SET(CUDA_FLAG "DISABLE_CUDA_PHYSX")
ENDIF()

SET(PHYSX_LINUX_COMPILE_DEFS "${CUDA_FLAG};${PHYSX_AUTOBUILD};${PHYSX_AUTOBUILDNUMBER}" CACHE INTERNAL "Base PhysX preprocessor definitions")

SET(PHYSX_LINUX_DEBUG_COMPILE_DEFS   "NDEBUG;PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1"  CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_LINUX_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_LINUX_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=1"  CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_LINUX_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")

SET(GENERATED_GPU_CUDA_FILES "")

# Include all of the projects
IF(PX_GENERATE_GPU_PROJECTS_ONLY)
	INCLUDE(PhysXCommon.cmake)
	INCLUDE(PhysXBroadphaseGpu.cmake)
	INCLUDE(PhysXCommonGpu.cmake)
	INCLUDE(PhysXNarrowphaseGpu.cmake)
	INCLUDE(PhysXSimulationControllerGpu.cmake)
	INCLUDE(PhysXSolverGpu.cmake)
	INCLUDE(CloudGpu.cmake)
	INCLUDE(PhysXCudaContextManager.cmake)
	INCLUDE(PhysXArticulationGpu.cmake)
	INCLUDE(PhysXGpu.cmake)	 # must be last
ELSE()
	INCLUDE(PhysXFoundation.cmake)
	INCLUDE(LowLevel.cmake)
	INCLUDE(LowLevelAABB.cmake)
	INCLUDE(LowLevelDynamics.cmake)
	INCLUDE(PhysX.cmake)
	INCLUDE(PhysXCharacterKinematic.cmake)
	INCLUDE(PhysXCommon.cmake)
	INCLUDE(PhysXCooking.cmake)
	INCLUDE(PhysXExtensions.cmake)
	INCLUDE(PhysXVehicle.cmake)
	INCLUDE(SceneQuery.cmake)
	INCLUDE(SimulationController.cmake)
	INCLUDE(FastXml.cmake)
	INCLUDE(PhysXPvdSDK.cmake)
	INCLUDE(PhysXTask.cmake)
	
	SET(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle)
	
	IF(PX_GENERATE_GPU_PROJECTS)
			INCLUDE(PhysXBroadphaseGpu.cmake)
			INCLUDE(PhysXCommonGpu.cmake)
			INCLUDE(PhysXNarrowphaseGpu.cmake)
			INCLUDE(PhysXSimulationControllerGpu.cmake)
			INCLUDE(PhysXSolverGpu.cmake)
			INCLUDE(CloudGpu.cmake)
			INCLUDE(PhysXCudaContextManager.cmake)
			INCLUDE(PhysXArticulationGpu.cmake)
			INCLUDE(PhysXGpu.cmake)		# must be last
			
			LIST(APPEND PHYSXDISTRO_LIBS PhysXGpu)
	ENDIF()
	
	INSTALL(
		TARGETS ${PHYSXDISTRO_LIBS}
		EXPORT PhysXSDK
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>	
		)
	
ENDIF()
