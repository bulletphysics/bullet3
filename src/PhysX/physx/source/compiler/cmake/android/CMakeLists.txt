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

# We define the CXX flags for this CMakeLists and all others that are included afterwards. This is a GLOBAL setting. 
# If/when the solutions go standalone (say, samples and visual tests) - those CMakeLists will need to be fixed.

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

SET(PHYSX_WARNING_DISABLES "-Wno-invalid-offsetof -Wno-maybe-uninitialized  -Wno-unused-variable -Wno-variadic-macros -Wno-array-bounds")

IF(${ANDROID_ABI} STREQUAL "armeabi-v7a")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -funwind-tables -fomit-frame-pointer -funswitch-loops -finline-limit=300 -fno-strict-aliasing -fstack-protector ${PHYSX_WARNING_DISABLES}" CACHE INTERAL "PhysX CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "armeabi-v7a with NEON")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -funwind-tables -fomit-frame-pointer -funswitch-loops -finline-limit=300 -fno-strict-aliasing -fstack-protector ${PHYSX_WARNING_DISABLES}" CACHE INTERAL "PhysX CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "arm64-v8a")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} " CACHE INTERAL "PhysX CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "x86")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} -fpack-struct=8 -malign-double " CACHE INTERAL "PhysX CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "x86_64")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} -mstackrealign -msse3 " CACHE INTERAL "PhysX CXX")
ENDIF()

# Build debug info for all configurations
SET(PHYSX_CXX_FLAGS_DEBUG "-O0 -g" CACHE INTERAL "PhysX Debug CXX Flags")
SET(PHYSX_CXX_FLAGS_CHECKED "-O3 -g" CACHE INTERAL "PhysX Checked CXX Flags")
SET(PHYSX_CXX_FLAGS_PROFILE "-O3 -g" CACHE INTERAL "PhysX Profile CXX Flags")
SET(PHYSX_CXX_FLAGS_RELEASE "-O3 -g" CACHE INTERAL "PhysX Release CXX Flags")


# These flags are local to the directory the CMakeLists.txt is in
SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

# Controls PX_NVTX for all projects
SET(NVTX_FLAG "PX_NVTX=0")

# Disable cuda and dx for all projects on Android
SET(PHYSX_ANDROID_COMPILE_DEFS "__STDC_LIMIT_MACROS;${PHYSX_AUTOBUILD};${PHYSX_AUTOBUILDNUMBER}" CACHE INTERNAL "Base PhysX preprocessor definitions")
SET(PHYSX_ANDROID_DEBUG_COMPILE_DEFS "_DEBUG;PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1" CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_ANDROID_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_ANDROID_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=1" CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_ANDROID_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")

SET(CMAKE_DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
SET(CMAKE_PROFILE_POSTFIX "${CMAKE_PROFILE_POSTFIX}")
SET(CMAKE_CHECKED_POSTFIX "${CMAKE_CHECKED_POSTFIX}")
SET(CMAKE_RELEASE_POSTFIX "${CMAKE_RELEASE_POSTFIX}")


# Include all of the projects
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
INCLUDE(PhysXPvdSDK.cmake)
INCLUDE(PhysXTask.cmake)
INCLUDE(SceneQuery.cmake)
INCLUDE(SimulationController.cmake)
INCLUDE(FastXml.cmake)

# Set folder PhysX SDK to all common SDK source projects
SET_PROPERTY(TARGET PhysX PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCharacterKinematic PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCooking PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXExtensions PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXVehicle PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevel PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelAABB PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelDynamics PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SceneQuery PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SimulationController PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET FastXml PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXPvdSDK PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXTask PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")