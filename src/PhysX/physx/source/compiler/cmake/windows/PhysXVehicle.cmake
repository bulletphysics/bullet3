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
# Build PhysXVehicle
#

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSXVEHICLE_COMPILE_DEFS

	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};PX_PHYSX_STATIC_LIB;${PHYSX_LIBTYPE_DEFS};${PHYSXGPU_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

SET(PHYSXVEHICLE_LIBTYPE STATIC)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSXVEHICLE_LIBTYPE STREQUAL "STATIC")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_DEBUG "PhysXVehicle_static${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_CHECKED "PhysXVehicle_static${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_PROFILE "PhysXVehicle_static${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_RELEASE "PhysXVehicle_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_DEBUG "PhysXVehicle${CMAKE_DEBUG_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_CHECKED "PhysXVehicle${CMAKE_CHECKED_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_PROFILE "PhysXVehicle${CMAKE_PROFILE_POSTFIX}")
	SET(PHYSXVEHICLE_COMPILE_PDB_NAME_RELEASE "PhysXVehicle${CMAKE_RELEASE_POSTFIX}")
ENDIF()

IF(PHYSXVEHICLE_LIBTYPE STREQUAL "SHARED")
	INSTALL(FILES $<TARGET_PDB_FILE:PhysXVehicle> 
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
ELSE()	
	INSTALL(FILES ${PHYSX_ROOT_DIR}/$<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>/$<$<CONFIG:debug>:${PHYSXVEHICLE_COMPILE_PDB_NAME_DEBUG}>$<$<CONFIG:checked>:${PHYSXVEHICLE_COMPILE_PDB_NAME_CHECKED}>$<$<CONFIG:profile>:${PHYSXVEHICLE_COMPILE_PDB_NAME_PROFILE}>$<$<CONFIG:release>:${PHYSXVEHICLE_COMPILE_PDB_NAME_RELEASE}>.pdb
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
ENDIF()