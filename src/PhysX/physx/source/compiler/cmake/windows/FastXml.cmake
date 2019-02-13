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
# Build FastXml
#

# Use generator expressions to set config specific preprocessor definitions
SET(FASTXML_COMPILE_DEFS 
	# Common to all configurations
	${PHYSX_WINDOWS_COMPILE_DEFS};PX_FOUNDATION_DLL=0;

	$<$<CONFIG:debug>:${PHYSX_WINDOWS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_WINDOWS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_WINDOWS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_WINDOWS_RELEASE_COMPILE_DEFS};>
)

SET(FASTXML_LIBTYPE STATIC)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND FASTXML_LIBTYPE STREQUAL "STATIC")
	SET(FASTXML_COMPILE_PDB_NAME_DEBUG "FastXml_static${CMAKE_DEBUG_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_CHECKED "FastXml_static${CMAKE_CHECKED_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_PROFILE "FastXml_static${CMAKE_PROFILE_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_RELEASE "FastXml_static${CMAKE_RELEASE_POSTFIX}")
ELSE()
	SET(FASTXML_COMPILE_PDB_NAME_DEBUG "FastXml${CMAKE_DEBUG_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_CHECKED "FastXml${CMAKE_CHECKED_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_PROFILE "FastXml${CMAKE_PROFILE_POSTFIX}")
	SET(FASTXML_COMPILE_PDB_NAME_RELEASE "FastXml${CMAKE_RELEASE_POSTFIX}")
ENDIF()