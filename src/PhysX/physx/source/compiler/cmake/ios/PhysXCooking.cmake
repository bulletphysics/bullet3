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
# Build PhysXCooking
#

SET(PHYSXCOOKING_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_IOS_COMPILE_DEFS};PX_PHYSX_LOADER_EXPORTS;PX_PHYSX_COOKING_EXPORTS;PX_PHYSX_CORE_EXPORTS;PX_COOKING;

	$<$<CONFIG:debug>:${PHYSX_IOS_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_IOS_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_IOS_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_IOS_RELEASE_COMPILE_DEFS};>

)

SET(PHYSXCOOKING_PLATFORM_SRC_FILES 
)

SET(PHYSXCOOKING_LIBTYPE STATIC)

SET(PHYSXCOOKING_LINK_FLAGS " ")
SET(PHYSXCOOKING_LINK_FLAGS_DEBUG " ")
SET(PHYSXCOOKING_LINK_FLAGS_CHECKED " ")
SET(PHYSXCOOKING_LINK_FLAGS_PROFILE " ")
SET(PHYSXCOOKING_LINK_FLAGS_RELEASE " ")