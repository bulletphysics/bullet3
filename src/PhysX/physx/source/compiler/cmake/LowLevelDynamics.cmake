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
# Build LowLevelDynamics common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/lowleveldynamics/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/LowLevelDynamics.cmake)


SET(LLDYNAMICS_BASE_DIR ${PHYSX_ROOT_DIR}/source/lowleveldynamics)
SET(LLDYNAMICS_INCLUDES
	${LLDYNAMICS_BASE_DIR}/include/DyArticulationCore.h
	${LLDYNAMICS_BASE_DIR}/include/DyVArticulation.h		
	${LLDYNAMICS_BASE_DIR}/include/DyArticulation.h
	${LLDYNAMICS_BASE_DIR}/include/DyFeatherstoneArticulation.h
	${LLDYNAMICS_BASE_DIR}/include/DyFeatherstoneArticulationJointData.h
	${LLDYNAMICS_BASE_DIR}/include/DyFeatherstoneArticulationUtils.h
	${LLDYNAMICS_BASE_DIR}/include/DyConstraint.h
	${LLDYNAMICS_BASE_DIR}/include/DyConstraintWriteBack.h
	${LLDYNAMICS_BASE_DIR}/include/DyContext.h
	${LLDYNAMICS_BASE_DIR}/include/DySleepingConfigulation.h
	${LLDYNAMICS_BASE_DIR}/include/DyThresholdTable.h
	${LLDYNAMICS_BASE_DIR}/include/DyArticulationJointCore.h
)
SOURCE_GROUP("Dynamics Includes" FILES ${LLDYNAMICS_INCLUDES})

SET(LLDYNAMICS_SOURCE		
	${LLDYNAMICS_BASE_DIR}/src/DyArticulation.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationContactPrep.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationContactPrepPF.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationHelper.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationScalar.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationSIMD.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyFeatherstoneArticulation.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyFeatherstoneForwardDynamic.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyFeatherstoneInverseDynamic.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyConstraintPartition.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyConstraintSetup.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyConstraintSetupBlock.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrep.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrep4.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrep4PF.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrepPF.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyDynamics.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyFrictionCorrelation.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyRigidBodyToSolverBody.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraints.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraintsBlock.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverControl.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverControlPF.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverPFConstraints.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverPFConstraintsBlock.cpp
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraint1DStep.h
	${LLDYNAMICS_BASE_DIR}/src/DyThreadContext.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyThresholdTable.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyTGSDynamics.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyTGSPartition.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyTGSContactPrep.cpp
	${LLDYNAMICS_BASE_DIR}/src/DyTGSContactPrepBlock.cpp
)
SOURCE_GROUP("Dynamics Source" FILES ${LLDYNAMICS_SOURCE})

SET(LLDYNAMICS_INTERNAL_INCLUDES			
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationContactPrep.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationFnsDebug.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationFnsScalar.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationFnsSimd.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationHelper.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationPImpl.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationReference.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationScalar.h
	${LLDYNAMICS_BASE_DIR}/src/DyArticulationUtils.h
	${LLDYNAMICS_BASE_DIR}/src/DyFeatherstoneArticulationLink.h
	${LLDYNAMICS_BASE_DIR}/src/DyBodyCoreIntegrator.h
	${LLDYNAMICS_BASE_DIR}/src/DyConstraintPartition.h
	${LLDYNAMICS_BASE_DIR}/src/DyConstraintPrep.h
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrep.h
	${LLDYNAMICS_BASE_DIR}/src/DyContactPrepShared.h
	${LLDYNAMICS_BASE_DIR}/src/DyContactReduction.h
	${LLDYNAMICS_BASE_DIR}/src/DyCorrelationBuffer.h
	${LLDYNAMICS_BASE_DIR}/src/DyDynamics.h
	${LLDYNAMICS_BASE_DIR}/src/DyFrictionPatch.h
	${LLDYNAMICS_BASE_DIR}/src/DyFrictionPatchStreamPair.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverBody.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraint1D.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraint1D4.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraintDesc.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraintExtShared.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraintsShared.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverConstraintTypes.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverContact.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverContact4.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverContactPF.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverContactPF4.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverContext.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverControl.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverControlPF.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverCore.h
	${LLDYNAMICS_BASE_DIR}/src/DySolverExt.h
	${LLDYNAMICS_BASE_DIR}/src/DySpatial.h
	${LLDYNAMICS_BASE_DIR}/src/DyThreadContext.h
	${LLDYNAMICS_BASE_DIR}/src/DyTGSPartition.h
	${LLDYNAMICS_BASE_DIR}/src/DyTGSDynamics.h
)
SOURCE_GROUP("Dynamics Internal Includes" FILES ${LLDYNAMICS_INTERNAL_INCLUDES})

ADD_LIBRARY(LowLevelDynamics ${LOWLEVELDYNAMICS_LIBTYPE}
	${LLDYNAMICS_INCLUDES}
	${LLDYNAMICS_SOURCE}
	${LLDYNAMICS_INTERNAL_INCLUDES}
)

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(LowLevelDynamics 
	PRIVATE ${LOWLEVELDYNAMICS_PLATFORM_INCLUDES}

	PRIVATE ${PHYSXFOUNDATION_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include

	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(LowLevelDynamics 

	# Common to all configurations
	PRIVATE ${LOWLEVELDYNAMICS_COMPILE_DEFS}
)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS)	
	SET_TARGET_PROPERTIES(LowLevelDynamics PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "LowLevelDynamics_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "LowLevelDynamics_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "LowLevelDynamics_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "LowLevelDynamics_static"
	)
ENDIF()

IF(LOWLEVELDYNAMICS_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(LowLevelDynamics PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${LOWLEVELDYNAMICS_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${LOWLEVELDYNAMICS_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${LOWLEVELDYNAMICS_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${LOWLEVELDYNAMICS_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LLDYNAMICS_INCLUDES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LLDYNAMICS_INTERNAL_INCLUDES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${LLDYNAMICS_SOURCE})	
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(LowLevelDynamics PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
