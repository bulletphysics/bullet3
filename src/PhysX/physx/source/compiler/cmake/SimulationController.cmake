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
# Build SimulationController common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(LL_SOURCE_DIR ${PHYSX_SOURCE_DIR}/simulationcontroller/src)

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/SimulationController.cmake)


SET(SIMULATIONCONTROLLER_BASE_DIR ${PHYSX_ROOT_DIR}/source/simulationcontroller)
SET(SIMULATIONCONTROLLER_HEADERS		
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScActorCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScArticulationCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScArticulationJointCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScBodyCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScConstraintCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScIterators.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScMaterialCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScPhysics.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScRigidCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScScene.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScShapeCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/include/ScStaticCore.h
)
SOURCE_GROUP(include FILES ${SIMULATIONCONTROLLER_HEADERS})

SET(SIMULATIONCONTROLLER_SOURCE
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScActorCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScActorPair.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScActorSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScActorSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationJointCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationJointSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationJointSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScArticulationSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScBodyCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScBodySim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScBodySim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScClient.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintGroupNode.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintGroupNode.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintInteraction.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintInteraction.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintProjectionManager.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintProjectionManager.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintProjectionTree.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintProjectionTree.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScConstraintSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScContactReportBuffer.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScContactStream.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScElementInteractionMarker.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScElementInteractionMarker.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScElementSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScElementSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScElementSimInteraction.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScInteraction.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScInteraction.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScInteractionFlags.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScIterators.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScMetaData.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScNPhaseCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScNPhaseCore.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScObjectIDTracker.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScPhysics.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScRigidCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScRigidSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScRigidSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScScene.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScShapeCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScShapeInteraction.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScShapeInteraction.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScShapeSim.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScShapeSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSimStateData.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSimStats.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSimStats.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSimulationController.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSimulationController.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSqBoundsManager.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScSqBoundsManager.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScStaticCore.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScStaticSim.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScTriggerInteraction.cpp
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScTriggerInteraction.h
	${SIMULATIONCONTROLLER_BASE_DIR}/src/ScTriggerPairs.h
)
SOURCE_GROUP(src FILES ${SIMULATIONCONTROLLER_SOURCE})

ADD_LIBRARY(SimulationController ${SIMULATIONCONTROLLER_LIBTYPE}
	${SIMULATIONCONTROLLER_HEADERS}
	${SIMULATIONCONTROLLER_SOURCE}
)

GET_TARGET_PROPERTY(PHYSXFOUNDATION_INCLUDES PhysXFoundation INTERFACE_INCLUDE_DIRECTORIES)

TARGET_INCLUDE_DIRECTORIES(SimulationController 
	PRIVATE ${SIMULATIONCONTROLLER_PLATFORM_INCLUDES}
	
	PRIVATE ${PHYSXFOUNDATION_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/pvd
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils
	
	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxgpu/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/include
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/contact
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/common
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/convex
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/distance
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/sweep
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/gjk
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/intersection
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/hf
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/pcm
	PRIVATE ${PHYSX_SOURCE_DIR}/geomutils/src/ccd
	
	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/collision
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/utils
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include

	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include

	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/include
	
)

# Use generator expressions to set config specific preprocessor definitions
TARGET_COMPILE_DEFINITIONS(SimulationController 

	# Common to all configurations
	PRIVATE ${SIMULATIONCONTROLLER_COMPILE_DEFS}
)


IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND SIMULATIONCONTROLLER_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(SimulationController PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "SimulationController_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "SimulationController_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "SimulationController_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "SimulationController_static"
	)
ENDIF()

IF(SIMULATIONCONTROLLER_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(SimulationController PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${SIMULATIONCONTROLLER_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${SIMULATIONCONTROLLER_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${SIMULATIONCONTROLLER_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${SIMULATIONCONTROLLER_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

IF(PX_GENERATE_SOURCE_DISTRO)				
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SIMULATIONCONTROLLER_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${SIMULATIONCONTROLLER_SOURCE})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(SimulationController PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
