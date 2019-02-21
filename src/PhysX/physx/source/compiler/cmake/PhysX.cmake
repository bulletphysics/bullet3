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
# Build PhysX (PROJECT not SOLUTION) common
#

SET(PHYSX_SOURCE_DIR ${PHYSX_ROOT_DIR}/source)
SET(PX_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physx/src)
SET(MD_SOURCE_DIR ${PHYSX_SOURCE_DIR}/physxmetadata)

SET(PHYSX_PLATFORM_LINK_FLAGS " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_DEBUG " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_CHECKED " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_PROFILE " ")
SET(PHYSX_PLATFORM_LINK_FLAGS_RELEASE " ")

# Include here after the directories are defined so that the platform specific file can use the variables.
include(${PHYSX_ROOT_DIR}/${PROJECT_CMAKE_FILES_DIR}/${TARGET_BUILD_PLATFORM}/PhysX.cmake)

SET(PHYSX_HEADERS
	${PHYSX_ROOT_DIR}/include/PxActor.h
	${PHYSX_ROOT_DIR}/include/PxAggregate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationReducedCoordinate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationBase.h
	${PHYSX_ROOT_DIR}/include/PxArticulation.h
	${PHYSX_ROOT_DIR}/include/PxArticulationJoint.h
	${PHYSX_ROOT_DIR}/include/PxArticulationJointReducedCoordinate.h
	${PHYSX_ROOT_DIR}/include/PxArticulationLink.h
	${PHYSX_ROOT_DIR}/include/PxBatchQuery.h
	${PHYSX_ROOT_DIR}/include/PxBatchQueryDesc.h
	${PHYSX_ROOT_DIR}/include/PxBroadPhase.h
	${PHYSX_ROOT_DIR}/include/PxClient.h
	${PHYSX_ROOT_DIR}/include/PxConstraint.h
	${PHYSX_ROOT_DIR}/include/PxConstraintDesc.h
	${PHYSX_ROOT_DIR}/include/PxContact.h
	${PHYSX_ROOT_DIR}/include/PxContactModifyCallback.h
	${PHYSX_ROOT_DIR}/include/PxDeletionListener.h
	${PHYSX_ROOT_DIR}/include/PxFiltering.h
	${PHYSX_ROOT_DIR}/include/PxForceMode.h
	${PHYSX_ROOT_DIR}/include/PxImmediateMode.h
	${PHYSX_ROOT_DIR}/include/PxLockedData.h
	${PHYSX_ROOT_DIR}/include/PxMaterial.h
	${PHYSX_ROOT_DIR}/include/PxPhysics.h
	${PHYSX_ROOT_DIR}/include/PxPhysicsAPI.h
	${PHYSX_ROOT_DIR}/include/PxPhysicsSerialization.h
	${PHYSX_ROOT_DIR}/include/PxPhysicsVersion.h
	${PHYSX_ROOT_DIR}/include/PxPhysXConfig.h
	${PHYSX_ROOT_DIR}/include/PxPruningStructure.h
	${PHYSX_ROOT_DIR}/include/PxQueryFiltering.h
	${PHYSX_ROOT_DIR}/include/PxQueryReport.h
	${PHYSX_ROOT_DIR}/include/PxRigidActor.h
	${PHYSX_ROOT_DIR}/include/PxRigidBody.h
	${PHYSX_ROOT_DIR}/include/PxRigidDynamic.h
	${PHYSX_ROOT_DIR}/include/PxRigidStatic.h
	${PHYSX_ROOT_DIR}/include/PxScene.h
	${PHYSX_ROOT_DIR}/include/PxSceneDesc.h
	${PHYSX_ROOT_DIR}/include/PxSceneLock.h
	${PHYSX_ROOT_DIR}/include/PxShape.h
	${PHYSX_ROOT_DIR}/include/PxSimulationEventCallback.h
	${PHYSX_ROOT_DIR}/include/PxSimulationStatistics.h
	${PHYSX_ROOT_DIR}/include/PxVisualizationParameter.h
)
SOURCE_GROUP(include FILES ${PHYSX_HEADERS})

SET(PHYSX_COMMON_HEADERS
	${PHYSX_ROOT_DIR}/include/common/PxBase.h
	${PHYSX_ROOT_DIR}/include/common/PxCollection.h
	${PHYSX_ROOT_DIR}/include/common/PxCoreUtilityTypes.h
	${PHYSX_ROOT_DIR}/include/common/PxMetaData.h
	${PHYSX_ROOT_DIR}/include/common/PxMetaDataFlags.h
	${PHYSX_ROOT_DIR}/include/common/PxPhysicsInsertionCallback.h
	${PHYSX_ROOT_DIR}/include/common/PxPhysXCommonConfig.h
	${PHYSX_ROOT_DIR}/include/common/PxRenderBuffer.h
	${PHYSX_ROOT_DIR}/include/common/PxSerialFramework.h
	${PHYSX_ROOT_DIR}/include/common/PxSerializer.h
	${PHYSX_ROOT_DIR}/include/common/PxStringTable.h
	${PHYSX_ROOT_DIR}/include/common/PxTolerancesScale.h
	${PHYSX_ROOT_DIR}/include/common/PxTypeInfo.h
	${PHYSX_ROOT_DIR}/include/common/PxProfileZone.h
)
SOURCE_GROUP(include\\common FILES ${PHYSX_COMMON_HEADERS})

SET(PHYSX_PVD_HEADERS
	${PHYSX_ROOT_DIR}/include/pvd/PxPvdSceneClient.h
	${PHYSX_ROOT_DIR}/include/pvd/PxPvd.h
	${PHYSX_ROOT_DIR}/include/pvd/PxPvdTransport.h
)
SOURCE_GROUP(include\\pvd FILES ${PHYSX_PVD_HEADERS})

SET(PHYSX_COLLISION_HEADERS
	${PHYSX_ROOT_DIR}/include/collision/PxCollisionDefs.h
)
SOURCE_GROUP(include\\collision FILES ${PHYSX_COLLISION_HEADERS})

SET(PHYSX_SOLVER_HEADERS
	${PHYSX_ROOT_DIR}/include/solver/PxSolverDefs.h
)
SOURCE_GROUP(include\\collision FILES ${PHYSX_SOLVER_HEADERS})

SET(PHYSX_METADATA_HEADERS
	${MD_SOURCE_DIR}/core/include/PvdMetaDataDefineProperties.h
	${MD_SOURCE_DIR}/core/include/PvdMetaDataExtensions.h
	${MD_SOURCE_DIR}/core/include/PvdMetaDataPropertyVisitor.h
	${MD_SOURCE_DIR}/core/include/PxAutoGeneratedMetaDataObjectNames.h
	${MD_SOURCE_DIR}/core/include/PxAutoGeneratedMetaDataObjects.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataCompare.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataCppPrefix.h
	${MD_SOURCE_DIR}/core/include/PxMetaDataObjects.h
	${MD_SOURCE_DIR}/core/include/RepXMetaDataPropertyVisitor.h
)
SOURCE_GROUP(metadata\\include FILES ${PHYSX_METADATA_HEADERS})

SET(PHYSX_METADATA_SOURCE
	${MD_SOURCE_DIR}/core/src/PxAutoGeneratedMetaDataObjects.cpp
	${MD_SOURCE_DIR}/core/src/PxMetaDataObjects.cpp
)
SOURCE_GROUP(metadata\\src FILES ${PHYSX_METADATA_SOURCE})

SET(PHYSX_IMMEDIATEMODE_SOURCE
	${PHYSX_ROOT_DIR}/source/immediatemode/src/NpImmediateMode.cpp
)
SOURCE_GROUP(src\\immediatemode FILES ${PHYSX_IMMEDIATEMODE_SOURCE})

SET(PHYSX_BUFFERING_SOURCE
	${PX_SOURCE_DIR}/buffering/ScbActor.cpp
	${PX_SOURCE_DIR}/buffering/ScbAggregate.cpp
	${PX_SOURCE_DIR}/buffering/ScbBase.cpp
	${PX_SOURCE_DIR}/buffering/ScbMetaData.cpp
	${PX_SOURCE_DIR}/buffering/ScbScene.cpp
	${PX_SOURCE_DIR}/buffering/ScbScenePvdClient.cpp
	${PX_SOURCE_DIR}/buffering/ScbShape.cpp
	${PX_SOURCE_DIR}/buffering/ScbActor.h
	${PX_SOURCE_DIR}/buffering/ScbAggregate.h
	${PX_SOURCE_DIR}/buffering/ScbArticulation.h
	${PX_SOURCE_DIR}/buffering/ScbArticulationJoint.h
	${PX_SOURCE_DIR}/buffering/ScbBase.h
	${PX_SOURCE_DIR}/buffering/ScbBody.h
	${PX_SOURCE_DIR}/buffering/ScbConstraint.h
	${PX_SOURCE_DIR}/buffering/ScbDefs.h
	${PX_SOURCE_DIR}/buffering/ScbNpDeps.h
	${PX_SOURCE_DIR}/buffering/ScbRigidObject.h
	${PX_SOURCE_DIR}/buffering/ScbRigidStatic.h
	${PX_SOURCE_DIR}/buffering/ScbScene.h
	${PX_SOURCE_DIR}/buffering/ScbSceneBuffer.h
	${PX_SOURCE_DIR}/buffering/ScbScenePvdClient.h
	${PX_SOURCE_DIR}/buffering/ScbShape.h
	${PX_SOURCE_DIR}/buffering/ScbType.h
)
SOURCE_GROUP(src\\buffering FILES ${PHYSX_BUFFERING_SOURCE})

SET(PHYSX_CORE_SOURCE
	${PX_SOURCE_DIR}/NpActor.cpp
	${PX_SOURCE_DIR}/NpAggregate.cpp
	${PX_SOURCE_DIR}/NpArticulationReducedCoordinate.cpp
	${PX_SOURCE_DIR}/NpArticulation.cpp
	${PX_SOURCE_DIR}/NpArticulationJoint.cpp
	${PX_SOURCE_DIR}/NpArticulationJointReducedCoordinate.cpp
	${PX_SOURCE_DIR}/NpArticulationLink.cpp
	${PX_SOURCE_DIR}/NpBatchQuery.cpp
	${PX_SOURCE_DIR}/NpConstraint.cpp
	${PX_SOURCE_DIR}/NpFactory.cpp
	${PX_SOURCE_DIR}/NpMaterial.cpp
	${PX_SOURCE_DIR}/NpMetaData.cpp
	${PX_SOURCE_DIR}/NpPhysics.cpp
	${PX_SOURCE_DIR}/NpPvdSceneQueryCollector.cpp
	${PX_SOURCE_DIR}/NpReadCheck.cpp
	${PX_SOURCE_DIR}/NpRigidDynamic.cpp
	${PX_SOURCE_DIR}/NpRigidStatic.cpp
	${PX_SOURCE_DIR}/NpScene.cpp
	${PX_SOURCE_DIR}/NpSceneQueries.cpp
	${PX_SOURCE_DIR}/NpSerializerAdapter.cpp
	${PX_SOURCE_DIR}/NpShape.cpp
	${PX_SOURCE_DIR}/NpShapeManager.cpp
	${PX_SOURCE_DIR}/NpWriteCheck.cpp
	${PX_SOURCE_DIR}/PvdMetaDataPvdBinding.cpp
	${PX_SOURCE_DIR}/PvdPhysicsClient.cpp
	${PX_SOURCE_DIR}/NpActor.h
	${PX_SOURCE_DIR}/NpActorTemplate.h
	${PX_SOURCE_DIR}/NpAggregate.h
	${PX_SOURCE_DIR}/NpArticulationReducedCoordinate.h
	${PX_SOURCE_DIR}/NpArticulation.h
	${PX_SOURCE_DIR}/NpArticulationJoint.h
	${PX_SOURCE_DIR}/NpArticulationJointReducedCoordinate.h
	${PX_SOURCE_DIR}/NpArticulationLink.h
	${PX_SOURCE_DIR}/NpArticulationTemplate.h
	${PX_SOURCE_DIR}/NpBatchQuery.h
	${PX_SOURCE_DIR}/NpCast.h
	${PX_SOURCE_DIR}/NpConnector.h
	${PX_SOURCE_DIR}/NpConstraint.h
	${PX_SOURCE_DIR}/NpFactory.h
	${PX_SOURCE_DIR}/NpMaterial.h
	${PX_SOURCE_DIR}/NpMaterialManager.h
	${PX_SOURCE_DIR}/NpPhysics.h
	${PX_SOURCE_DIR}/NpPhysicsInsertionCallback.h
	${PX_SOURCE_DIR}/NpPtrTableStorageManager.h
	${PX_SOURCE_DIR}/NpPvdSceneQueryCollector.h
	${PX_SOURCE_DIR}/NpQueryShared.h
	${PX_SOURCE_DIR}/NpReadCheck.h
	${PX_SOURCE_DIR}/NpRigidActorTemplate.h
	${PX_SOURCE_DIR}/NpRigidActorTemplateInternal.h
	${PX_SOURCE_DIR}/NpRigidBodyTemplate.h
	${PX_SOURCE_DIR}/NpRigidDynamic.h
	${PX_SOURCE_DIR}/NpRigidStatic.h
	${PX_SOURCE_DIR}/NpScene.h
	${PX_SOURCE_DIR}/NpSceneQueries.h
	${PX_SOURCE_DIR}/NpSceneAccessor.h
	${PX_SOURCE_DIR}/NpShape.h
	${PX_SOURCE_DIR}/NpShapeManager.h
	${PX_SOURCE_DIR}/NpWriteCheck.h
	${PX_SOURCE_DIR}/PvdMetaDataBindingData.h
	${PX_SOURCE_DIR}/PvdMetaDataPvdBinding.h
	${PX_SOURCE_DIR}/PvdPhysicsClient.h
	${PX_SOURCE_DIR}/PvdTypeNames.h
)
SOURCE_GROUP(src FILES ${PHYSX_CORE_SOURCE})

ADD_LIBRARY(PhysX ${PHYSX_LIBTYPE}
	${PHYSX_HEADERS}
	${PHYSX_COMMON_HEADERS}
	${PHYSX_PVD_HEADERS}
	${PHYSX_SOLVER_HEADERS}
	
	${PHYSX_METADATA_HEADERS}
	${PHYSX_METADATA_SOURCE}
	
	${PHYSX_CORE_SOURCE}
	${PHYSX_BUFFERING_SOURCE}
	${PHYSX_IMMEDIATEMODE_SOURCE}
	
	${PHYSX_PLATFORM_SRC_FILES}	
)

# Add the headers to the install
INSTALL(FILES ${PHYSX_HEADERS} DESTINATION include)
INSTALL(FILES ${PHYSX_COMMON_HEADERS} DESTINATION include/common)
INSTALL(FILES ${PHYSX_PVD_HEADERS} DESTINATION include/pvd)
INSTALL(FILES ${PHYSX_COLLISION_HEADERS} DESTINATION include/collision)
INSTALL(FILES ${PHYSX_SOLVER_HEADERS} DESTINATION include/solver)

TARGET_INCLUDE_DIRECTORIES(PhysX 
	PRIVATE ${PHYSX_PLATFORM_INCLUDES}

	PRIVATE ${PHYSX_ROOT_DIR}/include
	PRIVATE ${PHYSX_ROOT_DIR}/include/gpu
	PRIVATE ${PHYSX_ROOT_DIR}/include/common
	PRIVATE ${PHYSX_ROOT_DIR}/include/geometry
	PRIVATE ${PHYSX_ROOT_DIR}/include/pvd
	PRIVATE ${PHYSX_ROOT_DIR}/include/geomutils

	PRIVATE ${PHYSX_SOURCE_DIR}/common/include
	PRIVATE ${PHYSX_SOURCE_DIR}/common/src

	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/device
	PRIVATE ${PHYSX_SOURCE_DIR}/physx/src/buffering

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
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/api/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/software/include
	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevel/common/include/pipeline

	PRIVATE ${PHYSX_SOURCE_DIR}/lowlevelaabb/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/lowleveldynamics/include

	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/include
	PRIVATE ${PHYSX_SOURCE_DIR}/simulationcontroller/src
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src
	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src/mesh
	PRIVATE ${PHYSX_SOURCE_DIR}/physxcooking/src/convex
	
	PRIVATE ${PHYSX_SOURCE_DIR}/scenequery/include
	
	PRIVATE ${PHYSX_SOURCE_DIR}/physxmetadata/core/include

    PRIVATE ${PHYSX_SOURCE_DIR}/immediatemode/include

    PRIVATE ${PHYSX_SOURCE_DIR}/pvd/include
)

TARGET_COMPILE_DEFINITIONS(PhysX 

	# Common to all configurations
	PRIVATE ${PHYSX_COMPILE_DEFS}
)

SET_TARGET_PROPERTIES(PhysX PROPERTIES
	OUTPUT_NAME PhysX

)

IF(NV_USE_GAMEWORKS_OUTPUT_DIRS AND PHYSX_LIBTYPE STREQUAL "STATIC")
	SET_TARGET_PROPERTIES(PhysX PROPERTIES 
		ARCHIVE_OUTPUT_NAME_DEBUG "PhysX_static"
		ARCHIVE_OUTPUT_NAME_CHECKED "PhysX_static"
		ARCHIVE_OUTPUT_NAME_PROFILE "PhysX_static"
		ARCHIVE_OUTPUT_NAME_RELEASE "PhysX_static"
	)
ENDIF()

IF(PHYSX_COMPILE_PDB_NAME_DEBUG)
	SET_TARGET_PROPERTIES(PhysX PROPERTIES 
		COMPILE_PDB_NAME_DEBUG "${PHYSX_COMPILE_PDB_NAME_DEBUG}"
		COMPILE_PDB_NAME_CHECKED "${PHYSX_COMPILE_PDB_NAME_CHECKED}"
		COMPILE_PDB_NAME_PROFILE "${PHYSX_COMPILE_PDB_NAME_PROFILE}"
		COMPILE_PDB_NAME_RELEASE "${PHYSX_COMPILE_PDB_NAME_RELEASE}"
	)
ENDIF()

TARGET_LINK_LIBRARIES(PhysX 
	PRIVATE ${PHYSX_PRIVATE_PLATFORM_LINKED_LIBS}
	PRIVATE PhysXPvdSDK
	PUBLIC PhysXCommon 
	PUBLIC PhysXFoundation
	PUBLIC ${PHYSX_PLATFORM_LINKED_LIBS}
)

SET_TARGET_PROPERTIES(PhysX PROPERTIES 
	LINK_FLAGS "${PHYSX_PLATFORM_LINK_FLAGS}"
	LINK_FLAGS_DEBUG "${PHYSX_PLATFORM_LINK_FLAGS_DEBUG}"
	LINK_FLAGS_CHECKED "${PHYSX_PLATFORM_LINK_FLAGS_CHECKED}"
	LINK_FLAGS_PROFILE "${PHYSX_PLATFORM_LINK_FLAGS_PROFILE}"
	LINK_FLAGS_RELEASE "${PHYSX_PLATFORM_LINK_FLAGS_RELEASE}"
)

IF(PX_GENERATE_SOURCE_DISTRO)
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_COMMON_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_PVD_HEADERS})	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_METADATA_HEADERS})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_METADATA_SOURCE})	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_CORE_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_BUFFERING_SOURCE})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_IMMEDIATEMODE_SOURCE})	
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_PLATFORM_SRC_FILES})
	LIST(APPEND SOURCE_DISTRO_FILE_LIST ${PHYSX_SOLVER_HEADERS})
ENDIF()

# enable -fPIC so we can link static libs with the editor
SET_TARGET_PROPERTIES(PhysX PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

