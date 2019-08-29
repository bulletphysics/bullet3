SET(Bullet3OpenCL_clew_SRCS
	../clew/clew.c
	BroadphaseCollision/b3GpuGridBroadphase.cpp
	BroadphaseCollision/b3GpuSapBroadphase.cpp
	BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.cpp
	BroadphaseCollision/b3GpuParallelLinearBvh.cpp
	Initialize/b3OpenCLUtils.cpp
	NarrowphaseCollision/b3ContactCache.cpp
	NarrowphaseCollision/b3ConvexHullContact.cpp
	NarrowphaseCollision/b3GjkEpa.cpp
	NarrowphaseCollision/b3OptimizedBvh.cpp
	NarrowphaseCollision/b3QuantizedBvh.cpp
	NarrowphaseCollision/b3StridingMeshInterface.cpp
	NarrowphaseCollision/b3TriangleCallback.cpp
	NarrowphaseCollision/b3TriangleIndexVertexArray.cpp
	NarrowphaseCollision/b3VoronoiSimplexSolver.cpp
	ParallelPrimitives/b3BoundSearchCL.cpp
	ParallelPrimitives/b3FillCL.cpp
	ParallelPrimitives/b3LauncherCL.cpp
	ParallelPrimitives/b3PrefixScanCL.cpp
	ParallelPrimitives/b3PrefixScanFloat4CL.cpp
	ParallelPrimitives/b3RadixSort32CL.cpp
	Raycast/b3GpuRaycast.cpp
	RigidBody/b3GpuGenericConstraint.cpp
	RigidBody/b3GpuJacobiContactSolver.cpp
	RigidBody/b3GpuNarrowPhase.cpp
	RigidBody/b3GpuPgsConstraintSolver.cpp
	RigidBody/b3GpuPgsContactSolver.cpp
	RigidBody/b3GpuRigidBodyPipeline.cpp
	RigidBody/b3Solver.cpp
)

SET(Bullet3OpenCL_clew_HDRS
	Bullet3OpenCLApi.h
	# ${Root_HDRS}
)

ADD_LIBRARY(Bullet3OpenCL_clew ${Bullet3OpenCL_clew_SRCS} ${Bullet3OpenCL_clew_HDRS})
TARGET_INCLUDE_DIRECTORIES(Bullet3OpenCL_clew PUBLIC
	"$<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>"
	"$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>")
TARGET_LINK_LIBRARIES(Bullet3OpenCL_clew LinearMath Bullet3Dynamics ${CMAKE_DL_LIBS})
TARGET_COMPILE_DEFINITIONS(Bullet3OpenCL_clew PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:Bullet3OpenCL_clew,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
TARGET_COMPILE_DEFINITIONS(Bullet3OpenCL_clew PRIVATE B3_USE_CLEW)
SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES SOVERSION ${BULLET_VERSION})

IF(INSTALL_LIBS)
	IF(NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS Bullet3OpenCL_clew DESTINATION .)
		ELSE()
			INSTALL(TARGETS Bullet3OpenCL_clew RUNTIME DESTINATION bin
											LIBRARY DESTINATION lib${LIB_SUFFIX}
											ARCHIVE DESTINATION lib${LIB_SUFFIX})
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
				DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
			# INSTALL(FILES ../btBullet3OpenCL_clewCommon.h DESTINATION ${INCLUDE_INSTALL_DIR}/Bullet3OpenCL_clew)
		ENDIF()
		IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES PUBLIC_HEADER "${Root_HDRS}")
			# Have to list out sub-directories manually:
			SET_PROPERTY(SOURCE ${BroadphaseCollision_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/BroadphaseCollision)
		ENDIF()
	ENDIF()
ENDIF()
