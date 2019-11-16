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
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")
TARGET_LINK_LIBRARIES(Bullet3OpenCL_clew LinearMath Bullet3Dynamics ${CMAKE_DL_LIBS})
TARGET_COMPILE_DEFINITIONS(Bullet3OpenCL_clew PUBLIC
	$<$<STREQUAL:$<TARGET_PROPERTY:Bullet3OpenCL_clew,TYPE>,SHARED_LIBRARY>:BULLET_SHARED>)
TARGET_COMPILE_DEFINITIONS(Bullet3OpenCL_clew PRIVATE B3_USE_CLEW)
SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES SOVERSION ${BULLET_VERSION})

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	BULLET3_INSTALL(TARGETS Bullet3OpenCL_clew DESTINATION .)

	SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES FRAMEWORK ON)
	SET_TARGET_PROPERTIES(Bullet3OpenCL_clew PROPERTIES PUBLIC_HEADER "${Root_HDRS}")
	# Have to list out sub-directories manually:
	SET_PROPERTY(SOURCE ${BroadphaseCollision_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/BroadphaseCollision)
ELSE()
	BULLET3_INSTALL(TARGETS Bullet3OpenCL_clew
		RUNTIME DESTINATION ${BULLET_INSTALL_BINDIR}
		LIBRARY DESTINATION ${BULLET_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${BULLET_INSTALL_LIBDIR})
	BULLET3_INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		DESTINATION ${BULLET_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h" PATTERN "CMakeFiles" EXCLUDE)
	# BULLET3_INSTALL(FILES ../btBullet3OpenCL_clewCommon.h DESTINATION ${BULLET_INSTALL_INCLUDEDIR}/Bullet3OpenCL_clew)
ENDIF()
