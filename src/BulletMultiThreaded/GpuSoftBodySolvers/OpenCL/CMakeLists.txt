
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)


SET(OPENCL_DIR $ENV{ATISTREAMSDKROOT})
SET(OPENCL_INCLUDE_PATH "${ATISTREAMSDKROOT}/include" CACHE DOCSTRING "OpenCL SDK include path")

INCLUDE_DIRECTORIES(${OPENCL_INCLUDE_PATH} "../cpu/")

SET(BulletSoftBodyOpenCLSolvers_SRCS
	btSoftBodySolver_OpenCL.cpp
)

SET(BulletSoftBodyOpenCLSolvers_HDRS
	btSoftBodySolver_OpenCL.h
	../cpu/btSoftBodySolverData.h
	btSoftBodySolverVertexData_OpenCL.h
	btSoftBodySolverTriangleData_OpenCL.h
	btSoftBodySolverLinkData_OpenCL.h
	btSoftBodySolverBuffer_OpenCL.h
)

# OpenCL and HLSL Shaders.
# Build rules generated to stringify these into headers
# which are needed by some of the sources
SET(BulletSoftBodyOpenCLSolvers_Shaders
#	OutputToVertexArray
	UpdateNormals
	Integrate
	UpdatePositions
	UpdateNodes
	SolvePositions
	UpdatePositionsFromVelocities
	ApplyForces
	PrepareLinks
	VSolveLinks
)

foreach(f ${BulletSoftBodyOpenCLSolvers_Shaders})
    LIST(APPEND BulletSoftBodyOpenCLSolvers_OpenCLC "OpenCLC/${f}.cl")
endforeach(f) 



ADD_LIBRARY(BulletSoftBodySolvers_OpenCL  ${BulletSoftBodyOpenCLSolvers_SRCS} ${BulletSoftBodyOpenCLSolvers_HDRS} ${BulletSoftBodyOpenCLSolvers_OpenCLC})
SET_TARGET_PROPERTIES(BulletSoftBodySolvers_OpenCL PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletSoftBodySolvers_OpenCL PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletSoftBody BulletDynamics)
ENDIF (BUILD_SHARED_LIBS)


IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletSoftBodySolvers_OpenCL DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS BulletSoftBodySolvers_OpenCL DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletSoftBodySolvers_OpenCL PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletSoftBodySolvers_OpenCL PROPERTIES PUBLIC_HEADER "${BulletSoftBodyOpenCLSolvers_HDRS}")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
