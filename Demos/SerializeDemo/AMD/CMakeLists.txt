

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter
${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL
${AMD_OPENCL_INCLUDES}
)

ADD_DEFINITIONS(-DDESERIALIZE_SOFT_BODIES)
ADD_DEFINITIONS(-DUSE_AMD_OPENCL)
ADD_DEFINITIONS(-DCL_PLATFORM_AMD)

IF(WIN32)
ADD_DEFINITIONS(-DGLEW_STATIC)
ENDIF(WIN32)


IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport  
		BulletWorldImporter 
		BulletSoftBody 
		BulletDynamics  
		BulletCollision 
		BulletFileLoader 
		LinearMath  
		BulletSoftBodySolvers_OpenCL_AMD
		BulletMultiThreaded
			${GLUT_glut_LIBRARY} 
			${OPENGL_gl_LIBRARY} 
			${OPENGL_glu_LIBRARY}
			${CMAK_GLEW_LIBRARY}
			${CMAKE_ATISTREAMSDK_LIBRARY}
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppSerializeDemo_AMD
			../main.cpp
			../SerializeDemo.cpp 
			../SerializeDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLInclude.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.c
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.h
		
		)
	ELSE()
		ADD_EXECUTABLE(AppSerializeDemo_AMD
			../main.cpp
			../SerializeDemo.cpp 
			../SerializeDemo.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLInclude.h

		)
	ENDIF()
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (WIN32)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppSerializeDemo_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppSerializeDemo_AMD
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
				)
			ENDIF(CMAKE_CL_64)
		ENDIF(WIN32)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	
ELSE (USE_GLUT)

	LINK_LIBRARIES(
		OpenGLSupport  
		BulletWorldImporter 
		BulletSoftBody 
		BulletDynamics  
		BulletCollision 
		BulletFileLoader 
		LinearMath  
		BulletSoftBodySolvers_OpenCL_AMD
		BulletMultiThreaded
		${OPENGL_gl_LIBRARY} 
		${OPENGL_glu_LIBRARY}
		${CMAKE_GLEW_LIBRARY}
		${CMAKE_ATISTREAMSDK_LIBRARY}
	)

	ADD_EXECUTABLE(AppSerializeDemo_AMD
		WIN32
		../../OpenGL/Win32AppMain.cpp
		../Win32SerializeDemo.cpp
		../SerializeDemo.cpp 
		../SerializeDemo.h
		${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOpenCLInclude.h
	)
ENDIF (USE_GLUT)

IF(WIN32)
IF (CMAKE_CL_64)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		ADD_CUSTOM_COMMAND(	TARGET AppSerializeDemo_AMD		POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}	)
	ENDIF()
ELSE(CMAKE_CL_64)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		ADD_CUSTOM_COMMAND(	TARGET AppSerializeDemo_AMD	POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR})
	ENDIF()
ENDIF(CMAKE_CL_64)
ENDIF(WIN32)


IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES AND NOT INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
	ADD_CUSTOM_COMMAND(
		TARGET AppSerializeDemo_AMD
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SerializeDemo/testFileCloth.bullet ${CMAKE_CURRENT_BINARY_DIR}/testFile.bullet
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/ApplyForces.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/ApplyForces.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/ComputeBounds.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/ComputeBounds.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/Integrate.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/Integrate.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/OutputToVertexArray.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/OutputToVertexArray.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/PrepareLinks.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/PrepareLinks.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/SolveCollisionsAndUpdateVelocities.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/SolveCollisionsAndUpdateVelocities.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/SolveCollisionsAndUpdateVelocitiesSIMDBatched.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/SolveCollisionsAndUpdateVelocitiesSIMDBatched.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/SolvePositions.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/SolvePositions.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/SolvePositionsSIMDBatched.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/SolvePositionsSIMDBatched.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdateConstants.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdateConstants.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdateFixedVertexPositions.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdateFixedVertexPositions.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdateNodes.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdateNodes.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdateNormals.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdateNormals.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdatePositions.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdatePositions.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/UpdatePositionsFromVelocities.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/UpdatePositionsFromVelocities.cl
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/OpenCLC10/VSolveLinks.cl ${CMAKE_CURRENT_BINARY_DIR}/OpenCLC10/VSolveLinks.cl

	)
ENDIF ()


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
