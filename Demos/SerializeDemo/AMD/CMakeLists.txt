

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
IF (CMAKE_CL_64)
	SET(CMAK_GLEW_LIBRARY
		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew64.lib		)
ELSE(CMAKE_CL_64)
	SET(CMAK_GLEW_LIBRARY		${BULLET_PHYSICS_SOURCE_DIR}/Glut/glew32.lib		)
ENDIF(CMAKE_CL_64)


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
			${CMAKE_ATISTREAMSDK_LIBPATH}/OpenCL.lib
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppSerializeDemo_AMD
			../main.cpp
			../SerializeDemo.cpp 
			../SerializeDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
			${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.c
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.h
		
		)
	ELSE()
		ADD_EXECUTABLE(AppSerializeDemo_AMD
			../main.cpp
			../SerializeDemo.cpp 
			../SerializeDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.c
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/clew.h
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
		${CMAK_GLEW_LIBRARY}
		${CMAKE_ATISTREAMSDK_LIBPATH}/OpenCL.lib
	)

	ADD_EXECUTABLE(AppSerializeDemo_AMD
		WIN32
		../../OpenGL/Win32AppMain.cpp
		../Win32SerializeDemo.cpp
		../SerializeDemo.cpp 
		../SerializeDemo.h
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.h
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclUtils.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/SharedOpenCL/btOclCommon.cpp
	)
ENDIF (USE_GLUT)

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES AND NOT INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
	ADD_CUSTOM_COMMAND(
		TARGET AppSerializeDemo_AMD
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/SerializeDemo/testFile.bullet ${CMAKE_CURRENT_BINARY_DIR}/testFile.bullet
	)
ENDIF ()


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppSerializeDemo_AMD PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
