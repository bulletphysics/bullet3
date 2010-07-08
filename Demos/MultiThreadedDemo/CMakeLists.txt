# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
${VECTOR_MATH_INCLUDE}
)

LINK_LIBRARIES(
OpenGLSupport   BulletMultiThreaded BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(AppMultiThreadedDemo
	main.cpp
	MultiThreadedDemo.cpp
	MultiThreadedDemo.h
)
IF (UNIX)
  TARGET_LINK_LIBRARIES(AppMultiThreadedDemo pthread)
ENDIF(UNIX)

IF(WIN32)
IF (CMAKE_CL_64)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		ADD_CUSTOM_COMMAND(	TARGET AppMultiThreadedDemo		POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}	)
		ADD_CUSTOM_COMMAND(	TARGET AppMultiThreadedDemo	POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLEW64.DLL ${CMAKE_CURRENT_BINARY_DIR})
	ENDIF()
ELSE(CMAKE_CL_64)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		ADD_CUSTOM_COMMAND(	TARGET AppMultiThreadedDemo	POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR})
		ADD_CUSTOM_COMMAND(	TARGET AppMultiThreadedDemo	POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLEW32.DLL ${CMAKE_CURRENT_BINARY_DIR})
	ENDIF()
ENDIF(CMAKE_CL_64)
ENDIF(WIN32)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppMultiThreadedDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppMultiThreadedDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppMultiThreadedDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)