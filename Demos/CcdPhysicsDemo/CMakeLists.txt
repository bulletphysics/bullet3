# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################



INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
OpenGLSupport  BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

IF (WIN32)
	ADD_EXECUTABLE(AppCcdPhysicsDemo
		main.cpp
		CcdPhysicsDemo.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppCcdPhysicsDemo
		main.cpp
		CcdPhysicsDemo.cpp
	)
ENDIF()

IF (WIN32)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppCcdPhysicsDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
			)
		ELSE(CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppCcdPhysicsDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
			)
		ENDIF(CMAKE_CL_64)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF(WIN32)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppCcdPhysicsDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppCcdPhysicsDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppCcdPhysicsDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)