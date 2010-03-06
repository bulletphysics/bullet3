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

ADD_EXECUTABLE(AppBspPhysicsDemo
	main.cpp
	BspDemo.cpp
	BspLoader.cpp
	BspConverter.cpp	
)


IF (WIN32)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppBspPhysicsDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
			)
		ELSE(CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppBspPhysicsDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
			)
		ENDIF(CMAKE_CL_64)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF(WIN32)



IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET AppBspPhysicsDemo
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/BspDemo.bsp ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppBspPhysicsDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppBspPhysicsDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppBspPhysicsDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)