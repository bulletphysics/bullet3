# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
 OpenGLSupport  BulletDynamics BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)
SET(CharacterDemo_SRCS

	DynamicCharacterController.cpp
	DynamicCharacterController.h
	CharacterDemo.cpp
	CharacterDemo.h
	../BspDemo/BspConverter.cpp
	../BspDemo/BspConverter.h
	../BspDemo/BspLoader.cpp
	../BspDemo/BspLoader.h
	main.cpp
)

IF (WIN32)
	ADD_EXECUTABLE(AppCharacterDemo
		${CharacterDemo_SRCS}
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppCharacterDemo
		${CharacterDemo_SRCS}
	)
ENDIF()

IF (WIN32)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppCharacterDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
			)
		ELSE(CMAKE_CL_64)
			ADD_CUSTOM_COMMAND(
				TARGET AppCharacterDemo
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
			)
		ENDIF(CMAKE_CL_64)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF(WIN32)

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET AppCharacterDemo
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/BspDemo.bsp ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)



IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppCharacterDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppCharacterDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppCharacterDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
