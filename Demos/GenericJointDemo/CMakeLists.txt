# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################

IF (USE_GLUT)

	INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	)

	LINK_LIBRARIES(
	OpenGLSupport BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppGenericJointDemo
			GenericJointDemo.cpp
			GenericJointDemo.h
			Ragdoll.cpp
			main.cpp
			${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
		)
	ELSE()
		ADD_EXECUTABLE(AppGenericJointDemo
			GenericJointDemo.cpp
			GenericJointDemo.h
			Ragdoll.cpp
			main.cpp
		)
	ENDIF()
	IF (WIN32)
		IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGenericJointDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGenericJointDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
				)
			ENDIF(CMAKE_CL_64)
		ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ENDIF(WIN32)

ELSE(USE_GLUT)

	INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	)

	LINK_LIBRARIES(
	OpenGLSupport BulletDynamics  BulletCollision LinearMath  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppGenericJointDemo
	WIN32
		../OpenGL/Win32AppMain.cpp
		Win32GenericJointDemo.cpp
		GenericJointDemo.cpp
		GenericJointDemo.h
		Ragdoll.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
		
	)
ENDIF (USE_GLUT)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppGenericJointDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppGenericJointDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppGenericJointDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)