# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################



INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter
)


IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport BulletWorldImporter BulletDynamics  BulletCollision LinearMath  BulletFileLoader  ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppGyroscopicDemo
			GyroscopicDemo.cpp
			GyroscopicDemo.h
			main.cpp
	)
	IF (WIN32)
		IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGyroscopicDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppGyroscopicDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}/Debug

				)
			ENDIF(CMAKE_CL_64)
		ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ENDIF(WIN32)
ELSE (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport BulletWorldImporter BulletDynamics  BulletCollision LinearMath BulletFileLoader ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppGyroscopicDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		Win32GyroscopicDemo.cpp
		GyroscopicDemo.cpp
		GyroscopicDemo.h
	)
ENDIF (USE_GLUT)



IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppGyroscopicDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppGyroscopicDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppGyroscopicDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)