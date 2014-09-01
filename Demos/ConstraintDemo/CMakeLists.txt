# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################



INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
../OpenGL
${GLUT_INCLUDE_DIR}
)


IF (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport BulletDynamics  BulletCollision LinearMath  ${GLUT_glut_LIBRARY}
 ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppConstraintDemo
			ConstraintDemo.cpp
			ConstraintDemo.h
			main.cpp
	)
ELSE (USE_GLUT)
	LINK_LIBRARIES(
		OpenGLSupport BulletDynamics  BulletCollision LinearMath ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppConstraintDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		Win32ConstraintDemo.cpp
		ConstraintDemo.cpp
		ConstraintDemo.h
	)
ENDIF (USE_GLUT)



IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppConstraintDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppConstraintDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppConstraintDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
