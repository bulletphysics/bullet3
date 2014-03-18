# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  




# You shouldn't have to modify anything below this line 
########################################################

IF (USE_GLUT)

	INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	../OpenGL 
	${GLUT_INCLUDE_DIR}
	)

	LINK_LIBRARIES(
	OpenGLSupport  BulletSoftBody BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppSoftBodyDemo
			main.cpp
			SoftDemo.cpp
			${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
		)
	ELSE()
		ADD_EXECUTABLE(AppSoftBodyDemo
			main.cpp
			SoftDemo.cpp
			
		)
	ENDIF()



ELSE(USE_GLUT)

ENDIF (USE_GLUT)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppSoftBodyDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppSoftBodyDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppSoftBodyDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)