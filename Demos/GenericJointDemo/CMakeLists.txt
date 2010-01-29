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

	ADD_EXECUTABLE(AppGenericJointDemo
		GenericJointDemo.cpp
		GenericJointDemo.h
		Ragdoll.cpp
		main.cpp
	)

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
		
	)
ENDIF (USE_GLUT)