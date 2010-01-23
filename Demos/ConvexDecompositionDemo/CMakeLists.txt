# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

# You shouldn't have to modify anything below this line 
########################################################

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexDecomposition
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
)

IF (USE_GLUT)

	LINK_LIBRARIES(
	OpenGLSupport  BulletDynamics  BulletCollision LinearMath  BulletFileLoader ConvexDecomposition ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppConvexDecompositionDemo
		main.cpp
		ConvexDecompositionDemo.cpp 
		ConvexDecompositionDemo.h
	)
ELSE (USE_GLUT)

LINK_LIBRARIES(
	OpenGLSupport  BulletDynamics  BulletCollision LinearMath BulletFileLoader  ConvexDecomposition ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppConvexDecompositionDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		ConvexDecompositionDemo.cpp 
		ConvexDecompositionDemo.h
		Win32ConvexDecompositionDemo.cpp
	)
	
ENDIF (USE_GLUT)

