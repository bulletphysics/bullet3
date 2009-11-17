# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

# You shouldn't have to modify anything below this line 
########################################################

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
OpenGLSupport  BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(AppGpu2dDemo
	main.cpp
	BasicDemo.cpp
	BasicDemo.h
	btGpuDemoPairCache.cpp
	btGpuDemoPairCache.h
	btGpuDemo2dSharedTypes.h
	btGpuDemo2dCpuFunc.cpp
	btGpuDemoDynamicsWorld.cpp
	btGpuDemoDynamicsWorld.h
	oecakeLoader.cpp
	oecakeLoader.h
	btGpuDemo2dSharedCode.h
	btGpuDemo2dSharedDefs.h
)


