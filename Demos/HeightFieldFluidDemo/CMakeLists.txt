# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  




# You shouldn't have to modify anything below this line 
########################################################



INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
OpenGLSupport BulletDynamics  BulletCollision LinearMath ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(HfFluidDemo
	HfFluidDemo_GL_ShapeDrawer.cpp
	BulletHfFluid/btHfFluid.cpp
	BulletHfFluid/btHfFluidBuoyantConvexShape.cpp
	BulletHfFluid/btHfFluidBuoyantShapeCollisionAlgorithm.cpp
	BulletHfFluid/btHfFluidCollisionShape.cpp
	BulletHfFluid/btHfFluidRigidCollisionAlgorithm.cpp
	BulletHfFluid/btHfFluidRigidCollisionConfiguration.cpp
	BulletHfFluid/btHfFluidRigidDynamicsWorld.cpp
	main.cpp
	HfFluidDemo.cpp
)

