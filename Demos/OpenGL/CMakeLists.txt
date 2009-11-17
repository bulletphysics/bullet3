# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  




# You shouldn't have to modify anything below this line 
########################################################




INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexHull  
)

ADD_LIBRARY(OpenGLSupport
		GLDebugFont.cpp
		GL_DialogDynamicsWorld.cpp
		GL_DialogWindow.cpp
		GL_ShapeDrawer.cpp
		GL_Simplex1to4.cpp
		GLDebugDrawer.cpp
		GlutStuff.cpp
		RenderTexture.cpp
		DemoApplication.cpp
		GlutDemoApplication.cpp
)

IF (BUILD_SHARED_LIBS)
  TARGET_LINK_LIBRARIES(OpenGLSupport BulletDynamics BulletCollision ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF (BUILD_SHARED_LIBS)
