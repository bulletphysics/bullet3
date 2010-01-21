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


SET(SERIALIZE_FILES 
		bChunk.cpp
		bChunk.h
		bCommon.h
		bDefines.h
		bDNA.cpp
		bDNA.h
		bFile.cpp
		bFile.h
		btBulletFile.cpp
		btBulletFile.h
)

IF (USE_GLUT)
	LINK_LIBRARIES(
	OpenGLSupport  BulletDynamics  BulletCollision LinearMath  ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppSerializeDemo
		main.cpp
		$(SERIALIZE_FILES)
		SerializeDemo.cpp 
		SerializeDemo.h
	)
ELSE (USE_GLUT)

	LINK_LIBRARIES(
	OpenGLSupport  BulletDynamics  BulletCollision LinearMath  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppSerializeDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		Win32SerializeDemo.cpp
		${SERIALIZE_FILES}
		SerializeDemo.cpp 
		SerializeDemo.h
	)
ENDIF (USE_GLUT)