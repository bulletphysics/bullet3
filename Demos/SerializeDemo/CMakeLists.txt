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
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter
)


IF (USE_GLUT)
	LINK_LIBRARIES(
	OpenGLSupport  BulletWorldImporter BulletDynamics  BulletCollision LinearMath BulletFileLoader  ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppSerializeDemo
			main.cpp
			SerializeDemo.cpp 
			SerializeDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
		)
	ELSE()
		ADD_EXECUTABLE(AppSerializeDemo
			main.cpp
			SerializeDemo.cpp 
			SerializeDemo.h
		)
	ENDIF()
	
IF (WIN32)
	IF (CMAKE_CL_64)
		ADD_CUSTOM_COMMAND(
			TARGET AppSerializeDemo
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
		)
	ELSE(CMAKE_CL_64)
		ADD_CUSTOM_COMMAND(
			TARGET AppSerializeDemo
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
		)
	ENDIF(CMAKE_CL_64)
ENDIF(WIN32)

	
ELSE (USE_GLUT)

	LINK_LIBRARIES(
	OpenGLSupport  BulletWorldImporter BulletDynamics  BulletCollision LinearMath BulletFileLoader ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppSerializeDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		Win32SerializeDemo.cpp
		SerializeDemo.cpp 
		SerializeDemo.h
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ENDIF (USE_GLUT)

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET AppSerializeDemo 
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/testFileOriginal.bullet ${CMAKE_CURRENT_BINARY_DIR}/testFile.bullet
####### COMMAND copy /Y \"${BULLET_PHYSICS_SOURCE_DIR}/Demos/SerializeDemo/testFile.bullet\" \"${CMAKE_CURRENT_BINARY_DIR}\"
	)
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
