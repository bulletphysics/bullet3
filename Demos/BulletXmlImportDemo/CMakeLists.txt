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
${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletXmlWorldImporter
)

ADD_DEFINITIONS(-DDESERIALIZE_SOFT_BODIES)

IF (USE_GLUT)
	LINK_LIBRARIES(
	OpenGLSupport  BulletXmlWorldImporter BulletWorldImporter BulletSoftBody BulletDynamics  BulletCollision BulletFileLoader LinearMath   ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	IF (WIN32)
		ADD_EXECUTABLE(AppBulletXmlImportDemo
			main.cpp
			BulletXmlImportDemo.cpp 
			BulletXmlImportDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
		)
	ELSE()
		ADD_EXECUTABLE(AppBulletXmlImportDemo
			main.cpp
			BulletXmlImportDemo.cpp 
			BulletXmlImportDemo.h
		)
	ENDIF()
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (WIN32)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppBulletXmlImportDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppBulletXmlImportDemo
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
				)
			ENDIF(CMAKE_CL_64)
		ENDIF(WIN32)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	
ELSE (USE_GLUT)

	LINK_LIBRARIES(
	OpenGLSupport  BulletXmlWorldImporter BulletWorldImporter BulletSoftBody BulletDynamics  BulletCollision BulletFileLoader LinearMath  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

	ADD_EXECUTABLE(AppBulletXmlImportDemo
		WIN32
		../OpenGL/Win32AppMain.cpp
		Win32BulletXmlImportDemo.cpp
		BulletXmlImportDemo.cpp 
		BulletXmlImportDemo.h
		${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
	)
ENDIF (USE_GLUT)

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES AND NOT INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
	ADD_CUSTOM_COMMAND(
		TARGET AppBulletXmlImportDemo 
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/BulletXmlImportDemo/bullet_basic.xml ${CMAKE_CURRENT_BINARY_DIR}/bullet_basic.xml
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/Demos/BulletXmlImportDemo/bullet_basic.xml ${CMAKE_CURRENT_BINARY_DIR}/Debug/bullet_basic.xml
	)
ENDIF ()


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppBulletXmlImportDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppBulletXmlImportDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppBulletXmlImportDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


