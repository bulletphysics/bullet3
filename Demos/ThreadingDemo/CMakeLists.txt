# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

# You shouldn't have to modify anything below this line 
########################################################

#currently this demo has only been tested under Windows 32bit
#IF (WIN32)

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	${VECTOR_MATH_INCLUDE}
)

LINK_LIBRARIES(
BulletMultiThreaded BulletDynamics  BulletCollision LinearMath 
)

IF (WIN32)
ADD_EXECUTABLE(AppThreadingDemo
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppThreadingDemo
		main.cpp
	)
ENDIF()

IF (UNIX)
  TARGET_LINK_LIBRARIES(AppThreadingDemo pthread)
ENDIF(UNIX)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppThreadingDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppThreadingDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppThreadingDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

#ENDIF(WIN32)
