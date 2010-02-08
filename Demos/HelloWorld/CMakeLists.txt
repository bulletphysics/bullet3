# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath 
)

IF (WIN32)
	ADD_EXECUTABLE(AppHelloWorld
		HelloWorld.cpp 
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppHelloWorld
		HelloWorld.cpp 
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppHelloWorld PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppHelloWorld PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppHelloWorld PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)