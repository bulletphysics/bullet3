# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src 
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath 
)

IF (WIN32)
	ADD_EXECUTABLE(App_HelloWorld
		HelloWorld.cpp 
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(App_HelloWorld
		HelloWorld.cpp 
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)