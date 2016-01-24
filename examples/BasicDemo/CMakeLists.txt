# App_BasicExample is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath
)

IF (WIN32)
	ADD_EXECUTABLE(App_BasicExample
		BasicExample.cpp
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(App_BasicExample
		BasicExample.cpp
		main.cpp
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
