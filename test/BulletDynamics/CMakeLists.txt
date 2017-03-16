SUBDIRS(pendulum )

INCLUDE_DIRECTORIES(
		"${PROJECT_SOURCE_DIR}/src"
		"${PROJECT_SOURCE_DIR}/test/gtest-1.7.0/include")

ADD_DEFINITIONS(-DUSE_GTEST)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(BulletDynamics BulletCollision LinearMath gtest)

IF (NOT WIN32)
	LINK_LIBRARIES(pthread)
ENDIF()

ADD_EXECUTABLE(Test_btKinematicCharacterController test_btKinematicCharacterController.cpp)

ADD_TEST(Test_btKinematicCharacterController_PASS Test_btKinematicCharacterController)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
