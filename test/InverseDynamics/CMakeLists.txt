


INCLUDE_DIRECTORIES(
	.
	../../src
	../gtest-1.7.0/include
	../../Extras/InverseDynamics
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(
 BulletInverseDynamicsUtils BulletInverseDynamics Bullet3Common LinearMath gtest
)
IF (NOT WIN32)
	LINK_LIBRARIES(		pthread	)
ENDIF()


	ADD_EXECUTABLE(Test_BulletInverseDynamics
		test_invdyn_kinematics.cpp
	)

ADD_TEST(Test_BulletInverseDynamics_PASS Test_BulletInverseDynamics)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
