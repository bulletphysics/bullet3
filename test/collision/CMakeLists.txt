
INCLUDE_DIRECTORIES(
	.
	../../src
	../gtest-1.7.0/include
)


#ADD_DEFINITIONS(-DGTEST_HAS_PTHREAD=1)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(
 LinearMath gtest
)

IF (NOT WIN32)
	FIND_PACKAGE(Threads)
	LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} )
ENDIF()

	ADD_EXECUTABLE(Test_Collision
		main.cpp
		../../src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
		../../src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
		../../src/BulletCollision/CollisionShapes/btSphereShape.cpp
		../../src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp
		../../src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
		../../src/BulletCollision/CollisionShapes/btConvexShape.cpp
		../../src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
		../../src/BulletCollision/CollisionShapes/btCollisionShape.cpp
		../../src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
	)

ADD_TEST(Test_Collision_PASS Test_Collision)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_Collision PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
