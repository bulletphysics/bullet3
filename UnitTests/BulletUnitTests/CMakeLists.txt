
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/UnitTests/cppunit/include
	
	
	${VECTOR_MATH_INCLUDE}
)

LINK_LIBRARIES(
	cppunit BulletMultiThreaded BulletDynamics  BulletCollision LinearMath
)
	
ADD_EXECUTABLE(AppBulletUnitTests
			Main.cpp
		  TestBulletOnly.h
		  TestLinearMath.h
)