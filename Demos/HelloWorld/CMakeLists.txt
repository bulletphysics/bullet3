# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

LINK_LIBRARIES(
 LibBulletDynamics LibBulletCollision LibLinearMath 
)

ADD_EXECUTABLE(HelloWorld
	HelloWorld.cpp 
)

