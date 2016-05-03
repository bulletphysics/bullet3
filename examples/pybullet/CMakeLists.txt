IF (BUILD_SHARED_LIBS)

INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		/usr/include/python2.7
  )

SET(pybullet_SRCS
	pybullet.c
	../../examples/ExampleBrowser/ExampleEntries.cpp
)


ADD_LIBRARY(pybullet ${pybullet_SRCS})

SET_TARGET_PROPERTIES(pybullet PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(pybullet PROPERTIES SOVERSION ${BULLET_VERSION})

  TARGET_LINK_LIBRARIES(pybullet BulletExampleBrowserLib BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)
ENDIF (BUILD_SHARED_LIBS)


