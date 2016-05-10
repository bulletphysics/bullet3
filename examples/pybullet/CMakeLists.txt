
INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${PYTHON_INCLUDE_DIRS}
  )

SET(pybullet_SRCS
	pybullet.c
	../../examples/ExampleBrowser/ExampleEntries.cpp
)

IF(WIN32)
LINK_LIBRARIES(
		${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
	)

ENDIF(WIN32)

ADD_LIBRARY(pybullet SHARED ${pybullet_SRCS})

SET_TARGET_PROPERTIES(pybullet PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(pybullet PROPERTIES SOVERSION ${BULLET_VERSION})

TARGET_LINK_LIBRARIES(pybullet BulletExampleBrowserLib BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common ${PYTHON_LIBRARIES})



