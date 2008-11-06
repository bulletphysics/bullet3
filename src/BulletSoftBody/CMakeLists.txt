
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

SET(BulletSoftBody_SRCS
	btSoftBody.cpp
	btSoftBodyHelpers.cpp
	btSoftBodyRigidBodyCollisionConfiguration.cpp
	btSoftRigidCollisionAlgorithm.cpp
	btSoftSoftCollisionAlgorithm.cpp
	btSoftBodyConcaveCollisionAlgorithm.cpp
	btSoftRigidDynamicsWorld.cpp
)

SET(BulletSoftBody_HDRS
	btSoftBody.h
	btSparseSDF.h
	btSoftBodyHelpers.h
	btSoftRigidCollisionAlgorithm.h
	btSoftSoftCollisionAlgorithm.h
	btSoftBodyConcaveCollisionAlgorithm.h
	btSoftRigidDynamicsWorld.h
)



#ADD_LIBRARY(BulletSoftBody SHARED ${BulletSoftBody_SRCS} ${BulletSoftBody_HDRS})
ADD_LIBRARY(BulletSoftBody  ${BulletSoftBody_SRCS} ${BulletSoftBody_HDRS})
IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletSoftBody BulletDynamics)
ENDIF (BUILD_SHARED_LIBS)

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
	INSTALL(TARGETS BulletSoftBody DESTINATION lib)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include ${BulletSoftBody_HDRS})
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)

IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES FRAMEWORK true)
	SET_TARGET_PROPERTIES(BulletSoftBody PROPERTIES PUBLIC_HEADER "${BulletSoftBody_HDRS}")
ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
