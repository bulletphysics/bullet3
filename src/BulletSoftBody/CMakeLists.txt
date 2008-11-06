
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

SET(LibBulletSoftBody_SRCS
	btSoftBody.cpp
	btSoftBodyHelpers.cpp
	btSoftBodyRigidBodyCollisionConfiguration.cpp
	btSoftRigidCollisionAlgorithm.cpp
	btSoftSoftCollisionAlgorithm.cpp
	btSoftBodyConcaveCollisionAlgorithm.cpp
	btSoftRigidDynamicsWorld.cpp
)

SET(LibBulletSoftBody_HDRS
	btSoftBody.h
	btSparseSDF.h
	btSoftBodyHelpers.h
	btSoftRigidCollisionAlgorithm.h
	btSoftSoftCollisionAlgorithm.h
	btSoftBodyConcaveCollisionAlgorithm.h
	btSoftRigidDynamicsWorld.h
)



#ADD_LIBRARY(LibBulletSoftBody SHARED ${LibBulletSoftBody_SRCS} ${LibBulletSoftBody_HDRS})
ADD_LIBRARY(LibBulletSoftBody  ${LibBulletSoftBody_SRCS} ${LibBulletSoftBody_HDRS})
IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(LibBulletSoftBody LibBulletDynamics)
ENDIF (BUILD_SHARED_LIBS)

IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)
	INSTALL(TARGETS LibBulletSoftBody DESTINATION lib)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include ${LibBulletSoftBody_HDRS})
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.6)

IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	SET_TARGET_PROPERTIES(LibBulletSoftBody PROPERTIES FRAMEWORK true)
	SET_TARGET_PROPERTIES(LibBulletSoftBody PROPERTIES PUBLIC_HEADER "${LibBulletSoftBody_HDRS}")
ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
