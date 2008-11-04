
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

INSTALL(TARGETS LibBulletSoftBody DESTINATION lib)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")

