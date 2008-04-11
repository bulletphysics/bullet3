
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

ADD_LIBRARY(LibBulletSoftBody
	btSoftBody.cpp
	btSoftBody.h
	btSoftBodyHelpers.cpp
	btSparseSDF.h
	btDbvt.cpp  
	btDbvt.h
	btSoftBodyHelpers.h
	btSoftBodyRigidBodyCollisionConfiguration.cpp
	btSoftRigidCollisionAlgorithm.cpp
	btSoftRigidCollisionAlgorithm.h
	btSoftSoftCollisionAlgorithm.cpp
	btSoftSoftCollisionAlgorithm.h
	btSoftRigidDynamicsWorld.h
	btSoftRigidDynamicsWorld.cpp
)
