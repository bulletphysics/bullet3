if (CMAKE_SIZEOF_VOID_P MATCHES "8")
SUBDIRS( BulletSoftBody BulletCollision BulletDynamics LinearMath )
else (CMAKE_SIZEOF_VOID_P MATCHES "8")
SUBDIRS( BulletMultiThreaded BulletSoftBody BulletCollision BulletDynamics LinearMath )
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

INSTALL(FILES btBulletCollisionCommon.h btBulletDynamicsCommon.h Bullet-C-Api.h DESTINATION include)