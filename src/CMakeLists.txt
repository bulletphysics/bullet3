if (CMAKE_SIZEOF_VOID_P MATCHES "8")
SUBDIRS( BulletSoftBody BulletCollision BulletDynamics Linea
rMath )
else (CMAKE_SIZEOF_VOID_P MATCHES "8")
SUBDIRS( BulletMultiThreaded BulletSoftBody BulletCollision BulletDynamics Linea
rMath )
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
