INCLUDE_DIRECTORIES( 
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader
)

ADD_LIBRARY(
BulletWorldImporter 
btBulletWorldImporter.cpp 
btBulletWorldImporter.h 
)

IF (BUILD_SHARED_LIBS)
        TARGET_LINK_LIBRARIES(BulletWorldImporter BulletDynamics BulletCollision BulletFileLoader LinearMath)
ENDIF (BUILD_SHARED_LIBS)

