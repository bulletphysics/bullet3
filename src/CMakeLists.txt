TARGET_INCLUDE_DIRECTORIES(BulletInterface INTERFACE
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
	"$<INSTALL_INTERFACE:${BULLET_INSTALL_INCLUDEDIR}>")

IF(BUILD_BULLET3)
	SUBDIRS(
		Bullet3OpenCL
		Bullet3Serialize/Bullet2FileLoader
		Bullet3Dynamics
		Bullet3Collision
		Bullet3Geometry
	)
ENDIF()

SUBDIRS(
	BulletInverseDynamics
	BulletSoftBody
	BulletCollision
	BulletDynamics
	LinearMath
	Bullet3Common
)

IF(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	# Don't actually need to install any common files, the frameworks include everything
ELSE()
	BULLET3_INSTALL(FILES btBulletCollisionCommon.h btBulletDynamicsCommon.h DESTINATION ${BULLET_INSTALL_INCLUDEDIR})
ENDIF()
