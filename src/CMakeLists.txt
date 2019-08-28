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

IF(INSTALL_LIBS)
	IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		# Don't actually need to install any common files, the frameworks include everything
	ELSE()
		INSTALL(FILES btBulletCollisionCommon.h btBulletDynamicsCommon.h DESTINATION ${INCLUDE_INSTALL_DIR})
	ENDIF()
ENDIF()
