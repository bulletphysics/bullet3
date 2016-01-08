IF(BUILD_BULLET3)
	#SUBDIRS( TestBullet3OpenCL )
	SUBDIRS(  InverseDynamics )
ENDIF(BUILD_BULLET3)

SUBDIRS(  gtest-1.7.0  collision BulletDynamics/pendulum )

