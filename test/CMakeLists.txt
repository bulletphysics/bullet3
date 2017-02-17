IF(BUILD_BULLET3)
	#SUBDIRS( TestBullet3OpenCL )
	SUBDIRS(  InverseDynamics SharedMemory )
ENDIF(BUILD_BULLET3)

SUBDIRS(  gtest-1.7.0 collision RobotLogging BulletDynamics/pendulum )

