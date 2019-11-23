if(MSVC)
	message(STATUS "Forcing building test libraries as static libraries")
	set(BUILD_SHARED_LIBS OFF)
endif()

IF(BUILD_BULLET3)
	#SUBDIRS( TestBullet3OpenCL )
	SUBDIRS(  InverseDynamics SharedMemory )
ENDIF(BUILD_BULLET3)

SUBDIRS(  gtest-1.7.0 collision BulletDynamics )

