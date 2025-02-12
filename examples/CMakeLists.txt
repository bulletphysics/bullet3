OPTION(BUILD_BENCHMARKS "Enables building of cli benchmarks" ON)

SUBDIRS( HelloWorld BasicDemo)

IF(BUILD_BENCHMARKS)
  SUBDIRS(Benchmarks)
ENDIF()

IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser RobotSimulator SharedMemory ThirdPartyLibs/Gwen ThirdPartyLibs/BussIK ThirdPartyLibs/clsocket OpenGLWindow TwoJoint )
ENDIF()

IF(BUILD_PYBULLET)
	SUBDIRS(pybullet)
ENDIF(BUILD_PYBULLET)
