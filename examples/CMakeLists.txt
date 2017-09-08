SUBDIRS( HelloWorld BasicDemo )
IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser RobotSimulator SharedMemory ThirdPartyLibs/Gwen ThirdPartyLibs/BussIK ThirdPartyLibs/clsocket OpenGLWindow )
ENDIF()

IF(BUILD_PYBULLET)
	SUBDIRS(pybullet)
ENDIF(BUILD_PYBULLET)
