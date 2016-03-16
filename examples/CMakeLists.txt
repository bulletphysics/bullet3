SUBDIRS( HelloWorld BasicDemo )
IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser SimpleOpenGL3 ThirdPartyLibs/Gwen OpenGLWindow)
ENDIF()

