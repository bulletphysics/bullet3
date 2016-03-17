SUBDIRS( HelloWorld BasicDemo )
IF(BUILD_BULLET3)
	SUBDIRS( ExampleBrowser ThirdPartyLibs/Gwen OpenGLWindow)
ENDIF()

