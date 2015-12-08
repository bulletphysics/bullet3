SUBDIRS( Serialize ConvexDecomposition HACD GIMPACTUtils )

IF(BUILD_BULLET3)
	SUBDIRS( InverseDynamics)
ENDIF()

#Maya Dynamica plugin is moved to http://dynamica.googlecode.com

#IF (USE_GLUT AND GLUT_FOUND)
#	SUBDIRS (glui)
#ENDIF ()
