SUBDIRS( Serialize ConvexDecomposition GIMPACTUtils )

#Maya Dynamica plugin is moved to http://dynamica.googlecode.com

IF (USE_GLUT)
SUBDIRS (glui)
ENDIF (USE_GLUT)
