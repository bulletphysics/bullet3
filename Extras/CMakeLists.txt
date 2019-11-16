if(MSVC)
	message(STATUS "Forcing building extras libraries as static libraries")
	set(BUILD_SHARED_LIBS OFF)
endif()

SUBDIRS( InverseDynamics BulletRobotics obj2sdf Serialize ConvexDecomposition HACD GIMPACTUtils )



#Maya Dynamica plugin is moved to http://dynamica.googlecode.com

#IF (USE_GLUT AND GLUT_FOUND)
#	SUBDIRS (glui)
#ENDIF ()
