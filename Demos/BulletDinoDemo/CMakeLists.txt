# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# This is the variable for Windows.  I use this to define the root of my directory structure.
SET(GLUT_ROOT ${BULLET_PHYSICS_SOURCE_DIR}/Glut)

# You shouldn't have to modify anything below this line 
########################################################


# This is the shortcut to finding GLU, GLUT and OpenGL if they are properly installed on your system
# This should be the case.
INCLUDE (${CMAKE_ROOT}/Modules/FindGLU.cmake)
INCLUDE (${CMAKE_ROOT}/Modules/FindGLUT.cmake) 
INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)


IF (WIN32)
  # This is the Windows code for which Opengl, and Glut are not properly installed
  # since I can't install them I must cheat and copy libraries around
  INCLUDE_DIRECTORIES(${GLUT_ROOT})
 # LINK_DIRECTORIES(${GLUT_ROOT}\\lib)
  IF (${GLUT_glut_LIBRARY} MATCHES "GLUT_glut_LIBRARY-NOTFOUND")
  SET(GLUT_glut_LIBRARY ${BULLET_PHYSICS_SOURCE_DIR}/Glut/glut32.lib)
 #   LINK_LIBRARIES(${GLUT_ROOT}\\lib\\glut32 ${OPENGL_gl_LIBRARY} ${OPENGL_glU_LIBRARY})
 #   TARGET_LINK_LIBRARIES(table ${GLUT_ROOT}\\lib\\glut32)
#
#    ADD_CUSTOM_COMMAND(TARGET table POST_BUILD COMMAND copy ${GLUT_ROOT}\\lib\\glut32.dll ${GLUT_ROOT}\\bin\\vs2005\\Debug
#                        COMMAND copy ${GLUT_ROOT}\\lib\\glut32.dll ${GLUT_ROOT}\\bin\\vs2003\\Debug 
#            COMMAND copy ${GLUT_ROOT}\\lib\\glut32.dll ${GLUT_ROOT}\\bin\\vs6\\Debug)
  ELSE (${GLUT_glut_LIBRARY} MATCHES "GLUT_glut_LIBRARY-NOTFOUND")
#    LINK_LIBRARIES(${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glU_LIBRARY}) 
#    TARGET_LINK_LIBRARIES(table ${GLUT_glut_LIBRARY})
  ENDIF(${GLUT_glut_LIBRARY} MATCHES "GLUT_glut_LIBRARY-NOTFOUND")
#  TARGET_LINK_LIBRARIES(table ${OPENGL_gl_LIBRARY})
#  TARGET_LINK_LIBRARIES(table ${OPENGL_glu_LIBRARY}) 
ELSE (WIN32)
  # This is the lines for linux.  This should always work if everything is installed and working fine.
#  SET(CMAKE_BUILD_TYPE Debug)
#  SET(CMAKE_CXX_FLAGS_DEBUG "-g")
  INCLUDE_DIRECTORIES(/usr/include /usr/local/include ${GLUT_INCLUDE_DIR}) 
#  TARGET_LINK_LIBRARIES(table ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glU_LIBRARY})
#  TARGET_LINK_LIBRARIES(checker ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glU_LIBRARY})
ENDIF (WIN32)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL  
)

LINK_LIBRARIES(
 LibOpenGLSupport LibBulletDynamics  LibBulletCollision LibLinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glU_LIBRARY}
)

ADD_EXECUTABLE(BulletDino
	BulletDino.c	
)

