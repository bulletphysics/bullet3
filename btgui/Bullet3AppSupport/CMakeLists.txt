
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
)

FILE(GLOB Bullet3AppSupport_HDRS "*.h" )
FILE(GLOB Bullet3AppSupport_SRCS "*.cpp" )

IF (WIN32)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ENDIF(WIN32)
IF(NOT WIN32 AND NOT APPLE)
        ADD_DEFINITIONS(-DGLEW_STATIC)
	ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
        ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
ENDIF()

ADD_LIBRARY(Bullet3AppSupport ${Bullet3AppSupport_SRCS} ${Bullet3AppSupport_HDRS})
if (UNIX AND NOT APPLE)
  target_link_libraries(Bullet3AppSupport X11)
elseif (APPLE)
  target_link_libraries(Bullet3AppSupport ${COCOA_LIBRARY})
endif ()

if (BUILD_SHARED_LIBS)
  target_link_libraries(Bullet3AppSupport gwen OpenGLWindow ${OPENGL_gl_LIBRARY} BulletCollision LinearMath Bullet3Common)
endif()

