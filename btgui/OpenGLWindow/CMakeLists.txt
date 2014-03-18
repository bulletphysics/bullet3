
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
)

FILE(GLOB OpenGLWindow_HDRS "*.h" "../OpenGLTrueTypeFont/*.h")

FILE(GLOB OpenGLWindowMac_CPP "Mac*.mm")
FILE(GLOB OpenGLWindowWin32_CPP "Win32*.cpp")
FILE(GLOB OpenGLWindowLinux_CPP "X11*.cpp")

FILE(GLOB OpenGLWindowCommon_CPP "*.cpp" "../OpenGLTrueTypeFont/*.cpp" "../FontFiles/OpenSans.cpp" )

LIST(REMOVE_ITEM OpenGLWindowCommon_CPP ${OpenGLWindowMac_CPP} )
LIST(REMOVE_ITEM OpenGLWindowCommon_CPP ${OpenGLWindowWin32_CPP} )
LIST(REMOVE_ITEM OpenGLWindowCommon_CPP ${OpenGLWindowLinux_CPP} )
LIST(REMOVE_ITEM OpenGLWindowCommon_CPP X11OpenGLWindow.cpp )
#MESSAGE (${OpenGLWindowCommon_CPP})

IF (WIN32)
	SET(OpenGLWindow_SRCS ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows/glew.c ${OpenGLWindowWin32_CPP} ${OpenGLWindowCommon_CPP})	
	INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ENDIF(WIN32)

IF (APPLE)
	SET(OpenGLWindow_SRCS ${OpenGLWindowMac_CPP} ${OpenGLWindowCommon_CPP} )
ENDIF(APPLE)

#no Linux detection?
IF(NOT WIN32 AND NOT APPLE)
	SET(OpenGLWindow_SRCS ${OpenGLWindowLinux_CPP} ${OpenGLWindowCommon_CPP} )
ENDIF()



ADD_LIBRARY(OpenGLWindow ${OpenGLWindow_SRCS} ${OpenGLWindow_HDRS})

