cmake_minimum_required(VERSION 2.4)


#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(App_0_Console)


SET(App_0_Console_SRCS
	main.cpp
	physics_func.cpp
	../common/perf_func.win32.cpp
)

SET(App_0_Console_HDRS
	landscape.h
	physics_func.h
)

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/include
)


#ADD_DEFINITIONS(-DUNICODE)
#ADD_DEFINITIONS(-D_UNICODE)

ADD_EXECUTABLE(App_0_Console
	${App_0_Console_SRCS}
	${App_0_Console_HDRS}
)
TARGET_LINK_LIBRARIES(App_0_Console
	PfxLowLevel
	PfxBaseLevel
	PfxUtil
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
		SET_TARGET_PROPERTIES(App_0_Console PROPERTIES  DEBUG_POSTFIX "_Debug")
		SET_TARGET_PROPERTIES(App_0_Console PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
		SET_TARGET_PROPERTIES(App_0_Console PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF()



	
