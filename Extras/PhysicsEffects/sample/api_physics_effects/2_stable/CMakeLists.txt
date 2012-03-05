cmake_minimum_required(VERSION 2.4)


#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(App_2_Stable)


IF (NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 


OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" ON)
OPTION(USE_MSVC_INCREMENTAL_LINKING "Use MSVC Incremental Linking" OFF)

   
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
   

SET(App_2_Stable_SRCS
	main.cpp
	physics_func.cpp
	../common/ctrl_func.win32.cpp
	../common/perf_func.win32.cpp
	../common/render_func.win32.cpp
)


SET(App_2_Stable_HDRS
	physics_func.h
	landscape.h
	barrel.h
	../common/render_func.h
	../common/common.h
	../common/ctrl_func.h
	../common/perf_func.h
)


INCLUDE_DIRECTORIES(
	${PHYSICS_EFFECTS_SOURCE_DIR}/include
#	${VECTOR_MATH_INCLUDE}
)

#LINK_DIRECTORIES(
#
#)

#ADD_DEFINITIONS(-DUNICODE)
#ADD_DEFINITIONS(-D_UNICODE)

ADD_EXECUTABLE(App_2_Stable WIN32
	${App_2_Stable_SRCS} 
	${App_2_Stable_HDRS} 
)
TARGET_LINK_LIBRARIES(App_2_Stable
	PfxBaseLevel
	PfxLowLevel
	PfxUtil
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY}
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
		SET_TARGET_PROPERTIES(App_2_Stable PROPERTIES  DEBUG_POSTFIX "_Debug")
		SET_TARGET_PROPERTIES(App_2_Stable PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
		SET_TARGET_PROPERTIES(App_2_Stable PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF()



	