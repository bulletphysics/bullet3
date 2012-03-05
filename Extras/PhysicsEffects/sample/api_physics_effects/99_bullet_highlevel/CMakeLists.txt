cmake_minimum_required(VERSION 2.4)


#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(App_99_BulletPhysicsEffects)


IF (NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE) 


OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" ON)
OPTION(USE_MSVC_INCREMENTAL_LINKING "Use MSVC Incremental Linking" OFF)

   
IF(COMMAND cmake_policy)
   cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
   

SET(App_99_BulletPhysicsEffects_SRCS
	main.cpp
	bullet_physics_func.cpp
	../common/ctrl_func.win32.cpp
	../common/perf_func.win32.cpp
	../common/render_func.win32.cpp
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btBulletPhysicsEffectsWorld.cpp
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelConstraintSolver2.cpp
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelCollisionDispatcher.cpp
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelBroadphase.cpp
	
)


SET(App_99_BulletPhysicsEffects_HDRS
	physics_func.h
	landscape.h
	barrel.h
	../common/render_func.h
	../common/common.h
	../common/ctrl_func.h
	../common/perf_func.h
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btBulletPhysicsEffectsWorld.h
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelConstraintSolver2.h
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelCollisionDispatcher.h
	${PHYSICS_EFFECTS_SOURCE_DIR}/include/BulletPhysicsEffects/btLowLevelBroadphase.h
)


INCLUDE_DIRECTORIES(
	../../../../../src
#	${VECTOR_MATH_INCLUDE}
)

#LINK_DIRECTORIES(
#
#)

#ADD_DEFINITIONS(-DUNICODE)
#ADD_DEFINITIONS(-D_UNICODE)

ADD_EXECUTABLE(App_99_BulletPhysicsEffects WIN32
	${App_99_BulletPhysicsEffects_SRCS} 
	${App_99_BulletPhysicsEffects_HDRS} 
)
TARGET_LINK_LIBRARIES(App_99_BulletPhysicsEffects
	PfxBaseLevel
	PfxLowLevel
	PfxUtil
	BulletDynamics
	BulletCollision
	LinearMath
	${OPENGL_gl_LIBRARY} 
	${OPENGL_glu_LIBRARY}
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
		SET_TARGET_PROPERTIES(App_99_BulletPhysicsEffects PROPERTIES  DEBUG_POSTFIX "_Debug")
		SET_TARGET_PROPERTIES(App_99_BulletPhysicsEffects PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
		SET_TARGET_PROPERTIES(App_99_BulletPhysicsEffects PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF()



	