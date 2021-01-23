INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
  )


SET(RobotSimulator_SRCS
RobotSimulatorMain.cpp
	b3RobotSimulatorClientAPI.cpp
	b3RobotSimulatorClientAPI.h	
	MinitaurSetup.cpp
	MinitaurSetup.h
	../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/GraphicsClientExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
        ../SharedMemory/RemoteGUIHelperTCP.cpp
	../SharedMemory/GraphicsServerExample.h
	../SharedMemory/GraphicsClientExample.h
	../SharedMemory/RemoteGUIHelper.h
	../SharedMemory/GraphicsSharedMemoryCommands.h
	../SharedMemory/GraphicsSharedMemoryPublic.h
	../../examples/SharedMemory/PhysicsServerExample.cpp
	../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	



IF(BUILD_CLSOCKET)
 ADD_DEFINITIONS(-DBT_ENABLE_CLSOCKET)
 ADD_DEFINITIONS(${OSDEF})
ENDIF(BUILD_CLSOCKET)


#some code to support OpenGL and Glew cross platform
IF (WIN32)
	INCLUDE_DIRECTORIES(
			${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
	IF(APPLE)
		find_library(COCOA NAMES Cocoa)
		MESSAGE(${COCOA})
		link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

	ELSE(APPLE)
		INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
		)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
		ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

		FIND_PACKAGE(Threads)
		LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(App_RobotSimulator  ${RobotSimulator_SRCS})

SET_TARGET_PROPERTIES(App_RobotSimulator PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(App_RobotSimulator PROPERTIES DEBUG_POSTFIX "_d")
SET_TARGET_PROPERTIES(App_RobotSimulator PROPERTIES COMPILE_DEFINITIONS "B3_USE_ROBOTSIM_GUI")


TARGET_LINK_LIBRARIES(App_RobotSimulator BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)

IF(WIN32)
	IF(BUILD_ENET OR BUILD_CLSOCKET)
		TARGET_LINK_LIBRARIES(App_RobotSimulator ws2_32 Winmm)
	ENDIF(BUILD_ENET OR BUILD_CLSOCKET)
ENDIF(WIN32)



INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
  )

SET(RobotSimulator_NoGUI_SRCS
	RobotSimulatorMain.cpp
	MinitaurSetup.cpp
	MinitaurSetup.h
)

ADD_EXECUTABLE(App_RobotSimulator_NoGUI  ${RobotSimulator_NoGUI_SRCS})

SET_TARGET_PROPERTIES(App_RobotSimulator_NoGUI PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(App_RobotSimulator_NoGUI PROPERTIES DEBUG_POSTFIX "_d")


TARGET_LINK_LIBRARIES(App_RobotSimulator_NoGUI BulletRobotics BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath Bullet3Common)

IF(WIN32)
	IF(BUILD_ENET OR BUILD_CLSOCKET)
		TARGET_LINK_LIBRARIES(App_RobotSimulator_NoGUI ws2_32 Winmm)
	ENDIF(BUILD_ENET OR BUILD_CLSOCKET)
ELSE()
	IF(APPLE)
	ELSE(APPLE)
		FIND_PACKAGE(Threads)
		LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)



########

INCLUDE_DIRECTORIES(
                ${BULLET_PHYSICS_SOURCE_DIR}/src
                ${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
                ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
  )

SET(App_HelloBulletRobotics_SRCS
        HelloBulletRobotics.cpp
)

ADD_EXECUTABLE(App_HelloBulletRobotics  ${App_HelloBulletRobotics_SRCS})

SET_TARGET_PROPERTIES(App_HelloBulletRobotics PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(App_HelloBulletRobotics PROPERTIES DEBUG_POSTFIX "_d")


TARGET_LINK_LIBRARIES(App_HelloBulletRobotics BulletRobotics BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath Bullet3Common)

IF(WIN32)
        IF(BUILD_ENET OR BUILD_CLSOCKET)
                TARGET_LINK_LIBRARIES(App_HelloBulletRobotics ws2_32 Winmm)
        ENDIF(BUILD_ENET OR BUILD_CLSOCKET)
ELSE()
	IF(NOT APPLE)
		FIND_PACKAGE(Threads)
		TARGET_LINK_LIBRARIES( App_HelloBulletRobotics ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)


IF(WIN32)
	TARGET_LINK_LIBRARIES(App_RobotSimulator ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(WIN32)



