

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
)
 
 
SET(App_AllBullet2Demos_SRCS
	main.cpp
	BulletDemoInterface.h
	BulletDemoEntries.h
	../bullet2/BasicDemo/Bullet2RigidBodyDemo.cpp
	../bullet2/BasicDemo/Bullet2RigidBodyDemo.h
	../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.cpp
	../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h
	../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.cpp
	../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h
	../bullet2/BasicDemo/BasicDemo.cpp
	../bullet2/BasicDemo/BasicDemo.h
	../bullet2/BasicDemo/HingeDemo.cpp
	../bullet2/BasicDemo/HingeDemo.h
	../bullet2/ChainDemo/ChainDemo.cpp
	../bullet2/ChainDemo/ChainDemo.h
	../bullet2/RagdollDemo/RagdollDemo.cpp
	../bullet2/RagdollDemo/RagdollDemo.h
#	../bullet2/LuaDemo/LuaDemo.cpp
#	../bullet2/LuaDemo/LuaDemo.h
	../GpuDemos/gwenUserInterface.cpp
	../GpuDemos/gwenUserInterface.h
	../../btgui/Timing/b3Clock.cpp
	../../btgui/Timing/b3Clock.h

	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)

IF (WIN32)
	SET(App_AllBullet2Demos_SRCS ${App_AllBullet2Demos_SRCS}  ${App_AllBullet2Demos_Common_SRCS})
	INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
	IF(NOT APPLE)
		LINK_LIBRARIES( GLEW)
	ENDIF(NOT APPLE)
ENDIF(WIN32)
 
	

LINK_LIBRARIES(
	Bullet3Common BulletSoftBody BulletDynamics BulletCollision LinearMath OpenGLWindow gwen ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(App_AllBullet2Demos
		${App_AllBullet2Demos_SRCS}
)
	

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)