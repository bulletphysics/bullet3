
ADD_DEFINITIONS(-DB3_USE_CLEW)

INCLUDE_DIRECTORIES(
	..
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
)
 
 
SET(App_Bullet3_OpenCL_Demos_SRCS
  	../../src/clew/clew.c
    ../../btgui/OpenGLWindow/GlewWindows/glew.c
    GpuDemo.cpp
    gwenUserInterface.cpp
    main_opengl3core.cpp
    ParticleDemo.cpp
    broadphase/PairBench.cpp
    constraints/ConstraintsDemo.cpp
    raytrace/RaytracedShadowDemo.cpp
    rigidbody/Bullet2FileDemo.cpp
    rigidbody/BulletDataExtractor.cpp
    rigidbody/ConcaveScene.cpp
    rigidbody/GpuCompoundScene.cpp
    rigidbody/GpuConvexScene.cpp
    rigidbody/GpuRigidBodyDemo.cpp
    rigidbody/GpuSphereScene.cpp
    shadows/ShadowMapDemo.cpp
    softbody/GpuSoftBodyDemo.cpp
    ../Wavefront/tiny_obj_loader.cpp
    ../../btgui/stb_image/stb_image.cpp
    ../../btgui/Timing/b3Quickprof.cpp
    ../../btgui/Timing/b3Clock.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)

IF (WIN32)
	SET(App_Bullet3_OpenCL_Demos_SRCS ${App_Bullet3_OpenCL_Demos_SRCS}  ${App_Bullet3_OpenCL_Demos_Common_SRCS})
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
			Bullet2FileLoader	Bullet3OpenCL_clew Bullet3Dynamics Bullet3Collision Bullet3Geometry Bullet3Common OpenGLWindow gwen ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(App_Bullet3_OpenCL_Demos
		${App_Bullet3_OpenCL_Demos_SRCS}
)
	

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)