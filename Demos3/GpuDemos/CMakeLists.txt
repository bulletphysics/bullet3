
ADD_DEFINITIONS(-DB3_USE_CLEW)

INCLUDE_DIRECTORIES(
	..
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
)
 

SET(App_Bullet3_OpenCL_Demos_SRCS
  	../../src/clew/clew.c
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

LINK_LIBRARIES(
			Bullet2FileLoader	Bullet3OpenCL_clew Bullet3Dynamics Bullet3Collision Bullet3Geometry Bullet3Common OpenGLWindow gwen ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)


IF (WIN32)
	SET(App_Bullet3_OpenCL_Demos_SRCS ${App_Bullet3_OpenCL_Demos_SRCS}  ${App_Bullet3_OpenCL_Demos_Common_SRCS} ../../btgui/OpenGLWindow/GlewWindows/glew.c)
	INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
	IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA})
        ELSE(APPLE)
		 SET(App_Bullet3_OpenCL_Demos_SRCS ${App_Bullet3_OpenCL_Demos_SRCS}  ${App_Bullet3_OpenCL_Demos_Common_SRCS} ../../btgui/OpenGLWindow/GlewWindows/glew.c)

		 INCLUDE_DIRECTORIES(
                                ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
       		 )
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

		LINK_LIBRARIES( X11 pthread dl)
	ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(App_Bullet3_OpenCL_Demos
		${App_Bullet3_OpenCL_Demos_SRCS}
)
	

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_Bullet3_OpenCL_Demos PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
