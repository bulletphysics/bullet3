SUBDIRS(pendulum )

INCLUDE_DIRECTORIES(
		"${PROJECT_SOURCE_DIR}/src"
		"${PROJECT_SOURCE_DIR}/test/gtest-1.7.0/include")

ADD_DEFINITIONS(-DUSE_GTEST)
ADD_DEFINITIONS(-D_VARIADIC_MAX=10)

LINK_LIBRARIES(BulletDynamics BulletCollision LinearMath gtest)

IF (NOT WIN32)
	LINK_LIBRARIES(pthread)
ENDIF()

ADD_EXECUTABLE(Test_btKinematicCharacterController test_btKinematicCharacterController.cpp)

ADD_TEST(Test_btKinematicCharacterController_PASS Test_btKinematicCharacterController)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_btKinematicCharacterController PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

LINK_LIBRARIES(BulletDynamics BulletCollision LinearMath gtest BulletWorldImporter BulletFileLoader OpenGLWindow Bullet3Common)

IF (NOT WIN32)
	LINK_LIBRARIES(pthread)
ENDIF()

IF (DETERMINISM_VISUAL_DEBUG)
    ADD_DEFINITIONS(-DVISUAL_DEBUG)

    INCLUDE_DIRECTORIES(
            ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/glad
    )

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

                    LINK_LIBRARIES(  pthread ${DL} )
            ENDIF(APPLE)
    ENDIF(WIN32)

    SET(OPENGL_SRCS
        ../../examples/ExampleBrowser/OpenGLGuiHelper.cpp
        ../../examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp)
ENDIF()

ADD_EXECUTABLE(Test_deterministic_save_and_restore test_deterministic_save_and_restore.cpp
		../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
        ../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
        ${OPENGL_SRCS}
        )

ADD_TEST(Test_DeterministicSaveRestore_PASS Test_deterministic_save_and_restore)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(Test_deterministic_save_and_restore PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(Test_deterministic_save_and_restore PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(Test_deterministic_save_and_restore PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

