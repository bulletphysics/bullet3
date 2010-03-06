# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  



# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
OpenGLSupport BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

IF (WIN32)
	ADD_EXECUTABLE(AppConcaveRayCastDemo
			ConcaveRaycastDemo.cpp
			main.cpp
			${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppConcaveRayCastDemo
			ConcaveRaycastDemo.cpp
			main.cpp
	)
ENDIF()


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppConcaveRayCastDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppConcaveRayCastDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppConcaveRayCastDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)