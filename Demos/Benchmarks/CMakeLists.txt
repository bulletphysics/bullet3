# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################

IF (USE_GRAPHICAL_BENCHMARK)
IF (USE_GLUT)
	INCLUDE_DIRECTORIES(
		${BULLET_PHYSICS_SOURCE_DIR}/src 
		${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	)

	IF (USE_MULTITHREADED_BENCHMARK)
	INCLUDE_DIRECTORIES( ${VECTOR_MATH_INCLUDE} )
		LINK_LIBRARIES(	OpenGLSupport BulletMultiThreaded BulletDynamics  BulletCollision LinearMath ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
	ELSE()
		LINK_LIBRARIES(	OpenGLSupport BulletDynamics  BulletCollision LinearMath ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
	ENDIF(USE_MULTITHREADED_BENCHMARK)
	
	IF (WIN32)
		ADD_EXECUTABLE(AppBenchmarks
			main.cpp
			BenchmarkDemo.cpp 
			BenchmarkDemo.h
			${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
		)
	ELSE()
		ADD_EXECUTABLE(AppBenchmarks
			main.cpp
			BenchmarkDemo.cpp 
			BenchmarkDemo.h
		)
	ENDIF ()
	IF (WIN32)
		IF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
			IF (CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppBenchmarks
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/glut64.dll ${CMAKE_CURRENT_BINARY_DIR}
				)
			ELSE(CMAKE_CL_64)
				ADD_CUSTOM_COMMAND(
					TARGET AppBenchmarks
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${BULLET_PHYSICS_SOURCE_DIR}/GLUT32.DLL ${CMAKE_CURRENT_BINARY_DIR}
				)
			ENDIF(CMAKE_CL_64)
		ENDIF (NOT INTERNAL_CREATE_MSVC_RELATIVE_PATH_PROJECTFILES)
		IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(AppBenchmarks PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(AppBenchmarks PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(AppBenchmarks PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
		ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

	ENDIF(WIN32)
ELSE (USE_GLUT)
	INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	)

	IF (USE_MULTITHREADED_BENCHMARK)
		INCLUDE_DIRECTORIES( ${VECTOR_MATH_INCLUDE} )
		LINK_LIBRARIES(
		OpenGLSupport BulletMultiThreaded BulletDynamics  BulletCollision LinearMath ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
		)
	ELSE()
		LINK_LIBRARIES(
		OpenGLSupport BulletDynamics  BulletCollision LinearMath ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
		)
	
	ENDIF()
	
	ADD_EXECUTABLE(AppBenchmarks
		WIN32
		../OpenGL/Win32AppMain.cpp
		BenchmarkDemo.cpp 
		BenchmarkDemo.h
		Win32BenchmarkDemo.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/msvc/bullet.rc
	)
ENDIF (USE_GLUT)
ELSE (USE_GRAPHICAL_BENCHMARK)

	INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
	)

	IF (USE_MULTITHREADED_BENCHMARK)
		LINK_LIBRARIES(		BulletMultiThreaded BulletDynamics  BulletCollision LinearMath		)
	ELSE()
		LINK_LIBRARIES(		BulletDynamics  BulletCollision LinearMath		)
	ENDIF()
	
	
	ADD_EXECUTABLE(AppBenchmarks
		main.cpp
		BenchmarkDemo.cpp 
	)
ENDIF (USE_GRAPHICAL_BENCHMARK)