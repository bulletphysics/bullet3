# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################



INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL }
)

LINK_LIBRARIES(
  BulletDynamics  BulletCollision LinearMath
)

ADD_EXECUTABLE(AppBenchmarks
	main.cpp
	BenchmarkDemo.cpp 
)

