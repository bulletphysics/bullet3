# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################

LINK_DIRECTORIES(
"C:/MinGW/lib"
)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/BulletColladaConverter ${BULLET_PHYSICS_SOURCE_DIR}/Extras  ${BULLET_PHYSICS_SOURCE_DIR}/Extras/GIMPACTUtils  ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexDecomposition  ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML ${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML/include ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include/1.4 ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include ${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL }
)

LINK_LIBRARIES(
BulletColladaConverter GLUI  GIMPACTUtils ConvexDecomposition ColladaDom XML OpenGLSupport  BulletSoftBody BulletDynamics BulletCollision LinearMath ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)



ADD_EXECUTABLE(AppAllBulletDemos
	Main.cpp
	DemoEntries.cpp
 	 ../CcdPhysicsDemo/CcdPhysicsDemo.cpp
   ../BasicDemo/BasicDemo.cpp
	../Benchmarks/BenchmarkDemo.cpp
	../BspDemo/BspDemo.cpp
  ../BspDemo/BspConverter.cpp
  ../BspDemo/BspLoader.cpp
  ../DynamicControlDemo/MotorDemo.cpp
  ../ConcaveDemo/ConcavePhysicsDemo.cpp
  ../ConcaveRaycastDemo/ConcaveRaycastDemo.cpp
  ../ConcaveConvexcastDemo/ConcaveConvexcastDemo.cpp
  ../ConvexDecompositionDemo/ConvexDecompositionDemo.cpp
  ../SliderConstraintDemo/SliderConstraintDemo.cpp
	../RagdollDemo/RagdollDemo.cpp
  ../GimpactTestDemo/GimpactTestDemo.cpp
  ../Raytracer/Raytracer.cpp
  ../GjkConvexCastDemo/LinearConvexCastDemo.cpp
  ../ForkLiftDemo/ForkLiftDemo.cpp
  ../SoftDemo/SoftDemo.cpp
  ../ConstraintDemo/ConstraintDemo.cpp
)

