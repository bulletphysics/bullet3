noinst_PROGRAMS=AllBulletDemo

AllBulletDemo_SOURCES=\
		      	../SoftDemo/SoftDemo.cpp \
			../Benchmarks/BenchmarkDemo.cpp \
			../ForkLiftDemo/ForkLiftDemo.cpp \
			../RagdollDemo/RagdollDemo.cpp \
			../BasicDemo/BasicDemo.cpp \
			../CcdPhysicsDemo/CcdPhysicsDemo.cpp \
			../SliderConstraintDemo/SliderConstraintDemo.cpp \
			../ConcaveRaycastDemo/ConcaveRaycastDemo.cpp \
			../Raytracer/Raytracer.cpp \
			../BspDemo/BspDemo.cpp \
			../BspDemo/BspLoader.cpp \
			../BspDemo/BspConverter.cpp \
			../ConcaveConvexcastDemo/ConcaveConvexcastDemo.cpp \
			../ConvexDecompositionDemo/ConvexDecompositionDemo.cpp \
			../GimpactTestDemo/GimpactTestDemo.cpp \
			../GjkConvexCastDemo/LinearConvexCastDemo.cpp \
			../ConcaveDemo/ConcavePhysicsDemo.cpp \
			../DynamicControlDemo/MotorDemo.cpp \
			DemoEntries.cpp \
			DemoEntries.h\
			Main.cpp
AllBulletDemo_CXXFLAGS=\
		       	-I@top_builddir@/src \
		       	-I@top_builddir@/Extras/GIMPACTUtils \
		       	-I@top_builddir@/Extras/ConvexDecomposition \
		       	-I@top_builddir@/Extras \
			-I@top_builddir@/Demos/OpenGL \
			-I@top_builddir@/Demos/SoftDemo \
			-I@top_builddir@/Demos/Benchmarks \
			$(CXXFLAGS)
AllBulletDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -L../../Extras -lgimpactutils -lconvexdecomposition -lbulletsoftbody -lbulletdynamics -lbulletcollision -lbulletmath -lglui @opengl_LIBS@
