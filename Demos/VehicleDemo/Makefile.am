bin_PROGRAMS=VehicleDemo

VehicleDemo_SOURCES=VehicleDemo.cpp VehicleDemo.h main.cpp
VehicleDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
VehicleDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lbulletdynamics -lbulletcollision -lbulletmath @opengl_LIBS@
