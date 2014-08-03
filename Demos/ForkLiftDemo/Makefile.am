noinst_PROGRAMS=ForkLiftDemo

ForkLiftDemo_SOURCES=ForkLiftDemo.cpp ForkLiftDemo.h main.cpp
ForkLiftDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
ForkLiftDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
