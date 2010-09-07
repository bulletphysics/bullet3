noinst_PROGRAMS=BasicDemo

BasicDemo_SOURCES=BasicDemo.cpp BasicDemo.h main.cpp
BasicDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
BasicDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
