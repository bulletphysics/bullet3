noinst_PROGRAMS=FeatherstoneMultiBodyDemo

FeatherstoneMultiBodyDemo_SOURCES=FeatherstoneMultiBodyDemo.cpp FeatherstoneMultiBodyDemo.h main.cpp
FeatherstoneMultiBodyDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
FeatherstoneMultiBodyDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
