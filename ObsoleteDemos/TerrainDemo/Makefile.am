
noinst_PROGRAMS=TerrainDemo

TerrainDemo_SOURCES=TerrainDemo.cpp TerrainDemo.h main.cpp
TerrainDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
TerrainDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
