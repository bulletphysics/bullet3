noinst_PROGRAMS=CcdPhysicsDemo

CcdPhysicsDemo_SOURCES=CcdPhysicsDemo.cpp CcdPhysicsDemo.h main.cpp
CcdPhysicsDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
CcdPhysicsDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
