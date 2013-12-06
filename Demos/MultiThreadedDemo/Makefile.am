noinst_PROGRAMS=MultiThreadedDemo

MultiThreadedDemo_SOURCES=MultiThreadedDemo.cpp MultiThreadedDemo.h main.cpp
MultiThreadedDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL -I@top_builddir@/Extras  $(CXXFLAGS)
MultiThreadedDemo_LDADD=-L../OpenGL -L../../Extras  -L../../src  -lbulletmultithreaded -lbulletopenglsupport -lbulletdynamics -lbulletcollision -lbulletmath @opengl_LIBS@ -lpthread
