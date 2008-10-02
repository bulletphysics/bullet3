bin_PROGRAMS=ColladaDemo

ColladaDemo_SOURCES=ColladaDemo.cpp ColladaDemo.h
ColladaDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL -I@top_builddir@/Extras -I@top_builddir@/Extras/BulletColladaConverter  $(CXXFLAGS)
ColladaDemo_LDADD=-L../OpenGL -L../../Extras  -L../../src  -lbulletcolladaconverter -lbulletopenglsupport -lbulletdynamics -lbulletcollision -lbulletmath -lcollada_dom -lxml @opengl_LIBS@
