
HDRS  = fboSupport.h shaderSupport.h
OBJS  = GPU_physics_demo.o fboSupport.o shaderSupport.o

all: ${OBJS}
	g++ -o GPU_physics_demo ${OBJS} -lglut -lGLEW -lGL

shaderSupport.o : shaderSupport.cxx ${HDRS}
	g++ -c shaderSupport.cxx

fboSupport.o : fboSupport.cxx ${HDRS}
	g++ -c fboSupport.cxx

GPU_physics_demo.o : GPU_physics_demo.cxx ${HDRS}
	g++ -c GPU_physics_demo.cxx

clean:
	-rm -f ${OBJS}

