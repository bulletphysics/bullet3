
HDRS  = fboSupport.h shaderSupport.h GPU_physics.h clock.h
OBJS  = GPU_physics_demo.o fboSupport.o shaderSupport.o clock.o

all: ${OBJS}
	g++ -o GPU_physics_demo ${OBJS} -lglut -lGLEW -lGL

clock.o : clock.cpp ${HDRS}
	g++ -c clock.cpp

shaderSupport.o : shaderSupport.cpp ${HDRS}
	g++ -c shaderSupport.cpp

fboSupport.o : fboSupport.cpp ${HDRS}
	g++ -c fboSupport.cpp

GPU_physics_demo.o : GPU_physics_demo.cpp ${HDRS}
	g++ -c GPU_physics_demo.cpp

clean:
	-rm -f ${OBJS}

