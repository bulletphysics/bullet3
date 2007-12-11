
#include "Raytracer.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{
        Raytracer* raytraceDemo = new Raytracer();

        raytraceDemo->initPhysics();
        
        raytraceDemo->setCameraDistance(6.f);
		
        return glutmain(argc, argv,640,640,"Bullet GJK Implicit Shape Raytracer Demo",raytraceDemo);
}

