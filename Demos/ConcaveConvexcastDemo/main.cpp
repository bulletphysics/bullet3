
#include "ConcaveConvexcastDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{

        ConcaveConvexcastDemo* concaveConvexcastDemo = new ConcaveConvexcastDemo();
        concaveConvexcastDemo->initPhysics();
        concaveConvexcastDemo->setCameraDistance(30.f);

        return glutmain(argc, argv,640,480,"Concave Convexcast Demo",concaveConvexcastDemo);
}

