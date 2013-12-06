
#include "ConcaveRaycastDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{

        ConcaveRaycastDemo* concaveRaycastDemo = new ConcaveRaycastDemo();
        concaveRaycastDemo->initPhysics();
        concaveRaycastDemo->setCameraDistance(30.f);

        return glutmain(argc, argv,640,480,"Concave Raycast Demo",concaveRaycastDemo);
}

