
#include "ConcaveDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{

        ConcaveDemo* concaveDemo = new ConcaveDemo();
        concaveDemo->initPhysics();
        concaveDemo->setCameraDistance(30.f);

        return glutmain(argc, argv,640,480,"Static Concave Mesh Demo",concaveDemo);
}

