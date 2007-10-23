

#include "ConvexDecompositionDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{
        const char* filename = "file.obj";


        ConvexDecompositionDemo* convexDecompDemo = new ConvexDecompositionDemo();

        convexDecompDemo->initPhysics(filename);



        convexDecompDemo->clientResetScene();

        convexDecompDemo->setCameraDistance(26.f);

        return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",convexDecompDemo);
}

