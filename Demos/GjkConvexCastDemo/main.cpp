
#include "LinearConvexCastDemo.h"
#include "GlutStuff.h"

int screenWidth = 640;
int screenHeight = 480;

int main(int argc,char** argv)
{

        LinearConvexCastDemo* linearCastDemo = new LinearConvexCastDemo();

        linearCastDemo->initPhysics();


        return glutmain(argc, argv,screenWidth,screenHeight,"Linear Convex Cast Demo",linearCastDemo);
}
