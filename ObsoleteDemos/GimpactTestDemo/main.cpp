
#include "GimpactTestDemo.h"
#include "GlutStuff.h"

//################################## main #####################################
int main(int argc,char** argv)
{

        GimpactConcaveDemo* concaveDemo = new GimpactConcaveDemo();  /// This will not be Deleted!!!
        concaveDemo->initPhysics();
       
        return glutmain(argc, argv,640,480,"DevO,s GIMPACT Test Demo",concaveDemo);
}
