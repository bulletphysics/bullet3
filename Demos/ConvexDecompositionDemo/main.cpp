

#include "ConvexDecompositionDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"



int main(int argc,char** argv)
{
        const char* filename = "file.obj";


        ConvexDecompositionDemo* convexDecompDemo = new ConvexDecompositionDemo();

        convexDecompDemo->initPhysics(filename);

		


        glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",convexDecompDemo);

	delete convexDecompDemo;

	return 0;
}
