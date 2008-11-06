
#include "TerrainDemo.h"
#include "GlutStuff.h"

int main(int argc,char** argv)
{
	DemoApplication * demo = btCreateTerrainDemo();
	btAssert(demo && "failed to create terrain demo object");

        return glutmain(argc, argv, 800, 600,
	    "Terrain Demo. http://www.continuousphysics.com/Bullet/phpBB2/",
	    demo);
}

