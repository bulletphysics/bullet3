
#include "ConcaveDemo.h"
#include "GlutStuff.h"

#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;
int main(int argc,char** argv)
{

        ConcaveDemo* concaveDemo = new ConcaveDemo();
        concaveDemo->initPhysics();
        concaveDemo->setCameraDistance(30.f);
		concaveDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Static Concave Mesh Demo",concaveDemo);
}

