
#include "ForkLiftDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        ForkLiftDemo* pForkLiftDemo = new ForkLiftDemo;

        pForkLiftDemo->initPhysics(); 
		pForkLiftDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Bullet ForkLift Demo. http://www.continuousphysics.com/Bullet/phpBB2/", pForkLiftDemo);
}

