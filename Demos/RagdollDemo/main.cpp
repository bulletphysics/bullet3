
#include "RagdollDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        RagdollDemo demoApp;

        demoApp.initPhysics();
		demoApp.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",&demoApp);
}
