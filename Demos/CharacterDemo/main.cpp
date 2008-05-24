
#include "CharacterDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        CharacterDemo* characterDemo = new CharacterDemo;

        characterDemo->initPhysics(); 
		characterDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Bullet Character Demo. http://www.continuousphysics.com/Bullet/phpBB2/", characterDemo);
}

