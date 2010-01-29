
#include "CollisionInterfaceDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

int main(int argc,char** argv)
{
	CollisionInterfaceDemo* collisionInterfaceDemo = new CollisionInterfaceDemo();

	collisionInterfaceDemo->initPhysics();

	collisionInterfaceDemo->clientResetScene();

	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Interface Demo",collisionInterfaceDemo);
}
