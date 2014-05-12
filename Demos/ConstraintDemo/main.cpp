#include "ConstraintDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include "btBulletDynamicsCommon.h"

int main(int argc,char** argv)
{

	

    ConstraintDemo* constraintDemo = new ConstraintDemo();
	

    constraintDemo->initPhysics();
	constraintDemo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	
	return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}

