/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008
*/
#include "SliderConstraintDemo.h"
#include "GlutStuff.h"

#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        SliderConstraintDemo* sliderConstraintDemo = new SliderConstraintDemo();

        sliderConstraintDemo->initPhysics();
		sliderConstraintDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
		sliderConstraintDemo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
       

        return glutmain(argc, argv,640,480,"Slider Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/", sliderConstraintDemo);
}

