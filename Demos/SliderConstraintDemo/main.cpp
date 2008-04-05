/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008
*/
#include "SliderConstraintDemo.h"
#include "GlutStuff.h"

int main(int argc,char** argv)
{

        SliderConstraintDemo* sliderConstraintDemo = new SliderConstraintDemo();

        sliderConstraintDemo->initPhysics();

       

        return glutmain(argc, argv,640,480,"Slider Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/", sliderConstraintDemo);
}

