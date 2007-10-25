#include "ConstraintDemo.h"
#include "GlutStuff.h"

int main(int argc,char** argv)
{

        ConstraintDemo* constraintDemo = new ConstraintDemo();

        constraintDemo->initPhysics();

        constraintDemo->setCameraDistance(26.f);

        return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}

