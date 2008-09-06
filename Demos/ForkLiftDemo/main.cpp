
#include "ForkLiftDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{

        ForkLiftDemo* pForkLiftDemo = new ForkLiftDemo;

        pForkLiftDemo->initPhysics(); 

        return glutmain(argc, argv,640,480,"Bullet ForkLift Demo. http://www.continuousphysics.com/Bullet/phpBB2/", pForkLiftDemo);
}

