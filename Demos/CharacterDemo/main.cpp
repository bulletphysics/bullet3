
#include "CharacterDemo.h"
#include "GlutStuff.h"


int main(int argc,char** argv)
{

        CharacterDemo* characterDemo = new CharacterDemo;

        characterDemo->initPhysics(); 

        return glutmain(argc, argv,640,480,"Bullet Character Demo. http://www.continuousphysics.com/Bullet/phpBB2/", characterDemo);
}

