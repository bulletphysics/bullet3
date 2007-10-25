
#include "RagdollDemo.h"
#include "GlutStuff.h"

int main(int argc,char* argv[])
{
        RagdollDemo demoApp;

        demoApp.initPhysics();
       

        return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",&demoApp);
}
