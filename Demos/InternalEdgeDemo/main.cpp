
#include "InternalEdgeDemo.h"
#include "GlutStuff.h"

#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"


int main(int argc,char** argv)
{

        InternalEdgeDemo* internalEdgeDemo = new InternalEdgeDemo();
        internalEdgeDemo->initPhysics();
        internalEdgeDemo->setCameraDistance(30.f);


        return glutmain(argc, argv,640,480,"Internal Edge Demo",internalEdgeDemo);
}

