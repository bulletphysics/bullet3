
#include "MultiMaterialDemo.h"
#include "GlutStuff.h"

#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        MultiMaterialDemo* multiMaterialDemo = new MultiMaterialDemo();
        multiMaterialDemo->initPhysics();
        multiMaterialDemo->setCameraDistance(30.f);
        
        multiMaterialDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,640,480,"Multimaterial Mesh Demo",multiMaterialDemo);
}

