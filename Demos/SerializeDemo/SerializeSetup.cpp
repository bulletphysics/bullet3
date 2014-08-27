#include "SerializeSetup.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"

SerializeSetup::SerializeSetup()
{

}
SerializeSetup::~SerializeSetup()
{

}

void SerializeSetup::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
    this->createEmptyDynamicsWorld();
    gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);
    btBulletWorldImporter* importer = new btBulletWorldImporter(m_dynamicsWorld);
	const char* someFileName="spider.bullet";

    const char* prefix[]={"./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
        int numPrefixes = sizeof(prefix)/sizeof(const char*);
        char relativeFileName[1024];
        FILE* f=0;
        bool fileFound = false;
        int result = 0;

        for (int i=0;!f && i<numPrefixes;i++)
        {
                sprintf(relativeFileName,"%s%s",prefix[i],someFileName);
                f = fopen(relativeFileName,"rb");
                if (f)
                {
                    fileFound = true;
                    break;
                }
        }
        if (f)
        {
                fclose(f);
        }


    importer->loadFile(relativeFileName);

	//for now, guess the up axis from gravity
	if (m_dynamicsWorld->getGravity()[1] == 0.f)
	{
		gfxBridge.setUpAxis(2);
	} else
	{
		gfxBridge.setUpAxis(1);
	}


}

void SerializeSetup::stepSimulation(float deltaTime)
{
    CommonRigidBodySetup::stepSimulation(deltaTime);
}
