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
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    btBulletWorldImporter* importer = new btBulletWorldImporter(m_dynamicsWorld);
    const char* filename = "testFile.bullet";
    importer->loadFile(filename);
}