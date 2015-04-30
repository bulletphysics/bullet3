#include "SerializeSetup.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"


#include "../CommonInterfaces/CommonRigidBodyBase.h"

class SerializeSetup : public CommonRigidBodyBase
{
public:
	SerializeSetup(struct GUIHelperInterface* helper);
	virtual ~SerializeSetup();
	
	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);
};


SerializeSetup::SerializeSetup(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper)
{

}
SerializeSetup::~SerializeSetup()
{

}

void SerializeSetup::initPhysics()
{
    this->createEmptyDynamicsWorld();
   m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
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
		m_guiHelper->setUpAxis(2);
	} else
	{
		m_guiHelper->setUpAxis(1);
	}


	//example code to export the dynamics world to a .bullet file


	btDefaultSerializer*	serializer = new btDefaultSerializer();
	m_dynamicsWorld->serialize(serializer);

	FILE* file = fopen("testFile.bullet","wb");
	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
	fclose(file);

	


	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SerializeSetup::stepSimulation(float deltaTime)
{
    CommonRigidBodyBase::stepSimulation(deltaTime);
}

class CommonExampleInterface*    SerializeBulletCreateFunc(struct PhysicsInterface* pint, struct GUIHelperInterface* helper, int option)
{
	return new SerializeSetup(helper);
}
