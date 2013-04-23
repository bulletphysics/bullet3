
#include "Bullet2FileDemo.h"
#include "BulletDataExtractor.h"
#include "GpuRigidBodyDemoInternalData.h"


Bullet2FileDemo::Bullet2FileDemo()
{
	m_loader = 0;
}
Bullet2FileDemo::~Bullet2FileDemo()
{
	delete 	m_loader;
}

void Bullet2FileDemo::setupScene(const ConstructionInfo& ci)
{
	btAssert(ci.m_instancingRenderer);

	const char* fileName="data/testFile.bullet";

	FILE* f = 0;

	const char* prefix[]={"./","../","../../","../../../","../../../../"};
	int numPrefixes = sizeof(prefix)/sizeof(const char*);
	char relativeFileName[1024];

	for (int i=0;!f && i<numPrefixes;i++)
	{
		sprintf(relativeFileName,"%s%s",prefix[i],fileName);
		f = fopen(relativeFileName,"rb");
	}
	if (f)
	{
		fclose(f);
		createScene(*ci.m_instancingRenderer,*m_data->m_np,*m_data->m_rigidBodyPipeline,relativeFileName);
	}
//	m_loader = new btBulletDataExtractor(*ci.m_instancingRenderer,*m_data->m_np,*m_data->m_rigidBodyPipeline);
//	m_loader->convertAllObjects(bulletFile);
}
