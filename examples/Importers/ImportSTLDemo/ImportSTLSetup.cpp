#include "ImportSTLSetup.h"
#include <vector>
#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "LoadMeshFromSTL.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"




class ImportSTLSetup : public CommonRigidBodyBase
{
	
public:
    ImportSTLSetup(struct GUIHelperInterface* helper);
    virtual ~ImportSTLSetup();
    
	virtual void initPhysics();
	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -136;
		float yaw = 28;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

};


ImportSTLSetup::ImportSTLSetup(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper)
{
    
}

ImportSTLSetup::~ImportSTLSetup()
{
    
}



void ImportSTLSetup::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	this->createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);

	const char* fileName = "l_finger_tip.stl";
    char relativeFileName[1024];
	const char* prefix[]={"./data/","../data/","../../data/","../../../data/","../../../../data/"};
	int prefixIndex=-1;
	{
		
		int numPrefixes = sizeof(prefix)/sizeof(char*);
		
		for (int i=0;i<numPrefixes;i++)
		{
			FILE* f = 0;
			sprintf(relativeFileName,"%s%s",prefix[i],fileName);
			f = fopen(relativeFileName,"r");
			if (f)
			{
				fclose(f);
				prefixIndex = i;
				break;
			}
		}
	}
	
	if (prefixIndex<0)
		return;
	
	btVector3 shift(0,0,0);
	btVector3 scaling(10,10,10);
//	int index=10;
	
	{
		
		GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(relativeFileName);
		
		btTransform trans;
		trans.setIdentity();
		trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
		
		btVector3 position = trans.getOrigin();
		btQuaternion orn = trans.getRotation();
		
		btVector3 color(0,0,1);
		
		int shapeId = m_guiHelper->getRenderInterface()->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices);
		
		
	
		m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,position,orn,color,scaling);

	}
}

class CommonExampleInterface*    ImportSTLCreateFunc(struct CommonExampleOptions& options)
{
	return new ImportSTLSetup(options.m_guiHelper);
}
