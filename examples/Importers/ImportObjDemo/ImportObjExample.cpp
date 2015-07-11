#include "ImportObjExample.h"
#include <vector>
#include "../OpenGLWindow/GLInstancingRenderer.h"
#include"Wavefront/tiny_obj_loader.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "Wavefront2GLInstanceGraphicsShape.h"
#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"


#include "../CommonInterfaces/CommonRigidBodyBase.h"


class ImportObjSetup : public CommonRigidBodyBase
{

public:
    ImportObjSetup(struct GUIHelperInterface* helper);
    virtual ~ImportObjSetup();
    
	virtual void initPhysics();

	virtual void resetCamera()
	{
		float dist = 50;
		float pitch = 61;
		float yaw = 18;
		float targetPos[3]={-15,-15,47};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

};

ImportObjSetup::ImportObjSetup(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper)
{
    
}

ImportObjSetup::~ImportObjSetup()
{
    
}







void ImportObjSetup::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	this->createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);


   const char* fileName = "samurai_monastry.obj";
        char relativeFileName[1024];
        if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024))
        {
                char pathPrefix[1024];

                b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);


	
	btVector3 shift(0,0,0);
	btVector3 scaling(10,10,10);
//	int index=10;
	
	{
		
		std::vector<tinyobj::shape_t> shapes;
		std::string err = tinyobj::LoadObj(shapes, relativeFileName, pathPrefix);
		
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
		
		btTransform trans;
		trans.setIdentity();
		trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
		
		btVector3 position = trans.getOrigin();
		btQuaternion orn = trans.getRotation();
		
		btVector3 color(0,0,1);
		
		
		int shapeId = m_guiHelper->getRenderInterface()->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices);
		
		//int id = 
		m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,position,orn,color,scaling);

	
	}}
        else
        {
                b3Warning("Cannot find %s\n", fileName);
        }

}

 CommonExampleInterface*    ImportObjCreateFunc(struct CommonExampleOptions& options)
 {
	 return new ImportObjSetup(options.m_guiHelper);
 }
