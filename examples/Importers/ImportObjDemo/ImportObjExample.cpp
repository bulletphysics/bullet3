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

#include "stb_image/stb_image.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../ImportMeshUtility/b3ImportMeshUtility.h"

class ImportObjSetup : public CommonRigidBodyBase
{

    std::string m_fileName;
   
   
public:
    ImportObjSetup(struct GUIHelperInterface* helper, const char* fileName);
    virtual ~ImportObjSetup();
    
	virtual void initPhysics();

	virtual void resetCamera()
	{
		float dist = 18;
		float pitch = 120;
		float yaw = 46;
		float targetPos[3]={-2,-2,-2};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

};

ImportObjSetup::ImportObjSetup(struct GUIHelperInterface* helper, const char* fileName)
:CommonRigidBodyBase(helper)
{
    if (fileName)
    {
        m_fileName = fileName;
    } else
    {
        m_fileName = "cube.obj";//"sponza_closed.obj";//sphere8.obj";
    }
}

ImportObjSetup::~ImportObjSetup()
{
    
}



int loadAndRegisterMeshFromFile2(const std::string& fileName, CommonRenderInterface* renderer)
{
	int shapeId = -1;
	
	b3ImportMeshData meshData;
	if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fileName, meshData))
	{
		int textureIndex = -1;
		
		if (meshData.m_textureImage)
		{
			textureIndex = renderer->registerTexture(meshData.m_textureImage,meshData.m_textureWidth,meshData.m_textureHeight);
		}
		
		shapeId = renderer->registerShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0], 
										  meshData.m_gfxShape->m_numvertices, 
										  &meshData.m_gfxShape->m_indices->at(0), 
										  meshData.m_gfxShape->m_numIndices,
										  B3_GL_TRIANGLES,
										  textureIndex);
		delete meshData.m_gfxShape;
		delete meshData.m_textureImage;
	}
	return shapeId;
}



void ImportObjSetup::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	this->createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);


	btTransform trans;
    trans.setIdentity();
	trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
	btVector3 position = trans.getOrigin();
	btQuaternion orn = trans.getRotation();
		
    btVector3 scaling(1,1,1);
	btVector3 color(1,1,1);
		
   int shapeId = loadAndRegisterMeshFromFile2(m_fileName, m_guiHelper->getRenderInterface());    
   if (shapeId>=0)
   {
        //int id = 
        m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,position,orn,color,scaling);
   } 

}

 CommonExampleInterface*    ImportObjCreateFunc(struct CommonExampleOptions& options)
 {
	 return new ImportObjSetup(options.m_guiHelper, options.m_fileName);
 }
