#include "ImportObjExample.h"
#include <vector>
#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "Wavefront/tiny_obj_loader.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "Wavefront2GLInstanceGraphicsShape.h"
#include "../../Utils/b3ResourcePath.h"
#include "../../Utils/b3BulletDefaultFileIO.h"
#include "Bullet3Common/b3FileUtils.h"

#include "stb_image/stb_image.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../ImportMeshUtility/b3ImportMeshUtility.h"

#include "../src/BulletCollision/Gimpact/btGImpactShape.h"

#include <memory>

class ImportObjSetup : public CommonRigidBodyBase
{
	std::string m_fileName;
	std::vector<GLInstanceGraphicsShape*> graphicsShapes;

public:
	ImportObjSetup(struct GUIHelperInterface* helper, const char* fileName);
	virtual ~ImportObjSetup();

	virtual void initPhysics();

	virtual void resetCamera()
	{
		float dist = 18;
		float pitch = -46;
		float yaw = 120;
		float targetPos[3] = {-2, -2, -2};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	//void renderScene() override;
};

ImportObjSetup::ImportObjSetup(struct GUIHelperInterface* helper, const char* fileName)
	: CommonRigidBodyBase(helper)
{
	if (fileName)
	{
		m_fileName = fileName;
	}
	else
	{
		m_fileName = "klima.obj";  //"sponza_closed.obj";//sphere8.obj";
	}
}

ImportObjSetup::~ImportObjSetup()
{
	for (auto shape : graphicsShapes)
	{
		if (shape)
			delete shape;
	}
}

int loadAndRegisterMeshFromFile2(const std::string& fileName, CommonRenderInterface* renderer, btGImpactMeshShape** dynamicShape, btBvhTriangleMeshShape** staticShape, ImportObjSetup& importObjSetup, btTransform trans, std::vector<GLInstanceGraphicsShape*>& graphicsShapes, int& userIndex)
{
	int shapeId = -1;

	b3ImportMeshData meshData;
	b3BulletDefaultFileIO fileIO;
	if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fileName, meshData,&fileIO))
	{
		int textureIndex = -1;

		if (meshData.m_textureImage1)
		{
			textureIndex = renderer->registerTexture(meshData.m_textureImage1, meshData.m_textureWidth, meshData.m_textureHeight);
		}

		shapeId = renderer->registerShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0],
										  meshData.m_gfxShape->m_numvertices,
										  &meshData.m_gfxShape->m_indices->at(0),
										  meshData.m_gfxShape->m_numIndices,
										  B3_GL_TRIANGLES,
										  textureIndex);

		////////////////////////
		btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
		btIndexedMesh part;

		part.m_vertexBase = (const unsigned char*)&meshData.m_gfxShape->m_vertices->at(0).xyzw[0];
		part.m_vertexStride = sizeof(float) * 4 + sizeof(float) * 3 + sizeof(float) * 2;
		part.m_numVertices = meshData.m_gfxShape->m_numvertices;
		part.m_triangleIndexBase = (const unsigned char*)&meshData.m_gfxShape->m_indices->at(0);
		part.m_triangleIndexStride = sizeof(int) * 3;
		part.m_numTriangles = meshData.m_gfxShape->m_numIndices / 3;
		part.m_indexType = PHY_INTEGER;
		part.m_vertexType = PHY_FLOAT;

		meshInterface->addIndexedMesh(part, PHY_INTEGER);

		bool useQuantizedAabbCompression = true;

		if (staticShape)
		{
			*staticShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);

			{
				btScalar mass(0.0);
				auto body = importObjSetup.createRigidBody(mass, trans, *staticShape, btVector4(1.0f, 1.0f, 1.0f, 1.0f));
				body->setUserIndex(userIndex++);
			}
		}
		if (dynamicShape)
		{
			*dynamicShape = new btGImpactMeshShape(meshInterface);
			{
				btScalar mass(1.0);
				btVector3 localInertia(0, 0, 0);
				(*dynamicShape)->calculateLocalInertia(mass, localInertia);
				(*dynamicShape)->updateBound();
				auto body = importObjSetup.createRigidBody(mass, trans, *dynamicShape, btVector4(1.0f, 1.0f, 1.0f, 1.0f));
				body->setUserIndex(userIndex++);
			}
		}
		
		////////////////////////

		graphicsShapes.push_back(meshData.m_gfxShape);
		if (!meshData.m_isCached)
		{
			delete meshData.m_textureImage1;
		}
	}
	return shapeId;
}

//void ImportObjSetup::renderScene()
//{
//	if (m_dynamicsWorld)
//	{
//		{
//			m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
//		}
//
//		{
//			m_guiHelper->render(m_dynamicsWorld);
//		}
//	}
//}

void ImportObjSetup::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	this->createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	//btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//btTransform groundTransform;
	//groundTransform.setIdentity();
	//groundTransform.setOrigin(btVector3(0, -50, 0));

	//{
		//btScalar mass(0.);
		//createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	//}

	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(-20, 0, 0));
	btVector3 position = trans.getOrigin();
	btQuaternion orn = trans.getRotation();

	btVector3 scaling(1, 1, 1);
	btVector4 color(1, 1, 1,1);

	int shapeId = -1;
	int userIndex = 0;

	btBvhTriangleMeshShape* staticMeshShape;
	shapeId = loadAndRegisterMeshFromFile2("SESTAVADILYCATIA.OBJ", m_guiHelper->getRenderInterface(), nullptr, &staticMeshShape, *this, trans, graphicsShapes, userIndex);
	if (shapeId >= 0)
	{
		m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, position, orn, color, scaling);
	}

	btGImpactMeshShape* dynamicMeshShape;
	shapeId = loadAndRegisterMeshFromFile2(m_fileName, m_guiHelper->getRenderInterface(), &dynamicMeshShape, nullptr, *this, trans, graphicsShapes, userIndex);
	if (shapeId >= 0)
	{
		m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, position, orn, color, scaling);
	}
	//m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

CommonExampleInterface* ImportObjCreateFunc(struct CommonExampleOptions& options)
{
	return new ImportObjSetup(options.m_guiHelper, options.m_fileName);
}
