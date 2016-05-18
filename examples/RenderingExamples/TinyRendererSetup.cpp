
#include "RaytracerSetup.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../TinyRenderer/TinyRenderer.h"
#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../ExampleBrowser/CollisionShape2TriangleMesh.h"
#include "../Importers/ImportMeshUtility/b3ImportMeshUtility.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"

struct TinyRendererSetup : public CommonExampleInterface
{
	
	struct GUIHelperInterface* m_guiHelper;
	struct CommonGraphicsApp* m_app;
	struct TinyRendererSetupInternalData* m_internalData;

	TinyRendererSetup(struct GUIHelperInterface* guiHelper);

	virtual ~TinyRendererSetup();

	virtual void initPhysics();

	virtual void exitPhysics();

	virtual void stepSimulation(float deltaTime);


	virtual void	physicsDebugDraw(int debugFlags);

	virtual void syncPhysicsToGraphics(struct GraphicsPhysicsBridge& gfxBridge);

	virtual bool	mouseMoveCallback(float x,float y);

	virtual bool	mouseButtonCallback(int button, int state, float x, float y);

	virtual bool	keyboardCallback(int key, int state);

	virtual void	renderScene()
	{
	}
};

struct TinyRendererSetupInternalData
{
	
	TGAImage m_rgbColorBuffer;
	b3AlignedObjectArray<float> m_depthBuffer;


	int m_width;
	int m_height;

	btAlignedObjectArray<btConvexShape*> m_shapePtr;
	btAlignedObjectArray<btTransform> m_transforms;
	btAlignedObjectArray<TinyRenderObjectData*> m_renderObjects;

	btVoronoiSimplexSolver	m_simplexSolver;
	btScalar m_pitch;
	btScalar m_roll;
	btScalar m_yaw;
	
	int m_textureHandle;

	TinyRendererSetupInternalData(int width, int height)
		:m_roll(0),
		m_pitch(0),
		m_yaw(0),

		m_width(width),
		m_height(height),
		m_rgbColorBuffer(width,height,TGAImage::RGB),
		m_textureHandle(0)
	{
		m_depthBuffer.resize(m_width*m_height);

		#if 0
		btConeShape* cone = new btConeShape(1,1);
		btSphereShape* sphere = new btSphereShape(1);
		btBoxShape* box = new btBoxShape (btVector3(0.5,0.5,0.5));
		m_shapePtr.push_back(box);
		m_shapePtr.push_back(box);
		//m_shapePtr.push_back(sphere);
		//m_shapePtr.push_back(box);
		


		for (int i=0;i<m_shapePtr.size();i++)
		{
			TinyRenderObjectData* ob = new TinyRenderObjectData(m_width,m_height,m_rgbColorBuffer,m_depthBuffer);
			btAlignedObjectArray<btVector3> vertexPositions;
			btAlignedObjectArray<btVector3> vertexNormals;
			btAlignedObjectArray<int> indicesOut;
			btTransform ident;
			ident.setIdentity();
			CollisionShape2TriangleMesh(m_shapePtr[i],ident,vertexPositions,vertexNormals,indicesOut);

			
			//ob->createCube(0.5,0.5,0.5);//createCube
			ob->loadModel("cube.obj");
			m_renderObjects.push_back(ob);
			//ob->registerMesh2(vertexPositions,vertexNormals,indicesOut);
		}
		//ob->registerMeshShape(

		
		updateTransforms();
		#endif
	}
	void updateTransforms()
	{
		int numObjects = m_shapePtr.size();
		m_transforms.resize(numObjects);
		for (int i=0;i<numObjects;i++)
		{
			m_transforms[i].setIdentity();
			//btVector3	pos(0.f,-(2.5* numObjects * 0.5)+i*2.5f, 0.f);
			btVector3	pos(0.f,+i*2.5f, 0.f);
			m_transforms[i].setIdentity();
			m_transforms[i].setOrigin( pos );
			btQuaternion orn;
			if (i < 2)
			{
				orn.setEuler(m_yaw,m_pitch,m_roll);
				m_transforms[i].setRotation(orn);
			}
		}
		//m_pitch += 0.005f;
		//m_yaw += 0.01f;
	}

};

TinyRendererSetup::TinyRendererSetup(struct GUIHelperInterface* gui)
{
	m_guiHelper = gui;
	m_app = gui->getAppInterface();
	m_internalData = new TinyRendererSetupInternalData(gui->getAppInterface()->m_window->getWidth(),gui->getAppInterface()->m_window->getHeight());
	m_app->m_renderer->enableBlend(true);
	const char* fileName = "cube.obj";
	

	{
		
		{
			int shapeId = -1;

			b3ImportMeshData meshData;
			if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fileName, meshData))
			{
				int textureIndex = 0;

				if (meshData.m_textureImage)
				{
					textureIndex = m_guiHelper->getRenderInterface()->registerTexture(meshData.m_textureImage,meshData.m_textureWidth,meshData.m_textureHeight);
				}

				shapeId = m_guiHelper->getRenderInterface()->registerShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0], 
											meshData.m_gfxShape->m_numvertices, 
											&meshData.m_gfxShape->m_indices->at(0), 
											meshData.m_gfxShape->m_numIndices,
											B3_GL_TRIANGLES,
											textureIndex);

				float position[4]={0,0,0,1};
				float orn[4]={0,0,0,1};
				float color[4]={1,1,1,0.4};
				float scaling[4]={1,1,1,1};

				m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				m_guiHelper->getRenderInterface()->writeTransforms();

				m_internalData->m_shapePtr.push_back(0);
				TinyRenderObjectData* ob = new TinyRenderObjectData(m_internalData->m_width,m_internalData->m_height,
					m_internalData->m_rgbColorBuffer,
					m_internalData->m_depthBuffer);
					//ob->loadModel("cube.obj");
				const int* indices = &meshData.m_gfxShape->m_indices->at(0);
					ob->registerMeshShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0],
						meshData.m_gfxShape->m_numvertices,
						indices,
						meshData.m_gfxShape->m_numIndices, meshData.m_textureImage,meshData.m_textureWidth,meshData.m_textureHeight);
						
						
					m_internalData->m_renderObjects.push_back(ob);



				delete meshData.m_gfxShape;
				delete meshData.m_textureImage;
			}
		}
	}
		
		
}

TinyRendererSetup::~TinyRendererSetup()
{
	m_app->m_renderer->enableBlend(false);
	delete m_internalData;
}

void TinyRendererSetup::initPhysics()
{
	//request a visual bitma/texture we can render to
	
    m_app->setUpAxis(2);
    
	CommonRenderInterface* render = m_app->m_renderer;
	
	m_internalData->m_textureHandle = render->registerTexture(m_internalData->m_rgbColorBuffer.buffer(),m_internalData->m_width,m_internalData->m_height);
}


void TinyRendererSetup::exitPhysics()
{

}


void TinyRendererSetup::stepSimulation(float deltaTime)
{
	m_guiHelper->getRenderInterface()->renderScene();
	
	m_internalData->updateTransforms();
	
	TGAColor clearColor;
	clearColor.bgra[0] = 255;
	clearColor.bgra[1] = 255;
	clearColor.bgra[2] = 255;
	clearColor.bgra[3] = 255;
	for(int y=0;y<m_internalData->m_height;++y)
	{
		for(int x=0;x<m_internalData->m_width;++x)
		{
			m_internalData->m_rgbColorBuffer.set(x,y,clearColor);
			m_internalData->m_depthBuffer[x+y*m_internalData->m_width] = -1e30f;
		}
	}


	ATTRIBUTE_ALIGNED16(btScalar modelMat2[16]);
	ATTRIBUTE_ALIGNED16(float viewMat[16]);
	ATTRIBUTE_ALIGNED16(float projMat[16]);
	CommonRenderInterface* render = this->m_app->m_renderer;
	render->getActiveCamera()->getCameraViewMatrix(viewMat);
	render->getActiveCamera()->getCameraProjectionMatrix(projMat);
		

	
	for (int o=0;o<this->m_internalData->m_renderObjects.size();o++)
	{
			
		const btTransform& tr = m_internalData->m_transforms[o];
		tr.getOpenGLMatrix(modelMat2);
		
				
		for (int i=0;i<4;i++)
		{
			for (int j=0;j<4;j++)
			{
				m_internalData->m_renderObjects[o]->m_modelMatrix[i][j] = float(modelMat2[i+4*j]);
				m_internalData->m_renderObjects[o]->m_viewMatrix[i][j] = viewMat[i+4*j];
				m_internalData->m_renderObjects[o]->m_projectionMatrix[i][j] = projMat[i+4*j];
				
				float eye[4];
				float center[4];
				render->getActiveCamera()->getCameraPosition(eye);
				render->getActiveCamera()->getCameraTargetPosition(center);

				m_internalData->m_renderObjects[o]->m_eye.setValue(eye[0],eye[1],eye[2]);
				m_internalData->m_renderObjects[o]->m_center.setValue(center[0],center[1],center[2]);
			}
		}
		TinyRenderer::renderObject(*m_internalData->m_renderObjects[o]);
	}
	//m_app->drawText("hello",500,500);
	render->activateTexture(m_internalData->m_textureHandle);
	render->updateTexture(m_internalData->m_textureHandle,m_internalData->m_rgbColorBuffer.buffer());
	float color[4] = {1,1,1,0.5};
	m_app->drawTexturedRect(0,0,m_app->m_window->getWidth(), m_app->m_window->getHeight(),color,0,0,1,1,true);
	
}


void    TinyRendererSetup::physicsDebugDraw(int debugDrawFlags)
{
}

bool	TinyRendererSetup::mouseMoveCallback(float x,float y)
{
	return false;
}

bool	TinyRendererSetup::mouseButtonCallback(int button, int state, float x, float y)
{
	return false;
}

bool	TinyRendererSetup::keyboardCallback(int key, int state)
{
	return false;
}


void TinyRendererSetup::syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge)
{
}

 CommonExampleInterface*    TinyRendererCreateFunc(struct CommonExampleOptions& options)
 {
	 return new TinyRendererSetup(options.m_guiHelper);
 }
