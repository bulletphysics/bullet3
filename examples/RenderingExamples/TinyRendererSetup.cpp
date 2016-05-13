
#include "RaytracerSetup.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../TinyRenderer/TinyRenderer.h"

#include "../CommonInterfaces/Common2dCanvasInterface.h"
//#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
//#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
//#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../ExampleBrowser/CollisionShape2TriangleMesh.h"

struct TinyRendererSetup : public CommonExampleInterface
{
	
	struct CommonGraphicsApp* m_app;
	struct TinyRendererSetupInternalData* m_internalData;

	TinyRendererSetup(struct CommonGraphicsApp* app);

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
	int m_canvasIndex;
	struct Common2dCanvasInterface* m_canvas;
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
	
	TinyRendererSetupInternalData(int width, int height)
		:m_canvasIndex(-1),
		m_canvas(0),
		m_roll(0),
		m_pitch(0),
		m_yaw(0),

		m_width(width),
		m_height(height),
		m_rgbColorBuffer(width,height,TGAImage::RGB)
	{
		btConeShape* cone = new btConeShape(1,1);
		btSphereShape* sphere = new btSphereShape(1);
		btBoxShape* box = new btBoxShape (btVector3(1,1,1));
		m_shapePtr.push_back(cone);
		m_shapePtr.push_back(sphere);
		m_shapePtr.push_back(box);
		m_depthBuffer.resize(m_width*m_height);

		for (int i=0;i<m_shapePtr.size();i++)
		{
			TinyRenderObjectData* ob = new TinyRenderObjectData(m_width,m_height,m_rgbColorBuffer,m_depthBuffer);
			btAlignedObjectArray<btVector3> vertexPositions;
			btAlignedObjectArray<btVector3> vertexNormals;
			btAlignedObjectArray<int> indicesOut;
			btTransform ident;
			ident.setIdentity();
			CollisionShape2TriangleMesh(m_shapePtr[i],ident,vertexPositions,vertexNormals,indicesOut);

			m_renderObjects.push_back(ob);
			ob->registerMesh2(vertexPositions,vertexNormals,indicesOut);
		}
		//ob->registerMeshShape(

		
		updateTransforms();
	}
	void updateTransforms()
	{
		int numObjects = m_shapePtr.size();
		m_transforms.resize(numObjects);
		for (int i=0;i<numObjects;i++)
		{
			m_transforms[i].setIdentity();
			btVector3	pos(0.f,0.f,-(2.5* numObjects * 0.5)+i*2.5f);
			m_transforms[i].setIdentity();
			m_transforms[i].setOrigin( pos );
			btQuaternion orn;
			if (i < 2)
			{
				orn.setEuler(m_yaw,m_pitch,m_roll);
				m_transforms[i].setRotation(orn);
			}
		}
		m_pitch += 0.005f;
		m_yaw += 0.01f;
	}

};

TinyRendererSetup::TinyRendererSetup(struct CommonGraphicsApp* app)
{
	m_app = app;
	m_internalData = new TinyRendererSetupInternalData(128,128);
}

TinyRendererSetup::~TinyRendererSetup()
{
	delete m_internalData;
}

void TinyRendererSetup::initPhysics()
{
	//request a visual bitma/texture we can render to
	
	

	m_internalData->m_canvas = m_app->m_2dCanvasInterface;
	

	if (m_internalData->m_canvas)
	{
		
		m_internalData->m_canvasIndex = m_internalData->m_canvas->createCanvas("tinyrenderer",m_internalData->m_width,m_internalData->m_height);
		for (int i=0;i<m_internalData->m_width;i++)
		{
			for (int j=0;j<m_internalData->m_height;j++)
			{
				unsigned char red=255;
				unsigned char green=255;
				unsigned char blue=255;
				unsigned char alpha=255;
				m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex,i,j,red,green,blue,alpha);
			}
		}
		m_internalData->m_canvas->refreshImageData(m_internalData->m_canvasIndex);

		//int bitmapId = gfxBridge.createRenderBitmap(width,height);
	}
	



}


void TinyRendererSetup::exitPhysics()
{
	
	if (m_internalData->m_canvas && m_internalData->m_canvasIndex>=0)
	{
		m_internalData->m_canvas->destroyCanvas(m_internalData->m_canvasIndex);
	}
}


void TinyRendererSetup::stepSimulation(float deltaTime)
{

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


	ATTRIBUTE_ALIGNED16(float modelMat[16]);
	ATTRIBUTE_ALIGNED16(float viewMat[16]);
	CommonRenderInterface* render = this->m_app->m_renderer;
	render->getActiveCamera()->getCameraViewMatrix(viewMat);
		

	
	for (int o=0;o<this->m_internalData->m_renderObjects.size();o++)
	{
		
		const btTransform& tr = m_internalData->m_transforms[o];
		tr.getOpenGLMatrix(modelMat);
				
		for (int i=0;i<4;i++)
		{
			for (int j=0;j<4;j++)
			{
				m_internalData->m_renderObjects[o]->m_modelMatrix[i][j] = modelMat[i+4*j];
				m_internalData->m_renderObjects[o]->m_viewMatrix[i][j] = viewMat[i+4*j];
			}
		}
		TinyRenderer::renderObject(*m_internalData->m_renderObjects[o]);
	}

	for(int y=0;y<m_internalData->m_height;++y)
	{
		for(int x=0;x<m_internalData->m_width;++x)
		{
				
			const TGAColor& color = m_internalData->m_rgbColorBuffer.get(x,y);
			m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex,x,(m_internalData->m_height-1-y),
				color.bgra[2],color.bgra[1],color.bgra[0],255);
		}
	} 

    //m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex,x,y,255,0,0,255);
		
	m_internalData->m_canvas->refreshImageData(m_internalData->m_canvasIndex);
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
	 return new TinyRendererSetup(options.m_guiHelper->getAppInterface());
 }
