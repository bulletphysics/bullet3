#include "OpenGLGuiHelper.h"

#include "btBulletDynamicsCommon.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "Bullet3Common/b3Scalar.h"
#include "CollisionShape2TriangleMesh.h"



#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
//backwards compatibility
#include "GL_ShapeDrawer.h"


#define BT_LINE_BATCH_SIZE 512

struct MyDebugVec3
{
	MyDebugVec3(const btVector3& org)
	:x(org.x()),
	y(org.y()),
	z(org.z())
	{
	}

	float x;
	float y;
	float z;
};
class MyDebugDrawer : public btIDebugDraw
{
	CommonGraphicsApp* m_glApp;
	int m_debugMode;

    btAlignedObjectArray<MyDebugVec3> m_linePoints;
    btAlignedObjectArray<unsigned int> m_lineIndices;
    btVector3 m_currentLineColor;
	DefaultColors m_ourColors;

public:

	MyDebugDrawer(CommonGraphicsApp* app)
		: m_glApp(app)
		,m_debugMode(btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb),
		m_currentLineColor(-1,-1,-1)
	{
		
		
	}
	virtual DefaultColors	getDefaultColors() const	
	{	
		return m_ourColors;
	}
	///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
	virtual void setDefaultColors(const DefaultColors& colors) 
	{
		m_ourColors = colors;
	}


	virtual void	drawLine(const btVector3& from1,const btVector3& to1,const btVector3& color1)
	{
        //float from[4] = {from1[0],from1[1],from1[2],from1[3]};
        //float to[4] = {to1[0],to1[1],to1[2],to1[3]};
        //float color[4] = {color1[0],color1[1],color1[2],color1[3]};
		//m_glApp->m_instancingRenderer->drawLine(from,to,color);
		if (m_currentLineColor!=color1 || m_linePoints.size() >= BT_LINE_BATCH_SIZE)
        {
            flushLines();
            m_currentLineColor = color1;
        }
		MyDebugVec3 from(from1);
		MyDebugVec3 to(to1);
			
		m_linePoints.push_back(from);
		m_linePoints.push_back(to);

		m_lineIndices.push_back(m_lineIndices.size());
		m_lineIndices.push_back(m_lineIndices.size());

	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{
        drawLine(PointOnB,PointOnB+normalOnB,color);
	}
     

	virtual void	reportErrorWarning(const char* warningString)
	{
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}

	virtual void	setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}

	virtual int		getDebugMode() const
	{
		return m_debugMode;
	}

    virtual void flushLines()
	{
	    int sz = m_linePoints.size();
	    if (sz)
        {
			float debugColor[4];
		debugColor[0] = m_currentLineColor.x();
		debugColor[1] = m_currentLineColor.y();
		debugColor[2] = m_currentLineColor.z();
		debugColor[3] = 1.f;
		m_glApp->m_renderer->drawLines(&m_linePoints[0].x,debugColor,
														 m_linePoints.size(),sizeof(MyDebugVec3),
														 &m_lineIndices[0],
														 m_lineIndices.size(),
														 1);
            m_linePoints.clear();
            m_lineIndices.clear();
        }
	}

};

static btVector4 sColors[4] =
{
	btVector4(0.3,0.3,1,1),
	btVector4(0.6,0.6,1,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	//btVector4(1,1,0,1),
};


struct OpenGLGuiHelperInternalData
{
	struct CommonGraphicsApp* m_glApp;
	class MyDebugDrawer* m_debugDraw;
	GL_ShapeDrawer* m_gl2ShapeDrawer;
	
	btAlignedObjectArray<unsigned char> m_rgbaPixelBuffer;
	btAlignedObjectArray<float> m_depthBuffer;
};



OpenGLGuiHelper::OpenGLGuiHelper(CommonGraphicsApp* glApp, bool useOpenGL2)
{
	m_data = new OpenGLGuiHelperInternalData;
	m_data->m_glApp = glApp;
	m_data->m_debugDraw = 0;
	
	m_data->m_gl2ShapeDrawer = 0;

	if (useOpenGL2)
	{
		m_data->m_gl2ShapeDrawer = new GL_ShapeDrawer();

	}
}

OpenGLGuiHelper::~OpenGLGuiHelper()
{
	delete m_data->m_gl2ShapeDrawer;
	delete m_data;
}

struct CommonRenderInterface* OpenGLGuiHelper::getRenderInterface()
{
	return m_data->m_glApp->m_renderer;
}

void OpenGLGuiHelper::createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color)
{
	createCollisionObjectGraphicsObject(body,color);
}

void OpenGLGuiHelper::createCollisionObjectGraphicsObject(btCollisionObject* body, const btVector3& color)
{
	if (body->getUserIndex()<0)
	{
		btCollisionShape* shape = body->getCollisionShape();
		btTransform startTransform = body->getWorldTransform();
		int graphicsShapeId = shape->getUserIndex();
		if (graphicsShapeId>=0)
		{
		//	btAssert(graphicsShapeId >= 0);
			//the graphics shape is already scaled
			btVector3 localScaling(1,1,1);
			int graphicsInstanceId = m_data->m_glApp->m_renderer->registerGraphicsInstance(graphicsShapeId, startTransform.getOrigin(), startTransform.getRotation(), color, localScaling);
			body->setUserIndex(graphicsInstanceId);
		}
	}
}

int OpenGLGuiHelper::registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices)
{
	int shapeId = m_data->m_glApp->m_renderer->registerShape(vertices, numvertices,indices,numIndices);
	return shapeId;
}

int OpenGLGuiHelper::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	return m_data->m_glApp->m_renderer->registerGraphicsInstance(shapeIndex,position,quaternion,color,scaling);
}


void OpenGLGuiHelper::createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
{
	//already has a graphics object?
	if (collisionShape->getUserIndex()>=0)
		return;

	btAlignedObjectArray<GLInstanceVertex> gfxVertices;
	
	btAlignedObjectArray<int> indices;
	btTransform startTrans;startTrans.setIdentity();

    {
        btAlignedObjectArray<btVector3> vertexPositions;
        btAlignedObjectArray<btVector3> vertexNormals;
        CollisionShape2TriangleMesh(collisionShape,startTrans,vertexPositions,vertexNormals,indices);
        gfxVertices.resize(vertexPositions.size());
        for (int i=0;i<vertexPositions.size();i++)
        {
            for (int j=0;j<4;j++)
            {
                gfxVertices[i].xyzw[j] = vertexPositions[i][j];
            }
            for (int j=0;j<3;j++)
            {
                gfxVertices[i].normal[j] = vertexNormals[i][j];
            }
            for (int j=0;j<2;j++)
            {
                gfxVertices[i].uv[j] = 0.5;//we don't have UV info...
            }
        }
    }
    

	if (gfxVertices.size() && indices.size())
	{
		int shapeId = registerGraphicsShape(&gfxVertices[0].xyzw[0],gfxVertices.size(),&indices[0],indices.size());
		collisionShape->setUserIndex(shapeId);
	}
		
}
void OpenGLGuiHelper::syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
{
	int numCollisionObjects = rbWorld->getNumCollisionObjects();
	for (int i = 0; i<numCollisionObjects; i++)
	{
		btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
		btVector3 pos = colObj->getWorldTransform().getOrigin();
		btQuaternion orn = colObj->getWorldTransform().getRotation();
		int index = colObj->getUserIndex();
		if (index >= 0)
		{
			m_data->m_glApp->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, index);
		}
	}
	m_data->m_glApp->m_renderer->writeTransforms();
}



void OpenGLGuiHelper::render(const btDiscreteDynamicsWorld* rbWorld)
{

	m_data->m_glApp->m_renderer->renderScene();
	//backwards compatible OpenGL2 rendering

	if (m_data->m_gl2ShapeDrawer && rbWorld)
	{
		m_data->m_gl2ShapeDrawer->enableTexture(true);
		m_data->m_gl2ShapeDrawer->drawScene(rbWorld,true);
	}
}
void OpenGLGuiHelper::createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld)
{
	btAssert(rbWorld);
    m_data->m_debugDraw = new MyDebugDrawer(m_data->m_glApp);
    rbWorld->setDebugDrawer(m_data->m_debugDraw );


    m_data->m_debugDraw->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawAabb
        //btIDebugDraw::DBG_DrawContactPoints
        );

}

struct Common2dCanvasInterface*	OpenGLGuiHelper::get2dCanvasInterface()
{
	return m_data->m_glApp->m_2dCanvasInterface;
}

CommonParameterInterface* OpenGLGuiHelper::getParameterInterface()
{
	return m_data->m_glApp->m_parameterInterface;
}

void OpenGLGuiHelper::setUpAxis(int axis)
{
	m_data->m_glApp->setUpAxis(axis);

}

void OpenGLGuiHelper::resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ)
{
	if (getRenderInterface() && getRenderInterface()->getActiveCamera())
	{
		getRenderInterface()->getActiveCamera()->setCameraDistance(camDist);
		getRenderInterface()->getActiveCamera()->setCameraPitch(pitch);
		getRenderInterface()->getActiveCamera()->setCameraYaw(yaw);
		getRenderInterface()->getActiveCamera()->setCameraTargetPosition(camPosX,camPosY,camPosZ);
	}
}


void OpenGLGuiHelper::copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels, int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied)
{
    int w = m_data->m_glApp->m_window->getWidth()*m_data->m_glApp->m_window->getRetinaScale();
    int h = m_data->m_glApp->m_window->getHeight()*m_data->m_glApp->m_window->getRetinaScale();
    
    if (widthPtr)
        *widthPtr = w;
    if (heightPtr)
        *heightPtr = h;
	if (numPixelsCopied)
        *numPixelsCopied = 0;

    int numTotalPixels = w*h;
    int numRemainingPixels = numTotalPixels - startPixelIndex;
    int numBytesPerPixel = 4;//RGBA
    int numRequestedPixels  = btMin(rgbaBufferSizeInPixels,numRemainingPixels);
    if (numRequestedPixels)
    {
        if (startPixelIndex==0)
        {
			
			//quick test: render the scene
			getRenderInterface()->renderScene();
            //copy the image into our local cache
            m_data->m_rgbaPixelBuffer.resize(w*h*numBytesPerPixel);
            m_data->m_depthBuffer.resize(w*h);
            m_data->m_glApp->getScreenPixels(&(m_data->m_rgbaPixelBuffer[0]),m_data->m_rgbaPixelBuffer.size());
        }
        for (int i=0;i<numRequestedPixels*numBytesPerPixel;i++)
        {
            if (pixelsRGBA)
            {
                pixelsRGBA[i] = m_data->m_rgbaPixelBuffer[i+startPixelIndex*numBytesPerPixel];
            }
        }

		if (numPixelsCopied)
	        *numPixelsCopied = numRequestedPixels;


    }
    
   
}



struct MyConvertPointerSizeT
{
	union 
	{
			const void* m_ptr;
			size_t m_int;
	};
};
bool shapePointerCompareFunc(const btCollisionObject* colA, const btCollisionObject* colB)
{
	MyConvertPointerSizeT a,b;
	a.m_ptr = colA->getCollisionShape();
	b.m_ptr = colB->getCollisionShape();
	return (a.m_int<b.m_int);
}

void OpenGLGuiHelper::autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
{
	//sort the collision objects based on collision shape, the gfx library requires instances that re-use a shape to be added after eachother

	btAlignedObjectArray<btCollisionObject*> sortedObjects;
	sortedObjects.reserve(rbWorld->getNumCollisionObjects());
	for (int i=0;i<rbWorld->getNumCollisionObjects();i++)
	{
		btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
		sortedObjects.push_back(colObj);
	}
	sortedObjects.quickSort(shapePointerCompareFunc);
	for (int i=0;i<sortedObjects.size();i++)
	{
		btCollisionObject* colObj = sortedObjects[i];
		//btRigidBody* body = btRigidBody::upcast(colObj);
		//does this also work for btMultiBody/btMultiBodyLinkCollider?
		createCollisionShapeGraphicsObject(colObj->getCollisionShape());
		int colorIndex = colObj->getBroadphaseHandle()->getUid() & 3;

		btVector3 color= sColors[colorIndex];
		createCollisionObjectGraphicsObject(colObj,color);
			
	}
}
    
void OpenGLGuiHelper::drawText3D( const char* txt, float posX, float posY, float posZ, float size)
{
    btAssert(m_data->m_glApp);
    m_data->m_glApp->drawText3D(txt,posX,posY,posZ,size);
}

struct CommonGraphicsApp* OpenGLGuiHelper::getAppInterface()
{
	return m_data->m_glApp;
}

