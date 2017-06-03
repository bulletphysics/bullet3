#include "OpenGLGuiHelper.h"

#include "btBulletDynamicsCommon.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "Bullet3Common/b3Scalar.h"
#include "CollisionShape2TriangleMesh.h"

#include "../OpenGLWindow/ShapeData.h"

#include "../OpenGLWindow/SimpleCamera.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"


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

ATTRIBUTE_ALIGNED16( class )MyDebugDrawer : public btIDebugDraw
{
	CommonGraphicsApp* m_glApp;
	int m_debugMode;

    btAlignedObjectArray<MyDebugVec3> m_linePoints;
    btAlignedObjectArray<unsigned int> m_lineIndices;


    btVector3 m_currentLineColor;
	DefaultColors m_ourColors;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MyDebugDrawer(CommonGraphicsApp* app)
		: m_glApp(app)
		,m_debugMode(btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb),
		m_currentLineColor(-1,-1,-1)
	{
		
		
	}

	virtual ~MyDebugDrawer()
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
        drawLine(PointOnB,PointOnB+normalOnB*distance,color);
		btVector3 ncolor(0, 0, 0);
		drawLine(PointOnB, PointOnB + normalOnB*0.01, ncolor);
		
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
	btVector4(60./256.,186./256.,84./256.,1),
	btVector4(244./256.,194./256.,13./256.,1),
	btVector4(219./256.,50./256.,54./256.,1),
	btVector4(72./256.,133./256.,237./256.,1),

	//btVector4(1,1,0,1),
};



struct MyHashShape
{
	
	int m_shapeKey;
	int m_shapeType;
	btVector3 m_sphere0Pos;
	btVector3 m_sphere1Pos;
	btScalar m_radius0;
	btScalar m_radius1;
	btTransform m_childTransform;
	int m_deformFunc;
	int m_upAxis;
	btScalar m_halfHeight;

	MyHashShape()
		:m_shapeKey(0),
		m_shapeType(0),
		m_sphere0Pos(btVector3(0,0,0)),
		m_sphere1Pos(btVector3(0,0,0)),
		m_radius0(0),
		m_radius1(0),
		m_deformFunc(0),
		m_upAxis(-1),
		m_halfHeight(0)
	{
		m_childTransform.setIdentity();
	}

	bool equals(const MyHashShape& other) const
	{
		bool sameShapeType = m_shapeType==other.m_shapeType;
		bool sameSphere0= m_sphere0Pos == other.m_sphere0Pos;
		bool sameSphere1= m_sphere1Pos == other.m_sphere1Pos;
		bool sameRadius0 = m_radius0== other.m_radius0;
		bool sameRadius1 = m_radius1== other.m_radius1;
		bool sameTransform = m_childTransform== other.m_childTransform;
		bool sameUpAxis = m_upAxis == other.m_upAxis;
		bool sameHalfHeight = m_halfHeight == other.m_halfHeight;
		return sameShapeType && sameSphere0 && sameSphere1 && sameRadius0 && sameRadius1 && sameTransform && sameUpAxis && sameHalfHeight;
	}
	//to our success
	SIMD_FORCE_INLINE	unsigned int getHash()const
	{
		unsigned int key = m_shapeKey;
		// Thomas Wang's hash
		key += ~(key << 15);	key ^=  (key >> 10);	key +=  (key << 3);	key ^=  (key >> 6);	key += ~(key << 11);	key ^=  (key >> 16);
		
		return key;
	}
};
	
struct OpenGLGuiHelperInternalData
{
	struct CommonGraphicsApp* m_glApp;
	class MyDebugDrawer* m_debugDraw;
	bool m_vrMode;
	int m_vrSkipShadowPass;

	btAlignedObjectArray<unsigned char> m_rgbaPixelBuffer1;
	btAlignedObjectArray<float> m_depthBuffer1;
	btHashMap<MyHashShape, int> m_hashShapes;


	VisualizerFlagCallback m_visualizerFlagCallback;

	int m_checkedTexture;
	int m_checkedTextureGrey;

	OpenGLGuiHelperInternalData()
		:m_vrMode(false),
		m_vrSkipShadowPass(0),
		m_visualizerFlagCallback(0),
		m_checkedTexture(-1),
		m_checkedTextureGrey(-1)
	{
	}

};

void OpenGLGuiHelper::setVRMode(bool vrMode)
{
	m_data->m_vrMode = vrMode;
	m_data->m_vrSkipShadowPass = 0;
}



OpenGLGuiHelper::OpenGLGuiHelper(CommonGraphicsApp* glApp, bool useOpenGL2)
{
	m_data = new OpenGLGuiHelperInternalData;
	m_data->m_glApp = glApp;
	m_data->m_debugDraw = 0;
	
	
}

OpenGLGuiHelper::~OpenGLGuiHelper()
{
	delete m_data->m_debugDraw;

	delete m_data;
}

struct CommonRenderInterface* OpenGLGuiHelper::getRenderInterface()
{
	return m_data->m_glApp->m_renderer;
}

const struct CommonRenderInterface* OpenGLGuiHelper::getRenderInterface() const
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

int	OpenGLGuiHelper::registerTexture(const unsigned char* texels, int width, int height)
{
	int textureId = m_data->m_glApp->m_renderer->registerTexture(texels,width,height);
	return textureId;
}


int OpenGLGuiHelper::registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId)
{
	int shapeId = m_data->m_glApp->m_renderer->registerShape(vertices, numvertices,indices,numIndices,primitiveType, textureId);
	return shapeId;
}

int OpenGLGuiHelper::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	return m_data->m_glApp->m_renderer->registerGraphicsInstance(shapeIndex,position,quaternion,color,scaling);
}

void OpenGLGuiHelper::removeAllGraphicsInstances()
{
	m_data->m_hashShapes.clear();
    m_data->m_glApp->m_renderer->removeAllInstances();
}

void OpenGLGuiHelper::removeGraphicsInstance(int graphicsUid)
{
	if (graphicsUid>=0)
	{
		m_data->m_glApp->m_renderer->removeGraphicsInstance(graphicsUid);
	};
}

void OpenGLGuiHelper::changeRGBAColor(int instanceUid, const double rgbaColor[4])
{
	if (instanceUid>=0)
	{
		m_data->m_glApp->m_renderer->writeSingleInstanceColorToCPU(rgbaColor,instanceUid);
	};
}
void OpenGLGuiHelper::changeSpecularColor(int instanceUid, const double specularColor[3])
{
	if (instanceUid>=0)
	{
		m_data->m_glApp->m_renderer->writeSingleInstanceSpecularColorToCPU(specularColor,instanceUid);
	};
}
int OpenGLGuiHelper::createCheckeredTexture(int red,int green, int blue)
{
	int texWidth=1024;
		int texHeight=1024;
		btAlignedObjectArray<unsigned char> texels;
		texels.resize(texWidth*texHeight*3);
		for (int i=0;i<texWidth*texHeight*3;i++)
			texels[i]=255;
		
		
		for (int i=0;i<texWidth;i++)
		{
			for (int j=0;j<texHeight;j++)
			{
				int a = i<texWidth/2? 1 : 0;
				int b = j<texWidth/2? 1 : 0;

				if (a==b)
				{
					texels[(i+j*texWidth)*3+0] = red;
					texels[(i+j*texWidth)*3+1] = green;
					texels[(i+j*texWidth)*3+2] = blue;
//					texels[(i+j*texWidth)*4+3] = 255;

				} 
				/*else
				{
					texels[i*3+0+j*texWidth] = 255;
					texels[i*3+1+j*texWidth] = 255;
					texels[i*3+2+j*texWidth] = 255;
				}
				*/
			}
		}
		
	
		int texId = registerTexture(&texels[0],texWidth,texHeight);
		return texId;
}

void OpenGLGuiHelper::createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
{
	//already has a graphics object?
	if (collisionShape->getUserIndex()>=0)
		return;

	if (m_data->m_checkedTexture<0)
	{
		m_data->m_checkedTexture = createCheckeredTexture(192,192,255);
	}

	if (m_data->m_checkedTextureGrey<0)
	{
		m_data->m_checkedTextureGrey = createCheckeredTexture(192,192,192);
	}
	

	btAlignedObjectArray<GLInstanceVertex> gfxVertices;
	btAlignedObjectArray<int> indices;
	int strideInBytes = 9*sizeof(float);
	//if (collisionShape->getShapeType()==BOX_SHAPE_PROXYTYPE)
	{
	}
	if (collisionShape->getShapeType()==MULTI_SPHERE_SHAPE_PROXYTYPE)
	{
		btMultiSphereShape* ms = (btMultiSphereShape*) collisionShape;
		if (ms->getSphereCount()==2)
		{
			btAlignedObjectArray<float> transformedVertices;
			int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
			transformedVertices.resize(numVertices*9);
			btVector3 sphere0Pos = ms->getSpherePosition(0);
			btVector3 sphere1Pos = ms->getSpherePosition(1);
			btVector3 fromTo = sphere1Pos-sphere0Pos;
			MyHashShape shape;
			shape.m_sphere0Pos = sphere0Pos;
			shape.m_sphere1Pos = sphere1Pos;
			shape.m_radius0 = 2.*ms->getSphereRadius(0);
			shape.m_radius1 = 2.*ms->getSphereRadius(1);
			shape.m_deformFunc = 1;//vert.dot(fromTo)
			int graphicsShapeIndex = -1;
			int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
			if (graphicsShapeIndexPtr)
			{
				//cache hit
				graphicsShapeIndex = *graphicsShapeIndexPtr;
			} else
			{
				//cache miss
				for (int i=0;i<numVertices;i++)
				{
					btVector3 vert;
					vert.setValue(textured_detailed_sphere_vertices[i*9+0],
						textured_detailed_sphere_vertices[i*9+1],
						textured_detailed_sphere_vertices[i*9+2]);

					btVector3 trVer(0,0,0);

					if (vert.dot(fromTo)>0)
					{
						btScalar radiusScale = 2.*ms->getSphereRadius(1);
						trVer = radiusScale*vert;
						trVer+=sphere1Pos;
					} else
					{
						btScalar radiusScale = 2.*ms->getSphereRadius(0);
						trVer = radiusScale*vert;
						trVer+=sphere0Pos;
					}

					transformedVertices[i*9+0] = trVer[0];
					transformedVertices[i*9+1] = trVer[1];
					transformedVertices[i*9+2] = trVer[2];
					transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
					transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
					transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
					transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
					transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
					transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
				}
				
				int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
				graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
			
				m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
			}
			collisionShape->setUserIndex(graphicsShapeIndex);
			return;
		}
	}

	if (collisionShape->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
	{
			btSphereShape* sphereShape = (btSphereShape*) collisionShape;
			btScalar radius = sphereShape->getRadius();
			btScalar sphereSize = 2.*radius;
			btVector3 radiusScale(sphereSize,sphereSize,sphereSize);
			btAlignedObjectArray<float> transformedVertices;


			MyHashShape shape;
			shape.m_radius0 = sphereSize;
			shape.m_deformFunc = 0;////no deform
			int graphicsShapeIndex = -1;
			int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
			if (graphicsShapeIndexPtr)
			{
				graphicsShapeIndex = *graphicsShapeIndexPtr;
			} else
			{
				int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
				transformedVertices.resize(numVertices*9);
				for (int i=0;i<numVertices;i++)
				{

					btVector3 vert;
					vert.setValue(textured_detailed_sphere_vertices[i*9+0],
						textured_detailed_sphere_vertices[i*9+1],
						textured_detailed_sphere_vertices[i*9+2]);

					btVector3 trVer = radiusScale*vert;
					transformedVertices[i*9+0] = trVer[0];
					transformedVertices[i*9+1] = trVer[1];
					transformedVertices[i*9+2] = trVer[2];
					transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
					transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
					transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
					transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
					transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
					transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
				}
				
				int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
				graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
				m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
			}

			collisionShape->setUserIndex(graphicsShapeIndex);
			return;
	}
	if (collisionShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
	{
		btCompoundShape* compound = (btCompoundShape*)collisionShape;
		if (compound->getNumChildShapes()==1)
		{
			if (compound->getChildShape(0)->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
			{
				btSphereShape* sphereShape = (btSphereShape*) compound->getChildShape(0);
				btScalar radius = sphereShape->getRadius();
				btScalar sphereSize = 2.*radius;
				btVector3 radiusScale(sphereSize,sphereSize,sphereSize);

				MyHashShape shape;
				shape.m_radius0 = sphereSize;
				shape.m_deformFunc = 0;//no deform
				shape.m_childTransform =  compound->getChildTransform(0);

				int graphicsShapeIndex = -1;
				int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
				if (graphicsShapeIndexPtr)
				{
					graphicsShapeIndex = *graphicsShapeIndexPtr;
				} 
				else
				{

					btAlignedObjectArray<float> transformedVertices;
					int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
					transformedVertices.resize(numVertices*9);
					for (int i=0;i<numVertices;i++)
					{

						btVector3 vert;
						vert.setValue(textured_detailed_sphere_vertices[i*9+0],
						 textured_detailed_sphere_vertices[i*9+1],
						 textured_detailed_sphere_vertices[i*9+2]);

						btVector3 trVer = compound->getChildTransform(0)*(radiusScale*vert);
						transformedVertices[i*9+0] = trVer[0];
						transformedVertices[i*9+1] = trVer[1];
						transformedVertices[i*9+2] = trVer[2];
						transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
						transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
						transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
						transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
						transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
						transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
					}
				
					int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
					graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
					m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
				}

				collisionShape->setUserIndex(graphicsShapeIndex);
				return;
			}
			if (compound->getChildShape(0)->getShapeType()==CAPSULE_SHAPE_PROXYTYPE)
			{
				btCapsuleShape* sphereShape = (btCapsuleShape*)  compound->getChildShape(0);
				int up = sphereShape->getUpAxis();
				btScalar halfHeight = sphereShape->getHalfHeight();

				btScalar radius = sphereShape->getRadius();
				btScalar sphereSize = 2.*radius;
				
				btVector3 radiusScale = btVector3(sphereSize,sphereSize,sphereSize);


				MyHashShape shape;
				shape.m_radius0 = sphereSize;
				shape.m_deformFunc = 2;//no deform
				shape.m_childTransform =  compound->getChildTransform(0);
				shape.m_upAxis = up;

				int graphicsShapeIndex = -1;
				int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
				if (graphicsShapeIndexPtr)
				{
					graphicsShapeIndex = *graphicsShapeIndexPtr;
				} 
				else
				{

					btAlignedObjectArray<float> transformedVertices;
					int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
					transformedVertices.resize(numVertices*9);
					for (int i=0;i<numVertices;i++)
					{

						btVector3 vert;
						vert.setValue(textured_detailed_sphere_vertices[i*9+0],
							textured_detailed_sphere_vertices[i*9+1],
							textured_detailed_sphere_vertices[i*9+2]);
				
						btVector3 trVer = compound->getChildTransform(0)*(radiusScale*vert);
						if (trVer[up]>0)
							trVer[up]+=halfHeight;
						else
							trVer[up]-=halfHeight;


						transformedVertices[i*9+0] = trVer[0];
						transformedVertices[i*9+1] = trVer[1];
						transformedVertices[i*9+2] = trVer[2];
						transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
						transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
						transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
						transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
						transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
						transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
					}
				
					int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
					graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
					m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
				}

				collisionShape->setUserIndex(graphicsShapeIndex);
				return;

			}

			if (compound->getChildShape(0)->getShapeType()==MULTI_SPHERE_SHAPE_PROXYTYPE)
			{
				btMultiSphereShape* ms = (btMultiSphereShape*) compound->getChildShape(0);
				if (ms->getSphereCount()==2)
				{
					btAlignedObjectArray<float> transformedVertices;
					int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
					transformedVertices.resize(numVertices*9);
					btVector3 sphere0Pos = ms->getSpherePosition(0);
					btVector3 sphere1Pos = ms->getSpherePosition(1);
					btVector3 fromTo = sphere1Pos-sphere0Pos;
					btScalar radiusScale1 = 2.0*ms->getSphereRadius(1);
					btScalar radiusScale0 = 2.0*ms->getSphereRadius(0);

					MyHashShape shape;
					shape.m_radius0 = radiusScale0;
					shape.m_radius1 = radiusScale1;
					shape.m_deformFunc = 4;
					shape.m_sphere0Pos = sphere0Pos;
					shape.m_sphere1Pos = sphere1Pos;
					shape.m_childTransform = compound->getChildTransform(0);

					int graphicsShapeIndex = -1;
					int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
					if (graphicsShapeIndexPtr)
					{
						graphicsShapeIndex = *graphicsShapeIndexPtr;
					} 
					else
					{
						for (int i=0;i<numVertices;i++)
						{

							btVector3 vert;
							vert.setValue(textured_detailed_sphere_vertices[i*9+0],
								textured_detailed_sphere_vertices[i*9+1],
								textured_detailed_sphere_vertices[i*9+2]);

							btVector3 trVer(0,0,0);
							if (vert.dot(fromTo)>0)
							{

								trVer = vert*radiusScale1;
								trVer+=sphere1Pos;
								trVer = compound->getChildTransform(0)*trVer;
							} else
							{
								trVer = vert*radiusScale0;
								trVer+=sphere0Pos;
								trVer=compound->getChildTransform(0)*trVer;
							}



							transformedVertices[i*9+0] = trVer[0];
							transformedVertices[i*9+1] = trVer[1];
							transformedVertices[i*9+2] = trVer[2];
							transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
							transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
							transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
							transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
							transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
							transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
						}
				
						int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
						graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
						m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
					}
					collisionShape->setUserIndex(graphicsShapeIndex);
					return;
				}
			}
		}
	}
	if (collisionShape->getShapeType()==CAPSULE_SHAPE_PROXYTYPE)
	{
		btCapsuleShape* sphereShape = (btCapsuleShape*) collisionShape;//Y up
		int up = sphereShape->getUpAxis();
		btScalar halfHeight = sphereShape->getHalfHeight();

		btScalar radius = sphereShape->getRadius();
		btScalar sphereSize = 2.*radius;
		btVector3 radiusScale(sphereSize,sphereSize,sphereSize);
	
			
		MyHashShape shape;
		shape.m_radius0 = sphereSize;
		shape.m_deformFunc = 3;
		shape.m_upAxis = up;
		shape.m_halfHeight = halfHeight;
		int graphicsShapeIndex = -1;
		int* graphicsShapeIndexPtr = m_data->m_hashShapes[shape];
			
		if (graphicsShapeIndexPtr)
		{
			graphicsShapeIndex = *graphicsShapeIndexPtr;
		} 
		else
		{
			
			btAlignedObjectArray<float> transformedVertices;
			int numVertices = sizeof(textured_detailed_sphere_vertices)/strideInBytes;
			transformedVertices.resize(numVertices*9);
			for (int i=0;i<numVertices;i++)
			{

				btVector3 vert;
				vert.setValue(textured_detailed_sphere_vertices[i*9+0],
					textured_detailed_sphere_vertices[i*9+1],
					textured_detailed_sphere_vertices[i*9+2]);
				
				btVector3 trVer = radiusScale*vert;
				if (trVer[up]>0)
					trVer[up]+=halfHeight;
				else
					trVer[up]-=halfHeight;



				transformedVertices[i*9+0] = trVer[0];
				transformedVertices[i*9+1] = trVer[1];
				transformedVertices[i*9+2] = trVer[2];
				transformedVertices[i*9+3] =textured_detailed_sphere_vertices[i*9+3];
				transformedVertices[i*9+4] =textured_detailed_sphere_vertices[i*9+4];
				transformedVertices[i*9+5] =textured_detailed_sphere_vertices[i*9+5];
				transformedVertices[i*9+6] =textured_detailed_sphere_vertices[i*9+6];
				transformedVertices[i*9+7] =textured_detailed_sphere_vertices[i*9+7];
				transformedVertices[i*9+8] =textured_detailed_sphere_vertices[i*9+8];
			}
				
			int numIndices = sizeof(textured_detailed_sphere_indices)/sizeof(int);
			graphicsShapeIndex = registerGraphicsShape(&transformedVertices[0],numVertices,textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,m_data->m_checkedTextureGrey);
			m_data->m_hashShapes.insert(shape,graphicsShapeIndex);
		}
		collisionShape->setUserIndex(graphicsShapeIndex);
		return;

	}
	if (collisionShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
	{
		const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(collisionShape);
		btScalar planeConst = staticPlaneShape->getPlaneConstant();
		const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
		btVector3 planeOrigin = planeNormal * planeConst;
		btVector3 vec0,vec1;
		btPlaneSpace1(planeNormal,vec0,vec1);

		btScalar vecLen = 128;
		btVector3 verts[4];

		verts[0] = planeOrigin + vec0*vecLen + vec1*vecLen;
		verts[1] = planeOrigin - vec0*vecLen + vec1*vecLen;
		verts[2] = planeOrigin - vec0*vecLen - vec1*vecLen;
		verts[3] = planeOrigin + vec0*vecLen - vec1*vecLen;
		
		int startIndex = 0;
		indices.push_back(startIndex+0);
		indices.push_back(startIndex+1);
		indices.push_back(startIndex+2);
		indices.push_back(startIndex+0);
		indices.push_back(startIndex+2);
		indices.push_back(startIndex+3);
		btTransform parentTransform;
		parentTransform.setIdentity();
		btVector3 triNormal = parentTransform.getBasis()*planeNormal;
				
		gfxVertices.resize(4);

		for (int i=0;i<4;i++)
		{
			btVector3 vtxPos;
			btVector3 pos =parentTransform*verts[i];

			gfxVertices[i].xyzw[0] = pos[0];
			gfxVertices[i].xyzw[1] = pos[1];
			gfxVertices[i].xyzw[2] = pos[2];
			gfxVertices[i].xyzw[3] = 1;
			gfxVertices[i].normal[0] = triNormal[0];
			gfxVertices[i].normal[1] = triNormal[1];
			gfxVertices[i].normal[2] = triNormal[2];
		}

		//verts[0] = planeOrigin + vec0*vecLen + vec1*vecLen;
		//verts[1] = planeOrigin - vec0*vecLen + vec1*vecLen;
		//verts[2] = planeOrigin - vec0*vecLen - vec1*vecLen;
		//verts[3] = planeOrigin + vec0*vecLen - vec1*vecLen;

		gfxVertices[0].uv[0] = vecLen/2;
		gfxVertices[0].uv[1] = vecLen/2;
		gfxVertices[1].uv[0] = -vecLen/2;
		gfxVertices[1].uv[1] = vecLen/2;
		gfxVertices[2].uv[0] = -vecLen/2;
		gfxVertices[2].uv[1] = -vecLen/2;
		gfxVertices[3].uv[0] = vecLen/2;
		gfxVertices[3].uv[1] = -vecLen/2;

		int shapeId = registerGraphicsShape(&gfxVertices[0].xyzw[0],gfxVertices.size(),&indices[0],indices.size(),B3_GL_TRIANGLES,m_data->m_checkedTexture);
		collisionShape->setUserIndex(shapeId);
		return;
	}

	btTransform startTrans;startTrans.setIdentity();
	//todo: create some textured objects for popular objects, like plane, cube, sphere, capsule

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
		int shapeId = registerGraphicsShape(&gfxVertices[0].xyzw[0],gfxVertices.size(),&indices[0],indices.size(),B3_GL_TRIANGLES,-1);
		collisionShape->setUserIndex(shapeId);
	}
		
}
void OpenGLGuiHelper::syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
{
	//in VR mode, we skip the synchronization for the second eye
	if (m_data->m_vrMode && m_data->m_vrSkipShadowPass==1)
		return;

	int numCollisionObjects = rbWorld->getNumCollisionObjects();
	{
		B3_PROFILE("write all InstanceTransformToCPU");
		for (int i = 0; i<numCollisionObjects; i++)
		{
			B3_PROFILE("writeSingleInstanceTransformToCPU");
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			btVector3 pos = colObj->getWorldTransform().getOrigin();
			btQuaternion orn = colObj->getWorldTransform().getRotation();
			int index = colObj->getUserIndex();
			if (index >= 0)
			{
				m_data->m_glApp->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, index);
			}
		}
	}
	{
		B3_PROFILE("writeTransforms");
		m_data->m_glApp->m_renderer->writeTransforms();
	}
}



void OpenGLGuiHelper::render(const btDiscreteDynamicsWorld* rbWorld)
{
	if (m_data->m_vrMode)
	{
		//in VR, we skip the shadow generation for the second eye
		
		if (m_data->m_vrSkipShadowPass>=1)
		{
			m_data->m_glApp->m_renderer->renderSceneInternal(B3_USE_SHADOWMAP_RENDERMODE);
			m_data->m_vrSkipShadowPass=0;
			
		} else
		{
			m_data->m_glApp->m_renderer->renderScene();	
			m_data->m_vrSkipShadowPass++;
		}
	} else
	{
		m_data->m_glApp->m_renderer->renderScene();	
	}
	
	
}
void OpenGLGuiHelper::createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld)
{
	btAssert(rbWorld);
	if (m_data->m_debugDraw)
	{
		delete m_data->m_debugDraw;
		m_data->m_debugDraw = 0;
	}

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


void	OpenGLGuiHelper::setVisualizerFlagCallback(VisualizerFlagCallback callback)
{
	m_data->m_visualizerFlagCallback = callback;
}


void OpenGLGuiHelper::setVisualizerFlag(int flag, int enable)
{
	if (m_data->m_visualizerFlagCallback)
		(m_data->m_visualizerFlagCallback)(flag,enable);
}


void OpenGLGuiHelper::resetCamera(float camDist, float yaw, float pitch, float camPosX,float camPosY, float camPosZ)
{
	if (getRenderInterface() && getRenderInterface()->getActiveCamera())
	{
		getRenderInterface()->getActiveCamera()->setCameraDistance(camDist);
		getRenderInterface()->getActiveCamera()->setCameraPitch(pitch);
		getRenderInterface()->getActiveCamera()->setCameraYaw(yaw);
		getRenderInterface()->getActiveCamera()->setCameraTargetPosition(camPosX,camPosY,camPosZ);
	}
}

bool OpenGLGuiHelper::getCameraInfo(int* width, int* height, float viewMatrix[16], float projectionMatrix[16], float camUp[3], float camForward[3],float hor[3], float vert[3], float* yaw, float* pitch, float* camDist, float cameraTarget[3]) const
{
	if (getRenderInterface() && getRenderInterface()->getActiveCamera())
	{
		*width = m_data->m_glApp->m_window->getWidth()*m_data->m_glApp->m_window->getRetinaScale();
		*height = m_data->m_glApp->m_window->getHeight()*m_data->m_glApp->m_window->getRetinaScale();
		getRenderInterface()->getActiveCamera()->getCameraViewMatrix(viewMatrix);
		getRenderInterface()->getActiveCamera()->getCameraProjectionMatrix(projectionMatrix);
		getRenderInterface()->getActiveCamera()->getCameraUpVector(camUp);
		getRenderInterface()->getActiveCamera()->getCameraForwardVector(camForward);
		float frustumNearPlane =     getRenderInterface()->getActiveCamera()->getCameraFrustumNear();
		float frustumFarPlane =     getRenderInterface()->getActiveCamera()->getCameraFrustumFar();

		float top = 1.f;
		float bottom = -1.f;
		float tanFov = (top-bottom)*0.5f / frustumNearPlane;
		float fov = btScalar(2.0) * btAtan(tanFov);
		btVector3 camPos,camTarget;
		getRenderInterface()->getActiveCamera()->getCameraPosition(camPos);
		getRenderInterface()->getActiveCamera()->getCameraTargetPosition(camTarget);
		btVector3	rayFrom = camPos;
		btVector3 rayForward = (camTarget-camPos);
		rayForward.normalize();
		float farPlane = 10000.f;
		rayForward*= farPlane;

		btVector3 rightOffset;
		btVector3 cameraUp=btVector3(camUp[0],camUp[1],camUp[2]);
		btVector3 vertical = cameraUp;
		btVector3 hori;
		hori = rayForward.cross(vertical);
		hori.normalize();
		vertical = hori.cross(rayForward);
		vertical.normalize();
		float tanfov = tanf(0.5f*fov);
		hori *= 2.f * farPlane * tanfov;
		vertical *= 2.f * farPlane * tanfov;
		btScalar aspect =  *width / *height;
		hori*=aspect;
		//compute 'hor' and 'vert' vectors, useful to generate raytracer rays
		hor[0] = hori[0];
		hor[1] = hori[1];
		hor[2] = hori[2];
		vert[0] = vertical[0];
		vert[1] = vertical[1];
		vert[2] = vertical[2];
		
		*yaw = getRenderInterface()->getActiveCamera()->getCameraYaw();
		*pitch = getRenderInterface()->getActiveCamera()->getCameraPitch();
		*camDist = getRenderInterface()->getActiveCamera()->getCameraDistance();
		cameraTarget[0] = camTarget[0];
		cameraTarget[1] = camTarget[1];
		cameraTarget[2] = camTarget[2];
		return true;
	}
	return false;
}


void OpenGLGuiHelper::copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], 
                                          unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                          float* depthBuffer, int depthBufferSizeInPixels, 
                                          int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
                                          int startPixelIndex, int destinationWidth, 
                                          int destinationHeight, int* numPixelsCopied)
{
    int sourceWidth = m_data->m_glApp->m_window->getWidth()*m_data->m_glApp->m_window->getRetinaScale();
    int sourceHeight  = m_data->m_glApp->m_window->getHeight()*m_data->m_glApp->m_window->getRetinaScale();
    
	if (numPixelsCopied)
        *numPixelsCopied = 0;

    int numTotalPixels = destinationWidth*destinationHeight;
    int numRemainingPixels = numTotalPixels - startPixelIndex;
    int numBytesPerPixel = 4;//RGBA
    int numRequestedPixels  = btMin(rgbaBufferSizeInPixels,numRemainingPixels);
    if (numRequestedPixels)
    {
        if (startPixelIndex==0)
        {
            CommonCameraInterface* oldCam = getRenderInterface()->getActiveCamera();
			SimpleCamera tempCam;
			getRenderInterface()->setActiveCamera(&tempCam);
			getRenderInterface()->getActiveCamera()->setVRCamera(viewMatrix,projectionMatrix);
			{
				BT_PROFILE("renderScene");
				getRenderInterface()->renderScene();
			}
			getRenderInterface()->setActiveCamera(oldCam);
			
			{
				BT_PROFILE("copy pixels");
                btAlignedObjectArray<unsigned char> sourceRgbaPixelBuffer;
                btAlignedObjectArray<float> sourceDepthBuffer;
                //copy the image into our local cache
                sourceRgbaPixelBuffer.resize(sourceWidth*sourceHeight*numBytesPerPixel);
                sourceDepthBuffer.resize(sourceWidth*sourceHeight);
				{
					BT_PROFILE("getScreenPixels");
	                m_data->m_glApp->getScreenPixels(&(sourceRgbaPixelBuffer[0]),sourceRgbaPixelBuffer.size(), &sourceDepthBuffer[0],sizeof(float)*sourceDepthBuffer.size());
				}
			
                m_data->m_rgbaPixelBuffer1.resize(destinationWidth*destinationHeight*numBytesPerPixel);
                m_data->m_depthBuffer1.resize(destinationWidth*destinationHeight);
                //rescale and flip
				{
					BT_PROFILE("resize and flip");
					for (int j=0;j<destinationHeight;j++)
					{
						for (int i=0;i<destinationWidth;i++)
						{
							int xIndex = int(float(i)*(float(sourceWidth)/float(destinationWidth)));
							int yIndex = int(float(destinationHeight-1-j)*(float(sourceHeight)/float(destinationHeight)));
							btClamp(xIndex,0,sourceWidth);
							btClamp(yIndex,0,sourceHeight);
							int bytesPerPixel = 4; //RGBA
                        
							int sourcePixelIndex = (xIndex+yIndex*sourceWidth)*bytesPerPixel;
							int sourceDepthIndex = xIndex+yIndex*sourceWidth;
#define COPY4PIXELS 1
#ifdef COPY4PIXELS
							int* dst = (int*)&m_data->m_rgbaPixelBuffer1[(i+j*destinationWidth)*4+0];
							int* src = (int*)&sourceRgbaPixelBuffer[sourcePixelIndex+0];
							*dst = *src;
							
#else
							m_data->m_rgbaPixelBuffer1[(i+j*destinationWidth)*4+0] = sourceRgbaPixelBuffer[sourcePixelIndex+0];
							m_data->m_rgbaPixelBuffer1[(i+j*destinationWidth)*4+1] = sourceRgbaPixelBuffer[sourcePixelIndex+1];
							m_data->m_rgbaPixelBuffer1[(i+j*destinationWidth)*4+2] = sourceRgbaPixelBuffer[sourcePixelIndex+2];
							m_data->m_rgbaPixelBuffer1[(i+j*destinationWidth)*4+3] = 255;
#endif                        
							if (depthBuffer)
							{ 
								m_data->m_depthBuffer1[i+j*destinationWidth] = sourceDepthBuffer[sourceDepthIndex];
							}
                        
						}
					}
                }
            }
        }
        if (pixelsRGBA)
        {
			BT_PROFILE("copy rgba pixels");

            for (int i=0;i<numRequestedPixels*numBytesPerPixel;i++)
            {
                pixelsRGBA[i] = m_data->m_rgbaPixelBuffer1[i+startPixelIndex*numBytesPerPixel];
            }
        }
        if (depthBuffer)
        {
			BT_PROFILE("copy depth buffer pixels");

            for (int i=0;i<numRequestedPixels;i++)
            {
                depthBuffer[i] = m_data->m_depthBuffer1[i+startPixelIndex];
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
	//sortedObjects.quickSort(shapePointerCompareFunc);
	for (int i=0;i<sortedObjects.size();i++)
	{
		btCollisionObject* colObj = sortedObjects[i];
		//btRigidBody* body = btRigidBody::upcast(colObj);
		//does this also work for btMultiBody/btMultiBodyLinkCollider?
		createCollisionShapeGraphicsObject(colObj->getCollisionShape());
		int colorIndex = colObj->getBroadphaseHandle()->getUid() & 3;

		btVector4 color;
		color = sColors[colorIndex];
		if (colObj->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE)
		{
			color.setValue(1,1,1,1);
		}
		createCollisionObjectGraphicsObject(colObj,color);
			
	}
}
    
void OpenGLGuiHelper::drawText3D( const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlags)
{
	B3_PROFILE("OpenGLGuiHelper::drawText3D");

    btAssert(m_data->m_glApp);
    m_data->m_glApp->drawText3D(txt,position, orientation, color,size, optionFlags);

}

void OpenGLGuiHelper::drawText3D( const char* txt, float posX, float posY, float posZ, float size)
{
	B3_PROFILE("OpenGLGuiHelper::drawText3D");

    btAssert(m_data->m_glApp);
    m_data->m_glApp->drawText3D(txt,posX,posY,posZ,size);
}

struct CommonGraphicsApp* OpenGLGuiHelper::getAppInterface()
{
	return m_data->m_glApp;
}

void	OpenGLGuiHelper::dumpFramesToVideo(const char* mp4FileName)
{
	if (m_data->m_glApp)
	{
		m_data->m_glApp->dumpFramesToVideo(mp4FileName);
	}
}
