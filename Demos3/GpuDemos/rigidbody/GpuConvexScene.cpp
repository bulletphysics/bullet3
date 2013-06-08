#include "GpuConvexScene.h"
#include "GpuRigidBodyDemo.h"
#include "Bullet3Common/b3Quickprof.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "../gwenUserInterface.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3ConvexUtility.h"


void GpuConvexScene::setupScene(const ConstructionInfo& ci)
{
	m_primRenderer = ci.m_primRenderer;

	m_raycaster = new b3GpuRaycast(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);

	int index=0;
	createStaticEnvironment(ci);

	index+=createDynamicsObjects(ci);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4]={ci.arraySizeX,ci.arraySizeY/2,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};
	
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(100);
	

	m_instancingRenderer->updateCamera();

	char msg[1024];
	int numInstances = index;
	sprintf(msg,"Num objects = %d",numInstances);
	ci.m_gui->setStatusBarMessage(msg,true);
}

void	GpuConvexScene::destroyScene()
{
	delete m_raycaster;
	m_raycaster = 0;
}

int	GpuConvexScene::createDynamicsObjects(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(barrel_vertices)/strideInBytes;
	int numIndices = sizeof(barrel_indices)/sizeof(int);
	return createDynamicsObjects2(ci,barrel_vertices,numVertices,barrel_indices,numIndices);
}

int	GpuBoxPlaneScene::createDynamicsObjects(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	return createDynamicsObjects2(ci,cube_vertices,numVertices,cube_indices,numIndices);
}


int	GpuConvexScene::createDynamicsObjects2(const ConstructionInfo& ci, const float* vertices, int numVertices, const int* indices, int numIndices)
{
	int strideInBytes = 9*sizeof(float);

	int shapeId = ci.m_instancingRenderer->registerShape(&vertices[0],numVertices,indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;





	{
		b3Vector4 colors[4] =
		{
			b3Vector4(1,0,0,1),
			b3Vector4(0,1,0,1),
			b3Vector4(0,1,1,1),
			b3Vector4(1,1,0,1),
		};

		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		int prevBody = -1;
		int insta = 0;

		b3ConvexUtility* utilPtr = new b3ConvexUtility();

		{
			b3AlignedObjectArray<b3Vector3> verts;

			unsigned char* vts = (unsigned char*) vertices;
			for (int i=0;i<numVertices;i++)
			{
				float* vertex = (float*) &vts[i*strideInBytes];
				verts.push_back(b3Vector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
			}

			bool merge = true;
			if (numVertices)
			{
				utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
			}
		}

		int colIndex=-1;
		if (ci.m_useInstancedCollisionShapes)
			colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i=0;i<ci.arraySizeX;i++)
		{


			//printf("%d of %d\n", i, ci.arraySizeX);
			for (int j=0;j<ci.arraySizeY;j++)
			{

				for (int k=0;k<ci.arraySizeZ;k++)
				{
					//int colIndex = m_data->m_np->registerConvexHullShape(&vertices[0],strideInBytes,numVertices, scaling);
					if (!ci.m_useInstancedCollisionShapes)
						colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

					float mass = 1.f;
					if (j==0)//ci.arraySizeY-1)
					{
						//mass=0.f;
					}
					b3Vector3 position((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
					//b3Vector3 position(i*2.2,10+j*1.9,k*2.2);

					b3Quaternion orn(0,0,0,1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);


					if (prevBody>=0)
					{
						//b3Point2PointConstraint* p2p = new b3Point2PointConstraint(pid,prevBody,b3Vector3(0,-1.1,0),b3Vector3(0,1.1,0));
//						 m_data->m_rigidBodyPipeline->addConstraint(p2p);//,false);
					}
					prevBody = pid;

					index++;
				}
			}
		}
	}
	return index;
}


void GpuConvexScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;


	{
		b3Vector4 scaling(400,1,400,1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 position(0,0,0);
		b3Quaternion orn(0,0,0,1);

		b3Vector4 color(0,0,1,1);

		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

	}
}

void GpuConvexPlaneScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int index=0;
	b3Vector3 normal(0,1,0);
	float constant=0.f;
	int colIndex = m_data->m_np->registerPlaneShape(normal,constant);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	b3Vector3 position(0,0,0);
	b3Quaternion orn(0,0,0,1);
	//		b3Quaternion orn(b3Vector3(1,0,0),0.3);
	b3Vector4 color(0,0,1,1);
	b3Vector4 scaling(100,0.001,100,1);
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);


	int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
	int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

}

struct GpuRaytraceInternalData
{
	GLuint* m_texId;
	unsigned char* m_texels;
	int textureWidth;
	int textureHeight;
};
#include <string.h>

GpuRaytraceScene::GpuRaytraceScene()
{
	
	m_raytraceData = new GpuRaytraceInternalData;

	m_raytraceData->m_texId = new GLuint;
	m_raytraceData->textureWidth = 256;//1024;
	m_raytraceData->textureHeight = 256;

	//create new texture
	glGenTextures(1, m_raytraceData->m_texId);
	GLenum err = glGetError();
	assert(err==GL_NO_ERROR);

			
	
	glBindTexture(GL_TEXTURE_2D, *m_raytraceData->m_texId);
	m_raytraceData->m_texels = (unsigned char*)malloc(m_raytraceData->textureWidth*m_raytraceData->textureHeight*3);
	memset(m_raytraceData->m_texels,0,m_raytraceData->textureWidth*m_raytraceData->textureHeight*3);
	for (int i=0;i<m_raytraceData->textureWidth;i++)
	{
		for (int y=0;y<m_raytraceData->textureHeight;y++)
		{
			int color = 0;
			if (y<m_raytraceData->textureHeight-1 && (y>0) && (i>0 && i<m_raytraceData->textureWidth-1))
				color = 255;

			m_raytraceData->m_texels[(i+m_raytraceData->textureWidth*y)*3+0] = color;
			m_raytraceData->m_texels[(i+m_raytraceData->textureWidth*y)*3+1] = color;
			m_raytraceData->m_texels[(i+m_raytraceData->textureWidth*y)*3+2] = color;
		}
	}

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_raytraceData->textureWidth, m_raytraceData->textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, m_raytraceData->m_texels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	err = glGetError();
	assert(err==GL_NO_ERROR);

}
GpuRaytraceScene::~GpuRaytraceScene()
{
	glDeleteTextures(1,m_raytraceData->m_texId);
	delete[] m_raytraceData->m_texels;
	delete m_raytraceData;
}



int	GpuRaytraceScene::createDynamicsObjects(const ConstructionInfo& ci)
{
	float radius=1.f;
	int colIndex = m_data->m_np->registerSphereShape(radius);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int shapeId = registerGraphicsSphereShape(ci,radius,false);

	int group=1;
	int mask=1;
	int index=0;

	{
		b3Vector4 colors[4] =
	{
		b3Vector4(1,0,0,1),
		b3Vector4(0,1,0,1),
		b3Vector4(0,1,1,1),
		b3Vector4(1,1,0,1),
	};

		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		int prevBody = -1;
		int insta = 0;

		
		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i=0;i<10;i++)
		//for (int i=0;i<ci.arraySizeX;i++)
		{
			//for (int j=0;j<ci.arraySizeY;j++)
			for (int j=0;j<10;j++)
			{
			//	for (int k=0;k<ci.arraySizeZ;k++)
				for (int k=0;k<10;k++)
				{
					float mass = 1.f;
					if (j==0)//ci.arraySizeY-1)
					{
						//mass=0.f;
					}
					b3Vector3 position((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
					//b3Vector3 position(i*2.2,10+j*1.9,k*2.2);

					b3Quaternion orn(0,0,0,1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);


					if (prevBody>=0)
					{
						//b3Point2PointConstraint* p2p = new b3Point2PointConstraint(pid,prevBody,b3Vector3(0,-1.1,0),b3Vector3(0,1.1,0));
//						 m_data->m_rigidBodyPipeline->addConstraint(p2p);//,false);
					}
					prevBody = pid;

					index++;
				}
			}
		}
	}
	return index;

}


//create primary rays
b3AlignedObjectArray<b3RayInfo> rays;

void GpuRaytraceScene::renderScene()
{
	
	GpuBoxPlaneScene::renderScene();

	B3_PROFILE("raytrace");
	//raytrace into the texels
	m_instancingRenderer->updateCamera();
	//generate primary rays

	

	{
		B3_PROFILE("Generate primary rays");
		float top = 1.f;
		float bottom = -1.f;
		float nearPlane = 1.f;
		float farPlane = 1000.f;

		float tanFov = (top-bottom)*0.5f / nearPlane;
		float screenWidth = m_instancingRenderer->getScreenWidth();
		float screenHeight = m_instancingRenderer->getScreenHeight();

		float fov = 2. * atanf (tanFov);
		float aspect = screenWidth / screenHeight;

		b3Vector3	rayFrom, camTarget;
		m_instancingRenderer->getCameraPosition(rayFrom);
		m_instancingRenderer->getCameraTargetPosition(camTarget);
		b3Vector3 rayForward = camTarget-rayFrom;
		rayForward.normalize();
	
		rayForward*= farPlane;

		b3Vector3 rightOffset;
		b3Vector3 vertical(0.f,1.f,0.f);
		b3Vector3 hor;
		hor = rayForward.cross(vertical);
		hor.normalize();
		vertical = hor.cross(rayForward);
		vertical.normalize();

		float tanfov = tanf(0.5f*fov);

		hor *= aspect*2.f * farPlane * tanfov;
		vertical *= 2.f * farPlane * tanfov;

		b3Vector3 rayToCenter = rayFrom + rayForward;
		float texWidth = m_raytraceData->textureWidth;
		float texHeight = m_raytraceData->textureHeight;

	
		float widthFactor = (screenWidth/texWidth);
		float heightFactor = (screenHeight/texHeight);

		//should be screenwidth/height

		b3Vector3 dHor = hor * 1./float(screenWidth);
		b3Vector3 dVert = vertical * 1./float(screenHeight);

		b3Transform rayFromTrans;
		rayFromTrans.setIdentity();
		rayFromTrans.setOrigin(rayFrom);

		b3Transform rayFromLocal;
		b3Transform	rayToLocal;

		m_data->m_np->readbackAllBodiesToCpu();



		//create primary rays
		rays.resize(m_raytraceData->textureWidth*m_raytraceData->textureHeight);

		b3Vector3 rayTo;
		b3RayInfo ray;

		{
			for (int x=0;x<m_raytraceData->textureWidth;x++)
			{
				for (int y=0;y<m_raytraceData->textureHeight;y++)
				{

					rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
					rayTo += x * dHor*widthFactor;
					rayTo -= y * dVert*heightFactor;

					ray.m_from = rayFrom;
					ray.m_to = rayTo;
					rays[x+m_raytraceData->textureWidth*y] = ray;
				}
			}
		}
	}
	
	b3AlignedObjectArray<b3RayHit> hits;
	hits.resize(rays.size());

	{
		B3_PROFILE("init hits");
		for (int i=0;i<hits.size();i++)
		{
			hits[i].m_hitFraction = 1.f;
		}
	}


	m_raycaster->castRaysHost(rays, hits, this->m_data->m_np->getNumRigidBodies(), m_data->m_np->getBodiesCpu(), m_data->m_np->getNumCollidablesGpu(), m_data->m_np->getCollidablesCpu());

	{
		B3_PROFILE("write texels");

		for (int i=0;i<hits.size();i++)
		{
			bool hit = hits[i].m_hitFraction < 1.f;

			if (hit)
			{
				m_raytraceData->m_texels[(i)*3+0] = 255;
				m_raytraceData->m_texels[(i)*3+1] = 0;
				m_raytraceData->m_texels[(i)*3+2] = 0;
			} else
			{
				m_raytraceData->m_texels[(i)*3+0] = 0;
				m_raytraceData->m_texels[(i)*3+1] = 0;
				m_raytraceData->m_texels[(i)*3+2] = 0;
			}
		}
	}
	GLint err;
    
    err = glGetError();
    assert(err==GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);

	glBindTexture(GL_TEXTURE_2D, *m_raytraceData->m_texId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_raytraceData->textureWidth, m_raytraceData->textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, m_raytraceData->m_texels);

	err = glGetError();
    assert(err==GL_NO_ERROR);
	b3Assert(m_primRenderer);
	float color[4] = {1,1,1,0.2};
	//float rect[4] = {0,0,m_raytraceData->textureWidth,m_raytraceData->textureHeight};
	float rect[4] = {0,0,m_instancingRenderer->getScreenWidth(),m_instancingRenderer->getScreenHeight()};
	float u[2] = {0,1};
	float v[2] = {0,1};
	int useRGBA = 1;
	m_primRenderer->drawTexturedRect(rect[0],rect[1],rect[2],rect[3],color,u[0],v[0],u[1],v[1], useRGBA);
	err = glGetError();
    assert(err==GL_NO_ERROR);
}
