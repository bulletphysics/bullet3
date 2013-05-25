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

void GpuConvexScene::setupScene(const ConstructionInfo& ci)
{
	m_primRenderer = ci.m_primRenderer;

	int index=0;
	createStaticEnvironment(ci);

	index+=createDynamicsObjects(ci);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();


	float camPos[4]={0,0,0,0};//ci.arraySizeX,ci.arraySizeY/2,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};
	
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(30);
	m_instancingRenderer->setCameraYaw(0);
	m_instancingRenderer->setCameraPitch(0);

	m_instancingRenderer->updateCamera();

	char msg[1024];
	int numInstances = index;
	sprintf(msg,"Num objects = %d",numInstances);
	ci.m_gui->setStatusBarMessage(msg,true);
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

		int colIndex = m_data->m_np->registerConvexHullShape(&vertices[0],strideInBytes,numVertices, scaling);
		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i=0;i<ci.arraySizeX;i++)
		{
			for (int j=0;j<ci.arraySizeY;j++)
			{
				for (int k=0;k<ci.arraySizeZ;k++)
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
						b3Point2PointConstraint* p2p = new b3Point2PointConstraint(pid,prevBody,b3Vector3(0,-1.1,0),b3Vector3(0,1.1,0));
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
	m_raytraceData->textureWidth = 1024;
	m_raytraceData->textureHeight = 768;

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


bool sphere_intersect(const b3Vector3& spherePos,  b3Scalar radius, const b3Vector3& rayFrom, const b3Vector3& rayTo)
{
    // rs = ray.org - sphere.center
    const b3Vector3& rs = rayFrom - spherePos;
	b3Vector3 rayDir = rayTo-rayFrom;//rayFrom-rayTo;
	rayDir.normalize();

    float B = b3Dot(rs, rayDir);
    float C = b3Dot(rs, rs) - (radius * radius);
    float D = B * B - C;

    if (D > 0.0)
    {
        float t = -B - sqrt(D);
        if ( (t > 0.0))// && (t < isect.t) )
        {
            return true;//isect.t = t;
		}
	}
	return false;
}

void GpuRaytraceScene::renderScene()
{
	B3_PROFILE("raytrace");
	//raytrace into the texels
	m_instancingRenderer->updateCamera();
	//generate primary rays

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;

	float tanFov = (top-bottom)*0.5f / nearPlane;

	float fov = 2.0 * atanf (tanFov);

	b3Vector3	rayFrom, camTarget;
	m_instancingRenderer->getCameraPosition(rayFrom);
	m_instancingRenderer->getCameraTargetPosition(camTarget);
	b3Vector3 rayForward = camTarget-rayFrom;
	rayForward.normalize();
	float farPlane = 500.f;
	rayForward*= farPlane;

	b3Vector3 rightOffset;
	b3Vector3 vertical(0.f,1.f,0.f);
	b3Vector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);

	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	b3Vector3 rayToCenter = rayFrom + rayForward;
	
	//should be screenwidth/height
	b3Vector3 dHor = hor * 1.f/float(m_raytraceData->textureWidth);
	b3Vector3 dVert = vertical * 1.f/float(m_raytraceData->textureHeight);

	b3Transform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);

	b3Transform rayFromLocal;
	b3Transform	rayToLocal;



	//cast primary rays

	m_data->m_np->readbackAllBodiesToCpu();

	for (int x=0;x<m_raytraceData->textureWidth;x++)
	{
		for (int y=0;y<m_raytraceData->textureHeight;y++)
		{

			b3Vector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
			rayTo += x * dHor;
			rayTo -= y * dVert;

			//if there is a hit, color the pixels
			int numBodies = m_data->m_rigidBodyPipeline->getNumBodies();
			bool hits  = false;

			for (int i=0;i<numBodies && !hits;i++)
			{
				
				b3Vector3 pos;
				b3Quaternion orn;
				m_data->m_np->getObjectTransformFromCpu(pos,orn,i);
				b3Scalar radius = 1;

				hits = sphere_intersect(pos,  radius, rayFrom, rayTo);
			}

			if (hits)
			{
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+0] = 255;
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+1] = 0;
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+2] = 0;
			} else
			{
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+0] = 0;
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+1] = 0;
				m_raytraceData->m_texels[(x+m_raytraceData->textureWidth*y)*3+2] = 0;
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
	float color[4] = {1,1,1,1};
	float rect[4] = {0,0,m_raytraceData->textureWidth,m_raytraceData->textureHeight};
	float u[2] = {0,1};
	float v[2] = {0,1};
	int useRGBA = 1;
	m_primRenderer->drawTexturedRect(rect[0],rect[1],rect[2],rect[3],color,u[0],v[0],u[1],v[1], useRGBA);
	err = glGetError();
    assert(err==GL_NO_ERROR);
}
