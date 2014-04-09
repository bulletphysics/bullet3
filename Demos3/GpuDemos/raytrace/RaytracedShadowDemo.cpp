
#include "RaytracedShadowDemo.h"




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
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "../rigidbody/GpuRigidBodyDemoInternalData.h"
#include "../gwenUserInterface.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h"

#include "OpenGLWindow/GLRenderToTexture.h"



struct GpuRaytraceInternalData
{
	GLuint* m_texId;
	unsigned char* m_texels;
	int textureWidth;
	int textureHeight;
	struct GLRenderToTexture* m_renderToTexture;

};
#include <string.h>

GpuRaytraceScene::GpuRaytraceScene()
{
	
	m_raytraceData = new GpuRaytraceInternalData;
	m_raytraceData->m_renderToTexture = 0;//new GLRenderToTexture();
	
	m_raytraceData->m_texId = new GLuint;
	m_raytraceData->textureWidth = 512;//1024;//1024;
	m_raytraceData->textureHeight = 512;//1024;

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
	delete m_raytraceData->m_renderToTexture;
	delete m_raytraceData->m_texId;
	delete m_raytraceData;
}



int	GpuRaytraceScene::createDynamicsObjects(const ConstructionInfo& ci2)
{
	//m_raytraceData->m_renderToTexture->init(ci2.m_instancingRenderer->getScreenWidth(),ci2.m_instancingRenderer->getScreenHeight());
	ConstructionInfo ci = ci2;
	ci.arraySizeX = 2;
	ci.arraySizeY = 50;
	ci.arraySizeZ = 2;

	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	return createDynamicsObjects2(ci,cube_vertices,numVertices,cube_indices,numIndices);

	float radius=1.f;
	int colIndex = m_data->m_np->registerSphereShape(radius);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int shapeId = registerGraphicsSphereShape(ci,radius,false);

	int group=1;
	int mask=1;
	int index=0;

	{
		b3Vector4 colors[4] =
	{
		b3MakeVector4(1,0,0,1),
		b3MakeVector4(0,1,0,1),
		b3MakeVector4(0,1,1,1),
		b3MakeVector4(1,1,0,1),
	};

		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		int prevBody = -1;
		int insta = 0;

		
		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i=0;i<1;i++)
		//for (int i=0;i<ci.arraySizeX;i++)
		{
			//for (int j=0;j<ci.arraySizeY;j++)
			for (int j=0;j<10;j++)
			{
			//	for (int k=0;k<ci.arraySizeZ;k++)
				for (int k=0;k<1;k++)
				{
					float mass = 1.f;
					if (j==0)//ci.arraySizeY-1)
					{
						//mass=0.f;
					}
					b3Vector3 position=b3MakeVector3((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
					//b3Vector3 position(i*2.2,10+j*1.9,k*2.2);

					b3Quaternion orn(0,0,0,1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling=b3MakeVector4(1,1,1,1);
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

void GpuRaytraceScene::renderScene()
{
	
	renderScene2();
	return;
	//m_raytraceData->m_renderToTexture->enable();
	m_instancingRenderer->renderScene();
	//m_raytraceData->m_renderToTexture->disable();
}

void GpuRaytraceScene::renderScene2()
{
	//If using the BVH to accelerate raycasting, the AABBs need to be updated or else they will
	//not match the actual rigid body positions after integration. The result is that rigid bodies
	//are not drawn or appear clipped, especially if they are moving quickly.
	m_data->m_rigidBodyPipeline->setupGpuAabbsFull();
	
//	GpuBoxPlaneScene::renderScene();
//	return;
	B3_PROFILE("raytrace");

	//raytrace into the texels
	{
		B3_PROFILE("update camera");
		m_instancingRenderer->updateCamera();
	}
	//generate primary rays

	{
		B3_PROFILE("readbackAllBodiesToCpu");
		m_data->m_np->readbackAllBodiesToCpu();
	}
	

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
		b3Vector3 vertical=b3MakeVector3(0.f,1.f,0.f);
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

		



		//create primary rays
		primaryRays.resize(m_raytraceData->textureWidth*m_raytraceData->textureHeight);

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
					primaryRays[x+m_raytraceData->textureWidth*y] = ray;
				}
			}
		}
	}
	
	b3AlignedObjectArray<b3RayHit> hits;
	{
		B3_PROFILE("hits.resize");
		hits.resize(primaryRays.size());
	}
	if (1)
	{
		B3_PROFILE("init hits");
		for (int i=0;i<hits.size();i++)
		{
			hits[i].m_hitFraction = 1.f;
			hits[i].m_hitResult2 = -1;
		}
	}

	b3Vector3 lightPos=b3MakeVector3(1000,1000,100);

	{
		B3_PROFILE("cast primary rays");
		//m_raycaster->castRaysHost(primaryRays, hits, this->m_data->m_np->getNumRigidBodies(), m_data->m_np->getBodiesCpu(), m_data->m_np->getNumCollidablesGpu(), m_data->m_np->getCollidablesCpu(),m_data->m_np->getInternalData());
		m_raycaster->castRays(primaryRays, hits, this->m_data->m_np->getNumRigidBodies(), m_data->m_np->getBodiesCpu(), m_data->m_np->getNumCollidablesGpu(), m_data->m_np->getCollidablesCpu(), m_data->m_np->getInternalData(), m_data->m_bp);
	}
	

	b3AlignedObjectArray<b3RayInfo> shadowRays;
	{
		B3_PROFILE("shadowRays.resize");
		shadowRays.resize(primaryRays.size());
	}
	b3AlignedObjectArray<b3RayHit> shadowHits;
	{
		B3_PROFILE("shadowHits.resize");
		shadowHits.resize(hits.size());
	}
	
	{
		B3_PROFILE("init shadow rays");
		for (int i=0;i<hits.size();i++)
		{
			if(hits[i].m_hitFraction<1.f)
			{
			
				//hits[i].m_hitPoint.setInterpolate3(primaryRays[i].m_from,primaryRays[i].m_to,hits[i].m_hitFraction);
				//b3Vector3 shift = (lightPos-hits[i].m_hitPoint).normalize()*0.001f;
				shadowRays[i].m_from = hits[i].m_hitPoint;
				shadowRays[i].m_to = lightPos;
				shadowHits[i].m_hitFraction=1.f;
				shadowHits[i].m_hitBody = hits[i].m_hitBody;
			} else
			{
				shadowRays[i].m_from.setValue(0,0,0);
				shadowRays[i].m_to.setValue(0,0,0);

				shadowHits[i].m_hitFraction=1.f;
				shadowHits[i].m_hitResult2 = -2;
			}
		}
	}

	{
		B3_PROFILE("cast shadow rays");
	//m_raycaster->castRaysHost(primaryRays, hits, this->m_data->m_np->getNumRigidBodies(), m_data->m_np->getBodiesCpu(), m_data->m_np->getNumCollidablesGpu(), m_data->m_np->getCollidablesCpu());
		m_raycaster->castRays(shadowRays, shadowHits, this->m_data->m_np->getNumRigidBodies(), m_data->m_np->getBodiesCpu(), m_data->m_np->getNumCollidablesGpu(), m_data->m_np->getCollidablesCpu(), m_data->m_np->getInternalData(), m_data->m_bp);
	}

	{
		B3_PROFILE("write texels");

		for (int i=0;i<shadowHits.size();i++)
		{
			bool hit = hits[i].m_hitFraction < 1.f;

			if (hit)
			{
				float dotje = hits[i].m_hitNormal.dot(lightPos);
				if (dotje>0.f)
				{
					if (shadowHits[i].m_hitFraction<1.f)
					{
						dotje = -1.f;
					}
				}

				if (dotje>0.f)
				{
						m_raytraceData->m_texels[(i)*3+0] = 128+128.f*hits[i].m_hitNormal.x;
						m_raytraceData->m_texels[(i)*3+1] = 128+128.f*hits[i].m_hitNormal.y;
						m_raytraceData->m_texels[(i)*3+2] = 128+128.f*hits[i].m_hitNormal.z;

					if (hits[i].m_hitBody==0)
					{
						m_raytraceData->m_texels[(i)*3+0] = 255;
						m_raytraceData->m_texels[(i)*3+1] = 255;
						m_raytraceData->m_texels[(i)*3+2] = 255;
					} else
					{
					}
				} else
				{
					if (dotje == -1.f)
					{
						m_raytraceData->m_texels[(i)*3+0] = 0;
						m_raytraceData->m_texels[(i)*3+1] = 0;
						m_raytraceData->m_texels[(i)*3+2] = 255;
					} else
					{
						m_raytraceData->m_texels[(i)*3+0] = 255;
						m_raytraceData->m_texels[(i)*3+1] = 0;
						m_raytraceData->m_texels[(i)*3+2] = 0;
					}
				}
			} else
			{
				m_raytraceData->m_texels[(i)*3+0] = 128;
				m_raytraceData->m_texels[(i)*3+1] = 128;
				m_raytraceData->m_texels[(i)*3+2] = 192;
			}
		}
	}
	GLint err;
    
	{
		B3_PROFILE("get error");
		err = glGetError();
		assert(err==GL_NO_ERROR);
		glActiveTexture(GL_TEXTURE0);
	}
	{
		B3_PROFILE("glTexImage2D");
		glBindTexture(GL_TEXTURE_2D, *m_raytraceData->m_texId);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_raytraceData->textureWidth, m_raytraceData->textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, m_raytraceData->m_texels);
	}

	{
		B3_PROFILE("glGetError");
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}
	b3Assert(m_primRenderer);
	float color[4] = {1,1,1,1};
	//float rect[4] = {0,0,m_raytraceData->textureWidth,m_raytraceData->textureHeight};
	float rect[4] = {0,0,m_instancingRenderer->getScreenWidth(),m_instancingRenderer->getScreenHeight()};
	float u[2] = {0,1};
	float v[2] = {0,1};
	int useRGBA = 1;
	{
		B3_PROFILE("drawTexturedRect");
		m_primRenderer->drawTexturedRect(rect[0],rect[1],rect[2],rect[3],color,u[0],v[0],u[1],v[1], useRGBA);
	}
	{
		B3_PROFILE("glGetError");
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}
}
