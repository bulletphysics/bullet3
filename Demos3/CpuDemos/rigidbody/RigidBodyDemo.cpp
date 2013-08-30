#include "RigidBodyDemo.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/ShapeData.h"
#include "Bullet3Common/b3Quaternion.h"

#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.h"
#include "Bullet3Dynamics/b3CpuRigidBodyPipeline.h"
#include "Bullet3Dynamics/shared/b3IntegrateTransforms.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"


static b3Vector4 colors[4] =
{
	b3MakeVector4(1,0,0,1),
	b3MakeVector4(0,1,0,1),
	b3MakeVector4(0,1,1,1),
	b3MakeVector4(1,1,0,1),
};



void    RigidBodyDemo::initPhysics(const ConstructionInfo& ci)
{
	m_instancingRenderer = ci.m_instancingRenderer;

	int x_dim=1;
	int y_dim=2;
	int z_dim=1;

	int aabbCapacity = x_dim*y_dim*z_dim+10;

	b3Config config;

	m_bp = new b3DynamicBvhBroadphase(aabbCapacity);
	m_np = new b3CpuNarrowPhase(config);
	m_rb = new b3CpuRigidBodyPipeline(m_np,m_bp, config);





	m_instancingRenderer ->setCameraDistance(100);
	float target[4]={0,0,0,0};
	m_instancingRenderer->setCameraTargetPosition(target);
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	
	


	

	
	{
		static int curColor=0;
		b3Vector4 color = colors[curColor];
		curColor++;
		curColor&=3;

		b3Vector3 position = b3MakeVector3(0,0,0);//((j+1)&1)+i*2.2,1+j*2.,((j+1)&1)+k*2.2);
		b3Quaternion orn(0,0,0,1);
		b3Vector4 scaling=b3MakeVector4(100,1,100,1);
		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		float mass=0.f;
		int collidableIndex = m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		int bodyIndex = m_rb->getNumBodies();
		

		int userData=-1;
		int rbid = m_rb->registerPhysicsInstance(mass, position, orn, collidableIndex, userData);
	}

	ci.m_instancingRenderer->writeTransforms();

	{
		
		b3Vector4 scaling=b3MakeVector4(1,1,1,1);
		int collidableIndex = m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		for (int x=0;x<x_dim;x++)
		{
			//for (int y=-y_dim/2;y<y_dim/2;y++)
			for (int y=0;y<y_dim;y++)
			{
				for (int z=0;z<z_dim;z++)
				{
					static int curColor=0;
					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;

					b3Vector3 position = b3MakeVector3(x*2,2+y*2,z*2);
					b3Quaternion orn(0,0,0,1);
					
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					float mass=1.f;
					
					int userData=-1;
					int bodyIndex = m_rb->getNumBodies();
					int rbid = m_rb->registerPhysicsInstance(mass, position, orn, collidableIndex, userData);
					
				}
			}
		}
	}

	
	printf("num objects = %d\n",m_rb->getNumBodies());

	
}
	
void    RigidBodyDemo::exitPhysics()
{
	delete m_rb;
	m_rb=0;
	delete m_np;
	m_np=0;
	delete m_bp;
	m_bp=0;
}
	
void RigidBodyDemo::renderScene()
{
	{
		B3_PROFILE("writeSingleInstanceTransformToCPU");
		const b3RigidBodyData* bodies = m_rb->getBodyBuffer();
		//sync transforms
		int numBodies = m_rb->getNumBodies();
		for (int i=0;i<numBodies;i++)
		{
			m_instancingRenderer->writeSingleInstanceTransformToCPU(&bodies[i].m_pos.x,bodies[i].m_quat,i);
		}
	}
	{
		B3_PROFILE("writeTransforms");
		m_instancingRenderer->writeTransforms();
	}
	{
		B3_PROFILE("renderScene");
		m_instancingRenderer->renderScene();
	}
}
	
void RigidBodyDemo::clientMoveAndDisplay()
{
	m_rb->stepSimulation(1.f/60.f);
}
