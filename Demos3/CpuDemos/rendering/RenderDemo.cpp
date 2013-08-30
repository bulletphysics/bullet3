#include "RenderDemo.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/ShapeData.h"
#include "Bullet3Common/b3Quaternion.h"


static b3Vector4 colors[4] =
{
	b3MakeVector4(1,0,0,1),
	b3MakeVector4(0,1,0,1),
	b3MakeVector4(0,1,1,1),
	b3MakeVector4(1,1,0,1),
};


void    RenderDemo::initPhysics(const ConstructionInfo& ci)
{
	m_instancingRenderer = ci.m_instancingRenderer;
	m_instancingRenderer ->setCameraDistance(10);
	float target[4]={0,0,0,0};
	m_instancingRenderer->setCameraTargetPosition(target);
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	
	b3Vector3 position = b3MakeVector3(0,0,0);//((j+1)&1)+i*2.2,1+j*2.,((j+1)&1)+k*2.2);
	b3Quaternion orn(0,0,0,1);

	static int curColor=0;
	b3Vector4 color = colors[curColor];
	curColor++;
	curColor&=3;
	b3Vector4 scaling=b3MakeVector4(1,1,1,1);
	int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);

	ci.m_instancingRenderer->writeTransforms();


}
	
void    RenderDemo::exitPhysics()
{
}
	
void RenderDemo::renderScene()
{
	m_instancingRenderer->renderScene();
}
	
void RenderDemo::clientMoveAndDisplay()
{
}
