#include "ShadowMapDemo.h"
#include "ShadowMapDemoInternalData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/ShapeData.h"

ShadowMapDemo::ShadowMapDemo()
{
	m_shadowData = new ShadowMapDemoInternalData;
}

ShadowMapDemo::~ShadowMapDemo()
{
	delete m_shadowData;
}


void    ShadowMapDemo::initPhysics(const ConstructionInfo& ci)
{
	m_shadowData->m_instancingRenderer = ci.m_instancingRenderer;

	int sphereShape = registerGraphicsSphereShape(ci,0.1,false);
	float pos[4]={0,3,0,0};
	float orn[4]={0,0,0,1};
	float color[4]={1,0,0,1};
	float scaling[4]={1,1,1,1};

	ci.m_instancingRenderer->registerGraphicsInstance(sphereShape,pos,orn,color,scaling);
	

	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_vertices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int boxShape = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	pos[1]=0.f;
	scaling[0]=scaling[2]=10.f;
	color[0]=1.f;
	color[1]=1.f;
	color[2]=1.f;
	color[3]=1.f;
	ci.m_instancingRenderer->registerGraphicsInstance(boxShape ,pos,orn,color,scaling);


	m_shadowData->m_instancingRenderer->setCameraTargetPosition(pos);
	m_shadowData->m_instancingRenderer->setCameraDistance(15);
	ci.m_instancingRenderer->writeTransforms();
}
	
void    ShadowMapDemo::exitPhysics()
{
}
	
void ShadowMapDemo::renderScene()
{
	m_shadowData->m_instancingRenderer->renderScene(B3_CREATE_SHADOWMAP_RENDERMODE);
	//m_shadowData->m_instancingRenderer->renderScene();
	m_shadowData->m_instancingRenderer->renderScene(B3_USE_SHADOWMAP_RENDERMODE);
}
	
void ShadowMapDemo::clientMoveAndDisplay()
{
}
