#if 0
#include "ShadowMapDemo.h"
#include "ShadowMapDemoInternalData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"

#include "OpenGLWindow/ShapeData.h"
#include "Bullet3Common/b3Quaternion.h"
#include <stdio.h>

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
	m_shadowData->m_primitiveRenderer = ci.m_primRenderer;

	float pos[4]={0,3,0,0};
	float orn[4]={0,0,0,1};
	float color[4]={1,0,0,1};
	float scaling[4]={1,1,1,1};
	if (1)
	{
		///create a sphere
		int sphereShape = registerGraphicsSphereShape(ci,0.1,false);
		ci.m_instancingRenderer->registerGraphicsInstance(sphereShape,pos,orn,color,scaling);
	}
	if (1)
	{
		//create a cube
		int strideInBytes = 9*sizeof(float);
		int numVertices = sizeof(cube_vertices)/strideInBytes;
		int numIndices = sizeof(cube_vertices)/sizeof(int);
		//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int boxShape = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		pos[1]=0.f;
		scaling[0]=scaling[2]=50.f;
		color[0]=1.f;
		color[1]=1.f;
		color[2]=1.f;
		color[3]=1.f;
		ci.m_instancingRenderer->registerGraphicsInstance(boxShape ,pos,orn,color,scaling);
	}

	b3Vector3 shift(0,0,0);
	if (0)
	{
		b3Vector3 scaling(1,1,1);
		const char* filename="data/room_thickwalls.obj";
		this->createConcaveMesh(ci,filename,shift,scaling);
	}

	{
		 b3Vector3 camPos(0, 2,5);
		m_shadowData->m_instancingRenderer->setCameraTargetPosition(camPos);
		m_shadowData->m_instancingRenderer->setCameraPitch(0);
		m_shadowData->m_instancingRenderer->setCameraYaw(0);

		m_shadowData->m_instancingRenderer->setCameraDistance(15);
	}
	//m_shadowData->m_instancingRenderer->setCameraYaw(55);
	ci.m_instancingRenderer->writeTransforms();
}
	
void    ShadowMapDemo::exitPhysics()
{
}
	
	
void ShadowMapDemo::clientMoveAndDisplay()
{
}

#include"../../Wavefront/objLoader.h"
#include "OpenGLWindow/GLInstanceGraphicsShape.h"

static GLInstanceGraphicsShape* createGraphicsShapeFromWavefrontObj(objLoader* obj)
{
	b3AlignedObjectArray<GLInstanceVertex>* vertices = new b3AlignedObjectArray<GLInstanceVertex>;
	{
//		int numVertices = obj->vertexCount;
	//	int numIndices = 0;
		b3AlignedObjectArray<int>* indicesPtr = new b3AlignedObjectArray<int>;
			/*
		for (int v=0;v<obj->vertexCount;v++)
		{
			vtx.xyzw[0] = obj->vertexList[v]->e[0];
			vtx.xyzw[1] = obj->vertexList[v]->e[1];
			vtx.xyzw[2] = obj->vertexList[v]->e[2];
			b3Vector3 n(vtx.xyzw[0],vtx.xyzw[1],vtx.xyzw[2]);
			if (n.length2()>B3_EPSILON)
			{
				n.normalize();
				vtx.normal[0] = n[0];
				vtx.normal[1] = n[1];
				vtx.normal[2] = n[2];

			} else
			{
				vtx.normal[0] = 0; //todo
				vtx.normal[1] = 1;
				vtx.normal[2] = 0;
			}
			vtx.uv[0] = 0.5f;vtx.uv[1] = 0.5f;	//todo
			vertices->push_back(vtx);
		}
		*/

		for (int f=0;f<obj->faceCount;f++)
		{
			obj_face* face = obj->faceList[f];
			//b3Vector3 normal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
			if (face->vertex_count>=3)
			{
				b3Vector3 normal(0,1,0);
				int vtxBaseIndex = vertices->size();

				if (face->vertex_count<=4)
				{
					indicesPtr->push_back(vtxBaseIndex);
					indicesPtr->push_back(vtxBaseIndex+1);
					indicesPtr->push_back(vtxBaseIndex+2);
					
					GLInstanceVertex vtx0;
					vtx0.xyzw[0] = obj->vertexList[face->vertex_index[0]]->e[0];
					vtx0.xyzw[1] = obj->vertexList[face->vertex_index[0]]->e[1];
					vtx0.xyzw[2] = obj->vertexList[face->vertex_index[0]]->e[2];
					vtx0.xyzw[3] = 0.f;//obj->vertexList[face->vertex_index[0]]->e[2];

					vtx0.uv[0] = 0.5f;//obj->textureList[face->vertex_index[0]]->e[0];
					vtx0.uv[1] = 0.5f;//obj->textureList[face->vertex_index[0]]->e[1];

					GLInstanceVertex vtx1;
					vtx1.xyzw[0] = obj->vertexList[face->vertex_index[1]]->e[0];
					vtx1.xyzw[1] = obj->vertexList[face->vertex_index[1]]->e[1];
					vtx1.xyzw[2] = obj->vertexList[face->vertex_index[1]]->e[2];
					vtx1.xyzw[3]= 0.f;
					vtx1.uv[0] = 0.5f;//obj->textureList[face->vertex_index[1]]->e[0];
					vtx1.uv[1] = 0.5f;//obj->textureList[face->vertex_index[1]]->e[1];

					GLInstanceVertex vtx2;
					vtx2.xyzw[0] = obj->vertexList[face->vertex_index[2]]->e[0];
					vtx2.xyzw[1] = obj->vertexList[face->vertex_index[2]]->e[1];
					vtx2.xyzw[2] = obj->vertexList[face->vertex_index[2]]->e[2];
					vtx2.xyzw[3] = 0.f;
					vtx2.uv[0] = 0.5f;obj->textureList[face->vertex_index[2]]->e[0];
					vtx2.uv[1] = 0.5f;obj->textureList[face->vertex_index[2]]->e[1];


					b3Vector3 v0(vtx0.xyzw[0],vtx0.xyzw[1],vtx0.xyzw[2]);
					b3Vector3 v1(vtx1.xyzw[0],vtx1.xyzw[1],vtx1.xyzw[2]);
					b3Vector3 v2(vtx2.xyzw[0],vtx2.xyzw[1],vtx2.xyzw[2]);

					normal = (v1-v0).cross(v2-v0);
					normal.normalize();
					vtx0.normal[0] = normal[0];
					vtx0.normal[1] = normal[1];
					vtx0.normal[2] = normal[2];
					vtx1.normal[0] = normal[0];
					vtx1.normal[1] = normal[1];
					vtx1.normal[2] = normal[2];
					vtx2.normal[0] = normal[0];
					vtx2.normal[1] = normal[1];
					vtx2.normal[2] = normal[2];
					vertices->push_back(vtx0);
					vertices->push_back(vtx1);
					vertices->push_back(vtx2);
				}
				if (face->vertex_count==4)
				{

					indicesPtr->push_back(vtxBaseIndex);
					indicesPtr->push_back(vtxBaseIndex+1);
					indicesPtr->push_back(vtxBaseIndex+2);
					indicesPtr->push_back(vtxBaseIndex+3);
//
					GLInstanceVertex vtx3;
					vtx3.xyzw[0] = obj->vertexList[face->vertex_index[3]]->e[0];
					vtx3.xyzw[1] = obj->vertexList[face->vertex_index[3]]->e[1];
					vtx3.xyzw[2] = obj->vertexList[face->vertex_index[3]]->e[2];
					vtx3.uv[0] = 0.5;
					vtx3.uv[1] = 0.5;

					vtx3.normal[0] = normal[0];
					vtx3.normal[1] = normal[1];
					vtx3.normal[2] = normal[2];

					vertices->push_back(vtx3);

				}
			}
		}
		
		
		GLInstanceGraphicsShape* gfxShape = new GLInstanceGraphicsShape;
		gfxShape->m_vertices = vertices;
		gfxShape->m_numvertices = vertices->size();
		gfxShape->m_indices = indicesPtr;
		gfxShape->m_numIndices = indicesPtr->size();
		for (int i=0;i<4;i++)
			gfxShape->m_scaling[i] = 1;//bake the scaling into the vertices 
		return gfxShape;
	}
}


void ShadowMapDemo::createConcaveMesh(const ConstructionInfo& ci, const char* fileName, const b3Vector3& shift, const b3Vector3& scaling)
{
	objLoader* objData = new objLoader();
	
	FILE* f = 0;

	char relativeFileName[1024];
	{
		const char* prefix[]={"./","../","../../","../../../","../../../../"};
		int numPrefixes = sizeof(prefix)/sizeof(char*);

		for (int i=0;i<numPrefixes;i++)
		{
			
			sprintf(relativeFileName,"%s%s",prefix[i],fileName);
			f = fopen(relativeFileName,"r");
			if (f)
			{
				break;
			}
		}
	}

	if (f)
	{
		fclose(f);
		f=0;
	}
	else
		return;

	objData->load(relativeFileName);
	int index=10;

	{
		GLInstanceGraphicsShape* shape = createGraphicsShapeFromWavefrontObj(objData);
	

		b3AlignedObjectArray<b3Vector3> verts;
		for (int i=0;i<shape->m_numvertices;i++)
		{
			for (int j=0;j<3;j++)
				shape->m_vertices->at(i).xyzw[j] += shift[j];

			b3Vector3 vtx(shape->m_vertices->at(i).xyzw[0],
						  shape->m_vertices->at(i).xyzw[1],
						  shape->m_vertices->at(i).xyzw[2]);
			verts.push_back(vtx*scaling);
		}
	
		
		{
			int strideInBytes = 9*sizeof(float);
			int numVertices = sizeof(cube_vertices)/strideInBytes;
			int numIndices = sizeof(cube_indices)/sizeof(int);
			//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
			//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);


			int shapeId = ci.m_instancingRenderer->registerShape(&shape->m_vertices->at(0).xyzw[0], shape->m_numvertices, &shape->m_indices->at(0), shape->m_numIndices);
			b3Quaternion orn(0,0,0,1);
				
			b3Vector4 color(0.3,0.3,1,1.f);//0.5);//1.f
	
				
			{
				float mass = 0.f;
				b3Vector3 position(0,0,0);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				index++;
			}
		}
	}
	delete objData;
}




void ShadowMapDemo::renderScene()
{
	float color[4]={1,1,1,1};
	m_shadowData->m_instancingRenderer->renderScene();

	m_shadowData->m_instancingRenderer->enableShadowMap();
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE );
//	glTexParameteri( GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE );
	m_shadowData->m_primitiveRenderer->drawTexturedRect(10,10,90,90,color,0,0,1,1,true);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);

	
}
#endif 


