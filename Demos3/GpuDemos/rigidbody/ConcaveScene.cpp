#include "ConcaveScene.h"
#include "GpuRigidBodyDemo.h"
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
#include"../../Wavefront/objLoader.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3ConvexUtility.h"


#include "OpenGLWindow/GLInstanceGraphicsShape.h"
#define CONCAVE_GAPX 16
#define CONCAVE_GAPY 8
#define CONCAVE_GAPZ 16


GLInstanceGraphicsShape* createGraphicsShapeFromWavefrontObj(objLoader* obj)
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


void ConcaveScene::createConcaveMesh(const ConstructionInfo& ci, const char* fileName, const b3Vector3& shift, const b3Vector3& scaling)
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
	
		int colIndex = m_data->m_np->registerConcaveMesh(&verts,shape->m_indices,b3Vector3(1,1,1));
		
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
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);
				index++;
			}



			delete shape->m_indices;
			delete shape->m_vertices;
			delete shape;

		}
	}
	
	delete objData;

}

void ConcaveScene::setupScene(const ConstructionInfo& ci)
{

	if (1)
	{

		//char* fileName = "data/slopedPlane100.obj";
	//char* fileName = "data/plane100.obj";
//	char* fileName = "data/plane100.obj";

	//char* fileName = "data/teddy.obj";//"plane.obj";
//	char* fileName = "data/sponza_closed.obj";//"plane.obj";
	//char* fileName = "data/leoTest1.obj";
	char* fileName = "data/samurai_monastry.obj";
//	char* fileName = "data/teddy2_VHACD_CHs.obj";
	
		b3Vector3 shift1(0,0,0);//0,230,80);//150,-100,-120);
		
		b3Vector4 scaling(4,4,4,1);

	//	createConcaveMesh(ci,"data/plane100.obj",shift1,scaling);
		//createConcaveMesh(ci,"data/plane100.obj",shift,scaling);

	//	b3Vector3 shift2(0,0,0);//0,230,80);//150,-100,-120);
	//	createConcaveMesh(ci,"data/teddy.obj",shift2,scaling);

	//	b3Vector3 shift3(130,-150,-75);//0,230,80);//150,-100,-120);
	//	createConcaveMesh(ci,"data/leoTest1.obj",shift3,scaling);
		createConcaveMesh(ci,"data/samurai_monastry.obj",shift1,scaling);	
				
	} else
	{
		int strideInBytes = 9*sizeof(float);
		int numVertices = sizeof(cube_vertices)/strideInBytes;
		int numIndices = sizeof(cube_indices)/sizeof(int);
		int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int group=1;
		int mask=1;
		int index=0;
		{
			b3Vector4 scaling(400,0.001,400,1);
			int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
			b3Vector3 position(0,-2,0);
			b3Quaternion orn(0,0,0,1);
				
			b3Vector4 color(0,0,1,1);
		
			int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
			int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);
	
		}
	}

	createDynamicObjects(ci);
	
	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4]={0,0,0,0};//65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(155);

}

void ConcaveScene::createDynamicObjects(const ConstructionInfo& ci)
{
		int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	
	
	int index=0;
	

	if (1)
	{
		int curColor = 0;
		b3Vector4 colors[4] = 
		{
			b3Vector4(1,1,1,1),
			b3Vector4(1,1,0.3,1),
			b3Vector4(0.3,1,1,1),
			b3Vector4(0.3,0.3,1,1),
		};


		b3ConvexUtility* utilPtr = new b3ConvexUtility();
		b3Vector4 scaling(1,1,1,1);

		{
			b3AlignedObjectArray<b3Vector3> verts;

			unsigned char* vts = (unsigned char*) cube_vertices;
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

//		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);

		int colIndex=-1;
		if (ci.m_useInstancedCollisionShapes)
			colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

		for (int i=0;i<ci.arraySizeX;i++)
		{

			for (int j=0;j<ci.arraySizeY;j++)
			{
				for (int k=0;k<ci.arraySizeZ;k++)
				{
					if (!ci.m_useInstancedCollisionShapes)
						colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

					float mass = 1;

					//b3Vector3 position(-2*ci.gapX+i*ci.gapX,25+j*ci.gapY,-2*ci.gapZ+k*ci.gapZ);
					b3Vector3 position(-(ci.arraySizeX/2)*CONCAVE_GAPX+i*CONCAVE_GAPX,3+j*CONCAVE_GAPY,-(ci.arraySizeZ/2)*CONCAVE_GAPZ+k*CONCAVE_GAPZ);
					b3Quaternion orn(0,0,0,1);
				
					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
				
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);
				
					index++;
				}
			}
		}
	}

}

void ConcaveCompoundScene::setupScene(const ConstructionInfo& ci)
{
	ConcaveScene::setupScene(ci);

	float camPos[4]={0,50,0,0};//65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);


}

void ConcaveCompound2Scene::createDynamicObjects(const ConstructionInfo& ci)
{
	objLoader* objData = new objLoader();

	char* fileName = "data/teddy2_VHACD_CHs.obj";
	//char* fileName = "data/cube_offset.obj";

	
	b3Vector3 shift(0,0,0);//0,230,80);//150,-100,-120);
	b3Vector4 scaling(1,1,1,1);
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
				fclose(f);
				break;
			}
		}
	}

	if (f)
		fclose(f);
	else
		return;

	objData->load(relativeFileName);

	if (objData->objectCount>0)
	{


		int strideInBytes = 9*sizeof(float);

		b3AlignedObjectArray<GLInstanceVertex> vertexArray;
		b3AlignedObjectArray<int> indexArray;

	


		//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int group=1;
		int mask=1;
		int index=0;
		int colIndex = 0;

		b3AlignedObjectArray<GLInstanceVertex> vertices;
		int stride2 = sizeof(GLInstanceVertex);
		b3Assert(stride2 == strideInBytes);

		{
		
		
			b3AlignedObjectArray<b3GpuChildShape> childShapes;

			int numChildShapes = objData->objectCount;

			for (int i=0;i<numChildShapes;i++)
//			int i=4;
			{

				obj_object* object = objData->objectList[i];
				int numVertices = i==numChildShapes-1? objData->vertexCount-object->vertex_offset : objData->objectList[i+1]->vertex_offset - object->vertex_offset;
				int numFaces = i==numChildShapes-1? objData->faceCount - object->face_offset : objData->objectList[i+1]->face_offset-object->face_offset;




				//for now, only support polyhedral child shapes
				b3GpuChildShape child;
			
				b3Vector3 pos(0,0,0);
				b3Quaternion orn(0,0,0,1);
				for (int v=0;v<4;v++)
				{
					child.m_childPosition[v] = pos[v];
					child.m_childOrientation[v] = orn[v];
				}
				
				b3Transform tr;
				tr.setIdentity();
				tr.setOrigin(pos);
				tr.setRotation(orn);

				int baseIndex = vertexArray.size();
				
				for (int f=0;f<numFaces;f++)
				{
					obj_face* face = objData->faceList[object->face_offset+f];
					if (face->vertex_count==3)
					{
						for (int i=0;i<3;i++)
						{
							indexArray.push_back(face->vertex_index[i]);//-object->vertex_offset);
						}
					} else
					{
						b3Assert(0);
					}
				}

				b3Vector3 center(0,0,0);

				b3AlignedObjectArray<GLInstanceVertex> tmpVertices;
				//add transformed graphics vertices and indices
				b3Vector3 myScaling(1,1,1);//50,50,50);//300,300,300);
				for (int v=0;v<numVertices;v++)
				{
					GLInstanceVertex vert;
					obj_vector* orgVert = objData->vertexList[object->vertex_offset+v];
					vert.uv[0] = 0.5f;
					vert.uv[1] = 0.5f;
					vert.normal[0]=0.f;
					vert.normal[1]=1.f;
					vert.normal[2]=0.f;
					b3Vector3 vertPos;
					vertPos[0] = orgVert->e[0]*myScaling[0];
					vertPos[1] = orgVert->e[1]*myScaling[1];
					vertPos[2] = orgVert->e[2]*myScaling[2];
					vertPos[3] =0.f;
					center+=vertPos;
				}

				center/=numVertices;

				for (int v=0;v<numVertices;v++)
				{
					GLInstanceVertex vert;
					obj_vector* orgVert = objData->vertexList[object->vertex_offset+v];
					vert.uv[0] = 0.5f;
					vert.uv[1] = 0.5f;
					vert.normal[0]=0.f;
					vert.normal[1]=1.f;
					vert.normal[2]=0.f;
					b3Vector3 vertPos;
					vertPos[0] = orgVert->e[0]*myScaling[0];
					vertPos[1] = orgVert->e[1]*myScaling[1];
					vertPos[2] = orgVert->e[2]*myScaling[2];
					vertPos[3] =0.f;
	//				vertPos-=center;
					vert.xyzw[0] = vertPos[0];
					vert.xyzw[1] = vertPos[1];
					vert.xyzw[2] = vertPos[2];

					tmpVertices.push_back(vert);
					b3Vector3 newPos = tr*vertPos;
					vert.xyzw[0] = newPos[0];
					vert.xyzw[1] = newPos[1];
					vert.xyzw[2] = newPos[2];
					vert.xyzw[3] = 0.f;
					vertexArray.push_back(vert);
				}


				int childColIndex = m_data->m_np->registerConvexHullShape(&tmpVertices[0].xyzw[0],strideInBytes,numVertices, scaling);
				child.m_shapeIndex = childColIndex;
				childShapes.push_back(child);
				colIndex = childColIndex;
			}
			colIndex= m_data->m_np->registerCompoundShape(&childShapes);
		
		}

		//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
		int shapeId = ci.m_instancingRenderer->registerShape(&vertexArray[0].xyzw[0],vertexArray.size(),&indexArray[0],indexArray.size());

		b3Vector4 colors[4] = 
		{
			b3Vector4(1,0,0,1),
			b3Vector4(0,1,0,1),
			b3Vector4(0,0,1,1),
			b3Vector4(0,1,1,1),
		};
		
		int curColor = 0;
		for (int i=0;i<ci.arraySizeX;i++)
		{
			for (int j=0;j<ci.arraySizeY;j++)
			{
				for (int k=0;k<ci.arraySizeZ;k++)

				{
					float mass = 1;//j==0? 0.f : 1.f;

					//b3Vector3 position(i*10*ci.gapX,j*ci.gapY,k*10*ci.gapZ);
					b3Vector3 position(i*10*ci.gapX,10+j*ci.gapY,k*10*ci.gapZ);

				//	b3Quaternion orn(0,0,0,1);
					b3Quaternion orn(b3Vector3(0,0,1),1.8);
				
					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);
				
					index++;
				}
			}
		}

	}
	delete objData;
}

void ConcaveCompoundScene::createDynamicObjects(const ConstructionInfo& ci)
{
	
		int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);

	b3AlignedObjectArray<GLInstanceVertex> vertexArray;
	b3AlignedObjectArray<int> indexArray;
	

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;
	float scaling[4] = {1,1,1,1};
	int colIndex = 0;

	GLInstanceVertex* cubeVerts = (GLInstanceVertex*)&cube_vertices[0];
	int stride2 = sizeof(GLInstanceVertex);
	b3Assert(stride2 == strideInBytes);

	{
		int childColIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		

b3Vector3 childPositions[3] = {
			b3Vector3(0,-2,0),
			b3Vector3(0,0,0),
			b3Vector3(0,0,2)
		};
		
		b3AlignedObjectArray<b3GpuChildShape> childShapes;
		int numChildShapes = 3;
		for (int i=0;i<numChildShapes;i++)
		{
			//for now, only support polyhedral child shapes
			b3GpuChildShape child;
			child.m_shapeIndex = childColIndex;
			b3Vector3 pos = childPositions[i];
			b3Quaternion orn(0,0,0,1);
			for (int v=0;v<4;v++)
			{
				child.m_childPosition[v] = pos[v];
				child.m_childOrientation[v] = orn[v];
			}
			childShapes.push_back(child);
			b3Transform tr;
			tr.setIdentity();
			tr.setOrigin(pos);
			tr.setRotation(orn);

			int baseIndex = vertexArray.size();
			for (int j=0;j<numIndices;j++)
				indexArray.push_back(cube_indices[j]+baseIndex);

			//add transformed graphics vertices and indices
			for (int v=0;v<numVertices;v++)
			{
				GLInstanceVertex vert = cubeVerts[v];
				b3Vector3 vertPos(vert.xyzw[0],vert.xyzw[1],vert.xyzw[2]);
				b3Vector3 newPos = tr*vertPos;
				vert.xyzw[0] = newPos[0];
				vert.xyzw[1] = newPos[1];
				vert.xyzw[2] = newPos[2];
				vert.xyzw[3] = 0.f;
				vertexArray.push_back(vert);
			}

		}
		colIndex= m_data->m_np->registerCompoundShape(&childShapes);
		
	}

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&vertexArray[0].xyzw[0],vertexArray.size(),&indexArray[0],indexArray.size());

	b3Vector4 colors[4] = 
	{
		b3Vector4(1,0,0,1),
		b3Vector4(0,1,0,1),
		b3Vector4(0,0,1,1),
		b3Vector4(0,1,1,1),
	};
		
	int curColor = 0;
	for (int i=0;i<ci.arraySizeX;i++)
	{
		for (int j=0;j<ci.arraySizeY;j++)
		{
			for (int k=0;k<ci.arraySizeZ;k++)

			{
				float mass = 1;//j==0? 0.f : 1.f;

				b3Vector3 position(i*ci.gapX,50+j*ci.gapY,k*ci.gapZ);
				//b3Quaternion orn(0,0,0,1);
				b3Quaternion orn(b3Vector3(1,0,0),0.7);
				
				b3Vector4 color = colors[curColor];
				curColor++;
				curColor&=3;
				b3Vector4 scaling(1,1,1,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);
				
				index++;
			}
		}
	}
}


void ConcaveSphereScene::setupScene(const ConstructionInfo& ci)
{
	ConcaveScene::setupScene(ci);

	float camPos[4]={0,50,0,0};//65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraPitch(45);
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(40);


}

void ConcaveSphereScene::createDynamicObjects(const ConstructionInfo& ci)
{
		b3Vector4 colors[4] = 
	{
		b3Vector4(1,0,0,1),
		b3Vector4(0,1,0,1),
		b3Vector4(0,1,1,1),
		b3Vector4(1,1,0,1),
	};

	int index=0;
	int curColor = 0;
	float radius = 1;
	//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int colIndex = m_data->m_np->registerSphereShape(radius);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int prevGraphicsShapeIndex = registerGraphicsSphereShape(ci,radius,false);

	for (int i=0;i<ci.arraySizeX;i++)
	{
		for (int j=0;j<ci.arraySizeY;j++)
		{
			for (int k=0;k<ci.arraySizeZ;k++)
			{
				float mass = 1.f;

				
				b3Vector3 position(-(ci.arraySizeX/2)*8+i*8,50+j*8,-(ci.arraySizeZ/2)*8+k*8);
					
				//b3Vector3 position(0,-41,0);//0,0,0);//i*radius*3,-41+j*radius*3,k*radius*3);
					
				b3Quaternion orn(0,0,0,1);
				
				b3Vector4 color = colors[curColor];
				curColor++;
				curColor&=3;
				b3Vector4 scaling(radius,radius,radius,1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(prevGraphicsShapeIndex,position,orn,color,scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);
				
				index++;
			}
		}
	}
}
