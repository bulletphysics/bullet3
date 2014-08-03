#include "ImportSTLSetup.h"
#include <vector>
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"

ImportSTLDemo::ImportSTLDemo(SimpleOpenGL3App* app)
:m_app(app)
{
    
}

ImportSTLDemo::~ImportSTLDemo()
{
    
}

struct MySTLTriangle
{
	float normal[3];
	float vertex0[3];
	float vertex1[3];
	float vertex2[3];
};

GLInstanceGraphicsShape* LoadMeshFromSTL(const char* relativeFileName)
{
	GLInstanceGraphicsShape* shape = 0;
	
	FILE* file = fopen(relativeFileName,"rb");
	if (file)
	{
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
		{
			printf("Error: Cannot access file to determine size of %s\n", relativeFileName);
		} else
		{
			if (size)
			{
				printf("Open STL file of %d bytes\n",size);
				char* memoryBuffer = new char[size+1];
				int actualBytesRead = fread(memoryBuffer,1,size,file);
				if (actualBytesRead!=size)
				{
					printf("Error reading from file %s",relativeFileName);
				} else
				{
					int numTriangles = *(int*)&memoryBuffer[80];
					
					if (numTriangles)
					{
						
						shape = new GLInstanceGraphicsShape;
//						b3AlignedObjectArray<GLInstanceVertex>*	m_vertices;
//						int				m_numvertices;
//						b3AlignedObjectArray<int>* 		m_indices;
//						int				m_numIndices;
//						float			m_scaling[4];
						shape->m_scaling[0] = 1;
						shape->m_scaling[1] = 1;
						shape->m_scaling[2] = 1;
						shape->m_scaling[3] = 1;
						int index = 0;
						shape->m_indices = new b3AlignedObjectArray<int>();
						shape->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();
						for (int i=0;i<numTriangles;i++)
						{
							char* curPtr = &memoryBuffer[84+i*50];
							MySTLTriangle* tri = (MySTLTriangle*) curPtr;
							
							GLInstanceVertex v0,v1,v2;
							if (i==numTriangles-2)
							{
								printf("!\n");
							}
							v0.uv[0] = v1.uv[0] = v2.uv[0] = 0.5;
							v0.uv[1] = v1.uv[1] = v2.uv[1] = 0.5;
							for (int v=0;v<3;v++)
							{
								v0.xyzw[v] = tri->vertex0[v];
								v1.xyzw[v] = tri->vertex1[v];
								v2.xyzw[v] = tri->vertex2[v];
								v0.normal[v] = v1.normal[v] = v2.normal[v] = tri->normal[v];
							}
							v0.xyzw[3] = v1.xyzw[3] = v2.xyzw[3] = 0.f;
							
							shape->m_vertices->push_back(v0);
							shape->m_vertices->push_back(v1);
							shape->m_vertices->push_back(v2);
							
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							
						}
					}
				}
				
				delete[] memoryBuffer;
			}
		}
		fclose(file);
	}
	shape->m_numIndices = shape->m_indices->size();
	shape->m_numvertices = shape->m_vertices->size();
	return shape;
}


void ImportSTLDemo::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	this->createEmptyDynamicsWorld();
	gfxBridge.createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);

	const char* fileName = "l_finger_tip.stl";
    char relativeFileName[1024];
	const char* prefix[]={"./data/","../data/","../../data/","../../../data/","../../../../data/"};
	int prefixIndex=-1;
	{
		
		int numPrefixes = sizeof(prefix)/sizeof(char*);
		
		for (int i=0;i<numPrefixes;i++)
		{
			FILE* f = 0;
			sprintf(relativeFileName,"%s%s",prefix[i],fileName);
			f = fopen(relativeFileName,"r");
			if (f)
			{
				fclose(f);
				prefixIndex = i;
				break;
			}
		}
	}
	
	if (prefixIndex<0)
		return;
	
	btVector3 shift(0,0,0);
	btVector3 scaling(10,10,10);
	int index=10;
	
	{
		
		GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(relativeFileName);
		
		btTransform trans;
		trans.setIdentity();
		trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
		
		btVector3 position = trans.getOrigin();
		btQuaternion orn = trans.getRotation();
		
		btVector3 color(0,0,1);
		
		int shapeId = m_app->m_instancingRenderer->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices);
		
		
		int id = m_app->m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);

		/*

		btTriangleMesh* trimeshData = new btTriangleMesh();

		for (int i=0;i<gfxShape->m_numvertices;i++)
		{
			for (int j=0;j<3;j++)
				gfxShape->m_vertices->at(i).xyzw[j] += shift[j];
		}

		for (int i=0;i<gfxShape->m_numIndices;i+=3)
		{
			int index0 = gfxShape->m_indices->at(i);
			int index1 = gfxShape->m_indices->at(i+1);
			int index2 = gfxShape->m_indices->at(i+2);
			
			btVector3 v0(gfxShape->m_vertices->at(index0).xyzw[0],
										gfxShape->m_vertices->at(index0).xyzw[1],
										gfxShape->m_vertices->at(index0).xyzw[2]);
			btVector3 v1(gfxShape->m_vertices->at(index1).xyzw[0],
						 gfxShape->m_vertices->at(index1).xyzw[1],
						 gfxShape->m_vertices->at(index1).xyzw[2]);
			btVector3 v2(gfxShape->m_vertices->at(index2).xyzw[0],
						 gfxShape->m_vertices->at(index2).xyzw[1],
						 gfxShape->m_vertices->at(index2).xyzw[2]);
			
			trimeshData->addTriangle(v0,v1,v2);
		}
		
		//btConvexHullShape* convexShape = new btConvexHullShape(&verts[0].x(),verts.size(),sizeof(btVector3));
		btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(trimeshData,true);//meshInterface);
		
		btTransform startTrans;startTrans.setIdentity();
		btRigidBody* body = this->createRigidBody(0,startTrans,shape);
		//gfxBridge.createCollisionShapeGraphicsObject(shape);
		btVector3 color(0,0,1);
		 */
		//gfxBridge.createRigidBodyGraphicsObject(body,color);
	}
}
