#include "ImportSTLSetup.h"
#include <vector>
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "LoadMeshFromSTL.h"

ImportSTLDemo::ImportSTLDemo(SimpleOpenGL3App* app)
:m_app(app)
{
    
}

ImportSTLDemo::~ImportSTLDemo()
{
    
}



void ImportSTLDemo::initPhysics(GraphicsPhysicsBridge& gfxBridge)
{
	gfxBridge.setUpAxis(2);
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
//	int index=10;
	
	{
		
		GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(relativeFileName);
		
		btTransform trans;
		trans.setIdentity();
		trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
		
		btVector3 position = trans.getOrigin();
		btQuaternion orn = trans.getRotation();
		
		btVector3 color(0,0,1);
		
		int shapeId = m_app->m_instancingRenderer->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices);
		
		
	//	int id = 
		m_app->m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);

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
