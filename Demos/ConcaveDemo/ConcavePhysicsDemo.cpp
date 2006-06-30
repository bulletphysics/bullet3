/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "MyMotionState.h"
//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/Simplex1to4Shape.h"
#include "Dynamics/RigidBody.h"
#include "BroadphaseCollision/AxisSweep3.h"

#include "ConstraintSolver/SequentialImpulseConstraintSolver.h"
//#include "ConstraintSolver/OdeConstraintSolver.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "CollisionShapes/TriangleMeshShape.h"
#include "CollisionShapes/TriangleIndexVertexArray.h"
#include "CollisionShapes/BvhTriangleMeshShape.h"
#include "CollisionShapes/TriangleMesh.h"

#include "IDebugDraw.h"
//#include "GLDebugDrawer.h"

#include "PHY_Pro.h"


#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
#include <GL/glut.h>
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
	

const int numObjects = 80;

const int maxNumObjects = 100;
MyMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects] = {0,0,0,0};
int	shapeIndex[maxNumObjects];
CcdPhysicsEnvironment* physicsEnvironmentPtr = 0;

TriangleMesh meshData; 
StridingMeshInterface* ptr;


//GL_LineSegmentShape shapeE(SimdPoint3(-50,0,0),
//						   SimdPoint3(50,0,0));
CollisionShape* shapePtr[5] = 
{
	new BoxShape (SimdVector3(100,10,100)),
	new BoxShape (SimdVector3(2,2,2)),
	new BU_Simplex1to4(SimdPoint3(-2,-2,-2),SimdPoint3(2,-2,-2),SimdPoint3(-2,2,-2),SimdPoint3(0,0,2)),


	new BoxShape (SimdVector3(1,3,1)),
#ifdef DEBUG_MESH
	new TriangleMeshShape(&meshData),
#else
	NULL,
#endif
	//(&meshData)

};

static const int NUM_VERTICES = 5;
static const int NUM_TRIANGLES=4;

SimdVector3	gVertices[NUM_VERTICES];
int	gIndices[NUM_TRIANGLES*3];

int main(int argc,char** argv)
{

	printf("BroadphaseProxy: %i\n",sizeof(BroadphaseProxy));
	printf("AxisSweep3::Handle : %i\n",sizeof(AxisSweep3::Handle));

	printf("SimpleBroadphaseProxy : %i\n",sizeof(SimpleBroadphaseProxy));

	printf("RigidBody : %i\n",sizeof(RigidBody));

	printf("CcdPhysicsController: %i\n",sizeof(CcdPhysicsController));
	
	printf("ManifoldPoint: %i\n",sizeof(ManifoldPoint));
	
	

	setCameraDistance(30.f);

#define TRISIZE 10.f
#ifdef DEBUG_MESH
	SimdVector3 vert0(-TRISIZE ,0,TRISIZE );
	SimdVector3 vert1(TRISIZE ,10,TRISIZE );
	SimdVector3 vert2(TRISIZE ,0,-TRISIZE );
	meshData.AddTriangle(vert0,vert1,vert2);
	SimdVector3 vert3(-TRISIZE ,0,TRISIZE );
	SimdVector3 vert4(TRISIZE ,0,-TRISIZE );
	SimdVector3 vert5(-TRISIZE ,0,-TRISIZE );
	meshData.AddTriangle(vert3,vert4,vert5);
#else
#ifdef ODE_MESH
	SimdVector3 Size = SimdVector3(15.f,15.f,12.5f);
	
  gVertices[0][0] = -Size[0];
  gVertices[0][1] = Size[2];
  gVertices[0][2] = -Size[1];
  
  gVertices[1][0] = Size[0];
  gVertices[1][1] = Size[2];
  gVertices[1][2] = -Size[1];
  
  gVertices[2][0] = Size[0];
  gVertices[2][1] = Size[2];
  gVertices[2][2] = Size[1];  

  gVertices[3][0] = -Size[0];
  gVertices[3][1] = Size[2];
  gVertices[3][2] = Size[1];
  
  gVertices[4][0] = 0;
  gVertices[4][1] = 0;
  gVertices[4][2] = 0;
  
  gIndices[0] = 0;
  gIndices[1] = 1;
  gIndices[2] = 4;
  
  gIndices[3] = 1;
  gIndices[4] = 2;
  gIndices[5] = 4;
  
  gIndices[6] = 2;
  gIndices[7] = 3;
  gIndices[8] = 4;
  
  gIndices[9] = 3;
  gIndices[10] = 0;
  gIndices[11] = 4;

  int vertStride = sizeof(SimdVector3);
  int indexStride = 3*sizeof(int);

	TriangleIndexVertexArray* indexVertexArrays = new TriangleIndexVertexArray(NUM_TRIANGLES,
		gIndices,
		indexStride,
		NUM_VERTICES,(float*) &gVertices[0].x(),vertStride);

	//shapePtr[4] = new TriangleMeshShape(indexVertexArrays);
	shapePtr[4] = new BvhTriangleMeshShape(indexVertexArrays);
#else

	int vertStride = sizeof(SimdVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 50;
	const int NUM_VERTS_Y = 50;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	SimdVector3*	gVertices = new SimdVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];

	int i;

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*10.f,2.f*sinf((float)i)*cosf((float)j),(j-NUM_VERTS_Y*0.5f)*10.f);
		}
	}

	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	TriangleIndexVertexArray* indexVertexArrays = new TriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(float*) &gVertices[0].x(),vertStride);

	//shapePtr[4] = new TriangleMeshShape(indexVertexArrays);
	shapePtr[4] = new BvhTriangleMeshShape(indexVertexArrays);
#endif

	

#endif//DEBUG_MESH


//	GLDebugDrawer	debugDrawer;

	ConstraintSolver* solver = new SequentialImpulseConstraintSolver;
	//ConstraintSolver* solver = new OdeConstraintSolver;

	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
		
	OverlappingPairCache* broadphase = new SimpleBroadphase();


	physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	

	physicsEnvironmentPtr->setGravity(-1,-10,1);
	PHY_ShapeProps shapeProps;
	
	shapeProps.m_do_anisotropic = false;
	shapeProps.m_do_fh = false;
	shapeProps.m_do_rot_fh = false;
	shapeProps.m_friction_scaling[0] = 1.;
	shapeProps.m_friction_scaling[1] = 1.;
	shapeProps.m_friction_scaling[2] = 1.;

	shapeProps.m_inertia = 1.f;
	shapeProps.m_lin_drag = 0.95999998f;
	shapeProps.m_ang_drag = 0.89999998f;
	shapeProps.m_mass = 1.0f;
	
	PHY_MaterialProps materialProps;
	materialProps.m_friction = 0.f;// 50.5f;
	materialProps.m_restitution = 0.1f;

	CcdConstructionInfo ccdObjectCi;
	ccdObjectCi.m_friction = 0.f;//50.5f;

	ccdObjectCi.m_linearDamping = shapeProps.m_lin_drag;
	ccdObjectCi.m_angularDamping = shapeProps.m_ang_drag;

	SimdTransform tr;
	tr.setIdentity();

	
	for (i=0;i<numObjects;i++)
	{
		if (i>0)
			shapeIndex[i] = 1;//2 = tetrahedron
		else
			shapeIndex[i] = 4;
	}
	for (i=0;i<numObjects;i++)
	{
		shapeProps.m_shape = shapePtr[shapeIndex[i]];

		bool isDyna = i>0;
		
		if (!i)
		{
			//SimdQuaternion orn(0,0,0.1*SIMD_HALF_PI);
			//ms[i].setWorldOrientation(orn.x(),orn.y(),orn.z(),orn[3]);
			//ms[i].setWorldPosition(0,-10,0);
		} else
		{
				ms[i].setWorldPosition(10,i*15-10,0);
		}
		
//either create a few stacks, to show several islands, or create 1 large stack, showing stability
		//ms[i].setWorldPosition((i*5) % 30,i*15-10,0);
		

		ccdObjectCi.m_MotionState = &ms[i];
		ccdObjectCi.m_gravity = SimdVector3(0,0,0);
		ccdObjectCi.m_localInertiaTensor =SimdVector3(0,0,0);
		if (!isDyna)
		{
			shapeProps.m_mass = 0.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
		}
		else
		{
			shapeProps.m_mass = 1.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
		}

		
		SimdVector3 localInertia;
		if (shapeProps.m_mass>0.f)
		{
			shapePtr[shapeIndex[i]]->CalculateLocalInertia(shapeProps.m_mass,localInertia);
		} else
		{
			localInertia.setValue(0.f,0.f,0.f);

		}
		ccdObjectCi.m_localInertiaTensor = localInertia;

		ccdObjectCi.m_collisionShape = shapePtr[shapeIndex[i]];


		physObjects[i]= new CcdPhysicsController( ccdObjectCi);
		physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

/*		if (i==0)
		{
			physObjects[i]->SetAngularVelocity(0,0,-2,true);
			physObjects[i]->GetRigidBody()->setDamping(0,0);
		}
*/
		//for the line that represents the AABB extents
//	physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

		
	}
	return glutmain(argc, argv,640,480,"Static Concave Mesh Demo");
}



void renderme()
{
	float m[16];
	int i;

	for (i=0;i<numObjects;i++)
	{
		SimdTransform transA;
		transA.setIdentity();
		
		float pos[3];
		float rot[4];

		ms[i].getWorldPosition(pos[0],pos[1],pos[2]);
		ms[i].getWorldOrientation(rot[0],rot[1],rot[2],rot[3]);

		SimdQuaternion q(rot[0],rot[1],rot[2],rot[3]);
		transA.setRotation(q);

		SimdPoint3 dpos;
		dpos.setValue(pos[0],pos[1],pos[2]);

		transA.setOrigin( dpos );
		transA.getOpenGLMatrix( m );
		
		SimdVector3 wireColor(0.f,0.f,1.f); //wants deactivation

		///color differently for active, sleeping, wantsdeactivation states
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 1) //active
		{
			wireColor = SimdVector3 (1.f,0.f,0.f);
		}
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 2) //ISLAND_SLEEPING
		{
			wireColor = SimdVector3 (0.f,1.f,0.f);
		}

		char	extraDebug[125];
		//sprintf(extraDebug,"islId, Body=%i , %i",physObjects[i]->GetRigidBody()->m_islandTag1,physObjects[i]->GetRigidBody()->m_debugBodyId);
		shapePtr[shapeIndex[i]]->SetExtraDebugInfo(extraDebug);
		GL_ShapeDrawer::DrawOpenGL(m,shapePtr[shapeIndex[i]],wireColor,getDebugMode());
	}

}
void clientMoveAndDisplay()
{
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float deltaTime = 1.f/60.f;

	physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);
	
	renderme();

    glFlush();
    glutSwapBuffers();

}




void clientDisplay(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

    glFlush();
    glutSwapBuffers();
}


void clientResetScene()
{

}
void clientKeyboard(unsigned char key, int x, int y)
{
	defaultKeyboard(key, x, y);
}


void clientMouseFunc(int button, int state, int x, int y)
{

}
void	clientMotionFunc(int x,int y)
{
}