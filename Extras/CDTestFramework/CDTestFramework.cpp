/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"

#include "CollisionTest.h"
#include "SphereMeshQuery.h"
#include "OBBMeshQuery.h"
#include "CapsuleMeshQuery.h"
#include "CompleteBoxPruning.h"
#include "BulletSAPCompleteBoxPruningTest.h"
#include "BulletSAPCompleteBoxPruningTest.h"
#include "BipartiteBoxPruning.h"
#include "OpcodeArraySAPTest.h"
#include "RenderingHelpers.h"
#include "Terrain.h"
#include "Camera.h"
#include "GLFontRenderer.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "LinearMath/btQuickprof.h"

#define NUM_SAP_BOXES 8192
//#define NUM_SAP_BOXES 16384
//#define NUM_SAP_BOXES 1024

int		percentUpdate	=	100;
//float	objectSpeed		=	0.005f;
float	objectSpeed		=	0.01f;
bool	enableDraw		=	true;

//Broadphase comparison
//Static case (updating 10% of objects to same position ( -> no swaps)
//number of objects //OPCODE BoxPruning / Bullet SAP / Bullet MultiSAP
//1024                     0.35ms, 0.03ms, 0.15ms
//8192                       21ms, 0.2ms, 5ms
//16384                   92ms , 0.5ms, 28ms

//Dynamic case, 10% objects are moving as fast as this testbed allows (0.01?)
//number of objects //OPCODE BoxPruning / Bullet SAP / Bullet MultiSAP
//1024						0.35ms, 0.2ms, 0.25ms
//8192                        21ms , 15ms , 13ms
//16384                    92ms,  80ms,  49ms


#define WINDOW_WIDTH	1024
#define WINDOW_HEIGHT	768

static int gMouseX = 0;
static int gMouseY = 0;
static int gButton = 0;

static TwBar* gMainBar = null;
enum TestIndex
{
//	TEST_SPHERE_MESH_QUERY,
//	TEST_OBB_MESH_QUERY,
//	TEST_CAPSULE_MESH_QUERY,
//	TEST_COMPLETE_BOX_PRUNING=0,
 	TEST_COMPLETE_BOX_PRUNING_8192,
//	TEST_BULLET_SAP_1024,
//	TEST_BULLET_SAP_8192,
//	TEST_BULLET_SAP_SORTEDPAIRS_8192,
// 	TEST_BULLET_MULTISAP_8192,
//	TEST_BIPARTITE_BOX_PRUNING,
	TEST_DBVT_8192,
	TEST_BULLET_CUDA_8192,
	TEST_BULLET_3DGRID_8192,
 	TEST_OPCODE_ARRAY_SAP,
	MAX_NB_TESTS
};

//static int gTest = TEST_DBVT_8192;//TEST_BULLET_MULTISAP_8192;
//static int gSelectedTest = TEST_DBVT_8192;//TEST_BULLET_MULTISAP_8192;
static int gTest = TEST_BULLET_CUDA_8192;
static int gSelectedTest = TEST_BULLET_CUDA_8192;
static CollisionTest* gCollisionTests[MAX_NB_TESTS];

static GLFontRenderer gFnt;

/////////////////

static void KeyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:	exit(0); break;

		case '+':
		{
			if(gTest!=MAX_NB_TESTS-1)
			{
				gCollisionTests[gTest]->Deselect();
				gTest++;
				gSelectedTest++;
				gCollisionTests[gTest]->Select();
			}
		}
		break;

		case '-':
		{
			if(gTest)
			{
				gCollisionTests[gTest]->Deselect();
				gTest--;
				gSelectedTest--;
				gCollisionTests[gTest]->Select();
			}
		}
		break;

		case 101:	MoveCameraForward();									break;
		case 103:	MoveCameraBackward();									break;
		case 100:	MoveCameraRight();										break;
		case 102:	MoveCameraLeft();										break;
		default:	gCollisionTests[gTest]->KeyboardCallback(key, x, y);	break;
	}

	TwEventKeyboardGLUT(key, x, y);
}

static void ArrowKeyCallback(int key, int x, int y)
{
	KeyboardCallback(key, x, y);

	TwEventSpecialGLUT(key, x, y);
}
	
static void MouseCallback(int button, int state, int x, int y)
{
	gButton = button;
	gMouseX = x;
	gMouseY = y;

	if(!TwEventMouseButtonGLUT(button, state, x, y))
	{
		gCollisionTests[gTest]->MouseCallback(button, state, x, y);
	}
}

static void MotionCallback(int x, int y)
{
	if(!TwEventMouseMotionGLUT(x, y))
	{
		if(gButton==2)
		{
			int dx = gMouseX - x;
			int dy = gMouseY - y;
			
			RotateCamera(dx, dy);

			gMouseX = x;
			gMouseY = y;
		}
		else
			gCollisionTests[gTest]->MotionCallback(x, y);
	}
}

static void RenderCallback()
{
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	// Setup camera
	glMatrixMode(GL_PROJECTION);
	SetupCameraMatrix();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_LIGHTING);

	if(0 /*gRenderWireframe*/)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	gCollisionTests[gTest]->PerformTest();

	// Draw tweak bars
	TwDraw();

	glutSwapBuffers();
	glutPostRedisplay();

	if(gSelectedTest!=gTest)
	{
		gCollisionTests[gTest]->Deselect();
		gTest = gSelectedTest;
		gCollisionTests[gTest]->Select();
	}
}

static void ReshapeCallback(int width, int height)
{
	glViewport(0, 0, width, height);

	// Send the new window size to AntTweakBar
	TwWindowSize(width, height);
}

static void IdleCallback()
{
	glutPostRedisplay();
}

static void Terminate()
{
	ReleaseTerrain();

	for(int i=0;i<MAX_NB_TESTS;i++)
	{
		gCollisionTests[i]->Release();
		DELETESINGLE(gCollisionTests[i]);
	}

	if(gMainBar)
	{
		TwDeleteBar(gMainBar);
		gMainBar = null;
	}

	TwTerminate();
}

int main(int argc, char** argv)
{
	{
	::SetPriorityClass(::GetCurrentProcess(),REALTIME_PRIORITY_CLASS);
	/*btDbvt::benchmark();
	exit(0);*/
	}
	// Initialize AntTweakBar
	// (note that AntTweakBar could also be intialize after GLUT, no matter)
	if(!TwInit(TW_OPENGL, NULL))
	{
		// A fatal error occured	
		fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
	}

	// Initialize Glut
	glutInit(&argc, argv);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	int mainHandle = glutCreateWindow("CD Test Framework");

/*	HWND hWnd;
	hWnd = FindWindow("GLUT", "CD Test Framework");
	RECT Rect;
	GetWindowRect(hWnd, &Rect);
*/
	glutCreateMenu(NULL);
	glutSetWindow(mainHandle);
	glutDisplayFunc(RenderCallback);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(IdleCallback);
	glutKeyboardFunc(KeyboardCallback);
	glutSpecialFunc(ArrowKeyCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);
	atexit(Terminate);	// Called after glutMainLoop ends

	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	TwGLUTModifiersFunc(glutGetModifiers);

	// Setup default render states
	glClearColor(0.3f, 0.4f, 0.5f, 1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);

	// Setup lighting
	glEnable(GL_LIGHTING);
	float AmbientColor[] = { 0.0f, 0.1f, 0.2f, 0.0f };		glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientColor);
	float DiffuseColor[] = { 1.0f, 1.0f, 1.0f, 0.0f };		glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseColor);
	float SpecularColor[] = { 0.0f, 0.0f, 0.0f, 0.0f };		glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularColor);
	float Position[] = { -10.0f, 1000.0f, -4.0f, 1.0f };	glLightfv(GL_LIGHT0, GL_POSITION, Position);
	glEnable(GL_LIGHT0);

	gFnt.init();
	gFnt.setScreenResolution(WINDOW_WIDTH, WINDOW_HEIGHT);
	gFnt.setColor(1.0f, 1.0f, 1.0f, 1.0f);

	CreateTerrain();

	// Create main tweak bar
	{
		gMainBar = TwNewBar("CollisionTests");
		TwEnumVal testEV[MAX_NB_TESTS] = {
//			{TEST_SPHERE_MESH_QUERY, "Sphere-mesh query"},
//			{TEST_OBB_MESH_QUERY, "OBB-mesh query"},
//			{TEST_CAPSULE_MESH_QUERY, "Capsule-mesh query"},
//			{TEST_COMPLETE_BOX_PRUNING, "OPCODE SAP 1024"},
			{TEST_COMPLETE_BOX_PRUNING_8192, "OPCODE BOX PRUNING 8192"},
//			{TEST_BULLET_SAP_1024, "Bullet SAP HASHPAIR 1024"},
//			{TEST_BULLET_SAP_8192, "Bullet SAP HASHPAIR 8192"},
//			{TEST_BULLET_SAP_SORTEDPAIRS_8192, "Bullet SAP SORTEDPAIR 8192"},
//			{TEST_BULLET_MULTISAP_8192, "Bullet MultiSAP 8192"},
//			{TEST_BIPARTITE_BOX_PRUNING, "Bipartite box pruning"},
			{TEST_DBVT_8192, "Bullet DBVT 8192"},
			{TEST_BULLET_CUDA_8192, "Bullet CUDA 8192"},
			{TEST_BULLET_3DGRID_8192, "Bullet 3D Grid 8192"},
			{TEST_OPCODE_ARRAY_SAP, "OPCODE ARRAY SAP"},
		};
		TwType testType = TwDefineEnum("CollisionTest", testEV, MAX_NB_TESTS);
		TwAddVarRW(gMainBar, "CollisionTests", testType, &gSelectedTest, "");		
		TwAddVarRW(gMainBar, "% of updates",TW_TYPE_INT32,&percentUpdate,"min=0 max=100");
		TwAddVarRW(gMainBar, "Draw",TW_TYPE_BOOLCPP,&enableDraw,"");
	}

	// Create tests
	gTest = 0;
//	gCollisionTests[TEST_SPHERE_MESH_QUERY]	= new SphereMeshQuery;
//	gCollisionTests[TEST_OBB_MESH_QUERY]	= new OBBMeshQuery;
//	gCollisionTests[TEST_CAPSULE_MESH_QUERY]	= new CapsuleMeshQuery;
//	gCollisionTests[TEST_COMPLETE_BOX_PRUNING]	= new CompleteBoxPruningTest(NUM_SAP_BOXES);
 //	gCollisionTests[TEST_COMPLETE_BOX_PRUNING_8192]	= new CompleteBoxPruningTest(NUM_SAP_BOXES);
	gCollisionTests[TEST_COMPLETE_BOX_PRUNING_8192]	= new CompleteBoxPruningTest(NUM_SAP_BOXES);
//	gCollisionTests[TEST_BULLET_SAP_1024]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,1);
// 	gCollisionTests[TEST_BULLET_SAP_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,1);
//	gCollisionTests[TEST_BULLET_SAP_SORTEDPAIRS_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,3);
// 	gCollisionTests[TEST_BULLET_MULTISAP_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,6);
//	gCollisionTests[TEST_BIPARTITE_BOX_PRUNING]	= new BipartiteBoxPruningTest;
	gCollisionTests[TEST_DBVT_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,7);
	gCollisionTests[TEST_BULLET_CUDA_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,8);
	gCollisionTests[TEST_BULLET_3DGRID_8192]	= new BulletSAPCompleteBoxPruningTest(NUM_SAP_BOXES,9);
	gCollisionTests[TEST_OPCODE_ARRAY_SAP]	= new OpcodeArraySAPTest(NUM_SAP_BOXES);

	for(int i=0;i<MAX_NB_TESTS;i++)
		gCollisionTests[i]->Init();
	gCollisionTests[gTest]->Select();

	//
	MotionCallback(0,0);

	// Run
	glutMainLoop();

	return 0;
}


#ifdef OLDIES

#include "btBulletCollisionCommon.h"

class BulletMeshInterface : public btStridingMeshInterface
{
	public:
		/// get read and write access to a subpart of a triangle mesh
		/// this subpart has a continuous array of vertices and indices
		/// in this way the mesh can be handled as chunks of memory with striding
		/// very similar to OpenGL vertexarray support
		/// make a call to unLockVertexBase when the read and write access is finished	
		virtual void	getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0)
		{
			numverts = gTerrainData->nbVerts;
			(*vertexbase) = (unsigned char *)gTerrainData->verts;
			type = PHY_FLOAT;
			stride = sizeof(Point);

			numfaces = gTerrainData->nbFaces;
			(*indexbase) = (unsigned char *)gTerrainData->faces;
			indicestype = PHY_INTEGER;
			indexstride = 3*sizeof(udword);	// ??
		}
		
		virtual void	getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0) const
		{
			numverts = gTerrainData->nbVerts;
			(*vertexbase) = (unsigned char *)gTerrainData->verts;
			type = PHY_FLOAT;
			stride = sizeof(Point);

			numfaces = gTerrainData->nbFaces;
			(*indexbase) = (unsigned char *)gTerrainData->faces;
			indicestype = PHY_INTEGER;
			indexstride = 3*sizeof(udword);	// ??
		}
	
		/// unLockVertexBase finishes the access to a subpart of the triangle mesh
		/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
		virtual void	unLockVertexBase(int subpart)
		{
		}

		virtual void	unLockReadOnlyVertexBase(int subpart) const
		{
		}


		/// getNumSubParts returns the number of seperate subparts
		/// each subpart has a continuous array of vertices and indices
		virtual int		getNumSubParts() const
		{
			return 1;
		}

		virtual void	preallocateVertices(int numverts)
		{
		}
		virtual void	preallocateIndices(int numindices)
		{
		}
};

void BuildBulletTree()
{
/*	BulletMeshInterface btMeshInterface;

	btOptimizedBvh* btTree = new btOptimizedBvh;
	btTree->build(&btMeshInterface, true);


	struct MyNodeOverlapCallback : public btNodeOverlapCallback
	{
		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
		}
	};

	MyNodeOverlapCallback	myNodeCallback(callback,m_meshInterface);

	m_bvh->reportAabbOverlappingNodex(&myNodeCallback,aabbMin,aabbMax);
*/


}

#endif