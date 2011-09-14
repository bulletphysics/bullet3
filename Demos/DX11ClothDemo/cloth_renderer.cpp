//--------------------------------------------------------------------------------------
// File: BasicHLSL10.cpp
//
// This sample shows a simple example of the Microsoft Direct3D's High-Level 
// Shader Language (HLSL) using the Effect interface. 
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

#include "DXUT.h"
#include "DXUTcamera.h"
#include "DXUTgui.h"
#include "DXUTsettingsDlg.h"
#include "SDKmisc.h"
#include "SDKMesh.h"
#include "resource.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"
#include "btDirectComputeSupport.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/DX11/btSoftBodySolverVertexBuffer_DX11.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "vectormath/vmInclude.h"

class btDefaultSoftBodySolver;
class btCPUSoftBodySolver;
class btCPUSoftBodyVertexSolver;
class btDX11SoftBodySolver;
class btDX11SIMDAwareSoftBodySolver;

#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletSoftBody/btDefaultSoftBodySolver.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/DX11/btSoftBodySolver_DX11.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/DX11/btSoftBodySolver_DX11SIMDAware.h"

#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#define USE_SIMDAWARE_SOLVER
//#define USE_GPU_SOLVER
#define USE_GPU_COPY
const int numFlags = 5;
const int clothWidth = 40;
const int clothHeight = 60;
float _windAngle = 1.0;//0.4;
float _windStrength = 15;


//#define TABLETEST


#include <fstream>

#include <cmath>

using Vectormath::Aos::Vector3;


class piece_of_cloth;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

int paused = 0;

float global_shift_x = 0;
float global_shift_y = 0;
float global_shift_z = 0;

namespace BTAcceleratedSoftBody
{
	class BulletPhysicsDevice;
	class CPUDevice;
	class DX11Device;	
}
namespace Vectormath
{
	namespace Aos
	{
		class Transform3;
	}
}



const float flagSpacing = 30.f;

#include <iostream>
using namespace std;

//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------
CDXUTDialogResourceManager  g_DialogResourceManager; // manager for shared resources of dialogs
//CModelViewerCamera          g_Camera;               // A model viewing camera
CFirstPersonCamera          g_Camera;               // A model viewing camera
CDXUTDirectionWidget        g_LightControl;
CD3DSettingsDlg             g_D3DSettingsDlg;       // Device settings dialog
CDXUTDialog                 g_HUD;                  // manages the 3D   
CDXUTDialog                 g_SampleUI;             // dialog for sample specific controls
D3DXMATRIXA16               g_mCenterMesh;
float                       g_fLightScale;
int                         g_nNumActiveLights;
int                         g_nActiveLight;
bool                        g_bShowHelp = false;    // If true, it renders the UI control text

// Direct3D9 resources
CDXUTTextHelper*            g_pTxtHelper = NULL;

CDXUTSDKMesh                g_Mesh11;

ID3D11InputLayout*          g_pVertexLayout11 = NULL;
ID3D11Buffer*               g_pVertexBuffer = NULL;
//ID3D11Buffer*               g_pIndexBuffer = NULL;
ID3D11VertexShader*         g_pVertexShader = NULL;
ID3D11GeometryShader*         g_pGeometryShader = NULL;
ID3D11PixelShader*          g_pPixelShader = NULL;
ID3D11SamplerState*         g_pSamLinear = NULL;

ID3D11RasterizerState *g_pRasterizerState = NULL;
ID3D11RasterizerState *g_pRasterizerStateWF = NULL;

bool g_wireFrame = false;

struct CB_VS_PER_OBJECT
{
    D3DXMATRIX m_WorldViewProj;
    D3DXMATRIX m_World;
};
UINT                        g_iCBVSPerObjectBind = 0;

struct CB_PS_PER_OBJECT
{
    D3DXVECTOR4 m_vObjectColor;
};
UINT                        g_iCBPSPerObjectBind = 0;

struct CB_PS_PER_FRAME
{
    D3DXVECTOR4 m_vLightDirAmbient;
};
UINT                        g_iCBPSPerFrameBind = 1;

ID3D11Buffer*               g_pcbVSPerObject = NULL;
ID3D11Buffer*               g_pcbPSPerObject = NULL;
ID3D11Buffer*               g_pcbPSPerFrame = NULL;

ID3D11Device* g_pd3dDevice;




// Create our vertex input layout
const D3D11_INPUT_ELEMENT_DESC layout[] =
{
    { "POSITION",  0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,  D3D11_INPUT_PER_VERTEX_DATA, 0 },
    { "NORMAL",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
    { "TEXCOORD",  0, DXGI_FORMAT_R32G32_FLOAT,    0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
};

struct vertex_struct 
{
	D3DXVECTOR3 Pos;
	D3DXVECTOR3 Normal;
	D3DXVECTOR2 Texcoord;
};





#include "capsule.h"
#include "cap.h"
#include "cylinder.h"
#include "cloth.h"



//cylinder cyl_1;
//cap cap_1;
btRigidBody *capCollider;
capsule my_capsule;


btAlignedObjectArray<piece_of_cloth> cloths;

//////////////////////////////////////////
// Bullet globals

btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
btBroadphaseInterface*	m_broadphase;
btCollisionDispatcher*	m_dispatcher;
btConstraintSolver*	m_solver;
btDefaultCollisionConfiguration* m_collisionConfiguration;
BTAcceleratedSoftBody::DX11SupportHelper m_dxSupport;

btAlignedObjectArray<btSoftBody *> m_flags;
btSoftRigidDynamicsWorld* m_dynamicsWorld;

btDefaultSoftBodySolver *g_defaultSolver = NULL;
btCPUSoftBodySolver *g_cpuSolver = NULL;
btDX11SoftBodySolver *g_dx11Solver = NULL;
btDX11SIMDAwareSoftBodySolver *g_dx11SIMDSolver = NULL;

btSoftBodySolverOutput *g_softBodyOutput = NULL;

btSoftBodySolver *g_solver = NULL;

// End bullet globals
//////////////////////////////////////////

// Helper to test and add links correctly.
// Records links that have already been generated
static bool testAndAddLink( btAlignedObjectArray<int> &trianglesForLinks, btSoftBody *softBody, int triangle, int *triangleVertexIndexArray, int numVertices, int vertex0, int vertex1, int nonLinkVertex, btSoftBody::Material *structuralMaterial, bool createBendLinks, btSoftBody::Material *bendMaterial )
{		
	if( trianglesForLinks[ numVertices * vertex0 + vertex1 ] >= 0 && createBendLinks)
	{
		// Already have link so find other triangle and generate cross link

		int otherTriangle = trianglesForLinks[numVertices * vertex0 + vertex1];
		int otherIndices[3] = {triangleVertexIndexArray[otherTriangle * 3], triangleVertexIndexArray[otherTriangle * 3 + 1], triangleVertexIndexArray[otherTriangle * 3 + 2]};

		int nodeA;
		// Test all links of the other triangle against this link. The one that's not part of it is what we want.
		if( otherIndices[0] != vertex0 && otherIndices[0] != vertex1 )
			nodeA = otherIndices[0];
		if( otherIndices[1] != vertex0 && otherIndices[1] != vertex1 )
			nodeA = otherIndices[1];
		if( otherIndices[2] != vertex0 && otherIndices[2] != vertex1 )
			nodeA = otherIndices[2];

		softBody->appendLink( nodeA, nonLinkVertex, bendMaterial );

		return true;
	} else {
		// Don't yet have link so create it
		softBody->appendLink( vertex0, vertex1, structuralMaterial );

		// If we added a new link, set the triangle array
		trianglesForLinks[numVertices * vertex0 + vertex1] = triangle;
		trianglesForLinks[numVertices * vertex1 + vertex0] = triangle;
		return true;
	}
}

btSoftBody *createFromIndexedMesh( btVector3 *vertexArray, int numVertices, int *triangleVertexIndexArray, int numTriangles, bool createBendLinks )
{
	btSoftBody* softBody = new btSoftBody(&(m_dynamicsWorld->getWorldInfo()), numVertices, vertexArray, 0);
	btSoftBody::Material * structuralMaterial = softBody->appendMaterial();
	btSoftBody::Material * bendMaterial;
	if( createBendLinks )
	{
		bendMaterial = softBody->appendMaterial();
		bendMaterial->m_kLST = 0.7;
	} else {
		bendMaterial = NULL;
	}
	structuralMaterial->m_kLST = 1.0;
	

	// List of values for each link saying which triangle is associated with that link
	// -1 to start. Once a value is entered we know the "other" triangle
	// and can add a link across the link
	btAlignedObjectArray<int> triangleForLinks;
	triangleForLinks.resize( numVertices * numVertices, -1 );
	for( int triangle = 0; triangle < numTriangles; ++triangle )
	{
		int index[3] = {triangleVertexIndexArray[triangle * 3], triangleVertexIndexArray[triangle * 3 + 1], triangleVertexIndexArray[triangle * 3 + 2]};
		softBody->appendFace( index[0], index[1], index[2] );
		
		// Generate the structural links directly from the triangles
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[0], index[1], index[2], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[1], index[2], index[0], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[2], index[0], index[1], structuralMaterial, createBendLinks, bendMaterial);
	}

	return softBody;
}

/**
 * Create a sequence of flag objects and add them to the world.
 */
void createFlag( int width, int height, btAlignedObjectArray<btSoftBody *> &flags )
{
	// First create a triangle mesh to represent a flag

	using namespace BTAcceleratedSoftBody;	
	using Vectormath::Aos::Matrix3;
	using Vectormath::Aos::Vector3;

	// Allocate a simple mesh consisting of a vertex array and a triangle index array
	btIndexedMesh mesh;
	mesh.m_numVertices = width*height;
	mesh.m_numTriangles = 2*(width-1)*(height-1);

	btVector3 *vertexArray = new btVector3[mesh.m_numVertices];

	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertexArray);
	int *triangleVertexIndexArray = new int[3*mesh.m_numTriangles];	
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(triangleVertexIndexArray);
	mesh.m_triangleIndexStride = sizeof(int)*3;
	mesh.m_vertexStride = sizeof(Vector3);

	// Generate normalised object space vertex coordinates for a rectangular flag
	float zCoordinate = 0.0f;
	
	Matrix3 defaultScale(Vector3(5.f, 0.f, 0.f), Vector3(0.f, 20.f, 0.f), Vector3(0.f, 0.f, 1.f));
	for( int y = 0; y < height; ++y )
	{
		float yCoordinate = y*2.0f/float(height) - 1.0f;
		for( int x = 0; x < width; ++x )
		{			
			float xCoordinate = x*2.0f/float(width) - 1.0f;

			Vector3 vertex(xCoordinate, yCoordinate, zCoordinate);
			Vector3 transformedVertex = defaultScale*vertex;

			vertexArray[y*width + x] = btVector3(transformedVertex.getX(), transformedVertex.getY(), transformedVertex.getZ() );

		}
	}

	// Generate vertex indices for triangles
	for( int y = 0; y < (height-1); ++y )
	{
		for( int x = 0; x < (width-1); ++x )
		{	
			// Triangle 0
			// Top left of square on mesh
			{
				int vertex0 = y*width + x;
				int vertex1 = vertex0 + 1;
				int vertex2 = vertex0 + width;
				int triangleIndex = 2*y*(width-1) + 2*x;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+1)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+2)/sizeof(int)+2] = vertex2;
			}

			// Triangle 1
			// Bottom right of square on mesh
			{
				int vertex0 = y*width + x + 1;
				int vertex1 = vertex0 + width;
				int vertex2 = vertex1 - 1;
				int triangleIndex = 2*y*(width-1) + 2*x + 1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+2] = vertex2;
			}
		}
	}

	
	float rotateAngleRoundZ = 0.5;
	float rotateAngleRoundX = 0.5;
	btMatrix3x3 defaultRotate;
	defaultRotate[0] = btVector3(cos(rotateAngleRoundZ), sin(rotateAngleRoundZ), 0.f); 
	defaultRotate[1] = btVector3(-sin(rotateAngleRoundZ), cos(rotateAngleRoundZ), 0.f);
	defaultRotate[2] = btVector3(0.f, 0.f, 1.f);


	//btMatrix3x3 defaultRotateAndScale( (defaultRotateX*defaultRotate) );
#ifdef TABLETEST
	btMatrix3x3 defaultRotateX;
	rotateAngleRoundX = 3.141592654/2;
	defaultRotateX[0] = btVector3(1.f, 0.f, 0.f);
	defaultRotateX[1] = btVector3( 0.f, cos(rotateAngleRoundX), sin(rotateAngleRoundX));
	defaultRotateX[2] = btVector3(0.f, -sin(rotateAngleRoundX), cos(rotateAngleRoundX));
	btMatrix3x3 defaultRotateAndScale( (defaultRotateX) );
#else
	btMatrix3x3 defaultRotateX;
	defaultRotateX[0] = btVector3(1.f, 0.f, 0.f);
	defaultRotateX[1] = btVector3( 0.f, cos(rotateAngleRoundX), sin(rotateAngleRoundX));
	defaultRotateX[2] = btVector3(0.f, -sin(rotateAngleRoundX), cos(rotateAngleRoundX));
	btMatrix3x3 defaultRotateAndScale( (defaultRotateX) );
#endif


	// Construct the sequence flags applying a slightly different translation to each one to arrange them
	// appropriately in the scene.
	for( int i = 0; i < numFlags; ++i )
	{
		float zTranslate = flagSpacing * (i-numFlags/2);

		btVector3 defaultTranslate(0.f, 20.f, zTranslate);

		btTransform transform( defaultRotateAndScale, defaultTranslate );


		btSoftBody *softBody = createFromIndexedMesh( vertexArray, mesh.m_numVertices, triangleVertexIndexArray, mesh.m_numTriangles, true );


		for( int i = 0; i < mesh.m_numVertices; ++i )
		{
			softBody->setMass(i, 10.f/mesh.m_numVertices);
		}

#ifndef TABLETEST
		// Set the fixed points
		softBody->setMass((height-1)*(width), 0.f);
		softBody->setMass((height-1)*(width) + width - 1, 0.f);
		softBody->setMass((height-1)*width + width/2, 0.f);
#endif

		softBody->m_cfg.collisions = btSoftBody::fCollision::CL_SS+btSoftBody::fCollision::CL_RS;	
		softBody->m_cfg.kLF = 0.0005f;
		softBody->m_cfg.kVCF = 0.001f;
		softBody->m_cfg.kDP = 0.f;
		softBody->m_cfg.kDG = 0.f;
		
		flags.push_back( softBody );

		softBody->transform( transform );
		softBody->setFriction( 0.8f );
		m_dynamicsWorld->addSoftBody( softBody );
	}

	delete [] vertexArray;
	delete [] triangleVertexIndexArray;
}





void updatePhysicsWorld()
{
	static int counter = 0;

	// Change wind velocity a bit based on a frame counter
	if( (counter % 400) == 0 )
	{
		_windAngle = (_windAngle + 0.05f);
		if( _windAngle > (2*3.141) )
			_windAngle = 0;

		for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
		{		
			btSoftBody *cloth = 0;

			cloth = m_flags[flagIndex];

			float localWind = _windAngle + 0.5*(((float(rand())/RAND_MAX))-0.1);
			float xCoordinate = cos(localWind)*_windStrength;
			float zCoordinate = sin(localWind)*_windStrength;

			cloth->setWindVelocity( btVector3(xCoordinate, 0, zCoordinate) );
		}
	}
#ifndef TABLETEST
	if (capCollider)
	{
		btVector3 origin( capCollider->getWorldTransform().getOrigin() );
		origin.setZ( origin.getZ() + 0.01 );
		capCollider->getWorldTransform().setOrigin( origin );
	}
#endif

	counter++;
}

void initBullet(void)
{


#ifdef USE_GPU_SOLVER
	g_dx11Solver = new btDX11SoftBodySolver( g_pd3dDevice, DXUTGetD3D11DeviceContext() );
	g_solver = g_dx11Solver;
#ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputDXtoDX( g_pd3dDevice, DXUTGetD3D11DeviceContext() );
#else // #ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputDXtoCPU;
#endif // #ifdef USE_GPU_COPY
#else
#ifdef USE_SIMDAWARE_SOLVER
	g_dx11SIMDSolver = new btDX11SIMDAwareSoftBodySolver( g_pd3dDevice, DXUTGetD3D11DeviceContext() );
	g_solver = g_dx11SIMDSolver;
	g_softBodyOutput = new btSoftBodySolverOutputDXtoCPU;
#ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputDXtoDX( g_pd3dDevice, DXUTGetD3D11DeviceContext() );
#else // #ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputDXtoCPU;
#endif // #ifdef USE_GPU_COPY
#else
	g_cpuSolver = new btCPUSoftBodySolver;
	g_solver = g_cpuSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCPUtoCPU;
	//g_defaultSolver = new btDefaultSoftBodySolver;
	//g_solver = g_defaultSolver;
#endif
#endif

	if (g_dx11SIMDSolver)
		g_dx11SIMDSolver->setEnableUpdateBounds(true);

	if (g_dx11Solver)
		g_dx11Solver->setEnableUpdateBounds(true);

	// Initialise CPU physics device
	//m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, g_solver);	

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));	
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));	
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	m_dynamicsWorld->getWorldInfo().air_density			=	(btScalar)1.2;
	m_dynamicsWorld->getWorldInfo().water_density		=	0;
	m_dynamicsWorld->getWorldInfo().water_offset		=	0;
	m_dynamicsWorld->getWorldInfo().water_normal		=	btVector3(0,0,0);
	m_dynamicsWorld->getWorldInfo().m_gravity.setValue(0,-10,0);
	
#if 0
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
 
#endif
#if 0
	{		
		btScalar mass(0.);

		//btScalar mass(1.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		
		btCollisionShape *capsuleShape = new btCapsuleShape(5, 10);
		capsuleShape->setMargin( 0.5 );
		
		

		my_capsule.set_collision_shape(capsuleShape);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			capsuleShape->calculateLocalInertia(mass,localInertia);

		m_collisionShapes.push_back(capsuleShape);
		btTransform capsuleTransform;
		capsuleTransform.setIdentity();
#ifdef TABLETEST
		capsuleTransform.setOrigin(btVector3(0, 10, -11));
		const btScalar pi = 3.141592654;
		capsuleTransform.setRotation(btQuaternion(0, 0, pi/2));
#else
		capsuleTransform.setOrigin(btVector3(0, 20, -10));
		
		const btScalar pi = 3.141592654;
		//capsuleTransform.setRotation(btQuaternion(0, 0, pi/2));
		capsuleTransform.setRotation(btQuaternion(0, 0, 0));
#endif
		btDefaultMotionState* myMotionState = new btDefaultMotionState(capsuleTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,capsuleShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction( 0.8f );
		my_capsule.set_collision_object(body);

		m_dynamicsWorld->addRigidBody(body);
		//cap_1.collisionShape = body;
		capCollider = body;
	}
#endif


	createFlag( clothWidth, clothHeight, m_flags );

	// Create output buffer descriptions for ecah flag
	// These describe where the simulation should send output data to
	for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	{		
		// In this case we have a DX11 output buffer with a vertex at index 0, 8, 16 and so on as well as a normal at 3, 11, 19 etc.
		// Copies will be performed GPU-side directly into the output buffer
#ifdef USE_GPU_COPY
		btDX11VertexBufferDescriptor *vertexBufferDescriptor = new btDX11VertexBufferDescriptor(DXUTGetD3D11DeviceContext(), cloths[flagIndex].pVB[0], cloths[flagIndex].g_pVB_UAV, 0, 8, 3, 8);
		cloths[flagIndex].m_vertexBufferDescriptor = vertexBufferDescriptor;
#else  // #ifdef USE_GPU_COPY
		btCPUVertexBufferDescriptor *vertexBufferDescriptor = new btCPUVertexBufferDescriptor(cloths[flagIndex].cpu_buffer, 0, 8, 3, 8);
		cloths[flagIndex].m_vertexBufferDescriptor = vertexBufferDescriptor;
#endif // #ifdef USE_GPU_COPY
	}

	g_solver->optimize( m_dynamicsWorld->getSoftBodyArray() );

}



































//--------------------------------------------------------------------------------------
// UI control IDs
//--------------------------------------------------------------------------------------
#define IDC_TOGGLEFULLSCREEN    1
#define IDC_TOGGLEREF           3
#define IDC_CHANGEDEVICE        4
#define IDC_PAUSE               5
#define IDC_WIREFRAME           6

//--------------------------------------------------------------------------------------
// Forward declarations 
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings( DXUTDeviceSettings* pDeviceSettings, void* pUserContext );
void CALLBACK OnFrameMove( double fTime, float fElapsedTime, void* pUserContext );
LRESULT CALLBACK MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing,
                          void* pUserContext );
void CALLBACK OnKeyboard( UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext );
void CALLBACK OnGUIEvent( UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext );

bool CALLBACK IsD3D11DeviceAcceptable(const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo,
                                       DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext );
HRESULT CALLBACK OnD3D11CreateDevice( ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
                                      void* pUserContext );
HRESULT CALLBACK OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain,
                                          const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
void CALLBACK OnD3D11ReleasingSwapChain( void* pUserContext );
void CALLBACK OnD3D11DestroyDevice( void* pUserContext );
void CALLBACK OnD3D11FrameRender( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime,
                                  float fElapsedTime, void* pUserContext );

void InitApp();
void RenderText();


//--------------------------------------------------------------------------------------
// Entry point to the program. Initializes everything and goes into a message processing 
// loop. Idle time is used to render the scene.
//--------------------------------------------------------------------------------------
int WINAPI wWinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow )
{
    // Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	_CrtSetReportMode ( _CRT_ERROR,   _CRTDBG_MODE_DEBUG);
#endif

    // DXUT will create and use the best device (either D3D9 or D3D11) 
    // that is available on the system depending on which D3D callbacks are set below

    // Set DXUT callbacks
    DXUTSetCallbackDeviceChanging( ModifyDeviceSettings );
    DXUTSetCallbackMsgProc( MsgProc );
    DXUTSetCallbackKeyboard( OnKeyboard );
    DXUTSetCallbackFrameMove( OnFrameMove );    

	
    DXUTSetCallbackD3D11DeviceAcceptable( IsD3D11DeviceAcceptable );
    DXUTSetCallbackD3D11DeviceCreated( OnD3D11CreateDevice );
	
    DXUTSetCallbackD3D11SwapChainResized( OnD3D11ResizedSwapChain );
    DXUTSetCallbackD3D11FrameRender( OnD3D11FrameRender );
    DXUTSetCallbackD3D11SwapChainReleasing( OnD3D11ReleasingSwapChain );
	
    DXUTSetCallbackD3D11DeviceDestroyed( OnD3D11DestroyDevice );


    InitApp();
    DXUTInit( true, true, NULL ); // Parse the command line, show msgboxes on error, no extra command line params
    DXUTSetCursorSettings( true, true ); // Show the cursor and clip it when in full screen
    DXUTCreateWindow( L"Cloth Renderer" );
    DXUTCreateDevice (D3D_FEATURE_LEVEL_11_0, true, 800, 600 );
	DXUTSetMultimonSettings(false);
    //DXUTCreateDevice(true, 640, 480);
    DXUTMainLoop(); // Enter into the DXUT render loop

    return DXUTGetExitCode();
}


//--------------------------------------------------------------------------------------
// Initialize the app 
//--------------------------------------------------------------------------------------
void InitApp()
{
    D3DXVECTOR3 vLightDir( 1, 0, 0 );
    D3DXVec3Normalize( &vLightDir, &vLightDir );
    g_LightControl.SetLightDirection( vLightDir );

    // Initialize dialogs
    g_D3DSettingsDlg.Init( &g_DialogResourceManager );
    g_HUD.Init( &g_DialogResourceManager );
    g_SampleUI.Init( &g_DialogResourceManager );

    g_HUD.SetCallback( OnGUIEvent ); int iY = 10;
    g_HUD.AddButton( IDC_TOGGLEFULLSCREEN, L"Toggle full screen", 0, iY, 170, 23 );
	
    g_HUD.AddButton( IDC_TOGGLEREF, L"Toggle REF (F3)", 0, iY += 26, 170, 23, VK_F3 );
    g_HUD.AddButton( IDC_CHANGEDEVICE, L"Change device (F2)", 0, iY += 26, 170, 23, VK_F2 );
	g_HUD.AddButton( IDC_PAUSE, L"Pause", 0, iY += 26, 170, 23 );
	g_HUD.AddButton( IDC_WIREFRAME, L"Wire frame", 0, iY += 26, 170, 23 );
    
	g_SampleUI.SetCallback( OnGUIEvent ); iY = 10;
}


//--------------------------------------------------------------------------------------
// Called right before creating a D3D9 or D3D11 device, allowing the app to modify the device settings as needed
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings( DXUTDeviceSettings* pDeviceSettings, void* pUserContext )
{
    // Uncomment this to get debug information from D3D11
    //pDeviceSettings->d3d11.CreateFlags |= D3D11_CREATE_DEVICE_DEBUG;

    // For the first device created if its a REF device, optionally display a warning dialog box
    static bool s_bFirstTime = true;
    if( s_bFirstTime )
    {
        s_bFirstTime = false;
        if( ( DXUT_D3D11_DEVICE == pDeviceSettings->ver &&
              pDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE ) )
        {
            DXUTDisplaySwitchingToREFWarning( pDeviceSettings->ver );
        }
    }

    return true;
}


//--------------------------------------------------------------------------------------
// Handle updates to the scene.  This is called regardless of which D3D API is used
//--------------------------------------------------------------------------------------
void CALLBACK OnFrameMove( double fTime, float fElapsedTime, void* pUserContext )
{
    // Update the camera's position based on user input 
    g_Camera.FrameMove( fElapsedTime );
}


//--------------------------------------------------------------------------------------
// Render the help and statistics text
//--------------------------------------------------------------------------------------
void RenderText()
{
    UINT nBackBufferHeight = ( DXUTIsAppRenderingWithD3D9() ) ? DXUTGetD3D9BackBufferSurfaceDesc()->Height :
            DXUTGetDXGIBackBufferSurfaceDesc()->Height;

    g_pTxtHelper->Begin();
    g_pTxtHelper->SetInsertionPos( 2, 0 );
    g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 1.0f, 1.0f, 0.0f, 1.0f ) );
    g_pTxtHelper->DrawTextLine( DXUTGetFrameStats( DXUTIsVsyncEnabled() ) );
    g_pTxtHelper->DrawTextLine( DXUTGetDeviceStats() );

    // Draw help
    if( g_bShowHelp )
    {
        g_pTxtHelper->SetInsertionPos( 2, nBackBufferHeight - 20 * 6 );
        g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 1.0f, 0.75f, 0.0f, 1.0f ) );
        g_pTxtHelper->DrawTextLine( L"Controls:" );

        g_pTxtHelper->SetInsertionPos( 20, nBackBufferHeight - 20 * 5 );
		g_pTxtHelper->DrawTextLine( L"Rotate view: Left mouse button\n"
									L"Move camera: W, A, S, and D\n"
                                    L"Rotate light: Right mouse button\n"
                                    L"Zoom camera: Mouse wheel scroll\n" );

        g_pTxtHelper->SetInsertionPos( 550, nBackBufferHeight - 20 * 5 );
        g_pTxtHelper->DrawTextLine( L"Hide help: F1\n"
                                    L"Quit: ESC\n" );
    }
    else
    {
        g_pTxtHelper->SetForegroundColor( D3DXCOLOR( 1.0f, 1.0f, 1.0f, 1.0f ) );
        g_pTxtHelper->DrawTextLine( L"Press F1 for help" );
    }

    g_pTxtHelper->End();
}


//--------------------------------------------------------------------------------------
// Handle messages to the application
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing,
                          void* pUserContext )
{
    // Pass messages to dialog resource manager calls so GUI state is updated correctly
    *pbNoFurtherProcessing = g_DialogResourceManager.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    // Pass messages to settings dialog if its active
    if( g_D3DSettingsDlg.IsActive() )
    {
        g_D3DSettingsDlg.MsgProc( hWnd, uMsg, wParam, lParam );
        return 0;
    }

    // Give the dialogs a chance to handle the message first
    *pbNoFurtherProcessing = g_HUD.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;
    *pbNoFurtherProcessing = g_SampleUI.MsgProc( hWnd, uMsg, wParam, lParam );
    if( *pbNoFurtherProcessing )
        return 0;

    g_LightControl.HandleMessages( hWnd, uMsg, wParam, lParam );

    // Pass all remaining windows messages to camera so it can respond to user input
    g_Camera.HandleMessages( hWnd, uMsg, wParam, lParam );

    return 0;
}


//--------------------------------------------------------------------------------------
// Handle key presses
//--------------------------------------------------------------------------------------
void CALLBACK OnKeyboard( UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext )
{
    if( bKeyDown )
    {
        switch( nChar )
        {
            case VK_F1:
                g_bShowHelp = !g_bShowHelp; break;
        }
    }
}


//--------------------------------------------------------------------------------------
// Handles the GUI events
//--------------------------------------------------------------------------------------
void CALLBACK OnGUIEvent( UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext )
{
    switch( nControlID )
    {
        case IDC_TOGGLEFULLSCREEN:
            DXUTToggleFullScreen(); break;
        case IDC_TOGGLEREF:
            DXUTToggleREF(); break;
        case IDC_CHANGEDEVICE:
            g_D3DSettingsDlg.SetActive( !g_D3DSettingsDlg.IsActive() ); break;
		case IDC_PAUSE:
			paused = !paused;
			break;
		case IDC_WIREFRAME:
			g_wireFrame = !g_wireFrame;
			break;
    }

}


//--------------------------------------------------------------------------------------
// Reject any D3D11 devices that aren't acceptable by returning false
//--------------------------------------------------------------------------------------
bool CALLBACK IsD3D11DeviceAcceptable( const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo,
                                       DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext )
{
    return true;
}

//--------------------------------------------------------------------------------------
// Use this until D3DX11 comes online and we get some compilation helpers
//--------------------------------------------------------------------------------------
HRESULT CompileShaderFromFile( WCHAR* szFileName, LPCSTR szEntryPoint, LPCSTR szShaderModel, ID3DBlob** ppBlobOut )
{
    HRESULT hr = S_OK;

    // find the file
    WCHAR str[MAX_PATH];
    V_RETURN( DXUTFindDXSDKMediaFileCch( str, MAX_PATH, szFileName ) );

    // open the file
    HANDLE hFile = CreateFile( str, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
                               FILE_FLAG_SEQUENTIAL_SCAN, NULL );
    if( INVALID_HANDLE_VALUE == hFile )
        return E_FAIL;

    // Get the file size
    LARGE_INTEGER FileSize;
    GetFileSizeEx( hFile, &FileSize );

    // create enough space for the file data
    BYTE* pFileData = new BYTE[ FileSize.LowPart ];
    if( !pFileData )
        return E_OUTOFMEMORY;

    // read the data in
    DWORD BytesRead;
    if( !ReadFile( hFile, pFileData, FileSize.LowPart, &BytesRead, NULL ) )
        return E_FAIL; 

    CloseHandle( hFile );

    // Compile the shader
    ID3DBlob* pErrorBlob;
    hr = D3DCompile( pFileData, FileSize.LowPart, "none", NULL, NULL, szEntryPoint, szShaderModel, D3D10_SHADER_ENABLE_STRICTNESS, 0, ppBlobOut, &pErrorBlob );

    delete []pFileData;

    if( FAILED(hr) )
    {
        OutputDebugStringA( (char*)pErrorBlob->GetBufferPointer() );
        SAFE_RELEASE( pErrorBlob );
        return hr;
    }
    SAFE_RELEASE( pErrorBlob );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Create any D3D11 resources that aren't dependant on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11CreateDevice( ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
                                      void* pUserContext )
{
	
    g_pd3dDevice = pd3dDevice;
 
    HRESULT hr;

    ID3D11DeviceContext* pd3dImmediateContext = DXUTGetD3D11DeviceContext();

    V_RETURN( g_DialogResourceManager.OnD3D11CreateDevice( pd3dDevice, pd3dImmediateContext ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D11CreateDevice( pd3dDevice ) );
    g_pTxtHelper = new CDXUTTextHelper( pd3dDevice, pd3dImmediateContext, &g_DialogResourceManager, 15 );

    D3DXVECTOR3 vCenter( 0.25767413f, -28.503521f, 111.00689f);
    FLOAT fObjectRadius = 378.15607f;

    D3DXMatrixTranslation( &g_mCenterMesh, -vCenter.x, -vCenter.y, -vCenter.z );
    D3DXMATRIXA16 m;
    D3DXMatrixRotationY( &m, D3DX_PI );
    g_mCenterMesh *= m;
    D3DXMatrixRotationX( &m, D3DX_PI / 2.0f );
    g_mCenterMesh *= m;

    // Compile the shaders to a model based on the feature level we acquired
    ID3DBlob* pVertexShaderBuffer = NULL;
	ID3DBlob* pGeometryShaderBuffer = NULL;
    ID3DBlob* pPixelShaderBuffer = NULL;
  
    switch( DXUTGetD3D11DeviceFeatureLevel() )
    {
        case D3D_FEATURE_LEVEL_11_0:
            V_RETURN( CompileShaderFromFile( L"cloth_renderer_VS.hlsl", "VSMain", "vs_5_0" , &pVertexShaderBuffer ) );
			V_RETURN( CompileShaderFromFile( L"cloth_renderer_PS.hlsl", "GSMain", "gs_5_0" , &pGeometryShaderBuffer ) );
            V_RETURN( CompileShaderFromFile( L"cloth_renderer_PS.hlsl", "PSMain", "ps_5_0" , &pPixelShaderBuffer ) );
            break;        
    }

    // Create the shaders
    V_RETURN( pd3dDevice->CreateVertexShader( pVertexShaderBuffer->GetBufferPointer(),
                                              pVertexShaderBuffer->GetBufferSize(), NULL, &g_pVertexShader ) );


	V_RETURN( pd3dDevice->CreateGeometryShader( pGeometryShaderBuffer->GetBufferPointer(),
                                              pGeometryShaderBuffer->GetBufferSize(), NULL, &g_pGeometryShader ) );
    

    V_RETURN( pd3dDevice->CreatePixelShader( pPixelShaderBuffer->GetBufferPointer(),
                                             pPixelShaderBuffer->GetBufferSize(), NULL, &g_pPixelShader ) );

    

    V_RETURN( pd3dDevice->CreateInputLayout( layout, ARRAYSIZE( layout ), pVertexShaderBuffer->GetBufferPointer(),
                                             pVertexShaderBuffer->GetBufferSize(), &g_pVertexLayout11 ) );

    SAFE_RELEASE( pVertexShaderBuffer );
    SAFE_RELEASE( pPixelShaderBuffer );
	SAFE_RELEASE( pGeometryShaderBuffer );


    // Load the mesh
    V_RETURN( g_Mesh11.Create( pd3dDevice, L"tiny\\tiny.sdkmesh", true ) );


    

    // Create a sampler state
    D3D11_SAMPLER_DESC SamDesc;
    SamDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    SamDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    SamDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    SamDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    SamDesc.MipLODBias = 0.0f;
    SamDesc.MaxAnisotropy = 1;
    SamDesc.ComparisonFunc = D3D11_COMPARISON_ALWAYS;
    SamDesc.BorderColor[0] = SamDesc.BorderColor[1] = SamDesc.BorderColor[2] = SamDesc.BorderColor[3] = 0;
    SamDesc.MinLOD = 0;
    SamDesc.MaxLOD = D3D11_FLOAT32_MAX;
    V_RETURN( pd3dDevice->CreateSamplerState( &SamDesc, &g_pSamLinear ) );


	

    // Setup constant buffers
    D3D11_BUFFER_DESC Desc;
    Desc.Usage = D3D11_USAGE_DYNAMIC;
    Desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    Desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    Desc.MiscFlags = 0;

    Desc.ByteWidth = sizeof( CB_VS_PER_OBJECT );
    V_RETURN( pd3dDevice->CreateBuffer( &Desc, NULL, &g_pcbVSPerObject ) );

    Desc.ByteWidth = sizeof( CB_PS_PER_OBJECT );
    V_RETURN( pd3dDevice->CreateBuffer( &Desc, NULL, &g_pcbPSPerObject ) );

    Desc.ByteWidth = sizeof( CB_PS_PER_FRAME );
    V_RETURN( pd3dDevice->CreateBuffer( &Desc, NULL, &g_pcbPSPerFrame ) );

    // Setup the camera's view parameters
    
	
	D3DXVECTOR3 vecEye( 30.0f, 30.0f, -80.0f );
    D3DXVECTOR3 vecAt ( 10.0f, 20.0f, -0.0f );
    

	g_Camera.SetViewParams( &vecEye, &vecAt );

	cloths.resize(numFlags);

	for( int flagIndex =  0; flagIndex < numFlags; ++flagIndex )
	{
		cloths[flagIndex].create_buffers(clothWidth, clothHeight);
	}

	initBullet();

std::wstring flagTexsName[] = {
		L"atiFlag.bmp",
		L"amdFlag.bmp",
	};
	int numFlagTexs = 2;



	WCHAR flagTexs[2][MAX_PATH];

	HRESULT res = DXUTFindDXSDKMediaFileCch(flagTexs[0],MAX_PATH, flagTexsName[0].c_str());
	res = DXUTFindDXSDKMediaFileCch(flagTexs[1],MAX_PATH, flagTexsName[1].c_str());
	

	for( int flagIndex =  0; flagIndex < numFlags; ++flagIndex )
	{
		cloths[flagIndex].create_texture(flagTexs[flagIndex % numFlagTexs]);
		cloths[flagIndex].x_offset = 0; 
		cloths[flagIndex].y_offset = 0; 
		cloths[flagIndex].z_offset = 0;
	}

	

	my_capsule.create_buffers(50,40);
	my_capsule.create_texture();

	//Turn off backface culling
	D3D11_RASTERIZER_DESC rsDesc;
	ZeroMemory(&rsDesc,sizeof(D3D11_RASTERIZER_DESC) );
	rsDesc.CullMode = D3D11_CULL_NONE;
	rsDesc.FillMode = D3D11_FILL_SOLID;
	
	hr = pd3dDevice->CreateRasterizerState(&rsDesc, &g_pRasterizerState);	
	
	rsDesc.FillMode = D3D11_FILL_WIREFRAME;
	hr = pd3dDevice->CreateRasterizerState(&rsDesc, &g_pRasterizerStateWF);
	
	SAFE_RELEASE(pd3dImmediateContext);


	
    return S_OK;
}


//--------------------------------------------------------------------------------------
// Create any D3D11 resources that depend on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain,
                                          const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext )
{
    HRESULT hr;

    V_RETURN( g_DialogResourceManager.OnD3D11ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );
    V_RETURN( g_D3DSettingsDlg.OnD3D11ResizedSwapChain( pd3dDevice, pBackBufferSurfaceDesc ) );

    // Setup the camera's projection parameters
    float fAspectRatio = pBackBufferSurfaceDesc->Width / ( FLOAT )pBackBufferSurfaceDesc->Height;
    g_Camera.SetProjParams( D3DX_PI / 4, fAspectRatio, 2.0f, 4000.0f );
	//    g_Camera.SetWindow( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
	//    g_Camera.SetButtonMasks( MOUSE_MIDDLE_BUTTON, MOUSE_WHEEL, MOUSE_LEFT_BUTTON );


	D3DXVECTOR3 vMin = D3DXVECTOR3( -1000.0f, -1000.0f, -1000.0f );
    D3DXVECTOR3 vMax = D3DXVECTOR3( 1000.0f, 1000.0f, 1000.0f );
    g_Camera.SetRotateButtons(TRUE, FALSE, FALSE);
	
	
    g_Camera.SetScalers( 0.01f, 30.0f );
    g_Camera.SetDrag( true );
    g_Camera.SetEnableYAxisMovement( true );
    g_Camera.SetClipToBoundary( TRUE, &vMin, &vMax );
    g_Camera.FrameMove( 0 );


    g_HUD.SetLocation( pBackBufferSurfaceDesc->Width - 170, 0 );
    g_HUD.SetSize( 170, 170 );
    g_SampleUI.SetLocation( pBackBufferSurfaceDesc->Width - 170, pBackBufferSurfaceDesc->Height - 300 );
    g_SampleUI.SetSize( 170, 300 );

	//Turn off backface culling
	D3D11_RASTERIZER_DESC rsDesc;
	ZeroMemory(&rsDesc,sizeof(D3D11_RASTERIZER_DESC) );
	rsDesc.CullMode = D3D11_CULL_NONE;
	rsDesc.FillMode = D3D11_FILL_SOLID;
	//rsDesc.FillMode = D3D11_FILL_WIREFRAME;
	
	
	ID3D11RasterizerState *pRasterizerState = NULL;
	pd3dDevice->CreateRasterizerState(&rsDesc, &pRasterizerState);
	
	DXUTGetD3D11DeviceContext()->RSSetState(pRasterizerState);

	SAFE_RELEASE(pRasterizerState);

    return S_OK;
}


btClock m_clock;
//--------------------------------------------------------------------------------------
// Render the scene using the D3D11 device
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11FrameRender( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime,
                                  float fElapsedTime, void* pUserContext )
{




	//float ms = getDeltaTimeMicroseconds();
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();

	///step the simulation
	if (m_dynamicsWorld && !paused)
	{

		m_dynamicsWorld->stepSimulation(dt / 1000000.f);

		updatePhysicsWorld();
	}

	//paused = 1;
	


	///////////////////////////////////////////////////////

    HRESULT hr;

    // If the settings dialog is being shown, then render it instead of rendering the app's scene
    if( g_D3DSettingsDlg.IsActive() )
    {
        g_D3DSettingsDlg.OnRender( fElapsedTime );
        return;
    }

    // Clear the render target and depth stencil
    float ClearColor[4] = { 0.0f, 0.25f, 0.25f, 0.55f };
    ID3D11RenderTargetView* pRTV = DXUTGetD3D11RenderTargetView();
    pd3dImmediateContext->ClearRenderTargetView( pRTV, ClearColor );
    ID3D11DepthStencilView* pDSV = DXUTGetD3D11DepthStencilView();
    pd3dImmediateContext->ClearDepthStencilView( pDSV, D3D11_CLEAR_DEPTH, 1.0, 0 );


	for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	{	
		g_softBodyOutput->copySoftBodyToVertexBuffer( m_flags[flagIndex], cloths[flagIndex].m_vertexBufferDescriptor );
		cloths[flagIndex].draw();
	}

	my_capsule.draw();

	
    DXUT_BeginPerfEvent( DXUT_PERFEVENTCOLOR, L"HUD / Stats" );
    g_HUD.OnRender( fElapsedTime );
    g_SampleUI.OnRender( fElapsedTime );
    RenderText();
    DXUT_EndPerfEvent();


/*
	 SAFE_RELEASE(pRTV);
     SAFE_RELEASE(pDSV);
*/

 
}


//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11ResizedSwapChain 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11ReleasingSwapChain( void* pUserContext )
{
    g_DialogResourceManager.OnD3D11ReleasingSwapChain();
	DXUTGetD3D11DeviceContext()->ClearState();
}


//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11CreateDevice 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11DestroyDevice( void* pUserContext )
{
    g_DialogResourceManager.OnD3D11DestroyDevice();
    g_D3DSettingsDlg.OnD3D11DestroyDevice();
    DXUTGetGlobalResourceCache().OnDestroyDevice();
    SAFE_DELETE( g_pTxtHelper );

    g_Mesh11.Destroy();
                

	SAFE_RELEASE(g_pGeometryShader);
    SAFE_RELEASE( g_pVertexLayout11 );
    SAFE_RELEASE( g_pVertexBuffer );
    SAFE_RELEASE( g_pVertexShader );
    SAFE_RELEASE( g_pPixelShader );
    SAFE_RELEASE( g_pSamLinear );

    SAFE_RELEASE( g_pcbVSPerObject );
    SAFE_RELEASE( g_pcbPSPerObject );
    SAFE_RELEASE( g_pcbPSPerFrame );
	
	SAFE_RELEASE( g_pRasterizerState );
	SAFE_RELEASE( g_pRasterizerStateWF );


	for( int flagIndex =  0; flagIndex < numFlags; ++flagIndex )
	{
		cloths[flagIndex].destroy();
	}

	my_capsule.destroy();

	// Shouldn't need to delete this as it's just a soft body and will be deleted later by the collision object cleanup.
	//for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	//{	
		//delete m_flags[flagIndex];
	//}

	//cleanup in the reverse order of creation/initialization
	if( g_defaultSolver )
		delete g_defaultSolver;
	if( g_cpuSolver )
		delete g_cpuSolver;
	if( g_dx11Solver )
		delete g_dx11Solver;
	if( g_dx11SIMDSolver )
		delete g_dx11SIMDSolver;
	if( g_softBodyOutput )
		delete g_softBodyOutput;
	

	for(int i=0; i< m_collisionShapes.size(); i++)
		delete m_collisionShapes[i];

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	delete m_dynamicsWorld;	
	delete m_solver;	
	delete m_broadphase;	
	delete m_dispatcher;
	delete m_collisionConfiguration;

		
}










